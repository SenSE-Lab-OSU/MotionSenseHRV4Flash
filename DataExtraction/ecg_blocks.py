"""Incremental ECF2/ECB2 decoder. Uses only the Python standard library."""

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
import struct
import sys
import zlib

BLOCK_BYTES = 4096
SAMPLES_PER_BLOCK = 1358
FILE_BYTES = 4 * 1024 * 1024
DATA_BLOCKS_PER_FILE = 1023
UINT32_MASK = 0xFFFFFFFF


class DecodeError(ValueError):
    """An invalid page or a discontinuity ended the valid prefix."""


def _check_crc(page, offset):
    expected = struct.unpack_from('<I', page, offset)[0]
    crc = zlib.crc32(page[:offset])
    crc = zlib.crc32(b'\0' * 4, crc)
    crc = zlib.crc32(page[offset + 4:], crc)
    if crc != expected:
        raise DecodeError('CRC mismatch')


@dataclass(frozen=True)
class FileHeader:
    chunk_index: int
    recording_id: int


def decode_header(page):
    if len(page) != BLOCK_BYTES or page[:4] != b'ECF2':
        raise DecodeError('expected a 4096-byte ECF2 header')
    if any(page[20:]):
        raise DecodeError('nonzero header reserved bytes')
    _check_crc(page, 16)
    chunk_index, recording_id = struct.unpack_from('<IQ', page, 4)
    return FileHeader(chunk_index, recording_id)


@dataclass(frozen=True)
class Block:
    first_rtc_tick: int
    first_sample_index: int
    payload: bytes

    def samples(self):
        """Yield (tick, index, signed_count, etag, ptag); tags 1/3 are unusable."""
        for i in range(SAMPLES_PER_BLOCK):
            raw = int.from_bytes(self.payload[3 * i:3 * i + 3], 'big')
            count = raw >> 6
            if count & 0x20000:
                count -= 0x40000
            yield ((self.first_rtc_tick + i) & UINT32_MASK,
                   (self.first_sample_index + i) & UINT32_MASK,
                   count, (raw >> 3) & 7, raw & 7)


def decode_block(page, previous=None):
    if len(page) != BLOCK_BYTES or page[:4] != b'ECB2':
        raise DecodeError('expected a 4096-byte ECB2 block')
    if any(page[4090:]):
        raise DecodeError('nonzero block reserved bytes')
    _check_crc(page, 12)
    payload = bytes(page[16:4090])
    if any(((payload[i] >> 3) & 7) > 3 for i in range(2, len(payload), 3)):
        raise DecodeError('invalid ECG sample tag')
    tick, index = struct.unpack_from('<II', page, 4)
    block = Block(tick, index, payload)
    if previous is not None:
        expected_tick = (previous.first_rtc_tick + SAMPLES_PER_BLOCK) & UINT32_MASK
        expected_index = (previous.first_sample_index + SAMPLES_PER_BLOCK) & UINT32_MASK
        if tick != expected_tick or index != expected_index:
            raise DecodeError('sample-index or RTC continuity gap')
    return block


def iter_recording(paths):
    """Yield validated blocks from ordered chunks; raise at the first error.

    Already yielded blocks remain valid. An isolated later chunk is permitted.
    File handles close on completion, failure, or explicit generator.close().
    """
    previous = None
    previous_header = None
    previous_count = 0
    for name in paths:
        path = Path(name)
        try:
            if path.stat().st_size != FILE_BYTES:
                raise DecodeError('file size must be exactly 4 MiB')
            with path.open('rb') as source:
                header = decode_header(source.read(BLOCK_BYTES))
                if previous_header is not None:
                    if header.recording_id != previous_header.recording_id:
                        raise DecodeError('recording identity changed between chunks')
                    if header.chunk_index != previous_header.chunk_index + 1:
                        raise DecodeError('missing, duplicate or reordered chunk')
                    if previous_count != DATA_BLOCKS_PER_FILE:
                        raise DecodeError('a nonfinal chunk must be full')
                count = 0
                for slot in range(DATA_BLOCKS_PER_FILE):
                    page = source.read(BLOCK_BYTES)
                    if len(page) != BLOCK_BYTES:
                        raise DecodeError(f'data page {slot + 1}: short read')
                    if page[:4] == b'\xff' * 4:
                        break
                    try:
                        block = decode_block(page, previous)
                        if slot == 0 and header.chunk_index == 0 and block.first_sample_index != 0:
                            raise DecodeError('chunk zero must start at sample index zero')
                    except DecodeError as error:
                        raise DecodeError(f'data page {slot + 1}: {error}') from error
                    previous = block
                    count += 1
                    yield block
                previous_header = header
                previous_count = count
        except DecodeError as error:
            raise DecodeError(f'{path}: {error}') from error


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('files', nargs='+', type=Path,
                        help='one file or consecutive chunks in chunk-index order')
    parser.add_argument('--csv', type=Path, help='write validated samples incrementally')
    args = parser.parse_args(argv)
    if args.csv and args.csv.resolve() in {path.resolve() for path in args.files}:
        parser.error('CSV output must not overwrite an input file')
    count = 0
    output = None
    blocks = iter_recording(args.files)
    try:
        if args.csv:
            output = args.csv.open('w', newline='', encoding='utf-8')
            writer = csv.writer(output)
            writer.writerow(('rtc_tick', 'sample_index', 'ecg_count', 'etag', 'ptag', 'usable'))
        for block in blocks:
            if output:
                for sample in block.samples():
                    writer.writerow((*sample, int(sample[3] in (0, 2))))
            count += 1
    except (DecodeError, OSError) as error:
        print(f'ERROR: {error}\nValidated prefix: {count} blocks, '
              f'{count * SAMPLES_PER_BLOCK} samples', file=sys.stderr)
        return 1
    finally:
        blocks.close()
        if output:
            output.close()
    print(f'Validated {count} blocks, {count * SAMPLES_PER_BLOCK} samples '
          '(file format does not certify clean closure)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
