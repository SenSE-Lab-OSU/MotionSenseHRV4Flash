"""Independent byte-level tests for the ECF2 file decoder."""

from pathlib import Path
import struct
import tempfile
import unittest
import zlib

from ecg_blocks import (BLOCK_BYTES, DATA_BLOCKS_PER_FILE, FILE_BYTES,
                        SAMPLES_PER_BLOCK, DecodeError, decode_block,
                        decode_header, iter_recording)


def crc_page(page, offset):
    struct.pack_into('<I', page, offset, 0)
    struct.pack_into('<I', page, offset, zlib.crc32(page))
    return bytes(page)


def header(chunk=0, identity=1):
    page = bytearray(BLOCK_BYTES)
    struct.pack_into('<4sIQ', page, 0, b'ECF2', chunk, identity)
    return crc_page(page, 16)


def block(index=0, tick=None):
    page = bytearray(BLOCK_BYTES)
    struct.pack_into('<4sII', page, 0, b'ECB2', index if tick is None else tick, index)
    for i in range(SAMPLES_PER_BLOCK):
        raw = (i << 6) | ((i % 4) << 3) | (i % 8)
        page[16 + 3 * i:19 + 3 * i] = raw.to_bytes(3, 'big')
    return crc_page(page, 12)


class DecoderTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)

    def file(self, name, pages=(), chunk=0, identity=1):
        path = Path(self.temp.name) / name
        with path.open('wb') as out:
            out.write(header(chunk, identity))
            count = 0
            for page in pages:
                out.write(page)
                count += 1
            out.write(b'\xff' * (FILE_BYTES - BLOCK_BYTES * (count + 1)))
        return path

    def test_independent_golden_crc_and_sample_unpacking(self):
        page = block(0xffffff00)
        self.assertEqual(struct.unpack_from('<I', page, 12)[0], 0x3ab7821c)
        self.assertEqual(struct.unpack_from('<I', header(7, 0x0123456789abcdef), 16)[0], 0x3fdb88b5)
        samples = list(decode_block(page).samples())
        self.assertEqual(samples[1], (0xffffff01, 0xffffff01, 1, 1, 1))
        self.assertEqual(samples[256][:2], (0, 0))
        changed = bytearray(page)
        changed[16:19] = bytes.fromhex('800000')
        self.assertEqual(next(decode_block(crc_page(changed, 12)).samples())[2], -131072)

    def test_wrap_and_gap(self):
        previous = decode_block(block(0xffffff00))
        decode_block(block(0x44e), previous)
        with self.assertRaisesRegex(DecodeError, 'continuity'):
            decode_block(block(0x44f), previous)

    def test_corruption_and_nonconforming_valid_crc(self):
        page = bytearray(block())
        page[16] ^= 1
        with self.assertRaisesRegex(DecodeError, 'CRC'):
            decode_block(page)
        for offset, value in ((4090, 1), (18, 0x20)):
            page = bytearray(block())
            page[offset] = value
            with self.assertRaises(DecodeError):
                decode_block(crc_page(page, 12))
        with self.assertRaises(DecodeError):
            decode_block(b'ECB1' + block()[4:])
        with self.assertRaises(DecodeError):
            decode_block(block()[:-1])
        page = bytearray(header())
        page[20] = 1
        with self.assertRaises(DecodeError):
            decode_header(crc_page(page, 16))

    def test_empty_partial_and_full_file(self):
        self.assertEqual(list(iter_recording([self.file('empty')])), [])
        self.assertEqual(len(list(iter_recording([self.file('partial', [block()])]))), 1)
        full = self.file('full', (block(i * SAMPLES_PER_BLOCK)
                                for i in range(DATA_BLOCKS_PER_FILE)))
        self.assertEqual(sum(1 for _ in iter_recording([full])), DATA_BLOCKS_PER_FILE)

    def test_invalid_page_preserves_prefix_without_resuming(self):
        bad = bytearray(block(SAMPLES_PER_BLOCK))
        bad[16] ^= 1
        path = self.file('bad', [block(), bad, block(2 * SAMPLES_PER_BLOCK)])
        stream = iter_recording([path])
        self.assertEqual(next(stream).first_sample_index, 0)
        with self.assertRaisesRegex(DecodeError, 'data page 2'):
            next(stream)

    def test_sentinel_stops_without_scanning_later_pages(self):
        path = self.file('sentinel', [block(), b'\xff' * BLOCK_BYTES,
                                      block(2 * SAMPLES_PER_BLOCK)])
        self.assertEqual(sum(1 for _ in iter_recording([path])), 1)

    def test_chunk_rules_and_segment(self):
        full = self.file('chunk0', (block(i * SAMPLES_PER_BLOCK)
                                  for i in range(DATA_BLOCKS_PER_FILE)))
        tail = self.file('chunk1', [block(DATA_BLOCKS_PER_FILE * SAMPLES_PER_BLOCK)], chunk=1)
        self.assertEqual(sum(1 for _ in iter_recording([full, tail])), 1024)
        self.assertEqual(sum(1 for _ in iter_recording([tail])), 1)
        partial = self.file('short', [block()])
        with self.assertRaisesRegex(DecodeError, 'nonfinal chunk'):
            list(iter_recording([partial, tail]))
        wrong_id = self.file('identity', chunk=1, identity=2)
        with self.assertRaisesRegex(DecodeError, 'identity'):
            list(iter_recording([full, wrong_id]))
        wrong_chunk = self.file('missing', chunk=2)
        with self.assertRaisesRegex(DecodeError, 'chunk'):
            list(iter_recording([full, wrong_chunk]))
        bad_start = self.file('badstart', [block(1)])
        with self.assertRaisesRegex(DecodeError, 'index zero'):
            list(iter_recording([bad_start]))


if __name__ == '__main__':
    unittest.main()
