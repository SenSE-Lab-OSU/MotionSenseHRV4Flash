"""Measure uncached sequential reads from a Windows-mounted MSC file."""

import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import statistics
import time


GENERIC_READ = 0x80000000
FILE_SHARE_ALL = 0x00000007
OPEN_EXISTING = 3
FILE_FLAG_NO_BUFFERING = 0x20000000
FILE_FLAG_SEQUENTIAL_SCAN = 0x08000000
MEM_COMMIT_RESERVE = 0x3000
MEM_RELEASE = 0x8000
PAGE_READWRITE = 0x04
SECTOR_SIZE = 4096


def win_error(label):
    raise OSError(ctypes.get_last_error(), label)


def measure(path, runs, chunk_size):
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.CreateFileW.argtypes = [
        ctypes.c_wchar_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p,
        ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p,
    ]
    kernel32.CreateFileW.restype = ctypes.c_void_p
    kernel32.VirtualAlloc.argtypes = [
        ctypes.c_void_p, ctypes.c_size_t, ctypes.c_uint32, ctypes.c_uint32,
    ]
    kernel32.VirtualAlloc.restype = ctypes.c_void_p
    kernel32.VirtualFree.argtypes = [
        ctypes.c_void_p, ctypes.c_size_t, ctypes.c_uint32,
    ]
    kernel32.VirtualFree.restype = ctypes.c_int
    kernel32.ReadFile.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint32,
        ctypes.POINTER(ctypes.c_uint32), ctypes.c_void_p,
    ]
    kernel32.ReadFile.restype = ctypes.c_int
    kernel32.SetFilePointerEx.argtypes = [
        ctypes.c_void_p, ctypes.c_int64, ctypes.c_void_p, ctypes.c_uint32,
    ]
    kernel32.SetFilePointerEx.restype = ctypes.c_int
    kernel32.CloseHandle.argtypes = [ctypes.c_void_p]
    kernel32.CloseHandle.restype = ctypes.c_int

    size = path.stat().st_size
    measured_size = size - (size % SECTOR_SIZE)
    if measured_size < SECTOR_SIZE:
        raise ValueError(f"file is smaller than one {SECTOR_SIZE}-byte sector")

    handle = kernel32.CreateFileW(
        str(path), GENERIC_READ, FILE_SHARE_ALL, None, OPEN_EXISTING,
        FILE_FLAG_NO_BUFFERING | FILE_FLAG_SEQUENTIAL_SCAN, None,
    )
    if handle == ctypes.c_void_p(-1).value:
        win_error("CreateFileW")
    buffer = kernel32.VirtualAlloc(
        None, chunk_size, MEM_COMMIT_RESERVE, PAGE_READWRITE
    )
    if not buffer:
        kernel32.CloseHandle(handle)
        win_error("VirtualAlloc")

    results = []
    try:
        for run in range(1, runs + 1):
            if not kernel32.SetFilePointerEx(handle, 0, None, 0):
                win_error("SetFilePointerEx")
            remaining = measured_size
            digest = hashlib.sha256()
            started = time.perf_counter()
            while remaining:
                requested = min(chunk_size, remaining)
                transferred = ctypes.c_uint32()
                if not kernel32.ReadFile(
                    handle, buffer, requested, ctypes.byref(transferred), None
                ):
                    win_error("ReadFile")
                if transferred.value != requested:
                    raise OSError(
                        f"short read: requested {requested}, got {transferred.value}"
                    )
                digest.update(ctypes.string_at(buffer, transferred.value))
                remaining -= transferred.value
            seconds = time.perf_counter() - started
            results.append({
                "run": run,
                "seconds": seconds,
                "mib_per_second": measured_size / seconds / (1024 * 1024),
                "sha256": digest.hexdigest(),
            })
    finally:
        kernel32.VirtualFree(buffer, 0, MEM_RELEASE)
        kernel32.CloseHandle(handle)

    hashes = {result["sha256"] for result in results}
    if len(hashes) != 1:
        raise RuntimeError("read hashes differed between runs")
    rates = [result["mib_per_second"] for result in results]
    return {
        "file": str(path.resolve()),
        "file_size": size,
        "measured_size": measured_size,
        "sector_size": SECTOR_SIZE,
        "chunk_size": chunk_size,
        "cache_mode": "FILE_FLAG_NO_BUFFERING",
        "runs": results,
        "median_mib_per_second": statistics.median(rates),
        "min_mib_per_second": min(rates),
        "max_mib_per_second": max(rates),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", type=Path, required=True)
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--chunk-kib", type=int, default=1024)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    chunk_size = args.chunk_kib * 1024
    if chunk_size <= 0 or chunk_size % SECTOR_SIZE:
        parser.error("--chunk-kib must produce a positive 4096-byte multiple")

    result = measure(args.file, args.runs, chunk_size)
    encoded = json.dumps(result, indent=2)
    print(encoded)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
