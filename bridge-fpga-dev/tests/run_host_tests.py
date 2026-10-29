"""Cygwin host tests. No hardware or programmer access."""
import os
from pathlib import Path
import subprocess

app = Path(__file__).resolve().parents[1]
build = app / 'build' / 'host-tests'
build.mkdir(parents=True, exist_ok=True)
env = dict(os.environ)
env['PATH'] = r'C:\cygwin64\bin;' + env['PATH']
gcc = r'C:\cygwin64\bin\gcc.exe'

def compile_test(name, sources):
    exe = build / (name + '.exe')
    subprocess.run([gcc, '-std=c11', '-g', '-Wall', '-Wextra', '-Wno-unused-parameter',
                    '-I', str(app/'src'), *map(str, sources), '-o', str(exe)],
                   env=env, check=True)
    return exe

host = compile_test('host_vme', [app/'src/vme.c', app/'tests/host_vme.c'])
jtag = compile_test('test_jtag', [app/'src/jtag.c', app/'tests/test_jtag.c'])
subprocess.run([str(jtag)], env=env, check=True)

def run(name, path, expected, scan=0, mode=0):
    p = subprocess.run([str(host), str(path), str(build/(name+'.trace')), str(scan), str(mode)],
                       env=env, capture_output=True, text=True, timeout=30)
    (build/(name+'.log')).write_text(p.stdout + p.stderr)
    assert 'runtime error:' not in p.stderr, p.stderr
    assert f'SIMULATION ONLY result={expected} ' in p.stdout, p.stdout + p.stderr
    assert (p.returncode == 0) == (expected == 0), p.stdout
    if expected == 0:
        assert 'TRANSPORT bit-order/final-TMS/state=PASS' in p.stdout
    print('PASS:', name)

original = app / 'bridge-hello.vme'
run('full-image', original, 0)
run('id-mismatch', original, -1, scan=2)
run('page-verify-mismatch', original, -1, scan=2332)
run('busy-never-clears', original, -1, mode=2)
run('busy-clears-on-retry', original, 0, mode=6)
run('gpio-failure', original, -20, mode=3)
run('deadline', original, -21, mode=4)
data = original.read_bytes()
bad = build / 'bad-crc.vme'
bad.write_bytes(data[:100] + bytes([data[100] ^ 1]) + data[101:])
run('bad-crc', bad, -6)
bad.write_bytes(data[:-1])
run('truncated-crc', bad, -6)

def crc(data):
    table = [0,0xcc01,0xd801,0x1400,0xf001,0x3c00,0x2800,0xe401,
             0xa001,0x6c00,0x7800,0xb401,0x5000,0x9c01,0x8801,0x4400]
    value = 0
    for byte in data:
        byte = int(f'{byte:08b}'[::-1], 2)
        value = (value >> 4) ^ table[value & 15] ^ table[byte & 15]
        value = (value >> 4) ^ table[value & 15] ^ table[byte >> 4]
    return value.to_bytes(2, 'big')

def fixture(name, stream):
    payload = b'____12.1' + stream
    path = build / (name + '.vme')
    path.write_bytes(b'\x65' + crc(payload) + payload)
    return path

# Full VME with one masked comparison: only the first of eight bits matters.
stream = bytes.fromhex('f2 19 80 01 10 01 11 08 13 07 70 12 08 13 00 14 80 15 80 70 7f')
run('mask-match', fixture('masked', stream), 0)
run('mask-ignores-unselected-bit', fixture('masked', stream), 0, mode=5)
run('mask-selected-bit-fails', fixture('masked', stream), -1, scan=2)
run('truncated-with-valid-crc', fixture('short', stream[:-2]), -2)
bad.write_bytes(b'\x65'+data[1:3]+b'____99.0'+data[11:])
run('unsupported-version-rejected', bad, -3)
run('security-opcode-rejected', fixture('security', bytes.fromhex('f2 19 80 01 11 08 13 73 70 7f')), -23)
run('oversized-scan', fixture('size', bytes.fromhex('f2 19 80 01 11 81 01 13 07 70 7f')), -4)
run('varint-overflow', fixture('varint', bytes.fromhex('f2 19 ff ff ff ff ff ff 00 7f')), -4)
print('All tests passed. Synthetic TDO is not hardware verification.')
