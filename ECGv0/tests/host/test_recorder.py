"""Compile selected production recorder functions with deterministic I/O stubs.

Run: python ECGv0/tests/host/test_recorder.py [--cc path/to/gcc]
This checks boundary and file decisions, not Zephyr scheduling or hardware timing.
"""

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=shutil.which('gcc') or r'C:\cygwin64\bin\gcc.exe')
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    source = (here.parents[1] / 'src/ecgRecorder.c').read_text()
    functions = []
    for name in ('ecg_record_rotate_chunk', 'ecg_record_take_filling_block',
                 'ecg_record_queue_finalized_block', 'ecg_record_finish_file',
                 'ecg_record_process_samples'):
        match = re.search(r'static int ' + name + r'\([^;]*?\)\s*\{', source)
        if match is None:
            raise RuntimeError(f'production function not found: {name}')
        end = match.end()
        depth = 1
        while depth:
            depth += (source[end] == '{') - (source[end] == '}')
            end += 1
        functions.append(source[match.start():end])
    harness = (here / 'recorder_harness.c').read_text()
    harness = harness.replace('/* PRODUCTION_FUNCTIONS */', '\n\n'.join(functions))
    env = os.environ.copy()
    env['PATH'] = str(Path(args.cc).resolve().parent) + os.pathsep + env['PATH']
    with tempfile.TemporaryDirectory(prefix='ecg-recorder-') as directory:
        generated = Path(directory) / 'recorder.c'
        executable = Path(directory) / 'recorder.exe'
        generated.write_text(harness)
        subprocess.run([args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                        str(generated), '-o', str(executable)], env=env, check=True)
        subprocess.run([str(executable)], env=env, check=True)


if __name__ == '__main__':
    main()
