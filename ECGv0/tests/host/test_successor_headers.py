"""Compile successor-file lifecycle code with deterministic filesystem stubs."""

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def extract_function(source, name):
    match = re.search(r'(?:static )?(?:int|void) ' + name +
                      r'\([^;]*?\)\s*\{', source)
    if match is None:
        raise RuntimeError(f'production function not found: {name}')
    end = match.end()
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


def compile_and_run(cc, env, directory, harness_path, replacements):
    harness = harness_path.read_text()
    for marker, functions in replacements.items():
        harness = harness.replace(marker, '\n\n'.join(functions))
    generated = directory / harness_path.name
    executable = directory / (harness_path.stem + '.exe')
    generated.write_text(harness)
    subprocess.run([cc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                    str(generated), '-o', str(executable)], env=env, check=True)
    subprocess.run([str(executable)], env=env, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=shutil.which('gcc') or
                        r'C:\cygwin64\bin\gcc.exe')
    parser.add_argument('--source-ref', help='read production sources from a Git ref')
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    source_dir = here.parents[1] / 'src'
    def read_source(name):
        if args.source_ref:
            return subprocess.check_output(
                ['git', 'show', f'{args.source_ref}:ECGv0/src/{name}'],
                cwd=here, text=True)
        return (source_dir / name).read_text()
    filesystem = read_source('zephyrfilesystem.c')
    accel = read_source('accelRecorder.c')
    ecg = read_source('ecgRecorder.c')
    env = os.environ.copy()
    env['PATH'] = str(Path(args.cc).resolve().parent) + os.pathsep + env['PATH']

    with tempfile.TemporaryDirectory(prefix='successor-header-') as temp:
        directory = Path(temp)
        compile_and_run(
            args.cc, env, directory, here / 'successor_preallocate_harness.c',
            {'/* FILESYSTEM_PREALLOCATE_FUNCTION */': [extract_function(
                filesystem, 'filesystem_preallocate_file')]})
        compile_and_run(
            args.cc, env, directory, here / 'successor_recorder_harness.c',
            {
                '/* ACCEL_SUCCESSOR_FUNCTIONS */': [extract_function(accel, name)
                    for name in ('accel_record_prepare_next_chunk',
                                 'accel_record_activate_next_chunk',
                                 'accel_record_control_work_handler')],
                '/* ECG_SUCCESSOR_FUNCTIONS */': [extract_function(ecg, name)
                    for name in ('ecg_record_prepare_next_chunk',
                                 'ecg_record_activate_next_chunk',
                                 'ecg_record_control_work_handler')],
            })


if __name__ == '__main__':
    main()
