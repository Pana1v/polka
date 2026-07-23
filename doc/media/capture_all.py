#!/usr/bin/env python3
"""
Capture every feature config sequentially via run_capture.py subprocesses.

Each run is an isolated subprocess (avoids rclpy re-init and process-group
signal issues from looping captures in one shell). Run from the ws root with
ROS 2 + the workspace sourced.
"""
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).parent
CONFIGS = HERE / 'configs'
WORK = HERE / 'work'

# cpu_vs_cuda omitted: needs a working CUDA runtime (currently error 999).
DEFAULT = ['single', 'merged', 'filter_range', 'filter_angular', 'filter_box',
           'filter_height', 'invert_keep', 'invert_exclude', 'self_on',
           'voxel_on', 'dual']


def main():
    names = sys.argv[1:] or DEFAULT
    for cfg in names:
        out = WORK / f'cap_{cfg}'
        print(f'=== capture {cfg} ===', flush=True)
        try:
            subprocess.run(
                [sys.executable, str(HERE / 'run_capture.py'),
                 str(CONFIGS / f'{cfg}.yaml'), str(out), '--duration', '12'],
                timeout=60, check=False)
        except subprocess.TimeoutExpired:
            print(f'  {cfg}: TIMEOUT', flush=True)
    print('CAPTURE_ALL DONE', flush=True)


if __name__ == '__main__':
    main()
