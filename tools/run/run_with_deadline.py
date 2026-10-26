"""Run a command with a hard process-group deadline (default: 30 minutes).

Usage: python3 tools/run/run_with_deadline.py --seconds 120 -- python3 -m unittest ...
A timeout immediately SIGKILLs the command's process group and returns 124.
"""
from __future__ import annotations
import argparse
import math
import os
import signal
import subprocess
import sys

HARD_LIMIT_SECONDS = 1800.


def run_with_deadline(command, *, seconds=HARD_LIMIT_SECONDS, approved_long_run=False, env=None):
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('deadline seconds must be finite and positive')
    if seconds > HARD_LIMIT_SECONDS and not approved_long_run:
        raise ValueError('runs above 1800 seconds require explicit prior user approval')
    if not command:
        raise ValueError('command must not be empty')
    child_env = dict(os.environ if env is None else env)
    child_env['YAW_PROCESS_WATCHDOG_ACTIVE'] = '1'
    child = subprocess.Popen(command, env=child_env, start_new_session=True)
    try:
        return child.wait(timeout=seconds)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait()
        print(f'hard deadline exceeded ({seconds:g}s); process group killed', file=sys.stderr)
        return 124
    except BaseException:
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait()
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=HARD_LIMIT_SECONDS)
    parser.add_argument('--approved-long-run', action='store_true',
                        help='only use after explicit user approval for this longer run')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    try:
        return run_with_deadline(command, seconds=args.seconds, approved_long_run=args.approved_long_run)
    except (ValueError, OSError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    raise SystemExit(main())
