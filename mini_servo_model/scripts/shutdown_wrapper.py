#!/usr/bin/env python3
"""Run a GUI node so Ctrl-C does not dump RViz/Qt abort noise.

ros2 launch sends SIGINT to launched processes. RViz2 on macOS then aborts in
its destructor. This wrapper owns the signal, SIGKILLs the child, and exits 0.
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading

_stop = threading.Event()


def _request_stop(_signum: int, _frame: object | None) -> None:
    _stop.set()


def _preexec() -> None:
    os.setsid()
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    signal.signal(signal.SIGHUP, signal.SIG_IGN)


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: shutdown_wrapper.py <executable> [args...]", file=sys.stderr)
        return 2

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)
    signal.signal(signal.SIGHUP, _request_stop)

    child = subprocess.Popen(sys.argv[1:], preexec_fn=_preexec)
    while child.poll() is None and not _stop.is_set():
        try:
            child.wait(timeout=0.1)
        except subprocess.TimeoutExpired:
            pass

    if child.poll() is None:
        try:
            os.kill(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            child.wait(timeout=1)
        except subprocess.TimeoutExpired:
            pass
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
