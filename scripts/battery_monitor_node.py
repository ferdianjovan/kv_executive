#!/usr/bin/env python3
"""Compatibility entry point for the native passive battery monitor."""

import os
import sys
from pathlib import Path


def main():
    """Replace this process before loading any Python ROS runtime."""
    # Keep the installed path: resolving a symlink would lose the sibling binary.
    candidates = [Path(__file__).absolute().with_name('battery_monitor_node')]
    candidates.extend(
        Path(prefix) / 'lib' / 'kv_executive' / 'battery_monitor_node'
        for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
        if prefix
    )
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            os.execv(str(candidate), [str(candidate), *sys.argv[1:]])
    raise SystemExit(
        'Native battery_monitor_node not found. Build kv_executive and source '
        'install/setup.bash before running the monitor.'
    )


if __name__ == '__main__':
    main()
