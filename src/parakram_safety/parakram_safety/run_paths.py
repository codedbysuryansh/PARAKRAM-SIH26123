"""
Run-directory helpers for the safety layer.

Same convention as ``parakram_bringup.run_manifest`` (CLAUDE_CODE/00 logging): the log root is
``$PARAKRAM_LOG_ROOT`` if set, else ``<workspace>/bench/logs``, and ``latest`` points at the newest
run. Kept here so that parakram_safety does not depend on parakram_bringup (bringup includes the
safety launch, so the reverse dependency would be circular).
"""

import os

LOG_ROOT_ENV = 'PARAKRAM_LOG_ROOT'


def find_workspace_root(start):
    """Walk up from ``start`` to the colcon workspace root (a dir with ``src`` + ``install``)."""
    d = os.path.realpath(start)
    while True:
        if os.path.isdir(os.path.join(d, 'src')) and os.path.isdir(os.path.join(d, 'install')):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            return None
        d = parent


def default_log_root():
    """Return ``$PARAKRAM_LOG_ROOT`` or ``<workspace>/bench/logs``."""
    env = os.environ.get(LOG_ROOT_ENV)
    if env:
        return os.path.abspath(env)
    try:
        from ament_index_python.packages import get_package_prefix
        ws = find_workspace_root(get_package_prefix('parakram_safety'))
    except Exception:  # noqa: BLE001 - fall back to the current directory
        ws = None
    return os.path.join(ws or os.getcwd(), 'bench', 'logs')


def latest_run_dir():
    """Return the directory ``<log_root>/latest`` points at (the newest run)."""
    return os.path.realpath(os.path.join(default_log_root(), 'latest'))
