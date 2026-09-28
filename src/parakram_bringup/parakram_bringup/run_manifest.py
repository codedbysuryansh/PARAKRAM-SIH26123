"""
Run identity + reproducibility manifest (CLAUDE_CODE/00 "Logging + reproducibility").

Every trial gets a ``run_id`` (uuid4) and an integer ``seed``; everything a run produces goes to
``<log_root>/<run_id>/``. ``<log_root>`` is ``$PARAKRAM_LOG_ROOT`` if set, else
``<workspace>/bench/logs``. ``<log_root>/latest`` is a symlink to the most recent run.

``params_hash`` is the SHA-256 over the sorted (label, sha256(content)) pairs of every config file
that shapes the run (static configs + the per-robot files generated for it) plus the effective
simulation settings. Two runs with the same ``params_hash`` differ only in seed/loss/run_id.
"""

import datetime
import hashlib
import json
import os
import platform
import socket
import subprocess
import uuid

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
    """Return the log root directory (``$PARAKRAM_LOG_ROOT`` or ``<ws>/bench/logs``)."""
    env = os.environ.get(LOG_ROOT_ENV)
    if env:
        return os.path.abspath(env)
    try:
        from ament_index_python.packages import get_package_prefix
        ws = find_workspace_root(get_package_prefix('parakram_bringup'))
    except Exception:  # noqa: BLE001 - fall back to the current directory
        ws = None
    return os.path.join(ws or os.getcwd(), 'bench', 'logs')


def new_run_id():
    """Return a fresh run id (uuid4 hex string with dashes)."""
    return str(uuid.uuid4())


def sha256_file(path):
    """SHA-256 hex digest of a file's bytes."""
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 16), b''):
            h.update(chunk)
    return h.hexdigest()


def params_hash(files, settings):
    """
    Hash ``{label: path}`` files plus a JSON-able ``settings`` dict.

    Returns ``(hash, {label: file_sha256})``.
    """
    per_file = {label: sha256_file(path) for label, path in files.items()}
    h = hashlib.sha256()
    for label in sorted(per_file):
        h.update(f'{label}\0{per_file[label]}\n'.encode())
    h.update(json.dumps(settings, sort_keys=True).encode())
    return h.hexdigest(), per_file


def git_info(path):
    """Return ``{'git_sha', 'git_dirty'}`` for the repo containing ``path`` (if any)."""
    try:
        sha = subprocess.run(['git', '-C', path, 'rev-parse', 'HEAD'], capture_output=True,
                             text=True, timeout=5)
        if sha.returncode != 0:
            return {'git_sha': 'not-a-git-repo', 'git_dirty': None}
        status = subprocess.run(['git', '-C', path, 'status', '--porcelain'],
                                capture_output=True, text=True, timeout=5)
        return {'git_sha': sha.stdout.strip(), 'git_dirty': bool(status.stdout.strip())}
    except (OSError, subprocess.SubprocessError):
        return {'git_sha': 'unknown', 'git_dirty': None}


def write_manifest(run_dir, manifest):
    """Write ``run_manifest.json`` into ``run_dir`` and return its path."""
    os.makedirs(run_dir, exist_ok=True)
    path = os.path.join(run_dir, 'run_manifest.json')
    with open(path, 'w') as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
        f.write('\n')
    return path


def update_latest_symlink(log_root, run_id):
    """Point ``<log_root>/latest`` at ``run_id`` (relative symlink)."""
    link = os.path.join(log_root, 'latest')
    tmp = link + '.tmp'
    if os.path.lexists(tmp):
        os.remove(tmp)
    os.symlink(run_id, tmp)
    os.replace(tmp, link)


def base_manifest(run_id, seed, workspace_root):
    """Fields common to every run manifest."""
    manifest = {
        'run_id': run_id,
        'seed': int(seed),
        'created_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'host': socket.gethostname(),
        'platform': platform.platform(),
        'ros_distro': os.environ.get('ROS_DISTRO', ''),
        'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', '') or 'default',
        'workspace_root': workspace_root,
    }
    manifest.update(git_info(workspace_root or os.getcwd()))
    return manifest


def load_manifest(run_dir):
    """Load ``run_manifest.json`` from a run directory."""
    with open(os.path.join(run_dir, 'run_manifest.json'), 'r') as f:
        return json.load(f)
