"""Tests for the run manifest helpers."""

import json
import os
import uuid

from parakram_bringup import run_manifest as rm


def test_run_id_is_uuid():
    rid = rm.new_run_id()
    assert str(uuid.UUID(rid)) == rid
    assert rm.new_run_id() != rid


def test_params_hash_is_content_based(tmp_path):
    a = tmp_path / 'a.yaml'
    b = tmp_path / 'b.yaml'
    a.write_text('x: 1\n')
    b.write_text('y: 2\n')
    h1, per = rm.params_hash({'a': str(a), 'b': str(b)}, {'k': 1})
    h2, _ = rm.params_hash({'b': str(b), 'a': str(a)}, {'k': 1})
    assert h1 == h2 and set(per) == {'a', 'b'}
    assert rm.params_hash({'a': str(a), 'b': str(b)}, {'k': 2})[0] != h1
    a.write_text('x: 2\n')
    assert rm.params_hash({'a': str(a), 'b': str(b)}, {'k': 1})[0] != h1


def test_manifest_and_latest(tmp_path, monkeypatch):
    monkeypatch.setenv(rm.LOG_ROOT_ENV, str(tmp_path))
    assert rm.default_log_root() == str(tmp_path)
    rid = rm.new_run_id()
    run_dir = os.path.join(rm.default_log_root(), rid)
    m = rm.base_manifest(rid, 7, str(tmp_path))
    m.update({'n_robots': 3, 'scenario': 'intersection', 'params_hash': 'abc'})
    path = rm.write_manifest(run_dir, m)
    rm.update_latest_symlink(str(tmp_path), rid)
    rm.update_latest_symlink(str(tmp_path), rid)  # idempotent
    with open(path) as f:
        loaded = json.load(f)
    for key in ('run_id', 'seed', 'git_sha', 'n_robots', 'scenario', 'params_hash'):
        assert key in loaded
    assert loaded['seed'] == 7
    assert os.path.realpath(os.path.join(str(tmp_path), 'latest')) == os.path.realpath(run_dir)
    assert rm.load_manifest(os.path.join(str(tmp_path), 'latest'))['run_id'] == rid


def test_git_info_outside_repo(tmp_path):
    info = rm.git_info(str(tmp_path))
    assert info['git_sha'] in ('not-a-git-repo', 'unknown') or len(info['git_sha']) == 40


def test_find_workspace_root(tmp_path):
    (tmp_path / 'src').mkdir()
    (tmp_path / 'install' / 'pkg' / 'share').mkdir(parents=True)
    assert rm.find_workspace_root(str(tmp_path / 'install' / 'pkg' / 'share')) == \
        os.path.realpath(str(tmp_path))
