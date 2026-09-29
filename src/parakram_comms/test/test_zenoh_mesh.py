"""Unit tests of the per-robot Zenoh router mesh configs (CLAUDE_CODE/05)."""

import filecmp
import os

from parakram_comms import zenoh_mesh
import pytest

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')


def test_json5_subset_parser():
    text = """
    // header comment
    {
      mode: "router", /* block
      comment */
      listen: { endpoints: [ "tcp/[::]:7447", ], },   // trailing comma + inline comment
      nested: { flag: true, none: null, n: -1, x: 2.5e3, url: "http://a//b" },
    }"""
    cfg = zenoh_mesh.loads_json5(text)
    assert cfg == {'mode': 'router', 'listen': {'endpoints': ['tcp/[::]:7447']},
                   'nested': {'flag': True, 'none': None, 'n': -1, 'x': 2500.0,
                              'url': 'http://a//b'}}
    for bad in ("{ a: 'single' }", '{ a: Infinity }', '{ a: 0x10 }'):
        with pytest.raises(ValueError):
            zenoh_mesh.loads_json5(bad)


def test_parse_robots():
    assert zenoh_mesh.parse_robots(zenoh_mesh.SIM_ROBOTS) == [
        ('robot1', '127.0.0.1', 7447), ('robot2', '127.0.0.1', 7448),
        ('robot3', '127.0.0.1', 7449)]
    assert zenoh_mesh.parse_robots('a=10.0.0.1,b=10.0.0.2') == [('a', '10.0.0.1', 7447),
                                                                ('b', '10.0.0.2', 7447)]
    for bad in ('robot1=10.0.0.1', 'a=10.0.0.1,a=10.0.0.2', 'a=h:7447,b=h:7447', 'a'):
        with pytest.raises(ValueError):
            zenoh_mesh.parse_robots(bad)


def test_every_router_connects_to_every_other():
    robots = zenoh_mesh.parse_robots(zenoh_mesh.SIM_ROBOTS)
    router = {'mode': 'router', 'listen': {'endpoints': []}, 'connect': {'endpoints': []},
              'keep': {'x': 1}}
    session = {'mode': 'peer', 'listen': {'endpoints': ['tcp/localhost:0']},
               'connect': {'endpoints': []}}
    mesh = zenoh_mesh.mesh_configs(router, session, robots)
    for name, host, port in robots:
        r, s = mesh[name]
        assert r['listen']['endpoints'] == [f'tcp/{host}:{port}']
        assert sorted(r['connect']['endpoints']) == sorted(
            f'tcp/{h}:{p}' for n, h, p in robots if n != name)
        assert s['connect']['endpoints'] == [f'tcp/{host}:{port}']
        assert r['keep'] == {'x': 1} and s['listen'] == session['listen']   # untouched
    assert router['connect']['endpoints'] == []                             # defaults intact
    hw = zenoh_mesh.mesh_configs(router, session, zenoh_mesh.parse_robots('a=10.0.0.1,b=10.0.0.2'),
                                 listen_all=True)
    assert hw['a'][0]['listen']['endpoints'] == ['tcp/[::]:7447']
    assert hw['a'][0]['connect']['endpoints'] == ['tcp/10.0.0.2:7447']
    assert hw['a'][1]['connect']['endpoints'] == ['tcp/localhost:7447']


def test_committed_configs_match_the_installed_rmw_zenoh(tmp_path):
    try:
        zenoh_mesh.default_configs()
    except Exception:  # noqa: BLE001 - rmw_zenoh_cpp not installed here
        pytest.skip('rmw_zenoh_cpp is not installed')
    paths = zenoh_mesh.write_mesh(zenoh_mesh.parse_robots(zenoh_mesh.SIM_ROBOTS), str(tmp_path))
    assert len(paths) == 6
    for path in paths:
        committed = os.path.join(CONFIG, os.path.basename(path))
        assert filecmp.cmp(path, committed, shallow=False), f'{committed} is stale: regenerate'
