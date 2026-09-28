"""Tests for the shared Nav2 template and the per-robot params rendering."""

import os

from parakram_bringup.nav2_params import render_robot_params, write_robot_params
import pytest
import yaml

TEMPLATE = os.path.join(os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml')
NODES = ['amcl', 'bt_navigator', 'controller_server', 'planner_server', 'behavior_server',
         'velocity_smoother', 'collision_monitor']


@pytest.fixture(scope='module')
def template():
    with open(TEMPLATE) as f:
        return yaml.safe_load(f)


def _walk(d, path=()):
    for k, v in d.items():
        if isinstance(v, dict):
            yield from _walk(v, path + (k,))
        else:
            yield path + (k,), v


def test_template_has_every_node(template):
    for n in NODES:
        assert 'ros__parameters' in template[n], n
    assert 'ros__parameters' in template['local_costmap']['local_costmap']
    assert 'ros__parameters' in template['global_costmap']['global_costmap']


def test_template_is_namespace_agnostic(template):
    """Robot-local topics are relative; only the shared map is absolute; no use_sim_time."""
    absolute_ok = {'/map'}
    for path, value in _walk(template):
        key = path[-1]
        if key == 'use_sim_time':
            pytest.fail(f'use_sim_time must be set by the launch file, found at {path}')
        if isinstance(value, str) and (key.endswith('topic') or key == 'topic'):
            assert not value.startswith('/') or value in absolute_ok, (path, value)


def test_cmd_vel_chain(template):
    vs = template['velocity_smoother']['ros__parameters']
    cm = template['collision_monitor']['ros__parameters']
    assert cm['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert cm['cmd_vel_out_topic'] == 'cmd_vel'
    assert vs['max_velocity'][0] <= 0.22  # Burger limit


def test_groot_disabled(template):
    bt = template['bt_navigator']['ros__parameters']
    assert bt['navigate_to_pose']['enable_groot_monitoring'] is False
    assert bt['navigate_through_poses']['enable_groot_monitoring'] is False


def test_inflation_exceeds_footprint(template):
    for cm in ('local_costmap', 'global_costmap'):
        p = template[cm][cm]['ros__parameters']
        assert p['inflation_layer']['inflation_radius'] >= 0.138 + 0.1


def test_amcl_uses_beam_model(template):
    """
    CLAUDE_CODE/02 finding: the map's shelves are filled.

    The likelihood field scores an end point inside an obstacle as a perfect hit, so in dense
    multi-robot traffic AMCL could drift 0.2 m sideways; the beam model ray-casts.
    """
    assert template['amcl']['ros__parameters']['laser_model_type'] == 'beam'


def test_render_robot_params(template, tmp_path):
    out = render_robot_params(template, '/robot2/', (0.8, 0.0, 3.14159))
    assert list(out) == ['robot2']
    amcl = out['robot2']['amcl']['ros__parameters']
    assert amcl['set_initial_pose'] is True
    assert amcl['initial_pose'] == {'x': 0.8, 'y': 0.0, 'z': 0.0, 'yaw': 3.14159}
    # the template itself is untouched
    assert template['amcl']['ros__parameters']['initial_pose']['x'] == 0.0
    path = write_robot_params(TEMPLATE, str(tmp_path), 'robot2', (0.8, 0.0, 3.14159))
    with open(path) as f:
        again = yaml.safe_load(f)
    assert again == out
