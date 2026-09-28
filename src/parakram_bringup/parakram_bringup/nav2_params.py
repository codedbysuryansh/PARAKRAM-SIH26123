"""Render the per-robot, namespaced Nav2 parameter file from the shared template."""

import copy
import os
import re

import yaml

_FIND_PKG_SHARE = re.compile(r'\$\(find-pkg-share ([A-Za-z0-9_]+)\)')


def resolve_substitutions(value):
    """Recursively replace ``$(find-pkg-share pkg)`` in string values with the share path."""
    if isinstance(value, dict):
        return {k: resolve_substitutions(v) for k, v in value.items()}
    if isinstance(value, list):
        return [resolve_substitutions(v) for v in value]
    if isinstance(value, str) and '$(find-pkg-share' in value:
        from ament_index_python.packages import get_package_share_directory
        return _FIND_PKG_SHARE.sub(lambda m: get_package_share_directory(m.group(1)), value)
    return value


def template_path():
    """Path of the shared (namespace-agnostic) Nav2 params template."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('parakram_bringup'),
                        'config', 'nav2_params.yaml')


def render_robot_params(template, namespace, initial_pose):
    """
    Return the params dict for one robot.

    ``template`` is the parsed template (node names as top-level keys); the result nests it under
    ``namespace`` (so ``/robot1/amcl`` matches) and sets AMCL's initial pose ``(x, y, yaw)``.
    """
    params = resolve_substitutions(copy.deepcopy(template))
    amcl = params['amcl']['ros__parameters']
    x, y, yaw = initial_pose
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {'x': float(x), 'y': float(y), 'z': 0.0, 'yaw': float(yaw)}
    return {namespace.strip('/'): params}


def write_robot_params(template_file, out_dir, namespace, initial_pose):
    """Render and write ``nav2_<namespace>.yaml``; return its path."""
    with open(template_file, 'r') as f:
        template = yaml.safe_load(f)
    params = render_robot_params(template, namespace, initial_pose)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f'nav2_{namespace.strip("/")}.yaml')
    with open(path, 'w') as f:
        f.write(f'# GENERATED for {namespace} from {template_file}\n')
        yaml.safe_dump(params, f, sort_keys=False, default_flow_style=None)
    return path
