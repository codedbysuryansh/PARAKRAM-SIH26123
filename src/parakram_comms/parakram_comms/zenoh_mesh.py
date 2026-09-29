r"""
Per-robot Zenoh router / session configs for the rmw_zenoh peer mesh (CLAUDE_CODE/05).

One ``rmw_zenohd`` router per robot. Every router listens on its own endpoint and CONNECTS to
every other robot's router, so the routers form a full mesh and there is no machine the fleet
depends on: losing a router cuts off at most that robot's own nodes, which keep their reactive
safety layer and spatial leases (CLAUDE_CODE/03, 02) and rejoin when it is back (routers retry
their connections forever; TRANSIENT_LOCAL topics replay the task state). A robot's nodes open
their Zenoh session (peer mode) to their own router; gossip then links them to the rest.

The files are derived from the installed rmw_zenoh_cpp defaults
(``DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5`` / ``DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5``): only
the listen / connect endpoints change, every other rmw_zenoh setting stays as shipped.

    # single-host simulation (the committed config/ files): one port per robot on 127.0.0.1
    python3 -m parakram_comms.zenoh_mesh --out config
    # hardware: one router per robot on port 7447, listening on all interfaces
    python3 -m parakram_comms.zenoh_mesh --listen-all --out ~/zenoh \
        --robots robot1=192.168.1.11,robot2=192.168.1.12,robot3=192.168.1.13
"""

import argparse
import json
import os
import re

DEFAULT_PORT = 7447
SIM_ROBOTS = 'robot1=127.0.0.1:7447,robot2=127.0.0.1:7448,robot3=127.0.0.1:7449'
TOKEN = re.compile(r"""
    (?P<ws>\s+) | (?P<line>//[^\n]*) | (?P<block>/\*.*?\*/) | (?P<str>"(?:\\.|[^"\\])*") |
    (?P<num>-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?) | (?P<ident>[A-Za-z_][A-Za-z0-9_]*) |
    (?P<punct>[{}\[\]:,])
""", re.S | re.X)


def loads_json5(text):
    """
    Parse the JSON5 subset of the zenoh config files (comments, bare keys, trailing commas).

    Anything outside that subset raises ``ValueError`` rather than being guessed at.
    """
    toks, pos = [], 0
    while pos < len(text):
        m = TOKEN.match(text, pos)
        if not m:
            raise ValueError(f'unsupported JSON5 at offset {pos}: {text[pos:pos + 40]!r}')
        pos = m.end()
        kind = m.lastgroup
        if kind in ('ws', 'line', 'block'):
            continue
        toks.append((kind, m.group()))
    out = []
    for i, (kind, tok) in enumerate(toks):
        nxt = toks[i + 1][1] if i + 1 < len(toks) else ''
        if kind == 'ident':
            if nxt == ':':
                out.append(json.dumps(tok))                  # bare key
            elif tok in ('true', 'false', 'null'):
                out.append(tok)
            else:
                raise ValueError(f'unsupported bare word {tok!r}')
        elif tok == ',' and nxt in ('}', ']'):
            continue                                         # trailing comma
        else:
            out.append(tok)
    return json.loads(''.join(out))


def parse_robots(text):
    """``'robot1=10.0.0.1,robot2=10.0.0.2:7448'`` -> [(name, host, port)] (port 7447 default)."""
    robots = []
    for item in str(text).split(','):
        if not item.strip():
            continue
        name, sep, addr = item.strip().partition('=')
        if not sep or not name or not addr:
            raise ValueError(f'expected <robot>=<host>[:<port>], got {item!r}')
        host, _, port = addr.rpartition(':') if addr.count(':') == 1 else (addr, '', '')
        robots.append((name, host or addr, int(port) if port else DEFAULT_PORT))
    if len({r[0] for r in robots}) != len(robots) or len(robots) < 2:
        raise ValueError('need at least two robots with distinct names')
    if len({(h, p) for _, h, p in robots}) != len(robots):
        raise ValueError('two robots share a router endpoint')
    return robots


def mesh_configs(router_default, session_default, robots, listen_all=False):
    """Return ``{robot: (router_cfg, session_cfg)}`` for a full router mesh."""
    out = {}
    for name, host, port in robots:
        router = json.loads(json.dumps(router_default))           # deep copy
        session = json.loads(json.dumps(session_default))
        router['listen']['endpoints'] = [f'tcp/[::]:{port}' if listen_all
                                         else f'tcp/{host}:{port}']
        router['connect']['endpoints'] = [f'tcp/{h}:{p}' for n, h, p in robots if n != name]
        session['connect']['endpoints'] = [f'tcp/localhost:{port}' if listen_all
                                           else f'tcp/{host}:{port}']
        out[name] = (router, session)
    return out


def default_configs():
    """Return the installed rmw_zenoh_cpp default router / session configs and its version."""
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('rmw_zenoh_cpp')
    cfg = {}
    for kind in ('ROUTER', 'SESSION'):
        with open(os.path.join(share, 'config', f'DEFAULT_RMW_ZENOH_{kind}_CONFIG.json5')) as f:
            cfg[kind] = loads_json5(f.read())
    version = 'unknown'
    with open(os.path.join(share, 'package.xml')) as f:
        m = re.search(r'<version>([^<]+)</version>', f.read())
        version = m.group(1) if m else version
    return cfg['ROUTER'], cfg['SESSION'], version


def write_mesh(robots, out_dir, listen_all=False):
    """Write ``zenoh_router_<robot>.json5`` + ``zenoh_session_<robot>.json5``; return paths."""
    router_default, session_default, version = default_configs()
    os.makedirs(out_dir, exist_ok=True)
    mesh = ', '.join(f'{n}={h}:{p}' for n, h, p in robots)
    paths = []
    for name, (router, session) in mesh_configs(router_default, session_default, robots,
                                                listen_all).items():
        for kind, cfg, use in (
                ('router', router, f'ZENOH_ROUTER_CONFIG_URI=<this file> ros2 run rmw_zenoh_cpp '
                                   f'rmw_zenohd   (the {name} router)'),
                ('session', session, f'export ZENOH_SESSION_CONFIG_URI=<this file>   (every '
                                     f'{name} node; RMW_IMPLEMENTATION=rmw_zenoh_cpp)')):
            path = os.path.join(out_dir, f'zenoh_{kind}_{name}.json5')
            with open(path, 'w') as f:
                f.write(f'// PARAKRAM (CLAUDE_CODE/05) Zenoh {kind} config of {name}: one router '
                        f'per robot, full router mesh.\n'
                        f'// Generated by parakram_comms.zenoh_mesh from rmw_zenoh_cpp {version} '
                        f'DEFAULT_RMW_ZENOH_{kind.upper()}_CONFIG.json5;\n'
                        f'// only listen/connect endpoints differ from it. Mesh: {mesh}.\n'
                        f'// Use: {use}\n')
                json.dump(cfg, f, indent=2)
                f.write('\n')
            paths.append(path)
    return paths


def main(argv=None):
    """Generate the per-robot configs."""
    ap = argparse.ArgumentParser(description='rmw_zenoh per-robot router mesh configs')
    ap.add_argument('--robots', default=SIM_ROBOTS,
                    help='robot1=<host>[:port],robot2=... (default: the single-host sim layout)')
    ap.add_argument('--out', required=True, help='output directory')
    ap.add_argument('--listen-all', action='store_true',
                    help='routers listen on all interfaces (hardware), sessions on localhost')
    args = ap.parse_args(argv)
    for path in write_mesh(parse_robots(args.robots), args.out, args.listen_all):
        print(path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
