r"""
CLAUDE_CODE/06 money shot: liveness restoration after a silent kill vs packet loss.

    python3 -m parakram_bench.run_recovery_sweep --modes parakram,reauction_baseline \\
        --loss 0:60:10 --seeds 5 --plot

For every mode, loss level and seed it runs one S3 trial (``recovery_monitor --kind s3``) on the
``junction_stream`` scenario, headless, one trial at a time:

* ``parakram`` = ``recovery_mode:=lease``: spatial leases expire with no message; the watchdog
  reallocates a DEAD robot's task after the partition grace (accelerator only);
* ``reauction_baseline`` = ``recovery_mode:=release``: no leases; a silent robot's space is
  released when its task's re-auction award (watchdog -> ReAuction -> announce -> bid -> award)
  reaches each survivor: communication-positive recovery.

Both modes run the SAME scenario, task stream, watchdog and loss: ``loss_scope:=fleet``, the
per-link process of ``parakram_comms.link_loss`` on every inter-robot message (renewals,
heartbeats, announcements, bids, awards, digests), applied after delivery (no retransmission
can hide it). Per trial (``bench/logs/<run_id>/recovery_trial.json``): Gazebo ground-truth
contacts; t_kill; the victim's space freed at the first / EVERY survivor (PARAKRAM: its lease
expired there; baseline: a release message arrived) = **liveness restoration**, the Gate 2
metric that ``L(1+rho)`` bounds (``lease_expire_all_s``); **re-acquisition** = the first
survivor HOLDING movement authority over a cell the victim reserved (``restoration_s``: freed +
claim settle + acknowledgement; the trigger keeps that survivor within claiming reach so approach
travel stays out); ``entered_s`` = its footprint entering the contested junction (ground truth,
includes travel); detection; task reassignment; tasks completed in the ``post_window``;
duplicate completions; robots concurrently holding one task.

Outputs: ``results/moneyshot_flat_vs_exploding.png`` / ``.csv`` / ``.json`` (per mode and level
mean, std, 95 % CI of both metrics, censored at the window; regression slopes vs loss with their
95 % CI per mode; the Gate 1 / Gate 2 checks) and ``results/moneyshot_runs.csv``. ``--kind s4``
/ ``blackout`` run the partition and false-positive scenarios with the same machinery (outputs
``recovery_<kind>_*``). ``--aggregate-only --sweep-dir <dir>`` recomputes the outputs.
SIMULATION acceptance evidence (CLAIMS_LEDGER: TARGET / sim until hardware).
"""

import argparse
import csv
import datetime
import json
import math
import os
import statistics
import sys
import time
import uuid

from parakram_bench.run_loss_sweep import (_count, _grep, _pids, _signal, _wait_gone,
                                           BRINGUP_FAIL, LAUNCH_PATTERN, LEFTOVERS, spawn, stop)

MODES = {'parakram': 'lease', 'reauction_baseline': 'release'}
NAMES = LEFTOVERS + ('auction_node', 'task_generator', 'heartbeat_node', 'watchdog_node',
                     'recovery_coordi', 'recovery_logger')
LEASE_TTL, RHO, TICK = 2.0, 0.0, 0.1
RUN_COLUMNS = [
    'mode', 'loss_pct', 'seed', 'run_id', 'status', 'attempt', 'wall_s', 'kind', 'victim',
    'victim_task', 'trigger', 'n_contacts', 'min_rr_distance_m', 'restored', 'restoration_s',
    'entered_s', 'lease_expire_s', 'lease_expire_all_s', 'space_reclaimed_s', 'detect_s',
    'task_reassigned_s', 'duplicates', 'concurrent_holding', 'concurrent_execution',
    'completions', 'tasks_completed_post', 'throughput_post', 'freezes', 'victim_moved_m',
    'victim_task_completed_by', 'error']
CURVE_COLUMNS = [
    'mode', 'loss_pct', 'n_runs', 'freed_all_n', 'freed_all_mean', 'freed_all_std',
    'freed_all_ci95', 'freed_all_median', 'freed_all_max', 'reacquired_n', 'reacquired_mean',
    'reacquired_std', 'reacquired_ci95', 'reacquired_max', 'entered_mean', 'entered_max',
    'freed_first_mean', 'detect_mean', 'task_reassigned_mean', 'contacts_total',
    'duplicates_total', 'concurrent_execution_total', 'throughput_mean', 'throughput_std',
    'freezes_mean', 'run_ids']


def parse_loss(spec):
    """``'0:60:10'`` -> [0, 10, ..., 60]; ``'0,30,60'`` -> [0, 30, 60] (percent)."""
    if ':' in spec:
        a, b, c = (int(x) for x in spec.split(':'))
        levels = list(range(a, b + 1, c))
    else:
        levels = [int(x) for x in spec.split(',') if x.strip()]
    if not levels or any(not 0 <= v < 100 for v in levels):
        raise ValueError(f'loss levels must be percentages in [0, 100): {spec!r}')
    return levels


def _num(v):
    if v is None or v == '' or v == 'None':
        return None
    return float(v)


def _stats(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return None, None, None, 0
    m = statistics.fmean(vals)
    s = statistics.stdev(vals) if len(vals) > 1 else 0.0
    ci = None
    if len(vals) > 1:
        from scipy.stats import t as student
        ci = student.ppf(0.975, len(vals) - 1) * s / math.sqrt(len(vals))
    return m, s, ci, len(vals)


def regression(xs, ys):
    """Least-squares slope of ys on xs with its 95 % CI: (slope, lo, hi, n)."""
    pts = [(x, y) for x, y in zip(xs, ys) if y is not None]
    n = len(pts)
    if n < 3 or len({x for x, _ in pts}) < 2:
        return None, None, None, n
    mx = statistics.fmean(x for x, _ in pts)
    my = statistics.fmean(y for _, y in pts)
    sxx = sum((x - mx) ** 2 for x, _ in pts)
    slope = sum((x - mx) * (y - my) for x, y in pts) / sxx
    resid = sum((y - my - slope * (x - mx)) ** 2 for x, y in pts)
    se = math.sqrt(resid / (n - 2) / sxx)
    from scipy.stats import t as student
    h = student.ppf(0.975, n - 2) * se
    return slope, slope - h, slope + h, n


def _censored(rows, key, ok, window):
    """Per-trial values of ``key``, a trial where ``ok(row)`` fails counting as the window."""
    return [(_num(r[key]) if ok(r) and _num(r[key]) is not None else window) for r in rows]


def _freed(r):
    return _num(r['lease_expire_all_s']) is not None


def _reacquired(r):
    return str(r['restored']) == 'True'


def aggregate(rows, modes, levels, window):
    """
    Per mode / level statistics over ok trials.

    ``freed_all`` = liveness restoration (Gate 2): kill -> the victim's reserved space is free
    at EVERY survivor (PARAKRAM: its lease expired there, no message; baseline: a release
    message arrived). ``reacquired`` = kill -> a survivor HOLDS movement authority over a cell
    the victim had reserved (freed + re-claimed: claim settle and acknowledgement). A trial
    where that did not happen within the post window counts as the window (censored).
    """
    curve = []
    for mode in modes:
        for lv in levels:
            rs = [r for r in rows if r['mode'] == mode and int(_num(r['loss_pct'])) == lv and
                  r['status'] == 'ok']
            freed = _censored(rs, 'lease_expire_all_s', _freed, window)
            reac = _censored(rs, 'restoration_s', _reacquired, window)
            fm, fs, fci, _ = _stats(freed)
            rm_, rsd, rci, _ = _stats(reac)
            thr = _stats([_num(r['throughput_post']) for r in rs])
            first = [_num(r['lease_expire_s']) for r in rs]
            ent = [_num(r.get('entered_s')) for r in rs]
            curve.append({
                'mode': mode, 'loss_pct': lv, 'n_runs': len(rs),
                'freed_all_n': sum(1 for r in rs if _freed(r)),
                'freed_all_mean': fm, 'freed_all_std': fs, 'freed_all_ci95': fci,
                'freed_all_median': statistics.median(freed) if freed else None,
                'freed_all_max': max(freed) if freed else None,
                'reacquired_n': sum(1 for r in rs if _reacquired(r)),
                'reacquired_mean': rm_, 'reacquired_std': rsd, 'reacquired_ci95': rci,
                'reacquired_max': max(reac) if reac else None,
                'entered_mean': _stats(ent)[0],
                'entered_max': max([v for v in ent if v is not None], default=None),
                'freed_first_mean': _stats(first)[0],
                'detect_mean': _stats([_num(r['detect_s']) for r in rs])[0],
                'task_reassigned_mean': _stats([_num(r['task_reassigned_s']) for r in rs])[0],
                'contacts_total': int(sum(_num(r['n_contacts']) or 0 for r in rs)),
                'duplicates_total': int(sum(_num(r['duplicates']) or 0 for r in rs)),
                'concurrent_execution_total': int(sum(_num(r.get('concurrent_execution'))
                                                      or 0 for r in rs)),
                'throughput_mean': thr[0], 'throughput_std': thr[1],
                'freezes_mean': _stats([_num(r['freezes']) for r in rs])[0],
                'run_ids': ' '.join(r['run_id'] for r in rs)})
    return curve


def _slope(out, prefix, xs, ys):
    slope, lo, hi, n = regression(xs, ys)
    out[f'{prefix}_slope_s_per_pct'] = slope
    out[f'{prefix}_slope_ci95'] = [lo, hi]
    out[f'{prefix}_slope_ci_contains_0'] = None if lo is None else lo <= 0.0 <= hi
    return lo, hi


def checks(curve, rows, modes, levels, seeds, window):
    """Gate 1 / Gate 2 as CLAUDE_CODE/06 states them (targets, not guarantees)."""
    out = {'complete': all(c['n_runs'] == seeds for c in curve)}
    out['gate1_zero_collisions'] = out['complete'] and all(
        c['contacts_total'] == 0 for c in curve)
    for mode in modes:                      # no duplicate completion, no two robots working
        out[f'{mode}_at_most_once_every_trial'] = all(
            c['duplicates_total'] == 0 and c['concurrent_execution_total'] == 0
            for c in curve if c['mode'] == mode)
    for mode in modes:
        rs = [r for r in rows if r['mode'] == mode and r['status'] == 'ok']
        xs = [_num(r['loss_pct']) for r in rs]
        _slope(out, f'{mode}_freed_all', xs, _censored(rs, 'lease_expire_all_s', _freed, window))
        out[f'{mode}_freed_all_censored_trials'] = sum(1 for r in rs if not _freed(r))
        reac = _censored(rs, 'restoration_s', _reacquired, window)
        _slope(out, f'{mode}_reacquired', xs, reac)
        out[f'{mode}_reacquired_censored_trials'] = sum(1 for r in rs if not _reacquired(r))
        mine = [c for c in curve if c['mode'] == mode]
        out[f'{mode}_throughput_positive_every_level'] = all(
            (c['throughput_mean'] or 0) > 0 for c in mine)
    if 'parakram' in modes:
        mine = [r for r in rows if r['mode'] == 'parakram' and r['status'] == 'ok']
        freed = _censored(mine, 'lease_expire_all_s', _freed, window)
        bound = LEASE_TTL * (1 + RHO)
        out['L_1_plus_rho_s'] = bound
        out['tick_tolerance_s'] = TICK
        out['parakram_worst_freed_all_s'] = max(freed, default=None)
        out['parakram_worst_freed_all_within_L_1_plus_rho_plus_tick'] = bool(
            freed and max(freed) <= bound + TICK)
        out['parakram_worst_reacquired_s'] = max(
            _censored(mine, 'restoration_s', _reacquired, window), default=None)
        out['gate2_parakram_flat'] = bool(out.get('parakram_freed_all_slope_ci_contains_0')) \
            and out['parakram_worst_freed_all_within_L_1_plus_rho_plus_tick'] \
            and out['parakram_throughput_positive_every_level']
        out['parakram_reacquired_flat_too'] = bool(
            out.get('parakram_reacquired_slope_ci_contains_0'))
    if 'reauction_baseline' in modes:
        at40 = [c for c in curve if c['mode'] == 'reauction_baseline' and c['loss_pct'] == 40]
        mean40 = at40[0]['freed_all_mean'] if at40 else None
        out['baseline_freed_all_at_40pct_mean_s'] = mean40
        out['baseline_reacquired_at_40pct_mean_s'] = at40[0]['reacquired_mean'] if at40 \
            else None
        out['baseline_exceeds_10s_by_40pct'] = bool(mean40 is not None and mean40 > 10.0)
        lo = out.get('reauction_baseline_freed_all_slope_ci95', [None])[0]
        out['baseline_diverges_slope_ci_above_0'] = lo is not None and lo > 0.0
    out['note'] = (
        'Gate 2 metric (liveness restoration, the quantity L(1+rho) bounds; CLAIMS_LEDGER T2 '
        '~1.5-2.5 s): freed_all = kill -> the reserved space of the victim is free at EVERY '
        'survivor (PARAKRAM: lease expiry, no message; baseline: release message), per-trial '
        'column lease_expire_all_s. Also reported: reacquired = kill -> a survivor HOLDS '
        'movement authority over a cell the victim reserved (per-trial column restoration_s: '
        'freed + claim settle + acknowledgement round trip, so it cannot be <= L), and '
        'entered_s (ground-truth entry into the junction, includes travel). A trial where '
        'freed / reacquired did not happen within the post window counts as the window '
        '(censored). The L(1+rho) bound allows one coordination tick (0.1 s) of detection '
        'granularity.')
    return out


def write_csv(path, columns, rows):
    """Write dict rows with fixed columns (floats to 4 decimals)."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=columns, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else ('' if v is None else v))
                        for k, v in r.items()})


def read_runs(path):
    """All attempts in a sweep's runs.csv."""
    if not os.path.isfile(path):
        return []
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def latest_ok(rows):
    """Return the last ok attempt per (mode, loss, seed)."""
    best = {}
    for r in rows:
        if r['status'] == 'ok':
            best[(r['mode'], int(_num(r['loss_pct'])), int(_num(r['seed'])))] = r
    return [best[k] for k in sorted(best)]


def plot(curve, rows, path, title, caption, window):
    """Plot the money shot: liveness restoration, re-acquisition and throughput vs loss."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    colors = {'parakram': 'tab:blue', 'reauction_baseline': 'tab:red'}
    fig, ax = plt.subplots(1, 3, figsize=(17, 5.2))
    for mode in colors:
        cm = [c for c in curve if c['mode'] == mode and c['n_runs']]
        if not cm:
            continue
        x = [c['loss_pct'] for c in cm]
        mine = [r for r in rows if r['mode'] == mode and r['status'] == 'ok']
        lx = [int(_num(r['loss_pct'])) for r in mine]
        freed_by = 'lease expiry, no message' if mode == 'parakram' else 'release message'
        for a, key, ok, mean, ci in (
                (ax[0], 'lease_expire_all_s', _freed, 'freed_all_mean', 'freed_all_ci95'),
                (ax[1], 'restoration_s', _reacquired, 'reacquired_mean', 'reacquired_ci95')):
            a.scatter(lx, _censored(mine, key, ok, window), s=12, color=colors[mode],
                      alpha=0.35)
            a.errorbar(x, [c[mean] for c in cm], yerr=[c[ci] or 0.0 for c in cm], marker='o',
                       capsize=4, color=colors[mode],
                       label=f'{mode} (mean, 95 % CI)' + (f': {freed_by}' if a is ax[0]
                                                          else ''))
        ax[2].errorbar(x, [c['throughput_mean'] for c in cm],
                       yerr=[c['throughput_std'] or 0.0 for c in cm], marker='o', capsize=4,
                       color=colors[mode], label=mode)
    ax[0].axhline(LEASE_TTL * (1 + RHO), ls='--', color='0.4', label='L(1+rho)')
    ax[0].set_ylabel("liveness restoration [s after the kill]\n(the victim's reserved space "
                     'free at EVERY survivor)')
    ax[1].axhline(LEASE_TTL * (1 + RHO), ls='--', color='0.4', label='L(1+rho)')
    ax[1].set_ylabel('re-acquisition [s after the kill]\n(a survivor holds authority '
                     "through the victim's space)")
    ax[2].set_ylabel('tasks completed per minute after the kill')
    for a in ax:
        a.set_ylim(bottom=0)                           # no zoom: flat vs exploding at scale
    for a, t in zip(ax, ('liveness restoration vs loss (Gate 2)', 're-acquisition vs loss',
                         'throughput vs loss')):
        a.set_xlabel('packet loss on every inter-robot link [%]')
        a.set_title(t)
        a.grid(alpha=0.3)
        a.legend(fontsize=7, loc='upper left')
    fig.suptitle(title, fontsize=11)
    fig.text(0.5, 0.005, caption, ha='center', fontsize=8, color='0.3')
    fig.tight_layout(rect=(0, 0.04, 1, 0.94))
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------------------------- processes
def leftovers():
    """Return the pids of simulation / launch processes still alive."""
    pids = _pids(['-f', LAUNCH_PATTERN]) + _pids(['-f', 'parakram_bench.recovery_monitor'])
    for name in NAMES:
        pids += _pids(['-x', name])
    return sorted(set(pids))


def cleanup(log=print):
    """SIGINT stray launches, then TERM / KILL every leftover process."""
    for pid in _pids(['-f', LAUNCH_PATTERN]):
        _signal(pid, 2)
    _wait_gone(lambda: _pids(['-f', LAUNCH_PATTERN]), 30.0)
    for sig, wait in ((15, 5.0), (9, 5.0)):
        pids = leftovers()
        if not pids:
            return True
        if sig == 9:
            log(f'SIGKILL leftovers {pids}')
        for pid in pids:
            _signal(pid, sig)
        _wait_gone(leftovers, wait)
    return not leftovers()


def run_trial(args, sweep_dir, log_root, mode, lv, seed, attempt, env):
    """One trial; returns its runs.csv row."""
    run_id = str(uuid.uuid4())
    run_dir = os.path.join(log_root, run_id)
    tag = f'{mode}_loss{lv:02d}_seed{seed}_try{attempt}'
    logs = {k: os.path.join(sweep_dir, 'logs', f'{tag}_{k}.log')
            for k in ('fleet', 'monitor', 'coord', 'fault', 'tasks')}
    row = {'mode': mode, 'loss_pct': lv, 'seed': seed, 'run_id': run_id, 'attempt': attempt,
           'kind': args.kind, 'status': 'running', 'error': ''}
    n = args.n_robots
    t0 = time.monotonic()
    procs = []
    try:
        if not cleanup():
            raise RuntimeError(f'could not clear leftover processes {leftovers()}')
        fleet = spawn(['ros2', 'launch', 'parakram_bringup', 'fleet_sim.launch.py',
                       f'n_robots:={n}', f'scenario:={args.scenario}', f'seed:={seed}',
                       f'loss:={lv / 100.0}', f'loss_model:={args.loss_model}',
                       f'loss_burst_corr:={args.burst_corr}', 'loss_scope:=fleet',
                       f'recovery_mode:={MODES[mode]}', f'run_id:={run_id}'],
                      logs['fleet'], env)
        procs.append(fleet)
        end = time.monotonic() + args.bringup_timeout
        while _count(logs['fleet'], 'Managed nodes are active') < 2 * n + 1:
            bad = _grep(logs['fleet'], BRINGUP_FAIL)
            if bad or fleet.poll() is not None or time.monotonic() > end:
                why = bad[0][-160:] if bad else 'bringup timeout / exit'
                row.update(status='bringup_failed', error=why)
                return row
            time.sleep(2.0)
        monitor = spawn([sys.executable, '-m', 'parakram_bench.recovery_monitor',
                         '--run-dir', run_dir, '--kind', args.kind, '--n', str(n),
                         '--warmup', str(args.warmup), '--post-window', str(args.post_window),
                         '--trigger-primary', str(args.trigger_primary),
                         '--trigger-timeout', str(args.trigger_timeout),
                         '--partition', str(args.partition), '--blackout', str(args.blackout),
                         '--loss', str(lv / 100.0)],
                        logs['monitor'], env)
        procs.append(monitor)
        time.sleep(2.0)
        for key, cmd in (
                ('coord', ['ros2', 'launch', 'parakram_coord', 'coord.launch.py',
                           f'n_robots:={n}', f'scenario:={args.scenario}',
                           f'run_dir:={run_dir}']),
                ('fault', ['ros2', 'launch', 'parakram_fault', 'fault.launch.py',
                           f'n_robots:={n}', f'run_dir:={run_dir}']),
                ('tasks', ['ros2', 'launch', 'parakram_tasks', 'tasks.launch.py',
                           f'n_robots:={n}', f'run_dir:={run_dir}',
                           f'n_tasks:={args.n_tasks}', f'task_rate:={args.task_rate}'])):
            procs.append(spawn(cmd, logs[key], env))
            time.sleep(1.0)
        sim_span = args.warmup + args.trigger_timeout + args.post_window
        limit = time.monotonic() + 240.0 + 3.0 * sim_span
        while monitor.poll() is None:
            if fleet.poll() is not None:
                row.update(status='launch_exited', error='fleet launch exited')
                return row
            if time.monotonic() > limit:
                row.update(status='monitor_timeout', error='monitor wall limit')
                return row
            time.sleep(2.0)
    finally:
        stop(procs)
        cleanup()
        row['wall_s'] = round(time.monotonic() - t0, 1)
    try:
        with open(os.path.join(run_dir, 'recovery_trial.json')) as f:
            res = json.load(f)
    except (OSError, ValueError) as exc:
        row.update(status='no_result', error=f'recovery_trial.json: {exc}')
        return row
    if res.get('error'):
        row.update(status='no_trigger' if 'no ' in res['error'] else 'no_result',
                   error=res['error'])
        return row
    row.update({k: res.get(k) for k in (
        'victim', 'victim_task', 'trigger', 'n_contacts', 'min_rr_distance_m', 'restored',
        'restoration_s', 'entered_s', 'lease_expire_s', 'lease_expire_all_s', 'space_reclaimed_s',
        'detect_s', 'task_reassigned_s', 'completions', 'tasks_completed_post')})
    row['throughput_post'] = res.get('throughput_post_tasks_per_min')
    row['duplicates'] = len(res.get('duplicate_completions') or [])
    row['concurrent_holding'] = len(res.get('concurrent_holding') or [])
    row['concurrent_execution'] = len(res.get('concurrent_execution') or [])
    row['freezes'] = len(res.get('lease_lost_events') or [])
    row['victim_moved_m'] = res.get('victim_moved_while_isolated_m')
    row['victim_task_completed_by'] = ' '.join(res.get('victim_task_completed_by') or [])
    row['status'] = 'ok'
    return row


def append_run(path, row):
    """Append one attempt (keeping the file's own header)."""
    fields = RUN_COLUMNS
    if os.path.isfile(path):
        with open(path, newline='') as f:
            fields = next(csv.reader(f), None) or RUN_COLUMNS
    new = not os.path.isfile(path)
    with open(path, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        if new:
            w.writeheader()
        w.writerow({k: ('' if v is None else v) for k, v in row.items()})


def write_outputs(sweep_dir, manifest, results_dir):
    """Aggregate the ok trials into CSV / JSON / PNG."""
    attempts = read_runs(os.path.join(sweep_dir, 'runs.csv'))
    rows = latest_ok(attempts)
    modes, levels, seeds = manifest['modes'], manifest['levels'], manifest['seeds']
    window = manifest['post_window']
    curve = aggregate(rows, modes, levels, window)
    kind = manifest['kind']
    name = 'moneyshot_flat_vs_exploding' if kind == 's3' else f'recovery_{kind}'
    result = {'sweep_id': manifest['sweep_id'], 'sweep_dir': sweep_dir,
              'generated_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'claim_status': 'SIMULATION acceptance evidence (CLAUDE_CODE/06); TARGET/sim in '
                              'the CLAIMS_LEDGER until hardware',
              'manifest': manifest, 'curve': curve,
              'checks': checks(curve, rows, modes, levels, len(seeds), window)
              if kind == 's3' else {},
              'attempts_not_ok': [{k: r.get(k) for k in ('mode', 'loss_pct', 'seed', 'attempt',
                                                         'run_id', 'status', 'error')}
                                  for r in attempts if r['status'] != 'ok']}
    title = (f"CLAUDE_CODE/06 {kind.upper()} (simulation): {manifest['n_robots']} robots, "
             f"'{manifest['scenario']}', {manifest['loss_model']} loss on every inter-robot "
             f'link, seeds {seeds[0]}..{seeds[-1]}')
    caption = (f'SIMULATION, Gazebo ground truth; TARGET/sim, not a hardware claim.  sweep '
               f"{manifest['sweep_id']}  git {manifest.get('git_sha', '')[:10]}"
               f"{' (dirty)' if manifest.get('git_dirty') else ''}")
    for d in [sweep_dir] + ([results_dir] if results_dir else []):
        write_csv(os.path.join(d, f'{name}.csv'), CURVE_COLUMNS, curve)
        write_csv(os.path.join(d, name.replace('flat_vs_exploding', 'runs') + (
            '' if kind == 's3' else '_runs') + '.csv'), RUN_COLUMNS, rows)
        with open(os.path.join(d, f'{name}.json'), 'w') as f:
            json.dump(result, f, indent=2, default=str)
        if rows and kind == 's3':
            plot(curve, rows, os.path.join(d, f'{name}.png'), title, caption, window)
    return result


def main(argv=None):
    """Run (or resume, or re-aggregate) a recovery sweep."""
    ap = argparse.ArgumentParser(description='CLAUDE_CODE/06 recovery sweep (money shot)')
    ap.add_argument('--modes', default='parakram,reauction_baseline')
    ap.add_argument('--loss', default='0:60:10', help='a:b:step or a,b,c [%%]')
    ap.add_argument('--seeds', type=int, default=5)
    ap.add_argument('--seed-list', default='')
    ap.add_argument('--plot', action='store_true', help='(always plotted; kept for the spec)')
    ap.add_argument('--kind', default='s3', choices=('s3', 's4', 'blackout'))
    ap.add_argument('--scenario', default='junction_stream')
    ap.add_argument('--n-robots', type=int, default=3)
    ap.add_argument('--loss-model', default='bernoulli', choices=('bernoulli', 'gilbert_elliott'))
    ap.add_argument('--burst-corr', type=float, default=0.8)
    ap.add_argument('--n-tasks', type=int, default=80)
    ap.add_argument('--task-rate', type=float, default=0.25)
    ap.add_argument('--warmup', type=float, default=20.0)
    ap.add_argument('--trigger-primary', type=float, default=30.0)
    ap.add_argument('--trigger-timeout', type=float, default=180.0)
    ap.add_argument('--post-window', type=float, default=90.0)
    ap.add_argument('--partition', type=float, default=30.0)
    ap.add_argument('--blackout', type=float, default=5.0)
    ap.add_argument('--bringup-timeout', type=float, default=240.0)
    ap.add_argument('--retries', type=int, default=2)
    ap.add_argument('--sweep-dir', default='')
    ap.add_argument('--aggregate-only', action='store_true')
    ap.add_argument('--results-dir', default='')
    ap.add_argument('--no-results', action='store_true')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args(argv)

    from parakram_bringup import run_manifest as rm
    modes = [m for m in args.modes.split(',') if m]
    for m in modes:
        if m not in MODES:
            raise SystemExit(f'unknown mode {m}; choose from {list(MODES)}')
    levels = parse_loss(args.loss)
    seeds = ([int(s) for s in args.seed_list.split(',') if s.strip()] if args.seed_list
             else list(range(1, args.seeds + 1)))
    log_root = rm.default_log_root()
    try:
        from ament_index_python.packages import get_package_prefix
        ws = rm.find_workspace_root(get_package_prefix('parakram_bringup'))
    except Exception:  # noqa: BLE001
        ws = None
    results_dir = None if args.no_results else (
        args.results_dir or os.path.join(ws or os.getcwd(), 'src', 'parakram_bench', 'results'))
    if args.sweep_dir:
        sweep_dir = os.path.abspath(args.sweep_dir)
        with open(os.path.join(sweep_dir, 'sweep_manifest.json')) as f:
            manifest = json.load(f)
    else:
        sweep_id = str(uuid.uuid4())[:8]
        sweep_dir = os.path.join(log_root, f'recovery_sweep_{sweep_id}')
        manifest = rm.base_manifest(sweep_id, seeds[0], ws)
        manifest.pop('seed', None)
        manifest.pop('run_id', None)
        manifest.update({
            'kind': args.kind, 'sweep_id': sweep_id, 'work_order': 'CLAUDE_CODE/06',
            'modes': modes, 'mode_map': MODES, 'levels': levels, 'seeds': seeds,
            'scenario': args.scenario, 'n_robots': args.n_robots,
            'loss_model': args.loss_model, 'burst_corr': args.burst_corr,
            'loss_scope': 'fleet (every inter-robot topic, per-link process)',
            'n_tasks': args.n_tasks, 'task_rate': args.task_rate, 'warmup': args.warmup,
            'trigger_primary': args.trigger_primary, 'trigger_timeout': args.trigger_timeout,
            'post_window': args.post_window, 'partition': args.partition,
            'blackout': args.blackout, 'lease_ttl': LEASE_TTL, 'rho': RHO,
            'ground_truth': 'Gazebo /ground_truth/poses (parakram_bench.recovery_monitor)',
            'command': ' '.join([os.path.basename(sys.executable), '-m',
                                 'parakram_bench.run_recovery_sweep'] + list(argv or
                                                                             sys.argv[1:])),
            'log_root': log_root})
    runs_csv = os.path.join(sweep_dir, 'runs.csv')
    if args.aggregate_only:
        res = write_outputs(sweep_dir, manifest, results_dir)
        print(json.dumps(res['checks'], indent=1, default=str))
        return 0
    plan = [(m, lv, s) for lv in manifest['levels'] for m in manifest['modes']
            for s in manifest['seeds']]
    done = {(r['mode'], int(_num(r['loss_pct'])), int(_num(r['seed'])))
            for r in latest_ok(read_runs(runs_csv))}
    todo = [p for p in plan if p not in done]
    print(f'[recovery] {sweep_dir}: {len(plan)} trials, {len(todo)} to do', flush=True)
    if args.dry_run:
        for p in todo:
            print('  ', p)
        return 0
    for k in ('kind', 'scenario', 'n_robots', 'loss_model', 'burst_corr', 'n_tasks',
              'task_rate', 'warmup', 'trigger_primary', 'trigger_timeout', 'post_window',
              'partition', 'blackout'):
        setattr(args, k, manifest[k])
    os.makedirs(os.path.join(sweep_dir, 'logs'), exist_ok=True)
    rm.write_manifest(sweep_dir, manifest)
    os.replace(os.path.join(sweep_dir, 'run_manifest.json'),
               os.path.join(sweep_dir, 'sweep_manifest.json'))
    env = dict(os.environ)
    env.setdefault('TURTLEBOT3_MODEL', 'burger')
    lock = os.path.join(log_root, 'SIM_BUSY')
    with open(lock, 'w') as f:
        f.write(f'{os.getpid()} recovery sweep {sweep_dir}\n')
    try:
        for i, (mode, lv, s) in enumerate(todo, 1):
            for attempt in range(1, args.retries + 2):
                row = run_trial(args, sweep_dir, log_root, mode, lv, s, attempt, env)
                append_run(runs_csv, row)
                print(f'[recovery] {i}/{len(todo)} {mode} loss {lv}% seed {s} try {attempt}: '
                      f"{row['status']} run_id={row['run_id']} victim={row.get('victim')} "
                      f"contacts={row.get('n_contacts')} restore={row.get('restoration_s')} "
                      f"lease_all={row.get('lease_expire_all_s')} "
                      f"thr={row.get('throughput_post')} dup={row.get('duplicates')} "
                      f"wall={row.get('wall_s')}s"
                      + (f" ({row['error']})" if row['error'] else ''), flush=True)
                if row['status'] == 'ok':
                    break
            try:
                write_outputs(sweep_dir, manifest, None)
            except Exception as exc:  # noqa: BLE001 - a report must never stop the sweep
                print(f'[recovery] progress report failed: {exc!r}', flush=True)
    except KeyboardInterrupt:
        print('[recovery] interrupted; resume with --sweep-dir ' + sweep_dir, flush=True)
        cleanup()
        return 130
    finally:
        if os.path.exists(lock):
            os.remove(lock)
    res = write_outputs(sweep_dir, manifest, results_dir)
    print(json.dumps(res['checks'], indent=1, default=str))
    print(f'[recovery] done: {sweep_dir}' + (f' -> {results_dir}' if results_dir else ''))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
