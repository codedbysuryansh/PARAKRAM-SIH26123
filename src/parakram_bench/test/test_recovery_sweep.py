"""Unit tests of the CLAUDE_CODE/06 money-shot statistics (no simulation)."""

from parakram_bench import run_recovery_sweep as rs
import pytest


def test_parse_loss_specs():
    assert rs.parse_loss('0:60:10') == [0, 10, 20, 30, 40, 50, 60]
    assert rs.parse_loss('0,30') == [0, 30]
    with pytest.raises(ValueError):
        rs.parse_loss('0:100:50')


def test_regression_slope_and_ci():
    xs = [0, 10, 20, 30] * 3
    flat = [2.0, 2.1, 1.9, 2.0, 2.05, 1.95, 2.0, 2.1, 1.9, 2.02, 1.98, 2.0]
    slope, lo, hi, n = rs.regression(xs, flat)
    assert n == 12 and lo <= 0.0 <= hi and abs(slope) < 0.01
    rising = [2.0 + 0.3 * x + d for x, d in zip(xs, [0.1, -0.1, 0.05, 0.0] * 3)]
    slope, lo, hi, _ = rs.regression(xs, rising)
    assert slope == pytest.approx(0.3, abs=0.01) and lo > 0.0
    assert rs.regression([0, 0, 0], [1, 2, 3])[0] is None          # no spread in x


def row(mode, loss, seed, restoration, lease_all=1.9, thr=2.0, contacts=0, restored=True):
    return {'mode': mode, 'loss_pct': str(loss), 'seed': str(seed),
            'run_id': f'{mode}{loss}{seed}',
            'status': 'ok', 'restored': str(restored), 'restoration_s': str(restoration),
            'lease_expire_s': str(lease_all - 0.1), 'lease_expire_all_s': str(lease_all),
            'space_reclaimed_s': '2.3', 'detect_s': '1.1', 'task_reassigned_s': '11.0',
            'n_contacts': str(contacts), 'duplicates': '0', 'throughput_post': str(thr),
            'freezes': '1'}


def test_aggregate_and_gates():
    levels, seeds = [0, 20, 40, 60], [1, 2, 3]
    rows = [row('parakram', lv, s, 3.0 + 0.1 * s) for lv in levels for s in seeds]
    rows += [row('reauction_baseline', lv, s, 3.0 + 0.4 * lv + s, lease_all=2.5 + 0.4 * lv)
             for lv in levels for s in seeds]
    rows[-1]['restored'] = 'False'                     # not re-acquired: counts as the window
    rows[-2]['lease_expire_all_s'] = ''                # not freed everywhere: the window too
    modes = ['parakram', 'reauction_baseline']
    curve = rs.aggregate(rows, modes, levels, window=60.0)
    assert len(curve) == 8 and all(c['n_runs'] == 3 for c in curve)
    base60 = [c for c in curve if c['mode'] == 'reauction_baseline' and c['loss_pct'] == 60][0]
    assert base60['reacquired_n'] == 2 and base60['reacquired_max'] == 60.0
    assert base60['freed_all_n'] == 2 and base60['freed_all_max'] == 60.0
    c = rs.checks(curve, rows, modes, levels, 3, 60.0)
    assert c['complete'] and c['gate1_zero_collisions']
    assert c['parakram_at_most_once_every_trial']
    assert c['reauction_baseline_at_most_once_every_trial']
    assert c['parakram_freed_all_slope_ci_contains_0'] and c['gate2_parakram_flat']
    assert c['parakram_worst_freed_all_within_L_1_plus_rho_plus_tick']
    assert c['parakram_reacquired_flat_too']
    assert c['baseline_diverges_slope_ci_above_0'] and c['baseline_exceeds_10s_by_40pct']
    rows[0]['n_contacts'] = '1'                        # one contact anywhere fails Gate 1
    rows[1]['lease_expire_all_s'] = '2.2'              # beyond L(1+rho) + one tick
    c = rs.checks(rs.aggregate(rows, modes, levels, 60.0), rows, modes, levels, 3, 60.0)
    assert not c['gate1_zero_collisions'] and not c['gate2_parakram_flat']


def test_reacquisition_slope_is_reported_apart_from_the_gate_metric():
    levels, seeds = [0, 20, 40, 60], [1, 2, 3]
    # space freed flat at ~L, re-claiming a little slower under loss
    rows = [row('parakram', lv, s, 3.0 + 0.02 * lv + 0.01 * s, lease_all=1.95 + 0.01 * s)
            for lv in levels for s in seeds]
    curve = rs.aggregate(rows, ['parakram'], levels, 60.0)
    c = rs.checks(curve, rows, ['parakram'], levels, 3, 60.0)
    assert c['gate2_parakram_flat'] and not c['parakram_reacquired_flat_too']
    assert c['parakram_reacquired_slope_s_per_pct'] == pytest.approx(0.02, abs=1e-6)
