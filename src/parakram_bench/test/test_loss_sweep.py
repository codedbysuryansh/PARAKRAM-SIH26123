"""Unit tests of the Benchmark #2 sweep bookkeeping and the netem analysis (CLAUDE_CODE/05)."""

import csv
import os

from parakram_bench import netem_check
from parakram_bench import run_loss_sweep as sweep
import pytest


def _comms(path, rows):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'receiver', 'sender', 'topic', 'received', 'dropped', 'passed',
                    'first_seq', 'last_seq', 'drop_bursts', 'loss', 'model', 'burst_corr',
                    'seed'])
        for r in rows:
            w.writerow(r + [0.3, 'bernoulli', 0.8, 1])


def test_parse_levels():
    assert sweep.parse_levels('0,10,60') == [0, 10, 60]
    for bad in ('', '0,100', '-10'):
        with pytest.raises(ValueError):
            sweep.parse_levels(bad)


def test_comms_summary_uses_the_last_row_per_stream(tmp_path):
    _comms(tmp_path / 'comms_robot1.csv', [
        [5.0, 'robot1', 'robot2', 'intent', 40, 12, 28, 1, 40, 9],
        [10.0, 'robot1', 'robot2', 'intent', 100, 30, 70, 1, 100, 20],
        [10.0, 'robot1', 'robot2', 'state', 100, 31, 69, 1, 100, 22]])
    _comms(tmp_path / 'comms_robot2.csv', [
        [10.0, 'robot2', 'robot1', 'intent', 90, 27, 63, 11, 110, 18]])  # 10 lost in transport
    s = sweep.comms_summary(str(tmp_path), 'intent')
    assert s['streams'] == 2 and s['published'] == 200 and s['received'] == 190
    assert s['dropped'] == 57 and s['passed'] == 133
    assert s['drop_ratio'] == pytest.approx(57 / 190)
    assert s['delivered_ratio'] == pytest.approx(0.95)
    assert s['processed_ratio'] == pytest.approx(133 / 200)
    assert s['mean_drop_burst'] == pytest.approx(57 / 38)
    assert sweep.comms_summary(str(tmp_path), 'state')['dropped'] == 31
    empty = sweep.comms_summary(str(tmp_path / 'nothing'), 'intent')
    assert empty['streams'] == 0 and empty['drop_ratio'] is None


def test_monitor_metrics_maps_the_02_monitor_output():
    summary = {'robot_robot_contacts': [[['robot1', 'robot2'], 10.0, 10.4, 0.0]],
               'min_robot_robot_distance_m': 0.0, 'min_robot_static_clearance_m': 0.05,
               'legs_completed': {'robot1': 5, 'robot2': 4}, 'legs_assigned': {'robot1': 5,
                                                                               'robot2': 5},
               'checks': {'every_assigned_leg_completed': False}, 'makespan_s': None,
               'throughput_legs_per_min': 4.2, 'run_length_s': 330.0,
               'max_blocked_s': {'robot1': 3.0, 'robot2': 12.5},
               'breaker_events': {'robot1': {'a': 1, 'b': 0}, 'robot2': {'a': 2, 'b': 1}},
               'tick_compute_ms': {'all': {'p95': 3.1}}}
    m = sweep.monitor_metrics(summary)
    assert m['contacts'] == 1 and m['legs_completed'] == 9 and m['legs_assigned'] == 10
    assert m['all_legs_completed'] == 0 and m['max_blocked_s'] == 12.5
    assert m['breaker_events'] == 4 and m['tick_p95_ms'] == 3.1
    # no leg at all: the monitor has no throughput, the sweep counts zero
    none = sweep.monitor_metrics({'legs_completed': {'robot1': 0}})
    assert none['throughput_legs_per_min'] == 0.0 and none['contacts'] == 0


def test_safety_summary_counts_the_run_window_only(tmp_path):
    with open(tmp_path / 'safety_robot1.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'robot_id', 'min_obstacle_dist', 'filter_active', 'intervention',
                    'cmd_in_v', 'cmd_in_w', 'cmd_out_v', 'cmd_out_w'])
        for i in range(100):                       # 10 s at 10 Hz
            t = i * 0.1
            w.writerow([f'{t:.3f}', 'robot1', 0.5, int(40 <= i < 50), int(40 <= i < 60),
                        0, 0, 0, 0])
    s = sweep.safety_summary(str(tmp_path), 2.0, 7.95)           # ticks 20..79
    assert s['safety_ticks'] == 60
    assert s['safety_intervention_frac'] == pytest.approx(20 / 60)
    assert s['safety_filter_active_frac'] == pytest.approx(10 / 60)
    assert sweep.safety_summary(str(tmp_path), None, 5.0)['safety_intervention_frac'] is None
    # enrich() fills a row written before the metric existed from the run's logs
    run = tmp_path / 'run1'
    run.mkdir()
    (tmp_path / 'safety_robot1.csv').rename(run / 'safety_robot1.csv')
    with open(run / 'coord_acceptance.json', 'w') as f:
        f.write('{"coord_start_sim_time": 2.0, "end_sim_time": 7.95}')
    row = sweep.enrich({'run_id': 'run1', 'safety_intervention_frac': ''}, str(tmp_path))
    assert row['safety_intervention_frac'] == pytest.approx(20 / 60)


def _row(loss, seed, contacts=0, thr=4.5, status='ok', drop=None):
    p = loss / 100.0
    return {'loss_pct': str(loss), 'seed': str(seed), 'model': 'bernoulli',
            'run_id': f'r{loss}{seed}', 'status': status, 'contacts': str(contacts),
            'min_rr_distance_m': '0.12', 'throughput_legs_per_min': str(thr),
            'makespan_s': '330', 'legs_completed': '25', 'all_legs_completed': '1',
            'max_blocked_s': '5', 'breaker_events': '0',
            'intent_drop_ratio': str(p if drop is None else drop),
            'intent_delivered_ratio': '1.0', 'intent_processed_ratio': str(1 - p),
            'state_drop_ratio': str(p), 'error': ''}


def test_aggregate_and_checks():
    levels = sweep.parse_levels('0,10,20,30,40,50,60')
    rows = [_row(lv, s, thr=4.8 - lv / 40.0 + 0.1 * s) for lv in levels for s in (1, 2, 3)]
    rows[-1]['contacts'] = '2'                                  # a contact at 60 %
    curve = sweep.aggregate(rows, levels, 'bernoulli')
    assert [c['n_runs'] for c in curve] == [3] * 7
    assert curve[0]['throughput_mean'] == pytest.approx(5.0)
    assert curve[0]['throughput_std'] == pytest.approx(0.1)
    assert curve[3]['throughput_retained'] == pytest.approx((5.0 - 0.75) / 5.0)
    assert curve[6]['collisions_total'] == 2 and curve[6]['runs_with_collision'] == 1
    c = sweep.checks(curve, 3)
    assert c['all_planned_runs_present'] and c['zero_collisions_up_to_50pct']
    assert c['intent_rate_follows_loss'] and c['throughput_positive_every_level']
    assert c['largest_step_drop_of_baseline'] == pytest.approx(0.05)
    # a contact at <= 50 % fails the check; so does a missing run
    rows[12]['contacts'] = '1'                                  # loss 40 %, seed 1
    assert not sweep.checks(sweep.aggregate(rows, levels, 'bernoulli'), 3)[
        'zero_collisions_up_to_50pct']
    rows[12]['contacts'] = '0'
    rows[3]['status'] = 'bringup_failed'                        # loss 10 %, seed 1
    c = sweep.checks(sweep.aggregate(rows, levels, 'bernoulli'), 3)
    assert not c['all_planned_runs_present'] and not c['zero_collisions_up_to_50pct']
    # a drop that does not follow the setting fails the receive-rate check
    rows[3]['status'] = 'ok'
    for r in rows:
        if r['loss_pct'] == '60':
            r['intent_drop_ratio'] = '0.3'
    c = sweep.checks(sweep.aggregate(rows, levels, 'bernoulli'), 3)
    assert not c['intent_rate_follows_loss'] and not c['intent_rate_check_by_level'][60]


def test_append_keeps_an_older_header(tmp_path):
    path = tmp_path / 'runs.csv'
    old = [c for c in sweep.RUN_COLUMNS if not c.startswith('safety_')]
    sweep.write_csv(str(path), old, [_row(0, 1)])               # a sweep from an older version
    sweep.append_run(str(path), dict(_row(10, 1), safety_intervention_frac=0.5))
    rows = sweep.read_runs(str(path))
    assert [r['loss_pct'] for r in rows] == ['0', '10']
    assert rows[1]['error'] == '' and 'safety_intervention_frac' not in rows[1]
    fresh = tmp_path / 'fresh.csv'
    sweep.append_run(str(fresh), dict(_row(0, 1), safety_intervention_frac=0.5))
    assert sweep.read_runs(str(fresh))[0]['safety_intervention_frac'] == '0.5'


def test_outputs_round_trip(tmp_path):
    levels = [0, 30, 60]
    runs = tmp_path / 'runs.csv'
    rows = [_row(lv, s) for lv in levels for s in (1, 2)]
    rows.append(_row(30, 2, status='bringup_failed'))           # a failed later attempt
    sweep.write_csv(str(runs), sweep.RUN_COLUMNS, rows)
    ok = sweep.latest_ok(sweep.read_runs(str(runs)))
    assert len(ok) == 6 and all(r['status'] == 'ok' for r in ok)
    # 50 % is planned but has no run yet (a sweep in progress): reported, not drawn
    manifest = {'sweep_id': 'test', 'levels': [0, 30, 50, 60], 'seeds': [1, 2],
                'model': 'bernoulli',
                'n_robots': 3, 'scenario': 'intersection', 'git_sha': '0123456789abc',
                'git_dirty': True, 'assign_duration': 300.0}
    res = sweep.write_outputs(str(tmp_path), manifest, str(tmp_path / 'results'))
    assert not res['checks']['zero_collisions_up_to_50pct']       # 50 % has no evidence
    assert not res['checks']['all_planned_runs_present']
    assert len(res['attempts_not_ok']) == 1
    for name in ('benchmark2_loss_curve.csv', 'benchmark2_loss_curve.png',
                 'benchmark2_loss_curve.json', 'benchmark2_loss_runs.csv'):
        assert os.path.getsize(tmp_path / 'results' / name) > 0
    with open(tmp_path / 'results' / 'benchmark2_loss_curve.csv') as f:
        curve = list(csv.DictReader(f))
    assert [c['loss_pct'] for c in curve] == ['0', '30', '50', '60']
    assert [c['n_runs'] for c in curve] == ['2', '2', '0', '2']


def test_netem_analysis_separates_visible_and_hidden_loss():
    pub = [(i, i * 0.1) for i in range(1, 401)]                  # 10 Hz for 40 s
    sub = []
    for seq, t in pub:
        if seq % 10 >= 3:                                        # BEST_EFFORT: 30 % lost
            sub.append(('best_effort', seq, t, t + 0.002))
        late = 0.5 if seq % 10 < 3 else 0.003                    # RELIABLE: all, 30 % late
        sub.append(('reliable', seq, t, t + late))
    sub.append(('reliable', 5, 0.5, 0.6))                        # a duplicate
    res = netem_check.analyse(pub, sub, 1.0, 40.0, 30, 0)
    be, rel = res['best_effort'], res['reliable']
    assert be['n_sent'] == 391 and be['delivery_ratio'] == pytest.approx(0.7, abs=0.01)
    assert be['within_tolerance'] and be['latency_p50_ms'] == pytest.approx(2.0)
    assert rel['delivery_ratio'] == 1.0 and rel['within_tolerance']
    assert rel['on_time_ratio_100ms'] == pytest.approx(0.7, abs=0.01)
    assert rel['duplicates'] == 0                               # seq 5 is outside the window
    # correlated netem keeps no nominal mean: no verdict on BEST_EFFORT
    assert netem_check.analyse(pub, sub, 1.0, 40.0, 30, 25)['best_effort'][
        'within_tolerance'] is None
    assert netem_check.parse_cases('0:0, 30:25') == [(0.0, 0.0), (30.0, 25.0)]
    with pytest.raises(ValueError):
        netem_check.parse_cases('100:0')
