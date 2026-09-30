# W06 handoff — fault detection and communication-negative recovery

Work order: `SIH_2026/CLAUDE_CODE/06_fault_detection_recovery.md`. Status: implemented and
accepted in SIMULATION, committed as `feat: implement work order 06 fault detection and
recovery` (base: W05 `ac17ebc`). All numbers below are TARGET/sim in the CLAIMS_LEDGER (not
modified). Next: W07 (not started).

## Architecture implemented

Recovery is communication-NEGATIVE: a silent robot's lease expires, its space is reclaimed with
no message, the fleet moves on; the watchdog + re-auction only reallocate tasks.

- **Lease protocol** (`parakram_coord`): every intent is the lease renewal (= heartbeat) and
  carries acknowledgements of the peers' renewals, the held authority (body envelope) and the
  task award it renews. `leases.py` (OwnLease): my lease is valid while every live peer acked a
  renewal within L - 0.3 s and I hear a strict majority; otherwise I stop (Nav2 cancelled), drop
  my claims and re-acquire. A claim becomes authority only after `claim_settle` and an ack from
  every live peer. `reservation_table.py`: an expired peer's reserved space is freed, its body
  envelope stays a ghost obstacle until it is heard again or my lidar sees the cell empty
  (`ghost_view.py`). `pibt_rule.py`: `obstacles`, `frozen`, `claim_acked` (defaults = W02).
  L = 2 s, 10 Hz, rho = 0 in sim.
- **Loss** (`parakram_comms/link_loss.py`, `loss_scope:=fleet`): one seeded loss process per
  directed robot link (50 ms slots; Bernoulli or Gilbert-Elliott), applied after delivery to
  EVERY inter-robot topic of every node; partitions of one robot (or all) through
  `/fleet/fault_injection`. W05's `loss_scope:=coordination` path is unchanged (default).
- **Task layer** (`parakram_tasks`, lease mode + `award_gate`): quorum award leases — the holder
  works only while a strict majority acked a renewal of its award within 10 - 2 s; award time
  pauses and a robot takes no part in auctions while its own lease is invalid; a third robot's
  ack of a NEW renewal extends the holder's awards (partitioned is not dead); two awards of one
  round are settled by the lower announcer, carried on renewals (`Intent.task_announcer`,
  `TaskProgress.announcer_id`) and digests; a new award waits `award_settle` = 1 s; 1 Hz
  `TaskDigest` anti-entropy. ReAuction in lease mode only announces tasks whose award lease ran
  out (the holder, maybe alive behind a partition, has released them by then).
- **Baseline** `reauction_baseline` = `recovery_mode:=release`: no leases, no gates; a silent
  robot's space is released at a survivor when ANOTHER robot's newer-round message (announce /
  award / digest) arrives; watchdog DEAD at 1 s; every survivor re-announces (symmetric).
- **Fault layer** (`parakram_fault`): `watchdog.py`/`watchdog_node.py` (ALIVE / FLAKY / SUSPECT /
  PARTITIONED via third-party acks / DEAD: 10 s grace in PARAKRAM, 1 s detect in the baseline),
  `heartbeat_node.py`, `recovery_coordinator.py` (ReAuction once per death),
  `recovery_logger.py`, `config/fault.yaml`, `launch/fault.launch.py`.
- **Messages**: Intent, CoordStatus (06 fields), TaskProgress (+`announcer_id`, additive to a W04
  message), new PeerStatus, RecoveryEvent (t_kill, t_lease_expire, t_space_reclaimed,
  t_task_reassigned, loss_level, dead_id, task_id), TaskDigest.
- **Bench** (`parakram_bench`): `recovery_monitor.py` (S3 trigger + SIGKILL, S4/blackout
  partitions, Gazebo ground truth, metrics), `run_recovery_sweep.py` (acceptance command,
  censored stats, regression CI, Gate checks, money-shot plot). Scenario `junction_stream`.

## Final results (simulation)

- Money shot `bench/logs/recovery_sweep_24912b37` (70/70: 2 modes x 0-60 % x 5 seeds; outputs in
  `src/parakram_bench/results/`): Gate 1 = 0 ground-truth contacts in all 70. PARAKRAM liveness
  restoration (space free at every survivor) 1.89-1.99 s per level, slope -0.0008 s/% with 95 %
  CI [-0.0017, +0.0001] (contains 0), throughput 1.07-1.60 tasks/min at every level. Worst
  measured 2.010 s: strict bound L(1+rho) = 2.000 s from the victim's last renewal holds exactly
  (raw logs: its last renewal was stamped 10 ms after the monitor's recorded kill); as measured
  from the recorded kill it is 10 ms over, inside the 0.1 s tick. Baseline 1.14 s at 0 % ->
  29.4 s at 40 %, 32.1 s at 60 %, slope CI [0.17, 0.99]. PARAKRAM: no task executed twice (3
  same-round double awards dropped by the loser inside `award_settle`, 0.000 m moved).
- S4 `recovery_sweep_b00edf0d` 3/3 and 5 s blackout `recovery_sweep_b6c50a5e` 3/3: 0 contacts,
  isolated robot frozen, leases expired at survivors <= 2 s, task released before re-auction,
  no false DEAD, no duplicate or concurrent execution.
- Tests: 214 (205 passed, 9 skipped, 0 failures). W02/W05 regression spot check with the lease
  gate on: `bench/logs/loss_sweep_accbb496` (0 contacts, 4.70 / 4.29 legs/min at 0 / 60 %).
- Details, metric definitions, amendments: `src/parakram_bench/results/README.md`;
  limitations: `src/parakram_fault/README.md`.

## Important decisions and fixes

- Gate 2 is measured on space FREED at every survivor (the quantity L bounds; ledger T2
  ~1.5-2.5 s). Re-acquisition (a survivor holding the space) is reported, not gated: it adds the
  claim settle + ack round trip and depends on traffic (2.3-3.8 s typical, up to 54 s).
- W02 behaviour change (intended by W06): coordination is lease-gated by default
  (`recovery_mode:=lease`); checked by the regression spot check above.
- Found and fixed during acceptance: S4 exposed an isolated holder keeping its task -> quorum
  award leases; same-round double award persisting when the Award message was lost (run
  a548032c) -> announcer on renewals/digests + award_settle, all PARAKRAM trials re-run;
  baseline stall (one-sided ReAuction) -> symmetric re-announcement, baseline trials re-run;
  watchdog DEAD from a renewal recorded before the sim clock -> ignored; monitor lock deadlock.
  Superseded rows kept (never deleted): `runs_*_superseded.csv`, `runs_before_*.csv` in
  `bench/logs/recovery_sweep_24912b37/` and versioned in
  `src/parakram_bench/results/w06_sweep_records/`; `sweep_manifest.json` lists both amendments.
- PARAKRAM's DEAD waits the 10 s partition grace: a fully isolated robot cannot be told from a
  dead one and releases its task only 8 s after its last majority ack.

## Known limitations / caveats

Simulation only (Gazebo, 3 TurtleBot3, one scenario, one host, app-level loss and partition);
Bernoulli loss only (Gilbert-Elliott not swept); 5 seeds per point (06's test note asks S3 x10);
a lone survivor of 3 freezes (majority quorum); non-transitive connectivity not relayed; absolute
lease stamps need clock sync on hardware; a live-but-stuck robot (seen once, pinned at the edge
station (0,6)) waits for W04's 60 s stall timeout; the kill stops coordination/auction/fault
nodes, Nav2 and the reactive layer keep running; ghost cells clear on "no return inside the
cell"; readiness-gate hardware items open (Zenoh replay depth, Zenoh TCP+UDP netem test).

## Operational notes

`bench/` (run logs) is untracked by convention. The workspace build is not symlink-install:
`colcon build` after every source change. Sweeps own the simulator (`bench/logs/SIM_BUSY`).

## Exact next step: W07

Start `CLAUDE_CODE/07_benchmark_harness.md` (only when the user asks): baselines BL0-BL4
(`stop_and_wait`, `reactive_only`, `reauction_baseline` [done here], `pibt_no_lease`, central),
`run_benchmark1.py`, ablations (BL3 no-lease deadlocks under partition), aggregation/plotting,
captions with claim status; its acceptance reuses this work's
`run_recovery_sweep --modes parakram,reauction_baseline --loss 0:60:10 --seeds 5 --plot`.
Decisions to raise then: deployment fabric (Zenoh TCP vs UDP multilink vs Cyclone), GE sweep,
10 seeds, hardware clock sync.
