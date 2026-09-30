# results/ — CLAUDE_CODE/05 and 06 outputs (SIMULATION acceptance verification)

Nothing here is a MEASURED claim in the CLAIMS_LEDGER sense: all of it is simulation or a
single-host loopback probe, produced as CLAUDE_CODE/05 and 06 acceptance evidence. Every caption / slide
using it must say so.

## Benchmark #2 loss curve (app-level drop)

`benchmark2_loss_curve.csv` / `.png` / `.json`, `benchmark2_loss_runs.csv`

- Command: `python3 -m parakram_bench.run_loss_sweep --scenario intersection --seeds 5`
  (sweep `fba52626`, 2026-09-29; logs `bench/logs/loss_sweep_fba52626/`, every run under
  `bench/logs/<run_id>/`; the JSON carries the sweep manifest and the checks).
- Setup: Gazebo Harmonic, 3 TurtleBot3 Burger, the `intersection` crossing scenario, full stack
  (CLAUDE_CODE/02 coordination + 03 reactive safety layer), Fast DDS; 300 s sim crossing window
  per run; loss 0-60 % in steps of 10, seeds 1-5; git base `6161d2e` plus the uncommitted
  CLAUDE_CODE/05 tree.
- Loss: seeded app-level Bernoulli drop of peer state/intent (BEST_EFFORT) before
  coordination processes them; task/award traffic and the safety layer's inputs untouched.
- Ground truth: Gazebo poses via the CLAUDE_CODE/02 monitor; a collision = footprint distance
  <= 1 cm.
- Result: 0 contacts in all 35 runs; every assigned leg completed; throughput 4.38-4.57 legs/min
  per level (>= 95.8 % of the 0 % level); processed intents = 1 - loss at every level while the
  transport delivered 100 %.
- Scope: one scenario, three robots, independent (Bernoulli) loss. The throughput stays flat
  because peers repeat their intents at 10 Hz and a reservation lapses only when a peer's whole
  2 s lease (20 consecutive intents) is lost: at 60 % independent loss that happens about 0.3
  times per run (6 peer streams x ~3000 intents x 0.4 x 0.6^20), while 1 s gaps (the staleness
  timeout, 10 intents) happen about 40 times per run. Bursty loss is the harder case
  (Gilbert-Elliott at 30 %: mean drop burst 6.9 messages; one smoke run only,
  `bench/logs/loss_sweep_0a096838`), as are the CLAUDE_CODE/06-07 baselines and hardware.

## netem check (network-level loss, loopback probe)

`benchmark2_netem_check.csv`

- Command: `sudo bash src/parakram_bench/scripts/netem_verify.sh` (check `0fe10780`,
  2026-09-29; logs `bench/logs/netem_check_0fe10780/`).
- A publisher and a subscriber process (10 Hz, one `INTENT_QOS` BEST_EFFORT stream and one
  `TASK_QOS` RELIABLE stream) on Fast DDS (UDPv4), CycloneDDS (UDP, static peers) and rmw_zenoh
  0.2.10 (TCP, via a router; the data used a direct peer TCP link); netem on `lo` with
  independent loss on every packet, 40 s per case plus a 10 s drain.
- Result: over UDP, BEST_EFFORT delivery tracks 1 - loss and RELIABLE delivers 100 % with
  seconds of latency; over rmw_zenoh's TCP, delivery collapses (0.17 at 30 %, 0 at 60 %). The
  parakram_bench README has the table and the reading.
- Scope: loopback and synthetic independent loss, not WiFi (802.11 retries at the MAC layer and
  loses in bursts); it characterises the fabrics' transports, not the fleet.

## CLAUDE_CODE/06 money shot: liveness restoration after a silent kill vs loss

`moneyshot_flat_vs_exploding.png` / `.csv` / `.json`, `moneyshot_runs.csv`

- Command: `python3 -m parakram_bench.run_recovery_sweep --modes parakram,reauction_baseline
  --loss 0:60:10 --seeds 5 --plot` (sweep `24912b37`, 2026-09-29/30; logs
  `bench/logs/recovery_sweep_24912b37/`, every trial under `bench/logs/<run_id>/` with its
  `recovery_trial.json`; the JSON carries the sweep manifest, its amendments and the checks).
- Setup: Gazebo Harmonic, 3 TurtleBot3 Burger, the `junction_stream` scenario (every station
  pair can route through junction (5,6)), full stack (02 coordination with the 06 lease protocol,
  03 reactive safety layer, 04 auction, 06 fault layer), Fast DDS; 70 trials (2 modes x 7 loss
  levels x 5 seeds), each: warm-up 20 s, the S3 trigger (a task-holding robot reserving the
  junction while another robot's route needs it), SIGKILL of that robot's coordination, task
  and fault processes (Nav2 goal cancelled: the body stays), 90 s observed.
- Loss: `loss_scope:=fleet`, one seeded Bernoulli process per directed robot link applied after
  delivery to EVERY inter-robot topic (renewals, heartbeats, announcements, bids, awards,
  digests), identical in both modes; no retransmission hides it.
- Ground truth: Gazebo poses; a collision = footprint distance <= 1 cm, every pair including
  the dead body.
- Metrics (per trial, `recovery_trial.json` / `moneyshot_runs.csv`; t from the recorded kill):
  - liveness restoration = the victim's reserved space free at EVERY survivor
    (`lease_expire_all_s`; PARAKRAM: its lease expired there, no message; baseline: a release
    message from another robot's re-auction arrived) - the Gate 2 metric;
  - re-acquisition = a survivor HOLDS movement authority over a cell the victim had reserved
    (`restoration_s` = liveness restoration + claim settle + acknowledgement round trip, then
    whenever traffic lets a survivor need that cell) - reported, not gated;
  - entered = a survivor's footprint enters the junction (`entered_s`, ground truth, includes
    travel); throughput = tasks completed in the 90 s after the kill, per minute; contacts;
    duplicate completions; two robots' renewals carrying one task at overlapping times, and
    concurrent execution = both moving > 5 cm during that overlap;
  - per mode and level: mean, std, 95 % CI (Student t) over the 5 trials; a trial where the
    event did not happen within the 90 s window counts as 90 s (censored); Gate 2's flatness =
    least-squares slope of the 35 per-trial values on loss, 95 % CI (Student t, n - 2 dof).
- Result (checks in the JSON):
  - Gate 1: 0 contacts in all 70 trials, both modes, 0-60 %.
  - PARAKRAM liveness restoration (the victim's reserved space free at EVERY survivor: its lease
    expired there, no message): per-level means 1.89-1.99 s (headline: ~2 s at every loss
    level), regression slope -0.0008 s/% with 95 % CI [-0.0017, +0.0001] (contains 0), 0 of 35
    trials censored.
  - Worst case vs the bound. Strict bound (protocol): a survivor frees a silent robot's space at
    (its last renewal's stamp) + L(1+rho) = + 2.000 s (L = 2 s, rho = 0: one simulation clock);
    no message is involved. Implementation-resolution result (as measured): the sweep measures
    from the kill time the monitor records, its latest ground-truth clock sample when it sends
    SIGKILL, and survivors evaluate expiry on their 0.1 s coordination tick. Worst measured:
    2.010 s (run `6ace9bf0`, 10 %; also 2.001 s `fdffc6eb` and 2.002 s `f886642b`; the other
    32 trials 1.730-1.999 s): 10 ms over the nominal 2.000 s, inside the 0.1 s resolution. Raw
    logs place the excess in the kill timestamp, not in the lease: every node ticks on the same
    0.1 s grid (phase x.x02 s), both survivors expired the lease at the 57.502 s tick and not at
    57.402 s, so the victim's last renewal was stamped 55.502 s, 10 ms AFTER the recorded kill
    (55.492 s); from its last renewal the lease expired at exactly 2.000 s (same for the 2.001 /
    2.002 s trials: renewals at kill + 1 / 2 ms). The sweep's check therefore reads "worst <=
    L(1+rho) + one 0.1 s tick" and says so; it does not redefine L(1+rho).
  - reauction_baseline (release on another robot's re-auction message): 1.14 s at 0 %, 8.7 s at
    30 %, 29.4 s at 40 % (one of five trials not released within 90 s, counted as 90 s), 32.1 s
    at 60 %; slope +0.58 s/% (CI [0.17, 0.99]).
  - Throughput (tasks completed in the 90 s after the kill): PARAKRAM 1.07-1.60 per minute at
    every level (positive); baseline 0.80-1.47.
  - Re-acquisition (a survivor HOLDS authority through the freed space; freed + claim settle +
    acknowledgement): PARAKRAM 2.3-3.8 s in 29 of 35 trials, but traffic-dependent (the waiting
    robot may yield or detour: up to 53.7 s); slope CI contains 0; reported, not gated.
  - At most once: PARAKRAM 0 duplicate completions and 0 trials with two robots working on one
    task. Two trials (`ccfc45d1` 40 %, `ebfad9ed` 60 %) had same-round double awards (two
    announcers that missed each other's announcement), 3 in all, flagged by the monitor as
    0.1-0.6 s overlaps of two robots' renewals carrying the task. Raw logs: each loser dropped
    the award 0.51-0.63 s after winning it ("superseded by the round-1 award to robot1", "robot2
    holds round 1"), inside the 1 s `award_settle`, so its award gate never let it act: its
    coordination goal stayed empty / its own cell, it logged no renew, release or completion
    for the task, and its ground-truth displacement during the overlap was 0.000 m. Each task
    was completed once, by the lower announcer's winner: non-executing transient state, no
    duplicate execution.
  - The baseline had 0 duplicate completions but 5 trials (50-60 %) where two robots worked on
    the same task: 4 same-round double awards never settled (it keeps 04's Award-message-only
    tie-break) and 1 re-auction of a live robot's task on a false DEAD (0.6 s).
- Amendments (in the manifest): the baseline's ReAuction was made symmetric after trial 11 (its
  first 6 attempts re-run); the task layer's same-round tie-break was fixed after the first pass
  (all PARAKRAM trials re-run). Sweep-level records, versioned here: `w06_sweep_records/`
  (`sweep_manifest.json`, `runs.csv` with every attempt, `runs_*_superseded.csv`,
  `runs_before_*.csv`); per-trial logs stay under `bench/logs/<run_id>/` (untracked).
- Scope: one scenario, three robots, independent (Bernoulli) loss, 5 seeds per point (06's test
  note asks 10); bursty (Gilbert-Elliott) loss not swept. SIMULATION: TARGET/sim in the
  CLAIMS_LEDGER (T2), not a hardware claim.

## CLAUDE_CODE/06 S4 partition and 5 s blackout

`recovery_s4.*`, `recovery_blackout.*` (per-run rows in `recovery_*_runs.csv`)

- Commands: `python3 -m parakram_bench.run_recovery_sweep --modes parakram --kind s4 --loss 0
  --seeds 3` (sweep `b00edf0d`) and `... --kind blackout --loss 0 --seeds 3` (sweep
  `b6c50a5e`), 2026-09-30, final code; same scenario, stack and ground truth as above.
- S4 (one busy robot's links cut for 30 s, then healed), 3/3 trials: 0 contacts; the isolated
  robot froze 1.5-1.6 s after the cut (own lease not acknowledged, no quorum) and moved 0.0 m
  while isolated; both survivors expired its lease at 1.92-1.98 s and kept moving (0.4-3.7 m
  each during the partition); it released its task at 7.9-8.0 s, before the majority
  re-auctioned it at 10.3-10.7 s; the heal made no robot drop a task, no task was completed
  twice and no two robots held one task at once. The watchdog declared the isolated
  robot DEAD after the 10 s partition grace (it was silent everywhere: a full isolation cannot
  be told from death), which only re-announced tasks whose award lease had run out.
  In seed 3 a survivor was pinned against the wall at the edge pickup station (0,6) for ~60 s
  (reactive layer: obstacle at 3-10 cm) and the third robot yielded the junction to it for 27 s
  until the 04 stall timeout released its task: a live-but-stuck robot, not the recovery path.
- Blackout (every link down 5 s, every robot alive), 3/3 trials: 0 contacts; every robot froze
  at 1.55-1.64 s, the peers' leases expired at 1.94-1.97 s, and all resumed 0.25 s after the
  heal; no DEAD / PARTITIONED / re-auction / release event (no false positive), every task kept
  and completed once.
- SIMULATION evidence (TARGET/sim), one host, the partition injected at the application layer
  after delivery (`/fleet/fault_injection`), not a radio.
