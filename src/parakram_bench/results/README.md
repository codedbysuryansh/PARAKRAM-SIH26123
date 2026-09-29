# results/ — CLAUDE_CODE/05 outputs (SIMULATION acceptance verification)

Nothing here is a MEASURED claim in the CLAIMS_LEDGER sense: all of it is simulation or a
single-host loopback probe, produced as CLAUDE_CODE/05 acceptance evidence. Every caption / slide
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
