# parakram_bench — benchmark harness (CLAUDE_CODE/05 part; the rest is CLAUDE_CODE/07)

What exists so far is the Benchmark #2 loss sweep and the netem check of CLAUDE_CODE/05.

## Benchmark #2: throughput and collisions vs packet loss (`run_loss_sweep.py`)

```bash
source install/setup.bash
python3 -m parakram_bench.run_loss_sweep --scenario intersection --seeds 5
```

For every loss level (0, 10, ..., 60 %) and seed (1..5) it runs the same fixed scenario once,
headless, one run at a time (it owns the machine: it stops any leftover simulation first):

1. `fleet_sim.launch.py n_robots:=3 scenario:=intersection seed:=<s> loss:=<p> run_id:=<uuid>`
   (the full stack, reactive safety layer on);
2. the CLAUDE_CODE/02 monitor `coord_acceptance` (Gazebo ground truth, observer only; its
   roster-kill test is switched off and its PASS verdict is not used);
3. `coord.launch.py`, which reads loss and seed from the run manifest and applies the seeded
   app-level drop (`parakram_comms/loss.py`) to peer state/intent before coordination processes a
   message. Task/award traffic (RELIABLE) and the safety layer's inputs are never touched.

Per run (window `--assign-duration`, default 300 s sim, the 02 acceptance window): ground-truth
robot-robot contacts (footprint distance <= 1 cm), legs completed, makespan and throughput, with
the 02 monitor's definitions; and the loss filters' counters (`comms_<ns>.csv`): per peer stream
the messages the sender published (its seq span), delivered by the transport, dropped by the
injector and processed by coordination.

Outputs:

| file | content |
|---|---|
| `results/benchmark2_loss_curve.csv` | per level: mean and sample std of collisions, throughput, makespan, legs, intent receive-rate, realized drop |
| `results/benchmark2_loss_curve.png` | throughput vs loss, collisions vs loss, intent receive-rate vs loss |
| `results/benchmark2_loss_runs.csv` | every run with its `run_id` (`bench/logs/<run_id>/`) |
| `results/benchmark2_loss_curve.json` | provenance (sweep manifest, git SHA) and the checks |
| `bench/logs/loss_sweep_<id>/` | sweep manifest, `runs.csv` (every attempt), each process' console log |

`--sweep-dir bench/logs/loss_sweep_<id>` resumes a sweep (runs already done are skipped);
`--aggregate-only` rebuilds the outputs; `--model gilbert_elliott [--burst-corr 0.8]` sweeps the
bursty model instead (outputs get a `_gilbert_elliott` suffix); `--no-results` keeps a
verification sweep out of `results/`. An infrastructure failure (bring-up, a launch exiting, no
monitor result) is retried and recorded; a finished run is a result whatever it shows.

The checks in the JSON: `zero_collisions_up_to_50pct` (every planned run at <= 50 % present and
contact-free), `intent_rate_follows_loss` (realized drop within 0.03 of the setting and the
processed fraction within 0.03 of delivered x (1 - loss) at every level) and descriptive decay
figures (`throughput_retained_by_level`, `largest_step_drop_of_baseline`). "Graceful (non-cliff)"
has no number in the work order, so none is invented: it is read from the curve.

## netem, the secondary mechanism (`scripts/netem.sh`, `scripts/netem_verify.sh`)

`netem.sh` is the work order's script (`LOSS=30 CORR=25 IFACE=wlan0 netem.sh add|del|show`,
events logged to `bench/logs/netem_events.csv`); on hardware it goes on every robot's `wlan0`.

In the single-host simulation netem is not a faithful model: on `lo` it hits all local traffic
(Gazebo transport, Nav2, TF), while Fast DDS's default shared-memory transport bypasses `lo` so
the coordination traffic it should hit would escape it. The sim sweep therefore uses the
app-level drop, and `netem_verify.sh` checks netem on its own probe (`netem_check.py`): a
publisher and a subscriber in two processes on an isolated ROS domain, one BEST_EFFORT stream
(`INTENT_QOS`) and one RELIABLE stream (`TASK_QOS`) at 10 Hz, on every installed fabric: Fast
DDS forced onto UDPv4 (cases 0/10/30/60 % and 30 % with netem's 25 % correlation), CycloneDDS
with the static-peer config and rmw_zenoh through a router (0/30/60 % each). netem is applied
after discovery, measured for 40 s, removed, then RELIABLE gets 10 s to retransmit.

```bash
sudo bash src/parakram_bench/scripts/netem_verify.sh     # no simulation running; ~11 min
```

It refuses to run while a simulation or a sweep runs, removes the qdisc after every case and on
any exit, runs the ROS processes as the invoking user, and writes `bench/logs/netem_check_<id>/`
and `results/benchmark2_netem_check.csv`. Expected on the UDP fabrics: BEST_EFFORT delivery ~
1 - loss (the loss is visible), RELIABLE ~ 100 % with longer latency (retransmission hides it).
On rmw_zenoh every stream rides a TCP link, so the check shows how much of the loss reaches the
application at all (BEST_EFFORT is not expected to track 1 - loss there).

Measured (`bench/logs/netem_check_0fe10780`, 2026-09-29; 40 s at 10 Hz per case, ~400
messages; loopback probe, not a fleet run). At 0 % every stream on every fabric delivered 1.000
with p95 <= 3 ms.

| fabric (link) | netem loss | BEST_EFFORT delivered (1 - loss) | RELIABLE delivered | RELIABLE on time (<= 100 ms) | RELIABLE p95 latency |
|---|---|---|---|---|---|
| Fast DDS (UDP) | 10 % | 0.920 (0.90) | 1.000 | 0.142 | 6.9 s |
| Fast DDS (UDP) | 30 % | 0.720 (0.70) | 1.000 | 0.038 | 15.0 s |
| Fast DDS (UDP) | 60 % | 0.383 (0.40) | 1.000 | 0.000 | 25.0 s |
| Fast DDS (UDP) | 30 %, correlation 25 % | 0.790 (see below) | 1.000 | 0.075 | 15.7 s |
| CycloneDDS (UDP) | 30 % | 0.710 (0.70) | 1.000 | 0.608 | 0.40 s |
| CycloneDDS (UDP) | 60 % | 0.403 (0.40) | 1.000 | 0.078 | 4.3 s |
| rmw_zenoh (TCP) | 30 % | 0.172 (0.70) | 0.800 | 0.048 | 16.9 s |
| rmw_zenoh (TCP) | 60 % | 0.000 (0.40) | 0.000 | 0.000 | n/a |

- Over UDP, BEST_EFFORT tracks 1 - loss within the binomial tolerance at every level: the
  network-level mechanism confirms what the app-level drop models.
- RELIABLE (`TASK_QOS`) hides the loss completely and pays in latency: seconds on Fast DDS even
  at 10 % loss. That is the retransmission trap of the work order, measured, and it is the
  latency the task / award traffic of CLAUDE_CODE/04 sees under network loss.
- netem's correlated mode does not keep the nominal mean (30 % with 25 % correlation removed
  21 %), which is why the proportionality cases use correlation 0.
- rmw_zenoh (TCP) collapses instead of degrading: stalls of 10 s or more with stale bursts at
  30 %, no delivery at all at 60 %, and no recovery within 10 s of the loss being removed. The
  publisher kept its 10 Hz (400 / 400 messages sent) and the data took one direct TCP link, so
  this is TCP under loss, not the probe. See the parakram_comms README for what it means.

## Claim status

Everything here is SIMULATION acceptance verification (CLAIMS_LEDGER). A number moves to
MEASURED only with the CLAUDE_CODE/07 protocol (CIs, held-out levels, hardware ground truth).
