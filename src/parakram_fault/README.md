# parakram_fault — communication-negative recovery (CLAUDE_CODE/06)

Recovery in PARAKRAM is **communication-NEGATIVE**: a silent robot's spatial lease EXPIRES and
its space is reclaimed with no message; packet loss can only make that happen sooner. Task
re-auction is an opportunistic accelerator for the task layer, never a safety or liveness
dependency. This package holds the fault layer's nodes; the lease protocol itself lives in
coordination (`parakram_coord`, `leases.py`) because every cell entry is gated there.

## The lease protocol (parakram_coord, `recovery_mode:=lease`)

- **Lease renewal IS the heartbeat.** Every coordination intent (10 Hz) renews the robot's lease
  on all cells it reserves (`lease_expiry = now + L`, L = 2 s) and carries acknowledgements:
  for every peer, the latest renewal seq it processed (`Intent.ack_ids` / `ack_seqs`).
- **Every cell entry is lease-gated.** A claim becomes movement authority (a Nav2 goal) only
  after `claim_settle` AND once every live peer has acknowledged an intent that contained it.
- **Expired lease -> re-acquire before moving.** A robot's own lease is valid while every live
  peer has acknowledged one of its renewals within L (minus a 0.3 s stop margin) and it hears a
  strict majority of the fleet. Otherwise it stops (Nav2 cancelled), drops its claims and
  re-acquires them (claims re-made and re-acknowledged) before moving. An isolated robot
  freezes; a false positive (merely delayed robot) costs throughput, never safety.
- **Recovery without a message.** A peer whose lease expired stops blocking its reserved space.
  Its BODY does not vanish: its last renewal's body envelope (occupied cells + held authority)
  becomes a ghost obstacle until the peer is heard again or this robot's own lidar has seen the
  cell empty (`ghost_view.py`: beams across the cell pass beyond it, no return inside it).
- **Tasks ride the same lease, as quorum leases** (parakram_tasks): the holder's intents carry
  its award; it works on the task only while a strict majority of the fleet (itself included)
  has acknowledged a renewal of it within `award_lease_ttl - 2 s`, else it releases it
  (RELEASED). A robot counts award leases down, and announces, bids, awards or re-auctions,
  only while its OWN spatial lease is valid (it hears a majority and is acknowledged); another
  robot's acknowledgement of a NEW renewal of a holder renews that holder's awards (partitioned
  from me is not dead). A robot that could re-award a task hears a majority, which shares a
  robot with the holder's acknowledging majority, so the award cannot run out there before the
  holder has released it: at most one robot works on a task, across a partition and its heal
  too. Two awards of ONE round (two announcers that missed each other's announcement under
  loss) are settled by the lower announcer, which rides on the holder's renewal intents and on
  the digests (not only on the one-shot Award message), and a new award waits `award_settle`
  (1 s) before it is worked on, so a racing award surfaces first. A 1 Hz digest (completions,
  award rounds) lets a robot that missed messages converge without executing a task twice.

## Nodes (per robot, `ros2 launch parakram_fault fault.launch.py n_robots:=3`)

| node | role |
|---|---|
| `heartbeat` | relays the robot's own lease renewals as `/<ns>/heartbeat` (it never beats on its own: no renewal, no heartbeat) |
| `watchdog` | windowed watchdog over the peers' renewals (through the same lossy links): ALIVE, FLAKY, SUSPECT, PARTITIONED (another peer still acknowledges NEW renewals of it), DEAD (silent everywhere for `partition_grace`, 10 s); `/<ns>/peer_status` |
| `recovery_coordinator` | on DEAD: `/<ns>/reauction`, once per death (baseline: re-announce the peer's tasks; PARAKRAM: announce at once those whose award lease ran out); nothing for space |
| `recovery_logger` | `/fleet/recovery_event` -> `bench/logs/<run_id>/recovery_events.csv` |

Parameters: `config/fault.yaml`. Coordination also writes `recovery_<ns>.csv` (lease expiry,
release, ghost cleared, space reclaimed, own lease lost / regained, peer back).

## The comparison (both modes, identical conditions)

`fleet_sim.launch.py recovery_mode:=lease|release loss_scope:=fleet loss:=<p>`:

- `lease` = **PARAKRAM** (above; the watchdog's DEAD waits for the partition grace, and a task
  is reallocated only once its award lease has run out, i.e. after its holder, if alive, has
  released it; a robot heard by the fleet keeps its task).
- `release` = **reauction_baseline**: no spatial leases, no award leases, no own-lease gate. A
  silent robot's space is released at a survivor only when a message from ANOTHER robot shows
  the silent robot's task in a newer auction round (re-announcement, award or digest): the
  communication-positive "detect -> re-auction" recovery. Its watchdog declares DEAD at the
  1 s detection timeout (no partition grace) and triggers ReAuction; every survivor announces
  its own round of the dead robot's tasks (idempotent by task seq), so each can hear another's.

Both run the same scenario (`junction_stream`), task stream, watchdog and loss:
`loss_scope:=fleet` puts EVERY inter-robot topic (renewals, heartbeats, announcements, bids,
awards, digests) through one per-link loss process per robot (`parakram_comms.link_loss`),
after delivery, so no retransmission hides it. Partitions (`/fleet/fault_injection`) cut the
links of ONE robot (or all: blackout) without touching the simulation's own network.

## Benchmarks (`parakram_bench`)

```bash
python3 -m parakram_bench.run_recovery_sweep --modes parakram,reauction_baseline --loss 0:60:10 --seeds 5 --plot
python3 -m parakram_bench.run_recovery_sweep --modes parakram --kind s4 --loss 0 --seeds 3
python3 -m parakram_bench.run_recovery_sweep --modes parakram --kind blackout --loss 0 --seeds 3
```

S3: the victim is killed while it holds a task and a reservation on the central junction that
another robot needs next (its brain SIGKILLed, its Nav2 goal cancelled: the body stays).
Metrics: lease expiry at every survivor (the quantity L bounds), liveness restoration (a
survivor holds movement authority over a cell the victim had reserved), ground-truth entry into
the junction, detection, task reassignment, throughput after the kill, duplicate completions,
ground-truth contacts. Outputs: `results/moneyshot_flat_vs_exploding.{png,csv,json}`.

## Results (simulation, TARGET/sim)

`parakram_bench/results/README.md`: the money-shot sweep `24912b37` (70 trials: 0 contacts;
PARAKRAM liveness restoration 1.89-1.99 s per level with a slope 95 % CI containing 0; worst
measured 2.010 s, 10 ms over L(1+rho) = 2.000 s from the recorded kill, inside the 0.1 s tick,
while from the victim's last renewal the lease expired at exactly 2.000 s; the re-auction
baseline 1.1 s at 0 % rising to 29-32 s at 40-60 %; PARAKRAM: no task executed twice, the 3
same-round double awards dropped by the loser inside `award_settle` before it moved), S4
`b00edf0d` and blackout `b6c50a5e` (3 + 3 trials, 0 contacts, no false DEAD, at most once on
heal).

## Limitations

- The majority quorum makes a lone survivor of a 3-robot fleet freeze (it cannot tell two dead
  peers from its own isolation). A 2-2 split of 4 robots freezes both sides (safe, not live).
- Non-transitive connectivity (A hears C, C hears B, A and B not each other) is not relayed:
  the ack gate and the ghosts cover a robot's own neighbourhood; the reactive layer (03) is the
  floor. Lease expiry uses absolute stamps: one `/clock` in simulation; hardware needs clock
  synchronisation (or per-robot monotonic leases).
- A ghost cell is released only once this robot's lidar sees it empty (or the peer returns);
  occluded cells stay blocked (the floor is biconnected, so a detour always exists).
- The watchdog's DEAD is for task reallocation only. A fully isolated robot cannot be told from
  a dead one, and it releases its task only `award_lease_ttl - 2 s` after its last majority
  acknowledgement; reallocating earlier would break at-most-once. So in PARAKRAM, DEAD (after
  the partition grace) only makes a robot announce at once the dead robot's tasks whose award
  lease has already run out: the accelerator gains little over the 10 s award lease (it skips
  the announce jitter). A shorter `award_lease_ttl` would reallocate faster at the cost of
  more releases under loss (not tuned here: W04's value is kept).
- Liveness restoration here is the victim's space FREED at every survivor (what L bounds).
  Re-acquiring it (a survivor holding authority through it) adds the claim settle and an
  acknowledgement round trip (~1 s), and then depends on traffic (the waiting robot may yield
  or detour): it is reported, not bounded.
- Two awards of one auction round (two announcers that missed each other) still happen under
  loss; they are settled by the lower announcer within a delivery and the 1 s `award_settle`
  keeps the loser still meanwhile. A second award arriving more than ~1 s after the first
  would make both robots move until the tie is settled (not observed in the sweeps).
- A live robot that is physically stuck (e.g. pinned by the reactive layer against a wall at
  an edge station) keeps its priority and its task until 04's 60 s stall timeout; that is the
  02/04 layers' liveness, not the recovery path (seen once in S4).
- 06's silent kill is a SIGKILL of the coordination node; the bench kills the robot's
  coordination, auction and fault nodes (no renewal, heartbeat or task message) and cancels its
  Nav2 goal. Its Nav2 and reactive layer keep running, so the "dead" body may still be nudged by
  its own safety layer.
- A ghost cell is cleared when no lidar return falls inside it (4 cm inside the cell edges):
  a body intruding less than that, or missed by sparse beams from afar, does not keep it; the
  physical body stays an obstacle for the reactive layer and Nav2's costmaps.
- The watchdog reads the peers' renewals from their intents (the same stream the heartbeat node
  relays as `/<ns>/heartbeat`), because the intents carry the acknowledgements PARTITIONED needs.
- Evidence scope: 5 seeds per loss level (06's test note asks S3 x10), independent (Bernoulli)
  loss only (Gilbert-Elliott bursts not swept), 3 robots, one scenario.
- All numbers from these runs are SIMULATION (TARGET / sim in the CLAIMS_LEDGER).
