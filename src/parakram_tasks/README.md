# parakram_tasks — decentralized lease-gated Contract-Net (CLAUDE_CODE/04)

Task allocation by auction with **no central auctioneer**. It is an opportunistic throughput
accelerator: coordination (PIBT + spatial leases, CLAUDE_CODE/02) and the reactive safety layer
(CLAUDE_CODE/03) never depend on it.

## Nodes

- `auction_node` (one per robot, in its namespace, identical everywhere): keeps a replica of the
  task pool (`task_pool.py`) and plays every role: announcer (transient: whoever sees an unowned
  task first after a seeded random delay), bidder (`bidder.cost`, time-to-serve on the grid) and
  winner (drives the task through coordination and renews its lease). Services
  `/<ns>/reauction` (`ReAuction`, for the fault layer of CLAUDE_CODE/06).
- `task_generator` (simulation only): a task SOURCE that streams `n_tasks` pickup -> dropoff
  tasks from the scenario's `task_stations` at `task_rate`; it never announces or awards.

Flow: `/fleet/tasks` -> `TaskAnnounce` (round `seq`) -> `Bid` -> `Award` (lowest cost, ties by
robot id, `lease_expiry = now + award_lease_ttl`) -> `TaskProgress` renewals -> `TaskComplete`.
A lapsed lease (dead or stuck holder) or a missing award (announcer died mid-auction) returns
the task to every replica's pool and a peer re-announces it; a newer round (`seq`) supersedes
older awards, equal rounds go to the lower announcer id. The winner's goal goes to coordination
on `/<ns>/assigned_task` (pickup, then dropoff); arrival is read from `/<ns>/coord_status`.
Status per robot: `/fleet/task_status`. Events: `bench/logs/<run_id>/tasks.csv`
(`t, event, task_id, robot_id, cost, lease_expiry` as specified, plus `seq, detail`).

## Acceptance

```bash
ros2 launch parakram_bringup fleet_sim.launch.py n_robots:=3 scenario:=warehouse_stream seed:=1 safety:=false
ros2 launch parakram_coord coord.launch.py n_robots:=3 scenario:=warehouse_stream
ros2 launch parakram_safety safety.launch.py n_robots:=3
ros2 run parakram_tasks task_acceptance --n 3 --n-tasks 20 --kill-after 3
ros2 launch parakram_tasks tasks.launch.py n_robots:=3 task_rate:=0.2
```

`task_acceptance` kills the auction node of the robot announcing the next round after 3
completions and checks the six PASS items with Gazebo ground truth. Runs (acceptance
verification, not benchmark results): `288d8a91` and `8f4e5ca1` in `bench/logs`, both PASS.

## CLAUDE_CODE/06 additions (`recovery_mode`, from the fleet manifest)

- `lease` (PARAKRAM, default) with `award_gate` (tasks.launch.py turns it on in this mode; the
  node default keeps the 04 behaviour): **quorum award leases**, at most one robot per task.
  - The holder's lease-renewal intents (coordination, 10 Hz) also renew its award at every peer
    (`on_holder_renewal`); coordination reports on `coord_status` since when a strict majority
    of the fleet (the holder included) acknowledged renewals carrying it.
  - The holder works on the task only while that is within `award_lease_ttl - award_margin`
    (10 - 2 s); otherwise it releases it (RELEASED).
  - A robot counts award leases down, and announces, bids, awards or serves `ReAuction`, only
    while its own spatial lease is valid (it hears a majority and is acknowledged); while it is
    cut off its lease clock stops (`shift_leases`).
  - Another robot's acknowledgement of a NEW renewal of a holder renews that holder's awards
    (`extend_holder`): partitioned from this robot is not dead.
  - `ReAuction` (the watchdog's DEAD) announces at once the dead robot's tasks whose award lease
    has run out here, never earlier (the holder may be alive behind a partition).
  - Two awards of one round (two announcers that missed each other under loss): the lower
    announcer wins, as for Award messages, and the announcer also rides on the holder's
    renewal intents (`Intent.task_announcer`, from `TaskProgress.announcer_id` on the local
    `current_task`) and on the digests, so the loser drops within one delivery even if the
    winning Award was lost; a new award waits `award_settle` (1 s) before it is worked on.
  - `/fleet/task_digest` (1 Hz): completions and award rounds with their announcers, so a
    robot that missed messages converges (completions are final, newer rounds win, an entry
    without announcer never wins a same-round tie).
- `release` (`reauction_baseline`): awards never expire and there is no gate; a task moves only
  through a newer round (`ReAuction` on the watchdog's DEAD, 1 s: every survivor announces its
  own round, also of a task another survivor already re-opened, as in 06's "two peers both
  re-auction"), and a newer round of a task a silent peer held, seen from ANOTHER robot,
  releases that peer's space here (`release_peer`).
- `loss_scope:=fleet`: every inter-robot subscription goes through the robot's per-link loss
  process (`parakram_comms.link_loss`; counters `comms_<ns>_tasks.csv`).
- Tests: `test_task_pool_w06.py`, `test_node_award_gate.py` (the node's quorum rules).

## Known limitations and observations

- One task per robot at a time; busy robots do not bid (the cost supports a `load` term).
- While every live robot is busy, pending tasks are re-announced with backoff (<= 8 s): the two
  acceptance runs logged ~720-760 announcements for 21 awards.
- No fairness: an idle robot bids on its cheapest open round, so a task can wait long
  (`task_012` in `288d8a91`: ~410 s from its killed round to its award, ~480 s to completion).
- Heartbeats (`/<peer>/heartbeat`, relayed from the lease renewals by CLAUDE_CODE/06) are only
  read to skip bidders known to be stale at award time; the lease is what frees a dead holder's
  task. Stuck holders stop renewing after `stall_timeout` (60 s); coordination's
  `reroute_request` is not consumed.
- Battery: below `battery_min` a robot withholds its bids; charging is not implemented.
- Leases use sim time (one `/clock`); clock skew on hardware is CLAUDE_CODE/06's concern.
- Acceptance run numbers (makespan ~750-780 s for 20 tasks with one auction node killed at
  ~83 s) are verification only; throughput claims need the CLAUDE_CODE/07 benchmark.
