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

## Known limitations and observations

- One task per robot at a time; busy robots do not bid (the cost supports a `load` term).
- While every live robot is busy, pending tasks are re-announced with backoff (<= 8 s): the two
  acceptance runs logged ~720-760 announcements for 21 awards.
- No fairness: an idle robot bids on its cheapest open round, so a task can wait long
  (`task_012` in `288d8a91`: ~410 s from its killed round to its award, ~480 s to completion).
- Heartbeats (`/<peer>/heartbeat`, CLAUDE_CODE/06) are only read to skip bidders known to be
  stale at award time; until 06 nothing publishes them, and the lease is what frees a dead
  holder's task. Stuck holders stop renewing after `stall_timeout` (60 s); coordination's
  `reroute_request` is not consumed.
- Battery: below `battery_min` a robot withholds its bids; charging is not implemented.
- Leases use sim time (one `/clock`); clock skew on hardware is CLAUDE_CODE/06's concern.
- Acceptance run numbers (makespan ~750-780 s for 20 tasks with one auction node killed at
  ~83 s) are verification only; throughput claims need the CLAUDE_CODE/07 benchmark.
