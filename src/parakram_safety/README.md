# parakram_safety — reactive safety layer (CLAUDE_CODE/03)

Comms-free, lidar-only final `cmd_vel` gate. Honest claim: **probabilistically safe within
sensing range, no liveness guarantee** (not a guarantee: 5 Hz scans, 0.02 m range noise,
turret-only sensing, 0.12 m lidar blind radius).

## Chain (per robot, in its namespace)

```
controller_server / behavior_server --cmd_vel_nav--> orca_filter (Part B, NH-ORCA)
  --cmd_vel_safe--> collision_monitor (Part A) --cmd_vel_monitored--> velocity_smoother
  --cmd_vel--> base
```

- `launch/safety.launch.py` brings up `collision_monitor` (Nav2, `autostart_node`, no bond) and
  `orca_filter` per robot and writes `safety_manifest.json`. `fleet_sim.launch.py` includes it by
  default (`safety:=true`); for the work order's two-step acceptance flow start fleet_sim with
  `safety:=false`, then this launch file.
- Without this layer nothing publishes `cmd_vel_monitored`: the base gets no command (fail-safe).
- Lidar dropout (> `scan_timeout`) -> the filter commands zero; the monitor stops on an invalid
  source.
- `safety_enabled:=false` (ablation A1): filter pass-through and monitor toggled off.
- Status: `/<ns>/safety_status` (`parakram_msgs/SafetyStatus`); log:
  `bench/logs/<run_id>/safety_<ns>.csv` (`t, robot_id, min_obstacle_dist, filter_active,
  intervention, cmd_in_v, cmd_in_w, cmd_out_v, cmd_out_w`).

## Part B: NH-ORCA (`orca_core.py`, pure; `orca_filter.py`, node)

Disc about the axle (0.138 m + tracking error 0.012 m), allowed holonomic velocities within
w_max·T = 0.24 rad of the heading (T = 0.4 s), RVO2 half-planes and linear program. Peers
(robot-like lidar blobs, remembered briefly by `PeerTracker`) are discs of 0.116 m + `safety_margin`
0.04 m with shared (1/2) avoidance, tau 0.8 s; shelves/walls are points with `static_margin` 0,
full avoidance, tau 0.4 s. The documented choices (and why) are in the `orca_core` module
docstring: static v_opt = 0, the "inside a radius" rules, and keeping in-place turns at a
standstill solution.

## Acceptance

`ros2 run parakram_safety safety_acceptance --n 3 --mode on|off|peers_killed` with
`fleet_sim scenario:=headon seed:=1 safety:=false`, this launch file and
`coord.launch.py reactive_only:=true`. Collisions come from Gazebo ground truth with the
01/02 footprint checker (contact at <= 0.01 m), never from a robot's own pose.

## Known limitations and observations (measured on the final code)

- Closest approach in acceptance run 1 (`bb237ca6`): **0.031 m** body gap (robot2-robot3): in
  the standoff "dance" robot3's axle came 0.243 m from robot2's turret, inside the 0.306 m peer
  radius; no contact (0 collisions in all safety-ON runs).
- W02 coordination regression with this layer (`d8c7c88d`): PASS at **4.65 legs/min**
  (4.73-4.93 before 03) with a **35.7 s** longest block (<= 25 s before 03).
- W01 smoke regression: 3/3 PASS; robot1 ~2 s slower per goal (the monitor's slowdown zone).
  The committed 02 code without this layer already fails that test 2/3 (robot2 "Failed to make
  progress"), so that failure predates 03.
- **Static-peer fallback:** neighbour velocity is NOT estimated (neither from lidar tracking, the
  work order's preference, nor from messages): peers are treated as static discs and safety
  against moving peers relies on reciprocity (every robot runs this filter). A non-cooperating
  fast mover is outside this argument.
- **Interface naming:** the filter reads the Nav2 nominal on `cmd_vel_nav`, not `/<ns>/cmd_vel`
  as the work order writes it: `cmd_vel` is the base's input (Gazebo bridge now, the TurtleBot3
  driver on hardware) and Nav2 publishes its nominal on `cmd_vel_nav`.
- The disc over-covers the Burger behind its axle: in the 0.40 m aisles it leaves ~0.05 m of
  lateral freedom, and a robot >= 0.05 m off-centre is steered back hard (W01 robot2 had the
  filter active 4.7-9.7 s per goal).
- Sensing: peers are seen only by their lidar turret; nothing inside 0.12 m of the lidar is seen;
  occlusion and limited field of view are not handled beyond the peer memory (1 s, <= 5 s).
  Speeds stay low (<= 0.22 m/s), so the stop distance is far below the 3.5 m sensing range.
- No liveness: standoffs and reciprocal dances at chokepoints are expected (the coordination
  layer's job).
