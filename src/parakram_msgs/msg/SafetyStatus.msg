# SafetyStatus.msg (topic: /<ns>/safety_status, every safety-filter tick) — added in CLAUDE_CODE/03
# State of the robot's reactive safety layer, for the dashboard and logs. Everything here is
# computed from the robot's OWN lidar and commands; no peer message is used.
string   robot_id
builtin_interfaces/Time stamp
bool     safety_enabled       # false = ablation A1: filter passes through, collision monitor off
bool     monitor_enabled      # collision monitor state last confirmed by its toggle service
bool     scan_ok              # a fresh lidar scan is available (false -> fail-safe stop)
float32  min_obstacle_dist    # [m] sensed clearance: footprint to the nearest valid lidar return
bool     filter_active        # the NH-ORCA filter changed the nominal command this tick
bool     intervention         # filter_active, or the collision monitor is stopping / slowing
uint32   interventions        # interventions so far (rising edges of `intervention`)
uint8    monitor_action       # collision monitor action (nav2_msgs/CollisionMonitorState)
string   monitor_polygon      # the collision monitor zone that triggered it
uint16   n_peer_blobs         # robot-like lidar blobs inflated as robots this tick
float32  min_barrier          # [m] smallest d - R, obstacle distance minus inflated radius (< 0: inside)
float32  cmd_in_v             # nominal command from Nav2 (cmd_vel_nav)
float32  cmd_in_w
float32  cmd_safe_v           # filter output (cmd_vel_safe)
float32  cmd_safe_w
float32  cmd_out_v            # command the base receives (cmd_vel, collision monitor output)
float32  cmd_out_w
