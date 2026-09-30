# CoordStatus.msg (topic: /<ns>/coord_status, every coordination tick) — added in CLAUDE_CODE/02
# Coordination status for the dashboard / bench. Cells are flattened (r * cols + c); -1 = none.
string   robot_id
builtin_interfaces/Time stamp
int32    cell               # current cell (robot centre)
int32    goal_cell          # current goal cell
int32    priority           # effective priority (includes inheritance / deadlock bump)
int32[]  reserved_cells     # cells this robot reserves (occupied cells first, then claims ahead)
bool     blocked            # waiting/stuck longer than blocked_after
uint8    status             # RobotState.STATUS_*
uint32   yields
uint32   replans
uint32   goals_assigned
uint32   goals_completed
uint8    n_peers            # peers with a valid (unexpired) lease
float32  tick_compute_ms
string   note               # e.g. "evacuating for robot2", "goal cell physically occupied"
# --- CLAUDE_CODE/06 lease state (read by the robot's own task layer) -------------------------
bool     lease_valid        # own spatial lease acknowledged by every live peer and a quorum heard
builtin_interfaces/Time task_renewal_acked   # oldest, over live peers, of the latest ACKED
                            # renewal that carried the current task award (0: none acknowledged)
string   task_id            # the award the intents are renewing
uint32   task_seq
