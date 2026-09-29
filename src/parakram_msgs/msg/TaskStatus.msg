# TaskStatus.msg (topic: /fleet/task_status, QoS: RELIABLE, ~1 Hz, one per robot) — CLAUDE_CODE/04
# One robot's view of the replicated task pool, for the dashboard (pool size, assignments,
# completions). Each robot publishes its own view: there is no central pool.
string   robot_id
builtin_interfaces/Time stamp
uint32   n_pending          # in the pool, not awarded (includes tasks being auctioned)
uint32   n_auctioning       # announced, award not yet seen
uint32   n_assigned         # awarded, lease valid
uint32   n_completed
string[] assigned_task_ids  # parallel to assigned_robot_ids
string[] assigned_robot_ids
string   current_task_id    # this robot's task ("" = idle)
uint8    current_stage      # TaskProgress.STAGE_*
