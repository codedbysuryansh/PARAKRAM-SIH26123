# TaskComplete.msg (topic: /fleet/task_complete, QoS: RELIABLE + TRANSIENT_LOCAL, KEEP_ALL)
# CLAUDE_CODE/04: published by the winner on dropoff; the task leaves every pool.
string   task_id
string   robot_id
uint32   seq
builtin_interfaces/Time stamp
