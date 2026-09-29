# Task.msg (topic: /fleet/tasks, QoS: RELIABLE + TRANSIENT_LOCAL, KEEP_ALL) — CLAUDE_CODE/04
# A new pickup -> dropoff task entering the replicated pool (from the sim task_generator or any
# robot that discovers a task). It is NOT an announcement: robots announce it (TaskAnnounce).
string   task_id
geometry_msgs/Pose2D pickup
geometry_msgs/Pose2D dropoff
builtin_interfaces/Time stamp
string   source             # e.g. "task_generator"
