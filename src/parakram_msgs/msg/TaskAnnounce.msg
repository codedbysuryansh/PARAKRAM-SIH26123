# TaskAnnounce.msg (topic: /fleet/task_announce, QoS: RELIABLE + TRANSIENT_LOCAL, KEEP_ALL)
string   task_id
geometry_msgs/Pose2D pickup
geometry_msgs/Pose2D dropoff
string   announcer_id
builtin_interfaces/Time announce_time
uint32   seq                # CLAUDE_CODE/04: auction round / task version (a higher seq supersedes)
