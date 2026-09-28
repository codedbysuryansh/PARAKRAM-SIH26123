# Intent.msg      (topic: /<ns>/intent, QoS: BEST_EFFORT, KEEP_LAST 5, ~10 Hz)
string   robot_id
uint32   seq
int32[]  reserved_cells     # flattened (r,c) grid cells claimed for the next horizon
geometry_msgs/Pose2D[] planned_path
int32    priority           # dynamic priority for PIBT-style yielding
builtin_interfaces/Time lease_expiry   # reservations auto-expire (dead-peer safety)
