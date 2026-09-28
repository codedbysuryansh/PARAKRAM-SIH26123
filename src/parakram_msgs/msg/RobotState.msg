# RobotState.msg   (topic: /<ns>/state, QoS: BEST_EFFORT, KEEP_LAST 1, ~20 Hz)
string   robot_id
uint32   seq
builtin_interfaces/Time stamp
geometry_msgs/Pose2D pose
geometry_msgs/Twist  velocity
float32  battery            # 0..1
uint8    status             # 0 IDLE,1 MOVING,2 BLOCKED,3 CHARGING,4 DEAD(inferred)

uint8 STATUS_IDLE=0
uint8 STATUS_MOVING=1
uint8 STATUS_BLOCKED=2
uint8 STATUS_CHARGING=3
uint8 STATUS_DEAD=4
