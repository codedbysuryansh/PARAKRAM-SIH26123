# Bid.msg         (topic: /fleet/bid, QoS: RELIABLE)
string   task_id
string   robot_id
float32  cost               # estimated time-to-serve (lower = better)
