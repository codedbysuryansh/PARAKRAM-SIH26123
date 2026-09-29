# Bid.msg         (topic: /fleet/bid, QoS: RELIABLE)
string   task_id
string   robot_id
float32  cost               # estimated time-to-serve (lower = better)
uint32   seq                # CLAUDE_CODE/04: the auction round (TaskAnnounce.seq) this bid is for
