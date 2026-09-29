# TaskProgress.msg (topic: /fleet/task_progress, QoS: RELIABLE) — CLAUDE_CODE/04
# Lease renewal by the holder of an award ("task_progress renew"); RELEASED gives the task back.
string   task_id
string   robot_id
uint32   seq                # the award round being renewed
builtin_interfaces/Time lease_expiry   # the renewed lease
uint8    stage
uint8    STAGE_TO_PICKUP=0
uint8    STAGE_TO_DROPOFF=1
uint8    STAGE_RELEASED=2
