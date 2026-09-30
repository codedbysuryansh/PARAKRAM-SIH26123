# RecoveryEvent.msg (topic: /fleet/recovery_event) — CLAUDE_CODE/06 recovery log
# Every component reports what it saw; kind 'summary' (bench) carries every time of one recovery.
builtin_interfaces/Time stamp
string   kind               # kill | lease_expired | space_reclaimed | ghost_cleared | peer_back |
                            # suspect | partitioned | dead | reauction | task_reassigned |
                            # release | lease_lost | lease_regained | task_dropped | summary
string   robot_id           # reporter
string   dead_id            # the robot concerned
string   task_id
float32  loss_level
float64  t_event            # [s sim] when the reporter saw it
float64  t_kill             # summary fields [s sim], NaN when unknown
float64  t_lease_expire
float64  t_space_reclaimed
float64  t_task_reassigned
string   detail
