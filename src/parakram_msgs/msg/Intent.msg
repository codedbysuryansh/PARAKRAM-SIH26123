# Intent.msg      (topic: /<ns>/intent, QoS: BEST_EFFORT, KEEP_LAST 5, ~10 Hz)
string   robot_id
uint32   seq
int32[]  reserved_cells     # flattened (r,c) grid cells claimed for the next horizon
geometry_msgs/Pose2D[] planned_path
int32    priority           # dynamic priority for PIBT-style yielding
builtin_interfaces/Time lease_expiry   # reservations auto-expire (dead-peer safety)
# --- CLAUDE_CODE/06: the intent IS the lease renewal (and the heartbeat) ---------------------
builtin_interfaces/Time stamp          # send time of this renewal
string[] ack_ids            # peers whose intents this robot has processed ...
uint32[] ack_seqs           # ... and the latest seq processed from each (acknowledgements)
int32[]  authority_cells    # cells the robot may occupy while this lease holds: its occupied
                            # cells and the HELD claims Nav2 drives through (the body envelope)
string   task_id            # CLAUDE_CODE/04 award this renewal also renews ("" = none) ...
uint32   task_seq           # ... and its auction round
string   task_announcer     # ... and its announcer (two awards of one round: the lower wins)
bool     lease_valid        # own lease acknowledged by every live peer (false: frozen, re-acquiring)
