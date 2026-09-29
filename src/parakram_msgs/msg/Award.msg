# Award.msg       (topic: /fleet/award, QoS: RELIABLE + TRANSIENT_LOCAL)
string   task_id
string   winner_id
builtin_interfaces/Time lease_expiry     # winner must renew or forfeit
uint32   seq                # CLAUDE_CODE/04: auction round; on reconciliation the higher seq wins
string   announcer_id       # CLAUDE_CODE/04: who ran this round (equal seq: the lower id's award)
float32  cost               # CLAUDE_CODE/04: the winning bid
