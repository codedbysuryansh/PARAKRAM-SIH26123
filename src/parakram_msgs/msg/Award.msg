# Award.msg       (topic: /fleet/award, QoS: RELIABLE + TRANSIENT_LOCAL)
string   task_id
string   winner_id
builtin_interfaces/Time lease_expiry     # winner must renew or forfeit
