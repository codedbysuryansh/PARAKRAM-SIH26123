# PeerStatus.msg (topic: /<ns>/peer_status, ~2 Hz per peer) — CLAUDE_CODE/06 watchdog
# One robot's view of one peer, from the peer's lease renewals (its intents) as received here.
string   robot_id           # observer
string   peer_id
builtin_interfaces/Time stamp
uint8    status
float32  last_heard_age     # [s] since the peer's last renewal was received
float32  window_delivery    # fraction of the expected renewals received in the window
float32  heard_by_others_age  # [s] since another peer last acknowledged a NEW renewal of it
                            # (indirect evidence of life; < 0: never)
uint8 ALIVE=0               # renewals arriving
uint8 FLAKY=1               # renewals arriving with gaps (lossy link)
uint8 SUSPECT=2             # silent here longer than the detect timeout
uint8 PARTITIONED=3         # silent here, but other peers still receive its renewals
uint8 DEAD=4                # silent everywhere past the partition grace: tasks may be reallocated
