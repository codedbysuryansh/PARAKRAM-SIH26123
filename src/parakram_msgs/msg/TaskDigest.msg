# TaskDigest.msg (topic: /fleet/task_digest, ~1 Hz per robot) — CLAUDE_CODE/06 anti-entropy
# The FINAL facts this robot knows (completions) and the newest award round per assigned task,
# repeated so a robot that missed a message (loss, partition) converges without duplicates.
string   robot_id
builtin_interfaces/Time stamp
string[] done_task_ids
string[] done_by
string[] award_task_ids
string[] award_winners
uint32[] award_seqs
string[] award_announcers   # the round's awarding announcer ("~": unknown, never wins a tie)
builtin_interfaces/Time[] award_lease_expiry   # as this robot knows it: only the holder's own
                            # renewals extend a lease, so echoing digests can never keep a dead
                            # holder's award alive
