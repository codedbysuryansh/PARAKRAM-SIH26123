"""
QoS profiles by message class (CLAUDE_CODE/00 "QoS is not optional", RESEARCH/03 §1 table).

Every PARAKRAM node takes its QoS from here. CLAUDE_CODE/05 owns this module (Zenoh config,
degradation, loss injection); CLAUDE_CODE/02 and 04 added the profiles their topics need. The
profiles of the CLAUDE_CODE/05 spec: ``STATE_QOS``, ``INTENT_QOS``, ``TASK_QOS``,
``HEARTBEAT_QOS``.

* state / intent  : BEST_EFFORT, VOLATILE — loss must show up as staleness, never be hidden by
  retransmission (the loss benchmark depends on it; never switch these to RELIABLE).
* status          : diagnostics for dashboard/bench (not a coordination input).
* task pool       : CLAUDE_CODE/04 (as specified in 00): the replicated pool is reconciled from
  RELIABLE + TRANSIENT_LOCAL, KEEP_ALL topics (tasks, announcements, awards, completions), so a
  robot that (re)joins rebuilds the pool from history; bids and lease renewals are RELIABLE and
  VOLATILE (only their latest values matter). The auction is a throughput accelerator: safety and
  liveness never wait on these topics.
* heartbeat       : BEST_EFFORT + liveliness MANUAL_BY_TOPIC (05): a heartbeat is its own
  liveliness assertion; the liveliness lease (``HEARTBEAT_LEASE_S``) is a placeholder until
  CLAUDE_CODE/06 fixes the lease time L. Published by 06, read by the task layer.

The app-level loss of ``parakram_comms.loss`` is applied to state / intent only.
"""

from rclpy.duration import Duration
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, LivelinessPolicy, QoSProfile,
                       ReliabilityPolicy)

STATE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=1)

INTENT_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=5)

STATUS_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=10)

TASK_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_ALL)
TASK_POOL_QOS = TASK_QOS          # the name CLAUDE_CODE/04 uses

TASK_EVENT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE,
                            history=HistoryPolicy.KEEP_LAST, depth=100)

HEARTBEAT_LEASE_S = 2.0

HEARTBEAT_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE,
                           history=HistoryPolicy.KEEP_LAST, depth=5,
                           liveliness=LivelinessPolicy.MANUAL_BY_TOPIC,
                           liveliness_lease_duration=Duration(seconds=HEARTBEAT_LEASE_S))
