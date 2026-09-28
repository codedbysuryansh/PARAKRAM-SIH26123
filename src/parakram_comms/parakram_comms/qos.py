"""
QoS profiles by message class (CLAUDE_CODE/00 "QoS is not optional", RESEARCH/03 §1 table).

Every PARAKRAM node takes its QoS from here. CLAUDE_CODE/05 owns this module (Zenoh config,
degradation, loss injection); CLAUDE_CODE/02 added the profiles its coordination topics need.

* state / intent  : BEST_EFFORT, VOLATILE — loss must show up as staleness, never be hidden by
  retransmission (the loss benchmark depends on it; never switch these to RELIABLE).
* status          : diagnostics for dashboard/bench (not a coordination input).
"""

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

STATE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=1)

INTENT_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=5)

STATUS_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
