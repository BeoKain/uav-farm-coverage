"""Bounded causal pose assembly for a pending LiDAR frame."""
from collections import deque
import numpy as np

from corridor import Policy, assess
from timed_pose import PoseSeries, px4_rotation

POLICY = Policy(lookahead_m=12.)


class OnlineCorridor:
    def __init__(self):
        self.positions = deque(maxlen=32)
        self.attitudes = deque(maxlen=64)
        self.attitude_reset = None
        self.reset_fault = False

    def position(self, stamp, xyz):
        if not self.positions or stamp > self.positions[-1][0]:
            self.positions.append((stamp, xyz, [1, 0, 0, 0]))

    def attitude(self, stamp, q, reset):
        if self.attitude_reset is not None and reset != self.attitude_reset:
            self.reset_fault = True
        self.attitude_reset = reset
        if not self.attitudes or stamp > self.attitudes[-1][0]:
            self.attitudes.append((stamp, [0, 0, 0], q))

    def decide(self, xyz, stamp, now):
        if self.reset_fault:
            return dict(state='UNKNOWN', reason='attitude_reset')
        try:
            p, _, _ = PoseSeries(self.positions).at(stamp)
            _, q, _ = PoseSeries(self.attitudes).at(stamp)
        except (ValueError, IndexError):
            return None
        return assess(xyz, p, px4_rotation(q), [1, 0], stamp, now, POLICY)
