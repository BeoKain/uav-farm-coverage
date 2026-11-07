"""Cooperative cancellation: a signal requests recovery, never ROS shutdown."""
import signal


class StopRequest:
    def __init__(self):
        self.count=0
        self.signal_number=None

    @property
    def requested(self):return self.count>0

    def receive(self,signum,_frame=None):
        self.count+=1
        if self.signal_number is None:self.signal_number=signum

    def install(self):
        previous={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
        for s in previous:signal.signal(s,self.receive)
        return previous


def restore_handlers(previous):
    for s,handler in previous.items():signal.signal(s,handler)
