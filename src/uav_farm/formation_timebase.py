"""Clock and sample admission for SITL; no ROS dependencies.
Mission progress uses Gazebo time. Raw PX4/XRCE times are diagnostics/order hints only.
"""
import math


class SimulationClock:
    def __init__(self):
        self.us=None;self.last_step_us=None;self.received=None;self.fault=None;self.progress_received=None
    def update(self,sec,nanosec,received):
        if not isinstance(sec,int) or not isinstance(nanosec,int) or sec<0 or not 0<=nanosec<1000000000 or not math.isfinite(received):
            self.fault='Invalid Gazebo clock';return False
        stamp=sec*1000000+nanosec//1000
        if self.us is not None and stamp<self.us:
            self.fault='Gazebo simulation clock reset/backwards';return False
        if self.us is None or stamp>self.us:self.progress_received=received
        self.us=stamp;self.received=received;return True
    def ready(self,now):return self.fault is None and self.us is not None and self.received is not None and 0<=now-self.received<=.5 and self.progress_received is not None and 0<=now-self.progress_received<=.5
    def step(self,now):
        if self.fault:raise ValueError(self.fault)
        if not self.ready(now):raise ValueError('Gazebo clock stale or unavailable')
        if self.last_step_us is None:self.last_step_us=self.us;return 0.
        delta=(self.us-self.last_step_us)/1e6;self.last_step_us=self.us
        return min(.1,delta)
    def summary(self):return dict(source='Gazebo /clock',latest_us=self.us,fault=self.fault,progress_wall=self.progress_received)


class SampleGate:
    """Reject duplicate/old packets within one domain; report large rewinds.
    Domain switches match observed raw-boot vs UTC-adjusted XRCE stamps. They do
    not rebase mission time or override estimator/arming/reset safety checks.
    """
    def __init__(self):
        self.latest={};self.domain=None;self.switches=0;self.dropped=0;self.fault=None
    def accept(self,stamp):
        if not isinstance(stamp,int) or stamp<=0:self.fault='Invalid PX4 sample timestamp';return False
        domain='utc_adjusted' if stamp>=100000000000000 else 'boot'
        last=self.latest.get(domain)
        if last is not None and stamp<=last:
            self.dropped+=1
            if last-stamp>500000:self.fault='Large same-domain PX4 timestamp rewind'
            return False
        if self.domain is not None and self.domain!=domain:self.switches+=1
        self.domain=domain;self.latest[domain]=stamp;return True
    def summary(self):return dict(domain=self.domain,domain_switches=self.switches,dropped_packets=self.dropped,fault=self.fault)
