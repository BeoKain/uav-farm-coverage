"""Disarmed, independent LiDAR and PX4 pose association for each namespace."""
from bisect import bisect_left
import json
import math
from pathlib import Path
import sys
import signal
import time

import numpy as np
import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from std_msgs.msg import String
from probe_protocol import ProbeReceiver
from rclpy.qos import QoSProfile, ReliabilityPolicy

from collect_fleet import Collector, COUNT
from execution_plan import to_world
from online_corridor import OnlineCorridor
from timed_pose import PoseSeries, px4_rotation


class PoseCollector(Collector):
    def __init__(self, spawns, gate=None, flight=False, capture=None):
        self.flight=flight
        self.capture=capture
        self.capture_counts={}
        self.probe=ProbeReceiver()
        self.gate=gate
        self.ready_sent=False
        self.admit_from=None
        self.pre_ready_clouds={s['id']:0 for s in spawns}
        self.detectors = {s['id']: OnlineCorridor() for s in spawns}
        self.pending = {}
        self.origins = {}
        self.position_resets = {}
        self.decisions = {s['id']: [] for s in spawns}
        self.pose_timeline={s['id']:dict(position=[],attitude=[]) for s in spawns}
        self.clock_timeline=[]
        self.replaced={s['id']:[] for s in spawns}
        super().__init__(spawns)
        qos = QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.decision_publishers = {}
        for s in spawns:
            self.create_subscription(VehicleLocalPosition,s['namespace']+'/out/vehicle_local_position_v1',
                                     lambda msg, spawn=s:self.position(msg,spawn),qos)
            self.create_subscription(VehicleAttitude,s['namespace']+'/out/vehicle_attitude',
                                     lambda msg, name=s['id']:self.attitude(msg,name),qos)
            self.decision_publishers[s['id']] = self.create_publisher(String,f"/{s['id'].lower()}/corridor_decision",1)
        for spawn in spawns:
            self.create_subscription(String,f"/{spawn['id'].lower()}/probe_direction",
                                     self.probe_callback,1)
        self.create_timer(.02,self.process)

    def probe_callback(self,msg):
        try:
            self.probe.receive(json.loads(msg.data))
        except (TypeError,KeyError,ValueError):
            self.error='Invalid probe direction command'

    def status_callback(self,msg,spawn):
        if not self.flight:
            super().status_callback(msg,spawn)
        elif msg.system_id!=spawn['system_id']:
            self.error='Unexpected system ID'
        else:
            self.status[spawn['id']]=dict(system_id=msg.system_id,arming_state=msg.arming_state)

    def clock_callback(self,msg):
        super().clock_callback(msg)
        self.clock_timeline.append(dict(clock_s=self.clock,wall_s=time.monotonic()))

    def record_pose(self,name,kind,msg,valid):
        self.pose_timeline[name][kind].append(dict(receipt_sim_s=self.clock,
            receipt_wall_s=time.monotonic(),timestamp_us=int(msg.timestamp),
            timestamp_sample_us=int(msg.timestamp_sample),valid=bool(valid)))

    @staticmethod
    def bracket_status(history,stamp):
        times=[sample[0] for sample in history]
        i=bisect_left(times,stamp)
        before=times[i-1] if i else None
        after=times[i] if i<len(times) else None
        gap=after-before if before is not None and after is not None else None
        if after is not None and abs(after-stamp)<1e-8:
            reason='exact'
        elif before is None:
            reason='missing_before'
        elif after is None:
            reason='missing_after'
        else:
            reason='gap_too_wide' if gap>.12 else 'bracketed'
        return dict(reason=reason,before_s=before,after_s=after,gap_s=gap)

    def position(self,msg,spawn):
        name=spawn['id']
        valid=msg.xy_valid and msg.z_valid
        self.record_pose(name,'position',msg,valid)
        if self.clock is None or not valid:
            return
        reset=(msg.xy_reset_counter,msg.z_reset_counter)
        local=(msg.x,msg.y,msg.z)
        origin=self.origins.setdefault(name,local)
        if self.position_resets.setdefault(name,reset)!=reset:
            self.error=f'{name} position reset during collection'
            return
        self.detectors[name].position(self.clock,to_world(local,spawn['world_enu_m'],origin))

    def attitude(self,msg,name):
        self.record_pose(name,'attitude',msg,True)
        if self.clock is not None:
            self.detectors[name].attitude(self.clock,msg.q,msg.quat_reset_counter)

    def cloud_callback(self,msg,spawn):
        # Parent validates the frame, geometry and sequence. Keep one newest
        # message per vehicle to bound memory when pose processing lags.
        name=spawn['id']
        if self.gate is not None and (self.admit_from is None or
                msg.header.stamp.sec+msg.header.stamp.nanosec/1e9<self.admit_from):
            self.pre_ready_clouds[name]+=1
            return
        count=len(self.rows[name])
        points=super().cloud_callback(msg,spawn)
        if len(self.rows[name])>count:
            if name in self.pending:
                stamp=self.pending[name][0]
                detector=self.detectors[name]
                self.replaced[name].append(dict(stamp_s=stamp,replaced_sim_s=self.clock,
                    position=self.bracket_status(detector.positions,stamp),
                    attitude=self.bracket_status(detector.attitudes,stamp)))
            self.pending[name]=(self.rows[name][-1]['stamp_s'],msg.header.frame_id,points)

    def process(self):
        if self.gate is not None and self.admit_from is None:
            if not self.ready_sent and self.clock is not None and all(
                    len(d.positions)>=5 and len(d.attitudes)>=5 and not d.reset_fault
                    for d in self.detectors.values()):
                (self.gate/(next(iter(self.detectors))+'.ready')).touch()
                self.ready_sent=True
            if self.clock is None or not (self.gate/'start').exists():
                return
            self.admit_from=self.clock
        for name,(stamp,frame,points) in list(self.pending.items()):
            command=self.probe.for_scan(stamp)
            if command is None:
                self.pending.pop(name)
                continue
            began=time.monotonic()
            now=self.clock
            if now is None or now<stamp:
                continue
            detector=self.detectors[name]
            assess_begin=None
            capture=None
            try:
                position,_,gap_pos=PoseSeries(detector.positions).at(stamp)
                _,quaternion,gap_att=PoseSeries(detector.attitudes).at(stamp)
                if detector.reset_fault:
                    raise ValueError('Attitude reset')
            except ValueError:
                if now-stamp<.24:
                    continue
                decision=dict(state='UNKNOWN',reason='pose_bracket_unavailable',
                              position=self.bracket_status(detector.positions,stamp),
                              attitude=self.bracket_status(detector.attitudes,stamp),
                              attitude_reset=detector.reset_fault)
            else:
                from corridor import assess
                from online_corridor import POLICY
                rotation=px4_rotation(quaternion)
                assess_begin=time.monotonic()
                decision=assess(points,position,rotation,command['direction_enu'],stamp,now,POLICY)
                cells=decision.pop('cells',None)
                key=(name,decision['state'])
                if self.capture is not None and command['direction_enu'][0]<0 and self.capture_counts.get(key,0)<4:
                    self.capture_counts[key]=self.capture_counts.get(key,0)+1
                    capture=(key,cells,position,rotation)
                decision.update(pose_bracket_s=max(gap_pos,gap_att),
                                east_heading_deg=math.degrees(math.atan2(rotation[1,0],rotation[0,0])))
            finished=time.monotonic()
            self.pending.pop(name)
            decision.update(stamp_s=stamp,decision_sim_s=now,frame=frame,probe_epoch=command['epoch'],direction_enu=command['direction_enu'],
                            processing_wall_s=finished-began,process_begin_wall_s=began,
                            assess_begin_wall_s=assess_begin,process_end_wall_s=finished,
                            before_publish_wall_s=time.monotonic())
            self.decisions[name].append(decision)
            self.decision_publishers[name].publish(String(data=json.dumps(decision,allow_nan=False)))
            if capture:
                key,cells,position,rotation=capture
                filename=f'{name}_{key[1]}_{self.capture_counts[key]:02d}'
                np.savez(self.capture/f'{filename}.npz',sensor_xyz=points,
                         position=position,rotation=rotation)
                (self.capture/f'{filename}.json').write_text(
                    json.dumps(dict(decision=decision,cells=cells),indent=2)+'\n')


def main(output, spawns, gate=None, flight=False):
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=PoseCollector(spawns,gate,flight,output.parent if flight else None)
    try:
        stopping=False
        def stop(*_):
            nonlocal stopping
            stopping=True
        signal.signal(signal.SIGTERM,stop)
        signal.signal(signal.SIGINT,stop)
        deadline=time.monotonic()+(900. if flight else 65.)
        while time.monotonic()<deadline and not node.error and not stopping:
            rclpy.spin_once(node,timeout_sec=.1)
            if not flight and len(node.status)==len(spawns) and all(len(rows)>=COUNT for rows in node.decisions.values()):
                break
        metrics={}
        for name,rows in node.decisions.items():
            usable=[r for r in rows if 'pose_bracket_s' in r and r['state']=='RISE'
                    and 0<=r['decision_sim_s']-r['stamp_s']<.45]
            ages=[r['decision_sim_s']-r['stamp_s'] for r in usable]
            metrics[name]=dict(decisions=len(rows),usable=len(usable),clouds=len(node.rows[name]),
                               p95_age_s=float(np.percentile(ages,95)) if ages else None,
                               max_pose_bracket_s=max((r['pose_bracket_s'] for r in usable),default=None),
                               max_abs_heading_deg=max((abs(r['east_heading_deg']) for r in usable),default=None))
        passed=(not node.error and len(node.status)==len(spawns) and
                all(m['decisions']>=COUNT and m['usable']/max(1,m['clouds'])>=.8
                    and m['max_abs_heading_deg']<=5 for m in metrics.values()))
        output.write_text(json.dumps(dict(passed=bool(passed),error=node.error,status=node.status,metrics=metrics,
                                         decisions=node.decisions,clouds=node.rows,pose_timeline=node.pose_timeline,
                                         clock_timeline=node.clock_timeline,replaced=node.replaced,
                                         admitted_from_sim_s=node.admit_from,pre_ready_clouds=node.pre_ready_clouds),indent=2,allow_nan=False)+'\n')
        print(json.dumps(metrics,indent=2),flush=True)
        return 0 if (not node.error if flight else passed) else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    spawns=json.loads(Path(sys.argv[2]).read_text())['uav_spawns']
    if len(sys.argv)>=4:
        spawns=[s for s in spawns if s['id']==sys.argv[3]]
        if len(spawns)!=1:
            raise ValueError('Unknown UAV ID')
    raise SystemExit(main(Path(sys.argv[1]),spawns,Path(sys.argv[4]) if len(sys.argv)>=5 else None,'--flight' in sys.argv))
