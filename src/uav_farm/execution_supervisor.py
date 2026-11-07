"""Feedback-gated smooth-cruise mission and telemetry-based geometric service ledger."""
import math
from baseline_planner import IDS,PAIRS,poses,closest_pair
from terrain_links import evaluate
from cruise_profile import segment_kinematics


class ExecutionSupervisor:
    def __init__(self,plan,terrain,slow=False):
        self.plan,self.terrain,self.p=plan,terrain,plan['policy']
        self.ids=plan['ids'];self.segments=plan['mission']['segments']
        self.index=0;self.local=0.;self.time=0.;self.wall=0.;self.elapsed=0.
        self.duration=self.segments[-1]['end_s'];self.paused=False;self.pause_wall=0.;self.pause_ticks=0
        self.events=[];self.last_reason=None;self.done=False;self.settled=0.
        self.credit_log=[]
        self.slow=slow;self.held=None;self.velocity={u:(0.,0.,0.) for u in self.ids}
        self.services=[s for s in plan['mission']['services'] if s['uav'] in self.ids]
        self.sensors={s['id']:s for s in plan['config']['sensors']}
        self.ledger={s['sensor']:dict(uav=s['uav'],required_s=s['end_s']-s['start_s'],credited_s=0.,completed=False,
                                    completed_mission_sim_s=None,completed_since_arm_s=None) for s in self.services}

    def targets(self):
        if self.index>=len(self.segments):state=self.segments[-1]['end']
        else:
            seg=self.segments[self.index];s=segment_kinematics(seg,self.local)[0]
            state=[a+s*(b-a) for a,b in zip(seg['start'],seg['end'])]
        return {u:q for u,q in zip(IDS,poses(state)) if u in self.ids}

    def feedforward(self):
        if self.paused or self.index>=len(self.segments):return {u:(0.,0.,0.) for u in self.ids}
        seg=self.segments[self.index];ds=segment_kinematics(seg,self.local)[1]
        return {name:tuple(ds*(y-x) for x,y in zip(a,b)) for name,a,b in zip(IDS,poses(seg['start']),poses(seg['end'])) if name in self.ids}

    def tick(self,dt,positions,velocities,elapsed_sim_s,since_arm_s,allow=True,age_s=0.):
        if not math.isfinite(dt) or not 0<=dt<=self.p['max_step_s']+1e-8:raise ValueError('Invalid feedback step')
        if not math.isfinite(age_s) or not 0<=age_s<=self.p['max_age_s']:raise ValueError('Stale position data')
        if not math.isfinite(elapsed_sim_s) or elapsed_sim_s<self.elapsed or not math.isfinite(since_arm_s) or since_arm_s<elapsed_sim_s:
            raise ValueError('Invalid elapsed simulation clock')
        if set(positions)!=set(self.ids) or set(velocities)!=set(self.ids) or any(len(q)!=3 or not all(math.isfinite(v) for v in q) for q in [*positions.values(),*velocities.values()]):
            raise ValueError('Finite active-fleet telemetry required')
        self.wall+=dt;self.elapsed=elapsed_sim_s
        ref=self.targets();error=max(math.dist(positions[u],ref[u]) for u in self.ids)
        if error>self.p['tracking_budget_m']:raise ValueError('Measured tracking budget exceeded')
        distances=[math.dist(positions[IDS[a]],positions[IDS[b]]) for a,b in PAIRS] if len(self.ids)==3 else []
        if distances and (min(distances)<self.p['min_distance_m'] or max(distances)>self.p['max_distance_m']):
            raise ValueError('Measured formation envelope violated')
        if len(self.ids)==3:
            center=[sum(ref[u][k] for u in self.ids)/3 for k in range(3)]
            if max(math.dist(q,center) for q in positions.values())>self.p['max_radius_m']:
                raise ValueError('Measured formation radius violated')
        for u,q in positions.items():
            if q[2]-self.terrain.height(*q[:2])<self.p['min_agl_m']:raise ValueError('Measured AGL below policy')
        if dt==0:
            self.velocity={u:(0.,0.,0.) for u in self.ids}
            commands=self.targets()
            if self.held is not None and 10<=elapsed_sim_s<25:commands['UAV2']=self.held
            return commands
        threshold=self.p['resume_error_m'] if self.paused else self.p['pause_error_m']
        reason='tracking' if error>threshold else 'external_monitor' if not allow else None
        if distances and (min(distances)<self.p['warning_min_m'] or max(distances)>self.p['warning_max_m']):
            reason=reason or 'pair_warning'
        if self.index<len(self.segments) and self.segments[self.index]['kind']=='SERVICE':
            seg=self.segments[self.index]
            scheduled=[s for s in self.services if s['start_s']>=seg['start_s']-1e-8 and s['end_s']<=seg['end_s']+1e-8]
            ready=all(math.dist(positions[u],ref[u])<=self.p['service_position_error_m'] and math.sqrt(sum(v*v for v in velocities[u]))<=self.p['service_speed_m_s'] for u in self.ids)
            ready=ready and all(evaluate(self.terrain,positions[s['uav']],self.sensors[s['sensor']]['world_enu_m'],
                    max_distance=self.p['collection_range_m'],minimum_uav_agl=self.p['min_agl_m'])['geometry_eligible'] for s in scheduled)
            self.settled=self.settled+dt if ready and reason is None else 0.
            if not ready or self.settled<self.p['service_settle_s']-1e-8:reason=reason or 'service_not_settled'
        else:self.settled=0.
        self.paused=reason is not None
        if reason:
            self.pause_ticks+=1;self.pause_wall+=dt
            if self.pause_wall>self.p['max_pause_s']:raise TimeoutError('Feedback pause timeout')
        else:
            self.pause_wall=0.
            if self.index<len(self.segments):
                seg=self.segments[self.index];step=min(dt,seg['duration_s']-self.local)
                before=seg['start_s']+self.local;after=before+step
                if seg['kind']=='SERVICE':
                    for s in self.services:
                        overlap=max(0.,min(after,s['end_s'])-max(before,s['start_s']))
                        if overlap:
                            entry=self.ledger[s['sensor']];entry['credited_s']+=overlap
                            self.credit_log.append(dict(sensor=s['sensor'],uav=s['uav'],credited_s=overlap,
                                elapsed_sim_s=elapsed_sim_s,since_arm_s=since_arm_s,reference_start_s=max(before,s['start_s']),
                                reference_end_s=min(after,s['end_s']),position=positions[s['uav']],velocity=velocities[s['uav']],target=ref[s['uav']]))
                            if not entry['completed'] and entry['credited_s']>=entry['required_s']-1e-7:
                                entry.update(completed=True,completed_mission_sim_s=elapsed_sim_s,completed_since_arm_s=since_arm_s)
                self.local+=step;self.time=after
                if self.local>=seg['duration_s']-1e-8:
                    self.index+=1;self.local=0.;self.settled=0.
        if reason!=self.last_reason:
            self.events.append(dict(elapsed_sim_s=elapsed_sim_s,reference_s=self.time,event='pause' if reason else 'resume',reason=reason));self.last_reason=reason
        commands=self.targets();self.velocity=self.feedforward()
        if self.slow and 10<=elapsed_sim_s<25 and 'UAV2' in self.ids:
            if self.held is None:self.held=positions['UAV2']
            commands['UAV2']=self.held;self.velocity['UAV2']=(0.,0.,0.)
        elif self.held is not None:self.held=None
        if self.index>=len(self.segments):
            if not all(v['completed'] for v in self.ledger.values()):raise ValueError('Reference ended without complete geometric service ledger')
            self.done=max(math.dist(positions[u],commands[u]) for u in self.ids)<.3
        return commands

    def summary(self):
        return dict(reference_s=self.time,elapsed_mission_sim_s=self.elapsed,pause_ticks=self.pause_ticks,events=self.events,
                    expected_sensors=len(self.ledger),completed_sensors=sum(v['completed'] for v in self.ledger.values()),ledger=self.ledger,credit_log=self.credit_log,
                    deadline_misses_mission_clock=sum(v['completed'] and v['completed_mission_sim_s']>self.sensors[s]['deadline_s'] for s,v in self.ledger.items()),
                    deadline_misses_since_arm=sum(v['completed'] and v['completed_since_arm_s']>self.sensors[s]['deadline_s'] for s,v in self.ledger.items()),
                    scope='Telemetry-gated geometric service time; no actual sensor bytes, radio or battery measurement')
