"""Offline integration of LiDAR recovery with the existing mission supervisor."""
from itertools import combinations
import math

from execution_supervisor import ExecutionSupervisor


class LiftedSupervisor(ExecutionSupervisor):
    lift_m = 0.

    def targets(self):
        return {u: (x, y, z+self.lift_m) for u, (x, y, z) in super().targets().items()}


class FleetGuard:
    CLEAR = 'NO_RISE_OBSERVED'
    MAX_AGE = .45
    STEP = .5
    MAX_LIFT = 1.5
    RATE = .25
    SETTLE = .5
    MAX_RECOVERY = 20.

    def __init__(self, supervisor):
        if len(supervisor.ids) != 3:
            raise ValueError('Three active UAVs required')
        self.supervisor = supervisor
        self.state = 'TRACK'
        self.goal_lift = 0.
        self.started = self.still_since = None
        self.last_sim = None
        self.clear_stamps = {u: set() for u in supervisor.ids}
        self.events = []
        self.velocity = {u: (0., 0., 0.) for u in supervisor.ids}

    def change(self, state, now, **details):
        self.state = state
        self.events.append(dict(state=state, sim_s=now, reference_s=self.supervisor.time,
                                lift_m=self.supervisor.lift_m, **details))

    def tick(self, dt, positions, velocities, sim_s, arm_s, observations, age_s=0., allow=True):
        """Return fleet setpoints. Exceptions require coordinated abort/replan."""
        sup = self.supervisor
        if not all(math.isfinite(t) for t in (dt, sim_s, arm_s, age_s)):
            raise ValueError('Non-finite clock or telemetry age')
        if (not 0<=dt<=sup.p['max_step_s']+1e-8 or not 0<=age_s<=sup.p['max_age_s']
                or arm_s<sim_s or (self.last_sim is not None and sim_s<self.last_sim)):
            raise ValueError('Invalid time or stale fleet telemetry')
        self.last_sim = sim_s
        refs = sup.targets()
        for samples in (positions, velocities):
            if set(samples)!=set(sup.ids) or any(len(v)!=3 or not all(map(math.isfinite,v))
                                                for v in samples.values()):
                raise ValueError('Missing or invalid fleet telemetry')
        if any(not sup.p['min_distance_m']<=math.dist(positions[a],positions[b])<=sup.p['max_distance_m']
               for a,b in combinations(sup.ids,2)):
            raise ValueError('Measured formation envelope violated')
        if any(math.dist(positions[u],refs[u])>sup.p['tracking_budget_m'] for u in sup.ids):
            raise ValueError('Measured tracking budget exceeded')
        center = tuple(sum(refs[u][k] for u in sup.ids)/3 for k in range(3))
        if any(math.dist(q,center)>sup.p['max_radius_m'] for q in positions.values()):
            raise ValueError('Measured formation radius violated')
        if any(z-sup.terrain.height(x,y)<sup.p['min_agl_m'] for x,y,z in positions.values()):
            raise ValueError('Measured AGL below policy')

        states = {}
        for u in sup.ids:
            item = observations.get(u, {})
            stamp = item.get('stamp_s', -math.inf)
            state = item.get('state')
            states[u] = state if 0<=sim_s-stamp<self.MAX_AGE and state in (self.CLEAR,'RISE') else 'UNKNOWN'
        rises = [u for u in sup.ids if states[u]=='RISE']
        valid = all(v!='UNKNOWN' for v in states.values())
        clear = valid and not rises
        if self.state=='TRACK' and clear:
            if sup.lift_m and sup.index<len(sup.segments):
                segment = sup.segments[sup.index]
                if segment['kind']=='SERVICE' and not sup.plan.get('lifted_service_revalidation'):
                    raise RuntimeError('Lifted route requires replanning before service')
                # Formation offsets have zero vertical component and the
                # reference interpolates linearly between segment endpoints.
                ceiling = sup.plan.get('flight_ceiling_m', 22.)
                if max(segment['start'][2],segment['end'][2])+sup.lift_m>ceiling:
                    raise RuntimeError('Lifted transit exceeds altitude ceiling')
            commands = sup.tick(dt,positions,velocities,sim_s,arm_s,allow=allow,age_s=age_s)
            self.velocity = dict(sup.velocity)
            return commands
        self.velocity = {u:(0.,0.,0.) for u in sup.ids}
        # A guard pause is outside the supervisor's ticks. Require a new
        # settling interval before any subsequent service credit.
        sup.settled = 0.
        if self.state=='TRACK':
            self.started = sim_s
            self.still_since = None
            self.change('HOLD',sim_s,warning_uavs=rises,observations=states)
        if sim_s-self.started>self.MAX_RECOVERY:
            raise TimeoutError('Fleet LiDAR recovery timed out')
        settled = all(math.dist(positions[u],refs[u])<=.25 and math.hypot(*velocities[u])<=.10
                      for u in sup.ids)
        if self.state=='CLIMB' and sup.lift_m<self.goal_lift:
            if valid:
                sup.lift_m=min(self.goal_lift,sup.lift_m+self.RATE*dt)
            return sup.targets()
        if not settled:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = sim_s
        if not settled or not valid:
            for stamps in self.clear_stamps.values():
                stamps.clear()
            return refs
        gate = self.still_since+self.SETTLE
        if sim_s<gate or any(observations[u]['stamp_s']<gate for u in sup.ids):
            return refs
        if rises:
            if sup.lift_m+self.STEP>self.MAX_LIFT:
                raise RuntimeError('Common lift ceiling reached')
            ceiling = sup.plan.get('flight_ceiling_m', 22.)
            if max(q[2] for q in refs.values())+self.STEP>ceiling:
                raise RuntimeError('Absolute altitude ceiling reached')
            self.goal_lift = sup.lift_m+self.STEP
            self.still_since = None
            for stamps in self.clear_stamps.values():
                stamps.clear()
            self.change('CLIMB',sim_s,warning_uavs=rises,target_lift_m=self.goal_lift)
        else:
            for u in sup.ids:
                self.clear_stamps[u].add(observations[u]['stamp_s'])
            if all(len(s)>=2 for s in self.clear_stamps.values()):
                self.change('TRACK',sim_s,clear_stamps={u:sorted(s) for u,s in self.clear_stamps.items()})
                for stamps in self.clear_stamps.values():
                    stamps.clear()
        return refs
