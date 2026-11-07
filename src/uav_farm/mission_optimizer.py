"""Offline deadline-aware route search + energy-scored smooth formation transits.

No PX4 flight commands. Reference validation is conditional; energy remains uncalibrated.
"""
import argparse
import copy
import csv
import functools
import hashlib
import itertools
import json
import math
from pathlib import Path

from baseline_planner import (ROOT, IDS, PAIRS, POLICY, poses, terrain_ceiling, duration as quintic_duration,
                              closest_pair, eligible, state_at, plan as baseline_plan,
                              trajectory, render, verify_services)
from terrain_links import TerrainGrid
from energy_model import EnergyModel
from cruise_profile import duration,make_profile,validate_profile


def interpolate(a, b, t):
    return tuple(x+t*(y-x) for x, y in zip(a, b))


def corridor_clear(a, b, terrain, p):
    """Conservative tracking tube, subdivided in space (not trajectory sample time).

    In each slice, bound ground over all three swept XY rectangles. Every terrain
    triangle's height lies below its vertex max; common altitude is affine in
    the common spatial progress. This covers a horizontal/vertical 2 m box error.
    """
    count = max(1, math.ceil(max(math.dist(x,y) for x,y in zip(poses(a),poses(b)))/2))
    clearance = math.inf
    for i in range(count):
        x, y = interpolate(a,b,i/count), interpolate(a,b,(i+1)/count)
        ceiling = terrain_ceiling(terrain, poses(x)+poses(y), p['tracking_budget_m'])
        clearance = min(clearance, min(x[2],y[2])-ceiling-p['tracking_budget_m'])
        if clearance < p['min_agl_m']-1e-8:
            return False, clearance
    return True, clearance


def safe_waypoints(a, b, terrain, p):
    if corridor_clear(a, b, terrain, p)[0]:
        return [a,b]
    ceiling = terrain_ceiling(terrain, poses(a)+poses(b), p['tracking_budget_m'])
    high = max(a[2], b[2], ceiling+p['planned_clearance_m'])
    points = [a,(a[0],a[1],high,a[3]),(b[0],b[1],high,b[3]),b]
    points = [q for i,q in enumerate(points) if i==0 or math.dist(q,points[i-1])>1e-9]
    # Remove a redundant stop only when the whole shortcut's tracking tube clears.
    changed = True
    while changed:
        changed = False
        for i in range(1,len(points)-1):
            if corridor_clear(points[i-1],points[i+1],terrain,p)[0]:
                del points[i]
                changed = True
                break
    # Replace a vertical descent at the destination with a descending approach.
    # Try the longest diagonal first and retain the same tracking-tube verifier.
    if len(points)>=3 and points[-2][:2]==points[-1][:2] and points[-2][2]>points[-1][2]:
        incoming=points[-3]
        top=points[-2]
        if math.dist(incoming[:2],top[:2])>1e-8:
            for fraction in (0.,.25,.5,.75,.9,.95):
                bend=interpolate(incoming,top,fraction)
                if corridor_clear(incoming,bend,terrain,p)[0] and corridor_clear(bend,b,terrain,p)[0]:
                    points=[*points[:-2],*([bend] if fraction else []),b]
                    break
    if any(not corridor_clear(x,y,terrain,p)[0] for x,y in zip(points,points[1:])):
        raise ValueError('No safe connector found')
    return points


def validate_mission(result, config, terrain):
    p = result['policy']
    minimum, maximum, gap, radius, tube = math.inf, 0., math.inf, 0., math.inf
    previous = None
    end_s = 0.
    for seg in result['segments']:
        a,b,dt = seg['start'],seg['end'],seg['duration_s']
        if dt <= 0 or not math.isfinite(dt) or abs(seg['start_s']-end_s)>1e-7 or abs(seg['end_s']-seg['start_s']-dt)>1e-7:
            raise ValueError('Non-contiguous segment clock')
        if previous is not None and math.dist(a,previous)>1e-8:
            raise ValueError('Discontinuous formation reference')
        previous, end_s = b,seg['end_s']
        if seg['kind']=='SERVICE':
            if math.dist(a,b)>1e-8:
                raise ValueError('Moving service segment is forbidden')
        elif 'profile' not in seg and dt < quintic_duration(a,b,p)-1e-8:
            raise ValueError('Speed/acceleration bound failed')
        if 'profile' in seg:validate_profile(seg,p)
        pa,pb = poses(a),poses(b)
        for state in (a,b):
            if not all(math.isfinite(v) for v in state) or not .9 <= state[3] <= 1.1:
                raise ValueError('Invalid team state')
            if state[2] > p['max_world_altitude_m']+1e-8:
                raise ValueError('World altitude bound failed')
            for q in poses(state):
                margin = p['tracking_budget_m']
                xmin,xmax,ymin,ymax = config['flight_bounds_xy_m']
                if not xmin+margin-1e-8 <= q[0] <= xmax-margin+1e-8 or not ymin+margin-1e-8 <= q[1] <= ymax-margin+1e-8:
                    raise ValueError('Reference outside flight bounds with tracking margin')
            radius = max(radius,*(math.dist(q,state[:3]) for q in poses(state)))
        for i,j in PAIRS:
            minimum = min(minimum,closest_pair(pa[i],pb[i],pa[j],pb[j]))
            maximum = max(maximum,math.dist(pa[i],pa[j]),math.dist(pb[i],pb[j]))
        good, lower = corridor_clear(a,b,terrain,p)
        if not good:
            raise ValueError('Terrain tracking tube failed')
        tube = min(tube,lower)
        for x,y in zip(pa,pb):
            gap = min(gap,*(q['vertical_clearance_m'] for q in terrain.line_profile(x,y)))
    budget = p['tracking_budget_m']
    if minimum-2*budget < p['min_distance_m']-1e-8 or maximum+2*budget > p['max_distance_m']+1e-8 or radius+budget > p['max_radius_m']+1e-8:
        raise ValueError('Formation tracking margin failed')
    if math.dist(result['segments'][0]['start'],result['initial_state'])>1e-8 or math.dist(previous,result['initial_state'])>1e-8:
        raise ValueError('Mission does not return above launch pads')
    verify_services(result,config,terrain)
    service_segments = [s for s in result['segments'] if s['kind']=='SERVICE']
    if len(service_segments) != len(result['stages']):
        raise ValueError('Stage/service segment mismatch')
    for stage,seg in zip(result['stages'],service_segments):
        if math.dist(stage['state'],seg['start'])>1e-8 or abs(stage['start_s']-seg['start_s'])>1e-8 or abs(stage['end_s']-seg['end_s'])>1e-8:
            raise ValueError('Service stage not realized by trajectory')
    deadlines = {s['id']:s['deadline_s'] for s in config['sensors']}
    lateness = [max(0.,s['end_s']-deadlines[s['sensor']]) for s in result['services']]
    return dict(reference_min_pair_m=minimum,reference_max_pair_m=maximum,
                reference_min_agl_m=gap,reference_max_radius_m=radius,
                conservative_agl_with_tracking_m=tube,assumed_tracking_budget_m=budget,
                deadline_misses=sum(v>1e-8 for v in lateness),total_lateness_s=sum(lateness),
                deadline_feasible=all(v<=1e-8 for v in lateness),
                note='Conditional reference geometry only. Deadline clock starts above pads, excluding launch.')


class Search:
    def __init__(self,config,terrain,model,p,stages,start):
        self.config,self.terrain,self.model,self.p = config,terrain,model,p
        self.sensors = {s['id']:s for s in config['sensors']}
        self.start = tuple(start)
        self.stops = [(tuple(s['state']),tuple(s['sensor_ids'])) for s in stages]
        self.edge_cache = {}
        self.evaluations = 0
        self.service_cache = {}

    def service(self,index):
        if index in self.service_cache:
            return self.service_cache[index]
        state,sids = self.stops[index]
        options = {sid:eligible(self.terrain,state,self.sensors[sid],self.p) for sid in sids}
        if any(not options[sid] for sid in sids):
            raise ValueError('Stage lost sensor geometry')
        queues,assigned = [0.,0.,0.],[]
        for sid in sorted(sids,key=lambda s:(len(options[s]),self.sensors[s]['deadline_s'],s)):
            i = min(options[sid],key=lambda i:(queues[i],i))
            dt = 8*self.sensors[sid]['data_bytes']/self.p['nominal_link_bps']+self.p['service_guard_s']
            assigned.append(dict(sensor=sid,uav=IDS[i],start_s=queues[i],end_s=queues[i]+dt,
                                 assumed_bytes=self.sensors[sid]['data_bytes'],position=poses(state)[i]))
            queues[i] += dt
        output = (max(queues),assigned)
        self.service_cache[index] = output
        return output

    def edge(self,a,b,mode):
        key = (a,b,mode)
        if key in self.edge_cache:
            return self.edge_cache[key]
        points = safe_waypoints(a,b,self.terrain,self.p)
        output = []
        for x,y in zip(points,points[1:]):
            if math.dist(x,y)<1e-9:
                continue
            fastest = duration(x,y,self.p)
            factors = [1.] if mode=='fast' else [1.,1.15,1.4,1.8,2.5]
            choices = []
            for factor in factors:
                dt = fastest*factor
                energy = self.model.segment(x,y,dt,make_profile(x,y,self.p,dt))
                choices.append((sum(energy[u]['total_j'] for u in IDS),dt))
            _,dt = min(choices)
            output.append((x,y,dt))
        self.edge_cache[key] = output
        return output

    def build(self,order,mode='energy'):
        self.evaluations += 1
        result = dict(schema='formation_energy_offline_v6',executable_in_sitl=False,
                      scope='Airborne reference only; no ROS/PX4/Gazebo execution or measured radio/battery.',
                      policy=self.p,initial_state=self.start,stages=[],services=[],segments=[])
        clock,current = 0.,self.start
        def append(a,b,dt,kind):
            nonlocal clock
            result['segments'].append(dict(start=a,end=b,start_s=clock,end_s=clock+dt,duration_s=dt,kind=kind,
                                           profile=make_profile(a,b,self.p,dt)))
            clock += dt
        for i in order:
            state,sids = self.stops[i]
            for x,y,dt in self.edge(current,state,mode):
                append(x,y,dt,'TRANSIT')
            dt,assignments = self.service(i)
            start_s = clock
            stage_index = len(result['stages'])
            for s in assignments:
                result['services'].append(dict(s,stage=stage_index,start_s=start_s+s['start_s'],end_s=start_s+s['end_s']))
            append(state,state,dt,'SERVICE')
            result['stages'].append(dict(state=state,sensor_ids=sids,start_s=start_s,end_s=clock))
            current = state
        for x,y,dt in self.edge(current,self.start,mode):
            append(x,y,dt,'RETURN')
        # Geometry for every edge is cached after construction; full independent
        # verification is applied to the selected complete candidate below.
        late = [max(0.,s['end_s']-self.sensors[s['sensor']]['deadline_s']) for s in result['services']]
        energy = self.model.audit(result)
        result['energy_estimate'] = energy
        result['summary'] = dict(sensors=len(self.sensors),geometrically_scheduled=len(result['services']),
                                 stages=len(order),duration_s=clock,
                                 airborne_collection_complete_s=max(s['end_s'] for s in result['services']),
                                 deadline_misses=sum(v>1e-8 for v in late),total_lateness_s=sum(late),
                                 deadline_feasible=all(v<=1e-8 for v in late),
                                 loads={u:sum(s['uav']==u for s in result['services']) for u in IDS},
                                 min_world_altitude_m=min(s['start'][2] for s in result['segments']),
                                 max_world_altitude_m=max(s['start'][2] for s in result['segments']),
                                 scales_used=sorted({s['state'][3] for s in result['stages']}),
                                 retiming_mode=mode,team_energy_estimate_wh=energy['team_wh'])
        result['summary']['energy_budget_feasible'] = energy['budget_feasible']
        result['summary']['peak_horizontal_speed_m_s'] = max(
            s['profile'].get('peak_progress_rate_s_inv',0.)*math.hypot(y[0]-x[0],y[1]-x[1])
            for s in result['segments'] for x,y in zip(poses(s['start']),poses(s['end'])))
        return result

    @staticmethod
    def rank(result):
        s,e = result['summary'],result['energy_estimate']
        budget = e['mission_energy_limit_wh_per_uav']
        excess = 0. if budget is None else sum(max(0.,v['total_wh']-budget) for v in e['per_uav'].values())
        # Feasibility first; no weighted addition of metres, seconds and joules.
        return (s['deadline_misses'],round(s['total_lateness_s'],8),round(excess,8),
                round(e['team_wh'],8),round(s['duration_s'],8))

    def solve(self):
        n = len(self.stops)
        seeds = [tuple(range(n)),tuple(reversed(range(n)))]
        for first in range(n):
            order,remaining = [first],set(range(n))-{first}
            while remaining:
                last = self.stops[order[-1]][0]
                i = min(remaining,key=lambda i:sum(t for _,_,t in self.edge(last,self.stops[i][0],'fast')))
                order.append(i)
                remaining.remove(i)
            seeds.append(tuple(order))
        evaluated = {}
        def candidate(order):
            if order not in evaluated:
                values = [self.build(order,mode) for mode in ('fast','energy')]
                evaluated[order] = min(values,key=self.rank)
            return evaluated[order]
        order = min(seeds,key=lambda x:self.rank(candidate(x)))
        best = candidate(order)
        for _ in range(12):
            next_order,next_result = order,best
            neighbours = set()
            for i in range(n):
                for j in range(i+1,n):
                    neighbours.add(order[:i]+tuple(reversed(order[i:j+1]))+order[j+1:])
                for j in range(n):
                    item = list(order)
                    value = item.pop(i)
                    item.insert(j,value)
                    neighbours.add(tuple(item))
            for neighbour in sorted(neighbours):
                value = candidate(neighbour)
                if self.rank(value) < self.rank(next_result):
                    next_order,next_result = neighbour,value
            if next_order == order:
                break
            order,best = next_order,next_result
        best['search'] = dict(method='deterministic multistart + 2-opt/insertion; smooth-cruise segment duration search',
                              order=order,orders_evaluated=len(evaluated),mission_evaluations=self.evaluations,
                              optimality='No global optimum or infeasibility proof')
        return best


def bounded_stages(seed,terrain,p,config=None):
    stages = []
    sensors = {} if config is None else {s['id']:s for s in config['sensors']}
    pending = []
    for s in seed['stages']:
        x,y,z,scale = s['state']
        limit = 47-p['tracking_budget_m']
        x = max(-limit+5*scale,min(limit-10*scale,x))
        y = max(-limit+10*scale,min(limit-10*scale,y))
        state = state_at(terrain,x,y,scale,p)
        kept = []
        for sid in s['sensor_ids']:
            if config is None or eligible(terrain,state,sensors[sid],p):
                kept.append(sid)
            else:
                pending.append(sid)
        if kept:
            stages.append(dict(state=state,sensor_ids=kept))
    # Clipping a seed into the tracking-safe envelope can change range/LOS.
    # Re-cover every lost sensor at new feasible stops, rather than retaining
    # stale assignments or silently discarding sensors. No change when none lost.
    while pending:
        candidates = []
        for sid in pending:
            sx,sy,_ = sensors[sid]['world_enu_m']
            for scale in p['scales']:
                x=max(-limit+5*scale,min(limit-10*scale,sx))
                y=max(-limit+10*scale,min(limit-10*scale,sy))
                state=state_at(terrain,x,y,scale,p)
                if state[2]>p['max_world_altitude_m']:
                    continue
                covered=[k for k in pending if eligible(terrain,state,sensors[k],p)]
                if covered:
                    candidates.append((state,covered))
        if not candidates:
            raise ValueError('No feasible bounded repair candidate for '+','.join(pending))
        state,covered=min(candidates,key=lambda c:(-len(c[1]),c[0][2],tuple(c[0]),tuple(c[1])))
        stages.append(dict(state=state,sensor_ids=covered))
        pending=[sid for sid in pending if sid not in covered]
    return stages


def refine_altitudes(result,config,terrain,model,p):
    """One finite coordinate-search sweep; keep XY/scale/order and sensor groups.

    Try terrain-minimum and raised common altitudes. Every trial must retain a
    geometrically eligible collector for every assigned sensor. Accept only an
    improvement in the same deadline/budget/energy ranking as route search.
    """
    best = result
    info = dict(result['search'])
    before = copy.deepcopy(result['summary'])
    evaluations,accepted = 0,0
    edge_cache = {}
    sensors = {s['id']:s for s in config['sensors']}
    for i in range(len(best['stages'])):
        stages = [dict(state=tuple(s['state']),sensor_ids=s['sensor_ids']) for s in best['stages']]
        state = stages[i]['state']
        minimum = state_at(terrain,state[0],state[1],state[3],p)[2]
        heights = {minimum,minimum+2.,minimum+4.}
        if i:
            heights.add(stages[i-1]['state'][2])
        if i+1<len(stages):
            heights.add(stages[i+1]['state'][2])
        selected = best
        for z in sorted(heights):
            if z < minimum-1e-8 or z > p['max_world_altitude_m'] or abs(z-state[2])<1e-8:
                continue
            trial = (state[0],state[1],z,state[3])
            if any(not eligible(terrain,trial,sensors[sid],p) for sid in stages[i]['sensor_ids']):
                continue
            changed = [dict(s) for s in stages]
            changed[i]['state'] = trial
            search = Search(config,terrain,model,p,changed,result['initial_state'])
            search.edge_cache = edge_cache
            for mode in ('fast','energy'):
                value = search.build(tuple(range(len(changed))),mode)
                evaluations += 1
                if Search.rank(value) < Search.rank(selected):
                    selected = value
        if selected is not best:
            accepted += 1
            best = selected
    best['search'] = dict(info,altitude_refinement=dict(
        method='one coordinate sweep of feasible common heights, fixed XY/scale/order',
        mission_evaluations=evaluations,accepted_stage_changes=accepted,before_summary=before,
        max_world_altitude_m=p['max_world_altitude_m']))
    return best


def optimize(config,terrain,model,max_speed=3.,climb_speed=1.,acceleration=.8):
    limits = (max_speed,climb_speed,acceleration)
    if any(not math.isfinite(v) or v<=0 for v in limits):
        raise ValueError('Positive finite motion limits required')
    p = dict(POLICY,horizontal_speed_m_s=max_speed,vertical_speed_m_s=climb_speed,acceleration_m_s2=acceleration,
             max_world_altitude_m=18.)
    baseline = baseline_plan(config,terrain)
    baseline['energy_estimate'] = model.audit(baseline)
    fresh = baseline_plan(config,terrain,p)
    seeds = [('v3_stops',baseline),('new_greedy_stops',fresh)]
    results = []
    for label,seed in seeds:
        stages = bounded_stages(seed,terrain,p,config)
        search = Search(config,terrain,model,p,stages,state_at(terrain,-65.,0.,1.,p))
        result = search.solve()
        result['search']['seed'] = label
        control = min([search.build(tuple(range(len(stages))),mode) for mode in ('fast','energy')],key=Search.rank)
        control['summary']['checks'] = validate_mission(control,config,terrain)
        result = refine_altitudes(result,config,terrain,model,p)
        result['summary']['checks'] = validate_mission(result,config,terrain)
        result['control_before_search'] = control
        results.append(result)
    winner = min(results,key=Search.rank)
    winner['search']['alternatives'] = [dict(seed=r['search']['seed'],summary=r['summary']) for r in results]
    return baseline,winner


def write_case(path,result,config,terrain):
    path.mkdir(parents=True,exist_ok=False)
    (path/'plan.json').write_text(json.dumps(result,indent=2)+'\n')
    with (path/'trajectory.csv').open('w',newline='') as f:
        writer = csv.writer(f)
        writer.writerow(('time_s','uav','east_m','north_m','world_up_m','triangle_scale','kind'))
        writer.writerows(trajectory(result))
    render(result,config,terrain,path/'coverage-plan.png')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--count',type=int,choices=(30,50),required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-speed',type=float,default=3.)
    parser.add_argument('--climb-speed',type=float,default=1.)
    parser.add_argument('--acceleration',type=float,default=.8)
    parser.add_argument('--budget-wh',type=float,help='Optional mission-only energy limit per UAV; not battery capacity')
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    config_path = ROOT/f'config/regular_grid_n{args.count}.json'
    energy_path = ROOT/'config/energy_model.json'
    config = json.loads(config_path.read_text())
    energy = json.loads(energy_path.read_text())
    if args.budget_wh is not None:
        energy['mission_energy_limit_wh_per_uav'] = args.budget_wh
    terrain = TerrainGrid.from_csv(ROOT/'config/terrain.csv')
    baseline,winner = optimize(config,terrain,EnergyModel(energy),args.max_speed,args.climb_speed,args.acceleration)
    args.output.mkdir(parents=True,exist_ok=False)
    sources = [config_path,energy_path,ROOT/'config/terrain.csv',*Path(__file__).parent.glob('*.py')]
    control = winner.pop('control_before_search')
    metadata = dict(inputs_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                    effective_energy_model=energy,baseline=baseline['summary'],optimized=winner['summary'],
                    same_limits_before_route_search=control['summary'],
                    baseline_energy_estimate_wh=baseline['energy_estimate']['team_wh'],
                    optimized_energy_estimate_wh=winner['energy_estimate']['team_wh'],
                    caveat='Limits differ from v3: compare ablations before attributing changes to route search. Offline, uncalibrated, airborne only.')
    (args.output/'comparison.json').write_text(json.dumps(metadata,indent=2)+'\n')
    write_case(args.output/'baseline',baseline,config,terrain)
    write_case(args.output/'before_route_search',control,config,terrain)
    write_case(args.output/'optimized',winner,config,terrain)
    print(json.dumps(metadata,indent=2),flush=True)


if __name__ == '__main__':
    main()
