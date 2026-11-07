"""Offline cooperative sensor-service planner. No ROS, flight commands or radio claims.

One common center, common altitude and bounded triangle scale for all three UAVs.
Terrain is the original triangulated Gazebo mesh. Communication is geometric only.
"""
import argparse
import csv
import hashlib
import itertools
import json
import math
import os
from pathlib import Path

from terrain_links import TerrainGrid, evaluate

from project_paths import case_root, px4_root
ROOT = case_root()
IDS = ('UAV1', 'UAV2', 'UAV3')
OFFSETS = ((-5., -10.), (10., 0.), (-5., 10.))
PAIRS = tuple(itertools.combinations(range(3), 2))
POLICY = dict(min_distance_m=12., max_distance_m=28., max_radius_m=18.,
              tracking_budget_m=2., min_agl_m=5., planned_clearance_m=8.,
              collection_range_m=28., service_range_reserve_m=2.,
              horizontal_speed_m_s=.8, vertical_speed_m_s=.6,
              acceleration_m_s2=.4, nominal_link_bps=160000.,
              service_guard_s=.5, scales=[.9, 1., 1.1])


def poses(state):
    x, y, z, scale = state
    return [(x + scale*dx, y + scale*dy, z) for dx, dy in OFFSETS]


def terrain_ceiling(terrain, points, reserve):
    """Conservative upper bound over a rectangle enclosing all swept XY tubes.

    Expand by one mesh cell as well as the tracking reserve; every intersected
    triangle's vertices are included. Affine interpolation cannot exceed them.
    """
    xmin = min(p[0] for p in points)-reserve
    xmax = max(p[0] for p in points)+reserve
    ymin = min(p[1] for p in points)-reserve
    ymax = max(p[1] for p in points)+reserve
    if xmin < terrain.xs[0] or xmax > terrain.xs[-1] or ymin < terrain.ys[0] or ymax > terrain.ys[-1]:
        raise ValueError('Tracking tube outside known terrain')
    xs = [x for x in terrain.xs if xmin-terrain.dx <= x <= xmax+terrain.dx]
    ys = [y for y in terrain.ys if ymin-terrain.dy <= y <= ymax+terrain.dy]
    return max(terrain.heights[x, y] for x in xs for y in ys)


def state_at(terrain, x, y, scale, p, fixed_altitude=None):
    state = (x, y, 0., scale)
    required = terrain_ceiling(terrain, poses(state), p['tracking_budget_m']) + p['planned_clearance_m']
    z = required if fixed_altitude is None else fixed_altitude
    if z+1e-9 < required:
        raise ValueError('Fixed altitude violates conservative terrain bound')
    return (x, y, z, scale)


def duration(a, b, p):
    """Rest-to-rest quintic: max s'=1.875, max |s''|=10/sqrt(3)."""
    distances = [math.dist(x, y) for x, y in zip(poses(a), poses(b))]
    horizontal = max(math.hypot(x[0]-y[0], x[1]-y[1]) for x, y in zip(poses(a), poses(b)))
    vertical = abs(a[2]-b[2])
    return max(1.875*horizontal/p['horizontal_speed_m_s'],
               1.875*vertical/p['vertical_speed_m_s'],
               math.sqrt((10/math.sqrt(3))*max(distances)/p['acceleration_m_s2']))


def connect(a, b, terrain, p):
    ceiling = terrain_ceiling(terrain, poses(a)+poses(b), p['tracking_budget_m'])
    z = max(a[2], b[2], ceiling+p['planned_clearance_m'])
    waypoints = [a, (a[0], a[1], z, a[3]), (b[0], b[1], z, b[3]), b]
    return [(x, y, duration(x, y, p)) for x, y in zip(waypoints, waypoints[1:]) if math.dist(x, y)>1e-9]


def closest_pair(a0, a1, b0, b1):
    r = [x-y for x, y in zip(a0, b0)]
    v = [(a1[k]-a0[k])-(b1[k]-b0[k]) for k in range(3)]
    vv = sum(x*x for x in v)
    s = max(0., min(1., -sum(x*y for x, y in zip(r, v))/vv)) if vv else 0.
    return math.sqrt(sum((r[k]+s*v[k])**2 for k in range(3)))


def validate_segments(segments, terrain, p):
    minimum, maximum, clearance, radius = math.inf, 0., math.inf, 0.
    for seg in segments:
        a, b = seg['start'], seg['end']
        pa, pb = poses(a), poses(b)
        for i, j in PAIRS:
            minimum = min(minimum, closest_pair(pa[i], pb[i], pa[j], pb[j]))
            maximum = max(maximum, math.dist(pa[i], pa[j]), math.dist(pb[i], pb[j]))
        for state in (a, b):
            radius = max(radius, *(math.dist(q, state[:3]) for q in poses(state)))
        for x, y in zip(pa, pb):
            clearance = min(clearance, *(q['vertical_clearance_m'] for q in terrain.line_profile(x, y)))
        robust_gap = min(a[2], b[2])-terrain_ceiling(terrain, pa+pb, p['tracking_budget_m'])-p['tracking_budget_m']
        if robust_gap < p['min_agl_m']-1e-8:
            raise ValueError('Tracking tube terrain clearance failed')
        d = seg['duration_s']
        if d <= 0 or not math.isfinite(d):
            raise ValueError('Invalid segment duration')
        if seg['kind'] != 'SERVICE' and d < duration(a, b, p)-1e-8:
            raise ValueError('Speed/acceleration bound failed')
    budget = p['tracking_budget_m']
    if minimum-2*budget < p['min_distance_m']-1e-8 or maximum+2*budget > p['max_distance_m']+1e-8:
        raise ValueError('Formation pair margin failed')
    if radius+budget > p['max_radius_m']+1e-8:
        raise ValueError('Formation radius margin failed')
    return dict(reference_min_pair_m=minimum, reference_max_pair_m=maximum,
                reference_min_agl_m=clearance, reference_max_radius_m=radius,
                assumed_tracking_budget_m=budget,
                note='Continuous reference geometry, conditional tracking margins; not a flight guarantee')


def eligible(terrain, state, sensor, p):
    return [i for i, q in enumerate(poses(state)) if evaluate(
        terrain, q, sensor['world_enu_m'],
        max_distance=p['collection_range_m']-p['service_range_reserve_m'],
        minimum_uav_agl=p['min_agl_m'])['geometry_eligible']]


def improve_order(states, start, terrain, p):
    """Deterministic 2-opt on full-team transit time, preserving every service stop."""
    nodes = [start]+list(states)
    costs = [[0.]*len(nodes) for _ in nodes]
    for i, j in itertools.combinations(range(len(nodes)), 2):
        costs[i][j] = costs[j][i] = sum(leg[2] for leg in connect(nodes[i], nodes[j], terrain, p))
    def total(order):
        route = [0]+[i+1 for i in order]+[0]
        return sum(costs[a][b] for a, b in zip(route, route[1:]))
    order = list(range(len(states)))
    before = total(order)
    while True:
        best, best_cost = order, total(order)
        for i in range(len(order)):
            for j in range(i+1, len(order)):
                candidate = order[:i]+list(reversed(order[i:j+1]))+order[j+1:]
                cost = total(candidate)
                if cost < best_cost-1e-8:
                    best, best_cost = candidate, cost
        if best == order:
            return order, before, best_cost
        order = best


def plan(config, terrain, p=None, fixed_altitude=None, fixed_scale=None):
    p = dict(POLICY if p is None else p)
    for key in ('horizontal_speed_m_s', 'vertical_speed_m_s', 'acceleration_m_s2', 'nominal_link_bps'):
        if not math.isfinite(p[key]) or p[key] <= 0:
            raise ValueError('Positive finite limits required')
    if p['planned_clearance_m'] < p['min_agl_m']+p['tracking_budget_m']:
        raise ValueError('Insufficient vertical tracking reserve')
    sensors = config['sensors']
    if not sensors or len({s['id'] for s in sensors}) != len(sensors):
        raise ValueError('Nonempty unique sensor IDs required')
    scales = [fixed_scale] if fixed_scale is not None else p['scales']
    if not scales or any(not math.isfinite(s) or not .9 <= s <= 1.1 for s in scales):
        raise ValueError('This prototype supports triangle scale only in [0.9, 1.1]')
    candidates, seen = [], set()
    for sensor in sensors:
        for scale in scales:
            # Keep physical references two metres inside farm bounds.
            x = max(-48+5*scale, min(48-10*scale, sensor['world_enu_m'][0]))
            y = max(-48+10*scale, min(48-10*scale, sensor['world_enu_m'][1]))
            if (x, y, scale) in seen:
                continue
            seen.add((x, y, scale))
            state = state_at(terrain, x, y, scale, p, fixed_altitude)
            options = {s['id']: eligible(terrain, state, s, p) for s in sensors}
            candidates.append((state, {k: v for k, v in options.items() if v}))
    start = state_at(terrain, -65., 0., 1., p, fixed_altitude)
    if len(config['uav_spawns']) != 3 or any(
            spawn['id'] != name or math.dist(spawn['world_enu_m'][:2], q[:2]) > 1e-8
            for spawn, name, q in zip(config['uav_spawns'], IDS, poses(start))):
        raise ValueError('External triangle launch pads mismatch')
    current = start
    segments, stages, services = [], [], []
    remaining = {s['id']: s for s in sensors}
    loads = [0, 0, 0]
    clock = 0.

    def append(a, b, dt, kind):
        nonlocal clock
        segments.append(dict(start=a, end=b, duration_s=dt, start_s=clock, end_s=clock+dt, kind=kind))
        clock += dt

    while remaining:
        best = None
        for index, (state, options) in enumerate(candidates):
            new = sorted(set(options) & set(remaining))
            if not new:
                continue
            legs = connect(current, state, terrain, p)
            travel = sum(leg[2] for leg in legs)
            # Rank with sequential service upper bound; actual service is parallel by UAV.
            service_upper = sum(8*remaining[s]['data_bytes']/p['nominal_link_bps']+p['service_guard_s'] for s in new)
            score = len(new)/(travel+service_upper)
            rank = (score, len(new), -travel, -index)
            if best is None or rank > best[0]:
                best = (rank, state, options, new, legs)
        if best is None:
            raise ValueError('No feasible candidate serves remaining sensors: '+','.join(remaining))
        _, state, options, new, legs = best
        for a, b, dt in legs:
            append(a, b, dt, 'TRANSIT')
        stage_start = clock
        queues = [0., 0., 0.]
        for sid in new:
            sensor = remaining.pop(sid)
            i = min(options[sid], key=lambda i: (queues[i], loads[i], math.dist(poses(state)[i], sensor['world_enu_m']), i))
            service_time = 8*sensor['data_bytes']/p['nominal_link_bps']+p['service_guard_s']
            services.append(dict(sensor=sid, uav=IDS[i], stage=len(stages),
                                 start_s=stage_start+queues[i], end_s=stage_start+queues[i]+service_time,
                                 assumed_bytes=sensor['data_bytes'], position=poses(state)[i]))
            queues[i] += service_time
            loads[i] += 1
        append(state, state, max(queues), 'SERVICE')
        stages.append(dict(state=state, sensor_ids=new, start_s=stage_start, end_s=clock))
        current = state
    for a, b, dt in connect(current, start, terrain, p):
        append(a, b, dt, 'RETURN')
    order, before, after = improve_order([s['state'] for s in stages], start, terrain, p)
    old_stages, old_services = stages, services
    stages, services, segments, clock = [], [], [], 0.
    current = start
    for old_index in order:
        old = old_stages[old_index]
        state = old['state']
        for a, b, dt in connect(current, state, terrain, p):
            append(a, b, dt, 'TRANSIT')
        stage_start = clock
        for s in old_services:
            if s['stage'] == old_index:
                services.append(dict(s, stage=len(stages), start_s=stage_start+s['start_s']-old['start_s'],
                                     end_s=stage_start+s['end_s']-old['start_s']))
        append(state, state, old['end_s']-old['start_s'], 'SERVICE')
        stages.append(dict(old, start_s=stage_start, end_s=clock))
        current = state
    for a, b, dt in connect(current, start, terrain, p):
        append(a, b, dt, 'RETURN')
    checks = validate_segments(segments, terrain, p)
    result = dict(schema='cooperative_sensor_coverage_offline_v3', executable_in_sitl=False,
                  scope='Airborne mission from above external pads, returning above pads; excludes takeoff/landing. Geometric service only.',
                  policy=p, initial_state=start, stages=stages, segments=segments, services=services,
                  summary=dict(sensors=len(sensors), geometrically_scheduled=len(services), stages=len(stages),
                               duration_s=clock, loads=dict(zip(IDS, loads)),
                               min_world_altitude_m=min(s['start'][2] for s in segments),
                               max_world_altitude_m=max(s['start'][2] for s in segments),
                               scales_used=sorted({s['state'][3] for s in stages}), checks=checks,
                               transit_before_2opt_s=before, transit_after_2opt_s=after,
                               deadline_policy='Input 300 s deadlines are reported, not enforced in this feasibility baseline',
                               deadline_misses=sum(s['end_s'] > next(x['deadline_s'] for x in sensors if x['id']==s['sensor']) for s in services)))
    result['summary']['deadline_feasible'] = result['summary']['deadline_misses'] == 0
    verify_services(result, config, terrain)
    return result


def verify_services(result, config, terrain):
    sensors = {s['id']: s for s in config['sensors']}
    services = result['services']
    if len(services) != len(sensors) or {s['sensor'] for s in services} != set(sensors):
        raise ValueError('Missing or duplicate service')
    occupied = {u: [] for u in IDS}
    for s in services:
        stage = result['stages'][s['stage']]
        q = poses(stage['state'])[IDS.index(s['uav'])]
        if math.dist(q, s['position']) > 1e-8:
            raise ValueError('Service pose differs from team state')
        if IDS.index(s['uav']) not in eligible(terrain, stage['state'], sensors[s['sensor']], result['policy']):
            raise ValueError('Service geometry failed')
        needed = 8*sensors[s['sensor']]['data_bytes']/result['policy']['nominal_link_bps']+result['policy']['service_guard_s']
        if s['start_s'] < stage['start_s']-1e-8 or s['end_s'] > stage['end_s']+1e-8 or s['end_s']-s['start_s'] < needed-1e-8:
            raise ValueError('Insufficient stationary service time')
        for a, b in occupied[s['uav']]:
            if min(b, s['end_s']) > max(a, s['start_s'])+1e-8:
                raise ValueError('Overlapping UAV service')
        occupied[s['uav']].append((s['start_s'], s['end_s']))


def trajectory(result, sample_s=.5):
    rows = []
    for seg in result['segments']:
        count = max(1, math.ceil(seg['duration_s']/sample_s))
        for k in range(count):
            u = k/count
            from cruise_profile import segment_kinematics
            s = segment_kinematics(seg,u*seg['duration_s'])[0]
            state = [a+s*(b-a) for a, b in zip(seg['start'], seg['end'])]
            for name, q in zip(IDS, poses(state)):
                rows.append((seg['start_s']+u*seg['duration_s'], name, *q, state[3], seg['kind']))
    for name, q in zip(IDS, poses(result['segments'][-1]['end'])):
        rows.append((result['summary']['duration_s'], name, *q, 1., 'END'))
    return rows


def render(result, config, terrain, path):
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/coverage-v3-mpl')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows = trajectory(result)
    fig, axes = plt.subplots(2, 2, figsize=(13, 10))
    xy, altitude, shape, speed = axes.flat
    xs = [x for x in terrain.xs if -50 <= x <= 50]
    ys = [y for y in terrain.ys if -50 <= y <= 50]
    xy.contourf(xs, ys, [[terrain.height(x, y) for x in xs] for y in ys], levels=[-.01, .1, 2, 4, 6, 8.1], cmap='Greens', alpha=.5)
    for i, name in enumerate(IDS):
        r = [r for r in rows if r[1] == name]
        xy.plot([r[2] for r in r], [r[3] for r in r], label=name, alpha=.8)
        assigned = [sensors for sensors in config['sensors'] if any(s['sensor']==sensors['id'] and s['uav']==name for s in result['services'])]
        xy.scatter([s['world_enu_m'][0] for s in assigned], [s['world_enu_m'][1] for s in assigned], marker='x', color=f'C{i}')
        altitude.plot([r[0] for r in r], [r[4] for r in r], label=name, alpha=.6)
        ts, vs = [], []
        for seg in result['segments']:
            a, b = poses(seg['start'])[i], poses(seg['end'])[i]
            distance = math.hypot(b[0]-a[0], b[1]-a[1])
            for k in range(21):
                u = k/20
                ts.append(seg['start_s']+u*seg['duration_s'])
                from cruise_profile import segment_kinematics
                vs.append(distance*segment_kinematics(seg,u*seg['duration_s'])[1])
        speed.plot(ts, vs, label=name, alpha=.6)
    for i, stage in enumerate(result['stages']):
        x, y, z, scale = stage['state']
        xy.text(x, y, str(i+1), fontsize=8)
        shape.plot([stage['start_s'], stage['end_s']], [scale, scale], 'k.-')
    xy.set(xlabel='East (m)', ylabel='North (m)', title='Joint paths; crosses = assigned ground sensors', aspect='equal')
    xy.legend()
    altitude.set(xlabel='Mission time (s)', ylabel='World altitude (m)', title='Common altitude; terrain-aware climbs')
    shape.set(xlabel='Service stage time (s)', ylabel='Triangle scale', title='Selected formation scale at service stops')
    speed.set(xlabel='Mission time (s)', ylabel='Horizontal speed (m/s)', title='Rest-to-rest smooth reference speed')
    fig.suptitle(f"OFFLINE geometry only — {len(config['sensors'])} sensors; no SITL / radio / battery validation")
    fig.tight_layout()
    fig.savefig(path, dpi=145)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--count', type=int, choices=(30, 50), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--speed', type=float, default=.8, help='Horizontal speed limit, not constant speed')
    parser.add_argument('--fixed-altitude', type=float)
    parser.add_argument('--fixed-scale', type=float)
    args = parser.parse_args()
    config_path = ROOT/f'config/regular_grid_n{args.count}.json'
    config = json.loads(config_path.read_text())
    terrain = TerrainGrid.from_csv(ROOT/'config/terrain.csv')
    p = dict(POLICY, horizontal_speed_m_s=args.speed)
    result = plan(config, terrain, p, args.fixed_altitude, args.fixed_scale)
    result['inputs_sha256'] = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                               for path in (config_path, ROOT/'config/terrain.csv')}
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'plan.json').write_text(json.dumps(result, indent=2)+'\n')
    with (args.output/'trajectory.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(('time_s', 'uav', 'east_m', 'north_m', 'world_up_m', 'triangle_scale', 'kind'))
        writer.writerows(trajectory(result))
    render(result, config, terrain, args.output/'coverage-plan.png')
    print(json.dumps(result['summary'], indent=2))


if __name__ == '__main__':
    main()
