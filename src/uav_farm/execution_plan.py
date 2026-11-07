"""Frozen ARM-deadline-aware SITL bundles; no flight commands."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path

from baseline_planner import IDS, POLICY, poses, state_at
from mission_optimizer import Search, validate_mission
from terrain_links import TerrainGrid
from energy_model import EnergyModel
from arm_deadline import ArmSearch,assess

from project_paths import case_root, px4_root
ROOT = case_root()


def to_local(world,spawn,origin):
    return (origin[0]+world[1]-spawn[1],origin[1]+world[0]-spawn[0],origin[2]-(world[2]-spawn[2]))


def to_world(local,spawn,origin):
    return (spawn[0]+local[1]-origin[1],spawn[1]+local[0]-origin[0],spawn[2]-(local[2]-origin[2]))


def vector_to_ned(enu):
    return [float(enu[1]),float(enu[0]),float(-enu[2])]


def check_session(root=ROOT,vehicles=3):
    from farm_sim import alive
    if vehicles not in (1,3):raise ValueError('One or three SITL vehicles required')
    session=json.loads((root/'runtime/session.json').read_text())
    expected={'gazebo','agent',*IDS[:vehicles]}
    names={p['name'] for p in session['processes']}
    if not expected<=names or names-expected-{'gui'}:
        raise ValueError('Wrong launcher-owned SITL vehicle set')
    for p in session['processes']:
        if not alive(p):raise ValueError('Recorded process is no longer alive')
        if p['name'] in IDS:
            proc=Path('/proc')/str(p['pid'])
            if (proc/'exe').resolve()!=(px4_root()/'build/px4_sitl_default/bin/px4').resolve():
                raise ValueError('Unexpected PX4 executable')
            env=dict(s.split('=',1) for s in (proc/'environ').read_text().split('\0') if '=' in s)
            if env.get('PX4_SIM_MODEL')!='gz_x500' or env.get('PX4_GZ_WORLD')!='farm':
                raise ValueError('Not the farm x500 SITL')
            if env.get('ROS_DOMAIN_ID')!=os.environ.get('ROS_DOMAIN_ID') or env.get('GZ_PARTITION')!=os.environ.get('GZ_PARTITION'):
                raise ValueError('Controller and SITL transport isolation differ')
            if not (proc/'cwd').resolve().is_relative_to(root/'runtime'):
                raise ValueError('PX4 runtime is not owned by this experiment')
    return session


def make(vehicles=1,stages=1,speed=.8,root=ROOT):
    if vehicles!=3 or stages!=0 or not math.isfinite(speed) or abs(speed-1.2)>1e-9:
        raise ValueError('Frozen v12 adapter supports only complete 3-UAV 1.2m/s reference')
    config=json.loads((root/'config/farm.json').read_text())
    reference=json.loads((root/'config/reference_execution_v11.json').read_text())
    mission=json.loads((root/'config/optimized_connectors_v12.json').read_text())
    p=dict(mission['policy'])
    if config!=reference['config'] or p!=reference['mission']['policy']:
        raise ValueError('v12 changes frozen farm or reference motion/geometry policy')
    identity=lambda s:(s['sensor'],s['uav'],s['stage'],round(s['end_s']-s['start_s'],7),tuple(s['position']))
    if sorted(map(identity,mission['services']))!=sorted(map(identity,reference['mission']['services'])):
        raise ValueError('Service identity/assignment/duration changed')
    if [s['state'] for s in mission['stages']]!=[s['state'] for s in reference['mission']['stages']]:
        raise ValueError('Frozen service states/order changed')
    terrain=TerrainGrid.from_csv(root/'config/terrain.csv');checks=validate_mission(mission,config,terrain)
    budget=json.loads((root/'config/arm_budget.json').read_text())
    expected=assess(mission,config,budget)
    if any(mission['arm_deadline_prediction'].get(k)!=v for k,v in expected.items()):
        raise ValueError('ARM prediction differs from recomputation')
    if expected['deadline_misses']:raise ValueError('Selected plan misses ARM deadline')
    model=EnergyModel(json.loads((root/'config/energy_model.json').read_text()))
    if abs(model.audit(mission)['team_wh']-mission['energy_estimate']['team_wh'])>1e-7:
        raise ValueError('Energy proxy differs from recomputation')
    active=list(IDS[:vehicles])
    assignments={u:[s['sensor'] for s in mission['services'] if s['uav']==u] for u in active}
    policy=dict(p,pause_error_m=.8,resume_error_m=.3,max_pause_s=45.,max_step_s=.1,max_age_s=.5,
                service_position_error_m=.35,service_speed_m_s=.25,service_settle_s=.5,
                warning_min_m=14.,warning_max_m=26.)
    time=max((mission['initial_state'][2]-s['world_enu_m'][2])/.6 for s in config['uav_spawns'][:vehicles])
    initial=dict(zip(IDS,poses(mission['initial_state'])))
    climb={s['id']:[(0.,*s['world_enu_m']),(time,*initial[s['id']])] for s in config['uav_spawns'][:vehicles]}
    duration=mission['summary']['duration_s']
    routes={u:[(0.,*initial[u]),(duration,*initial[u])] for u in active}
    sources=['config/farm.json','config/terrain.csv','config/energy_model.json','config/optimized_connectors_v12.json','config/reference_execution_v11.json','config/arm_budget.json','scripts/cruise_profile.py','scripts/arm_deadline.py','scripts/execution_plan.py']
    return dict(schema='formation_execution_v12',ids=active,formation=True,geometric=True,
                config=dict(config,uav_spawns=config['uav_spawns'][:vehicles]),policy=policy,
                mission=mission,assignments=assignments,
                services={u:[s for s in mission['services'] if s['uav']==u] for u in active},
                flight_bounds_xy_m=config['flight_bounds_xy_m'],phases=dict(CLIMB=climb,MISSION=routes),
                safety_policy=dict(mission_abort_separation_m=12.,prediction_warning_m=14.,tracking_abort_m=2.,tracking_pause_m=.8),
                source_sha256={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in sources},
                preparation=dict(vehicles=vehicles,stages=stages,speed=speed),
                summary=dict(vehicles=vehicles,service_stages=len(mission['stages']),expected_sensors=sum(map(len,assignments.values())),
                             planned_mission_reference_s=duration,checks=checks,
                             predicted_arm_deadline=mission['arm_deadline_prediction'],
                             scope='Frozen full-team v12 connector verification; geometric service, not radio bytes.'))


def load(path,vehicles=1,root=ROOT):
    data=json.loads(Path(path).read_text())
    if data.get('schema')!='formation_execution_v12' or data['preparation']['vehicles']!=vehicles:
        raise ValueError('Bundle vehicle/schema mismatch')
    expected=make(root=root,**data['preparation'])
    if data!=json.loads(json.dumps(expected)):
        raise ValueError('Bundle differs from freshly validated source/limits')
    return data


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vehicles',type=int,choices=(1,3),default=1)
    parser.add_argument('--stages',type=int,default=1,help='0 = complete source route; default = first service stop')
    parser.add_argument('--speed',type=float,default=.8)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=make(args.vehicles,args.stages,args.speed)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as f:f.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result['summary'],indent=2))
