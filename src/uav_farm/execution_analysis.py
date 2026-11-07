"""Independent checks of executed telemetry and geometric service credit evidence."""
import csv
import json
import math
from pathlib import Path
from execution_plan import ROOT,make
from baseline_planner import closest_pair
from terrain_links import TerrainGrid,evaluate
from energy_model import EnergyModel


def analyze_flight(path,output=None):
    path=Path(path);summary=json.loads(path.read_text());plan=summary['plan'];policy=plan['policy'];ids=plan['ids']
    if plan!=json.loads(json.dumps(make(**plan['preparation']))):
        raise ValueError('Executed bundle differs from revalidated source')
    if summary['result']!='DONE' or not summary['landing_confirmed'] or any(s['arming_state']!=1 or s['landed'] is not True for s in summary['final_states'].values()):
        raise ValueError('Successful landing/disarm not confirmed')
    feedback=summary['formation_feedback']
    if feedback['completed_sensors']!=plan['summary']['expected_sensors']:
        raise ValueError('Incomplete geometric service ledger')
    sensors={s['id']:s for s in plan['config']['sensors']}
    terrain=TerrainGrid.from_csv(ROOT/'config/terrain.csv')
    groups={};by_elapsed={}
    with Path(summary['csv']).open() as f:
        for row in csv.DictReader(f):
            if row['stage']=='MISSION':
                groups.setdefault(int(row['master_sample_us']),{})[row['uav']]=row
                by_elapsed[round(float(row['mission_elapsed_sim_s'])*1e6),row['uav']]=row
    if not groups or any(set(g)!=set(ids) for g in groups.values()):raise ValueError('Incomplete fleet telemetry')
    scheduled={s['sensor']:s for u in ids for s in plan['services'][u]}
    credited={s:0. for s in feedback['ledger']}
    intervals={s:[] for s in credited}
    for item in feedback['credit_log']:
        if item['sensor'] not in credited or item['uav'] not in ids or not 0<item['credited_s']<=.1+1e-8:
            raise ValueError('Invalid service credit evidence')
        if abs(item['credited_s']-(item['reference_end_s']-item['reference_start_s']))>1e-7:
            raise ValueError('Service credit/reference interval mismatch')
        service=scheduled[item['sensor']]
        if item['uav']!=service['uav'] or item['reference_start_s']<service['start_s']-1e-7 or item['reference_end_s']>service['end_s']+1e-7:
            raise ValueError('Credit outside assigned service window')
        intervals[item['sensor']].append((item['reference_start_s'],item['reference_end_s']))
        row=by_elapsed.get((round(item['elapsed_sim_s']*1e6),item['uav']))
        if row is None or math.dist(item['position'],[float(row[k]) for k in ('world_est_e_m','world_est_n_m','world_est_u_m')])>1e-7:
            raise ValueError('Credit evidence does not match recorded telemetry')
        if not all(math.isfinite(x) for x in (*item['position'],*item['velocity'],*item['target'])):
            raise ValueError('Non-finite credit telemetry')
        if math.dist(item['position'],item['target'])>policy['service_position_error_m']+1e-7 or math.sqrt(sum(v*v for v in item['velocity']))>policy['service_speed_m_s']+1e-7:
            raise ValueError('Service was credited while not settled')
        if not evaluate(terrain,item['position'],sensors[item['sensor']]['world_enu_m'],
                max_distance=policy['collection_range_m'],minimum_uav_agl=policy['min_agl_m'])['geometry_eligible']:
            raise ValueError('Service credited without geometric link')
        credited[item['sensor']]+=item['credited_s']
    for s,v in feedback['ledger'].items():
        required=scheduled[s]['end_s']-scheduled[s]['start_s']
        if abs(v['required_s']-required)>1e-7 or not v['completed'] or abs(credited[s]-v['credited_s'])>1e-7 or credited[s]<required-1e-7:
            raise ValueError('Insufficient service evidence')
        ordered=sorted(intervals[s])
        if any(b[0]<a[1]-1e-7 for a,b in zip(ordered,ordered[1:])):raise ValueError('Repeated service credit interval')
    minimum,maximum,gap,error=math.inf,0.,math.inf,0.
    previous=None;maximum_speed=0.
    model=EnergyModel(json.loads((ROOT/'config/energy_model.json').read_text()))
    energy={u:0. for u in ids}
    for stamp,group in sorted(groups.items()):
        positions={u:tuple(float(group[u][k]) for k in ('world_est_e_m','world_est_n_m','world_est_u_m')) for u in ids}
        velocities={u:tuple(float(group[u][k]) for k in ('velocity_e_m_s','velocity_n_m_s','velocity_u_m_s')) for u in ids}
        for u in ids:
            q=positions[u];v=velocities[u]
            if not all(math.isfinite(x) for x in (*q,*v)):raise ValueError('Non-finite mission telemetry')
            if float(group[u]['rx_age_s'])>.5:raise ValueError('Stale recorded position')
            gap=min(gap,q[2]-terrain.height(*q[:2]));error=max(error,float(group[u]['track_error_m']))
            maximum_speed=max(maximum_speed,math.sqrt(sum(x*x for x in v)))
        for i,a in enumerate(ids):
            for b in ids[i+1:]:
                d=math.dist(positions[a],positions[b]);minimum=min(minimum,d);maximum=max(maximum,d)
                if previous:minimum=min(minimum,closest_pair(previous[1][a],positions[a],previous[1][b],positions[b]))
        if previous:
            dt=(stamp-previous[0])/1e6
            for u,v in velocities.items():
                old=previous[2][u]
                def power(v):return model.level_power(math.hypot(*v[:2]))+model.config['avionics_w']+model.weight*max(v[2],0.)/model.config['climb_efficiency']
                kinetic=max(0.,.5*model.config['mass_kg']*(sum(x*x for x in v)-sum(x*x for x in old)))/model.config['kinetic_efficiency']
                energy[u]+=.5*(power(old)+power(v))*dt+kinetic
        previous=(stamp,positions,velocities)
    if gap<policy['min_agl_m'] or error>policy['tracking_budget_m']:raise ValueError('Executed mission tracking/terrain bounds failed')
    if len(ids)==3 and (minimum<policy['min_distance_m'] or maximum>policy['max_distance_m']):raise ValueError('Executed formation separation failed')
    for sid,v in feedback['ledger'].items():energy[v['uav']]+=credited[sid]*model.config['radio_active_w']
    report=dict(status='passed',vehicles=len(ids),completed_geometric_services=len(credited),
                metrics=dict(min_interpolated_pair_m=minimum if math.isfinite(minimum) else None,max_sampled_pair_m=maximum if len(ids)==3 else None,
                             min_sampled_agl_m=gap,max_tracking_error_m=error,max_sampled_speed_3d_m_s=maximum_speed,
                             mission_samples=len(groups),service_credit_records=len(feedback['credit_log'])),
                deadline_misses_mission_clock=feedback['deadline_misses_mission_clock'],
                deadline_misses_since_arm=feedback['deadline_misses_since_arm'],
                mission_energy_estimate_wh_from_telemetry={u:v/3600 for u,v in energy.items()},
                scope='Spawn-anchored PX4 telemetry estimates, not ground truth. Geometric service time, not received data. Energy is an uncalibrated proxy; excludes climb/landing and is sensitive to velocity noise.')
    out=Path(output) if output is not None else path.with_name(path.stem+'_execution_checks.json')
    with out.open('x') as f:f.write(json.dumps(report,indent=2)+'\n')
    return report
