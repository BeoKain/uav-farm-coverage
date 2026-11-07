"""Postflight audit for a full guarded fleet route, using native Gazebo poses."""
from bisect import bisect_left
from collections import defaultdict
import csv
from itertools import combinations
import json
import math
from pathlib import Path

from fleet_plan import ROOT
from terrain_links import TerrainGrid,evaluate


def read(path):
    with path.open(newline='') as handle:
        return list(csv.DictReader(handle))


def analyze_flight(summary_path, logs):
    flight=json.loads(summary_path.read_text())
    p=flight['plan']
    ids=p['ids']
    mission=[r for r in read(Path(flight['csv'])) if r['stage']=='MISSION']
    epoch=int(mission[0]['master_sample_us'])/1e6-float(mission[0]['mission_elapsed_sim_s'])
    duration=float(mission[-1]['mission_elapsed_sim_s'])
    truth=defaultdict(dict)
    for row in read(logs/'gazebo_truth.csv'):
        stamp=float(row['stamp_s'])
        if epoch<=stamp<=epoch+duration:
            truth[stamp][row['uav']]=tuple(float(row[k]) for k in ('x','y','z'))
    stamps=sorted(truth)
    if not stamps or any(set(t)!=set(ids) for t in truth.values()):
        raise ValueError('Missing Gazebo fleet poses')
    terrain=TerrainGrid.from_csv(ROOT/'config/terrain.csv')
    distances=[math.dist(a,b) for t in truth.values() for a,b in combinations(t.values(),2)]
    agl=min(z-terrain.height(x,y) for t in truth.values() for x,y,z in t.values())
    peak_z=max(z for t in truth.values() for _,_,z in t.values())
    fb=flight['formation_feedback']
    sensors={s['id']:s for s in p['config']['sensors']}
    worst_range={}
    credit_geometry_ok=True
    for c in fb['credit_log']:
        t=epoch+c['elapsed_sim_s']
        idx=bisect_left(stamps,t)
        stamp=min(stamps[max(0,idx-1):idx+1],key=lambda v:abs(v-t))
        link=evaluate(terrain,truth[stamp][c['uav']],sensors[c['sensor']]['world_enu_m'],
                      p['policy']['collection_range_m'],p['policy']['min_agl_m'])
        credit_geometry_ok &= abs(t-stamp)<=.06 and link['geometry_eligible']
        worst_range[c['sensor']]=max(worst_range.get(c['sensor'],0.),link['distance_3d_m'])
    slack={name:sensors[name]['deadline_s']-e['completed_since_arm_s']
           for name,e in fb['ledger'].items() if e['completed']}
    previous=0.
    guarded=True
    for row in flight['guard_rows']:
        if row['reference_s']>previous+1e-8:
            guarded &= row['state']=='TRACK' and set(row['admitted_probe_uavs'])==set(ids)
        previous=row['reference_s']
    criteria=dict(route_completed=flight['result']=='DONE' and previous>=p['mission']['segments'][-1]['end_s']-1e-7,
                  all_served=fb['completed_sensors']==30 and len(worst_range)==30,
                  service_geometry=credit_geometry_ok and bool(worst_range),
                  arm_deadlines=len(slack)==30 and min(slack.values())>=0,
                  safe_spacing=min(distances)>=12 and max(distances)<=28,
                  terrain_clearance=agl>=p['policy']['min_agl_m'],
                  altitude_ceiling=peak_z<=p['flight_ceiling_m'],
                  guarded_progress=guarded and bool(flight['probe_events']),
                  landed=flight['landing_confirmed'] and all(v=={'arming_state':1,'landed':True}
                                                               for v in flight['final_states'].values()))
    logs_by_u={}
    for u in ids:
        data=json.loads((logs/f'{u}.json').read_text())
        logs_by_u[u]={x['stamp_s']:x for x in data['decisions'][u]}
    events=flight['guard_events']
    recoveries=[]
    for i,climb in enumerate(events):
        if climb['state']!='CLIMB':
            continue
        hold=next((e for e in reversed(events[:i]) if e['state']=='HOLD'),None)
        release=next((e for e in events[i+1:] if e['state']=='TRACK'),None)
        if not hold or not release or any(r['release']==release for r in recoveries):
            continue
        tick=next(r for r in flight['guard_rows'] if r['sim_s']==climb['sim_s'])
        evidence={}
        for u in climb['warning_uavs']:
            stamp=tick['observations'][u]['stamp_s']
            d=logs_by_u[u].get(stamp)
            if d and d['state']=='RISE' and 0<=epoch+climb['sim_s']-stamp<.45:
                evidence[u]=dict(stamp_s=stamp,nearest_rise_return_m=d['nearest_rise_return_m'])
        start=min(stamps,key=lambda t:abs(t-epoch-hold['sim_s']))
        end=min(stamps,key=lambda t:abs(t-epoch-release['sim_s']))
        frozen=all(abs(row['reference_s']-hold['reference_s'])<=1e-7 for row in flight['guard_rows']
                   if hold['sim_s']<=row['sim_s']<=release['sim_s'])
        recoveries.append(dict(hold=hold,climb=climb,release=release,evidence=evidence,
                               frozen=frozen,climb_gain_truth_m={u:truth[end][u][2]-truth[start][u][2]
                                                                 for u in ids}))
    climb_count=sum(e['state']=='CLIMB' for e in events)
    criteria.update(all_climbs_verified=len(recoveries)==climb_count and all(r['evidence'] and r['frozen']
                         and min(r['climb_gain_truth_m'].values())>=.2 for r in recoveries),
                    no_credit_during_recovery=not any(r['hold']['sim_s']<c['elapsed_sim_s']<r['release']['sim_s']
                         for r in recoveries for c in fb['credit_log']))
    result=dict(status='passed' if all(criteria.values()) else 'failed',criteria=criteria,
                minimum_agl_m=agl,maximum_world_z_m=peak_z,recoveries=recoveries,separation_range_m=[min(distances),max(distances)],
                slack_since_arm_s=slack,worst_range_m=worst_range,probe_events=flight['probe_events'])
    summary_path.with_name(summary_path.stem+'_fleet_checks.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
