"""Development-calibrated deadline budget and finite coverage/order search. No flight IO."""
import argparse
import copy
import hashlib
import itertools
import json
import math
from pathlib import Path

from baseline_planner import ROOT, IDS, eligible, poses, state_at
from energy_model import EnergyModel
from mission_optimizer import Search,validate_mission,write_case
from terrain_links import TerrainGrid


def validate_budget(budget):
    if budget.get('schema')!='arm_deadline_budget_v1':raise ValueError('Unknown deadline budget')
    for k in ('launch_budget_s','per_service_stop_reserve_s','global_reserve_s'):
        if not math.isfinite(budget[k]) or budget[k]<0:raise ValueError('Invalid deadline reserve')


def assess(plan,config,budget):
    validate_budget(budget)
    deadlines={s['id']:s['deadline_s'] for s in config['sensors']}
    entries=[]
    for service in plan['services']:
        stops=service['stage']+1
        predicted=(budget['launch_budget_s']+service['end_s']+
                   stops*budget['per_service_stop_reserve_s']+budget['global_reserve_s'])
        entries.append(dict(sensor=service['sensor'],stage=service['stage'],reference_end_s=service['end_s'],
                            predicted_since_arm_s=predicted,deadline_s=deadlines[service['sensor']],
                            slack_s=deadlines[service['sensor']]-predicted))
    return dict(clock='since first ARM command',services=entries,
                deadline_misses=sum(e['slack_s'] < -1e-8 for e in entries),
                total_lateness_s=sum(max(0.,-e['slack_s']) for e in entries),
                last_service_since_arm_s=max(e['predicted_since_arm_s'] for e in entries),
                minimum_slack_s=min(e['slack_s'] for e in entries),
                budget=copy.deepcopy(budget),
                caveat='Conservative design reserve based on development flights; not a statistical or worst-case guarantee.')


class ArmSearch(Search):
    def __init__(self,*args,budget,**kwargs):
        super().__init__(*args,**kwargs);validate_budget(budget);self.budget=budget

    def build(self,order,mode='energy'):
        result=super().build(order,mode)
        result['arm_deadline_prediction']=assess(result,self.config,self.budget)
        reserve=self.budget['per_service_stop_reserve_s']*len(result['stages'])+self.budget['global_reserve_s']
        extra=3*(self.model.hover_w+self.model.config['avionics_w'])*reserve/3600
        result['arm_deadline_prediction'].update(wait_reserve_energy_wh=extra,
                    budgeted_mission_energy_proxy_wh=result['energy_estimate']['team_wh']+extra)
        return result

    @staticmethod
    def rank(result):
        prediction=result['arm_deadline_prediction']
        # Keep mission-only metrics untouched; ARM feasibility is the new priority.
        e=result['energy_estimate'];limit=e['mission_energy_limit_wh_per_uav']
        extra=prediction['wait_reserve_energy_wh']/3
        excess=0. if limit is None else sum(max(0.,v['total_wh']+extra-limit) for v in e['per_uav'].values())
        return (prediction['deadline_misses'],round(prediction['total_lateness_s'],8),round(excess,8),
                round(prediction['budgeted_mission_energy_proxy_wh'],8),round(result['summary']['duration_s'],8))


def calibrate(summaries):
    launches=[];increments=[];sources=[]
    for path in summaries:
        path=Path(path);data=json.loads(path.read_text())
        if data['result']!='DONE' or not data['landing_confirmed'] or data.get('disturbance_hold_uav2',False):
            raise ValueError('Calibration requires successful normal development flights')
        feedback=data['formation_feedback'];ledger=feedback['ledger'];mission=data['plan']['mission']
        launch=max(v['completed_since_arm_s']-v['completed_mission_sim_s'] for v in ledger.values())
        launches.append(launch);previous_drift=0.;stage_observations=[]
        for i,stage in enumerate(mission['stages']):
            services=[s for s in mission['services'] if s['stage']==i]
            drift=max(ledger[s['sensor']]['completed_mission_sim_s']-s['end_s'] for s in services)
            increase=max(0.,drift-previous_drift);increments.append(increase);previous_drift=drift
            stage_observations.append(dict(stage=i,cumulative_overhead_s=drift,increment_s=increase))
        sources.append(dict(path=str(path.resolve()),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                            launch_to_mission_s=launch,stage_observations=stage_observations))
    if not sources:raise ValueError('At least one calibration flight required')
    # Fixed, explicit design margins above development observations. Freeze before search.
    return dict(schema='arm_deadline_budget_v1',launch_budget_s=math.ceil(max(launches)+2.),
                per_service_stop_reserve_s=math.ceil((max(increments)+.25)*4)/4,
                global_reserve_s=3.,observed_max_launch_s=max(launches),observed_max_stage_increment_s=max(increments),
                derivation='ceil(max launch + 2 s); ceil_to_0.25(max incremental service drift + 0.25 s); global 3 s',
                calibration=sources,scope='Development calibration, including seed404; future replay is same-case verification, not unseen validation.')


def assign_subset(config,terrain,states,indices,p):
    sensors={s['id']:s for s in config['sensors']}
    options={sid:[(i,u) for i in indices for u in eligible(terrain,states[i],s,p)] for sid,s in sensors.items()}
    if any(not v for v in options.values()):return None
    queues={i:[0.,0.,0.] for i in indices};groups={i:[] for i in indices}
    # Allocate scarce geometry first; minimize added serial service across selected stops.
    for sid in sorted(sensors,key=lambda k:(len(options[k]),sensors[k]['deadline_s'],k)):
        dt=8*sensors[sid]['data_bytes']/p['nominal_link_bps']+p['service_guard_s']
        def rank(pair):
            i,u=pair;new=max(max(queues[i]),queues[i][u]+dt)
            return (new-max(queues[i]),new,math.dist(poses(states[i])[u],sensors[sid]['world_enu_m']),i,u)
        i,u=min(options[sid],key=rank);queues[i][u]+=dt;groups[i].append(sid)
    return [dict(state=states[i],sensor_ids=sorted(groups[i])) for i in indices if groups[i]]


def refine(result,config,terrain,model,budget):
    best=result;info=copy.deepcopy(result['search']);accepted=0;cache={}
    sensors={s['id']:s for s in config['sensors']};p=result['policy']
    for i in range(len(best['stages'])):
        stages=[dict(state=tuple(s['state']),sensor_ids=s['sensor_ids']) for s in best['stages']]
        x,y,z,scale=stages[i]['state'];low=state_at(terrain,x,y,scale,p)[2]
        heights={low,low+2,low+4,z}
        if i:heights.add(stages[i-1]['state'][2])
        if i+1<len(stages):heights.add(stages[i+1]['state'][2])
        selected=best
        for height in sorted(heights):
            if height<low-1e-8 or height>p['max_world_altitude_m'] or abs(height-z)<1e-8:continue
            state=(x,y,height,scale)
            if any(not eligible(terrain,state,sensors[sid],p) for sid in stages[i]['sensor_ids']):continue
            changed=copy.deepcopy(stages);changed[i]['state']=state
            search=ArmSearch(config,terrain,model,p,changed,result['initial_state'],budget=budget);search.edge_cache=cache
            for mode in ('fast','energy'):
                candidate=search.build(tuple(range(len(changed))),mode)
                if ArmSearch.rank(candidate)<ArmSearch.rank(selected):selected=candidate
        if selected is not best:best=selected;accepted+=1
    best['search']=dict(info,arm_altitude_refinement=dict(accepted_stage_changes=accepted,
                         method='one coordinate sweep, same XY/scale and ARM deadline ranking'))
    return best


def optimize(config,terrain,model,source,budget,output):
    output.mkdir(parents=True,exist_ok=False)
    validate_mission(source,config,terrain)
    frozen=copy.deepcopy(source);frozen['arm_deadline_prediction']=assess(frozen,config,budget)
    write_case(output/'reference_v6',frozen,config,terrain)
    p=source['policy'];start=source['initial_state'];states=[tuple(s['state']) for s in source['stages']]
    fixed=ArmSearch(config,terrain,model,p,source['stages'],start,budget=budget).solve()
    fixed=refine(fixed,config,terrain,model,budget)
    fixed['summary']['checks']=validate_mission(fixed,config,terrain)
    write_case(output/'arm_fixed_groups',fixed,config,terrain)
    if len(states)>10:raise ValueError('Finite subset enumeration limited to 10 existing stops')
    best=fixed;cache={};evaluated=[]
    for n in range(1,len(states)+1):
        for subset in itertools.combinations(range(len(states)),n):
            stages=assign_subset(config,terrain,states,subset,p)
            if stages is None:continue
            search=ArmSearch(config,terrain,model,p,stages,start,budget=budget);search.edge_cache=cache
            candidate=search.solve()
            evaluated.append(dict(original_stop_indices=subset,stops=len(stages),summary=candidate['summary'],
                                  arm=candidate['arm_deadline_prediction']))
            if ArmSearch.rank(candidate)<ArmSearch.rank(best):best=candidate
    best=refine(best,config,terrain,model,budget)
    best['summary']['checks']=validate_mission(best,config,terrain)
    best['search']['coverage_selection']=dict(method='enumerate covering subsets of existing service states; scarce-first reassignment',
                                             feasible_subsets_evaluated=len(evaluated),original_stops=len(states),
                                             optimality='Heuristic assignment/order/height within a finite candidate set; no global optimality claim')
    write_case(output/'optimized',best,config,terrain)
    comparison=dict(reference=dict(summary=frozen['summary'],arm=frozen['arm_deadline_prediction']),
                    arm_fixed_groups=dict(summary=fixed['summary'],arm=fixed['arm_deadline_prediction']),
                    optimized=dict(summary=best['summary'],arm=best['arm_deadline_prediction']),
                    subsets=evaluated,budget=budget)
    (output/'comparison.json').write_text(json.dumps(comparison,indent=2)+'\n')
    return best


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    config=json.loads((ROOT/'config/farm.json').read_text());terrain=TerrainGrid.from_csv(ROOT/'config/terrain.csv')
    model=EnergyModel(json.loads((ROOT/'config/energy_model.json').read_text()))
    source=json.loads((ROOT/'config/reference_n30_v6.json').read_text());budget=json.loads((ROOT/'config/arm_budget.json').read_text())
    best=optimize(config,terrain,model,source,budget,args.output)
    with (ROOT/'config/optimized_n30_v7.json').open('x') as f:f.write(json.dumps(best,indent=2)+'\n')
    print(json.dumps(dict(summary=best['summary'],arm=best['arm_deadline_prediction']),indent=2))
