"""Static telemetry plot and sampled geometric link footprint; not a camera image."""
import argparse
import csv
import itertools
import json
import math
import os
from pathlib import Path

from execution_plan import ROOT
from terrain_links import TerrainGrid,evaluate


def render(summary_path,output):
    data=json.loads(Path(summary_path).read_text());plan=data['plan'];ids=plan['ids']
    rows=[]
    with Path(data['csv']).open() as f:
        rows=[r for r in csv.DictReader(f) if r['stage']=='MISSION']
    if not rows:raise ValueError('No mission telemetry')
    output.mkdir(parents=True,exist_ok=False)
    os.environ.setdefault('MPLCONFIGDIR','/tmp/formation-v5-mpl')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    terrain=TerrainGrid.from_csv(ROOT/'config/terrain.csv')
    fig,axes=plt.subplots(2,2,figsize=(13,10))
    xy,agl,progress,detail=axes.flat
    sensors=plan['config']['sensors'];feedback=data['formation_feedback']
    xy.scatter([s['world_enu_m'][0] for s in sensors],[s['world_enu_m'][1] for s in sensors],color='.65',s=15,label='Other sensors')
    sample=np.linspace(-50,50,41)
    for i,u in enumerate(ids):
        r=[r for r in rows if r['uav']==u]
        ts=[float(r['mission_elapsed_sim_s']) for r in r]
        xyz=[tuple(float(r[k]) for k in ('world_est_e_m','world_est_n_m','world_est_u_m')) for r in r]
        xy.plot([q[0] for q in xyz],[q[1] for q in xyz],color=f'C{i}',label=u)
        xy.plot([float(r['command_e_m']) for r in r],[float(r['command_n_m']) for r in r],color=f'C{i}',linestyle='--',alpha=.5)
        agl.plot(ts,[q[2]-terrain.height(*q[:2]) for q in xyz],color=f'C{i}',label=u)
        assigned=[s for s in sensors if feedback['ledger'].get(s['id'],{}).get('completed') and feedback['ledger'][s['id']]['uav']==u]
        xy.scatter([s['world_enu_m'][0] for s in assigned],[s['world_enu_m'][1] for s in assigned],color=f'C{i}',marker='x',s=45)
        first=next((item for item in feedback['credit_log'] if item['uav']==u),None)
        if first:
            mask=np.zeros((len(sample),len(sample)))
            for j,y in enumerate(sample):
                for k,x in enumerate(sample):
                    sensor=(x,y,terrain.height(x,y)+.65)
                    if math.dist(first['position'],sensor)<=plan['policy']['collection_range_m']:
                        mask[j,k]=evaluate(terrain,first['position'],sensor,max_distance=plan['policy']['collection_range_m'],minimum_uav_agl=5.)['geometry_eligible']
            from matplotlib.colors import ListedColormap
            xy.contourf(sample,sample,mask,levels=[.5,1.5],colors=[f'C{i}'],alpha=.12)
    master=[r for r in rows if r['uav']==ids[0]]
    ts=[float(r['mission_elapsed_sim_s']) for r in master]
    progress.plot(ts,[float(r['phase_time_s']) for r in master],label='Reference progress')
    progress.plot(ts,ts,'k--',alpha=.4,label='No pauses')
    progress.legend()
    if len(ids)==3:
        for field,label in [('d12_m','UAV1–UAV2'),('d13_m','UAV1–UAV3'),('d23_m','UAV2–UAV3')]:
            detail.plot(ts,[float(r[field]) for r in master],label=label)
        detail.axhline(12,color='r',linestyle='--');detail.axhline(28,color='r',linestyle='--')
        detail.set(ylabel='Metres',title='Measured estimated pair distances');detail.legend()
    else:
        detail.step(ts,[int(r['service_completed_count']) for r in master],where='post')
        detail.set(ylabel='Completed geometric services',title='Telemetry-gated service ledger')
    xy.set(xlabel='East (m)',ylabel='North (m)',title='Actual estimate (solid), command (dashed); shaded eligibility at first credit',aspect='equal')
    xy.legend(fontsize=8)
    agl.axhline(5,color='r',linestyle='--',label='Minimum 5 m');agl.legend()
    agl.set(xlabel='Mission simulation time (s)',ylabel='AGL (m)',title='Terrain clearance from PX4 estimate')
    progress.set(xlabel='Mission simulation time (s)',ylabel='Reference time (s)',title='Progress includes group waits and settling')
    detail.set_xlabel('Mission simulation time (s)')
    fig.suptitle(f"PX4/Gazebo telemetry — {len(ids)} UAV; {feedback['completed_sensors']}/{feedback['expected_sensors']} scheduled geometric services\nFootprint sampled 2.5 m at antenna height +0.65 m; no camera / packet / ground-truth proof",fontsize=12)
    fig.tight_layout(rect=(0,0,1,.95))
    fig.savefig(output/'execution.png',dpi=145)
    plt.close(fig)
    (output/'source.json').write_text(json.dumps(dict(summary=str(Path(summary_path).resolve()),scope='Sampled geometric footprint at per-UAV first credited pose, not imagery or received bytes'),indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--summary',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();render(args.summary,args.output)
