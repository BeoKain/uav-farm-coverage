"""Build/replay the retained Gazebo traces with ns-3.43, without modifying recorded results."""
import argparse,csv,json,os,subprocess
from datetime import datetime
from pathlib import Path
REPO=Path(__file__).resolve().parents[1]
def build(ns3):
    if not (ns3/'VERSION').is_file() or (ns3/'VERSION').read_text().strip()!='3.43':raise ValueError('Set NS3_ROOT to a built ns-3.43 source tree')
    lib=ns3/'build/lib';binary=REPO/'build/ns3_replay';binary.parent.mkdir(exist_ok=True)
    command=['g++','-std=c++20','-O2','-I'+str(ns3/'build/include'),'-I'+str(REPO/'network'),str(REPO/'network/replay.cc'),'-L'+str(lib),'-Wl,-rpath,'+str(lib),*['-lns3.43-'+m+'-default' for m in ('applications','wifi','internet','mobility','propagation','network','core')],'-o',str(binary)]
    subprocess.run(command,check=True);return binary
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seed',type=int,choices=(101,202,303,404,505),default=202)
    p.add_argument('--build-only',action='store_true');p.add_argument('--runs',type=int,nargs='+',default=[1,2,3])
    a=p.parse_args()
    if any(r<1 for r in a.runs):p.error('RNG runs must be positive')
    binary=build(Path(os.environ.get('NS3_ROOT',Path.home()/'ns-allinone-3.43/ns-3.43')).expanduser().resolve())
    if a.build_only:print(binary);raise SystemExit(0)
    data=REPO/f'data/replay/seed_{a.seed}';meta=json.loads((data/'metadata.json').read_text())
    out=REPO/'runs'/f'ns3_seed_{a.seed}_{datetime.now().strftime("%Y%m%d_%H%M%S_%f")}';out.mkdir(parents=True,exist_ok=False)
    commands=[]
    for run in a.runs:
        command=[str(binary),f'--positions={data/"positions.csv"}',f'--terrain={REPO/f"scenarios/seed_{a.seed}/config/terrain.csv"}',f'--trajectory={data/"trajectories.csv"}',f'--assignments={data/"assignments.csv"}',f'--armOffset={meta["arm_to_mission_s"]}','--deadline=300','--interval=0.05','--arpDeadTimeout=5','--extraLossDb=25','--seed=12345',f'--run={run}',f'--output={out/f"run_{run}.csv"}',f'--timeline={out/f"run_{run}_timeline.csv"}',f'--diagnostics={out/f"run_{run}_arp.csv"}']
        commands.append(command)
        with (out/f'run_{run}.log').open('w') as stream:subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,check=True,timeout=240)
        with (out/f'run_{run}.csv').open() as stream:rows=list(csv.DictReader(stream))
        assert len(rows)==30 and all(int(r['tx_attempts'])==100 for r in rows)
        print('run',run,'unique bytes',sum(int(r['unique_bytes']) for r in rows))
    (out/'commands.json').write_text(json.dumps(commands,indent=2))
    print('New outputs:',out)
