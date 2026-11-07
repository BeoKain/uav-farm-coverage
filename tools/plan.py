"""Run the retained per-seed generator into a fresh case, keeping supplied plans intact."""
import argparse,os,subprocess,sys
from prepare_case import prepare,REPO,SEEDS
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seed',type=int,choices=SEEDS,default=202)
p.add_argument('--start-z',type=float,default=7.);p.add_argument('--speed',type=float,default=1.2)
a=p.parse_args();case=prepare(a.seed)
env=dict(os.environ,UAV_FARM_CASE_ROOT=str(case),PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(REPO/'src/uav_farm'))
print('New planning case:',case,flush=True)
raise SystemExit(subprocess.call([sys.executable,str(REPO/f'scenarios/seed_{a.seed}/generate.py'),'--start-z',str(a.start_z),'--speed',str(a.speed)],env=env,cwd=case))
