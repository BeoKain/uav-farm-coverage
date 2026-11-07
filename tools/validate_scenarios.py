"""Validate all distributed reference plans offline."""
import json,os,subprocess,sys
from pathlib import Path
REPO=Path(__file__).resolve().parents[1]
for seed in (101,202,303,404,505):
    env=dict(os.environ,UAV_FARM_CASE_ROOT=str(REPO/f'scenarios/seed_{seed}'),PYTHONDONTWRITEBYTECODE='1')
    subprocess.run([sys.executable,str(REPO/'tools/validate_one.py')],env=env,check=True)
