"""Create a disposable case; never modify the published scenario configurations."""
import argparse
from datetime import datetime
import os
from pathlib import Path
import shutil
REPO = Path(__file__).resolve().parents[1]
SEEDS = (101, 202, 303, 404, 505)
def prepare(seed, output=None):
    if seed not in SEEDS: raise ValueError('Unknown scenario seed')
    target = Path(output).expanduser().resolve() if output else REPO/'runs'/f'seed_{seed}_{datetime.now().strftime("%Y%m%d_%H%M%S_%f")}'
    if target == REPO or any(target.is_relative_to(REPO/p) for p in ('src','scenarios','assets','data','results','docs','tools','scripts','.git')):
        raise ValueError('Output must be outside published source/data directories')
    target.mkdir(parents=True, exist_ok=False)
    source = REPO/'scenarios'/f'seed_{seed}'
    for name in ('config','worlds'): shutil.copytree(source/name, target/name)
    shutil.copy2(source/'gui_trails.config',target/'gui_trails.config')
    for name,path in [('scripts',REPO/'src/uav_farm'),('models',REPO/'assets/models')]:
        (target/name).symlink_to(os.path.relpath(path,target),target_is_directory=True)
    (target/'runtime').mkdir(); (target/'analysis').mkdir()
    return target
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seed',type=int,choices=SEEDS,default=202);p.add_argument('--output',type=Path)
    a=p.parse_args();print(prepare(a.seed,a.output))
