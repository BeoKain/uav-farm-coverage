"""Load the frozen scenario schedule and its local farm assets."""
import json
from pathlib import Path

from execution_plan import ROOT, check_session, to_world, to_local, vector_to_ned

FROZEN = ROOT/'config/fleet_drill.json'


def load(path,vehicles):
    source=Path(path).read_bytes()
    if vehicles!=3 or source!=FROZEN.read_bytes():
        raise ValueError('Expected the frozen scenario fleet plan')
    plan=json.loads(source)
    if len(plan['mission']['services'])!=30 or len(plan['ids'])!=3:
        raise ValueError('Unexpected scenario schedule')
    if plan['config']!=json.loads((ROOT/'config/farm.json').read_text()):
        raise ValueError('Frozen plan and farm config disagree')
    return plan
