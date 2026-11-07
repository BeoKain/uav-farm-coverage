"""Resolve configured case and external firmware paths without user-specific paths."""
import os
from pathlib import Path
REPO_ROOT = Path(__file__).resolve().parents[2]
def case_root():
    return Path(os.environ.get("UAV_FARM_CASE_ROOT", REPO_ROOT/"scenarios/seed_202")).expanduser().resolve()
def px4_root():
    return Path(os.environ.get("PX4_ROOT", Path.home()/"PX4-Autopilot")).expanduser().resolve()
