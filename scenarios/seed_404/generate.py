"""Generate the seed-202 flight using its sensor layout and motion inputs."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

from project_paths import case_root
ROOT = case_root()

from arm_deadline import ArmSearch, assign_subset, refine
from baseline_planner import plan as greedy, poses
from energy_model import EnergyModel
from mission_optimizer import bounded_stages, validate_mission
from terrain_links import TerrainGrid, evaluate
from probe_protocol import ProbeGate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-z", type=float, required=True)
    parser.add_argument("--speed", type=float, required=True)
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.start_z, args.speed)):
        parser.error("Expected positive finite start-z and speed")
    cfg = ROOT/"config"
    read = lambda name: json.loads((cfg/name).read_text())
    adapter = read("adapter_template.json")
    config = read("farm.json")
    terrain = TerrainGrid.from_csv(cfg/"terrain.csv")
    model = EnergyModel(read("energy_model.json"))
    budget = read("arm_budget.json")
    p = dict(adapter["mission"]["policy"], horizontal_speed_m_s=args.speed)
    initial = (-65., 0., args.start_z, 1.)
    stops = bounded_stages(greedy(config, terrain, p), terrain, p, config)
    mission = ArmSearch(config,terrain,model,p,stops,initial,budget=budget).solve()
    alternatives = [dict(source="original", slack_s=mission["arm_deadline_prediction"]["minimum_slack_s"])]
    if mission["arm_deadline_prediction"]["deadline_misses"]:
        source = read("stops_hint_source.json")
        states = [tuple(t["state"]) for t in source["mission"]["stages"]]
        stops = assign_subset(config,terrain,states,range(len(states)),p)
        if stops is None:
            raise ValueError("Historic candidate positions cannot cover sensors")
        hinted = ArmSearch(config,terrain,model,p,stops,initial,budget=budget).solve()
        options = [hinted,refine(hinted,config,terrain,model,budget)]
        accepted = []
        for i,result in enumerate(options):
            row = dict(source=i, slack_s=result["arm_deadline_prediction"]["minimum_slack_s"])
            alternatives.append(row)
            try:
                validate_mission(result,config,terrain)
                gate = ProbeGate(adapter["ids"],poses)
                for seg in result["segments"]:
                    gate.select(seg,seg["start_s"])
                if max(seg[k][2] for seg in result["segments"] for k in ("start","end"))+1.5 > adapter["flight_ceiling_m"]:
                    raise ValueError("ceiling with lift reserve")
                sensors = {item["id"]:item for item in config["sensors"]}
                for task in result["services"]:
                    x,y,z = task["position"]
                    for h in (0.,.5,1.,1.5):
                        if not evaluate(terrain,(x,y,z+h),sensors[task["sensor"]]["world_enu_m"],
                                        p["collection_range_m"],p["min_agl_m"])["geometry_eligible"]:
                            raise ValueError("lifted link "+task["sensor"])
                accepted.append(result)
            except ValueError as exc:
                row["rejected"] = str(exc)
        mission = min([mission,*accepted],key=ArmSearch.rank)
    (ROOT/"analysis"/"search_options.json").write_text(json.dumps(alternatives,indent=2)+"\n")
    checks = validate_mission(mission,config,terrain)
    if mission["arm_deadline_prediction"]["deadline_misses"]:
        raise ValueError("Generated plan misses the predicted ARM deadline")
    ids = adapter["ids"]
    spawns = {s["id"]:s["world_enu_m"] for s in config["uav_spawns"]}
    launch = max((args.start_z-s[2])/p["vertical_speed_m_s"] for s in spawns.values())
    segments = mission["segments"]
    gate = ProbeGate(ids, poses)
    for segment in segments:
        gate.select(segment,segment["start_s"])
    sensors = {s["id"]:s for s in config["sensors"]}
    for t in mission["services"]:
        x,y,z = t["position"]
        for lift in (0.,.5,1.,1.5):
            if not evaluate(terrain,(x,y,z+lift),sensors[t["sensor"]]["world_enu_m"],
                            p["collection_range_m"],p["min_agl_m"])["geometry_eligible"]:
                raise ValueError(f"Lifted service geometry failed for {t['sensor']}")
    if max(s[k][2] for s in segments for k in ("start","end"))+1.5 > adapter["flight_ceiling_m"]:
        raise ValueError("Generated route exceeds ceiling with lift reserve")
    services = {u:[t for t in mission["services"] if t["uav"]==u] for u in ids}
    for a,b in zip(poses(initial),[spawns[u] for u in ids]):
        if math.dist(a[:2],b[:2]) > 1e-7:
            raise ValueError("Initial formation XY differs from spawn pads")
    files = ("farm.json","terrain.csv","energy_model.json","arm_budget.json","adapter_template.json","stops_hint_source.json")
    adapter.update(config=config, policy=dict(adapter["policy"],**p), mission=mission, services=services,
                   assignments={u:[t["sensor"] for t in ts] for u,ts in services.items()},
                   lifted_service_revalidation=True,
                   source_sha256={f"config/{f}":hashlib.sha256((cfg/f).read_bytes()).hexdigest() for f in files},
                   preparation=dict(start_world_z_m=args.start_z,speed_m_s=args.speed,seed=404),
                   summary=dict(vehicles=3,expected_sensors=len(sensors),planned_mission_reference_s=segments[-1]["end_s"],
                                predicted_arm_slack_s=mission["arm_deadline_prediction"]["minimum_slack_s"],checks=checks))
    adapter["phases"] = dict(
        CLIMB={u:[(0.,*spawns[u]),(launch,*a)] for u,a in zip(ids,poses(initial))},
        MISSION={u:[(0.,*poses(initial)[i])]+[(s["end_s"],*poses(s["end"])[i]) for s in segments]
                 for i,u in enumerate(ids)})
    (cfg/"fleet_drill.json").write_text(json.dumps(adapter,indent=2)+"\n")
    print(json.dumps(adapter["summary"],indent=2))


if __name__ == "__main__":
    main()
