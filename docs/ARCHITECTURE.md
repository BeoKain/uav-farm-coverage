# Architecture and code map

## Pipeline

1. Ground sensor coordinates and a triangulated synthetic terrain define eligible UAV–sensor pairs: 3D distance, line of sight and AGL.
2. `baseline_planner.py` creates formation states and candidate service stops. `mission_optimizer.py` provides bounded states, safe 3D connectors and multistart 2-opt/insertion route improvement. `arm_deadline.py` ranks ARM-deadline feasibility and energy proxies.
3. `cruise_profile.py` parameterizes motion. `energy_model.py` evaluates an uncalibrated reference power/energy model.
4. `execution_supervisor.py` gates a shared progress clock and the sensor-service ledger using fleet telemetry. `fleet_guard.py`, `corridor.py`, `probe_protocol.py` and related pose helpers gate movement on fresh LiDAR evidence and bounded recovery.
5. `execution_flight_test.py` sends ROS 2 `px4_msgs` Offboard messages to three local PX4 SITL instances. `farm_sim.py` owns the simulator session. `evaluate_fleet.py` checks recorded Gazebo truth after execution.
6. `network/replay.cc` consumes recorded XYZ trajectories and service windows. It does not control PX4 or optimize a route.

## Formation and coordinates

Reference offsets are (-5,-10), (10,0), (-5,10) m around a common center, with a bounded scale. Orientation is fixed in the world frame; a general rotating virtual structure is not implemented. Motion is 3D with common reference altitude. World ENU is converted to local PX4 NED using the per-UAV spawn/origin. Distances are checked for all three pairs.

PX4 estimates drive online control; Gazebo world pose is separately recorded for postflight checks. The fleet uses the Gazebo simulation clock, with independent receipt/freshness checks. The existing ROS namespaces are /fmu, /px4_1/fmu and /px4_2/fmu. The controller does not use /cmd_vel.

## Published plans versus new planning

The published `scenarios/seed_*/config/fleet_drill.json` plans are retained v69 1.4 m/s references, produced by retiming the selected v67/v68 paths. They keep service positions/order and include the same motion profiles as the historical experiment.

Each seed has a retained `generate.py` entry point. Seed 202 uses greedy/bounded states and ArmSearch; 303 follows the analogous flow; 101 includes finite scale/height refinements. Seeds **404 and 505** may load `stops_hint_source.json`, which supplies historical candidate positions. These are disclosed inputs, not independently discovered positions on unseen layouts.

`tools/plan.py` runs that generator into a newly created disposable case. It is distinct from replaying the published plan. Changing speed can produce a different search result or an explicit infeasibility error; a generated reference still needs SITL validation.

## Runtime layout

`tools/prepare_case.py` copies the scenario config/world into ignored `runs/`, links shared source/models relative to that location, and creates empty runtime/analysis folders. `UAV_FARM_CASE_ROOT` chooses the case; `PX4_ROOT` chooses external firmware. Case paths are inherited by child processes. Shared planner/control function bodies are retained; distribution changes are catalogued in [PROVENANCE.md](PROVENANCE.md).

## Coverage semantics

Service credit is a discrete sum of eligible time; it is not a count of received packets. During a service segment all members must settle; telemetry, geometry and monitoring must permit progress. HOLD/CLIMB do not earn service credit. The ns-3 prepared data uses continuous executed service windows, with ARM-to-MISSION offsets recorded per flight. Camera area coverage is outside the implemented metric.

## Online terrain behavior

The known terrain shapes the route before flight. The online guard can hold and add finite common-altitude lift when fresh LiDAR observes RISE, then require fresh evidence before resuming. It is not a general online planner for arbitrary horizontal detours. An UNKNOWN corridor is not automatically free space. AGL is vertical height above terrain, not the shortest Euclidean distance to every slope point.

Use the documented `tools/` and `scripts/` entry points. Some retained module-level legacy CLIs still expect historical grid/config filenames that are intentionally not part of this curated release.
