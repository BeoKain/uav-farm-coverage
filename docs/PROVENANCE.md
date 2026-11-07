# Provenance and distribution changes

This source release is a curated export of the UAV farm project, prepared on 2026-11-07. The original handover and raw experiment history remain separately preserved by the project owner.

## Selection

- Shared modules: the 33 Python modules in the five v69 1.4 m/s cases were byte-identical before export. One copy is distributed in `src/uav_farm/`.
- Scenario plans/worlds/configurations: retained speed-v69 cases, derived from selected v67/v68 geometry. Numeric plan content is retained.
- Per-seed generators: corresponding v67/v68 entry points, retaining their finite-search branches and candidate hints.
- Network replay source: v70 `replay.cc` plus the project terrain helper. Prepared replay inputs are copied from v70's retained executed-flight inputs.
- Tables: 15 speed rows, 15 network rows and 450 network sensor/run rows. Numeric measurements are unchanged.
- Figures: project report figures reconstructed from retained records, plus conceptual diagrams and one explicitly historical GUI image.
- Firmware/message source revisions: recorded in `configs/environment.lock.json`. External build directories are excluded.

## Path and metadata adaptations

1. `project_paths.py` resolves `UAV_FARM_CASE_ROOT` and `PX4_ROOT`. Root resolution in `baseline_planner.py`, `execution_plan.py` and `farm_sim.py` uses this helper. The PX4 executable ownership check still checks the configured firmware executable and the case-owned runtime directory.
2. New shell wrappers read a user-created ignored `.env`. They source the configured ROS installation/workspace, then prepare a fresh disposable case. Existing Offboard, session, telemetry, clock and geometry checks remain active.
3. New ns-3 runner builds the retained C++ program against a configured `NS3_ROOT` and consumes the included prepared inputs. It preserves the historical replay parameters; new outputs are written to ignored `runs/`.
4. Per-seed generator root resolution is adapted to the new disposable case. Their planning/search formulas are retained.
5. Private absolute paths in JSON/CSV metadata are normalized to `archive://farm-sim/...`, `upstream://...` or `original-host://...`. These identifiers describe historical provenance and are not runtime file paths. Embedded original SHA-256 values identify original pre-export files. They do **not** certify the reserialized public JSON; `source_manifest.json` and the release checks identify the exported files.
6. Source comments/error wording that incorrectly referred only to seed 202 are generalized. No numerical planning/control change is intended by this distribution work.

The machine-readable [source_manifest.json](source_manifest.json) records source and distribution hashes. [core_function_hashes.json](core_function_hashes.json) records the unchanged algorithm-function comparison against the original retained source. New tools are distribution adapters, not a new flight-planning version or new experimental result.

## Evidence boundaries

All published result values are historical. Offline plan validation, unit tests, path relocation, environment inspection and compilation are packaging checks only. A fresh live GUI flight from this release is not asserted unless separately recorded in RELEASE_CHECKS.md. The historical v56 RISE recovery case and final v69 zero-CLIMB speed cases remain distinct.

The public export intentionally excludes full ULogs, old process/session state, build/install products, the full thesis Word document and unrelated historical experiments. Those exclusions reduce repository size; they do not erase the owner's original evidence archive.
