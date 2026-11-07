# Release checks — source export 0.1.0

Prepared and checked on **2026-11-07**, before public upload. This file records packaging checks, not new experimental results.

| Check | Outcome |
|---|---|
| Supplied reference plans | All five layouts passed full offline mission/profile/terrain/service checks, ARM prediction, probe direction checks and lifted-service geometry at 0/0.5/1.0/1.5 m. |
| Unit tests | 7/7 passed: range versus terrain blocking, profile speed/acceleration limits and endpoints, blocked service credit, pair-envelope violation, replay input coverage, recorded byte totals and case isolation. |
| Independent path relocation | Copied the distributable tree to a temporary path containing spaces; all five plan checks, seven tests and a disposable plan-load command passed there. |
| Gazebo resource syntax | All five relocated world/model SDF checks passed. No simulator process or vehicle flight was started by these checks. |
| ns-3 build | Retained C++ source compiled against local ns-3.43; its help command succeeded. No network replay was executed for this packaging session. Generated executable removed from the release tree. |
| Local environment | Configured ROS/PX4/message dependencies and ros_gz_bridge were found; PX4 and px4_msgs commits matched the recorded IDs. This inspected the existing workstation, not a fresh install. |
| Algorithm preservation | 172 of 175 original functions are AST-identical. Changes inside three functions concern configured PX4 paths and scenario error wording. All five distributed plan numeric values match the retained source. |
| Release content audit | No private absolute home paths, common credential markers, missing local Markdown links or file at/above 50 MiB found in the audited distributable tree. Shell syntax/Python syntax/JSON parsing checks passed. |

The initial unit-test attempt used the wrong terrain-fixture representation (a nested list instead of the constructor's coordinate map). The fixture was corrected; project terrain code was unchanged and all seven tests then passed.

## What was not certified

- No new live Gazebo/PX4 fleet flight from the portable repository. The supplied plans' historical SITL results remain the flight evidence.
- No new ns-3 measurement; published tables remain the retained 15 runs.
- No fresh OS/dependency installation, complete reproducible container image, hardware flight or calibrated energy/radio model.
- GitHub Actions has been configured for offline checks but has not run on GitHub until the owner uploads the repository.
- The credential/content scan is a targeted packaging check, not an exhaustive security audit.

`core_function_hashes.json` and `source_manifest.json` preserve the source comparison. New runtime outputs and local environment configuration are ignored by Git. The public tree excludes the full thesis Word file and raw ULogs; those remain in the private/full handover.
