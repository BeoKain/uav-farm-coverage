# Third-party notices

The root MIT license applies to original project code and documentation, except material explicitly identified below. It does not relicense upstream software, trademarks or cited papers.

| Component | Origin | License / treatment |
|---|---|---|
| `assets/models/x500/`, `assets/models/x500_base/` | PX4-Autopilot, recorded commit `0bb36a30ed4390f327c04f8f5d7c5cdcb8e9d235`, with project LiDAR/model changes | BSD-3-Clause; full notice in `licenses/PX4-BSD-3-Clause.txt`. Existing model/mesh/texture names are retained. |
| `px4_msgs` | External ROS package; pinned checkout in `configs/dependencies.repos` | BSD-3-Clause; notice in `licenses/px4_msgs-BSD-3-Clause.txt`. Package is fetched separately, not vendored. |
| PX4 firmware | External runtime/build dependency | BSD-3-Clause; not redistributed as a binary in this repository. |
| ns-3.43 | External simulator, <https://www.nsnam.org/> | GNU GPLv2; full license in `licenses/ns-3-GPL-2.0.txt`. No ns-3 library or linked replay binary is committed. |
| Gazebo, ROS 2, Micro XRCE-DDS, NumPy, Matplotlib, pymavlink, pyulog | External dependencies | Retain their respective upstream licenses. Dependency installation does not transfer their copyright to this project. |

`network/replay.cc` and `network/terrain_mesh.h` are project application/helper source. Their MIT licensing is GPLv2-compatible. Building and distributing a combined executable linked with ns-3 requires compliance with the applicable GPLv2 terms; the root license is not a substitute for those terms. See ns-3's [license overview](https://www.nsnam.org/about/) and [contribution/license guidance](https://www.nsnam.org/docs/contributing/html/general.html).

The project-created `farm_terrain` mesh and analytical figures are generated artifacts of this project. Scientific figures reconstructed from retained logs are identified in `docs/PROVENANCE.md`. The historical Gazebo screenshot is an illustration of the simulation application, not camera payload data. Referenced articles are cited, not copied into the repository. Academic model citations are listed in `docs/REFERENCES.md`.
