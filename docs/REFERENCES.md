# Selected theoretical and platform references

- Zeng, Xu and Zhang, *Energy Minimization for Wireless Communication with Rotary-Wing UAV*, [arXiv:1804.02238](https://arxiv.org/abs/1804.02238). The level-flight power relationship informs the uncalibrated proxy; project extensions and coefficients are explicit assumptions.
- LaValle, *Planning Algorithms*, [author's open book](https://lavalle.pl/planning/). Background on motion planning and configuration spaces.
- Lewis and Tan, *High Precision Formation Control of Mobile Robots Using Virtual Structures*, [DOI](https://doi.org/10.1023/A:1008814708459). Conceptual background; this project's three-UAV implementation is evaluated separately.
- [PX4 ROS 2 guide](https://docs.px4.io/main/en/ros/ros2_comm) and [uXRCE-DDS](https://docs.px4.io/main/en/middleware/uxrce_dds). Message definitions and agent/client compatibility.
- [Gazebo Harmonic documentation](https://gazebosim.org/docs/harmonic/) and [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/). Platform descriptions; exact observed code revisions are recorded separately.
- [ns-3.43 propagation models](https://www.nsnam.org/docs/release/3.43/models/html/propagation.html). Background for log-distance propagation. The 25 dB terrain-blocked penalty is a project assumption, not an empirically validated farm value.

The distributed repository does not implement PSO, APSO or NSGA-II as its core planner. It retains finite heuristic search, route-order improvement and disclosed historical candidate inputs. Do not cite those alternative optimizers as implemented components of this release.
