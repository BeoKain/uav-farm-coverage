# Installation and software versions

Target: Ubuntu 24.04, Python 3.12, ROS 2 Jazzy, Gazebo Harmonic (Sim 8), PX4 SITL and optional ns-3.43. The retained host had Gazebo 8.15.0. The PX4 checkout was a **development commit**, not a stable 1.18 release. Exact inspected source IDs are in [environment.lock.json](../configs/environment.lock.json).

## 1. Offline Python tools

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python tools/validate_scenarios.py
python -m unittest discover -s tests -v
```

NumPy is required by planning. Matplotlib is for optional plots; pymavlink supplies the heartbeat helper. ROS Python modules come from the ROS installation, not PyPI. Optional ULog analysis uses `requirements-analysis.txt`.

## 2. ROS 2 and Gazebo

Install [ROS 2 Jazzy on Ubuntu Noble](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) and the Gazebo/ROS integration packages following the official documentation. The controller needs `rclpy`, `sensor_msgs`, `geometry_msgs`, `rosgraph_msgs`, `std_msgs` and `ros_gz_bridge`. Ensure the ROS `gz` command, `ros2` and `colcon` are available. A working GUI/OpenGL setup is needed for interactive Gazebo; offline tests do not need it.

Use a virtual environment with `--system-site-packages` if needed so that ROS's apt-provided Python modules remain available. Activate it before running the shell wrappers, or set `PYTHON_BIN` explicitly in `.env`.

## 3. PX4 firmware

Use a separate PX4 checkout and build its SITL target. For the recorded source revision:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git "$HOME/PX4-Autopilot"
cd "$HOME/PX4-Autopilot"
git checkout 0bb36a30ed4390f327c04f8f5d7c5cdcb8e9d235
git submodule update --init --recursive
# Follow the upstream Ubuntu dependency installation procedure first.
make px4_sitl
```

Follow [PX4's Ubuntu setup](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html) for platform dependencies. Do not substitute a different firmware release while claiming the exact recorded configuration. Required paths below PX4_ROOT include `build/px4_sitl_default/bin/px4`, `build/px4_sitl_default/etc` and `Tools/simulation/gz/models`.

## 4. DDS agent and message definitions

PX4 firmware and the ROS workspace must use compatible message definitions. See the official [ROS 2 guide](https://docs.px4.io/main/en/ros/ros2_comm) and [uXRCE-DDS version table](https://docs.px4.io/main/en/middleware/uxrce_dds). For Jazzy with the recorded v2 client, the documented standalone Agent version is **v2.4.3**. Build/install Micro-XRCE-DDS-Agent using the upstream instructions, then ensure `MicroXRCEAgent` is on PATH.

Back in this repository:

```bash
cp .env.example .env
# Set ROS2_WS to a separate workspace (default ~/ros2_px4_ws).
bash scripts/setup_ros_workspace.sh
```

The script fetches the pinned `px4_msgs` commit and builds only that package. It refuses to alter an existing checkout at a different commit or with local edits. The package's version string alone is not a compatibility guarantee.

## 5. Configure and inspect

Edit `.env` with your own `ROS_SETUP`, `ROS2_WS`, `PX4_ROOT`, `NS3_ROOT` and optionally `PYTHON_BIN`. Paths containing spaces must be quoted as in the example.

```bash
bash scripts/check_environment.sh
bash scripts/run_gazebo.sh 202
```

This launches a new local simulation and executes the fleet plan. First ensure no old PX4/Gazebo fleet session is active. Normal completion includes landing and disarm. The runtime and process ownership gates are retained; do not bypass them to connect to physical vehicles.

## 6. Optional ns-3.43

Install/build [ns-3.43](https://www.nsnam.org/releases/ns-3-43/) with the applications, Wi-Fi, Internet, mobility, propagation, network and core modules. The replay build expects the default shared libraries in `build/lib`, include headers in `build/include`, and g++ with C++20 support. Custom/sanitizer/static ns-3 builds need corresponding linker changes.

```bash
bash scripts/run_ns3.sh 202 --build-only
bash scripts/run_ns3.sh 202
```

The replay source is compiled locally into ignored `build/`. No ns-3 or PX4 binary is redistributed. See the license notices before distributing a linked binary yourself.

## Portability status

Local offline validation, path relocation and compilation checks are recorded in [RELEASE_CHECKS.md](RELEASE_CHECKS.md). These checks do not certify every fresh Ubuntu installation. Check ROS middleware versions, DDS message compatibility, rendering drivers and upstream dependency changes when reproducing on another workstation.
