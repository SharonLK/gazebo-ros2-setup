# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

A ROS 2 + Gazebo (gz-sim, via `ros_gz_*`) simulation of a differential-drive robot in a walled "lab" world. The repo root is a colcon workspace whose `src/` holds four packages. There is no README and no test suite beyond the ament lint stubs.

## Commands

Run from the repo root in a sourced ROS 2 environment (Linux/WSL; `build/`, `install/`, `log/` are gitignored):

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_gazebo sim.launch.py          # args: robot:=diffdrive_lidar world:=lab behavior:=drive_demo
ros2 run robot_logic drive_demo --ros-args -p use_sim_time:=true      # publish demo TwistStamped commands
ros2 run robot_logic basic_mapper --ros-args -p use_sim_time:=true    # odom+scan -> /map OccupancyGrid
colcon test --packages-select robot_logic       # ament copyright / flake8 / pep257 only
colcon test-result --verbose
```

Python formatting is ruff with single quotes (`src/pyproject.toml`).

## Architecture

Packages are split by concern:

- `robots` — one self-contained folder per robot (`<name>/robot.urdf.xacro`, `controllers.yaml`, `bridges.yaml`). Currently `diffdrive_lidar`: links, `gpu_lidar` sensor, `ros2_control` block, the `gz_ros2_control` plugin (loads `$(find robots)/diffdrive_lidar/controllers.yaml`), 100 Hz controller manager and `diff_drive_controller` parameters. To add a robot, copy the nearest folder, edit it, and add it to the `install(DIRECTORY ...)` list in `robots/CMakeLists.txt`.
- `robot_gazebo` — `worlds/lab.sdf` and `launch/sim.launch.py`, which wires everything together.
- `robot_logic` — ament_python, robot-independent nodes (`drive_demo`, `basic_mapper`), registered as console scripts in `setup.py`.

### Launch flow (`sim.launch.py`)

Gazebo starts the world, `robot_state_publisher` publishes the xacro-expanded URDF, and `ros_gz_sim create` spawns it from the `robot_description` topic. Controllers are started via chained `OnProcessExit` handlers: spawn robot → `joint_state_broadcaster` spawner → `diff_drive_controller` spawner. Add new controllers to this chain rather than starting them in parallel. Launch arguments `robot`, `world` and `behavior` (a `robot_logic` executable, started with sim time) select the configuration. One `parameter_bridge` reads the robot's `bridges.yaml`; new Gazebo topics need an entry there.

### Conventions that span files

- The `gz_frame_id` of the lidar sensor in the URDF must stay `lidar_link` so `/scan` frames match the TF tree.
- Topic contract every robot must provide, so `robot_logic` nodes stay robot-independent: `/cmd_vel` (`geometry_msgs/TwistStamped`), `/odom`, `/scan`, `/clock`. The launch file remaps the controller's `cmd_vel`/`odom` onto them via `--controller-ros-args` (`CONTROLLER_REMAPS` in `sim.launch.py`).
- `basic_mapper` is deliberately naive "SLAM": it projects scan hits through odometry pose only (no scan matching, no `map→odom` TF); it Bresenham-traces each ray to mark free cells (0) and the hit cell occupied (100), never overwriting occupied cells with free, and clears to max range on `inf` readings. It publishes a 400x400 grid at 5 cm in the `odom` frame. It assumes the scan frame coincides with the robot origin/yaw (no lidar offset applied).
- Sim time comes from the `/clock` bridge, but only `robot_state_publisher` sets `use_sim_time`. `drive_demo` and `basic_mapper` run on wall time unless started with `--ros-args -p use_sim_time:=true`. Pass it to any new node, since robot timing follows sim time, not wall time.
