# gazebo-ros2-setup

A ROS 2 + Gazebo simulation of a differential-drive robot with a 2D LiDAR, driving in a walled lab world filled with obstacles. It includes a simple demo driver and a basic odometry-based occupancy-grid mapper.

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue)

## Packages

| Package | Purpose |
| --- | --- |
| `robot_description` | Robot URDF/xacro: chassis, wheels, caster, LiDAR, `ros2_control` hardware interface |
| `robot_control` | `ros2_control` configuration (`joint_state_broadcaster`, `diff_drive_controller`) |
| `robot_gazebo` | Gazebo world (`lab.sdf`) and the `sim.launch.py` launch file |
| `robot_logic` | Python nodes: `drive_demo` (drives the robot) and `basic_mapper` (builds a `/map`) |

## Running on WSL (Ubuntu)

These steps assume **Ubuntu 24.04 on WSL2 with ROS 2 Jazzy and Gazebo Harmonic**. The diff-drive controller is commanded with `TwistStamped`, which matches Jazzy.

### 1. WSL with GUI support

Use Windows 11 (or Windows 10 with the latest WSL) with WSLg, which provides GUI apps out of the box. In PowerShell:

```powershell
wsl --install -d Ubuntu-24.04
wsl --update
```

Gazebo needs OpenGL. If rendering is slow or fails, update your GPU driver on Windows to one with WSL support. Software rendering also works: `export LIBGL_ALWAYS_SOFTWARE=1`.

### 2. Install ROS 2 Jazzy

Follow the [official instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), then install the dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro \
  python3-colcon-common-extensions
```

### 3. Build

Clone the repo **inside the WSL filesystem** (e.g. `~/`), not under `/mnt/c`. Builds are much faster and file watching is more reliable.

```bash
git clone https://github.com/SharonLK/gazebo-ros2-setup.git
cd gazebo-ros2-setup
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Run

Start the simulation (Gazebo, robot, controllers, `/clock` and `/scan` bridges):

```bash
ros2 launch robot_gazebo sim.launch.py
```

In a second WSL terminal (source `/opt/ros/jazzy/setup.bash` and `install/setup.bash` in each new terminal):

```bash
ros2 run robot_logic drive_demo --ros-args -p use_sim_time:=true       # drive the robot around
ros2 run robot_logic basic_mapper --ros-args -p use_sim_time:=true     # publish an occupancy grid on /map
```

`use_sim_time:=true` makes a node take its timers and `now()` from Gazebo's `/clock` instead of the wall clock. Without it, timing drifts from the robot whenever the simulation runs slower than real time, which is common on WSL. The launch file only sets it for `robot_state_publisher`, so pass it to every node you start yourself.

To view the map, run `rviz2` and add a **Map** display on `/map` with the fixed frame set to `odom`. You can also add a **LaserScan** display on `/scan`.

## Topics

| Topic | Type | Notes |
| --- | --- | --- |
| `/diff_drive_controller/cmd_vel` | `geometry_msgs/TwistStamped` | Velocity command input |
| `/diff_drive_controller/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR, bridged from Gazebo |
| `/map` | `nav_msgs/OccupancyGrid` | Published by `basic_mapper` |

Manual driving example:

```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.3}}}"
```

## Notes

- `basic_mapper` uses odometry only (no scan matching or loop closure), so the map drifts as odometry does.
- Linting: `colcon test --packages-select robot_logic` runs the ament copyright, flake8 and pep257 checks. Python is formatted with ruff (single quotes).
