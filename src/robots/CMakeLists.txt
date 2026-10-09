cmake_minimum_required(VERSION 3.8)
project(robots)

find_package(ament_cmake REQUIRED)

# Each subdirectory is one complete robot: <name>/robot.urdf.xacro,
# <name>/controllers.yaml and <name>/bridges.yaml
install(
  DIRECTORY diffdrive_lidar
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
