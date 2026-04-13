cmake_minimum_required(VERSION 3.8)
project(cobot_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install Python package (exposes demo_motion node)
ament_python_install_package(${PROJECT_NAME})

# Install all ROS data directories
install(
  DIRECTORY urdf meshes launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
