cmake_minimum_required(VERSION 3.5)
project(nao_trajectory_following)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(
  ${rclpy_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

# Installation of Python nodes
install(PROGRAMS
   nodes/nao_writer_naoqi.py
   nodes/trajectory_visualizer.py
   nodes/writing_surface_positioner.py
   DESTINATION lib/${PROJECT_NAME}
)

# Installation of other files
install(DIRECTORY launch config res
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rclpy)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(visualization_msgs)

# Package setup
ament_package()
