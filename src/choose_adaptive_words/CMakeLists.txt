cmake_minimum_required(VERSION 3.5)
project(choose_adaptive_words)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# uncomment to enable Python setup.py
# find_package(ament_cmake_python REQUIRED)

# this would be where you'd declare Python nodes
# ament_python_install_package(${PROJECT_NAME})

# include_directories(include)
# if you had a C++ library or node, you would add:
# add_library(${PROJECT_NAME} SHARED
#   src/${PROJECT_NAME}/choose_adaptive_words.cpp
# )
# ament_target_dependencies(${PROJECT_NAME} rclpy std_msgs)

# If you had C++ executables
# add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(my_node rclpy std_msgs)

# install Python nodes, launch files, etc.
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION lib/${PROJECT_NAME}
# )

# if you had C++ executables or libraries:
# install(TARGETS
#   my_node
#   ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )

# if you had header files for your library:
# install(DIRECTORY include/
#   DESTINATION include
# )

# if you had other resources to install:
# install(DIRECTORY
#   launch
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

# if you want to perform tests
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   # Add your tests here
#   # ament_add_gtest(test_my_stuff test/test_my_stuff.cpp)
# endif()

ament_package()
