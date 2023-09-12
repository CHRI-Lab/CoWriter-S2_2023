cmake_minimum_required(VERSION 3.5)

project(letter_learning_interaction)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare ROS messages and services
set(msg_files
  "msg/Shape.msg"
)
set(srv_files
  "srv/ClearAllShapes.srv"
  "srv/DisplayNewShape.srv"
  "srv/IsPossibleToDisplayNewShape.srv"
  "srv/IndexOfLocation.srv"
  "srv/ShapeAtLocation.srv"
  "srv/ClosestShapesToLocation.srv"
  "srv/DisplayShapeAtLocation.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

# Install Python nodes
install(PROGRAMS
  nodes/display_manager_server.py
  nodes/word_card_detector.py
  nodes/tablet_input_interpreter.py
  nodes/learning_words_nao.py
  nodes/watchdog_clearer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Find testing dependencies
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  
  # Add tests (Note: this is just a template; adapt accordingly)
  # ament_add_pytest_test(test_display_manager_server tests/test_display_manager_server.py)
  # ... repeat for other tests ...
  
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
