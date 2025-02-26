cmake_minimum_required(VERSION 3.8)
project(vr_interface)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

# find dependencies
find_package(rclpy REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(libstatistics_collector REQUIRED)
find_package(rcl_lifecycle REQUIRED)
find_package(class_loader REQUIRED)

find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(franka_example_controllers REQUIRED)



# Create the executable from the C++ file only
add_executable(robot_controller src/robot_controller.cpp 
)



target_include_directories(robot_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> 
)

include_directories(/opt/ros/humble/include)
include_directories(${rosidl_typesupport_interface_INCLUDE_DIRS})
include_directories(${rcl_INCLUDE_DIRS})
include_directories(${builtin_interfaces_INCLUDE_DIRS})
include_directories(${libstatistics_collector_INCLUDE_DIRS})
include_directories(${rcl_lifecycle_INCLUDE_DIRS})
include_directories(${class_loader_INCLUDE_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${franka_example_controllers_INCLUDE_DIRS}
  ${franka_semantic_components_INCLUDE_DIRS}
)

message(STATUS "Include directories: ${CMAKE_INCLUDE_DIRECTORIES}")
message(STATUS "class_loader include directories: ${class_loader_INCLUDE_DIRS}")

# add_executable(move_test src/move_test.cpp
# )

# target_include_directories(move_test PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

add_library(
  ${PROJECT_NAME}
  SHARED
  src/move_test.cpp
)
target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
)


# target_link_libraries(move_test
#   ${catkin_LIBRARIES}
#   ${franka_example_controllers_LIBRARIES}
#   ${franka_semantic_components_LIBRARIES}
# )


target_compile_features(robot_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  robot_controller 
  rclcpp
  
  rclpy
  moveit_ros_planning_interface 
  geometry_msgs
  std_msgs
  controller_interface
  franka_msgs
  franka_semantic_components
  hardware_interface
  moveit_core
  moveit_msgs
  pinocchio
  pluginlib
  rclcpp_lifecycle
  std_srvs
  Eigen3
  franka_example_controllers
)

# Install the Python script separately
install(PROGRAMS
  scripts/move_controller.py
  scripts/keyboard_input.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  robot_controller 
  DESTINATION lib/${PROJECT_NAME}
)

# install(TARGETS 
#   move_test 
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
