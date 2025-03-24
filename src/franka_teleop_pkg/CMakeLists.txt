cmake_minimum_required(VERSION 3.8)
project(franka_teleop_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(franka_teleop_pkg_interfaces REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(franka_msgs REQUIRED)  # Added for robot state message dependency

# Build the library for the controller
add_library(${PROJECT_NAME} SHARED
  src/cartesian_impedance_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PUBLIC ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(${PROJECT_NAME}
  PUBLIC c_std_99
  PUBLIC cxx_std_17
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  franka_teleop_pkg_interfaces
  franka_semantic_components
)
pluginlib_export_plugin_description_file(
  controller_interface franka_teleop_pkg.xml
)
install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Add executable for oscillatory pose publisher (test node)
add_executable(testeo src/testeo.cpp)
target_include_directories(testeo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(testeo PUBLIC cxx_std_17)
ament_target_dependencies(testeo
  rclcpp
  geometry_msgs
  franka_msgs
)
install(
  TARGETS testeo
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(${PROJECT_NAME}_test_load
    test/test_load_cartesian_impedance_controller.cpp
  )
  target_include_directories(${PROJECT_NAME}_test_load
    PRIVATE include
  )
  ament_target_dependencies(${PROJECT_NAME}_test_load
    controller_manager
    ros2_control_test_assets
  )

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
