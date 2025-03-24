cmake_minimum_required(VERSION 3.8)
project(ft_sensor_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include/ft_sensor_node
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(ft_sensor_node 
    src/ft_sensor_node.cpp
    src/BotaForceTorqueSensorComm.cpp
)
ament_target_dependencies(ft_sensor_node rclcpp geometry_msgs)
install(TARGETS
  ft_sensor_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(ft_filter_node src/ft_filter_node.cpp)
ament_target_dependencies(ft_filter_node rclcpp geometry_msgs std_msgs)
install(TARGETS
  ft_filter_node
  DESTINATION lib/${PROJECT_NAME})


install(
  PROGRAMS src/serial_driver
  DESTINATION lib/${PROJECT_NAME}
)


install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
