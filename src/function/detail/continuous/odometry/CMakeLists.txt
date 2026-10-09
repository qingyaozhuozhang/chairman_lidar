cmake_minimum_required(VERSION 3.8)
project(odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(custom_msg REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(odometry_transform_math src/odometry_transform_math.cpp)
add_executable(trans src/trans.cpp)

ament_target_dependencies(odometry_transform_math
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  custom_msg
  ament_index_cpp
)
target_link_libraries(odometry_transform_math yaml-cpp)

ament_target_dependencies(trans
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
)

install(PROGRAMS odometry.py DESTINATION lib/${PROJECT_NAME} RENAME odometry)
install(TARGETS
  odometry_transform_math
  trans
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
