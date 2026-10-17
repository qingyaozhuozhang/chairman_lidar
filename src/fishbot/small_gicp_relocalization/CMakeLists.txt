cmake_minimum_required(VERSION 3.10)
project(small_gicp_relocalization)

## small_gicp uses C++17 type traits and atomic initialization.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
find_package(small_gicp REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/small_gicp_relocalization.cpp
)

target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX small_gicp::small_gicp)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN small_gicp_relocalization::SmallGicpRelocalizationNode
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

# 增量安装时清除已停用入口，map -> odom 由定位节点统一发布。
install(CODE "file(REMOVE \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/tf_relay_node\")")
install(CODE "file(REMOVE \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/localization.yaml\")")

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_transform test/test_pose_transform.cpp)
  target_include_directories(test_pose_transform PRIVATE include)
  target_link_libraries(test_pose_transform Eigen3::Eigen)
  ament_add_gtest(test_map_preprocessing test/test_map_preprocessing.cpp)
  target_include_directories(test_map_preprocessing PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_map_preprocessing OpenMP::OpenMP_CXX small_gicp::small_gicp)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
