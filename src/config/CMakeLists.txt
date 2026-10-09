cmake_minimum_required(VERSION 3.8)
project(chairman_config)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
install(PROGRAMS resource/sync DESTINATION lib/${PROJECT_NAME})
install(FILES manifest.json DESTINATION share/${PROJECT_NAME})
install(CODE "file(REMOVE
  \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/framework/on_demand/special.yaml\"
  \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/framework/on_demand/modes/base.yaml\"
  \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/framework/on_demand/modes/dynamic.yaml\"
  \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/framework/on_demand/modes/pre_align.yaml\")")
install(DIRECTORY fast_lio fishbot_navigation2 livox_ros_driver2 odometry framework DESTINATION share/${PROJECT_NAME})
ament_package()
