cmake_minimum_required(VERSION 3.8)
project(framework)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
ament_python_install_package(chairman_tasks PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../detail/on_demand")
install(PROGRAMS resource/sum DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS resource/preset_nav_node DESTINATION lib/${PROJECT_NAME})
# 删除已停用的配置文件名，避免增量安装后出现重复模式编号。
install(CODE "file(REMOVE
  \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/on_demand/special.yaml\"
  \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/on_demand/modes/base.yaml\"
  \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/on_demand/modes/dynamic.yaml\"
  \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/on_demand/modes/pre_align.yaml\")")
install(DIRECTORY ../config/ DESTINATION share/${PROJECT_NAME}/config FILES_MATCHING PATTERN "*.yaml")
ament_package()
