cmake_minimum_required(VERSION 3.8)
project(micro_ros)
find_package(ament_cmake REQUIRED)
# Keep the bundle root free of package.xml so colcon also discovers src/.
install(PROGRAMS ../agent.py DESTINATION lib/${PROJECT_NAME} RENAME agent)
install(DIRECTORY ../config DESTINATION share/${PROJECT_NAME})
ament_package()
