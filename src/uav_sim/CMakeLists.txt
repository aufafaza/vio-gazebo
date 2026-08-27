cmake_minimum_required(VERSION 3.16)
project(uav_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY models worlds launch config params
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE)

install(
  PROGRAMS scripts/static_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME static_tf_broadcaster)

ament_package()
