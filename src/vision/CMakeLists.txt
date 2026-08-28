cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED) 
find_package(libmavconn REQUIRED)
find_package(mavlink REQUIRED)
find_package(tf2_ros REQUIRED)
add_executable(t265_subscriber src/t265_subscriber.cpp)
target_include_directories(t265_subscriber PUBLIC include)
ament_target_dependencies(t265_subscriber rclcpp nav_msgs mavros_msgs geometry_msgs libmavconn tf2_ros)

install(TARGETS t265_subscriber DESTINATION lib/${PROJECT_NAME})

ament_package()
