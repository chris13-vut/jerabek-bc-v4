cmake_minimum_required(VERSION 3.8)
project(my_arming_client)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(geographic_msgs REQUIRED) 

add_executable(arming_client 
  src/main.cpp 
  src/drone_interface.cpp
  src/mission_manager.cpp
  src/flight_node.cpp
)

target_include_directories(arming_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

ament_target_dependencies(arming_client
  rclcpp
  mavros_msgs
  sensor_msgs
  geometry_msgs
  std_msgs 
  geographic_msgs 
)

install(TARGETS
  arming_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
