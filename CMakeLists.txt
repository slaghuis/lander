cmake_minimum_required(VERSION 3.5)
project(target_lander)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lander_interfaces REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(target_lander 
  src/target_lander_node.cpp
  src/target_lander_state.cpp
  src/pending_state.cpp
  src/seeking_state.cpp
  src/approaching_state.cpp
  src/descending_state.cpp
  src/landing_state.cpp)
ament_target_dependencies(target_lander 
  rclcpp 
  rclcpp_action 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  lander_interfaces
  drone_interfaces
  tf2 
  tf2_ros 
  tf2_msgs
  tf2_geometry_msgs)

target_include_directories(target_lander PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  target_lander
  DESTINATION lib/${PROJECT_NAME}) 
  
ament_package()
