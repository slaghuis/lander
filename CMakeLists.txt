cmake_minimum_required(VERSION 3.5)
project(lander)

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
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lander_interfaces REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(lander_node 
  src/lander_node.cpp
  src/pending_state.cpp
  src/seeking_state.cpp
  src/approaching_state.cpp
  src/descending_state.cpp
  src/landing_state.cpp)
ament_target_dependencies(lander_node 
  rclcpp 
  rclcpp_action 
  std_msgs 
  geometry_msgs 
  nav_msgs
  drone_interfaces 
  lander_interfaces
  tf2 
  tf2_ros 
  tf2_msgs
  tf2_geometry_msgs)

target_include_directories(lander_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>) 

add_executable(tracker_node src/tracker_node.cpp)
ament_target_dependencies(tracker_node
  rclcpp 
  std_msgs 
  nav_msgs
  sensor_msgs 
  geometry_msgs 
  tf2 
  tf2_ros 
  tf2_msgs
  tf2_geometry_msgs
  OpenCV
  cv_bridge
  lander_interfaces)
  
target_include_directories(tracker_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  lander_node
  tracker_node
  DESTINATION lib/${PROJECT_NAME}) 

  
ament_package()
