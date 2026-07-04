cmake_minimum_required(VERSION 3.8)
project(robot_low_connect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(io_context REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gpgga.msg"
  DEPENDENCIES std_msgs
)

#robot_info_transform node
add_executable(robot_info_transform src/robot_info_transform.cpp)
ament_target_dependencies(robot_info_transform 
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  serial_driver
  io_context
)

target_include_directories(robot_info_transform PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#H30 node
add_executable(H30_node src/H30_node.cpp)
ament_target_dependencies(H30_node 
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)
target_include_directories(H30_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


#G60 node
add_executable(G60_node src/G60_node.cpp)
ament_target_dependencies(G60_node 
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  
)
target_include_directories(G60_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(G60_node
  ${cpp_typesupport_target})

install(TARGETS
  robot_info_transform
  H30_node
  G60_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch 
  urdf
  config
  msg
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
