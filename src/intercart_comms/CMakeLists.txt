cmake_minimum_required(VERSION 3.8)
project(intercart_comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate ROS 2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CartState.msg"
  DEPENDENCIES geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Skip copyright check
  set(ament_cmake_copyright_FOUND TRUE)

  # Skip cpplint
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_executable(intercart_comm_node
  src/intercart_comms_node.cpp
  src/cart_state.cpp
  src/conflict_resolver.cpp
)

ament_target_dependencies(intercart_comm_node
  rclcpp
  geometry_msgs
  std_msgs
)

target_include_directories(intercart_comm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Make sure the generated CartState interface is available
rosidl_target_interfaces(intercart_comm_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS
  intercart_comm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()