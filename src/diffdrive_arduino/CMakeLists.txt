cmake_minimum_required(VERSION 3.16)
project(diffdrive_arduino LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# LibSerial
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)
include_directories(${LIBSERIAL_INCLUDE_DIRS})
link_directories(${LIBSERIAL_LIBRARY_DIRS})

# Build library
add_library(
  diffdrive_arduino
  SHARED
  src/diffbot_system.cpp
)

target_compile_features(diffdrive_arduino PUBLIC cxx_std_17)

target_include_directories(diffdrive_arduino PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/diffdrive_arduino>
  ${LIBSERIAL_INCLUDE_DIRS}
)

ament_target_dependencies(
  diffdrive_arduino PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(diffdrive_arduino PUBLIC ${LIBSERIAL_LIBRARIES})

target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFFDRIVE_ARDUINO_BUILDING_DLL")

# Pluginlib export
pluginlib_export_plugin_description_file(hardware_interface diffdrive_arduino.xml)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/diffdrive_arduino
)

# Install plugin XML
install(FILES diffdrive_arduino.xml
  DESTINATION share/diffdrive_arduino
)

# Install library
install(TARGETS diffdrive_arduino
  EXPORT export_diffdrive_arduino
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export
ament_export_targets(export_diffdrive_arduino HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()