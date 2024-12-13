cmake_minimum_required(VERSION 3.16)
project(focbox_unity_driver  LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  serial
  controller_manager 
  rclcpp
  rclcpp_lifecycle
  control_toolbox
)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  focbox_unity_driver 
  SHARED
  hardware/focbox_unity_driver.cpp
  hardware/focbox_unity_interface.cpp
  hardware/focbox_unity_packet.cpp
  hardware/focbox_unity_packer.cpp
)
target_compile_features(focbox_unity_driver  PUBLIC cxx_std_17)
target_include_directories(focbox_unity_driver  PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/focbox_unity_driver>
)

ament_target_dependencies(
  focbox_unity_driver PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "FOCBOX_DRIVER_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(focbox_unity_driver focbox_unity_driver.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/focbox_unity_driver 
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/focbox_unity_driver 
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/focbox_unity_driver 
)
install(TARGETS focbox_unity_driver 
  EXPORT export_focbox_unity_driver 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_focbox_unity_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()