cmake_minimum_required(VERSION 3.5)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(focbox_unity_driver)



# find_package(ament_cmake REQUIRED COMPONENTS
#   roscpp
#   std_msgs
#   serial
#   roslaunch
#   controller_manager
#   hardware_interface
# )
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)

find_package(controller_manager REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

#ament_export_include_directories(
#  include
#)

ament_export_dependencies( # replace CATKIN_DEPENDS
  rclcpp
  std_msgs
  serial
  controller_manager

  hardware_interface
  pluginlib
)

ament_export_include_directories( # replace INCLUDE_DIRS
  include
)

ament_package(

)

###########
## Build ##
###########

# include_directories(
#   include
#   ${Boost_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
# )
include_directories(include ${Boost_INCLUDE_DIRS} ${hardware_interface_INCLUDE_DIRS})

# node executable
add_executable(focbox_unity_driver_node src/focbox_unity_driver_node.cpp
                                        src/focbox_unity_driver.cpp
                                        src/focbox_unity_interface.cpp
                                        src/focbox_unity_packet.cpp
                                        src/focbox_unity_packer.cpp)
ament_target_dependencies(
  focbox_unity_driver_node 
  rclcpp 
  std_msgs 
  serial 
  controller_manager 
  hardware_interface 
  pluginlib
)

# target_link_libraries(focbox_unity_driver_node
#   ${catkin_LIBRARIES}
# )


#############
## Install ##
#############

# install(TARGETS focbox_unity_driver_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS focbox_unity_driver_node
  DESTINATION lib/${CATKIN_PACKAGE_BIN_DESTINATION}
)


# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY include/
  DESTINATION lib/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#############
## Testing ##
#############

# TODO