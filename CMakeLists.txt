cmake_minimum_required(VERSION 2.8.3)
project(focbox_unity_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
  roslaunch
  controller_manager
  hardware_interface
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  std_msgs
  serial
  roslaunch
  controller_manager
  hardware_interface
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# node executable
add_executable(focbox_unity_driver_node src/focbox_unity_driver_node.cpp
                                        src/focbox_unity_driver.cpp
                                        src/focbox_unity_interface.cpp
                                        src/focbox_unity_packet.cpp
                                        src/focbox_unity_packer.cpp)
add_dependencies(focbox_unity_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(focbox_unity_driver_node
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS focbox_unity_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#############
## Testing ##
#############

# TODO