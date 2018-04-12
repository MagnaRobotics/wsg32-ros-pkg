cmake_minimum_required(VERSION 2.8.3)
project(wsg_32_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  actionlib
  control_msgs
  wsg_32_common
  hardware_interface
  controller_manager
)

catkin_package(
  # INCLUDE_DIRS
  # LIBRARIES 
  CATKIN_DEPENDS roscpp std_msgs std_srvs wsg_32_common control_msgs actionlib hardware_interface controller_manager
  # DEPENDS system_lib
)


# WSG_50_TCP version
set(DRIVER_SOURCES 
  src/checksum.cpp include/wsg_32/checksum.h
  src/cmd.c include/wsg_32/cmd.h
  src/common.cpp include/wsg_32/common.h
  src/functions.cpp include/wsg_32/functions.h
  src/interface.cpp include/wsg_32/interface.h
  # src/main.cpp
  # src/gripper_action_server.cpp include/wsg_32/gripper_action_server.h
  src/wsg_32_ros_interface.cpp include/wsg_32/wsg_32_ros_interface.h
  src/wsg_32_hardware_interface.cpp include/wsg_32/wsg_32_hardware_interface.h
  src/msg.c include/wsg_32/msg.h
  src/serial.c include/wsg_32/serial.h
  src/tcp.c include/wsg_32/tcp.h
  src/udp.c include/wsg_32/udp.h)

# WSG_50_CAN version
set(DRIVER_SOURCES_CAN
  src/checksum.cpp
  src/common.cpp
  src/functions_can.cpp
  src/msg.c)

include_directories(include ${catkin_INCLUDE_DIRS}) 
add_definitions(-DOSNAME_LINUX)
add_definitions(-g)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x") # Need C++11 

# add_executable(wsg_32_ip src/main.cpp ${DRIVER_SOURCES})
# add_executable(gripper_server src/gripper_action_server.cpp ${DRIVER_SOURCES})
# add_dependencies(gripper_server ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gripper_server ${catkin_LIBRARIES})

add_executable(gripper_ros_control src/wsg_32_hardware_interface.cpp ${DRIVER_SOURCES})
add_dependencies(gripper_ros_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(gripper_ros_control ${catkin_LIBRARIES})

#add_executable(wsg_32_can src/main_can.cpp src/checksum.cpp src/msg.c src/common.cpp src/functions_can.cpp)
#add_executable(wsg_32_can src/main_can.cpp ${DRIVER_SOURCES_CAN})

#include_directories(/home/marc/peak-linux-driver-7.5/driver/src/)
#link_directories(/home/marc/peak-linux-driver-7.5/lib/)
#add_compile_flags(wsg_32_can -g -Wall)
#target_link_libraries(wsg_32_can pcan)


