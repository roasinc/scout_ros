cmake_minimum_required(VERSION 2.8.3)
project(scout_lib)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  hardware_interface
  can_msgs
  socketcan_interface
  rosconsole_bridge
)

catkin_package(
  LIBRARIES scout_controller scout_hardware
  INCLUDE_DIRS include
  DEPENDS
  CATKIN_DEPENDS
)

add_library(scout_controller
  src/scout_can_bridge.cpp
  src/scout_controller.cpp
)
target_link_libraries(scout_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(scout_hardware
  src/scout_hardware.cpp
)
target_link_libraries(scout_hardware ${catkin_LIBRARIES})

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

install(
  FILES include/${PROJECT_NAME}/resource.h include/${PROJECT_NAME}/scout_can_bridge.h include/${PROJECT_NAME}/scout_controller.h include/${PROJECT_NAME}/scout_hardware.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}
)

install(
  TARGETS scout_controller scout_hardware
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)