cmake_minimum_required(VERSION 2.8.3)
project(scout_base)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  controller_manager 
  realtime_tools
  std_srvs
  std_msgs
  scout_msgs
  scout_lib
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(scout_base_node
  src/scout_base.cpp
)
add_dependencies(scout_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scout_base_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
  DIRECTORY config launch lib rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS scout_base_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)