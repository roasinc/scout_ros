cmake_minimum_required(VERSION 2.8.3)
project(scout_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  BatteryState.msg
  DriverState.msg
  FaultState.msg
  Light.msg
  LightCommand.msg
  LightState.msg
  MotorState.msg
  RobotState.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

install(
  DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)