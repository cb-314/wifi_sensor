cmake_minimum_required(VERSION 2.8.3)
project(wifi_sensor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  Rssi.msg
  RssiMulti.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
  CATKIN_DEPENDS rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  bin/wifi_sensor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
