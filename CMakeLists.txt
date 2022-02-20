cmake_minimum_required(VERSION 3.0.2)
project(wifi_localization)

find_package(
  catkin REQUIRED
  COMPONENTS
  roslib
  std_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  transformUpdateService.srv
  transformRequestService.srv
)

add_message_files(
  FILES
  rssData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  bin/service
  bin/receiver
  bin/simulator
  bin/collector
  bin/process
  bin/localizer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)