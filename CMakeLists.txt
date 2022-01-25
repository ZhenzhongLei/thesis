cmake_minimum_required(VERSION 3.0.2)
project(wifi_localization)

find_package(
  catkin REQUIRED
  COMPONENTS
)

catkin_python_setup()

catkin_package()

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  bin/collect_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)