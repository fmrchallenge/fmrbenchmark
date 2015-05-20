cmake_minimum_required (VERSION 2.8.3)
project (sci_concrete_example)

find_package (catkin REQUIRED COMPONENTS rospy dynamaestro)

catkin_package ()

install (FILES launch/lqr.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install (PROGRAMS scripts/lqr.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
