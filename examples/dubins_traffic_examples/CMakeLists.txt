cmake_minimum_required (VERSION 2.8.3)
project (dubins_traffic_examples)

find_package (catkin REQUIRED COMPONENTS rospy integrator_chains_msgs dubins_traffic_msgs)

catkin_package ()

install (FILES launch/simple.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install (PROGRAMS scripts/simple.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
