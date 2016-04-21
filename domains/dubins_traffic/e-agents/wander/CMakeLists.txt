cmake_minimum_required (VERSION 2.8.3)
project (wander)
add_compile_options (-std=c++11)

find_package (catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  sensor_msgs
  dubins_traffic_utils
)

find_package (Eigen3 REQUIRED)
include_directories (${EIGEN3_INCLUDE_DIR})

catkin_package ()

include_directories (
  ${dubins_traffic_utils_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable (wander src/wander.cpp)
target_link_libraries (wander ${catkin_LIBRARIES})

install (DIRECTORY launch
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install (TARGETS wander
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
