cmake_minimum_required (VERSION 2.8.3)
project (dubins_traffic_utils)
add_compile_options (-std=c++11)

include_directories (include)

find_package (Eigen3 REQUIRED)
include_directories (${EIGEN3_INCLUDE_DIR})

if (CATKIN_ENV)
  message (STATUS "Building for ROS")

  find_package (catkin REQUIRED)
  catkin_package ()

  install (FILES example/data/square.rnd example/data/square.json
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/example/data
  )
else (CATKIN_ENV)
  message (WARNING "Not all components can be built as standalone.")

  add_executable (helloroadnet examples/standalone/helloroadnet.cpp)
endif (CATKIN_ENV)
