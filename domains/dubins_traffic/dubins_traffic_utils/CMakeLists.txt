cmake_minimum_required (VERSION 2.8.3)
project (dubins_traffic_utils)
add_compile_options (-std=c++11)

include_directories (include)

find_package (Boost REQUIRED COMPONENTS thread)
include_directories (${Boost_INCLUDE_DIRS})

find_package (Eigen3 REQUIRED)
include_directories (${EIGEN3_INCLUDE_DIR})

if (CATKIN_ENV)
  message (STATUS "Building for ROS")

  add_executable (monitor src/monitor.cpp)

  find_package (catkin REQUIRED dubins_traffic_msgs)
  catkin_package ()

  include_directories (include ${catkin_INCLUDE_DIRS})
  target_link_libraries (monitor ${catkin_LIBRARIES})
  install (TARGETS monitor
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install (FILES example/data/square.rnd example/data/square.json
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/example/data
  )
else (CATKIN_ENV)
  message (WARNING "Not all components can be built as standalone.")

  add_executable (helloroadnet examples/standalone/helloroadnet.cpp)
  add_executable (readrnd examples/standalone/readrnd.cpp)
endif (CATKIN_ENV)
