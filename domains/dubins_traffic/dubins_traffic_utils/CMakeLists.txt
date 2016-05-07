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
  add_executable (maestro src/maestro.cpp)

  find_package (catkin REQUIRED roscpp geometry_msgs gazebo_msgs std_srvs dubins_traffic_msgs)
  catkin_package (INCLUDE_DIRS include)

  include_directories (include ${catkin_INCLUDE_DIRS})
  target_link_libraries (monitor ${catkin_LIBRARIES})
  target_link_libraries (maestro ${catkin_LIBRARIES})
  install (TARGETS monitor maestro
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install (DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
  )

  install (FILES examples/data/square.rnd examples/data/square.json
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples/data
  )
  install (DIRECTORY examples/trialsconf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
    )
  install (DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
  )
else (CATKIN_ENV)
  message (WARNING "Not all components can be built as standalone.")

  add_executable (helloroadnet examples/standalone/helloroadnet.cpp)
  add_executable (readrnd examples/standalone/readrnd.cpp)
endif (CATKIN_ENV)
