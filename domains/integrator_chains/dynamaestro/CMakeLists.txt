cmake_minimum_required (VERSION 2.8.3)
project (dynamaestro)

include_directories (include)
add_executable (dm src/main.cpp)
target_link_libraries (dm pthread)

if (CATKIN_ENV)
  message (STATUS "Building for ROS")
  add_definitions (-DUSE_ROS)
  find_package (catkin REQUIRED COMPONENTS roscpp geometry_msgs)
  catkin_package ()
  include_directories (include ${catkin_INCLUDE_DIRS})
  target_link_libraries (dm ${catkin_LIBRARIES})
  install (TARGETS dm
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install (FILES launch/trivial-demo.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
else (CATKIN_ENV)
  message (FATAL_ERROR "Building as standalone is not yet available.")
endif (CATKIN_ENV)
