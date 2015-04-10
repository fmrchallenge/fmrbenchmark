cmake_minimum_required (VERSION 2.8.3)
project (dynamaestro)

include_directories (include)

find_package (Boost REQUIRED COMPONENTS thread)
include_directories (${Boost_INCLUDE_DIRS})

find_package (Eigen3 REQUIRED)
include_directories (${EIGEN3_INCLUDE_DIR})

if (CATKIN_ENV)
  message (STATUS "Building for ROS")

  add_executable (dm src/main.cpp)
  target_link_libraries (dm ${Boost_LIBRARIES})

  add_definitions (-DUSE_ROS)
  find_package (catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

  add_message_files (DIRECTORY msg FILES VectorStamped.msg)
  add_service_files (DIRECTORY srv FILES DMMode.srv)
  generate_messages (DEPENDENCIES std_msgs)

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
  message (WARNING "Not all components can be built as standalone.")

  add_executable (hellopolytope examples/standalone/hellopolytope.cpp)
endif (CATKIN_ENV)
