cmake_minimum_required (VERSION 2.8.3)
project (integrator_chains_msgs)

find_package (catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files (DIRECTORY msg FILES
  VectorStamped.msg Vector.msg LabelStamped.msg
  ProblemInstanceJSON.msg
)
add_service_files (DIRECTORY srv FILES DMMode.srv)
generate_messages (DEPENDENCIES std_msgs)

catkin_package ()
