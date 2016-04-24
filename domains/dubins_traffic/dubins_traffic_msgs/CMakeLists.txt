cmake_minimum_required (VERSION 2.8.3)
project (dubins_traffic_msgs)

find_package (catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files (DIRECTORY msg FILES
  LabelStamped.msg RoadNetworkJSON.msg
)
generate_messages (DEPENDENCIES std_msgs)

catkin_package (CATKIN_DEPENDS message_runtime std_msgs)
