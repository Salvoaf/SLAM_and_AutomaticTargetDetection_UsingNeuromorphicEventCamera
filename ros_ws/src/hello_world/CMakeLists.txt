cmake_minimum_required(VERSION 3.0.2)
project(hello_world)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
)

add_message_files(
  FILES
  EventArray.msg
  Event.msg
  BoundingBoxWithTime.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/bag_to_h5_node.py
  scripts/bounding_box_publisher.py
  scripts/event_img_subscriber.py
  scripts/sync_nod.py
  scripts/generate_poses.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
