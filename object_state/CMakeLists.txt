cmake_minimum_required(VERSION 2.8.3)
project(object_state)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosprolog
  json_prolog_msgs
  knowrob_common
  rosjava_build_tools
  rosjava_messages
  rospy
  suturo_perception_msgs
)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_state
#  CATKIN_DEPENDS geometry_msgs rosprolog json_prolog_msgs knowrob_common rosjava_build_tools rosjava_messages rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


