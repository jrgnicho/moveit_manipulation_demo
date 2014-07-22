cmake_minimum_required(VERSION 2.8.3)
project(robot_pick_and_place)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  moveit_msgs
  moveit_ros_manipulation
  moveit_ros_move_group
  moveit_ros_planning_interface
  robot_io
  object_manipulation_msgs
  actionlib
  industrial_robot_client
  industrial_robot_simulator
  std_msgs
  shape_msgs
  pcl_ros
  pcl_msgs
  tf_conversions
	message_generation
	handle_detector
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

#######################################
## Declare ROS messages and services ##
#######################################
## Generate services in the 'srv' folder
add_service_files(
   FILES
   GetTargetPose.srv
	 RobotPickAndPlace.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES   
		std_msgs
		shape_msgs
		pcl_msgs
		visualization_msgs
		moveit_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
	CATKIN_DEPENDS
		roscpp
		moveit_msgs
		moveit_ros_manipulation
		moveit_ros_move_group
		moveit_ros_planning_interface
		robot_io
		object_manipulation_msgs
		actionlib
		industrial_robot_client
		industrial_robot_simulator
		std_msgs
		shape_msgs
		pcl_ros
		pcl_msgs
		tf_conversions
		message_runtime
)

###########
## Build ##
###########

#add_executable(grasp_pose_service src/services/grasp_pose_service.cpp)
#add_dependencies(grasp_pose_service ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(grasp_pose_service ${catkin_LIBRARIES})

#add_executable(target_recognition_service src/services/target_recognition_service.cpp)
#add_dependencies(target_recognition_service ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(target_recognition_service ${catkin_LIBRARIES})

add_executable(pick_and_place_node 
	src/nodes/pick_and_place_node.cpp
	src/pick_and_place.cpp
	src/utilities/pick_and_place_utilities.cpp)
add_dependencies(pick_and_place_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(pick_and_place_node ${catkin_LIBRARIES})

add_executable(pick_and_place_service 
	src/services/pick_and_place_service.cpp
	src/pick_and_place.cpp
	src/utilities/pick_and_place_utilities.cpp)
add_dependencies(pick_and_place_service ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(pick_and_place_service ${catkin_LIBRARIES})


	
