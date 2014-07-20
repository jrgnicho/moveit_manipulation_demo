#ifndef PICK_AND_PLACE_UTILITIES_H_
#define PICK_AND_PLACE_UTILITIES_H_

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/Constraints.h>
#include <visualization_msgs/Marker.h>
#include <robot_pick_and_place/GetTargetPose.h>
#include <XmlRpc.h>

namespace robot_pick_and_place{

std::vector<geometry_msgs::Pose> create_manipulation_poses(double retreat_dis,
		double approach_dis,const tf::Transform &target_tf);

std::vector<geometry_msgs::Pose> transform_from_tcp_to_wrist(tf::Transform tcp_to_wrist_tf,
		const std::vector<geometry_msgs::Pose> tcp_poses);

std::ostream& operator<<(std::ostream& os, const tf::Vector3 vec);
std::ostream& operator<<(std::ostream& os, const geometry_msgs::Point pt);

moveit_msgs::Constraints create_path_orientation_constraints(const geometry_msgs::Pose &goal_pose,
		float x_tolerance,float y_tolerance,float z_tolerance,std::string link_name);

bool parse_pose_parameter(XmlRpc::XmlRpcValue pose_param,geometry_msgs::Pose &pose);

bool parse_pose_parameter(XmlRpc::XmlRpcValue pose_param,tf::Transform &t);

class PickAndPlaceConfig
{
public:

	// =============================== Parameters ===============================
  std::string ARM_GROUP_NAME;  // MoveIt Planning Group associated with the robot arm
  std::string TCP_LINK_NAME;   // Link / frame name for the suction gripper tool-tip
  std::string WRIST_LINK_NAME; // Link / frame name for the robot wrist tool-flange
  std::string ATTACHED_OBJECT_LINK_NAME; // attached object link in robot
  std::string WORLD_FRAME_ID;  // Frame name for the fixed world reference frame
  std::string HOME_POSE_NAME;  // Named pose for robot Home position (set in SRDF)
  std::string WAIT_POSE_NAME;  // Named pose for robot WAIT position (set in SRDF)
  std::vector<std::string> TOUCH_LINKS; // List of links that the attached payload is allow to be in contact with
  double RETREAT_DISTANCE;     // Distance to back away from pick/place pose after grasp/release
  double APPROACH_DISTANCE;    // Distance to stand off from pick/place pose before grasp/release
  double GRIPPER_WORKRANGE;    // maximum allowed gripper workrange

  // =============================== Topic, services and action names ===============================
  std::string GRASP_ACTION_NAME;  // Action name used to control suction gripper
  std::string MARKER_TOPIC; // Topic for publishing visualization of attached object.
  std::string PLANNING_SCENE_TOPIC; // Topic for publishing the planning scene
  std::string GRASP_POSES_SERVICE; // service for grasp pose
  std::string IK_SERVICE; // service for requesting ik
  std::string MOTION_PLAN_SERVICE; // service for requesting moveit for a motion plan
  std::string PICK_AND_PLACE_SERVICE;

  // =============================== Messages and variables ===============================
  geometry_msgs::Pose TCP_TO_TARGET_POSE;
  geometry_msgs::Pose WORLD_TO_PLACE_POSE;

  PickAndPlaceConfig()
  {
    ARM_GROUP_NAME  = "manipulator";
    TCP_LINK_NAME   = "tcp_frame";
    MARKER_TOPIC = "pick_and_place_marker";
    PLANNING_SCENE_TOPIC = "planning_scene";
    GRASP_POSES_SERVICE = "grasp_poses";
    IK_SERVICE = "compute_ik";
    MOTION_PLAN_SERVICE = "plan_kinematic_path";
    PICK_AND_PLACE_SERVICE = "robot_pick_and_place";
    WRIST_LINK_NAME = "ee_link";
    ATTACHED_OBJECT_LINK_NAME = "attached_object_link";
    WORLD_FRAME_ID  = "world_frame";
    HOME_POSE_NAME  = "home";
    WAIT_POSE_NAME  = "wait";
    GRASP_ACTION_NAME = "grasp_execution_action";
    TOUCH_LINKS = std::vector<std::string>();
    RETREAT_DISTANCE  = 0.05f;
    APPROACH_DISTANCE = 0.05f;
  }

  bool init();
};
}

#endif /* PICK_AND_PLACE_UTILITIES_H_ */
