#ifndef PICK_AND_PLACE_H_
#define PICK_AND_PLACE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetStateValidity.h>
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <robot_pick_and_place/pick_and_place_utilities.h>
#include <handle_detector/GraspPoseCandidates.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <geometric_shapes/shape_operations.h>

// =============================== aliases ===============================
typedef actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction> GraspActionClient;
typedef boost::shared_ptr<GraspActionClient> GraspActionClientPtr;
typedef boost::shared_ptr<move_group_interface::MoveGroup> MoveGroupPtr;
typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;
typedef std::vector<move_group_interface::MoveGroup::Plan> MotionPlanArray;
typedef std::vector<moveit_msgs::RobotState> RobotStateMsgArray;

namespace robot_pick_and_place
{
	class PickAndPlace
	{
	public:
		PickAndPlace()
		{

		}

		bool init();

		void run();

		void run_test();

	protected:

		bool load_parameters();

		void move_to_wait_position();

		void set_gripper(bool do_grasp);

		void set_attached_object(bool attach,
				const geometry_msgs::Pose &pose,moveit_msgs::RobotState &robot_state);

		void reset_world(bool refresh_octomap = true);

		geometry_msgs::Pose detect_box_pick();

		geometry_msgs::Pose detect_target_poses();

		std::vector<geometry_msgs::Pose> create_pick_moves(geometry_msgs::Pose &box_pose);

		std::vector<geometry_msgs::Pose> create_place_moves();

		void pickup_box(std::vector<geometry_msgs::Pose>& pick_poses,const geometry_msgs::Pose& box_pose);

		void place_box(std::vector<geometry_msgs::Pose>& place_poses,const geometry_msgs::Pose& box_pose);


		bool create_motion_plan(const geometry_msgs::Pose &pose_target,
				const moveit_msgs::RobotState &start_robot_state,move_group_interface::MoveGroup::Plan &plan);

		void show_box(bool show=true)
		{
			ROS_WARN_STREAM("Showing marker from deprecated method");
			// updating marker action
			cfg.MARKER_MESSAGE.action =
					show ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;

			// publish messages
			marker_publisher.publish(cfg.MARKER_MESSAGE);
		}

		void show_target_on_world(bool show);

		void show_target_attached(bool show);

		void scene_update_callback(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType t);

		bool update_planning_scene();

		bool evaluate_poses(std::vector<geometry_msgs::Pose>& tcp_poses,RobotStateMsgArray &robot_states);


		// new methods

		bool get_robot_states_at_pick(RobotStateMsgArray& rs);

		bool get_robot_states_at_place(RobotStateMsgArray& rs);

		bool solve_ik(const geometry_msgs::PoseArray& tcp_poses,RobotStateMsgArray& rs);

		bool validate_states(const RobotStateMsgArray& rs);

		bool create_pick_motion_plans(const moveit_msgs::RobotState &start_state,
				const RobotStateMsgArray& pick_states,
				MotionPlanArray& mp);

		bool create_place_motion_plans(const moveit_msgs::RobotState &start_state,
				const RobotStateMsgArray& place_states,MotionPlanArray& mp);

		bool execute_pick_motion_plans(const MotionPlanArray& mp);

		bool execute_place_motion_plans(const MotionPlanArray& mp);

		bool attach_object(bool attach,const moveit_msgs::AttachedCollisionObject &att,moveit_msgs::RobotState &rs);

		bool create_motion_plan(const moveit_msgs::RobotState &start_state,
				const moveit_msgs::RobotState &end_state,move_group_interface::MoveGroup::Plan& plan);

		bool plan_all_motions();

		geometry_msgs::PoseArray create_poses_at_pick(const tf::Transform &world_to_tcp_tf);

		geometry_msgs::PoseArray create_poses_at_place(const tf::Transform &world_to_tcp_tf);


	protected:

		// parameters
		PickAndPlaceConfig cfg;

		// ros comm
		ros::Publisher marker_publisher;
		ros::Publisher planning_scene_publisher;
		ros::ServiceClient target_recognition_client;
		ros::ServiceClient planning_scene_client;
		ros::ServiceClient grasp_pose_client;
		ros::ServiceClient ik_client;
		ros::ServiceClient motion_plan_client;
		GraspActionClientPtr grasp_action_client_ptr;

		// moveit
		MoveGroupPtr move_group_ptr;
		robot_model_loader::RobotModelLoaderPtr robot_model_loader_ptr;
		moveit::core::RobotStatePtr kinematic_state_ptr;
		moveit::core::RobotModelPtr kinematic_model_ptr;
		planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_ptr_;
		planning_scene::PlanningScenePtr planning_scene_ptr;

		// motion planning
		MotionPlanArray pick_motion_plans_;
		MotionPlanArray place_motion_plans_;
		move_group_interface::MoveGroup::Plan home_motion_plan_;

		// planning support
		moveit_msgs::CollisionObject target_obj_on_world_;
		moveit_msgs::AttachedCollisionObject target_obj_attached_;


		// tf
		TransformListenerPtr transform_listener_ptr;


	};
}

#endif /* PICK_AND_PLACE_H_ */
