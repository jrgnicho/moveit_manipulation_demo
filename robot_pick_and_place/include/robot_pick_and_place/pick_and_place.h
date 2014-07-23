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
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <robot_pick_and_place/pick_and_place_utilities.h>
#include <robot_pick_and_place/RobotPickAndPlace.h>
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
typedef boost::shared_ptr<tf::TransformBroadcaster> TransformBroadcasterPtr;
typedef std::vector<move_group_interface::MoveGroup::Plan> MotionPlanArray;
typedef std::vector<moveit_msgs::RobotState> RobotStateMsgArray;
typedef std::vector<moveit_msgs::CollisionObject> CollisionObjectArray;
typedef std::vector<moveit_msgs::CollisionObject>::iterator CollisionObjectIterator;
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

		void run_as_service();

	protected:

		bool load_parameters();

		void move_to_wait_position();

		void set_gripper(bool do_grasp);


		bool create_motion_plan(const geometry_msgs::Pose &pose_target,
				const moveit_msgs::RobotState &start_robot_state,move_group_interface::MoveGroup::Plan &plan);

		bool create_motion_plan(const moveit_msgs::RobotState &start_state,
				const moveit_msgs::RobotState &end_state,move_group_interface::MoveGroup::Plan& plan);

		bool create_cartesian_plan(const geometry_msgs::Pose &tcp_start,
				const geometry_msgs::Pose &tcp_end,double eef_step,move_group_interface::MoveGroup::Plan& plan);

		void show_target_at_pick(bool show);

		void show_target_at_place(bool show);

		void show_target_attached(bool show);

		void scene_update_callback(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType t);

		bool update_planning_scene();

		bool get_grasp_candidates(handle_detector::GraspPoseCandidates::Response& grasp_candidates);

		bool get_robot_states_at_pick(const handle_detector::GraspPoseCandidates::Response& grasp_candidates,
				RobotStateMsgArray& rs);

		bool get_robot_states_at_place(RobotStateMsgArray& rs);

		bool solve_ik(const geometry_msgs::PoseArray& tcp_poses,RobotStateMsgArray& rs,int attempts = 10, double duration=0.1f);

		bool validate_states(const RobotStateMsgArray& rs);

		bool create_pick_motion_plans(const moveit_msgs::RobotState &start_state,
				const RobotStateMsgArray& pick_states,
				MotionPlanArray& mp);

		bool create_place_motion_plans(const moveit_msgs::RobotState &start_state,
				const RobotStateMsgArray& place_states,MotionPlanArray& mp);

		bool execute_pick_motion_plans(const MotionPlanArray& mp);

		bool execute_place_motion_plans(const MotionPlanArray& mp);

		bool attach_object(bool attach,const moveit_msgs::AttachedCollisionObject &att,moveit_msgs::RobotState &rs);

		bool plan_all_motions();

		geometry_msgs::PoseArray create_poses_at_pick(const tf::Transform &world_to_tcp_tf);

		geometry_msgs::PoseArray create_poses_at_place(const tf::Transform &world_to_tcp_tf);

		void add_obstacles_to_planning_scene(const CollisionObjectArray &obstacles,bool add);

		void add_obstacle_to_planning_scene(const moveit_msgs::CollisionObject &obstacle,bool add);

		void remove_obstacle_from_planning_scene(const moveit_msgs::CollisionObject &obstacle);

		void reset_planning_scene();

		void publish_planning_scene(bool diff = false);

		void broadcast_tcp_candidate(const geometry_msgs::Pose& world_to_tcp_pose);

		bool pick_and_place_server_callback(robot_pick_and_place::RobotPickAndPlace::Request &req,
				robot_pick_and_place::RobotPickAndPlace::Response &res);


	protected:

		// parameters
		PickAndPlaceConfig cfg;

		// ros comm
		ros::Publisher marker_publisher;
		ros::Publisher planning_scene_publisher;
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
		planning_scene::PlanningScenePtr planning_scene_ptr;

		// motion planning
		MotionPlanArray pick_motion_plans_;
		MotionPlanArray place_motion_plans_;
		move_group_interface::MoveGroup::Plan home_motion_plan_;

		// planning support
		CollisionObjectArray obstacles_;
		//std::map<std::string,moveit_msgs::CollisionObject> world_obstacles_map_;
		//moveit_msgs::CollisionObject world_obstacles_;
		moveit_msgs::CollisionObject target_obj_on_world_;
		moveit_msgs::AttachedCollisionObject target_obj_attached_;
		geometry_msgs::PoseArray tcp_pick_poses_;
		geometry_msgs::PoseArray tcp_place_poses_;

		// visualization
		visualization_msgs::Marker target_marker_;

		// tf
		TransformListenerPtr transform_listener_ptr;
		TransformBroadcasterPtr transform_broadcast_ptr_;



	};
}

#endif /* PICK_AND_PLACE_H_ */
