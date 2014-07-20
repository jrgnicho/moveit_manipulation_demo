/*
 * pick_and_place.cpp
 *
 *  Created on: Jun 30, 2014
 *      Author: ros developer 
 */

#include <robot_pick_and_place/pick_and_place.h>

namespace robot_pick_and_place
{

bool PickAndPlace::init()
{
	using namespace moveit::core;

	  // reading parameters
	  if(load_parameters())
	  {
	    ROS_INFO_STREAM("Parameters successfully read");
	  }
	  else
	  {
	    ROS_ERROR_STREAM("Parameters not found");
	    return false;
	  }

	  ros::NodeHandle nh;

	  // marker publisher
	  marker_publisher = nh.advertise<visualization_msgs::Marker>(
			  cfg.MARKER_TOPIC,1);

	  // planning scene publisher
	  planning_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>(
	  		cfg.PLANNING_SCENE_TOPIC,1);

	  // moveit interface
	  move_group_ptr = MoveGroupPtr(
			  new move_group_interface::MoveGroup(cfg.ARM_GROUP_NAME));

	  // motion plan client
	  motion_plan_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(cfg.MOTION_PLAN_SERVICE);

	  // transform listener
	  transform_listener_ptr = TransformListenerPtr(new tf::TransformListener());
	  transform_broadcast_ptr_ = TransformBroadcasterPtr(new tf::TransformBroadcaster());

	  // marker publisher (rviz visualization)
	  marker_publisher = nh.advertise<visualization_msgs::Marker>(
			  cfg.MARKER_TOPIC,1);

	  grasp_pose_client = nh.serviceClient<handle_detector::GraspPoseCandidates>(
			  cfg.GRASP_POSES_SERVICE);

	  // grasp action client (vacuum gripper)
	  grasp_action_client_ptr = GraspActionClientPtr(
			  new GraspActionClient(cfg.GRASP_ACTION_NAME,true));

	  // ik client (inverse kinematics)
	  ik_client = nh.serviceClient<moveit_msgs::GetPositionIK>(cfg.IK_SERVICE);

	  // ik solver
	  robot_model_loader_ptr = robot_model_loader::RobotModelLoaderPtr(
			  new robot_model_loader::RobotModelLoader("robot_description"));
	  kinematic_model_ptr = robot_model_loader_ptr->getModel();
	  kinematic_state_ptr = RobotStatePtr(new RobotState(kinematic_model_ptr));

	  // planning scene client
	  planning_scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

	  // initializing local planning scene
	  planning_scene_ptr = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(kinematic_model_ptr));
	  update_planning_scene();


	  // waiting to establish connections
	  ROS_INFO_STREAM("Connecting to services and actions servers");
	  if(ros::ok()
	  	  && grasp_action_client_ptr->waitForServer(ros::Duration(10.0f))
			  && grasp_pose_client.waitForExistence(ros::Duration(10.0f))
			  && ik_client.waitForExistence(ros::Duration(10.0f))
			  && planning_scene_client.waitForExistence(ros::Duration(10.0f)))
	  {
		  ROS_INFO_STREAM("Connected services and action servers");
		  return true;
	  }
	  else
	  {
		  ROS_ERROR_STREAM("Connections to services and actions not found");
	  }

	  return true;
}

bool PickAndPlace::load_parameters()
{
	  return cfg.init();
}

void PickAndPlace::run()
{
	// move to a "clear" position
	move_to_wait_position();

	if(plan_all_motions())
	{
		if(execute_pick_motion_plans(pick_motion_plans_) &&
				execute_place_motion_plans(place_motion_plans_) &&
				move_group_ptr->execute(home_motion_plan_))
		{
			show_target_at_place(false);
			ROS_INFO_STREAM("Pick and place motions completed");
		}
		else
		{
			ROS_INFO_STREAM("Pick and place motions failed");
		}
	}
	else
	{
		ROS_ERROR_STREAM("Motion planning failed");
	}
}

void PickAndPlace::run_as_service()
{
	ros::NodeHandle nh;


	ros::ServiceServer pick_and_place_server = nh.advertiseService(
			cfg.PICK_AND_PLACE_SERVICE,
			&PickAndPlace::pick_and_place_server_callback,this);

	ros::spin();
	ros::waitForShutdown();

}

bool PickAndPlace::pick_and_place_server_callback(
		robot_pick_and_place::RobotPickAndPlace::Request &req,
				robot_pick_and_place::RobotPickAndPlace::Response &res)
{

	while(ros::ok())
	{
		handle_detector::GraspPoseCandidates::Response grasp_candidates;
		RobotStateMsgArray pick_states;
		RobotStateMsgArray place_states;
		moveit_msgs::RobotState pick_start_state;
		moveit_msgs::RobotState place_start_state;
		moveit_msgs::RobotState start_to_wait_state,end_to_wait_state;

		// clear results
		pick_motion_plans_.clear();
		place_motion_plans_.clear();
		home_motion_plan_ = move_group_interface::MoveGroup::Plan();

		// moving home
		move_to_wait_position();

		update_planning_scene();
		reset_planning_scene();
		publish_planning_scene();

		// request grasp candidates
		if(!get_grasp_candidates(grasp_candidates))
		{
			ROS_ERROR_STREAM("Grasp candidates not available, exiting");
			break;
		}


		ROS_INFO_STREAM("Evaluating candidate pick poses");
		if(!get_robot_states_at_pick(grasp_candidates,pick_states) )
		{
			ROS_ERROR_STREAM("creation of robot states at pick failed");
			continue;
		}

		ROS_INFO_STREAM("Evaluating place poses");
		if(!get_robot_states_at_place(place_states))
		{
			ROS_ERROR_STREAM("creation of robot states at place failed");
			continue;
		}

		// creating pick motion plans
		ROS_INFO_STREAM("Creating pick motion plans");
		robot_state::robotStateToRobotStateMsg(*move_group_ptr->getCurrentState(),pick_start_state);
		if(!create_pick_motion_plans(pick_start_state,pick_states,pick_motion_plans_))
		{
			ROS_ERROR_STREAM("creation of pick motion plans failed");
			continue;
		}

		// creating place motion plans
		ROS_INFO_STREAM("Creating place motion plans");
		place_start_state = moveit_msgs::RobotState(pick_states.back());
		if(!create_place_motion_plans(place_start_state,place_states,place_motion_plans_))
		{
			ROS_ERROR_STREAM("creation of place motion plans failed");
			continue;
		}

		// create motion plan to go back to home
		// planning move to wait pose
		if(move_group_ptr->setNamedTarget(cfg.WAIT_POSE_NAME))
		{
			start_to_wait_state = place_states.back();
			attach_object(false,target_obj_attached_,start_to_wait_state);
			robot_state::robotStateToRobotStateMsg(move_group_ptr->getJointValueTarget(),end_to_wait_state);
		}
		else
		{
			ROS_ERROR_STREAM("could not set joint target: '"<<cfg.WAIT_POSE_NAME<<"'");
			continue;
		}

		if(!create_motion_plan(place_states.back(),end_to_wait_state,home_motion_plan_))
		{
			ROS_ERROR_STREAM("creation of home motion plan failed");
			continue;
		}

		// removing obstacles from planning scene
		add_obstacles_to_planning_scene(obstacles_,false);
		//planning_scene_ptr->setCurrentState(end_to_wait_state);
		publish_planning_scene();

		// run all motion plans
		if(execute_pick_motion_plans(pick_motion_plans_) &&
				execute_place_motion_plans(place_motion_plans_) &&
				move_group_ptr->execute(home_motion_plan_))
		{
			show_target_at_place(false);
			move_group_ptr->setStartState(end_to_wait_state);

			ROS_INFO_STREAM("Pick and place motions completed");
		}
		else
		{
			ROS_INFO_STREAM("Pick and place motions failed");

			move_group_ptr->setStartState(end_to_wait_state);
			continue;
		}


		res.pick_and_place_completed++;

	}

	return true;
}

bool PickAndPlace::plan_all_motions()
{

	handle_detector::GraspPoseCandidates::Response grasp_candidates;
	RobotStateMsgArray pick_states;
	RobotStateMsgArray place_states;
	moveit_msgs::RobotState pick_start_state;
	moveit_msgs::RobotState place_start_state;
	moveit_msgs::RobotState start_to_wait_state,end_to_wait_state;

	update_planning_scene();
	reset_planning_scene();
	publish_planning_scene();

	// request grasp candidates
	if(!get_grasp_candidates(grasp_candidates))
	{
		ROS_ERROR_STREAM("Grasp candidates not available, exiting");
		return false;
	}


	ROS_INFO_STREAM("Evaluating candidate pick poses");
	if(!get_robot_states_at_pick(grasp_candidates,pick_states) )
	{
		ROS_ERROR_STREAM("creation of robot states at pick failed");
		return false;
	}

	ROS_INFO_STREAM("Evaluating place poses");
	if(!get_robot_states_at_place(place_states))
	{
		ROS_ERROR_STREAM("creation of robot states at place failed");
		return false;
	}

	// creating pick motion plans
	ROS_INFO_STREAM("Creating pick motion plans");
	robot_state::robotStateToRobotStateMsg(*move_group_ptr->getCurrentState(),pick_start_state);
	if(!create_pick_motion_plans(pick_start_state,pick_states,pick_motion_plans_))
	{
		ROS_ERROR_STREAM("creation of pick motion plans failed");
		return false;
	}

	// creating place motion plans
	ROS_INFO_STREAM("Creating place motion plans");
	place_start_state = moveit_msgs::RobotState(pick_states.back());
	if(!create_place_motion_plans(place_start_state,place_states,place_motion_plans_))
	{
		ROS_ERROR_STREAM("creation of place motion plans failed");
		return false;
	}

	// create motion plan to go back to home
	// planning move to wait pose
	if(move_group_ptr->setNamedTarget(cfg.WAIT_POSE_NAME))
	{
		start_to_wait_state = place_states.back();
		attach_object(false,target_obj_attached_,start_to_wait_state);
		robot_state::robotStateToRobotStateMsg(move_group_ptr->getJointValueTarget(),end_to_wait_state);
	}
	else
	{
		ROS_ERROR_STREAM("could not set joint target: '"<<cfg.WAIT_POSE_NAME<<"'");
		return false;
	}

	if(!create_motion_plan(place_states.back(),end_to_wait_state,home_motion_plan_))
	{
		ROS_ERROR_STREAM("creation of home motion plan failed");
		return false;
	}

	// removing obstacles from planning scene
	add_obstacles_to_planning_scene(obstacles_,false);
	publish_planning_scene();

	return true;
}

bool PickAndPlace::create_motion_plan(const moveit_msgs::RobotState &start_state,
				const moveit_msgs::RobotState &end_state,move_group_interface::MoveGroup::Plan& plan)
{
	// creating motion plan request
	moveit_msgs::GetMotionPlan motion_plan;
	moveit_msgs::MotionPlanRequest &req = motion_plan.request.motion_plan_request;
	moveit_msgs::MotionPlanResponse &res = motion_plan.response.motion_plan_response;

	// updating internal robot state
	robot_state::robotStateMsgToRobotState(end_state,*kinematic_state_ptr);

	// creating goal
	moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(*kinematic_state_ptr,
			kinematic_state_ptr->getJointModelGroup(cfg.ARM_GROUP_NAME),0.01f,0.01f);

	// completing planning request
	req.planner_id="RRTConnectkConfigDefault";
	req.start_state = start_state;
	req.start_state.is_diff = true;
	req.group_name = cfg.ARM_GROUP_NAME;
	req.goal_constraints.push_back(joint_goal);
	req.allowed_planning_time = 30.0f;
	req.num_planning_attempts = 1;

	// call planner
	bool success = false;
	if(motion_plan_client.call(motion_plan) && res.error_code.val == res.error_code.SUCCESS)
	{
		// saving motion plan results
		plan.start_state_ = res.trajectory_start;
		plan.trajectory_ = res.trajectory;
		success = true;
	}

	return success;
}

bool PickAndPlace::create_motion_plan(const geometry_msgs::Pose &pose_target,
		const moveit_msgs::RobotState &start_robot_state,move_group_interface::MoveGroup::Plan &plan)
{
	// constructing motion plan goal constraints
	std::vector<double> position_tolerances(3,0.01f);
	std::vector<double> orientation_tolerances(3,0.01f);
	geometry_msgs::PoseStamped p;
	p.header.frame_id = cfg.WORLD_FRAME_ID;
	p.pose = pose_target;
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(cfg.WRIST_LINK_NAME,p,position_tolerances,
			orientation_tolerances);

	// creating motion plan request
	moveit_msgs::GetMotionPlan motion_plan;
	moveit_msgs::MotionPlanRequest &req = motion_plan.request.motion_plan_request;
	moveit_msgs::MotionPlanResponse &res = motion_plan.response.motion_plan_response;
	req.start_state = start_robot_state;
	req.start_state.is_diff = true;
	req.group_name = cfg.ARM_GROUP_NAME;
	req.goal_constraints.push_back(pose_goal);
	req.allowed_planning_time = 60.0f;
	req.num_planning_attempts = 1;

	// request motion plan
	bool success = false;
	if(motion_plan_client.call(motion_plan) && res.error_code.val == res.error_code.SUCCESS)
	{
		// saving motion plan results
		plan.start_state_ = res.trajectory_start;
		plan.trajectory_ = res.trajectory;
		success = true;
	}

	return success;
}

bool PickAndPlace::get_grasp_candidates(handle_detector::GraspPoseCandidates::Response& grasp_candidates)
{
	using namespace robot_pick_and_place;
	using namespace handle_detector;

	ROS_INFO_STREAM("requesting grasp candidates");

	// grasp candidates request
	GraspPoseCandidates gp;
	gp.request.candidates_per_pose = 6;
	gp.request.gripper_workrange=cfg.GRIPPER_WORKRANGE;
	gp.request.planning_frame_id = cfg.WORLD_FRAME_ID;

	if(grasp_pose_client.call(gp))
	{
		grasp_candidates = gp.response;
		ROS_INFO_STREAM("received "<<grasp_candidates.candidate_collision_objects.size()<<" targets with "<<
				grasp_candidates.candidate_grasp_poses[0].poses.size()<<" candidate grasp candidates each");
		return true;
	}
	else
	{
		ROS_ERROR_STREAM("received 0 grasp candidates");
		return false;
	}
}

bool PickAndPlace::get_robot_states_at_pick(const handle_detector::GraspPoseCandidates::Response& grasp_candidates,
		RobotStateMsgArray& rs)
{
	using namespace robot_pick_and_place;
	using namespace handle_detector;

	// collision objects
	moveit_msgs::CollisionObject col;
	moveit_msgs::AttachedCollisionObject att;
	att.link_name = cfg.TCP_LINK_NAME;
	att.touch_links = cfg.TOUCH_LINKS;

	// finding gripper pose at pick
	geometry_msgs::PoseArray tcp_poses;
	geometry_msgs::Pose world_to_target_pose, world_to_tcp_pose;
	tf::Transform world_to_tcp_tf, world_to_target_tf;
	bool found_valid_poses = false;

	// update scene
	update_planning_scene();

	// saving all graspable objects detected for collision avoidance
	obstacles_ = grasp_candidates.candidate_collision_objects;


	for(int i = 0; i <grasp_candidates.candidate_grasp_poses.size();i++)
	{
		const std::vector<geometry_msgs::Pose> &poses = grasp_candidates.candidate_grasp_poses[i].poses;
		CollisionObjectArray current_obs = grasp_candidates.candidate_collision_objects;
		current_obs.erase(current_obs.begin()+i);

		// adding obstacles to planning scene not including target
		add_obstacles_to_planning_scene(obstacles_,false);
		add_obstacles_to_planning_scene(current_obs,true);

		//marker_publisher.publish(grasp_candidates.candidate_objects.markers[i]);

		ROS_INFO_STREAM("Evaluating "<<poses.size()<<" candidate poses");

		for(int j = 0;j < poses.size();j++)
		{
			// creating tcp pose from target pose
			const geometry_msgs::Pose &p = poses[j];
			tf::poseMsgToTF(p,world_to_target_tf);
			world_to_tcp_tf.setRotation(world_to_target_tf.getRotation()*
					tf::Quaternion(tf::Vector3(1,0,0),M_PI));// inverting z vector
			world_to_tcp_tf.setOrigin(world_to_target_tf.getOrigin());
			tf::poseTFToMsg(world_to_tcp_tf,world_to_tcp_pose);

			// creating pick poses for tcp
			tcp_poses = create_poses_at_pick(world_to_tcp_tf);

			// shows current candidate pose in rviz
			broadcast_tcp_candidate(tcp_poses.poses[1]);
			rs.clear();

			if(solve_ik(tcp_poses,rs))
			{
				// attach and evaluate
				ROS_INFO_STREAM("Ik solution found "<<rs.size() <<" poses with pick at:\n"
						<<tcp_poses.poses[1]);

				//att.object = grasp_candidates.candidate_collision_objects[i];
				//attach_object(true,att,rs[2]);
			}
			else
			{
				ROS_ERROR_STREAM("Ik not found for pick poses "<<j);
				continue;
			}

			//publish_planning_scene();
			RobotStateMsgArray rstempts;
			rstempts.push_back(rs[0]);
			rstempts.push_back(rs[2]);
			if(validate_states(rstempts))
			{
				// saving results
				target_obj_on_world_= grasp_candidates.candidate_collision_objects[i];
				target_obj_attached_.object = grasp_candidates.candidate_collision_objects[i];
				target_marker_= grasp_candidates.candidate_objects.markers[i];
				obstacles_.erase(obstacles_.begin()+i); // removing target from obstacles
				found_valid_poses = true;


				//ROS_INFO_STREAM("Found valid tcp grasp poses at index "<< j <<" with grasp pose at:\n"<<world_to_tcp_pose);
				ROS_INFO_STREAM("Found valid tcp grasp poses at index "<< j );
				break;
			}

		}



		if(found_valid_poses)
		{
			break;
		}
	}

	if(!found_valid_poses)
	{
		ROS_ERROR_STREAM("Reachable grasp pose not found, exiting");
	}


	return found_valid_poses;

}

bool PickAndPlace::get_robot_states_at_place(RobotStateMsgArray& rs)
{
	// creating tcp place poses
	tf::Transform world_to_tcp_tf,world_to_target_tf,tcp_to_target_tf;
	geometry_msgs::PoseArray tcp_place_poses;

	tf::poseMsgToTF(cfg.WORLD_TO_PLACE_POSE,world_to_target_tf);
	tf::poseMsgToTF(cfg.TCP_TO_TARGET_POSE,tcp_to_target_tf);

	world_to_tcp_tf = world_to_target_tf*(tcp_to_target_tf.inverse());

	tcp_place_poses = create_poses_at_place(world_to_tcp_tf);

	//ROS_INFO_STREAM("Planning for place pose:\n"<<tcp_place_poses);

	// adding obstacles to planning scene
	add_obstacles_to_planning_scene(obstacles_,true);

	rs.clear();
	if(solve_ik(tcp_place_poses,rs,20,0.5f))
	{
		// attach collision object
		attach_object(true,target_obj_attached_,rs[0]);
		attach_object(true,target_obj_attached_,rs[1]);
		attach_object(false,target_obj_attached_,rs[2]);
	}
	else
	{
		ROS_ERROR_STREAM("Ik not found for place poses");
		return false;
	}

	if(!validate_states(rs))
	{
		ROS_ERROR_STREAM("Invalid state for place poses");
		return false;
	}

	return true;
}

bool PickAndPlace::solve_ik(const geometry_msgs::PoseArray& tcp_poses,RobotStateMsgArray& rs,int attempts, double duration)
{
	// ik request
	moveit_msgs::GetPositionIK ik;
	ik.request.ik_request.group_name = cfg.ARM_GROUP_NAME;
	ik.request.ik_request.pose_stamped.header.frame_id = cfg.WORLD_FRAME_ID;
	ik.request.ik_request.ik_link_name = cfg.TCP_LINK_NAME;
	ik.request.ik_request.attempts= attempts;
	ik.request.ik_request.timeout = ros::Duration(duration);
	ik.request.ik_request.avoid_collisions = true;
	ik.request.ik_request.robot_state.is_diff=true;


	bool all_reachable = true;
	for(int i = 0;i < tcp_poses.poses.size();i++)
	{
		// checking ik solution
		ik.request.ik_request.pose_stamped.pose = tcp_poses.poses[i];
		if(ik_client.call(ik) && (ik.response.error_code.val == ik.response.error_code.SUCCESS))
		{
			//ROS_INFO_STREAM("IK solution found at pose "<<i);
			ik.response.solution.attached_collision_objects.clear();
			rs.push_back(ik.response.solution);
		}
		else
		{
			ROS_WARN_STREAM("IK not available at pose "<<i<<" with code "<<ik.response.error_code.val<<"\n:"
					<<tcp_poses.poses[i]);
			all_reachable = false;
			break;
		}

		//ROS_INFO_STREAM("Pose "<<i<<" is reachable and collision free");
	}

	return all_reachable;
}

geometry_msgs::PoseArray PickAndPlace::create_poses_at_pick(const tf::Transform &world_to_tcp_tf)
{
  geometry_msgs::Pose start_pose, target_pose, end_pose;
  geometry_msgs::PoseArray poses;
  double approach_dis = cfg.APPROACH_DISTANCE;
  double retreat_dis = cfg.RETREAT_DISTANCE;

  // creating start pose by applying a translation along +z by approach distance
  //tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,approach_dis))*world_to_tcp_tf,start_pose);
  tf::poseTFToMsg(world_to_tcp_tf*tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(0,0,-approach_dis)),start_pose);

  // converting target pose
  tf::poseTFToMsg(world_to_tcp_tf,target_pose);

  // creating end pose by applying a translation along +z by retreat distance
  tf::poseTFToMsg(tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(0,0,retreat_dis))*world_to_tcp_tf,end_pose);
  //tf::poseTFToMsg(world_to_tcp_tf*Transform(Quaternion::getIdentity(),Vector3(0,0,-retreat_dis)),end_pose);

  poses.poses.clear();
  poses.poses.push_back(start_pose);
  poses.poses.push_back(target_pose);
  poses.poses.push_back(end_pose);

  return poses;
}

geometry_msgs::PoseArray PickAndPlace::create_poses_at_place(const tf::Transform &world_to_tcp_tf)
{
  geometry_msgs::Pose start_pose, target_pose, end_pose;
  geometry_msgs::PoseArray poses;
  double approach_dis = cfg.APPROACH_DISTANCE;
  double retreat_dis = cfg.RETREAT_DISTANCE;

  // creating start pose by applying a translation along +z by approach distance
  tf::poseTFToMsg(tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(0,0,approach_dis))*world_to_tcp_tf,start_pose);
  //tf::poseTFToMsg(world_to_tcp_tf*Transform(Quaternion::getIdentity(),Vector3(0,0,-approach_dis)),start_pose);

  // converting target pose
  tf::poseTFToMsg(world_to_tcp_tf,target_pose);

  // creating end pose by applying a translation along +z by retreat distance
  tf::poseTFToMsg(tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(0,0,retreat_dis))*world_to_tcp_tf,end_pose);
  //tf::poseTFToMsg(world_to_tcp_tf*Transform(Quaternion::getIdentity(),Vector3(0,0,-retreat_dis)),end_pose);

  poses.poses.clear();
  poses.poses.push_back(start_pose);
  poses.poses.push_back(target_pose);
  poses.poses.push_back(end_pose);

  return poses;
}

void PickAndPlace::show_target_at_pick(bool show)
{
	// updating marker action
	visualization_msgs::Marker target =target_marker_;
	target.id = 0;
	target.ns = "targets";
	target.frame_locked = true;
	target.action =
			show ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;

	// publish messages
	marker_publisher.publish(target);
}

void PickAndPlace::show_target_at_place(bool show)
{
	// updating marker action
	visualization_msgs::Marker target =target_marker_;
	target.id = 0;
	target.ns = "targets";
	target.frame_locked = true;
	target.header.frame_id = cfg.WORLD_FRAME_ID;
	target.pose = cfg.WORLD_TO_PLACE_POSE;
	target.action =
			show ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;

	// publish messages
	marker_publisher.publish(target);
}

void PickAndPlace::show_target_attached(bool show)
{
	// updating marker action
	visualization_msgs::Marker target =target_marker_;

	target.id = 0;
	target.ns = "targets";
	target.frame_locked = true;
	target.action =
		show ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
	target.header.frame_id = cfg.TCP_LINK_NAME;
	target.pose = cfg.TCP_TO_TARGET_POSE;

	// publish messages
	marker_publisher.publish(target);
}

void PickAndPlace::broadcast_tcp_candidate(const geometry_msgs::Pose& world_to_tcp_pose)
{
	tf::Transform t;
	tf::poseMsgToTF(world_to_tcp_pose,t);
	transform_broadcast_ptr_->sendTransform(tf::StampedTransform(t,ros::Time::now(),cfg.WORLD_FRAME_ID,"tcp_candidate"));
}

bool PickAndPlace::validate_states(const RobotStateMsgArray& rs)
{

	bool all_valid = true;
	for(int i = 0;i < rs.size();i++)
	{
		// checking states
		moveit_msgs::RobotState  state_msg =moveit_msgs::RobotState(rs[i]);
		state_msg.is_diff = true;
		if(planning_scene_ptr->isStateColliding(state_msg,cfg.ARM_GROUP_NAME,true))
		/*if(planning_scene_ptr->isStateValid(state_msg,cfg.ARM_GROUP_NAME,true))*/
		{
			ROS_WARN_STREAM("Collision detected at pose "<<i);
			all_valid = false;
			break;
		}

		ROS_INFO_STREAM("Pose "<<i<<" is collision free");
	}

	return all_valid;
}

bool PickAndPlace::create_pick_motion_plans(
		const moveit_msgs::RobotState &start_state,
						const RobotStateMsgArray& pick_states,
						MotionPlanArray& mp)
{
	// planning scene msg
	moveit_msgs::PlanningScene ps;

	reset_planning_scene();
	if(!update_planning_scene())
	{
		return false;
	}

	// adding all obstacles to planning scene
	add_obstacles_to_planning_scene(obstacles_,true);
	add_obstacle_to_planning_scene(target_obj_on_world_,true);
	publish_planning_scene();

	// planning pick move 1
	move_group_interface::MoveGroup::Plan plan;
	if(create_motion_plan(start_state,pick_states[0],plan))
	{
		mp.push_back(plan);
		ROS_INFO_STREAM("Pick motion plan 1 completed");
	}
	else
	{
		ROS_ERROR_STREAM("Failed to create pick move 1");
		return false;
	}

	// removing object from planning scene
	add_obstacle_to_planning_scene(target_obj_on_world_,false);
	publish_planning_scene();

	// planning pick move 2
	if(create_motion_plan(pick_states[0],pick_states[1],plan))
	{
		mp.push_back(plan);
		ROS_INFO_STREAM("Pick motion plan 2 completed");
	}
	else
	{
		ROS_ERROR_STREAM("Failed to create pick move 2");
		return false;
	}

	// planning pick move 3
	if(create_motion_plan(pick_states[1],pick_states[2],plan))
	{
		mp.push_back(plan);
		ROS_INFO_STREAM("Pick motion plan 3 completed");
	}
	else
	{
		ROS_ERROR_STREAM("Failed to create pick move 3");
		return false;

	}
	return true;
}

bool PickAndPlace::create_place_motion_plans(const moveit_msgs::RobotState &start_state,
		const RobotStateMsgArray& place_states,MotionPlanArray& mp)
{
	// all states
	RobotStateMsgArray all_states;
	all_states.reserve(1 + place_states.size());
	all_states.push_back(start_state);
	all_states.insert(all_states.begin()+1,place_states.begin(),place_states.end());


	move_group_interface::MoveGroup::Plan plan;
	for(int i =1;i < all_states.size();i++)
	{
		moveit_msgs::RobotState &start = all_states[i-1];
		moveit_msgs::RobotState &end = all_states[i];

		if(i==all_states.size()-1) // detach on last motion plan
		{
			attach_object(false,target_obj_attached_,start);
		}
		else
		{
			attach_object(true,target_obj_attached_,start);
		}

		if(create_motion_plan(start,end,plan))
		{
			mp.push_back(plan);
		}
		else
		{
			ROS_ERROR_STREAM("Failed to create place motion plan "<<i);
			return false;
		}
	}

	return true;
}

void PickAndPlace::move_to_wait_position()
{

	moveit_msgs::RobotState start_to_wait_state,end_to_wait_state;

	move_group_ptr->setNamedTarget(cfg.WAIT_POSE_NAME);
	robot_state::robotStateToRobotStateMsg(*move_group_ptr->getCurrentState(),start_to_wait_state);
	robot_state::robotStateToRobotStateMsg(move_group_ptr->getJointValueTarget(),end_to_wait_state);

	// set allowed planning time
	//move_group_ptr->setPlanningTime(60.0f);
	if(create_motion_plan(start_to_wait_state,end_to_wait_state,home_motion_plan_) &&
			move_group_ptr->execute(home_motion_plan_))
	{
		ROS_INFO_STREAM("Move to " << cfg.WAIT_POSE_NAME<< " Succeeded");
	}
	else
	{
		ROS_ERROR_STREAM("Move to " << cfg.WAIT_POSE_NAME<< " Failed");
		exit(1);
	}
}

bool PickAndPlace::execute_pick_motion_plans(const MotionPlanArray& mp)
{
	show_target_at_pick(true);
	for(int i = 0; i < mp.size();i++)
	{
		const move_group_interface::MoveGroup::Plan &p = mp[i];
		if(move_group_ptr->execute(p))
		{
			if(i == 0) // do pre-grasp
			{
				set_gripper(false);
			}

			if(i == 1) // do grasp
			{
				set_gripper(true);
				show_target_attached(true);
			}
		}
		else
		{
			return false;
		}
	}

	return true;
}

bool PickAndPlace::execute_place_motion_plans(const MotionPlanArray &mp)
{
	show_target_attached(true);
	for(int i = 0; i < mp.size();i++)
	{
		const move_group_interface::MoveGroup::Plan &p = mp[i];
		if(move_group_ptr->execute(p))
		{
			if(i == 1) // do pregrasp
			{
				set_gripper(false);
				show_target_at_place(true);
			}

		}
		else
		{
			return false;
		}
	}

	return true;
}

bool PickAndPlace::attach_object(bool attach,const moveit_msgs::AttachedCollisionObject &att,moveit_msgs::RobotState &rs)
{
	rs.attached_collision_objects.clear();
	rs.attached_collision_objects.push_back(att);
	moveit_msgs::AttachedCollisionObject &rs_attached = rs.attached_collision_objects[0];


	rs_attached.object.primitive_poses[0] = cfg.TCP_TO_TARGET_POSE;
	rs_attached.object.id = cfg.ATTACHED_OBJECT_LINK_NAME;
	rs_attached.object.header.frame_id = cfg.TCP_LINK_NAME;
	rs_attached.touch_links = cfg.TOUCH_LINKS;
	rs_attached.link_name = cfg.TCP_LINK_NAME;
	rs_attached.object.operation = attach ? moveit_msgs::CollisionObject::ADD : moveit_msgs::CollisionObject::REMOVE;


	//ROS_INFO_STREAM("attaching collision object\n "<<rs_attached);

	rs.is_diff = true;
	return true;
}

void PickAndPlace::add_obstacles_to_planning_scene(const CollisionObjectArray &obstacles,bool add)
{

	if(obstacles.empty())
	{
		return;
	}

	// adding collision objects to planning scene
	for(CollisionObjectArray::const_reverse_iterator i = obstacles.rbegin();i != obstacles.rend();i++)
	{
		moveit_msgs::CollisionObject obj = *i;
		if(add)
		{
			//obj = *i;
			obj.operation = moveit_msgs::CollisionObject::ADD;
		}
		else
		{
			//obj.id = (*i).id;
			obj.operation = moveit_msgs::CollisionObject::REMOVE;
		}

		if(!planning_scene_ptr->processCollisionObjectMsg(obj))
		{
			ROS_WARN_STREAM("Collision object "<<obj.id<<", could not be processed");
		}
		else
		{
			ROS_INFO_STREAM("Collision object "<<obj.id<<(add?" Added" :" Removed"));
		}
	}
}

void PickAndPlace::remove_obstacle_from_planning_scene(const moveit_msgs::CollisionObject &obstacle)
{
	if(planning_scene_ptr->getWorldNonConst()->hasObject(obstacle.id))
	{
		planning_scene_ptr->getWorldNonConst()->removeObject(obstacle.id);
		ROS_INFO_STREAM("Collision object "<<obstacle.id<<" was removed");
	}
	else
	{
		ROS_WARN_STREAM("Collision object not found, ignoring remove");
	}
}

void PickAndPlace::reset_planning_scene()
{
	planning_scene_ptr->getWorldNonConst()->clearObjects();
}

void PickAndPlace::publish_planning_scene(bool diff)
{
	moveit_msgs::PlanningScene ps;
	planning_scene_ptr->getPlanningSceneMsg(ps);
	ps.is_diff = diff;
	planning_scene_publisher.publish(ps);
	ros::Duration(1).sleep();
}

void PickAndPlace::add_obstacle_to_planning_scene(const moveit_msgs::CollisionObject &obj,bool add)
{
	CollisionObjectArray obstacles;
	obstacles.push_back(obj);
	add_obstacles_to_planning_scene(obstacles,add);
}

void PickAndPlace::set_gripper(bool do_grasp)
{

  // task variables
  object_manipulation_msgs::GraspHandPostureExecutionGoal grasp_goal;
  bool success;

  // set the corresponding gripper action in the "grasp_goal" object.
  if (do_grasp)
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP;
  else
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE;


  grasp_action_client_ptr->sendGoal(grasp_goal);

  success = grasp_action_client_ptr->waitForResult(ros::Duration(4.0f));

  if(success)
  {
    if (do_grasp)
      ROS_INFO_STREAM("Gripper closed");
    else
      ROS_INFO_STREAM("Gripper opened");
  }
  else
  {
    ROS_ERROR_STREAM("Gripper failure");
    exit(1);
  }
}

bool PickAndPlace::update_planning_scene()
{
	bool succeeded = false;
	moveit_msgs::GetPlanningScene ps_srv;
	ps_srv.request.components.components = ps_srv.request.components.SCENE_SETTINGS |
		      ps_srv.request.components.ROBOT_STATE |
		      ps_srv.request.components.ROBOT_STATE_ATTACHED_OBJECTS |
		      ps_srv.request.components.WORLD_OBJECT_NAMES |
		      ps_srv.request.components.WORLD_OBJECT_GEOMETRY |
		      ps_srv.request.components.OCTOMAP |
		      ps_srv.request.components.TRANSFORMS |
		      ps_srv.request.components.ALLOWED_COLLISION_MATRIX |
		      ps_srv.request.components.LINK_PADDING_AND_SCALING;

	if(planning_scene_client.call(ps_srv))
	{
		planning_scene_ptr->setPlanningSceneMsg(ps_srv.response.scene);

		//ROS_INFO_STREAM(ps_srv.response.scene.allowed_collision_matrix);
		ROS_INFO_STREAM("Octomap found in scene with "<<ps_srv.response.scene.world.octomap.octomap.data.size()<<" voxels");
		ROS_INFO_STREAM("Planning scene updated");

		// adding octomap entry to allowed collision matrix
		if(!ps_srv.response.scene.world.octomap.octomap.data.empty())
		{
			collision_detection::AllowedCollisionMatrix& alcm = planning_scene_ptr->getAllowedCollisionMatrixNonConst();
			//alcm.setEntry(planning_scene_ptr->OCTOMAP_NS,false);
			alcm.setDefaultEntry(planning_scene_ptr->OCTOMAP_NS,false);
		}

		succeeded = true;
	}
	else
	{
		ROS_WARN_STREAM("Planning scene could not be updated");
	}

	return succeeded;
}

}




