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

	  // marker publisher (rviz visualization)
	  marker_publisher = nh.advertise<visualization_msgs::Marker>(
			  cfg.MARKER_TOPIC,1);

	  // target recognition client (perception)
	  target_recognition_client = nh.serviceClient<robot_pick_and_place::GetTargetPose>(
			  cfg.TARGET_RECOGNITION_SERVICE);

	  grasp_pose_client = nh.serviceClient<handle_detector::GraspPoseCandidates>(
			  cfg.GRASP_POSES_SERVICE);

	  // grasp action client (vacuum gripper)
	  grasp_action_client_ptr = GraspActionClientPtr(
			  new GraspActionClient(cfg.GRASP_ACTION_NAME,true));

	  // ik client (inverse kinematics)
	  ik_client = nh.serviceClient<moveit_msgs::GetPositionIK>(cfg.IK_SERVICE);

	  // ik solver
	  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	  kinematic_model_ptr = robot_model_loader.getModel();
	  kinematic_state_ptr = RobotStatePtr(new RobotState(kinematic_model_ptr));

	  // plannig scene client
	  planning_scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

	  // planning scene monitor
	  planning_scene_monitor_ptr_ = planning_scene_monitor::PlanningSceneMonitorPtr(
			  new planning_scene_monitor::PlanningSceneMonitor("robot_description",transform_listener_ptr));
	  //planning_scene_monitor_ptr_->addUpdateCallback(boost::bind(&PickAndPlace::scene_update_callback,this,_1));
	  planning_scene_monitor_ptr_->startStateMonitor();
	  planning_scene_monitor_ptr_->startSceneMonitor();
	  planning_scene_monitor_ptr_->startWorldGeometryMonitor();

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
	/* ========================================*/
	/* Pick & Place Tasks                      */
	/* ========================================*/

	geometry_msgs::Pose box_pose;
	std::vector<geometry_msgs::Pose> pick_poses, place_poses;

	// move to a "clear" position
	move_to_wait_position();

	// turn off vacuum gripper
	set_gripper(false);

	// get the box position and orientation
	box_pose = detect_target_poses();

	// build a sequence of poses to "pick" the box
	pick_poses = create_pick_moves(box_pose);

	// plan/execute the sequence of "pick" moves
	pickup_box(pick_poses,box_pose);

	// build a sequence of poses to "place" the box
	place_poses = create_place_moves();

	// plan/execute the "place" moves
	place_box(place_poses,box_pose);

	// move back to the "clear" position
	move_to_wait_position();
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

std::vector<geometry_msgs::Pose> PickAndPlace::create_pick_moves(geometry_msgs::Pose &box_pose)
{
  //ROS_ERROR_STREAM("create_pick_moves is not implemented yet.  Aborting."); exit(1);

  // task variables
  tf::Transform world_to_tcp_tf;
  tf::Transform world_to_box_tf;
  tf::StampedTransform tcp_to_wrist_tf;
  std::vector<geometry_msgs::Pose> tcp_pick_poses, wrist_pick_poses;


  /* Fill Code:
   * Goal:
   * - Create tcp pose at box pick.
   * Hints:
   * - Use the 'setOrigin' to set the position of 'world_to_tcp_tf'.
   * */
  tf::poseMsgToTF(box_pose,world_to_box_tf);
  tf::Vector3 box_position(box_pose.position.x, box_pose.position.y, box_pose.position.z);
  world_to_tcp_tf.setOrigin(box_position);

  // setting tcp orientation
  /* Inverting the approach direction so that the tcp points towards the box instead of
   * away from it.*/
  world_to_tcp_tf.setRotation(world_to_box_tf.getRotation()* tf::Quaternion(tf::Vector3(1,0,0),M_PI));


  // create all the poses for tcp's pick motion (approach, pick and retreat)
  tcp_pick_poses = create_manipulation_poses(cfg.RETREAT_DISTANCE, cfg.APPROACH_DISTANCE, world_to_tcp_tf);


  /* Fill Code:
   * Goal:
   * - Find transform from tcp to wrist.
   * Hints:
   * - Use the 'lookupTransform' method in the transform listener.
   * */
  transform_listener_ptr->waitForTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME,ros::Time::now(),ros::Duration(3.0f));
  transform_listener_ptr->lookupTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME, ros::Time(0.0f), tcp_to_wrist_tf);


  /* Fill Code:
   * Goal:
   * - Transform list of pick poses from tcp frame to wrist frame
   * Hint:
   * - Use the 'transform_from_tcp_to_wrist' function and save results into
   * 	'wrist_pick_poses'.
   * 	*/
  wrist_pick_poses = transform_from_tcp_to_wrist(tcp_to_wrist_tf, tcp_pick_poses);

  // printing some results
  ROS_INFO_STREAM("tcp position at pick: " << world_to_tcp_tf.getOrigin());
  ROS_INFO_STREAM("tcp z direction at pick: " << world_to_tcp_tf.getBasis().getColumn(2));
  ROS_INFO_STREAM("wrist position at pick: " << wrist_pick_poses[1].position);

  return wrist_pick_poses;
}

std::vector<geometry_msgs::Pose> PickAndPlace::create_place_moves()
{
  //ROS_ERROR_STREAM("create_place_moves is not implemented yet.  Aborting."); exit(1);

  // task variables
  tf::Transform world_to_tcp_tf;
  tf::StampedTransform tcp_to_wrist_tf;
  std::vector<geometry_msgs::Pose> tcp_place_poses, wrist_place_poses;


  /* Fill Code:
   * Objective:
   * - Find the desired tcp pose at box place
   * Hints:
   * - Use the 'setOrigin' method to set the position of 'world_to_tcp_tf'
   * 	using cfg.BOX_PLACE_TF.
   * - cfg.BOX_PLACE_TF is a tf::Transform object so it provides a getOrigin() method.
   * */
  world_to_tcp_tf.setOrigin(cfg.BOX_PLACE_TF.getOrigin());

  /* Fill Code:
   * Goal:
   * - Reorient the tool so that the tcp points towards the box.
   * Hints:
   * - Use the 'setRotation' to set the orientation of 'world_to_tcp_tf'.
   * - Use the same "pointing down" orientation as in create_pick_moves().
   * - The quaternion value "tf::Quaternion(M_PI, 0, M_PI/2.0f)" will point
   * 	the tcp's direction towards the box.
   * 	*/
  world_to_tcp_tf.setRotation(tf::Quaternion(M_PI, 0, M_PI/2.0f));


  /* Fill Code:
   * Goal:
   * - Create place poses for tcp.   *
   * Hints:
   * - Use the 'create_manipulation_poses' and save results to 'tcp_place_poses'.
   * */
  tcp_place_poses = create_manipulation_poses(cfg.RETREAT_DISTANCE, cfg.APPROACH_DISTANCE, world_to_tcp_tf);


  /* Fill Code:
   * Goal:
   * - Find transform from tcp to wrist.
   * Hints:
   * - Use the 'lookupTransform' method in the transform listener.
   * */
  transform_listener_ptr->waitForTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME, ros::Time(0.0f), ros::Duration(3.0f));
  transform_listener_ptr->lookupTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME, ros::Time(0.0f), tcp_to_wrist_tf);


  /* Fill Code:
   * Goal:
   * - Transform list of place poses from the tcp to the wrist coordinate frame.
   * Hints:
   * - Use the 'transform_from_tcp_to_wrist' function and save results into
   * 	'wrist_place_poses'.
   * */
  wrist_place_poses = transform_from_tcp_to_wrist(tcp_to_wrist_tf, tcp_place_poses);

  // printing results
  ROS_INFO_STREAM("tcp position at place: " << world_to_tcp_tf.getOrigin());
  ROS_INFO_STREAM("wrist position at place: "<<wrist_place_poses[1].position);

  return wrist_place_poses;
}

geometry_msgs::Pose PickAndPlace::detect_box_pick()
{
  //ROS_ERROR_STREAM("detect_box_pick is not implemented yet.  Aborting."); exit(1);

  // creating shape for recognition
  shape_msgs::SolidPrimitive shape;
  shape.type = shape_msgs::SolidPrimitive::BOX;
  shape.dimensions.resize(3);
  shape.dimensions[0] = cfg.BOX_SIZE.getX();
  shape.dimensions[1] = cfg.BOX_SIZE.getY();
  shape.dimensions[2] = cfg.BOX_SIZE.getZ();

  // creating request object
  robot_pick_and_place::GetTargetPose srv;
  srv.request.shape = shape;
  srv.request.world_frame_id = cfg.WORLD_FRAME_ID;
  srv.request.ar_tag_frame_id = cfg.AR_TAG_FRAME_ID;
  geometry_msgs::Pose place_pose;
  tf::poseTFToMsg(cfg.BOX_PLACE_TF,place_pose);
  srv.request.remove_at_poses.push_back(place_pose);

  //
  /* Fill Code:
   * Goal:
   * - Call target recognition service and save results.
   * Hint:
   * - Use the service response member to access the
   * 	detected pose "srv.response.target_pose".
   * - Assign the target_pose in the response to the box_pose variable in
   * 	order to save the results.
   * 	*/
  geometry_msgs::Pose box_pose;
  if(target_recognition_client.call(srv))
  {
	  if(srv.response.succeeded)
	  {
		  box_pose = srv.response.target_pose;
		  ROS_INFO_STREAM("target recognition succeeded");
	  }
	  else
	  {
		  ROS_ERROR_STREAM("target recognition failed");
		  exit(0);

	  }
  }
  else
  {
	  ROS_ERROR_STREAM("Service call for target recognition failed with response '"<<
			  (srv.response.succeeded ?"SUCCESS":"FAILURE")
					  <<"', exiting");
	  exit(0);
  }

  // updating box marker for visualization in rviz
	visualization_msgs::Marker marker = cfg.MARKER_MESSAGE;
	cfg.MARKER_MESSAGE.header.frame_id = cfg.WORLD_FRAME_ID;
	cfg.MARKER_MESSAGE.pose = box_pose;
	cfg.MARKER_MESSAGE.pose.position.z = box_pose.position.z - 0.5f*cfg.BOX_SIZE.z();

	show_box(true);

	return box_pose;
}

geometry_msgs::Pose PickAndPlace::detect_target_poses()
{
	using namespace robot_pick_and_place;
	using namespace handle_detector;

	ROS_INFO_STREAM("detecting target");

	// grasp candidates requst
	GraspPoseCandidates gp;
	gp.request.candidates_per_pose = 10;
	gp.request.gripper_workrange=0.087f;
	gp.request.planning_frame_id = cfg.WORLD_FRAME_ID;


	// finding gripper pose at pick
	geometry_msgs::PoseArray tcp_poses;
	geometry_msgs::Pose world_to_target_pose, world_to_tcp_pose;
	tf::Transform world_to_tcp_tf, world_to_target_tf;
	bool found_valid_poses = false;
	if(grasp_pose_client.call(gp))
	{
		// update scene
		update_planning_scene();

		for(int i = 0; i <gp.response.candidate_grasp_poses.size();i++)
		{
			std::vector<geometry_msgs::Pose> &poses = gp.response.candidate_grasp_poses[i].poses;
			for(int j = 0;j < poses.size();j++)
			{
				// creating tcp pose from target pose
				geometry_msgs::Pose &p = poses[j];
				tf::poseMsgToTF(p,world_to_target_tf);
				world_to_tcp_tf.setRotation(world_to_target_tf.getRotation()*
						tf::Quaternion(tf::Vector3(1,0,0),M_PI));// inverting z vector
				world_to_tcp_tf.setOrigin(world_to_target_tf.getOrigin());
				tf::poseTFToMsg(world_to_tcp_tf,world_to_tcp_pose);

				// creating pick poses for tcp
				tcp_poses.poses = create_manipulation_poses(cfg.RETREAT_DISTANCE,cfg.APPROACH_DISTANCE,world_to_tcp_tf);

				// evaluating tcp poses at pick
				if(evaluate_poses(tcp_poses.poses))
				{
					// saving results
					world_to_target_pose = p;
					target_obj_on_world_= gp.response.candidate_collision_objects[i];
					target_obj_attached_.object = gp.response.candidate_collision_objects[i];

					found_valid_poses = true;


					cfg.MARKER_MESSAGE = gp.response.candidate_objects.markers[i];
					show_box(true);

					ROS_INFO_STREAM("Found valid tcp grasp poses at index "<< j <<" with grasp pose at:\n"<<world_to_tcp_pose);
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
			exit(0);
		}

	}
	else
	{
		ROS_ERROR_STREAM("grasp pose call to service failed, exiting");
		exit(0);
	}

	return world_to_target_pose;
}

bool PickAndPlace::evaluate_poses(std::vector<geometry_msgs::Pose>& tcp_poses)
{
	// robot state
	moveit_msgs::RobotState state_msg;


	// ik request
	moveit_msgs::GetPositionIK ik;
	ik.request.ik_request.group_name = cfg.ARM_GROUP_NAME;
	ik.request.ik_request.pose_stamped.header.frame_id = cfg.WORLD_FRAME_ID;
	ik.request.ik_request.ik_link_name = cfg.TCP_LINK_NAME;
	ik.request.ik_request.attempts= 10;
	ik.request.ik_request.timeout = ros::Duration(0.1f);
	ik.request.ik_request.avoid_collisions = true;
	ik.request.ik_request.robot_state.is_diff=true;

	bool all_reachable = true;
	for(int i = 0;i < tcp_poses.size();i++)
	{
		// checking ik solution
		ik.request.ik_request.pose_stamped.pose = tcp_poses[i];
		if(ik_client.call(ik) && (ik.response.error_code.val == ik.response.error_code.SUCCESS))
		{
			//ROS_INFO_STREAM("IK solution found at pose "<<i);
		}
		else
		{
			//ROS_WARN_STREAM("IK not available at pose "<<i<<" with code "<<ik.response.error_code.val);
			all_reachable = false;
			break;
		}

		state_msg = ik.response.solution;
		state_msg.is_diff = true;
		if(planning_scene_ptr->isStateColliding(state_msg,cfg.ARM_GROUP_NAME,true))
		{
			ROS_WARN_STREAM("Collision detected at pose "<<i);
			all_reachable = false;
			break;
		}
		else
		{
			//ROS_INFO_STREAM("Collision free state found at pose "<<i);
		}

		//ROS_INFO_STREAM("Pose "<<i<<" is reachable and collision free");
	}

	return all_reachable;
}

bool PickAndPlace::create_pick_motion_plans(
		const geometry_msgs::PoseArray &tcp_poses,MotionPlanArray& mp)
{


	return true;
}

bool PickAndPlace::create_place_motion_plans(
		const geometry_msgs::PoseArray &tcp_poses,MotionPlanArray& mp)
{
	return true;
}

void PickAndPlace::move_to_wait_position()
{

  // task variables
  bool success; // saves the move result

  /* Fill Code:
   * Goal:
   * - Set robot wait target
   * Hints:
   * - Use the 'setNamedTarget' method in the 'move_group' object.
   * */
  move_group_ptr->setNamedTarget(cfg.WAIT_POSE_NAME);

  // set allowed planning time
  move_group_ptr->setPlanningTime(60.0f);

  /* Fill Code:
   * Goal:
   * - Move the robot
   * Hints:
   * - Use the 'move' method in the 'move_group' object and save the result
   *  in the 'success' variable*/
  success = move_group_ptr->move();
  if(success)
  {
    ROS_INFO_STREAM("Move " << cfg.WAIT_POSE_NAME<< " Succeeded");
  }
  else
  {
    ROS_ERROR_STREAM("Move " << cfg.WAIT_POSE_NAME<< " Failed");
    exit(1);
  }
}

void PickAndPlace::pickup_box(std::vector<geometry_msgs::Pose>& pick_poses,const geometry_msgs::Pose& box_pose)
{
	  //ROS_ERROR_STREAM("move_through_pick_poses is not implemented yet.  Aborting."); exit(1);

	  // task variables
	  bool success;


	  /* Fill Code:
	   * Goal:
	   * - Set the wrist as the end-effector link
	     	 - The robot will try to move this link to the specified pose
	     	 - If not specified, moveit will use the last link in the arm group.
	   * Hints:
	   * - Use the 'setEndEffectorLink' in the 'move_group_ptr' object.
	   * - The WRIST_LINK_NAME field in the "cfg" configuration member contains
	   * 	the name for the arm's wrist link.
	   */
	  move_group_ptr->setEndEffectorLink(cfg.WRIST_LINK_NAME);

	  // set allowed planning time
	  move_group_ptr->setPlanningTime(60.0f);


	  /* Fill Code:
	   * Goal:
	   * - Set world frame as the reference
	    	- The target position is specified relative to this frame
	   	  	- If not specified, MoveIt will use the parent frame of the SRDF "Virtual Joint"
	   * Hints:
	   * - Use the 'setPoseReferenceFrame' in the 'move_group_ptr' object.
	   * - The WORLD_FRAME_ID in the "cfg" configuration member contains the name
	   * 	for the reference frame.
	   */
	  move_group_ptr->setPoseReferenceFrame(cfg.WORLD_FRAME_ID);

	  // move the robot to each wrist pick pose
	  for(unsigned int i = 0; i < pick_poses.size(); i++)
	  {
	  	moveit_msgs::RobotState robot_state;

	  /* Inspect Code:
	   * Goal:
	   * - Look in the "set_attached_object()" method to understand
	   * 	how to attach a payload using moveit.
	   */
		set_attached_object(false,geometry_msgs::Pose(),robot_state);


	  /* Inspect Code:
	   * Goal:
	   * - Look in the "set_attached_object()" method to observe how an
	   * 	entire moveit motion plan is created.
	   */
	    move_group_interface::MoveGroup::Plan plan;
	    success = create_motion_plan(pick_poses[i],robot_state,plan) && move_group_ptr->execute(plan);

	    // verifying move completion
	    if(success)
	    {
	      ROS_INFO_STREAM("Pick Move " << i <<" Succeeded");
	    }
	    else
	    {
	      ROS_ERROR_STREAM("Pick Move " << i <<" Failed");
	      set_gripper(false);
	      exit(1);
	    }


	    if(i == 0)
	    {

		/* Fill Code:
		 * Goal:
		 * - Turn on gripper suction after approach pose
		 * Hints:
		 * - Call the 'set_gripper' function to turn on suction.
		 * - The input to the set_gripper method takes a "true" or "false"
		 * 	  boolean argument.
		 */
	      set_gripper(true);

	    }

	  }

}

void PickAndPlace::place_box(std::vector<geometry_msgs::Pose>& place_poses,
		const geometry_msgs::Pose& box_pose)
{
  //ROS_ERROR_STREAM("move_through_place_poses is not implemented yet.  Aborting."); exit(1);

  // task variables
  bool success;

  /* Fill Code:
   * Goal:
   * - Set the ReferenceFrame and EndEffectorLink
   * Hints:
   * - Use the 'setEndEffectorLink' and 'setPoseReferenceFrame' methods of 'move_group']*/
  move_group_ptr->setEndEffectorLink(cfg.WRIST_LINK_NAME);
  move_group_ptr->setPoseReferenceFrame(cfg.WORLD_FRAME_ID);

  // set allowed planning time
  move_group_ptr->setPlanningTime(60.0f);

  // move the robot to each wrist place pose
  for(unsigned int i = 0; i < place_poses.size(); i++)
  {
  	moveit_msgs::RobotState robot_state;
  	if(i==0 || i == 1)
  	{
      // attaching box
      set_attached_object(false,box_pose,robot_state);
      show_box(true);

  	}
  	else
  	{
      // detaching box
      set_attached_object(false,geometry_msgs::Pose(),robot_state);
      show_box(false);
  	}

  	// create motion plan
    move_group_interface::MoveGroup::Plan plan;
    success = create_motion_plan(place_poses[i],robot_state,plan) && move_group_ptr->execute(plan);

    if(success)
    {
      ROS_INFO_STREAM("Place Move " << i <<" Succeeded");
    }
    else
    {
      ROS_ERROR_STREAM("Place Move " << i <<" Failed");
      set_gripper(false);
      exit(1);
    }


    if(i == 1)
    {
	/* Fill Code:
	 * Goal:
	 * - Turn off gripper suction after the release pose is reached.
	 * Hints:
	 * - Call the 'set_gripper' function to turn on suction.
	 * - The input to the set_gripper method takes a "true" or "false"
	 * 	  boolean argument.
	 */
      set_gripper(false);

    }

  }
}

void PickAndPlace::reset_world(bool refresh_octomap)
{

	// detach box if one is attached
	moveit_msgs::RobotState robot_state;
	set_attached_object(false,geometry_msgs::Pose(),robot_state);

	// get new sensor snapshot
	if(refresh_octomap)
	{
		detect_box_pick();
	}

	show_box(false);
}

void PickAndPlace::set_attached_object(bool attach, const geometry_msgs::Pose &pose,moveit_msgs::RobotState &robot_state)
{
	// get robot state
	robot_state::RobotStatePtr current_state= move_group_ptr->getCurrentState();

	if(attach)
	{

		// constructing shape
		std::vector<shapes::ShapeConstPtr> shapes_array;
		shapes::ShapeConstPtr shape( shapes::constructShapeFromMsg( cfg.ATTACHED_OBJECT.primitives[0]));
		shapes_array.push_back(shape);

		// constructing pose
		tf::Transform attached_tf;
		tf::poseMsgToTF(cfg.ATTACHED_OBJECT.primitive_poses[0],attached_tf);
		EigenSTL::vector_Affine3d pose_array(1);
		tf::transformTFToEigen(attached_tf,pose_array[0]);

		// attaching
		current_state->attachBody(cfg.ATTACHED_OBJECT_LINK_NAME,shapes_array,pose_array,cfg.TOUCH_LINKS,cfg.TCP_LINK_NAME);

		// update box marker
		cfg.MARKER_MESSAGE.header.frame_id = cfg.TCP_LINK_NAME;
		cfg.MARKER_MESSAGE.pose = cfg.TCP_TO_BOX_POSE;
	}
	else
	{

		// detaching
		if(current_state->hasAttachedBody(cfg.ATTACHED_OBJECT_LINK_NAME))
				current_state->clearAttachedBody(cfg.ATTACHED_OBJECT_LINK_NAME);
	}

	// save robot state data
	robot_state::robotStateToRobotStateMsg(*current_state,robot_state);
}

void PickAndPlace::set_gripper(bool do_grasp)
{
  //ROS_ERROR_STREAM("set_gripper is not implemented yet.  Aborting."); exit(1);

  // task variables
  object_manipulation_msgs::GraspHandPostureExecutionGoal grasp_goal;
  bool success;

  // set the corresponding gripper action in the "grasp_goal" object.
  if (do_grasp)
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP;
  else
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE;

  /* Fill Code:
   * Goal:
   * - Send the grasp goal to the server.
   * Hints:
   * - Use the 'sendGoal' method of the grasp client "grasp_action_client_ptr"
   * to make a call the server.
   */
  grasp_action_client_ptr->sendGoal(grasp_goal);


  /* Fill Code:
   * Goal:
   * - Confirm that client service call succeeded.
   * Hints:
   * - Use the "waitForResult" method of the client to wait for completion.
   * - Give "waitForResult" a timeout value of 4 seconds
   * - Timeouts in ros can be created using "ros::Duration(4.0f)".
   * - Save returned boolean from waitForResult() in the "success" variable.
   */
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

void PickAndPlace::scene_update_callback(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType t)
{
	planning_scene_monitor_ptr_->lockSceneRead();

	// initializing local scene
	if(planning_scene_ptr==0)
	{
		planning_scene_ptr = planning_scene::PlanningScene::clone(planning_scene_monitor_ptr_->getPlanningScene());
	}

	// updating local planning scene
	moveit_msgs::PlanningScene planning_scene_msg;
	planning_scene_monitor_ptr_->getPlanningScene()->getPlanningSceneMsg(planning_scene_msg);
	planning_scene_ptr->setPlanningSceneMsg(planning_scene_msg);
	planning_scene_monitor_ptr_->unlockSceneRead();

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
	}
	else
	{
		ROS_WARN_STREAM("Planning scene could not be updated");
	}

	return succeeded;
}

}




