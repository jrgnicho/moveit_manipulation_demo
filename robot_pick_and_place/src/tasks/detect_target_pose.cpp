/*
 * pick_and_place.cpp
 *
 *  Created on: Jun 24, 2014
 *      Author: ros developer 
 */

#include <robot_pick_and_place/pick_and_place.h>

using namespace robot_pick_and_place;
using namespace handle_detector;
geometry_msgs::Pose robot_pick_and_place::PickAndPlace::detect_target_pose()
{

	ros::Duration(2.0f).sleep();

	ROS_INFO_STREAM("detecting target");

	// grasp candidates requst
	GraspPoseCandidates gp;
	gp.request.candidates_per_pose = 10;
	gp.request.gripper_workrange=0.087f;
	gp.request.planning_frame_id = cfg.WORLD_FRAME_ID;

	// ik request
	moveit_msgs::GetPositionIK ik;
	ik.request.ik_request.group_name = cfg.ARM_GROUP_NAME;
	ik.request.ik_request.pose_stamped.header.frame_id = cfg.TCP_LINK_NAME;
	//ik.request.ik_request.ik_link_name = cfg.TCP_LINK_NAME;
	ik.request.ik_request.attempts= 20;
	ik.request.ik_request.timeout = ros::Duration(4);
	ik.request.ik_request.avoid_collisions = true;
	ik.request.ik_request.robot_state.is_diff=true;

	// robot state check request;
	moveit_msgs::GetStateValidity st;
	st.request.group_name = cfg.ARM_GROUP_NAME;

	// creating motion plan request
	std::vector<double> position_tolerances(3,0.01f);
	std::vector<double> orientation_tolerances(3,0.01f);
	moveit_msgs::GetMotionPlan mp;
	mp.request.motion_plan_request.group_name = cfg.ARM_GROUP_NAME;
	mp.request.motion_plan_request.allowed_planning_time = 60.0f;
	mp.request.motion_plan_request.num_planning_attempts = 1;

	// finding gripper pose at pick
	geometry_msgs::Pose world_to_target_pose, world_to_tcp_pose;
	tf::Transform world_to_tcp_tf, world_to_target_tf;
	bool success = false;
	if(grasp_pose_client.call(gp))
	{
		for(int i = 0; i <gp.response.candidate_grasp_poses.size();i++)
		{
			std::vector<geometry_msgs::Pose> &poses = gp.response.candidate_grasp_poses[i].poses;
			// checking ik for identifying reachable pose
			for(int j = 0;j < poses.size();j++)
			{
				geometry_msgs::Pose &p = poses[j];
				tf::poseMsgToTF(p,world_to_target_tf);
				world_to_tcp_tf.setRotation(world_to_target_tf.getRotation()*
						tf::Quaternion(tf::Vector3(1,0,0),M_PI));// inverting z vector
				world_to_target_tf.setOrigin(world_to_target_tf.getOrigin());
				tf::poseTFToMsg(world_to_tcp_tf,world_to_tcp_pose);

				// calling ik service
				ik.request.ik_request.pose_stamped.pose = world_to_tcp_pose;

				if(ik_client.call(ik) && ik.response.error_code.val == ik.response.error_code.SUCCESS)
				{
/*					moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(
							cfg.TCP_LINK_NAME,ik.request.ik_request.pose_stamped,position_tolerances,
							orientation_tolerances);
					ROS_INFO_STREAM(ik.response.solution);
					mp.request.motion_plan_request.start_state = ik.response.solution;
					mp.request.motion_plan_request.start_state.is_diff = true;
					mp.request.motion_plan_request.goal_constraints.clear();
					mp.request.motion_plan_request.goal_constraints.push_back(pose_goal);*/

					world_to_target_pose = p;
					cfg.MARKER_MESSAGE = gp.response.candidate_objects.markers[i];
					show_box(true);
					success = true;

					ROS_INFO_STREAM("Found reachable pose at index "<< j <<" with value:\n"<<world_to_target_pose);
					break;

				}
				else
				{
					ROS_WARN_STREAM("Grasp pose "<<j<<" not reachable");
				}


			}

			if(success)
			{
				break;
			}
		}

		if(!success)
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




