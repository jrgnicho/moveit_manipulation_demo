/*
 * pick_and_place.cpp
 *
 *  Created on: Jun 24, 2014
 *      Author: ros developer 
 */

#include <robot_pick_and_place/pick_and_place.h>

using namespace robot_pick_and_place;
geometry_msgs::Pose robot_pick_and_place::PickAndPlace::detect_target_pose()
{
	// ik model
	robot_state::RobotStatePtr robot_state_ptr = move_group_ptr->getCurrentState();


	GraspPose gp;
	gp.request.candidates_per_pose = 10;
	gp.request.gripper_workrange=0.087f;
	gp.request.planning_frame_id = cfg.WORLD_FRAME_ID;

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

				if(robot_state_ptr->setFromIK(robot_state_ptr->getJointModelGroup(cfg.ARM_GROUP_NAME),
						world_to_tcp_pose,cfg.TCP_LINK_NAME,
						20,4))
				{
					world_to_target_pose = p;
					cfg.MARKER_MESSAGE = gp.response.candidate_objects.markers[i];
					show_box(true);
					success = true;
					break;

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




