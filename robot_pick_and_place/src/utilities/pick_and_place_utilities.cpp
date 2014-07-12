/*
 * pick_and_place_utilities.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <robot_pick_and_place/pick_and_place_utilities.h>
#include <moveit/kinematic_constraints/utils.h>
#include <tf/transform_listener.h>
#include <boost/assign/std/vector.hpp>
#include <boost/assign/list_of.hpp>
#include <iostream>
#include <ros/ros.h>

using namespace tf;
using namespace boost::assign;

namespace robot_pick_and_place
{

// =============================== Utility functions ===============================

std::vector<geometry_msgs::Pose> create_manipulation_poses(double retreat_dis,double approach_dis,const tf::Transform &tcp_at_target_tf)
{
  geometry_msgs::Pose start_pose, target_pose, end_pose;
  std::vector<geometry_msgs::Pose> poses;

  // creating start pose by applying a translation along +z by approach distance
  //tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,approach_dis))*tcp_at_target_tf,start_pose);
  tf::poseTFToMsg(tcp_at_target_tf*Transform(Quaternion::getIdentity(),Vector3(0,0,-approach_dis)),start_pose);

  // converting target pose
  tf::poseTFToMsg(tcp_at_target_tf,target_pose);

  // creating end pose by applying a translation along +z by retreat distance
  //tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,retreat_dis))*tcp_at_target_tf,end_pose);
  tf::poseTFToMsg(tcp_at_target_tf*Transform(Quaternion::getIdentity(),Vector3(0,0,-retreat_dis)),end_pose);

  poses.clear();
  poses.push_back(start_pose);
  poses.push_back(target_pose);
  poses.push_back(end_pose);

  return poses;
}

std::vector<geometry_msgs::Pose> transform_from_tcp_to_wrist(tf::Transform tcp_to_wrist_tf,const std::vector<geometry_msgs::Pose> tcp_poses)
{
  // array for poses of the wrist
  std::vector<geometry_msgs::Pose> wrist_poses;
  wrist_poses.resize(tcp_poses.size());

  // applying transform to each tcp poses
  tf::Transform world_to_wrist_tf, world_to_tcp_tf;
  wrist_poses.resize(tcp_poses.size());
  for(unsigned int i = 0; i < tcp_poses.size(); i++)
  {
    tf::poseMsgToTF(tcp_poses[i],world_to_tcp_tf);
    world_to_wrist_tf = world_to_tcp_tf * tcp_to_wrist_tf;
    tf::poseTFToMsg(world_to_wrist_tf,wrist_poses[i]);
  }

  return wrist_poses;
}

moveit_msgs::Constraints create_path_orientation_constraints(const geometry_msgs::Pose &goal_pose,
		float x_tolerance,float y_tolerance,float z_tolerance,std::string link_name)
{
	moveit_msgs::Constraints path_constraints = moveit_msgs::Constraints();
	path_constraints.name = "tcp_orientation_constraint";

	// setting constraint properties
	moveit_msgs::OrientationConstraint orientation_constraint = moveit_msgs::OrientationConstraint();
	orientation_constraint.header.frame_id="world_frame";
	//orientation_constraint.orientation = goal_pose.orientation;
	orientation_constraint.orientation.w = 1;
	orientation_constraint.absolute_x_axis_tolerance = x_tolerance;
	orientation_constraint.absolute_y_axis_tolerance = y_tolerance;
	orientation_constraint.absolute_z_axis_tolerance = z_tolerance;
	orientation_constraint.weight = 1.0f;
	orientation_constraint.link_name = link_name;

	// adding orientation constraint to path_constraints object
	path_constraints.orientation_constraints.push_back(orientation_constraint);
	return path_constraints;
}

std::ostream& operator<<(std::ostream& os, const tf::Vector3 vec)
{
  return os << "[" << vec.getX() << ", " << vec.getY() << ", " << vec.getZ() << "]";
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::Point pt)
{
  return os << "[" << pt.x << ", " << pt.y << ", " << pt.z << "]";
}

bool parse_pose_parameter(XmlRpc::XmlRpcValue pose_param,tf::Transform &t)
{
	std::map<std::string,double> fields_map =
			boost::assign::map_list_of("x",0.0d)
			("y",0.0d)
			("z",0.0d)
			("rx",0.0d)
			("ry",0.0d)
			("rz",0.0d);

	// parsing fields
	std::map<std::string,double>::iterator i;
	bool succeeded = true;
	for(i= fields_map.begin();i != fields_map.end();i++)
	{
		if(pose_param.hasMember(i->first) && pose_param[i->first].getType() == XmlRpc::XmlRpcValue::TypeDouble)
		{
			fields_map[i->first] = static_cast<double>(pose_param[i->first]);
		}
		else
		{
			succeeded = false;
			break;
		}
	}

	tf::Vector3 pos = tf::Vector3(fields_map["x"],fields_map["y"],fields_map["z"]);
	tf::Quaternion q;
	q.setRPY(fields_map["rx"],fields_map["ry"],fields_map["rz"]); // fixed axis
	t.setOrigin(pos);
	t.setRotation(q);

	return succeeded;
}

bool parse_pose_parameter(XmlRpc::XmlRpcValue pose_param,geometry_msgs::Pose &pose)
{
	tf::Transform t;
	if(parse_pose_parameter(pose_param,t))
	{
		tf::poseTFToMsg(t,pose);
		return true;
	}
	else
	{
		return false;
	}
}

bool PickAndPlaceConfig::init()
{
  ros::NodeHandle nh("~");
  double w, l, h, x, y, z;
  XmlRpc::XmlRpcValue list,tcp_to_target,world_to_place;

  if(nh.getParam("arm_group_name",ARM_GROUP_NAME)
      && nh.getParam("tcp_link_name",TCP_LINK_NAME)
      && nh.getParam("wrist_link_name",WRIST_LINK_NAME)
      && nh.getParam("attached_object_link",ATTACHED_OBJECT_LINK_NAME)
      && nh.getParam("world_frame_id",WORLD_FRAME_ID)
      && nh.getParam("home_pose_name",HOME_POSE_NAME)
      && nh.getParam("wait_pose_name",WAIT_POSE_NAME)
      && nh.getParam("touch_links",list)
      && nh.getParam("retreat_distance",RETREAT_DISTANCE)
      && nh.getParam("approach_distance",APPROACH_DISTANCE)
      && nh.getParam("tcp_to_target_pose",tcp_to_target)
      && nh.getParam("world_to_place_pose",world_to_place)
      && nh.getParam("gripper_workrange",GRIPPER_WORKRANGE))
  {

    // parsing touch links list
    for(int32_t i =0 ; i < list.size();i++)
    {
    	if(list[i].getType() == XmlRpc::XmlRpcValue::TypeString)
    	{
    		TOUCH_LINKS.push_back(static_cast<std::string>(list[i]));
    	}
    	else
    	{
    		ROS_ERROR_STREAM("touch links list contains invalid data.  Provide an array of strings 'touch_links:{link_a,link_b,link_c ..}'");
    	}
    }

    // load poses
    if(parse_pose_parameter(tcp_to_target,TCP_TO_TARGET_POSE) && parse_pose_parameter(world_to_place,WORLD_TO_PLACE_POSE))
    {
    	ROS_INFO_STREAM("Pose parameters parsed correctly");
    }
    else
    {
    	ROS_ERROR_STREAM("Error parsing pose parameters");
    	return false;
    }


    return true;
  }
  else
  {
    return false;
  }

}

}
