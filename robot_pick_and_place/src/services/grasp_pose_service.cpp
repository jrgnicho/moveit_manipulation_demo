/*
 * grasp_plan_service.cpp
 *
 *  Created on: Jun 23, 2014
 *      Author: ros developer 
 */


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <robot_pick_and_place/GetTargetPose.h>
#include <handle_detector/HandleListMsg.h>
#include <robot_pick_and_place/GraspPose.h>
#include <math.h>

// constants
const std::string SENSOR_CLOUD_TOPIC = "sensor_cloud";
const std::string FILTERED_CLOUD_TOPIC = "filtered_cloud";
const std::string TARGET_RECOGNITION_SERVICE = "target_recognition";
const std::string HANDLE_DETECTION_TOPIC = "handle_list";
const double WAIT_FOR_MESSAGE_TIMEOUT = 5.0f;

namespace tf
{
	typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;
}

class GraspPoseServer
{
public:
	GraspPoseServer()
	{

	}

	~GraspPoseServer()
	{

	}

	void run()
	{
		if(init())
		{
			ros::spin();
		}
		else
		{
			ROS_ERROR_STREAM("Grasp Pose server initialization failed, exiting");
		}
	}

protected:

	bool init()
	{
		ros::NodeHandle nh;

		// initializing service server
		target_pose_server_ = nh.advertiseService(TARGET_RECOGNITION_SERVICE,&GraspPoseServer::target_pose_service_callback,this);

		// initializing publisher
		filtered_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(FILTERED_CLOUD_TOPIC,1);

		// tf setup
		tf_listener_ptr_ =  tf::TransformListenerPtr(new tf::TransformListener(nh,ros::Duration(1.0f)));


		return true;
	}

	bool load_parameters()
	{
		return true;
	}

	bool wait_for_point_cloud_msg(sensor_msgs::PointCloud2& msg)
	{
		// grab sensor data snapshot
		ros::NodeHandle nh;
		sensor_msgs::PointCloud2ConstPtr msg_ptr =
				ros::topic::waitForMessage<sensor_msgs::PointCloud2>(SENSOR_CLOUD_TOPIC,nh,
						ros::Duration(WAIT_FOR_MESSAGE_TIMEOUT));

		// check for empty message
		if(msg_ptr != sensor_msgs::PointCloud2ConstPtr())
		{

			msg = *msg_ptr;
		}

		return msg_ptr != sensor_msgs::PointCloud2ConstPtr();
	}

	bool wait_for_handle_list_msg(handle_detector::HandleListMsg &msg)
	{
		ros::NodeHandle nh;
		handle_detector::HandleListMsgConstPtr msg_ptr =
				ros::topic::waitForMessage<handle_detector::HandleListMsg>(HANDLE_DETECTION_TOPIC,nh,
						ros::Duration(WAIT_FOR_MESSAGE_TIMEOUT));

		// check for empty message
		if(msg_ptr != handle_detector::HandleListMsgConstPtr())
		{

			msg = *msg_ptr;
		}

		return msg_ptr != handle_detector::HandleListMsgConstPtr();
	}

	bool target_pose_service_callback(robot_pick_and_place::GraspPose::Request& req,
			robot_pick_and_place::GraspPose::Response& res)
	{
		sensor_msgs::PointCloud2 sensor_cloud_msg;
		handle_detector::HandleListMsg handles_msg;

		if(wait_for_point_cloud_msg(sensor_cloud_msg))
		{
			sensor_cloud_msg.header.stamp = ros::Time::now()-ros::Duration(0.5f);
			filtered_cloud_pub_.publish(sensor_cloud_msg);
		}
		else
		{
			//res.succeeded = false;
			return false;
		}

		// transform for converting handle to planning coordinate frame
		tf::Transform handle_to_planning_tf;

		if(wait_for_handle_list_msg(handles_msg) &&
				transform_to_robot_planning_frame(req.planning_frame_id,handles_msg.header.frame_id,handle_to_planning_tf))
		{

			geometry_msgs::Pose grasp_pose,cylinder_pose;
			tf::Transform grasp_tf, cylinder_tf;
			tf::Vector3 normal,axis;
			double angle = 2*M_PI/req.candidates_per_pose;

			std::vector<handle_detector::CylinderArrayMsg> &handles = handles_msg.handles;
			for(std::vector<handle_detector::CylinderArrayMsg>::iterator i = handles.begin();
					i!=handles.end();i++)
			{
				handle_detector::CylinderArrayMsg &ca = *i;
				std::vector<handle_detector::CylinderMsg> &c = ca.cylinders;

				for(int j = 0; j < ca.cylinders.size();j++)
				{
					handle_detector::CylinderMsg &cylinder = ca.cylinders[j];

					if(cylinder.radius < req.gripper_workrange )
					{

						// converting msg to tf
						tf::poseMsgToTF(cylinder.pose,cylinder_tf);
						tf::vector3MsgToTF(cylinder.normal,normal);
						tf::vector3MsgToTF(cylinder.axis,axis);

						// transforming to planning frame
						cylinder_tf = handle_to_planning_tf*cylinder_tf;
						normal = handle_to_planning_tf*normal;
						axis = handle_to_planning_tf*axis;

						tf::Vector3 rz_grasp = normal.normalized();
						tf::Vector3 rx_grasp = axis.normalized();
						tf::Vector3 ry_grasp = (rz_grasp.cross(rx_grasp)).normalized();

						tf::Matrix3x3 rot_grasp = tf::Matrix3x3(rx_grasp.getX(),rx_grasp.getY(),rx_grasp.getZ(),
								ry_grasp.getX(),ry_grasp.getY(),ry_grasp.getZ(),
								rz_grasp.getX(),rx_grasp.getY(),rz_grasp.getZ());

						// creating multiple candidate poses
						geometry_msgs::PoseArray grasp_poses;
						for(int e = 0; e < req.candidates_per_pose;e++)
						{
							// rotating about cylinder axis by the angle
							tf::Quaternion rot_about_axis = tf::Quaternion(rx_grasp,e*angle);
							grasp_tf.setBasis(rot_grasp * tf::Matrix3x3(rot_about_axis));

							// setting position
							grasp_tf.setOrigin(cylinder_tf.getOrigin());

							tf::poseTFToMsg(grasp_tf,grasp_pose);
							grasp_poses.poses.push_back(grasp_pose);
						}
						res.candidate_grasp_poses.push_back(grasp_poses);

						// creating cylinder marker
						visualization_msgs::Marker marker;
						marker.header.frame_id = req.planning_frame_id;
						marker.type = marker.CYLINDER;
						marker.action = marker.ADD;
						//marker.pose
						marker.scale.x = marker.scale.y = cylinder.radius*2;
						marker.scale.z = cylinder.extent;
						res.candidate_objects.markers.push_back(marker);

					}
				}

			}

		}
		else
		{
			//res.succeeded = false;
			return false;
		}

		return true;
	}

	bool transform_to_robot_planning_frame(std::string planning_frame_id,std::string source_frame_id,tf::Transform &t)
	{
		// find transform of handles from tf
		tf::StampedTransform robot_to_target_tf;
		ros::Time query_time = ros::Time::now();

		// equal frames, return identiy
		if(planning_frame_id.compare(source_frame_id) == 0)
		{
			t.setIdentity();
			return true;
		}
		else
		{

			if(!tf_listener_ptr_->waitForTransform(planning_frame_id,source_frame_id,query_time,ros::Duration(5)))
			{
				return false;
			}
		}

		try
		{
			tf_listener_ptr_->lookupTransform(planning_frame_id,source_frame_id,ros::Time(0),robot_to_target_tf);
			t.setOrigin(robot_to_target_tf.getOrigin());
			t.setRotation(robot_to_target_tf.getRotation());
		}
		catch(tf::TransformException &e)
		{
			ROS_ERROR_STREAM("Transform lookup failed");
			return false;
		}
		catch(tf::LookupException &e)
		{
			ROS_ERROR_STREAM("Transform lookup failed");
			return false;
		}

		return true;
	}


protected:

	// members
	std::string world_frame_id_;
	sensor_msgs::PointCloud2 filtered_cloud_msg_;
	sensor_msgs::PointCloud2 sensor_cloud_msg_;

	// ros comm
	ros::ServiceServer target_pose_server_;
	ros::Publisher filtered_cloud_pub_;

	// tf
	tf::TransformListenerPtr tf_listener_ptr_;;
};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"grasp_pose_service");
	GraspPoseServer s;
	s.run();

	return 0;
}
