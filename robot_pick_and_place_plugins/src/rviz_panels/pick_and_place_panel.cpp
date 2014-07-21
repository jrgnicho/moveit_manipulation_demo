/*
 * pick_and_place_panel.cpp
 *
 *  Created on: Jul 20, 2014
 *      Author: ros developer 
 */

#include <robot_pick_and_place_plugins/rviz_panels/pick_and_place_panel.h>
#include <pluginlib/class_list_macros.h>

namespace robot_pick_and_place_plugins {
namespace rviz_panels{

const std::string PickAndPlacePanel::PICK_AND_PLACE_SERVICE = "robot_pick_and_place";

PickAndPlacePanel::PickAndPlacePanel(QWidget* parent):
		rviz::Panel(parent)
{
	// TODO Auto-generated constructor stub
	init();

}

PickAndPlacePanel::~PickAndPlacePanel() {
	// TODO Auto-generated destructor stub
}

void PickAndPlacePanel::init()
{
	ui_.setupUi(this);

	ui_.PushButtonPickPlaceRequest->setEnabled(false);
	// connections
	connect(this,SIGNAL(pick_and_place_started()),this,SLOT(pick_and_place_started_handler()));
	connect(this,SIGNAL(pick_and_place_completed()),this,SLOT(pick_and_place_completed_handler()));
	connect(this,SIGNAL(connect_started()),this,SLOT(connect_started_handler()));
	connect(this,SIGNAL(connect_completed()),this,SLOT(connect_completed_handler()));
	connect(ui_.PushButtonPickPlaceRequest,SIGNAL(clicked()),this,SLOT(button_click_handler()));

	Q_EMIT connect_started();

}

void PickAndPlacePanel::onInitialize()
{

	ROS_INFO_STREAM("Pick and Place Service Ready");
}

void PickAndPlacePanel::button_click_handler()
{
	Q_EMIT pick_and_place_started();
}

void PickAndPlacePanel::connect_to_service()
{
	ros::NodeHandle nh;

	pick_and_place_client_ = nh.serviceClient<robot_pick_and_place::RobotPickAndPlace>(PICK_AND_PLACE_SERVICE,true);
	while(!pick_and_place_client_.waitForExistence(ros::Duration(4.0f)))
	{
		ROS_INFO_STREAM("Waiting for "<<PICK_AND_PLACE_SERVICE<<" service");
	}
	Q_EMIT connect_completed();
}

void PickAndPlacePanel::call_service()
{
	robot_pick_and_place::RobotPickAndPlace sr;
	pick_and_place_client_.call(sr);
	Q_EMIT pick_and_place_completed();
}

void PickAndPlacePanel::pick_and_place_started_handler()
{
	ui_.PushButtonPickPlaceRequest->setEnabled(false);

	// call from a separate thread
	QFuture<void> future = QtConcurrent::run(this,&PickAndPlacePanel::call_service);
}

void PickAndPlacePanel::pick_and_place_completed_handler()
{
	ui_.PushButtonPickPlaceRequest->setEnabled(true);
}

void PickAndPlacePanel::connect_started_handler()
{
	// connect from a separate thread
	QFuture<void> future = QtConcurrent::run(this,&PickAndPlacePanel::connect_to_service);
}

void PickAndPlacePanel::connect_completed_handler()
{
	ui_.PushButtonPickPlaceRequest->setEnabled(true);
}

void PickAndPlacePanel::load(const rviz::Config& config)
{
	rviz::Panel::load(config);
}

void PickAndPlacePanel::save(rviz::Config config) const
{
	ROS_INFO_STREAM("Saving configuration");
	rviz::Panel::save(config);
}

}
} /* namespace robot_pick_and_place_plugins */

PLUGINLIB_EXPORT_CLASS(robot_pick_and_place_plugins::rviz_panels::PickAndPlacePanel,rviz::Panel )
