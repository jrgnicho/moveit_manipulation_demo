/*
 * pick_and_place_panel.h
 *
 *  Created on: Jul 20, 2014
 *      Author: ros developer 
 */

#ifndef PICK_AND_PLACE_PANEL_H_
#define PICK_AND_PLACE_PANEL_H_

#include <ros/ros.h>
#include <robot_pick_and_place/RobotPickAndPlace.h>
#include <rviz/panel.h>

// qt headers
#include <ui_pick_and_place_widget.h>
#include <QtConcurrentRun>

namespace robot_pick_and_place_plugins {
namespace rviz_panels{

class PickAndPlacePanel: public rviz::Panel {
Q_OBJECT
public:
	PickAndPlacePanel(QWidget* parent = 0);
	virtual ~PickAndPlacePanel();

	virtual void onInitialize();

Q_SIGNALS:

	void pick_and_place_started();
	void pick_and_place_completed();
	void connect_started();
	void connect_completed();

public:

	static const std::string PICK_AND_PLACE_SERVICE;

protected Q_SLOTS:

	virtual void load(const rviz::Config& config);
	virtual void save(rviz::Config config) const;

	void pick_and_place_started_handler();
	void pick_and_place_completed_handler();
	void connect_started_handler();
	void connect_completed_handler();
	void button_click_handler();

protected:

	void init();
	void connect_to_service();
	void call_service();

protected:

	Ui::PickAndPlaceWidget ui_;
	ros::ServiceClient pick_and_place_client_;

};

}
} /* namespace robot_pick_and_place_plugins */
#endif /* PICK_AND_PLACE_PANEL_H_ */
