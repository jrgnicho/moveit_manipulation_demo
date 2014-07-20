/*
 * pick_and_place_service.cpp
 *
 *  Created on: Jul 20, 2014
 *      Author: ros developer 
 */


#include <robot_pick_and_place/pick_and_place.h>

using namespace robot_pick_and_place;

// =============================== Main Thread ===============================
int main(int argc,char** argv)
{

  // ros initialization
  ros::init(argc,argv,"pick_and_place_sevice");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // creating pick and place application instance
  PickAndPlace application;

  if(application.init())
  {
	  application.run_as_service();
  }

  return 0;
}

