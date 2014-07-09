#include <robot_pick_and_place/pick_and_place.h>

using namespace robot_pick_and_place;

// =============================== Main Thread ===============================
int main(int argc,char** argv)
{

  // ros initialization
  ros::init(argc,argv,"pick_and_place_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // creating pick and place application instance
  PickAndPlace application;

  if(application.init())
  {
	  application.run_test();
  }

  return 0;
}
