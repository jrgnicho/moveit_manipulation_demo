/*
 * soem_manager.cpp
 *
 *  Created on: Jul 16, 2014
 *      Author: ros developer 
 */
#include <sstream>
#include <ros/ros.h>

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>

#include <ethercat_soem/ethercattype.h>
#include <ethercat_soem/nicdrv.h>
#include <ethercat_soem/ethercatbase.h>
#include <ethercat_soem/ethercatmain.h>
#include <ethercat_soem/ethercatdc.h>
#include <ethercat_soem/ethercatcoe.h>
#include <ethercat_soem/ethercatfoe.h>
#include <ethercat_soem/ethercatconfig.h>
#include <ethercat_soem/ethercatprint.h>



#define EC_TIMEOUTMON 500

class SoemManager
{
public:

	SoemManager():
		ifname_("eth0"),
		device_name_("")
	{

	}

	~SoemManager()
	{

	}

	bool init()
	{

		char ifname[1024];

		if(load_parameters())
		{
			ROS_INFO_STREAM("Parameters loaded");
		}
		else
		{
			ROS_ERROR_STREAM("Parameters not found, exiting");
			return false;
		}

		// copying characters
		strncpy(ifname,ifname_.c_str(),sizeof(ifname));
		ifname[sizeof(ifname)-1] = 0;

		ROS_INFO_STREAM("Initializing SOEM");

		if(ec_init(ifname))
		{
			ROS_INFO_STREAM("SOEM initialization on "<<ifname_<<" succeeded");

			// configuration
			if(ec_config_init(FALSE)>0)
			{
				ROS_INFO_STREAM("SOEM found and configured "<<ec_slavecount<< "slaves");

				// mapping mailboxes for all found slaves
				ec_config_map(&IOmap_);
				ROS_INFO_STREAM("DC slaves "<< (ec_configdc()?"found": "not found"));

				// wait for all slaves to reach SAFE_OP state
				if(ec_statecheck(0,EC_STATE_SAFE_OP,EC_TIMEOUTSTATE * 4) == EC_STATE_SAFE_OP)
				{
					ROS_INFO_STREAM("SAFE_OP state set");
				}
				else
				{
					ROS_ERROR_STREAM("SAFE_OP state not set, exiting");
					return false;
				}

				// saving output frame on stack for subsequent retrieval
				ec_send_processdata();
				ec_receive_processdata(EC_TIMEOUTRET);

				// setting operation state on all slaves
				ec_writestate(0); // all
				if(ec_statecheck(0,EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE))
				{
					ROS_INFO_STREAM("OPERATIONAL state set");
				}
				else
				{
					ROS_ERROR_STREAM("OPERATIONAL state not set, exiting");
					return false;
				}

				list_devices();

				// finding device
			}
			else
			{
				ROS_ERROR_STREAM("SOEM initialization of all slaves failed");
				return false;
			}

		}
		else
		{
			ROS_ERROR_STREAM("SOEM failed on "<<ifname_ <<" failed to initialize");
			return 	false;
		}

		return true;
	}

	void run()
	{

		ROS_INFO_STREAM("test soem");
		if(init())
		{

		}
	}

protected:

	bool load_parameters()
	{
		ros::NodeHandle ph("~");
		bool success =  ph.getParam("ifname",ifname_) &&
				ph.getParam("device_name",device_name_);
		return success;
	}

	void list_devices()
	{
		if(ec_slavecount > 0)
		{
			std::stringstream ss;
			for(int i =0;i < ec_slavecount;i++)
			{
				ec_slavet &sl = ec_slave[i];
				ss<<"\tSlave["<<i+1<<"]\n";
				ss<<"\t- Name: "<<std::string(sl.name)<<"\n";
				ss<<"\t- Input Bytes: "<<sl.Ibytes<<"\n";
				ss<<"\t- Input Bits: "<<sl.Ibits<<"\n";
				ss<<"\t- Output Bytes: "<<sl.Obytes<<"\n";
				ss<<"\t- Output Bits: "<<sl.Obits<<"\n";
			}

			ROS_INFO_STREAM("Ethercat devices found:\n"<<ss.str());
		}
		else
		{
			ROS_ERROR_STREAM("No Ethercat devices were found");
		}
	}

protected:

	// parameters
	std::string ifname_; // usually eth0
	std::string device_name_;

	char IOmap_[4096];


};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"soem_manager");
	ros::NodeHandle nh;
	SoemManager sm;
	sm.run();
	return 0;
}




