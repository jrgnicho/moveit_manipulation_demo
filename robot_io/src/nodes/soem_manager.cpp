/*
 * soem_manager.cpp
 *
 *  Created on: Jul 16, 2014
 *      Author: ros developer 
 */
#include <sstream>
#include <ros/ros.h>
#include <robot_io/SoemIO.h>
#include <boost/assign.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/assign/std/vector.hpp>

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

using namespace boost::assign;

#define EC_TIMEOUTMON 500

typedef std::vector<uint8_t> BitArray;

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

		if(load_parameters())
		{
			ROS_INFO_STREAM("Parameters loaded");
		}
		else
		{
			ROS_ERROR_STREAM("Parameters not found, exiting");
			return false;
		}

		if(!init_soem())
		{
			return false;
		}

		return true;
	}

	void run()
	{


		ROS_INFO_STREAM("test bit|byte conversions");
		test_conversions();

		if(init())
		{
			if(test_activate_gripper())
			{
				ROS_INFO_STREAM("Gripper activated");
				return;
			}
			else
			{
				ROS_ERROR_STREAM("Gripper activate failed, exiting");
				return;
			}

			if(test_close_gripper())
			{
				ros::Duration(4.0f).sleep();
				ROS_INFO_STREAM("Gripper closed");
			}
			else
			{
				ROS_ERROR_STREAM("Gripper close failed, exiting");
				return;
			}

			if(test_open_gripper())
			{

				ros::Duration(4.0f).sleep();
				ROS_INFO_STREAM("Gripper opened");
			}
			else
			{
				ROS_ERROR_STREAM("Gripper open failed, exiting");
				return;
			}

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
			for(int i =1;i <= ec_slavecount;i++)
			{
				ec_slavet &sl = ec_slave[i];
				ss<<"\tSlave["<<i<<"]\n";
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

	bool init_soem()
	{
		// copying characters
		char ifname[1024];
		strncpy(ifname,ifname_.c_str(),sizeof(ifname));
		ifname[sizeof(ifname)-1] = 0;

		ROS_INFO_STREAM("Initializing SOEM");

		if(ec_init(ifname))
		{
			ROS_INFO_STREAM("SOEM initialization on "<<ifname_<<" succeeded");

			// configuration
			if(ec_config_init(FALSE)>0)
			{
				ROS_INFO_STREAM("SOEM found and configured "<<ec_slavecount<< " slaves");

				// mapping mailboxes for all found slaves
				ec_config_map(&IOmap_);
				ec_configdc();
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
			ROS_ERROR_STREAM("SOEM on "<<ifname_ <<" failed to initialize");
			return 	false;
		}

		return true;
	}

	void test_conversions()
	{
		std::vector<uint8> bytes;
		uint8 result;
		BitArray bit_array;

		// creating array of test bytes
		bytes.push_back(12);
		bytes.push_back(52);
		bytes.push_back(14);
		bytes.push_back(72);
		bytes.push_back(126);


		for(int i = 0;i < bytes.size();i++)
		{
			byte_to_bits(bytes[i],bit_array);
			result = bits_to_byte(bit_array);

			ROS_INFO_STREAM("Byte "<<(int)bytes[i]<<" -> "<<bits_to_str(bit_array)<<
					" -> "<<(int)result);
		}

	}

	uint8 bits_to_byte(const BitArray& bit_array)
	{
		uint8 byte = 0;
		for(int i = 0;i < bit_array.size(); i++)
		{
			byte = bit_array[i]<<i | byte;
		}

		return byte;
	}

	void byte_to_bits(uint8 byte, BitArray& bit_array)
	{
		bit_array.resize(8);
		for(int i=0;i < 8;i++)
		{
			uint8_t val = 1 << i;
			bit_array[i] = (byte & (val)) == val ? 1 : 0;
		}

	}

	std::string bits_to_str(const BitArray& bit_array)
	{
		std::stringstream ss;
		ss<<"[";
		for(int i=0;i < 8;i++)
		{
			ss<<(int)bit_array[i]<<" ";
		}
		ss<<"]";
		return ss.str();
	}

	bool test_activate_gripper()
	{
		robot_io::RegisterData out1; //action requested byte
		robot_io::SoemIO::Request out_registers;
		out_registers.slave_no = 1;

		out1.bits = list_of(1)(0)(0)(0)(0)(0)(0)(0);// activate gripper
		out1.info.channel=0;

		// saving output registers
		out_registers.output_data.push_back(out1);

		// writing registers
		for(int i = 0; i < out_registers.output_data.size();i++)
		{
			if(!write_register(out_registers.slave_no,out_registers.output_data[i]))
			{
				ROS_ERROR_STREAM("write registers operation failed");
				return false;
			}
		}

		if(ec_send_processdata()<=0)
		{
			ROS_ERROR_STREAM("Process data was not transmitted");
			return false;
		}

		ros::Duration(2.0f).sleep();
		// read gripper status input register
		robot_io::RegisterData in1;
		in1.info.channel = 0;


		// reading all input registers
		ec_receive_processdata(EC_TIMEOUTRET);

		for(int i = 0;i < 6;i++)
		{
			in1.info.channel=i;

			if(read_register(out_registers.slave_no,in1.info,in1))
			{
				ROS_INFO_STREAM("Input register ["<<i<<"]: "<<bits_to_str(in1.bits));
			}
			else
			{
				ROS_ERROR_STREAM("Input register ["<<i<<"] read error");
				return false;
			}
		}

		return true;
	}


	bool test_open_gripper()
	{
		robot_io::RegisterData out1; //action requested byte
		robot_io::RegisterData out3; // position requested byte
		robot_io::RegisterData out4; // speed byte ( 0(min) to (255) max )
		robot_io::SoemIO::Request out_registers;
		out_registers.slave_no = 1;


		out1.bits = list_of(1)(0)(0)(1)(0)(0)(0)(0);// go to requested position
		out1.info.channel=0;

		out3.bits = list_of(0)(0)(0)(0)(0)(0)(0)(0);// fully opened
		out3.info.channel=2;

		out4.bits = list_of(0)(0)(0)(0)(0)(0)(0)(0);// slowest
		out4.info.channel=3;

		// saving output registers
		out_registers.output_data.push_back(out1);
		out_registers.output_data.push_back(out3);
		out_registers.output_data.push_back(out4);

		// writing registers
		for(int i = 0; i < out_registers.output_data.size();i++)
		{
			if(!write_register(out_registers.slave_no,out_registers.output_data[i]))
			{
				ROS_ERROR_STREAM("write registers operation failed");
				return false;
			}
		}

		ec_send_processdata();

		return true;
	}

	bool test_close_gripper()
	{
		robot_io::RegisterData out1; //action requested byte
		robot_io::RegisterData out3; // position requested byte
		robot_io::RegisterData out4; // speed byte ( 0(min) to (255) max )
		robot_io::SoemIO::Request out_registers;
		out_registers.slave_no = 1;


		out1.bits = list_of(1)(0)(0)(1)(0)(0)(0)(0);// go to requested position
		out1.info.channel=0;

		out3.bits = list_of(1)(1)(1)(1)(1)(1)(1)(1);// fully closed
		out3.info.channel=2;

		out4.bits = list_of(0)(0)(0)(0)(0)(0)(0)(0);// slowest
		out4.info.channel=3;

		// saving output registers
		out_registers.output_data.push_back(out1);
		out_registers.output_data.push_back(out3);
		out_registers.output_data.push_back(out4);

		// writing registers
		for(int i = 0; i < out_registers.output_data.size();i++)
		{
			if(!write_register(out_registers.slave_no,out_registers.output_data[i]))
			{
				ROS_ERROR_STREAM("write registers operation failed");
				return false;
			}
		}

		ec_send_processdata();

		return true;

	}

	bool write_register(int slave_no,const robot_io::RegisterData& regist)
	{
		if(slave_no<= ec_slavecount 	)
		{
			// case when theres less than 8 bits available in register
			if((ec_slave[slave_no].Obytes== 0) &&
					(regist.info.channel != ec_slave[slave_no].Obytes))
			{
				ROS_ERROR_STREAM("Byte index greater than 0 not permitted");
				return false;
			}

			if((ec_slave[slave_no].Obytes> 0) &&
					(regist.info.channel >= ec_slave[slave_no].Obytes))
			{
				ROS_ERROR_STREAM("Byte index exceeds output bytes available");
				return false;
			}

			write_output(slave_no, regist.info.channel,bits_to_byte(regist.bits));
		}
		else
		{
			ROS_ERROR_STREAM("slave index "<<slave_no<<" does not exists");
			return false;
		}

		return true;
	}

	bool read_register(int slave_no,const robot_io::RegisterInfo& info,robot_io::RegisterData& regist)
	{

		if(slave_no<= ec_slavecount 	)
		{
			// case when theres less than 8 bits available in register
			if((ec_slave[slave_no].Ibytes== 0) &&
					(info.channel != ec_slave[slave_no].Ibytes))
			{
				ROS_ERROR_STREAM("Byte index greater than 0 not permitted");
				return false;
			}

			if((ec_slave[slave_no].Ibytes> 0) &&
					(info.channel >= ec_slave[slave_no].Ibytes))
			{
				ROS_ERROR_STREAM("Byte index exceeds input bytes available");
				return false;
			}

			uint8 val;
			read_input(slave_no,info.channel,&val);
			byte_to_bits(val,regist.bits);
			regist.info = info;
		}
		else
		{
			ROS_ERROR_STREAM("slave index "<<slave_no<<" does not exists");
			return false;
		}

		return true;
	}

	void write_output(int slave_no,uint8 channel,uint8 val)
	{
		uint8 *ptr;
		ptr = ec_slave[slave_no].outputs;
		ptr += channel;

		*ptr = val;

		ROS_INFO_STREAM("output register ["<<(int)channel<<"] "<<(int)*ptr);
	}

	void read_input(int slave_no,uint8 channel,uint8* val)
	{
		uint8 *ptr;
		ptr = ec_slave[slave_no].inputs;
		ptr+=channel;
		*val = *ptr;

		ROS_INFO_STREAM("input register ["<<(int)channel<<"] "<<(int)*val);
	}

protected:

	// ros comm
	ros::ServiceServer soem_io_server_;

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




