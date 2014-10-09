moveit_manipulation_demo
========================


###############################################################################################################
INSTRUCTIONS TO START DEMO

 	1 - Open ROS ready terminal

		1.1 - In a new terminal run the following to bring up the "ROS" terminals
			
					'ros_session hydro'
			
			This will bring up 7 terminal tabs with ROS fully configured
	---------------------------------------------------------------------------------------------
	
	2 - FULL Hardware Demo

			2.1	- In a ROS terminal run the following:

					'roslaunch robot_pick_and_place ur5_demo.launch sim_sensor:=false sim_gripper:=false sim_robot:=false robot_ip:=192.168.32.5'

				This will bring up rviz and all of the nodes.


			2.2 - Rviz Start
							- Click the "Run" button in the rviz window to start the demo.  The button will became disable while the demo is running.

					 		- The "Run" button will became enabled when there aren't any objects in the table.  
	---------------------------------------------------------------------------------------------


	3 - (Optional) Full Virtual Demo

			3.1	- In a ROS terminal run the following:

					'roslaunch robot_pick_and_place ur5_demo.launch'

				This will bring up rviz and all of the nodes.


			3.2 - Rviz Start
							- Click the "Run" button in the rviz window to start the demo.  The button will became disable while the demo is running.

########################################################################################################################
INSTRUCTIONS TO CALIBRATE SENSOR

	1 - Move the robot in the field of view of the camera.


	2 -	In a ROS terminal run the following to bring up the robot drive, camera and rviz
		
				'roslaunch robot_pick_and_place ur5_setup.launch sim_sensor:=false sim_robot:=false robot_ip:=192.168.32.5'


	3 -	In another ROS terminal run the following to bring up the manual calibration help routine

				'roslaunch industrial_pcl_utilities pointcloud_transform.launch'


	4 - In rviz, select the "/cloud_out" topic under the the PointCloud2 display in order to view the adjustable point cloud


	5 - In yet another ROS terminal, for example set "y" position calibration parameter to "0.6" by running the following:

				'rosparam set /pointcloud_transform_node/parent_to_child_pose/y 0.6'

			This will move the the point cloud in rviz to the corresponding new position.


	6 -  Similarly, the other calibration parameters can be modified through the rosparam command:

 				"rosparam set /pointcloud_transform_node/parent_to_child_pose/[variable name] [new value]"

				Where "variable name" can take of the values of x, y, z, rx, ry, rz and the values can be any double value representation.


	7 - Keep adjusting the calibration parameters until the robot from the point cloud aligns with the robot model in rviz.

			
	8 - In an open ROS terminal run the following to obtain the final calibration transform:

				'rosrun tf tf_echo kinect_parent_link kinect_child_link'

			After the first print out message pres Ctrl-C and the data should look as shown below:

			- Translation: [0.000, -0.030, 0.000]
			- Rotation: in Quaternion [-0.500, 0.500, -0.500, 0.500]
            in RPY [-1.570, -0.000, -1.570]

	9 - In another ROS terminal open the robot workcell xacro file in order to enter the calibration data

				'roscd robot_model_config'
				'gedit urdf/ur5_2fgripper.xacro &'


	10 - In line 181 of the xacro file, enter the values that were obtained from step 8 then save.  At this point the sensor is calibrated.  This is the end.

########################################################################################################################
INSTRUCTIONS TO CHANGE THE PLACE LOCATION

	1 - In a ROS terminal, go to the robot_pick_and_place package and open the file
				'roscd robot_pick_and_place'
				'gedit config/ur5/pick_and_place_parameters.yaml &'

	2 - Changed the parameters under line 19 for the "world_to_place_pose", then save the file and re-run the demo.  To check that this pose is
			reachable run the demo in simulation mode.

	
