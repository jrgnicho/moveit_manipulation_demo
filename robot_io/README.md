#### Instructions for SOEM setup

* Enable root access to socket commands
```
	cd ~/[catkin_workspace directory]/devel/lib/robot_io
	sudo setcap cap_net_raw+ep soem_manager
```
