# ros-navbot
Indoor Mapping and Navigation Robot Build with ROS and Nvidia Jetson Nano

Please Go to wiki for more information about the build.

This resipotary is still under progress. I have to clean the codes, mostly for dependencies and launch files. Other than that it is tested and working fine. 

Useage of the each packeges as follows..


differential_drive - I started the build using most of the nodes from this package and eventually ended up replacing most of them with other nodes for various reasons. For final build, only twist to motors node used from this.

kinect_launch - Launch file for depth image to laser scan node.

navbot - Contain following launch files.
  amcl_diff.launch        - launch amcl
	mapping.launch 	        - used to combine and launch several launch files to start robot.
	move_base.launch        - launch fine for ROS Navigation
	navigation.launch 	    - another launch file used to combine and launch several launch files to start robot. 
	rtabmap_mapping.launch  - To start rtabmap
	wifi_comm.launch        - To launch two rosserial nodes to communicate with two esp8266 througth wifi. Node code edited to lauch under different names
  
  navbot_control          - ROS Control parameter files, hardware interface and PID controller Node.
  openslam_gmapping
  slam_gmapping           - To build Gmapping from source as ROS Melodic doesn't have Gmapping from Ubuntu repositories.
