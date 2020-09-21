# sparrowhawk
This is the overarching repo for the "sparrowhawk" project, an robot car with simple mapping and autonomy capabilities. (*Sparrowhawk is the name of the rc car used for the robot chassis*) 
There is one overarching GitHub project repo called sparrowhawk (name of the rc car)\
-There are shared files in the top level of the repo\
-There are two subdirectories which are each a ros workspace\
-The first is the laptop workspace called home_base_ws\
  -This is main access point/workstation in which we can access the robot through the pi through ssh and through ros nodes\
-The second is the raspberry pi workspace called control_center_ws\
  -This is where all the action happens and where robot control happens\

When entering these workspaces the first step is to overlay them onto your current environment using the following command from the top level directory
source devel/setup.bash

Within each workspace src folder we have ros packages which are the software building blocks that contain all the useful tools we need and built\
Within home_base_ws we have the home_utils package which contains all the ros nodes, scripts, and files needed to communicate with the robot\

The ROS master will be the laptop
