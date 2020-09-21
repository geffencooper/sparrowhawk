# sparrowhawk
This is the overarching repo for the "sparrowhawk" project, a robot car with simple mapping and autonomous driving capabilities. (*Sparrowhawk is the name of the rc car used for the robot's chassis*) \
## Repo Structure
* At the top level of the repo there are generic files including documentation and arduino code
* There are two subdirectories within the top level for the two components of the project. Each of these directories is a ros workspace
* The first one is the laptop workspace called **home_base_ws**
    * This is main access point/workstation in which we can access the robot through the pi through ssh and through ros nodes
* The second one is the raspberry pi workspace called **control_center_ws**
    * This is where all the action happens and where direct robot control happens

When entering these workspaces the first step is to overlay them onto your current environment using the following command from the top level directory
source devel/setup.bash

Within each workspace src folder we have ros packages which are the software building blocks that contain all the useful tools we need and built\
Within home_base_ws we have the home_utils package which contains all the ros nodes, scripts, and files needed to communicate with the robot\

The ROS master will be the laptop
