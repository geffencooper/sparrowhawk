# sparrowhawk
This is the overarching repo for the "sparrowhawk" project, a robot car with simple mapping and autonomous driving capabilities.  
*Sparrowhawk is the name of the rc car used for the robot's chassis*
## Repo Structure
* At the top level of the repo there are generic files including documentation and arduino code
* There are two subdirectories within the top level for the two components of the project. Each of these directories is a **ros workspace**
* The first one is the **laptop** workspace called **home_base_ws**
    * This is main access point/workstation in which we can communicate with the robot (the pi) using ros nodes
    * All computer vision related tasks are executed on the laptop which accesses the video stream from the rpi through *uv4l*
* The second one is the **raspberry pi** workspace called **control_center_ws**
    * This is where all the action happens and where the robot is directly controlled
    * All decision making is executed from the pi which has direct control of the motor servos
## Using the Repo
In this project there are two different computers, the laptop and the Raspberry Pi so one must be assigned as the "ROS master" (In this case the laptop is the master).
* Within each workspace src folder we have **ros packages** which are the software building blocks that contain all the useful tools we need
* Within **home_base_ws** we have the **home_utils** package which contains all the ros nodes, scripts, and files needed to communicate with the robot

