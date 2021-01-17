#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

extern "C"{
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringSerial.h>
#include "stdio.h"
#include "unistd.h"
}

#include "UartHelper.h"

int serial_port;
char dat;
UartHelper* uh;
TX_RPI_DATA motor_msg;
RX_STM_DATA sensor_data;
bool result;
int count;
int steer;
int drive;
geometry_msgs::Point::ConstPtr point;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
enum CarState
{
	FORWARD = 0, // in view but far
	BACKWARD, // out of view and reversing
	OOV_WAIT, // out of view before trying to reverse
	STOPPED, // object lost or in field of view, stay put
	BREAKING, // breaking when object gets close in view
	LOST // object outside view and tried search, wait till reenters FOV
};

enum ObjectState
{
	IN_FOV_FAR = 0, // can see the object but far so drive to it
	IN_FOV_CLOSE, // in fov, stay, equilibrium
	OUT_OF_FOV // can't find object
};

void follow_object()
{
	motor_msg.steering_value = 100*point->x;
	motor_msg.drive_value = 60;
}

#define BREAK_PERIOD 5 // reverse for 50 frames
#define OUT_OF_VIEW_PERIOD 5 // wait 100 frames before reversing
#define BACKWARD_PERIOD 3 // reverse for 100 frames
void tracker_callback(const geometry_msgs::Point::ConstPtr& p)
{
	  static float last_z = 0;
	  static int oov_count = 0;
	  static int backward_count = 0;
	  static int break_count = 0;
	  static CarState car_state = LOST;
	  static ObjectState object_state = OUT_OF_FOV;

	  point = p;

          ROS_INFO("X: [%f]", p->x);
          ROS_INFO("Y: [%f]", p->y);
          ROS_INFO("Z: [%f]  last_z: [%f]", p->z, last_z);
	  
	  // first determine the object state
	  // object area same so left the frame
	 // float delta_z1 = p->z-last_z;
	 // float delta_z2 = last_z-p->z;
	 // if(((delta_z1 < 0.01) && delta_z1 > -0.01) || ((delta_z2< 0.01) && delta_z2 > -0.01))
	  if(p->z == last_z)
	  {
		  ROS_INFO("OBJECT STATE: OUT OF VIEW");
		  object_state = OUT_OF_FOV;
	  }
          // object area outside threshold
	  else if((p->z > 100) && (p->z < 100000))
	  {
		ROS_INFO("OBJECT STATE: IN FOV FAR");
		object_state = IN_FOV_FAR;
	  }
	  else if(p->z > 100000)
	  {
		  ROS_INFO("OBJECT_STATE: IN FOV CLOSE");
		  object_state = IN_FOV_CLOSE;
	  }

	  // next determine the car state transition
	  switch(car_state)
	  {
		  // equilibrium state
		  case(STOPPED):
		  {
			  // object moved away, follow it
			  if(object_state == IN_FOV_FAR)
			  {
				  car_state = FORWARD;
				  follow_object();
			  }
			  // lost object, enter a waiting state
			  else if(object_state == OUT_OF_FOV)
			  {
				  car_state = OOV_WAIT;
			  }
			  else if(object_state == IN_FOV_CLOSE)
			  {
				  motor_msg.steering_value = 100*p->x;
			  }
			  break;
		  }
		  // following an object
		  case(FORWARD):
		  {
			  // if the object gets to equilibrium or lost then break
			  if((object_state == IN_FOV_CLOSE) || (object_state == OUT_OF_FOV))
			  {
				  car_state = BREAKING;
				  motor_msg.drive_value = -20;
			  }
			  // otherwise keep following and update steering
			  else
			  {
				  follow_object();
			  }
			  break;
		  }
		  // breaking state
		  case(BREAKING):
		  {
			  // reverse for small period
			  if(break_count <= BREAK_PERIOD)
			  {
				  break_count++;
			  }
			  // now enter equilibrium
			  else
			  {
				  break_count = 0;
				  car_state = STOPPED;
				  motor_msg.drive_value = 0;
			  }
			  break;
		  }
		  // when object is lost enter a waiting period
		  case(OOV_WAIT):
		  {
			  // if period passed then try to go backward to increase FOV
			  if(oov_count > OUT_OF_VIEW_PERIOD)
			  {
				  oov_count = 0;
				  car_state = BACKWARD;
				  ROS_INFO("GOING BACKWARD----------------");
				  motor_msg.drive_value = -50;
			  }
			  // if object returned to FOV, follow it
			  else if(object_state == IN_FOV_FAR)
			  {
				  oov_count = 0;
				  car_state = FORWARD;
				  follow_object();
			  }
			  // if object retturned to FOV and close
			  else if(object_state == IN_FOV_CLOSE)
			  {
				  oov_count = 0;
				  car_state = STOPPED;
			  }
			  // pause for small period, object may return
			  else if(oov_count <= OUT_OF_VIEW_PERIOD)
			  {
				  ROS_INFO("wait count [%i]", oov_count);
				  oov_count++;
			  }
			  break;
		  }
		  case(BACKWARD):
		  {
			  // if period passed then enter a lost state
			  if(backward_count > BACKWARD_PERIOD)
			  {
				  backward_count = 0;
				  motor_msg.steering_value = 0;
				  motor_msg.drive_value = 0;
				  car_state = LOST;
			  }
			  // if returned to FOV far
			  else if(object_state == IN_FOV_FAR)
			  {
				  backward_count = 0;
				  car_state = FORWARD;
				  follow_object();
			  }
			  else if(object_state == IN_FOV_CLOSE)
			  {
				  car_state = STOPPED;
				  motor_msg.drive_value = 0;
				  backward_count = 0;
			  }
			  // reverse for small period, change this if add another sensor for reverse
			  else if(backward_count <= BACKWARD_PERIOD)
			  {
				  ROS_INFO("backward count [%i]", backward_count);
				  backward_count++;
			  }
			  break;
		  }
		  case(LOST):
		  {
			  if(object_state == IN_FOV_FAR)
			  {
				  car_state = FORWARD;
				  follow_object;
			  }
			  else if(object_state == IN_FOV_CLOSE)
			  {
				  car_state = STOPPED;
				  motor_msg.drive_value = 0;
				  motor_msg.steering_value = 100*p->x;
			  }
			  break;
		  }
	  }

	  // transmit motor commands over UART
	  uh->tx_msg((uint8_t*)(&motor_msg), sizeof(TX_RPI_DATA));
	  uh->tx_msg((uint8_t*)(&motor_msg), sizeof(TX_RPI_DATA));
	  usleep(10000);
	
	  // keep track of last z	  
	  last_z = p->z;

	  // display state
	  switch(car_state)
	  {
		  case(FORWARD): 
	          {
			  ROS_INFO("CAR STATE: FORWARD");
			  break;
	          }
		  case(BACKWARD): 
	          {
			  ROS_INFO("CAR STATE: BACKWARD");
			  break;
	          }
		  case(OOV_WAIT): 
	          {
			  ROS_INFO("CAR STATE: OUT OF VIEW... WAITING");
	          	  break;
		  }
		  case(LOST): 
	          {
			  ROS_INFO("CAR STATE: NO OBJECT, LOST");
			  break;
	          }
		  case(STOPPED): 
	          {
			  ROS_INFO("CAR STATE: STOPPED");
			  break;
	          }
		  case(BREAKING): 
	          {
			  ROS_INFO("CAR STATE: BREAKING");
			  break;
	          }
	  }
}

void key_press_callback(const std_msgs::String::ConstPtr& msg)
{
  if(msg->data == "R")
  {
  	  ROS_INFO("I heard: [%s]", msg->data.c_str());
	  motor_msg.steering_value = 90;
  }
  else if(msg->data == "L")
  {
  	  ROS_INFO("I heard: [%s]", msg->data.c_str());
	  motor_msg.steering_value = -90;
  }
  else if(msg->data == "PT")
  {
  	  ROS_INFO("I heard: [%s]", msg->data.c_str());
	  motor_msg.steering_value = 1;
  }
  else if(msg->data == "UL")
  {
  	  ROS_INFO("I heard: [%s]", msg->data.c_str());
	  motor_msg.steering_value = -90;
	  motor_msg.drive_value = 35;
  }
  else if(msg->data == "UR")
  {
  	  ROS_INFO("I heard: [%s]", msg->data.c_str());
	  motor_msg.steering_value = 90;
	  motor_msg.drive_value = 35;
  }
  else if(msg->data == "PD")
  {
  	  ROS_INFO("I heard: [%s]", msg->data.c_str());
	  motor_msg.drive_value = 1;
  }
  else if(msg->data == "DL")
  {
  	  ROS_INFO("I heard: [%s]", msg->data.c_str());
	  motor_msg.steering_value = -90;
	  motor_msg.drive_value = -35;
  }
  else if(msg->data == "DR")
  {
  	  ROS_INFO("I heard: [%s]", msg->data.c_str());
	  motor_msg.steering_value = 90;
	  motor_msg.drive_value = -35;
  }
  else if(msg->data == "D")
  {
  	  ROS_INFO("I heard: [%s]", msg->data.c_str());
	  motor_msg.drive_value = -50;
  }
  else if(msg->data == "U")
  {
  	  ROS_INFO("I heard: [%s]", msg->data.c_str());
	  motor_msg.drive_value = 50;
  }
  uh->tx_msg((uint8_t*)(&motor_msg), sizeof(TX_RPI_DATA));
  uh->tx_msg((uint8_t*)(&motor_msg), sizeof(TX_RPI_DATA));
  usleep(10000);
}


int main(int argc, char **argv)
{

  if ((serial_port = serialOpen ("/dev/ttyS0", 38400)) < 0)     /* open serial port */
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }

  if (wiringPiSetup () == -1)                                   /* initializes wiringPi setup */
  {
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    return 1 ;
  }

  //serialFlush(serial_port);
  UartHelper uhlpr = UartHelper(serial_port);
  uh = &uhlpr;
  motor_msg = {RPI_START_CODE, 0, 0, 0};
  result = 0;
  count = 0;
  steer = 0;
  drive = 0;
      

	/**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  printf("init node\n");
  ros::init(argc, argv, "stm_com");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("keys_pressed", 1000, key_press_callback);
  ros::Subscriber sub2 = n.subscribe("tracker", 1000, tracker_callback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
