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

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

void tracker_callback(const geometry_msgs::Point::ConstPtr& p)
{
          ROS_INFO("X: [%f]", p->x);
          ROS_INFO("Y: [%f]", p->y);
          ROS_INFO("Z: [%f]", p->z);
	  motor_msg.steering_value = (p->x)*100;
	  uh->tx_msg((uint8_t*)(&motor_msg), sizeof(TX_RPI_DATA));
	  uh->tx_msg((uint8_t*)(&motor_msg), sizeof(TX_RPI_DATA));
	  usleep(10000);
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
