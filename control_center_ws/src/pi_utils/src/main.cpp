#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringSerial.h>
#include "UartHelper.h"
#include "stdio.h"
#include "unistd.h"

int main ()
{
  int serial_port;
  char dat;
  if ((serial_port = serialOpen ("/dev/ttyS0", 38400)) < 0)	/* open serial port */
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }

  if (wiringPiSetup () == -1)					/* initializes wiringPi setup */
  {
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    return 1 ;
  }

  //serialFlush(serial_port);
  UartHelper uh = UartHelper(serial_port);
  TX_RPI_DATA motor_msg = {RPI_START_CODE, 0, 0, 0};
  RX_STM_DATA sensor_data; 
  bool result = 0;
  int count = 0;
  int steer = 0;
  int drive = 0;

  #define TX
//  #define RX

  char res;
  while(true)
  {
        #ifdef RX
  	while(serialDataAvail (serial_port) <=0){}//printf("waiting for data...\n");}
	
	result = uh.rx_msg((uint8_t*)(&sensor_data), sizeof(RX_STM_DATA), STM_START_CODE);
        if(result != 0)
   	{	
//  		 printf("start code: %08X\nframe_id: %i\nultrasonic_data: %i\ntof1_data: %f\ntof2_data: %f\nencoder_distance: %f\n", sensor_data.start_code, sensor_data.frame_id, sensor_data.ultrasonic_data, sensor_data.tof1_data, sensor_data.tof2_data, sensor_data.encoder_distance);
  	//	printf("encoder_data: %f rots\n", sensor_data.encoder_distance);
  		printf("ultrasonic_data: %i cm\n", sensor_data.ultrasonic_data);
	}
        #endif
	


	#ifdef TX
	
        printf("\nenter steer: ");
        scanf("%i", &steer);

	printf("\nenter drive: ");
	scanf("%i", &drive);
	
	motor_msg.steering_value = steer;
	motor_msg.drive_value = drive;
		
//	printf("%08X %i %f %f %i\n", motor_msg.start_code, motor_msg.frame_id, motor_msg.steering_value, motor_msg.drive_value, sizeof(TX_RPI_DATA));	
	uh.tx_msg((uint8_t*)(&motor_msg), sizeof(TX_RPI_DATA));
	uh.tx_msg((uint8_t*)(&motor_msg), sizeof(TX_RPI_DATA));
	usleep(10000); // 10ms
	printf("sent\n");
//	uh.tx_msg((uint8_t*)(&motor_msg), sizeof(TX_RPI_DATA));

	//	printf("%c\n", serialGetchar(serial_port));
//	motor_msg.frame_id++;

	#endif
  }
}
