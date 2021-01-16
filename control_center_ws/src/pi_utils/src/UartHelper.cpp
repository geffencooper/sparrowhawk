#include "UartHelper.h"
#include "string.h"
#include "stdio.h"
#include <unistd.h>
#include <wiringSerial.h>

// function to send data over uart, pass in a pointer to the buffer, data size
void UartHelper::tx_msg(uint8_t* data, int size)
{
	for(int i = 0; i < size; i++)
	{
		serialPutchar(fd, data[i]);
	}
}
	
// function to receive data over uart, will wait for start code 
uint8_t UartHelper::rx_msg(uint8_t* buffer, int buffer_size, uint32_t start_code)
{
	uint8_t result = find_start(start_code);
	if(result == 0)
	{
		return 0;
	}

	// copy the start code into the buffer
	memcpy(buffer, start_code_buffer, 4);

	// get the rest of the message
	for(int i = 4; i < buffer_size; i++)
	{
		buffer[i] = serialGetchar(fd);
	}
	memset(start_code_buffer, 0, 4);
	return 1;
}

uint8_t UartHelper::find_start(uint32_t start_code)
{
	// copy the start code into a buffer
	static uint8_t start[4];
	memcpy(start, &start_code, 4);

	int i = 0;
	int count = 0;
	while(true)
	{
		// read the next byte from the uart
		char next = serialGetchar(fd);

		// if it is equal to the first letter of the start code then start copying byte by byte
		if(next == start[i])
		{
			start_code_buffer[i] = next;
			i++;
			if(i == 4)
			{
				return 1;
				count = 0;
			}
		}
		else// otherwise start over
		{
			//printf("%02X\n", next);
			i = 0;
		      //  count = 0;
		}
		count++;
		if(count > 100) // 40 is max in case start reading in middle of first message
		{
			return 0;
		}
	}
}
