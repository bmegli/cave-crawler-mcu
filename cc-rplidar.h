/*
 * cave-crawler-mcu firmware sketch for Teensy 3.5
 *
 * Copyright (C) 2018 Bartosz Meglicki <meglickib@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef CC_RPLIDAR
#define CC_RPLIDAR

#include "cc-common.h"
#include "rplidar.h"

/* Pins */
const int RPLIDAR_PWM_PIN = 2;

const int RPLIDAR_RPM=600; 

/* Serials */
#define RPLIDAR_SERIAL Serial1


typedef RPLidarPacket rplidar_usb_packet;
/* Use RPLidarPacket also as internal for MCU
struct RPLidarPacket
{	
	uint32_t timestamp_us;//timestamp in microseconds
	uint8_t sequence;	  //0-255, wrap-around for checking if packets are consecutive
	uint8_t data[RPLidarPacketDataSize]; //raw measurement packet data
};
*/

/* Functions */ 
void setupRPLidar();
bool processRPLidar(rplidar_usb_packet &packet);

#endif
