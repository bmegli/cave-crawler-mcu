/*
 * cave-crawler-mcu firmware sketch for Teensy 3.5
 *
 * Copyright (C) 2018-2019 Bartosz Meglicki <meglickib@gmail.com>
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

/* VERTICAL LIDAR */

#define RPLIDAR_VERTICAL_SERIAL Serial1
const int RPLIDAR_VERTICAL_PWM_PIN = 2;
const int RPLIDAR_VERTICAL_RPM=600;
const uint8_t RPLIDAR_VERTICAL_ID=0;

/* HORIZONTAL LIDAR */
#define RPLIDAR_HORIZONTAL_SERIAL Serial2
const int RPLIDAR_HORIZONTAL_PWM_PIN = 6;
const int RPLIDAR_HORIZONTAL_RPM=600;
const uint8_t RPLIDAR_HORIZONTAL_ID=1;

/* RPLidarPacket defined in rplidar.h from rplidar-a3-arduino
struct RPLidarPacket
{
  uint32_t timestamp_us;//timestamp in microseconds
  uint8_t sequence;   //0-255, wrap-around for checking if packets are consecutive
  uint8_t data[RPLidarPacketDataSize]; //raw measurement packet data
};
*/

struct rplidar_usb_packet
{
  uint8_t device_id;//ID to distinguish multiple lidars
  RPLidarPacket rp; //from rplidar-a3-arduino
};

/* Functions */ 
void setupRPLidarHorizontal();
void setupRPLidarVertical();
bool processRPLidarHorizontal(rplidar_usb_packet &packet);
bool processRPLidarVertical(rplidar_usb_packet &packet);

#endif
