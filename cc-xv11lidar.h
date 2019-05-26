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

#ifndef CC_XV11LIDAR
#define CC_XV11LIDAR

#include <stdint.h> //uint8_t, uint16_t, uint32_T

/* Pins */
const int XV11LIDAR_PWM_PIN = 29;

/* Serials */
#define XV11LIDAR_SERIAL Serial2

/* Vertical lidar */
const int XV11LIDAR_RPM=250;

struct xv11lidar_usb_packet
{
  uint32_t timestamp_us;
  uint8_t angle_quad; //0-89 for readings 0-3 356-359
  uint16_t speed64;    //divide by 64 for speed in rpm 
  uint16_t distances[4]; //flags and distance or error code
};

/* Functions */ 
void setupXV11Lidar();
bool processXV11Lidar(xv11lidar_usb_packet &packet);

#endif
