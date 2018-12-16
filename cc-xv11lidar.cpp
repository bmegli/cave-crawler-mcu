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

#include "cc-xv11lidar.h"

#include "xv11lidar.h"

/* Global data */
static XV11Lidar lidar(XV11LIDAR_SERIAL, XV11LIDAR_PWM_PIN );

void setupXV11Lidar()
{
  lidar.setup(XV11LIDAR_RPM);
}

bool processXV11Lidar(xv11lidar_usb_packet &packet)
{
  XV11Packet xv11_packet;
  bool got_packet=lidar.processAvailable(&xv11_packet);

  lidar.applyMotorPID();

  if(!got_packet)
    return false;

  packet.timestamp_us = xv11_packet.timestamp_us;
  packet.angle_quad = xv11_packet.angle_quad;
  packet.speed64 = xv11_packet.speed64;
  memcpy(packet.distances, xv11_packet.distances, sizeof(packet.distances));
  
  return true;
}
