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

#include "cc-rplidar.h"

static RPLidar lidar(RPLIDAR_SERIAL, RPLIDAR_PWM_PIN); 

void setupRPLidar()
{
  lidar.setup(RPLIDAR_RPM);
  lidar.start();
}

void processRPLidar(rplidar_usb_packet &packet)
{
  if( !lidar.processAvailable(&packet) )
    return;

  //encode and emit packet over USB
}
