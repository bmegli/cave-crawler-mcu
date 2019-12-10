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

#include "cc-rplidar.h"
#include "cc-common.h"

static RPLidar lidar_v(RPLIDAR_VERTICAL_SERIAL, RPLIDAR_VERTICAL_PWM_PIN);
static RPLidar lidar_h(RPLIDAR_HORIZONTAL_SERIAL, RPLIDAR_HORIZONTAL_PWM_PIN);


void setupRPLidarVertical()
{
  lidar_v.setup(RPLIDAR_VERTICAL_RPM);
  lidar_v.start();
}

void setupRPLidarHorizontal()
{
  lidar_h.setup(RPLIDAR_HORIZONTAL_RPM);
  lidar_h.start();
}

bool processRPLidarVertical(rplidar_usb_packet &packet)
{
  packet.device_id=RPLIDAR_VERTICAL_ID;
  return lidar_v.processAvailable(&packet.rp);
}

bool processRPLidarHorizontal(rplidar_usb_packet &packet)
{
  packet.device_id=RPLIDAR_HORIZONTAL_ID;
  return lidar_h.processAvailable(&packet.rp);
}
