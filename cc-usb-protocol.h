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

#ifndef CC_USB_PROTOCOL
#define CC_USB_PROTOCOL

/* the usb packet types */
#include "cc-odometry.h" //struct odometry_usb_packet
#include "cc-xv11lidar.h" //struct xv11lidar_usb_packet
#include "cc-rplidar.h" //struct rplidar_usb_packet

void usb_send_odometry(odometry_usb_packet &packet);
void usb_send_xv11lidar(xv11lidar_usb_packet &packet);
void usb_send_rplidar(rplidar_usb_packet &packet);

#endif
