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

#ifndef CC_USB_PROTOCOL
#define CC_USB_PROTOCOL

//use USB as main MCU serial
#define CAVECRAWLER_SERIAL Serial //this is for USB, alternatively use Serial1, Serial2...
#define CAVECRAWLER_SERIAL_TYPE usb_serial_class //this is for USB, alternatively use HardwareSerial

/* the serial/usb packet data */
#include "cc-odometry.h" //struct odometry_usb_packet
#include "cc-xv11lidar.h" //struct xv11lidar_usb_packet
#include "cc-rplidar.h" //struct rplidar_usb_packet

void serialPush(const odometry_usb_packet &packet);
void serialPush(const xv11lidar_usb_packet &packet);
void serialPush(const rplidar_usb_packet &packet);

int serialSend();

#endif
