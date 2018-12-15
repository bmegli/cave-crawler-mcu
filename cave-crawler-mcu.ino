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

#include "cc-odometry.h"
#include "cc-xv11lidar.h"
#include "cc-rplidar.h"

/* The protocol 

Start of message byte
Size of message  byte
Type of message  byte
Timestamp in us  uint32
Message payload  variable
End of message   byte

*/

void timeStats();

//temp time measure
unsigned long last_loop_time_us;
unsigned long last_print_time_us;
unsigned long max_loop_time_us;
//end temp

//temp
#define MEM_LEN 256
char databuf[MEM_LEN];
//endtemp

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("setup...");

  setupOdometry();
  setupXV11Lidar();
  setupRPLidar();

  //temp
  last_loop_time_us=micros();
  last_print_time_us=micros();
  //end temp
}

void loop()
{
  odometry_usb_packet odometry_packet;
  xv11lidar_usb_packet xv11lidar_packet;
  rplidar_usb_packet rplidar_packet;
  
  processOdometry(odometry_packet);
  processXV11Lidar(xv11lidar_packet);
  processRPLidar(rplidar_packet);
  
  timeStats();
}

void timeStats()
{
  unsigned long loop_time, now_us;
  now_us=micros();
  loop_time=now_us-last_loop_time_us;
  last_loop_time_us=now_us;

  if(loop_time>max_loop_time_us)
    max_loop_time_us=loop_time;

  if(now_us-last_print_time_us > 1000000)
  {
    last_print_time_us=now_us;
    Serial.print("current ");
    Serial.print(loop_time, DEC);
    Serial.print(" max ");
    Serial.println(max_loop_time_us, DEC);
    
    max_loop_time_us=0;
  }  
}

