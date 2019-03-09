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

#ifndef CC_ODOMETRY
#define CC_ODOMETRY

#include <stdint.h> //uint8_t, uint16_t, uint32_T

/* Pins */

const int LEFT_ENCODER_PIN1=24;
const int LEFT_ENCODER_PIN2=25;

const int RIGHT_ENCODER_PIN1=11;
const int RIGHT_ENCODER_PIN2=12;

const int IMU_INTERRUPT_PIN = 17;

/* IMU */
const uint8_t  MAG_RATE       = 100;  // Hz
const uint16_t ACCEL_RATE     = 200;  // Hz
const uint16_t GYRO_RATE      = 200;  // Hz
const uint8_t  BARO_RATE      = 50;   // Hz
const uint8_t  Q_RATE_DIVISOR = 2;    // 1/2 gyro rate

struct odometry_usb_packet
{
  uint32_t timestamp_us;
  int32_t left_encoder_counts;
  int32_t right_encoder_counts;
  float qw,qx,qy,qz; //orientation quaternion
};

/* Functions */ 

void setupOdometry();
bool processOdometry(odometry_usb_packet &packet);

#endif
