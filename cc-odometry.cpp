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
#include "cc-common.h"

#include <EM7180.h>
#include <Encoder.h>
#include <i2c_t3.h>

void imuInterruptHandler();

enum ImuState {WAITING_FOR_NEW_DATA, READING_STATUS, READING_QUATERNION};

static ImuState imuState;
static Encoder left_encoder(LEFT_ENCODER_PIN1, LEFT_ENCODER_PIN2);
static Encoder right_encoder(RIGHT_ENCODER_PIN1, RIGHT_ENCODER_PIN2);
static EM7180_Master em7180 = EM7180_Master(MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);

static volatile bool imu_new_data;
static volatile unsigned long imu_timestamp_us;

void imuInterruptHandler()
{
  imu_timestamp_us=micros();
  imu_new_data = true;
}

void setupOdometry()
{
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);

  pinMode(IMU_INTERRUPT_PIN, INPUT);
  attachInterrupt(IMU_INTERRUPT_PIN, imuInterruptHandler, RISING);

  delay(100);

  if (!em7180.begin())
    while (true)
    {
      DEBUG_SERIAL.println(em7180.getErrorString());
      delay(1000);
    }
}

bool processOdometry(odometry_usb_packet &packet)
{
  //we have to keep those between calls until state machine reads from IMU
  //we should not read them after reading from I2C, this way the readings
  //would not be synchronized
  static uint32_t s_timestamp_us;
  static int32_t s_left_encoder_counts;
  static int32_t s_right_encoder_counts;

  switch(imuState)
  {
  case WAITING_FOR_NEW_DATA:
    if(!imu_new_data)
      return false;

    imu_new_data = false;

    //we should not set packet argument fields in different states of state machine
    //there is no guarantee that the argument is the same between calls
    //so setting fields has to happen right before returning true from the function
    noInterrupts();
      s_timestamp_us=imu_timestamp_us;
    interrupts();

    s_left_encoder_counts=left_encoder.read();
    s_right_encoder_counts=right_encoder.read();

    imuState=READING_STATUS;
  case READING_STATUS:
    if(!em7180.checkEventStatusAsync())
      return false;

    if (em7180.gotError() || Wire.getError())
    {
      DEBUG_SERIAL.print("em7180 ERROR: ");
      DEBUG_SERIAL.print(em7180.getErrorString());
      DEBUG_SERIAL.print("Wire status ");
      DEBUG_SERIAL.println(Wire.status(), DEC);
      imuState=WAITING_FOR_NEW_DATA;
      return false;
    }

    if (!em7180.gotQuaternion())
    {
      imuState=WAITING_FOR_NEW_DATA;
      return false;
    }
    imuState=READING_QUATERNION;
  case READING_QUATERNION:
    if(!em7180.readQuaternionAsync(packet.qw, packet.qx, packet.qy, packet.qz))
      return false;

    if (Wire.getError())
    {
      DEBUG_SERIAL.print("Wire status ");
      DEBUG_SERIAL.println(Wire.status(), DEC);
      imuState=WAITING_FOR_NEW_DATA;
      return false;
    }

    imuState=WAITING_FOR_NEW_DATA;

    //we have to set all the fields together just before sending
    //(not in different states of state machine)
    packet.timestamp_us=s_timestamp_us;
    packet.left_encoder_counts=s_left_encoder_counts;
    packet.right_encoder_counts=s_right_encoder_counts;

    return true;
  }
  return false;
}
