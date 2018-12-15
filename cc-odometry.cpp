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
      Serial.println(em7180.getErrorString());       
      delay(1000);
    }   
}

void processOdometry(odometry_usb_packet &packet)
{
  switch(imuState)
  {
  case WAITING_FOR_NEW_DATA:
    if(!imu_new_data)
      return;

    imu_new_data = false;
  
    noInterrupts();
      packet.timestamp_us=imu_timestamp_us;
    interrupts();

    packet.left_encoder_counts=left_encoder.read();
    packet.right_encoder_counts=right_encoder.read();

    imuState=READING_STATUS;
    
  case READING_STATUS:
    if(!em7180.checkEventStatusAsync())
      return;    

    if(Wire.getError())
    {
      Serial.print("Wire ERROR: ");
      Serial.println(Wire.getError(), DEC);
    }
    if (em7180.gotError() || Wire.getError()) 
    {
      Serial.print("em7180 ERROR: ");
      Serial.print(em7180.getErrorString());
      Serial.print("Wire status ");
      Serial.println(Wire.status(), DEC);
      imuState=WAITING_FOR_NEW_DATA;
      return;
    }

    if (!em7180.gotQuaternion())
    {
      imuState=WAITING_FOR_NEW_DATA;
      return;
    }  
    imuState=READING_QUATERNION;    
  case READING_QUATERNION:
    if(!em7180.readQuaternionAsync(packet.qw, packet.qx, packet.qy, packet.qz))
      return;

    if (Wire.getError()) 
    {
      Serial.print("Wire status ");
      Serial.println(Wire.status(), DEC);
      imuState=WAITING_FOR_NEW_DATA;
      return;
    }

    imuState=WAITING_FOR_NEW_DATA;
    //encode & emit packet over USB

    Serial.println("got quaternion");
  }  
}
