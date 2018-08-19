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

#include <xv11lidar.h>
#include <EM7180.h>
#include <Encoder.h>

#include <i2c_t3.h>

#include <stdint.h> //uint8_t, uint16_t, uint32_T
#include <string.h> //memcpy

/* The protocol 

Start of message byte
Size of message  byte
Type of message  byte
Message payload  variable
End of message   byte

*/

/****** Tunable constansts *********/

/* Pins */

const int LEFT_ENCODER_PIN1=24;
const int LEFT_ENCODER_PIN2=25;

const int RIGHT_ENCODER_PIN1=11;
const int RIGHT_ENCODER_PIN2=12;

const int VERTICAL_LIDAR_PWM_PIN = 35;

const int IMU_INTERRUPT_PIN = 2;

/* Serials */
#define VERTICAL_LIDAR_SERIAL Serial5

/* Sensor setup */

/* Vertical lidar */
const int VERTICAL_LIDAR_RPM=250;

/* IMU */
const uint8_t  MAG_RATE       = 100;  // Hz
const uint16_t ACCEL_RATE     = 200;  // Hz
const uint16_t GYRO_RATE      = 200;  // Hz
const uint8_t  BARO_RATE      = 50;   // Hz
const uint8_t  Q_RATE_DIVISOR = 2;    // 1/2 gyro rate

enum ImuState {WAITING_FOR_NEW_DATA, READING_STATUS, READING_QUATERNION};
ImuState imuState;

/* Data types */

struct imu_packet
{
  uint32_t timestamp_us;
  int32_t left_encoder_counts;
  int32_t right_encoder_counts;
  float qw,qx,qy,qz; //orientation quaternion
};

enum : uint8_t {LIDAR_HORIZONTAL=0, LIDAR_VERTICAL=1};

struct lidar_packet
{
  uint32_t timestamp_us;
  uint8_t lidar_id; //e.g. 0 for horizontal, 1 for vertical, etc
  uint8_t angle_quad; //0-89 for readings 0-3 356-359
  uint16_t speed64;    //divide by 64 for speed in rpm 
  uint16_t distances[4]; //flags and distance or error code
};

/* Functions */ 
volatile bool imu_new_data;
volatile unsigned long imu_timestamp_us;
void imuInterruptHandler();

void processIMU();
void setupIMU();

void processLidar();

void timeStats();

/* Global data */

Encoder left_encoder(LEFT_ENCODER_PIN1, LEFT_ENCODER_PIN2);
Encoder right_encoder(RIGHT_ENCODER_PIN1, RIGHT_ENCODER_PIN2);

XV11Lidar vertical_lidar(VERTICAL_LIDAR_SERIAL, VERTICAL_LIDAR_PWM_PIN );
EM7180_Master em7180 = EM7180_Master(MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);

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

  setupIMU();

  vertical_lidar.setup(VERTICAL_LIDAR_RPM);

  //temp
  last_loop_time_us=micros();
  last_print_time_us=micros();
  //end temp
}

void loop()
{
  processIMU();
  //processLidar();
  
  //vertical_lidar.applyMotorPID();

  timeStats();
}

void processLidar()
{
  XV11Packet xv11_packet;
  bool got_packet=vertical_lidar.processAvailable(&xv11_packet);

  if(!got_packet)
    return;

  lidar_packet packet;
  packet.timestamp_us = xv11_packet.timestamp_us;
  packet.lidar_id = LIDAR_VERTICAL;
  packet.angle_quad = xv11_packet.angle_quad;
  packet.speed64 = xv11_packet.speed64;
  memcpy(packet.distances, xv11_packet.distances, sizeof(packet.distances));
  
  //encode & emit packet over USB
}

void imuInterruptHandler()
{
  imu_timestamp_us=micros();
  imu_new_data = true;    
}

unsigned long max_diff; //temp

void processIMU()
{
  imu_packet packet;
  unsigned long diff;

  switch(imuState)
  {
  case WAITING_FOR_NEW_DATA:
    if(!imu_new_data)
      return;

    imu_new_data = false;
  
    noInterrupts();
      packet.timestamp_us=imu_timestamp_us;
    interrupts();

    diff=micros() - packet.timestamp_us;
    if(diff > max_diff)
      max_diff=diff;

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

    if (em7180.gotQuaternion())
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


void setupIMU()
{
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  
  pinMode(IMU_INTERRUPT_PIN, INPUT);
  attachInterrupt(IMU_INTERRUPT_PIN, imuInterruptHandler, RISING);  

  delay(100);

  if (!em7180.begin())
    while (true) 
      Serial.println(em7180.getErrorString()); 
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
    Serial.print(" max diff ");
    Serial.println(max_diff, DEC);
    
    max_loop_time_us=0;
    max_diff=0;
  }  
}

