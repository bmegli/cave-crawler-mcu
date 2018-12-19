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

#include "cc-usb-protocol.h"

#include "serial-nb.h" //non blocking serial communication

#include <string.h> //memcpy

/* global values */
static SerialNB<CAVECRAWLER_SERIAL_TYPE> g_serial(CAVECRAWLER_SERIAL);

/*
 *
 * TEMP
 * - for now ignore data if no place in buffer
 * - for now communication is simple little endian
 * - for now shift buffer (instead of circular buffer)
 * 
 *
## The packet structure

|          |  Start Byte   |  Size     | Type            |      Payload   | End Byte      |
| ---------|---------------|-----------|-----------------|----------------|---------------|
|   bytes  |    1          |      1    |    1            | type dependent |     1         |
|   value  | fixed 0xFB    |  0-255    | defined set     | type dependent |   fixed 0xFC  |

- CRC is already included in USB (not needed)
- Start Byte, Size, Type, End Byte can all be used for sync
- Size includes all the bytes ( payload bytes + 4)
- Payload starts with 4 bytes timestamp in microseconds
*/

enum {USBNB_START_BYTE=0xFB, USBNB_END_BYTE=0xFC};

/*  
## Types

Preliminary

|  Type       | Value  | Payload bytes |  Info                                                         |
| ------------|--------|---------------|-----------------------------------------|
|  ODOMETRY   | 0x01   |      28       |  encoders and IMU quaternions           |
|  XV11LIDAR  | 0x02   |      15       |  lidar data                             |
|  RPLIDARA3  | 0x03   |     137       |  compressed ultra capsules, sequence    |

*/

enum {USBNB_ODOMETRY_TYPE=0x01, USBNB_XV11LIDAR_TYPE=0x02, USBNB_RPLIDAR_TYPE=0x03};
enum {USBNB_ODOMETRY_BYTES=28+4, USBNB_XV11LIDAR_BYTES=15+4, USBNB_RPLIDAR_BYTES=137+4};

/*
### ODOMETRY

|           | Timestamp | Left encoder | Right encoder |    QW      |     QX     |    QY      |     QZ     |
| ----------|-----------|--------------|---------------|------------|------------|------------|------------|
|   bytes   |    4      |      4       |      4        |     4      |      4     |     4      |     4      |
|   type    | uint32    |   int32      |    int32      |   float    |   float    |   float    |   float    |
|   unit    |   us      |   counts     |    counts     | quaternion | quaternion | quaternion | quaternion |
*/
void serialPush(const odometry_usb_packet& packet)
{
	if(g_serial.free_bytes() < USBNB_ODOMETRY_BYTES)
		return; 

	g_serial.push((uint8_t)USBNB_START_BYTE);
	g_serial.push((uint8_t)USBNB_ODOMETRY_BYTES);
	g_serial.push((uint8_t)USBNB_ODOMETRY_TYPE);
	
	g_serial.push(packet.timestamp_us);
	g_serial.push(packet.left_encoder_counts);
	g_serial.push(packet.right_encoder_counts);
	g_serial.push(packet.qw);
	g_serial.push(packet.qx);
	g_serial.push(packet.qy);
	g_serial.push(packet.qz);
	
	g_serial.push((uint8_t)USBNB_END_BYTE);
}

/*
### XV11LIDAR

|          | Timestamp | Angle quad            |  Speed64       | Distances x 4 [mm]            |
| ---------|-----------|-----------------------|----------------|-------------------------------|
|   bytes  |    4      |      1                |    2           |        8                      |
|   type   | uint32    |   uint8_t             |  uint16_t      | uint16_t  x 4 (array)         | 
|   unit   |   us      | 0,89 for 0-3,356-359  | rpm = Speed/64 | flag, distances or error_code |

- the packet is raw XV11Lidar packet with timestamp, without signal strength, CRC
- Distances are 14 bits mm distances or error code, 1 bit strength warning, 1 bit invalid_data
    - if invalid_data bit is set, the field carries error code
    - otherwise it is distance in mm
    - strength warning when power received is lower than expected for the distance

 */

void serialPush(const xv11lidar_usb_packet& packet)
{
	if(g_serial.free_bytes() < USBNB_XV11LIDAR_BYTES)
		return; 

	g_serial.push((uint8_t)USBNB_START_BYTE);
	g_serial.push((uint8_t)USBNB_XV11LIDAR_BYTES);
	g_serial.push((uint8_t)USBNB_XV11LIDAR_TYPE);
	
	g_serial.push(packet.timestamp_us);
	g_serial.push(packet.angle_quad);
	g_serial.push(packet.speed64);
	g_serial.push((uint8_t*)packet.distances, sizeof(packet.distances));
	
	g_serial.push((uint8_t)USBNB_END_BYTE);	
}

/*
### RPLIDARA3

|          | Timestamp | Sequence | Data               |
| ---------|-------- --|----------|--------------------|
|   bytes  |     4     |    1     |  132               |
|   type   |   uint32  |  uint8   | ultra_capsules     |
|   unit   |    us     |  counts  | RPLidarA3 internal |

- data is not decoded on MCU due to complexity
- sequence if for checking if data angles follow one another
- data corresponds to [rplidar_response_ultra_capsule_measurement_nodes_t](https://github.com/Slamtec/rplidar_sdk/blob/8291e232af614842447a634b6dbd725b81f24713/sdk/sdk/include/rplidar_cmd.h#L197) in [rplidar_sdk](https://github.com/Slamtec/rplidar_sdk)
- decoding should do the same as [_ultraCapsuleToNormal](https://github.com/Slamtec/rplidar_sdk/blob/master/sdk/sdk/src/rplidar_driver.cpp#L1071) in [rplidar_sdk](https://github.com/Slamtec/rplidar_sdk)
*/

void serialPush(const rplidar_usb_packet& packet)
{
	if(g_serial.free_bytes() < USBNB_RPLIDAR_BYTES)
		return; 

	g_serial.push((uint8_t)USBNB_START_BYTE);
	g_serial.push((uint8_t)USBNB_RPLIDAR_BYTES);
	g_serial.push((uint8_t)USBNB_RPLIDAR_TYPE);
	
	g_serial.push(packet.timestamp_us);
	g_serial.push(packet.sequence);
	g_serial.push(packet.data, sizeof(packet.data));
	
	g_serial.push((uint8_t)USBNB_END_BYTE);

}
int serialSend()
{
	return g_serial.send();
}
