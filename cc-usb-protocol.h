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

#include <stdint.h> //uint8_t
#include <string.h> //memcpy

enum {USB_BUFFER_SIZE=512};

class UsbNB
{
public:
	void push(const odometry_usb_packet &packet);
	void push(const xv11lidar_usb_packet &packet);
	void push(const rplidar_usb_packet &packet);
	int send();

	inline int free_bytes() {return USB_BUFFER_SIZE-m_buffer_bytes;};

	void push(const uint8_t *data, int size)
	{
		memcpy(m_buffer+m_buffer_bytes, data, size);
		m_buffer_bytes += size;
	}
	
	template <class T>
	void push(T val)
	{
		memcpy(m_buffer+m_buffer_bytes, &val, sizeof(T));
		m_buffer_bytes += sizeof(T);
	}
private:
	uint8_t m_buffer[USB_BUFFER_SIZE];
	int m_buffer_bytes=0;	
};

#endif
