#include "cc-xv11lidar.h"

#include "xv11lidar.h"

/* Global data */
XV11Lidar lidar(XV11LIDAR_SERIAL, XV11LIDAR_PWM_PIN );

void setupXV11Lidar()
{
  lidar.setup(XV11LIDAR_RPM);
}

void processXV11Lidar()
{
  XV11Packet xv11_packet;
  bool got_packet=lidar.processAvailable(&xv11_packet);

  lidar.applyMotorPID();

  if(!got_packet)
    return;

  xv11lidar_packet packet;
  packet.timestamp_us = xv11_packet.timestamp_us;
  packet.angle_quad = xv11_packet.angle_quad;
  packet.speed64 = xv11_packet.speed64;
  memcpy(packet.distances, xv11_packet.distances, sizeof(packet.distances));
  
  //encode & emit packet over USB
}
