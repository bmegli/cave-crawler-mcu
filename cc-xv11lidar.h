#ifndef CC_XV11LIDAR
#define CC_XV11LIDAR

#include <stdint.h> //uint8_t, uint16_t, uint32_T

/* Pins */
const int XV11LIDAR_PWM_PIN = 35;

/* Serials */
#define XV11LIDAR_SERIAL Serial5

/* Vertical lidar */
const int XV11LIDAR_RPM=250;

struct xv11lidar_packet
{
  uint32_t timestamp_us;
  uint8_t angle_quad; //0-89 for readings 0-3 356-359
  uint16_t speed64;    //divide by 64 for speed in rpm 
  uint16_t distances[4]; //flags and distance or error code
};

/* Functions */ 
void setupXV11Lidar();
void processXV11Lidar();

#endif
