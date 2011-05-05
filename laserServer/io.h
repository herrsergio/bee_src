
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Welcome!
 *****
 ***** This file is part of the BeeSoft robot control software.
 ***** The version number is:
 *****
 *****                  v1.3.8 (released Aug 5, 1998)
 *****                  this is not an official BeeSoft version
 *****
 ***** Please refer to bee/src/COPYRIGHT for copyright and liability
 ***** information.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#ifndef IO_H_LOADED
#define IO_H_LOADED

/* -------------------------------------------------------- */

#define NUMBER_LASERS 180

#define MAX_LASER_RANGE 500.0

#define ROBOT_RADIUS 35.0

#define MAX_LASER_MESSAGE_LENGTH 400

#define LASER_BUFFER_SIZE 8196

#define MMM(ch) fprintf(stderr, "%s: uCrc16: %.2x%.2x  abData: %.2x%.2x  uLen: %d CommData: %.2x\n", ch, *(p_uCrc16[0]), *(p_uCrc16[1]), abData[0], abData[1], (int) uLen, *CommData)

#define CRC16_GEN_POL  0x8005

#define CRC16_GEN_POL0 0x80

#define CRC16_GEN_POL1 0x05

/* -------------------------------------------------------- */

typedef struct {
  DEV_TYPE dev;
  /* put in here whatever is needed for starting up the real device.*/
} LaserDeviceType, *LaserDevicePtr;

typedef struct {
  int   defined;
  float angles[NUMBER_LASERS];
  float values[NUMBER_LASERS];
  int   blendung[NUMBER_LASERS];
  int   wfv[NUMBER_LASERS];
  int   sfv[NUMBER_LASERS];
  struct timeval time;
} LaserSensorValueType, *LaserSensorValuePtr;

LaserSensorValueType LaserSensors;

LaserDeviceType LaserDevice;

void LaserShutdownLaser();

void LaserProcessOutputLine( unsigned char *line, int length );

void LaserHandleOutput( int fd, long chars_available );

void LaserHandleTimeout();

int LaserWriteCommand( unsigned char command, char *argument, int arg_length );

void LaserInit();

void LaserSetBaud19200();

void LaserSetBaud9600();

void LasterRequestReading(int num);

#endif

/*
 * $Log: io.h,v $
 * Revision 1.2  1997/08/07 02:45:51  swa
 * First version of a laserServer. Compiles nicely, yet produces nothing useful. ;)
 * Just to copy things to a safe place ...
 *
 * Revision 1.1  1997/08/06 15:12:32  swa
 * Very first and incomplete version of a laserServer. No TCX comm yet.
 *
 *
 */
