
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






/*****************************************************************************
 * PROJECT: rhino
 *****************************************************************************/

#ifndef LASER_INTERFACE_LOADED
#define LASER_INTERFACE_LOADED

#include "devUtils.h"
#include "handlers.h"

#include "collision.h"

/************************************************************************
 *  Specifications of the range finders. 
 ************************************************************************/

#define FRONT_LASER 0
#define REAR_LASER  1

#define USE_FRONT_LASER 1
#define USE_REAR_LASER  0

/* #define BONN_LASER */

#ifdef UNIBONN
#define BONN_LASER
#endif

#ifdef BONN_LASER
#define USE_REAR_LASER  1
#else
#define USE_REAR_LASER  0
#endif

#ifdef BONN_LASER
#ifdef i386
#define FRONT_LASER_DEVICE "/dev/cur6"
#define REAR_LASER_DEVICE "/dev/cur7"
#else
#define FRONT_LASER_DEVICE "/dev/cur6"
#define REAR_LASER_DEVICE "/dev/cur7"
#endif

#else
#define FRONT_LASER_DEVICE "/dev/cur6"
#define REAR_LASER_DEVICE "/dev/cur7"
#endif


/* 14 is for a baud rate of 19200 */
#define LASER_BAUD_RATE 13

#define FRONT_LASER_ANGLE 0.0   /* Angle relative to the robot */  
#define FRONT_LASER_OFFSET 11.5 /* Offset in cm from the center of the robot */

#define REAR_LASER_ANGLE DEG_180  /* Angle relative to the robot */  
#define REAR_LASER_OFFSET 11.5    /* Offset in cm from the center of the robot */

#define LASER_MAX_RANGE 500

/************************************************************************
 *  Type for the laser readings.
 ************************************************************************/

typedef struct LASER_reading {
 /* Have the values been converted to obstacle points? */
  int new;

  /* Values specific to the mode of the scans. */
  int numberOfReadings;
  float* reading;
  int*  blendung;
  int*  wfv;
  int*  sfv;
  float startAngle;
  float angleResolution;
  Point rPos;
  float rRot;

  /* Time of the scan. */
  struct timeval time;
} LASER_reading;

/* Readings of the two laser range finders. */
extern LASER_reading frontLaserReading;
extern LASER_reading rearLaserReading;

typedef struct {
  int laserNumber;
  LASER_reading* scan;
  DEV_TYPE dev;
}  LASER_TYPE, *LASER_PTR;

#define LASER_POLLSECONDS          1


/************************************************************************
 *  Simple laser commands
 *  NOTE : LASER_init must be called prior to any other LASER command.
 ************************************************************************/

BOOLEAN start_laser();
void
requestNextLaserScan( int laserNumber);
void stop_laser(void);
void LASER_look_for_laser_device(void);

/* Events of the lasers and the corresponding handler functions. */
#define SINGLE_LASER_REPORT     0  /* laser_readings of one laser complete */
#define COMPLETE_LASER_REPORT   1  /* laser_readings of all lasers complete */
#define LASER_REPORT_MISSED     2  /* not used */
#define LASER_NUMBER_EVENTS     3

void  LASER_InstallHandler(Handler handler, int event, Pointer client_data);
void  LASER_RemoveHandler(Handler handler, int event);
void  LASER_RemoveAllHandlers(int event);

/* Functions called by the device. */
void FRONT_LASER_outputHnd( int fd, long chars_available);
void REAR_LASER_outputHnd( int fd, long chars_available);
void LASER_timeoutHnd(void);


#ifdef DECLARE_LASER_VARS

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/************************************************************************
 *  Laser device type.
 ************************************************************************/

LASER_TYPE    frontLaserDevice = 
{
  FRONT_LASER,
  &frontLaserReading,
  { FALSE,
    { "", DEFAULT_PORT},
    /* 14 is for a baud rate of 19200 */
    { FRONT_LASER_DEVICE, LASER_BAUD_RATE},
    LASER_DEV_NAME,
    -1,
    TRUE,
    FALSE,
    (FILE *) NULL,
    (fd_set *) NULL,
    (DEVICE_OUTPUT_HND) FRONT_LASER_outputHnd,
    LASER_timeoutHnd,
    (DEVICE_SET_TIMEOUT)  setTimeout,  
    (DEVICE_CANCEL_TIMEOUT) cancelTimeout,
    (void (*)(void)) NULL,  
    {0, 0},
    {LONG_MAX, 0},
    {0x2FFFFFFF, 0},
    (void (*)(void)) NULL,
    FALSE
  }
};


LASER_TYPE    rearLaserDevice = 
{
  REAR_LASER,
  &rearLaserReading,
  { FALSE,
    { "", DEFAULT_PORT},
    /* 14 is for a baud rate of 19200 */
    { REAR_LASER_DEVICE, LASER_BAUD_RATE},
    LASER_DEV_NAME,
    -1,
    TRUE,
    FALSE,
    (FILE *) NULL,
    (fd_set *) NULL,
    (DEVICE_OUTPUT_HND) REAR_LASER_outputHnd,
    LASER_timeoutHnd,
    (DEVICE_SET_TIMEOUT)  setTimeout,  
    (DEVICE_CANCEL_TIMEOUT) cancelTimeout,
    (void (*)(void)) NULL,  
    {0, 0},
    {LONG_MAX, 0},
    {0x2FFFFFFF, 0},
    (void (*)(void)) NULL,
    FALSE
  }
};

#else

extern LASER_TYPE   frontLaserDevice;
extern LASER_TYPE   rearLaserDevice;

#endif

#endif

