
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



/*****************************************************************************/

#ifndef PANTILT_INTERFACE_LOADED
#define PANTILT_INTERFACE_LOADED

#define USE_OLD_TTY
#include "devUtils.h"

/************************************************************************/
/*  Simplest panTilt commands                                            */
/************************************************************************/
/* PANTILT_init must be called prior to any of the PANTILT commands being 
   utilised. 
   */

typedef struct {
  DEV_TYPE dev;
  /* put in here whatever is needed for starting up the real device.*/
}  PANTILT_TYPE, *PANTILT_PTR;


int PANTILT_init(void);
void PANTILT_outputHnd(int fd, long chars_available);
void PANTILT_terminate(void);

#ifdef DECLARE_PANTILT_VARS

/*
 * the port and baud rates are now overridden by the beeSoft.ini file
 */

PANTILT_TYPE  panTilt_device = 
{ 
  { FALSE,
      { "", DEFAULT_PORT},
      { "/dev/ttyS1", 13}, 
      PANTILT_DEV_NAME,
      -1,
      TRUE,
      FALSE,			/* debug! */
      (FILE *) NULL,
      (fd_set *) NULL,
      (DEVICE_OUTPUT_HND) PANTILT_outputHnd,
      (void (*)(void)) NULL,  
      (DEVICE_SET_TIMEOUT)  setTimeout,  
      (DEVICE_CANCEL_TIMEOUT) cancelTimeout,
      (void (*)(void)) NULL,  
      {0, 0},
      {LONG_MAX, 0},
      {LONG_MAX, 0},
      (void (*)(void)) NULL
      }
};

#else

extern PANTILT_TYPE  panTilt_device;
extern char failureString[80];

BOOLEAN PANTILT_init(void);
void PANTILT_outputHnd (int fd, long chars_available);
void PANTILT_terminate(void);
BOOLEAN PANTILT_setVelocity (float panVelocity, float tiltVelocity);
BOOLEAN PANTILT_setAcceleration (float panAcceleration, float tiltAcceleration);
BOOLEAN PANTILT_position (DEGREES *panAngle, DEGREES *tiltAngle);
BOOLEAN PANTILT_pan (DEGREES panAngle);
BOOLEAN PANTILT_panRelative (DEGREES panAngle);
BOOLEAN PANTILT_tilt (DEGREES tiltAngle);
BOOLEAN PANTILT_tiltRelative (DEGREES tiltAngle);
BOOLEAN PANTILT_move (DEGREES panAngle, DEGREES tiltAngle);
BOOLEAN PANTILT_moveRelative (DEGREES panAngle, DEGREES tiltAngle);
BOOLEAN PANTILT_reset(void);
void PANTILT_limits (DEGREES *minPanAngle, DEGREES *maxPanAngle,
		     DEGREES *minTiltAngle, DEGREES *maxTiltAngle,
		     float *maxPanVelocity, float *maxTiltVelocity);
#endif

#endif
