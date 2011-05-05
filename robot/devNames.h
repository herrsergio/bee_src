
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
 * PROJECT: Rhino
 *
 * FILE: devNames.h
 *
 * ABSTRACT:
 * 
 * This file provides a common set of names for devices.
 *
 * Note : ANSI C headers are used.
 *
 *****************************************************************************/

#ifndef DEVNAMES_LOADED
#define DEVNAMES_LOADED

typedef enum  {
  RWIBASE_DEV, /* The rwi base */
  SPEECH_DEV,  /* the speech synthesizer */
  SONAR_DEV,   /* sonar ring. */
  LASER_DEV,   /* laser range finder */
  PANTILT_DEV, /* pan tilt head */
  CAMERA_DEV,  /* color camera */
  ARM_DEV,     /* arm and magnet  */
  NUM_DEVS     /* the maximum number of devices */
  } simDevices;

#define RWIBASE_DEV_NAME "rwibase"
#define RWIARM_DEV_NAME "rwiarm"
#define RWIARM_GRIPPER_DEV_NAME "rwigripper"
#define SPEECH_DEV_NAME "speech"
#define SONAR_DEV_NAME "sonar"
#define LASER_DEV_NAME "laser"
#define PANTILT_DEV_NAME "pantilt"
#define CAMERA_DEV_NAME "camera"
#define ARM_DEV_NAME "arm"
#define SIM_DEV_NAME "simulator"

#endif




