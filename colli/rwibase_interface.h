
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
 * FILE: base_interface.h
 *
 * ABSTRACT:
 * 
 * Header file to be included to use Xavier motion functions.
 * Straightforward translation of the base interface commands to C.
 * Most of the comments are from the RWI documentation.
 *
 *****************************************************************************/


#ifndef BASE_INTERFACE_H
#define BASE_INTERFACE_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <sys/time.h>

#include "Common.h"
#include "libc.h"

#include "LIB-math.h"
#include "devUtils.h"
#include "handlers.h"

/*****************************************************************************
 * Global Constants
 *****************************************************************************/

#define COUNTS_IN_360_DEGREES  1024
#define COUNTS_PER_DEGREE      2.84

/* Differences between old and new baseServer. */
#define BASE_COUNTS_PER_CM                     273.0
#define BASE_SERVER_COUNTS_PER_CM               10.0

#define COUNTS_PER_RAD              (COUNTS_PER_DEGREE *  180.0 / PI)
#define STREAMING_MILLISECONDS      1000
#define CHECKING_SECONDS            1
#define BATTERY_QUERY_INTERVAL      10 

/* These are the values sent by the base after a reset. */
#define ROBOT_INIT_POS_X 1213.3
#define ROBOT_INIT_POS_Y 1213.3
#define ROBOT_INIT_ROT   0.0

/* for example: every 10th status report update battery status */ 

/*
 *      EVENT                       Type of callback data 
 *      *****                       *********************                   
 */

#define ANSWER_BATTERY_STATUS    0  /* int */
#define ANSWER_ROTATE_POSITION   1  /* int */   /* busy */
#define STATUS_REPORT            2  /* int */   /* busy */
#define BUMP                     3  /* int */
#define NOT_BUMP                 4  /* int */
#define TRANS_STOPPED            5  /* nothing */
#define ROT_STOPPED              6  /* nothing */
#define ERROR_TRANSLATION        7
#define ERROR_ROTATION           8
#define BASE_STOPPED             9
#define BASE_REPORT_MISSED      10  /* nothing */

#define BASE_NUMBER_EVENTS      11

extern HandlerList rwibase_handlers;


/*****************************
 * Constants for status report
 *****************************/

#define CLOCK              0x2
#define BUMP_SWITCHES      0x8
#define TRANS_POS_X        0x10
#define TRANS_POS_Y        0x20
#define TRANS_VELOC        0x200
#define TRANS_STATUS       0x800
#define ROT_POS            0x40
#define ROT_VELOC          0x20000
#define ROT_STATUS         0x80000


typedef unsigned long b8_integer;
typedef unsigned long b16_integer;
typedef unsigned long b32_integer;


typedef enum {
  POSITIVE, NEGATIVE
  } directionStatus;

typedef struct {
  double voltage;
  double current;
  double time;
} BATTERY_TYPE;

typedef struct {
  BOOLEAN target_reached;
} COLLISION_TYPE;

typedef struct _Base {
  double          time;                /* updated by status report: base clock */
  double          rot_acceleration;    /* set by command */
  double          rot_current_speed;   /* updated by status_report */
  double          rot_set_speed;       /* set by command */
  double          rot_position;        /* updated by status report */
  double          trans_acceleration;  /* set by command */
  double          trans_current_speed; /* updated by status report */
  double          trans_set_speed;     /* set by command */ 
  double          trans_position;      /* updated by status report */
  double          pos_x;               /* updated by status report */
  double          pos_y;               /* updated by status report */
  double          orientation;         /* updated by status report */
  directionStatus trans_direction;     /* updated by status report */
  directionStatus trans_set_direction; /* updated by command */
  directionStatus rot_direction;       /* updated by status report */
  directionStatus rot_set_direction;   /* updated by command */
  BOOLEAN         rot_moving;          /* computed from status report */
  BOOLEAN         trans_moving;        /* computed from status report */
  int             bumpers;             /* updated by status report */
  BOOLEAN         bump;
  BOOLEAN         emergency;           /* updated by status report */
  BOOLEAN         emergencyProcedure;  /* updated by command */
  BATTERY_TYPE    battery_state;       /* updated by command */
  COLLISION_TYPE  collision_state;     /* updated by collision avoidance */
  /* The next two values are used for recovery after a WatchDogStop */
  /* Did someone send a BASE_Rotate() or a BASE_RotateTo() command? */
  BOOLEAN         rotate_relative_mode; 
  /* how many degress ? */
  double          still_to_rotate;
  /* Did the watch dog stop the robot? */
  BOOLEAN         stopped_by_watch_dog;
  BOOLEAN         stopped_by_colli;
} _Base, *Base;


typedef struct {
  DEV_TYPE dev;
  /* put in here whatever is needed for starting up the real device.*/
}  BASE_TYPE, *BASE_PTR;


int   BASE_init();
void  BASE_GetReady(void);
void  BASE_outputHnd(int fd, long chars_available);
void  BASE_timeoutHnd(void);
void  BASE_terminate(void);

void  BASE_DisableEmergencyProcedure(void);
void  BASE_EnableEmergencyProcedure(void);

void  BASE_SetIntervalUpdates(long milliseconds);

void  BASE_InstallHandler(Handler handler, int event, Pointer client_data);
void  BASE_RemoveHandler(Handler handler, int event);
void  BASE_RemoveAllHandlers(int event);

/* Two files will be created with the following function:
   debug_file.out and debug_file.in, in which will be recorded the
   data going to or coming from the base */

void  BASE_Debug(BOOLEAN debug_flag, char *debug_file); 
void  BASE_WatchDogTimer(int milliseconds);
void  BASE_Kill(void);

void  BASE_JoystickDisable(void);
void  BASE_ResetJoystick(void);
void  BASE_SetRotationReference(void);
void  BASE_SetPos(double x, double y, double orientation);
void  BASE_QueryBatteryStatus(void);
void  BASE_QueryRotatePosition(Handler handler, Pointer client_data);
void  BASE_RotateAcceleration(double degrees_per_second_sqr);
void  BASE_RotateCollisionAcceleration(double degrees_per_second_sqr);
void  BASE_RotateHalt(void);
void  BASE_RotateVelocity (double degrees_per_second);
void  BASE_RotateCollisionVelocity (double degrees_per_second);
void  BASE_RotateClockwise(void);
void  BASE_RotateAnticlockwise(void);
void  BASE_Rotate (double degrees);
void  BASE_RotateTo (double degrees);
void  BASE_TranslateAcceleration (double cms_per_second_sqr);
void  BASE_TranslateCollisionAcceleration (double cms_per_second_sqr);
void  BASE_TranslateHalt(void);
void  BASE_TranslateVelocity (double cms_per_second);
void  BASE_TranslateCollisionVelocity (double cms_per_second);
void  BASE_TranslateForward(void);
void  BASE_TranslateBackward(void);
void  BASE_Translate (double cms);
void  BASE_HyperJump (float x, float y, float o);
void  BASE_missed(Handler handler, Pointer client_data);
void ReceivedStatusReport( BOOLEAN report_corrupted);
long SafeSub (unsigned long l1, unsigned long l2);

extern BOOLEAN BASE_waiting_for_battery_status;


/*****************************************************************************
 * External variables
 *****************************************************************************/

#ifdef DECLARE_RWIBASE_VARS

BASE_TYPE    base_device = 
{ 
  { FALSE,
      { "", DEFAULT_PORT},
      
      /* 14 is for a baud rate of 19200 */
      { "/dev/ttyS0", 13},

      RWIBASE_DEV_NAME,
      -1,
      TRUE,
      FALSE,
      (FILE *) NULL,
      (fd_set *) NULL,
      (DEVICE_OUTPUT_HND) BASE_outputHnd,
      BASE_timeoutHnd,  
      (DEVICE_SET_TIMEOUT)  setTimeout,  
      (DEVICE_CANCEL_TIMEOUT) cancelTimeout,
      (void (*)(void)) NULL,  
      {0, 0},
      {LONG_MAX, 0},
      {LONG_MAX, 0},
      (void (*)(void)) NULL,
      FALSE
      }
};

#else

extern BASE_TYPE    base_device;
extern _Base rwi_base;
extern _Base tmp_state;
extern _Base previous_state;
extern BATTERY_TYPE battery_state;

extern float desired_rot_velocity;
extern float desired_trans_velocity;


#endif

#endif


void SIMULATOR_message_to_base_handler(TCX_REF_PTR   ref,
				       char        **message);

