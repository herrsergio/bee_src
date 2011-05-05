
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




#ifndef COLLI_messages_defined
#define COLLI_messages_defined



#include "tcx.h"
/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_COLLI_MODES

/* Different modes for the collision avoidance. */
#define DEFAULT_MODE 0        
#define FAST_TRAVEL_MODE 1
#define FIND_DOOR_MODE 2    
#define SERVO_MODE 3
#define ARM_OUT_MODE 4

/* The next modes are necessary for demonstrations. */
#define RANDOM_MODE 5
#define APPROACH_OBJECT_MODE 6
#define APPROACH_TRASH_BIN_MODE 7
#define ARM_OUT_RANDOM_MODE 8
#define NUMBER_OF_MODES 9

#endif /*DEFINE_COLLI_MODES */

/**** TCX module name - this is a unique identifier for TCX *******/

#define TCX_COLLI_MODULE_NAME "BASE"


#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR COLLI;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR COLLI;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** COLLI data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/


typedef struct iPoint {
  int x;
  int y;
} iPoint;


typedef struct iLine {
  iPoint pt1;
  iPoint pt2;
} iLine;

typedef struct iCircle {
  iPoint M;
  int rad;
} iCircle;

typedef struct COLLI_colli_reply_type {

  iPoint rpos;		/* Robot position */
  int rrot;			/* Rotation */
  int tvel;
  int rvel;
  int dist;
  
  iPoint targetpoint;		/* The target */

  /* Two points of the arm. */
  iPoint innerArmPoint;
  iPoint outerArmPoint;		

  /* The trajectories */
  iLine leftLine;
  iLine rightLine;

  iCircle innerCircle;
  iCircle outerCircle;
  iCircle armCircle;

  /* The collision line field. */
  int rememberInterval;	
  iLine sonar_lines[24];

  /* For usage of external information. */
  int no_of_external_points;
  iPoint *external_points;

  /* The laser points. */
  int no_of_laser_points;
  iPoint *laser_points;

  /* The bumper and ir points. */
  int no_of_ir_points;
  iPoint *ir_points;

  int no_of_bumper_points;
  iPoint *bumper_points;
} COLLI_colli_reply_type, *COLLI_colli_reply_ptr;


typedef struct COLLI_vision_line_type {
  int no_of_lines;
  iLine *lines;
} COLLI_vision_line_type, *COLLI_vision_line_ptr;

typedef struct COLLI_vision_point_type {
  int no_of_points;
  iPoint *points;
} COLLI_vision_point_type, *COLLI_vision_point_ptr;


typedef struct COLLI_parameter_type {
  float velocity_factor;
  float angle_factor;
  float distance_factor;
  float target_max_trans_speed;
  float target_max_rot_speed;
  float target_trans_acceleration;
  float target_rot_acceleration;
} COLLI_parameter_type, *COLLI_parameter_ptr;



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



#define COLLI_vision_line_format "{int, <{int,int,int,int}:1>}"

#define COLLI_vision_point_format "{int, <{int,int}:1>}"

#define COLLI_parameter_format "{float, float, float, float, float, float, float}"

#define COLLI_colli_reply_format "{{int,int}, int, int, int, int, {int,int}, {int,int}, {int,int}, {int,int,int,int}, {int,int,int,int}, {int,int,int}, {int,int,int}, {int,int,int}, int, [{int,int,int,int} : 24], int, <{int,int}:16>, int, <{int,int}:18>, int, <{int,int}:20>, int, <{int,int}:22>}" 

#define COLLI_disconnect_format NULL

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


#ifdef TCX_define_variables		/* do this exactly once! */

#define COLLI_messages \
  {"COLLI_colli_reply",         COLLI_colli_reply_format},\
  {"COLLI_vision_line",         COLLI_vision_line_format},\
  {"COLLI_vision_point",        COLLI_vision_point_format},\
  {"COLLI_parameter",           COLLI_parameter_format},\
  {"COLLI_disconnect",          COLLI_disconnect_format}
#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_REPLY_HANDLERS


/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with COLLI ******/

/******* (a) Procedure headers ******/

void COLLI_colli_reply_handler(TCX_REF_PTR                ref,
			       COLLI_colli_reply_ptr      data);

/******* (b) Handler array ******/

TCX_REG_HND_TYPE COLLI_reply_handler_array[] = {
  {"COLLI_colli_reply", "COLLI_colli_reply_handler",
     COLLI_colli_reply_handler, TCX_RECV_ALL, NULL}
};


#endif









#endif
