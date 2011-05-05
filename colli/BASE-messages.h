
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




#ifndef BASE_messages_defined
#define BASE_messages_defined


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

#include "tcx.h"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* 
   Notice: when including this file, you need to have the flag

   TCX_define_variables

   be defined exactly once. This will allocate memory for the
   module pointer and the message arrays.
*/

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** TCX module name - this is a unique identifier for TCX *******/

#define TCX_BASE_MODULE_NAME "BASE"



#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR BASE;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR BASE;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** BASE data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/


/*
 * SUBSCRIPTION message. 
 *	0 = do not receive that data regularily
 *	n = receive every n-th copy of that data. n=1 will make
 *	    you receive lots of messages!
 */
typedef struct {
  int subscribe_status_report;	/* >=1 = subscribe, 0 = unsubscribe */
  int subscribe_sonar_report;	/* >=1 = subscribe, 0 = unsubscribe */
  int subscribe_laser_report;	/* >=1 = subscribe, 0 = unsubscribe */
  int subscribe_ir_report;	/* >=1 = subscribe, 0 = unsubscribe */
  int subscribe_colli_report;	/* >=1 = subscribe, 0 = unsubscribe */
} BASE_register_auto_update_type, *BASE_register_auto_update_ptr;

#define BASE_register_auto_update_format "{int, int, int, int, int}"

#define BASE_robot_position_query_format NULL

#define BASE_stop_robot_format NULL

#define BASE_translate_halt NULL

#define BASE_rotate_halt NULL

#define BASE_update_status_query_format NULL

/* Set values that should not be changed to DONT_CHANGE */
#define DONT_CHANGE -1
typedef struct {
  int modeNumber;
  int useSonar; /* > 0: use sonar, 0: don't use sonar, else don't change */
  int useLaser;
} BASE_setmode_type, *BASE_setmode_ptr;

#define BASE_setmode_format "{int, int, int}"

#define BASE_disconnect_format NULL

typedef struct {
  float x;
  float y;
  float orientation;
} BASE_robot_position_reply_type, *BASE_robot_position_reply_ptr;

#define BASE_robot_position_reply_format "{float, float, float}"

typedef struct {
  float x, y, orientation;
} BASE_action_executed_reply_type, *BASE_action_executed_reply_ptr;

#define BASE_action_executed_reply_format "{float, float, float}"


typedef struct obstaclePoint {
  int x;
  int y;
} obstaclePoint;

typedef struct BASE_obstacle_points_type {
  int no_of_points;
  obstaclePoint *points;
} BASE_obstacle_points_type, *BASE_obstacle_points_ptr;

#define BASE_obstacle_points_format "{int, <{int,int}:1>}"

typedef struct obstacleLine {
  obstaclePoint pt1;
  obstaclePoint pt2;
} obstacleLine;

typedef struct BASE_obstacle_lines_type {
  int no_of_lines;
  obstacleLine *lines;
} BASE_obstacle_lines_type, *BASE_obstacle_lines_ptr;

#define BASE_obstacle_lines_format "{int, <{int,int,int,int}:1>}"


#define BASE_reset_obstacle_line_field_format NULL

typedef struct {
  int target_reached;
} COLLISION_reply_type;


typedef struct {		/* copied from rwibase_interface.h */
  float  time;			/* updated by status report: base clock */
  float  rot_acceleration;	/* set by command */
  float  rot_current_speed;	/* updated by status_report */
  float  rot_set_speed;		/* set by command */
  float  rot_position;		/* updated by status report */
  float  trans_acceleration;	/* set by command */
  float  trans_current_speed;	/* updated by status report */
  float  trans_set_speed;	/* set by command */ 
  float  trans_position;	/* updated by status report */
  float  pos_x;			/* updated by status report */
  float  pos_y;			/* updated by status report */
  float  orientation;		/* updated by status report */
  unsigned char   trans_direction;	/* updated by status report */
  unsigned char   trans_set_direction;	/* updated by command */
  unsigned char   rot_direction;	/* updated by status report */
  unsigned char   rot_set_direction;	/* updated by command */
  unsigned char   rot_moving;		/* computed from status report */
  unsigned char   trans_moving;		/* computed from status report */
  unsigned char   bumpers;		/* updated by status report */
  unsigned char   bump;
  unsigned char   emergency;		/* updated by status report */
  unsigned char   emergencyProcedure;	/* updated by command */
  COLLISION_reply_type  collision_state; /* updated by collision avoidance */
  int targetDefined;                     /* is a target point given? */
  float targetX;                         /* absolute x position of target */  
  float targetY;                         /* absolute y position of target */  
} BASE_update_status_reply_type, *BASE_update_status_reply_ptr;

#define BASE_update_status_reply_format "{float, float, float, float, float, float, float, float, float, float, float, float, int, int, int, int, int, int, int, int, int, int, int, int, float, float}"



typedef struct {
  float rot_velocity;
  float trans_velocity;
} BASE_set_velocity_type, *BASE_set_velocity_ptr;

#define BASE_set_velocity_format "{float, float}"


typedef struct {
  float rot_acceleration;
  float trans_acceleration;
} BASE_set_acceleration_type, *BASE_set_acceleration_ptr;

#define BASE_set_acceleration_format "{float, float}"


#define BASE_rotate_clockwise_format NULL


#define BASE_rotate_anticlockwise_format NULL


#define BASE_translate_forward_format NULL


#define BASE_translate_backward_format NULL


typedef struct {
  float rel_target_x;
  float rel_target_y;
} BASE_goto_relative_type, *BASE_goto_relative_ptr;

#define BASE_goto_relative_format "{float, float}"


/*
 * The following two command do _not_ make use of the built-int
 * obstacle avoidance mechanisms of BASE. However, the speed may
 * be reduced automatically.
 */ 

#define BASE_translate_by_format "float"

#define BASE_rotate_by_format "float"


typedef struct {
  int   new_target;
  float abs_target_x;
  float abs_target_y;
} BASE_goto_absolute_type, *BASE_goto_absolute_ptr;

#define BASE_goto_absolute_format "{int, float, float}"


typedef struct {
  int   new_target;
  float approach_dist;
  float abs_target_x;
  float abs_target_y;
  int mode;
} BASE_approach_absolute_type, *BASE_approach_absolute_ptr;

#define BASE_approach_absolute_format "{int, float, float, float, int}"


typedef struct {
  int   new_target;
  float approach_dist;
  float abs_target_x1;
  float abs_target_y1;
  float abs_target_x2;
  float abs_target_y2;
} BASE_approach_absolute_two_points_type, *BASE_approach_absolute_two_points_ptr;

#define BASE_approach_absolute_two_points_format "{int, float, float, float, float, float}"



#define BASE_reset_joystick_format NULL



#define BASE_notify_wall_orientation_format "float" /* will be in the range
						     * 0....90 */


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** BASE commands - these are the commands/queries understood by BASE ****/


#ifdef TCX_define_variables		/* do this exactly once! */

#define BASE_messages \
  {"BASE_register_auto_update", BASE_register_auto_update_format},\
  {"BASE_robot_position_query", BASE_robot_position_query_format},\
  {"BASE_robot_position_reply", BASE_robot_position_reply_format},\
  {"BASE_stop_robot",           BASE_stop_robot_format},\
  {"BASE_translate_halt",       BASE_translate_halt},\
  {"BASE_rotate_halt",          BASE_rotate_halt},\
  {"BASE_update_status_query",  BASE_update_status_query_format},\
  {"BASE_update_status_reply",  BASE_update_status_reply_format},\
  {"BASE_set_velocity",         BASE_set_velocity_format},\
  {"BASE_set_acceleration",     BASE_set_acceleration_format},\
  {"BASE_rotate_clockwise",     BASE_rotate_clockwise_format},\
  {"BASE_rotate_anticlockwise", BASE_rotate_anticlockwise_format},\
  {"BASE_translate_forward",    BASE_translate_forward_format},\
  {"BASE_translate_backward",   BASE_translate_backward_format},\
  {"BASE_goto_relative",        BASE_goto_relative_format},\
  {"BASE_translate_by",         BASE_translate_by_format},\
  {"BASE_rotate_by",            BASE_rotate_by_format},\
  {"BASE_goto_absolute",        BASE_goto_absolute_format},\
  {"BASE_approach_absolute",    BASE_approach_absolute_format},\
  {"BASE_approach_absolute_two_points", BASE_approach_absolute_two_points_format},\
  {"BASE_disconnect",           BASE_disconnect_format},\
  {"BASE_setmode",              BASE_setmode_format},\
  {"BASE_obstacle_points",      BASE_obstacle_points_format},\
  {"BASE_obstacle_lines",      BASE_obstacle_lines_format},\
  {"BASE_reset_obstacle_line_field", BASE_reset_obstacle_line_field_format},\
  {"BASE_action_executed_reply",BASE_action_executed_reply_format},\
  {"BASE_reset_joystick",       BASE_reset_joystick_format},\
  {"BASE_notify_wall_orientation",  BASE_notify_wall_orientation_format}


#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_REPLY_HANDLERS



/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with BASE ******/




/******* (a) Procedure headers ******/

void BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				       BASE_robot_position_reply_ptr pos);

void BASE_update_status_reply_handler(TCX_REF_PTR                   ref,
				      BASE_update_status_reply_ptr status);

void BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
					BASE_action_executed_reply_ptr data);



/******* (b) Handler array ******/



TCX_REG_HND_TYPE BASE_reply_handler_array[] = {
  {"BASE_robot_position_reply", "BASE_robot_position_reply_handler",
     (void (*)()) BASE_robot_position_reply_handler, TCX_RECV_ALL, NULL},
  {"BASE_update_status_reply", "BASE_update_status_reply_handler",
     (void (*)()) BASE_update_status_reply_handler, TCX_RECV_ALL, NULL},
  {"BASE_action_executed_reply", "BASE_action_executed_reply_handler",
     (void (*)()) BASE_action_executed_reply_handler, TCX_RECV_ALL, NULL}

};


#endif







#endif
