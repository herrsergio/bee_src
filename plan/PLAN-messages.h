
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




#ifndef PLAN_messages_defined
#define PLAN_messages_defined




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

#define TCX_PLAN_MODULE_NAME "PLAN"



#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR PLAN;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR PLAN;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** PLAN data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/





typedef struct {
  int subscribe_status;		/* 1=subscribe, 0=unsubscribe */
} PLAN_register_auto_update_type, *PLAN_register_auto_update_ptr;

#define PLAN_register_auto_update_format "{int}"



typedef struct{
  float x;
  float y;
  float max_radius;
  float reward;
  int   name;			/* "name" of the goal (->marker) */
  int   add;			/* 0=delete, 1=create */
} PLAN_goal_message_type, *PLAN_goal_message_ptr;

#define PLAN_goal_message_format "{float, float, float, float, int, int}"





typedef struct{
  float x, y, orientation;
} PLAN_new_robot_pos_message_type, *PLAN_new_robot_pos_message_ptr;

#define PLAN_new_robot_pos_message_format "{float, float, float}"





typedef struct{
  float x, y, orientation;
  int   stuck;
} PLAN_action_query_type, *PLAN_action_query_ptr;
#define PLAN_action_query_format "{float, float, float, int}"


typedef struct{
  float turn;
  float base;
  float goal_x;
  float goal_y;
  float goal_dist;
  int   speed;
  int   final_action;	   /* set 1 iff action will achieve goal pos. */
  int   active_goal_name;  /* nane (number) of the goal selected, -1 if none*/
} PLAN_action_reply_type, *PLAN_action_reply_ptr;

#define PLAN_action_reply_format "{float, float, float, float, float, int, int, int}"






typedef struct{
  int   type;			/* 0=reset all, 1=new pos, 2=new neg */
  float from_x, from_y, to_x, to_y;
} PLAN_constraints_message_type, *PLAN_constraints_message_ptr;

#define PLAN_constraints_message_format "{int, float, float, float, float}"

#define PLAN_start_autonomous_message_format "int" /* 1=exploration mode */

#define PLAN_stop_autonomous_message_format "int" /* 1=stop the robot,
						   * 0=let go*/

#define PLAN_quit_message_format NULL


#define PLAN_status_query_format NULL
  

typedef struct{
  int   num_goals;

  float *goal_x;
  float *goal_y;
  float *goal_distance;
  int   *goal_name;
  int   *goal_visible;
  
  int   goal_mode;
  int   autonomous;

  float robot_x;
  float robot_y;
  float robot_orientation;
} PLAN_status_reply_type, *PLAN_status_reply_ptr;

#define PLAN_status_reply_format "{int, <float : 1>, <float : 1>, <float : 1>, <int : 1>, <int : 1>, int, int, float, float, float}"


typedef struct{
  float max_security_dist;
  float max_adjust_angle ;
  float collision_threshold;
  float max_goal_distance;
  float max_final_approach_distance;
  float max_approach_distance;
} PLAN_parameter_message_type, *PLAN_parameter_message_ptr;

#define PLAN_parameter_message_format "{float, float, float, float, float, float}"


#define PLAN_remove_all_goals_format NULL


#define PLAN_reset_exploration_table_format

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** PLAN commands - these are the commands/queries understood by PLAN ****/


#ifdef TCX_define_variables		/* do this exactly once! */


#define PLAN_messages \
  {"PLAN_register_auto_update",     PLAN_register_auto_update_format},\
  {"PLAN_goal_message",             PLAN_goal_message_format},\
  {"PLAN_new_robot_pos_message",    PLAN_new_robot_pos_message_format},\
  {"PLAN_action_query",             PLAN_action_query_format},\
  {"PLAN_action_reply",             PLAN_action_reply_format},\
  {"PLAN_constraints_message",      PLAN_constraints_message_format},\
  {"PLAN_start_autonomous_message", PLAN_start_autonomous_message_format},\
  {"PLAN_stop_autonomous_message",  PLAN_stop_autonomous_message_format},\
  {"PLAN_quit_message",             PLAN_quit_message_format},\
  {"PLAN_status_query",             PLAN_status_query_format},\
  {"PLAN_status_reply",             PLAN_status_reply_format},\
  {"PLAN_remove_all_goals",         PLAN_remove_all_goals_format},\
  {"PLAN_reset_exploration_table",  PLAN_reset_exploration_table_format},\
  {"PLAN_parameter_message",        PLAN_parameter_message_format}
#endif



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_REPLY_HANDLERS



/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with PLAN ******/




/******* (a) Procedure headers ******/



void PLAN_action_reply_handler(TCX_REF_PTR              ref,
			       PLAN_action_reply_ptr    action);

void PLAN_status_reply_handler(TCX_REF_PTR              ref,
			       PLAN_status_reply_ptr    status);


/******* (b) Handler array ******/



TCX_REG_HND_TYPE PLAN_reply_handler_array[] = {

  {"PLAN_action_reply", "PLAN_action_reply_handler",
     PLAN_action_reply_handler, TCX_RECV_ALL, NULL},
  {"PLAN_status_reply", "PLAN_status_reply_handler",
     PLAN_status_reply_handler, TCX_RECV_ALL, NULL}
};


#endif

#endif
