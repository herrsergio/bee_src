
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





/*change planner such that it resets the utility table if change in exploration/path planning mode
*/

#ifdef VMS
#include "vms.h"
#include "vmstime.h"
#endif

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#ifndef VMS
#include <malloc.h>
#endif
#include <sys/time.h>
#include "tcx.h"
#include "tcxP.h"
#include "o-graphics.h"

#include "bUtils.h"

#define pi  3.14159265358979323846
#define sqrt2 1.41436
#define MAX_RANDOM 1073741823.5
#define RAND() ((((float) random())/MAX_RANDOM)-1.0)
#define RAND_POS() ((((float) random())/MAX_RANDOM)*0.5)


#define TCX_define_variables /* this makes sure variables are installed */
#include "PLAN-messages.h"
#include "COLLI-messages.h"
#ifdef UNIBONN
#include "SPEECH-messages.h"
#endif

#define DEFINE_REPLY_HANDLERS
#include "MAP-messages.h"
#include "BASE-messages.h"
#include "SONAR-messages.h"

#include "PLAN.h"

#include "beeSoftVersion.h"
#include "libbUtils.h"
#include "librobot.h"
#include "libezx.h"

/* #define PLAN_DEBUG */

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* CONSTRAINTS */


PLAN_constraints_message_type constraints[max_n_constraints];

int n_constraints = 0;




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/******* auto-reply stuff, data definitions ***************/


#define MAX_N_AUTO_UPDATE_MODULES 100


int n_auto_update_modules = 0; /* number of processes to whom
				* position should be forwarded
				* automatically upon change */

int n_auto_status_update_modules         = 0;



typedef struct{
  TCX_MODULE_PTR module;	/* pointer to TCX module */
  int            status;	/* 1=subscribed to regular plan updates */
} auto_update_type;


auto_update_type auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];
                                /* collection of TCX-module ptrs */



/************************************************************************
 *
 *   NAME:         count_auto_update_modules()
 *                 
 *   FUNCTION:     Counts, how many modules of the different types require
 *                 auto-updates
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/

void count_auto_update_modules()
{
  int i;

  n_auto_status_update_modules         = 0;
  
  for (i = 0; i < n_auto_update_modules; i++)
    if (auto_update_modules[i].status)
      n_auto_status_update_modules++;

}


/************************************************************************
 *
 *   NAME:         add_auto_update_module()
 *                 
 *   FUNCTION:     Adds a module to the list of modules that
 *                 get automatical plan updates
 *                 
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *                 int plan                     1, if subscribe to plan
 *                 int correction              1, if subscribe to correction
 *                 
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/



static int add_auto_update_module(TCX_MODULE_PTR module, int status)
{
  int i;

  if (n_auto_update_modules >= MAX_N_AUTO_UPDATE_MODULES){
    fprintf(stderr, "ERROR: auto-module memory overflow. Request rejected.\n");
    return 0;
  }
  else
    for (i = 0; i < n_auto_update_modules; i++)
      if (auto_update_modules[i].module == module){
	fprintf(stderr, 
		"Module %s already known. Subscription modified: %d\n",
		tcxModuleName(module), status);
	auto_update_modules[i].status = status; /* subsrc? */
	count_auto_update_modules();
	return 1;
      }
  fprintf(stderr, "Add %s to auto-reply list.\n",
	  tcxModuleName(module));
  auto_update_modules[n_auto_update_modules].module = module; /* pointer*/
  auto_update_modules[n_auto_update_modules].status = status; /* subsrc? */ 
  n_auto_update_modules++;
  count_auto_update_modules();

  if (status) send_automatic_status_update(module);
  return 1;
}



/************************************************************************
 *
 *   NAME:         remove_auto_update_module()
 *                 
 *   FUNCTION:     Attempts to remove a module from the list of 
 *                 modules that get automatical plan updates 
 *                 
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *                 
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/


static int remove_auto_update_module(TCX_MODULE_PTR module)
{     
  int i, j, found = 0;;
  for (i = 0; i < n_auto_update_modules; i++) /* search for module */
    if (auto_update_modules[i].module == module){ /* if module found */
      fprintf(stderr, "Remove %s from auto-reply list.\n",
	      tcxModuleName(module));
      found++;
      n_auto_update_modules--;	/* remove that entry, one less now */
      for (j = i; j < n_auto_update_modules; j++){
	auto_update_modules[j].module = 
	  auto_update_modules[j+1].module; /* shift back */
	auto_update_modules[j].status = 
	  auto_update_modules[j+1].status; /* shift back */
      }
    }
  if (!found)
    fprintf(stderr, "(%s: no auto-replies removed)\n",
	    tcxModuleName(module));
  count_auto_update_modules();
  return found;
}
  


/************************************************************************
 *
 *   NAME:         send_automatic_status_update
 *                 
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void send_automatic_status_update(TCX_MODULE_PTR recipient)
{
  int i;
  PLAN_status_reply_type status_data;
  PROGRAM_STATE_PTR program_state = all->program_state;
  ROBOT_STATE_PTR robot_state = all->robot_state;
  ROBOT_SPECIFICATIONS_PTR robot_specifications = all->robot_specifications;
  ACTION_PTR action = all->action;
  float x, y;


  if (recipient == NULL)
    program_state->send_automatic_update = 0;	/* done! */


  /*
   * Check, if her is a recipient at all
   */


  if (n_auto_update_modules <= 0 && recipient == NULL)
    return;

  /*
   * Collect all goals
   */

  status_data.num_goals = G_return_num_markers(GOALS[NUMBER], 1);

  status_data.goal_x          = (float *) (malloc(sizeof(float) 
						  * status_data.num_goals));
  status_data.goal_y          = (float *) (malloc(sizeof(float) 
						  * status_data.num_goals));
  status_data.goal_distance   = (float *) (malloc(sizeof(float) 
						 * status_data.num_goals));
  status_data.goal_name       = (int *) (malloc(sizeof(int) 
						* status_data.num_goals));
  status_data.goal_visible    = (int *) (malloc(sizeof(int) 
						* status_data.num_goals));

  if (status_data.goal_x == NULL || status_data.goal_y == NULL ||
      status_data.goal_distance == NULL || status_data.goal_name == NULL ||
      status_data.goal_visible == NULL){
    putc(7, stderr);
    fprintf(stderr, 
	    "ERROR: Out of memoery in send_automatic_status_update.\n");
    return;			/* instead of exit! */
  }

  status_data.robot_x           = robot_state->x;
  status_data.robot_y           = robot_state->y;
  status_data.robot_orientation = robot_state->orientation;
  status_data.goal_mode         = program_state->goal_modus;
  status_data.autonomous        = program_state->autonomous;

  for (i = 0; i < status_data.num_goals; i++){
    G_return_marker_coordinates(GOALS[NUMBER], i, 
				&(status_data.goal_x[i]), 
				&(status_data.goal_y[i]), 
				&(status_data.goal_name[i]));
    status_data.goal_distance[i] = sqrt(((status_data.goal_x[i]
					  - robot_state->x) 
					 * (status_data.goal_x[i]
					    - robot_state->x))
					+ ((status_data.goal_y[i] 
					    - robot_state->y)
					   * (status_data.goal_y[i] 
					      - robot_state->y)));

    x = ((int) (status_data.goal_x[i] / robot_specifications->resolution))
      + robot_specifications->autoshifted_int_x;
    y = ((int) (status_data.goal_y[i] / robot_specifications->resolution))
      + robot_specifications->autoshifted_int_y;

    if (x < 0 || x >= robot_specifications->global_map_dim_x ||
	y < 0 || y >= robot_specifications->global_map_dim_y)
      status_data.goal_visible[i] = 0;
    else
      status_data.goal_visible[i] = 1;
  }

  /*
   * Send status update
   */

  if (recipient == NULL)
    
    for (i = 0; i < n_auto_update_modules; i++){
#ifdef PLAN_debug
      fprintf(stderr, "Send plan update to %s.\n",
	      tcxModuleName(auto_update_modules[i].module));
#endif
      tcxSendMsg(auto_update_modules[i].module, "PLAN_status_reply",
		 &status_data);
    }

  else
      tcxSendMsg(recipient, "PLAN_status_reply", &status_data);

  free(status_data.goal_x);
  free(status_data.goal_y);
  free(status_data.goal_distance);
  free(status_data.goal_name);
  free(status_data.goal_visible);
}






/************************************************************************
 *
 *   NAME: PLAN_register_auto_update_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void PLAN_register_auto_update_handler(TCX_REF_PTR  ref,
				      PLAN_register_auto_update_ptr data)
{
  PROGRAM_STATE_PTR program_state = all->program_state;
  
  if (PLAN_debug)
    fprintf(stderr, "Received a  PLAN_register_auto_update_message from %s.\n",
	    tcxModuleName(ref->module));

  
  /*program_state->busy++;*/	/* general flag, indicates that a TCX message
				 * was processed. */
  
  add_auto_update_module(ref->module, 
			 data->subscribe_status);

  tcxFree("PLAN_register_auto_update", data);
}





/************************************************************************
 *
 *   NAME:         TCX: PLAN_quit_message_handler
 *                 
 *   FUNCTION:     quits planner
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/




void PLAN_quit_message_handler(TCX_REF_PTR   ref,
			       void         *msg)
{
  PROGRAM_STATE_PTR program_state = all->program_state;

  /*program_state->busy++;*/
  
  if (PLAN_debug) printf("Received PLAN_quit_message.\n");
  program_state->quit = 1;
}



/************************************************************************
 *
 *   NAME:         TCX: PLAN_reset_exploration_table_handler
 *                 
 *   FUNCTION:     quits planner
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/




void PLAN_reset_exploration_table_handler(TCX_REF_PTR   ref,
					  void         *msg)
{
  ROBOT_SPECIFICATIONS_PTR robot_specifications = all->robot_specifications;
  PROGRAM_STATE_PTR program_state = all->program_state;
  int i;

  if (PLAN_debug) printf("Received PLAN_reset_exploration_table message.\n");
  
  program_state->busy++;
  
  for (i = 0; i < robot_specifications->global_map_dim_x
       * robot_specifications->global_map_dim_y; i++)
    global_visited[i]    = 0;
  
}


/************************************************************************
 *
 *   NAME:         TCX: PLAN_start_autonomous_message_handler
 *                 
 *   FUNCTION:     starts autonomous walk, may use exploration mode
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/




void PLAN_start_autonomous_message_handler(TCX_REF_PTR   ref,
					   int          *expl)
{
  PROGRAM_STATE_PTR program_state = all->program_state;
  ROBOT_STATE_PTR robot_state = all->robot_state;
  ROBOT_SPECIFICATIONS_PTR robot_specifications = all->robot_specifications;
  ACTION_PTR action = all->action;
  
  /*program_state->busy++;*/

  if (PLAN_debug) printf("Received PLAN_start_autonomous_message.\n");
  if (program_state->base_connected){
    program_state->autonomous = 1;
    G_display_switch(AUTONOMOUS_BUTTON, program_state->autonomous);

    /*
     * Previously, I used this value to determine the mode
     * of exploration. With 1, explorationgoal points were generated
     * in order to focus exploration. This turned out to be
     * computationally inefficient. Therefore I dropped this idea,
     * although the mechanism is still working in principle.
     */
    /*
       program_state->exploration = *expl;
       G_display_switch(EXPLORATION_BUTTON, program_state->exploration);
       */
    /*
     * Now, we can use this value to switch between global_active and
     * global_visited.
     */
    
    if (*expl == 1){
      global_explored = global_visited; /* just a pointer */
      G_display_switch(EXPLORATION_TYPE_BUTTON, 1);
    }
    else if (*expl == 0){
      global_explored = global_active; /* just a pointer */
      G_display_switch(EXPLORATION_TYPE_BUTTON, 0);
    }


    program_state->target_not_reached = 0;
    generate_action(robot_specifications, program_state, robot_state,
		    action, 0, 1); /* just to make sure we have an 
				    * action ready */
    if (program_state->graphics_initialized){
      G_display_markers(ADJUSTED_ACTION);
      G_display_markers(PLAN_DISPLAY);
    }
  }

  /*program_state->send_automatic_update = 1;*/
  send_automatic_status_update(NULL);

}


/************************************************************************
 *
 *   NAME:         TCX: PLAN_stop_autonomous_message_handler
 *                 
 *   FUNCTION:     stops autonomous walk
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/




void PLAN_stop_autonomous_message_handler(TCX_REF_PTR   ref,
					  int          *stop_base)
{
  PROGRAM_STATE_PTR program_state = all->program_state;
  
  /*program_state->busy++;*/

  if (PLAN_debug) printf("Received PLAN_stop_autonomous_message (%d).\n",
			      *stop_base);

  program_state->autonomous = 0;
  G_display_switch(AUTONOMOUS_BUTTON, program_state->autonomous);
  /*
     program_state->exploration = 0;
     G_display_switch(EXPLORATION_BUTTON, program_state->exploration);
     */
  if (*stop_base)
    tcx_base_stop(program_state);
  /*program_state->send_automatic_update = 1;*/
  send_automatic_status_update(NULL);

}


/************************************************************************
 *
 *   NAME:         TCX: PLAN_goal_message_handler
 *                 
 *   FUNCTION:     adds/deletes new goal points
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void PLAN_goal_message_handler(TCX_REF_PTR   ref,
			       PLAN_goal_message_ptr goal_msg)
{
  PROGRAM_STATE_PTR program_state = all->program_state;
  ROBOT_STATE_PTR robot_state = all->robot_state;
  ROBOT_SPECIFICATIONS_PTR robot_specifications = all->robot_specifications;


  /*program_state->busy++;*/

  if (PLAN_debug) printf("Received PLAN_goal_message.\n");



  if (program_state->program_initialized)
    modify_goal_set(program_state, robot_state, robot_specifications,
		    goal_msg->x, goal_msg->y, goal_msg->add, goal_msg->name);
  else				/* program not initialized */
    printf("WARNING: goal points are not accepted before initialization.\n");
  
  display_robot(robot_state, program_state);

  tcxFree("PLAN_goal_message", goal_msg); 
}



/************************************************************************
 *
 *   NAME:         TCX: PLAN_remove_all_goals_handler
 *                 
 *   FUNCTION:     adds/deletes new goal points
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void PLAN_remove_all_goals_handler(TCX_REF_PTR   ref,
				   void          *data)
{
  PROGRAM_STATE_PTR program_state = all->program_state;
  ROBOT_STATE_PTR robot_state = all->robot_state;
  ROBOT_SPECIFICATIONS_PTR robot_specifications = all->robot_specifications;



  if (PLAN_debug) printf("Received PLAN_remove_all_goals message.\n");

  remove_all_goals(program_state, robot_state, robot_specifications);

}




/************************************************************************
 *
 *   NAME:         TCX: PLAN_new_robot_pos_message_handler
 *                 
 *   FUNCTION:     updates robot position internally
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void PLAN_new_robot_pos_message_handler(TCX_REF_PTR                 ref,
					PLAN_new_robot_pos_message_ptr
					robot_msg)
{
  ROBOT_STATE_PTR robot_state = all->robot_state;
  ROBOT_SPECIFICATIONS_PTR robot_specifications = all->robot_specifications;
  PROGRAM_STATE_PTR program_state = all->program_state;
  int r_x, r_y;

  /*program_state->busy++;*/

  
  if (PLAN_debug) printf("Received PLAN_new_robot_pos_message\n");

  if (robot_msg == NULL){	/* pseudo-input simulation */
    robot_state->x = RAND() * 1000.0;
    robot_state->y = RAND() * 1000.0;
    robot_state->orientation = RAND_POS() * 360.0;
  }
  else if (ref != NULL){	/* real input */
    compute_forward_correction(robot_msg->x, 
			       robot_msg->y, 
			       robot_msg->orientation,
			       robot_state->correction_parameter_x,
			       robot_state->correction_parameter_y,
			       robot_state->correction_parameter_angle,
			       robot_state->correction_type,
			       &(robot_state->x),
			       &(robot_state->y),
			       &(robot_state->orientation));
    tcxFree("PLAN_new_robot_pos_message", robot_msg);
  }
  else{
    robot_state->x = robot_msg->x;
    robot_state->y = robot_msg->y;
    robot_state->orientation = robot_msg->orientation;
  }

  robot_state->stuck = 0;
  autoshift_display(robot_state->x,  robot_state->y,
		    robot_state, program_state, robot_specifications);
  if (program_state->graphics_initialized){
    G_activate(ROBOT);
    display_robot(robot_state, program_state);
  }

  /* adjust internal planning borders */
  r_x = ((int) (robot_state->x / robot_specifications->resolution))
    + robot_specifications->autoshifted_int_x;
  r_y = ((int) (robot_state->y / robot_specifications->resolution))
    + robot_specifications->autoshifted_int_y;
  if (r_x < robot_specifications->min_display_index_x)
    robot_specifications->min_display_index_x = r_x;
  if (r_y < robot_specifications->min_display_index_y)
    robot_specifications->min_display_index_y = r_y;
  if (r_x + 1 > robot_specifications->max_display_index_x)
    robot_specifications->max_display_index_x = r_x + 1;
  if (r_y + 1 > robot_specifications->max_display_index_y)
    robot_specifications->max_display_index_y = r_y + 1;
  check_index(robot_specifications, 1);


}

/************************************************************************
 *
 *   NAME:         TCX: PLAN_status_query_handler
 *                 
 *   FUNCTION:     handles an status query 
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void PLAN_status_query_handler(TCX_REF_PTR   ref,
			       void          *msg)
{
  int i;
  PLAN_status_reply_type status_data;
  PROGRAM_STATE_PTR program_state = all->program_state;
  ROBOT_STATE_PTR robot_state = all->robot_state;
  ROBOT_SPECIFICATIONS_PTR robot_specifications = all->robot_specifications;
  ACTION_PTR action = all->action;
  float x, y;

  /*
   * Check, if here is a recipient at all
   */

  if (n_auto_update_modules <= 0 && ref == NULL)
    return;

  /*
   * Collect all goals
   */

  status_data.num_goals = G_return_num_markers(GOALS[NUMBER], 1);

  status_data.goal_x          = (float *) (malloc(sizeof(float) 
						  * status_data.num_goals));
  status_data.goal_y          = (float *) (malloc(sizeof(float) 
						  * status_data.num_goals));
  status_data.goal_distance   = (float *) (malloc(sizeof(float) 
						 * status_data.num_goals));
  status_data.goal_name       = (int *) (malloc(sizeof(int) 
						* status_data.num_goals));
  status_data.goal_visible    = (int *) (malloc(sizeof(int) 
						* status_data.num_goals));

  if (status_data.goal_x == NULL || status_data.goal_y == NULL ||
      status_data.goal_distance == NULL || status_data.goal_name == NULL ||
      status_data.goal_visible == NULL){
    putc(7, stderr);
    fprintf(stderr, 
	    "ERROR: Out of memoery in send_automatic_status_update.\n");
    return;			/* instead of exit! */
  }

  status_data.robot_x           = robot_state->x;
  status_data.robot_y           = robot_state->y;
  status_data.robot_orientation = robot_state->orientation;
  status_data.goal_mode         = program_state->goal_modus;
  status_data.autonomous        = program_state->autonomous;

  for (i = 0; i < status_data.num_goals; i++){
    G_return_marker_coordinates(GOALS[NUMBER], i, 
				&(status_data.goal_x[i]), 
				&(status_data.goal_y[i]), 
				&(status_data.goal_name[i]));
    status_data.goal_distance[i] = sqrt(((status_data.goal_x[i]
					  - robot_state->x) 
					 * (status_data.goal_x[i]
					    - robot_state->x))
					+ ((status_data.goal_y[i] 
					    - robot_state->y)
					   * (status_data.goal_y[i] 
					      - robot_state->y)));

    x = ((int) (status_data.goal_x[i] / robot_specifications->resolution))
      + robot_specifications->autoshifted_int_x;
    y = ((int) (status_data.goal_y[i] / robot_specifications->resolution))
      + robot_specifications->autoshifted_int_y;

    if (x < 0 || x >= robot_specifications->global_map_dim_x ||
	y < 0 || y >= robot_specifications->global_map_dim_y)
      status_data.goal_visible[i] = 0;
    else
      status_data.goal_visible[i] = 1;
  }

  /*
   * Send status reply
   */

  tcxSendMsg(ref->module, "PLAN_status_reply", &status_data);
  
  free(status_data.goal_x);
  free(status_data.goal_y);
  free(status_data.goal_distance);
  free(status_data.goal_name);
  free(status_data.goal_visible);

}


/************************************************************************
 *
 *   NAME:         TCX: PLAN_action_query_handler
 *                 
 *   FUNCTION:     handles an action query, also gives new robot positions
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void PLAN_action_query_handler(TCX_REF_PTR   ref,
			       PLAN_action_query_ptr question_msg)
{
  ROBOT_STATE_PTR robot_state = all->robot_state;
  ROBOT_SPECIFICATIONS_PTR robot_specifications = all->robot_specifications;
  PROGRAM_STATE_PTR program_state = all->program_state;
  PLAN_action_reply_type answer;
  int r_x, r_y, i;

  /* =======================================
   *      Action is kept local
   * =======================================
   */

  ACTION_TYPE              local_action_data;
  ACTION_PTR               local_action               = &local_action_data;


  /*program_state->busy++;*/

  /* -------- receive request and set internal state accordingly ------ */

  if (ref != NULL && question_msg != NULL){
    if (PLAN_debug) printf("Received PLAN_action_query\n");

 
    robot_state->x = question_msg->x;
    robot_state->y = question_msg->y;
    robot_state->orientation = question_msg->orientation;
    robot_state->stuck = question_msg->stuck;


    if (program_state->graphics_initialized){
      G_activate(ROBOT);
      display_robot(robot_state, program_state);
    }
    
    if (robot_state->stuck)
      printf("\n     ...exception planning: robot is stuck\n\n");

    tcxFree("PLAN_action_query", question_msg); 
    question_msg = NULL;	/* we are being careful - will cause segm
				 * fault if accidentally used */

    generate_action(robot_specifications, program_state, robot_state,
		    local_action, 1, 0);
    
    answer.turn =             local_action->turn;
    answer.base =             local_action->base;
    answer.final_action =     local_action->final_action;
    answer.active_goal_name = local_action->active_goal_name;
    answer.goal_x           = local_action->goal_x;
    answer.goal_y           = local_action->goal_y;
    answer.goal_dist        = local_action->goal_dist;
    

    tcxReply(ref, "PLAN_action_reply", &answer);

    if (program_state->graphics_initialized){
      G_display_markers(ADJUSTED_ACTION);
      G_display_markers(PLAN_DISPLAY);
    }

  }
  
}


#ifdef UNIBONN

/************************************************************************
 *
 *
 *   NAME:         tcx_speech_talk_text
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void tcx_speech_talk_text(PROGRAM_STATE_PTR program_state, char *text)
{
  if (!program_state->speech_connected)
    connect_to_SPEECH(program_state);
  if (program_state->speech_connected)
    tcxSendMsg(SPEECH, "SPEECH_talk_text", &text);
}
#endif


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         MAP_partial_map_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void MAP_partial_map_reply_handler(TCX_REF_PTR                 ref,
				   MAP_partial_map_reply_ptr map_msg)
{
  ROBOT_STATE_PTR robot_state = all->robot_state;
  PROGRAM_STATE_PTR program_state = all->program_state;
  ROBOT_SPECIFICATIONS_PTR robot_specifications = all->robot_specifications;
  int x, y, global_x, global_y;
  int map_msg_index, global_map_index;
  float point_costs, costs_factor;
  float x_rotated, y_rotated;
  float x_float, y_float, angle;
  unsigned char char_value;
  int initialize;

#ifdef PLAN_debug
  fprintf(stderr, "   Received MAP_partial_map_reply\n");
#endif
  
  if (map_msg == NULL) {
    fprintf(stderr, "NULL map.\n");
    return;
  }

  if (map_msg->resolution != robot_specifications->resolution){
    fprintf(stderr, "ERROR: Resolution mismatch. Message ignored.\n");
  }
  else{


    if ( PLAN_debug) printf("   Received MAP_partial_map_reply\n");
    /* Fox: we need to make sure that each map is used. */
    if (0 && program_state->busy >= 3){
      fprintf(stderr, "WARNING: Lost map update. Too busy.\n");
      tcxFree("MAP_partial_map_reply", map_msg); 
      return;
    }


    if ( PLAN_debug)
      fprintf(stderr, "BUSY-%d\n", program_state->busy);

    program_state->busy++;


    if (map_msg->delete_previous_map){
      clear_maps(robot_specifications);
      display_all(-1, robot_state, robot_specifications, program_state);
    }


    initialize =  (program_state->actual_map_number != map_msg->number_of_map);

    program_state->actual_map_number = map_msg->number_of_map;
    /* fprintf(stderr, "MAP: %d\n", map_msg->number_of_map); */

    autoshift_display((((float) map_msg->first_x)
		       * robot_specifications->resolution)
		      + robot_specifications->robot_size,
		      (((float) map_msg->first_y)
		       * robot_specifications->resolution)
		      + robot_specifications->robot_size,
		      robot_state, program_state, robot_specifications);


    autoshift_display((((float) (map_msg->first_x + map_msg->size_x))
		       * robot_specifications->resolution)
		      - robot_specifications->robot_size,
		      (((float) (map_msg->first_y + map_msg->size_y))
		       * robot_specifications->resolution)
		      - robot_specifications->robot_size,
		      robot_state, program_state, robot_specifications);


    angle = robot_state->map_orientation * M_PI / 180.0;


    for (x = 0, global_x = x + map_msg->first_x 
	   + robot_specifications->autoshifted_int_x;
	 x < map_msg->size_x; x++, global_x++)
      for (y = 0, global_y = y + map_msg->first_y 
	     + robot_specifications->autoshifted_int_y;
	   y < map_msg->size_y; y++, global_y++){
	map_msg_index = x * map_msg->size_y + y;
	global_map_index  = global_x * 
	  robot_specifications->global_map_dim_y + global_y;
	if (global_x >= 0 && global_y >= 0 &&
	    global_x < robot_specifications->global_map_dim_x &&
	    global_y < robot_specifications->global_map_dim_y){


	  /* ====== set value and active =========== */
	  char_value = map_msg->char_values[map_msg_index];
	  if (char_value == (unsigned char) 0){
	    global_active[global_map_index] = 0;
	    global_values[global_map_index] = 1.0;
	    global_values[global_map_index]     = 0.0;
	    global_active[global_map_index]     = 0;
	    global_visited[global_map_index]    = 0;
	    global_costs[global_map_index]      =
	      robot_specifications->average_costs;
	    global_costs2[global_map_index]     =
	      robot_specifications->average_costs2;
	    global_utility[global_map_index] = 0.0;
	    global_goal[global_map_index]    = 0;
	    global_succ[global_map_index]    = -1;
	  }
	  else if (char_value == (unsigned char) 255){
	    global_active[global_map_index] = 1;
	    global_values[global_map_index] = 1.0;
	  }
	  else{
	    global_active[global_map_index] = 1;
	    global_values[global_map_index] = ((float) (char_value-1)) / 253.0;
	  }
	
	  if (global_active[global_map_index]){
	  
	    /* ===== adjust bounding box ======= */
	    if (robot_state->map_orientation_defined){
	      x_float = 
		(((float) (global_x - robot_specifications->autoshifted_int_x))
		 * robot_specifications->resolution);
	      y_float = 
		(((float) (global_y - robot_specifications->autoshifted_int_y))
		 * robot_specifications->resolution);
	      x_rotated = (cos(-angle) * x_float) - (sin(-angle) * y_float);
	      y_rotated = (cos(-angle) * y_float) + (sin(-angle) * x_float);

	      if (program_state->bounding_box_min_x >
		  x_rotated + robot_specifications->border_to_interior ||
		  !program_state->bounding_box_defined)
		program_state->bounding_box_min_x = 
		  x_rotated + robot_specifications->border_to_interior;
	      if (program_state->bounding_box_max_x <
		  x_rotated - robot_specifications->border_to_interior ||
		  !program_state->bounding_box_defined)
		program_state->bounding_box_max_x = 
		  x_rotated - robot_specifications->border_to_interior;
	      if (program_state->bounding_box_min_y >
		  y_rotated + robot_specifications->border_to_interior||
		  !program_state->bounding_box_defined)
		program_state->bounding_box_min_y =
		  y_rotated + robot_specifications->border_to_interior;
	      if (program_state->bounding_box_max_y < 
		  y_rotated - robot_specifications->border_to_interior ||
		  !program_state->bounding_box_defined)
		program_state->bounding_box_max_y = 
		  y_rotated - robot_specifications->border_to_interior;
	      program_state->bounding_box_defined = 1;
	    }
	  
	    /* ====== set cost =========== */
	    global_costs[global_map_index] = 
	      costs_function(robot_specifications, 
			     global_values[global_map_index]);
	    global_costs2[global_map_index] = pow(global_costs[global_map_index],
						  sqrt2);

	  
	    /* ====== set utility =========== */

	    global_utility[global_map_index] = 
	      utility_function(global_costs[global_map_index], 
			       global_goal[global_map_index]);

	    /* ====== set succ =========== */
	    global_succ[global_map_index] = -1;

	    /* ====== set index =========== */
	    if (global_x - 1 < robot_specifications->min_display_index_x)
	      robot_specifications->min_display_index_x = global_x - 1;
	    if (global_y - 1 < robot_specifications->min_display_index_y)
	      robot_specifications->min_display_index_y = global_y - 1;
	    if (global_x + 2 > robot_specifications->max_display_index_x)
	      robot_specifications->max_display_index_x = global_x + 2;
	    if (global_y + 2 > robot_specifications->max_display_index_y)
	      robot_specifications->max_display_index_y = global_y + 2;
	  

	  }
	}
      }

  
    if (robot_state->map_orientation_defined){
      G_clear_markers(BOUNDING_BOX);
      
      G_add_marker(BOUNDING_BOX, 
		   (program_state->bounding_box_min_x
		    * cos (angle))
		   - (program_state->bounding_box_min_y
		      * sin (angle)),
		   (program_state->bounding_box_min_y
		    * cos (angle))
		   + (program_state->bounding_box_min_x
		      * sin (angle)), 0);

      G_add_marker(BOUNDING_BOX, 
		   (program_state->bounding_box_max_x
		    * cos (angle))
		   - (program_state->bounding_box_min_y
		      * sin (angle)),
		   (program_state->bounding_box_min_y
		    * cos (angle))
		   + (program_state->bounding_box_max_x
		      * sin (angle)), 0);
      G_add_marker(BOUNDING_BOX, 
		   (program_state->bounding_box_max_x
		    * cos (angle))
		   - (program_state->bounding_box_max_y
		      * sin (angle)),
		   (program_state->bounding_box_max_y
		    * cos (angle))
		   + (program_state->bounding_box_max_x
		      * sin (angle)), 0);
      G_add_marker(BOUNDING_BOX, 
		   (program_state->bounding_box_min_x
		    * cos (angle))
		   - (program_state->bounding_box_max_y
		      * sin (angle)),
		   (program_state->bounding_box_max_y
		    * cos (angle))
		   + (program_state->bounding_box_min_x
		      * sin (angle)), 0);

      G_add_marker(BOUNDING_BOX, 
		   (program_state->bounding_box_min_x
		    * cos (angle))
		   - (program_state->bounding_box_min_y
		      * sin (angle)),
		   (program_state->bounding_box_min_y
		    * cos (angle))
		   + (program_state->bounding_box_min_x
		      * sin (angle)), 0);

      fprintf(stderr, "BOUNDING BOX size = %f\n",
	      (program_state->bounding_box_max_x
	       - program_state->bounding_box_min_x)
	      * (program_state->bounding_box_max_y
		 - program_state->bounding_box_min_y)
	      * robot_specifications->resolution
	      * robot_specifications->resolution);
    }
    
    if (!program_state->interior_mode &&
	(program_state->bounding_box_max_x
	 - program_state->bounding_box_min_x)
	* (program_state->bounding_box_max_y
	   - program_state->bounding_box_min_y)
	* robot_specifications->resolution
	* robot_specifications->resolution > 
	robot_specifications->max_bounding_box_size){
      program_state->interior_mode = 1;
      G_display_switch(EXPLORATION_INTERIOR_MODE_BUTTON,
		       program_state->interior_mode);
      count_goals_and_reset_utilities(program_state, robot_specifications, 
				      robot_state);
    }

      

    /*G_display_markers(BOUNDING_BOX);*/
  
  

    if (initialize)
      count_goals_and_reset_utilities(program_state, robot_specifications,
				      robot_state);
    else{
      check_index(robot_specifications, 1);
      reset_descending_utilities(robot_specifications, program_state,
				 robot_state);    
      program_state->reset_descending_utilities_flag = 0;
      search_for_inconsistencies_in_succ(robot_specifications, "MAP_reply");
    }
  }
  tcxFree("MAP_partial_map_reply", map_msg); 


}


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         MAP_correction_parameters_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void MAP_correction_parameters_reply_handler(TCX_REF_PTR                 ref,
				     MAP_correction_parameters_reply_ptr corr)
{
  ROBOT_STATE_PTR robot_state = all->robot_state;
  PROGRAM_STATE_PTR program_state = all->program_state;
  ROBOT_SPECIFICATIONS_PTR robot_specifications = all->robot_specifications;
  float uncorr_robot_x, uncorr_robot_y, uncorr_robot_orientation;
  int x, y, index;
  float x_float, y_float, angle, x_rotated, y_rotated;

  /*program_state->busy++;*/

  if (PLAN_debug)
    fprintf(stderr, 
	    "TCX: Received a MAP_correction_parameters_reply message.\n");

    
  compute_backward_correction(robot_state->x, robot_state->y, 
			      robot_state->orientation,
			      robot_state->correction_parameter_x,
			      robot_state->correction_parameter_y,
			      robot_state->correction_parameter_angle,
			      robot_state->correction_type,
			      &uncorr_robot_x, &uncorr_robot_y,  
			      &uncorr_robot_orientation);
  
  robot_state->correction_parameter_x     = corr->parameter_x;
  robot_state->correction_parameter_y     = corr->parameter_y;
  robot_state->correction_parameter_angle = corr->parameter_angle;
  robot_state->correction_type            = corr->type;

  compute_forward_correction(uncorr_robot_x, 
			     uncorr_robot_y, 
			     uncorr_robot_orientation,
			     robot_state->correction_parameter_x,
			     robot_state->correction_parameter_y,
			     robot_state->correction_parameter_angle,
			     robot_state->correction_type,
			     &(robot_state->x),
			     &(robot_state->y),
			     &(robot_state->orientation));

  /*
   * copy the wall angle, and update the bounding box, if necessary
   */


  if (robot_state->map_orientation_defined &&
      !corr->current_wall_angle_defined)

    program_state->bounding_box_defined = 0;


  else if (!robot_state->map_orientation_defined &&
	   corr->current_wall_angle_defined){

    program_state->bounding_box_defined = 0;
    angle = corr->current_wall_angle * M_PI / 180.0;

    for (x = 0; x < robot_specifications->global_map_dim_x; x += 2)
      for (y = 0; y < robot_specifications->global_map_dim_y; y += 2){
	index = x * robot_specifications->global_map_dim_y + y;
	if (global_active[index]){
	  x_float = 
	    (((float) (x - robot_specifications->autoshifted_int_x))
	     * robot_specifications->resolution);
	  y_float = 
	    (((float) (y - robot_specifications->autoshifted_int_y))
	     * robot_specifications->resolution);
	  x_rotated = (cos(-angle) * x_float) - (sin(-angle) * y_float);
	  y_rotated = (cos(-angle) * y_float) + (sin(-angle) * x_float);

	  if (program_state->bounding_box_min_x >
	      x_rotated + robot_specifications->border_to_interior ||
	      !program_state->bounding_box_defined)
	    program_state->bounding_box_min_x = 
	      x_rotated + robot_specifications->border_to_interior;
	  if (program_state->bounding_box_max_x <
	      x_rotated - robot_specifications->border_to_interior ||
	      !program_state->bounding_box_defined)
	    program_state->bounding_box_max_x = 
	      x_rotated - robot_specifications->border_to_interior;
	  if (program_state->bounding_box_min_y >
	      y_rotated + robot_specifications->border_to_interior||
	      !program_state->bounding_box_defined)
	    program_state->bounding_box_min_y =
	      y_rotated + robot_specifications->border_to_interior;
	  if (program_state->bounding_box_max_y < 
	      y_rotated - robot_specifications->border_to_interior ||
	      !program_state->bounding_box_defined)
	    program_state->bounding_box_max_y = 
	      y_rotated - robot_specifications->border_to_interior;
	  program_state->bounding_box_defined = 1;
	}
      }


  G_clear_markers(BOUNDING_BOX);

  G_add_marker(BOUNDING_BOX, 
	       (program_state->bounding_box_min_x
		* cos (angle))
	       - (program_state->bounding_box_min_y
		  * sin (angle)),
	       (program_state->bounding_box_min_y
		* cos (angle))
	       + (program_state->bounding_box_min_x
		  * sin (angle)), 0);

  G_add_marker(BOUNDING_BOX, 
	       (program_state->bounding_box_max_x
		* cos (angle))
	       - (program_state->bounding_box_min_y
		  * sin (angle)),
	       (program_state->bounding_box_min_y
		* cos (angle))
	       + (program_state->bounding_box_max_x
		  * sin (angle)), 0);
  G_add_marker(BOUNDING_BOX, 
	       (program_state->bounding_box_max_x
		* cos (angle))
	       - (program_state->bounding_box_max_y
		  * sin (angle)),
	       (program_state->bounding_box_max_y
		* cos (angle))
	       + (program_state->bounding_box_max_x
		  * sin (angle)), 0);
  G_add_marker(BOUNDING_BOX, 
	       (program_state->bounding_box_min_x
		* cos (angle))
	       - (program_state->bounding_box_max_y
		  * sin (angle)),
	       (program_state->bounding_box_max_y
		* cos (angle))
	       + (program_state->bounding_box_min_x
		  * sin (angle)), 0);

  G_add_marker(BOUNDING_BOX, 
	       (program_state->bounding_box_min_x
		* cos (angle))
	       - (program_state->bounding_box_min_y
		  * sin (angle)),
	       (program_state->bounding_box_min_y
		* cos (angle))
	       + (program_state->bounding_box_min_x
		  * sin (angle)), 0);

    if (!program_state->interior_mode &&
	(program_state->bounding_box_max_x
	 - program_state->bounding_box_min_x)
	* (program_state->bounding_box_max_y
	   - program_state->bounding_box_min_y)
	* robot_specifications->resolution
	* robot_specifications->resolution > 
	robot_specifications->max_bounding_box_size){
      program_state->interior_mode = 1;
      G_display_switch(EXPLORATION_INTERIOR_MODE_BUTTON,
		       program_state->interior_mode);
      count_goals_and_reset_utilities(program_state, robot_specifications, 
				      robot_state);
    }
    
      
    G_display_markers(BOUNDING_BOX);
 }

  
  robot_state->map_orientation_defined = corr->current_wall_angle_defined;
  robot_state->map_orientation         = corr->current_wall_angle;


  tcxFree("MAP_correction_parameters_reply", corr);
}



/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         SONAR_ir_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SONAR_ir_reply_handler (TCX_REF_PTR          ref,
			     SONAR_ir_reply_ptr   ir)
{


/*#ifdef TCX_debug*/
  fprintf(stderr, "TCX: Received a SONAR_ir_reply message.\n");
/*#endif*/

  tcxFree("SONAR_ir_reply", ir);

}


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         BASE_update_status_reply_handler 
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_update_status_reply_handler(TCX_REF_PTR                   ref,
				      BASE_update_status_reply_ptr status)
{
  ROBOT_STATE_PTR robot_state = all->robot_state;
  ROBOT_SPECIFICATIONS_PTR robot_specifications = all->robot_specifications;
  PROGRAM_STATE_PTR program_state = all->program_state;
  ACTION_PTR action = all->action;
  int r_x, r_y, x, y, circle, index;
  int generate_new_action;
  float maxGoalDistance;
  
  /*program_state->busy++;*/

  if (PLAN_debug) fprintf(stderr, "Received BASE_update_status_reply: %g %g %g.\n",
			      status->pos_x, status->pos_y,
			      status->orientation);
  
  if (status == NULL){	/* pseudo-input simulation */
    robot_state->x = RAND_POS() * 2000.0;
    robot_state->y = RAND_POS() * 2000.0;
    robot_state->orientation = RAND_POS() * 360.0;
    fprintf(stderr, "\n\n\t\t? ? ? \n\n");
  }
  else{				/* real input */
    compute_forward_correction(status->pos_x, 
			       status->pos_y, 
			       90.0 - status->orientation,
			       robot_state->correction_parameter_x,
			       robot_state->correction_parameter_y,
			       robot_state->correction_parameter_angle,
			       robot_state->correction_type,
			       &(robot_state->x),
			       &(robot_state->y),
			       &(robot_state->orientation));
  }

  /* adjust internal planning borders */
  r_x = ((int) (robot_state->x / robot_specifications->resolution))
    + robot_specifications->autoshifted_int_x;
  r_y = ((int) (robot_state->y / robot_specifications->resolution))
    + robot_specifications->autoshifted_int_y;
  if (r_x < robot_specifications->min_display_index_x)
    robot_specifications->min_display_index_x = r_x;
  if (r_y < robot_specifications->min_display_index_y)
    robot_specifications->min_display_index_y = r_y;
  if (r_x + 1 > robot_specifications->max_display_index_x)
    robot_specifications->max_display_index_x = r_x + 1;
  if (r_y + 1 > robot_specifications->max_display_index_y)
    robot_specifications->max_display_index_y = r_y + 1;
  
  check_index(robot_specifications, 1);


  robot_state->stuck = 0;
  autoshift_display(robot_state->x,  robot_state->y,
		    robot_state, program_state, robot_specifications);


  
  

  if (global_explored == global_visited){
    
    
    if (program_state->busy >= 3)
      
      fprintf(stderr, "WARNING: Lost status update. Too busy.\n");
    
    else{
      
      /*
       * update exploration table
       */
      
      
      circle = ((int) (robot_specifications->exploration_circle_size
		       / robot_specifications->resolution));
      

      if ( program_state->busy)
	  fprintf(stderr, "BUSY-%d\n", program_state->busy);
      
      program_state->busy++;
      

      if (r_x - circle - 1 < robot_specifications->min_display_index_x)
	robot_specifications->min_display_index_x = r_x - circle - 1;
      if (r_y - circle - 1 < robot_specifications->min_display_index_y)
	robot_specifications->min_display_index_y = r_y - circle - 1;
      if (r_x + circle + 2 > robot_specifications->max_display_index_x)
	robot_specifications->max_display_index_x = r_x + circle + 2;
      if (r_y + circle + 2 > robot_specifications->max_display_index_y)
	robot_specifications->max_display_index_y = r_y + circle + 2;
      
      if (r_x - circle - 1 < robot_specifications->min_plan_index_x)
	robot_specifications->min_plan_index_x = r_x - circle - 1;
      if (r_y - circle - 1 < robot_specifications->min_plan_index_y)
	robot_specifications->min_plan_index_y = r_y - circle - 1;
      if (r_x + circle + 2 > robot_specifications->max_plan_index_x)
	robot_specifications->max_plan_index_x = r_x + circle + 2;
      if (r_y + circle + 2 > robot_specifications->max_plan_index_y)
	robot_specifications->max_plan_index_y = r_y + circle + 2;
    
      check_index(robot_specifications, 1);
      
      for (x = r_x - circle; x <= r_x + circle; x++)
	if (x >= 0 && x < robot_specifications->global_map_dim_x)
	  for (y = r_y - circle; y <= r_y + circle; y++)
	    if (y >= 0 && y < robot_specifications->global_map_dim_y){
	      if (((r_x-x) * (r_x-x)) + ((r_y-y) * (r_y-y)) 
		  < circle * circle){
		index = x * robot_specifications->global_map_dim_x + y;
		/* ====== set visited =========== */
		global_visited[index] = 1;
#ifdef xxx
		/* ====== set utility =========== */
		if (program_state->goal_modus)
		  global_utility[index] = 
		    utility_function(global_costs[index], 
				     global_goal[index]);
		else
		  global_utility[index] = 0.0;
		
		/* ====== set succ =========== */
		global_succ[index] = -1;
#endif
	      }
	    }
      reset_descending_utilities(robot_specifications, program_state,
				 robot_state);    
      program_state->reset_descending_utilities_flag = 0;
      search_for_inconsistencies_in_succ(robot_specifications, 
					 "status_update");
    }
  }
  
  
  /* Fox: if the current action is the final one we check for max_final_goal_distance. */
  maxGoalDistance = action->final_action ?
    robot_specifications->max_final_approach_distance : robot_specifications->max_goal_distance;
  
  check_if_goal_reached(program_state,
			robot_state,
			robot_specifications,
			robot_state->x, robot_state->y,
			robot_specifications->max_goal_distance);

  
  if (program_state->base_connected){
    
    
    /*
     * Sending action commands are only considered when we are allowd to
     */
    
    if (program_state->autonomous){

    
      generate_new_action = 0;
      /*
      {
	static int i = 0;
	if (!i){
	  fprintf(stderr, "\n\n\t\t HACK IN THE PROGRAM. Used to be 0.\n\n");
	  i = 1;
	}
      }
      */      
      
      /*
       * We'd like to generate a new action, if we figure that
       * the robot cannot get to the suboal on a straight line
       */
      
      if (!check_if_point_reachable_by_straight_line(action->absolute_x,
						     action->absolute_y,
						     robot_state,
						     robot_specifications)){
	generate_new_action = 1;
      }

      else if (add_exploration_goal(robot_specifications, program_state,
			       robot_state)){
	generate_new_action = 1;
      }
      
      if (program_state->actual_map_number == 0 &&
	  robot_specifications->number_active_goals[BEST_NUMBER] > 0){
	if (check_if_goal_reached(program_state,
				  robot_state,
				  robot_specifications,
				  robot_state->x, robot_state->y,
				  robot_specifications->max_goal_distance)){
	  if (robot_specifications->number_active_goals[BEST_NUMBER] == 0){
	    generate_new_action = 0;
	    program_state->autonomous = 0;
	    G_display_switch(AUTONOMOUS_BUTTON, program_state->autonomous);
	    fprintf(stderr, "Rhino reached goal.\n");
	    putc( 7, stderr);
	    putc( 7, stderr);
#ifdef UNIBONN
	    tcx_speech_talk_text(program_state, "I reached my goal.");
#endif
	    tcx_base_stop(program_state);
	    program_state->send_automatic_update = 1;
	  }
	  else{
	    generate_new_action = 1;
	  }
	}
      }
      
      
      
      /*
       * Generate a new action, if necessary. Oherwise, refresh the old one
       */
      
      if (program_state->autonomous && 
	  (generate_new_action ||
	   robot_specifications->generate_actions_continuously)){
	/*
	 * putc(7, stderr);
	 * fprintf(stderr, "### status ###\n");
	 */
	generate_action(robot_specifications, program_state, robot_state,
			action, 1, 1);

	if (0 && program_state->graphics_initialized){
	  G_display_markers(ADJUSTED_ACTION);
	  G_display_markers(PLAN_DISPLAY);
	}
      }
      /*
       * else
       * fprintf(stderr, " # \n");
       */
      
      if (program_state->autonomous)
	tcx_base_goto_absolute(program_state, robot_specifications,
			       robot_state, action, generate_new_action);
      
    }

  }


  if (program_state->graphics_initialized){
    G_activate(ROBOT);
    display_robot(robot_state, program_state);
  }
  
    
  
  
  tcxFree("PLAN_new_robot_pos_message", status);
}




/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         BASE_action_executed_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void BASE_action_executed_reply_handler(TCX_REF_PTR                    ref,
					BASE_action_executed_reply_ptr pos)
{
  ROBOT_STATE_PTR robot_state = all->robot_state;
  ROBOT_SPECIFICATIONS_PTR robot_specifications = all->robot_specifications;
  PROGRAM_STATE_PTR program_state = all->program_state;
  ACTION_PTR action = all->action;
  int r_x, r_y;


  /*program_state->busy++;*/
  
  if (PLAN_debug) fprintf(stderr, "Received BASE_action_executed_reply: %g %g %g.\n",
			 pos->x, pos->y,
			 pos->orientation);
  if (pos != NULL){
    compute_forward_correction(pos->x, 
			       pos->y, 
			       90.0 - pos->orientation,
			       robot_state->correction_parameter_x,
			       robot_state->correction_parameter_y,
			       robot_state->correction_parameter_angle,
			       robot_state->correction_type,
			       &(robot_state->x),
			       &(robot_state->y),
			       &(robot_state->orientation));
  }
  else 
    fprintf(stderr, "\n\n\t\t? ? ? \n\n");



  robot_state->stuck = 0;
  autoshift_display(robot_state->x,  robot_state->y,
		    robot_state, program_state, robot_specifications);


  /* adjust internal planning borders */
  r_x = ((int) (robot_state->x / robot_specifications->resolution))
    + robot_specifications->autoshifted_int_x;
  r_y = ((int) (robot_state->y / robot_specifications->resolution))
    + robot_specifications->autoshifted_int_y;
  if (r_x < robot_specifications->min_display_index_x)
    robot_specifications->min_display_index_x = r_x;
  if (r_y < robot_specifications->min_display_index_y)
    robot_specifications->min_display_index_y = r_y;
  if (r_x + 1 > robot_specifications->max_display_index_x)
    robot_specifications->max_display_index_x = r_x + 1;
  if (r_y + 1 > robot_specifications->max_display_index_y)
    robot_specifications->max_display_index_y = r_y + 1;
  check_index(robot_specifications, 1);


  
  
    
  if (program_state->autonomous){

#ifdef JUNK_DO_NOT_USE
    /*
     *  BASE is done with the latest target point, let's generate a new one
     */
    if (action->final_action &&
	robot_specifications->number_active_goals[BEST_NUMBER] > 0){
      /*
       * Ohh, this target point was a "goal" in the planner.
       * Let's remove it
       */
      fprintf(stderr, "Robot at %g %g. Remove goal at %g %g\n", 
	      robot_state->x, robot_state->y, action->goal_x, action->goal_y);
      modify_goal_set(program_state, robot_state, robot_specifications,
		      action->goal_x, action->goal_y, 0, 0);
      action->final_action = 0;
      program_state->target_not_reached = 0;
      
      if (robot_specifications->number_active_goals[BEST_NUMBER] == 0){
	/*
	 *  And it so happened that this was the last target point.
	 * Then we are done and can finish the autonomous mode.
	 */
	fprintf(stderr, "Rhino has reached the global target point.\n");
	if (!program_state->exploration){
	  program_state->autonomous = 0;
	  G_display_switch(AUTONOMOUS_BUTTON, program_state->autonomous);
	  tcx_base_stop(program_state);
	  program_state->send_automatic_update = 1;
	}
	else
	  add_exploration_goal(robot_specifications, program_state,
			       robot_state);
      }
    }
#endif	
    
    if (program_state->autonomous){
      add_exploration_goal(robot_specifications, program_state,
			   robot_state);
      /*
       * putc(7, stderr);
       * fprintf(stderr, "### executed ###\n");
       */
      generate_action(robot_specifications, program_state, robot_state,
		      action, 1, 1);
      /*display_plan_box(robot_specifications, program_state);*/
      if (0 && program_state->graphics_initialized){
	G_display_markers(ADJUSTED_ACTION);
	G_display_markers(PLAN_DISPLAY);
      }
      
      tcx_base_goto_absolute(program_state, robot_specifications,
			     robot_state, action, 1);
    }
  }
      
}






void BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				       BASE_robot_position_reply_ptr pos)
{
  PROGRAM_STATE_PTR program_state = all->program_state;

  /*program_state->busy++;*/

  if (PLAN_debug) printf("Received BASE_robot_position_reply. Not handled.\n");
  
}

void SONAR_sonar_reply_handler(TCX_REF_PTR                ref,
			       SONAR_sonar_reply_ptr      data)
{
  PROGRAM_STATE_PTR program_state = all->program_state;

  /*program_state->busy++;*/

  if (PLAN_debug) printf("Received SONAR_sonar_reply. Not handled.\n");
}




/************************************************************************
 *
 *   NAME:         TCX: PLAN_constraints_message_handler
 *                 
 *   FUNCTION:     adds a new constraint to (or resets) the constraints
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void PLAN_constraints_message_handler(TCX_REF_PTR   ref,
				      PLAN_constraints_message_ptr 
				      new_constraint)
{
  ROBOT_STATE_PTR robot_state = all->robot_state;
  PROGRAM_STATE_PTR program_state = all->program_state;
  ROBOT_SPECIFICATIONS_PTR robot_specifications = all->robot_specifications;
  int i;

  /*program_state->busy++;*/


  if (PLAN_debug) printf("   Received PLAN_constraints_message\n");

  if (new_constraint->type == 0){
    n_constraints = 0;
    change_table(robot_specifications, 0, program_state);
    printf("All constraints deleted.\n");
    program_state->warmer_colder_game = 0;
    if (program_state->graphics_initialized){
      for (i = 1; i < nNUMBER; i++)
	G_clear_markers(GOALS[i]);
      G_clear_markers(CONSTRAINTS);
      display_all(-1, robot_state, robot_specifications, program_state);
    }
  }
  else if (n_constraints == max_n_constraints)
    printf("WARNING: too many constraints.\n");
  

  else{
    if (!program_state->warmer_colder_game){
      if (program_state->graphics_initialized){
	for (i = 1; i < nNUMBER; i++)
	  G_clear_markers(GOALS[i]);
      }
      program_state->warmer_colder_game = 1;
      display_all(-1, robot_state, robot_specifications, program_state);
    }
    constraints[n_constraints].type   = new_constraint->type;
    constraints[n_constraints].from_x = new_constraint->from_x;
    constraints[n_constraints].to_x   = new_constraint->to_x;
    constraints[n_constraints].from_y = new_constraint->from_y;
    constraints[n_constraints].to_y   = new_constraint->to_y;
    if (program_state->graphics_initialized){
      G_add_marker(CONSTRAINTS, new_constraint->from_x, new_constraint->from_y,
		   0);
      G_add_marker(CONSTRAINTS, new_constraint->to_x, new_constraint->to_y,
		   new_constraint->type);
    }
    
    if (PLAN_debug) printf("New Constraint: # %d type %d =  %g %g  %g %g\n",
			   n_constraints,
			   constraints[n_constraints].type,
			   constraints[n_constraints].from_x,
			   constraints[n_constraints].to_x,
			   constraints[n_constraints].from_y,
			   constraints[n_constraints].to_y);
    n_constraints++;
  }    

  tcxFree("PLAN_constraints_message", new_constraint);
}




/************************************************************************
 *
 *   NAME:         PLAN_parameter_handler
 *                 
 *   FUNCTION:     handles a message including some parameters
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void PLAN_parameter_message_handler( TCX_REF_PTR                    ref,
				     PLAN_parameter_message_ptr parameter)
{
  ROBOT_SPECIFICATIONS_PTR robot_specifications = all->robot_specifications;
  
  fprintf( stderr, "Got new parameters.\n");
  
  robot_specifications->max_security_dist             =
    parameter->max_security_dist;
  robot_specifications->max_adjust_angle              =
    parameter->max_adjust_angle;
  robot_specifications->collision_threshold           =
    parameter->collision_threshold;
  robot_specifications->max_goal_distance             =
    parameter->max_goal_distance;
  robot_specifications->max_final_approach_distance   =
    parameter->max_final_approach_distance;
  robot_specifications->max_approach_distance         =
    parameter->max_approach_distance;

  tcxFree( "PLAN_parameter_message", parameter);
}
  




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

TCX_REG_HND_TYPE PLAN_handler_array[] = {
  {"PLAN_register_auto_update", "PLAN_register_auto_update_handler",
     PLAN_register_auto_update_handler, TCX_RECV_ALL, NULL},
  {"PLAN_goal_message", "PLAN_goal_message_handler",
     PLAN_goal_message_handler, TCX_RECV_ALL, NULL},
  {"PLAN_new_robot_pos_message", "PLAN_new_robot_pos_message_handler",
     PLAN_new_robot_pos_message_handler, TCX_RECV_ALL, NULL},
  {"PLAN_status_query", "PLAN_status_query_handler",
     PLAN_status_query_handler, TCX_RECV_ALL, NULL},
  {"PLAN_action_query", "PLAN_action_query_handler",
     PLAN_action_query_handler, TCX_RECV_ALL, NULL},
  {"PLAN_constraints_message", "PLAN_constraints_message_handler",
     PLAN_constraints_message_handler, TCX_RECV_ALL, NULL},
  {"PLAN_quit_message", "PLAN_quit_message_handler",
     PLAN_quit_message_handler, TCX_RECV_ALL, NULL},
  {"PLAN_start_autonomous_message", "PLAN_start_autonomous_message_handler",
     PLAN_start_autonomous_message_handler, TCX_RECV_ALL, NULL},
  {"PLAN_stop_autonomous_message", "PLAN_stop_autonomous_message_handler",
     PLAN_stop_autonomous_message_handler, TCX_RECV_ALL, NULL},
  {"PLAN_remove_all_goals", "PLAN_remove_all_goals_handler",
     PLAN_remove_all_goals_handler, TCX_RECV_ALL, NULL},
  {"PLAN_reset_exploration_table", "PLAN_reset_exploration_table_handler",
     PLAN_reset_exploration_table_handler, TCX_RECV_ALL, NULL},
  {"PLAN_parameter_message", "PLAN_parameter_message_handler",
     PLAN_parameter_message_handler, TCX_RECV_ALL, NULL},
};




/************************************************************************
 *
 *   NAME:         init_tcx
 *                 
 *   FUNCTION:     waits and connects to tcx 
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/




void init_tcx(PROGRAM_STATE_PTR  program_state)
{
  const char *tcxMachine = NULL;
  
  int i;

  TCX_REG_MSG_TYPE TCX_message_array[] = {
    BASE_messages,
    SONAR_messages,
    COLLI_messages,
    PLAN_messages,
#ifdef UNIBONN
    SPEECH_messages,
#endif
    MAP_messages
    };

  
  
  if (program_state->use_tcx){

    /* ====== INITIALIZING TCX ============ */
    tcxMachine = bRobot.TCXHOST;

    if (tcxMachine != NULL){
      
      if (program_state->tcx_initialized == 0){
	printf("Initializing TCX...");
	fflush(stdout);
	
	tcxInitialize(TCX_PLAN_MODULE_NAME, (char *) tcxMachine);
	check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
			     BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
			     NULL, 0);
	check_version_number(libezx_major, libezx_minor,
			     libezx_robot_type, libezx_date,
			     "libezx", 0);
	check_version_number(librobot_major, librobot_minor,
			     librobot_robot_type, librobot_date,
			     "librobot", 0);
	check_version_number(libbUtils_major, libbUtils_minor,
			     libbUtils_robot_type, libbUtils_date,
			     "libbUtils", 1);




      

	program_state->tcx_initialized = 1;
	printf("done.\n");
	
	
	
	/* -------- register messages and handler routines ---------- */
	
	
	tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
			    / sizeof(TCX_REG_MSG_TYPE));
	
	tcxRegisterHandlers(PLAN_handler_array,
			    sizeof(PLAN_handler_array)
			    / sizeof(TCX_REG_HND_TYPE));
	
	tcxRegisterHandlers(MAP_reply_handler_array,
			    sizeof(MAP_reply_handler_array)
			    / sizeof(TCX_REG_HND_TYPE));
	
	tcxRegisterHandlers(BASE_reply_handler_array,
			    sizeof(BASE_reply_handler_array)
			    / sizeof(TCX_REG_HND_TYPE));
	
	tcxRegisterHandlers(SONAR_reply_handler_array,
			    sizeof(SONAR_reply_handler_array)
			    / sizeof(TCX_REG_HND_TYPE));
	
	tcxRegisterCloseHnd(PLAN_close_handler);
      }
    }
    else{
      printf("WARNING: Environment variable TCXHOST not set. Connect failed.\n");
      program_state->use_tcx = 0;
    }
  }
}



/************************************************************************
 *
 *   NAME:         connect_to_MAP
 *                 
 *   FUNCTION:     checks, and connects to MAP, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



static struct timeval last_attempt_connect_MAP = {0, 0};

void connect_to_MAP(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		    PROGRAM_STATE_PTR program_state)
{
  MAP_register_auto_update_type subs;
  struct timeval current_time;

  if (!program_state->use_tcx)
    return;

  if (program_state->use_tcx && !program_state->tcx_initialized)
    /* ====== INITIALIZING TCX ============ */
    init_tcx(program_state);

  if(program_state->tcx_initialized && !program_state->map_connected){

    gettimeofday(&current_time, NULL);
    if (current_time.tv_sec < last_attempt_connect_MAP.tv_sec + 5
	|| (current_time.tv_sec == last_attempt_connect_MAP.tv_sec + 5 &&
	    current_time.tv_usec < last_attempt_connect_MAP.tv_usec))
      return;
    
    last_attempt_connect_MAP.tv_sec  = current_time.tv_sec;
    last_attempt_connect_MAP.tv_usec = current_time.tv_usec;
    

    MAP = tcxConnectOptional(TCX_MAP_MODULE_NAME); /* checks, but does 
						      * not wait */

    if (MAP != NULL){
      fprintf(stderr, "PLAN: connected to: %s\n", TCX_MAP_MODULE_NAME);
      subs.subscribe_maps                = 
	robot_specifications->map_update_frequency;
      subs.subscribe_position_correction = 1;
      tcxSendMsg(MAP, "MAP_register_auto_update", &subs);
      tcxSendMsg(MAP, "MAP_correction_parameters_query", NULL);
      G_display_switch(MAP_CONNECTED_BUTTON, 1);
      
      program_state->map_connected = 1;
    }
  }
}


#ifdef UNIBONN

/************************************************************************
 *
 *   NAME:         connect_to_SPEECH
 *                 
 *   FUNCTION:     checks, and connects to SPEECH, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



static struct timeval last_attempt_connect_SPEECH = {0, 0};

void connect_to_SPEECH(PROGRAM_STATE_PTR program_state)
{
  struct timeval current_time;

  if (!program_state->use_tcx)
    return;

  if (program_state->use_tcx && !program_state->tcx_initialized)
    /* ====== INITIALIZING TCX ============ */
    init_tcx(program_state);

  if (program_state->tcx_initialized && !program_state->speech_connected){

    gettimeofday(&current_time, NULL);
    if (current_time.tv_sec < last_attempt_connect_SPEECH.tv_sec + 5
	|| (current_time.tv_sec == last_attempt_connect_SPEECH.tv_sec + 5 &&
	    current_time.tv_usec < last_attempt_connect_SPEECH.tv_usec))
      return;
    
    last_attempt_connect_SPEECH.tv_sec  = current_time.tv_sec;
    last_attempt_connect_SPEECH.tv_usec = current_time.tv_usec;
    

    SPEECH = tcxConnectOptional(TCX_SPEECH_MODULE_NAME); /* checks, but does 
						      * not wait */

    if (SPEECH != NULL){
      fprintf(stderr, "PLAN: connected to: %s\n", TCX_SPEECH_MODULE_NAME);
      program_state->speech_connected = 1;
      tcx_speech_talk_text(program_state, "Planner ready.");
    }
  }
}
#endif


/************************************************************************
 *
 *   NAME:         connect_to_BASE
 *                 
 *   FUNCTION:     checks, and connects to BASE, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



static struct timeval last_attempt_connect_BASE = {0, 0};


void connect_to_BASE(PROGRAM_STATE_PTR program_state)
{
  BASE_register_auto_update_type data;
  struct timeval current_time;

  if (!program_state->use_tcx)
    return;

  if (program_state->use_tcx && !program_state->tcx_initialized)
    /* ====== INITIALIZING TCX ============ */
    init_tcx(program_state);

  if(program_state->tcx_initialized && !program_state->base_connected){

    gettimeofday(&current_time, NULL);
    if (current_time.tv_sec < last_attempt_connect_BASE.tv_sec + 5
	|| (current_time.tv_sec == last_attempt_connect_BASE.tv_sec + 5 &&
	    current_time.tv_usec < last_attempt_connect_BASE.tv_usec))
      return;
    
    last_attempt_connect_BASE.tv_sec  = current_time.tv_sec;
    last_attempt_connect_BASE.tv_usec = current_time.tv_usec;
    

    BASE = tcxConnectOptional(TCX_BASE_MODULE_NAME); /* checks, but does 
						      * not wait */

    if (BASE != NULL){
      fprintf(stderr, "PLAN: connected to: %s\n", TCX_BASE_MODULE_NAME);
      data.subscribe_status_report = 2;
      data.subscribe_sonar_report  = 0;
      data.subscribe_colli_report  = 0;
      data.subscribe_laser_report  = 0;
      data.subscribe_ir_report     = 0;

      tcxSendMsg(BASE, "BASE_register_auto_update", &data);
      G_display_switch(BASE_CONNECTED_BUTTON, 1);
      program_state->base_connected = 1;
    }
  }
}





  



/************************************************************************
 *
 *   NAME:         tcx_base_goto_relative
 *                 
 *   FUNCTION:     issues a motion command to the BASE
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void tcx_base_goto_relative(PROGRAM_STATE_PTR program_state,
			    ROBOT_SPECIFICATIONS_PTR robot_specifications,
			    ROBOT_STATE_PTR robot_state,
			    ACTION_PTR action)
{
  BASE_goto_relative_type tcx_coord;
  float target_pos_x, target_pos_y;
  
  


  target_pos_x = - action->base * sin((action->turn) / 180.0 * pi);
  target_pos_y =   action->base * cos((action->turn) / 180.0 * pi);

  if (PLAN_debug)
    fprintf(stderr, "TCX: base_goto_relative: %g %g (orient=%g turn=%g)\n",
	    target_pos_x, target_pos_y, 
	    robot_state->orientation, action->turn);
  
  if (!program_state->base_connected)
    connect_to_BASE(program_state);
  if (program_state->tcx_initialized && program_state->base_connected){
    tcx_coord.rel_target_x = target_pos_x;
    tcx_coord.rel_target_y = target_pos_y;
    fprintf(stderr, "BASE_goto_relative %g %g\n", target_pos_x, target_pos_y);
    tcxSendMsg(BASE, "BASE_goto_relative", &tcx_coord);
  }
}




/************************************************************************
 *
 *   NAME:         tcx_base_stop
 *                 
 *   FUNCTION:     issues a motion command to the BASE
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void tcx_base_stop(PROGRAM_STATE_PTR program_state)
{
  if (program_state->tcx_initialized && program_state->base_connected){
    if (PLAN_verbose)
      fprintf(stderr, "BASE_stop_robot\n");
    tcxSendMsg(BASE, "BASE_stop_robot", NULL);
  }    
}

/************************************************************************
 *
 *   NAME:         tcx_base_goto_absolute
 *                 
 *   FUNCTION:     issues a motion command to the BASE
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


static int mode4set = 0;

void tcx_base_goto_absolute(PROGRAM_STATE_PTR program_state,
			    ROBOT_SPECIFICATIONS_PTR robot_specifications,
			    ROBOT_STATE_PTR robot_state,
			    ACTION_PTR action,
			    int new_target)
{
  BASE_approach_absolute_two_points_type tcx_coord;
  float target_pos_x1, target_pos_y1;
  float target_pos_x2, target_pos_y2;
  float uncorr_target_pos_x1, uncorr_target_pos_y1, uncorr_target_pos_o1;
  float uncorr_target_pos_x2, uncorr_target_pos_y2, uncorr_target_pos_o2;
  int m = APPROACH_OBJECT_MODE;
  
  /* The absolute position is computed in generate_action and stored in
   * action->absolute_x,action->absolute_y. */
  
  target_pos_x1 = action->absolute_x;
  target_pos_y1 = action->absolute_y;
  target_pos_x2 = action->potential_next_absolute_x;
  target_pos_y2 = action->potential_next_absolute_y;


  
  if ( PLAN_debug) 
    fprintf(stderr, "TCX: base_goto_absolute: %g %g (orient=%g turn=%g corr=%g,%g,%g,%d)\n",
            target_pos_x1, target_pos_y1, 
	    robot_state->orientation, robot_state->orientation + action->turn,
	    robot_state->correction_parameter_x,
	    robot_state->correction_parameter_y,
	    robot_state->correction_parameter_angle,
	    robot_state->correction_type);
  
  
  
  
  if (program_state->tcx_initialized && program_state->base_connected){
  
    compute_backward_correction(target_pos_x1, target_pos_y1, 0.0,
				robot_state->correction_parameter_x,
				robot_state->correction_parameter_y,
				robot_state->correction_parameter_angle,
				robot_state->correction_type,
				&uncorr_target_pos_x1, &uncorr_target_pos_y1, 
				&uncorr_target_pos_o1);

    compute_backward_correction(target_pos_x2, target_pos_y2, 0.0,
				robot_state->correction_parameter_x,
				robot_state->correction_parameter_y,
				robot_state->correction_parameter_angle,
				robot_state->correction_type,
				&uncorr_target_pos_x2, &uncorr_target_pos_y2, 
				&uncorr_target_pos_o2);
    

    tcx_coord.new_target = new_target;

    tcx_coord.abs_target_x1   = uncorr_target_pos_x1;
    tcx_coord.abs_target_y1   = uncorr_target_pos_y1;
    tcx_coord.abs_target_x2   = uncorr_target_pos_x2;
    tcx_coord.abs_target_y2   = uncorr_target_pos_y2;

    if (action->final_action ||
	(tcx_coord.abs_target_x1 == tcx_coord.abs_target_x2 &&
	 tcx_coord.abs_target_y1 == tcx_coord.abs_target_y2)){
      tcx_coord.approach_dist   = 
	robot_specifications->max_final_approach_distance;
      /*       fprintf( stderr, "final goal\n"); */
      /*       fprintf(stderr, "\n\n\t@@@@@@@@\n\n"); */
      /*       putc(7, stderr); */
    }
    else
      tcx_coord.approach_dist   =
	robot_specifications->max_approach_distance;
    
    
    if (!program_state->base_connected)
      connect_to_BASE(program_state);

    if (program_state->base_connected){
      /*! hack for AAAI
	if (action->final_action && !mode4set && program_state->goal_modus){
	tcxSendMsg(BASE, "BASE_setmode", &m);
	fprintf(stderr, "\n\t######## mode 4 ###########\n");
	mode4set = 1;
	}
	else if (!action->final_action)
	mode4set = 0;
	*/
      if (PLAN_debug)
	fprintf(stderr, "BASE_approach_absolute_two_points %g %g  %g %g  %g\n",
		uncorr_target_pos_x1, uncorr_target_pos_y1,
		uncorr_target_pos_x2, uncorr_target_pos_y2,
		tcx_coord.approach_dist);
      
      tcxSendMsg(BASE, "BASE_approach_absolute_two_points", &tcx_coord);
    }
  }
}





/************************************************************************
 *
 *   NAME:         PLAN_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void PLAN_close_handler(char *name, TCX_MODULE_PTR module)
{
  PROGRAM_STATE_PTR program_state = all->program_state;

  /*program_state->busy++;*/

  remove_auto_update_module(module);

  if (PLAN_debug) fprintf(stderr, 
			  "PLAN: closed connection detected: %s\n", name);

  if (!strcmp(name, TCX_BASE_MODULE_NAME)){ /* BASE shut down */
    program_state->base_connected = 0;
    program_state->autonomous = 0;
    G_display_switch(AUTONOMOUS_BUTTON, program_state->autonomous);
    G_display_switch(BASE_CONNECTED_BUTTON, 0);
    /*
       program_state->exploration = 0;
       G_display_switch(EXPLORATION_BUTTON, program_state->exploration);*/

  }
  else if (!strcmp(name, TCX_MAP_MODULE_NAME)){ /* MAP shut down */
    program_state->map_connected = 0;
    G_display_switch(MAP_CONNECTED_BUTTON, 0);
  }
  else
    if (!strcmp(name, "TCX Server")) /* TCX shut down */
    exit(0);
}
  





