
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

#include <bUtils.h>

#include "tcx.h"
#include "o-graphics.h"
#include "bUtils.h"

#define pi  3.14159265358979323846
#define sqrt2 1.41436
#define MAX_RANDOM 1073741823.5
#define RAND() ((((float) random())/MAX_RANDOM)-1.0)
#define RAND_POS() ((((float) random())/MAX_RANDOM)*0.5)




#include "PLAN-messages.h"
#include "COLLI-messages.h"
#include "MAP-messages.h"
#include "BASE-messages.h"
#include "SONAR-messages.h"

#include "PLAN.h"

struct bParamList * bParamList = NULL;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/*---- check_consistency - if you change anything in the DP process
 *---- this flag allows you to do regular consitency checks! */

#define CHECK_CONSISTENCY 0


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



#define MAX_NUM_PLAN_STEPS  (robot_specifications->global_map_dim_x + robot_specifications->global_map_dim_y)



struct timeval TCX_waiting_time = {0, 0};

struct timeval block_waiting_time = {1, 0};

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


ALL_PTR all;			/* this is now global */



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

int     nNUMBER   = 1;
int     NUMBER    = 0;
int BEST_NUMBER   = 0;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


int block_wait(struct timeval *timeout, int tcx_initialized,
	       int X_initialized);


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



/* MAPS */


int maps_allocated = 0;

float *global_values     = NULL; /* occupancy likelihoods */
int   *global_active     = NULL; /* 1 if defined, 0 if not */
int   *global_visited    = NULL;
int   *global_explored   = NULL;
int   *global_succ       = NULL; /* saves the update-state */
float *global_costs      = NULL;
float *global_costs2     = NULL;
int   *global_goal       = NULL;
float *global_utility    = NULL;
/*int   *global_interior   = NULL;*/

int   *global_goal_table[max_nNUMBER];
float *global_utility_table[max_nNUMBER];
int   *global_succ_table[max_nNUMBER];








/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




/************************************************************************
 *
 *   NAME:         init_program
 *                 
 *   FUNCTION:     Initializes the structure "program_state"
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure 
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *                 ROBOT_STATE_PTR robot_state       pointer to robot state
 *                 ACTION_PTR      action            pointer to action
 *                 ALL_PTR            structure to all variables
 *                 
 *                 
 *                 NOTE: "NULL" disables initialization
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void init_program(PROGRAM_STATE_PTR        program_state,
		  ROBOT_SPECIFICATIONS_PTR robot_specifications,
		  ROBOT_STATE_PTR          robot_state,
		  ACTION_PTR               action,
		  ALL_PTR                  all)
{
  int i, size, k;
  
  nNUMBER = 1;

  
  if (program_state != NULL){
    /* program_state->use_tcx                = 1; is now command line option */
    /* program_state->use_graphics           = 1; is now command line option */
    program_state->tcx_initialized           = 0;
    program_state->base_connected            = 0;
    program_state->map_connected             = 0;
    program_state->speech_connected          = 0;
    program_state->graphics_initialized      = 0;
    program_state->program_initialized       = 0;
    program_state->warmer_colder_game        = 0;
    program_state->goal_modus                = 0;
    program_state->autonomous                = 0; /* 1, then the robot goes! */
    program_state->exploration               = 0; /* 1, then the robot goes! */
    program_state->quit                      = 0;
    program_state->target_not_reached        = 0;
    program_state->actual_map_number         = 0;
    program_state->reset_descending_utilities_flag = 0;
    program_state->send_automatic_update     = 0;
    program_state->busy                      = 0;
    program_state->bounding_box_defined      = 0;
    program_state->bounding_box_min_x        = 0.0;
    program_state->bounding_box_max_x        = 0.0;
    program_state->bounding_box_min_y        = 0.0;
    program_state->bounding_box_max_y        = 0.0;
    program_state->interior_mode             = 0;
  }


  if (robot_state != NULL){

    robot_state->correction_parameter_x      = 0.0;
    robot_state->correction_parameter_y      = 0.0;
    robot_state->correction_parameter_angle  = 0.0;
    robot_state->correction_type             = 0;

    robot_state->x                           = 0.0;
    robot_state->y                           = 0.0;
    robot_state->orientation                 = 0.0;
    robot_state->stuck                       = 0;

    robot_state->map_orientation_defined     = 0;
    robot_state->map_orientation             = 0.0;
  }


  if (action != NULL){
    action->base = 0.0;
    action->turn = 0.0;
    action->absolute_x = 0.0;
    action->absolute_y = 0.0;
    action->success = 0;
    action->no_plan = 1;
    action->active_goal_name = -1;
    action->goal_x = 0.0;
    action->goal_y = 0.0;
    action->final_action = 0;
    action->goal_dist = 0.0;
  }

  if (robot_specifications != NULL){
    
    robot_specifications->global_mapsize_x = 3000.0;
    robot_specifications->global_mapsize_y = 3000.0;
    robot_specifications->resolution       = 10.0;
    robot_specifications->robot_size       = 30.0;
    robot_specifications->autoshift        = 1;
    robot_specifications->autoshift_safety_margin = 50.0;
    robot_specifications->autoshift_distance = 400.0;

    robot_specifications->max_security_dist = 40.0;
    robot_specifications->min_base = 60.0;
    robot_specifications->max_base = 99999999.9;
    robot_specifications->max_adjust_angle = 100.0;  
    robot_specifications->max_goal_distance  = 30.0;
    robot_specifications->max_approach_distance = 27.0;
    robot_specifications->max_final_approach_distance = 27.0;
      

    robot_specifications->autoshifted_x             = 0.0;
    robot_specifications->autoshifted_y             = 0.0;
    robot_specifications->autoshifted_int_x         = 0;
    robot_specifications->autoshifted_int_y         = 0;
    
    
    robot_specifications->min_costs                = 0.0;
    robot_specifications->max_costs                = 0.0;
    robot_specifications->dp_direction             = 0;
    for (i = 0; i < nNUMBER; i++)
      robot_specifications->number_active_goals[i]  = 0;
    
    robot_specifications->average_value             = 0.7; /* Prior belief
							    * for each value */
    robot_specifications->costs_exponent            = 0.4; /* exponent for
							    * computing costs
							    * from values */
    robot_specifications->collision_threshold       = 0.5; /* map values lower
							    * than this point 
							    * considered 
							    * dangerous */
    robot_specifications->map_update_frequency      = 2;

    robot_specifications->exploration_circle_size   = 250.0;
    robot_specifications->min_mapvalue_along_path   = 0.0;

    robot_specifications->border_to_interior        = 200.0;
    
    robot_specifications->exterior_factor           = 0.1;
    robot_specifications->max_bounding_box_size     = 900000000.0;

    robot_specifications->fast_exploration          = 1;

    robot_specifications->generate_actions_continuously = 1;

    robot_specifications->X_window_size                    = 80.0;

  }


  
  program_state->program_initialized = 1;
  
  
  

  all->program_state        = program_state;
  all->robot_specifications = robot_specifications;
  all->robot_state          = robot_state;
  all->action               = action;

  printf("PLAN INITIALIZED.\n");


}




/************************************************************************
 *
 *   NAME:         allocate_maps()
 *                 
 *   FUNCTION:     Allocates maps and stuff
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void allocate_maps(ROBOT_STATE_PTR          robot_state,
		   PROGRAM_STATE_PTR program_state, 
		   ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int i, k, size;
  
  robot_state->x = 0.5 * robot_specifications->global_mapsize_x;
  robot_state->y = 0.5 * robot_specifications->global_mapsize_y;
  robot_state->orientation  = 0.0;

  robot_specifications->average_costs             = 
    pow(robot_specifications->average_value, 
	robot_specifications->costs_exponent);
  robot_specifications->average_costs2            = 
    pow(robot_specifications->average_costs, sqrt2);
  
    robot_specifications->global_map_dim_x = (int)
      (robot_specifications->global_mapsize_x
       / robot_specifications->resolution);


    robot_specifications->global_map_dim_y = (int)
      (robot_specifications->global_mapsize_y
       / robot_specifications->resolution);


    
    robot_specifications->min_plan_index_x     = 
      robot_specifications->global_map_dim_x + 1;
    robot_specifications->max_plan_index_x     = -1;
    robot_specifications->min_plan_index_y     = 
      robot_specifications->global_map_dim_x + 1;
    robot_specifications->max_plan_index_y     =  -1;
    robot_specifications->min_display_index_x  = 
      robot_specifications->global_map_dim_x + 1;
    robot_specifications->max_display_index_x  =  -1;
    robot_specifications->min_display_index_y  = 
      robot_specifications->global_map_dim_x + 1;
    robot_specifications->max_display_index_y  =  -1;


  /* IF MAPS ALREADY ALLOCATED - FREE THE MEMORY  (currently commented out)*/
  if (maps_allocated){
    free(global_values);
    free(global_active);
    free(global_visited);
    free(global_costs);
    free(global_costs2);
    /*    free(global_interior);*/
    for (k = 0; k < nNUMBER; k++){
      free(global_goal_table[k]);
      free(global_utility_table[k]);
      free(global_succ_table[k]);
    }
    maps_allocated = 0;
  }


  /* ALLOCATE NEW MEMORY FOR THE MAPS */

  size = robot_specifications->global_map_dim_x
    * robot_specifications->global_map_dim_y;

  printf("Allocate %d parallel plans, size %d\n", nNUMBER, size);
  
  if (!maps_allocated){
    global_values     = (float *) (malloc(sizeof(float) * size));
    global_active     = (int *)   (malloc(sizeof(int)   * size));
    global_visited    = (int *)   (malloc(sizeof(int)   * size));
    global_costs      = (float *) (malloc(sizeof(float) * size));
    global_costs2     = (float *) (malloc(sizeof(float) * size));
    /*global_interior   = (int *)   (malloc(sizeof(int) * size));*/
    if (global_values == NULL || global_active == NULL || 
	global_costs == NULL || global_costs2 == NULL ||
	/* global_interior == NULL || */ global_visited == NULL){
      printf("ABORT: out of memory (1)!\n");
      exit(1);
    }
    
    for (k = 0; k < nNUMBER; k++){
      global_utility_table[k] = (float *) (malloc(sizeof(float) * size));
      global_goal_table[k] =    (int *)   (malloc(sizeof(int)   * size));
      global_succ_table[k] =    (int *)   (malloc(sizeof(int)   * size));
      if (global_utility_table[k] == NULL || global_succ_table[k] == NULL ||
	  global_goal_table[k] == NULL){
	printf("ABORT: out of memory (2-%d)!\n", k);
	exit(1);
      }
    }
    global_utility = &(global_utility_table[0][0]);
    global_goal    = &(global_goal_table[0][0]);
    global_succ    = &(global_succ_table[0][0]);
    maps_allocated = 1;
  }

  /* INITIALIZE MAPS */

  for (i = 0; i < size; i++){
    global_values[i]     = 0.0;
    global_active[i]     = 0;
    global_visited[i]    = 0;
    global_costs[i]      = robot_specifications->average_costs;
    global_costs2[i]     = robot_specifications->average_costs2;
    /* global_interior[i]   = 0;*/
  }

  for (k = 0; k < nNUMBER; k++){
    global_utility = global_utility_table[k];
    global_goal    = global_goal_table[k];
    global_succ    = global_succ_table[k];
    for (i = 0; i < size; i++){
      global_utility[i] = 0.0;
      global_goal[i]    = 0;
      global_succ[i]    = -1;
    }
  }
  
  global_utility = &(global_utility_table[0][0]);
  global_goal    = &(global_goal_table[0][0]);
  global_succ    = &(global_succ_table[0][0]);


  global_explored = global_active; /* just a pointer */
}




/************************************************************************
 *
 *   NAME:         reset_internal_exploration_table
 *                 
 *   FUNCTION:     reinitializes global_visited
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void reset_internal_exploration_table(PROGRAM_STATE_PTR program_state, 
				      ROBOT_SPECIFICATIONS_PTR 
				      robot_specifications)
{
  int i;


  for (i = 0; i < robot_specifications->global_map_dim_x
       * robot_specifications->global_map_dim_y; i++){
    global_explored[i] = 0;
  }
}



/************************************************************************
 *
 *   NAME:         check_commandline_parameters
 *                 
 *   FUNCTION:     checks the command line options!
 *                 
 *   PARAMETERS:   
 *                 
 *                 NOTE: "NULL" disables initialization
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void check_commandline_parameters(int argc, char **argv, 
				  PROGRAM_STATE_PTR program_state)
{
  int i, bug = 0;

  program_state->use_tcx                   = 1;	/* default value */
  program_state->use_map                   = 1;	/* default value */
  program_state->use_graphics              = 1;	/* default value */

  for (i = 1; i < argc; i++){
    if (!strcmp(argv[i], "-notcx") || !strcmp(argv[i], "-nt"))
      program_state->use_tcx = 0;
    else if (!strcmp(argv[i], "-nodisplay") || !strcmp(argv[i], "-nd"))
      program_state->use_graphics = 0; 
    else if (!strcmp(argv[i], "-nomap"))
      program_state->use_map = 0; 
    else
      bug = 1;
  }

  if (bug){
    fprintf(stderr, "\nUsage: '%s [-notcx] [-nodisplay]\n", argv[0]);

    exit(1);

  }
}


/************************************************************************
 *
 *   NAME:         
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *                 
 *                 
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void get_global_range(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		 float *global_array,
		 int   *global_active,
		 float *min_value, 
		 float *max_value)
{
  int i, n;

  *min_value =  9999999999999.9;
  *min_value = -9999999999999.9;
  n = robot_specifications->global_map_dim_x * 
    robot_specifications->global_map_dim_y;
  fflush(stdout);

  if (global_active == NULL)
    for (i = 0; i < n; i++){
      if (global_array[i] < *min_value)
	*min_value = global_array[i];
      if (global_array[i] > *max_value)
	*max_value = global_array[i];
    }
  else
    for (i = 0; i < n; i++){
      if (global_active[i]){
	if (global_array[i] < *min_value)
	  *min_value = global_array[i];
	if (global_array[i] > *max_value)
	  *max_value = global_array[i];
      }
    }
  fflush(stdout);
  fprintf(stderr, "buggy");
}



/************************************************************************
 *
 *   NAME:         
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *                 
 *                 
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/




void check_index(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	    int copy_flag)
{
  /* copy plan_index to display_index */
  if (copy_flag){
    if (robot_specifications->min_display_index_x < 
	robot_specifications->min_plan_index_x)
      robot_specifications->min_plan_index_x =
	robot_specifications->min_display_index_x;
    if (robot_specifications->min_display_index_y < 
	robot_specifications->min_plan_index_y)
      robot_specifications->min_plan_index_y =
	robot_specifications->min_display_index_y;
    if (robot_specifications->max_display_index_x >
	robot_specifications->max_plan_index_x)
      robot_specifications->max_plan_index_x =
	robot_specifications->max_display_index_x;
    if (robot_specifications->max_display_index_y > 
	robot_specifications->max_plan_index_y)
      robot_specifications->max_plan_index_y =
	robot_specifications->max_display_index_y;
  }

  /* cut plan_index */
  if (robot_specifications->min_plan_index_x < 1)
    robot_specifications->min_plan_index_x = 1;
  if (robot_specifications->min_plan_index_y < 1)
    robot_specifications->min_plan_index_y = 1;
  if (robot_specifications->max_plan_index_x > 
      robot_specifications->global_map_dim_x - 1)
    robot_specifications->max_plan_index_x = 
      robot_specifications->global_map_dim_x - 1;
  if (robot_specifications->max_plan_index_y > 
      robot_specifications->global_map_dim_y - 1)
    robot_specifications->max_plan_index_y = 
      robot_specifications->global_map_dim_y - 1;

  /* cut display_index */
  if (robot_specifications->min_display_index_x < 0)
    robot_specifications->min_display_index_x = 0;
  if (robot_specifications->min_display_index_y < 0)
    robot_specifications->min_display_index_y = 0;
  if (robot_specifications->max_display_index_x > 
      robot_specifications->global_map_dim_x)
    robot_specifications->max_display_index_x = 
      robot_specifications->global_map_dim_x;
  if (robot_specifications->max_display_index_y >
      robot_specifications->global_map_dim_y)
    robot_specifications->max_display_index_y = 
      robot_specifications->global_map_dim_y;
}


/************************************************************************
 *
 *   NAME:         search_for_inconsistencies_in_succ
 *                 
 *   FUNCTION:     a pure testing function for detecting complex pointer bugs
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

static int print_message = 0;

int search_for_inconsistencies_in_succ(ROBOT_SPECIFICATIONS_PTR 
				       robot_specifications,
				       char *text)
{
  int x, y, index, succ_index, num_errors;

  num_errors = 0;
  if (CHECK_CONSISTENCY){

    if (!print_message){
      fprintf(stderr, "### Cheching mode is ON - uses up *your* time! ###\n");
      print_message = 1;
    }
    
    for (x = 0; x < robot_specifications->global_map_dim_x; x++)
      for (y = 0; y < robot_specifications->global_map_dim_y; y++){
	
	index = x * robot_specifications->global_map_dim_y + y;
	
	succ_index = global_succ[index];
	if (succ_index >= 0){	/* there is a successor! */
	  if (global_utility[succ_index] < global_utility[index])
	    num_errors++;		/* mon-monotonic: must be a bug! */
	  if (global_succ[succ_index] == index)
	    num_errors++;		/* loop: must be a bug! */
	}
      }
    if (num_errors){
      EZX_bell();
      if (text == NULL)
	fprintf(stderr, "WARNING: %d inconsistent succ-values found.\n",
		num_errors);
      else
	fprintf(stderr, "WARNING(%s): %d inconsistent succ-values found.\n",
		text, num_errors);
    }
  }
  return (num_errors);
}



/************************************************************************
 *
 *   NAME:         check_if_inside_interior
 *                 
 *   FUNCTION:     checks, if a point is in the interior of a map
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *                 
 *                 
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



int check_if_inside_interior(int x, int y, 
			     ROBOT_SPECIFICATIONS_PTR robot_specifications,
			     ROBOT_STATE_PTR robot_state,
			     PROGRAM_STATE_PTR program_state)
{
  float angle;
  float x_rotated, y_rotated;
  float x_float, y_float;



  if (!program_state->bounding_box_defined)
    return 0;


  x_float = 
    (((float) (x - robot_specifications->autoshifted_int_x))
     * robot_specifications->resolution);
  y_float = 
    (((float) (y - robot_specifications->autoshifted_int_y))
     * robot_specifications->resolution);
  



  angle = robot_state->map_orientation * M_PI / 180.0;
  x_rotated = (cos(-angle) * x_float) - (sin(-angle) * y_float);
  y_rotated = (cos(-angle) * y_float) + (sin(-angle) * x_float);
  if (x_rotated >= program_state->bounding_box_min_x &&
      x_rotated <= program_state->bounding_box_max_x &&
      y_rotated >= program_state->bounding_box_min_y &&
      y_rotated <= program_state->bounding_box_max_y)
    return 1;
  else
    return 0;
}









float exploration_utility_factor(int x, int y, 
				 ROBOT_SPECIFICATIONS_PTR robot_specifications,
				 ROBOT_STATE_PTR robot_state,
				 PROGRAM_STATE_PTR program_state)
{
  if (!program_state->interior_mode)
    return 1.0;

  else if (check_if_inside_interior(x, y, robot_specifications,
				    robot_state, program_state))
    return 1.0;

  else
    return robot_specifications->exterior_factor;
}
      


/************************************************************************
 *
 *   NAME:         
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *                 
 *                 
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void reset_descending_utilities(ROBOT_SPECIFICATIONS_PTR robot_specifications,
				PROGRAM_STATE_PTR program_state,
				ROBOT_STATE_PTR robot_state)
{
  int from_x, to_x, increment_x, from_y, to_y, increment_y;
  register int  x, y, index;
  float new_utility;
  register float help;
  int min_plan_index_x, max_plan_index_x, min_plan_index_y, max_plan_index_y;
  int new_min_plan_index_x, new_max_plan_index_x;
  int new_min_plan_index_y, new_max_plan_index_y;
  int dp_direction, done;
  register float new_utility2;
  register int new_utility_index, new_utility2_index;

  if (PLAN_verbose){
    printf("{");
    fflush(stdout);
  }
  /* copy current planning border  */
  min_plan_index_x = robot_specifications->min_plan_index_x;
  max_plan_index_x = robot_specifications->max_plan_index_x;
  min_plan_index_y = robot_specifications->min_plan_index_y;
  max_plan_index_y = robot_specifications->max_plan_index_y;
  dp_direction = 0;
  done = 0;
  
  
  do{
    if (min_plan_index_x < max_plan_index_x &&
	min_plan_index_y < max_plan_index_y){
      
      if (dp_direction == 0 || dp_direction == 2){
	from_x      = min_plan_index_x;
	to_x        = max_plan_index_x;
	increment_x = 1;
      }
      else{
	from_x      = max_plan_index_x -1;
	to_x        = min_plan_index_x -1;
	increment_x = -1;
      }
      if (dp_direction == 1 || dp_direction == 2){
	from_y      = min_plan_index_y;
	to_y        = max_plan_index_y;
	increment_y = 1;
      }
      else{
	from_y      = max_plan_index_y -1;
	to_y        = min_plan_index_y -1;
	increment_y = -1;
      }
      dp_direction = 
	(dp_direction + 1) % 4;
      
      new_min_plan_index_x = max_plan_index_x + 1;
      new_max_plan_index_x = -1;
      new_min_plan_index_y = max_plan_index_y + 1;
      new_max_plan_index_y = -1;
      
      
      /* ********** MAIN DYNAMIC PROGRAMMING RESET LOOP ********** */
      for (x = from_x; x != to_x; x += increment_x){
	index = x * robot_specifications->global_map_dim_y + from_y;
	for (y = from_y; y != to_y; y += increment_y, index += increment_y){
	  if (global_utility[index] > 0.0){
	    if (program_state->actual_map_number == 0 && global_goal[index]){
	      new_utility = 1.0;
	      new_utility2 = 1.0;
	      new_utility_index  = -1;
	      new_utility2_index = -1;
	    }
	    else if ((program_state->actual_map_number != 0 ||
		      robot_specifications->number_active_goals[NUMBER] == 0)
		     && !global_explored[index]){
	      new_utility = exploration_utility_factor(x, y, 
						       robot_specifications,
						       robot_state,
						       program_state); 
	      new_utility2 = exploration_utility_factor(x, y, 
							robot_specifications,
							robot_state,
							program_state); 
	      new_utility_index  = -1;
	      new_utility2_index = -1;
	    }

	    else{			/* do the DP here! */
	      new_utility = global_utility[index-1]; /* field on the left */
	      new_utility_index = index-1;
	      
	      help = global_utility[index+1];
	      if (help > new_utility){
		new_utility = help; /* right */
		new_utility_index = index+1;
	      }

	      help = 
		global_utility[index-robot_specifications->global_map_dim_y];
	      if (help > new_utility){	/* below */
		new_utility = help;
		new_utility_index = 
		  index-robot_specifications->global_map_dim_y;
	      }

	      help = 
		global_utility[index+robot_specifications->global_map_dim_y];
		  if (help > new_utility){	/* above */
		new_utility = help;
		new_utility_index = 
		  index+robot_specifications->global_map_dim_y;
	      }

	      new_utility2 =	/* below left */
		global_utility[index 
			       - robot_specifications->global_map_dim_y - 1];
	      new_utility2_index = index 
		- robot_specifications->global_map_dim_y - 1;

	      help = global_utility[index
				    - robot_specifications->global_map_dim_y
				    + 1];
	      if (help > new_utility2){	/* below right */
		new_utility2 = help;
		new_utility2_index = index
		  - robot_specifications->global_map_dim_y + 1;
	      }

	      help = global_utility[index + 
				    robot_specifications->global_map_dim_y -1];
	      if (help > new_utility2){	/* above left */
		new_utility2 = help;
		new_utility2_index = index + 
		  robot_specifications->global_map_dim_y - 1;
	      }

	      help = global_utility[index + 
				    robot_specifications->global_map_dim_y +1];
	      if (help > new_utility2){	/* above right */
		new_utility2 = help;
		new_utility2_index = index + 
		  robot_specifications->global_map_dim_y + 1;
	      }
	    }


	    new_utility *= global_costs[index];
	    new_utility2 *= global_costs2[index];
	    if (new_utility2 > new_utility){
	      new_utility = new_utility2;
	      new_utility_index = new_utility2_index;
	    }
	    if (global_utility[index] > new_utility || /* this is an indication
							* that value will 
							* have to decrease */
		(global_utility[index] == new_utility && /* value stays the 
							  * same, but */
		 global_utility[index] == /* ...but there might be a stable
					   * loop due to zero-costs. Let's
					   * be suspicious and decrease that
					   * value anyhow. */
		 global_utility[global_succ[index]])){ 
	      global_utility[index] = 0.0; /* reinitialize the value */
	      global_succ[index]    = -1;
	      if (x < new_min_plan_index_x) new_min_plan_index_x = x;
	      if (x > new_max_plan_index_x) new_max_plan_index_x = x;
	      if (y < new_min_plan_index_y) new_min_plan_index_y = y;
	      if (y > new_max_plan_index_y) new_max_plan_index_y = y;
	    }
	    else		/* this will make sure that even
				 * with changing costs functions the succ
				 * table will stay consistent*/
	      global_succ[index] = new_utility_index;
	      

	  }
	}
      }
      
      /* ======= update planning bounding interval =========== */
	new_min_plan_index_x -= 1;
      new_max_plan_index_x += 2;
      new_min_plan_index_y -= 1;
      new_max_plan_index_y += 2;
      if (new_min_plan_index_x < robot_specifications->min_display_index_x)
	new_min_plan_index_x = robot_specifications->min_display_index_x;
      if (new_max_plan_index_x > robot_specifications->max_display_index_x)
	new_max_plan_index_x = robot_specifications->max_display_index_x;
      if (new_min_plan_index_y < robot_specifications->min_display_index_y)
	new_min_plan_index_y = robot_specifications->min_display_index_y;
      if (new_max_plan_index_y > robot_specifications->max_display_index_y)
	new_max_plan_index_y = robot_specifications->max_display_index_y;
      if (new_min_plan_index_x < 1)
	new_min_plan_index_x = 1;
      if (new_max_plan_index_x > robot_specifications->global_map_dim_x - 1)
	new_max_plan_index_x = robot_specifications->global_map_dim_x - 1;
      if (new_min_plan_index_y < 1)
	new_min_plan_index_y = 1;
      if (new_max_plan_index_y > robot_specifications->global_map_dim_y - 1)
	new_max_plan_index_y = robot_specifications->global_map_dim_y - 1;
      min_plan_index_x = new_min_plan_index_x;
      max_plan_index_x = new_max_plan_index_x;
      min_plan_index_y = new_min_plan_index_y;
      max_plan_index_y = new_max_plan_index_y;
    }
    else
      done = 1;
  }
  while (!done);
  if (PLAN_verbose){
    printf("}");
    fflush(stdout);
  }

  search_for_inconsistencies_in_succ(robot_specifications, "reset DP");

  program_state->reset_descending_utilities_flag = 0;
/*

G_activate(UTILITY);
G_display_partial_matrix(UTILITY, 
robot_specifications->min_display_index_x,
robot_specifications->max_display_index_x -
robot_specifications->min_display_index_x,
robot_specifications->min_display_index_y,
robot_specifications->max_display_index_y -
robot_specifications->min_display_index_y);
G_deactivate(UTILITY);
*/
}
  




/************************************************************************
 *
 *   NAME:         fake_map_update
 *                 
 *   FUNCTION:     Fakes a map update, just for testing
 *                 
 *   PARAMETERS:   ROBOT_STATE_PTR    robot_state    Pointer to robot state
 *                                                   description structure
 *                 PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure 
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *                 
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


#define MIN_COSTS_IN_FAKE_MAPS 0.01


void fake_map_update(ROBOT_STATE_PTR robot_state,
		     PROGRAM_STATE_PTR  program_state,
		     ROBOT_SPECIFICATIONS_PTR robot_specifications,
		     float robot_x, float robot_y)
{
  int x, y, global_x, global_y, i, j;
  int global_map_index, global_map_index2, count;
  float point_costs, costs_factor, cum_neighbors;
  int   robot_int_x, robot_int_y;
  int    first_x;
  int    first_y;
  int    size_x;
  int    size_y;

  robot_int_x = ((int) (robot_x / robot_specifications->resolution));
  robot_int_y = ((int) (robot_y / robot_specifications->resolution));
  first_x = robot_int_x - 30;
  first_y = robot_int_y - 30;
  size_x  = 60;
  size_y  = 60;


  for (x = 0, global_x = x + first_x 
       + robot_specifications->autoshifted_int_x;
       x < size_x; x++, global_x++)
    for (y = 0, global_y = y + first_y 
	 + robot_specifications->autoshifted_int_y;
	 y < size_y; y++, global_y++){
      global_map_index  = global_x * 
	robot_specifications->global_map_dim_y + global_y;
      if (global_x >= 0 && global_y >= 0 &&
	  global_x < robot_specifications->global_map_dim_x &&
	  global_y < robot_specifications->global_map_dim_y){
	global_active[global_map_index] = 1;
	if (global_active[global_map_index]){
	  
	  /* ====== set value =========== */
	  cum_neighbors = 0.0;
	  count = 0;
	  for (i = -1; i <= 1; i++)
	    for (j = -1; j <= 1; j++){
	      global_map_index2  = (global_x + i) * 
		robot_specifications->global_map_dim_y + (global_y + j);
	      if ((i != 0 || j != 0) && global_active[global_map_index2]){
		cum_neighbors += global_values[global_map_index2];
		count++;
	      }
	    }
	  if (count == 0){
	    global_values[global_map_index] = RAND_POS() 
	      * (1.0 - MIN_COSTS_IN_FAKE_MAPS);
	  }
	  else{
	    global_values[global_map_index] = 0.1 * RAND()
	      + (cum_neighbors / ((float) count));
	    if (global_values[global_map_index] < 0.0)
	      global_values[global_map_index] = 0.0;
	    if (global_values[global_map_index] > 1.0 - MIN_COSTS_IN_FAKE_MAPS)
	      global_values[global_map_index] = 1.0 - MIN_COSTS_IN_FAKE_MAPS;
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
			     program_state->actual_map_number == 0 && 
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
  check_index(robot_specifications, 1);
  reset_descending_utilities(robot_specifications, program_state,
			     robot_state);    
  search_for_inconsistencies_in_succ(robot_specifications, "fake map update");
}



       


/************************************************************************
 *
 *   NAME:         costs_function
 *                 
 *   FUNCTION:     calculates a cost value from a map value
 *                 
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *                 
 *                 
 *                 
 *                 
 *   RETURN-VALUE: float costs     - cost vlaue
 *                 
 ************************************************************************/

float costs_function(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		     float value)
{
  float costs, costs_factor;

  if (value < 0.1)
    costs = 0.01;
  else
    if (value > 0.6)
      costs = pow(0.98, robot_specifications->costs_exponent);
  else
    costs = pow(value, robot_specifications->costs_exponent);
  /*
    if (confidence < robot_specifications->confidence_when_explored){
    costs_factor = sqrt(confidence / 
    robot_specifications->confidence_when_explored);
    costs = costs_factor * costs + (1.0 - costs_factor);
    }
    */
  if (robot_specifications->min_costs > costs)
    robot_specifications->min_costs = costs;
  if (robot_specifications->max_costs < costs)
    robot_specifications->max_costs = costs;

  return costs;
}	  
     


/************************************************************************
 *
 *   NAME:         utility_function
 *                 
 *   FUNCTION:     calculates a cost value from a map value
 *                 
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *                 
 *                 
 *                 
 *                 
 *   RETURN-VALUE: float utility     - utility vlaue
 *                 
 ************************************************************************/

float utility_function(float costs, int goal)
{
  float utility;

  if (goal)
    return costs;
  else
    return 0.0;
}	  

  


/************************************************************************
 *
 *   NAME:         dynamic_programming
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void dynamic_programming(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			 PROGRAM_STATE_PTR program_state,
			 ROBOT_STATE_PTR robot_state)
{
  int from_x, to_x, increment_x, from_y, to_y, increment_y;
  register int  x, y, index;
  register float help, new_utility;
  int new_min_plan_index_x, new_max_plan_index_x;
  int new_min_plan_index_y, new_max_plan_index_y;
  register float  new_utility2;
  register int new_utility_index, new_utility2_index;

  if (robot_specifications->min_plan_index_x < 
      robot_specifications->max_plan_index_x &&
      robot_specifications->min_plan_index_y < 
      robot_specifications->max_plan_index_y){
    
    if (robot_specifications->dp_direction == 0 ||
	robot_specifications->dp_direction == 2){
      from_x      = robot_specifications->min_plan_index_x;
      to_x        = robot_specifications->max_plan_index_x;
      increment_x = 1;
    }
    else{
      from_x      = robot_specifications->max_plan_index_x -1;
      to_x        = robot_specifications->min_plan_index_x -1;
      increment_x = -1;
    }
    if (robot_specifications->dp_direction == 1 ||
	robot_specifications->dp_direction == 2){
      from_y      = robot_specifications->min_plan_index_y;
      to_y        = robot_specifications->max_plan_index_y;
      increment_y = 1;
    }
    else{
      from_y      = robot_specifications->max_plan_index_y -1;
      to_y        = robot_specifications->min_plan_index_y -1;
      increment_y = -1;
    }
    robot_specifications->dp_direction = 
      (robot_specifications->dp_direction + 1) % 4;

    new_min_plan_index_x = robot_specifications->max_plan_index_x + 1;
    new_max_plan_index_x = -1;
    new_min_plan_index_y = robot_specifications->max_plan_index_y + 1;
    new_max_plan_index_y = -1;


    /* ********** MAIN DYNAMIC PROGRAMMING PLANNING LOOP ********** */
    for (x = from_x; x != to_x; x += increment_x){
      index = x * robot_specifications->global_map_dim_y + from_y;
      for (y = from_y; y != to_y; y += increment_y, index += increment_y){
	if (program_state->actual_map_number == 0 && global_goal[index]){
	  new_utility  = 1.0;
	  new_utility2 = 1.0;
	  new_utility_index  = -1;
	  new_utility2_index = -1;
	}
 	else if ((program_state->actual_map_number != 0 ||
		  robot_specifications->number_active_goals[NUMBER] == 0) && 
		 !global_explored[index]){
	  new_utility  = exploration_utility_factor(x, y, 
						    robot_specifications,
						    robot_state,
						    program_state); 
	  new_utility2 = exploration_utility_factor(x, y, 
						    robot_specifications,
						    robot_state,
						    program_state);
	  new_utility_index  = -1;
	  new_utility2_index = -1;
	}
	else{			/* do the DP here! */
	  new_utility = global_utility[index-1]; /* field on the left */
	  new_utility_index = index-1;
	  
	  help = global_utility[index+1];
	  if (help > new_utility){
	    new_utility = help; /* right */
	    new_utility_index = index+1;
	  }
	  
	  help = 
	    global_utility[index-robot_specifications->global_map_dim_y];
	  if (help > new_utility){	/* below */
	    new_utility = help;
	    new_utility_index = index-robot_specifications->global_map_dim_y;
	  }
	  
	  help = 
	    global_utility[index+robot_specifications->global_map_dim_y];
	  if (help > new_utility){	/* above */
	    new_utility = help;
	    new_utility_index = index+robot_specifications->global_map_dim_y;
	  }
	  
	  
	  
	  new_utility2 =	/* below left */
	    global_utility[index 
			   - robot_specifications->global_map_dim_y - 1];
	  new_utility2_index = index 
	    - robot_specifications->global_map_dim_y - 1;
	  
	  help = global_utility[index
				- robot_specifications->global_map_dim_y
				+ 1];
	  if (help > new_utility2){	/* below right */
	    new_utility2 = help;
	    new_utility2_index = index
	      - robot_specifications->global_map_dim_y + 1;
	  }
	  
	  help = global_utility[index + 
				robot_specifications->global_map_dim_y 
				- 1] ;
	  if (help > new_utility2){	/* above left */
	    new_utility2 = help;
	    new_utility2_index = index + 
	      robot_specifications->global_map_dim_y - 1;
	  }
	  
	  help = global_utility[index + 
				robot_specifications->global_map_dim_y 
				+ 1] ;
	  if (help > new_utility2){	/* above right */
	    new_utility2 = help;
	    new_utility2_index = index + 
	      robot_specifications->global_map_dim_y + 1;
	  }
	}

	new_utility  *= global_costs[index];
	new_utility2 *= global_costs2[index];
	if (new_utility2 > new_utility){
	  new_utility = new_utility2;
	  new_utility_index = new_utility2_index;
	}


	/*	fprintf(stderr, "index = %d\n", index);*/
	if (global_utility[index] < new_utility * 0.9999){ /*! here was a != */
	  /* even more important: Linux/Pentium/gcc is buggy: a simple
	     check of (global_utility[index] == new_utility) will
	     be true after the immediate next statement, but often become 
	     flase after the statement following the next one. Rats. 
	     That's why I introduced the 0.9999 here. */
	  global_utility[index] = new_utility;
	  global_succ[index] = new_utility_index;

#ifdef show_that_pentiums_make_errors
	  if (global_utility[index] < new_utility){ /*!*/
	    fprintf(stderr, "ERROR in Linux/Pentium %g - %g = %g\n",
		    global_utility[index], new_utility,
		     global_utility[index] - new_utility);
	    fprintf(stderr, "%p %p",
		  &(global_utility[index]), &(global_succ[index]));
	    fprintf(stderr, " (%d)\n", index);
	    exit(-1);
	  }
#endif
	  if (x < new_min_plan_index_x) new_min_plan_index_x = x;
	  if (x > new_max_plan_index_x) new_max_plan_index_x = x;
	  if (y < new_min_plan_index_y) new_min_plan_index_y = y;
	  if (y > new_max_plan_index_y) new_max_plan_index_y = y;
	}
      }	
    }

    /* ======= update planning bounding interval =========== */
    /*    if (new_max_plan_index_x == -1 || new_max_plan_index_y == -1)
	  printf("planning done\n");*/
    new_min_plan_index_x -= 1;
    new_max_plan_index_x += 2;
    new_min_plan_index_y -= 1;
    new_max_plan_index_y += 2;
    if (new_min_plan_index_x < robot_specifications->min_display_index_x)
      new_min_plan_index_x = robot_specifications->min_display_index_x;
    if (new_max_plan_index_x > robot_specifications->max_display_index_x)
      new_max_plan_index_x = robot_specifications->max_display_index_x;
    if (new_min_plan_index_y < robot_specifications->min_display_index_y)
      new_min_plan_index_y = robot_specifications->min_display_index_y;
    if (new_max_plan_index_y > robot_specifications->max_display_index_y)
      new_max_plan_index_y = robot_specifications->max_display_index_y;
    robot_specifications->min_plan_index_x = new_min_plan_index_x;
    robot_specifications->max_plan_index_x = new_max_plan_index_x;
    robot_specifications->min_plan_index_y = new_min_plan_index_y;
    robot_specifications->max_plan_index_y = new_max_plan_index_y;
    check_index(robot_specifications, 0);
  }
  search_for_inconsistencies_in_succ(robot_specifications, "forward DP");
}






/************************************************************************
 *
 *   NAME:         compute_plan
 *                 
 *   FUNCTION:     generates a plan by search in the utility table.
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *                 
 *                 
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void compute_plan(ROBOT_STATE_PTR robot_state,
		  ROBOT_SPECIFICATIONS_PTR robot_specifications,
		  ACTION_PTR  action,
		  PROGRAM_STATE_PTR program_state,
		  int display_plan,
		  int iteration)
{
  int x, y, last_x, last_y, delta_x, delta_y, index;
  int border, local_minimum;
  float best_util, delta_robot_x, delta_robot_y, threshold;
  int from_x, from_y, test_index;
  float test_x, test_y, increment_x, increment_y, test_distance;
  int first_action, i, n, counter, first_action_set;
  

  from_x = x = ((int) (robot_state->x / robot_specifications->resolution))
    + robot_specifications->autoshifted_int_x;
  from_y = y = ((int) (robot_state->y / robot_specifications->resolution))
    + robot_specifications->autoshifted_int_y;
  index = x * robot_specifications->global_map_dim_y + y;
  threshold = robot_specifications->collision_threshold;
#if 0
   threshold = global_values[index];
   if (threshold > 0.7) threshold = 0.7;
   if (threshold < 0.3) threshold = 0.3; /*!*/
#endif
  border = 0;
  local_minimum = 0;
  first_action = 1;
  first_action_set = 1;
  if (program_state->graphics_initialized && display_plan){
    G_clear_markers(PLAN_DISPLAY);
    G_add_marker(PLAN_DISPLAY, robot_state->x, robot_state->y, 
		 first_action_set);
  }
  action->base = 0.0;
  action->turn = 0.0;
  action->active_goal_name = -1;
  action->goal_x = 0.0;
  action->goal_y = 0.0;
  counter = 0;
  action->goal_dist = 0.0;
  

  if (
      /*
       * Are we in goal hunting mode? If so, we reached a local miminum
       * if the robot's at a goal
       */
	(program_state->actual_map_number == 0 && global_goal[index]) ||
      /*
       * Are we in exploration mode? There are two observations that would
       * justify exploration mode: actual map != 0 or, alternatively,
       * no goal defined. 
       * If this is the case, a local minimum is reached if we are at
       * an unexplored point.
       */
      ((program_state->actual_map_number != 0 ||
	robot_specifications->number_active_goals[BEST_NUMBER] == 0)
       && !global_explored[index])){ /*! don't know if this is correct */

    /*
     * Okay, we have to stop here
     */
    local_minimum = 1;
    if (iteration == 0)
      printf("LOCAL MINIMUM REACHED: %g %g %d %d %d %d %d %d\n",
	     robot_state->x, robot_state->y, x, y, index, 
	     global_goal[index],
	     robot_specifications->number_active_goals[BEST_NUMBER], 
	     global_explored[index]);
    /*!
     * set up a default robot move
     */
    action->base = robot_specifications->min_base;
    action->turn = 0.0;

  }
  else{				/* there is no obvious local minimum here
				 * let's do some deliberation */
    do{

      /*======= search ========*/
      index = x * robot_specifications->global_map_dim_y + y;
      delta_x = delta_y = 0;

      if (global_succ[index] < 0){
	local_minimum = 1;
	if (program_state->actual_map_number == 0)
	  action->active_goal_name = global_goal[index] - 1;
	else
	  action->active_goal_name = -1;
	/*if (action->active_goal_name == -1)
	  printf("$");
	  else{ */
	action->goal_x = ((float) 
			  (x - robot_specifications->autoshifted_int_x))
	  * robot_specifications->resolution;
	action->goal_y = ((float) 
			  (y - robot_specifications->autoshifted_int_y))
	  * robot_specifications->resolution;
	/*} */
	
      }
      
      else{			/* micro-action found */
	
	if (global_succ[index] == index + 1)
	  delta_y = 1;
	else if (global_succ[index] == index - 1)
	  delta_y = -1;
	else if (global_succ[index] == 
		 index + robot_specifications->global_map_dim_y)
	  delta_x = 1;
	else if (global_succ[index] == 
		 index - robot_specifications->global_map_dim_y)
	  delta_x = -1;
	
	else if (global_succ[index] == 
		 index + robot_specifications->global_map_dim_y + 1){
	  delta_y = 1;
	  delta_x = 1;
	}
	else if (global_succ[index] == 
		 index + robot_specifications->global_map_dim_y - 1){
	  delta_y = -1;
	  delta_x = 1;
	}
	else if (global_succ[index] == 
		 index - robot_specifications->global_map_dim_y + 1){
	  delta_y = 1;
	  delta_x = -1;
	}
	else if (global_succ[index] == 
		 index - robot_specifications->global_map_dim_y - 1){
	  delta_y = -1;
	  delta_x = -1;
	}
	else
	  fprintf(stderr, "ERROR: successor of cell %d is %d (y_dim=%d)\n",
		  index, global_succ[index], 
		  robot_specifications->global_map_dim_y);



	last_x = x;
	last_y = y;
	x += delta_x;
	y += delta_y;
	
	if (x <= 0 || x >= robot_specifications->global_map_dim_x - 1 ||
	    y <= 0 || y >= robot_specifications->global_map_dim_y - 1)
	  border = 1;
	
	
	if (first_action && counter > 1 && (x != from_x || y != from_y)){ 
	  /* ============== test for smoothing ============== */
	  increment_x = (float) (x - from_x);
	  increment_y = (float) (y - from_y);
	  test_distance = sqrt((increment_x * increment_x) 
			       + (increment_y * increment_y));
	  increment_x /= test_distance;
	  increment_y /= test_distance;
	  for (test_x = (float) from_x, test_y = (float) from_y, i = 0;
	       i < (int) test_distance;
	       test_x += increment_x, test_y += increment_y, i++){
	    test_index = ((int) test_x)
	      * robot_specifications->global_map_dim_y
		+ ((int) test_y);
	    if (global_active[test_index] &&
		global_values[test_index] < threshold){
	      first_action = 0;
	      x = last_x;
	      y = last_y;
	    }
	  }
	}
      }
      
      if (!first_action || local_minimum || border){
	if (program_state->graphics_initialized && display_plan)
	  G_add_marker(PLAN_DISPLAY, 
		       ((float) (x - robot_specifications->autoshifted_int_x))
		       * robot_specifications->resolution,
		       ((float) (y - robot_specifications->autoshifted_int_y))
		       * robot_specifications->resolution, first_action_set);
	
	/* compute first action */
	if (first_action_set == 1){ 	/* this is the first macro action! */
	  delta_robot_x = 
	    ((float) (x - robot_specifications->autoshifted_int_x))
	    * robot_specifications->resolution - robot_state->x;
	  delta_robot_y = 
	    ((float) (y - robot_specifications->autoshifted_int_y))
	    * robot_specifications->resolution - robot_state->y;
	  action->base = sqrt((delta_robot_x * delta_robot_x) +
			      (delta_robot_y * delta_robot_y));
	  if (action->base > 0.0){
	    action->turn = atan2(delta_robot_y, delta_robot_x) / pi * 180.0;
	    action->turn -= robot_state->orientation;
	  }
	}	
	first_action_set = 0;
      }
      counter++;
    }
    while (counter < MAX_NUM_PLAN_STEPS && /* OLD STUFF, GOES BACK TO
	      TIMES WHERE THERE WERE BUG IN succ THAT CAUSED CYCLES 
	      
	      ... seems we have that same old bug again
	      
	      */
	   border == 0 && local_minimum == 0);
  }
  
  action->final_action = first_action; /* preslimnary */
  action->goal_dist = ((float) counter) * robot_specifications->resolution;
  
  if (action->base < robot_specifications->min_base &&
      (!action->final_action || !program_state->goal_modus))
    action->base = robot_specifications->min_base;	/*!*/
  if (action->base > robot_specifications->max_base && 
      (!action->final_action || !program_state->goal_modus))
    action->base = robot_specifications->max_base;	/*!*/

  
  /* truncate action */
  for (; action->turn <= -180.0; ) action->turn += 360.0;
  for (; action->turn >   180.0; ) action->turn -= 360.0;
  
  /* check if action will lead to (generalized) goal, or to local min. */
  
  action->success = local_minimum &&
    ((program_state->actual_map_number == 0 && global_goal[index]) ||
     ((program_state->actual_map_number != 0 ||
       robot_specifications->number_active_goals[BEST_NUMBER] == 0)
      && !global_explored[index])); /*!same as above: maybe global_active */
  /* check if replanning absolutely nesessary */
  action->no_plan = (from_x == x && from_y == y); 


  if (iteration == 0 && action->no_plan == action->success){
    fprintf(stderr, "###??? action->no_plan %d action->success %d\n",
	    action->no_plan, action->success);
  }

  /*
   *==================== 
   *   new part: clipping values along a (successful) plan
   *=====================
   */

  if (action->success && robot_specifications->min_mapvalue_along_path > 0.0){

    /*
     * robot coordinates
     */

    x = ((int) (robot_state->x / robot_specifications->resolution))
      + robot_specifications->autoshifted_int_x;
    y = ((int) (robot_state->y / robot_specifications->resolution))
      + robot_specifications->autoshifted_int_y;
    local_minimum = 0;
    border = 0;
    counter = 0;

    do{
      
      /*======= search ========*/
      index = x * robot_specifications->global_map_dim_y + y;
      delta_x = delta_y = 0;

      if (global_succ[index] < 0)
	local_minimum = 1;

      else{
	
	if (global_succ[index] == index + 1)
	  delta_y = 1;
	else if (global_succ[index] == index - 1)
	  delta_y = -1;
	else if (global_succ[index] == 
		 index + robot_specifications->global_map_dim_y)
	  delta_x = 1;
	else if (global_succ[index] == 
		 index - robot_specifications->global_map_dim_y)
	  delta_x = -1;
	
	else if (global_succ[index] == 
		 index + robot_specifications->global_map_dim_y + 1){
	  delta_y = 1;
	  delta_x = 1;
	}
	else if (global_succ[index] == 
		 index + robot_specifications->global_map_dim_y - 1){
	  delta_y = -1;
	  delta_x = 1;
	}
	else if (global_succ[index] == 
		 index - robot_specifications->global_map_dim_y + 1){
	  delta_y = 1;
	  delta_x = -1;
	}
	else if (global_succ[index] == 
		 index - robot_specifications->global_map_dim_y - 1){
	  delta_y = -1;
	  delta_x = -1;
	}
	else
	  fprintf(stderr, "ERROR: successor of cell %d is %d (y_dim=%d)\n",
		  index, global_succ[index], 
		  robot_specifications->global_map_dim_y);
	/*
	 * clipping
	 */

	if (global_active[index] &&
	    global_values[index] 
	    < robot_specifications->min_mapvalue_along_path)
	  global_values[index] =
	    robot_specifications->min_mapvalue_along_path;
	
	
	if (x <= 0 || x >= robot_specifications->global_map_dim_x - 1 ||
	    y <= 0 || y >= robot_specifications->global_map_dim_y - 1)
	  border = 1;
	
	x += delta_x;
	y += delta_y;
	
      }
      
      counter++;
    }
    while (counter < MAX_NUM_PLAN_STEPS && 
	   global_succ[index] >= 0 && !border && !local_minimum);
  }
}




/************************************************************************
 *
 *   NAME:         adjust_action
 *                 
 *   FUNCTION:     changes action such that the security distance is kept
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *                 
 *                 
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

#define increment 1.0		/*!*/

void adjust_action(ROBOT_STATE_PTR robot_state,
		   ROBOT_SPECIFICATIONS_PTR robot_specifications,
		   ACTION_PTR  action,
		   PROGRAM_STATE_PTR program_state,
		   int display_action)
{
  int from_x, from_y, to_x, to_y, from_index, test_index;
  float increment_x, increment_y, threshold, test_angle, help;
  float next_state_x, next_state_y, test_x, test_y, test_distance;
  float this_state_x, this_state_y;
  int i, j, k, occupied[360], distance_to_occupied[360]; /* be carefule when
							  * you change this
							  * constant */
  int wall_parallel[360];
  float wall_angle;
  int wall_angle_k;
  int range, search_index, change;
  int diff_search_index;


  if (program_state->graphics_initialized && display_action)
    G_clear_markers(ADJUSTED_ACTION);
  if (robot_specifications->max_security_dist > 0.0 &&  action->base > 0.0){
    test_distance = action->base + robot_specifications->robot_size;

    this_state_x = /* - robot_specifications->robot_size
      * cos((robot_state->orientation + action->turn)
	    / 180.0 * pi) + */ robot_state->x;
    this_state_y = /* - robot_specifications->robot_size
      * sin((robot_state->orientation + action->turn) 
	    / 180.0 * pi) + */ robot_state->y;
    from_x = ((int) (this_state_x / robot_specifications->resolution))
      + robot_specifications->autoshifted_int_x;
    from_y = ((int) (this_state_y / robot_specifications->resolution))
      + robot_specifications->autoshifted_int_y;
    from_index = from_x * robot_specifications->global_map_dim_y + from_y;
    threshold = robot_specifications->collision_threshold;
#ifdef hjunkl
    {
      static int iii = 0;
      if (!iii){/*!*/
	fprintf(stderr, "HACK HACK HACK HACK HACK HACK HACK HACK\n");
	iii = 1;
      }
      if (action->final_action && program_state->goal_modus)
	threshold = 1.0;
    }
#endif
#if 0
      threshold = global_values[from_index];
      if (threshold > 0.65) threshold = 0.65;
      if (threshold < 0.3) threshold = 0.3;/*!*/
#endif
    /* ************* SEARCH FOR ALL FREE DIRECTIONS ************* */

    
    for (test_angle = -robot_specifications->max_adjust_angle, k = 0;
	 test_angle <= robot_specifications->max_adjust_angle;
	 test_angle += increment, k++){

      
      next_state_x = (action->base 
		      + (fabs(test_angle) 
			 / robot_specifications->max_adjust_angle 
			 * robot_specifications->robot_size))
	* cos((robot_state->orientation + action->turn + test_angle)
	      / 180.0 * pi) + robot_state->x;
      next_state_y = (action->base 
		      + (fabs(test_angle) 
			 / robot_specifications->max_adjust_angle 
			 * robot_specifications->robot_size))
	* sin((robot_state->orientation + action->turn + test_angle) 
	      / 180.0 * pi) + robot_state->y;

      to_x = ((int) (next_state_x / robot_specifications->resolution))
	+ robot_specifications->autoshifted_int_x;
      to_y = ((int) (next_state_y / robot_specifications->resolution))
	+ robot_specifications->autoshifted_int_y;

      increment_x = (float) (to_x - from_x);
      increment_y = (float) (to_y - from_y);
      increment_x /= test_distance;
      increment_y /= test_distance;
      if (k == 0 ||		/* extrema: set to occupied
				 * This is important, since otherwise
				 * the robot might be driven right next 
				 * to an obstacle (outside this interval) */
	  test_angle + increment > robot_specifications->max_adjust_angle)
	occupied[k] = 1;
      else
	occupied[k] = 0;

      /*
       * check, if we are parallel/orthogonal to walls
       */

      wall_parallel[k] = 0;
      if (robot_state->map_orientation_defined){
	for (i = 0; i < 4; i++){
	  wall_angle = robot_state->orientation + action->turn + test_angle
	    - robot_state->map_orientation + (((float) i) * 90.0);
	  for (;wall_angle <= -180.0;) wall_angle += 360.0;
	  for (;wall_angle >   180.0;) wall_angle -= 360.0;
	  if (fabs(wall_angle) < increment)
	    wall_parallel[k] = 1;
	}
      }





      for (test_x = (float) from_x, test_y = (float) from_y, i = 0;
	   i < (int) test_distance && !occupied[k];
	   test_x += increment_x, test_y += increment_y, i++){
	if (test_x >= 0 && test_x < robot_specifications->global_map_dim_x &&
	    test_y >= 0 && test_y < robot_specifications->global_map_dim_y){
	  test_index = ((int) test_x) * robot_specifications->global_map_dim_y
	    + ((int) test_y);
	  if (global_active[test_index] && 
	      global_values[test_index] < threshold)
	    occupied[k] = 1;
	}
      }
      
      
      if (test_x >= 0 && test_x < robot_specifications->global_map_dim_x &&
	  test_y >= 0 && test_y < robot_specifications->global_map_dim_y)
	if (program_state->graphics_initialized && display_action)
	  G_add_marker(ADJUSTED_ACTION, 
		       (test_x -
			((float) robot_specifications->autoshifted_int_x))
		       * robot_specifications->resolution,
		       (test_y - 
			((float) robot_specifications->autoshifted_int_y))
		       * robot_specifications->resolution, occupied[k]);
    }

    /* ************** DETERMINE WALL-PARALLEL LINES *************** */
    
#ifdef xxx    
    if (robot_state->map_orientation_defined){
      for (i = 0; i < 4; i++){
	/*
	 * angle of a wall relative to the non-adjusted robot's action
	 */
	wall_angle = robot_state->map_orientation - 
	  (robot_state->orientation + action->turn)
	    + (((float) i) * 90.0);
	for (;wall_angle <  0.0;)   wall_angle += 360.0;
	for (;wall_angle >= 360.0;) wall_angle -= 360.0;

	fprintf(stderr, "[ANGLE: %g ", wall_angle);

	/*
	 * compute the field index for that angle
	 */
	wall_angle = (wall_angle 
		      + robot_specifications->max_adjust_angle) / increment;
	for (;wall_angle <  0.0;)   wall_angle += 360.0;
	for (;wall_angle >= 360.0;) wall_angle -= 360.0;
	wall_angle_k = (int) (wall_angle + 0.5);

	fprintf(stderr, " %g %d]\n", wall_angle, wall_angle_k);

	if (wall_angle_k >= 0 && wall_angle_k < 360)
	  wall_parallel[wall_angle_k] = 1;
      }
    }
#endif
    /* ************** DETERMINE SECURITY RADIUS *************** */
    
    /* what is the angle of a triangle with side lengths given by
       robot_specifications->max_security_dist, action->base, action->base ? */
    
    if (!robot_state->stuck){
      help = 0.5 * robot_specifications->max_security_dist / action->base;
      if (fabs(help) <= 1.0)
	range = ((int) (2.0 * asin(help) * 180.0 / pi / increment)) + 1;
      else
	range = k;
    }
    else
      range = k;


#ifdef xxx
    range = k;			/*!*/
    {
      static int i = 0;
      if (!i){
	fprintf(stderr, "\n\n\t\t HACK IN THE PROGRAM\n\n");
	i = 1;
      }
    }
#endif

    /* ************** DETERMINE "DISTANCE" TO NEXT OBSTACLE *************** */
    
    for (i = 0; i < k; i++){
      distance_to_occupied[i] = 10000;
      for (j = 0; j <= k /*! range */
	   && distance_to_occupied[i] == 10000; j++){
	if (i+j < k && occupied[i+j])
	  distance_to_occupied[i] = j;
	if (i-j >= 0 && occupied[i-j])
	  distance_to_occupied[i] = j;
      }
    }



    /* ************** SEARCH FOR CLOSEST SECURE POINT *************** */
    
    diff_search_index = 0;
    search_index = (robot_specifications->max_adjust_angle / increment);
    change = 1;
    do{
      if (wall_parallel[search_index] && abs(diff_search_index) >= range) {
	/*putc(7, stderr);*/
	fprintf(stderr, "Yup\n");
	change = 0;
      }
      else if (search_index >= 1 && distance_to_occupied[search_index-1] >
	  distance_to_occupied[search_index]){
	search_index--;
	diff_search_index--;
	action->turn -= increment;
      }
      else
      if (search_index < k-1 && distance_to_occupied[search_index+1] >
	  distance_to_occupied[search_index]){
	search_index++;
	diff_search_index++;
	action->turn += increment;
      }
      else
	change = 0;
    }
    while (change);
    
    /* ************** DISPLAY ADJUSTED ACTION *************** */

    if (program_state->graphics_initialized && display_action){
      G_add_marker(ADJUSTED_ACTION, robot_state->x, robot_state->y, 0);
      G_add_marker(ADJUSTED_ACTION, action->base 
		   * cos((robot_state->orientation + action->turn) 
			 / 180.0 * pi)
		   + robot_state->x,
		   action->base 
		   * sin((robot_state->orientation  + action->turn)
			 / 180.0 * pi)
		   + robot_state->y, 2);
    }
  }

  /* fox: We have to store the absolute values in action */
  action->absolute_x = robot_state->x + 
    action->base * cos((robot_state->orientation + action->turn) / 180.0 * pi);
  action->absolute_y = robot_state->y + 
    action->base * sin((robot_state->orientation + action->turn) / 180.0 * pi);
  
}

/************************************************************************
 *
 *   NAME:         check_for_simple_exploration_action
 *
 *   FUNCTION:     checks, if in the exploraiton mode we can get
 *                 to something unexplored with a straight line
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

int check_for_simple_exploration_action(ROBOT_SPECIFICATIONS_PTR
					robot_specifications,
					PROGRAM_STATE_PTR program_state,
					ROBOT_STATE_PTR robot_state,
					ACTION_PTR action)
{
  float from_x, from_y, to_x, to_y;
  float increment_x, increment_y;
  float x, y, d;
  float test_distance;
  float target_x, target_y;
  int   free, test_index, unexplored;
  float target_angle;
  
  if (program_state->goal_modus || !robot_specifications->fast_exploration ||
      !robot_state->map_orientation_defined)
    return 0;

  /*
   * Calculate grid cell numbers, but in float
   */

  target_angle = 0.0;

  if (robot_state->map_orientation_defined){
    target_angle = robot_state->map_orientation 
      - robot_state->orientation;
    for (; target_angle < -45.0; ) target_angle += 90.0;
    for (; target_angle >  45.0; ) target_angle -= 90.0;
  }    

  from_x =  (robot_state->x / robot_specifications->resolution)
    + ((float) robot_specifications->autoshifted_int_x);
  from_y =  (robot_state->y / robot_specifications->resolution)
    + ((float) robot_specifications->autoshifted_int_y);

  target_x = robot_state->x + 
    (cos((robot_state->orientation + target_angle) * M_PI / 180.0) 
     * robot_specifications->max_base);
  target_y = robot_state->y + 
    (sin((robot_state->orientation + target_angle) * M_PI / 180.0) 
     * robot_specifications->max_base);

  to_x =  (target_x / robot_specifications->resolution)
    + ((float) robot_specifications->autoshifted_int_x);
  to_y =  (target_y / robot_specifications->resolution)
    + ((float) robot_specifications->autoshifted_int_y);

  
  increment_x = to_x - from_x;
  increment_y = to_y - from_y;

  test_distance = sqrt((increment_x * increment_x)
		       + (increment_y * increment_y));

  if (test_distance > 0.0){
    increment_x /= test_distance;
    increment_y /= test_distance;
  }


  /*
   * Check, if free!
   */

  free       = 1;
  unexplored = 0;
  x          = from_x;
  y          = from_y;

  for (d = 0.0; d <= test_distance && free; d += 1.0){
    test_index = ((int) x) * robot_specifications->global_map_dim_y +((int) y);
    if (global_active[test_index] && 
	global_values[test_index] < robot_specifications->collision_threshold)
      free = 0;
    if (!global_active[test_index])
      unexplored = 1;
    x += increment_x;
    y += increment_y;
  }
  fprintf(stderr, "   free %d unexplored %d\n", free, unexplored);

  if (!unexplored || !free)
    return 0;

  action->base = d;
  if (action->base < robot_specifications->min_base)
    action->base = robot_specifications->min_base;
  action->turn = target_angle;
  action->absolute_x = robot_state->x + 
    action->base * cos((robot_state->orientation + action->turn) / 180.0 * pi);
  action->absolute_y = robot_state->y + 
    action->base * sin((robot_state->orientation + action->turn) / 180.0 * pi);
  
  action->potential_next_base       = action->base;
  action->potential_next_turn       = action->turn;
  action->potential_next_absolute_x = action->absolute_x;
  action->potential_next_absolute_y = action->absolute_y;

  action->potential_next_absolute_x = robot_state->x + 
    (action->base + robot_specifications->min_base)
      * cos((robot_state->orientation + action->turn) / 180.0 * pi);
  action->potential_next_absolute_y = robot_state->y + 
    (action->base + robot_specifications->min_base)
      * sin((robot_state->orientation + action->turn) / 180.0 * pi);

  action->success = 1;
  action->no_plan = 1;
  action->final_action = 1;
  /*putc(7, stderr);*/
  fprintf(stderr, "\t\t\tsimple action.\n");

  if (program_state->graphics_initialized){
    G_add_marker(ADJUSTED_ACTION, robot_state->x, robot_state->y, 0);
    G_add_marker(ADJUSTED_ACTION, action->base 
		 * cos((robot_state->orientation + action->turn) 
		       / 180.0 * pi)
		 + robot_state->x,
		 action->base 
		 * sin((robot_state->orientation  + action->turn)
		       / 180.0 * pi)
		 + robot_state->y, 2);
    G_display_markers(ADJUSTED_ACTION);
  }
  
  

  return 1;
}

/************************************************************************
 *
 *   NAME:         generate_action
 *                 
 *   FUNCTION:     generates an action. 
 *                 
 *   PARAMETERS:   int plan_till_success   will continue planning till
 *                                         an action is found
 *                 int display_plan        displays action
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void generate_action(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		     PROGRAM_STATE_PTR program_state,
		     ROBOT_STATE_PTR robot_state,
		     ACTION_PTR action,
		     int plan_till_success,
		     int display_plan)
{
  int r_x, r_y, i, timeout;
  float dist;
  ACTION_TYPE              local_action_data;
  ACTION_PTR               local_action            = &local_action_data;
  ROBOT_STATE              local_robot_state_data;
  ROBOT_STATE_PTR          local_robot_state       = &local_robot_state_data;


  program_state->busy++;

  /* -------------- compute plan and action -------------- */

  if (check_for_simple_exploration_action(robot_specifications,
					  program_state,
					  robot_state,
					  action))
    return;

  
  compute_plan(robot_state, robot_specifications, action, program_state, 
	       display_plan, 0); 
  adjust_action(robot_state, robot_specifications, action, program_state, 
		display_plan);
  
    
  if (!action->success && plan_till_success){	/* no perfect plan */
    if (PLAN_verbose)
      printf("--------> some more planning necessary - please wait..");
    fflush(stdout);
    /*reset_descending_utilities(robot_specifications, program_state,
      robot_state);*/
    timeout = 0;

    do{
      mouse_test_loop(robot_state, program_state, robot_specifications, 
		      action);
      for (i = 0; i < 8; i++)
	dynamic_programming(robot_specifications, program_state, robot_state);
      if (PLAN_verbose){
	printf(".");
	fflush(stdout);
      }
      compute_plan(robot_state, robot_specifications, action,
		   program_state, 1, 0); 
      timeout++;
    }
    while (!action->success && 
	   timeout < robot_specifications->global_mapsize_x
	   + robot_specifications->global_mapsize_y);
    adjust_action(robot_state, robot_specifications, action, program_state, 1);
    if (PLAN_verbose){
      if (!action->success)
	printf("gave up!\n");
      else
	printf("thanks!\n");
    }
  }


  /* -------------- compute potential second action -------------- */

  local_robot_state_data  = *robot_state;
  local_action_data       = *action;
  /*
    fprintf(stderr, "[%g %g %g] [%g %g %g]\n",
    robot_state->x, robot_state->y, robot_state->orientation,
    local_robot_state->x, local_robot_state->y, 
    local_robot_state->orientation);
    */

  local_robot_state->x = action->absolute_x;
  local_robot_state->y = action->absolute_y;
  
  compute_plan(local_robot_state, robot_specifications, local_action, 
	       program_state, 0, 1); 
  adjust_action(local_robot_state, robot_specifications, local_action, 
		program_state, 0);
  
  action->potential_next_base       = local_action->base;
  action->potential_next_turn       = local_action->turn;
  action->potential_next_absolute_x = local_action->absolute_x;
  action->potential_next_absolute_y = local_action->absolute_y;

  
  /* -------------- print answer-------------- */
  if (PLAN_debug){
    
    printf("### 1st ACTION: x %g  y %g  turn %g  base %g  success %d  goal %d final %d dist %g\n", 
	   action->absolute_x, action->absolute_y, 
	   action->turn, action->base, action->success,
	   action->active_goal_name, action->final_action,
	   action->goal_dist);
    
    printf("    2nd ACTION: x %g  y %g  turn %g  base %g  success %d  goal %d final %d dist %g\n", 
	   local_action->absolute_x, local_action->absolute_y, 
	   local_action->turn, local_action->base, local_action->success,
	   local_action->active_goal_name, local_action->final_action,
	   local_action->goal_dist);
  }

  /* --------------  adjust the planning bounding box -------------- */
  
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

  check_index(robot_specifications, 0);
  
  /* -------------- and print the resuls -------------- */

  if (display_plan){
    if (program_state->graphics_initialized){
      /*G_activate(GLOBAL_BACKGROUND);
	G_display_switch(GLOBAL_BACKGROUND, 0); */
      G_display_markers(ADJUSTED_ACTION);
      G_display_markers(PLAN_DISPLAY);
    }
  }
  
}


/************************************************************************
 *
 *   NAME:         check_if_point_free
 *                 
 *   FUNCTION:     Checks, if a point is considered "free"
 *                 
 *   PARAMETERS:   point_x, point_y
 *
 *   RETURN-VALUE: 1, if free
 *                 
 ************************************************************************/

int check_if_point_free(float point_x, float point_y,
			ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int x, y, index;

  x = ((int) (point_x / robot_specifications->resolution))
    + robot_specifications->autoshifted_int_x;
  y = ((int) (point_y / robot_specifications->resolution))
    + robot_specifications->autoshifted_int_y;
  index = x * robot_specifications->global_map_dim_y + y;

  return (int) (!global_active[index] ||
		global_values[index] > 
		robot_specifications->collision_threshold);
}



/************************************************************************
 *
 *   NAME:         compute_forward_correction
 *                 
 *   FUNCTION:     Computes a correction to robot position estimate
 *                 
 *   PARAMETERS:   robot_x, robot_y, robot_orientation  non-corrected
 *                                                      robot pos
 *                 corr_x, corr_y, corr_angle           corretion parameters
 *                 corr_type                            ditto
 *                 *corr_robot_x, *corr_robot_y, *corr_orientation
 *                                                      corrected position
 *
 *   RETURN-VALUE: nonw
 *                 
 ************************************************************************/


void compute_forward_correction(float robot_x, 
				float robot_y, 
				float robot_orientation,
				float corr_x, 
				float corr_y, 
				float corr_angle, /* in deg. */
				int   corr_type,
				float *corr_robot_x,
				float *corr_robot_y, 
				float *corr_orientation)
{
  float delta_x, delta_y;
  float angle;

  if (corr_type){		/* 1=rotation about corr_x|corr_y */
    delta_x = robot_x - corr_x;	/* difference to center of rotation */
    delta_y = robot_y - corr_y;	/* difference to center of rotation */
    angle = corr_angle / 180.0 * M_PI;
    *corr_robot_x = corr_x + (delta_x * cos(angle)) - (delta_y * sin(angle));
    *corr_robot_y = corr_y + (delta_x * sin(angle)) + (delta_y * cos(angle));
    *corr_orientation = robot_orientation + corr_angle;
    for (;*corr_orientation >  180.0;) *corr_orientation -= 360.0;
    for (;*corr_orientation < -180.0;) *corr_orientation += 360.0;
  }

  else{				/* otherwise: just translation */
    *corr_robot_x = robot_x + corr_x;
    *corr_robot_y = robot_y + corr_y;
    *corr_orientation = robot_orientation + corr_angle;
  }
}




/************************************************************************
 *
 *   NAME:         compute_backward_correction
 *                 
 *   FUNCTION:     Computes a BASE robot position estimate
 *                 based on a corrected position
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: nonw
 *                 
 ************************************************************************/



void compute_backward_correction(float corr_robot_x, 
				 float corr_robot_y, 
				 float corr_robot_orientation,
				 float corr_x, 
				 float corr_y, 
				 float corr_angle, /* in deg. */
				 int   corr_type,
				 float *robot_x,
				 float *robot_y, 
				 float *orientation)
{
  if (corr_type == 1)
    compute_forward_correction(corr_robot_x, corr_robot_y, 
			       corr_robot_orientation,
			       corr_x, corr_y, -corr_angle, corr_type,
			       robot_x, robot_y, orientation);
  else
    compute_forward_correction(corr_robot_x, corr_robot_y, 
			       corr_robot_orientation,
			       -corr_x, -corr_y, -corr_angle, corr_type,
			       robot_x, robot_y, orientation);
}





/************************************************************************
 *
 *   NAME:         clear_maps()
 *                 
 *   FUNCTION:     clears all internal maps
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void clear_maps(ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int i, size, k;

  size = robot_specifications->global_map_dim_x
    * robot_specifications->global_map_dim_y;

  
  /* INITIALIZE MAPS */

  for (i = 0; i < size; i++){
    global_values[i]     = 0.0;
    global_active[i]     = 0;
    global_visited[i]    = 0;
    global_costs[i]      = robot_specifications->average_costs;
    global_costs2[i]     = robot_specifications->average_costs2;
    /*global_interior[i]   = 0;*/
  }

  for (k = 0; k < nNUMBER; k++){
    global_utility = global_utility_table[k];
    global_goal    = global_goal_table[k];
    global_succ    = global_succ_table[k];
    for (i = 0; i < size; i++){
      global_utility[i] = 0.0;
      /*      global_goal[i]    = 0;*/
      global_succ[i]    = -1;
    }
  }

  global_utility = global_utility_table[NUMBER];
  global_goal    = global_goal_table[NUMBER];
  global_succ    = global_succ_table[NUMBER];

}





/************************************************************************
 *
 *   NAME:         change_table
 *                 
 *   FUNCTION:     changes the internal utility table and NUMBER
 *                 
 *   PARAMETERS:   int  NEW_NUMBER
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void change_table(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		  int NEW_NUMBER,
		  PROGRAM_STATE_PTR program_state)
{
  if (NEW_NUMBER < 0 || NEW_NUMBER >= nNUMBER)
    printf("ERROR: Cannot change to table %d.\n", NEW_NUMBER);

  else{
    NUMBER         = NEW_NUMBER;
    global_utility = global_utility_table[NUMBER];
    global_goal    = global_goal_table[NUMBER];
    global_succ    = global_succ_table[NUMBER];
    if (program_state->graphics_initialized)
      G_change_matrix(UTILITY, global_utility, NULL, -1, -1);
    robot_specifications->min_plan_index_x = 
      robot_specifications->min_display_index_x;
    robot_specifications->max_plan_index_x = 
      robot_specifications->max_display_index_x;
    robot_specifications->min_plan_index_y = 
      robot_specifications->min_display_index_y;
    robot_specifications->max_plan_index_y = 
      robot_specifications->max_display_index_y;
    if (NEW_NUMBER == 0)
      BEST_NUMBER = 0;

  }
}











/************************************************************************
 *
 *   NAME:         main 
 *                 
 *   FUNCTION:     main loop - checks for mouse events and/or tcx events
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



main(int argc, char **argv)
{
  struct timeval read_begin_time;
  struct timeval read_end_time;
  long int sleep_duration;
  ROBOT_SPECIFICATIONS     robot_specifications_data;
  ROBOT_STATE              robot_state_data;
  PROGRAM_STATE            program_state_data;
  ACTION_TYPE              action_data;
  ALL_TYPE                 all_data;
  ROBOT_SPECIFICATIONS_PTR robot_specifications = &robot_specifications_data;
  ROBOT_STATE_PTR          robot_state          = &robot_state_data;
  PROGRAM_STATE_PTR        program_state        = &program_state_data;
  ACTION_PTR               action               = &action_data;
  int wait, i;

  /* add some parameter files */
  bParamList = bParametersAddFile(bParamList, "etc/beeSoft.ini");
  
  /* add some enviroment variables */
  bParamList = bParametersAddEnv(bParamList, "", "TCXHOST");
  
  /* add command line arguements */
  bParamList = bParametersAddArray(bParamList, "", argc, argv);
  
  /* Fill the global parameter structure */
  bParametersFillParams(bParamList);
  
  all = &all_data;
  check_commandline_parameters(argc, argv, program_state);
  init_program(program_state, robot_specifications, robot_state, action, all);
  if (!load_parameters(PLAN_INIT_NAME, 1, robot_specifications))
    exit(1);
  allocate_maps(robot_state, program_state, robot_specifications);
  init_graphics(robot_state, program_state, robot_specifications);
  init_tcx(program_state);
  connect_to_BASE(program_state);
  if ( program_state->use_map) connect_to_MAP(robot_specifications, program_state);
#ifdef UNIBONN
  connect_to_SPEECH(program_state);
#endif  


  program_state->busy = 0;
  

  do{

    if (program_state->warmer_colder_game &&
	program_state->program_initialized)
      search_for_goal_points(robot_specifications, program_state, 
			     robot_state, action);
  
    if (!program_state->base_connected)
      connect_to_BASE(program_state);
    if (program_state->use_map && !program_state->map_connected)
      connect_to_MAP(robot_specifications, program_state);



    if (program_state->send_automatic_update)
      send_automatic_status_update(NULL);


    if ((robot_specifications->max_plan_index_x <=
	 robot_specifications->min_plan_index_x ||
	 robot_specifications->max_plan_index_y <=
	 robot_specifications->min_plan_index_y) &&
	program_state->graphics_initialized){
      if (PLAN_debug)
	gettimeofday(&read_begin_time, NULL);
      block_waiting_time.tv_sec  = 1;
      block_waiting_time.tv_usec = 0;
      (void) block_wait(&block_waiting_time, program_state->tcx_initialized,
			program_state->graphics_initialized &&
			program_state->use_graphics);   
      if (PLAN_debug){
	gettimeofday(&read_end_time, NULL);
	sleep_duration = read_end_time.tv_usec - read_begin_time.tv_usec;
	sleep_duration += 1000000 
	  * (read_end_time.tv_sec - read_begin_time.tv_sec);
	fprintf(stderr, "PLAN:main:skipping = %f sec\n",
		(float)sleep_duration/1000000.0);
      }
    }
    
    
    if (program_state->use_tcx){
      TCX_waiting_time.tv_sec = 0;
      TCX_waiting_time.tv_usec = 0;
      tcxRecvLoop((void *) &TCX_waiting_time);
    }

    program_state->busy = 0;

    if (program_state->graphics_initialized)
      do ; while (mouse_test_loop(robot_state, program_state, 
				  robot_specifications, action));
    if (!program_state->base_connected)
      add_exploration_goal(robot_specifications, program_state,
			   robot_state);/*!*/

    

    if (program_state->program_initialized && 
	!program_state->warmer_colder_game){
      if (program_state->reset_descending_utilities_flag)
	reset_descending_utilities(robot_specifications, program_state, 
				   robot_state);    
      dynamic_programming(robot_specifications, program_state, robot_state);
      /*fprintf(stderr, "+");*/
    }


  } while (program_state->quit == 0);

  tcx_base_stop(program_state);
  
}


/* -------- FILE ENDS HERE ----------------- */



/************************************************************************
 *
 *   NAME:         
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *                 
 *                 
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


