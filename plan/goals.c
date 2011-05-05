
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
#include "o-graphics.h"

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




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



/************************************************************************
 *
 *   NAME:         count_goals_and_reset_utilities
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



void count_goals_and_reset_utilities(PROGRAM_STATE_PTR  program_state,
				     ROBOT_SPECIFICATIONS_PTR 
				     robot_specifications,
				     ROBOT_STATE_PTR robot_state)
{
  int x, y, index, num;

  /* ======================>  count goals */
  /*  num = 0;
      for (x = robot_specifications->min_display_index_x;
      x < robot_specifications->max_display_index_x; x++){
      index = x * robot_specifications->global_map_dim_y + 
      robot_specifications->min_display_index_y;
      for (y = robot_specifications->min_display_index_y;
      y < robot_specifications->max_display_index_y; y++, index++)
      if (global_goal[index])
      num++;
      }
      */

  program_state->busy++;

  num = G_return_num_markers(GOALS[NUMBER], 0);
  
  if (PLAN_verbose)
    printf("GOALS ACTIVE: %d", num);


  /* +++++++ insert here the reestimation of the _plan_ and _display_ */


  /* ======================>  reset utilities, if nesessary */
  if ( num < robot_specifications->number_active_goals[NUMBER] ||
      (num > 0  && !program_state->goal_modus)){

    if (num == 0){
      if (PLAN_verbose)
	printf(" ---> exploration mode 1 (initialize global_util)\n");
      if (program_state->goal_modus)
	program_state->send_automatic_update = 1;
      program_state->goal_modus = 0;
    }
    else if (program_state->actual_map_number != 0){
      if (PLAN_verbose)
	printf(" ---> exploration mode 2 (%d) (initialize global_util)\n",
	       program_state->actual_map_number);
      if (program_state->goal_modus)
	program_state->send_automatic_update = 1;
      program_state->goal_modus = 0;
    }
    else{
      if (PLAN_verbose)
	printf(" ---> path planning mode (initialize global_util)\n");
      if (!program_state->goal_modus)
	program_state->send_automatic_update = 1;
      program_state->goal_modus = 1;
    }
    if (PLAN_verbose)
      fflush(stdout);

    if (!program_state->goal_modus){ /* pure exploration mode */
      for (x = robot_specifications->min_display_index_x;
	   x < robot_specifications->max_display_index_x; x++){
	index = x * robot_specifications->global_map_dim_y + 
	  robot_specifications->min_display_index_y;
	for (y = robot_specifications->min_display_index_y;
	     y < robot_specifications->max_display_index_y; y++, index++){
	  if (global_explored[index]){
	    global_utility[index] = 0.0;
	    global_succ[index] = -1;
	  }
	  else{
	    global_utility[index] =
	      exploration_utility_factor(x, y, 
					 robot_specifications,
					 robot_state,
					 program_state)
		* global_costs[index];
	    global_succ[index]    = -1;
	  }
	}	    
      }
    }
    else{			/* goal hunting mode */
      for (x = robot_specifications->min_display_index_x;
	   x < robot_specifications->max_display_index_x; x++){
	index = x * robot_specifications->global_map_dim_y + 
	  robot_specifications->min_display_index_y;
	for (y = robot_specifications->min_display_index_y;
	     y < robot_specifications->max_display_index_y; y++, index++){
	  if (global_goal[index]){
	    global_utility[index] = global_costs[index];
	    global_succ[index] = -1;
	  }
	  else{
	    global_utility[index] = 0.0;
	    global_succ[index] = -1;
	  }
	}	    
      }
    }
    robot_specifications->number_active_goals[NUMBER] = num;
    if (PLAN_verbose)
      printf("\n");
    fflush(stdout);

    robot_specifications->min_plan_index_x =
      robot_specifications->min_display_index_x;
    robot_specifications->max_plan_index_x =
      robot_specifications->max_display_index_x;
    robot_specifications->min_plan_index_y =
      robot_specifications->min_display_index_y;
    robot_specifications->max_plan_index_y =
      robot_specifications->max_display_index_y;
    


  }
  else{
    robot_specifications->number_active_goals[NUMBER] = num;
    reset_descending_utilities(robot_specifications, program_state, 
			       robot_state); 
  }
  search_for_inconsistencies_in_succ(robot_specifications, "reset utils");
}      
 

/************************************************************************
 *
 *   NAME:         find_nearest_goal()
 *                  
 *   FUNCTION:     finds nearest goal, returns distance and name
 *                 
 *   PARAMETERS:   x, y = position, 
 *                 
 *   RETURN-VALUE: 0, if there is no goal at all
 *                 
 ************************************************************************/

 
int find_nearest_goal(float robot_x, float robot_y, float *dist, int *name,
		      float *goal_x, float *goal_y)
{
  int i, n, number;
  float g_dist, g_x, g_y;

  *dist = -1.0;

  n = G_return_num_markers(GOALS[NUMBER], 0);	/* count all visible goals */
    for (i = 0; i < n ; i++){
      G_return_marker_coordinates(GOALS[NUMBER], i, &g_x, &g_y, &number);
      g_dist = sqrt(((g_x - robot_x) * (g_x - robot_x)) 
		    + ((g_y - robot_y) * (g_y - robot_y)));
      if (g_dist < *dist || *dist < -0.5){
	*goal_x = g_x;
	*goal_y = g_y;
	*name   = number;
	*dist   = g_dist;
      }
    }
  
  if (*dist < -0.5)
    return 0;
  else
    return 1;
}


/************************************************************************
 *
 *   NAME:         check_if_goal_reached()
 *                  
 *   FUNCTION:     checks, if one (or multiple) goals have been reached.
 *                 removes those goals
 *
 *   PARAMETERS:   x, y = position, 
 *                 
 *   RETURN-VALUE: number of goals that were removed
 *                 
 ************************************************************************/


int check_if_goal_reached(PROGRAM_STATE_PTR program_state,
			  ROBOT_STATE_PTR robot_state,
			  ROBOT_SPECIFICATIONS_PTR robot_specifications,
			  float robot_x, float robot_y, 
			  float min_dist)
{
  int found, name, count;
  float goal_x, goal_y, dist;

  count = 0;
  do{
    found = 0;
    if (find_nearest_goal(robot_x, robot_y, &dist, &name, &goal_x, &goal_y))

      if (dist <= min_dist){

	found = 1;
#ifdef UNIBONN
	tcx_speech_talk_text(program_state, "I reached my goal.");
#endif
	if (PLAN_verbose)
	  fprintf(stderr, "reached goal %d (%5.3f): %5.3f %5.3f (%5.3f %5.3f)\n",
		  name, dist, goal_x, goal_y, robot_x, robot_y);
	modify_goal_set(program_state, robot_state, robot_specifications,
			goal_x, goal_y, 0, name);
	if (robot_specifications->number_active_goals[NUMBER] == 0){
	  if (program_state->base_connected)
	    tcxSendMsg(BASE, "BASE_stop_robot", NULL);
	  program_state->autonomous = 0;
	  program_state->exploration = 0;
	  G_display_switch(AUTONOMOUS_BUTTON, program_state->autonomous);
	}
	count++;
      }
  }
  while (found);
  
  return count;
}


/************************************************************************
 *
 *   NAME:         modify_goal_set
 *                 
 *   FUNCTION:     adds/removes goals from the set of active goals.
 *                 
 *   PARAMETERS:   x, y = position, 
 *                 add: 1=add, 0=remove
 *                 name: name of the goal (only relevant for adding)
 *                 
 *   RETURN-VALUE: 0, if error  1, if success
 *                 
 ************************************************************************/


  
int modify_goal_set(PROGRAM_STATE_PTR program_state,
		    ROBOT_STATE_PTR robot_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications,
		    float x,
		    float y,
		    int add,	/* 1=set, 0=remove */
		    int name)	/* name of the goal [currently 1..20] */
{
  int goal_x, goal_y, goal_index;
  
  goal_x = ((int) (x / robot_specifications->resolution))
    + robot_specifications->autoshifted_int_x;
  goal_y = ((int) (y / robot_specifications->resolution))
    + robot_specifications->autoshifted_int_y;
  goal_index = goal_x * robot_specifications->global_map_dim_y + goal_y;
  
  if (add){		/* add a goal */
    if (!global_goal[goal_index]){ /* no goal at that point */
      if (program_state->graphics_initialized)
	G_add_marker(GOALS[NUMBER], x, y, name);
      if (goal_x >= 0 && goal_x < robot_specifications->global_map_dim_x &&
	  goal_y >= 0 && goal_y < robot_specifications->global_map_dim_y)
	global_goal[goal_index] = 1 + name;
      /*robot_specifications->number_active_goals[NUMBER] += 1;*/
      display_robot(robot_state, program_state);
      program_state->send_automatic_update = 1;
    }
    else{			/* goal already set */
      printf("WARNING: attempt to define multiple goals at a point.\n");
      return 0;
    }
  }
  
  else{			/* delete a goal */
    if (global_goal[goal_index]){ /* there is a goal at that point */
      if (program_state->graphics_initialized)
	G_delete_marker(GOALS[NUMBER], x, y);
      if (goal_x >= 0 && goal_x < robot_specifications->global_map_dim_x &&
	  goal_y >= 0 && goal_y < robot_specifications->global_map_dim_y)
	global_goal[goal_index] = 0;
      /* robot_specifications->number_active_goals[NUMBER] -= 1;*/
      display_all(-1, robot_state, robot_specifications, program_state);
      program_state->send_automatic_update = 1;
    }
    else{			/* there is no goal at that point */
      printf("WARNING: attempt to delete an undefined goal.\n");
      return 0;
    }
  }
  
  /* adjust internal planning borders */
  if (goal_x < robot_specifications->min_display_index_x)
    robot_specifications->min_display_index_x = goal_x;
  if (goal_y < robot_specifications->min_display_index_y)
    robot_specifications->min_display_index_y = goal_y;
  if (goal_x + 1 > robot_specifications->max_display_index_x)
    robot_specifications->max_display_index_x = goal_x + 1;
  if (goal_y + 1 > robot_specifications->max_display_index_y)
    robot_specifications->max_display_index_y = goal_y + 1;

  check_index(robot_specifications, 1);
  count_goals_and_reset_utilities(program_state, robot_specifications, 
				  robot_state);
  /* reset_descending_utilities(robot_specifications);    */
  search_for_inconsistencies_in_succ(robot_specifications, "modify goals");
  send_automatic_status_update(NULL);
  return 1;			/* success */
}




/************************************************************************
 *
 *   NAME:         remove_all_goals
 *                 
 *   FUNCTION:     removes aall current goals
 *                 
 *                 
 ************************************************************************/


  
void remove_all_goals(PROGRAM_STATE_PTR program_state,
		      ROBOT_STATE_PTR robot_state,
		      ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int x, y, index;
  
  G_clear_markers(GOALS[NUMBER]);
  
  
  for (x = robot_specifications->min_display_index_x;
       x < robot_specifications->max_display_index_x; x++)
    for (y = robot_specifications->min_display_index_y;
	 y < robot_specifications->max_display_index_y; y++){
      index = x * robot_specifications->global_map_dim_y + y;
      global_goal[index] = 0;
    }
  
  
  display_all(-1, robot_state, robot_specifications, program_state);
  count_goals_and_reset_utilities(program_state, robot_specifications, 
				  robot_state);
  search_for_inconsistencies_in_succ(robot_specifications, "remove goals");
}



/************************************************************************
 *
 *   NAME:         check_if_point_reachable_by_straight_line
 *                 
 *   FUNCTION:     checks, if a point (x,y) can be reached from the
 *                 current robot position with a straight line
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: 1, if reachable, 0 if not
 *                 
 ************************************************************************/


int check_if_point_reachable_by_straight_line(float target_x, float target_y,
					      ROBOT_STATE_PTR robot_state,
					      ROBOT_SPECIFICATIONS_PTR 
					      robot_specifications)
{
  float from_x, from_y, to_x, to_y;
  float increment_x, increment_y;
  float x, y, d;
  float test_distance;
  int   free, test_index;
  
  /*fprintf(stderr, "check");*/

  /*
   * Calculate grip cell numbers, but in float
   */

  from_x =  (robot_state->x / robot_specifications->resolution)
    + ((float) robot_specifications->autoshifted_int_x);
  from_y =  (robot_state->y / robot_specifications->resolution)
    + ((float) robot_specifications->autoshifted_int_y);


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
  else{
    /*fprintf(stderr, "?");*/
    return 1;			/* we must be there already */
  }    

  /*
   * Check, if free!
   */

  free = 1;
  x = from_x;
  y = from_y;

  for (d = 0.0; d <= test_distance && free; d += 1.0){
    test_index = ((int) x) * robot_specifications->global_map_dim_y +((int) y);
    if (global_active[test_index] && 
	global_values[test_index] < robot_specifications->collision_threshold)
      free = 0;
    x += increment_x;
    y += increment_y;
  }

  /*fprintf(stderr, "%d ", free);*/
  return free;
}


/************************************************************************
 *
 *   NAME:         add_exploration_goal
 *                 
 *   FUNCTION:     tries to add a goal according to the exploration criterium
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: 1, if replanning necessary
 *                 
 ************************************************************************/


static float expl_goal_x;
static float expl_goal_y;
static int   expl_goal_def = 0;
static float expl_best_goal_dist;
static float max_dist_increase = 30.0;

int add_exploration_goal(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			  PROGRAM_STATE_PTR program_state,
			  ROBOT_STATE_PTR robot_state)
{
  ACTION_TYPE              local_action_data;
  ACTION_PTR               local_action               = &local_action_data;
  int generate_new_goal;


  if (!program_state->autonomous || !program_state->exploration )
    return 0;



  generate_new_goal = 0;

  
  /*
   * generate a goal if there is none
   */

  if (robot_specifications->number_active_goals[BEST_NUMBER] == 0)
    generate_new_goal = 1;
  

  if (expl_goal_def && 
      !check_if_point_free(expl_goal_x, expl_goal_y, robot_specifications))
    generate_new_goal = 1;


  /*
   * check the current distance, and generate a new goal if 
   * the distance is too large
   */

  
  generate_action(robot_specifications, program_state, robot_state,
		  local_action, 1, 0);

  if (!generate_new_goal){
    if (local_action->goal_dist < expl_best_goal_dist)
      expl_best_goal_dist = local_action->goal_dist;
    
    else if (local_action->goal_dist > expl_best_goal_dist + max_dist_increase)
      generate_new_goal = 1;
  }

  /*
   * Return, if we don't need to modify the goal set.
   */

  if (!generate_new_goal)
    return 0;

  /*
   * Now, if we have to define a new goal, we will first have to delete
   * previous goals.
   */

  if (expl_goal_def && 
      robot_specifications->number_active_goals[BEST_NUMBER] > 0){
    modify_goal_set(program_state, robot_state, robot_specifications,
		    expl_goal_x, expl_goal_y, 0, 1);
    expl_goal_def = 0;
  }
  
  /*putc(7, stderr);*/

  /*
   * generate a new goal point
   */

  generate_action(robot_specifications, program_state, robot_state,
		  local_action, 1, 0);

  if (local_action->no_plan){
    expl_goal_x = robot_state->x + 
		    1000.0 * cos(robot_state->orientation * M_PI / 180.0);
    expl_goal_y = robot_state->y + 
		    1000.0 * sin(robot_state->orientation * M_PI / 180.0);
  }
  else{
      expl_goal_x = local_action->goal_x;
      expl_goal_y = local_action->goal_y;
    }

  modify_goal_set(program_state, robot_state, robot_specifications,
		  expl_goal_x, expl_goal_y, 1, 1);

  expl_goal_def = 1;
  expl_best_goal_dist = local_action->goal_dist;

  generate_action(robot_specifications, program_state, robot_state,
		  local_action, 1, 0);

  /*  fprintf(stderr, "\t $$$$$$$$$$$$$$$ %6.4f (%d) $$$$$$$$$$\n", 
	  expl_best_goal_dist,
	  generate_new_goal); */

  /*
  if (program_state->graphics_initialized)
    G_display_markers(GOALS[NUMBER]);
    */
  return 1;
}




float evaluate_constraints(ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int i, from_x, from_y, to_x, to_y;
  float value, utility_from, utility_to, diff;
  
  value = 0.0;
  for (i = 0; i < n_constraints; i++){
    
    from_x = ((int) (constraints[i].from_x / robot_specifications->resolution))
      + robot_specifications->autoshifted_int_x;
    from_y = ((int) (constraints[i].from_y / robot_specifications->resolution))
      + robot_specifications->autoshifted_int_y;
    to_x = ((int) (constraints[i].to_x / robot_specifications->resolution))
      + robot_specifications->autoshifted_int_x;
    to_y = ((int) (constraints[i].to_y / robot_specifications->resolution))
      + robot_specifications->autoshifted_int_y;
    
    if (from_x >= 0 && from_x < robot_specifications->global_map_dim_x &&
	from_y >= 0 && from_y < robot_specifications->global_map_dim_y &&
	to_x >= 0 && to_x < robot_specifications->global_map_dim_x &&
	to_y >= 0 && to_y < robot_specifications->global_map_dim_y){

      utility_from = global_utility[from_x
				    * robot_specifications->global_map_dim_y 
				    + from_y];
      utility_to = global_utility[to_x
				  * robot_specifications->global_map_dim_y 
				  + to_y];
      if (utility_from > 0.0)
	diff = utility_to / utility_from;
      else
	diff = utility_to;

      if (constraints[i].type == 1){ /* pos. constraint */
	value += diff;
	if (diff > 1.0)
	  value += 1000.0;
      }
      else{			/* neg. constraint */
	value -= diff;
	if (diff < 1.0)
	  value += 1000.0;
      }

    }
  }
  return value;
}



#define probability_mutation 0.7 /*!*/

void create_new_goal_point(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		      float *x, 
		      float *y) /* GENETIC ALGORITHM HERE */
{
  float goals_x[max_nNUMBER], goals_y[max_nNUMBER], g_x, g_y, lambda;
  int   n_goals, number, n, i, goal_number, index1, index2;
  int   int_x, int_y;

  n_goals = 0;
  for (number = 1; number < nNUMBER; number++){
    n = G_return_num_markers(GOALS[number], 1);
    for (i = 0; i < n && n_goals < max_nNUMBER; i++){
      G_return_marker_coordinates(GOALS[number], i, &g_x, 
				  &g_y, &goal_number);
      goals_x[n_goals] = g_x;
      goals_y[n_goals] = g_y;
      n_goals++;
    }
  }
  printf("-----> %d goals total found\n", n_goals);
  

  if (n_goals < 2 || RAND_POS() <= probability_mutation){/* apply "mutation" */
    int_x = (int) (RAND_POS() 
		   * ((float) (robot_specifications->max_display_index_x 
			       - robot_specifications->min_display_index_x))
		   + ((float) robot_specifications->min_display_index_x));
    int_y = (int) (RAND_POS() 
		   * ((float) (robot_specifications->max_display_index_y 
			       - robot_specifications->min_display_index_y))
		   + ((float) robot_specifications->min_display_index_y));
    *x = ((float) (int_x - robot_specifications->autoshifted_int_x))
      * robot_specifications->resolution;
    *y = ((float) (int_y - robot_specifications->autoshifted_int_y))
      * robot_specifications->resolution;
    
  }
  else{				/* apply "crossover" */
    if (n_goals == 2){
      index1 = 0;
      index2 = 1;
    }
    else{
      index1 = (int) (RAND_POS() * ((float) n_goals));
      do
	index2 = (int) (RAND_POS() * ((float) n_goals));
      while (index2 == index1);
    }

    lambda = RAND_POS();
    *x = (lambda * goals_x[index1]) + ((1.0 - lambda) * goals_x[index2]);
    lambda = RAND_POS();
    *y = (lambda * goals_y[index1]) + ((1.0 - lambda) * goals_y[index2]);
  }
}






int search_for_goal_points(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			    PROGRAM_STATE_PTR program_state,
			    ROBOT_STATE_PTR robot_state,
			    ACTION_PTR action)
{
  int i, k, n, number, goal_number, int_goal_x, int_goal_y, int_goal_index;
  float worst = 0.0, best = 0.0, goal_x, goal_y;
  float rank[max_nNUMBER];
  int worst_index = -1, best_index = -1;


  for (number = 1; number < nNUMBER; number++){
    change_table(robot_specifications, number, program_state);	/* sets NUMBER */
    n = G_return_num_markers(GOALS[NUMBER], 1);
    if (n <= 0){		/* create a new goal, if none known! */
      create_new_goal_point(robot_specifications, &goal_x, &goal_y);

      G_add_marker(GOALS[NUMBER], goal_x, goal_y, NUMBER);
      
      for (k = 0; k < robot_specifications->global_map_dim_x *
	   robot_specifications->global_map_dim_y; k++){
	global_goal[k]    = 0;
	global_utility[k] = 0.0;
	global_succ[k]    = -1;
      }
      int_goal_x = ((int) (goal_x / robot_specifications->resolution))
	+ robot_specifications->autoshifted_int_x;
      int_goal_y = ((int) (goal_y / robot_specifications->resolution))
	+ robot_specifications->autoshifted_int_y;
      int_goal_index = int_goal_x * robot_specifications->global_map_dim_y 
	+ int_goal_y;
      global_goal[int_goal_index] = 1;
      robot_specifications->number_active_goals[NUMBER] = 0; /* trick! */
      count_goals_and_reset_utilities(program_state, robot_specifications, 
				  robot_state);
      if (program_state->graphics_initialized)
	G_display_markers(GOALS[NUMBER]);
    }
    

    check_index(robot_specifications, 1);
    reset_descending_utilities(robot_specifications, program_state, 
			       robot_state);    
    
    do{
      if (program_state->use_tcx) tcxRecvLoop((void *) &TCX_waiting_time);
      
      if (program_state->graphics_initialized)
	mouse_test_loop(robot_state, program_state, robot_specifications, 
			action);
      
      if (program_state->program_initialized)
	dynamic_programming(robot_specifications, program_state, 
			    robot_state);
    }
    while (robot_specifications->max_plan_index_x >
	   robot_specifications->min_plan_index_x &&
	   robot_specifications->max_plan_index_y >
	   robot_specifications->min_plan_index_y);
    
    rank[NUMBER] = evaluate_constraints(robot_specifications);
    printf("%d", NUMBER);
    fflush(stdout);
    
  }

  fprintf(stderr, "\n-----> HERE MIGHT BE A BUG: START WITH 0!!!\n\n");
  for (i = 1; i < nNUMBER; i++)
    if (rank[i] < worst || i == 1){
      worst_index = i;
      worst = rank[i];
    }
  
  for (i = 1; i < nNUMBER; i++)
    if (rank[i] > best || i == 1){
      best_index = i;
      best = rank[i];
    }

  printf("RANKS:");
  for (i = 1; i < nNUMBER; i++){
    printf(" %g", rank[i]);
    if (i == best_index)
      printf("(best)");
    if (i == worst_index)
      printf("(worst)");
  }
  printf("\n");
  
  if (program_state->graphics_initialized){
    G_undisplay_markers(GOALS[worst_index], -1, C_STEELBLUE4);
    G_clear_markers(GOALS[worst_index]);
  }

/*
  for (number = 1; i < nNUMBER; i++){
    n = G_return_num_markers(GOALS[number], 1);
    for (i = 0; i < n; i++){
      G_return_marker_coordinates(GOALS[number], 0, &goal_x, &goal_y,
				  &goal_number);
      G_delete_marker(GOALS[number], goal_x, goal_y);
      G_add_marker(GOALS[number], goal_x, goal_y, number);
    }
    G_display_markers(GOALS[number]);
  }
*/

  BEST_NUMBER = best_index;
  change_table(robot_specifications, BEST_NUMBER, program_state);
  if (program_state->graphics_initialized){
    G_activate(UTILITY);
    display_all(DISPLAY_UTILITY_BUTTON, robot_state, robot_specifications,
		program_state);
    G_deactivate(UTILITY);
  }

/*
  n = G_return_num_markers(GOALS[best_index], 1);
  for (i = 0; i < n; i++){
    G_return_marker_coordinates(GOALS[best_index], i, &goal_x, &goal_y,
				&goal_number);
    G_delete_marker(GOALS[best_index], goal_x, goal_y);
    G_add_marker(GOALS[best_index], goal_x, goal_y, 0);
  }
  G_display_markers(GOALS[best_index]);
*/
}

