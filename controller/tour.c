
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





#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include "all.h"



#ifdef TOURGUIDE_VERSION


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define TOUR_ITEM_TIMEOUT 10.0	/* in sec's, set to a negative value
				 * to disable timeouts */

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define MSG_xylophon                   1     /* Xylophon */
#define MSG_gong                       0     /* GONG */
#define MSG_fanfare                    4     /* Cromwell-Fanfare */
#define MSG_lok                        11    /* Lok */


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define MAX_TOUR_ITEMS (MAX_NUMBER_TOUR_GOALS)

typedef struct{
  int   number;
  int   tour_number;
  float recording_x;
  float recording_y;
  float recording_orientation;
  float object_x;
  float object_y;
  int   direct;
  int   condition;
  int   action;
  int   last_tour_item;
  int   text_1_number;
  int   text_2_number;
  int   text_3_number;
  int   text_4_number;
  int   text_5_number;
  int   action_outcome;
} tour_item_type, *tour_item_ptr;

tour_item_type tour[MAX_TOUR_ITEMS];

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


struct timeval com_time = {0, 0};

#define TOUR_FILE_NAME "tour.dat"


struct timeval goal_reached_time;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



/************************************************************************
 *
 *   NAME:         start_learning_tour()
 *                 
 *   FUNCTION:     starts learning a tour
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void 
start_learning_tour(ALL_PARAMS)
{
  int i;
/*
  if (!program_state->tcx_connected_to_BASE){
    fprintf(stderr, "ERROR: Cannot learn a tour without BASE.\n");
    return;
  }

*/
  if (!program_state->tcx_connected_to_BUTTONS){
    fprintf(stderr, "ERROR: Cannot learn a tour without BUTTONS.\n");
    return;
  }

  if (program_state->autonomous_mode){
    fprintf(stderr, "ERROR: Cannot learn a tour when in autonomous_mode.\n");
    return;
  }

  if (program_state->giving_tour){
    fprintf(stderr, "ERROR: Cannot learn a tour when in program_state->giving_tour.\n");
    return;
  }


  program_state->learning_tour = 1; /* okay, we are learning! */
  G_display_switch(TOUR_LEARN_BUTTON, 1);

  CD_play_track(ALL, MSG_xylophon, -1, -1);

  G_clear_markers(PATH);

      


  for (i = 0; i < MAX_NUM_TOURS; i++){
    G_clear_markers(TOUR_GOALS[i]);
    G_clear_markers(TOUR_OBJECTS[i]);
  }
    display_global_robot(ALL);

  robot_state->red_light_status         = BUTTON_LIGHT_STATUS_FLASHING;
  robot_state->yellow_light_status      = BUTTON_LIGHT_STATUS_OFF;
  robot_state->green_light_status       = BUTTON_LIGHT_STATUS_OFF;
  robot_state->blue_light_status        = BUTTON_LIGHT_STATUS_ON;
  set_buttons(ALL, -1);

  program_state->num_tour_goals             = 0;

  printf("\n### INSTRUCTIONS: Push red button for every goal along a tour.\n");
  printf("### Push theblue button to terminate the tour.\n\n");
}




/************************************************************************
 *
 *   NAME:         add_tour_goal()
 *                 
 *   FUNCTION:     adds a specific location as a tour goal.
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


static float prev_tour_goal_x = 0.0, prev_tour_goal_y = 0.0;
static float prev_tour_goal_orientation = 0.0;


void 
add_tour_goal(ALL_PARAMS)
{
  int i;

  if (program_state->num_tour_goals > 0 &&
      robot_state->x == prev_tour_goal_x &&
      robot_state->y == prev_tour_goal_y &&
      prev_tour_goal_orientation == robot_state->orientation){
    fprintf(stderr, "WARNING: Goal identical to the last one.\n");
    return;
  }

  CD_play_track(ALL, MSG_gong, -1, -1);



  tour[program_state->num_tour_goals].number                = 
    program_state->num_tour_goals;
  tour[program_state->num_tour_goals].tour_number           = 0;
  tour[program_state->num_tour_goals].recording_x           = robot_state->x;
  tour[program_state->num_tour_goals].recording_y           = robot_state->y;
  tour[program_state->num_tour_goals].recording_orientation = 
    robot_state->orientation;
  tour[program_state->num_tour_goals].object_x              = 
    robot_state->x + (100.0 * cos(robot_state->orientation * M_PI / 180.0));
  tour[program_state->num_tour_goals].object_y              = 
    robot_state->y + (100.0 * sin(robot_state->orientation * M_PI / 180.0));
  tour[program_state->num_tour_goals].direct                = 0;
  tour[program_state->num_tour_goals].condition             = -1;
  tour[program_state->num_tour_goals].action                = 0;
  tour[program_state->num_tour_goals].text_1_number         = -1;
  tour[program_state->num_tour_goals].text_2_number         = -1;
  tour[program_state->num_tour_goals].text_3_number         = -1;
  tour[program_state->num_tour_goals].text_4_number         = -1;
  tour[program_state->num_tour_goals].text_5_number         = -1;
  program_state->tour_defined[tour[program_state->num_tour_goals].tour_number]                            = 1;
  tour[program_state->num_tour_goals].last_tour_item = 1;
  if (program_state->num_tour_goals > 0)
    tour[program_state->num_tour_goals - 1].last_tour_item = 0;

  G_add_marker(TOUR_GOALS[tour[program_state->num_tour_goals].tour_number], 
	       tour[program_state->num_tour_goals].recording_x,
	       tour[program_state->num_tour_goals].recording_y,
	       tour[program_state->num_tour_goals].tour_number);

  G_add_marker(TOUR_OBJECTS[tour[program_state->num_tour_goals].tour_number], 
	       tour[program_state->num_tour_goals].object_x,
	       tour[program_state->num_tour_goals].object_y,
	       tour[program_state->num_tour_goals].tour_number);

  prev_tour_goal_x = tour[program_state->num_tour_goals].recording_x;
  prev_tour_goal_y = tour[program_state->num_tour_goals].recording_y;

  prev_tour_goal_orientation = robot_state->orientation;

  G_display_switch(GLOBAL_ROBOT_BACKGROUND, 0);
  G_display_matrix(OCCUPANCY_MAP);
  for (i = 0; i < MAX_NUM_TOURS; i++){
    G_display_markers(TOUR_GOALS[i]);
    G_display_markers(TOUR_OBJECTS[i]);
  }

  program_state->num_tour_goals++;

  printf("### TOUR GOAL ADDED.\n");


  if (program_state->num_tour_goals == MAX_TOUR_ITEMS)
    finish_learning_tour(ALL);
}




/************************************************************************
 *
 *   NAME:         finish_learning_tour()
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void 
finish_learning_tour(ALL_PARAMS)
{
  int i;

  if (!program_state->learning_tour){
    fprintf(stderr, "STRANGE: Wasn't even learning a tour.\n");
    return;
  }

  CD_play_track(ALL, MSG_fanfare, -1, -1);

  program_state->learning_tour = 0; /* okay, we finished learning! */
  G_display_switch(TOUR_LEARN_BUTTON, 0);

  robot_state->red_light_status         = BUTTON_LIGHT_STATUS_OFF;
  robot_state->yellow_light_status      = BUTTON_LIGHT_STATUS_OFF;
  robot_state->green_light_status       = BUTTON_LIGHT_STATUS_OFF;
  robot_state->blue_light_status        = BUTTON_LIGHT_STATUS_OFF;

  set_buttons(ALL, -1);

  printf("\n### TOUR LEARNING COMPLETED, %d GOALS.\n\n", 
	 program_state->num_tour_goals);
}




/************************************************************************
 *
 *   NAME:         start_giving_tour()
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void 
start_giving_tour(ALL_PARAMS)
{
  int i;

  fprintf(stderr, "TOUR: start_giving_tour().\n");



  if (!program_state->tcx_connected_to_BASE){
    fprintf(stderr, "ERROR: Cannot give a tour without BASE.\n");
    return;
  }
  if (!program_state->tcx_connected_to_MAP){
    fprintf(stderr, "ERROR: Cannot give a tour without MAP.\n");
    return;
  }
  if (!program_state->tcx_connected_to_PLAN){
    fprintf(stderr, "ERROR: Cannot give a tour without PLAN.\n");
    return;
  }
#ifdef UNIBONN
   if (!program_state->tcx_connected_to_CD){
    fprintf(stderr, "ERROR: Cannot give a tour without CD.\n");
    return;
  } 
#endif /* UNIBONN */
  if (!program_state->tcx_connected_to_BUTTONS){
    fprintf(stderr, "ERROR: Cannot give a tour without BUTTONS.\n");
    return;
  }

  if (program_state->autonomous_mode){
    fprintf(stderr, "ERROR: Cannot give a tour when in autonomous_mode.\n");
    return;
  }

  if (program_state->autonomous_mode){
    fprintf(stderr, "ERROR: Cannot learn a tour when in autonomous_mode.\n");
    return;
  }

  if (program_state->learning_tour){
    fprintf(stderr, "ERROR: Cannot learn a tour when in program_state->learning_tour.\n");
    return;
  }



  program_state->giving_tour = 1; /* okay, we are giving a tour, and wait! */

  program_state->tour_modus = 1; /* noone requested the robot yet */

  program_state->current_tour_goal = -1;

  program_state->current_tour = -1;

  program_state->actual_tour_text = 0;

  for (i = 0; i < program_state->num_tour_goals; i++)
    tour[i].action_outcome = 0;

  G_display_switch(TOUR_GIVE_BUTTON, 1);

  CD_play_track(ALL, 313, 0, 310);

  G_clear_markers(PATH);
  G_display_switch(GLOBAL_ROBOT_BACKGROUND, 0);
  G_display_matrix(OCCUPANCY_MAP);
  for (i = 0; i < MAX_NUM_TOURS; i++){
    G_display_markers(TOUR_GOALS[i]);
    G_display_markers(TOUR_OBJECTS[i]);
  }

  /*tcx_map_disable_map_update(ALL);*/
  clear_all_sensor_maps(ALL);


  if (program_state->tour_defined[0])
    robot_state->red_light_status         = BUTTON_LIGHT_STATUS_ON;
  else
    robot_state->red_light_status         = BUTTON_LIGHT_STATUS_OFF;

  if (program_state->tour_defined[1])
    robot_state->yellow_light_status         = BUTTON_LIGHT_STATUS_ON;
  else
    robot_state->yellow_light_status         = BUTTON_LIGHT_STATUS_OFF;

  if (program_state->tour_defined[2])
    robot_state->green_light_status         = BUTTON_LIGHT_STATUS_ON;
  else
    robot_state->green_light_status         = BUTTON_LIGHT_STATUS_OFF;

  if (program_state->tour_defined[3])
    robot_state->blue_light_status         = BUTTON_LIGHT_STATUS_ON;
  else
    robot_state->blue_light_status         = BUTTON_LIGHT_STATUS_OFF;

  set_buttons(ALL, -1);


  printf("\n### INSTRUCTIONS: Pick a button to select a tour.\n");
  printf("### Push two buttons to terminate the tour.\n\n");
}



/************************************************************************
 *
 *   NAME:         move_to_next_tour_goal()
 *                 
 *   FUNCTION:     initiates a move to a tour goal
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void 
move_to_next_tour_goal(ALL_PARAMS)
{
  float x, y;
  int type, condition_fulfiled, i, goal_nr;



  goal_nr = program_state->current_tour_goal + 1;

	  

  /*
   * check if this tour goal is a valid one (conditional)
   * otherwise go straight to the next one
   */
  
  condition_fulfiled = 0;
  while (((tour[goal_nr].condition >= 0 && !condition_fulfiled) ||
	  tour[goal_nr].tour_number != program_state->current_tour) &&
	 goal_nr < program_state->num_tour_goals){
    if (tour[goal_nr].tour_number != program_state->current_tour)
      goal_nr++;
    else{
      for (i = 0; i < goal_nr && !condition_fulfiled; i++)
	if (tour[i].number == tour[goal_nr].condition &&
	    tour[i].action_outcome)
	  condition_fulfiled = 1;
      if (!condition_fulfiled) goal_nr++;
    }
  }





  /*
   * check if any tour goal left
   */

 if (goal_nr >= program_state->num_tour_goals)
   stop_giving_tour(ALL, 1);


 
  else{

    program_state->current_tour_goal = goal_nr;
    program_state->tour_modus = 2;
    
    G_display_switch(TOUR_GIVE_BUTTON, 2);

    /* tcx_base_set_mode(DEFAULT_MODE, ALL); */
    
    clear_all_sensor_maps(ALL);    
    buttons_effect(ALL);

    if (tour[program_state->current_tour_goal].direct){
      reached_a_tour_goal(ALL);	/* direct approaching the goal */
      printf("### Moving to tour-goal number %d (direct).\n", goal_nr);
    }
    else{
      x    = tour[program_state->current_tour_goal].recording_x;
      y    = tour[program_state->current_tour_goal].recording_y;
      type = tour[program_state->current_tour_goal].number;
      tcx_remove_all_goals(ALL);
      tcx_plan_goal_message(x, y, type, 1, ALL); /* planning */
      usleep(500000);
      tcx_plan_start_autonomous(0, ALL);
      printf("### Moving to tour-goal number %d (indirect).\n", goal_nr);
    }

    program_state->actual_tour_text = 0;
    
    x    = tour[program_state->current_tour_goal].object_x;
    y    = tour[program_state->current_tour_goal].object_y;

    tcx_pantilt_track_point(ALL, x, y);

    CD_play_track(ALL, MSG_lok, -1, 309);


  }
}



/************************************************************************
 *
 *   NAME:         reached_a_tour_goal()
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void 
reached_a_tour_goal(ALL_PARAMS)
{
  float x, y;
  struct timeval this_time;
  float  time_difference;

  /*  fprintf(stderr, "reached_a_tour_goal: mode = %d\n",
	  program_state->tour_modus);*/

  if (program_state->tour_modus == 3){ /* was: approaching, now wait */
    gettimeofday(&this_time, NULL);
    time_difference = ((float) (this_time.tv_sec - com_time.tv_sec))
      + (((float) (this_time.tv_usec - com_time.tv_usec)) / 1000000.0);
    if (time_difference > 2.0){
      tcx_base_stop_robot(ALL);
      if (tour[program_state->current_tour_goal].text_1_number >= 0)
	CD_play_track(ALL, 
		      tour[program_state->current_tour_goal].text_1_number, 
		      0, -1);


      robot_state->red_light_status         = BUTTON_LIGHT_STATUS_OFF;
      robot_state->yellow_light_status      = BUTTON_LIGHT_STATUS_OFF;
      robot_state->green_light_status       = BUTTON_LIGHT_STATUS_OFF;
      robot_state->blue_light_status        = BUTTON_LIGHT_STATUS_OFF;
      
      if (program_state->current_tour == -1)
	fprintf(stderr, "INTERNAL ERROR: SHOULD KNOW WHAT TOUR WE ARE IN\n");
      else if (program_state->current_tour == 0)
	robot_state->red_light_status         = BUTTON_LIGHT_STATUS_ON;
      else if (program_state->current_tour == 1)
	robot_state->yellow_light_status      = BUTTON_LIGHT_STATUS_ON;
      else if (program_state->current_tour == 2)
	robot_state->green_light_status       = BUTTON_LIGHT_STATUS_ON;
      else
	robot_state->blue_light_status        = BUTTON_LIGHT_STATUS_ON;


      if (tour[program_state->current_tour_goal].action == 1){ /* ask track */
#ifdef UNI_BONN
	flow_request_info(ALL);
#else
	sensation->flow_result = 0;
	handle_flow_reply_in_tour(ALL);
#endif
      }
      set_buttons(ALL, -1);
      program_state->actual_tour_text = 1;
      G_display_switch(TOUR_GIVE_BUTTON, 1);

      tcx_plan_stop_autonomous(0, ALL);

      gettimeofday(&goal_reached_time, NULL);

      program_state->tour_modus = 1; /* okay, we'll wait */
      action->status          = 1; /* 1=new command issued */
      action->type            = 1; /* 1=stop */
      action->continuous_ping = 0; /* 0=ping not necessary */
      initiate_action(ALL);	/* stops the robot */
      printf("### Text.\n");
      
      if (tour[program_state->current_tour_goal].last_tour_item)
	stop_giving_tour(ALL, 1);
    }
    else{
      fprintf(stderr, " [2 sec] ");
      action->status          = 1; /* 1=new command issued */
      action->type            = 4; /* 4=absolute target coordinates */
      action->continuous_ping = 1; /* 1=refresh necessary */
    }
  }

  else if (program_state->tour_modus == 2){ /* was: planning, now approach */
    /*
     * Okay, let's stop the planner, and issue an action command for
     * picking up the object
     */
    tcx_plan_stop_autonomous(0, ALL);
    tcx_remove_all_goals(ALL);
    
    
    /* tcx_base_set_mode(APPROACH_OBJECT_MODE, ALL); */

    x    = tour[program_state->current_tour_goal].object_x;
    y    = tour[program_state->current_tour_goal].object_y;
    
    
    action->status          = 1; /* 1=new command issued */
    action->type            = 4; /* 4=absolute target coordinates */
    action->continuous_ping = 1; /* 1=refresh necessary */
    action->target_pos_x    = x;
    action->target_pos_y    = y;

    tcx_base_approach_two_points_absolute(1, ALL);

    tcx_plan_stop_autonomous(0, ALL);

    program_state->tour_modus = 3; /* okay, we'll wait */

    gettimeofday(&com_time, NULL);

    printf("### Fine motion now.\n");

  }

  else
    fprintf(stderr, "WARNING %d: How on earth did I get here???\n",
	    program_state->tour_modus);
}



/************************************************************************
 *
 *   NAME:         tourguide_check_for_timeout(ALL_PARAMS);
 *                 
 *   FUNCTION:     checks if robot timed out at a goal
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void
tourguide_check_for_timeout(ALL_PARAMS)
{
  struct timeval this_time;
  float  time_difference;

  if (program_state->current_tour < 0 ||
      program_state->tour_modus != 1 ||
      TOUR_ITEM_TIMEOUT < 0.0)
    return;

  gettimeofday(&this_time, NULL);

  time_difference = 
    ((float) (this_time.tv_sec - goal_reached_time.tv_sec))
    + (((float) (this_time.tv_usec 
		 - goal_reached_time.tv_usec))
       /  1000000.0);
  
  if (time_difference > TOUR_ITEM_TIMEOUT)
    move_to_next_tour_goal(ALL);
}



/************************************************************************
 *
 *   NAME:        next_text()
 *                 
 *   FUNCTION:     plays the next text
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void
next_text(ALL_PARAMS)
{
  int next_text = -1;

  if (program_state->actual_tour_text == 0)
    return;

  program_state->actual_tour_text++; /* next text */

  if (program_state->actual_tour_text == 2){
    CD_play_track(ALL, 
		  tour[program_state->current_tour_goal].text_2_number, 
		  -1, -1);
    next_text = tour[program_state->current_tour_goal].text_3_number;
  }
  else if (program_state->actual_tour_text == 3){
    CD_play_track(ALL, 
		  tour[program_state->current_tour_goal].text_3_number, 
		  -1, -1);
    next_text = tour[program_state->current_tour_goal].text_4_number;
  }
  else if (program_state->actual_tour_text == 4){
    CD_play_track(ALL, 
		  tour[program_state->current_tour_goal].text_4_number, 
		  -1, -1);
    next_text = tour[program_state->current_tour_goal].text_5_number;
  }
  else if (program_state->actual_tour_text == 5){
    CD_play_track(ALL, 
		  tour[program_state->current_tour_goal].text_5_number, 
		  -1, -1);
    next_text = -1;
  }
  else				/* no more information */
    CD_play_track(ALL, 263, 1, -1);

  if (next_text == -1){
    program_state->actual_tour_text = 10;


    if (program_state->tour_defined[0])
      robot_state->red_light_status         = BUTTON_LIGHT_STATUS_ON;
    else
      robot_state->red_light_status         = BUTTON_LIGHT_STATUS_OFF;
    
    if (program_state->tour_defined[1])
      robot_state->yellow_light_status         = BUTTON_LIGHT_STATUS_ON;
    else
      robot_state->yellow_light_status         = BUTTON_LIGHT_STATUS_OFF;
    
    if (program_state->tour_defined[2])
      robot_state->green_light_status         = BUTTON_LIGHT_STATUS_ON;
    else
      robot_state->green_light_status         = BUTTON_LIGHT_STATUS_OFF;
    
    if (program_state->tour_defined[3])
      robot_state->blue_light_status         = BUTTON_LIGHT_STATUS_ON;
    else
      robot_state->blue_light_status         = BUTTON_LIGHT_STATUS_OFF;
    
    set_buttons(ALL, -1);
  }
}



/************************************************************************
 *
 *   NAME:         stop_giving_tour()
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void 
stop_giving_tour(ALL_PARAMS, int restart)
{

  if (!program_state->giving_tour){
    fprintf(stderr, "STRANGE: Wasn't even lgiving a tour.\n");
    return;
  }

  CD_play_track(ALL, MSG_fanfare, -1, -1);

  program_state->giving_tour = 0; /* okay, we finished learning! */
  program_state->tour_modus = 0; /* tour over */
  /* tcx_base_set_mode(DEFAULT_MODE, ALL); */
  tcx_pantilt_stop_tracking(ALL);
  tcx_pantilt_move_absolute(ALL, -90.0, 0.0);  

  /*tcx_map_enable_map_update(ALL);*/
  tcx_plan_stop_autonomous(0, ALL);

  printf("\n### TOUR COMPLETED.\n\n");


  if (restart){
    start_giving_tour(ALL);
  }
  else{
    G_display_switch(TOUR_GIVE_BUTTON, 0);
    robot_state->red_light_status         = BUTTON_LIGHT_STATUS_OFF;
    robot_state->yellow_light_status      = BUTTON_LIGHT_STATUS_OFF;
    robot_state->green_light_status       = BUTTON_LIGHT_STATUS_OFF;
    robot_state->blue_light_status        = BUTTON_LIGHT_STATUS_OFF;
    set_buttons(ALL, -1);
  }
}




/************************************************************************
 *
 *   NAME:         save_tour()
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void 
save_tour(ALL_PARAMS)
{
  int i;
  FILE *iop;

   
  if ((iop = fopen(TOUR_FILE_NAME, "w")) == 0){
    fprintf(stderr, "ERROR: Could not open output file %s.\n", 
	    TOUR_FILE_NAME);
    return;
  }

  G_display_switch(TOUR_SAVE_BUTTON, 1);

  for (i = 0; i < program_state->num_tour_goals; i++){
    
    fprintf(iop, "ITEM %d\n", i);
    fprintf(iop, "\tnumber                  = %d\n", tour[i].number);
    fprintf(iop, "\ttour_number             = %d\n", tour[i].tour_number);
    fprintf(iop, "\trecording_x             = %g\n", tour[i].recording_x);
    fprintf(iop, "\trecording_y             = %g\n", tour[i].recording_y);
    fprintf(iop, "\trecording_orientation   = %g\n", tour[i].recording_orientation);
    fprintf(iop, "\tobject_x                = %g\n", tour[i].object_x);
    fprintf(iop, "\tobject_y                = %g\n", tour[i].object_y);
    fprintf(iop, "\tdirect                  = %d\n", tour[i].direct);
    fprintf(iop, "\tcondition               = %d\n", tour[i].condition);
    fprintf(iop, "\taction                  = %d\n", tour[i].action);
    fprintf(iop, "\ttext_1_number           = %d\n", tour[i].text_1_number);
    fprintf(iop, "\ttext_2_number           = %d\n", tour[i].text_2_number);
    fprintf(iop, "\ttext_3_number           = %d\n", tour[i].text_3_number);
    fprintf(iop, "\ttext_4_number           = %d\n", tour[i].text_4_number);
    fprintf(iop, "\ttext_5_number           = %d\n", tour[i].text_5_number);
    fprintf(iop, "\n");
  }

  fclose(iop);

  fprintf(stderr, "File %s successfully written.\n", TOUR_FILE_NAME);
  usleep(300000);

  G_display_switch(TOUR_SAVE_BUTTON, 0);

  return;
}




/************************************************************************
 *
 *   NAME:         load_tour()
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void 
load_tour(ALL_PARAMS)
{
  int i, int_value, number;
  float float_value;
  FILE *iop;
  int file_ended;
  char command[256];
  int verbose = 0;

  
  for (i = 0; i < MAX_NUM_TOURS; i++)
    program_state->tour_defined[i]          = 0;

  if ((iop = fopen(TOUR_FILE_NAME, "r")) == 0){
    fprintf(stderr, "ERROR: Could not open input file %s.\n", TOUR_FILE_NAME);
    return;
  }

  G_display_switch(TOUR_LOAD_BUTTON, 1);

  file_ended = 0;
  number = 0;
  
  do{
    if (fscanf(iop, "%s", command) == EOF)
      file_ended = 1;
    else if (strcmp(command, "ITEM")){
      fprintf(stderr, "Unexpected keyword: %s (instead of ITEM)\n", command);
      file_ended = 2;
    }

    if (!file_ended){
      if (fscanf(iop, "%d", &int_value) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (verbose)
	printf("ITEM: %d\n", int_value);
    }
      

    /* -------------------------------------------------- */
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "number")){
	fprintf(stderr, "Unexpected keyword: %s (instead of number)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "=")){
	fprintf(stderr, "Unexpected keyword: %s (instead of =)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%d", &int_value) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else
	tour[number].number = int_value;
    }

    /* -------------------------------------------------- */
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "tour_number")){
	fprintf(stderr, "Unexpected keyword: %s (instead of tour_number)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "=")){
	fprintf(stderr, "Unexpected keyword: %s (instead of =)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%d", &int_value) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (int_value < 0 || int_value >= MAX_NUM_TOURS){
	fprintf(stderr, "Invalid tour number encountered: %d.\n", int_value);
	file_ended = 2;
      }
      else if (number > 0 && int_value < tour[number-1].tour_number){
	fprintf(stderr, "Tours must be arranged in increasing order.\n");
	file_ended = 2;
      }
	
      else{
	tour[number].tour_number = int_value;
	program_state->tour_defined[int_value] = 1;
      }
    }

    /* -------------------------------------------------- */
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "recording_x")){
	fprintf(stderr, "Unexpected keyword: %s (instead of recording_x)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "=")){
	fprintf(stderr, "Unexpected keyword: %s (instead of =)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%f", &float_value) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else
	tour[number].recording_x = float_value;
    }



    /* -------------------------------------------------- */
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "recording_y")){
	fprintf(stderr, "Unexpected keyword: %s (instead of recording_y)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "=")){
	fprintf(stderr, "Unexpected keyword: %s (instead of =)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%f", &float_value) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else
	tour[number].recording_y = float_value;
    }


    /* -------------------------------------------------- */
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "recording_orientation")){
	fprintf(stderr, "Unexpected keyword: %s (instead of recording_orientation)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "=")){
	fprintf(stderr, "Unexpected keyword: %s (instead of =)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%f", &float_value) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else
	tour[number].recording_orientation = float_value;
    }



    /* -------------------------------------------------- */
    

    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "object_x")){
	fprintf(stderr, "Unexpected keyword: %s (instead of object_x)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "=")){
	fprintf(stderr, "Unexpected keyword: %s (instead of =)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%f", &float_value) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else
	tour[number].object_x = float_value;
    }
      


    /* -------------------------------------------------- */
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "object_y")){
	fprintf(stderr, "Unexpected keyword: %s (instead of object_y)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "=")){
	fprintf(stderr, "Unexpected keyword: %s (instead of =)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%f", &float_value) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else
	tour[number].object_y = float_value;
    }



    /* -------------------------------------------------- */


    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "direct")){
	fprintf(stderr, "Unexpected keyword: %s (instead of direct)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "=")){
	fprintf(stderr, "Unexpected keyword: %s (instead of =)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%d", &int_value) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else
	tour[number].direct = int_value;
    }

    /* -------------------------------------------------- */


    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "condition")){
	fprintf(stderr, "Unexpected keyword: %s (instead of condition)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "=")){
	fprintf(stderr, "Unexpected keyword: %s (instead of =)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%d", &int_value) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else
	tour[number].condition = int_value;
    }

    /* -------------------------------------------------- */


    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "action")){
	fprintf(stderr, "Unexpected keyword: %s (instead of action)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "=")){
	fprintf(stderr, "Unexpected keyword: %s (instead of =)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%d", &int_value) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else
	tour[number].action = int_value;
    }

    /* -------------------------------------------------- */


    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "text_1_number")){
	fprintf(stderr, "Unexpected keyword: %s (instead of text_1_number)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "=")){
	fprintf(stderr, "Unexpected keyword: %s (instead of =)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%d", &int_value) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else
	tour[number].text_1_number = int_value;
    }

    /* -------------------------------------------------- */
    

    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "text_2_number")){
	fprintf(stderr, "Unexpected keyword: %s (instead of text_2_number)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "=")){
	fprintf(stderr, "Unexpected keyword: %s (instead of =)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%d", &int_value) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else
	tour[number].text_2_number = int_value;
    }

    /* -------------------------------------------------- */

    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "text_3_number")){
	fprintf(stderr, "Unexpected keyword: %s (instead of text_3_number)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "=")){
	fprintf(stderr, "Unexpected keyword: %s (instead of =)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%d", &int_value) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else
	tour[number].text_3_number = int_value;
    }

    /* -------------------------------------------------- */

    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "text_4_number")){
	fprintf(stderr, "Unexpected keyword: %s (instead of text_4_number)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "=")){
	fprintf(stderr, "Unexpected keyword: %s (instead of =)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%d", &int_value) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else
	tour[number].text_4_number = int_value;
    }

    /* -------------------------------------------------- */
    

    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "text_5_number")){
	fprintf(stderr, "Unexpected keyword: %s (instead of text_5_number)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%s", command) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else if (strcmp(command, "=")){
	fprintf(stderr, "Unexpected keyword: %s (instead of =)\n",
		command);
	file_ended = 2;
      }
    }
    
    if (!file_ended){
      if (fscanf(iop, "%d", &int_value) == EOF){
	fprintf(stderr, "Unexpected end of file.\n");
	file_ended = 2;
      }
      else
	tour[number].text_5_number = int_value;
    }

    /* -------------------------------------------------- */

    if (!file_ended){
      if (verbose){
	printf("\tnumber                  = %d\n", tour[number].number);
	printf("\tnumber                  = %d\n", tour[number].tour_number);
	printf("\trecording_x             = %g\n", tour[number].recording_x);
	printf("\trecording_y             = %g\n", tour[number].recording_y);
	printf("\trecording_orientation   = %g\n", tour[number].recording_orientation);
	printf("\tobject_x                = %g\n", tour[number].object_x);
	printf("\tobject_y                = %g\n", tour[number].object_y);
	printf("\tdirect                  = %d\n", tour[number].direct);
	printf("\tcondition               = %d\n", tour[number].condition);
	printf("\taction                  = %d\n", tour[number].action);
	printf("\ttext_1_number           = %d\n", tour[number].text_1_number);
	printf("\ttext_2_number           = %d\n", tour[number].text_2_number);
	printf("\ttext_3_number           = %d\n", tour[number].text_3_number);
	printf("\ttext_4_number           = %d\n", tour[number].text_4_number);
	printf("\ttext_5_number           = %d\n", tour[number].text_5_number);
	printf("\n");
      }
      number++;
    }

  } while (!file_ended);

  fclose(iop);

  for (i = 0; i < MAX_NUM_TOURS; i++){
    G_clear_markers(TOUR_GOALS[i]);
    G_clear_markers(TOUR_OBJECTS[i]);
  }

  if (file_ended == 2)
    program_state->num_tour_goals             = 0;

  else{
    program_state->num_tour_goals = number;
    for (number = 0; number < program_state->num_tour_goals; number++){
      G_add_marker(TOUR_GOALS[tour[number].tour_number], 
		   tour[number].recording_x,
		   tour[number].recording_y, tour[number].tour_number);
      G_add_marker(TOUR_OBJECTS[tour[number].tour_number], 
		   tour[number].object_x,
		   tour[number].object_y, tour[number].tour_number);
    }

    for (i = 0; i < program_state->num_tour_goals; i++)
      if (i == program_state->num_tour_goals - 1 ||
	  tour[i].tour_number != tour[i+1].tour_number)
	tour[i].last_tour_item = 1;
      else
	tour[i].last_tour_item = 0;
  }

  G_display_switch(GLOBAL_ROBOT_BACKGROUND, 0);
  G_display_matrix(OCCUPANCY_MAP);
  for (i = 0; i < MAX_NUM_TOURS; i++){
    G_display_markers(TOUR_GOALS[i]);
    G_display_markers(TOUR_OBJECTS[i]);
  }
    
  G_display_switch(TOUR_LOAD_BUTTON, 0);

}





/************************************************************************
 *
 *   NAME:         handle_button_in_tour()
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void 
handle_button_in_tour(ALL_PARAMS)
{
  int i;
  int something_changed;

  something_changed =
    sensation->red_button_changed ||
    sensation->yellow_button_changed ||
    sensation->green_button_changed ||
    sensation->blue_button_changed;



  if (program_state->learning_tour){
    if (sensation->red_button_status && sensation->red_button_changed)
      add_tour_goal(ALL);
      
    else if (sensation->blue_button_status && sensation->blue_button_changed)
      finish_learning_tour(ALL);
  } 



  if (program_state->giving_tour){

    i = 0;
    if (sensation->red_button_status) i++;
    if (sensation->yellow_button_status) i++;
    if (sensation->green_button_status) i++;
    if (sensation->blue_button_status) i++;

    if (i >= 2)
      stop_giving_tour(ALL, 1);

    
    else if (program_state->tour_modus == 1 &&
	     ((program_state->current_tour == -1 &&
	       something_changed) ||
	      (program_state->current_tour == 0 &&
	       sensation->red_button_status && 
	       sensation->red_button_changed) ||
	      (program_state->current_tour == 1 &&
	       sensation->yellow_button_status && 
	       sensation->yellow_button_changed) ||
	      (program_state->current_tour == 2 &&
	       sensation->green_button_status && 
	       sensation->green_button_changed) ||
	      (program_state->current_tour == 3 &&
	       sensation->blue_button_status && 
	       sensation->blue_button_changed))){
      /* next_text(ALL);*/
      
      
      if (program_state->current_tour == -1){
	if (sensation->red_button_status &
	    sensation->red_button_changed)
	  program_state->current_tour = 0;
	if (sensation->yellow_button_status &
	    sensation->yellow_button_changed)
	  program_state->current_tour = 1;
	if (sensation->green_button_status &
	    sensation->green_button_changed)
	  program_state->current_tour = 2;
	if (sensation->blue_button_status &
	    sensation->blue_button_changed)
	  program_state->current_tour = 3;
	fprintf(stderr, "-3-");

      }

      move_to_next_tour_goal(ALL);
    }
  }
}



/************************************************************************
 *
 *   NAME:         handle_flow_reply_in_tour()
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void 
handle_flow_reply_in_tour(ALL_PARAMS)
{
  int i;
  
  if (program_state->giving_tour && program_state->tour_modus){
    if (sensation->flow_result)
      tour[program_state->current_tour_goal].action_outcome = 1;
    else
      tour[program_state->current_tour_goal].action_outcome = 0;
    move_to_next_tour_goal(ALL);
  }

  for (i = 0; i < program_state->num_tour_goals; i++){
    
    printf("ITEM %d\n", i);
    printf("\tnumber                  = %d\n", tour[i].number);
    printf("\ttour_number             = %d\n", tour[i].tour_number);
    printf("\trecording_x             = %g\n", tour[i].recording_x);
    printf("\trecording_y             = %g\n", tour[i].recording_y);
    printf("\trecording_orientation   = %g\n", tour[i].recording_orientation);
    printf("\tobject_x                = %g\n", tour[i].object_x);
    printf("\tobject_y                = %g\n", tour[i].object_y);
    printf("\tdirect                  = %d\n", tour[i].direct);
    printf("\tcondition               = %d\n", tour[i].condition);
    printf("\taction                  = %d\n", tour[i].action);
    printf("\ttext_1_number           = %d\n", tour[i].text_1_number);
    printf("\ttext_2_number           = %d\n", tour[i].text_2_number);
    printf("\ttext_3_number           = %d\n", tour[i].text_3_number);
    printf("\ttext_4_number           = %d\n", tour[i].text_4_number);
    printf("\ttext_5_number           = %d\n", tour[i].text_5_number);
    printf("\taction_outcome          = %d\n", tour[i].action_outcome);
    printf("\n");
    fflush(stdout);
  }

}

#endif /* TOURGUIDE_VERSION */
