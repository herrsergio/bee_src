
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

/* BUTTONS and other graphic objects */


int GLOBAL_BACKGROUND;
int MAPVALUES;
int COSTS;
int UTILITY;
int DISPLAY_MAPVALUES_BUTTON;
int DISPLAY_COSTS_BUTTON;
int DISPLAY_UTILITY_BUTTON;
int ROBOT;
int ADJUSTED_ACTION;
int GOALS[max_nNUMBER];
int PLAN_DISPLAY;
int PLAN_BOX;
int PLAN_BUTTON;
int CONSTRAINTS;
int QUIT_BUTTON;
int BASE_CONNECTED_BUTTON;
int MAP_CONNECTED_BUTTON;
int AUTONOMOUS_BUTTON;
int EXPLORATION_TYPE_BUTTON;
int BOUNDING_BOX;
int EXPLORATION_INTERIOR_MODE_BUTTON;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/





/************************************************************************
 *
 *   NAME:         init_graphics
 *                 
 *   FUNCTION:     Initiaizes graphics window
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
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


#define RIGHT_BAR  6.1, 8.0


void init_graphics(ROBOT_STATE_PTR    robot_state,
		   PROGRAM_STATE_PTR  program_state,
		   ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int i,j, ij;
  
  static char *myfonts[] = {"5x8", "6x10", "7x13bold", 
			      "9x15", "10x20", "12x24", "lucidasans-bold-24"};
  
  
  /* =============================================================
     ====================  1) SPECIFICATIONS  ====================
     ============================================================= */
  
  
  
  
  /******** ROBOT (BIG) IN GLOBAL MAP *************************************/
  static float pos_r_0[]                 = {0.0, 6.0, 0.0, 6.0};
  static char *text_r_0                  = "PLANNER";
  static int robot_0_font                = 4;
  static int colors_r_0[]                = {NO_COLOR, C_GREY40, C_RED,
					      C_BLACK, C_VIOLET, NO_COLOR, 
					      NO_COLOR, NO_COLOR};
  
  /******** GLOBAL MAP *******************************************/
  static float matrix_0_pos[]            = {0.0, 6.0, 0.0, 6.0};
  char *matrix_0_text                    = "Oops - should not be here";
  static int matrix_0_font               = 0;
  float min_value_matrix_0               = 0.0;
  float max_value_matrix_0               = 1.0;
  static int matrix_0_colors[]           = {C_STEELBLUE4,C_GREY40,NO_COLOR}; 
  
  
 

  /******** ADJUSTED_ACTION **************************************/
  int num_mar5                            = 3;
  static char *text_mar5[]                =
    {"(action)", "(action)", "(action)"};
  int connected_mar5                      = 1;
  static int mar5_frame_color             = NO_COLOR;
  static int mar5_background_color        = NO_COLOR;
  static int mar5_foreground_color[]      = {NO_COLOR, C_PINK, C_DEEPPINK};
  static int mar5_text_color[]            = {NO_COLOR, NO_COLOR, NO_COLOR};
  static int mar5_fonts[]                 = {0, 0, 0};


  /******** GOALS **************************************/

  int num_mar2                            = 20;
  static char *text_mar2[]                =
    {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
       "10", "11", "12", "13", "14", "15", "16", "17", "18", "19"};
  static char *text_mar2_prime[]                =
    {"X", "1", "2", "3", "4", "5", "6", "7", "8", "9",
       "10", "11", "12", "13", "14", "15", "16", "17", "18", "19"};
  int connected_mar2                      = 0;
  static int mar2_frame_color             = NO_COLOR;
  static int mar2_background_color        = NO_COLOR;
  static int mar2_foreground_color[]      = 
    {C_YELLOW, C_YELLOW, C_YELLOW, C_YELLOW, C_YELLOW, C_YELLOW, C_YELLOW,
       C_YELLOW, C_YELLOW, C_YELLOW, C_YELLOW, C_YELLOW, C_YELLOW, C_YELLOW,
       C_YELLOW, C_YELLOW, C_YELLOW, C_YELLOW, C_YELLOW, C_YELLOW};
  static int mar2_prime_foreground_color[]      = 
    {C_KHAKI4, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED,
       C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED,
       C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, 
       C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED,
       C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED,
       C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED,
       C_MEDIUMVIOLETRED};
  static int mar2_text_color[]            = 
    {C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
       C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, 
       C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, 
       C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK};
  static int mar2_fonts[]                 = 
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

  /******** PLAN_DISPLAY **************************************/
  int num_mar3                            = 2;
  static char *text_mar3[]                = {"(plan)", "(plan)"};
  int connected_mar3                      = 1;
  static int mar3_frame_color             = NO_COLOR;
  static int mar3_background_color        = NO_COLOR;
  static int mar3_foreground_color[]      = {C_LAWNGREEN, C_CYAN};
  static int mar3_text_color[]            = {NO_COLOR, NO_COLOR};
  static int mar3_fonts[]                 = {0, 0};

  /******** PLAN_BOX **************************************/
  int num_mar4                            = 1;
  static char *text_mar4[]                = {"(planbox)"};
  int connected_mar4                      = 1;
  static int mar4_frame_color             = NO_COLOR;
  static int mar4_background_color        = NO_COLOR;
  static int mar4_foreground_color[]      = {C_MEDIUMPURPLE3};
  static int mar4_text_color[]            = {NO_COLOR};
  static int mar4_fonts[]                 = {0};


  /******** BOUNDING_BOX **************************************/
  int num_mar44                            = 1;
  static char *text_mar44[]                = {"(planbox)"};
  int connected_mar44                      = 1;
  static int mar44_frame_color             = NO_COLOR;
  static int mar44_background_color        = NO_COLOR;
  static int mar44_foreground_color[]      = {C_YELLOW};
  static int mar44_text_color[]            = {NO_COLOR};
  static int mar44_fonts[]                 = {0};


  /******** GLOBAL_BACKGROUND **************************************/
  int switch_0_num                      = 1;
  static float switch_0_pos[]           = {0.0, 6.0, 0.0, 6.0};
  static char *switch_0_texts[]         = {"oops"};
  static int switch_0_fonts[]           = {5};
  static int switch_0_background_color[]= {C_STEELBLUE4};
  static int switch_0_frame_color[]     = {C_GREY40};
  static int switch_0_text_color[]      = {NO_COLOR};

  /******** DISPLAY_MAPVALUES_BUTTON **************************************/
  int switch_1_num                      = 2;
  static float switch_1_pos[]           = {RIGHT_BAR, 5.7, 6.0};
  static char *switch_1_texts[]         = {"map", "map"};
  static int switch_1_fonts[]           = {2,2};
  static int switch_1_background_color[]= {C_GREY90, C_YELLOW};
  static int switch_1_frame_color[]     = {C_GREY40, C_GREY40};
  static int switch_1_text_color[]      = {C_BLACK, C_BLACK};

  /******** DISPLAY_COSTS_BUTTON **************************************/
  static float switch_3_pos[]           = {RIGHT_BAR, 5.3, 5.6};
  static char *switch_3_texts[]         = {"costs", "costs"};

  /******** DISPLAY_UTILITY_BUTTON **************************************/
  static float switch_4_pos[]           = {RIGHT_BAR, 4.9, 5.2};
  static char *switch_4_texts[]         = {"utility", "utility"};


  /******** AUTONOMOUS_BUTTON **************************************/
  int switch_7_num                      = 2;
  static float switch_7_pos[]           = {RIGHT_BAR, 4.5, 4.8};
  static char *switch_7_texts[]         = {"autonomous", "autonomous ON"};
  static int switch_7_fonts[]           = {2,2};
  static int switch_7_background_color[]= {C_GREY90, C_LAWNGREEN};
  static int switch_7_frame_color[]     = {C_GREY40, C_GREY40};
  static int switch_7_text_color[]      = {C_BLACK, C_BLACK};


  /******** EXPLORATION_TYPE_BUTTON **************************************/
  int switch_77_num                      = 3;
  static float switch_77_pos[]           = {RIGHT_BAR, 4.1, 4.4};
  static char *switch_77_texts[]         = 
    {"expl: map", "expl: base", "..resetting"};
  static int switch_77_fonts[]           = {2,2,2};
  static int switch_77_background_color[]= {C_GREY90, C_GREY90, C_RED};
  static int switch_77_frame_color[]     = {C_GREY40, C_GREY40, C_GREY40};
  static int switch_77_text_color[]      = {C_BLACK, C_BLACK, C_WHITE};

  /******** EXPLORATION_INTERIOR_MODE_BUTTON ******************************/
  int switch_777_num                      = 2;
  static float switch_777_pos[]           = {RIGHT_BAR, 3.7, 4.0};
  static char *switch_777_texts[]         = 
    {"expl: exterior", "expl: interior"};
  static int switch_777_fonts[]           = {2,2,2};
  static int switch_777_background_color[]= {C_GREY90, C_YELLOW};
  static int switch_777_frame_color[]     = {C_GREY40, C_GREY40};
  static int switch_777_text_color[]      = {C_BLACK, C_BLACK};


  /******** BASE_CONNECTED_BUTTON ****************************/
  int switch_10_num                      = 2;
  static float switch_10_pos[]           = {RIGHT_BAR, 2.8, 3.1};
  static char *switch_10_texts[]         = {"(BASE not connected)",
					      "(BASE connected)"};
  static int switch_10_fonts[]           = {2,2};
  static int switch_10_background_color[]= {C_SLATEGREY, C_SLATEGREY};
  static int switch_10_frame_color[]     = {NO_COLOR, NO_COLOR};
  static int switch_10_text_color[]      = {C_WHITE, C_LAWNGREEN};

    
  /******** MAP_CONNECTED_BUTTON ****************************/
  static float switch_11_pos[]           = {RIGHT_BAR, 2.5, 2.75};
  static char *switch_11_texts[]         = {"(MAP not connected)",
					      "(MAP connected)"};

  /******** QUIT_BUTTON **************************************/

  static float switch_6_pos[]           = {RIGHT_BAR, 2.1, 2.4};
  static char *switch_6_texts[]         = {"quit", "quit"};

  /******** PLAN_BUTTON **************************************/
  int switch_5_num                      = 1;
  static float switch_5_pos[]           = {RIGHT_BAR, 0.0, 2.0};
  static char *switch_5_texts[]         = {"new plan"};
  static int switch_5_fonts[]           = {5};
  static int switch_5_background_color[]= {C_RED};
  static int switch_5_frame_color[]     = {C_GREY40};
  static int switch_5_text_color[]      = {C_WHITE};

 
  /******** CONSTRAINTS **************************************/
  int num_mar1                            = 3;
  static char *text_mar1[]                = {"oopsy", "oopsy", "oopsy"};
  int connected_mar1                      = 1;
  static int mar1_frame_color             = NO_COLOR;
  static int mar1_background_color        = NO_COLOR;
  static int mar1_foreground_color[]      = {NO_COLOR, C_RED, C_BLUE};
  static int mar1_text_color[]            = {NO_COLOR, NO_COLOR, NO_COLOR};
  static int mar1_fonts[]                 = {0, 0, 0};


  /* insert new graphics object here XXXXX */
  
 
  G_set_display(program_state->use_graphics); 
  
  /* =============================================================
     ====================  3) INITIALIZATIONS  ===================
     ============================================================= */

  if (!program_state->graphics_initialized){
    
    G_initialize_fonts(7, myfonts);
    G_initialize_graphics("BeeSoft Planner",
			  robot_specifications->X_window_size,
			  robot_specifications->X_window_size / 8.0,
			  C_SLATEGREY);
    
    
    
    DISPLAY_MAPVALUES_BUTTON
      = G_create_switch_object(switch_1_pos, switch_1_num, switch_1_texts,
			       switch_1_background_color,switch_1_frame_color, 
			       switch_1_text_color, switch_1_fonts);
    DISPLAY_COSTS_BUTTON
      = G_create_switch_object(switch_3_pos, switch_1_num, switch_3_texts,
			       switch_1_background_color,switch_1_frame_color, 
			       switch_1_text_color, switch_1_fonts);
    DISPLAY_UTILITY_BUTTON
      = G_create_switch_object(switch_4_pos, switch_1_num, switch_4_texts,
			       switch_1_background_color,switch_1_frame_color, 
			       switch_1_text_color, switch_1_fonts);
    

    AUTONOMOUS_BUTTON
      = G_create_switch_object(switch_7_pos, switch_7_num, switch_7_texts,
			       switch_7_background_color,switch_7_frame_color, 
			       switch_7_text_color, switch_7_fonts);

    EXPLORATION_TYPE_BUTTON
      = G_create_switch_object(switch_77_pos, switch_77_num, switch_77_texts,
			       switch_77_background_color,
			       switch_77_frame_color, 
			       switch_77_text_color, switch_77_fonts);

    EXPLORATION_INTERIOR_MODE_BUTTON
      = G_create_switch_object(switch_777_pos, switch_777_num, 
			       switch_777_texts,
			       switch_777_background_color,
			       switch_777_frame_color, 
			       switch_777_text_color, switch_777_fonts);

    BASE_CONNECTED_BUTTON
      = G_create_switch_object(switch_10_pos, switch_10_num, switch_10_texts,
			       switch_10_background_color,
			       switch_10_frame_color, 
			       switch_10_text_color, switch_10_fonts);
    
    MAP_CONNECTED_BUTTON
      = G_create_switch_object(switch_11_pos, switch_10_num, switch_11_texts,
			       switch_10_background_color,
			       switch_10_frame_color, 
			       switch_10_text_color, switch_10_fonts);
    
    QUIT_BUTTON
      = G_create_switch_object(switch_6_pos, switch_1_num, switch_6_texts,
			       switch_1_background_color,switch_1_frame_color, 
			       switch_1_text_color, switch_1_fonts);
    
    GLOBAL_BACKGROUND 
      = G_create_switch_object(switch_0_pos, switch_0_num, switch_0_texts,
			       switch_0_background_color,switch_0_frame_color, 
			       switch_0_text_color, switch_0_fonts);
    PLAN_BUTTON 
      = G_create_switch_object(switch_5_pos, switch_5_num, switch_5_texts,
			       switch_5_background_color,switch_5_frame_color, 
			       switch_5_text_color, switch_5_fonts);
    
    
    MAPVALUES = G_create_matrix_object(matrix_0_pos, matrix_0_text, 
				       global_values, global_active,
				       robot_specifications->global_map_dim_x,
				       robot_specifications->global_map_dim_y,
				       min_value_matrix_0, max_value_matrix_0, 
				       matrix_0_colors, matrix_0_font);
    
    COSTS = G_create_matrix_object(matrix_0_pos, matrix_0_text, 
				   global_costs, NULL,
				   robot_specifications->global_map_dim_x,
				   robot_specifications->global_map_dim_y,
				   min_value_matrix_0, max_value_matrix_0, 
				   matrix_0_colors, matrix_0_font);
    UTILITY = G_create_matrix_object(matrix_0_pos, matrix_0_text, 
				     global_utility, NULL,
				     robot_specifications->global_map_dim_x,
				     robot_specifications->global_map_dim_y,
				     min_value_matrix_0, max_value_matrix_0, 
				     matrix_0_colors, matrix_0_font);
    
    
    ADJUSTED_ACTION =
      G_create_markers_object(pos_r_0, connected_mar5, 
			      robot_specifications->robot_size * 0.3,
			      0.0, robot_specifications->global_mapsize_x,
			      0.0, robot_specifications->global_mapsize_y,
			      num_mar5, text_mar5, mar5_background_color, 
			      mar5_frame_color, mar5_foreground_color, 
			      mar5_text_color, mar5_fonts);
    
    
    PLAN_DISPLAY = 
      G_create_markers_object(pos_r_0, connected_mar3, 0.0,
			      0.0, robot_specifications->global_mapsize_x,
			      0.0, robot_specifications->global_mapsize_y,
			      num_mar3, text_mar3, mar3_background_color, 
			      mar3_frame_color, mar3_foreground_color, 
			      mar3_text_color, mar3_fonts);
    
    PLAN_BOX =
      G_create_markers_object(pos_r_0, connected_mar4, 0.0,
			      0.0, robot_specifications->global_mapsize_x,
			      0.0, robot_specifications->global_mapsize_y,
			      num_mar4, text_mar4, mar4_background_color, 
			      mar4_frame_color, mar4_foreground_color, 
			      mar4_text_color, mar4_fonts);
    
    BOUNDING_BOX =
      G_create_markers_object(pos_r_0, connected_mar44, 0.0,
			      0.0, robot_specifications->global_mapsize_x,
			      0.0, robot_specifications->global_mapsize_y,
			      num_mar44, text_mar44, mar44_background_color, 
			      mar44_frame_color, mar44_foreground_color, 
			      mar44_text_color, mar44_fonts);
    
    
    CONSTRAINTS =
      G_create_markers_object(pos_r_0, connected_mar1,
			      robot_specifications->robot_size * 0.5,
			      0.0, robot_specifications->global_mapsize_x,
			      0.0, robot_specifications->global_mapsize_y,
			      num_mar1, text_mar1, mar1_background_color, 
			      mar1_frame_color, mar1_foreground_color, 
			      mar1_text_color, mar1_fonts);
    
    
    for (i = 0; i < nNUMBER; i++){
      if (i == 0)
	GOALS[i] = 
	  G_create_markers_object(pos_r_0, connected_mar2, 
				  robot_specifications->robot_size,
				  0.0, robot_specifications->global_mapsize_x,
				  0.0, robot_specifications->global_mapsize_y,
				  num_mar2, text_mar2, mar2_background_color, 
				  mar2_frame_color, mar2_foreground_color, 
				  mar2_text_color, mar2_fonts);
      else
	GOALS[i] = 
	  G_create_markers_object(pos_r_0, connected_mar2, 
				  robot_specifications->robot_size,
				  0.0, robot_specifications->global_mapsize_x,
				  0.0, robot_specifications->global_mapsize_y,
				  num_mar2, text_mar2_prime, 
				  mar2_background_color, mar2_frame_color, 
				  mar2_prime_foreground_color, 
				  mar2_text_color, mar2_fonts);
    }
    
    ROBOT =
      G_create_robot_object(pos_r_0, text_r_0, 
			    0.0, robot_specifications->global_mapsize_x,
			    0.0,  robot_specifications->global_mapsize_y,
			    robot_state->x, robot_state->y,
			    robot_state->orientation, 
			    robot_specifications->robot_size, 
			    0, 0.0, NULL, NULL,
			    colors_r_0, robot_0_font);
    
    
    /* insert new graphics object here XXXXX */
    
    
    /* =============================================================
       ====================  4) DISPLAY  ===========================
       ============================================================= */
  }  
#ifndef UNIBONN
  G_deactivate(EXPLORATION_TYPE_BUTTON);
  G_deactivate(EXPLORATION_INTERIOR_MODE_BUTTON);
#endif /*UNIBONN*/
  G_set_matrix_display_style(1);
  G_deactivate(MAPVALUES);
/*  G_deactivate(BOUNDING_BOX);*/
  G_deactivate(COSTS);
  G_deactivate(UTILITY);
  G_deactivate(ROBOT);
  G_deactivate(PLAN_BOX);
  G_clear_markers(PLAN_DISPLAY);
  G_clear_markers(ADJUSTED_ACTION);
  for (i = 0; i < nNUMBER; i++)
    G_clear_markers(GOALS[i]);
/*  autoshift_display_by(robot_state, program_state, robot_specifications,
		       robot_specifications->global_map_dim_x / 2,
		       robot_specifications->global_map_dim_y / 2);
*/
  if (program_state->use_graphics)
    G_display_all();
  program_state->graphics_initialized = 1;
}





/************************************************************************
 *
 *   NAME:         display_all()
 *                 
 *   FUNCTION:     displays all, with resetting some of the buttons
 *                 
 *   PARAMETERS:   int name     name of bright button, -1 if none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void display_all(int name,
		 ROBOT_STATE_PTR robot_state,
		 ROBOT_SPECIFICATIONS_PTR robot_specifications, 
		 PROGRAM_STATE_PTR program_state)
{
  if (!program_state->graphics_initialized)
    return;


  if (name >= 0)
    G_display_switch(name, 1);

  if (name == DISPLAY_MAPVALUES_BUTTON){
    G_display_partial_matrix(MAPVALUES,
			     robot_specifications->min_display_index_x,
			     robot_specifications->max_display_index_x -
			     robot_specifications->min_display_index_x,
			     robot_specifications->min_display_index_y,
			     robot_specifications->max_display_index_y -
			     robot_specifications->min_display_index_y);
    display_robot(robot_state, program_state);
  }

  else if (name == DISPLAY_COSTS_BUTTON){
    G_display_partial_matrix(COSTS, robot_specifications->min_display_index_x,
			     robot_specifications->max_display_index_x -
			     robot_specifications->min_display_index_x,
			     robot_specifications->min_display_index_y,
			     robot_specifications->max_display_index_y -
			     robot_specifications->min_display_index_y);
    display_robot(robot_state, program_state);
  }

  else if (name == DISPLAY_UTILITY_BUTTON){
    G_display_partial_matrix(UTILITY, 
			     robot_specifications->min_display_index_x,
			     robot_specifications->max_display_index_x -
			     robot_specifications->min_display_index_x,
			     robot_specifications->min_display_index_y,
			     robot_specifications->max_display_index_y -
			     robot_specifications->min_display_index_y);
    display_robot(robot_state, program_state);
  }

  else
    G_display_all();

  display_plan_box(robot_specifications, program_state);
  G_display_markers(ADJUSTED_ACTION);
  G_display_markers(PLAN_DISPLAY);
  G_display_markers(BOUNDING_BOX);

  if (name >= 0)
    G_display_switch(name, 0);
}



/************************************************************************
 *
 *   NAME:         display_robot()
 *                 
 *   FUNCTION:     displays robot and goals
 *                 
 *   PARAMETERS:   ROBOT_STATE_PTR robot_state
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void display_robot(ROBOT_STATE_PTR robot_state, 
		   PROGRAM_STATE_PTR program_state)
{
  int i;

  if (!program_state->graphics_initialized)
    return;

  G_display_markers(CONSTRAINTS);
  for (i = 0; i < nNUMBER; i++)
    G_display_markers(GOALS[i]);

  G_display_robot(ROBOT, robot_state->x, robot_state->y,
		  robot_state->orientation, 0, NULL);
}




/************************************************************************
 *
 *   NAME:         autoshift_display_by
 *                 
 *   FUNCTION:     Shifts robot by (shift_x, shift_y) units
 *                 
 *   PARAMETERS:   ROBOT_STATE_PTR    robot_state    Pointer to robot state
 *                                                   description structure
 *                 PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure 
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *                 int shift_x, shift_y              units to shift
 *                 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void autoshift_display_by(ROBOT_STATE_PTR robot_state,
			  PROGRAM_STATE_PTR  program_state,
			  ROBOT_SPECIFICATIONS_PTR robot_specifications,
			  int shift_x, 
			  int shift_y)
{
  int from_x, to_x, increment_x, from_y, to_y, increment_y;
  int x, y, goal_number;
  int index_from, index_to, index;
  float goal_x, goal_y;
  int i, n, k;

    
  
  if (shift_x != 0 || shift_y != 0){	/* ie., if autoshift necessary */
    
    if (shift_x <= 0){
      from_x = 0;
      to_x   = robot_specifications->global_map_dim_x+shift_x;
      increment_x = 1;
    }
    else{
      from_x = robot_specifications->global_map_dim_x-1;
      to_x   = shift_x-1;
      increment_x = -1;
    }
    if (shift_y <= 0){
      from_y = 0;
      to_y   = robot_specifications->global_map_dim_y+shift_y;
      increment_y = 1;
    }
    else{
      from_y = robot_specifications->global_map_dim_y-1;
      to_y   = shift_y-1;
      increment_y = -1;
    }	 
    
    /* ---------> SHIFT MAP */
    if (from_x >= 0 && to_x >= 0 && from_y >= 0 && to_y >= 0
	/* && from_x <= to_x && from_y <= to_y */
	){
      fprintf(stderr, "## Shift by: %d %d\n", shift_x, shift_y);
      if (shift_x != 0)
	for (x = from_x; x != to_x; x += increment_x)
	  for (y = 0; y < robot_specifications->global_map_dim_y; y++){
	    index_from = (x-shift_x) * robot_specifications->global_map_dim_y
	      + y;
	    index_to   = x * robot_specifications->global_map_dim_y + y;
	    global_values[index_to]       = global_values[index_from];
	    global_active[index_to]       = global_active[index_from];
	    global_visited[index_to]      = global_visited[index_from];
	    global_costs[index_to]        = global_costs[index_from];
	    global_costs2[index_to]       = global_costs2[index_from];
	    global_values[index_from]     = 0.0;
	    global_active[index_from]     = 0;
	    global_visited[index_from]     = 0;
	    global_costs[index_from]    = robot_specifications->average_costs;
	    global_costs2[index_from]   = robot_specifications->average_costs2;
	    for (k = 0; k < nNUMBER; k++){
	      (global_goal_table[k])[index_to]   
		= (global_goal_table[k])[index_from];
	      (global_utility_table[k])[index_to] 
		= (global_utility_table[k])[index_from];
	      (global_succ_table[k])[index_to] 
		= (global_succ_table[k])[index_from];
	      (global_goal_table[k])[index_from]   = 0;
	      (global_utility_table[k])[index_from]= 0.0;
	      (global_succ_table[k])[index_from]   = -1;
	    }
	  }
      if (shift_y != 0)
	for (x = 0; x < robot_specifications->global_map_dim_x; x++)
	  for (y = from_y; y != to_y; y += increment_y){
	    index_from = x * robot_specifications->global_map_dim_y
	      + (y-shift_y);
	    index_to   = x * robot_specifications->global_map_dim_y + y;
	    global_values[index_to]       = global_values[index_from];
	    global_active[index_to]       = global_active[index_from];
	    global_visited[index_to]      = global_visited[index_from];
	    global_costs[index_to]        = global_costs[index_from];
	    global_costs2[index_to]       = global_costs2[index_from];
	    global_values[index_from]     = 0.0;
	    global_active[index_from]     = 0;
	    global_visited[index_from]    = 0;
	    global_costs[index_from]    = robot_specifications->average_costs;
	    global_costs2[index_from]   = robot_specifications->average_costs2;
	    for (k = 0; k < nNUMBER; k++){
	      (global_goal_table[k])[index_to]   
		= (global_goal_table[k])[index_from];
	      (global_utility_table[k])[index_to] 
		= (global_utility_table[k])[index_from];
	      (global_succ_table[k])[index_to] 
		= (global_succ_table[k])[index_from];
	      (global_goal_table[k])[index_from]   = 0;
	      (global_utility_table[k])[index_from]= 0.0;
	      (global_succ_table[k])[index_from]   = -1;
	    }
	  }
    }
    else
      for (index_from = 0; 
	   index_from < robot_specifications->global_map_dim_x
	   * robot_specifications->global_map_dim_y; index_from++){
	global_values[index_from]     = 0.0;
	global_active[index_from]     = 0;
	global_visited[index_from]   = 0;
	global_costs[index_from]      = robot_specifications->average_costs;
	global_costs2[index_from]     = robot_specifications->average_costs2;
	for (k = 0; k < nNUMBER; k++){
	  (global_goal_table[k])[index_from]   = 0;
	  (global_utility_table[k])[index_from]= 0.0;
	  (global_succ_table[k])[index_from]   = -1;
	}
      }    
    
    /* ---------> CLEAR THE BORDERS - VERY IMPORTANT! */
    for (k = 0; k < nNUMBER; k++){
      for (x = 0; x < robot_specifications->global_map_dim_x; x++){
	(global_utility_table[k])[x * robot_specifications->global_map_dim_y]
	  = 0.0;
	(global_utility_table[k])[(x+1) * 
				  robot_specifications->global_map_dim_y - 1]
				    = 0.0;
	(global_succ_table[k])[x * robot_specifications->global_map_dim_y]
	  = -1;
	(global_succ_table[k])[(x+1) * 
			       robot_specifications->global_map_dim_y - 1]
				 = -1;
      }
      for (y = 0; y < robot_specifications->global_map_dim_y; y++){
	(global_utility_table[k])[y] = 0.0;
	(global_utility_table[k])[(robot_specifications->global_map_dim_x - 1) 
				  * robot_specifications->global_map_dim_y
				  + y] = 0.0;
	(global_succ_table[k])[y] = -1;
	(global_succ_table[k])[(robot_specifications->global_map_dim_x - 1) 
			       * robot_specifications->global_map_dim_y
			       + y] = -1;
      }
    }
    
    /* ---------> CHANGE GLOBAL VARIABLES */
    robot_specifications->autoshifted_x += 	
      ((float) shift_x) * robot_specifications->resolution;
    robot_specifications->autoshifted_y += 
      ((float) shift_y) * robot_specifications->resolution;
    robot_specifications->autoshifted_int_x    += shift_x;
    robot_specifications->autoshifted_int_y    += shift_y;
    robot_specifications->min_plan_index_x     += shift_x; 
    robot_specifications->max_plan_index_x     += shift_x;
    robot_specifications->min_plan_index_y     += shift_y;
    robot_specifications->max_plan_index_y     += shift_y;
    robot_specifications->min_display_index_x  += shift_x;
    robot_specifications->max_display_index_x  += shift_x;
    robot_specifications->min_display_index_y  += shift_y;
    robot_specifications->max_display_index_y  += shift_y;
    /*    check_index(robot_specifications, 0); */
    
    
    if (program_state->graphics_initialized){
      G_shift_robot_local_coordinates(ROBOT, 
				      ((float) shift_x)
				      * robot_specifications->resolution,
				      ((float) shift_y)
				      * robot_specifications->resolution);
      for (k = 0; k < nNUMBER; k++)
	G_shift_markers_local_coordinates(GOALS[k], 
					  ((float) shift_x)
					  * robot_specifications->resolution,
					  ((float) shift_y)
					  * robot_specifications->resolution);
      G_shift_markers_local_coordinates(ADJUSTED_ACTION, 
					((float) shift_x)
					* robot_specifications->resolution,
					((float) shift_y)
					* robot_specifications->resolution);
      G_shift_markers_local_coordinates(PLAN_DISPLAY, 
					((float) shift_x)
					* robot_specifications->resolution,
					((float) shift_y)
					* robot_specifications->resolution);
      G_shift_markers_local_coordinates(PLAN_BOX, 
					((float) shift_x)
					* robot_specifications->resolution,
					((float) shift_y)
					* robot_specifications->resolution);
      G_shift_markers_local_coordinates(BOUNDING_BOX, 
					((float) shift_x)
					* robot_specifications->resolution,
					((float) shift_y)
					* robot_specifications->resolution);
      
      /* ---------> INSERT PREVIOUSLY HIDDEN AND NOW VISIBLE GOALS */
      n = G_return_num_markers(GOALS[NUMBER], 0);
      for (i = 0; i < n; i++){
	G_return_marker_coordinates(GOALS[NUMBER], i, &goal_x, &goal_y,
				    &goal_number);
	x = ((int) (goal_x / robot_specifications->resolution))
	  + robot_specifications->autoshifted_int_x;
	y = ((int) (goal_y / robot_specifications->resolution))
	  + robot_specifications->autoshifted_int_y;
	if (x >= 0 && x < robot_specifications->global_map_dim_x &&
	    y >= 0 && y < robot_specifications->global_map_dim_y){
	  index = x * robot_specifications->global_map_dim_y + y;
	  global_goal[index] = 1 + goal_number;
	  
	  /* adjust internal planning borders */
	  if (x < robot_specifications->min_display_index_x)
	    robot_specifications->min_display_index_x = x;
	  if (y < robot_specifications->min_display_index_y)
	    robot_specifications->min_display_index_y = y;
	  if (x + 1 > robot_specifications->max_display_index_x)
	    robot_specifications->max_display_index_x = x + 1;
	  if (y + 1 > robot_specifications->max_display_index_y)
	    robot_specifications->max_display_index_y = y + 1;
	}
      }	  
      check_index(robot_specifications, 1);
    }
    
    /* ---------> COUNT GOALS */
    count_goals_and_reset_utilities(program_state, robot_specifications, 
				  robot_state);
    
    /* ---------> GRAPHICS DISPLAY */
    display_all(-1, robot_state, robot_specifications, program_state);
  }
}





/************************************************************************
 *
 *   NAME:         autoshift_display
 *                 
 *   FUNCTION:     Shifts robot display to always keep robot centered
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
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void autoshift_display(float point_x, 
		       float point_y,
		       ROBOT_STATE_PTR robot_state,
		       PROGRAM_STATE_PTR  program_state,
		       ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  float robot_x, robot_y;
  int shift_x, shift_y;
  
  
  if (robot_specifications->autoshift == 1){
    shift_x = 0;
    shift_y = 0;
    robot_x = point_x + robot_specifications->autoshifted_x;
    robot_y = point_y + robot_specifications->autoshifted_y;
    
    /* test left margin */
    if (robot_x < robot_specifications->autoshift_safety_margin)
      shift_x = 1 + ((int) ((robot_specifications->autoshift_distance 
			     - robot_x) 
			    / robot_specifications->resolution)); /* > 0 */
    
    /* test right margin */
    if (robot_x > robot_specifications->global_mapsize_x -
	robot_specifications->autoshift_safety_margin)
      shift_x = -1 + ((int) ((robot_specifications->global_mapsize_x - robot_x 
			      - robot_specifications->autoshift_distance) 
			     / robot_specifications->resolution)); /* < 0 */
    
    
    /* test lower margin */
    if (robot_y < robot_specifications->autoshift_safety_margin)
      shift_y = 1 + ((int) ((robot_specifications->autoshift_distance 
			     - robot_y) 
			    / robot_specifications->resolution)); /* > 0 */
    
    /* test upper margin */
    if (robot_y > robot_specifications->global_mapsize_y -
	robot_specifications->autoshift_safety_margin)
      shift_y = -1 + ((int) ((robot_specifications->global_mapsize_y - robot_y 
			      - robot_specifications->autoshift_distance) 
			     / robot_specifications->resolution)); /* < 0 */
    

    autoshift_display_by(robot_state, program_state, robot_specifications, 
			 shift_x, shift_y);
    
  }
}



/************************************************************************
 *
 *   NAME:         display_plan_box
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




void display_plan_box(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		      PROGRAM_STATE_PTR program_state)
{
  /* display current plan-box */
  if (!program_state->graphics_initialized)
    return;

  G_activate(PLAN_BOX);
  G_clear_markers(PLAN_BOX);
  if (robot_specifications->min_plan_index_x < 
      robot_specifications->max_plan_index_x &&
      robot_specifications->min_plan_index_y < 
      robot_specifications->max_plan_index_y){
    G_add_marker(PLAN_BOX, 
		 ((float) (robot_specifications->min_plan_index_x
			   - robot_specifications->autoshifted_int_x))
		 * robot_specifications->resolution,
		 ((float) (robot_specifications->min_plan_index_y
			   - robot_specifications->autoshifted_int_y))
		 * robot_specifications->resolution, 0);
    G_add_marker(PLAN_BOX, 
		 ((float) (robot_specifications->min_plan_index_x
			   - robot_specifications->autoshifted_int_x))
		 * robot_specifications->resolution,
		 ((float) (robot_specifications->max_plan_index_y
			   - robot_specifications->autoshifted_int_y))
		 * robot_specifications->resolution, 0);
    G_add_marker(PLAN_BOX, 
		 ((float) (robot_specifications->max_plan_index_x
			   - robot_specifications->autoshifted_int_x))
		 * robot_specifications->resolution,
		 ((float) (robot_specifications->max_plan_index_y
			   - robot_specifications->autoshifted_int_y))
		 * robot_specifications->resolution, 0);
    G_add_marker(PLAN_BOX, 
		 ((float) (robot_specifications->max_plan_index_x
			   - robot_specifications->autoshifted_int_x))
		 * robot_specifications->resolution,
		 ((float) (robot_specifications->min_plan_index_y
			   - robot_specifications->autoshifted_int_y))
		 * robot_specifications->resolution, 0);
    G_add_marker(PLAN_BOX, 
		 ((float) (robot_specifications->min_plan_index_x
			   - robot_specifications->autoshifted_int_x))
		 * robot_specifications->resolution,
		 ((float) (robot_specifications->min_plan_index_y
			   - robot_specifications->autoshifted_int_y))
		 * robot_specifications->resolution, 0);
    G_display_markers(PLAN_BOX);
    G_deactivate(PLAN_BOX);
  }
}




/************************************************************************
 *
 *   NAME:         mouse_test_loop
 *                 
 *   FUNCTION:     Checks mouse events and changes the variables
 *                 "action" and "Program_state" correspondingly
 *                 
 *   PARAMETERS:   ROBOT_STATE_PTR    robot_state    Pointer to robot state
 *                                                   description structure
 *                 PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure (see above)
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *                 ACTION_PTR   action               pointer to action
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/




int mouse_test_loop(ROBOT_STATE_PTR          robot_state,
		    PROGRAM_STATE_PTR        program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications,
		    ACTION_PTR               action)
{
  
  G_mouse_ptr mouse_events;
  int num_mouse_events, button;
  float mouse_x, mouse_y;
  int i, number;
  float min_value, max_value;
  int test;
  
  if (!program_state->graphics_initialized)
    return 0;


  /****************** CHECK FOR MOUSE EVENT *******************/
  
  test = G_test_mouse(0);

  if (test == 1){
    program_state->busy++;

    mouse_events = G_evaluate_mouse(&mouse_x, &mouse_y, 
				    &button, &num_mouse_events);
    
    /****************** EVALUATE MOUSE EVENT *******************/
    
    
    

    if (G_mouse_event_at(DISPLAY_MAPVALUES_BUTTON, mouse_events, &number)){
      G_deactivate(GLOBAL_BACKGROUND);
      G_activate(MAPVALUES);
      display_all(DISPLAY_MAPVALUES_BUTTON, robot_state, robot_specifications,
		  program_state);
      G_deactivate(MAPVALUES);
      G_activate(GLOBAL_BACKGROUND);
    }

    

    else if (G_mouse_event_at(DISPLAY_COSTS_BUTTON, mouse_events, &number)){
      G_matrix_set_display_range(COSTS, 
				 robot_specifications->min_costs, 
				 robot_specifications->max_costs);
      G_activate(COSTS);
      display_all(DISPLAY_COSTS_BUTTON, robot_state, robot_specifications, 
		  program_state);
      G_deactivate(COSTS);
    }

    else if (G_mouse_event_at(DISPLAY_UTILITY_BUTTON, mouse_events, &number)){
      G_activate(UTILITY);
      display_all(DISPLAY_UTILITY_BUTTON, robot_state, robot_specifications,
		  program_state);
      G_deactivate(UTILITY);
    }


    else if (G_mouse_event_at(AUTONOMOUS_BUTTON, mouse_events, &number)){
      if (program_state->autonomous)
	tcx_base_stop(program_state);
      program_state->autonomous ^= 1;
      if (!program_state->base_connected && program_state->autonomous)
	program_state->autonomous = 0;
      if (!program_state->autonomous)
	program_state->exploration = 0;
      G_display_switch(AUTONOMOUS_BUTTON, program_state->autonomous);

      if (program_state->autonomous){
	program_state->target_not_reached = 0;
	generate_action(robot_specifications, program_state, robot_state,
			action, 0, 1); /* just to make sure we have an 
					* action ready */
	G_display_switch(GLOBAL_BACKGROUND, 0);
	G_display_markers(ADJUSTED_ACTION);
	G_display_markers(PLAN_DISPLAY);
      }
      send_automatic_status_update(NULL);
      /*program_state->send_automatic_update = 1;*/
    }


    else if (G_mouse_event_at(EXPLORATION_TYPE_BUTTON, mouse_events, &number)){
      if (button == MIDDLE_BUTTON){
	reset_internal_exploration_table(program_state, robot_specifications);
	G_display_switch(EXPLORATION_TYPE_BUTTON, 2);
	usleep(500000);
	putc(7, stderr);
      }
      else{
	if (mouse_events[number].actual_text == 0)
	  global_explored = global_visited; /* just a pointer */
	else
	  global_explored = global_active; /* just a pointer */
      }
      G_display_switch(EXPLORATION_TYPE_BUTTON, 
		       1 - mouse_events[number].actual_text);
      count_goals_and_reset_utilities(program_state, robot_specifications, 
				  robot_state);
    }


    else if (G_mouse_event_at(EXPLORATION_INTERIOR_MODE_BUTTON,
			      mouse_events, &number)){
      program_state->interior_mode ^= 1;
      G_display_switch(EXPLORATION_INTERIOR_MODE_BUTTON,
		       program_state->interior_mode);
      count_goals_and_reset_utilities(program_state, robot_specifications, 
				      robot_state);
    }
    

    else if (G_mouse_event_at(QUIT_BUTTON, mouse_events, &number)){
      program_state->quit = 1;
    }


    else if (G_mouse_event_at(PLAN_BUTTON, mouse_events, &number)){
      robot_state->stuck = 0;

      generate_action(robot_specifications, program_state, robot_state,
		      action, 1, 1);/*! used to be ...0,1 */
    }
    

    /*============ goal modifications ==========*/

    else if (button == LEFT_BUTTON &&
	     G_mouse_event_at(GOALS[0], mouse_events, &number)){
      if (PLAN_verbose)
	fprintf(stderr, "button[%g %g]\n", mouse_events[number].value_x,
		mouse_events[number].value_y);
      if (mouse_events[number].marker_name == -1)
	modify_goal_set(program_state, robot_state, robot_specifications,
			mouse_events[number].value_x,
			mouse_events[number].value_y,
			1, 0);
      else
	modify_goal_set(program_state, robot_state, robot_specifications,
			mouse_events[number].marker_x,
			mouse_events[number].marker_y,
			0, 0);

    }


    /*============ pseudo-maps and pseudo-robot-pos ==========*/

    else if (button == MIDDLE_BUTTON && !program_state->use_tcx &&
	     G_mouse_event_at(GOALS[NUMBER], mouse_events, &number)){
      fake_map_update(robot_state, program_state, robot_specifications,
		      mouse_events[number].value_x, 
		      mouse_events[number].value_y);
      
    }
    else if (button == RIGHT_BUTTON && /*! !program_state->base_connected && */
	     G_mouse_event_at(GOALS[0], mouse_events, &number)){
      PLAN_new_robot_pos_message_type msg;
      fprintf(stderr, "button[%g %g]\n", mouse_events[number].value_x,
	      mouse_events[number].value_y);
      msg.x           = mouse_events[number].value_x;
      msg.y           = mouse_events[number].value_y;
      msg.orientation = robot_state->orientation = RAND_POS() * 360.0;
      PLAN_new_robot_pos_message_handler(NULL, &msg);
      
    }



    /* insert new graphics object here XXXXX */
    
    
    else{
      display_all(-1, robot_state, robot_specifications, program_state);
    }
  }
  
  return test;
}


