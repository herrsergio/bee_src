
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





#ifdef VMS
#include "vms.h"
#include "vmstime.h"
#endif



#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <signal.h>
#include "tcx.h"
#include "tcxP.h"
#include "Application.h"
#include "Net.h"
#include "MAP-messages.h"
#include "SONAR-messages.h"
#include "BASE-messages.h"
#include "SONARINT.h"
#include "EZX11.h"
#include "o-graphics.h"

#ifdef RHINO_PLUS
#include "rst.h" /* rhino stuff */
#endif

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* BUTTONS and other graphic objects */


int DISPLAY_LOCAL_MAPVALUES_BUTTON;
int LOCAL_ROBOT_;
int LOCAL_BACKGROUND;
int LOCAL_MAPVALUES;
int QUIT_BUTTON;
int SCRIPT_BUTTON;
int BASE_CONNECTED_BUTTON;
int MAP_CONNECTED_BUTTON;
int REGRESSION;

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


#define LOCAL_WINDOW_SIZE 2.0
#define RIGHT_BAR  6.1, (6.1+LOCAL_WINDOW_SIZE)


void init_graphics(ROBOT_STATE_PTR    robot_state,
		   PROGRAM_STATE_PTR  program_state,
		   ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int i,j, ij;
  

  static char *myfonts[] = {"5x8", "6x10", "7x13bold", 
			      "9x15", "10x20", "12x24", "lucidasans-bold-24"};

  if (!program_state->use_graphics)
    return;

  /* =============================================================
     ====================  1) INITIALIZATIONS  ===================
     ============================================================= */
  
  if (!program_state->graphics_initialized){
    
    G_initialize_fonts(7, myfonts);
    G_initialize_graphics("BeeSoft Sonar Interpreter", 80.0, 10.0, C_SLATEGREY);
    
    
    
    /* =============================================================
       ====================  2) SPECIFICATIONS  ====================
       ============================================================= */

    
    {
      /******** DISPLAY_LOCAL_MAPVALUES_BUTTON ****************************/
      int switch_num                      = 3;
      static float switch_pos[]           = {RIGHT_BAR, 5.7, 6.0};
      static char *switch_texts[]         = 
	{"display map (OFF)", "display map (ON)", "..busy"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_WHITE, C_YELLOW, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK, C_WHITE};
      DISPLAY_LOCAL_MAPVALUES_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    {
      
      /******** QUIT_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_BAR, 5.3, 5.6};
      static char *switch_texts[]         = {"quit", "quit"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_WHITE, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_WHITE};
      QUIT_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    {      
      /******** SCRIPT_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_BAR, 4.9, 5.2};
      static char *switch_texts[]         = {"read script", "..processing script"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_WHITE, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      SCRIPT_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
    {      
      /******** BASE_CONNECTED_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_BAR, 4.5, 4.8};
      static char *switch_texts[]         = {"(COLLI not connected)",
					       "(COLLI connected)"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_SLATEGREY, C_SLATEGREY};
      static int switch_frame_color[]     = {NO_COLOR, NO_COLOR};
      static int switch_text_color[]      = {C_WHITE, C_LAWNGREEN};
      BASE_CONNECTED_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
    {
      /******** MAP_CONNECTED_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_BAR, 4.1, 4.4};
      static char *switch_texts[]         = {"(MAP not connected)",
					       "(MAP connected)"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_SLATEGREY, C_SLATEGREY};
      static int switch_frame_color[]     = {NO_COLOR, NO_COLOR};
      static int switch_text_color[]      = {C_WHITE, C_LAWNGREEN};
      MAP_CONNECTED_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    
    {
      /******** LOCAL_BACKGROUND **************************************/
      int switch_num                      = 1;
      static float switch_pos[]           = {4.0, 5.9, 4.1, 6.0}; 
      static char *switch_texts[]         = {"oops"};
      static int switch_fonts[]           = {5};
      static int switch_background_color[]= {C_TURQUOISE4};
      static int switch_frame_color[]     = {C_GREY40};
      static int switch_text_color[]      = {NO_COLOR};

      LOCAL_BACKGROUND 
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
    


    {
      /******** LOCAL_MAPVALUES *******************************************/
      static float matrix_pos[]            = {4.0, 5.9, 4.1, 6.0};
      char *matrix_text                    = "Oops - should not be here";
      static int matrix_font               = 5;
      float min_value_matrix               = 0.0;
      float max_value_matrix               = 1.0;
      static int matrix_colors[]           = {C_TURQUOISE4,C_GREY40,NO_COLOR}; 
      
      LOCAL_MAPVALUES = 
	G_create_matrix_object(matrix_pos, matrix_text, 
			       local_map, local_active,
			       robot_specifications->local_map_dim_x,
			       robot_specifications->local_map_dim_y,
			       min_value_matrix, max_value_matrix, 
			       matrix_colors, matrix_font);
    }

    {
      /******** ROBOT (SMALL) IN LOCAL MAP *********************************/ 
      static float pos_r[]                 = {4.0, 5.9, 4.1, 6.0};
      static char *text_r                  = "local";
      static int robot_font                = 3;
      static int colors_r[]                = {NO_COLOR, C_GREY40, C_RED,
						C_GREY50, C_VIOLET, C_BLACK, 
						NO_COLOR, NO_COLOR};

      
      
      LOCAL_ROBOT_ =
	G_create_robot_object(pos_r, text_r, 
			      -robot_specifications->max_sensors_range*1.01,
			      robot_specifications->max_sensors_range*1.01,
			      -robot_specifications->max_sensors_range*1.01,
			      robot_specifications->max_sensors_range*1.01,
			      0.0, 0.0, 90.0,
			      robot_specifications->robot_size, 
			      robot_specifications->num_sensors, 
			      robot_specifications->max_sensors_range, 
			      robot_state->sensor_values, 
			      robot_specifications->sensor_angles,
			      colors_r, robot_font);
      
    }


    /******** REGRESSION *********************************************/
    {
      static float pos_mar[]                   = {4.0, 5.9, 4.1, 6.0};
      int num_mar                            = 1;
      static char *text_mar[]                = {"REGRESSION"};
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = {C_YELLOW, C_YELLOW};
      static int mar_text_color[]            = {NO_COLOR};
      static int mar_fonts[]                 = {2};
      
      REGRESSION
	= G_create_markers_object(pos_mar, 1,
				  robot_specifications->robot_size * 0.3,
				  -robot_specifications->max_sensors_range*1.01,
				  robot_specifications->max_sensors_range*1.01,
				  -robot_specifications->max_sensors_range*1.01,
				  robot_specifications->max_sensors_range*1.01,
				  num_mar, text_mar, mar_background_color, 
				  mar_frame_color, mar_foreground_color, 
				  mar_text_color, mar_fonts);
      
    }
  

    
    /* insert new graphics object here XXXXX */

  }
    
  
  
  /* =============================================================
     ====================  4) DISPLAY  ===========================
     ============================================================= */


  G_deactivate(LOCAL_MAPVALUES);
  G_set_matrix_display_style(1);
  G_display_all();
  G_display_switch(MAP_CONNECTED_BUTTON, program_state->map_connected);
  G_display_switch(BASE_CONNECTED_BUTTON, program_state->base_connected);


  program_state->graphics_initialized = 1;
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
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/




int mouse_test_loop(ROBOT_STATE_PTR          robot_state,
		    PROGRAM_STATE_PTR        program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  
  G_mouse_ptr mouse_events;
  int num_mouse_events, button;
  float mouse_x, mouse_y;
  int i, number;
  float min_value, max_value;
  int test;
  float change_x, change_y, change_orientation;


  if (!program_state->graphics_initialized)
    return 0;



  /****************** CHECK FOR MOUSE EVENT *******************/
  
  test = G_test_mouse(0);

  if (test == 1){
    mouse_events = G_evaluate_mouse(&mouse_x, &mouse_y, 
				    &button, &num_mouse_events);
    
    /****************** EVALUATE MOUSE EVENT *******************/
    
    

    if (G_mouse_event_at(DISPLAY_LOCAL_MAPVALUES_BUTTON, 
			      mouse_events, &number)){
      program_state->regular_local_map_display ^= 1;
      G_display_switch(DISPLAY_LOCAL_MAPVALUES_BUTTON, 2);
      if (program_state->regular_local_map_display){
	G_deactivate(LOCAL_BACKGROUND);
	G_activate(LOCAL_MAPVALUES);
      }
      else{
	G_activate(LOCAL_BACKGROUND);
	G_deactivate(LOCAL_MAPVALUES);
      }
      G_display_switch(LOCAL_BACKGROUND, 0);
      G_display_matrix(LOCAL_MAPVALUES);
      G_display_robot(LOCAL_ROBOT_, 0.0, 0.0, 90.0,
		      robot_specifications->num_sensors, 
		      robot_state->sensor_values);
      G_display_switch(DISPLAY_LOCAL_MAPVALUES_BUTTON, 
		       program_state->regular_local_map_display);
    }


    else if (button == RIGHT_BUTTON &&
	     G_mouse_event_at(LOCAL_ROBOT_, mouse_events, &number)){
      G_display_switch(DISPLAY_LOCAL_MAPVALUES_BUTTON, 2);
      G_activate(LOCAL_MAPVALUES);
      G_display_matrix(LOCAL_MAPVALUES);
      G_display_robot(LOCAL_ROBOT_, 0.0, 0.0, 90.0,
		      robot_specifications->num_sensors, 
		      robot_state->sensor_values);
      if (!program_state->regular_local_map_display)
	G_deactivate(LOCAL_MAPVALUES);
      G_display_switch(DISPLAY_LOCAL_MAPVALUES_BUTTON, 
		       program_state->regular_local_map_display);
    }


    else if (G_mouse_event_at(QUIT_BUTTON, mouse_events, &number)){
      G_display_switch(QUIT_BUTTON, 1);
      usleep(200000);
      program_state->quit = 1;
    }


    else if (G_mouse_event_at(SCRIPT_BUTTON, mouse_events, &number)){
      
      if (!program_state->processing_script)
	initiate_read_script(robot_state, program_state, robot_specifications,
			     SCRIPT_NAME);      
      else
	close_script(robot_state, program_state, robot_specifications);
    }



    /* insert new graphics object here XXXXX */
    
    
    else if (button == RIGHT_BUTTON)
      G_display_all();


  }
  

  return test;
}


