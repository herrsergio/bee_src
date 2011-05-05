
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




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

int SONAR_ROBOT;
int LASER_ROBOT;
int SONAR_ROBOT_BACKGROUND;
int TITLE_BUTTON;
int BUSY_BUTTON;
int IN_MOTION_BUTTON;
int GLOBAL_ROBOT;
int GLOBAL_ROBOT_BACKGROUND;
int PATH;
int SCRIPT_PATH;
int CAMERA_TILT;
int CAMERA_PAN;
int SCRIPT_CAMERA_TILT;
int SCRIPT_CAMERA_PAN;
/*int LOGGING_BUTTON;*/
/*int SCRIPT_BUTTON;*/
/*int EPISODE_DIAL;*/
/*int EVENT_DIAL;*/
/*int DECREASE_EPISODE_BUTTON;*/
/*int INCREASE_EPISODE_BUTTON;*/
/*int DECREASE_EVENT_BUTTON;*/
/*int INCREASE_EVENT_BUTTON;*/
int SCRIPT_EVENT_DATE_BUTTON;
int RESET_ROBOT_BUTTON;
int PAN_TILT_BACKGROUND;
int PSEUDO_PICTURE;

int CONNECT_BASE_BUTTON;
int CONNECT_PANTILT_BUTTON;
int CONNECT_MAP_BUTTON;
int CONNECT_PLAN_BUTTON;
int CONNECT_SPEECH_BUTTON;
int CONNECT_BUTTONS_BUTTON;
int CONNECT_BASESERVER_BUTTON;
int CONNECT_SIMULATOR_BUTTON;
int CONNECT_SONARINT_BUTTON;
int CONNECT_LASERINT_BUTTON;
int CONNECT_CAMERA_BUTTON;

int ACQUIRE_CAMERA_IMAGE_BUTTON;
int CONTINUOUS_CAMERA_IMAGE_BUTTON;
#ifdef UNIBONN
int LEFT_CAMERA_BUTTON;
int RIGHT_CAMERA_BUTTON;
int CONNECT_ARM_BUTTON;
int CONNECT_FLOW_BUTTON;
int CONNECT_SUNVIS_BUTTON;
int CONNECT_TRACKER_BUTTON;
int CAMERA_X_DISPLAY_BUTTON;
int CAMERA_OBJECTS_BUTTON;
int CAMERA_MAPS_BUTTON;
int CAMERA_COLLI_LINES_BUTTON;
int HUNTING_BUTTON;
int NUM_OBJECTS_BUTTON;
int SONAR_BUTTON;
int TARGET_OBJECTS;
int GOALS;
int MARKERS;
int TRACKER_BUTTON;
int ARM_MOVE_OUT_IN_BUTTON;
int ARM_PICK_BUTTON;
int ARM_CLOSE_OPEN_GRIPPER_BUTTON;
int ARM_MAST_POSITION_BUTTON;
int ARM_GRIPPER_ORIENTATION_BUTTON;
int ARM_GRIPPER_PAD_POSITION_BUTTON;
int ARM_IN_OUT_BUTTON;
#endif /* UNIBONN */

#ifdef CD_VERSION
int CONNECT_CD_BUTTON;
int CD_BUTTONS[NUM_CD_TEXTS];
#endif /* CD_VERSION */

#ifdef TOURGUIDE_VERSION
int TOUR_GOALS[MAX_NUM_TOURS];
int TOUR_ACTIVE_GOALS[MAX_NUM_TOURS];
int TOUR_OBJECTS[MAX_NUM_TOURS];
int TOUR_LEARN_BUTTON;
int TOUR_GIVE_BUTTON;
int TOUR_SAVE_BUTTON;
int TOUR_LOAD_BUTTON;
#endif /* TOURGUIDE_VERSION */


int TARGET_POINT_LOCAL;
int TARGET_POINT_GLOBAL;
int SONAR_ROBOT_SUBTITLE;
int SONAR_ROBOT_SUBTITLE2;
int SONAR_ROBOT_SUBTITLE3;
int PANTILT_SUBTITLE;
int GLOBAL_ROBOT_SUBTITLE;
int ROBOT_TRANS_SET_VELOCITY;
int ROBOT_ROT_SET_VELOCITY;
int ROBOT_TRANS_VELOCITY;
int ROBOT_ROT_VELOCITY;
int ROBOT_TRANS_SET_ACCELERATION;
int ROBOT_ROT_SET_ACCELERATION;
int POSITION_TEXT_BUTTON;
int OCCUPANCY_MAP;
int DUMP_BUTTON;
int AUTONOMOUS_BUTTON;
int STATUS_NEW_AREA_BUTTON;
int STATUS_TOTAL_AREA_BUTTON;
int STATUS_ADVANCEMENT_BUTTON;
int STATUS_BACKGROUND;
int BASE_MODE_BUTTON;
int EXPL_RESET_BUTTON;
int QUIT_BUTTON;
int REFRESH_BUTTON;
int RED_LIGHT_BUTTON;
int GREEN_LIGHT_BUTTON;
int YELLOW_LIGHT_BUTTON;
int BLUE_LIGHT_BUTTON;
int RED_PUSHED_BUTTON;
int GREEN_PUSHED_BUTTON;
int YELLOW_PUSHED_BUTTON;
int BLUE_PUSHED_BUTTON;
int MAP_UPDATE_BUTTON;
int BATTERY_STATUS;
int STOP_ROBOT;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




#ifdef CD_VERSION


extern CD_msg_type MSGs[NUM_MSGs];

#ifdef UNIBONN

cd_texts_type CD_TEXTS[NUM_CD_ROWS] =
{
  {MSG_gong, -1},
  {MSG_xylophon, -1},
  {MSG_tata, -1},
  {MSG_akkord, -1},
  {MSG_fanfare, -1},
  {MSG_dont_loose, -1},
  {MSG_wau, -1},
  {MSG_wauwau, -1},
  {MSG_groove, -1},
  {MSG_no_education, -1},
  {MSG_i_quit, -1},
  {MSG_lok, -1},
  {MSG_ruelps, -1},
  {MSG_lachen, -1},
  {MSG_lachen2, -1},
  {MSG_pfeife, -1},
  {MSG_star_wars, -1},
  {MSG_pers_diener, 0},
  {MSG_rhein_tach_auch, 0},
  {MSG_rhein_driss_he, 0},
  {MSG_rhein_jrade_muell, 0},
  {MSG_rhein_zement_ens, 0},
  {MSG_rhein_fott_damit, 0},
  {174, 0},
  {309, -1},
  {310, -1},
  {311, -1},
  {312, -1}
};

#else

cd_texts_type CD_TEXTS[NUM_CD_ROWS] =
{
  {MSG_gong, -1},
  {MSG_xylophon, -1},
  {MSG_tata, -1},
  {MSG_akkord, -1},
  {MSG_fanfare, -1},
  {MSG_dont_loose, -1},
  {MSG_wau, -1}
};

#endif

#endif /* CD_VERSION */



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/**** memory for the downsampled picture display ****/

float pseudo_picture[PSEUDO_PICTURE_SIZE_HORIZONTAL
		     *PSEUDO_PICTURE_SIZE_VERTICAL];
int pseudo_picture_active[PSEUDO_PICTURE_SIZE_HORIZONTAL
			  *PSEUDO_PICTURE_SIZE_VERTICAL];


/**** memory for map display ***/

int *map_active = NULL;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/************************************************************************
 *
 *   NAME:         init_graphics
 *                 
 *   FUNCTION:     Initiaizes graphics window
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                                                   
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void init_graphics(ALL_PARAMS)
{
  char button_text[128];
  float max_right = 12.8;
  float worldsize;

  /* =============================================================
     ====================  1) DISPLAY  ===========================
     ============================================================= */

  G_set_display(program_state->do_Xdisplay);
  
  
  /* =============================================================
     ====================  2) INITIALIZATIONS  ===================
     ============================================================= */

  if (!program_state->graphics_initialized){
    
    static char *myfonts1[] = {"5x8", "6x10", "7x13bold", 
				 "9x15", "10x20", "12x24", 
				 "lucidasans-bold-18"};
    static char *myfonts2[] = {"5x8", "5x8", "6x10", 
				 "9x15", "10x20", "12x24", 
				 "lucidasans-bold-14"};
    if (robot_specifications->X_window_size >= 60.0)
      G_initialize_fonts(7, myfonts1);
    else
      G_initialize_fonts(7, myfonts2);

    G_initialize_graphics("BeeSoft Commander", 
			  robot_specifications->X_window_size, 10.0, 
			  C_STEELBLUE4 /*C_TURQUOISE4*/);

    G_set_matrix_display_style(1);
    
    /* =============================================================
       ====================  3) SPECIFICATIONS  ====================
       ============================================================= */
   

    
    /******** SONAR_ROBOT_BACKGROUND **************************************/
    {
      int switch_num                      = 1;
      static float switch_pos[]           = {-0.5, 4.8, 6.0, 9.9};
      static char *switch_texts[]         = {""};
      static int switch_fonts[]           = {2};
      static int switch_background_color[]= {C_GREY90};
      static int switch_frame_color[]     = {C_GREY40};
      static int switch_text_color[]      = {NO_COLOR};
      
      SONAR_ROBOT_BACKGROUND = G_create_switch_object(switch_pos, switch_num, 
						      switch_texts,
						      switch_background_color,
						      switch_frame_color, 
						      switch_text_color,
						      switch_fonts);
      
    }

    /******** LASER_ROBOT *************************************/
    {
      static float pos_r[]                 = {0.0, 3.9, 6.0, 9.9};
      static char *text_r                  = "ROBOT";
      static int robot_font                = 4;
      static int colors_r[]                = {NO_COLOR, NO_COLOR, NO_COLOR,
						NO_COLOR, C_YELLOW, NO_COLOR,
						C_GREY90, NO_COLOR};
      
      LASER_ROBOT =
	G_create_robot_object(pos_r, text_r, 
			      -robot_specifications->max_laser_range *1.05,
			      robot_specifications->max_laser_range *1.05,
			      -robot_specifications->max_laser_range *1.05,
			      robot_specifications->max_laser_range *1.05,
			      0.0, 0.0, 90.0, 
			      robot_specifications->robot_size,
			      robot_specifications->num_laser_sensors, 
			      robot_specifications->max_laser_range,
			      dummy_sensors,
			      robot_specifications->laser_angles, 
			      colors_r, robot_font);
    }    

    /******** TARGET_POINT_LOCAL **************************************/
    {
      static float pos_r[]                   = {0.0, 3.9, 6.0, 9.9};
      int num_mar                            = 1;
      static char *text_mar[]                = {""};
      int connected_mar                      = 0;
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = {C_RED};
      static int mar_text_color[]            = {NO_COLOR};
      static int mar_fonts[]                 = {2};
      
      TARGET_POINT_LOCAL =
	G_create_markers_object(pos_r, connected_mar, 
				0.6 * robot_specifications->robot_size,
				-robot_specifications->max_sonar_range *1.05,
				robot_specifications->max_sonar_range *1.05,
				-robot_specifications->max_sonar_range *1.05,
				robot_specifications->max_sonar_range *1.05,
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, mar_foreground_color, 
				mar_text_color, mar_fonts);
    }


    
    /******** SONAR_ROBOT *************************************/
    {
      static float pos_r[]                 = {0.0, 3.9, 6.0, 9.9};
      static char *text_r                  = "ROBOT";
      static int robot_font                = 4;
      static int colors_r[]                = {NO_COLOR, NO_COLOR, C_GREY90,
						C_BLACK, C_BLUE, C_GREY40,
						NO_COLOR, NO_COLOR};
      
      SONAR_ROBOT =
	G_create_robot_object(pos_r, text_r, 
			      -robot_specifications->max_sonar_range *1.05,
			      robot_specifications->max_sonar_range *1.05,
			      -robot_specifications->max_sonar_range *1.05,
			      robot_specifications->max_sonar_range *1.05,
			      0.0, 0.0, 90.0, 
			      robot_specifications->robot_size,
			      robot_specifications->num_sonar_sensors, 
			      robot_specifications->max_sonar_range,
			      dummy_sensors,
			      robot_specifications->sonar_angles, 
			      colors_r, robot_font);
    }    
    




    /******** ROBOT_TRANS_VELOCITY *****************************/
    {
      static float value_pos[]            = {-0.4, -0.25, 6.3, 9.8};
      static char *value_text             = "T";
      static int value_font               = 2;
      int direction_value                 = 2;
      static float value                  = 1.0;
      float min_value                     = 0.0;
      float max_value                     = MAX_TRANS_VELOCITY;
      static int value_colors[]           = {C_GREY70, C_MEDIUMVIOLETRED,
					       C_GREY40, C_WHITE}; 
      
      
      ROBOT_TRANS_VELOCITY = G_create_value_object(value_pos, value_text, 
					  direction_value, value, 
					  min_value,
					  max_value, value_colors, 
					  value_font);
    }

    
    /******** ROBOT_ROT_VELOCITY *****************************/
    {
      static float value_pos[]            = {-0.2, -0.05, 6.3, 9.8};
      static char *value_text             = "R";
      static int value_font               = 2;
      int direction_value                 = 2;
      static float value                  = 1.0;
      float min_value                     = 0.0;
      float max_value                     = MAX_ROT_VELOCITY;
      static int value_colors[]           = {C_GREY70, C_MEDIUMVIOLETRED,
					       C_GREY40, C_WHITE}; 
      
      
      ROBOT_ROT_VELOCITY = G_create_value_object(value_pos, value_text, 
					  direction_value, value, 
					  min_value,
					  max_value, value_colors, 
					  value_font);
    }


    /******** ROBOT_TRANS_SET_VELOCITY *****************************/
    {
      static float value_pos[]            = {3.95, 4.1, 6.3, 9.8};
      static char *value_text             = "T";
      static int value_font               = 2;
      int direction_value                 = 2;
      static float value                  = 1.0;
      float min_value                     = 0.0;
      float max_value                     = MAX_TRANS_VELOCITY;
      static int value_colors[]           = {C_GREY70, C_MEDIUMVIOLETRED,
					       C_GREY40, C_WHITE}; 
      
      
      ROBOT_TRANS_SET_VELOCITY = G_create_value_object(value_pos, value_text, 
					  direction_value, value, 
					  min_value,
					  max_value, value_colors, 
					  value_font);
    }

    
    /******** ROBOT_ROT_SET_VELOCITY *****************************/
    {
      static float value_pos[]            = {4.15, 4.3, 6.3, 9.8};
      static char *value_text             = "R";
      static int value_font               = 2;
      int direction_value                 = 2;
      static float value                  = 1.0;
      float min_value                     = 0.0;
      float max_value                     = MAX_ROT_VELOCITY;
      static int value_colors[]           = {C_GREY70, C_MEDIUMVIOLETRED,
					       C_GREY40, C_WHITE}; 
      
      
      ROBOT_ROT_SET_VELOCITY = G_create_value_object(value_pos, value_text, 
					  direction_value, value, 
					  min_value,
					  max_value, value_colors, 
					  value_font);
    }

   
    /******** ROBOT_TRANS_SET_ACCELERATION *****************************/
    {
      static float value_pos[]            = {4.35, 4.5, 6.3, 9.8};
      static char *value_text             = "T";
      static int value_font               = 2;
      int direction_value                 = 2;
      static float value                  = 1.0;
      float min_value                     = 0.0;
      float max_value                     = MAX_TRANS_ACCELERATION;
      static int value_colors[]           = {C_GREY70, C_MEDIUMPURPLE3,
					       C_GREY40, C_WHITE}; 
      
      
      ROBOT_TRANS_SET_ACCELERATION = G_create_value_object(value_pos, value_text, 
					  direction_value, value, 
					  min_value,
					  max_value, value_colors, 
					  value_font);
    }
 
    
    /******** ROBOT_ROT_SET_ACCELERATION *****************************/
    {
      static float value_pos[]            = {4.55, 4.7, 6.3, 9.8};
      static char *value_text             = "R";
      static int value_font               = 2;
      int direction_value                 = 2;
      static float value                  = 1.0;
      float min_value                     = 0.0;
      float max_value                     = MAX_ROT_ACCELERATION;
      static int value_colors[]           = {C_GREY70,C_MEDIUMPURPLE3 ,
					       C_GREY40, C_WHITE}; 
      
      
      ROBOT_ROT_SET_ACCELERATION = G_create_value_object(value_pos, value_text, 
					  direction_value, value, 
					  min_value,
					  max_value, value_colors, 
					  value_font);
    }

   
    /******** RED_LIGHT_BUTTON **************************************/
    {
      int switch_num                      = 3;
      static float switch_pos[]           = {0.05,0.20,9.65,9.8};
      static char *switch_texts[]         = {"", "", "*"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_GREY90, C_RED, C_RED};
      static int switch_frame_color[]     = {C_GREY70, C_GREY70, C_GREY70};
      static int switch_text_color[]      = {C_BLACK, C_BLACK, C_BLACK};
      
      RED_LIGHT_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,
				 switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
   
   
    /******** YELLOW_LIGHT_BUTTON **************************************/
    {
      int switch_num                      = 3;
      static float switch_pos[]           = {0.05,0.20,9.47,9.62};
      static char *switch_texts[]         = {"", "", "*"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_GREY90, C_YELLOW, C_YELLOW};
      static int switch_frame_color[]     = {C_GREY70, C_GREY70, C_GREY70};
      static int switch_text_color[]      = {C_BLACK, C_BLACK, C_BLACK};
      
      YELLOW_LIGHT_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,
				 switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
   
    /******** GREEN_LIGHT_BUTTON **************************************/
    {
      int switch_num                      = 3;
      static float switch_pos[]           = {0.05,0.20,9.29,9.44};
      static char *switch_texts[]         = {"", "", "*"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_GREY90,C_LAWNGREEN,C_LAWNGREEN};
      static int switch_frame_color[]     = {C_GREY70, C_GREY70, C_GREY70};
      static int switch_text_color[]      = {C_BLACK, C_BLACK, C_BLACK};

      GREEN_LIGHT_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,
				 switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
   
    /******** BLUE_LIGHT_BUTTON **************************************/
    {
      int switch_num                      = 3;
      static float switch_pos[]           = {0.05,0.20,9.11,9.26};
      static char *switch_texts[]         = {"", "", "*"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_GREY90, C_BLUE, C_BLUE};
      static int switch_frame_color[]     = {C_GREY70, C_GREY70, C_GREY70};
      static int switch_text_color[]      = {C_BLACK, C_BLACK, C_WHITE};
      
      BLUE_LIGHT_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,
				 switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
    
   
    /******** RED_PUSHED_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {0.23,0.37,9.65,9.8};
      static char *switch_texts[]         = {"", ""};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_RED};
      static int switch_frame_color[]     = {C_GREY70, C_GREY70};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      RED_PUSHED_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,
				 switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
   
   
    /******** YELLOW_PUSHED_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {0.23,0.37,9.47,9.62};
      static char *switch_texts[]         = {"", ""};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_YELLOW};
      static int switch_frame_color[]     = {C_GREY70, C_GREY70};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      YELLOW_PUSHED_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,
				 switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
   
    /******** GREEN_PUSHED_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {0.23,0.37,9.29,9.44};
      static char *switch_texts[]         = {"", ""};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_LAWNGREEN};
      static int switch_frame_color[]     = {C_GREY70, C_GREY70};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      GREEN_PUSHED_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,
				 switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
   
    /******** BLUE_PUSHED_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {0.23,0.37,9.11,9.26};
      static char *switch_texts[]         = {"", ""};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_BLUE};
      static int switch_frame_color[]     = {C_GREY70, C_GREY70};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      BLUE_PUSHED_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,
				 switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
      
    /******** IN_MOTION_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {7.7, 12.7, 10.1, 10.3};
      static char *switch_texts[]         = {"...moving", "...moving"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {NO_COLOR, NO_COLOR};
      static int switch_frame_color[]     = {NO_COLOR, NO_COLOR};
      static int switch_text_color[]      = {C_GREY90, C_RED};
      
      IN_MOTION_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,
				 switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
    
    /******** BUSY_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {7.7, 12.7, 10.3, 10.5};
      static char *switch_texts[]         = {"...busy", "...busy"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {NO_COLOR, NO_COLOR};
      static int switch_frame_color[]     = {NO_COLOR, NO_COLOR};
      static int switch_text_color[]      = {C_GREY90, C_RED};
      
      BUSY_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,
				 switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
     
    
    /******** GLOBAL_ROBOT_BACKGROUND **************************************/
    {
      int switch_num                      = 1;
      static float switch_pos[]           = {8.9, 12.8, 6.0, 9.9};
      static char *switch_texts[]         = {""};
      static int switch_fonts[]           = {2};
      static int switch_background_color[]= {C_GREY90};
      static int switch_frame_color[]     = {C_GREY40};
      static int switch_text_color[]      = {NO_COLOR};
      
      GLOBAL_ROBOT_BACKGROUND = G_create_switch_object(switch_pos, switch_num, 
						      switch_texts,
						      switch_background_color,
						      switch_frame_color, 
						      switch_text_color,
						      switch_fonts);
      
    }

    /******** OCCUPANCY_MAP *****************************/
    {
      static float matrix_pos[]             = {8.9, 12.8, 6.0, 9.9};
      char *matrix_text                     = "map";
      static int matrix_font                = 2;
      float min_value_matrix                = 0.0;
      float max_value_matrix                = 1.0000001;
      static int matrix_colors[]            = 
	{NO_COLOR, C_GREY40, NO_COLOR}; 

      
      OCCUPANCY_MAP
	= G_create_matrix_object(matrix_pos, matrix_text, 
				 robot_specifications->occupancy_values,
				 robot_specifications->map_active,
				 robot_specifications->global_map_dim_x,
				 robot_specifications->global_map_dim_y,
				 min_value_matrix, max_value_matrix,
				 matrix_colors, matrix_font);
    }

    
    /******** PATH *************************************************/
    {
      static float pos_mar[]                   = {8.9, 12.8, 6.0, 9.9};
      int num_mar                            = 1;
      static char *text_mar[]                = {"(path)"};
      int connected_mar                      = 1;
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = {C_GREY40};
      static int mar_text_color[]            = {NO_COLOR};
      static int mar_fonts[]                 = {2};
      
      PATH 
	= G_create_markers_object(pos_mar, connected_mar,
				  robot_specifications->robot_size * 0.3,
				  0.0, 
				  robot_specifications->global_worldsize_x,
				  0.0, 
				  robot_specifications->global_worldsize_y,
				  num_mar, text_mar, mar_background_color, 
				  mar_frame_color, mar_foreground_color, 
				  mar_text_color, mar_fonts);
      
    }

#ifdef UNIBONN
    /******** GOALS **************************************/
    {
      static float pos_r[]                 = {8.9, 12.8, 6.0, 9.9};
      int num_mar                            = MAX_NUMBER_GOALS;
      static char *text_mar[]                =
	{"0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
	   "10", "11", "12", "13", "14", "15", "16", "17", "18", "19"};
      int connected_mar                      = 0;
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = 
	{C_MEDIUMVIOLETRED,
	   C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED,
	   C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED,
	   C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, 
	   C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED,
	   C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED,
	   C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED,
	   C_MEDIUMVIOLETRED};
      static int mar_text_color[]            = 
	{NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
	   NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, 
	   NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, 
	   NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR};
      static int mar_fonts[]                 = 
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
      

      GOALS =
	G_create_markers_object(pos_r, connected_mar, 
				robot_specifications->robot_size,
				0.0, robot_specifications->global_worldsize_x,
				0.0, robot_specifications->global_worldsize_y,
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, mar_foreground_color, 
				mar_text_color, mar_fonts);
    }
    
    /******** TARGET_OBJECTS **************************************/
    {
      static float pos_r[]                 = {8.9, 12.8, 6.0, 9.9};
      int num_mar                            = 4;
      static char *text_mar[]                =
	{"0", "1", "2", "3", "?"};
      int connected_mar                      = 0;
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = 
	{C_LAWNGREEN, C_RED, C_BLUE, C_SIENNA4, NO_COLOR};
      static int mar_text_color[]            = 
	{NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR};
      static int mar_fonts[]                 = 
	{2, 2, 2, 2};
      

      TARGET_OBJECTS =
	G_create_markers_object(pos_r, connected_mar, 
				robot_specifications->robot_size,
				0.0, robot_specifications->global_worldsize_x,
				0.0, robot_specifications->global_worldsize_y,
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, mar_foreground_color, 
				mar_text_color, mar_fonts);
    }




    

    

#endif /* UNIBONN */

#ifdef CD_VERSION

    /******** CD_BUTTONS  **************************************/
    {
      int i, j, k;
      int text_undefined = 0;
      char txt[80];

      int switch_num                      = 2;
      static float switch_pos[4];
      static char *switch_texts[2];
      static int switch_fonts[]           = {0,0};
      static int switch_background_color[]= {C_GREY90,C_YELLOW};
      static int switch_frame_color[]     = {C_GREY40,C_GREY40};
      static int switch_text_color[]      = {C_BLACK,C_BLACK};

      switch_texts[0] = txt;
      switch_texts[1] = txt;

      for (i = 0, k = 0; i < NUM_CD_COLUMNS; i++)
	for (j = 0; j < NUM_CD_ROWS; j++, k++){
	  switch_pos[0] = RIGHT_WINDOW_BORDER 
	    + (BUTTON_SEPARATOR * ((float) (i+1)))
	      + (1.5 * ((float) (i)));
	  switch_pos[1] = switch_pos[0] + 1.5;
	  switch_pos[3] = TOP_BUTTON_BORDER2
	    - (((TOP_BUTTON_BORDER2 
		 - BOTTOM_BUTTON_BORDER
		 + BUTTON_SEPARATOR) / NUM_CD_ROWS) * ((float) (j)));
	  switch_pos[2] = switch_pos[3] -
	    ((TOP_BUTTON_BORDER2 
	      - BOTTOM_BUTTON_BORDER
	      + BUTTON_SEPARATOR) / NUM_CD_ROWS) + BUTTON_SEPARATOR;
	  
	  if (max_right < switch_pos[1])
	    max_right = switch_pos[1];



	  strncpy(txt, MSGs[CD_TEXTS[k].msg].text, 20);

	  CD_BUTTONS[k] = 
	    G_create_switch_object(switch_pos, switch_num, 
				   switch_texts,
				   switch_background_color,
				   switch_frame_color, 
				   switch_text_color,
				   switch_fonts);
	}
    }    
#endif /* CD_VERSION */




#ifdef TOURGUIDE_VERSION

    /******** TOUR_GOALS **************************************/
    {
      static float pos_r[]                 = {8.9, 12.8, 6.0, 9.9};
      int num_mar                            = MAX_NUMBER_TOUR_GOALS;
      static char *text_mar[]                =
	{"0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
	   "10", "11", "12", "13", "14", "15", "16", "17", "18", "19"};
      int connected_mar                      = 1;
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = 
	{C_MEDIUMVIOLETRED,
	   C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED,
	   C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED,
	   C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, 
	   C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED,
	   C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED,
	   C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED, C_MEDIUMVIOLETRED,
	   C_MEDIUMVIOLETRED};
      static int mar_text_color[]            = 
	{C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
	   C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, 
	   C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, 
	   C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK};
      static int mar_fonts[]                 = 
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
      int i;
      
      for (i = 0; i < MAX_NUM_TOURS; i++)
	TOUR_GOALS[i] =
	  G_create_markers_object(pos_r, connected_mar, 
				  robot_specifications->robot_size,
				  0.0, robot_specifications->global_worldsize_x,
				  0.0, robot_specifications->global_worldsize_y,
				  num_mar, text_mar, mar_background_color, 
				  mar_frame_color, mar_foreground_color, 
				  mar_text_color, mar_fonts);
    }
    

    /******** TOUR_ACTIVE_GOALS **************************************/
    {
      static float pos_r[]                 = {8.9, 12.8, 6.0, 9.9};
      int num_mar                            = MAX_NUMBER_TOUR_GOALS;
      static char *text_mar[]                =
	{"0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
	   "10", "11", "12", "13", "14", "15", "16", "17", "18", "19"};
      int connected_mar                      = 1;
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = 
	{C_BLUE,
	   C_BLUE, C_BLUE, C_BLUE,
	   C_BLUE, C_BLUE, C_BLUE,
	   C_BLUE, C_BLUE, C_BLUE, 
	   C_BLUE, C_BLUE, C_BLUE,
	   C_BLUE, C_BLUE, C_BLUE,
	   C_BLUE, C_BLUE, C_BLUE,
	   C_BLUE};
      static int mar_text_color[]            = 
	{C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
	   C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, 
	   C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, 
	   C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK};
      static int mar_fonts[]                 = 
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
      int i;
      
      for (i = 0; i < MAX_NUM_TOURS; i++)
	TOUR_ACTIVE_GOALS[i] =
	  G_create_markers_object(pos_r, connected_mar, 
				  robot_specifications->robot_size,
				  0.0, robot_specifications->global_worldsize_x,
				  0.0, robot_specifications->global_worldsize_y,
				  num_mar, text_mar, mar_background_color, 
				  mar_frame_color, mar_foreground_color, 
				  mar_text_color, mar_fonts);
    }
    
    /******** TOUR_OBJECTS **************************************/
    {
      static float pos_r[]                 = {8.9, 12.8, 6.0, 9.9};
      int num_mar                            = MAX_NUMBER_TOUR_GOALS;
      static char *text_mar[]                =
	{"0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
	   "10", "11", "12", "13", "14", "15", "16", "17", "18", "19"};
      int connected_mar                      = 0;
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = 
	{C_TURQUOISE4,
	   C_TURQUOISE4, C_TURQUOISE4, C_TURQUOISE4,
	   C_TURQUOISE4, C_TURQUOISE4, C_TURQUOISE4,
	   C_TURQUOISE4, C_TURQUOISE4, C_TURQUOISE4, 
	   C_TURQUOISE4, C_TURQUOISE4, C_TURQUOISE4,
	   C_TURQUOISE4, C_TURQUOISE4, C_TURQUOISE4,
	   C_TURQUOISE4, C_TURQUOISE4, C_TURQUOISE4,
	   C_TURQUOISE4};
      static int mar_text_color[]            = 
	{C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
	   C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, 
	   C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, 
	   C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK};
      static int mar_fonts[]                 = 
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
      int i;
      
      for (i = 0; i < MAX_NUM_TOURS; i++)
	TOUR_OBJECTS[i] =
	  G_create_markers_object(pos_r, connected_mar, 
				  robot_specifications->robot_size,
				  0.0, robot_specifications->global_worldsize_x,
				  0.0, robot_specifications->global_worldsize_y,
				  num_mar, text_mar, mar_background_color, 
				  mar_frame_color, mar_foreground_color, 
				  mar_text_color, mar_fonts);
    }
    /******** TOUR_LEARN_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(0), BAR_ROW(3)};
      static char *switch_texts[]         = 
	{"learn tour", "...learning"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_LAWNGREEN};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      TOUR_LEARN_BUTTON = G_create_switch_object(switch_pos, 
						  switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    
 
    /******** TOUR_GIVE_BUTTON **************************************/
    {
      int switch_num                      = 3;
      static float switch_pos[]           = {BAR_COLUMN(3), BAR_ROW(3)};
      static char *switch_texts[]         = 
	{"give tour", "...waiting", "...giving tour"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= 
	{C_GREY90, C_YELLOW, C_LAWNGREEN};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK, C_BLACK};
      
      TOUR_GIVE_BUTTON = G_create_switch_object(switch_pos, 
						  switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    
 
    /******** TOUR_SAVE_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(1), BAR_ROW(3)};
      static char *switch_texts[]         = 
	{"saving tour", "...saving"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_LAWNGREEN};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      TOUR_SAVE_BUTTON = G_create_switch_object(switch_pos, 
						  switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    
 
    /******** TOUR_LOAD_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(2), BAR_ROW(3)};
      static char *switch_texts[]         = 
	{"loading tour", "...loading tour"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_LAWNGREEN};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      TOUR_LOAD_BUTTON = G_create_switch_object(switch_pos, 
						  switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    
#endif /* TOURGUIDE_VERSION */
    
    /******** TITLE_BUTTON: RLA *******************************************/
    {
      int switch_num                      = 4;
      static float switch_pos[]           = {-1.0, 12.8, 10.0, 10.4};
      static char *switch_texts[]         = 
	{"...initializing -- please wait",
	   "RWI BeeSoft Commander", 
	   "(c) Sebastian Thrun 1993-97",
	 "...connecting to TCX -- please wait"};
      static int switch_fonts[]           = {6, 6, 6, 6};
      static int switch_background_color[]= 
      {C_RED,C_GREY90,C_GREY90, C_YELLOW};
      static int switch_frame_color[]     =
      {C_GREY40,C_GREY40,C_GREY40,C_GREY40};
      static int switch_text_color[]      = 
	{C_GREY90,C_ORANGERED4,C_ORANGERED4,C_ORANGERED4};

      switch_pos[1] = max_right;
      
      TITLE_BUTTON = G_create_switch_object(switch_pos, switch_num, 
					    switch_texts,
					    switch_background_color,
					    switch_frame_color, 
					    switch_text_color,
					    switch_fonts);

    }



    /******** GLOBAL_ROBOT *************************************/
    {
      static float pos_r[]                 = {8.9, 12.8, 6.0, 9.9};
      static char *text_r                  = "ROBOT";
      static int robot_font                = 4;
      static int colors_r[]                = {NO_COLOR, C_GREY40, C_GREY90,
						C_BLACK, C_BLUE, C_GREY40,
						C_GREY90, NO_COLOR};
      

      GLOBAL_ROBOT =
	G_create_robot_object(pos_r, text_r, 
			      0.0,
			      robot_specifications->global_worldsize_x,
			      0.0,
			      robot_specifications->global_worldsize_y,
			      robot_state->x, robot_state->y,
			      robot_state->orientation, 
			      robot_specifications->robot_size,
			      0, 0.0,
			      dummy_sensors, dummy_sensors,
			      colors_r, robot_font);
    }    


    /******** TARGET_POINT_GLOBAL **************************************/
    {
      static float pos_r[]                 = {8.9, 12.8, 6.0, 9.9};
      int num_mar                            = 1;
      static char *text_mar[]                = {""};
      int connected_mar                      = 0;
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = {C_RED};
      static int mar_text_color[]            = {NO_COLOR};
      static int mar_fonts[]                 = {2};
      
      TARGET_POINT_GLOBAL =
	G_create_markers_object(pos_r, connected_mar, 
				0.6 * robot_specifications->robot_size,
				0.0, robot_specifications->global_worldsize_x,
				0.0, robot_specifications->global_worldsize_y,
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, mar_foreground_color, 
				mar_text_color, mar_fonts);
    }

    /******** POSITION_TEXT_BUTTON **************************************/
    {
      int switch_num                      = 1;
      static float switch_pos[]           = {8.9, 12.8, 5.7, 6.0};
      static char *switch_texts[]         = {""};
      static int switch_fonts[]           = {2};
      static int switch_background_color[]= {C_GREY40};
      static int switch_frame_color[]     = {C_GREY70};
      static int switch_text_color[]      = {C_WHITE};
      
      POSITION_TEXT_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,
				 switch_frame_color, 
				 switch_text_color, switch_fonts);
      
    }
    
    /******** SCRIPT_PATH **************************************/
    {
      static float pos_mar[]                   = {8.9, 12.8, 6.0, 9.9};
      int num_mar                            = 1;
      static char *text_mar[]                = {"(path)"};
      int connected_mar                      = 1;
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = {C_RED};
      static int mar_text_color[]            = {NO_COLOR};
      static int mar_fonts[]                 = {2};
      
      
      SCRIPT_PATH 
	= G_create_markers_object(pos_mar, connected_mar,
				  robot_specifications->robot_size * 0.5,
				  0.0, 
				  robot_specifications->global_worldsize_x,
				  0.0, 
				  robot_specifications->global_worldsize_y,
				  num_mar, text_mar, mar_background_color, 
				  mar_frame_color, mar_foreground_color, 
				  mar_text_color, mar_fonts);
    }
    
    /******** CONNECT_BASE_BUTTON **************************************/
    {


      int switch_num                      = 4;
      static float switch_pos[]           = {CONNECT_COORDNIATES(0)};
      static char *switch_texts[]         = 
	{"(COLLI)", "COLLI", "?COLLI?",
	   "?TCX?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY50, C_GREY70, C_GREY70, C_GREY70};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_WHITE, C_BLUE, C_RED, C_RED};
      
      CONNECT_BASE_BUTTON = G_create_switch_object(switch_pos, switch_num, 
					      switch_texts,
					      switch_background_color,
					      switch_frame_color, 
					      switch_text_color,
					      switch_fonts);
    }    

    /******** CONNECT_PANTILT_BUTTON **************************************/
    {


      int switch_num                      = 4;
      static float switch_pos[]           = {CONNECT_COORDNIATES(1)};
      static char *switch_texts[]         = 
	{"(PAN/TILT)", "PAN/TILT", "?PAN/TILT?",
	   "?TCX?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY50, C_GREY70, C_GREY70, C_GREY70};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_WHITE, C_BLUE, C_RED, C_RED};
      
      CONNECT_PANTILT_BUTTON = G_create_switch_object(switch_pos, switch_num, 
					      switch_texts,
					      switch_background_color,
					      switch_frame_color, 
					      switch_text_color,
					      switch_fonts);
    }    
    
    /******** CONNECT_MAP_BUTTON **************************************/
    {


      int switch_num                      = 4;
      static float switch_pos[]           = {CONNECT_COORDNIATES(2)};
      static char *switch_texts[]         = 
	{"(MAP)", "MAP", "?MAP?",
	   "?TCX?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY50, C_GREY70, C_GREY70, C_GREY70};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_WHITE, C_BLUE, C_RED, C_RED};
      
      CONNECT_MAP_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    
    
    /******** CONNECT_PLAN_BUTTON **************************************/
    {

      int switch_num                      = 4;
      static float switch_pos[]           = {CONNECT_COORDNIATES(3)};
      static char *switch_texts[]         = 
	{"(PLAN)", "PLAN", "?PLAN?",
	   "?TCX?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY50, C_GREY70, C_GREY70, C_GREY70};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_WHITE, C_BLUE, C_RED, C_RED};
      
      CONNECT_PLAN_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    
    
    /******** CONNECT_BUTTONS_BUTTON **************************************/
    {

      int switch_num                      = 4;
      static float switch_pos[]           = {CONNECT_COORDNIATES(4)};
      static char *switch_texts[]         = 
	{"(BUTTONS)", "BUTTONS", "?BUTTONS?",
	   "?TCX?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY50, C_GREY70, C_GREY70, C_GREY70};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_WHITE, C_BLUE, C_RED, C_RED};
      
      CONNECT_BUTTONS_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    

    /******** CONNECT_SPEECH_BUTTON **************************************/
    {


      int switch_num                      = 4;
      static float switch_pos[]           = {CONNECT_COORDNIATES(5)};
      static char *switch_texts[]         = 
	{"(SPEECH)", "SPEECH", "?SPEECH?",
	   "?TCX?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY50, C_GREY70, C_GREY70, C_GREY70};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_WHITE, C_BLUE, C_RED, C_RED};
      
      CONNECT_SPEECH_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						     switch_texts,
						     switch_background_color,
						     switch_frame_color, 
						     switch_text_color,
						     switch_fonts);
    }    

    /******** CONNECT_CAMERA_BUTTON **************************************/
    {


      int switch_num                      = 4;
      static float switch_pos[]           = {CONNECT_COORDNIATES(6)};
      static char *switch_texts[]         = 
	{"(CAMERA)", "CAMERA", "?CAMERA?",
	   "?TCX?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY50, C_GREY70, C_GREY70, C_GREY70};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_WHITE, C_BLUE, C_RED, C_RED};
      
      CONNECT_CAMERA_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						     switch_texts,
						     switch_background_color,
						     switch_frame_color, 
						     switch_text_color,
						     switch_fonts);
    }    
    
    /******** CONNECT_BASESERVER_BUTTON **************************************/
    {


      int switch_num                      = 4;
      static float switch_pos[]           = {CONNECT_COORDNIATES(7)};
      static char *switch_texts[]         = 
	{"(B-SERVER)", "B-SERVER", "?B-SERVER?",
	   "?TCX?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY50, C_GREY70, C_GREY70, C_GREY70};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_WHITE, C_BLUE, C_RED, C_RED};
      
      CONNECT_BASESERVER_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						     switch_texts,
						     switch_background_color,
						     switch_frame_color, 
						     switch_text_color,
						     switch_fonts);
    }    
    /******** CONNECT_SIMULATOR_BUTTON **************************************/
    {


      int switch_num                      = 4;
      static float switch_pos[]           = {CONNECT_COORDNIATES(8)};
      static char *switch_texts[]         = 
	{"(SIMUL)", "SIMUL", "?SIMUL?",
	   "?TCX?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY50, C_GREY70, C_GREY70, C_GREY70};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_WHITE, C_BLUE, C_RED, C_RED};
      
      CONNECT_SIMULATOR_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						     switch_texts,
						     switch_background_color,
						     switch_frame_color, 
						     switch_text_color,
						     switch_fonts);
    }    
     /******** CONNECT_SONARINT_BUTTON **************************************/
    {


      int switch_num                      = 4;
      static float switch_pos[]           = {CONNECT_COORDNIATES(9)};
      static char *switch_texts[]         = 
	{"(SONARINT)", "SONARINT", "?SONARINT?",
	   "?TCX?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY50, C_GREY70, C_GREY70, C_GREY70};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_WHITE, C_BLUE, C_RED, C_RED};
      
      CONNECT_SONARINT_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						     switch_texts,
						     switch_background_color,
						     switch_frame_color, 
						     switch_text_color,
						     switch_fonts);
    }    
     /******** CONNECT_LASERINT_BUTTON **************************************/
    {


      int switch_num                      = 4;
      static float switch_pos[]           = {CONNECT_COORDNIATES(10)};
      static char *switch_texts[]         = 
	{"(LASERINT)", "LASERINT", "?LASERINT?",
	   "?TCX?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY50, C_GREY70, C_GREY70, C_GREY70};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_WHITE, C_BLUE, C_RED, C_RED};
      
      CONNECT_LASERINT_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						     switch_texts,
						     switch_background_color,
						     switch_frame_color, 
						     switch_text_color,
						     switch_fonts);
    }    
     
#ifdef UNIBONN
    /******** CONNECT_ARM_BUTTON **************************************/
    {

      int switch_num                      = 4;
      static float switch_pos[]           = {CONNECT_COORDNIATES(15)};
      static char *switch_texts[]         = 
	{"(ARM)", "ARM", "?ARM?",
	   "?TCX?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY50, C_GREY70, C_GREY70, C_GREY70};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_WHITE, C_BLUE, C_RED, C_RED};
      
      CONNECT_ARM_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    

    /******** CONNECT_FLOW_BUTTON **************************************/
    {

      int switch_num                      = 4;
      static float switch_pos[]           = {CONNECT_COORDNIATES(12)};
      static char *switch_texts[]         = 
	{"(FLOW)", "FLOW", "?FLOW?",
	   "?TCX?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY50, C_GREY70, C_GREY70, C_GREY70};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_WHITE, C_BLUE, C_RED, C_RED};
      
      CONNECT_FLOW_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    


    /******** CONNECT_SUNVIS_BUTTON **************************************/
    {

      int switch_num                      = 4;
      static float switch_pos[]           = {CONNECT_COORDNIATES(13)};
      static char *switch_texts[]         = 
	{"(SUNVIS)", "SUNVIS", "?SUNVIS?",
	   "?TCX?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY50, C_GREY70, C_GREY70, C_GREY70};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_WHITE, C_BLUE, C_RED, C_RED};
      
      CONNECT_SUNVIS_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    
  
    /******** CONNECT_TRACKER_BUTTON **************************************/
    {

      int switch_num                      = 4;
      static float switch_pos[]           = {CONNECT_COORDNIATES(14)};
      static char *switch_texts[]         = 
	{"(TRACKER)", "TRACKER", "?TRACKER?",
	   "?TCX?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY50, C_GREY70, C_GREY70, C_GREY70};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_WHITE, C_BLUE, C_RED, C_RED};
      
      CONNECT_TRACKER_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    


#endif /* UNIBONN */
    
#ifdef CD_VERSION
    /******** CONNECT_CD_BUTTON **************************************/
    {

      int switch_num                      = 4;
      static float switch_pos[]           = {CONNECT_COORDNIATES(11)};
      static char *switch_texts[]         = 
	{"(CD)", "CD", "?CD?",
	   "?TCX?"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY50, C_GREY70, C_GREY70, C_GREY70};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_WHITE, C_BLUE, C_RED, C_RED};
      
      CONNECT_CD_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    
#endif /* CD_VERSION */
 
    /******** QUIT_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(1), BAR_ROW(0)};
      static char *switch_texts[]         = {"quit program", "quit program"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90,C_RED};
      static int switch_frame_color[]     = {C_GREY40,C_GREY40};
      static int switch_text_color[]      = {C_BLACK,C_WHITE};

      QUIT_BUTTON = G_create_switch_object(switch_pos, switch_num, 
					     switch_texts,
					     switch_background_color,
					     switch_frame_color, 
					     switch_text_color,
					     switch_fonts);
    }


    /******** STOP_ROBOT **************************************/
    {
      int switch_num                      = 1;
      static float switch_pos[]           = {BAR_COLUMN(0), BAR_ROW_3(0)};
      static char *switch_texts[]         = {"stop robot"};
      static int switch_fonts[]           = {6};
      static int switch_background_color[]= {C_RED};
      static int switch_frame_color[]     = {C_GREY40};
      static int switch_text_color[]      = {C_WHITE};

      STOP_ROBOT = G_create_switch_object(switch_pos, switch_num, 
					     switch_texts,
					     switch_background_color,
					     switch_frame_color, 
					     switch_text_color,
					     switch_fonts);
    }

    /******** REFRESH_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(1), BAR_ROW(1)};
      static char *switch_texts[]         = {"refresh", "refresh"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90,C_RED};
      static int switch_frame_color[]     = {C_GREY40,C_GREY40};
      static int switch_text_color[]      = {C_BLACK,C_WHITE};

      REFRESH_BUTTON = G_create_switch_object(switch_pos, switch_num, 
					     switch_texts,
					     switch_background_color,
					     switch_frame_color, 
					     switch_text_color,
					     switch_fonts);
    }

    /******** MAP_UPDATE_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(3), BAR_ROW(0)};
      static char *switch_texts[]         = {"map display (OFF)", "map display (ON)"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90,C_YELLOW};
      static int switch_frame_color[]     = {C_GREY40,C_GREY40};
      static int switch_text_color[]      = {C_BLACK,C_BLACK};

      MAP_UPDATE_BUTTON = G_create_switch_object(switch_pos, switch_num, 
					     switch_texts,
					     switch_background_color,
					     switch_frame_color, 
					     switch_text_color,
					     switch_fonts);
    }

    /******** DUMP_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(1), BAR_ROW(2)};
      static char *switch_texts[]         = 
	{"plot map", "...plotting"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      DUMP_BUTTON = G_create_switch_object(switch_pos, 
					   switch_num, 
					   switch_texts,
					   switch_background_color,
					   switch_frame_color, 
					   switch_text_color,
					   switch_fonts);
    }    

    
    /******** RESET_ROBOT_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(2), BAR_ROW(2)};
      static char *switch_texts[]         = 
	{"reset joystick", "..resetting"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      RESET_ROBOT_BUTTON = G_create_switch_object(switch_pos, 
						  switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    

    /******** BASE_MODE_BUTTON **************************************/
    {

      int switch_num                      = NUMBER_OF_MODES;
      static float switch_pos[]           = {BAR_COLUMN(2), BAR_ROW(1)};
      static char *switch_texts[]         = 
	{"colli: default", "colli: fast", "colli: random", 
	 "colli: servo", "colli: arm out", "colli: find door", 
	 "colli: appr-object", "colli: appr-bin", "colli: arm-random"};
      static int switch_fonts[]           = {2,2,2,2,2,2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY90, C_GREY90, C_GREY90, C_GREY90, C_GREY90, C_GREY90, C_GREY90,
	 C_GREY90, C_GREY90};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40, C_GREY40, C_GREY40, C_GREY40,
	 C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
	 C_BLACK, C_BLACK};

      if (NUMBER_OF_MODES != 9){
	fprintf(stderr, "ERROR: Adjust BASE_MODE_BUTTON in graphics.c()\n");
	exit (-1);
      }

      BASE_MODE_BUTTON = G_create_switch_object(switch_pos, switch_num, 
					      switch_texts,
					      switch_background_color,
					      switch_frame_color, 
					      switch_text_color,
					      switch_fonts);
    }    

     /******** ACQUIRE_CAMERA_IMAGE_BUTTON **************************************/
    {


      int switch_num                      = 3;
      static float switch_pos[]           = {BAR_COLUMN(3), BAR_ROW(1)};
      static char *switch_texts[]         = 
	{"single image", "single image", "single image"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_GREY90, C_YELLOW,C_YELLOW};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40,C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK,C_BLACK};
      
      ACQUIRE_CAMERA_IMAGE_BUTTON = 
	G_create_switch_object(switch_pos, switch_num, 
			       switch_texts,
			       switch_background_color,
			       switch_frame_color, 
			       switch_text_color,
			       switch_fonts);
    }    
        
   /******** CONTINUOUS_CAMERA_IMAGE_BUTTON **************************************/
    {


      int switch_num                      = 3;
      static float switch_pos[]           = {BAR_COLUMN(3), BAR_ROW(2)};
      static char *switch_texts[]         = 
	{"continuous images", "continuous images", "continuous images"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_GREY90, C_YELLOW,C_YELLOW};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40,C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK,C_BLACK};

      
      CONTINUOUS_CAMERA_IMAGE_BUTTON =
	G_create_switch_object(switch_pos, switch_num, 
			       switch_texts,
			       switch_background_color,
			       switch_frame_color, 
			       switch_text_color,
			       switch_fonts);
    }    
        
#ifdef UNIBONN
     /******** LEFT_CAMERA_BUTTON **************************************/
    {


      int switch_num                      = 3;
      static float switch_pos[]           = {BAR_COLUMN(3), BAR_ROW(1)};
      static char *switch_texts[]         = 
	{"left camera", "left camera", "left camera"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_GREY90, C_RED,C_YELLOW};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40,C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_WHITE,C_BLACK};
      
      LEFT_CAMERA_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    
        
   /******** RIGHT_CAMERA_BUTTON **************************************/
    {


      int switch_num                      = 3;
      static float switch_pos[]           = {BAR_COLUMN(3), BAR_ROW(2)};
      static char *switch_texts[]         = 
	{"right camera", "right camera", "right camera"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_GREY90, C_RED,C_YELLOW};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40,C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_WHITE,C_BLACK};

      
      RIGHT_CAMERA_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						   switch_texts,
						   switch_background_color,
						   switch_frame_color, 
						   switch_text_color,
						   switch_fonts);
    }    
        

    /******** SONAR_BUTTON **************************************/
    {


      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(1), BAR_ROW(3)};
      static char *switch_texts[]         = 
	{"sonar (off)", "sonar (on)"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_LAWNGREEN};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      SONAR_BUTTON = G_create_switch_object(switch_pos, switch_num, 
					      switch_texts,
					      switch_background_color,
					      switch_frame_color, 
					      switch_text_color,
					      switch_fonts);
    }    

    /******** EXPL_RESET_BUTTON **************************************/
    {


      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(1), BAR_ROW(4)};
      static char *switch_texts[]         = 
	{"expl-autoreset (off)", "expl-autoreset (on)"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_YELLOW};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      EXPL_RESET_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						 switch_texts,
						 switch_background_color,
						 switch_frame_color, 
						 switch_text_color,
						 switch_fonts);
    }    

    /******** ARM_BACKGROUND **************************************/
    {
      int switch_num                      = 1;
      static float switch_pos[]           = 
	{12.9, 12.9+(2.0*(SLIDER_WIDTH))+(3.0*(SLIDER_SEP_SPACE)), 
	   (BOTTOM_BUTTON_BORDER) + 0.3, 9.9};
      static char *switch_texts[]         = {""};
      static int switch_fonts[]           = {2};
      static int switch_background_color[]= {C_GREY90};
      static int switch_frame_color[]     = {C_GREY40};
      static int switch_text_color[]      = {NO_COLOR};
      
      G_create_switch_object(switch_pos, switch_num, 
			     switch_texts,
			     switch_background_color,
			     switch_frame_color, 
			     switch_text_color,
			     switch_fonts);
      
    }

    /******** ARM_MOVE_OUT_IN_BUTTON **************************************/
    {


      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(3), BAR_ROW(2)};
      static char *switch_texts[]         = 
	{"move arm out", "move arm in"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_YELLOW};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      ARM_MOVE_OUT_IN_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						      switch_texts,
						      switch_background_color,
						      switch_frame_color, 
						      switch_text_color,
						      switch_fonts);
    }    

    /******** ARM_PICK_BUTTON **************************************/
    {


      int switch_num                      = 3;
      static float switch_pos[]           = {BAR_COLUMN(3), BAR_ROW(3)};
      static char *switch_texts[]         = 
	{"pickup", "lift object", "drop object"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_GREY90, C_YELLOW, C_YELLOW};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK, C_BLACK};
      
      ARM_PICK_BUTTON = G_create_switch_object(switch_pos, switch_num, 
  						      switch_texts,
						      switch_background_color,
						      switch_frame_color, 
						      switch_text_color,
						      switch_fonts);
    }    
    
    /******** ARM_CLOSE_OPEN_GRIPPER_BUTTON *******************************/
    {


      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(3), BAR_ROW(4)};
      static char *switch_texts[]         = 
	{"close gripper (OPEN)", "open gripper (CLOSED)"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_GREY90};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      ARM_CLOSE_OPEN_GRIPPER_BUTTON =
	G_create_switch_object(switch_pos, switch_num, 
			       switch_texts,
			       switch_background_color,
			       switch_frame_color, 
			       switch_text_color,
			       switch_fonts);
    }    


    /******** ARM_MAST_POSITION_BUTTON *****************************/
    {
      static float value_pos[]            = 
	{12.9 + (2.0 * (SLIDER_SEP_SPACE)) + SLIDER_WIDTH, 
	   12.9 + (2.0 * (SLIDER_SEP_SPACE)) + (2.0 * (SLIDER_WIDTH)), 
	   ((BOTTOM_BUTTON_BORDER) + 0.62 + 9.8) * 0.5 + 0.05,
	   9.8};
      static char *value_text             = "MA";
      static int value_font               = 2;
      int direction_value                 = 2;
      static float value                  = 1.0;
      float min_value                     = 0.0;
      float max_value                     = 1.0;
      static int value_colors[]           = {C_GREY70, C_MEDIUMVIOLETRED,
					       C_GREY40, C_WHITE}; 
      
      
      ARM_MAST_POSITION_BUTTON = 
	G_create_value_object(value_pos, value_text, 
			      direction_value, value, 
					  min_value,
					  max_value, value_colors, 
					  value_font);
    }
    

    /******** ARM_IN_OUT_BUTTON *****************************/
    {
      static float value_pos[]            = 
	{12.9 + SLIDER_SEP_SPACE, 12.9+ SLIDER_WIDTH + SLIDER_SEP_SPACE, 
	   ((BOTTOM_BUTTON_BORDER) + 0.62 + 9.8) * 0.5 + 0.05,
	   9.8};
      static char *value_text             = "ARM";
      static int value_font               = 2;
      int direction_value                 = 2;
      static float value                  = 1.0;
      float min_value                     = 0.0;
      float max_value                     = 1.0;
      static int value_colors[]           = {C_GREY70, C_MEDIUMVIOLETRED,
					       C_GREY40, C_WHITE}; 
      
      
      ARM_IN_OUT_BUTTON =
	G_create_value_object(value_pos, value_text, 
			      direction_value, value, 
			      min_value,
			      max_value, value_colors, 
			      value_font);

    }


    /******** ARM_GRIPPER_PAD_POSITION_BUTTON *****************************/
    {
      static float value_pos[]            = 
	{12.9 + (2.0 * (SLIDER_SEP_SPACE)) + SLIDER_WIDTH, 
	   12.9 + (2.0 * (SLIDER_SEP_SPACE)) + (2.0 * (SLIDER_WIDTH)), 
	   (BOTTOM_BUTTON_BORDER) + 0.62,
	   ((BOTTOM_BUTTON_BORDER) + 0.62 + 9.8) * 0.5 - 0.05};
      static char *value_text             = "GR";
      static int value_font               = 2;
      int direction_value                 = 2;
      static float value                  = 1.0;
      float min_value                     = 0.0;
      float max_value                     = 1.0;
      static int value_colors[]           = {C_GREY70, C_MEDIUMVIOLETRED,
					       C_GREY40, C_WHITE}; 
      
      
      ARM_GRIPPER_PAD_POSITION_BUTTON =
	G_create_value_object(value_pos, value_text, 
			      direction_value, value, 
					  min_value,
					  max_value, value_colors, 
					  value_font);
    }

    
    /******** ARM_GRIPPER_ORIENTATION_BUTTON *****************************/
    {
      static float value_pos[]            = 
	{12.9 + SLIDER_SEP_SPACE, 12.9+ SLIDER_WIDTH + SLIDER_SEP_SPACE, 
	   (BOTTOM_BUTTON_BORDER) + 0.62,
	   ((BOTTOM_BUTTON_BORDER) + 0.62 + 9.8) * 0.5 - 0.05};
      static char *value_text             = "WR";
      static int value_font               = 2;
      int direction_value                 = 2;
      static float value                  = 1.0;
      float min_value                     = 0.0;
      float max_value                     = 1.0;
      static int value_colors[]           = {C_GREY70, C_MEDIUMVIOLETRED,
					       C_GREY40, C_WHITE}; 
      
      
      ARM_GRIPPER_ORIENTATION_BUTTON =
	G_create_value_object(value_pos, value_text, 
			      direction_value, value, 
			      min_value,
			      max_value, value_colors, 
			      value_font);

    }



    /******** ARM_ROBOT_SUBTITLE **************************************/
    {
      int switch_num                      = 1;
      static float switch_pos[]           = 
	{12.9, 12.9+(2.0*(SLIDER_WIDTH))+(3.0*(SLIDER_SEP_SPACE)),
	   (BOTTOM_BUTTON_BORDER),
	   (BOTTOM_BUTTON_BORDER) + 0.52};
      static char *switch_texts[]         = 
	{"ARM"};
      static int switch_fonts[]           = {2};
      static int switch_background_color[]= {C_GREY40};
      static int switch_frame_color[]     = {C_GREY70};
      static int switch_text_color[]      = {C_WHITE};
      
	G_create_switch_object(switch_pos, switch_num, switch_texts,
			       switch_background_color,
			       switch_frame_color, 
			       switch_text_color, switch_fonts);
    }
    
   /******** CAMERA_X_DISPLAY_BUTTON **************************************/
    {


      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(3), BAR_ROW(2)};
      static char *switch_texts[]         = 
	{"X/display  (off)", "X/display (on)"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_LAWNGREEN};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      CAMERA_X_DISPLAY_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						   switch_texts,
						   switch_background_color,
						   switch_frame_color, 
						   switch_text_color,
						   switch_fonts);
    }    
        
   /******** CAMERA_OBJECTS_BUTTON **************************************/
    {


      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(3), BAR_ROW(3)};
      static char *switch_texts[]         = 
	{"Find objects (off)", "Find objects(on)"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_GREY90, C_LAWNGREEN};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      CAMERA_OBJECTS_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						   switch_fonts);
    }    
        
 
   /******** CAMERA_MAPS_BUTTON **************************************/
    {


      int switch_num                      = 3;
      static float switch_pos[]           = {BAR_COLUMN(3), BAR_ROW(4)};
      static char *switch_texts[]         = 
	{"Camera maps (off)", "Camera maps (on)", "Camera maps (??)"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_GREY90, C_LAWNGREEN, C_GREY90};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK, C_BLACK};
      
      CAMERA_MAPS_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						   switch_fonts);
    }    
        
 
   /******** CAMERA_COLLI_LINES_BUTTON ************************************/
    {


      int switch_num                      = 3;
      static float switch_pos[]           = {BAR_COLUMN(3), BAR_ROW(5)};
      static char *switch_texts[]         = 
	{"CamColLines (off)", "CamColLines (on)", "CamColLines (??)"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_GREY90, C_LAWNGREEN, C_GREY90};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK, C_BLACK};
      
      CAMERA_COLLI_LINES_BUTTON
	= G_create_switch_object(switch_pos, switch_num, 
				 switch_texts,
				 switch_background_color,
				 switch_frame_color, 
				 switch_text_color,
				 switch_fonts);
    }    

    /******** TRACKER_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(2), BAR_ROW(2)};
      static char *switch_texts[]         = 
	{"tracker (is off)", "...tracking"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_LAWNGREEN};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      TRACKER_BUTTON = G_create_switch_object(switch_pos, 
						  switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    

    
#endif /* UNIBONN */
    
#ifdef use_the_unused_old_stuff 
    /******** LOGGING_BUTTON **************************************/
    {


      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(1), BAR_ROW(5)};
      static char *switch_texts[]         = 
	{"logging (off)", "logging (on)"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_LAWNGREEN};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      LOGGING_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						 switch_texts,
						 switch_background_color,
						 switch_frame_color, 
						 switch_text_color,
						 switch_fonts);
    }    
    

    /******** SCRIPT_BUTTON **************************************/
    {
      int switch_num                      = 3;
      static float switch_pos[]           = {BAR_COLUMN(2), BAR_ROW(2)};
      static char *switch_texts[]         = 
	{"script (off)", "...reading script", "script (on)"};
      static int switch_fonts[]           = {2, 2, 2};
      static int switch_background_color[]= {C_GREY90, C_RED, C_LAWNGREEN};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK, C_BLACK};
      
      SCRIPT_BUTTON = G_create_switch_object(switch_pos, switch_num, 
					     switch_texts,
					     switch_background_color,
					     switch_frame_color, 
					     switch_text_color,
					     switch_fonts);
    }

    

    /******** EPISODE_DIAL *******************************************/
    {
      static float value_pos[]            =
	{0.5, 12.3, 0.05+BAR_ROW(NUM_ROWS-2)-0.05};
      static char *value_text             = "episodes";
      static int value_font               = 2;
      int direction_value                 = 1;
      static float value                  = 1.0;
      float min_value                     = 0.0;
      float max_value                     = 1.0;
      static int value_colors[]           = {C_GREY90, C_YELLOW2, C_GREY40,
					       C_BLACK}; 
      EPISODE_DIAL = G_create_value_object(value_pos, value_text, 
					   direction_value, value, 
					   min_value,
					   max_value, value_colors, 
					   value_font);
    }    
    
    /******** EVENT_DIAL *******************************************/
    {
      static float value_pos[]            = 
	{0.5, 12.3, 0.05+BAR_ROW(NUM_ROWS-1)-0.05};
      static char *value_text             = "events";
      static int value_font               = 2;
      int direction_value                 = 1;
      static float value                  = 1.0;
      float min_value                     = 0.0;
      float max_value                     = 1.0;
      static int value_colors[]           = {C_GREY90, C_YELLOW2, C_GREY40,
					       C_BLACK}; 
      EVENT_DIAL = G_create_value_object(value_pos, value_text, 
					 direction_value, value, 
					 min_value,
					 max_value, value_colors, 
					 value_font);
    }    


    /******** DECREASE_EPISODE_BUTTON **************************************/
    {
      int switch_4_num                      = 2;
      static float switch_4_pos[]           = {0.0, 0.4, BAR_ROW(NUM_ROWS-2)};
      static char *switch_4_texts[]         = {"<<", "<<"};
      static int switch_4_fonts[]           = {2,2};
      static int switch_4_background_color[]= {C_GREY90, C_RED};
      static int switch_4_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_4_text_color[]      = {C_BLACK, C_BLACK};
      DECREASE_EPISODE_BUTTON
	= G_create_switch_object(switch_4_pos, switch_4_num, 
				 switch_4_texts,
				 switch_4_background_color,
				 switch_4_frame_color, 
				 switch_4_text_color,
				 switch_4_fonts);
    }    
    
    
    /******** INCREASE_EPISODE_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {12.4, 12.8, BAR_ROW(NUM_ROWS-2)};
      static char *switch_texts[]         = {">>", ">>"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      INCREASE_EPISODE_BUTTON
	= G_create_switch_object(switch_pos, switch_num, 
				 switch_texts,
				 switch_background_color,
				 switch_frame_color, 
				 switch_text_color,
				 switch_fonts);
    }
    
    /******** DECREASE_EVENT_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {0.0, 0.4, BAR_ROW(NUM_ROWS-1)};
      static char *switch_texts[]         = {"<<", "<<"};
      static int switch_fonts[]           = {3,3};
      static int switch_background_color[]= {C_GREY90, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
      DECREASE_EVENT_BUTTON
	= G_create_switch_object(switch_pos, switch_num, 
				 switch_texts,
				 switch_background_color,
				 switch_frame_color, 
				 switch_text_color,
				 switch_fonts);
    }
    /******** INCREASE_EVENT_BUTTON **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {12.4, 12.8, BAR_ROW(NUM_ROWS-1)};
      static char *switch_texts[]         = {">>", ">>"};
      static int switch_fonts[]           = {3,3};
      static int switch_background_color[]= {C_GREY90, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      INCREASE_EVENT_BUTTON
	= G_create_switch_object(switch_pos, switch_num, 
				 switch_texts,
				 switch_background_color,
				 switch_frame_color, 
				 switch_text_color,
				 switch_fonts);
      
    }

#endif    
    /******** PAN_TILT_BACKGROUND **************************************/
    {
      int switch_num                      = 1;
      static float switch_pos[]           = {4.9, 8.8, 6.0, 9.9};
      static char *switch_texts[]         = {""};
      static int switch_fonts[]           = {2};
      static int switch_background_color[]= {C_GREY90};
      static int switch_frame_color[]     = {C_GREY40};
      static int switch_text_color[]      = {NO_COLOR};
      
      PAN_TILT_BACKGROUND = G_create_switch_object(switch_pos, switch_num, 
						   switch_texts,
						   switch_background_color,
						   switch_frame_color, 
						   switch_text_color,
						   switch_fonts);
      
    }

    /******** CAMERA_TILT, SCRIPT_CAMERA_TILT *****************************/
    {
      static float value_pos[]            = {5.05, 5.2, 6.4, 9.8};
      static char *value_text             = "tilt";
      static int value_font               = 3;
      int direction_value                 = 2;
      static float value                  = 1.0;
      float min_value                     = MIN_TILT_ANGLE;
      float max_value                     = MAX_TILT_ANGLE;
      static int value_colors[]           = {C_GREY70, C_BLUE, C_GREY40,
					       NO_COLOR}; 
      
      
      CAMERA_TILT = G_create_value_object(value_pos, value_text, 
					  direction_value, value, 
					  min_value,
					  max_value, value_colors, 
					  value_font);
      
      SCRIPT_CAMERA_TILT = G_create_value_object(value_pos, value_text, 
						 direction_value, value, 
						 min_value,
						 max_value, value_colors, 
						 value_font);
    }
    
    /******** CAMERA_PAN, SCRIPT_CAMERA_PAN *****************************/
    {
      static float value_pos[]            = {5.3, 8.7, 6.15, 6.3};
      static char *value_text             = "tilt";
      static int value_font               = 3;
      int direction_value                 = 1;
      static float value                  = 1.0;
      float min_value                     = MIN_PAN_ANGLE;
      float max_value                     = MAX_PAN_ANGLE;
      static int value_colors[]           = {C_GREY70, C_BLUE, C_GREY40,
					       NO_COLOR}; 
      
      CAMERA_PAN = G_create_value_object(value_pos, value_text, 
					 direction_value, value, 
					 min_value,
					 max_value, value_colors, 
					 value_font);
      
      SCRIPT_CAMERA_PAN = G_create_value_object(value_pos, value_text, 
						direction_value, value, 
						min_value,
						max_value, value_colors, 
						value_font);
    }

    
#ifdef UNIBONN
    /******** MARKERS  **************************************/
    {
      static float pos_r[]                   = {5.3, 8.7, 6.4, 9.8};
      int num_mar                            = 6;
      static char *text_mar[]                =
	{"BR", "GR", "RB", "GB", "BG", "RG"};
      int connected_mar                      = 0;
      static int mar_frame_color             = C_GREY70;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = 
	{C_BLUE, C_LIMEGREEN, C_RED, C_LIMEGREEN, C_BLUE, C_RED};
      static int mar_text_color[]            = 
	{C_YELLOW, C_YELLOW, C_YELLOW, C_YELLOW, C_YELLOW, C_YELLOW};
      static int mar_fonts[]                 = {1, 1, 1, 1, 1, 1};
      
      
      MARKERS =
	G_create_markers_object(pos_r, connected_mar, 
				robot_specifications->robot_size,
				0, (float) X_SIZE, 0, (float) Y_SIZE,
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, mar_foreground_color, 
				mar_text_color, mar_fonts);
      
    }
#endif /* UNIBONN */

    /******** SCRIPT_EVENT_DATE_BUTTON **************************************/
    {
      int switch_num                      = 1;
      static float switch_pos[]           = {0.0, 3.9, 6.2, 6.4};
      static char *switch_texts[]         = {""};
      static int switch_fonts[]           = {2};
      static int switch_background_color[]= {NO_COLOR};
      static int switch_frame_color[]     = {NO_COLOR};
      static int switch_text_color[]      = {C_WHITE};
      
      SCRIPT_EVENT_DATE_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,
				 switch_frame_color, 
				 switch_text_color, switch_fonts);
      
    }

      
    /******** SONAR_ROBOT_SUBTITLE **************************************/
    {
      int switch_num                      = 2;
      static float switch_pos[]           = {0.0, 3.9, 5.7, 6.0};
      static char *switch_texts[]         = 
	{"mode: target point", "mode: joystick emulation"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY40,C_GREY40};
      static int switch_frame_color[]     = {C_GREY70,C_GREY70};
      static int switch_text_color[]      = {C_WHITE,C_WHITE};
      
      SONAR_ROBOT_SUBTITLE =
	G_create_switch_object(switch_pos, switch_num, switch_texts,
			       switch_background_color,
			       switch_frame_color, 
			       switch_text_color, switch_fonts);
    }

     
    /******** SONAR_ROBOT_SUBTITLE2 **************************************/
    {
      int switch_num                      = 1;
      static float switch_pos[]           = {3.9, 4.8, 5.7, 6.22};
      static char *switch_texts[]         = 
	{"VEL ACC"};
      static int switch_fonts[]           = {2};
      static int switch_background_color[]= {C_GREY40};
      static int switch_frame_color[]     = {C_GREY70};
      static int switch_text_color[]      = {C_WHITE};
      
      SONAR_ROBOT_SUBTITLE2 =
	G_create_switch_object(switch_pos, switch_num, switch_texts,
			       switch_background_color,
			       switch_frame_color, 
			       switch_text_color, switch_fonts);
    }

     
    /******** SONAR_ROBOT_SUBTITLE3 **************************************/
    {
      int switch_num                      = 1;
      static float switch_pos[]           = {-0.5,0.0, 5.7, 6.22};
      static char *switch_texts[]         = 
	{"VEL"};
      static int switch_fonts[]           = {2};
      static int switch_background_color[]= {C_GREY40};
      static int switch_frame_color[]     = {C_GREY70};
      static int switch_text_color[]      = {C_WHITE};
      
      SONAR_ROBOT_SUBTITLE3 =
	G_create_switch_object(switch_pos, switch_num, switch_texts,
			       switch_background_color,
			       switch_frame_color, 
			       switch_text_color, switch_fonts);
    }

     
    /******** PANTILT_SUBTITLE **************************************/
    {
      int switch_num                      = 3;
      static float switch_pos[]           = {4.9, 8.8, 5.7, 6.0};
      static char *switch_texts[]         = 
	{"pan/tilt SLOW", "pan/tilt NORMAL", "pan/tilt FAST"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_GREY40,C_GREY40,C_GREY40};
      static int switch_frame_color[]     = {C_GREY70,C_GREY70,C_GREY70};
      static int switch_text_color[]      = {C_WHITE,C_WHITE,C_WHITE};
      
      PANTILT_SUBTITLE =
	G_create_switch_object(switch_pos, switch_num, switch_texts,
			       switch_background_color,
			       switch_frame_color, 
			       switch_text_color, switch_fonts);
    }



   /******** AUTONOMOUS_BUTTON **************************************/
    {


      int switch_num                      = 2;
      static float switch_pos[]           = {BAR_COLUMN(2), BAR_ROW(0)};
      static char *switch_texts[]         = 
	{"autonomous", "...autonomous"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_WHITE};
      
      AUTONOMOUS_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    
    /******** BATTERY_STATUS *******************************************/
    {
      int switch_num                      = 1;
      static float switch_pos[]           = {-1.0, -0.6, 5.7, 9.9};
      static char *switch_texts[]         = {""};
      static int switch_fonts[]           = {2};
      static int switch_background_color[]= {C_GREY90};
      static int switch_frame_color[]     = {C_GREY70};
      static int switch_text_color[]      = {NO_COLOR};
      
      G_create_switch_object(switch_pos, switch_num, switch_texts,
			     switch_background_color,
			     switch_frame_color, 
			     switch_text_color, switch_fonts);
    }
    {      
      int switch_num                      = 1;
      static float switch_pos[]           = {-1.0, -0.6, 5.7, 6.22};
      static char *switch_texts[]         = {"BAT"};
      static int switch_fonts[]           = {2};
      static int switch_background_color[]= {C_GREY40};
      static int switch_frame_color[]     = {C_GREY70};
      static int switch_text_color[]      = {C_WHITE};
      
      G_create_switch_object(switch_pos, switch_num, switch_texts,
			     switch_background_color,
			     switch_frame_color, 
			     switch_text_color, switch_fonts);
      
    }
    {
      static float value_pos[]            =
	{-0.9, -0.7, 6.3, 9.8};
      static char *value_text             = "";
      static int value_font               = 1;
      int direction_value                 = 2;
      static float value                  = 0.3;
      float min_value                     = 0.0;
      float max_value                     = 1.0;
      static int value_colors[]           = {C_GREY70, C_TURQUOISE4, C_GREY40,
					       NO_COLOR}; 
      BATTERY_STATUS = G_create_value_object(value_pos, value_text, 
					     direction_value, value, 
					     min_value,
					     max_value, value_colors, 
					     value_font);
    }    
  }    
        
    
#ifdef UNIBONN
   /******** HUNTING_BUTTON **************************************/
    {


      int switch_num                      = 7;
      static float switch_pos[]           = {BAR_COLUMN(2), BAR_ROW(1)};
      static char *switch_texts[]         = 
	{"hunting (is off)", "..searching object",
	   "..approaching object", "..lifting object",
	   "..searching bin", "..approaching bin", "..dumping objects"};
      static int switch_fonts[]           = {2,2,2,2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY90, C_RED, C_RED, C_RED, C_RED, C_RED, C_RED};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_BLACK, C_WHITE, C_WHITE, C_WHITE, C_WHITE, C_WHITE, C_WHITE};
      
      HUNTING_BUTTON = G_create_switch_object(switch_pos, switch_num, 
					      switch_texts,
					      switch_background_color,
					      switch_frame_color, 
					      switch_text_color,
					      switch_fonts);
    }    
        
       
    
    /******** NUM_OBJECTS_BUTTON **************************************/

    {


      int switch_num                      = 5;
      static float switch_pos[]           = {BAR_COLUMN(2), BAR_ROW(2)};
      static char *switch_texts[]         = {
	"carrying: 0",
	"carrying: 1",
	"carrying: 2",
	"carrying: 3",
	"carrying: 4"};
      static int switch_fonts[]           = {2,2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY90, C_GREY90, C_GREY90, C_GREY90, C_GREY90};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK};
      
      NUM_OBJECTS_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						  switch_texts,
						  switch_background_color,
						  switch_frame_color, 
						  switch_text_color,
						  switch_fonts);
    }    

        
#endif /* UNIBONN */
    
    /******** STATUS_BACKGROUND **************************************/
    {
      int switch_num                      = 1;
      static float switch_pos[]           = 
	{RIGHT_BUTTON_BORDER+BUTTON_SEPARATOR,12.8,
	   BOTTOM_BUTTON_BORDER,TOP_BUTTON_BORDER};
      static char *switch_texts[]         = {""};
      static int switch_fonts[]           = {2};
      static int switch_background_color[]= {C_GREY40};
      static int switch_frame_color[]     = {C_GREY70};
      static int switch_text_color[]      = {NO_COLOR};
      
      STATUS_BACKGROUND = G_create_switch_object(switch_pos, switch_num, 
						      switch_texts,
						      switch_background_color,
						      switch_frame_color, 
						      switch_text_color,
						      switch_fonts);
      
    }

    /******** STATUS_TOTAL_AREA_BUTTON *****************************/
    {
      static float value_pos[]            = {RIGHT_SLIDER(0)};
      static char *value_text             = "total map size";
      static int value_font               = 2;
      int direction_value                 = 1;
      static float value                  = 0.0;
      float min_value                     = 0.0;
      float max_value                     = 
	robot_specifications->global_worldsize_x
	  * robot_specifications->global_worldsize_y;
      static int value_colors[]           = {C_GREY90, C_PINK, C_GREY40,
					       C_BLACK}; 
       STATUS_TOTAL_AREA_BUTTON =
	 G_create_value_object(value_pos, value_text, 
			       direction_value, value, 
			       min_value,
			       max_value, value_colors, 
			       value_font);
    }
   
    /******** STATUS_NEW_AREA_BUTTON *****************************/
    {
      static float value_pos[]            = {RIGHT_SLIDER(1)};
      static char *value_text             = "relative map size";
      static int value_font               = 2;
      int direction_value                 = 1;
      static float value                  = 0.0;
      float min_value                     = 0.0;
      float max_value                     = 30000.0;
      static int value_colors[]           = {C_GREY90, C_PINK, C_GREY40,
					       C_BLACK}; 
       STATUS_NEW_AREA_BUTTON =
	 G_create_value_object(value_pos, value_text, 
			       direction_value, value, 
			       min_value,
			       max_value, value_colors, 
			       value_font);
    }
   
    /******** STATUS_ADVANCEMENT_BUTTON *****************************/
    {
      static float value_pos[]            = {RIGHT_SLIDER(2)};
      static char *value_text             = "relative advancement";
      static int value_font               = 2;
      int direction_value                 = 1;
      static float value                  = 0.0;
      float min_value                     = 0.0;
      float max_value                     = 70.0;
      static int value_colors[]           = {C_GREY90, C_PINK, C_GREY40,
					       C_BLACK}; 
      STATUS_ADVANCEMENT_BUTTON = 
	 G_create_value_object(value_pos, value_text, 
			       direction_value, value, 
			       min_value,
			       max_value, value_colors, 
			       value_font);
    }
   
    /******** PSEUDO_PICTURE *****************************/
    {
      static float matrix_pos[]             = {5.3, 8.7, 6.4, 9.8};
      char *matrix_text                     = "picture";
      static int matrix_font                = 2;
      float min_value_matrix                = 0.0;
      float max_value_matrix                = 1.0;
      static int matrix_colors[]            = 
	{C_GREY70, C_GREY40, NO_COLOR}; 

      int i;
    
      for (i = 0; i < PSEUDO_PICTURE_SIZE_HORIZONTAL 
	   * PSEUDO_PICTURE_SIZE_VERTICAL; i++){
	pseudo_picture_active[i] = 1;
	pseudo_picture[i] = ((float) i)
	  / ((float) (PSEUDO_PICTURE_SIZE_HORIZONTAL
		      * PSEUDO_PICTURE_SIZE_VERTICAL));
      }
      
    PSEUDO_PICTURE
      = G_create_matrix_object(matrix_pos, matrix_text, 
			       pseudo_picture, pseudo_picture_active,
			       PSEUDO_PICTURE_SIZE_HORIZONTAL, 
			       PSEUDO_PICTURE_SIZE_VERTICAL, 
			       min_value_matrix, max_value_matrix,
			       matrix_colors, matrix_font);
    }

  
  /* =============================================================
     ====================  4) INIT STATE =========================
     ============================================================= */
  
  select_active_objects(ALL);
  
  
  /* =============================================================
     ====================  5) DISPLAY ===========================
     ============================================================= */
  

  G_deactivate(BUSY_BUTTON);
  G_deactivate(IN_MOTION_BUTTON);
  
#ifdef UNIBONN
  /*
   * I cleaned up the following buttons, which were generated for
   * particular applications
   */
  G_deactivate(CAMERA_X_DISPLAY_BUTTON);
  G_deactivate(CAMERA_OBJECTS_BUTTON);
  G_deactivate(CAMERA_MAPS_BUTTON);
  G_deactivate(CAMERA_COLLI_LINES_BUTTON);
  G_deactivate(SONAR_BUTTON);
  G_deactivate(EXPL_RESET_BUTTON);
  G_deactivate(NUM_OBJECTS_BUTTON);
  G_deactivate(HUNTING_BUTTON);
#endif /* UNIBONN */


  worldsize = robot_specifications->global_worldsize_x;
  if (robot_specifications->global_worldsize_y < worldsize)
    worldsize = robot_specifications->global_worldsize_y;

  autoshift_display(robot_state->x, 
		    robot_state->y, 
		    0.5 * worldsize,
		    0.5 * worldsize,
		    ALL);

#if 0
#ifndef UNIBONN
    G_deactivate(STATUS_TOTAL_AREA_BUTTON);
    G_deactivate(STATUS_NEW_AREA_BUTTON);
    G_deactivate(STATUS_ADVANCEMENT_BUTTON);
#endif /* UNIBONN */
#endif  
  
  /* G_deactivate(LOGGING_BUTTON);	currently not used */
  /* G_deactivate(SCRIPT_BUTTON);	currently not used */


  if (!program_state->graphics_initialized)
    G_display_all();

  G_display_switch(IN_MOTION_BUTTON, robot_state->in_motion);
  
  /* G_display_switch(LOGGING_BUTTON, robot_specifications->do_log); */
 
  G_display_switch(CONNECT_BASE_BUTTON, 
		   program_state->tcx_connected_to_BASE);

  /* NEW */
  G_display_switch(QUIT_BUTTON, 0 );
  G_display_switch(MAP_UPDATE_BUTTON, 
		   program_state->map_regular_update_on);
  G_display_switch(PANTILT_SUBTITLE, 
		   program_state->pantilt_window_mode);


#ifdef UNIBONN
  G_display_switch(CAMERA_MAPS_BUTTON, 2);  
  G_display_switch(CAMERA_COLLI_LINES_BUTTON, 2);
  G_display_value(ARM_GRIPPER_ORIENTATION_BUTTON, 0.0);
  G_display_value(ARM_MAST_POSITION_BUTTON, 0.0);
  G_display_value(ARM_GRIPPER_PAD_POSITION_BUTTON, 0.0);
  G_display_value(ARM_IN_OUT_BUTTON, 0.0);
#endif /* UNIBONN */
  
  G_display_value(CAMERA_TILT, sensation->tilt);
  G_display_value(CAMERA_PAN,  -sensation->pan); /*!sign fixed*/
  G_display_switch(SONAR_ROBOT_BACKGROUND, 0);
  G_display_switch(SONAR_ROBOT_SUBTITLE2, 0);
  G_display_switch(SONAR_ROBOT_SUBTITLE3, 0);
  G_display_robot(LASER_ROBOT, 0.0, 0.0, 90.0,
		  robot_specifications->num_laser_sensors, dummy_sensors);
  G_display_robot(SONAR_ROBOT, 0.0, 0.0, 90.0,
		  robot_specifications->num_sonar_sensors, dummy_sensors);
  G_display_value(ROBOT_TRANS_SET_VELOCITY, robot_state->trans_set_velocity);
  G_display_value(ROBOT_ROT_SET_VELOCITY, robot_state->rot_set_velocity);
  G_display_value(ROBOT_TRANS_VELOCITY, robot_state->trans_velocity);
  G_display_value(ROBOT_ROT_VELOCITY, robot_state->rot_velocity);
  G_display_value(ROBOT_TRANS_SET_ACCELERATION, 
		  robot_state->trans_set_acceleration);
  G_display_value(ROBOT_ROT_SET_ACCELERATION, 
		  robot_state->rot_set_acceleration);


  
  G_display_switch(RED_LIGHT_BUTTON, robot_state->red_light_status);
  G_display_switch(YELLOW_LIGHT_BUTTON, robot_state->yellow_light_status);
  G_display_switch(GREEN_LIGHT_BUTTON, robot_state->green_light_status);
  G_display_switch(BLUE_LIGHT_BUTTON, robot_state->blue_light_status);
  G_display_switch(RED_PUSHED_BUTTON, sensation->red_button_status);
  G_display_switch(YELLOW_PUSHED_BUTTON, sensation->yellow_button_status);
  G_display_switch(GREEN_PUSHED_BUTTON, sensation->green_button_status);
  G_display_switch(BLUE_PUSHED_BUTTON, sensation->blue_button_status);

  
  G_display_switch(EXPL_RESET_BUTTON, program_state->auto_reset_expl_mode);

  sprintf(button_text,  "total area (%g)", program_state->total_area_covered);
  G_set_new_text(STATUS_TOTAL_AREA_BUTTON, button_text, 0);
  G_display_value(STATUS_TOTAL_AREA_BUTTON,  
		  program_state->total_area_covered);


  sprintf(button_text,  "new area (%g)", program_state->d_total_area_covered);
  G_set_new_text(STATUS_NEW_AREA_BUTTON, button_text, 0);
  G_display_value(STATUS_NEW_AREA_BUTTON,  
		  program_state->d_total_area_covered);


  sprintf(button_text,  "advance (%g)", program_state->advance_rate);
  G_set_new_text(STATUS_ADVANCEMENT_BUTTON, button_text, 0);
  G_display_value(STATUS_ADVANCEMENT_BUTTON,  
		  program_state->advance_rate);

  /*  G_display_value(EPISODE_DIAL, 0.0);*/
  /*  G_display_value(EVENT_DIAL, 0.0);*/
  

  if (program_state->graphics_initialized)
    G_display_all();
  display_robot_window(ALL);
  
  program_state->graphics_initialized = 1;
}



/************************************************************************
 *
 *   NAME:         select_active_objects()
 *                 
 *   FUNCTION:     depending on the status of the progam, graphic
 *                 objects are activated and de-activated
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void select_active_objects(ALL_PARAMS)
{
  if (!program_state->script_on){ /* normal mode of operation */
    G_activate(PATH);
    G_activate(CAMERA_TILT);
    G_activate(CAMERA_PAN);

    G_activate(PATH);
    G_deactivate(SCRIPT_PATH);
/*    G_deactivate(EPISODE_DIAL);
    G_deactivate(EVENT_DIAL);
    G_deactivate(DECREASE_EPISODE_BUTTON);
    G_deactivate(INCREASE_EPISODE_BUTTON);
    G_deactivate(DECREASE_EVENT_BUTTON);
    G_deactivate(INCREASE_EVENT_BUTTON); */
    G_deactivate(SCRIPT_CAMERA_TILT);
    G_deactivate(SCRIPT_CAMERA_PAN);
    G_deactivate(SCRIPT_EVENT_DATE_BUTTON);
  }
  else{			/* data comes from a script file */
    
    G_deactivate(PATH);
    G_deactivate(CAMERA_TILT);
    G_deactivate(CAMERA_PAN);
    
    G_activate(SCRIPT_PATH);
    G_deactivate(PATH);
    /* G_activate(EPISODE_DIAL); */
    /* G_activate(EVENT_DIAL);*/
    /*G_activate(DECREASE_EPISODE_BUTTON);*/
    /*G_activate(INCREASE_EPISODE_BUTTON);*/
    /*G_activate(DECREASE_EVENT_BUTTON);*/
    /*G_activate(INCREASE_EVENT_BUTTON);*/
    G_activate(SCRIPT_CAMERA_TILT);
    G_activate(SCRIPT_CAMERA_PAN);
    G_activate(SCRIPT_EVENT_DATE_BUTTON);
  }
}




/************************************************************************
 *
 *   NAME:         autoshift_display
 *                 
 *   FUNCTION:     Shifts robot display to always keep robot centered
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void autoshift_display(float target_x, float target_y, 
		       float safety_margin, float autoshift_distance,
		       ALL_PARAMS)
{
  float robot_x, robot_y;
  int from_x, to_x, increment_x, from_y, to_y, increment_y;
  int x,y, i;
  int index_from, index_to;
  float *global_map;
  int   *global_map_active;
  register int shift_x = 0, shift_y = 0;

  if (robot_specifications->autoshift && program_state->do_Xdisplay){
    shift_x = 0;
    shift_y = 0;
    robot_x = target_x + robot_specifications->autoshifted_x;
    robot_y = target_y + robot_specifications->autoshifted_y;


      /* test left margin */
    if (robot_x < safety_margin)
      shift_x = 0 + ((int) ((autoshift_distance 
			      - robot_x) 
			     / robot_specifications->map_resolution)); /* >0 */
    
    /* test right margin */
    if (robot_x > robot_specifications->global_worldsize_x -
	safety_margin)
      shift_x = -0 + ((int) ((robot_specifications->global_worldsize_x
			       - robot_x 
			       - autoshift_distance) 
			      / robot_specifications->map_resolution)); /* <0*/
    
    
    /* test lower margin */
    if (robot_y < safety_margin)
      shift_y = 0 + ((int) ((autoshift_distance 
			      - robot_y) 
			     / robot_specifications->map_resolution)); /* >0 */
    
    /* test upper margin */
    if (robot_y > robot_specifications->global_worldsize_y -
	safety_margin)
      shift_y = -0 + ((int) ((robot_specifications->global_worldsize_y 
			       - robot_y 
			       - autoshift_distance) 
			      / robot_specifications->map_resolution)); /* <0*/
    
    
	
    
    if (shift_x != 0 || shift_y != 0){ /* ie., if autoshift necessary */
      

      global_map         = robot_specifications->occupancy_values;
      global_map_active  = robot_specifications->map_active;

      
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
      
      if (from_x >= 0 && to_x >= -1 && from_y >= 0 && to_y >= -1 &&
	  from_x < robot_specifications->global_map_dim_x &&
	  to_x <= robot_specifications->global_map_dim_x &&
	  from_y < robot_specifications->global_map_dim_y &&
	  to_y <= robot_specifications->global_map_dim_y){
	if (shift_x != 0)
	  for (x = from_x; x != to_x; x += increment_x)
	    for (y = 0; y < robot_specifications->global_map_dim_y; y++){
	      index_from = (x-shift_x)
		* robot_specifications->global_map_dim_y
		  + y;
	      index_to   = x * robot_specifications->global_map_dim_y + y;
#ifdef forget		
	      if (index_from < 0 ||
		  index_from >= robot_specifications->global_map_dim_x
		  * robot_specifications->global_map_dim_y ||
		  index_to < 0 ||
		  index_to >= robot_specifications->global_map_dim_x
		  * robot_specifications->global_map_dim_y){
		fprintf(stderr, "\n@@@ SEVERE ERROR1: %g  %g %g %g %d %d %d %d  %g %g %g %d %d %d %d\n",
			autoshift_distance,
			target_x, robot_x, robot_specifications->autoshifted_x,
			shift_x, from_x, to_x, increment_x, 
			target_y, robot_y, robot_specifications->autoshifted_y,
			shift_y, from_y, to_y, increment_y);
		putc(7,stderr);
	      }
#endif		
	      
	      global_map[index_to] = global_map[index_from];
	      global_map_active[index_to] = global_map_active[index_from];
	      global_map[index_from] = 0.5;
	      global_map_active[index_from] = 0;
	    }
	if (shift_y != 0)
	  for (x = 0; x < robot_specifications->global_map_dim_x; x++)
	    for (y = from_y; y != to_y; y += increment_y){
	      index_from = x * robot_specifications->global_map_dim_y
		+ (y-shift_y);
	      index_to   = x * robot_specifications->global_map_dim_y + y;
	      
	      
	      global_map[index_to] = global_map[index_from];
	      global_map_active[index_to] = global_map_active[index_from]; 
	      global_map[index_from] = 0.5;
	      global_map_active[index_from] = 0;
	    }
      }
      else{
	fprintf(stderr, "##################################3\n");
	fprintf(stderr, "##################################3\n");
	fprintf(stderr, "##################################3\n");
	fprintf(stderr, "##################################3 %d %d\n",
		shift_x, shift_y);
	for (index_from = 0; 
	     index_from < robot_specifications->global_map_dim_x
	     * robot_specifications->global_map_dim_y; index_from++){
	  global_map[index_from] = 0.5;
	  global_map_active[index_from] = 0;
	}	  
      }
      

      /* ---------> CHANGE GLOBAL VARIABLES */
      robot_specifications->autoshifted_x += 	
	((float) shift_x) * robot_specifications->map_resolution;
      robot_specifications->autoshifted_y += 
	((float) shift_y) * robot_specifications->map_resolution;
      robot_specifications->autoshifted_int_x += shift_x;
      robot_specifications->autoshifted_int_y += shift_y;
      
      G_shift_robot_local_coordinates(GLOBAL_ROBOT, 
				      ((float) shift_x)
				      * robot_specifications->map_resolution,
				      ((float) shift_y)
				      * robot_specifications->map_resolution);
      G_shift_markers_local_coordinates(PATH, 
				      ((float) shift_x)
				      * robot_specifications->map_resolution,
				      ((float) shift_y)
				      * robot_specifications->map_resolution);
      G_shift_markers_local_coordinates(SCRIPT_PATH, 
				      ((float) shift_x)
				      * robot_specifications->map_resolution,
				      ((float) shift_y)
				      * robot_specifications->map_resolution);
      G_shift_markers_local_coordinates(TARGET_POINT_GLOBAL,
				      ((float) shift_x)
				      * robot_specifications->map_resolution,
				      ((float) shift_y)
				      * robot_specifications->map_resolution);
#ifdef UNIBONN
      G_shift_markers_local_coordinates(GOALS, 
				      ((float) shift_x)
				      * robot_specifications->map_resolution,
				      ((float) shift_y)
				      * robot_specifications->map_resolution);
      G_shift_markers_local_coordinates(TARGET_OBJECTS, 
				      ((float) shift_x)
				      * robot_specifications->map_resolution,
				      ((float) shift_y)
				      * robot_specifications->map_resolution);
#endif /* UNIBONN */
#ifdef TOURGUIDE_VERSION
      for (i = 0; i < MAX_NUM_TOURS; i++){
	G_shift_markers_local_coordinates(TOUR_GOALS[i], 
					  ((float) shift_x)
					  * robot_specifications->map_resolution,
					  ((float) shift_y)
					  * robot_specifications->map_resolution);
	G_shift_markers_local_coordinates(TOUR_ACTIVE_GOALS[i], 
					  ((float) shift_x)
					  * robot_specifications->map_resolution,
					  ((float) shift_y)
					  * robot_specifications->map_resolution);
	G_shift_markers_local_coordinates(TOUR_OBJECTS[i], 
					  ((float) shift_x)
					  * robot_specifications->map_resolution,
					  ((float) shift_y)
					  * robot_specifications->map_resolution);
      }
#endif /* TOURGUIDE_VERSION */

      /* ---------> GRAPHICS DISPLAY */

      G_display_switch(GLOBAL_ROBOT_BACKGROUND, 0);
      display_global_robot(ALL);
    }
  }
}



/************************************************************************
 *
 *   NAME:         display_robot_window
 *                 
 *   FUNCTION:     displays the robot's windows, can't you read?
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

struct timeval last_display_robot_window_time = {0, 0};


void display_robot_window(ALL_PARAMS)
{
  struct timeval this_time;
  float  time_difference;
  int    i, target_type;
  float  target_x, target_y, target_x2, target_y2, target_dist;

  if (program_state->do_Xdisplay){

    gettimeofday(&this_time, NULL);
    time_difference = 
      ((float) (this_time.tv_sec - last_display_robot_window_time.tv_sec))
	+ (((float) (this_time.tv_usec 
		       - last_display_robot_window_time.tv_usec))
	   /  1000000.0);
    if (time_difference > 
	robot_specifications->sensor_update_interval){
      
      autoshift_display(robot_state->x, 
			robot_state->y, 
			robot_specifications->safety_margin, 
			robot_specifications->autoshift_distance, 
			ALL);

      G_clear_markers(TARGET_POINT_LOCAL);
      for (i = 0; i < G_return_num_markers(TARGET_POINT_GLOBAL, 1); i++){
	G_return_marker_coordinates(TARGET_POINT_GLOBAL, i,
				    &target_x, &target_y, &target_type);
	target_x2 = (target_x - robot_state->x);
	target_y2 = (target_y - robot_state->y);
	target_dist = sqrt((target_x2 * target_x2) + (target_y2 * target_y2));
	if (target_dist < robot_specifications->max_sonar_range)
	  G_add_marker(TARGET_POINT_LOCAL, 
		       (target_x2
			* sin(robot_state->orientation * M_PI / 180.0))
		       - (target_y2
			  * cos(robot_state->orientation * M_PI / 180.0)),
		       (target_x2
			* cos(robot_state->orientation * M_PI / 180.0))
		       + (target_y2
			  * sin(robot_state->orientation * M_PI / 180.0)), 0);
      }

      G_display_value(CAMERA_TILT, sensation->tilt);
      G_display_value(CAMERA_PAN,  -sensation->pan); /*!sign  */
      /* G_display_switch(SONAR_ROBOT_BACKGROUND, 0);*/
      G_display_switch(SONAR_ROBOT_SUBTITLE2, 0);
      G_display_switch(SONAR_ROBOT_SUBTITLE3, 0);
      G_display_robot(LASER_ROBOT, 0.0, 0.0, 90.0,
		      robot_specifications->num_laser_sensors, 
		      sensation->laser_values);
      G_display_markers(TARGET_POINT_LOCAL);
      G_display_robot(SONAR_ROBOT, 0.0, 0.0, 90.0,
		      robot_specifications->num_sonar_sensors, 
		      sensation->sonar_values);
      G_display_value(ROBOT_TRANS_SET_VELOCITY, robot_state->trans_set_velocity);
      G_display_value(ROBOT_ROT_SET_VELOCITY, robot_state->rot_set_velocity);
      G_display_value(ROBOT_TRANS_VELOCITY, robot_state->trans_velocity);
      G_display_value(ROBOT_ROT_VELOCITY, robot_state->rot_velocity);
      G_display_value(ROBOT_TRANS_SET_ACCELERATION, 
		      robot_state->trans_set_acceleration);
      G_display_value(ROBOT_ROT_SET_ACCELERATION, 
		      robot_state->rot_set_acceleration);
      
      G_display_switch(RED_LIGHT_BUTTON, robot_state->red_light_status);
      G_display_switch(YELLOW_LIGHT_BUTTON, robot_state->yellow_light_status);
      G_display_switch(GREEN_LIGHT_BUTTON, robot_state->green_light_status);
      G_display_switch(BLUE_LIGHT_BUTTON, robot_state->blue_light_status);
      G_display_switch(RED_PUSHED_BUTTON, sensation->red_button_status);
      G_display_switch(YELLOW_PUSHED_BUTTON, sensation->yellow_button_status);
      G_display_switch(GREEN_PUSHED_BUTTON, sensation->green_button_status);
      G_display_switch(BLUE_PUSHED_BUTTON, sensation->blue_button_status);
      /*
	 G_display_matrix(OCCUPANCY_MAP);
	 */
      G_display_markers(PATH);
      G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y, 
		      robot_state->orientation, 0, dummy_sensors);
      G_display_switch(POSITION_TEXT_BUTTON, 0);

      G_display_value(BATTERY_STATUS, robot_state->battery_level);

      last_display_robot_window_time.tv_sec  = this_time.tv_sec;
      last_display_robot_window_time.tv_usec = this_time.tv_usec;
      
    }
      
  }
}






