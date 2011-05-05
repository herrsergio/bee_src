
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
#include "tcx.h"
#include "tcxP.h"
#include "MAP-messages.h"
#include "SONAR-messages.h"
#include "BASE-messages.h"
#include "MAP.h"
#include "EZX11.h"
#include "o-graphics.h"



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* BUTTONS and other graphic objects */


int GLOBAL_BACKGROUND;
int GLOBAL_MAPVALUES;
int GLOBAL_VORONOI;
int GLOBAL_CORRELATIONS;
int DISPLAY_GLOBAL_MAPVALUES_BUTTON;
int DISPLAY_LOCAL_MAPVALUES_BUTTON;
int DISPLAY_VORONOI_DIAGRAM_BUTTON;
int DISPLAY_VORONOI_DIAGRAM_INCREMENT_BUTTON;
int DISPLAY_VORONOI_DIAGRAM_DECREMENT_BUTTON;
#ifdef must_this_be
int DISPLAY_GLOBAL_CORRELATIONS_BUTTON;
#endif
int GLOBAL_ROBOT;
int LOCAL_BACKGROUND;
int LOCAL_MAPVALUES;
int QUIT_BUTTON;
int SAVE_BUTTON;
int LOAD_BUTTON;
int DUMP_BUTTON;
int PATH;
int BASE_CONNECTED_BUTTON;
int LOCALIZE_CONNECTED_BUTTON;
int LINE_ANGLE_BUTTON;
int ACTUAL_MAP_BUTON;
int BEST_FIT_POINT;
int COMBINE_MAPS_BUTTON;
int FREEZE_BUTTON;
int MAP_UPDATE_ENABLE_BUTTON;
int TOPOLOGICAL_GRAPH;
int CLEAR_MAP_BUTTON;

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
	  "9x15", "10x20", "12x24", 
	  "lucidasans-bold-18"};
/*  static char *myfonts[] = {"5x8", "6x10", "6x10", 
			      "9x15", "9x15", "12x24", 
			      "10x20"};
*/
  if (!program_state->use_graphics)
    return;

  /* =============================================================
     ====================  1) INITIALIZATIONS  ===================
     ============================================================= */
  
  if (!program_state->graphics_initialized){
    
    G_initialize_fonts(7, myfonts);
    G_initialize_graphics("BeeSoft Mapper",
			  robot_specifications->X_window_size,
			  robot_specifications->X_window_size / 7.0,
			  C_SLATEGREY);
    
    
    
    /* =============================================================
       ====================  2) SPECIFICATIONS  ====================
       ============================================================= */


    {
      /******** DISPLAY_GLOBAL_MAPVALUES_BUTTON ****************************/
      int switch_num                      = 3;
      static float switch_pos[]           = {RIGHT_BAR, 6.1, 6.4};
      static char *switch_texts[]         = 
	{"show global map", "show global map", "...busy"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= {C_GREY90, C_YELLOW, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK, C_WHITE};
      DISPLAY_GLOBAL_MAPVALUES_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }




    {
      /******** ACTUAL_MAP_BUTON ****************************/
      int switch_num                      = NUM_GLOBAL_MAPS;
      static float switch_pos[]           = {RIGHT_BAR, 5.7, 6.0};
      static char *switch_texts[]         = {
	"(combined map)", "(sonar map)", "(laser map)", "(CAD map)"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]=
	{C_GREY90,C_GREY90,C_GREY90,C_GREY90};
      static int switch_frame_color[]     =
	{C_GREY40,C_GREY40,C_GREY40,C_GREY40};
      static int switch_text_color[]      = {C_BLACK,C_BLACK,C_BLACK,C_BLACK};
      ACTUAL_MAP_BUTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    if (NUM_GLOBAL_MAPS > 4){
      fprintf(stderr, "You must fix ACTUAL_MAP_BUTON in map-graphics.c\n");
      fprintf(stderr, "when changing the number of maps.\n");
      
    }
    
    {
      /******** DISPLAY_LOCAL_MAPVALUES_BUTTON ****************************/
      int switch_num                      = 3;
      static float switch_pos[]           = {RIGHT_BAR, 5.3, 5.6};
      static char *switch_texts[]         = 
	{"show local map", "show local map", "...busy"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_GREY90, C_YELLOW, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK, C_WHITE};
      DISPLAY_LOCAL_MAPVALUES_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
        
    {
      /******** DISPLAY_VORONOI_DIAGRAM_BUTTON ****************************/
      int switch_num                      = NUM_VORONOI_MODI+1;
      static float switch_pos[]           = 
	{6.1, (5.73+LOCAL_WINDOW_SIZE), 4.9, 5.2};
      static char *switch_texts[NUM_VORONOI_MODI+1] = 
	{"voronoi diag",
	   "critical lines",
	   "regions",
	   "regions and text",
	   "graph",
	   "pruned reg.",
	   "pruned reg+text",
	   "pruned graph",
	   "...busy"};
      static int switch_fonts[]           = {2,2,2,2,2,2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY90, C_GREY90, C_GREY90, C_GREY90, C_GREY90, C_GREY90,
	   C_GREY90, C_GREY90, C_RED};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40, C_GREY40, C_GREY40,
	   C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, 
	   C_BLACK, C_WHITE};
      DISPLAY_VORONOI_DIAGRAM_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    {
      /******** DISPLAY_VORONOI_DIAGRAM_INCREMENT_BUTTON ********************/
      int switch_num                      = 1;
      static float switch_pos[]           = 
	{(5.76+LOCAL_WINDOW_SIZE), (6.1+LOCAL_WINDOW_SIZE), 5.06, 5.2};
      static char *switch_texts[NUM_VORONOI_MODI+1] = {"+"};
      static int switch_fonts[]           = {1};
      static int switch_background_color[]= {C_GREY90};
      static int switch_frame_color[]     = {C_GREY40};
      static int switch_text_color[]      = {C_BLACK};
      DISPLAY_VORONOI_DIAGRAM_INCREMENT_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    {
      /******** DISPLAY_VORONOI_DIAGRAM_DECREMENT_BUTTON ********************/
      int switch_num                      = 1;
      static float switch_pos[]           = 
	{(5.76+LOCAL_WINDOW_SIZE), (6.1+LOCAL_WINDOW_SIZE), 4.9, 5.03};
      static char *switch_texts[NUM_VORONOI_MODI+1] = {"-"};
      static int switch_fonts[]           = {1};
      static int switch_background_color[]= {C_GREY90};
      static int switch_frame_color[]     = {C_GREY40};
      static int switch_text_color[]      = {C_BLACK};
      DISPLAY_VORONOI_DIAGRAM_DECREMENT_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

#ifdef UNIBONN
#define DIFFx (0)
#else    
#define DIFFx (0.4)
#endif    

    {
      /******** MAP_UPDATE_ENABLE_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_BAR, 4.5+DIFFx, 4.8+DIFFx};
      static char *switch_texts[]         = 
	{"update disabled", "update enabled"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_RED, C_GREY90};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_WHITE, C_BLACK};
      MAP_UPDATE_ENABLE_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    

    {
      /******** CLEAR_MAP_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_BAR, 4.1+DIFFx, 4.4+DIFFx};
      static char *switch_texts[]         = {"clear all maps", "...clearing"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90,C_GREY90};
      static int switch_frame_color[]     = {C_GREY40,C_GREY40};
      static int switch_text_color[]      = {C_BLACK,C_BLACK};
      CLEAR_MAP_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    {
      /******** FREEZE_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_BAR, 3.7+DIFFx, 4.0+DIFFx};
      static char *switch_texts[]         = 
	{"clip map", "..busy"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= 
	{C_GREY90, C_RED};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_BLACK, C_WHITE};
      FREEZE_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    
    {
      /******** COMBINE_MAPS_BUTTON ****************************/
      int switch_num                      = 4;
      static float switch_pos[]           = {RIGHT_BAR, 4.5+DIFFx, 4.8+DIFFx};
      static char *switch_texts[]         = 
	{"auto fit", "ENTER POINTS", "(activated)", "*** ERROR ***",
       "...busy"};
      static int switch_fonts[]           = {2,2,2,2};
      static int switch_background_color[]= 
	{C_GREY90, C_BLUE, C_YELLOW, C_RED, C_RED};
      static int switch_frame_color[]     = 
	{C_GREY40, C_GREY40, C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = 
	{C_BLACK, C_WHITE, C_BLACK, C_WHITE, C_WHITE};
      COMBINE_MAPS_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }


#ifdef must_this_be
    {    
      /******** DISPLAY_GLOBAL_CORRELATIONS_BUTTON **************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_BAR, 4.5, 4.8};
      static char *switch_texts[]         = 
	{"correlations", "...busy"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_WHITE};
      DISPLAY_GLOBAL_CORRELATIONS_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
#endif    

    {      
      /******** LOAD_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = 
	{6.1, (6.1+(0.5 * LOCAL_WINDOW_SIZE)-0.03), 3.3+DIFFx, 3.6+DIFFx};
      static char *switch_texts[]         = {"load map", "loading"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_WHITE};
      LOAD_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
 
    {      
      /******** SAVE_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = 
	{(6.1+(0.5 * LOCAL_WINDOW_SIZE)+0.03), 
	   (6.1+LOCAL_WINDOW_SIZE), 3.3+DIFFx, 3.6+DIFFx};
      static char *switch_texts[]         = {"save map", "saving"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_GREY90, C_YELLOW};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK};
      SAVE_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    {      
      /******** DUMP_BUTTON ****************************/
      int switch_num                      = 3;
      static float switch_pos[]           = {RIGHT_BAR, 2.9+DIFFx, 3.2+DIFFx};
      static char *switch_texts[]         = 
      {"plot map", "...plotting", "...plotting (special)"};
      static int switch_fonts[]           = {2,2,2};
      static int switch_background_color[]= {C_GREY90, C_YELLOW, C_YELLOW};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_BLACK, C_BLACK};
      DUMP_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

    {
      
      /******** QUIT_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = {RIGHT_BAR, 2.5+DIFFx, 2.8+DIFFx};
      static char *switch_texts[]         = {"quit", "quit"};
      static int switch_fonts[]           = {2,2};
      static int switch_background_color[]= {C_YELLOW, C_RED};
      static int switch_frame_color[]     = {C_GREY40, C_GREY40};
      static int switch_text_color[]      = {C_BLACK, C_WHITE};
      QUIT_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }

   {
      /******** GLOBAL_BACKGROUND **************************************/
      int switch_num                      = 1;
      static float switch_pos[]           = {-0.4, 6.0, 0.0, 6.4};
      static char *switch_texts[]         = {"oops"};
      static int switch_fonts[]           = {5};
      static int switch_background_color[]= {C_TURQUOISE4};
      static int switch_frame_color[]     = {C_GREY40};
      static int switch_text_color[]      = {NO_COLOR};
      GLOBAL_BACKGROUND 
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
    {
      /******** LOCAL_BACKGROUND **************************************/
      int switch_num                      = 1;
      static float switch_pos[]           =
	{RIGHT_BAR, 0.0, LOCAL_WINDOW_SIZE}; 
      static char *switch_texts[]         = {"                       "};
      static int switch_fonts[]           = {2};
      static int switch_background_color[]= {C_TURQUOISE4};
      static int switch_frame_color[]     = {C_GREY40};
      static int switch_text_color[]      = {C_RED};

      LOCAL_BACKGROUND 
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    
    

    {
      /******** GLOBAL_CORRELATIONS ****************************************/
      static float matrix_pos[]            = {-0.4, 6.0, 0.0, 6.4};
      char *matrix_text                    = "Oops - should not be here";
      static int matrix_font               = 0;
      float min_value_matrix               = -1.0;
      float max_value_matrix               = 1.0;
      static int matrix_colors[]           = {C_TURQUOISE4,C_GREY40,NO_COLOR}; 
      
      GLOBAL_CORRELATIONS = 
	G_create_matrix_object(matrix_pos, matrix_text, 
			       global_local_correlations, global_active,
			       robot_specifications->global_map_dim_x,
			       robot_specifications->global_map_dim_y,
			       min_value_matrix, max_value_matrix, 
			       matrix_colors, matrix_font);
    }
    
    {
      /******** GLOBAL_VORONOI *******************************************/
      static float matrix_pos[]            = {-0.4, 6.0, 0.0, 6.4};
      char *matrix_text                    = "Oops - should not be here";
      static int matrix_font               = 0;
      float min_value_matrix               = 0.0;
      float max_value_matrix               = 1.0;
      static int matrix_colors[]           = {C_TURQUOISE4,C_GREY40,NO_COLOR}; 
      
      GLOBAL_VORONOI = 
	G_create_matrix_object(matrix_pos, matrix_text, 
			       global_voronoi, global_voronoi_active,
			       robot_specifications->global_map_dim_x,
			       robot_specifications->global_map_dim_y,
			       min_value_matrix, max_value_matrix, 
			       matrix_colors, matrix_font);
    }
    {
      /******** GLOBAL_MAPVALUES *******************************************/
      static float matrix_pos[]            = {-0.4, 6.0, 0.0, 6.4};
      char *matrix_text                    = "Oops - should not be here";
      static int matrix_font               = 0;
      float min_value_matrix               = 0.0;
      float max_value_matrix               = 1.0;
      static int matrix_colors[]           = {C_TURQUOISE4,C_GREY40,NO_COLOR}; 
      
      GLOBAL_MAPVALUES = 
	G_create_matrix_object(matrix_pos, matrix_text, 
			       global_map, global_active,
			       robot_specifications->global_map_dim_x,
			       robot_specifications->global_map_dim_y,
			       min_value_matrix, max_value_matrix, 
			       matrix_colors, matrix_font);
    }
    

    {
      /******** LOCAL_MAPVALUES *******************************************/
      static float matrix_pos[]            = 
	{RIGHT_BAR, 0.0, LOCAL_WINDOW_SIZE}; 
      char *matrix_text                    = "                       ";
      static int matrix_font               = 2;
      float min_value_matrix               = 0.0;
      float max_value_matrix               = 1.0;
      static int matrix_colors[]           = {C_TURQUOISE4,C_GREY40,C_RED}; 
      
      LOCAL_MAPVALUES = 
	G_create_matrix_object(matrix_pos, matrix_text, 
			       local_map, local_active,
			       robot_specifications->local_map_dim_x,
			       robot_specifications->local_map_dim_y,
			       min_value_matrix, max_value_matrix, 
			       matrix_colors, matrix_font);
    }
    

    /******** PATH *************************************************/
    {
      static float pos_mar[]                   = {-0.4, 6.0, 0.0, 6.4};
      int num_mar                            = 1;
      static char *text_mar[]                = {"PATH"};
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[]      = {C_BLUE};
      static int mar_text_color[]            = {NO_COLOR};
      static int mar_fonts[]                 = {2};
      int i;

      PATH
	= G_create_markers_object(pos_mar, 1,
				  /*robot_specifications->robot_size * 0.3,*/
				  0.0, 
				  0.0, 
				  robot_specifications->global_mapsize_x,
				  0.0, 
				  robot_specifications->global_mapsize_y,
				  num_mar, text_mar, mar_background_color, 
				  mar_frame_color, mar_foreground_color, 
				  mar_text_color, mar_fonts);
      
    }
    
    {
      /******** ROBOT (BIG) IN GLOBAL MAP *********************************/ 
      static float pos_r[]                 = {-0.4, 6.0, 0.0, 6.4};
      static char *text_r                  = "MAPPER";
      static int robot_font                = 4;
      static int colors_r[]                = {NO_COLOR, C_GREY40, C_RED,
						C_BLACK, C_VIOLET, NO_COLOR, 
						NO_COLOR, NO_COLOR};
      
      GLOBAL_ROBOT =
	G_create_robot_object(pos_r, text_r, 
			      0.0, robot_specifications->global_mapsize_x,
			      0.0, robot_specifications->global_mapsize_y,
			      robot_specifications->global_mapsize_x / 2.0,
			      robot_specifications->global_mapsize_y / 2.0,
			      robot_state->orientation, 
			      robot_specifications->robot_size, 
			      0, 0.0, NULL, NULL,
			      colors_r, robot_font);
      
    }

    {      
      /******** LINE_ANGLE_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = 
	{RIGHT_BAR, LOCAL_WINDOW_SIZE + 0.4, LOCAL_WINDOW_SIZE + 0.5};
      static char *switch_texts[]         = {"",
					       "???????????????????"};
      static int switch_fonts[]           = {0,0};
      static int switch_background_color[]= {C_SLATEGREY, C_SLATEGREY};
      static int switch_frame_color[]     = {NO_COLOR, NO_COLOR};
      static int switch_text_color[]      = {C_WHITE, C_LAWNGREEN};
      LINE_ANGLE_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    

    {      
      /******** BASE_CONNECTED_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = 
	{RIGHT_BAR, LOCAL_WINDOW_SIZE + 0.25, LOCAL_WINDOW_SIZE + 0.35};
      static char *switch_texts[]         = {"(COLLI not connected)",
					       "(COLLI connected)"};
      static int switch_fonts[]           = {0,0};
      static int switch_background_color[]= {C_SLATEGREY, C_SLATEGREY};
      static int switch_frame_color[]     = {NO_COLOR, NO_COLOR};
      static int switch_text_color[]      = {C_GREY90, C_LAWNGREEN};
      BASE_CONNECTED_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    

    

    {      
      /******** LOCALIZE_CONNECTED_BUTTON ****************************/
      int switch_num                      = 2;
      static float switch_pos[]           = 
	{RIGHT_BAR, LOCAL_WINDOW_SIZE + 0.1, LOCAL_WINDOW_SIZE + 0.2};
      static char *switch_texts[]         = {"(LOC not connected)",
					       "(LOC connected)"};
      static int switch_fonts[]           = {0,0};
      static int switch_background_color[]= {C_SLATEGREY, C_SLATEGREY};
      static int switch_frame_color[]     = {NO_COLOR, NO_COLOR};
      static int switch_text_color[]      = {C_GREY90, C_LAWNGREEN};
      LOCALIZE_CONNECTED_BUTTON
	= G_create_switch_object(switch_pos, switch_num, switch_texts,
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
    }
    

    

    {
      /******** BEST_FIT_POINT *********************************/ 
      static float pos_r[]                 = {-0.4, 6.0, 0.0, 6.4};
      static char *text_r                  = "Yuppie";
      static int robot_font                = 4;
      static int colors_r[]                = {NO_COLOR, NO_COLOR, C_LAWNGREEN,
						C_BLACK, NO_COLOR, NO_COLOR, 
						NO_COLOR, NO_COLOR};
      
      BEST_FIT_POINT =
	G_create_robot_object(pos_r, text_r, 
			      0.0, robot_specifications->global_mapsize_x,
			      0.0, robot_specifications->global_mapsize_y,
			      robot_specifications->global_mapsize_x / 2.0,
			      robot_specifications->global_mapsize_y / 2.0,
			      90.0,
			      3.0 * robot_specifications->resolution,
			      0, 0.0, NULL, NULL,
			      colors_r, robot_font);
      
    }
   
    /******** TOPOLOGICAL_GRAPH **************************************/
    {
      static float pos_r[]                 = {-0.4, 6.0, 0.0, 6.4};
      int num_mar                            = 500;
      static char *text_mar[] = {"0", "1", "2", "3", "4", "5", "6",
        "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", "17",
        "18", "19", "20", "21", "22", "23", "24", "25", "26", "27",
        "28", "29", "30", "31", "32", "33", "34", "35", "36", "37",
        "38", "39", "40", "41", "42", "43", "44", "45", "46", "47",
        "48", "49", "50", "51", "52", "53", "54", "55", "56", "57",
        "58", "59", "60", "61", "62", "63", "64", "65", "66", "67",
        "68", "69", "70", "71", "72", "73", "74", "75", "76", "77",
        "78", "79", "80", "81", "82", "83", "84", "85", "86", "87",
        "88", "89", "90", "91", "92", "93", "94", "95", "96", "97",
        "98", "99", "100", "101", "102", "103", "104", "105", "106",
        "107", "108", "109", "110", "111", "112", "113", "114", "115",
        "116", "117", "118", "119", "120", "121", "122", "123", "124",
        "125", "126", "127", "128", "129", "130", "131", "132", "133",
        "134", "135", "136", "137", "138", "139", "140", "141", "142",
        "143", "144", "145", "146", "147", "148", "149", "150", "151",
        "152", "153", "154", "155", "156", "157", "158", "159", "160",
        "161", "162", "163", "164", "165", "166", "167", "168", "169",
        "170", "171", "172", "173", "174", "175", "176", "177", "178",
        "179", "180", "181", "182", "183", "184", "185", "186", "187",
        "188", "189", "190", "191", "192", "193", "194", "195", "196",
        "197", "198", "199", "200", "201", "202", "203", "204", "205",
        "206", "207", "208", "209", "210", "211", "212", "213", "214",
        "215", "216", "217", "218", "219", "220", "221", "222", "223",
        "224", "225", "226", "227", "228", "229", "230", "231", "232",
        "233", "234", "235", "236", "237", "238", "239", "240", "241",
        "242", "243", "244", "245", "246", "247", "248", "249", "250",
        "251", "252", "253", "254", "255", "256", "257", "258", "259",
        "260", "261", "262", "263", "264", "265", "266", "267", "268",
        "269", "270", "271", "272", "273", "274", "275", "276", "277",
        "278", "279", "280", "281", "282", "283", "284", "285", "286",
        "287", "288", "289", "290", "291", "292", "293", "294", "295",
        "296", "297", "298", "299", "300", "301", "302", "303", "304",
        "305", "306", "307", "308", "309", "310", "311", "312", "313",
        "314", "315", "316", "317", "318", "319", "320", "321", "322",
        "323", "324", "325", "326", "327", "328", "329", "330", "331",
        "332", "333", "334", "335", "336", "337", "338", "339", "340",
        "341", "342", "343", "344", "345", "346", "347", "348", "349",
        "350", "351", "352", "353", "354", "355", "356", "357", "358",
        "359", "360", "361", "362", "363", "364", "365", "366", "367",
        "368", "369", "370", "371", "372", "373", "374", "375", "376",
        "377", "378", "379", "380", "381", "382", "383", "384", "385",
        "386", "387", "388", "389", "390", "391", "392", "393", "394",
        "395", "396", "397", "398", "399", "400", "401", "402", "403",
        "404", "405", "406", "407", "408", "409", "410", "411", "412",
        "413", "414", "415", "416", "417", "418", "419", "420", "421",
        "422", "423", "424", "425", "426", "427", "428", "429", "430",
        "431", "432", "433", "434", "435", "436", "437", "438", "439",
        "440", "441", "442", "443", "444", "445", "446", "447", "448",
        "449", "450", "451", "452", "453", "454", "455", "456", "457",
        "458", "459", "460", "461", "462", "463", "464", "465", "466",
        "467", "468", "469", "470", "471", "472", "473", "474", "475",
        "476", "477", "478", "479", "480", "481", "482", "483", "484",
        "485", "486", "487", "488", "489", "490", "491", "492", "493",
        "494", "495", "496", "497", "498", "499"};
      int connected_mar                      = 0;
      static int mar_frame_color             = NO_COLOR;
      static int mar_background_color        = NO_COLOR;
      static int mar_foreground_color[] = {NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR,
        NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR};
      static int mar_text_color[] = {C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK,
        C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK, C_BLACK};
      static int mar_fonts[] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2};
  
      TOPOLOGICAL_GRAPH = 
        G_create_markers_object(pos_r, connected_mar, 
  				  robot_specifications->robot_size,
  				  0.0, robot_specifications->global_mapsize_x,
  				  0.0, robot_specifications->global_mapsize_y,
  				  num_mar, text_mar, mar_background_color, 
  				  mar_frame_color, mar_foreground_color, 
  				  mar_text_color, mar_fonts);
    }
    
    /* insert new graphics object here XXXXX */

  }
    
  
  
  /* =============================================================
     ====================  4) DISPLAY  ===========================
     ============================================================= */


#ifndef UNIBONN
  G_deactivate(DISPLAY_VORONOI_DIAGRAM_BUTTON);
  G_deactivate(GLOBAL_VORONOI);
  G_deactivate(DISPLAY_VORONOI_DIAGRAM_INCREMENT_BUTTON);
  G_deactivate(DISPLAY_VORONOI_DIAGRAM_DECREMENT_BUTTON);
#endif /*UNIBONN*/
  G_deactivate(COMBINE_MAPS_BUTTON);
  G_deactivate(BEST_FIT_POINT);
  G_deactivate(GLOBAL_MAPVALUES);
  G_deactivate(GLOBAL_CORRELATIONS);
  G_deactivate(LOCAL_MAPVALUES);
  G_deactivate(GLOBAL_VORONOI);
  G_deactivate(TOPOLOGICAL_GRAPH);
  G_set_matrix_display_style(1);
  G_display_all();
  G_clear_markers(PATH);
  G_display_switch(MAP_UPDATE_ENABLE_BUTTON, program_state->map_update_on);

  G_display_switch(BASE_CONNECTED_BUTTON, program_state->tcx_base_connected);
  G_shift_robot_local_coordinates(GLOBAL_ROBOT, 
				  robot_specifications->autoshifted_x,
				  robot_specifications->autoshifted_y);
  G_shift_robot_local_coordinates(BEST_FIT_POINT, 
				  robot_specifications->autoshifted_x,
				  robot_specifications->autoshifted_y);
  G_shift_markers_local_coordinates(PATH, 
				    robot_specifications->autoshifted_x,
				    robot_specifications->autoshifted_y);
  G_shift_markers_local_coordinates(TOPOLOGICAL_GRAPH,
				    robot_specifications->autoshifted_x,
				    robot_specifications->autoshifted_y);
  
  program_state->graphics_initialized = 1;
}







/************************************************************************
 *
 *   NAME:         autoshift_display_by
 *                 
 *   FUNCTION:     Shifts robot by (shift_x, shift_y) units
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR  program_state  Pointer  to general
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

void autoshift_display_by(PROGRAM_STATE_PTR  program_state,
			  ROBOT_SPECIFICATIONS_PTR robot_specifications,
			  int shift_x, int shift_y)
{
  int from_x, to_x, increment_x, from_y, to_y, increment_y;
  register int x = 0, y = 0;
  register int i, j;
  int index_from = 0, index_to = 0;


  if (shift_x != 0 || shift_y != 0){	/* ie., if non-zero autoshift  */
/*
    fprintf(stderr, "MAP-Graphics autoshift_display_by: -a %d %d -\n", shift_x, shift_y);
*/
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
	    index_from = (x-shift_x) * robot_specifications->global_map_dim_y
	      + y;
	    index_to   = x * robot_specifications->global_map_dim_y + y;
	    for (i = 0; i < NUM_GLOBAL_MAPS; i++){
	      global_map_x[i][index_to] = global_map_x[i][index_from];
	      global_active_x[i][index_to] = global_active_x[i][index_from];
	      global_label_x[i][index_to] = global_label_x[i][index_from];
	      global_map_x[i][index_from] = 0.5;
	      global_active_x[i][index_from] = 0;
	      global_label_x[i][index_from] = (unsigned char) 0;
	    }
#ifdef RHINO_PLUS
            rhino_init2 /* rhino stuff without ; */
#endif
	  }
      if (shift_y != 0)
	for (x = 0; x < robot_specifications->global_map_dim_x; x++)
	  for (y = from_y; y != to_y; y += increment_y){
	    index_from = x * robot_specifications->global_map_dim_y
	      + (y-shift_y);
	    index_to   = x * robot_specifications->global_map_dim_y + y;
	    for (i = 0; i < NUM_GLOBAL_MAPS; i++){
	      global_map_x[i][index_to] = global_map_x[i][index_from];
	      global_active_x[i][index_to] = global_active_x[i][index_from];
	      global_label_x[i][index_to] = global_label_x[i][index_from];
	      global_map_x[i][index_from] = 0.5;
	      global_active_x[i][index_from] = 0;
	      global_label_x[i][index_from] = (unsigned char) 0;
	    }
#ifdef RHINO_PLUS
            rhino_init2 /* rhino stuff without ; */
#endif
	  }
    }
    else{
      fprintf(stderr, "##################################\n");
      fprintf(stderr, "########### error ################\n");
      fprintf(stderr, "##################################\n");
      fprintf(stderr, "########################### %d %d %d   %d %d %d\n",
	      from_x, to_x, robot_specifications->global_map_dim_x,
	      from_y, to_y, robot_specifications->global_map_dim_y);
      fprintf(stderr, 
	      "\n@@@: %g   %g %d %d %d %d %d %d   %g %d %d %d %d %d %d\n",
	      robot_specifications->autoshift_distance,
	      robot_specifications->autoshifted_x,
	      shift_x, from_x, to_x, increment_x, 
	      robot_specifications->autoshifted_int_x,
	      robot_specifications->global_map_dim_x,
	      robot_specifications->autoshifted_y,
	      shift_y, from_y, to_y, increment_y, 
	      robot_specifications->autoshifted_int_y,
	      robot_specifications->global_map_dim_y);
      putc(7,stderr);
      for (index_from = 0; 
	   index_from < robot_specifications->global_map_dim_x
	   * robot_specifications->global_map_dim_y; index_from++){
	for (i = 0; i < NUM_GLOBAL_MAPS; i++){
	  global_map_x[i][index_from] = 0.5;
	  global_active_x[i][index_from] = 0;
	  global_label_x[i][index_from] = (unsigned char) 0;
	}
#ifdef RHINO_PLUS
        rhino_init2 /* rhino stuff without ; */
#endif
      }    
    }    

    /* ---------> CHANGE GLOBAL VARIABLES */
    robot_specifications->autoshifted_x += 	
      ((float) shift_x) * robot_specifications->resolution;
    robot_specifications->autoshifted_y += 
      ((float) shift_y) * robot_specifications->resolution;
    robot_specifications->autoshifted_int_x += shift_x;
    robot_specifications->autoshifted_int_y += shift_y;
    for (j = 0; j < NUM_GLOBAL_MAPS; j++){
      robot_specifications->min_display_index_x[j] += shift_x;
      robot_specifications->max_display_index_x[j] += shift_x;
      robot_specifications->min_display_index_y[j] += shift_y;
      robot_specifications->max_display_index_y[j] += shift_y;
      
      /* cut display_index */
      if (robot_specifications->min_display_index_x[j] < 0)
	robot_specifications->min_display_index_x[j] = 0;
      if (robot_specifications->min_display_index_y[j] < 0)
	robot_specifications->min_display_index_y[j] = 0;
      if (robot_specifications->max_display_index_x[j] > 
	  robot_specifications->global_map_dim_x)
	robot_specifications->max_display_index_x[j] = 
	  robot_specifications->global_map_dim_x;
      if (robot_specifications->max_display_index_y[j] >
	  robot_specifications->global_map_dim_y)
	robot_specifications->max_display_index_y[j] = 
	  robot_specifications->global_map_dim_y;
    }

    
    if (program_state->graphics_initialized){
      G_shift_robot_local_coordinates(GLOBAL_ROBOT, 
				      ((float) shift_x)
				      * robot_specifications->resolution,
				      ((float) shift_y)
				      * robot_specifications->resolution);
      G_shift_robot_local_coordinates(BEST_FIT_POINT, 
				      ((float) shift_x)
				      * robot_specifications->resolution,
				      ((float) shift_y)
				      * robot_specifications->resolution);
      G_shift_markers_local_coordinates(PATH,
					((float) shift_x)
					* robot_specifications->resolution,
					((float) shift_y)
					* robot_specifications->resolution);
      G_shift_markers_local_coordinates(TOPOLOGICAL_GRAPH,
					((float) shift_x)
					* robot_specifications->resolution,
					((float) shift_y)
					* robot_specifications->resolution);
      
    }

    /* ---------> REDISPLAY */

    if (program_state->graphics_initialized)
      G_display_all();
  }
}



/************************************************************************
 *
 *   NAME:         autoshift_display
 *                 
 *   FUNCTION:     Shifts robot display to always keep robot centered
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure 
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *                 float  pos_x       position point x
 *                 float  pos_y       position point y
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void autoshift_display(PROGRAM_STATE_PTR  program_state,
		       ROBOT_SPECIFICATIONS_PTR robot_specifications,
		       float pos_x, float pos_y)
{
  float robot_x, robot_y;
  int shift_x, shift_y;
  
  if (robot_specifications->autoshift == 1){
    shift_x = 0;
    shift_y = 0;
    robot_x = pos_x + robot_specifications->autoshifted_x;
    robot_y = pos_y + robot_specifications->autoshifted_y;

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
    
    
    autoshift_display_by(program_state, robot_specifications, 
			 shift_x, shift_y);
    
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
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


static int automatch_pos_x_defined = 0;

int mouse_test_loop(ROBOT_STATE_PTR          robot_state,
		    PROGRAM_STATE_PTR        program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  
  G_mouse_ptr mouse_events;
  int num_mouse_events, button;
  float mouse_x, mouse_y;
  int i, number;
  float min_value, max_value, fit;
  int test, error;
  float change_x, change_y, change_orientation;
  static int enter_automatch_point_mode = 0;
  static int enter_position_mode = 0;
  static float pos_x;
  static float pos_y;

  if (!program_state->graphics_initialized)
    return 0;


  /****************** CHECK FOR MOUSE EVENT *******************/
  
  test = G_test_mouse(0);

  if (test == 1){
    mouse_events = G_evaluate_mouse(&mouse_x, &mouse_y, 
				    &button, &num_mouse_events);

    if (enter_position_mode){
      if (G_mouse_event_at(GLOBAL_ROBOT, mouse_events, &number) &&
	  button == LEFT_BUTTON){
	float uncorr_robot_x;
	float uncorr_robot_y;
	float uncorr_robot_orientation;
	float angle, diff;
	
	angle = atan2(mouse_events[number].value_y -pos_y,
		      mouse_events[number].value_x -pos_x)
	  * 180.0 / M_PI;

	if (robot_state->map_orientation_defined){
	  diff = robot_state->map_orientation
	    - angle;
	  for (; diff >  45.0;) diff -= 90.0;
	  for (; diff < -45.0;) diff += 90.0;
	  angle += diff;
	}
	  
	compute_backward_correction(robot_state->x, robot_state->y, 
				    robot_state->orientation,
				    robot_state->correction_parameter_x,
				    robot_state->correction_parameter_y,
				    robot_state->correction_parameter_angle,
				    robot_state->correction_type,
				    &uncorr_robot_x, &uncorr_robot_y,  
				    &uncorr_robot_orientation);
	
	update_correction_parameters(robot_state->x,
				     robot_state->y,
				     robot_state->orientation,
				     pos_x - robot_state->x,
				     pos_y - robot_state->y,
				     angle - robot_state->orientation,
				     &(robot_state->correction_parameter_x),
				     &(robot_state->correction_parameter_y),
				     &(robot_state->correction_parameter_angle),
				     &(robot_state->correction_type));
	
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
	
	n_path_entries = 0;
	if (program_state->graphics_initialized){
	  G_clear_markers(PATH);
	  G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y,
			  robot_state->orientation, 0, NULL);
	}
	putc(7, stderr);
      }
      
      enter_position_mode = 0;
      
    }

    
    /****************** EVALUATE MOUSE EVENT,
     ****************** SPECIAL CASE: MARK A POINT *******************/
    
    else if (enter_automatch_point_mode){
      error = 0;
      enter_automatch_point_mode = 1;

      if (G_mouse_event_at(GLOBAL_ROBOT, mouse_events, &number) &&
	  button == LEFT_BUTTON){
	if (!automatch_pos_x_defined){
	  robot_state->automatch_pos_x = mouse_events[number].value_x;
	  robot_state->automatch_pos_y = mouse_events[number].value_y;
	  robot_state->automatch_pos_orientation = 0.0;
	  automatch_pos_x_defined = 1;
	}
	else{/*! Hack. Will allow to position robot */
	  robot_state->automatch_pos_orientation = 
	    atan2(mouse_events[number].value_y - robot_state->automatch_pos_y,
		  mouse_events[number].value_x - robot_state->automatch_pos_x)
	      * 180.0 / M_PI;
	  /*fprintf(stderr, "<< %g %g %g >>\n",
		  robot_state->automatch_pos_x, robot_state->automatch_pos_y,
		  robot_state->automatch_pos_orientation);*/
	}
	i = (((int)		/* index in the global map */
	      (robot_state->automatch_pos_x
	       / robot_specifications->resolution)
	      + robot_specifications->autoshifted_int_x)
	     * robot_specifications->global_map_dim_y)
	  + ((int) 
	     (robot_state->automatch_pos_y 
	      / robot_specifications->resolution)
	     + robot_specifications->autoshifted_int_y);
	if (!global_active[i])
	  error = 1;
	else
	  enter_automatch_point_mode = 2;
      }
      else if (G_mouse_event_at(GLOBAL_ROBOT, mouse_events, &number)){
	i = (((int)		/* index in the global map */
	      (robot_state->automatch_pos_x
	       / robot_specifications->resolution)
	      + robot_specifications->autoshifted_int_x)
	     * robot_specifications->global_map_dim_y)
	  + ((int) 
	     (robot_state->automatch_pos_y 
	      / robot_specifications->resolution)
	     + robot_specifications->autoshifted_int_y);
	if (global_active[i]){
	  G_display_switch(COMBINE_MAPS_BUTTON, 2);
	  robot_state->automatch_on = 1;
	  robot_state->automatch_cumul_distance = 0.0;
	  set_map(1);
	  clear_maps(1, robot_state, program_state, robot_specifications);
	  G_display_switch(DISPLAY_GLOBAL_MAPVALUES_BUTTON, 2);
	  if (program_state->regular_global_map_display){
	    G_deactivate(GLOBAL_BACKGROUND);
	    G_activate(GLOBAL_MAPVALUES);
	  }
	  else{
	    G_activate(GLOBAL_BACKGROUND);
	    G_deactivate(GLOBAL_MAPVALUES);
	  }
	  G_display_switch(GLOBAL_BACKGROUND, 0);
	  G_display_partial_matrix(GLOBAL_MAPVALUES,
				   robot_specifications->
				   min_display_index_x
				   [program_state->actual_map],
				   robot_specifications->
				   max_display_index_x
				   [program_state->actual_map] -
				   robot_specifications->
				   min_display_index_x
				   [program_state->actual_map],
				   robot_specifications->
				   min_display_index_y
				   [program_state->actual_map],
				   robot_specifications->
				   max_display_index_y
				   [program_state->actual_map] -
				   robot_specifications->
				   min_display_index_y
				   [program_state->actual_map]);
	  /*G_display_matrix(GLOBAL_MAPVALUES); old and inefficient! */
	  G_display_markers(PATH);
	  G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y,
			  robot_state->orientation, 0, NULL);
	  G_display_switch(DISPLAY_GLOBAL_MAPVALUES_BUTTON, 
			   program_state->regular_global_map_display);
	}
	else
	  error = 1;
      }
      else
	error = 1;

      if (error){
	G_display_switch(COMBINE_MAPS_BUTTON, 3); /* error */
	putc(7,stderr);
	usleep(600000);
	G_display_switch(COMBINE_MAPS_BUTTON, 0);
      }
      
      if (enter_automatch_point_mode != 2)
	enter_automatch_point_mode = 0;
    }
    
    /****************** EVALUATE MOUSE EVENT *******************/
    
    
    
    else if (button == RIGHT_BUTTON &&
	     G_mouse_event_at(GLOBAL_MAPVALUES, mouse_events, &number)){
      global_map[mouse_events[number].index_x
		 * robot_specifications->global_map_dim_y
		 + mouse_events[number].index_y] = 0.0;
      global_map[(mouse_events[number].index_x+1)
		 * robot_specifications->global_map_dim_y
		 + mouse_events[number].index_y] = 0.0;
      global_map[(mouse_events[number].index_x)
		 * robot_specifications->global_map_dim_y
		 + (mouse_events[number].index_y+1)] = 0.0;
      global_map[(mouse_events[number].index_x+1)
		 * robot_specifications->global_map_dim_y
		 + (mouse_events[number].index_y+1)] = 0.0;
      G_display_all();
    }


    else if (button == MIDDLE_BUTTON &&
	     G_mouse_event_at(GLOBAL_MAPVALUES, mouse_events, &number)){
      global_map[mouse_events[number].index_x
		 * robot_specifications->global_map_dim_y
		 + mouse_events[number].index_y] = 1.0;
      global_map[(mouse_events[number].index_x+1)
		 * robot_specifications->global_map_dim_y
		 + mouse_events[number].index_y] = 1.0;
      global_map[(mouse_events[number].index_x)
		 * robot_specifications->global_map_dim_y
		 + (mouse_events[number].index_y+1)] = 1.0;
      global_map[(mouse_events[number].index_x+1)
		 * robot_specifications->global_map_dim_y
		 + (mouse_events[number].index_y+1)] = 1.0;
      G_display_all();
    }

    else if (G_mouse_event_at(GLOBAL_ROBOT, mouse_events, &number) &&
	     button == LEFT_BUTTON){
      float uncorr_robot_x;
      float uncorr_robot_y;
      float uncorr_robot_orientation;

      enter_position_mode = 1;

      pos_x = mouse_events[number].value_x;
      pos_y = mouse_events[number].value_y;
    }

    /*
     *********** GLOBAL MAP DISPLAY (occ. values / Voronoi)
     */

    else if (G_mouse_event_at(DISPLAY_VORONOI_DIAGRAM_INCREMENT_BUTTON, 
			      mouse_events, &number)){
      program_state->actual_topology_option =
	(program_state->actual_topology_option + 1)
	  % NUM_VORONOI_MODI;
      G_display_switch(DISPLAY_VORONOI_DIAGRAM_BUTTON, 
		       program_state->actual_topology_option);
    }


    else if (G_mouse_event_at(DISPLAY_VORONOI_DIAGRAM_DECREMENT_BUTTON, 
			      mouse_events, &number)){
      program_state->actual_topology_option =
	(program_state->actual_topology_option + NUM_VORONOI_MODI - 1)
	  % NUM_VORONOI_MODI;
      G_display_switch(DISPLAY_VORONOI_DIAGRAM_BUTTON, 
		       program_state->actual_topology_option);
    }


    else if (G_mouse_event_at(DISPLAY_VORONOI_DIAGRAM_BUTTON, 
			      mouse_events, &number)){
      G_display_switch(DISPLAY_VORONOI_DIAGRAM_BUTTON, NUM_VORONOI_MODI);
      compute_voronoi_diagram(program_state->actual_topology_option,
			      robot_specifications, program_state, 
			      robot_state);
      G_activate(GLOBAL_VORONOI);
      G_display_partial_matrix(GLOBAL_VORONOI,
			       robot_specifications->
			       min_display_index_x
			       [program_state->actual_map],
			       robot_specifications->
			       max_display_index_x
			       [program_state->actual_map] -
			       robot_specifications->
			       min_display_index_x
			       [program_state->actual_map],
			       robot_specifications->
			       min_display_index_y
			       [program_state->actual_map],
			       robot_specifications->
			       max_display_index_y
			       [program_state->actual_map] -
			       robot_specifications->
			       min_display_index_y
			       [program_state->actual_map]);
      G_deactivate(GLOBAL_VORONOI);
      G_display_markers(TOPOLOGICAL_GRAPH);
      G_deactivate(TOPOLOGICAL_GRAPH);
      G_display_switch(DISPLAY_VORONOI_DIAGRAM_BUTTON, 
		       mouse_events[number].actual_text);
      
    }

    else if (G_mouse_event_at(DISPLAY_GLOBAL_MAPVALUES_BUTTON, 
			      mouse_events, &number)){

#ifdef RHINO_PLUS
      /*
       * The next few lines are a testamony of a funny programmer:
       */

      /*if (nc<6) nc++;*/ /* rhino stuff start */
      /*else if (nc==6) nc = 0;
	fprintf(stderr,"\n\nnumber:%d\n",nc);*/
      /*if (nc==3) {
	gmap_to_nmap(4 ,robot_specifications->global_map_dim_x ,robot_specifications->global_map_dim_y);
	tmp_map=new_map;
	}*/
      /*gmap_to_nmap(nc ,robot_specifications->global_map_dim_x ,robot_specifications->global_map_dim_y);*/
      /* rhino stuff end */
#endif


      program_state->regular_global_map_display ^= 1;
      G_display_switch(DISPLAY_GLOBAL_MAPVALUES_BUTTON, 2);
      if (program_state->regular_global_map_display){
	G_deactivate(GLOBAL_BACKGROUND);
	G_activate(GLOBAL_MAPVALUES);
      }
      else{
	G_activate(GLOBAL_BACKGROUND);
	G_deactivate(GLOBAL_MAPVALUES);
      }
      G_display_switch(GLOBAL_BACKGROUND, 0);
      G_display_partial_matrix(GLOBAL_MAPVALUES,
			       robot_specifications->
			       min_display_index_x
			       [program_state->actual_map],
			       robot_specifications->
			       max_display_index_x
			       [program_state->actual_map] -
			       robot_specifications->
			       min_display_index_x
			       [program_state->actual_map],
			       robot_specifications->
			       min_display_index_y
			       [program_state->actual_map],
			       robot_specifications->
			       max_display_index_y
			       [program_state->actual_map] -
			       robot_specifications->
			       min_display_index_y
			       [program_state->actual_map]);
      /*G_display_matrix(GLOBAL_MAPVALUES); old and inefficient! */
      G_display_markers(PATH);
      G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y,
		      robot_state->orientation, 0, NULL);
      G_display_switch(DISPLAY_GLOBAL_MAPVALUES_BUTTON, 
		       program_state->regular_global_map_display);

      /*
       * hack to prohibit this mode  - it is too slow
       */
      if (button != MIDDLE_BUTTON)
	program_state->regular_global_map_display = 0; /* prohibits this mode */
      if (program_state->regular_global_map_display){
	G_deactivate(GLOBAL_BACKGROUND);
	G_activate(GLOBAL_MAPVALUES);
      }
      else{
	G_activate(GLOBAL_BACKGROUND);
	G_deactivate(GLOBAL_MAPVALUES);
      }
      G_display_switch(DISPLAY_GLOBAL_MAPVALUES_BUTTON, 
		       program_state->regular_global_map_display);
    }

    else if (G_mouse_event_at(ACTUAL_MAP_BUTON, mouse_events, &number)){
      if (button == LEFT_BUTTON)
	/* one up! */
	set_map(program_state->actual_map + 1);
      else if (button == MIDDLE_BUTTON)
	/* one down! */
	set_map(program_state->actual_map - 1);
      else
	set_map(0);

      G_activate(GLOBAL_BACKGROUND);
      G_display_switch(DISPLAY_GLOBAL_MAPVALUES_BUTTON, 2);
      G_activate(GLOBAL_MAPVALUES);
      G_display_switch(GLOBAL_BACKGROUND, 0);
      G_display_partial_matrix(GLOBAL_MAPVALUES,
			       robot_specifications->
			       min_display_index_x
			       [program_state->actual_map],
			       robot_specifications->
			       max_display_index_x
			       [program_state->actual_map] -
			       robot_specifications->
			       min_display_index_x
			       [program_state->actual_map],
			       robot_specifications->
			       min_display_index_y
			       [program_state->actual_map],
			       robot_specifications->
			       max_display_index_y
			       [program_state->actual_map] -
			       robot_specifications->
			       min_display_index_y
			       [program_state->actual_map]);
      /*G_display_matrix(GLOBAL_MAPVALUES); old and inefficient! */
      G_display_markers(PATH);
      G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y,
		      robot_state->orientation, 0, NULL);
      G_display_switch(DISPLAY_GLOBAL_MAPVALUES_BUTTON, 
		       program_state->regular_global_map_display);
      if (program_state->regular_global_map_display){
	G_deactivate(GLOBAL_BACKGROUND);
	G_activate(GLOBAL_MAPVALUES);
      }
      else{
	G_activate(GLOBAL_BACKGROUND);
	G_deactivate(GLOBAL_MAPVALUES);
      }

    }

    else if (G_mouse_event_at(DISPLAY_LOCAL_MAPVALUES_BUTTON, 
			      mouse_events, &number)){
      program_state->regular_local_map_display ^= 1;
      G_display_switch(DISPLAY_LOCAL_MAPVALUES_BUTTON, 2);
      if (program_state->regular_local_map_display){
	G_deactivate(LOCAL_BACKGROUND);
	G_activate(LOCAL_MAPVALUES);
	G_change_matrix(LOCAL_MAPVALUES, local_map, local_active,
			robot_specifications->local_map_dim_x,
			robot_specifications->local_map_dim_y);
      }
      else{
	G_activate(LOCAL_BACKGROUND);
	G_deactivate(LOCAL_MAPVALUES);
      }
      G_display_switch(LOCAL_BACKGROUND, 0);
      G_display_matrix(LOCAL_MAPVALUES);
      G_display_switch(DISPLAY_LOCAL_MAPVALUES_BUTTON, 
		       program_state->regular_local_map_display);

      /*
       * hack to prohibit this mode  - it is too slow
       */
      program_state->regular_local_map_display = 0;
      G_activate(LOCAL_BACKGROUND);
      G_deactivate(LOCAL_MAPVALUES);
      G_display_switch(DISPLAY_LOCAL_MAPVALUES_BUTTON, 
		       program_state->regular_local_map_display);
    }


    else if (button == RIGHT_BUTTON &&
	     G_mouse_event_at(GLOBAL_ROBOT, mouse_events, &number)){
      G_display_switch(DISPLAY_GLOBAL_MAPVALUES_BUTTON, 2);
      G_activate(GLOBAL_MAPVALUES);
      G_display_partial_matrix(GLOBAL_MAPVALUES,
			       robot_specifications->
			       min_display_index_x
			       [program_state->actual_map],
			       robot_specifications->
			       max_display_index_x
			       [program_state->actual_map] -
			       robot_specifications->
			       min_display_index_x
			       [program_state->actual_map],
			       robot_specifications->
			       min_display_index_y
			       [program_state->actual_map],
			       robot_specifications->
			       max_display_index_y
			       [program_state->actual_map] -
			       robot_specifications->
			       min_display_index_y
			       [program_state->actual_map]);
      /*G_display_matrix(GLOBAL_MAPVALUES); old and inefficient! */
      G_display_markers(PATH);
      G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y,
		      robot_state->orientation, 0, NULL);
      if (!program_state->regular_global_map_display)
	G_deactivate(GLOBAL_MAPVALUES);
      G_display_switch(DISPLAY_GLOBAL_MAPVALUES_BUTTON, 
		       program_state->regular_global_map_display);
    }


    else if (button == RIGHT_BUTTON && 
	     (G_mouse_event_at(LOCAL_BACKGROUND, mouse_events, &number) ||
	      G_mouse_event_at(LOCAL_MAPVALUES, mouse_events, &number))){
      G_display_switch(DISPLAY_LOCAL_MAPVALUES_BUTTON, 2);
      G_activate(LOCAL_MAPVALUES);
      G_change_matrix(LOCAL_MAPVALUES, local_map, local_active,
		      robot_specifications->local_map_dim_x,
		      robot_specifications->local_map_dim_y);
      G_display_matrix(LOCAL_MAPVALUES);
      if (!program_state->regular_local_map_display)
	G_deactivate(LOCAL_MAPVALUES);
      G_display_switch(DISPLAY_LOCAL_MAPVALUES_BUTTON, 
		       program_state->regular_local_map_display);
    }

/*#ifdef old_stuff_dont_activate*/
    else if (G_mouse_event_at(GLOBAL_ROBOT, mouse_events, &number)){
      combine_global_maps(0, 1,
			  mouse_events[number].value_x,
			  mouse_events[number].value_y,
			  -999.9,/*!*/
			  button == MIDDLE_BUTTON,	/* combine! */
			  1,
			  &fit,
			  robot_specifications,
			  program_state,
			  robot_state);
    }
/*#endif*/

    else if (G_mouse_event_at(COMBINE_MAPS_BUTTON, mouse_events, &number)){
      if (robot_state->automatch_on){
	robot_state->automatch_on = 0;
	G_display_switch(COMBINE_MAPS_BUTTON, 0);
	enter_automatch_point_mode = 0;
      }
      else{
	G_display_switch(COMBINE_MAPS_BUTTON, 4);
	enter_automatch_point_mode = 1;
	automatch_pos_x_defined = 0;
      }
      
      if (program_state->actual_map != 0){
	set_map(0);
	G_display_switch(DISPLAY_GLOBAL_MAPVALUES_BUTTON, 2);
	G_activate(GLOBAL_MAPVALUES);
	G_display_partial_matrix(GLOBAL_MAPVALUES,
				 robot_specifications->
				 min_display_index_x
				 [program_state->actual_map],
				 robot_specifications->
				 max_display_index_x
				 [program_state->actual_map] -
				 robot_specifications->
				 min_display_index_x
				 [program_state->actual_map],
				 robot_specifications->
				 min_display_index_y
				 [program_state->actual_map],
				 robot_specifications->
				 max_display_index_y
				 [program_state->actual_map] -
				 robot_specifications->
				 min_display_index_y
				 [program_state->actual_map]);
	/*G_display_matrix(GLOBAL_MAPVALUES); old and inefficient! */
	G_display_markers(PATH);
	G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y,
			robot_state->orientation, 0, NULL);
	if (!program_state->regular_global_map_display)
	  G_deactivate(GLOBAL_MAPVALUES);
	G_display_switch(DISPLAY_GLOBAL_MAPVALUES_BUTTON, 
			 program_state->regular_global_map_display);
      }

      if (enter_automatch_point_mode)
	G_display_switch(COMBINE_MAPS_BUTTON, 1);
    }

    else if (G_mouse_event_at(QUIT_BUTTON, mouse_events, &number)){
      G_display_switch(QUIT_BUTTON, 1);
      usleep(200000);
      program_state->quit = 1;
    }


    else if (G_mouse_event_at(CLEAR_MAP_BUTTON, mouse_events, &number)){
      G_display_switch(CLEAR_MAP_BUTTON, 1);
      program_state->force_map_update = 1;
      clear_maps(-1, robot_state, program_state, robot_specifications);
      clear_path(robot_state, program_state, robot_specifications);
      compute_map_0(robot_specifications, program_state, robot_state);
      set_map(0);
      G_activate(GLOBAL_BACKGROUND);
      G_display_switch(DISPLAY_GLOBAL_MAPVALUES_BUTTON, 2);
      G_activate(GLOBAL_MAPVALUES);
      G_display_switch(GLOBAL_BACKGROUND, 0);
      G_display_partial_matrix(GLOBAL_MAPVALUES,
			       robot_specifications->
			       min_display_index_x
			       [program_state->actual_map],
			       robot_specifications->
			       max_display_index_x
			       [program_state->actual_map] -
			       robot_specifications->
			       min_display_index_x
			       [program_state->actual_map],
			       robot_specifications->
			       min_display_index_y
			       [program_state->actual_map],
			       robot_specifications->
			       max_display_index_y
			       [program_state->actual_map] -
			       robot_specifications->
			       min_display_index_y
			       [program_state->actual_map]);
      G_display_markers(PATH);
      G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y,
		      robot_state->orientation, 0, NULL);
      G_display_switch(DISPLAY_GLOBAL_MAPVALUES_BUTTON, 
		       program_state->regular_global_map_display);
      if (program_state->regular_global_map_display){
	G_deactivate(GLOBAL_BACKGROUND);
	G_activate(GLOBAL_MAPVALUES);
      }
      else{
	G_activate(GLOBAL_BACKGROUND);
	G_deactivate(GLOBAL_MAPVALUES);
      }

      G_display_switch(CLEAR_MAP_BUTTON, 0);
    }



    else if (G_mouse_event_at(MAP_UPDATE_ENABLE_BUTTON, mouse_events,
			      &number)){
      program_state->map_update_on ^= 1;
      G_display_switch(MAP_UPDATE_ENABLE_BUTTON, program_state->map_update_on);
    }


    else if (G_mouse_event_at(LOAD_BUTTON, mouse_events, &number)){
      G_display_switch(LOAD_BUTTON, 1);
      load_parameters(MAP_NAME, 0);
      send_complete_map(robot_specifications, program_state, NULL);
      G_display_switch(LOAD_BUTTON, 0);
    }

    else if (G_mouse_event_at(SAVE_BUTTON, mouse_events, &number)){
      G_display_switch(SAVE_BUTTON, 1);
      save_parameters(MAP_NAME);
      G_display_switch(SAVE_BUTTON, 0);
    }

    else if (G_mouse_event_at(DUMP_BUTTON, mouse_events, &number)){
      G_display_switch(DUMP_BUTTON, 1);
      MAP_dump_handler(NULL, NULL);
      G_display_switch(DUMP_BUTTON, 0);
    }

    else if (G_mouse_event_at(FREEZE_BUTTON, mouse_events, &number)){
      G_display_switch(FREEZE_BUTTON, 1);
      prepare_2nd_run(robot_specifications, program_state);
      send_complete_map(robot_specifications, program_state, NULL);
      G_display_switch(FREEZE_BUTTON, 0);
    }



    else if (button == MIDDLE_BUTTON &&
	     G_mouse_event_at(LINE_ANGLE_BUTTON, mouse_events, &number)){
      robot_state->map_orientation_defined ^= 1;
      G_display_switch(LINE_ANGLE_BUTTON, 
		       robot_state->map_orientation_defined);
    }





#ifdef must_this_be
    else if (G_mouse_event_at(DISPLAY_GLOBAL_CORRELATIONS_BUTTON,
			 mouse_events, &number)){
      G_display_switch(DISPLAY_GLOBAL_CORRELATIONS_BUTTON, 1);
      G_deactivate(GLOBAL_MAPVALUES);
      G_deactivate(GLOBAL_BACKGROUND);
      G_activate(GLOBAL_CORRELATIONS);
      compute_correlation_matrix(robot_state->orientation);
      G_display_matrix(GLOBAL_CORRELATIONS);
      G_display_markers(PATH);
      G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y,
		      robot_state->orientation, 0, NULL);
      if (program_state->regular_global_map_display)
	G_activate(GLOBAL_MAPVALUES);
      else
	G_activate(GLOBAL_BACKGROUND);
      G_display_switch(DISPLAY_GLOBAL_CORRELATIONS_BUTTON, 0);
    }
#endif

    /* insert new graphics object here XXXXX */
    
    
    else if (button == RIGHT_BUTTON){
      /*
	 clear_maps(-2, robot_state, program_state, robot_specifications);
	 compute_map_0(robot_specifications, program_state, robot_state);
	 */
      G_display_all();
    }


  }
  
  return test;
}


