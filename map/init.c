
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
#include "EZX11.h"
#include "o-graphics.h"
#include "MAP-messages.h"
#include "SONAR-messages.h"
#include "BASE-messages.h"
#include "MAP.h"

#ifdef RHINO_PLUS
#include "rst.h" /* rhino stuff */
#endif

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

char init_file_name[256];
extern int read_map_dat;

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

  program_state->use_graphics             = 1; /* default */
  program_state->use_tcx                  = 1; /* default */
  robot_specifications->broadcasts_on     = 1; /* default */

  strcpy(init_file_name, MAP_INIT_NAME);


  for (i = 1; i < argc && !bug; i++){
	if (!strcmp(argv[i], "-nodisplay") || !strcmp(argv[i], "-nd"))
      program_state->use_graphics = 0;
    else if (!strcmp(argv[i], "-notcx") || !strcmp(argv[i], "-nt"))
      program_state->use_tcx = 0;
    else if (!strcmp(argv[i], "-nobroadcasts") || !strcmp(argv[i], "-nb")){
       robot_specifications->broadcasts_on = 0;
       fprintf(stderr, "ALL BROADCASTS ARE SWITCHED OFF.\n");
    }
    else if (!strcmp(argv[i], "-file") || !strcmp(argv[i], "-file")){
	  read_map_dat = 1;
	  if (i < argc - 1){
	strcpy(init_file_name, argv[++i]);
	if (init_file_name[0] == '-')
	bug = 2;
      }
      else
	bug = 2;
    }
    else
      bug = 1;
  }
  
  if (bug == 1)
    fprintf(stderr, "\nUsage: '%s [-nodisplay] [-notcx] [-nobroadcast] [-file <filename>]\n", 
	    argv[0]);
  else if (bug == 2)
    fprintf(stderr, "\nError: Option -file must be followed by file name.\n");


  if (bug >= 1)
    exit(1);


}



/************************************************************************
 *
 *   NAME:         init_program
 *                 
 *   FUNCTION:     Initializes the structure "program_state" and "robot_state"
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure 
 *                 ROBOT_STATE_PTR  robot_state      Pointer  to general
 *                                                   robot state description
 *                                                   structure 
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *                 ALL_PTR            structure to all variables
 *                 
 *                 
 *                 NOTE: "NULL" disables initialization
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void init_program(PROGRAM_STATE_PTR        program_state,
		  ROBOT_STATE_PTR          robot_state,
		  ROBOT_SPECIFICATIONS_PTR robot_specifications,
		  ALL_PTR                  all)
{
  int i;
  if (program_state != NULL){
    program_state->tcx_initialized                         = 0;
    program_state->graphics_initialized                    = 0;
    /* program_state->use_graphics           = 1; command line option */
    /* program_state->use_tcx                = 1; command line option */
    program_state->tcx_base_connected                      = 0;
    program_state->tcx_localize_connected                  = 0;
    program_state->regular_global_map_display              = 0;
    program_state->regular_local_map_display               = 0;
    program_state->maps_allocated                          = 0;
    program_state->quit                                    = 0;
    program_state->map_update_pending                      = 0;
    program_state->actual_map                              = 0;
    program_state->global_map_matching_mode                = 0;
    program_state->map_update_on                           = 1;
    program_state->force_map_update                        = 0;
    program_state->actual_topology_option                  = 0;
    program_state->first_base_report_received              = 0;
  }


  if (robot_state != NULL){
    robot_state->x                                         = 0.0;
    robot_state->y                                         = 0.0;
    robot_state->orientation                               = 0.0;
    robot_state->translational_speed                       = 0.0;
    robot_state->rotational_speed                          = 0.0;
    robot_state->known                                     = 0;
    robot_state->sensor_x                                  = 0.0;
    robot_state->sensor_y                                  = 0.0;
    robot_state->sensor_orientation                        = 0.0;
    robot_state->sensor_translational_speed                = 0.0;
    robot_state->sensor_rotational_speed                   = 0.0;
    robot_state->sensor_org_x                              = 0.0;
    robot_state->sensor_org_y                              = 0.0;
    robot_state->sensor_org_orientation                    = 0.0;
    robot_state->sensor_best_x                             = 0.0;
    robot_state->sensor_best_y                             = 0.0;
    robot_state->sensor_best_orientation                   = 0.0;
    robot_state->sensor_best_fit                           = 1.0;
    robot_state->sensor_values                             = NULL;

    robot_state->last_change_x                             = 0.0;
    robot_state->last_change_y                             = 0.0;
    robot_state->last_change_orientation                   = 0.0;


    robot_state->correction_parameter_x                    = 0.0;
    robot_state->correction_parameter_y                    = 0.0;
    robot_state->correction_parameter_angle                = 0.0;
    robot_state->correction_type                           = 0;

    robot_state->map_orientation_defined              = 0;
    robot_state->map_orientation                      = 0.0;
      
    robot_state->sensor_uncertainty                        = 0.001;

    robot_state->automatch_on                              = 0;
    robot_state->automatch_pos_x                           = 0.0;
    robot_state->automatch_pos_y                           = 0.0;
    robot_state->automatch_pos_orientation                 = 0.0;
    robot_state->automatch_cumul_distance                  = 0.0;
  }


  if (robot_specifications != NULL){
    robot_specifications->global_mapsize_x                 = 3000.0;
    robot_specifications->global_mapsize_y                 = 3000.0;
    robot_specifications->global_map_dim_x                 = 300;
    robot_specifications->global_map_dim_y                 = 300;
    robot_specifications->local_map_dim_x                  = 2;
    robot_specifications->local_map_dim_y                  = 3;
    robot_specifications->resolution                       = 10.0;

    robot_specifications->local_map_origin_x               = 0.0;
    robot_specifications->local_map_origin_y               = 0.0;
    robot_specifications->local_map_origin_orientation     = 0.0;
    robot_specifications->smooth_radius                    = 3;

    robot_specifications->robot_size                       = 30.0;
    robot_specifications->drift                            = 0.0;
  
    robot_specifications->autoshift                        = 1;
    robot_specifications->autoshift_safety_margin          = 400.0;
    robot_specifications->autoshift_distance               = 1000.0;
    robot_specifications->autoshifted_x                    = 0.0;
    robot_specifications->autoshifted_y                    = 0.0;
    robot_specifications->autoshifted_int_x                = 0;
    robot_specifications->autoshifted_int_y                = 0;
  

    robot_specifications->do_position_correction           = 1;
    robot_specifications->max_distance_in_match            = 100.0;
    robot_specifications->map_fit_norm_L2                  = 1;
    robot_specifications->prev_pos_norm_L2                 = 1;
    robot_specifications->max_niterations_in_search        = 200;
    robot_specifications->niterations_in_map_fitting       = 50;
    robot_specifications->niterations_in_search            = 0;
    robot_specifications->max_translation_in_search        = 100.0;
    robot_specifications->translation_weight_fit           = 10000.0;
    robot_specifications->translation_weight_fit_global_match = 10000.0;
    robot_specifications->translation_weight_prev_position = 1.0;
    robot_specifications->translation_stepsize             = 0.01;
    robot_specifications->translation_momentum             = 0.8;
    robot_specifications->max_rotation_in_search           = 100.0;
    robot_specifications->rotation_weight_fit              = 2000.0;
    robot_specifications->rotation_weight_fit_global_match = 2000.0;
    robot_specifications->rotation_weight_prev_position    = 1.0;
    robot_specifications->rotation_stepsize                = 0.03;
    robot_specifications->rotation_momentum                = 0.7;
    robot_specifications->search_granularity               = 2;

    robot_specifications->do_path_fitting                  = 1;
    robot_specifications->weight_path_fit                  = 1.0;
    robot_specifications->n_path_points_in_fit             = 100;

    robot_specifications->wall_error_threshold             = 10.0;
    robot_specifications->wall_weight                      = 0.1;
    robot_specifications->number_subsequent_adjacent_walls = 5;
    robot_specifications->min_advance_for_map_fitting      = 400.0;

    robot_specifications->decay_old                        = 0.99;
    robot_specifications->decay_new                        = 0.5;
    robot_specifications->prior                            = 0.5;
    robot_specifications->update_extreme_likelihoods       = 0;

    robot_specifications->lower_clipping_value             = 0.05;
    robot_specifications->upper_clipping_value             = 0.95;

    robot_specifications->reposition_robot_initially       = 1;
    robot_specifications->regular_gif_output_in_sec        = -1;
    /* robot_specifications->broadcasts_on                 = 1; */
    robot_specifications->X_window_size                    = 70.0;
    robot_specifications->data_logging                     = 0;

    for (i = 0; i < NUM_GLOBAL_MAPS; i++){
      robot_specifications->min_display_index_x[i]  = 
	robot_specifications->global_map_dim_x + 1;
      robot_specifications->max_display_index_x[i]  =  -1;
      robot_specifications->min_display_index_y[i]  = 
	robot_specifications->global_map_dim_x + 1;
      robot_specifications->max_display_index_y[i]  =  -1;
    }
  }


  if (all != NULL){
    all->program_state        = program_state;
    all->robot_state          = robot_state;
    all->robot_specifications = robot_specifications;
  }

  n_path_entries = 0;
}


/************************************************************************
 *
 *   NAME:         allocate_everything
 *                 
 *   FUNCTION:     allocates memory for internal maps
 *                 
 *   PARAMETERS:   ROBOT_STATE_PTR    robot_state    Pointer to robot state
 *                                                   description structure
 *                 PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure 
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void allocate_everything(ROBOT_STATE_PTR    robot_state,
			 PROGRAM_STATE_PTR  program_state,
			 ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int i, j, global_size, local_size, out_of_mem;


  if (!program_state->maps_allocated){
    
    
    /* IF MAPS ALREADY ALLOCATED - FREE THE MEMORY (currently commented out)
     * if (program_state->maps_allocated){
     *   free(global_map);
     *   free(global_active);
     *   free(local_map);
     *   free(local_active);
     *   free(global_map_ext); 
     *   program_state->maps_allocated = 0;
     * } 
     */
#ifdef RHINO_PLUS
    /*rhino_free*/ /* rhino stuff without ; */
#endif
    
    robot_specifications->global_map_dim_x = 
      (int) (robot_specifications->global_mapsize_x
	     / robot_specifications->resolution);
    robot_specifications->global_map_dim_y = 
      (int) (robot_specifications->global_mapsize_y
	     / robot_specifications->resolution);
    
    /* ALLOCATE NEW MEMORY FOR THE MAPS */
    
    global_size = robot_specifications->global_map_dim_x
      * robot_specifications->global_map_dim_y;

    local_size = robot_specifications->local_map_dim_x
      * robot_specifications->local_map_dim_y;

    if (!program_state->maps_allocated){
      out_of_mem = 0;
      for (i = 0; i < NUM_GLOBAL_MAPS; i++){
	global_map_x[i]     = (float *) (malloc(sizeof(float) * global_size));
	global_active_x[i]  = (int *)   (malloc(sizeof(int)   * global_size));
	global_label_x[i]   = (unsigned char *)
	  (malloc(sizeof(unsigned char)   * global_size));
	if (global_map_x[i] == NULL || global_active_x[i] == NULL
	    || global_label_x[i] == NULL)
	  out_of_mem = 1;
      }
      global_map    = global_map_x[0];
      global_active = global_active_x[0];
      global_label  = global_label_x[0];

      global_local_correlations  = (float *) (malloc(sizeof(float)
						     * global_size));
      global_voronoi  = (float *) (malloc(sizeof(float) * global_size));
      global_voronoi_active  = (int *) (malloc(sizeof(int) * global_size));

      local_map     = (float *) (malloc(sizeof(float) * local_size));
      local_smooth_map  = (float *) (malloc(sizeof(float) * local_size));
      local_active  = (int *)   (malloc(sizeof(int)   * local_size));
      
      if (out_of_mem ||
	  global_map                 == NULL || 
	  global_active              == NULL ||
	  global_label               == NULL ||
	  local_map                  == NULL ||
	  local_active               == NULL ||
	  local_smooth_map           == NULL ||
	  global_local_correlations  == NULL){

	printf("ABORT: out of memory!\n");
	exit(1);
      }
      program_state->maps_allocated = 1;
    }
    
    /* INITIALIZE MAPS */
    
    for (i = 0; i < global_size; i++){
      for (j = 0; j < NUM_GLOBAL_MAPS; j++){
	global_map_x[j][i]     = robot_specifications->prior;
	global_active_x[j][i]  = 0;
	global_label_x[j][i]   = (unsigned char) 0;
      }
      global_local_correlations[i] = 0.0;
    }

    for (i = 0; i < local_size; i++){
      local_map[i]                = 0.8;
      local_smooth_map[i]         = 0.8;
      local_active[i]             = 0;
    }
    

    frozen = (int *) malloc(sizeof(int) * 4);
    frozen[0] = 0;
    frozen[1] = 1;
    frozen[2] = 0;
    frozen[3] = 0;		/* see MAP.c for a definition of frozen */
    

  }
  else 
    printf("ERROR: Maps were already allocated. Ignored.\n");
}





/************************************************************************
 *
 *   NAME:         read_init_file()
 *                 
 *   FUNCTION:     reads the initialization file. If not found,
 *                 the default values are kept
 *                 
 *   PARAMETERS:   ROBOT_STATE_PTR    robot_state    Pointer to robot state
 *                                                   description structure
 *                 PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure 
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void read_init_file(ROBOT_STATE_PTR    robot_state,
		    PROGRAM_STATE_PTR  program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  if (!load_parameters(init_file_name, 1)){
    putc(7,stderr);
    usleep(800000);
    putc(7,stderr);
    fprintf(stderr, "WARNING: Initialization file %s not found.",
	    init_file_name);
    fprintf(stderr, " Take default values.\n");
  }
}





/************************************************************************
 *
 *   NAME:         clear_all_maps()
 *                 
 *   FUNCTION:     Clears all the maps, and also clears path-memory
 *                 
 *   PARAMETERS:   ROBOT_STATE_PTR    robot_state    Pointer to robot state
 *                                                   description structure
 *                 PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure 
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void clear_maps(int number,	/* map number - -1 for all maps,
				 * -2 for all but the CAD map */
		ROBOT_STATE_PTR    robot_state,
		PROGRAM_STATE_PTR  program_state,
		ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  register int i, x, y, index, from_i, num_i;
  
  if (number == -1){
    from_i = 1;
    num_i  = NUM_GLOBAL_MAPS - 1;
  }
  else if (number == -2){
    from_i = 1;
    num_i  = NUM_GLOBAL_MAPS - 2;
  }
  else{
    from_i = number;
    num_i  = 1;
  }
  if (from_i < 0 || from_i + num_i > NUM_GLOBAL_MAPS)
    fprintf(stderr, "ERROR in bounds: %d %d\n", from_i, num_i);
  for (i = from_i; i < from_i + num_i; i++){
    for (x = 0; x < robot_specifications->global_map_dim_x; x++)
      for (y = 0; y < robot_specifications->global_map_dim_y; y++){
	index = x * robot_specifications->global_map_dim_y + y;
	if (global_map_x[i] != NULL)
	  global_map_x[i][index] = 0.5;
	if (global_active_x[i] != NULL)
	  global_active_x[i][index] = 0;
	if (global_label_x[i] != NULL)
	  global_label_x[i][index] = (unsigned char) 0;
	if (global_local_correlations != NULL)
	  global_local_correlations[index] = 0.0;
      }
    robot_specifications->min_display_index_x[i]  = 
      robot_specifications->global_map_dim_x + 1;
    robot_specifications->max_display_index_x[i]  =  -1;
    robot_specifications->min_display_index_y[i]  = 
      robot_specifications->global_map_dim_x + 1;
    robot_specifications->max_display_index_y[i]  =  -1;
  }

  program_state->force_map_update = 1;

  /* compute_map_0(robot_specifications, program_state, robot_state); */

  
  /*
     for (x = 0; x < robot_specifications->local_map_dim_x; x++)
     for (y = 0; y < robot_specifications->local_map_dim_y; y++){
     index = x * robot_specifications->local_map_dim_y + y;
     if (local_map != NULL)
     local_map[index] = 0.5;
     if (local_smooth_map != NULL)
     local_smooth_map[index] = 0.5;
     if (local_active != NULL)
     local_active[index] = 0;
     }
     */

}  
void clear_path(ROBOT_STATE_PTR    robot_state,
		PROGRAM_STATE_PTR  program_state,
		ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  n_path_entries = 0;
  if (program_state->graphics_initialized)
    G_clear_markers(PATH);
}


