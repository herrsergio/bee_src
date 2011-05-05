
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
#include <stdlib.h>
#include <sys/time.h>
#include <signal.h>

#define TCX_define_variables /* this makes sure variables are installed */
#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */

#include "all.h"

#include "beeSoftVersion.h"
#include "libbUtils.h"
#include "librobot.h"
#include "libezx.h"





/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

float *dummy_sensors;
float *null_sensors;
static struct timeval last_attempt_tcx_connect = {0, 0};

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



TCX_MODULE_PTR BASESERVER = NULL;
TCX_MODULE_PTR SONARINT = NULL;
TCX_MODULE_PTR LASERINT = NULL;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




/************************************************************************
 *
 *   NAME:         check_commandline_parameters
 *                 
 *   FUNCTION:     browses the command-line parameters for useful info.
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void check_commandline_parameters(int argc, char **argv, ALL_PARAMS)
{
  int i;
  int unknown_parameter = 0;

  program_state->do_Xdisplay              = 1; /* default */
  program_state->tcx_autoconnect          = 1; /* default */


  for (i = 1; i < argc && !unknown_parameter; i++){
    if (!strcmp(argv[i], "-notcx") || !strcmp(argv[i], "-nt"))
      program_state->tcx_autoconnect = 0;
    /* else if (!strcmp(argv[i], "-nodisplay") || !strcmp(argv[i], "-nd"))
       program_state->do_Xdisplay = 0; */
    else{
      /*      printf("I don't understand '%s'.\n", argv[i]); */
      unknown_parameter = 1;
    }
  }
 
  if (unknown_parameter){
    fprintf(stderr, "\nUsage: '%s [-notcx]\n", argv[0]);
    fprintf(stderr, "where:  -notcx      =   no TCX\n");
    /*fprintf(stderr, "        -nodisplay  =   no X-Windows display\n");*/
    fflush(stderr);
    exit(-1);
  }
}

/************************************************************************
 *
 *   NAME:         init_program
 *                 
 *   FUNCTION:     Initializes the three structures "robot_state",
 *                 "action_ptr", and "program_state"
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 NOTE: "NULL" disables initialization
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void init_program(ALL_PARAMS)
{
  int i;
  float help;
  
  /* ============== PROGRAM_STATE ==================== */
  
  if (program_state != NULL && !program_state->is_initialized){
    program_state->is_initialized             = 1;
    program_state->graphics_initialized       = 0;

    program_state->tcx_initialized            = 0;
    program_state->do_connect_to_BASE         = 0;
    program_state->do_connect_to_PANTILT      = 0;
    program_state->do_connect_to_MAP          = 0;
    program_state->do_connect_to_PLAN         = 0;
    program_state->do_connect_to_TRACKER      = 0;
    program_state->do_connect_to_CD           = 0;
    program_state->do_connect_to_BUTTONS      = 0;
    program_state->do_connect_to_FLOW         = 0;
    program_state->do_connect_to_BASESERVER   = 0;
    program_state->do_connect_to_SIMULATOR    = 0;
    program_state->do_connect_to_SONARINT     = 0;
    program_state->do_connect_to_LASERINT     = 0;

    program_state->tcx_connected_to_BASE      = 0;
    program_state->tcx_connected_to_PANTILT   = 0;
    program_state->tcx_connected_to_MAP       = 0;
    program_state->tcx_connected_to_PLAN      = 0;
    program_state->tcx_connected_to_TRACKER   = 0;
    program_state->tcx_connected_to_CD        = 0;
    program_state->tcx_connected_to_BUTTONS   = 0;
    program_state->tcx_connected_to_FLOW      = 0;
    program_state->tcx_connected_to_BASESERVER= 0;
    program_state->tcx_connected_to_SIMULATOR = 0;
    program_state->tcx_connected_to_SONARINT  = 0;
    program_state->tcx_connected_to_LASERINT  = 0;

    program_state->quit                       = 0;
    program_state->something_happened         = 0;
    program_state->script_on                  = 0;
    program_state->continuous_script_mode     = 0;
    program_state->script_number_episodes     = 0;
    program_state->script_nevents_per_episode = NULL;
    program_state->current_episode_displayed  = 0;
    program_state->current_event_displayed    = 0;
    program_state->display_model_on           = 0;
    program_state->active_network             = 0;
    program_state->continuous_mode            = 0;
    program_state->random_continuous_mode     = 0;

    program_state->total_area_covered         = 0.0;
    program_state->d_total_area_covered       = 0.0;
    program_state->total_distance_traveled    = 0.0;
    program_state->advance_rate               = 0.0;


    program_state->autonomous_mode            = 0;
    program_state->robot_pos_x_when_going_autonomous = 0.0;
    program_state->robot_pos_y_when_going_autonomous = 0.0;
    program_state->auto_reset_expl_mode       = 0;

    program_state->tcx_connected_to_SPEECH    = 0;
    program_state->do_connect_to_CAMERA       = 0;
    program_state->tcx_connected_to_CAMERA    = 0;
    program_state->camera_continuous_grabbing = 0;
    program_state->control_window_mode        = 0;
    program_state->pantilt_window_mode        = 1;
    program_state->map_regular_update_on      = 1;
#ifdef UNIBONN
    program_state->do_connect_to_SPEECH       = 0;
    program_state->do_connect_to_ARM          = 0;
    program_state->tcx_connected_to_ARM       = 0;
    program_state->do_connect_to_SUNVIS       = 0;
    program_state->tcx_connected_to_SUNVIS    = 0;
    program_state->approaching_mode           = 0;
    program_state->active_goal_number         = 0;
    program_state->n_carried_objetcs          = 0;
    program_state->hunting_mode               = 0;
    program_state->looking_for_trash_bin_mode = 0;

#endif /* UNIBONN */

#ifdef CD_VERSION
    program_state->last_cd_phrase             = -1;
#endif /* CD_VERSION */

#ifdef TOURGUIDE_VERSION
    program_state->learning_tour              = 0; /* 1, to indicate tour
						    * learning mode */
    program_state->giving_tour                = 0; /* 1, to indicate that 
						    * robot is willing
						    * togive a tour */
    program_state->tour_modus                 = 0; /* 0 = no tour
						    * 1 = robot waiting 
						    * 2 = planner
						    * 3 = local obst-avoid */
    program_state->num_tour_goals             = 0;
    program_state->current_tour_goal          = 0;

    program_state->current_tour               = -1;

    for (i = 0; i < MAX_NUM_TOURS; i++)
      program_state->tour_defined[i]          = 0;
#endif /* TOURGUIDE_VERSION */


  }
  
  
  /* ============== ROBOT_SPECIFICATIONS ==================== */
  
  if (robot_specifications != NULL && !robot_specifications->is_initialized){
    robot_specifications->is_initialized       = 1;

    robot_specifications->robot_size           = 30.0; /* in cm */
    robot_specifications->global_worldsize_x   = 4000.0; /* in cm */
    robot_specifications->global_worldsize_y   = 4000.0; /* in cm */


    robot_specifications->map_resolution       = 10.0; /* in cm,
							* must be divisor of
							* worldsize !!! */

    robot_specifications->refresh_interval        = 1;
    robot_specifications->dist_action_achieved = 150.0;

    robot_specifications->translational_speed  = 0.0; /* in cm/sec */
    robot_specifications->rotational_speed     = 0.0; /* in degree(?)/sec */

    robot_specifications->max_security_dist    = 100.0;

    robot_specifications->sonar_defined        = 1;


    robot_specifications->num_sonar_sensors    = bRobot.sonar_cols[0];
    if (bRobot.sonar_cols[0] == 24){
      robot_specifications->first_sonar_angle    = 352.5;
      robot_specifications->last_sonar_angle     = 7.5;
    }
    else if (bRobot.sonar_cols[0] == 16){
      robot_specifications->first_sonar_angle    = 360.0-11.25;
      robot_specifications->last_sonar_angle     = 11.25;
    }
    else{
      fprintf(stderr, "Current software is not configured for %d sonar sensors.\n", bRobot.sonar_cols[0]);
      exit(-1);
    }
    robot_specifications->min_sonar_range      = 1.0; /* in cm */
    robot_specifications->max_sonar_range      = 400.0; /* in cm */

    robot_specifications->num_laser_sensors    = 360;
    robot_specifications->first_laser_angle    = -90.0;
    robot_specifications->last_laser_angle     = 270.0;
    robot_specifications->min_laser_range      = 1.0; /* in cm */
    robot_specifications->max_laser_range      = 400.0; /* in cm */
    

    robot_specifications->autoshift            = 1;
    robot_specifications->safety_margin        = 400.0;
    robot_specifications->autoshift_distance   = 700.0;
    robot_specifications->autoshifted_x        = 0.0;
    robot_specifications->autoshifted_y        = 0.0;
    robot_specifications->autoshifted_int_x    = 0;
    robot_specifications->autoshifted_int_y    = 0;
    robot_specifications->statistics_time_decay_rate = 0.8;


    robot_specifications->min_dist_between_objects = 200.0;

    robot_specifications->camera_1_opening_angle = 42.28;
    robot_specifications->camera_1_min_perceptual_dist = 70.0;
    robot_specifications->camera_1_max_perceptual_dist = 300.0;

    robot_specifications->camera_2_opening_angle = 71.7;
    robot_specifications->camera_2_min_perceptual_dist = 40.0;
    robot_specifications->camera_2_max_perceptual_dist = 300.0;

    robot_specifications->X_window_size       = 65.0;

#ifdef UNIBONN
    robot_specifications->max_num_allowed_failures_in_object_recognition = 3;
    robot_specifications->approach_object_distance = 30.0;
    robot_specifications->set_points_when_searching_objects = 1;
    robot_specifications->set_points_when_searching_bins    = 1;
#endif /* UNIBONN */


    robot_specifications->base_status_update_rate = 4;


    strcpy(robot_specifications->log_file_name, "data/learn.logg");
    robot_specifications->log_file_open        = 0;
    if (program_state != NULL)
      robot_specifications->do_log             = 
	program_state->do_connect_to_BASE;
    else
      robot_specifications->do_log             = 0;
    robot_specifications->log_iop              = NULL;
    robot_specifications->log_file_unopenable  = 0;

    strcpy(robot_specifications->script_file_name, "data/learn.script");
    strcpy(robot_specifications->doc_file_name,    "data/learn.doc");


    robot_specifications->arm_move_out_in_force = 20.0;
    robot_specifications->display_update_interval  = 0.0;

    robot_specifications->map_update_interval       = 5.0;
    robot_specifications->sensor_update_interval    = 0.6;
  }
  
  



  
  /* ============== ROBOT_STATE ==================== */
  
  if (robot_state != NULL && !robot_state->is_initialized){
    robot_state->is_initialized            = 1;

    robot_state->x                         = 0.0;
    robot_state->y                         = 0.0;
    robot_state->orientation               = 90.0;
    robot_state->trans_set_velocity        = INITIAL_TRANS_VELOCITY;
    robot_state->rot_set_velocity          = INITIAL_ROT_VELOCITY;
    robot_state->trans_velocity            = 0.0;
    robot_state->rot_velocity              = 0.0;
    robot_state->trans_set_acceleration    = INITIAL_TRANS_ACCELERATION;
    robot_state->rot_set_acceleration      = INITIAL_ROT_ACCELERATION;
    robot_state->trans_direction           = 0;
    robot_state->rot_direction             = 0;
    robot_state->in_motion                 = 0;
    robot_state->state_is_known            = 0;
    robot_state->sonar_on                  = 0;
    robot_state->base_mode                 = DEFAULT_MODE;

    robot_state->arm_moved_out             = 0;
    robot_state->pick_sequence             = 0;
    robot_state->arm_gripper_closed        = 0;

    robot_state->correction_parameter_x     = 0.0;
    robot_state->correction_parameter_y     = 0.0;
    robot_state->correction_parameter_angle = 0.0;/*!*/
    robot_state->correction_type            = 0;

    robot_state->red_light_status         = 0;
    robot_state->yellow_light_status      = 0;
    robot_state->green_light_status       = 0;
    robot_state->blue_light_status        = 0;

    robot_state->battery_level            = 0.3;
  }



  /* ============== SENSATION ==================== */
  
  if (sensation != NULL && !sensation->is_initialized){
    sensation->is_initialized           = 1;
    sensation->real_world_data          = 0;
    sensation->new_sonar                = 0;
    sensation->new_laser                = 0;
    sensation->tilt                     = INITIAL_TILT;
    sensation->pan                      = INITIAL_PAN;
    sensation->last_update_time.tv_sec  = 0;	/* /usr/include/sys/time.h */
    sensation->last_update_time.tv_usec = 0;	/* /usr/include/sys/time.h */

    sensation->red_button_status        = 0;
    sensation->yellow_button_status     = 0;
    sensation->green_button_status      = 0;
    sensation->blue_button_status       = 0;
    sensation->red_button_changed       = 0;
    sensation->yellow_button_changed    = 0;
    sensation->green_button_changed     = 0;
    sensation->blue_button_changed      = 0;

    sensation->flow_result              = 0;

    sensation->last_camera_defined      = 0;
    sensation->last_camera_robot_x      = 0.0;
    sensation->last_camera_robot_y      = 0.0;
    sensation->last_camera_robot_orientation = 0.0;

  }


  /* ============== ACTION ==================== */

  if (action != NULL && !action->is_initialized){

    action->is_initialized                 = 1;	/* is now initialized! */
    action->status                         = 0;
    action->type                           = 0;
    action->continuous_ping                = 0;
    action->trans_velocity                 = 0.0;
    action->rot_velocity                   = 0.0;
    action->trans_acceleration             = 0.0;
    action->rot_acceleration               = 0.0;
    action->trans_direction                = 0;
    action->rot_direction                  = 0;
    action->target_pos_x                   = 0.0;
    action->target_pos_y                   = 0.0;
   
    action->last_ping_time.tv_sec          = 0;	/* /usr/include/sys/time.h */
    action->last_ping_time.tv_usec         = 0;	/* /usr/include/sys/time.h */


  }


  /* ============== ALL ==================== */
  
  if (!allGlobal.is_initialized){
    allGlobal.is_initialized       = 1;
    allGlobal.robot_state          = robot_state;
    allGlobal.program_state        = program_state;
    allGlobal.sensation            = sensation;
    allGlobal.action               = action;
    allGlobal.robot_specifications = robot_specifications;
  }
}





/************************************************************************
 *
 *   NAME:         allocate_memory
 *                 
 *   FUNCTION:     Allocales all memory
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void allocate_memory(ALL_PARAMS)
{
  int i, max_nsensors;

  robot_specifications->global_map_dim_x     =
    (int) (robot_specifications->global_worldsize_x / 
	   robot_specifications->map_resolution);
  robot_specifications->global_map_dim_y     =
    (int) (robot_specifications->global_worldsize_y / 
	   robot_specifications->map_resolution);
  
  
  robot_specifications->occupancy_values = 
    (float *) (malloc(sizeof(float)
		      * robot_specifications->global_map_dim_x
		      * robot_specifications->global_map_dim_y));
  robot_specifications->map_active = 
    (int *) (malloc(sizeof(int)
		    * robot_specifications->global_map_dim_x
		    * robot_specifications->global_map_dim_y));
  
  for (i = 0; i < robot_specifications->global_map_dim_x
       * robot_specifications->global_map_dim_y; i++){
    robot_specifications->occupancy_values[i] = 0.5;
    robot_specifications->map_active[i] = 0;
  }
  
  
  robot_specifications->sonar_angles
    = (float *) (malloc(sizeof(float)
			* robot_specifications->num_sonar_sensors));
  if (robot_specifications->sonar_angles == NULL){
    fprintf(stderr, "MALLOC-Error 1 in allocate_memory(). Abort.\n");
    exit (-1);
  }
  for (i = 0; i < robot_specifications->num_sonar_sensors; i++)
    robot_specifications->sonar_angles[i] = 
      ((float) i) * (robot_specifications->last_sonar_angle 
		     - robot_specifications->first_sonar_angle)
	/ ((float) (robot_specifications->num_sonar_sensors - 1))
	  + robot_specifications->first_sonar_angle;
  
  
  robot_specifications->laser_angles
    = (float *) (malloc(sizeof(float)
			* robot_specifications->num_laser_sensors));
  if (robot_specifications->laser_angles == NULL){
    fprintf(stderr, "MALLOC-Error 1 in allocate_memory(). Abort.\n");
    exit (-1);
  }
  for (i = 0; i < robot_specifications->num_laser_sensors; i++)
    robot_specifications->laser_angles[i] = 0.0;
  
  
  
  
  max_nsensors = robot_specifications->num_sonar_sensors;
  if (max_nsensors < robot_specifications->num_laser_sensors)
    max_nsensors = robot_specifications->num_laser_sensors;

  dummy_sensors = (float *) (malloc(sizeof(float) * max_nsensors));
  null_sensors  = (float *) (malloc(sizeof(float) * max_nsensors));

  if (dummy_sensors == NULL || null_sensors == NULL){
    printf("MALLOC-Error 6 in allocate_memory(). Abort.\n");
    exit(1);
    }
  
  
  for (i = 0; i < (max_nsensors); i++){
    dummy_sensors[i] = robot_specifications->max_sonar_range
      * 0.5 * (sin(((float) i) * 0.3) + 1.0);
    null_sensors[i] = 0.0;
  }
  
  for (i = 0; i < robot_specifications->global_map_dim_x
       * robot_specifications->global_map_dim_y; i++)
    robot_specifications->map_active[i] = 0; /* default: not def. */
  
  
  sensation->sonar_values
    = (float *) (malloc(sizeof(float)
			* robot_specifications->num_sonar_sensors));
  if (sensation->sonar_values == NULL){
    fprintf(stderr, "MALLOC-Error 3 in allocate_memory(). Abort.\n");
    exit (-1);
  }
  
  
  for (i = 0; i < (robot_specifications->num_sonar_sensors); i++)
    sensation->sonar_values[i] = robot_specifications->max_sonar_range
      * 0.5 * (sin(((float) (i+5)) * 0.3) + 1.0);

  sensation->laser_values
    = (float *) (malloc(sizeof(float)
			* robot_specifications->num_laser_sensors));
  if (sensation->laser_values == NULL){
    fprintf(stderr, "MALLOC-Error 3b in allocate_memory(). Abort.\n");
    exit (-1);
  }
  
  
  for (i = 0; i < (robot_specifications->num_laser_sensors); i++)
    sensation->laser_values[i] = robot_specifications->max_laser_range
      * 0.5 * (sin(((float) (i+5)) * 0.3) + 1.0);
}


/************************************************************************
 *
 *   NAME:         free_memory
 *                 
 *   FUNCTION:     Counterpart to "allocate_memory()"
 *                 frees memory
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 NOTE: "NULL" disables free-ing
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void free_memory(ALL_PARAMS)
{
  /* ============== PROGRAM_STATE ==================== */
  
  if (program_state != NULL && program_state->is_initialized){
    program_state->is_initialized             = 0;
  }
  
  
  /* ============== ROBOT_SPECIFICATIONS ==================== */
  
  if (robot_specifications != NULL && robot_specifications->is_initialized){
    robot_specifications->is_initialized       = 0;


    if (robot_specifications->sonar_angles != NULL){
      free(robot_specifications->sonar_angles);
      robot_specifications->sonar_angles = NULL;
    }
    
  }

  
  /* ============== ROBOT_STATE ==================== */
  
  if (robot_state != NULL && robot_state->is_initialized){
    robot_state->is_initialized            = 0;
  }


  /* ============== SENSATION ==================== */
  
  if (sensation != NULL && sensation->is_initialized){
    sensation->is_initialized           = 0;

    if (sensation->sonar_values != NULL){
      free(sensation->sonar_values);
      sensation->sonar_values = NULL;
    }

  }


  /* ============== ACTION ==================== */

  if (action != NULL && action->is_initialized){
    action->is_initialized                 = 1;
  }


}




/************************************************************************
 *
 *   NAME:         clear_maps
 *                 
 *   FUNCTION:     Clears the maps
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void clear_maps(ALL_PARAMS)
{
  char button_text[128];
  int i;
  
  if (robot_specifications->occupancy_values != NULL &&
    robot_specifications->map_active != NULL)
    for (i = 0; i < robot_specifications->global_map_dim_x
	 * robot_specifications->global_map_dim_y; i++){
      robot_specifications->occupancy_values[i] = 0.5;
      robot_specifications->map_active[i] = 0;
    }
  program_state->total_area_covered         = 0.0;
  program_state->d_total_area_covered       = 0.0;

  sprintf(button_text,  "total area (%g)", program_state->total_area_covered);
  G_set_new_text(STATUS_TOTAL_AREA_BUTTON, button_text, 0);
  G_display_value(STATUS_TOTAL_AREA_BUTTON,  
		  program_state->total_area_covered);


  sprintf(button_text,  "new area (%g)", program_state->d_total_area_covered);
  G_set_new_text(STATUS_NEW_AREA_BUTTON, button_text, 0);
  G_display_value(STATUS_NEW_AREA_BUTTON,  
		  program_state->d_total_area_covered);


}
  


/************************************************************************
 *
 *   NAME:         connect_to_tcx
 *                 
 *   FUNCTION:     connects and registers messages
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void connect_to_tcx(ALL_PARAMS)
{
  char tcxName[128], hostname[80], pidname[80];
  const char *tcxMachine = NULL;
  struct timeval current_time;
  int auto_connect_now;

  /*
   * check if it is time to auto-connect 
   */
  
  gettimeofday(&current_time, NULL);
  auto_connect_now = program_state->tcx_autoconnect &&
    (current_time.tv_sec > last_attempt_tcx_connect.tv_sec + 3
     || (current_time.tv_sec == last_attempt_tcx_connect.tv_sec + 3 &&
	 current_time.tv_usec > last_attempt_tcx_connect.tv_usec));
  if (auto_connect_now){
    /*fprintf(stderr, "@");*/
    last_attempt_tcx_connect.tv_sec  = current_time.tv_sec;
    last_attempt_tcx_connect.tv_usec = current_time.tv_usec;
  }
  

  if (((auto_connect_now ||
	program_state->do_connect_to_BASE)
       && !program_state->tcx_connected_to_BASE)
      || ((auto_connect_now ||
	   program_state->do_connect_to_PANTILT)
	  && !program_state->tcx_connected_to_PANTILT)
      || ((auto_connect_now ||
	   program_state->do_connect_to_MAP)
	  && !program_state->tcx_connected_to_MAP)
      || ((auto_connect_now ||
	   program_state->do_connect_to_PLAN)
	  && !program_state->tcx_connected_to_PLAN)
      || ((auto_connect_now ||
	   program_state->do_connect_to_BUTTONS)
	  && !program_state->tcx_connected_to_BUTTONS)
      || ((auto_connect_now ||
	   program_state->do_connect_to_CAMERA)
	  && !program_state->tcx_connected_to_CAMERA)
#if 1
      || ((auto_connect_now ||
	   program_state->do_connect_to_BASESERVER)
	  && !program_state->tcx_connected_to_BASESERVER)
#endif
      || ((auto_connect_now ||
	   program_state->do_connect_to_SIMULATOR)
	  && !program_state->tcx_connected_to_SIMULATOR)
      || ((auto_connect_now ||
	   program_state->do_connect_to_SONARINT)
	  && !program_state->tcx_connected_to_SONARINT)
      || ((auto_connect_now ||
	   program_state->do_connect_to_LASERINT)
	  && !program_state->tcx_connected_to_LASERINT)
#ifdef UNIBONN
      || ((auto_connect_now ||
	   program_state->do_connect_to_SPEECH)
	  && !program_state->tcx_connected_to_SPEECH)
      || ((auto_connect_now ||
	   program_state->do_connect_to_ARM)
	  && !program_state->tcx_connected_to_ARM)
      || ((auto_connect_now ||
	   program_state->do_connect_to_FLOW)
	  && !program_state->tcx_connected_to_FLOW)
      || ((auto_connect_now ||
	   program_state->do_connect_to_TRACKER)
	  && !program_state->tcx_connected_to_TRACKER)
      || ((auto_connect_now ||
	   program_state->do_connect_to_SUNVIS)
	  && !program_state->tcx_connected_to_SUNVIS)
      || ((auto_connect_now ||
	   program_state->do_connect_to_CD)
	  && !program_state->tcx_connected_to_CD)
#endif /* UNIBONN */
#ifdef CD_VERSION
      || ((auto_connect_now ||
	   program_state->do_connect_to_CD)
	  && !program_state->tcx_connected_to_CD)
#endif /* CD_VERSION */
      ){ /* something to connect */
    
    
    
    if (program_state->do_connect_to_BASE
	&& !program_state->tcx_connected_to_BASE)
      G_display_switch(CONNECT_BASE_BUTTON, 3); /* just for display */
    
    if (program_state->do_connect_to_PANTILT
	&& !program_state->tcx_connected_to_PANTILT)
      G_display_switch(CONNECT_PANTILT_BUTTON, 3); /* just for display */
    
    if (program_state->do_connect_to_MAP
	&& !program_state->tcx_connected_to_MAP)
      G_display_switch(CONNECT_MAP_BUTTON, 3); /* just for display */
    
    if (program_state->do_connect_to_PLAN
	&& !program_state->tcx_connected_to_PLAN)
      G_display_switch(CONNECT_PLAN_BUTTON, 3); /* just for display */
    
    if (program_state->do_connect_to_BUTTONS
	&& !program_state->tcx_connected_to_BUTTONS)
      G_display_switch(CONNECT_BUTTONS_BUTTON, 3); /* just for display */
    
    if (program_state->do_connect_to_CAMERA
	&& !program_state->tcx_connected_to_CAMERA)
      G_display_switch(CONNECT_CAMERA_BUTTON, 3); /* just for display */
    
    if (program_state->do_connect_to_BASESERVER
	&& !program_state->tcx_connected_to_BASESERVER)
      G_display_switch(CONNECT_BASESERVER_BUTTON, 3); /* just for display */
    
    if (program_state->do_connect_to_SIMULATOR
	&& !program_state->tcx_connected_to_SIMULATOR)
      G_display_switch(CONNECT_SIMULATOR_BUTTON, 3); /* just for display */
    
    if (program_state->do_connect_to_SONARINT
	&& !program_state->tcx_connected_to_SONARINT)
      G_display_switch(CONNECT_SONARINT_BUTTON, 3); /* just for display */
    
    if (program_state->do_connect_to_LASERINT
	&& !program_state->tcx_connected_to_LASERINT)
      G_display_switch(CONNECT_LASERINT_BUTTON, 3); /* just for display */
    
#ifdef UNIBONN
    if (program_state->do_connect_to_SPEECH
	&& !program_state->tcx_connected_to_SPEECH)
      G_display_switch(CONNECT_SPEECH_BUTTON, 3); /* just for display */

    if (program_state->do_connect_to_ARM
	&& !program_state->tcx_connected_to_ARM)
      G_display_switch(CONNECT_ARM_BUTTON, 3); /* just for display */
    
      if (program_state->do_connect_to_TRACKER
	  && !program_state->tcx_connected_to_TRACKER)
	G_display_switch(CONNECT_TRACKER_BUTTON, 3); /* just for display */


      if (program_state->do_connect_to_FLOW
	  && !program_state->tcx_connected_to_FLOW)
	G_display_switch(CONNECT_FLOW_BUTTON, 3); /* just for display */

      if (program_state->do_connect_to_SUNVIS
	  && !program_state->tcx_connected_to_SUNVIS)
	G_display_switch(CONNECT_SUNVIS_BUTTON, 3); /* just for display */
#endif /* UNIBONN */

#ifdef CD_VERSION
      if (program_state->do_connect_to_CD
	  && !program_state->tcx_connected_to_CD)
	G_display_switch(CONNECT_CD_BUTTON, 3); /* just for display */
#endif /* CD_VERSION */



      if (!program_state->tcx_initialized){ /* TCX not yet initialized! */

	fprintf(stderr, "Connecting to TCX...");
	fflush(stderr);
      
	strcpy(tcxName, TCX_RHINO_MODULE_NAME);
	sprintf(pidname, "_%d_", getpid());
	strcat(tcxName, pidname);
	gethostname(hostname, 32);
	strcat(tcxName, hostname);
      
      
	/* ====== INITIALIZING TCX ============ */
	tcxMachine = bRobot.TCXHOST;
      
	fprintf(stderr, "%s:%6d:%s() - TCXHOST[%s]\n",
		__FILE__, __LINE__, __FUNCTION__, tcxMachine);

	if (tcxMachine != NULL){

	  G_display_switch(TITLE_BUTTON, 3);
	
	  tcxInitialize(tcxName, (char *)tcxMachine);
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




	

	  tcxRegisterCloseHnd(tcx_close_handler);
	  fprintf(stderr, "done.\n");
	  fflush(stderr);
	

	  /********* register messages and handlers *****************/

	  {
	    TCX_REG_MSG_TYPE TCX_message_array[] = {
	      BASE_messages,
	      SONAR_messages,
	      LASER_messages,
	      IR_messages,
	      COLLI_messages,
	      PANTILT_messages,
	      MAP_messages,
	      PLAN_messages,
	      BUTTONS_messages,
	      SIMULATOR_messages,
	      CAMERA_messages,
#ifdef CD_VERSION
	      CD_messages,
#endif /* CD_VERSION */
#ifdef UNIBONN
	      SPEECH_messages,
	      ARM_messages,
	      FLOW_messages,
	      EZR_messages,
	      SUNVIS_messages
#endif /* UNIBONN */
	    };

	    tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
				/ sizeof(TCX_REG_MSG_TYPE));
	    tcxRegisterHandlers(BASE_reply_handler_array,
				sizeof(BASE_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
	    tcxRegisterHandlers(SONAR_reply_handler_array,
				sizeof(SONAR_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
	    tcxRegisterHandlers(LASER_reply_handler_array,
				sizeof(LASER_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
	    tcxRegisterHandlers(COLLI_reply_handler_array,
				sizeof(COLLI_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
	    tcxRegisterHandlers(IR_reply_handler_array,
				sizeof(IR_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
	    tcxRegisterHandlers(PANTILT_reply_handler_array,
				sizeof(PANTILT_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
	    tcxRegisterHandlers(MAP_reply_handler_array,
				sizeof(MAP_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
	    tcxRegisterHandlers(PLAN_reply_handler_array,
				sizeof(PLAN_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
	    tcxRegisterHandlers(BUTTONS_reply_handler_array,
				sizeof(BUTTONS_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
	    tcxRegisterHandlers(SIMULATOR_reply_handler_array,
				sizeof(SIMULATOR_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
	    tcxRegisterHandlers(CAMERA_reply_handler_array,
				sizeof(CAMERA_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
#ifdef UNIBONN
	    tcxRegisterHandlers(SPEECH_reply_handler_array,
				sizeof(SPEECH_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
	    tcxRegisterHandlers(ARM_reply_handler_array,
				sizeof(ARM_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
	    tcxRegisterHandlers(FLOW_reply_handler_array,
				sizeof(FLOW_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
	    tcxRegisterHandlers(SUNVIS_reply_handler_array,
				sizeof(SUNVIS_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
	    tcxRegisterHandlers(EZR_reply_handler_array,
				sizeof(EZR_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
#endif /* UNIBONN */
#ifdef CD_VERSION
	    tcxRegisterHandlers(CD_reply_handler_array,
				sizeof(CD_reply_handler_array)
				/ sizeof(TCX_REG_HND_TYPE));
#endif /* CD_VERSION */
	  }

	  program_state->tcx_initialized = 1;
	}
	else
	  printf("WARNING: Environment variable TCXHOST not set. Connect failed.\n");
      }

      /********* connect to BASE *****************/
    
    if ((auto_connect_now || program_state->do_connect_to_BASE)
	&& !program_state->tcx_connected_to_BASE){

      if (program_state->do_connect_to_BASE)
	G_display_switch(CONNECT_BASE_BUTTON, 2);

      BASE = SONAR = tcxConnectOptional(TCX_BASE_MODULE_NAME);
      
      if (BASE != NULL){
	program_state->tcx_connected_to_BASE = 1;
	
	
	tcx_base_subscribe(ALL, robot_specifications->base_status_update_rate,
			   1, 0, 1, 1);
	tcx_base_set_velocity(ALL, INITIAL_TRANS_VELOCITY, 
			      INITIAL_ROT_VELOCITY);
	tcx_base_set_acceleration(ALL, INITIAL_TRANS_ACCELERATION,
				  INITIAL_ROT_ACCELERATION);
	tcx_base_set_mode(robot_state->base_mode, ALL);
	/* tcx_sonar_switch_on(ALL); */
      }
      else if (program_state->do_connect_to_BASE){
	putc(7, stderr);
	usleep(50000);
      }

      if (program_state->do_connect_to_BASE ||
	  program_state->tcx_connected_to_BASE)
	G_display_switch(CONNECT_BASE_BUTTON, 
			 program_state->tcx_connected_to_BASE);
      program_state->do_connect_to_BASE = 0;
    }    
    
    /********* connect to PANTILT *****************/
    
    if ((auto_connect_now || program_state->do_connect_to_PANTILT)
	&& !program_state->tcx_connected_to_PANTILT){

      if (program_state->do_connect_to_PANTILT)
	G_display_switch(CONNECT_PANTILT_BUTTON, 2);


      PANTILT = tcxConnectOptional(TCX_PANTILT_MODULE_NAME);

      if (PANTILT != NULL){
	program_state->tcx_connected_to_PANTILT = 1;
	
	tcx_pantilt_init(ALL);
	tcx_pantilt_set_acceleration(ALL, 10000.0, 10000.0);
      }
      else if (program_state->do_connect_to_PANTILT){
	putc(7, stderr);
	usleep(50000);
      }

      if (program_state->do_connect_to_PANTILT ||
	  program_state->tcx_connected_to_PANTILT)
	G_display_switch(CONNECT_PANTILT_BUTTON, 
			 program_state->tcx_connected_to_PANTILT);
      program_state->do_connect_to_PANTILT = 0;
    }    

    
    /********* connect to MAP *****************/
    
    if ((auto_connect_now || program_state->do_connect_to_MAP)
	&& !program_state->tcx_connected_to_MAP){

      if (program_state->do_connect_to_MAP)
	G_display_switch(CONNECT_MAP_BUTTON, 2);

      MAP = tcxConnectOptional(TCX_MAP_MODULE_NAME);

      if (MAP != NULL){
	program_state->tcx_connected_to_MAP = 1;
	

	tcx_map_register_auto_update(ALL, 1, 1);
	tcx_map_correction_parameters_query(ALL);
      }	
      else if (program_state->do_connect_to_MAP){
	putc(7, stderr);
	usleep(50000);
      }

      if (program_state->do_connect_to_MAP ||
	  program_state->tcx_connected_to_MAP)
	G_display_switch(CONNECT_MAP_BUTTON, 
			 program_state->tcx_connected_to_MAP);
      program_state->do_connect_to_MAP = 0;
    }    

    /********* connect to PLAN *****************/
    
    if ((auto_connect_now || program_state->do_connect_to_PLAN)
	&& !program_state->tcx_connected_to_PLAN){

      if (program_state->do_connect_to_PLAN)
	G_display_switch(CONNECT_PLAN_BUTTON, 2);

      PLAN = tcxConnectOptional(TCX_PLAN_MODULE_NAME);

      if (PLAN != NULL){
	program_state->tcx_connected_to_PLAN = 1;
	tcx_plan_register_auto_update(ALL, 1);

      }
      else if (program_state->do_connect_to_PLAN){
	putc(7, stderr);
	usleep(50000);
      }

      if (program_state->do_connect_to_PLAN ||
	  program_state->tcx_connected_to_PLAN)
	G_display_switch(CONNECT_PLAN_BUTTON, 
			 program_state->tcx_connected_to_PLAN);
      program_state->do_connect_to_PLAN = 0;
    }    

    /********* connect to CAMERA *****************/
    
    if ((auto_connect_now || program_state->do_connect_to_CAMERA)
	&& !program_state->tcx_connected_to_CAMERA){

      if (program_state->do_connect_to_CAMERA)
	G_display_switch(CONNECT_CAMERA_BUTTON, 2);

      CAMERA = tcxConnectOptional(TCX_CAMERA_MODULE_NAME);

      if (CAMERA != NULL){
	program_state->tcx_connected_to_CAMERA = 1;
	
	tcx_camera_register_auto_update(ALL, 0);

	/*tcx_camera_initialize_marker(ALL);*/
	/*tcx_camera_find_marker(ALL, 0);*/
	
      }
      else if (program_state->do_connect_to_CAMERA){
	putc(7, stderr);
	usleep(50000);
      }

      if (program_state->do_connect_to_CAMERA ||
	  program_state->tcx_connected_to_CAMERA)
	G_display_switch(CONNECT_CAMERA_BUTTON, 
			 program_state->tcx_connected_to_CAMERA);
      program_state->do_connect_to_CAMERA = 0;
    }    


    /********* connect to BASESERVER *****************/
    
      /*
       * baseServer can only accept one connection at a time.
       * This connection will mess up COLLI or other baseServer clients.
       *  -tds
       */
#if 1
    if ((auto_connect_now || program_state->do_connect_to_BASESERVER)
	&& !program_state->tcx_connected_to_BASESERVER){

      if (program_state->do_connect_to_BASESERVER)
	G_display_switch(CONNECT_BASESERVER_BUTTON, 2);

      BASESERVER = tcxConnectOptional(TCX_BASESERVER_MODULE_NAME);

      if (BASESERVER != NULL){
	program_state->tcx_connected_to_BASESERVER = 1;
	/* insert initializations here */
      }
      else if (program_state->do_connect_to_BASESERVER){
	putc(7, stderr);
	usleep(50000);
      }

      if (program_state->do_connect_to_BASESERVER ||
	  program_state->tcx_connected_to_BASESERVER)
	G_display_switch(CONNECT_BASESERVER_BUTTON, 
			 program_state->tcx_connected_to_BASESERVER);
      program_state->do_connect_to_BASESERVER = 0;
    }    
#endif

    /********* connect to SIMULATOR *****************/
    
    if ((auto_connect_now || program_state->do_connect_to_SIMULATOR)
	&& !program_state->tcx_connected_to_SIMULATOR){

      if (program_state->do_connect_to_SIMULATOR)
	G_display_switch(CONNECT_SIMULATOR_BUTTON, 2);

      SIMULATOR = tcxConnectOptional(TCX_SIMULATOR_MODULE_NAME);

      if (SIMULATOR != NULL){
	program_state->tcx_connected_to_SIMULATOR = 1;
	/* insert initializations here */
      }
      else if (program_state->do_connect_to_SIMULATOR){
	putc(7, stderr);
	usleep(50000);
      }

      if (program_state->do_connect_to_SIMULATOR ||
	  program_state->tcx_connected_to_SIMULATOR)
	G_display_switch(CONNECT_SIMULATOR_BUTTON, 
			 program_state->tcx_connected_to_SIMULATOR);
      program_state->do_connect_to_SIMULATOR = 0;
    }    


    /********* connect to SONARINT *****************/
    
    if ((auto_connect_now || program_state->do_connect_to_SONARINT)
	&& !program_state->tcx_connected_to_SONARINT){

      if (program_state->do_connect_to_SONARINT)
	G_display_switch(CONNECT_SONARINT_BUTTON, 2);

      SONARINT = tcxConnectOptional(TCX_SONARINT_MODULE_NAME);

      if (SONARINT != NULL){
	program_state->tcx_connected_to_SONARINT = 1;
	/* insert initializations here */
      }
      else if (program_state->do_connect_to_SONARINT){
	putc(7, stderr);
	usleep(50000);
      }

      if (program_state->do_connect_to_SONARINT ||
	  program_state->tcx_connected_to_SONARINT)
	G_display_switch(CONNECT_SONARINT_BUTTON, 
			 program_state->tcx_connected_to_SONARINT);
      program_state->do_connect_to_SONARINT = 0;
    }    


    /********* connect to LASERINT *****************/
    
    if ((auto_connect_now || program_state->do_connect_to_LASERINT)
	&& !program_state->tcx_connected_to_LASERINT){

      if (program_state->do_connect_to_LASERINT)
	G_display_switch(CONNECT_LASERINT_BUTTON, 2);

      LASERINT = tcxConnectOptional(TCX_LASERINT_MODULE_NAME);

      if (LASERINT != NULL){
	program_state->tcx_connected_to_LASERINT = 1;
	/* insert initializations here */
      }
      else if (program_state->do_connect_to_LASERINT){
	putc(7, stderr);
	usleep(50000);
      }

      if (program_state->do_connect_to_LASERINT ||
	  program_state->tcx_connected_to_LASERINT)
	G_display_switch(CONNECT_LASERINT_BUTTON, 
			 program_state->tcx_connected_to_LASERINT);
      program_state->do_connect_to_LASERINT = 0;
    }    
#ifdef UNIBONN
    /********* connect to SPEECH *****************/
    
    if ((auto_connect_now || program_state->do_connect_to_SPEECH)
	&& !program_state->tcx_connected_to_SPEECH){

      if (program_state->do_connect_to_SPEECH)
	G_display_switch(CONNECT_SPEECH_BUTTON, 2);

      SPEECH = tcxConnectOptional(TCX_SPEECH_MODULE_NAME);

      if (SPEECH != NULL){
	program_state->tcx_connected_to_SPEECH = 1;

	tcx_speech_talk_text(ALL, "Hi there.");
	
      }
      else if (program_state->do_connect_to_SPEECH){
	putc(7, stderr);
	usleep(50000);
      }

      if (program_state->do_connect_to_SPEECH ||
	  program_state->tcx_connected_to_SPEECH)
	G_display_switch(CONNECT_SPEECH_BUTTON, 
			 program_state->tcx_connected_to_SPEECH);
      program_state->do_connect_to_SPEECH = 0;
    }    
    

    /********* connect to ARM *****************/
    
    if ((auto_connect_now || program_state->do_connect_to_ARM)
	&& !program_state->tcx_connected_to_ARM){

      if (program_state->do_connect_to_ARM)
	G_display_switch(CONNECT_ARM_BUTTON, 2);

      ARM = tcxConnectOptional(TCX_ARM_MODULE_NAME);

      if (ARM != NULL){
	program_state->tcx_connected_to_ARM = 1;
	tcx_arm_register_auto_update(ALL, 1);

      }
      else if (program_state->do_connect_to_ARM){
	putc(7, stderr);
	usleep(50000);
      }

      if (program_state->do_connect_to_ARM ||
	  program_state->tcx_connected_to_ARM)
	G_display_switch(CONNECT_ARM_BUTTON, 
			 program_state->tcx_connected_to_ARM);
      program_state->do_connect_to_ARM = 0;
    }    


    /********* connect to SUNVIS *****************/

    if ((auto_connect_now || program_state->do_connect_to_SUNVIS)
	&& !program_state->tcx_connected_to_SUNVIS){

      if (program_state->do_connect_to_SUNVIS)
	G_display_switch(CONNECT_SUNVIS_BUTTON, 2);

      SUNVIS = tcxConnectOptional(TCX_SUNVIS_MODULE_NAME);

      if (SUNVIS != NULL){
	program_state->tcx_connected_to_SUNVIS = 1;
	tcx_sunvis_register_auto_update(1, ALL);
      }

      else if (program_state->do_connect_to_SUNVIS){
	putc(7, stderr);
	usleep(50000);
      }

      if (program_state->do_connect_to_SUNVIS ||
	  program_state->tcx_connected_to_SUNVIS)
	G_display_switch(CONNECT_SUNVIS_BUTTON, 
			 program_state->tcx_connected_to_SUNVIS);
      program_state->do_connect_to_SUNVIS = 0;
    }

    /********* connect to TRACKER *****************/


    if ((auto_connect_now || program_state->do_connect_to_TRACKER)
	&& !program_state->tcx_connected_to_TRACKER){

      if (program_state->do_connect_to_TRACKER)
	G_display_switch(CONNECT_TRACKER_BUTTON, 2);

      TRACKER = tcxConnectOptional(TCX_TRACKER_MODULE_NAME);

      if (TRACKER != NULL){
	program_state->tcx_connected_to_TRACKER = 1;
	tracker_init(ALL);
      }

      else if (program_state->do_connect_to_TRACKER){
	putc(7, stderr);
	usleep(50000);
      }

      if (program_state->do_connect_to_TRACKER ||
	  program_state->tcx_connected_to_TRACKER)
	G_display_switch(CONNECT_TRACKER_BUTTON, 
			 program_state->tcx_connected_to_TRACKER);
      program_state->do_connect_to_TRACKER = 0;
    }

    /********* connect to FLOW *****************/


    if ((auto_connect_now || program_state->do_connect_to_FLOW)
	&& !program_state->tcx_connected_to_FLOW){

      if (program_state->do_connect_to_FLOW)
	G_display_switch(CONNECT_FLOW_BUTTON, 2);

      FLOW = tcxConnectOptional(TCX_FLOW_MODULE_NAME);

      if (FLOW != NULL){
	program_state->tcx_connected_to_FLOW = 1;
	flow_request_info(ALL);
      }


      if (program_state->do_connect_to_FLOW ||
	  program_state->tcx_connected_to_FLOW)
	G_display_switch(CONNECT_FLOW_BUTTON, 
			 program_state->tcx_connected_to_FLOW);
      program_state->do_connect_to_FLOW = 0;
    }
#endif /* UNIBONN */

#ifdef CD_VERSION
    /********* connect to CD *****************/


    if ((auto_connect_now || program_state->do_connect_to_CD)
	&& !program_state->tcx_connected_to_CD){

      if (program_state->do_connect_to_CD)
	G_display_switch(CONNECT_CD_BUTTON, 2);

      CD = tcxConnectOptional(TCX_CD_MODULE_NAME);

      if (CD != NULL){
	if (program_state->last_cd_phrase != -1)
	  G_display_switch(CD_BUTTONS[program_state->last_cd_phrase], 0);
	program_state->last_cd_phrase = -1;
	program_state->tcx_connected_to_CD = 1;
	/* cd_init(ALL); */
      }

      else if (program_state->do_connect_to_CD){
	putc(7, stderr);
	usleep(50000);
      }

      if (program_state->do_connect_to_CD ||
	  program_state->tcx_connected_to_CD)
	G_display_switch(CONNECT_CD_BUTTON, 
			 program_state->tcx_connected_to_CD);
      program_state->do_connect_to_CD = 0;
    }
#endif /* CD_VERSION */

    /********* connect to BUTTONS *****************/


    if ((auto_connect_now || program_state->do_connect_to_BUTTONS)
	&& !program_state->tcx_connected_to_BUTTONS){

      if (program_state->do_connect_to_BUTTONS)
	G_display_switch(CONNECT_BUTTONS_BUTTON, 2);

      BUTTONS = tcxConnectOptional(TCX_BUTTONS_MODULE_NAME);

      if (BUTTONS != NULL){
	program_state->tcx_connected_to_BUTTONS = 1;
	buttons_init(ALL);
      }


      if (program_state->do_connect_to_BUTTONS ||
	  program_state->tcx_connected_to_BUTTONS)
	G_display_switch(CONNECT_BUTTONS_BUTTON, 
			 program_state->tcx_connected_to_BUTTONS);
      program_state->do_connect_to_BUTTONS = 0;
    }

  }
}




/************************************************************************
 *
 *   NAME:         interrupt_handler()
 *                 
 *   FUNCTION:     some ^C signal or kill command arrived
 *                 
 *   PARAMETERS:   int sig           signal
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void interrupt_handler(int sig){


  ROBOT_SPECIFICATIONS_PTR robot_specifications = 
    allGlobal.robot_specifications;
  PROGRAM_STATE_PTR        program_state        = allGlobal.program_state;
  ROBOT_STATE_PTR          robot_state          = allGlobal.robot_state;
  ACTION_PTR               action               = allGlobal.action;
  SENSATION_PTR            sensation            = allGlobal.sensation;

  if (program_state->tcx_connected_to_BUTTONS){
    robot_state->red_light_status    = BUTTON_LIGHT_STATUS_OFF;
    robot_state->yellow_light_status = BUTTON_LIGHT_STATUS_OFF;
    robot_state->green_light_status  = BUTTON_LIGHT_STATUS_OFF;
    robot_state->blue_light_status   = BUTTON_LIGHT_STATUS_OFF;
    set_buttons(ALL, -1);
  }

  tcx_base_stop_robot(ALL);
  if (program_state->tcx_connected_to_BASE) /* still connected? */
    disconnect_BASE(ALL);
  if (program_state->tcx_connected_to_PANTILT) /* still connected? */
    disconnect_PANTILT(ALL);
  if (program_state->tcx_connected_to_MAP) /* still connected? */
    disconnect_MAP(ALL);
  if (program_state->tcx_connected_to_PLAN) /* still connected? */
    disconnect_PLAN(ALL);
  if (program_state->tcx_connected_to_BUTTONS) /* still connected? */
    disconnect_BUTTONS(ALL);
  if (program_state->tcx_connected_to_CAMERA) /* still connected? */
    disconnect_CAMERA(ALL);
  if (program_state->tcx_connected_to_SONARINT) /* still connected? */
    disconnect_SONARINT(ALL);
  if (program_state->tcx_connected_to_LASERINT) /* still connected? */
    disconnect_LASERINT(ALL);
  if (program_state->tcx_connected_to_SIMULATOR) /* still connected? */
    disconnect_SIMULATOR(ALL);
  if (program_state->tcx_connected_to_BASESERVER) /* still connected? */
    disconnect_BASESERVER(ALL);
#ifdef UNIBONN
  if (program_state->tcx_connected_to_SPEECH) /* still connected? */
    disconnect_SPEECH(ALL);
  if (program_state->tcx_connected_to_ARM) /* still connected? */
    disconnect_ARM(ALL);
  if (program_state->tcx_connected_to_FLOW) /* still connected? */
    disconnect_FLOW(ALL);
  if (program_state->tcx_connected_to_SUNVIS) /* still connected? */
    disconnect_SUNVIS(ALL);
  if (program_state->tcx_connected_to_TRACKER) /* still connected? */
    disconnect_TRACKER(ALL);
#endif /* UNIBONN */
#ifdef CD_VERSION
  if (program_state->tcx_connected_to_CD) /* still connected? */
    disconnect_CD(ALL);
#endif /* CD_VERSION */

  fprintf(stderr, "Thanks for making a stupid program mildly content.\n");
  fflush(stderr);
  exit(0);
}






