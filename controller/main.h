
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








#ifndef LEARN_LOADED
#define LEARN_LOADED
#include <sys/time.h>


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define TCX_RHINO_MODULE_NAME "RHINO"


#ifdef TOURGUIDE_VERSION
#define MAX_NUM_TOURS 4
#endif /* TOURGUIDE_VERSION */


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* GENERAL STATE VARIABLE TYPES */




typedef struct{
  int   is_initialized;		/* one, if this has been init before */

  int    in_motion;
  int    state_is_known;	/* zero, if just connected, one otherwise */

  /*** adopted from rwibase_interface.h/c ***/
  float  last_update_time;	/* updated by status report: base clock */
  float  rot_set_acceleration;	/* set by command */
  float  rot_velocity;	        /* updated by status_report */
  float  rot_set_velocity;	/* set by command */
  float  rot_position;		/* updated by status report */
  float  trans_set_acceleration;/* set by command */
  float  trans_velocity;        /* updated by status report */
  float  trans_set_velocity;	/* set by command */ 
  float  trans_position;	/* updated by status report */
  float  x;			/* updated by status report */
  float  y;			/* updated by status report */
  float  orientation;		/* updated by status report */
  int    trans_direction;	/* updated by status report */
  int    trans_set_direction;	/* updated by command */
  int    rot_direction;		/* updated by status report */
  int    rot_set_direction;	/* updated by command */
  int    rot_moving;		/* computed from status report */
  int    trans_moving;		/* computed from status report */
  int    bumpers;		/* updated by status report */
  int    bump;
  int    emergency;		/* updated by status report */
  int    emergencyProcedure;	/* updated by command */
  int    sonar_on;		/* updated by button/status report */

  int    base_mode;		/* set by command */
  int    arm_moved_out;		/* 1, if the arm is moved out and
				 * ready to operate */
  int    pick_sequence;         /* 0, if pick is next command,
                                   1, if lift object
                                   2, if drop object */
  int    arm_gripper_closed;	/* 1, if the gripper is closed */
  
  float correction_parameter_x;	/* used for position correction */
  float correction_parameter_y;
  float correction_parameter_angle;
  int   correction_type;	/* rotation or translation correction */


  int red_light_status;		/* 1 = light is on, 0 off, 2 = flashing */
  int yellow_light_status;	/* 1 = light is on, 0 off, 2 = flashing */
  int green_light_status;	/* 1 = light is on, 0 off, 2 = flashing */
  int blue_light_status;	/* 1 = light is on, 0 off, 2 = flashing */

  float battery_level;


} ROBOT_STATE, *ROBOT_STATE_PTR;



typedef struct{
  int is_initialized;		/* one, if this has been init before */
  int do_Xdisplay;		/* set by command-line: 1=generate X-Win dis.*/
  int graphics_initialized;	/* 1, if graphics initialized */
  int quit;			/* 1, if program shall terminate */

  int tcx_initialized;		/* 1, if TCX initialized (and connected) */
  int do_connect_to_BASE;	/* shall we connect to the BASE device? */
  int do_connect_to_PANTILT;	/* shall we connect to the PAN/TILT device? */
  int do_connect_to_MAP;	/* shall we connect to the MAP? */
  int do_connect_to_PLAN;	/* shall we connect to the PLAN? */
  int do_connect_to_ARM;	/* shall we connect to the ARM? */
  int do_connect_to_TRACKER;	/* shall we connect to the TRACKER? */
  int do_connect_to_CD;		/* shall we connect to the CD? */
  int do_connect_to_BUTTONS;	/* shall we connect to the CD? */
  int do_connect_to_FLOW;	/* shall we connect to the FLOW? */
  int do_connect_to_BASESERVER;	/* shall we connect to the BASESERVER? */
  int do_connect_to_SIMULATOR;	/* shall we connect to the SIMULATOR? */
  int do_connect_to_SONARINT;	/* shall we connect to the SONARINT? */
  int do_connect_to_LASERINT;	/* shall we connect to the LASERINT? */
  int tcx_connected_to_BASE;	/* 1, if connection established */
  int tcx_connected_to_PANTILT;	/* 1, if connection established */
  int tcx_connected_to_MAP;	/* 1, if connection established */
  int tcx_connected_to_PLAN;	/* 1, if connection established */
  int tcx_connected_to_ARM;	/* 1, if connection established */
  int tcx_connected_to_TRACKER;	/* 1, if connection established */
  int tcx_connected_to_CD;	/* 1, if connection established */
  int tcx_connected_to_BUTTONS;	/* 1, if connection established */
  int tcx_connected_to_FLOW;	/* 1, if connection established */
  int tcx_connected_to_BASESERVER;
  int tcx_connected_to_SIMULATOR;
  int tcx_connected_to_SONARINT;
  int tcx_connected_to_LASERINT;
  int tcx_autoconnect;		/* 1, if tcx tries to connect to all
				 * clients in regular time intervals */

  int do_connect_to_SPEECH;	/* shall we connect to the SPEECH device? */
  int tcx_connected_to_SPEECH;	/* 1, if connection established */
  int do_connect_to_CAMERA;	/* shall we connect to the CAMERA device? */
  int tcx_connected_to_CAMERA;	/* 1, if connection established */
#ifdef UNIBONN
  int do_connect_to_SUNVIS;	/* shall we connect to the PLAN? */
  int tcx_connected_to_SUNVIS;	/* 1, if connection established */
#endif /* UNIBONN */


  int something_happened;	/* for main loop: inidicates we cannot sleep */
  
  int script_on;		/* 1, if in replay-script mode */
  int continuous_script_mode;	/* 1, if replay shall be continuous */
  int script_number_episodes;	/* number of episodes in the script */
  int *script_nevents_per_episode; /* vector of number of events */
  int current_episode_displayed; /* number of the episode displayed last */
  int current_event_displayed;	/* number of the event displayed last */

  int display_model_on;		/* display model derivatives and stuff? */
  int active_network;		/* network that is displayed */
  int continuous_mode;		/* continuous actions, greedy! */
  int random_continuous_mode;	/* continuous actions, random! */

  float total_area_covered;	/* total area that is coevered by maps */
  float d_total_area_covered;	/* first derivative - smooth approximation */
  float total_distance_traveled; /* total distance traveled in cm */
  float advance_rate;		/* Rate at which the robot changes its
				 * location */

  
  int   autonomous_mode;	/* 1, if the planner is in autonomous mode */

  float robot_pos_x_when_going_autonomous;
  float robot_pos_y_when_going_autonomous;

  int   auto_reset_expl_mode;	/*  */

#ifdef TOURGUIDE_VERSION
  int learning_tour;		/* 1, to indicate tour learning mode */
  int giving_tour;		/* 1, to indicate that robot is willing
				 * to give a tour */
  int tour_modus;		/* 0 = no tour
				 * 1 = robot waiting 
				 * 2 = planner
				 * 3 = local obst-avoid */
  int num_tour_goals;		/* total number of tour goals */
  int current_tour;		/* number of the current tour */
  int current_tour_goal;	/* current goal */
  int actual_tour_text;		/* actual text that is being played */
  int tour_defined[MAX_NUM_TOURS];/* number of different tours */

#endif /* TOURGUIDE_VERSION */

#ifdef UNIBONN
  int   hunting_mode;		/* top-level mode: robot is hunting
				 * an object */

  int   approaching_mode;	/* 1, if the controller gave a direct
				 * action command for graping the cup */
  int   active_goal_number;	/* number of the current object in the 
				 * database that is being approached*/

  int   n_carried_objetcs;	/* number of objects the robot carries */
  
  int   looking_for_trash_bin_mode; /* 1, if we are currently trying to get
				     * next to a trash bin */

#endif /* UNIBONN */

#ifdef CD_VERSION
  int   last_cd_phrase;		/* number of the last phrase, -1 if none */
#endif /* CD_VERSION */


  int   camera_continuous_grabbing; /* -1 = off, otherwise camera number */

  int   control_window_mode;	/* 0=target point, 1=joystick */
  int   pantilt_window_mode;	/* speed of pan/tilt */
  int   map_regular_update_on;	/* flag, if regular update */

} PROGRAM_STATE, *PROGRAM_STATE_PTR;



typedef struct{
  int   is_initialized;		/* one, if this has been init before */

  int status;			/* 1 = new command present 
				 * 2 = command in execution
				 * 0 = command finished */

  int type;			/* 0 = no action present
				 * 1 = emergency stop (overwrites everything)
				 * 2 = specified by velocities (open action) 
				 * 3 = specified by relative target point 
				 * 4 = specified by absolute target point 
				 */

  
  int continuous_ping;		/* 1, if the action command should be
				 * repeated in regular intervals */

  float trans_acceleration;	/* acceleration for translation */
  float rot_acceleration;	/* acceleration for rotation */
  float trans_velocity;		/* velocity for translation */
  float rot_velocity;		/* velocity for rotation */
  
  int rot_direction;		/* 0=clockwise, 1=anticlockwise, only VELact.*/
  int trans_direction;		/* 0=forward, 1=backward, only VELocity act. */

  float target_pos_x;		/* coordinate of the target pos. */
  float target_pos_y;		/* coordinate of the target pos. */


  struct timeval last_ping_time; /* time of last ping */
  
} ACTION, *ACTION_PTR;




typedef struct{
  int   is_initialized;		/* one, if this has been init before */

  int real_world_data;		/* 1, if the data are real (and not random) */

  int new_sonar;		/* 1, if new sonar sensatitons */
  float *sonar_values;		/* values of the sonar */

  int new_laser;		/* 1, if new laser sensatitons */
  float *laser_values;		/* values of the laser */


  int new_camera;		/* 1, if sensartions new */

  float tilt;			/* tilting angle */
  float pan;			/* pan angle */

  struct timeval last_update_time; /* time when values were updated last */


  int red_button_status;	/* 1 = pushed, 0 = not pushed */
  int red_button_changed;	/* 1 = just cnaged, 0 = didn't change */
  int yellow_button_status;	/* 1 = pushed, 0 = not pushed */
  int yellow_button_changed;	/* 1 = just cnaged, 0 = didn't change */
  int green_button_status;	/* 1 = pushed, 0 = not pushed */
  int green_button_changed;	/* 1 = just cnaged, 0 = didn't change */
  int blue_button_status;	/* 1 = pushed, 0 = not pushed */
  int blue_button_changed;	/* 1 = just cnaged, 0 = didn't change */

  int flow_result;             /* result from the FLOW routine */

  int last_camera_defined;
  float last_camera_robot_x;
  float last_camera_robot_y;
  float last_camera_robot_orientation;


} SENSATION, *SENSATION_PTR;



typedef struct{
  int   is_initialized;		/* one, if this has been init before */

  float robot_size;		/* size of the robot in cm */
  float global_worldsize_x;	/* size of the world-display wondow */
  float global_worldsize_y;	/* size of the world-display wondow */
  float map_resolution;		/* for the map display, if used. in cm */
  int  global_map_dim_x;	/* ditto, number of pixels */
  int  global_map_dim_y;	/* ditto, number of pixels */
  float *occupancy_values;	/* internal map, will be allocated anyhow */
  int   *map_active;		/* indicates if map defined at all... */

  float translational_speed;	/* max. speed for translations */
  float rotational_speed;	/* max. speed for rotations */
  float max_security_dist;	/* for the planner: max dist. value */
    
  int sonar_defined;		/* 1, if robot uses sonar sensors */

  int   num_sonar_sensors;	/* number of sonar sensors */
  float first_sonar_angle;	/* offset of the first sensor */
  float last_sonar_angle;	/* offset of the last sensor */
  float min_sonar_range;	/* minimum sensor range in cm */
  float max_sonar_range;	/* maximum sensor range in cm */
  float *sonar_angles;		/* sensor angle values (vector) */

  int   num_laser_sensors;	/* number of laser sensors */
  float first_laser_angle;	/* offset of the first sensor */
  float last_laser_angle;	/* offset of the last sensor */
  float min_laser_range;	/* minimum sensor range in cm */
  float max_laser_range;	/* maximum sensor range in cm */
  float *laser_angles;		/* sensor angle values (vector) */

  int   autoshift;		/* 1, if autoshift of the map defined, 0 else*/
  float autoshift_distance;	/* amount we shifty the display */
  float safety_margin;		/* minimum distance to walls */

  float autoshifted_x;		/* cm the display has been shifted */
  float autoshifted_y;		/* cm the display has been shifted */
  int   autoshifted_int_x;	/* number of boxes display has been shifted*/
  int   autoshifted_int_y;	/* number of boxes display has been shifted*/

  long  refresh_interval;	/* interval for refreshing action commands */
  float dist_action_achieved; /* if the robot is closer than that
			       * distance, we consider the action a 
			       * success */

  char log_file_name[80];	/* filename for logging */
  int  log_file_open;		/* flag, 1 if file opened */
  int  do_log;			/* flag, 1 if file should be used */
  int  log_file_unopenable;	/* 1, if open failed in the past */
  FILE *log_iop;		/* pointer to file (for fprintf(...)) */

  char script_file_name[80];	/* filename for scripts */
  char doc_file_name[80];	/* comments on the script file */

  float statistics_time_decay_rate; /* rate with which we decay values */


  int   base_status_update_rate; /* rate at which the BASE process shall send
				  * status updates */

  float min_dist_between_objects; /* for recognizing objects: min
				   * distance between two objects */

  float camera_1_opening_angle;	/* angle of camera 1 */
  float camera_1_min_perceptual_dist;	/* minimum perceptual dist for 
					   * camera 1 */
  float camera_1_max_perceptual_dist;	/* maximum perceptual dist for 
					   * camera 1 */
  float camera_2_opening_angle;	/* angle of camera 2 */
  float camera_2_min_perceptual_dist;	/* minimum perceptual dist for 
					   * camera 2 */
  float camera_2_max_perceptual_dist;	/* maximum perceptual dist for 
					   * camera 2 */
#ifdef UNIBONN
  int max_num_allowed_failures_in_object_recognition; /* name says it */
  float approach_object_distance; /* distance to the object when
				   * approaching it*/

  int  set_points_when_searching_objects;
  int  set_points_when_searching_bins;
#endif /* UNIBONN */

  float  arm_move_out_in_force;	/* standard force for moving the arm
				 * in or out - is expected as an
				 * argument in the TCX command */
  float display_update_interval; /* how often will we refresh the display? */

  float X_window_size;		/* scale factor for the X-window */
  float map_update_interval;	/* update intervals for maps, in seconds */
  float sensor_update_interval;	/* same for sensor display */

} ROBOT_SPECIFICATIONS, *ROBOT_SPECIFICATIONS_PTR;





typedef struct{
  int   is_initialized;		/* one, if this has been init before */
  ROBOT_STATE_PTR          robot_state;
  PROGRAM_STATE_PTR        program_state;
  ACTION_PTR               action;
  SENSATION_PTR            sensation;
  ROBOT_SPECIFICATIONS_PTR robot_specifications;
} ALL_TYPE;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


extern ALL_TYPE allGlobal;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define TCX_BASESERVER_MODULE_NAME "baseTCXServer"
#define TCX_SONARINT_MODULE_NAME "SONARINT"
#define TCX_LASERINT_MODULE_NAME "LASERINT"


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define RELATIVE_DISTANCE_DIVISOR 0.05

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define ALL robot_state, action, sensation, program_state, robot_specifications
#define ALL_PARAMS ROBOT_STATE_PTR          robot_state, \
                   ACTION_PTR               action, \
		   SENSATION_PTR            sensation, \
		   PROGRAM_STATE_PTR        program_state, \
		   ROBOT_SPECIFICATIONS_PTR robot_specifications



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


#endif

