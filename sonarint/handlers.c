
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
#include "bUtils.h"
#include "tcx.h"
#include "tcxP.h"
#include "Application.h"
#include "Net.h"
#include "EZX11.h"
#include "o-graphics.h"


#define TCX_define_variables /* this makes sure variables are installed */

#include "MAP-messages.h"


#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */
#include "SONAR-messages.h"
#include "LASER-messages.h"
#include "BASE-messages.h"


#include "SONARINT.h"

#ifdef RHINO_PLUS
#include "rst.h" /* rhino stuff */
#endif

#include "libc.h"
#include "beeSoftVersion.h"
#include "libbUtils.h"
#include "librobot.h"
#include "libezx.h"



int new_sonar_reading = 0;


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         BASE_update_status_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void BASE_update_status_reply_handler(TCX_REF_PTR                  ref,
				      BASE_update_status_reply_ptr status)
{
  float uncertainty;
  float diff_x, diff_y, distance, angle, new_x, new_y;
  float status_new_x, status_new_y;
  char out_text[256];
  struct timeval t; 
  time_t current_time;

#ifdef SONARINT_debug
  fprintf(stderr, "TCX: Received a BASE_update_status_reply message.\n");
  fprintf(stderr, "robot: %g %g %g\n", 
	  status->pos_x, status->pos_y, status->orientation);
#endif

  
  /*
   * LOGGING DATA
   */

  if (ref != NULL &&	/* NULL indicates internal call, no TCX */
      log_iop != NULL){	/* NULL: No logging */
    time(&current_time);
    strftime (out_text, 50, "\n@SENS %d-%m-%y %H:%M:%S",
	      localtime(&current_time));
    gettimeofday (&t,NULL);
    fprintf (log_iop, "%s.%d\n", out_text, t.tv_usec);
    fprintf (log_iop,"#ROBOT %f %f %f\n\n",
	     status->pos_x, status->pos_y, 90.0 - status->orientation);
  }
  
  
  /*
   * When processing a script file, robot position that has been received
   * via TCX will be ignored
   */

  if (!program_state->processing_script || ref == NULL){

    if (!robot_state->known ||	/* sanity check! */
	fabs(robot_state->x - status->pos_x)
	+ fabs(robot_state->y - status->pos_y) < 300.0){
      
      
      /*
       * And now compute the internal belief.
       */
      
      robot_state->x                   = status->pos_x;
      robot_state->y                   = status->pos_y;
      robot_state->orientation         = 90.0 - status->orientation;
      robot_state->translational_speed = status->trans_current_speed;
      robot_state->rotational_speed    = status->rot_current_speed;
      robot_state->known       = 1;
      for (;robot_state->orientation > 360.0;)
	robot_state->orientation -= 360.0;
      for (;robot_state->orientation < 0.0;)
	robot_state->orientation += 360.0;
    }
    else{
      putc(7,stderr);
      fprintf(stderr, "## STATE UNKNOWN ##\n");
      robot_state->known = 0;
    }
  }

  if (ref != NULL)		/* NULL indicates internal call, no TCX */
    tcxFree("BASE_update_status_reply", status);
}



/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         SONAR_ir_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SONAR_ir_reply_handler (TCX_REF_PTR          ref,
			     SONAR_ir_reply_ptr   ir)
{


/*#ifdef TCX_debug*/
  fprintf(stderr, "TCX: Received a SONAR_ir_reply message.\n");
/*#endif*/

  tcxFree("SONAR_ir_reply", ir);

}


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         SONAR_sonar_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

struct timeval last_message_time = {0, 0};

static float last_sonar_x;
static float last_sonar_y;
static float last_sonar_orientation;
static int   last_sonar_def = 0;

void SONAR_sonar_reply_handler(TCX_REF_PTR           ref,
			       SONAR_sonar_reply_ptr sonar)
{
  int i, j;
  struct timeval this_message_time;
  time_t current_time;
  struct timeval t; 
  char out_text[256];
  float wall_angle; 
  MAP_saw_a_wall_type wall_msg;
  float distance;

#ifdef SONARINT_debug
  fprintf(stderr, "TCX: Received a SONAR_sonar_reply message.\n");
  for (i = 0; i < robot_specifications->num_sensors; i++)
    fprintf(stderr, " %5.2f", sonar->values[i]);
  fprintf(stderr, "\n");
#endif
  
  /*
   * LOGGING DATA
   */

  if (ref != NULL &&	/* NULL indicates internal call, no TCX */
      log_iop != NULL){	/* NULL: No logging */
    time(&current_time);
    strftime (out_text, 50, "\n@SENS %d-%m-%y %H:%M:%S",
	      localtime(&current_time));
    gettimeofday (&t,NULL);
    fprintf(log_iop, "%s.%d\n", out_text, t.tv_usec);
    fprintf(log_iop,"#SONAR %d:", robot_specifications->num_sensors);
    for (i = 0; i < robot_specifications->num_sensors; i++)
      fprintf(log_iop, " %f", sonar->values[i]);
    fprintf(log_iop, "\n\n");
  }
  
  
  /*
   * When processing a script file, sonar information received
   * via TCX will be ignored
   */

      

  if (!program_state->processing_script || ref == NULL){

    
    /* 
     * Print out a message of how long a sensor reading took
     */
    
#ifdef SONARINT_debug
    gettimeofday(&this_message_time, NULL);
    if (last_message_time.tv_sec != 0)
      printf("Sonar reading took %d msec\n", (int)
	     ((this_message_time.tv_sec - last_message_time.tv_sec) * 1000
	      + (this_message_time.tv_usec - 
		 last_message_time.tv_usec) / 1000));
    last_message_time.tv_sec  = this_message_time.tv_sec;
    last_message_time.tv_usec = this_message_time.tv_usec;
#endif
    /*
       diff_orientation = last_sonar_orientation - robot_state->orientation;
       for (;diff_orientation < -180.0;) diff_orientation += 360.0;
       for (;diff_orientation > 180.0;)  diff_orientation -= 360.0;
       */

    distance = sqrt(((last_sonar_x - robot_state->x)
		     * (last_sonar_x - robot_state->x))
		    + ((last_sonar_y - robot_state->y)
		       * (last_sonar_y - robot_state->y)));

    
    /* *******************************************************************
     *
     * Take this reading if (and only if)
     *   - the robot position is known
     *   - The robot is moving
     *
     * Otherwise, we'd better drop it.
     */
    
    if (robot_state->known && program_state->maps_allocated &&
	distance >= 
	robot_specifications->min_advancement_between_interpretations &&
	(robot_state->translational_speed >= 0.1 ||
	 robot_state->rotational_speed >= 0.1)){ /* move, robot move! */
      
      last_sonar_x = robot_state->x;
      last_sonar_y = robot_state->y;
      last_sonar_orientation = robot_state->orientation;
      last_sonar_def = 1;

      
      
      /* 
       * Copy the sensor values into memory. Special care has been taken
       * for corrupted sensor information (-1)
       */
      
#ifdef RHINO_PLUS
      if (selector) {		/* neural network */
#endif
      
	/*================================*\
	 *========= NEURAL NET ===========*
	 \*================================*/

	for (i = 0, j = 0; i < robot_specifications->num_sensors; i++, j--){
	  robot_state->raw_sensor_values[i] = sonar->values[i];
	  if (j < 0)
	    j += robot_specifications->num_sensors;
	  if (sonar->values[i] < 0.0)
	    robot_state->sensor_values[j] = robot_specifications->robot_size;
	  else if (sonar->values[i] > robot_specifications->max_sensors_range - 
		   robot_specifications->robot_size ||
		   sonar->values[i] < robot_specifications->min_sensors_range)
	    robot_state->sensor_values[j] =
	      robot_specifications->max_sensors_range;
	  else
	    robot_state->sensor_values[j] = sonar->values[i] + 
	      robot_specifications->robot_size;
	}

#ifdef RHINO_PLUS
      }

      else {			/* rhino stuff start */

	/*================================*\
	 *============ RHINO+ ============*
	 \*================================*/
      
	rhino_sensor		/* rhino stuff without ; */
	} /* rhino stuff end */
#endif
    
      /* 
       * Constructs a "local" map from these reading only, using
       * neural networks or Frank's ultimate method!
       */
    
      compute_local_map(neural_network, robot_specifications, program_state,
			robot_state);
      /*
       * smoothes the local map 
       */

      smooth_local_map(robot_specifications);

      clip_local_map(robot_specifications);
      /*
       * Compute the next wall
       */
      
      if (angle_to_wall(robot_state->sensor_values,
			robot_specifications->sensor_angles,
			&wall_angle)){
#ifdef SONARINT_debug
	fprintf(stderr, "\t--> Found a wall, orientation:%6.2f (%g)\n",
	   wall_angle, wall_angle + robot_state->orientation);  
#endif

	wall_msg.robot_x           = robot_state->x;
	wall_msg.robot_y           = robot_state->y;
	wall_msg.robot_orientation = robot_state->orientation;
	wall_msg.wall_angle        = wall_angle;

	if (program_state->tcx_initialized &&
	    program_state->map_connected)
	  tcxSendMsg(MAP, "MAP_saw_a_wall", &wall_msg);
      }

      /*
       * Now that we have the local map, let's communicate it to MAP
       */
     
      broadcast_local_map_to_MAP(robot_specifications, program_state,
				 robot_state);


      /* 
       * Now, finally, the new sensor values are displayed.
       */

      if (program_state->graphics_initialized){
	G_display_switch(LOCAL_BACKGROUND, 0);
	G_display_matrix(LOCAL_MAPVALUES);
	G_display_robot(LOCAL_ROBOT_, 0.0, 0.0, 90.0,
			robot_specifications->num_sensors, 
			robot_state->sensor_values);
	G_display_markers(REGRESSION);
      }

      
      new_sonar_reading = 1;
    
    }
    else
      new_sonar_reading = 0;
    
    
    /* 
     * For the user's convenience we print out how long the whole action
     * took.
     */
    
    gettimeofday(&this_message_time, NULL);
#ifdef SONARINT_debug
    if (last_message_time.tv_sec != 0)
      printf("Map update took %d msec\n", (int)
	     ((this_message_time.tv_sec - last_message_time.tv_sec) * 1000
	      + (this_message_time.tv_usec - 
		 last_message_time.tv_usec) / 1000));
#endif
  }    

  /* 
   * Very Important: Free Memory.
   */
  
  if (ref != NULL)		/* NULL indicates internal call, no TCX */
    tcxFree("SONAR_sonar_reply", sonar);
}




/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         LASER_laser_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void LASER_laser_reply_handler(TCX_REF_PTR           ref,
			       LASER_laser_reply_ptr laser)
{
  int i, j;
  time_t current_time;
  struct timeval t; 
  char out_text[256];

#ifdef SONARINT_debug
  fprintf(stderr, "TCX: Received a LASER_laser_reply message.\n");
  for (i = 0; i < 360; i++){
    j = i % (robot_specifications->num_sensors / 2);
    if (i < robot_specifications->num_sensors / 2){
      if (laser->f_reading != NULL)
	fprintf(stderr, " %d", laser->f_reading[i]);
      else
	fprintf(stderr, " -1");
    }
    else{
      if (laser->r_reading != NULL)
	fprintf(stderr, " %d", laser->r_reading[j]);
      else
	fprintf(stderr, " -1");
    }
  }
  fprintf(stderr, "\n");
#endif
  
  /*
   * LOGGING DATA
   */

  if (ref != NULL &&	/* NULL indicates internal call, no TCX */
      log_iop != NULL){	/* NULL: No logging */
    time(&current_time);
    strftime (out_text, 50, "\n@SENS %d-%m-%y %H:%M:%S",
	      localtime(&current_time));
    gettimeofday (&t,NULL);
    fprintf(log_iop, "%s.%d\n", out_text, t.tv_usec);
    fprintf(log_iop,"#LASER %d %d:", laser->f_numberOfReadings, laser->r_numberOfReadings);
    for (i = 0; i < 360; i++){
      j = i % (360 / 2);
      if (i < 360 / 2){
	if (laser->f_reading != NULL)
	  fprintf(log_iop, " %d", laser->f_reading[j]);
	else
	  fprintf(log_iop, " -1");
      }
      else{
	if (laser->r_reading != NULL)
	  fprintf(log_iop, " %d", laser->r_reading[j]);
	else
	  fprintf(log_iop, " -1");
      }
    }
    fprintf(log_iop, "\n\n");
  }
  
  /* 
   * Very Important: Free Memory.
   */
  
  if (ref != NULL)		/* NULL indicates internal call, no TCX */
    tcxFree("LASER_laser_reply", laser);
}




/************************************************************************
 *
 *   NAME:         SONARINT_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SONARINT_close_handler(char *name, TCX_MODULE_PTR module)
{
#ifdef SONARINT_debug
  fprintf(stderr, "SONARINT: closed connection detected: %s\n", name);
#endif
  
  
  if (!strcmp(name, "MAP")){ /* MAP shut down */
    if (program_state->graphics_initialized)
      G_display_switch(MAP_CONNECTED_BUTTON, 0);
    program_state->map_connected = 0;
    close_script(robot_state, program_state, robot_specifications);
  }
  else if (!strcmp(name, "BASE")){ /* BASE shut down */
    if (program_state->graphics_initialized)
      G_display_switch(BASE_CONNECTED_BUTTON, 0);
    program_state->base_connected = 0;
  }
  else if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    exit(0);
  }
}



/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         broadcast_local_map_to_MAP
 *                 
 *   FUNCTION:     sends a partial local map to MAP
 *                 
 *   PARAMETERS:   as below
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void broadcast_local_map_to_MAP(ROBOT_SPECIFICATIONS_PTR robot_specifications,
				PROGRAM_STATE_PTR        program_state,
				ROBOT_STATE_PTR          robot_state)
{
  MAP_sensor_interpretation_type local; 
  int i, j;

  /*
   * Did we ever allocate maps? If not, something weird is going on.
   * Try to prevent the worst and quit.
   */
  
  if (!program_state->maps_allocated)
    return;

  /*
   * Are we connected to map? If not, let's try to establish a connection
   */
  
  if(!program_state->map_connected)
    connect_to_MAP(program_state);

  /*
   * Okay, if we are not connected by now, we'd give up the idea of 
   * broadcasting our local map altogether.
   */

  if(!program_state->map_connected)
    return;

  /*
   * Where was the robot when the sensor  reading was taken?
   * Update these values by status report! 
   */
  

  local.robot_x             = robot_state->x;
  local.robot_y             = robot_state->y;
  local.robot_orientation   = robot_state->orientation;
  local.translational_speed = robot_state->translational_speed;
  local.rotational_speed    = robot_state->rotational_speed;

  if (robot_specifications->broadcast_sensor_data_to_map){
    local.num_sensor_values_enclosed = robot_specifications->num_sensors;
    local.sensor_values              = robot_state->raw_sensor_values;
  }
  else{
    local.num_sensor_values_enclosed = 0;
    local.sensor_values              = NULL;
  }


  /*
   * Where is the origin of the local map in this message with respect to the
   * robot? Assume that the robot is at (0,0) and facing into the direction
   * of the x-axis. What is the coordinate of the pixel values[0][0]?,
   * and what is the angle of the x-coordinate in values[] in the robot's
   * local coordinate system?
   */

  local.origin_x           = robot_specifications->max_sensors_range;
  local.origin_y           = robot_specifications->max_sensors_range; 
  local.origin_orientation = 90.0;

  local.resolution         = robot_specifications->resolution;

  local.size_x             = robot_specifications->local_map_dim_x;
  local.size_y             = robot_specifications->local_map_dim_y;

  local.delete_previous_map = 0;
  local.map_number         = SONARINT_MAP_NUMBER;
     
  local.char_likelihoods   = (unsigned char *)
    malloc(sizeof(float) * local.size_x * local.size_y);

  for (i = 0; i < local.size_x * local.size_y; i++)
    if (local_active[i]){
      if (local_robot[i])	/* everything under the robot is free! */
	local.char_likelihoods[i] = (unsigned char) 255;
      else if (local_map[i] >= 1.0)
	local.char_likelihoods[i] = (unsigned char) 254;
      else if (local_map[i] <= 0.0)
	local.char_likelihoods[i] = (unsigned char) 1;
      else
	local.char_likelihoods[i] = ((unsigned char) 
				     (local_map[i] * 253.0)) + 1;
    }
    else
      local.char_likelihoods[i] = 0; /* passive! */
  
  /*
   * for (i = 0; i < local.size_x * local.size_y; i++)
   * fprintf(stderr, " %d", (int) local.char_likelihoods[i]);
   */

#ifdef SONARINT_debug
  fprintf(stderr, "TCX: MAP_sensor_interpretation\n");
#endif

  /* 
   * send the message
   */
  

  if (program_state->map_connected)
    tcxSendMsg(MAP, "MAP_sensor_interpretation", &local);

  free(local.char_likelihoods);
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


static struct timeval last_attempt_connect_BASE = {0, 0};


/************************************************************************
 *
 *   NAME:         connect_to_BASE
 *                 
 *   FUNCTION:     checks, and connects to BASE, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void connect_to_BASE(PROGRAM_STATE_PTR program_state)
{
  BASE_register_auto_update_type data;
  struct timeval current_time;

  if (!program_state->use_tcx)
    return;


  if(!program_state->base_connected){

    gettimeofday(&current_time, NULL);
    if (current_time.tv_sec < last_attempt_connect_BASE.tv_sec + 5
	|| (current_time.tv_sec == last_attempt_connect_BASE.tv_sec + 5 &&
	    current_time.tv_usec < last_attempt_connect_BASE.tv_usec))
      return;
    
    last_attempt_connect_BASE.tv_sec  = current_time.tv_sec;
    last_attempt_connect_BASE.tv_usec = current_time.tv_usec;
    

    BASE = tcxConnectOptional(TCX_BASE_MODULE_NAME); /* checks, but does 
						      * not wait */

    if (BASE != NULL){
      if (program_state->graphics_initialized)
	G_display_switch(BASE_CONNECTED_BUTTON, 1);

      data.subscribe_status_report = 1;
      data.subscribe_sonar_report  = 1;
      data.subscribe_colli_report  = 0;
      data.subscribe_laser_report  = program_state->logging_on;
      data.subscribe_ir_report  = 0;
      
      tcxSendMsg(BASE, "BASE_register_auto_update", &data);

      program_state->base_connected = 1;
    }
    else if (program_state->graphics_initialized)
      G_display_switch(BASE_CONNECTED_BUTTON, 0);
  }
}




/************************************************************************
 *
 *   NAME:         connect_to_MAP
 *                 
 *   FUNCTION:     checks, and connects to MAP, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

static struct timeval last_attempt_connect_MAP = {0, 0};

void connect_to_MAP(PROGRAM_STATE_PTR program_state)
{
  MAP_register_auto_update_type data;
  struct timeval current_time;

  if (!program_state->use_tcx)
    return;


  if(!program_state->map_connected){

    gettimeofday(&current_time, NULL);
    if (current_time.tv_sec < last_attempt_connect_MAP.tv_sec + 5
	|| (current_time.tv_sec == last_attempt_connect_MAP.tv_sec + 5 &&
	    current_time.tv_usec < last_attempt_connect_MAP.tv_usec))
      return;
    
    last_attempt_connect_MAP.tv_sec  = current_time.tv_sec;
    last_attempt_connect_MAP.tv_usec = current_time.tv_usec;
    

    MAP = tcxConnectOptional(TCX_MAP_MODULE_NAME); /* checks, but does 
						      * not wait */

    if (MAP != NULL){
      if (program_state->graphics_initialized)
	G_display_switch(MAP_CONNECTED_BUTTON, 1);

      program_state->map_connected = 1;
    }
    else if (program_state->graphics_initialized)
      G_display_switch(MAP_CONNECTED_BUTTON, 0);
  }
}






/************************************************************************
 *
 *   NAME:         init_tcx
 *                 
 *   FUNCTION:     connects to tcx and the two clients.
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/






void init_tcx(PROGRAM_STATE_PTR program_state)
{
  const char *tcxMachine = NULL;
  
  char tcxName[128], hostname[80], pidname[80];
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    BASE_messages,
    SONAR_messages,
    LASER_messages,
    MAP_messages
    };
  
  if (!program_state->use_tcx)
    return;

  fprintf(stderr, "Connecting to TCX...");
  fflush(stderr);
  
  
  /*
   * Build a special, unique name that takes host machine and process number
   * into account.
   */

  /* strcpy(tcxName, TCX_SONARINT_MODULE_NAME);
     sprintf(pidname, "_%d_", getpid());
     strcat(tcxName, pidname);
     gethostname(hostname, 32);
     strcat(tcxName, hostname); 
     tcxInitialize(tcxName, (void * ) getenv("TCXHOST")); */
  
  


  
  /*
   * Initialize TCX, register yourself.
   */
  
  /* ====== INITIALIZING TCX ============ */
#if 0
  tcxMachine = bParametersGetParam(bParamList,
				   "", 
				   "TCXHOST");
#else
  tcxMachine = bRobot.TCXHOST;
#endif

  if (tcxMachine != NULL){

  
    printf("Initializing TCX...");
    fflush(stdout);
    tcxInitialize(TCX_SONARINT_MODULE_NAME, (char *)tcxMachine);
    check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
			 BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
			 NULL, 0);
    check_version_number(librobot_major, librobot_minor,
			 librobot_robot_type, librobot_date,
			 "librobot", 0);
    check_version_number(libezx_major, libezx_minor,
			 libezx_robot_type, libezx_date,
			 "libezx", 0);
    check_version_number(libbUtils_major, libbUtils_minor,
			 libbUtils_robot_type, libbUtils_date,
			 "libbUtils", 1);




    fprintf(stderr, "done.\n");
    fflush(stderr);
    
    
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
    
    tcxRegisterCloseHnd(SONARINT_close_handler);
    
    connect_to_BASE(program_state);
    
    connect_to_MAP(program_state);
    
    program_state->tcx_initialized = 1;
  }
  else{
    fprintf(stderr, "Error in TCX: TCXHOST not set appropriately\n");
  }
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/**** the following handlers won't be used! *****/

void BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				       BASE_robot_position_reply_ptr pos){
  ;
}

void BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
					BASE_action_executed_reply_ptr data)

{
;
}

