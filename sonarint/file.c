
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

#include <signal.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <sys/time.h>
#include "tcx.h"
#include "tcxP.h"
#include "bUtils.h"
#include "Application.h"
#include "Net.h"
#include "MAP-messages.h"
#include "SONAR-messages.h"
#include "BASE-messages.h"
#include "SONARINT.h"
#include "EZX11.h"
#include "o-graphics.h"


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/*** 
 *** These variables are used to communicate parameters when reading 
 *** script files
 ***/

static FILE *script_iop = NULL;
static char script_filename[256];
       FILE *log_iop = NULL;
static char log_filename[256];
static int robot_pos_defined = 0;
#ifndef VMS
static struct timeval read_begin_time;
#endif

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/************************************************************************
 *
 *   NAME:         save_parameters()
 *                 
 *   FUNCTION:     saves a map into a file. Map-intrinsic format.
 *                 
 *   PARAMETERS:   filename
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


int save_parameters(char *filename)
{
  int i, x, y, index;
  FILE *iop;
  char filename2[256];
  

  sprintf(filename2, "%s", filename);
  if ((iop = fopen(filename2, "w")) == 0){
    fprintf(stderr, "WARNING: Could not open output file %s.\n", filename2);
    sprintf(filename2, "../etc/%s", filename);
    if ((iop = fopen(filename2, "w")) == 0){
      fprintf(stderr, "WARNING: Could not open output file %s.\n", filename2);
      sprintf(filename2, "../../etc/%s", filename);
      if ((iop = fopen(filename2, "w")) == 0){
	fprintf(stderr, 
		"WARNING: Could not open output file %s. Not saved.\n",
		filename2);
	return 0;
      }
    }
  }



  fprintf(iop, "robot_specifications->local_mapsize_x                  %g\n",
	  robot_specifications->local_mapsize_x);
  fprintf(iop, "robot_specifications->local_mapsize_y                  %g\n",
	  robot_specifications->local_mapsize_y);
  fprintf(iop, "robot_specifications->resolution                       %g\n",
	  robot_specifications->resolution);
  fprintf(iop, "robot_specifications->robot_size                       %g\n",
	  robot_specifications->robot_size);
  fprintf(iop, "robot_specifications->smooth_radius                    %d\n",
	  robot_specifications->smooth_radius);

#ifdef UNIBONN
  fprintf(iop, "robot_specifications->num_sensors                      %d\n",
	  robot_specifications->num_sensors);
  fprintf(iop, "robot_specifications->first_sensor_angle               %g\n",
	  robot_specifications->first_sensor_angle);
#endif /*UNIBONN*/
  fprintf(iop, "robot_specifications->max_sensors_range                %g\n",
	  robot_specifications->max_sensors_range);
  fprintf(iop, "robot_specifications->min_sensors_range                %g\n",
	  robot_specifications->min_sensors_range);
  fprintf(iop, "robot_specifications->neuronet_max_sensors_range       %g\n",
	  robot_specifications->neuronet_max_sensors_range);
  fprintf(iop, "robot_specifications->neuronet_min_sensors_range       %g\n",
	  robot_specifications->neuronet_min_sensors_range);
  fprintf(iop, "robot_specifications->max_occupied_sensors_range       %g\n",
	  robot_specifications->max_occupied_sensors_range);
  fprintf(iop, "robot_specifications->occupied_outer_width             %g\n",
	  robot_specifications->occupied_outer_width);
  fprintf(iop, "robot_specifications->network_occupied_value           %g\n",
	  robot_specifications->network_occupied_value);

  fprintf(iop, "robot_specifications->network_value_mean               %g\n",
	  robot_specifications->network_value_mean);
  fprintf(iop, "robot_specifications->decay_with_distance              %g\n",
	  robot_specifications->decay_with_distance);

  fprintf(iop, "robot_specifications->line_recognition_neighbors       %d\n",
	  robot_specifications->line_recognition_neighbors);
  fprintf(iop, "robot_specifications->line_recognition_threshold       %g\n",
	  robot_specifications->line_recognition_threshold);
  fprintf(iop, "robot_specifications->min_advancement_between_interpretations %g\n",
	  robot_specifications->min_advancement_between_interpretations);
  fprintf(iop, "robot_specifications->broadcast_sensor_data_to_map     %d\n",
	  robot_specifications->broadcast_sensor_data_to_map);
  fprintf(iop, "program_state->delay_in_replay                         %g\n",
	  program_state->delay_in_replay);

  fclose(iop);

  fprintf(stderr, "File %s successfully written.\n", filename2);

  return 1;
}



/************************************************************************
 *
 *   NAME:         load_parameters()
 *                 
 *   FUNCTION:     loads a map from a file. Map-intrinsic format.
 *                 
 *   PARAMETERS:   filename
 *                 init        1, if MAP is initialized
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/


int load_parameters(char *filename, int init)
{
  int i, x, y, index, int_value, int_value1, int_value2;
  float float_value;
  FILE *iop;
  int file_ended, error;
  char command[256];

  char filename2[256];
  char *filename3;

  sprintf(filename2, "etc/%s", filename);

  filename3 = bFindFileM(filename2);
  
  if ((iop = fopen(filename3, "r")) == 0){
    fprintf(stderr, "Could not open input file %s. File not loaded.\n",
            filename3);
    fprintf(stderr, "WARNING: Failed to read file %s.\n", filename);
    free(filename3);
    return 0;
  }

  free(filename3);

  file_ended = 0;
  error = 0;
  do{
    if (fscanf(iop, "%s", command) == EOF)
      file_ended = 1;
    
    else{
      
      if (!strcmp(command, "end"))
	file_ended = 2;
      
      else if (command[0] == '#'){ /* comment */
	if (!fgets(command,sizeof(command),iop))
	  file_ended = 2;	/* file may end with hash-sign /  comment */
	}


      else if (!strcmp(command, "robot_specifications->local_mapsize_x")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init){
	  robot_specifications->local_mapsize_x = float_value;
	  robot_specifications->local_map_dim_x   =
	    1 + (int) /* 1=safety margin */
	    (robot_specifications->local_mapsize_x 
	     / robot_specifications->resolution);
	}
	else if (float_value != robot_specifications->local_mapsize_x){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->local_mapsize_x");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->local_mapsize_y")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init){
	  robot_specifications->local_mapsize_y = float_value;
	  robot_specifications->local_map_dim_y   =
	    1 + (int) /* 1=safety margin */
	    (robot_specifications->local_mapsize_y 
	     / robot_specifications->resolution);
	}
	else if (float_value != robot_specifications->local_mapsize_y){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->local_mapsize_y");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      
      
      else if (!strcmp(command, "robot_specifications->resolution")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init){
	  robot_specifications->resolution = float_value;
	  robot_specifications->local_map_dim_x   =
	    1 + (int) /* 1=safety margin */
	    (robot_specifications->local_mapsize_x 
	     / robot_specifications->resolution);
	  robot_specifications->local_map_dim_y   =
	    1 + (int) /* 1=safety margin */
	    (robot_specifications->local_mapsize_y 
	     / robot_specifications->resolution);
	}
	else if (float_value != robot_specifications->resolution){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->resolution");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->robot_size")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->robot_size = float_value;
	else if (float_value != robot_specifications->robot_size){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->robot_size");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->smooth_radius")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->smooth_radius = int_value;
	else if (int_value != robot_specifications->smooth_radius){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->smooth_radius");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
#ifdef UNIBONN

      else if (!strcmp(command, "robot_specifications->num_sensors")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->num_sensors = int_value;
	else if (int_value != robot_specifications->num_sensors){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->num_sensors");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }

      
      
      else if (!strcmp(command, "robot_specifications->first_sensor_angle")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->first_sensor_angle = float_value;
	else if (float_value != robot_specifications->first_sensor_angle){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->first_sensor_angle");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
#endif /*UNIBONN*/      
      else if (!strcmp(command, "robot_specifications->max_sensors_range")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_sensors_range = float_value;
	else if (float_value != robot_specifications->max_sensors_range){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->max_sensors_range");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->min_sensors_range")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->min_sensors_range = float_value;
	else if (float_value != robot_specifications->min_sensors_range){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->min_sensors_range");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      

      else if (!strcmp(command, "robot_specifications->neuronet_max_sensors_range")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->neuronet_max_sensors_range = float_value;
	else if (float_value != robot_specifications->neuronet_max_sensors_range){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->neuronet_max_sensors_range");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->neuronet_min_sensors_range")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->neuronet_min_sensors_range = float_value;
	else if (float_value != robot_specifications->neuronet_min_sensors_range){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->neuronet_min_sensors_range");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
            
      
      else if (!strcmp(command,
		       "robot_specifications->max_occupied_sensors_range")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_occupied_sensors_range = float_value;
	else if (float_value != 
		 robot_specifications->max_occupied_sensors_range){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->max_occupied_sensors_range");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command,
		       "robot_specifications->occupied_outer_width")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->occupied_outer_width = float_value;
	else if (float_value != 
		 robot_specifications->occupied_outer_width){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->occupied_outer_width");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      
      else if (!strcmp(command,
		       "robot_specifications->network_occupied_value")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->network_occupied_value = float_value;
	else if (float_value != 
		 robot_specifications->network_occupied_value){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->network_occupied_value");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      

      else if (!strcmp(command, "robot_specifications->network_value_mean")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->network_value_mean = float_value;
      }


      else if (!strcmp(command, "robot_specifications->decay_with_distance")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->decay_with_distance = float_value;
      }


      else if (!strcmp(command, 
		       "robot_specifications->line_recognition_neighbors")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->line_recognition_neighbors = int_value;
      }

      else if (!strcmp(command,
		       "robot_specifications->line_recognition_threshold")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->line_recognition_threshold = float_value;
      }


      else if (!strcmp(command,
		       "robot_specifications->min_advancement_between_interpretations")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->min_advancement_between_interpretations
	    = float_value;
      }


      else if (!strcmp(command, 
		       "robot_specifications->broadcast_sensor_data_to_map")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->broadcast_sensor_data_to_map = int_value;
      }


      else if (!strcmp(command, "program_state->delay_in_replay")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  program_state->delay_in_replay = float_value;
      }

      
      else{
	fprintf(stderr, "ERROR: Unknown keyword \"%s\" in %s. Must exit.\n ", 
		command, filename2);
	error = 1;
      }
 
      if (file_ended == 1)
	fprintf(iop, "Surprising end of file %s.\n ", filename2);
      
    }
  } while (!file_ended);
  fclose(iop);
  
  if (!error)
    fprintf(stderr, "File %s successfully read.\n", filename2);

  return (1 - error);
}


/************************************************************************
 *
 *   NAME:         initiate_read_script()
 *                 
 *   FUNCTION:     opens a script file. Clears all maps.
 *                 
 *   PARAMETERS:   filename
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/



int initiate_read_script(ROBOT_STATE_PTR    robot_state,
			 PROGRAM_STATE_PTR  program_state,
			 ROBOT_SPECIFICATIONS_PTR robot_specifications,
			 char *filename)
{
  if (program_state->graphics_initialized)
    G_display_switch(SCRIPT_BUTTON, 1);

  if ((script_iop = fopen(filename, "r")) == 0){
    fprintf(stderr, "WARNING: Could not open script file %s.\n", filename);
    if (program_state->graphics_initialized)
      G_display_switch(SCRIPT_BUTTON, 0);
    script_iop = NULL;
    return 0;
  }

  strcpy(script_filename, filename);
  clear_all_maps(robot_state, program_state, robot_specifications);
  program_state->processing_script = 1;
  program_state->read_next_script_event = 1;
  robot_pos_defined = 0;
#ifndef VMS
  gettimeofday(&read_begin_time, NULL);
#endif
  return 1;
}





#ifndef VMS  /* seems to be no way to support ualarm() with VMS */

/************************************************************************
 *
 *   NAME:         alarm_handler()
 *                 
 *   FUNCTION:     opens a script file. Clears all maps.
 *                 
 *   PARAMETERS:   filename
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/




void alarm_handler(int sig)
{
  ualarm(0, 0);
  gettimeofday(&read_begin_time, NULL);
  program_state->read_next_script_event = 1;
  signal(SIGALRM, alarm_handler);
  return;
}
#endif


/************************************************************************
 *
 *   NAME:         read_script()
 *                 
 *   FUNCTION:     opens a script file. Clears all maps.
 *                 
 *   PARAMETERS:   filename
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/

static int   last_reading_defined = 0;
static int   this_reading_defined = 0;
static float this_reading_time = 0.0;
static float last_reading_time = 0.0;
static float last_reading_x = 0.0;
static float last_reading_y = 0.0;
static int   velocity_defined = 0;
static float min_velocity = 0.0;
static float max_velocity = 0.0;
static float cumul_velocity = 0.0;
static float cumul_time = 0.0;

int read_script(ROBOT_STATE_PTR    robot_state,
		PROGRAM_STATE_PTR  program_state,
		ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int i, j, int_value;
  int int_value1, int_value2;
  float float_value, float_value1, float_value2, float_value3;
  int file_ended, error;
  char command[256];
  static BASE_update_status_reply_type status;
  static SONAR_sonar_reply_type sonar;
  struct timeval read_end_time;
  int print_flag = 0;
  int done_for_now;
  long int sleep_duration;
  float velocity, velocity2; 

  int new_format = 0;
  int reading_pattern = 0;
  int laser_data_found = 0;
  int sonar_data_found = 0;
  int position_found = 0;

#ifdef SONARINT_debug
  print_flag = 1;
#endif

#ifndef VMS
  ualarm(0, 0);
  program_state->read_next_script_event = 0;
#endif

  file_ended = 0;
  error = 0;
  done_for_now = 0;


  do{
    if (script_iop == NULL || fscanf(script_iop, "%s", command) == EOF)
      file_ended = 1;
    else if (!strcmp(command, "#END"))
      file_ended = 1;
    
    else{
      
      
      /*----------------------------------------------*\
       *        begin(patternset)
       *----------------------------------------------*/
      
      if (!strcmp(command, "begin(patternset)")){
	new_format = 1;
      }

      /*----------------------------------------------*\
       *        begin(pattern)
       *----------------------------------------------*/

      else if (new_format && !strcmp(command, "begin(pattern)")){
	reading_pattern = 1;
	laser_data_found = 0;
	sonar_data_found = 0;
	position_found = 0;
      }


      /*----------------------------------------------*\
       *        lasers
       *----------------------------------------------*/

      else if (new_format && reading_pattern &&
	       !strcmp(command, "lasers")){
	if (fscanf(script_iop, "%d %d :", &int_value1, &int_value2) == EOF)
	  file_ended = 1;
	else{
	  /*
	    if (int_value1 + int_value2 > robot_specifications->num_sensors){
	    fprintf(stderr, 
	    "ERROR: Format mismatch. Too many laser values (%d).\n",
	    int_value1 + int_value2);
	    error = 1;
	    }
	  else */{
	    for (i = 0; i < int_value1; i++)
	      if (fscanf(script_iop, "%f", &float_value) == EOF)
		file_ended = 1;
	    /* else
	       laser.f_reading[i] = (int) float_value;*/
	    for (i = 0; i < int_value2; i++)
	      if (fscanf(script_iop, "%f", &float_value) == EOF)
		file_ended = 1;
	    /* else
	       laser.r_reading[i] = (int) float_value; */
	  }
	  if (!file_ended)
	    laser_data_found = 1;
	}
      }

      /*----------------------------------------------*\
       *        sonars
       *----------------------------------------------*/

    
      else if (new_format && reading_pattern &&
	       !strcmp(command, "sonars")){
	if (fscanf(script_iop, "%d :", &int_value) == EOF)
	  file_ended = 1;
	else{
	  if (int_value != robot_specifications->num_sensors){
	    fprintf(stderr, 
		    "ERROR: Format mismatch. Wrong number of sonar values (%d).\n",
		    int_value);
	    error = 1;
	  }
	  else{
	    for (i = 0, j = robot_specifications->num_sensors / 2;
		 i < int_value;
		 i++, j = (j+1) % robot_specifications->num_sensors)
	      if (fscanf(script_iop, "%f", &float_value) == EOF)
		file_ended = 1;
	      else if (i < bRobot.sonar_cols[0]){
		/* current max. value on SONAR-messages.h */
		sonar.values[j] = float_value;
	      }
	      else
		fprintf(stderr, " overflow");
	  }
	  if (!file_ended)
	    sonar_data_found = 1;
	}
      }


      /*----------------------------------------------*\
       *        position:
       *----------------------------------------------*/

      else if (new_format && reading_pattern &&
	       !strcmp(command, "position:")){

	if (fscanf(script_iop, "%f %f %f", &float_value1, &float_value2, 
		   &float_value3) == EOF)
	  file_ended = 1;
	else{

	  float_value3 = 90.0 - float_value3; /* correction for different
					       * ideas of what an angle is */

	  if (!robot_pos_defined){
	    status.trans_current_speed = status.rot_current_speed = 0.0;
	  }
	  else{
	    status.trans_current_speed = 
	      0.5 * sqrt(((status.pos_x - float_value1) 
			  * (status.pos_x - float_value1)) 
			 + ((status.pos_y - float_value2) 
			    * (status.pos_y - float_value2)));
	    status.rot_current_speed = fabs(status.orientation
					    - float_value3);
	    if (status.rot_current_speed >= 180.0)
	      status.rot_current_speed = 360.0 - status.rot_current_speed;
	  }
	  status.pos_x       = float_value1;
	  status.pos_y       = float_value2;
	  status.orientation = float_value3;
	  
	  if (print_flag) 
	    printf("#ROBOT %f %f %f (%f %f)\n",
		   float_value1, float_value2, float_value3,
		   status.trans_current_speed, status.rot_current_speed);
	  BASE_update_status_reply_handler(NULL, &status);
	  robot_pos_defined = 1;

	  /*
	   * statistics - velocity in the script file
	   */

	  if (this_reading_defined){
	    if (last_reading_defined && 
		last_reading_time < this_reading_time){
	      velocity2 = sqrt(((last_reading_x - status.pos_x) * 
				(last_reading_x - status.pos_x)) +
			       ((last_reading_y - status.pos_y) * 
				(last_reading_y - status.pos_y)));
	      velocity = velocity2 / (this_reading_time - last_reading_time);
	      if (!velocity_defined){
		velocity_defined = 1;
		min_velocity = max_velocity = velocity;
	      }
	      else{
		if (velocity < min_velocity)
		  min_velocity = velocity;
		if (velocity > max_velocity)
		  max_velocity = velocity;
	      }
	      cumul_velocity += velocity2;
	      cumul_time += (this_reading_time - last_reading_time);
	      
	      if (print_flag)
		printf("\tVelocity: current=%g min=%g max=%g avg=%g dist=%g\n",
		       velocity,
		       min_velocity, max_velocity, cumul_velocity /
		       cumul_time, cumul_velocity);
	      
	    }
	    
	    last_reading_defined = 1;
	    last_reading_time = this_reading_time;
	    this_reading_defined = 0;
	    last_reading_x = status.pos_x;
	    last_reading_y = status.pos_y;
	  }
	}
	position_found = 1;
      }


      /*----------------------------------------------*\
       *        end(pattern)
       *----------------------------------------------*/

      else if (new_format && reading_pattern &&
	       !strcmp(command, "end(pattern)")){
	if (position_found && laser_data_found){
	  /* fprintf(stderr, "L"); */
	  /* LASER_laser_reply_handler(NULL, &laser); */
	}
	if (position_found && sonar_data_found){
	  /* fprintf(stderr, "S"); */
	  SONAR_sonar_reply_handler(NULL, &sonar);
	}
	reading_pattern = 0;
      }


      /*----------------------------------------------*\
       *++++++ @SENS +++++++++++++++++++++++++++++++++*|
       *----------------------------------------------*/

      else if (!new_format && !strcmp(command, "@SENS")){
	if (fscanf(script_iop, "%f-%f-%f", 
		   &float_value1, &float_value2, &float_value3) == EOF)
	  file_ended = 1;
	else{
	  if (print_flag) 
	    printf("@SENS %g-%g-%g",
		   float_value1, float_value2, float_value3);
	  if (fscanf(script_iop, "%f:%f:%f",
		     &float_value1, &float_value2, &float_value3) == EOF){
	    file_ended = 1;
	    if (print_flag) 
	      printf("\n");
	  }
	  else{
	    if (print_flag) 
	      printf(" %g:%g:%6.3f\n", 
		     float_value1, float_value2, float_value3);
	    this_reading_time = (float_value1 * 3600.0) +
	      (float_value2 * 60.0) + float_value3;
	    this_reading_defined = 1;
	  }
	}
      }

      /*----------------------------------------------*\
       *++++++ @OPEN +++++++++++++++++++++++++++++++++*|
       *----------------------------------------------*/


      else if (!new_format && !strcmp(command, "@OPEN")){
	if (fscanf(script_iop, "%f-%f-%f", 
		   &float_value1, &float_value2, &float_value3) == EOF)
	  file_ended = 1;
	else{
	  if (print_flag) 
	    printf("@OPEN %g-%g-%g",
		   float_value1, float_value2, float_value3);
	  if (fscanf(script_iop, "%f:%f:%f",
		     &float_value1, &float_value2, &float_value3) == EOF){
	    file_ended = 1;
	    if (print_flag) 
	      printf("\n");
	  }
	  else
	    if (print_flag) 
	      printf(" %g:%g:%6.3f\n",
		     float_value1, float_value2, float_value3);
	}
      } 


      /*----------------------------------------------*\
       *++++++ #ROBOT ++++++++++++++++++++++++++++++++*|
       *----------------------------------------------*/


      else if (!new_format && !strcmp(command, "#ROBOT")){
	if (fscanf(script_iop, "%f %f %f %d", &float_value1, &float_value2, 
		   &float_value3, &int_value) == EOF)
	  file_ended = 1;
	else{

	  float_value3 = 90.0 - float_value3; /* correction for different
					       * ideas of what an angle is */

	  if (!robot_pos_defined){
	    status.trans_current_speed = status.rot_current_speed = 0.0;
	  }
	  else{
	    status.trans_current_speed = 
	      0.5 * sqrt(((status.pos_x - float_value1) 
			  * (status.pos_x - float_value1)) 
			 + ((status.pos_y - float_value2) 
			    * (status.pos_y - float_value2)));
	    status.rot_current_speed = fabs(status.orientation
					    - float_value3);
	    if (status.rot_current_speed >= 180.0)
	      status.rot_current_speed = 360.0 - status.rot_current_speed;
	  }
	  status.pos_x       = float_value1;
	  status.pos_y       = float_value2;
	  status.orientation = float_value3;

	  if (print_flag) 
	    printf("#ROBOT %f %f %f (%f %f)\n",
		   float_value1, float_value2, float_value3,
		   status.trans_current_speed, status.rot_current_speed);
	  BASE_update_status_reply_handler(NULL, &status);
	  robot_pos_defined = 1;

	  /*
	   * statistics - velocity in the script file
	   */

	  if (this_reading_defined){
	    if (last_reading_defined && 
		last_reading_time < this_reading_time){
	      velocity2 = sqrt(((last_reading_x - status.pos_x) * 
				(last_reading_x - status.pos_x)) +
			       ((last_reading_y - status.pos_y) * 
				(last_reading_y - status.pos_y)));
	      velocity = velocity2 / (this_reading_time - last_reading_time);
	      if (!velocity_defined){
		velocity_defined = 1;
		min_velocity = max_velocity = velocity;
	      }
	      else{
		if (velocity < min_velocity)
		  min_velocity = velocity;
		if (velocity > max_velocity)
		  max_velocity = velocity;
	      }
	      cumul_velocity += velocity2;
	      cumul_time += (this_reading_time - last_reading_time);
	      
	      if (print_flag)
		printf("\tVelocity: current=%g min=%g max=%g avg=%g dist=%g\n",
		     velocity,
		     min_velocity, max_velocity, cumul_velocity /
		     cumul_time, cumul_velocity);
	      
	    }
	    
	    last_reading_defined = 1;
	    last_reading_time = this_reading_time;
	    this_reading_defined = 0;
	    last_reading_x = status.pos_x;
	    last_reading_y = status.pos_y;
	  }
	}
      } 
      
      /*----------------------------------------------*\
       *++++++ #SONAR ++++++++++++++++++++++++++++++++*|
       *----------------------------------------------*/


      else if (!new_format && !strcmp(command, "#SONAR")){
	if (fscanf(script_iop, "%d:", &int_value) == EOF)
	  file_ended = 1;
	else{
	  if (int_value != robot_specifications->num_sensors){
	    fprintf(stderr, 
		    "ERROR: Format mismatch. Wrong number sonar values.\n");
	    error = 1;
	  }
	  else{
	    if (print_flag) 
	      printf("#SONAR %d:", int_value);
	    for (i = 0; i < int_value && !file_ended; i++){
	      if (fscanf(script_iop, "%f", &float_value) == EOF)
		file_ended = 1;
	      else if (i < bRobot.sonar_cols[0]){
		/* current max. value on SONAR-messages.h */
		sonar.values[i] = float_value;
		if (print_flag) 
		  printf(" %f", float_value);
	      }
	      else
		fprintf(stderr, " overflow");
	    }
	    if (print_flag) 
	      printf("\n");
	    if (!file_ended){
	      SONAR_sonar_reply_handler(NULL, &sonar);
	      if (new_sonar_reading)
		done_for_now = 1;
	    }
	  }
	}
      }
      
      
      /*----------------------------------------------*\
       *++++++ #LASER ++++++++++++++++++++++++++++++++*|
       *----------------------------------------------*/


      else if (!new_format && !strcmp(command, "#LASER")){
	if (fscanf(script_iop, "%d %d:", &int_value1, &int_value2) == EOF)
	  file_ended = 1;
	else{
	  int_value = int_value1 + int_value2;
	  if (int_value != 360 && int_value != 180){
	    fprintf(stderr, 
		    "ERROR: Format mismatch. Wrong number laser values.\n");
	    error = 1;
	  }
	  else{
	    if (print_flag) 
	      printf("#LASER %d:", int_value);
	    for (i = 0; i < int_value && !file_ended; i++){
	      if (fscanf(script_iop, "%f", &float_value) == EOF)
		file_ended = 1;
	      else if (i < 360){	
		if (print_flag) 
		  printf(" %f", float_value);
	      }
	      else
		fprintf(stderr, " overflow");
	    }
	    if (print_flag) 
	      printf("\n");
	  }
	}
      }
      
      
      if (file_ended == 1){
	  fprintf(stderr, "Surprising end of script file %s.\n ", 
		  script_filename);
	error = 1;
      }
      
    }
  } while (!file_ended && !error && !done_for_now);


  if (file_ended || error){
    fclose(script_iop);
    program_state->processing_script      = 0;
    program_state->read_next_script_event = 0;
    if (program_state->graphics_initialized)
      G_display_switch(SCRIPT_BUTTON, 0);
    script_iop = NULL;
    last_reading_defined = 0;
    this_reading_defined = 0;
    velocity_defined = 0;
  }
#ifndef VMS
  else{
    gettimeofday(&read_end_time, NULL);
    sleep_duration  = (long) (program_state->delay_in_replay 
			      * 1000000.0); /* total duration between two 
					     * readings, set by command
					     * line argument and/or file */
    sleep_duration -= read_end_time.tv_usec - read_begin_time.tv_usec;
    sleep_duration -= 1000000  /* subtract time that was spent in this
				* routine */
      * (read_end_time.tv_sec - read_begin_time.tv_sec);
    sleep_duration -= 5000;	/* and that we lost in the computaton here */
    if (sleep_duration > 5000){	/* lminimum sleep time */
      ualarm(sleep_duration, 0);	/* and start laying back! */
    }

    else{
      gettimeofday(&read_begin_time, NULL);
      program_state->read_next_script_event = 1; /* or read immediately! */
    }
  }
#endif
  return (!error);
}




/************************************************************************
 *
 *   NAME:         close_script()
 *                 
 *   FUNCTION:     opens a script file. Clears all maps.
 *                 
 *   PARAMETERS:   filename
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/



void close_script(ROBOT_STATE_PTR    robot_state,
		  PROGRAM_STATE_PTR  program_state,
		  ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
#ifndef VMS
  ualarm(0, 0);
#endif
  if (program_state->processing_script){
    fclose(script_iop);
    program_state->processing_script = 0;
    program_state->read_next_script_event = 0;
    if (program_state->graphics_initialized)
      G_display_switch(SCRIPT_BUTTON, 0);
    script_iop = NULL;
  }
}


/************************************************************************
 *
 *   NAME:         open_log()
 *                 
 *   FUNCTION:     Opens a log file
 *                 
 *   PARAMETERS:   filename
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

int open_log(char *filename)
{
  char filename2[256];
  time_t current_time;
  char out_text[256];
  struct timeval t; 
  
  if (!program_state->logging_on){
    log_iop = NULL;
    fprintf(stderr, "No logging.\n");
    return 0;
  }
  sprintf(filename2, "%s", filename);
  if ((log_iop = fopen(filename2, "a")) == 0){
    fprintf(stderr, "AARNING: Could not open log file %s.\n", filename2);
    sprintf(filename2, "../etc/%s", filename);
    if ((log_iop = fopen(filename2, "a")) == 0){
      fprintf(stderr, "AARNING: Could not open log file %s.\n", filename2);
      sprintf(filename2, "../../etc/%s", filename);
      if ((log_iop = fopen(filename2, "a")) == 0){
	fprintf(stderr, 
		"WARNING: Could not open log file %s. Not saved.\n",
		filename2);
	log_iop = NULL;
	return 0;
      }
    }
  }
  
  
  
  time(&current_time);
  strftime(out_text, 50 , "@OPEN %d-%m-%y %H:%M:%S", 
	   localtime(&current_time));
  fprintf(log_iop, "%s.%d\n\n", out_text, (int) t.tv_usec);

  fprintf(stderr, "Log file %s successfully opened. Logging is on.\n", 
	  filename2);
  return 1;
}


/************************************************************************
 *
 *   NAME:         close_log()
 *                 
 *   FUNCTION:     Closes a log file
 *                 
 *   PARAMETERS:   filename
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void close_log()
{
  if (log_iop == NULL)
    return;


  fclose(log_iop);

  fprintf(stderr, "Log file closed. Logging is off.\n");
}



