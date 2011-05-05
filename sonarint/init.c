
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
#include "EZX11.h"
#include "o-graphics.h"
#include "MAP-messages.h"
#include "SONAR-messages.h"
#include "BASE-messages.h"
#include "SONARINT.h"
#include "bUtils.h"

#ifdef RHINO_PLUS
#include "rst.h" /* rhino stuff */
#endif

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

char init_file_name[256];


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


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

#ifdef RHINO_PLUS
  selector                                = 1; /* default from rhino stuff */
#endif


  program_state->use_graphics             = 1; /* default */
  program_state->use_tcx                  = 1; /* default */
  program_state->delay_in_replay          = 0.75; /* default, like RHINO */
  program_state->logging_on               = 1; /* default */



  strcpy(init_file_name, SONARINT_INIT_NAME);

  for (i = 1; i < argc && !bug; i++){
    if (!strcmp(argv[i], "-nodisplay") || !strcmp(argv[i], "-nd"))
      program_state->use_graphics = 0;
    else if (!strcmp(argv[i], "-notcx") || !strcmp(argv[i], "-nt"))
      program_state->use_tcx = 0;
    else if (!strcmp(argv[i], "-nolog")  || !strcmp(argv[i], "-nl"))
      program_state->logging_on = 0;
    else if (!strcmp(argv[i], "-delay") || !strcmp(argv[i], "-de")){
      if (++i >= argc)
	bug = 3;
      else if (argv[i][0] == '-')
	bug = 3;
      else
	program_state->delay_in_replay = atof(argv[i]);
    }
    else if (!strcmp(argv[i], "-file") || !strcmp(argv[i], "-file")){
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
#ifdef RHINO_PLUS
    fprintf(stderr, "\nUsage: '%s [-nodisplay] [-tcx] [-rhino] [-file <filename>][-delay <sec>] [-nolog]\n", 
	    argv[0]);
#else
    fprintf(stderr, "\nUsage: '%s [-nodisplay] [-tcx] [-file <filename>][-delay <sec>] [-nolog]\n", 
	    argv[0]);
#endif

  else if (bug == 2)
    fprintf(stderr, "\nError: Option -file must be followed by file name.\n");

  else if (bug == 3)
    fprintf(stderr, "\nError: Option -delay must be followed by float.\n");


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
 *                 NEURAL_NETWORK_PTR neural_network   pointer to network
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
		  NEURAL_NETWORK_PTR       neural_network,
		  ALL_PTR                  all)
{
  if (program_state != NULL){
    program_state->tcx_initialized                         = 0;
    program_state->graphics_initialized                    = 0;
    /* program_state->use_graphics           = 1; command line option */
    /* program_state->use_tcx                = 1; command line option */
    program_state->regular_local_map_display               = 0;
    program_state->maps_allocated                          = 0;
    program_state->base_connected                          = 0;
    program_state->map_connected                           = 0;
    /* program_state->delay_in_replay        = 0.75; command line option */
    program_state->quit                                    = 0; 
    program_state->processing_script                       = 0;
    program_state->read_next_script_event                  = 0;
    /* program_state->logging_on             = 1; command line option */
  }


  if (robot_state != NULL){
    robot_state->known                                     = 0;
    robot_state->x                                         = 0.0;
    robot_state->y                                         = 0.0;
    robot_state->orientation                               = 0.0;
    robot_state->translational_speed                       = 0.0;
    robot_state->rotational_speed                          = 0.0;
  }


  if (robot_specifications != NULL){
    robot_specifications->local_mapsize_x                  = 600.0;
    robot_specifications->local_mapsize_y                  = 600.0;
    robot_specifications->local_map_dim_x                  = 61;
    robot_specifications->local_map_dim_y                  = 61;
    robot_specifications->resolution                       = 10.0;
    robot_specifications->smooth_radius                    = 3;
  
    robot_specifications->robot_size                       = 30.0;
  
  
    robot_specifications->num_sensors            = bRobot.sonar_cols[0];
    if (bRobot.sonar_cols[0] == 24)
      robot_specifications->first_sensor_angle             = -7.5;
    else if (bRobot.sonar_cols[0] == 16)
      robot_specifications->first_sensor_angle             = -11.25;
    robot_specifications->max_sensors_range                = 300.0;
    robot_specifications->min_sensors_range                = 1.0;
    robot_specifications->neuronet_max_sensors_range       = 300.0;
    robot_specifications->neuronet_min_sensors_range       = 1.0;
    robot_specifications->sensor_angles                    = NULL;

    robot_specifications->max_occupied_sensors_range       = 300.0;
    robot_specifications->occupied_outer_width             = 300.0;
    robot_specifications->network_occupied_value           = 0.75;

    robot_specifications->network_value_mean               = 0.75;
    robot_specifications->decay_with_distance              = 1;

    robot_specifications->line_recognition_neighbors       = 5;
    robot_specifications->line_recognition_threshold       = 0.01;
    robot_specifications->min_advancement_between_interpretations = 5.0;
    robot_specifications->broadcast_sensor_data_to_map     = 0;
  }

  
  if (all != NULL){
    all->program_state        = program_state;
    all->robot_specifications = robot_specifications;
    all->robot_state          = robot_state;
    all->neural_network       = neural_network;
  }
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
  int i, global_size, local_size;


  if (!program_state->maps_allocated){
    
    robot_specifications->local_mapsize_x   = 
      robot_specifications->max_sensors_range * 2.0;
    robot_specifications->local_mapsize_y   = 
      robot_specifications->max_sensors_range * 2.0;
    robot_specifications->local_map_dim_x   = 1 + (int) /* 1=safety margin */
      (robot_specifications->local_mapsize_x 
       / robot_specifications->resolution);
    robot_specifications->local_map_dim_y   = 1 + (int) /* 1=safety margin */
      (robot_specifications->local_mapsize_y 
       / robot_specifications->resolution);


    
    /* ALLOCATE NEW MEMORY FOR THE MAPS */
    
    local_size = robot_specifications->local_map_dim_x
      * robot_specifications->local_map_dim_y;
    if (!program_state->maps_allocated){
      local_map     = (float *) (malloc(sizeof(float) * local_size));
      local_robot   = (int *)   (malloc(sizeof(int)   * local_size));
      local_active  = (int *)   (malloc(sizeof(int)   * local_size));

      if (local_map == NULL || local_active == NULL || local_robot == NULL){
	printf("ABORT: out of memory!\n");
	exit(1);
      }
      program_state->maps_allocated = 1;
    }
    
    /* ALLOCATE NEW MEMORY FOR THE SENSORS */
    
    if (robot_specifications->sensor_angles != NULL)
      free(robot_specifications->sensor_angles);
    if (robot_state->raw_sensor_values != NULL)
      free(robot_state->raw_sensor_values);
    if (robot_state->sensor_values != NULL)
      free(robot_state->sensor_values);
    
    robot_specifications->sensor_angles = 
      (float *) (malloc(sizeof(float) * 
			robot_specifications->num_sensors ));
    robot_state->raw_sensor_values = 
      (float *) (malloc(sizeof(float) * 
			robot_specifications->num_sensors ));
    robot_state->sensor_values = 
      (float *) (malloc(sizeof(float) * 
			robot_specifications->num_sensors ));
    if (robot_specifications->sensor_angles == NULL ||
	robot_state->raw_sensor_values == NULL ||
	robot_state->sensor_values == NULL){
      printf("ABORT: out of memory!\n");
      exit(1);
    }
    for (i = 0; i < robot_specifications->num_sensors; i++){
      robot_specifications->sensor_angles[i] = 
	robot_specifications->first_sensor_angle 
	  + (360.0 * ((float) i)
	     / ((float)  robot_specifications->num_sensors));
      robot_state->raw_sensor_values[i] = 
      robot_state->sensor_values[i] = 
	robot_specifications->max_sensors_range
	  * 0.5 * (sin(((float) i) * 0.3) + 1.0); 
    }
    
    
    /* INITIALIZE MAPS */
    for (i = 0; i < local_size; i++){
      local_map[i]     = 0.5;
      local_robot[i]   = 0;
      local_active[i]  = 0;
    }
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
  if (load_parameters(init_file_name, 1))
    fprintf(stderr, "Initialization file %s successfully read.\n",
	    init_file_name);    
  else{
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

void clear_all_maps(ROBOT_STATE_PTR    robot_state,
		    PROGRAM_STATE_PTR  program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int x, y, index;
  
  for (x = 0; x < robot_specifications->local_map_dim_x; x++)
    for (y = 0; y < robot_specifications->local_map_dim_y; y++){
      index = x * robot_specifications->local_map_dim_y + y;
      if (local_map != NULL)
	local_map[index] = 0.5;
      if (local_active != NULL)
	local_active[index] = 0;
      if (local_robot != NULL)
	local_robot[index] = 0;
    }
}






/************************************************************************
 *
 *   NAME:         init_network
 *                 
 *   FUNCTION:     Initializes and creates network.
 *                 
 *   PARAMETERS:   NEURAL_NETWORK_PTR neural_network   pointer to network
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

#define HS_NUM_INPUTS  6
#define HS_NUM_OUTPUTS 1

#define new

void init_network(NEURAL_NETWORK_PTR neural_network)
{
  int i, j, ii;
#ifdef old
#define HS_NUM_HIDDEN  8
  static float hs_hidden_weights[HS_NUM_HIDDEN][HS_NUM_INPUTS] =
    {{-2.05827,9.81856,5.19281,-1.38505,-10.012,-0.617538},
       {-1.10835,2.52366,1.93989,-1.4488,-6.27143,-0.950382},
       {-1.78398,31.7569,4.81413,0.475091,-12.2744,-1.92544},
       {-1.2781,7.0219,4.34878,-1.52254,-9.0912,-0.891812},
       {0.144838,13.6954,10.7992,0.195507,-24.1377,-0.0327454},
       {-2.44775,11.7664,23.3102,-0.867641,-29.2833,3.60979},
       {-0.592681,17.2338,-0.158817,1.66644,-20.435,-3.8556},
       {-0.944728,7.98625,2.1898,4.92418,-18.9219,-7.10565}};
  static float hs_output_weights[HS_NUM_HIDDEN] =
    {2.88664,0.254665,16.4488,2.03083,-6.97433,7.76537,21.5508,-14.1887};
  static float hs_biases[HS_NUM_HIDDEN+HS_NUM_OUTPUTS] =
    {-9.54385,-5.98636,-22.2858,-8.56083,1.58459,-4.63209,
       -0.180325,0.351602,0.435347};

#endif

#ifdef old
/* 29. Erster Run ueber nacht, erster run mit Winkel-noise, low noise params */
#define HS_NUM_HIDDEN  10
  static float hs_hidden_weights[HS_NUM_HIDDEN][HS_NUM_INPUTS] =
    {{0.451211, 0.565258, 19.1794, 0.435424, -18.9606, 1.83088},
       {0.974816, -0.176986, 2.11199, 1.16313, 13.5604, -0.0819151},
       {-0.252529, 8.33473, 2.12479, -16.7232, 7.75783, 1.27509},
       {4.70693, 1.01073, -0.100825, 6.08254, -8.39754, -0.511258},
       {-0.808649, 32.8859, 1.24902, -0.201688, -31.834, -0.581324},
       {-1.03353, 26.6343, 0.296703, 0.0765959, -23.6871, 1.01494},
       {0.819786, -0.260834, -0.0961089, 1.54047, -21.2746, 0.293654},
       {-0.225093, 3.44549, 0.257066, 1.26886, -1.70415, -0.601072},
       {0.468051, 0.151056, 29.301, -1.09633, -27.3064, 0.495083},
       {1.52523, 2.30916, 2.83315, 2.71737, -2.49694, -2.56493}};
  
  static float hs_output_weights[HS_NUM_HIDDEN] =
    {-10.8154, 9.63425, -1.98927, 3.03774, -14.0025, 16.6945, 
       11.2689, -11.1621, 10.7734, 2.34629};
  
  static float hs_biases[HS_NUM_HIDDEN+HS_NUM_OUTPUTS] =
    {-1.56704, -18.7414, -7.29566, -5.75324, 1.68459, -1.02394,
       -1.35751, -4.25297, -1.64473, -6.65456, 0.857793};
#endif

#ifdef new
#define HS_NUM_HIDDEN  10
  static float hs_hidden_weights[HS_NUM_HIDDEN][HS_NUM_INPUTS] =
    {{4.9788, 1.62944, -0.341989, -4.27919, -8.5487, 0.728123},
       {1.41565, 2.34086, 16.2411, 4.46966, 2.36868, 1.60035},
       {-0.0623119, 7.63422, 3.34079, -0.29597, -2.6555, 0.309042},
       {0.151993, 17.336, 14.9557, -3.26972, -10.2095, -0.670753},
       {-0.564835, 24.5744, -0.776519, 0.363162, -24.3041, -0.692061},
       {-1.26829, 27.5586, -0.163926, 0.820662, -24.242, -0.218101},
       {2.49868, -0.704481, 0.130852, -0.235837, -12.9077, 0.495915},
       {0.168114, 0.126811, 0.168029, -0.0288237, -2.97025, -0.459604},
       {2.54263, 4.73341, 4.63776, 0.968635, -6.87045, 0.89195},
       {0.680483, 0.5075, 0.593876, 0.478556, -3.62527, -0.521582}};
  static float hs_output_weights[HS_NUM_HIDDEN] =
    {-10.8793, 11.959, -17.662, 6.87394, -13.0402, 15.8301, 
       14.7835, 0.0916997, 6.85235, 0.128403};
  static float hs_biases[HS_NUM_HIDDEN+HS_NUM_OUTPUTS] =
    {-4.65525, -28.897, -8.47242, -18.7972, 1.97622, -1.65051, 
       -1.0335, -4.45921, -8.44385, -5.37972, 0.533241};
#endif

  
  
  neural_network->network_size[0] = HS_NUM_INPUTS; /* number input units */
  neural_network->network_size[1] = HS_NUM_HIDDEN; /* number hidden, layer 1 */
  neural_network->network_size[2] = 0;             /* number hidden, layer 2 */
  neural_network->network_size[3] = HS_NUM_OUTPUTS; /* number output units */
  
  
  neural_network->net = create_network(neural_network->network_size[0],
					neural_network->network_size[1],
					neural_network->network_size[2],
					neural_network->network_size[3],
					0, 0, 0);


  for (i = neural_network->net->first_hidden1, ii = 0;
       i < neural_network->net->first_output; i++, ii++)
    for (j = 0; j < neural_network->net->first_hidden1; j++)
      neural_network->net->w[i][j] = hs_hidden_weights[ii][j];
  
  for (i = neural_network->net->first_hidden1, ii = 0;
       i < neural_network->net->first_output; i++, ii++)
    neural_network->net->w[neural_network->net->first_output][i] =
      hs_output_weights[ii];

 
  for (i = neural_network->net->first_hidden1, ii = 0;
       i < neural_network->net->nunits; i++, ii++)
    neural_network->net->bias[i] = hs_biases[ii];
 


  
/*
  if (!read_weights(neural_network->net, "hs.wts"))
    printf("WARNING: error while reading weights file %s.\n", "hs.wts");

*/    
}
