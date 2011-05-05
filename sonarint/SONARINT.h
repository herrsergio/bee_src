
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







/*--- 'SONARINT_debug' prints out messages upon receipt of a TCX message ----*/
/*#define SONARINT_debug*/


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



#define TCX_SONARINT_MODULE_NAME "SONARINT"


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


#define SONARINT_NAME "sonarint.dat"
#define SONARINT_INIT_NAME "sonarint.ini"
#define SCRIPT_NAME "sonarint.script"
#define LOG_NAME "sonarint.log"

extern FILE *log_iop;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* GENERAL STATE VARIABLE TYPES */


typedef struct{
  int   tcx_initialized;
  int   graphics_initialized;
  int   use_graphics;
  int   use_tcx;
  int   regular_local_map_display;
  int   maps_allocated;
  int   base_connected;
  int   map_connected;
  int   processing_script;
  int   read_next_script_event;
  float delay_in_replay;	/* delay between sonar readings in script 
				 * in sec.*/
  int   logging_on;
  int   quit;
} PROGRAM_STATE, *PROGRAM_STATE_PTR;

typedef struct{
  int   known;
  float x;
  float y;
  float orientation;
  float translational_speed;
  float rotational_speed;
  float *sensor_values;
  float *raw_sensor_values;
} ROBOT_STATE, *ROBOT_STATE_PTR;



typedef struct{
  float local_mapsize_x;	/* size of the local window in cm */
  float local_mapsize_y;	/* size of the local window in cm */
  int   local_map_dim_x;	/* dimension of the local map in #fields */
  int   local_map_dim_y;	/* dimension of the local map in #fields */
  float resolution;		/* number of cm per displayed pixel */
  int smooth_radius;		/* smoothing wondow size */

  float robot_size;		/* size of the robot in cm */
  
  int   num_sensors;		/* number of sensors */
  float first_sensor_angle;	/* offset of the first sensor */
  float max_sensors_range;	/* maximum sensor range in cm */
  float min_sensors_range;	/* minimum sensor range in cm */
  float neuronet_max_sensors_range; /* maximum sensor range in cm
				     * when training the neural net */
  float neuronet_min_sensors_range; /* maximum sensor range in cm
				     * when training the neural net */
  float max_occupied_sensors_range; /* cut-off range for occupied regions */
  float occupied_outer_width;	/* max width of obstacle */
  float network_occupied_value; /* cutt-off threshold */
  float *sensor_angles;		/* sensor angle values (vector) */

  float network_value_mean;	/* where is the zero interpretation - the mean
				 * value for neural network predicitons? */
  float decay_with_distance;	/* If 1 sonar interpretations will be
				 * weighted by the distance to the robot 
				 * If 0, they won't. In between: rate of 
				 * deradation */

  int   line_recognition_neighbors; /* number of neighbors to be considered
				     * for the recognition of lines */
  float line_recognition_threshold; /* threshold in recognition */
  float min_advancement_between_interpretations; /* How far shall the
						  * robot move, before we 
						  * consider a reading
						  * worth interpreting? */

  int   broadcast_sensor_data_to_map; /* this flag makes sonarint forward
				      * sensor data to MAP */

} ROBOT_SPECIFICATIONS, *ROBOT_SPECIFICATIONS_PTR;




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* NEURAL NETWORK VARIABLES */

typedef struct{
  int network_size[4];	/* ninputs,nhidden1/2, noutput for net */
  net_ptr net;		/* pointer to the network */
} NEURAL_NETWORK, *NEURAL_NETWORK_PTR;



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


typedef struct{
  PROGRAM_STATE_PTR        program_state;
  ROBOT_STATE_PTR          robot_state;
  ROBOT_SPECIFICATIONS_PTR robot_specifications;
  NEURAL_NETWORK_PTR       neural_network;
} ALL_TYPE, *ALL_PTR;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern PROGRAM_STATE            program_state_data;
extern ROBOT_STATE              robot_state_data;
extern ROBOT_SPECIFICATIONS     robot_specifications_data;
extern NEURAL_NETWORK           neural_network_data;
extern PROGRAM_STATE_PTR        program_state;
extern ROBOT_STATE_PTR          robot_state;
extern ROBOT_SPECIFICATIONS_PTR robot_specifications;
extern NEURAL_NETWORK_PTR       neural_network;
extern ALL_TYPE                 all;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* MAPS */

extern float *local_map;
extern int   *local_active;
extern int   *local_robot;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern int new_sonar_reading;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


extern int DISPLAY_LOCAL_MAPVALUES_BUTTON;
extern int LOCAL_ROBOT_;
extern int LOCAL_BACKGROUND;
extern int LOCAL_MAPVALUES;
extern int QUIT_BUTTON;
extern int SCRIPT_BUTTON;
extern int BASE_CONNECTED_BUTTON;
extern int MAP_CONNECTED_BUTTON;
extern int REGRESSION;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern struct bParamList * bParamList;
extern const char *bRobotType;




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

float evaluate_network(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		       ROBOT_STATE_PTR robot_state,
		       net_ptr netX,
		       float distance, float angle, float prediction);

void compute_local_map(NEURAL_NETWORK_PTR neural_network,
		       ROBOT_SPECIFICATIONS_PTR robot_specifications,
		       PROGRAM_STATE_PTR  program_state,
		       ROBOT_STATE_PTR  robot_state);

void init_graphics(ROBOT_STATE_PTR    robot_state,
		   PROGRAM_STATE_PTR  program_state,
		   ROBOT_SPECIFICATIONS_PTR robot_specifications);

int mouse_test_loop(ROBOT_STATE_PTR          robot_state,
		    PROGRAM_STATE_PTR        program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications);

void check_commandline_parameters(int argc, char **argv, 
				  PROGRAM_STATE_PTR program_state);

void init_program(PROGRAM_STATE_PTR        program_state,
		  ROBOT_STATE_PTR          robot_state,
		  ROBOT_SPECIFICATIONS_PTR robot_specifications,
		  NEURAL_NETWORK_PTR       neural_network,
		  ALL_PTR                  all);

void allocate_everything(ROBOT_STATE_PTR    robot_state,
			 PROGRAM_STATE_PTR  program_state,
			 ROBOT_SPECIFICATIONS_PTR robot_specifications);

void read_init_file(ROBOT_STATE_PTR    robot_state,
		    PROGRAM_STATE_PTR  program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications);

void clear_all_maps(ROBOT_STATE_PTR    robot_state,
		    PROGRAM_STATE_PTR  program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications);

void init_network(NEURAL_NETWORK_PTR neural_network);

int save_parameters(char *filename);

int load_parameters(char *filename, int init);

int initiate_read_script(ROBOT_STATE_PTR    robot_state,
			 PROGRAM_STATE_PTR  program_state,
			 ROBOT_SPECIFICATIONS_PTR robot_specifications,
			 char *filename);

void  alarm_handler(int sig);

int read_script(ROBOT_STATE_PTR    robot_state,
		PROGRAM_STATE_PTR  program_state,
		ROBOT_SPECIFICATIONS_PTR robot_specifications);


void close_script(ROBOT_STATE_PTR    robot_state,
		  PROGRAM_STATE_PTR  program_state,
		  ROBOT_SPECIFICATIONS_PTR robot_specifications);

void BASE_update_status_reply_handler(TCX_REF_PTR                  ref,
				      BASE_update_status_reply_ptr status);

void SONAR_sonar_reply_handler(TCX_REF_PTR           ref,
			       SONAR_sonar_reply_ptr sonar);

void SONARINT_close_handler(char *name, TCX_MODULE_PTR module);

void broadcast_local_map_to_MAP(ROBOT_SPECIFICATIONS_PTR robot_specifications,
				PROGRAM_STATE_PTR        program_state,
				ROBOT_STATE_PTR          robot_state);

void connect_to_BASE(PROGRAM_STATE_PTR program_state);

void connect_to_MAP(PROGRAM_STATE_PTR program_state);

void init_tcx(PROGRAM_STATE_PTR program_state);

void smooth_local_map(ROBOT_SPECIFICATIONS_PTR robot_specifications);

int open_log(char *filename);

void close_log();


int angle_to_wall(float *sonars, /* vector */
		  float *sonar_angles, /* vector */
		  float *angle); /* return value - parameter */
