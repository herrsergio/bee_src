
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







/*---- 'MAP_debug' prints out messages upon receipt of a TCX message ----*/
/*#define MAP_debug*/




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




#define MAP_NAME "map.dat"
#define MAP_INIT_NAME "map.ini"
#define SCRIPT_NAME "map.script"
#define MAP_LOG_NAME "map.log"

#ifdef UNIBONN
#define MAP_GIF_NAME "/home/rhino/b/p/rhino/public_html/tourguide/bilder/map_raw.gif"
#else
#define MAP_GIF_NAME "map.gif"
#endif
/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define NUM_GLOBAL_MAPS 4

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


int *frozen;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* GENERAL STATE VARIABLE TYPES */


typedef struct{
  int tcx_initialized;
  int quit;
  int graphics_initialized;
  int use_graphics;
  int use_tcx;
  int tcx_base_connected;
  int tcx_localize_connected;
  int regular_global_map_display;
  int regular_local_map_display;
  int maps_allocated;
  int map_update_pending;	/* 1, if we need to apply map update */
  int actual_map;
  int global_map_matching_mode;
  int actual_topology_option;
  int map_update_on;
  int force_map_update;
  int first_base_report_received;
} PROGRAM_STATE, *PROGRAM_STATE_PTR;

typedef struct{
  float x;
  float y;
  float orientation;
  float translational_speed;
  float rotational_speed;
  float sensor_x;
  float sensor_y;
  float sensor_orientation;
  float sensor_translational_speed;
  float sensor_rotational_speed;
  float sensor_uncertainty;	/* is proportional to speed */
  float sensor_org_x;		/* original belief as to where the sensor */
  float sensor_org_y;		/* reading was taken */
  float sensor_org_orientation;
  float sensor_best_x;		/* best match */
  float sensor_best_y;	
  float sensor_best_orientation;
  float sensor_best_fit;
  float *sensor_values;
  int   known;

  float last_change_x;	/* for positionsearch: previous update */
  float last_change_y;
  float last_change_orientation;

  float correction_parameter_x;	/* used for position correction */
  float correction_parameter_y;
  float correction_parameter_angle;
  int   correction_type;	/* rotation or translation correction */

  int   map_orientation_defined;
  float map_orientation;

  int   automatch_on;
  float automatch_pos_x;
  float automatch_pos_y;
  float automatch_pos_orientation;
  float automatch_cumul_distance;
} ROBOT_STATE, *ROBOT_STATE_PTR;



typedef struct{
  float global_mapsize_x;	/* size of the world in cm */
  float global_mapsize_y;	/* size of the world in cm */
  int   global_map_dim_x;	/* dimension of the global map in #fields */
  int   global_map_dim_y;	/* dimension of the global map in #fields */
  int   local_map_dim_x;	/* dimension of the local map in #fields */
  int   local_map_dim_y;	/* dimension of the local map in #fields */
  float resolution;		/* number of cm per displayed pixel */

  float local_map_origin_x;	/* origin of the local map wrt to global
				 * coordinate system */
  float local_map_origin_y;	/* ditto */
  float local_map_origin_orientation; /* ditto */
  int   local_map_number;	/* number of the local map (multiple maps) */
  int   smooth_radius;		/* for generating smoothed maps */
  
  float robot_size;		/* size of the robot in cm */
  float drift;			/* rotational drift in deg/cm */
  
  int   autoshift;		/* 1, if autoshift of the map defined, 0 else*/
  float autoshift_distance;	/* amount we shift in autoshift */
  float autoshift_safety_margin;/* minimum distance to walls */
  float autoshifted_x;		/* cm the display has been shifted */
  float autoshifted_y;		/* cm the display has been shifted */
  int   autoshifted_int_x;	/* #field-units the display has been shifted */
  int   autoshifted_int_y;	/* #field-units the display has been shifted */
  
  int   do_position_correction;	/* 1, if MAP should correct for pos. errors */
  float max_distance_in_match;	/* specifies how far away from the robot
				 * we'll use points for pos control. */
  int   map_fit_norm_L2;	/* type norm: 0=L1, 1=L2 */
  int   prev_pos_norm_L2;	/* type norm: 0=L1, 1=L2 */
  int   max_niterations_in_search; /* max. number of iterations, -1=infinity */
  int   niterations_in_map_fitting; /* number of iterations in map-fitting.
				     * -1 is not a legal choice here */
  int   niterations_in_search;	/* actual counter for iterations */
  float max_translation_in_search; /* maximum amount for translational step */
  float translation_weight_fit;	/* weight of the fitting in position search */
  float translation_weight_fit_global_match; /* same, but applied in
					      * global map matching */
  float translation_weight_prev_position; /* weight of the prev. position */
  float translation_stepsize;	/* stepsize in pos. search */
  float translation_momentum;	/* momentum term */
  float max_rotation_in_search;	/* same for rotation */
  float rotation_weight_fit;	/* weight of the fitting in position search */
  float rotation_weight_fit_global_match; /* same, but applied in
					   * global map matching */
  float rotation_weight_prev_position; /* weight of the prev. position */
  float rotation_stepsize;	/* stepsize in pos. search */
  float rotation_momentum;	/* momentum term */
  int   search_granularity;	/* 1=finest, >1 gradually coarse */

  int   do_path_fitting;	/* 1=use path information for fitting maps */
  float weight_path_fit;	/* weight wrt to map fit */
  int   n_path_points_in_fit;	/* number of previous pos. that will be used.
				 * -1 = all*/
  float wall_error_threshold;	/* threshold on which we won't believe wall
				 * angles */
  float wall_weight;		/* weight for fitting walls */
  int   number_subsequent_adjacent_walls; /* number of roughly adjacent 
					 * walls that define the wall
					 * orientation */

  float  min_advance_for_map_fitting; /* min distance traveled by the
				      * robot before
				      * we begin fitting maps*/


  float prior;			/* Bayesian prior for occupancy/freeness */
  float decay_old;		/* weight for decaying map likelihoods */
  float decay_new;		/* weight for sensor interpretations */
  int   update_extreme_likelihoods; /* if 0, the map likelihood values 0
				     * and 1 will never ever change */

  float lower_clipping_value;
  float upper_clipping_value;	/* for clipping maps */

  int   min_display_index_x[NUM_GLOBAL_MAPS]; /* just for display purposes */
  int   max_display_index_x[NUM_GLOBAL_MAPS];
  int   min_display_index_y[NUM_GLOBAL_MAPS];
  int   max_display_index_y[NUM_GLOBAL_MAPS];

  int   reposition_robot_initially; /* 1, if the first reading will
				     * be used to reposition the robot
				     * to the center of the map */

  int   regular_gif_output_in_sec; /* -1, if not wantes */

  int   broadcasts_on;		/* if 0, we will suppress all broadcasts */
  float X_window_size;		/* scale factor for the X-window */

  int   data_logging;		/* if 1, certain values will be logged,
				 * do not use, just for testing purposes */
  int   map_erasing_period;	/* if >= 1, then all maps will be erased
				 * periodically,
				 * do not use, just for testing purposes */

} ROBOT_SPECIFICATIONS, *ROBOT_SPECIFICATIONS_PTR;






/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


typedef struct{
  PROGRAM_STATE_PTR        program_state;
  ROBOT_STATE_PTR          robot_state;
  ROBOT_SPECIFICATIONS_PTR robot_specifications;
} ALL_TYPE, *ALL_PTR;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern PROGRAM_STATE            program_state_data;
extern ROBOT_STATE              robot_state_data;
extern ROBOT_SPECIFICATIONS     robot_specifications_data;
extern PROGRAM_STATE_PTR        program_state;
extern ROBOT_STATE_PTR          robot_state;
extern ROBOT_SPECIFICATIONS_PTR robot_specifications;
extern ALL_TYPE                 all;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* MAPS */




extern int   *global_active;
extern float *global_map;
extern float *global_local_correlations;
extern float *local_map;
extern int   *local_active;
extern float *local_smooth_map;
extern float *global_map_x[NUM_GLOBAL_MAPS];
extern int   *global_active_x[NUM_GLOBAL_MAPS];
extern unsigned char  *global_label_x[NUM_GLOBAL_MAPS];
extern unsigned char  *global_label;
extern float *global_voronoi;
extern int   *global_voronoi_active;
extern FILE  *log_iop;

/* PATH */

/* The VMS linker is not case sensitive!  */
/* This causes path to be confuse with PATH */

#ifdef VMS
#define path path_
#endif

#define MAX_N_PATH_ENTRIES 20000 /* about 3-6 hours or robot operation */
extern float path[MAX_N_PATH_ENTRIES][3];
extern int   n_path_entries;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern int something_happened;
/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern struct bParamList * bParamList;
extern const char *bRobotType;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



extern int GLOBAL_BACKGROUND;
extern int GLOBAL_MAPVALUES;
extern int GLOBAL_VORONOI;
extern int GLOBAL_CORRELATIONS;
extern int DISPLAY_GLOBAL_MAPVALUES_BUTTON;
extern int DISPLAY_LOCAL_MAPVALUES_BUTTON;
extern int DISPLAY_VORONOI_DIAGRAM_BUTTON;
extern int DISPLAY_VORONOI_DIAGRAM_INCREMENT_BUTTON;
extern int DISPLAY_VORONOI_DIAGRAM_DECREMENT_BUTTON;
#ifdef must_this_be
extern int DISPLAY_GLOBAL_CORRELATIONS_BUTTON;
#endif
extern int GLOBAL_ROBOT;
extern int LOCAL_BACKGROUND;
extern int LOCAL_MAPVALUES;
extern int QUIT_BUTTON;
extern int SAVE_BUTTON;
extern int LOAD_BUTTON;
extern int DUMP_BUTTON;
extern int PATH;
extern int BASE_CONNECTED_BUTTON;
extern int LOCALIZE_CONNECTED_BUTTON;
extern int LINE_ANGLE_BUTTON;
extern int ACTUAL_MAP_BUTON;
extern int BEST_FIT_POINT;
extern int COMBINE_MAPS_BUTTON;
extern int FREEZE_BUTTON;
extern int MAP_UPDATE_ENABLE_BUTTON;
extern int TOPOLOGICAL_GRAPH;
extern int CLEAR_MAP_BUTTON;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


#define MAX_NUM_NODES 500
#define MAX_NUM_EDGES  20
#define NUM_VORONOI_MODI 8

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

int extract_value_from_local_map(float robot_x, float robot_y, 
				 float robot_orientation,
				 float query_x, float query_y,
				 float *map_value,
				 float *partial__value__wrt__robot_x,
				 float *partial__value__wrt__robot_y,
				 float *partial__value__wrt__robot_orientation,
				 int *found_extreme_likelihood,
				 ROBOT_SPECIFICATIONS_PTR robot_specifications);

void compute_bounds(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		    float robot_x, float robot_y, float robot_orientation,
		    int *from_x, int *to_x, int *from_y, int *to_y);

void update_internal_map(int new_map,
			 ROBOT_SPECIFICATIONS_PTR robot_specifications,
			 PROGRAM_STATE_PTR        program_state,
	 		 ROBOT_STATE_PTR          robot_state);

float compute_local_fit(float robot_x, 
			float robot_y,
			float robot_orientation,
			int granularity,
			float *partial__correlation__wrt__robot_x,
			float *partial__correlation__wrt__robot_y,
			float *partial__correlation__wrt__robot_orientation);

void compute_correlation_matrix(float robot_orientation);

    
void initiate_position_search(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			      PROGRAM_STATE_PTR        program_state,
			      ROBOT_STATE_PTR          robot_state);

    
void do_search_step(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		    PROGRAM_STATE_PTR        program_state,
		    ROBOT_STATE_PTR          robot_state);

void compute_forward_correction(float robot_x, 
				float robot_y, 
				float robot_orientation,
				float corr_x, 
				float corr_y, 
				float corr_angle, /* in deg. */
				int   corr_type,
				float *corr_robot_x,
				float *corr_robot_y, 
				float *corr_orientation);

void compute_backward_correction(float corr_robot_x, 
				 float corr_robot_y, 
				 float corr_robot_orientation,
				 float corr_x, 
				 float corr_y, 
				 float corr_angle, /* in deg. */
				 int   corr_type,
				 float *robot_x,
				 float *robot_y, 
				 float *orientation);

void compute_correction_parameters(float robot_x, 
				   float robot_y, 
				   float robot_orientation,
				   float corr_robot_x,
				   float corr_robot_y, 
				   float corr_orientation,
				   float *corr_x, 
				   float *corr_y, 
				   float *corr_angle,
				   int   *corr_type);

void update_correction_parameters(float corr_robot_x, 
				  float corr_robot_y, 
				  float corr_robot_orientation,
				  float delta_x,
				  float delta_y,
				  float delta_orientation,
				  float *corr_x, 
				  float *corr_y, 
				  float *corr_angle,
				  int   *corr_type);

void test_correction();

int save_parameters(char *filename);

int load_parameters(char *filename, int init);

void count_auto_update_modules();

static int add_auto_update_module(TCX_MODULE_PTR module, int map,
				  int correction);

static int remove_auto_update_module(TCX_MODULE_PTR module);

void send_automatic_map_update(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			       PROGRAM_STATE_PTR  program_state,
			       int first_x, int num_x, int first_y, int num_y,
			       int delete_previous_map);


void send_automatic_correction_update(void);

void MAP_dump_handler(TCX_REF_PTR ref,
		      void       *data);

void MAP_dump_handler2(TCX_REF_PTR ref,
		      void       *data);

void BASE_update_status_reply_handler(TCX_REF_PTR                  ref,
				      BASE_update_status_reply_ptr status);

void MAP_quit_handler(TCX_REF_PTR   ref,
		      void         *data);

void MAP_register_auto_update_handler(TCX_REF_PTR  ref,
				      MAP_register_auto_update_ptr data);

void MAP_partial_map_query_handler(TCX_REF_PTR               ref,
				   MAP_partial_map_query_ptr data);

void MAP_correction_parameters_query_handler(TCX_REF_PTR  ref,
					     void        *data);

MAP_partial_map_reply_ptr make_map_message(ROBOT_SPECIFICATIONS_PTR 
					   robot_specifications,
					   PROGRAM_STATE_PTR  
					   program_state,
					   int first_x, int num_x, 
					   int first_y, int num_y,
					   int delete_previous_map);

void update_maps(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		 PROGRAM_STATE_PTR  program_state,
		 int first_x, int num_x, int first_y, int num_y,
		 TCX_REF_PTR  ref, /* NULL, if to be broadcasted everywhere! */
		 int delete_previous_map);

void MAP_sensor_interpretation_handler(TCX_REF_PTR                   ref,
				       MAP_sensor_interpretation_ptr interpret);

void MAP_close_handler(char *name, TCX_MODULE_PTR module);

void connect_to_BASE(PROGRAM_STATE_PTR program_state);

void connect_to_LOCALIZE(PROGRAM_STATE_PTR program_state);

void init_tcx(PROGRAM_STATE_PTR program_state);

void SONAR_sonar_reply_handler(TCX_REF_PTR           ref,
			       SONAR_sonar_reply_ptr sonar);

void check_commandline_parameters(int argc, char **argv, 
				  PROGRAM_STATE_PTR program_state);

void init_program(PROGRAM_STATE_PTR        program_state,
		  ROBOT_STATE_PTR          robot_state,
		  ROBOT_SPECIFICATIONS_PTR robot_specifications,
		  ALL_PTR                  all);

void allocate_everything(ROBOT_STATE_PTR    robot_state,
			 PROGRAM_STATE_PTR  program_state,
			 ROBOT_SPECIFICATIONS_PTR robot_specifications);

void read_init_file(ROBOT_STATE_PTR    robot_state,
		    PROGRAM_STATE_PTR  program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications);

void clear_maps(int number,	/* map number - -1 for all maps */
		ROBOT_STATE_PTR    robot_state,
		PROGRAM_STATE_PTR  program_state,
		ROBOT_SPECIFICATIONS_PTR robot_specifications);

void init_graphics(ROBOT_STATE_PTR    robot_state,
		   PROGRAM_STATE_PTR  program_state,
		   ROBOT_SPECIFICATIONS_PTR robot_specifications);

void autoshift_display_by(PROGRAM_STATE_PTR  program_state,
			  ROBOT_SPECIFICATIONS_PTR robot_specifications,
			  int shift_x, int shift_y);

void autoshift_display(PROGRAM_STATE_PTR  program_state,
		       ROBOT_SPECIFICATIONS_PTR robot_specifications,
		       float pos_x, float pos_y);

int mouse_test_loop(ROBOT_STATE_PTR          robot_state,
		    PROGRAM_STATE_PTR        program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications);

void set_map(int number);

void smooth_map(float *original_map, 
		int   *original_active,
		float *smoothed_map,
		int    dim_x,
		int    dim_y,
		int    smooth_radius);

int combine_global_maps(int base_map_number, /* serves as "global" map */
			int comp_map_number, /* serves as "local"  map */
			float global_map_start_x,
			float global_map_start_y,
			float global_map_start_orientation,
			int   combine, /* if 0, we do not want to 
					* change anything */
			int display,
			float *correlation,
			ROBOT_SPECIFICATIONS_PTR robot_specifications,
			PROGRAM_STATE_PTR        program_state,
			ROBOT_STATE_PTR          robot_state);


void send_complete_map(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		       PROGRAM_STATE_PTR  program_state,
		       TCX_REF_PTR  ref);


int check_and_combine_global_maps(ROBOT_SPECIFICATIONS_PTR 
				  robot_specifications,
				  PROGRAM_STATE_PTR        program_state,
				  ROBOT_STATE_PTR          robot_state);


void prepare_2nd_run(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		     PROGRAM_STATE_PTR        program_state);


void compute_voronoi_diagram(int                      plot_modus,
			     ROBOT_SPECIFICATIONS_PTR robot_specifications,
			     PROGRAM_STATE_PTR        program_state,
			     ROBOT_STATE_PTR          robot_state);

int save_gif(char *filename, int map_number, int force);

void clear_path(ROBOT_STATE_PTR    robot_state,
		PROGRAM_STATE_PTR  program_state,
		ROBOT_SPECIFICATIONS_PTR robot_specifications);

void
compute_map_0(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	       PROGRAM_STATE_PTR        program_state,
	       ROBOT_STATE_PTR          robot_state);

void
log_map(int                map_number,
	ROBOT_STATE_PTR    robot_state,
	PROGRAM_STATE_PTR  program_state,
	ROBOT_SPECIFICATIONS_PTR robot_specifications);
