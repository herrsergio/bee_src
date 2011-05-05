
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




#ifndef MAP_messages_defined
#define MAP_messages_defined





#include "tcx.h"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* 
   Notice: when including this file, you need to have the flag

   TCX_define_variables

   be defined exactly once. This will allocate memory for the
   module pointer and the message arrays.
*/

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** TCX module name - this is a unique identifier for TCX *******/

#define TCX_MAP_MODULE_NAME "MAP"



#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR MAP;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR MAP;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** MAP data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/







#define MAP_quit_format NULL

#define MAP_dump_format NULL

/**********************************************************************
 *
 * This message allows to subscribe for regular updates (like partial
 * maps, or position correction parameters)
 *
 */


typedef struct {
  int subscribe_maps;		/* 1=subscribe, 0=unsubscribe */
  int subscribe_position_correction; /* 1=subscribe, 0=unsubscribe */
} MAP_register_auto_update_type, *MAP_register_auto_update_ptr;

#define MAP_register_auto_update_format "{int, int}"



/**********************************************************************
 *
 * This message allows other programs to ask for maps, or parts thereof.
 *
 * Notice: If you subscribe to auto-updates, MAP will send you regular 
 * updates on the most recent changes using exactly this reply format.
 */

typedef struct{			
  int    first_x;
  int    first_y;
  int    size_x;
  int    size_y;
  float    resolution;
} MAP_partial_map_query_type, *MAP_partial_map_query_ptr;

#define MAP_partial_map_query_format "{int, int, int, int, float}"


typedef struct{			
  int    first_x;		/* Global map coordinates */
  int    first_y;		/* Global map coordinates */
  int    size_x;		/* Size of the map, global map coord. */
  int    size_y;		/* Size of the map, global map coord. */
  float    resolution;

  int    delete_previous_map;	/* 1, if this map replaces previous
				 * maps */
  
  int    number_of_map;		/* refers to the MAP-internal number
				 * of the map, if this is interesting 
				 * to someone */
  
  unsigned char *char_values;	/* New format for likelihoods/maps:
				 *  0 = passive,
				 *  1..254 = active, and
				 *  will be translated to floats in [0,1] 
				 *  255 = active, an will never change 
				 *  (used, for example, for points under the
				 *  robot)
				 */

  
} MAP_partial_map_reply_type, *MAP_partial_map_reply_ptr;

#define MAP_partial_map_reply_format "{int, int, int, int, float, int, int, <char : 3, 4>}"




/**********************************************************************
 *
 * This message allows other programs to ask corretion parameters
 * that are used when computing the actual robot position.
 *
 * Notice: If you subscribe to auto-updates, MAP will send you regular 
 * updates on the most recent changes using exactly this reply format.
 */


#define MAP_correction_parameters_query_format NULL


typedef struct{			
  float parameter_x;		/* used for position correction */
  float parameter_y;
  float parameter_angle;
  int   type;			/* rotation or translation correction */
  float current_wall_angle;
  int   current_wall_angle_defined;
} MAP_correction_parameters_reply_type, *MAP_correction_parameters_reply_ptr;

#define MAP_correction_parameters_reply_format "{float, float, float, int, float, int}"


typedef struct{			
  float parameter_x;		/* used for position correction */
  float parameter_y;
  float parameter_angle;
  int   type;			/* rotation or translation correction */
} MAP_correction_parameters_inform_type, *MAP_correction_parameters_inform_ptr;

#define MAP_correction_parameters_inform_format "{float, float, float, int}"



/**********************************************************************
 *
 * This message allows other programs to send their subjective likelihood
 * estimates for occupancy.
 * Examples are: Sonar interpretations, camera interpretations.
 * The likelihoods are then integrated into the global map. They
 * are also used for potition control.
 */


typedef struct{			
  /*
   * Where was the robot when the sensor  reading was taken?
   * Update these values by status report! Make sure, there is no big time lag
   * between the status report and the sensor reading, since the accuracy
   * of these values is crucial for the integration.
   */

  float robot_x;		/* x-position */
  float robot_y;		/* y-position */
  float robot_orientation;	/* orientation */
  float translational_speed;	/* the robot's translational speed */
  float rotational_speed;	/* the robot's rotational speed */

  /*
   * Where is the robot position with respect to the origin of the
   * local map? In most cases, this will be a constant. 
   */

  float origin_x;
  float origin_y;
  float origin_orientation;

  /*
   * Resolution of the map
   */

  float    resolution;

  /*
   * Delete the previous map? If yes, this will copy in the new map
   */

  int    delete_previous_map;

  /*
   * The number of the map.
   */

#define COMPOSED_MAP_NUMBER 0 /* do not use this one */
#define SONARINT_MAP_NUMBER 1
#define LASERINT_MAP_NUMBER 2
#define CAD_MAP_NUMBER      3

  int   map_number;


  /*
   * Size of the map. Local coordinates. Tells TCX how big the
   * package has to be.
   */

  int    size_x;		/* Size of the map. Local coordinates */
  int    size_y;		/* Size of the map. Local coordinates */

  /*
   * Partial Map. Likelihoods and active-flags
   *
   * (Note: At some point we might think about tranferring chars
   * instead of ints and floats here.)
   */

  unsigned char  *char_likelihoods;	/* New format! Likelihoods. Replaces
					 * likelihoods and active in one
					 * field, in order to save memory.
					 * char_likelihoods = 0 = passive,
					 * char_likelihoods = 1..254 = active,
					 * will be translated to floats in
					 * [0,1] 
					 * 255 is not used. */


  /* Previously, we used floats and ints...*/
  /* float *likelihoods;*/	/* Occupancy likelihoods. 1=free, 0=occup. */
  /* int   *active;*/		/* 1, if value[i] defined, 0 if not */

  /* Sometimes, it is necessary to send the raw sensor data along.
   * The following variables can be used to submit the original sensor
   * data */

  int    num_sensor_values_enclosed; /* > 0 if this message comes with the
				      * original sensor values enclosed 
				      * o, if no sensor data enclosed */
  float  *sensor_values;	/* the original sensor values, if any */


} MAP_sensor_interpretation_type, *MAP_sensor_interpretation_ptr;


#define MAP_sensor_interpretation_format "{float, float, float, float, float, float, float, float, float, int, int, int, int, <char : 12, 13>, int, <float : 15>}"




/**********************************************************************
 *
 * This message allows other programs to inform MAP that
 * a wall has been detected. This helps MAP tremendeously when 
 * estimating its orientation
 */


typedef struct {
  float robot_x;
  float robot_y;
  float robot_orientation;
  float wall_angle;
} MAP_saw_a_wall_type, *MAP_saw_a_wall_ptr;

#define MAP_saw_a_wall_message_format "{float, float, float, float}"




/**********************************************************************
 *
 * This message allows to mark certain regions by topoligical
 * information. Currently, is is used by TOPOL to indicate door regions
 * and other stuff
 * 
 */

#define CELL_TYPE_UNKNOWN    0
#define CELL_TYPE_DOOR       1
#define CELL_TYPE_OBSTACLE   2

typedef struct{			
  int    first_x;		/* Global map coordinates */
  int    first_y;		/* Global map coordinates */
  int    size_x;		/* Size of the map, global map coord. */
  int    size_y;		/* Size of the map, global map coord. */

  unsigned char *labels;	/* Information, as defined above by the
				 * defines.
				 */

} MAP_label_grid_type, *MAP_label_grid_ptr;

#define MAP_label_grid_format "{int, int, int, int, <char : 3, 4>}"



#define MAP_enable_map_update_format NULL

#define MAP_disable_map_update_format NULL


/*
 * the following message clears all sensor-based maps (but not the
 * CAD map).
 */
#define MAP_clear_all_sensor_maps_format NULL

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** MAP commands - these are the commands/queries understood by MAP ****/


#ifdef TCX_define_variables		/* do this exactly once! */


#define MAP_messages \
  {"MAP_register_auto_update",        MAP_register_auto_update_format},\
  {"MAP_dump",                        MAP_dump_format},\
  {"MAP_quit",                        MAP_quit_format},\
  {"MAP_partial_map_query",           MAP_partial_map_query_format},\
  {"MAP_partial_map_reply",           MAP_partial_map_reply_format},\
  {"MAP_correction_parameters_query", MAP_correction_parameters_query_format},\
  {"MAP_correction_parameters_reply", MAP_correction_parameters_reply_format},\
  {"MAP_correction_parameters_inform", MAP_correction_parameters_inform_format},\
  {"MAP_sensor_interpretation",       MAP_sensor_interpretation_format},\
  {"MAP_saw_a_wall",                  MAP_saw_a_wall_message_format},\
  {"MAP_label_grid",                  MAP_label_grid_format},\
  {"MAP_enable_map_update",           MAP_enable_map_update_format},\
  {"MAP_disable_map_update",          MAP_disable_map_update_format},\
  {"MAP_clear_all_sensor_maps",       MAP_clear_all_sensor_maps_format}
#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_REPLY_HANDLERS



/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with MAP ******/




/******* (a) Procedure headers ******/



void MAP_partial_map_reply_handler(TCX_REF_PTR                 ref,
				     MAP_partial_map_reply_ptr map);

void MAP_correction_parameters_reply_handler(TCX_REF_PTR                 ref,
				     MAP_correction_parameters_reply_ptr map);


/******* (b) Handler array ******/



TCX_REG_HND_TYPE MAP_reply_handler_array[] = {

  {"MAP_partial_map_reply", "MAP_partial_map_reply_handler",
     MAP_partial_map_reply_handler, TCX_RECV_ALL, NULL},
  {"MAP_correction_parameters_reply", 
     "MAP_correction_parameters_reply_handler",
     MAP_correction_parameters_reply_handler, TCX_RECV_ALL, NULL}
};


#endif

#endif
