
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
#include <sys/time.h>
/*#include <malloc.h>*/
#include "all.h"

#ifdef UNIBONN


#define MAX_NUM_OBJECTS 5000
#define MAX_OBJECTS_TO_CARRY 1


typedef struct 
{
  float obj_x;
  float obj_y;
  int   obj_type;		/* according to tasks.h:
				 *
				 *    OBJ_BASKET  0
				 *    OBJ_SODACAN 1
				 *    OBJ_CUP     2
				 *    OBJ_TRASH   3
				 */
  int   obj_status;		/* 2=collected, 1= not collected, 0=deleted */
  int   counter;		/* how often did we update the position
				 * of the object */
  
  float total_dist_traveled_when_last_seen; /* long names are
					     * self-explanatory */

  int   non_seen_counter;	/* how often did we not see that
				 * object, when we expected to see it? */
} object_type, *object_ptr;



object_type objects[MAX_NUM_OBJECTS];
int         n_objects = 0;

void dump_object2(ALL_PARAMS);

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


static char *pick_up_texts[VIS_NUM_OBJECTS] =
{
  "I pick up trash",
  "I pick up trash",
  "I pick up trash",
  "What the hell. Why do I want to pick up a basket?"
};
    


static char drop_text[128] = "I drop all items into the trash bin";
    
static char found_bin_text[128] = "There is a trash bin?";


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

int prev_travel_mode = -1;



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




/************************************************************************
 *
 *   NAME:         check_and_add_object
 *                 
 *   FUNCTION:     checks, if an object is known. Add the object, if unknown,
 *                 and updates the object location, if known
 *                 
 *   PARAMETERS:   object location, type as defined above
 *                 
 *   RETURN-VALUE: returns object number in database, -1 if not found
 *                 
 ************************************************************************/


int check_and_add_object(float obj_x, float obj_y, int obj_type,
			 ALL_PARAMS)
{
  int i, found, n;
  float  dist, min_dist = -1.0;
  float x, y;
  int type;
  
  /*
   * Check if the object is known in the database
   *
   * Note: currently we neglect most of the type information!
   */
  
  found = -1;
  for (i = 0; i < n_objects; i++){
    if (objects[i].obj_status){
      dist = sqrt(((objects[i].obj_x - obj_x) * (objects[i].obj_x - obj_x))
		  + ((objects[i].obj_y - obj_y) * (objects[i].obj_y - obj_y)));
      /* 
       * "min_dist_between_objects" applies currently only to trash bins
       */
      if (((objects[i].obj_type == VIS_OBJECT_BASKET &&
	    dist < robot_specifications->min_dist_between_objects
	    * sqrt((float) objects[i].counter)) || 
	   dist == 0.0) && 
	  obj_type == objects[i].obj_type)
	if (found == -1 || dist < min_dist){
	  found = i;
	  min_dist = dist;
	}
    }
  }
  
  if (found == -1 && obj_type != -1){
    
    
    /*
     * Must be a new object. Let's check if there is space and then add it.
     */
    
    if (n_objects >= MAX_NUM_OBJECTS){
      fprintf(stderr, "WARNING: Capacity exceeded in object database.\n");
      return -1;
    }
    
    objects[n_objects].obj_x      = obj_x;
    objects[n_objects].obj_y      = obj_y;
    objects[n_objects].obj_type   = obj_type;
    
    /*
     * defaults 
     */
    
    objects[n_objects].counter    = 1; /* one data point */
    objects[n_objects].obj_status = 1; /* certainly not picked up yet */
    objects[n_objects].total_dist_traveled_when_last_seen = 
      program_state->total_distance_traveled;
    objects[n_objects].non_seen_counter = 0;
    
    
    /*
     * Special treatment for baskets, if we are trying to
     * get there
     */

    
    if (robot_specifications->set_points_when_searching_bins &&
	program_state->looking_for_trash_bin_mode &&
	objects[n_objects].obj_type == VIS_OBJECT_BASKET)
      tcx_plan_goal_message(objects[n_objects].obj_x, objects[n_objects].obj_y,
			    objects[n_objects].obj_type, 1, ALL);
    
    
    /*
     * Special treatment for objects, if we are trying to
     * get there
     */

    
    if (robot_specifications->set_points_when_searching_objects &&
	!program_state->looking_for_trash_bin_mode &&
	objects[n_objects].obj_type >= 0 &&
	objects[n_objects].obj_type < VIS_OBJECT_BASKET)
      tcx_plan_goal_message(objects[n_objects].obj_x, objects[n_objects].obj_y,
			    objects[n_objects].obj_type, 1, ALL);
    
    n_objects++;
        
    return (n_objects - 1);
  }




  if (found == -1)
    /*
     * could not find the object, and the object does not even have a type.
     */
    return -1;
  
  
  /*
   * Seems there is a similar object in the database. If the type is the
   * same, let's update the position of that object
   */
  
  if (obj_type == objects[found].obj_type){
    
    if (objects[found].obj_type == VIS_OBJECT_BASKET)
      tcx_speech_talk_text(ALL, found_bin_text);

    /*
     * Special treatment for baskets, if we are trying to
     * get there
     */
    
    if (robot_specifications->set_points_when_searching_bins &&
	program_state->looking_for_trash_bin_mode &&
	objects[found].obj_type == VIS_OBJECT_BASKET)
      tcx_plan_goal_message(objects[found].obj_x, objects[found].obj_y,
			    objects[found].obj_type, 0, ALL);
    

    /*
     * Special treatment for objects, if we are trying to
     * get there
     */

    
    if (robot_specifications->set_points_when_searching_objects &&
	!program_state->looking_for_trash_bin_mode &&
	objects[found].obj_type >= 0 &&
	objects[found].obj_type < VIS_OBJECT_BASKET)
      tcx_plan_goal_message(objects[found].obj_x, objects[found].obj_y,
			    objects[found].obj_type, 0, ALL);
    
     
    /*
     * Undisplay the old position
     */
    
    if (program_state->graphics_initialized){

      n = G_return_num_markers(TARGET_OBJECTS, 1);
      for (i = 0; i < n; i++){
	G_return_marker_coordinates(TARGET_OBJECTS, i, &x, &y, &type);
	if (type == objects[found].obj_type && x == objects[found].obj_x
	    && y == objects[found].obj_y)
	  G_undisplay_markers(TARGET_OBJECTS, i, C_GREY90);
      }
      
      G_delete_marker(TARGET_OBJECTS, objects[found].obj_x,
		      objects[found].obj_y);
    }
      
      
    /*
     * Update location of the object
     */
      
      
    objects[found].obj_x = (obj_x + (objects[found].obj_x
				     * ((float) objects[found].counter)))
      / ((float) (objects[found].counter + 1));
    
    objects[found].obj_y = (obj_y + (objects[found].obj_y
				     * ((float) objects[found].counter)))
      / ((float) (objects[found].counter + 1));
    
    objects[found].counter += 1;	/* plain, simple average.
					 * Maybe we should add a certainty
					 * factor here... */
    
    objects[found].total_dist_traveled_when_last_seen = 
      program_state->total_distance_traveled;

    objects[found].non_seen_counter = 0;

    
    /*
     * Special treatment for baskets, if we are trying to
     * get there
     */
    
    if (robot_specifications->set_points_when_searching_bins &&
	program_state->looking_for_trash_bin_mode &&
	objects[found].obj_type == VIS_OBJECT_BASKET)
      tcx_plan_goal_message(objects[found].obj_x, objects[found].obj_y,
			    objects[found].obj_type, 1, ALL);
    
    /*
     * Special treatment for objects, if we are trying to
     * get there
     */

    
    if (robot_specifications->set_points_when_searching_objects &&
	!program_state->looking_for_trash_bin_mode &&
	objects[found].obj_type >= 0 &&
	objects[found].obj_type < VIS_OBJECT_BASKET)
      tcx_plan_goal_message(objects[found].obj_x, objects[found].obj_y,
			    objects[found].obj_type, 1, ALL);
    
  
    return found;
  }
  
  return -1;			/* btw, we should never
				 * arrive here in the current version */
}
  
  



/************************************************************************
 *
 *   NAME:         check_if_point_visible
 *                 
 *   FUNCTION:     marks objects that we *should* be able to see
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

int check_if_point_visible(float robot_x, 
			   float robot_y,
			   float robot_orientation,
			   float point_x, 
			   float point_y,
			   int   camera,
			   ALL_PARAMS)
{
  float actual_opening_angle;
  float actual_min_perceptual_range, actual_max_perceptual_range;
  float x, y;
  int   int_x, int_y;
  int   i, n, index;
  float diff_x, diff_y, diff_angle, dist;
  float occupancy_value;
  int   *global_map_active = NULL;
  float *global_map        = NULL;

  
  if (camera == 0){
    actual_opening_angle = 
      robot_specifications->camera_1_opening_angle;
    actual_min_perceptual_range =
      robot_specifications->camera_1_min_perceptual_dist;
    actual_max_perceptual_range =
      robot_specifications->camera_1_max_perceptual_dist;
  }
  else if (camera == 1){
    actual_opening_angle = 
      robot_specifications->camera_2_opening_angle;
    actual_min_perceptual_range =
      robot_specifications->camera_2_min_perceptual_dist;
    actual_max_perceptual_range =
      robot_specifications->camera_2_max_perceptual_dist;
  }

  else{
    fprintf(stderr, "ERROR: camera number %d not recognized.\n", camera);
    actual_opening_angle = 120.0;
    actual_min_perceptual_range = 0.0;
    actual_max_perceptual_range = 9999999999.9;
  }
  
  diff_x = point_x - robot_x;
  diff_y = point_y - robot_y;
  dist   = sqrt((diff_x * diff_x) + (diff_y * diff_y));
  
  /*
   * check, if the object is where the robot is
   */

  if (dist <= actual_min_perceptual_range ||
      dist >= actual_max_perceptual_range)
    /* fprintf(stderr, "STRANGE: cannot see what's under my wheels.\n"); */
    return 0;

  
  /*
   * compute the angle to the robot.
   */

  diff_angle = (atan2(diff_y, diff_x) * 180.0 / M_PI) - robot_orientation;
  for (; diff_angle < -180.0; ) diff_angle += 360.0;
  for (; diff_angle >  180.0; ) diff_angle -= 360.0;

  
  /*
   * check, if the robot can see the object or not
   */

  if (fabs(diff_angle) > 0.5 * actual_opening_angle)
    return 0;

  
  /*
   * Now, it seems that the objetc is in front of the robot. Let's now
   * check if there is a wall between the robot and the object, in which
   * case we won;t believe the object
   */

  n = (int) ((dist - robot_specifications->robot_size)
	     / robot_specifications->map_resolution * 2.0); /* number of
							     * query
							     * steps */

  global_map           = robot_specifications->occupancy_values;
  global_map_active    = robot_specifications->map_active;
  


  for (i = 0; i <= n; i++){
    /*
     * calculate query point and map-value
     */
    x     = robot_x + ((((float) i) / ((float) n)) * diff_x);
    int_x = ((int) (x / robot_specifications->map_resolution))
      + robot_specifications->autoshifted_int_x;
    y     = robot_y + ((((float) i) / ((float) n)) * diff_y);
    int_y = ((int) (y / robot_specifications->map_resolution))
      + robot_specifications->autoshifted_int_y;
    index = int_x * robot_specifications->global_map_dim_y + int_y;

    if (int_x >= 0 && int_x < robot_specifications->global_map_dim_x &&
	int_y >= 0 && int_y < robot_specifications->global_map_dim_y &&
	global_map_active[index])
      occupancy_value = global_map[index];
    else
      occupancy_value = 1.0;

    if (occupancy_value < 0.3)
      /*
       * We believe an object is hidden if it is behind something that is
       * occupied (< 0.5)
       */
      return 0;
  }

  /*
   * Return "visible", since object is not hidden
   */
  
  return 1;
}

int find_nearest_basket_index(ALL_PARAMS)
{
  int i;
  float dist, min_dist = 0.0;
  int index = -1;

  for (i = 0; i < n_objects; i++)
    if (objects[i].obj_type == VIS_OBJECT_BASKET &&
	objects[i].obj_status != 0){
      dist = sqrt(((objects[i].obj_x - robot_state->x)
	    * (objects[i].obj_x - robot_state->x))
	   + ((objects[i].obj_y - robot_state->y)
	      * (objects[i].obj_y - robot_state->y)));
      if (index == -1 || dist < min_dist){
	index = i;
	min_dist = dist;
      }
    }
  return index;
}
      
    


/************************************************************************
 *
 *   NAME:         remove_objects_that_are_not_in_freespace
 *                 
 *   FUNCTION:     marks objects that we *should* be able to see
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void remove_objects_that_are_not_in_freespace(ALL_PARAMS)
{
  int i, int_x, int_y, index;

  for (i = 0; i < n_objects; i++)


    if (objects[i].obj_status > 0 && objects[i].obj_type != VIS_OBJECT_BASKET){
      
      int_x = ((int) (objects[i].obj_x / robot_specifications->map_resolution))
	+ robot_specifications->autoshifted_int_x;
      int_y = ((int) (objects[i].obj_y / robot_specifications->map_resolution))
	+ robot_specifications->autoshifted_int_y;

      index = int_x * robot_specifications->global_map_dim_y + int_y;

      if (robot_specifications->map_active[index] &&  
	  robot_specifications->occupancy_values[index] < 0.2)	/*!*/
	objects[i].obj_status = 0;
    }
}



/************************************************************************
 *
 *   NAME:         process_object_message
 *                 
 *   FUNCTION:     marks objects that we *should* be able to see
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void process_object_message(float robot_x, 
			    float robot_y,
			    float robot_orientation,
			    float pan_angle,
			    float tilt_angle,
			    int   num_objects,
			    float *point_x, 
			    float *point_y,
			    int   *obj_type,
			    int   camera,
			    ALL_PARAMS)
{
  int i, j, k, n, num;
  int might_not_exist[MAX_NUM_OBJECTS];
  int just_seen[MAX_NUM_OBJECTS];
  float x, y;
  int   type;
  int   approach_obj_number;
  float approach_object_distance, dist;

  /*
   * clear off all objects that lie in the wall
   */

  remove_objects_that_are_not_in_freespace(ALL);

  /*
   * check all objects the robot *should* see
   */

  for (i = 0; i < MAX_NUM_OBJECTS; i++){
    if (i < n_objects && objects[i].obj_type == VIS_OBJECT_BASKET)

      might_not_exist[i] = 0;

    else if (i < n_objects && objects[i].obj_status)	/* n_objects: global, 
							 * database */
      might_not_exist[i] = check_if_point_visible(robot_x,
						  robot_y,
						  robot_orientation,
						  /* pan_angle, */
						  objects[i].obj_x,
						  objects[i].obj_y,
						  camera,
						  ALL);
    else
      might_not_exist[i] = 0;
    
    just_seen[i] = 0;
  }
  
  /*
   * Incorporate objects into the database
   */

  for (i = 0; i < num_objects; i++){ /* num_objects: proc header */
    if (obj_type[i] != -1){
      if (obj_type[i] == VIS_OBJECT_BASKET ||
	  check_if_point_visible(robot_x,
				 robot_y,
				 robot_orientation,
				 /*pan_angle,*/
				 point_x[i], 
				 point_y[i],
				 camera,
				 ALL)){
	num = check_and_add_object(point_x[i], point_y[i], obj_type[i], ALL);
	fprintf(stderr, "Observation %d added/updated, database index %d\n", i,
		num);
      
	if (num >= 0){
	  might_not_exist[num] = 0; /* we did *not* miss that object */
	  just_seen[num]       = 1; /* and indeed we just saw it */
	}
      }  
    }
    else
      fprintf(stderr, "Object %d not visible\n", i);
  }

  /*
   * Increment the counter for objects we did not see
   */

  for (i = 0; i < n_objects; i++) /* n_objects: global, database */
    if (might_not_exist[i])
      objects[i].non_seen_counter += 1;

  
  /*
   * Remove those object that were not seen for too many times
   */


  for (i = 0; i < n_objects; i++){ /* n_objects: global, database */
    if (objects[i].obj_status && 
	(objects[i].obj_type == VIS_OBJECT_BASKET &&
	 objects[i].non_seen_counter >= 
	 robot_specifications->max_num_allowed_failures_in_object_recognition)
	||
	(objects[i].obj_type < VIS_OBJECT_BASKET &&
	 objects[i].non_seen_counter >= 1)){
      
      /*fprintf(stderr, "Database index %d removed\n", i);*/
      
      objects[i].obj_status = 0;
      
      /*
       * Remove the object from the display
       */
      
      n = G_return_num_markers(TARGET_OBJECTS, 1); /* _all_ objetcs */
      for (j = 0; j < n; j++){
	G_return_marker_coordinates(TARGET_OBJECTS, j, &x, &y, &type);
	if (type == objects[i].obj_type &&
	    x == objects[i].obj_x && 
	    y == objects[i].obj_y)
	  G_undisplay_markers(TARGET_OBJECTS, j, C_GREY90);
      }
      G_delete_marker(TARGET_OBJECTS, objects[i].obj_x, objects[i].obj_y);
      


    /*
     * Special treatment for baskets, if we are trying to
     * get there
     */
    
    if (robot_specifications->set_points_when_searching_bins &&
	program_state->looking_for_trash_bin_mode &&
	objects[i].obj_type == VIS_OBJECT_BASKET)
      tcx_plan_goal_message(objects[i].obj_x, objects[i].obj_y,
			    objects[i].obj_type, 0, ALL);
    

    /*
     * Special treatment for objects, if we are trying to
     * get there
     */

    
    if (robot_specifications->set_points_when_searching_objects &&
	!program_state->looking_for_trash_bin_mode &&
	objects[i].obj_type >= 0 &&
	objects[i].obj_type < VIS_OBJECT_BASKET)
      tcx_plan_goal_message(objects[i].obj_x, objects[i].obj_y,
			    objects[i].obj_type, 0, ALL);
    
     

    }
  }

  /*
   * display all objects
   */

  /*fprintf(stderr, "======= list of all objects ==========\n");*/
  G_clear_markers(TARGET_OBJECTS);
  for (i = 0; i < n_objects; i++){
    if (objects[i].obj_status)
      G_add_marker(TARGET_OBJECTS,
		   objects[i].obj_x, objects[i].obj_y, objects[i].obj_type);
    /*fprintf(stderr, "\t%d\tst:%d\tty:%d\n",
      i, objects[i].obj_status, objects[i].obj_type);*/
  }
  G_display_markers(TARGET_OBJECTS);

  /*
   * check, if we can approach an object or a trash bin, respectively
   */
/*
  fprintf(stderr, "----> Stat: %d %d %d\n", 
  program_state->hunting_mode,
  program_state->approaching_mode,
  program_state->autonomous_mode);*/

  if (program_state->hunting_mode /*&&
      !program_state->approaching_mode &&
      program_state->autonomous_mode*/){
    
    /*
     * Okay, there might be something to approach...
     */
    if (program_state->approaching_mode){
      approach_obj_number = -100;
      approach_object_distance = 
	sqrt(((action->target_pos_x - robot_state->x)
	      * (action->target_pos_x - robot_state->x))
	     + ((action->target_pos_y - robot_state->y)
	      * (action->target_pos_y - robot_state->y))) * 1.3;
    }
    else{
      approach_obj_number = -1;
      approach_object_distance = 0.0;
    }


    for (i = 0; i < n_objects; i++)
      if (objects[i].obj_status == 1 &&
	  objects[i].obj_type >= 0 &&
	  objects[i].obj_type < VIS_NUM_OBJECTS && /*! necessary??? */
	  just_seen[i]){
	
	dist = sqrt(((objects[i].obj_x - robot_state->x)
		     * (objects[i].obj_x - robot_state->x))
		    + ((objects[i].obj_y - robot_state->y)
		       * (objects[i].obj_y - robot_state->y)));
	
	/*
	 * basket hunting mode
	 */
	
	if (program_state->looking_for_trash_bin_mode &&
	    objects[i].obj_type == VIS_OBJECT_BASKET &&
	    (approach_obj_number == -1 ||
	     approach_object_distance > dist)){
	  approach_obj_number = i;
	  approach_object_distance = dist;
	}

	/*
	 * object hunting mode
	 */

	else if (!program_state->looking_for_trash_bin_mode &&
		 objects[i].obj_type < VIS_OBJECT_BASKET &&
		 (approach_obj_number == -1 ||
		  approach_object_distance > dist)){
	  approach_obj_number = i;
	  approach_object_distance = dist;
	}
      }

    
    if (approach_obj_number >= 0){
      fprintf(stderr, "#### %d %d %d ###\n", 
	      program_state->looking_for_trash_bin_mode,
	      approach_obj_number,
	      objects[approach_obj_number].obj_type);
      /*
       * okay, we found an object that is worth approaching
       */
      if (!program_state->approaching_mode)
	start_approach_object(approach_obj_number, ALL);
      else{
	action->target_pos_x = objects[approach_obj_number].obj_x;
	action->target_pos_y = objects[approach_obj_number].obj_y;
	fprintf(stderr, "\t----> %f %f\n",
		action->target_pos_x,
		action->target_pos_y);
      }
    }
  }


  /*
   * And display the stuff
   */

  G_display_markers(TARGET_OBJECTS);

}




/************************************************************************
 *
 *   NAME:         start_hunting
 *                 
 *   FUNCTION:     starts object hunting mode
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void start_hunting(ALL_PARAMS)
{
  fprintf(stderr, "\t\t\tstart_hunting()\n");
  
  if (!program_state->tcx_connected_to_BASE ||
      /*! !program_state->tcx_connected_to_SUNVIS || */
      !program_state->tcx_connected_to_PLAN){
    fprintf(stderr, "WARNING: Must be connected at least to BASE, SUNVIS");
    fprintf(stderr, " and PLAN to hunt objects.\n");
    return;
  }

  /*
   * Set the mode
   */
    
  program_state->hunting_mode = 1;


  /*
   * Put the base mode on stack
   */

  prev_travel_mode = robot_state->base_mode;


  /*
   * Set all other flags appropriately
   */

  program_state->active_goal_number         = -1;
  program_state->autonomous_mode            = 0;
  program_state->approaching_mode           = 0;
  program_state->active_goal_number         = 0;
  program_state->looking_for_trash_bin_mode = 0;

  program_state->n_carried_objetcs          = 0;
  G_display_switch(NUM_OBJECTS_BUTTON, program_state->n_carried_objetcs);
  
  /*
   * Tell the planner to explore
   */
  
  start_hunting_objects(ALL); 

}





/************************************************************************
 *
 *   NAME:         stop_hunting
 *                 
 *   FUNCTION:     starts object hunting mode
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void stop_hunting(ALL_PARAMS)
{
  fprintf(stderr, "\t\t\tstop_hunting()\n");

  if (!program_state->hunting_mode)
    return;


  tcx_plan_stop_autonomous(1, ALL);

  tcx_base_stop_robot(ALL);

  /*
   * Reset the base mode
   */

  if (prev_travel_mode >= 0)
    tcx_base_set_mode(prev_travel_mode, ALL);

  /*
   * reset all parameters
   */
    
  program_state->hunting_mode = 0;
  G_display_switch(HUNTING_BUTTON, 0);

  program_state->looking_for_trash_bin_mode  = 0;
  program_state->approaching_mode            = 0;
  program_state->hunting_mode                = 0;
  program_state->autonomous_mode             = 0;
  program_state->active_goal_number          = -1;

  program_state->n_carried_objetcs           = 0;
  G_display_switch(NUM_OBJECTS_BUTTON, program_state->n_carried_objetcs);
}  

  
  



/************************************************************************
 *
 *   NAME:         start_hunting_objects
 *                 
 *   FUNCTION:     starts hunting objects that can be picked up
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void start_hunting_objects(ALL_PARAMS)
{
  int i;

  fprintf(stderr, "\t\t\tstart_hunting_objects()\n");

  /*
   * Check if everything is consistent
   */

  if (!program_state->hunting_mode){
    fprintf(stderr, "ERROR: can only hunt objects in hunting mode.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }


  if (program_state->approaching_mode){
    fprintf(stderr, "ERROR: Cannot hunt and approach an objects simltaneously.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }

  if (program_state->looking_for_trash_bin_mode){
    fprintf(stderr, "ERROR: looking for a trash bin. Cannot hunt objects\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }


  if (program_state->n_carried_objetcs >= MAX_OBJECTS_TO_CARRY){ /* soft bug */
    fprintf(stderr, "WARNING: Carrying aleady %d objects\n",
	    program_state->n_carried_objetcs);
    putc(7, stderr);
  }


  
  /*
   * Okay, let's use the planner to get to an object
   */


  tcx_remove_all_goals(ALL);


  /*
   * Send all trash bins as goal points
   */

  tcx_remove_all_goals(ALL);

  if (robot_specifications->set_points_when_searching_objects)
    for (i = 0; i < n_objects; i++)
      if (objects[i].obj_status == 1
	  && objects[i].obj_type >= 0
	  && objects[i].obj_type < VIS_OBJECT_BASKET)
	tcx_plan_goal_message(objects[i].obj_x, objects[i].obj_y,
			      objects[i].obj_type, 1, ALL);


  /*
   * Let BASE be fast
   */

  tcx_base_set_mode(FIND_DOOR_MODE, ALL);

  /*
   * And activate our planner.
   */

  tcx_plan_start_autonomous(1, ALL);

  program_state->active_goal_number = -1;

  G_display_switch(HUNTING_BUTTON, 1);

}




/************************************************************************
 *
 *   NAME:         start_approach_object
 *                 
 *   FUNCTION:     switches the planner off, approaches an object
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void start_approach_object(int number, ALL_PARAMS)
{

  float target_x, target_y;
  float dist_to_target, angle_to_target;

  fprintf(stderr, "\t\t\tstart_approach_object()\n");

  /*
   * Check if everything is consistent
   */

  if (!program_state->hunting_mode){
    fprintf(stderr, "ERROR: can only approach objects in hunting mode.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }


  if (program_state->approaching_mode){
    fprintf(stderr, "ERROR: Already approaching object.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }

  if (!program_state->autonomous_mode){
    fprintf(stderr, "ERROR: approach object not in autonomous mode.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }


  if (!program_state->looking_for_trash_bin_mode &&
      program_state->n_carried_objetcs >= MAX_OBJECTS_TO_CARRY){ /* soft bug */
    fprintf(stderr, "WARNING: Carrying already %d objects.\n",
	   program_state->n_carried_objetcs);
    putc(7, stderr);
  }

  if (program_state->looking_for_trash_bin_mode &&
      program_state->n_carried_objetcs < MAX_OBJECTS_TO_CARRY){ /* soft bug */
    fprintf(stderr, "WARNING: Carrying only %d objects.\n",
	    program_state->n_carried_objetcs);
    putc(7, stderr);
  }


  /*
   * Where would one have to go to pick up the object?
   */


  tcx_plan_stop_autonomous(0, ALL); /* ..just in case */

  tcx_base_set_mode(APPROACH_OBJECT_MODE, ALL);

  dist_to_target = sqrt(((objects[number].obj_x - robot_state->x) * 
			 (objects[number].obj_x - robot_state->x))
			+ ((objects[number].obj_y - robot_state->y) * 
			   (objects[number].obj_y - robot_state->y)));

  if (dist_to_target > 0.0)
    angle_to_target = atan2(objects[number].obj_y - robot_state->y,
			    objects[number].obj_x - robot_state->x);
  else
    angle_to_target = 0.0;

  /* subtract safety distance */
  if (dist_to_target > robot_specifications->approach_object_distance)
    dist_to_target -= robot_specifications->approach_object_distance;
  else
    dist_to_target = 0.0;

  
  target_x = robot_state->x + (dist_to_target * cos(angle_to_target));
  target_y = robot_state->y + (dist_to_target * sin(angle_to_target));

  /*
   * Okay, let's stop the planner, and issue an action command for
   * picking up the object
   */


  action->status          = 1; /* 1=new command issued */
  action->type            = 4; /* 4=absolute target coordinates */
  action->continuous_ping = 1; /* 1=refresh necessary */
  action->target_pos_x    = target_x;
  action->target_pos_y    = target_y;

  program_state->approaching_mode           = 1;
  program_state->active_goal_number         = number;

  if (!program_state->looking_for_trash_bin_mode)
    G_display_switch(HUNTING_BUTTON, 2);
  else
    G_display_switch(HUNTING_BUTTON, 5);

  /*
   * Potential conflicts: the planner might not receive the stop_autonomous
   * quickly enough and issue a new target point. This should not 
   * hurt, however, since we keep re-issuing our action command
   */
}





/************************************************************************
 *
 *   NAME:         start_hunting_trash_bins
 *                 
 *   FUNCTION:     starts hunting trash bins.
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void start_hunting_trash_bins(ALL_PARAMS)
{
  int i;
  int num_bins;

  fprintf(stderr, "\t\t\tstart_hunting_trash_bins()\n");

  /*
   * Check if everything is consistent
   */

  if (!program_state->hunting_mode){
    fprintf(stderr, "ERROR: can only hunt trash bins in hunting mode.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }


  if (program_state->approaching_mode){
    fprintf(stderr, "ERROR: Cannot hunt trash bins and approach an object.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }

  if (program_state->looking_for_trash_bin_mode){
    fprintf(stderr, "ERROR: already looking for a trash bin .\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }


  if (program_state->autonomous_mode){
    fprintf(stderr, "ERROR: Planner is already working. On what?.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }


  if (program_state->n_carried_objetcs < MAX_OBJECTS_TO_CARRY){ /* soft bug */
    fprintf(stderr, "WARNING: Carrying only %d objects\n",
	    program_state->n_carried_objetcs);
    putc(7, stderr);
  }


  
  /*
   * Okay, let's use the planner to get to a trash bin
   */


  /*
   * Send all trash bins as goal points
   */

  tcx_remove_all_goals(ALL);

  num_bins = 0;
  if (robot_specifications->set_points_when_searching_bins)
    for (i = 0; i < n_objects; i++){
      if (objects[i].obj_status && objects[i].obj_type == VIS_OBJECT_BASKET){
	tcx_plan_goal_message(objects[i].obj_x, objects[i].obj_y,
			      objects[i].obj_type, 1, ALL);
	num_bins++;
      }
    }
  
  /*
   * Let the base be very fast
   */

  if (num_bins == 0)
    tcx_base_set_mode(FIND_DOOR_MODE, ALL);
  else
    tcx_base_set_mode(FAST_TRAVEL_MODE, ALL);

  /*
   * And activate our planner.
   */

  tcx_plan_start_autonomous(1, ALL);

  
  program_state->looking_for_trash_bin_mode = 1;
  program_state->active_goal_number = -1;

  G_display_switch(HUNTING_BUTTON, 4);

}



/************************************************************************
 *
 *   NAME:         finish_approaching_mode
 *                 
 *   FUNCTION:     handler that is called when BASE successfully
 *                 aproached an object
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void finish_approaching_mode(ALL_PARAMS)
{
  fprintf(stderr, "\t\t\tfinish_approaching_mode()\n");


  if (!program_state->hunting_mode){
    fprintf(stderr, "ERROR: finish_approaching encountered, but not hunting,\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }
 
  /*
   * now check if we have to pick up something or to dump something
   *
   * (those procedures have a more elaborate error checking....)
   */

  if (program_state->looking_for_trash_bin_mode)

    dump_object2(ALL);

  else

    pick_up_object(ALL);
}




/************************************************************************
 *
 *   NAME:         pick_up_object()
 *                 
 *   FUNCTION:     starts hunting trash bins.
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void pick_up_object(ALL_PARAMS)
{
  int obj_type;
  
  fprintf(stderr, "\t\t\tpick_up_object()\n");

  
  /*
   * Check if everything is consistent
   */

  if (!program_state->hunting_mode){
    fprintf(stderr, "ERROR: can only pick up objects in hunting mode.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }


  if (!program_state->approaching_mode){
    fprintf(stderr, "ERROR: Can only pick up an object after approaching it.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }

  if (program_state->looking_for_trash_bin_mode){
    fprintf(stderr, "ERROR: Picking up object while looking for trash bin.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }


  if (program_state->autonomous_mode){
    fprintf(stderr, "ERROR: Planner is working when picking up object. On what?.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }
  
  
  if (program_state->active_goal_number < 0 ||
      program_state->active_goal_number > n_objects){
    fprintf(stderr, "ERROR: Object %d not known in the database(%d).\n", 
	    program_state->active_goal_number, n_objects);
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }
  
  /* don't change. Is used below */
  obj_type = objects[program_state->active_goal_number].obj_type;


  if (obj_type < 0 || obj_type >= VIS_NUM_OBJECTS){
    fprintf(stderr, "ERROR: Object %d (type %d) cannot be picked up.\n", 
	    program_state->active_goal_number, obj_type);
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }


  if (obj_type == VIS_OBJECT_BASKET){
    fprintf(stderr, "Attempt to pick up a trash bin.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }


  if (program_state->n_carried_objetcs >= MAX_OBJECTS_TO_CARRY){ /* soft bug */
    fprintf(stderr, "WARNING: Carrying already %d objects.\n",
	    program_state->n_carried_objetcs);
    putc(7, stderr);
  }



  /*
   * Fine. let's pretend we pick up the object
   */
  
  G_display_switch(HUNTING_BUTTON, 3); 
  sleep(1); 
  tcx_speech_talk_text(ALL, pick_up_texts[obj_type]);
  /*tcx_base_stop_robot(ALL);*/
  sleep(2); 
  
  program_state->n_carried_objetcs++; 
  G_display_switch(NUM_OBJECTS_BUTTON, program_state->n_carried_objetcs);
  program_state->approaching_mode = 0;
  objects[program_state->active_goal_number].obj_status = 2; /* picked up */
  program_state->active_goal_number = -1;


  /*
   * Check, if we have to get to a trash bin now, or if we should search
   * for the next object
   */


  if (program_state->n_carried_objetcs < MAX_OBJECTS_TO_CARRY){
    /*
     * Less then MAX_OBJECTS_TO_CARRY objects. Let's keep hunting
     */
    
    start_hunting_objects(ALL); 
    
  }


  else{

    /*
     * Enough objects. Let's get to a trash bin
     */

    start_hunting_trash_bins(ALL);

  }
}



/************************************************************************
 *
 *   NAME:         dump_object()
 *                 
 *   FUNCTION:     dumps all objects
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void dump_object2(ALL_PARAMS)
{
  int obj_type;
  
  fprintf(stderr, "\t\t\tdump_object()\n");

  
  /*
   * Check if everything is consistent
   */

  if (!program_state->hunting_mode){
    fprintf(stderr, "ERROR: can only pick up objects in hunting mode.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }

  /*
  if (!program_state->approaching_mode){
    fprintf(stderr, "ERROR: Can only dump objects after approaching bin.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }
  */
  if (!program_state->looking_for_trash_bin_mode){
    fprintf(stderr, "ERROR: I have not even been looking for a trash bin.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }

  /*
  if (program_state->autonomous_mode){
    fprintf(stderr, "ERROR: Planner is working when dumping objects. On what?.\n");
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }
  */
#ifdef xxx
  if (program_state->active_goal_number < 0 ||
      program_state->active_goal_number > n_objects){
    fprintf(stderr, "ERROR: Object %d not known in the database(%d).\n", 
	    program_state->active_goal_number, n_objects);
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }
  
  /* don't change. Is used below */
  obj_type = objects[program_state->active_goal_number].obj_type;


  if (obj_type != VIS_OBJECT_BASKET){
    fprintf(stderr, "ERROR: Object %d is not a trash bin. Dump failed.\n", 
	    program_state->active_goal_number);
    putc(7, stderr);
    stop_hunting(ALL);
    return;
  }

#endif

  if (program_state->n_carried_objetcs < MAX_OBJECTS_TO_CARRY){ /* soft bug */
    fprintf(stderr, "WARNING: Dumping only %d objects.\n",
	    program_state->n_carried_objetcs);
    putc(7, stderr);
  }

  

  /*
   * Fine. let's pretend we dump all objects
   */
  
  tcx_plan_stop_autonomous(0, ALL);

  sleep(1);
  G_display_switch(HUNTING_BUTTON, 6);
  tcx_speech_talk_text(ALL, drop_text);
  /*tcx_base_stop_robot(ALL);*/
  sleep(4);

  program_state->n_carried_objetcs = 0;
  G_display_switch(NUM_OBJECTS_BUTTON, program_state->n_carried_objetcs);
  program_state->approaching_mode  = 0;
  program_state->looking_for_trash_bin_mode = 0;

  /*
   * Let's go hunting for more objects
   */
  
  start_hunting_objects(ALL); 
    
}




void dump_object(ALL_PARAMS)
{
  int index;
  
  index = find_nearest_basket_index(ALL);
  if (index == -1){
    fprintf(stderr, "STRANGE: No target bin found???\n");
    dump_object2(ALL);
  }
  

  tcx_plan_stop_autonomous(0, ALL); /* ..just in case */

  tcx_base_set_mode(APPROACH_OBJECT_MODE, ALL);


  /*
   * Okay, let's stop the planner, and issue an action command for
   * picking up the object
   */


  action->status          = 1; /* 1=new command issued */
  action->type            = 4; /* 4=absolute target coordinates */
  action->continuous_ping = 1; /* 1=refresh necessary */
  action->target_pos_x    = objects[index].obj_x;
  action->target_pos_y    = objects[index].obj_y;

  program_state->approaching_mode           = 1;
  program_state->active_goal_number         = index;

  G_display_switch(HUNTING_BUTTON, 5);

  if (program_state->auto_reset_expl_mode)
    tcx_plan_reset_exploration_table(ALL);

}

#endif /* UNIBONN */





