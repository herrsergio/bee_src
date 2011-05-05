
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









#define MAX_NUMBER_GOALS 20


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


int check_and_add_object(float obj_x, float obj_y, int obj_type,
			 ALL_PARAMS);

int check_if_point_visible(float robot_x, 
			   float robot_y,
			   float robot_orientation,
			   float point_x, 
			   float point_y,
			   int   camera,
			   ALL_PARAMS);

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
			    ALL_PARAMS);

void start_hunting(ALL_PARAMS);

void stop_hunting(ALL_PARAMS);

void start_hunting_objects(ALL_PARAMS);

void start_approach_object(int number, ALL_PARAMS);

void start_hunting_trash_bins(ALL_PARAMS);

void finish_approaching_mode(ALL_PARAMS);

void pick_up_object(ALL_PARAMS);

void dump_object(ALL_PARAMS);

void remove_objects_that_are_not_in_freespace(ALL_PARAMS);
