
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
#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#ifndef i386
#include <sys/unistd.h>
#endif
#include "all.h"


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


struct timeval BASE_last_send_message_time = {0, 0};

static struct timeval last_statistics_update = {0, 0};
static float  last_uncorr_robot_x;
static float  last_uncorr_robot_y;
static int    last_uncorr_robot_def = 0;

struct timeval last_map_update = {0,0};




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void 
wait_until_ready(struct timeval *last_time, long u_delay)
{ /* max: 1 sec*/
  struct timeval now;
  int ready = 1;
  
  do{
    if (!ready){
#ifdef TCX_debug
      fprintf(stderr, "+");
#endif
      usleep(u_delay/4);
    }
    gettimeofday(&now, NULL);
    ready = ( now.tv_sec >  last_time->tv_sec + 1 ||
	     (now.tv_sec == last_time->tv_sec + 1 &&
	      now.tv_usec + 1000000 > last_time->tv_usec + u_delay) ||
	     (now.tv_sec == last_time->tv_sec &&
	      now.tv_usec > last_time->tv_usec + u_delay)); 
  }
  while (!ready);
  
  last_time->tv_sec  = now.tv_sec;
  last_time->tv_usec = now.tv_usec;
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_base_subscribe(ALL_PARAMS, 
			int subscribe_status_report,
			int subscribe_sonar_report,
			int subscribe_colli_report,
			int subscribe_ir_report,
			int subscribe_laser_report)
{
  BASE_register_auto_update_type data;

  data.subscribe_status_report = subscribe_status_report;
  data.subscribe_sonar_report  = subscribe_sonar_report;
  data.subscribe_colli_report  = subscribe_colli_report;
  data.subscribe_ir_report     = subscribe_ir_report;
  data.subscribe_laser_report  = subscribe_laser_report;

#ifdef TCX_debug
  fprintf(stderr, "TCX: base_init_robot\n");
#endif

  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(BASE, "BASE_register_auto_update", &data);
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_base_get_robot_position(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: base_get_robot_position\n");
#endif

  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(BASE, "BASE_robot_position_query", NULL);

}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_base_set_mode(int mode, ALL_PARAMS)
{
  int m;
  BASE_setmode_type modeInfo;
  modeInfo.useSonar   = DONT_CHANGE;
  modeInfo.useLaser   = DONT_CHANGE;
  
#ifdef TCX_debug
  fprintf(stderr, "TCX: base_set_mode\n");
#endif
  m = mode;

#ifdef UNIBONN
  for (; m < 0;) m += NUMBER_OF_MODES;
  for (; m >= NUMBER_OF_MODES;) m -= NUMBER_OF_MODES;
#else
  for (; m < 0;) m += 3;
  for (; m >= 3;) m -= 3;
#endif
  
  modeInfo.modeNumber = m;

  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(BASE, "BASE_setmode", &modeInfo);
  
  G_display_switch(BASE_MODE_BUTTON, m);
  robot_state->base_mode = m;
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_base_stop_robot(ALL_PARAMS)
{
  int i;
#ifdef TCX_debug
  fprintf(stderr, "TCX: base_stop_robot\n");
#endif

  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(BASE, "BASE_stop_robot", NULL);

  i = G_return_num_markers(TARGET_POINT_GLOBAL, 0);
  G_clear_markers(TARGET_POINT_GLOBAL);
  if (i >= 1)
    display_global_robot(ALL);

  for (i = 0; i < G_return_num_markers(TARGET_POINT_LOCAL, 1); i++)
    G_undisplay_markers(TARGET_POINT_LOCAL, i, C_GREY90);
  G_clear_markers(TARGET_POINT_LOCAL);

}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_reset_joystick(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: base_reset_joystick\n");
#endif

  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(BASE, "BASE_reset_joystick", NULL);
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



void tcx_base_update_status(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: base_update_status\n");
#endif

  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(BASE, "BASE_update_status_query", NULL);
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_base_set_velocity(ALL_PARAMS, float trans_speed, float rot_speed)
{
  BASE_set_velocity_type tcx_velocity;

#ifdef TCX_debug
  fprintf(stderr, "TCX: base_set_velocity: %g %g\n", trans_speed, rot_speed);
#endif

  if (trans_speed < 0.0) trans_speed = 0.0;
  if (rot_speed < 0.0)   rot_speed = 0.0;

  tcx_velocity.rot_velocity   = robot_state->rot_set_velocity   = rot_speed;
  tcx_velocity.trans_velocity = robot_state->trans_set_velocity = trans_speed;
  
  
  if (program_state->tcx_connected_to_BASE){
    wait_until_ready(&BASE_last_send_message_time, 250000); /* at most 4
							     * messages per
							     * sec*/
    tcxSendMsg(BASE, "BASE_set_velocity", &tcx_velocity);
  }

  G_display_value(ROBOT_TRANS_SET_VELOCITY, robot_state->trans_set_velocity);
  G_display_value(ROBOT_ROT_SET_VELOCITY, robot_state->rot_set_velocity);

}




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_base_set_acceleration(ALL_PARAMS, float trans_accel, float rot_accel)
{
  BASE_set_acceleration_type tcx_acceleration;

#ifdef TCX_debug
  fprintf(stderr, "TCX: base_set_acceleration: %g %g\n", 
	  trans_accel, rot_accel);
#endif

  if (trans_accel < 0.0) trans_accel = 0.0;
  if (rot_accel < 0.0)   rot_accel = 0.0;

  tcx_acceleration.rot_acceleration   = robot_state->rot_set_acceleration 
    = rot_accel;
  tcx_acceleration.trans_acceleration = robot_state->trans_set_acceleration 
    = trans_accel;
  
  
  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(BASE, "BASE_set_acceleration", &tcx_acceleration);

  G_display_value(ROBOT_TRANS_SET_ACCELERATION, 
		  robot_state->trans_set_acceleration);
  G_display_value(ROBOT_ROT_SET_ACCELERATION, 
		  robot_state->rot_set_acceleration);

}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_base_rotate_clockwise(ALL_PARAMS)
{

#ifdef TCX_debug
  fprintf(stderr, "TCX: base_rotate_clockwise.\n");
#endif

  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(BASE, "BASE_rotate_clockwise", NULL);
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




void tcx_base_rotate_anticlockwise(ALL_PARAMS)
{

#ifdef TCX_debug
  fprintf(stderr, "TCX: base_rotate_anticlockwise.\n");
#endif

  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(BASE, "BASE_rotate_anticlockwise", NULL);
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



void tcx_base_translate_forward(ALL_PARAMS)
{

#ifdef TCX_debug
  fprintf(stderr, "TCX: base_translate_forward.\n");
#endif

  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(BASE, "BASE_translate_forward", NULL);
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




void tcx_base_translate_backward(ALL_PARAMS)
{

#ifdef TCX_debug
  fprintf(stderr, "TCX: base_translate_backward.\n");
#endif

  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(BASE, "BASE_translate_backward", NULL);
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_base_goto_relative(ALL_PARAMS)
{
  BASE_goto_relative_type tcx_coord;
  int i;

#ifdef TCX_debug
  fprintf(stderr, "TCX: base_goto_relative: %g %g\n",
	  - action->target_pos_x, action->target_pos_y);
#endif


  tcx_coord.rel_target_x = - action->target_pos_x;
  tcx_coord.rel_target_y = action->target_pos_y;
  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(BASE, "BASE_goto_relative", &tcx_coord);
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_base_goto_absolute(int new_target, ALL_PARAMS)
{

  BASE_goto_absolute_type tcx_coord;
  int i;
  float dummy;			/* captures an orientation that 
				 * won't be used anyhow. */
  
  
  compute_backward_correction(action->target_pos_x,
			      action->target_pos_y,
			      0.0, /* orientation does not matter here! */
			      robot_state->correction_parameter_x,
			      robot_state->correction_parameter_y,
			      robot_state->correction_parameter_angle,
			      robot_state->correction_type,
			      &(tcx_coord.abs_target_x),
			      &(tcx_coord.abs_target_y),
			      &dummy);
  

  tcx_coord.new_target = new_target;

#ifdef TCX_debug
  fprintf(stderr, "TCX: base_goto_absolute: %g %g (corrected: %g %g)\n",
	  tcx_coord.abs_target_x, tcx_coord.abs_target_y,
	  action->target_pos_x, action->target_pos_y);
#endif

  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(BASE, "BASE_goto_absolute", &tcx_coord);
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_base_approach_absolute(int new_target, ALL_PARAMS)
{

  BASE_approach_absolute_type tcx_coord;
  int i;
  float dummy;			/* captures an orientation that 
				 * won't be used anyhow. */
  
  
  compute_backward_correction(action->target_pos_x,
			      action->target_pos_y,
			      0.0, /* orientation does not matter here! */
			      robot_state->correction_parameter_x,
			      robot_state->correction_parameter_y,
			      robot_state->correction_parameter_angle,
			      robot_state->correction_type,
			      &(tcx_coord.abs_target_x),
			      &(tcx_coord.abs_target_y),
			      &dummy);
  

  tcx_coord.new_target = new_target;
  tcx_coord.approach_dist = robot_specifications->dist_action_achieved;
  tcx_coord.mode = robot_state->base_mode;

#ifdef TCX_debug
  fprintf(stderr, "TCX: base_approach_absolute: %g %g (corrected: %g %g)\n",
	  tcx_coord.abs_target_x, tcx_coord.abs_target_y,
	  action->target_pos_x, action->target_pos_y);
#endif

  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(BASE, "BASE_approach_absolute", &tcx_coord);
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_base_approach_two_points_absolute(int new_target, ALL_PARAMS)
{

  BASE_approach_absolute_two_points_type tcx_coord;
  float dummy;			/* captures an orientation that 
				 * won't be used anyhow. */
  int i;
  
  compute_backward_correction(action->target_pos_x,
			      action->target_pos_y,
			      0.0, /* orientation does not matter here! */
			      robot_state->correction_parameter_x,
			      robot_state->correction_parameter_y,
			      robot_state->correction_parameter_angle,
			      robot_state->correction_type,
			      &(tcx_coord.abs_target_x1),
			      &(tcx_coord.abs_target_y1),
			      &dummy);
  

  tcx_coord.new_target = new_target;
  tcx_coord.approach_dist = robot_specifications->dist_action_achieved;
  tcx_coord.abs_target_x2 = tcx_coord.abs_target_x1;
  tcx_coord.abs_target_y2 = tcx_coord.abs_target_y1;

#ifdef TCX_debug
  fprintf(stderr, "TCX: base_approach_absolute_2_points: %g %g (corr %g %g), dist %g\n",
	  tcx_coord.abs_target_x1, tcx_coord.abs_target_y1,
	  action->target_pos_x, action->target_pos_y, tcx_coord.approach_dist);
#endif

  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(BASE, "BASE_approach_absolute_two_points", &tcx_coord);

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_base_disconnect(ALL_PARAMS)	/* don't call from user routines */
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: base_disconnect.\n");
#endif


  if (program_state->tcx_connected_to_BASE){
    tcxSendMsg(BASE, "BASE_disconnect", NULL);
  }
}




/*********************************************************************\
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
\*********************************************************************/





void tcx_sonar_get_values(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: sonar_get_values\n");
#endif

  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(SONAR, "SONAR_sonar_query", NULL);
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



void tcx_sonar_switch_on(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: sonar_switch_on\n");
#endif

  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(SONAR, "SONAR_switch_on", NULL);
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef dd
/*DDD*/
void tcx_sonar_test(ALL_PARAMS)
{
  place 
  int test_mask[] = {
    0x1000
    };
  
  if (program_state->tcx_connected_to_BASE) {
    tcxSendMsg(SONAR, "SONAR_define_mask", 
    tcxSendMsg(SONAR, "SONAR_switch_on", NULL);
  }
  else fprintf(stderr, "not connected!!!\n");
}
#endif

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



void tcx_sonar_switch_off(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: sonar_switch_off\n");
#endif

  if (program_state->tcx_connected_to_BASE)
    tcxSendMsg(SONAR, "SONAR_switch_off", NULL);
}




/*********************************************************************\
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
\*********************************************************************/



void tcx_speech_talk_text(ALL_PARAMS, char *text)
{
#ifdef UNIBONN
#ifdef TCX_debug
  fprintf(stderr, "TCX: speech_talk_text\n");
#endif

  if (program_state->tcx_connected_to_SPEECH)
    tcxSendMsg(SPEECH, "SPEECH_talk_text", &text);
#endif
}




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_speech_disconnect(ALL_PARAMS)	/* don't call from user routines */
{
#ifdef UNIBONN
#ifdef TCX_debug
  fprintf(stderr, "TCX: speech_disconnect.\n");
#endif

  ;
  
  /* no action necessary for disconnecting from SPEECH */
#endif
}





/*********************************************************************\
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
\*********************************************************************/


void tcx_pantilt_init(ALL_PARAMS)
{
  int auto_reply_position = 1;

#ifdef TCX_debug
  fprintf(stderr, "TCX: pantilt_init.\n");
#endif

  

  if (program_state->tcx_connected_to_PANTILT)
    tcxSendMsg(PANTILT, "PANTILT_init_query", &auto_reply_position);
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_pantilt_move_absolute(ALL_PARAMS, float pan2, float tilt2)
{
  PANTILT_move_type data;

/*#ifdef TCX_debug*/
  fprintf(stderr, "TCX: pantilt_move_absolute: %g %g\n", pan2, tilt2);
/*#endif*/

  tcx_pantilt_stop_tracking(ALL);

  if (program_state->tcx_connected_to_PANTILT){
    data.pan_target  = pan2;
    data.tilt_target = tilt2;
    tcxSendMsg(PANTILT, "PANTILT_move", &data);
  }

  sensation->tilt = tilt2;
  sensation->pan  = pan2;
 
  G_display_value(CAMERA_TILT, sensation->tilt);
  G_display_value(CAMERA_PAN,  -sensation->pan);
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_pantilt_set_velocity(ALL_PARAMS, 
			      float pan_speed, float tilt_speed)
{
  PANTILT_set_velocity_type data;

#ifdef TCX_debug
  fprintf(stderr, "TCX: pantilt_set_velocity: %g %g\n", 
	  pan_speed, tilt_speed);
#endif

  if (program_state->tcx_connected_to_PANTILT){
    data.pan_velocity  = pan_speed;
    data.tilt_velocity = tilt_speed;
    tcxSendMsg(PANTILT, "PANTILT_set_velocity", &data);
  }
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_pantilt_set_acceleration(ALL_PARAMS, 
				  float pan_accel, float tilt_accel)
{
  PANTILT_set_acceleration_type data;

#ifdef TCX_debug
  fprintf(stderr, "TCX: pantilt_set_acceleration: %g %g\n", 
	  pan_accel, tilt_accel);
#endif


  if (program_state->tcx_connected_to_PANTILT){
    data.pan_acceleration  = pan_accel;
    data.tilt_acceleration = tilt_accel;
    tcxSendMsg(PANTILT, "PANTILT_set_acceleration", &data);
  }
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_pantilt_track_point(ALL_PARAMS, 
			     float x, float y)
{
  PANTILT_track_point_type point;
  float dummy;			/* captures an orientation that 
				 * won't be used anyhow. */

#ifndef TCX_debug
  fprintf(stderr, "TCX: pantilt_track_point: %g %g\n", x, y);
#endif
  
  


  if (program_state->tcx_connected_to_PANTILT){
    compute_backward_correction(x, y, -90.0,
				robot_state->correction_parameter_x,
				robot_state->correction_parameter_y,
				robot_state->correction_parameter_angle,
				robot_state->correction_type,
				&(point.x),
				&(point.y),
				&dummy);
    point.height = /*5*/0.0;
    fprintf(stderr, "PANTILT tracking: %6.4f %6.4f (%6.4f %6.4f)\n",
	    x, y, point.x, point.y);
    tcxSendMsg(PANTILT, "PANTILT_track_point", &point);
  }
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




void tcx_pantilt_stop_tracking(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: pantilt_stop_tracking\n");
#endif


  if (program_state->tcx_connected_to_PANTILT)
    tcxSendMsg(PANTILT, "PANTILT_stop_tracking", NULL);
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



void tcx_pantilt_terminate(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: pantilt_terminate.\n");
#endif

  if (program_state->tcx_connected_to_PANTILT)
    tcxSendMsg(PANTILT, "PANTILT_terminate", NULL);
}





/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_pantilt_disconnect(ALL_PARAMS)	/* don't call from user routines */
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: pantilt_disconnect.\n");
#endif


  if (program_state->tcx_connected_to_PANTILT){
    tcxSendMsg(PANTILT, "PANTILT_disconnect", NULL);
  }
}




/*********************************************************************\
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
\*********************************************************************/




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_camera_disconnect(ALL_PARAMS)	/* don't call from user routines */
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: camera_disconnect.\n");
#endif

  /*
   * new framegrabber needs no termination command
   *

  if (program_state->tcx_connected_to_CAMERA){
    tcxSendMsg(CAMERA, "CAMERA_terminate", NULL);
  }
  */
}




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



void tcx_camera_x_display_on(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: camera_x_display_on.\n");
#endif
  /*
   * I doubt this is installed for the new framegrabber
   *
  if (program_state->tcx_connected_to_CAMERA){
    G_display_switch(CAMERA_X_DISPLAY_BUTTON, 1);
    tcxSendMsg(CAMERA, "CAMERA_x_display_on", NULL);
  }
  */
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



void tcx_camera_x_display_off(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: camera_x_display_off.\n");
#endif

  /*
   * I doubt this is installed for the new framegrabber
   *
  if (program_state->tcx_connected_to_CAMERA){
    G_display_switch(CAMERA_X_DISPLAY_BUTTON, 0);
    tcxSendMsg(CAMERA, "CAMERA_x_display_off", NULL);
  }
  */
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



void tcx_camera_register_auto_update(ALL_PARAMS, int subscribe_image)
{
  CAMERA_register_auto_update_type camera_subscribe;

#ifdef TCX_debug
  fprintf(stderr, "TCX: camera_register_auto_update.\n");
#endif

  
  camera_subscribe.image = subscribe_image;
  camera_subscribe.numGrabber = 0; /* always use first grabber */
  camera_subscribe.image_xsize = PSEUDO_PICTURE_SIZE_HORIZONTAL;
  camera_subscribe.image_ysize = PSEUDO_PICTURE_SIZE_VERTICAL;
  
  if (program_state->tcx_connected_to_CAMERA)
    tcxSendMsg(CAMERA, "CAMERA_register_auto_update", &camera_subscribe);

}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



void tcx_camera_initialize_marker(ALL_PARAMS)
{
  /* CAMERA_initialize_marker_type marker_info;*/
  int i;

#ifdef TCX_debug
  fprintf(stderr, "TCX: camera_initialize_marker.\n");
#endif

  /*
   * I doubt this is installed for the new framegrabber
   *

  if (program_state->tcx_connected_to_CAMERA){

    marker_info.from_x       = 0;
    marker_info.to_x         = X_SIZE;
    marker_info.from_y       = 0;
    marker_info.to_y         = Y_SIZE;
    marker_info.resolution_x = 4;
    marker_info.resolution_y = 6;
    marker_info.range_x      = 12;
    marker_info.range_y      = 42;
    marker_info.cut_off      = 3;
    marker_info.threshold    = 350;
    for (i = 0; i < NUM_MARKERS; i++)
      marker_info.markers[i] = 1;

    tcxSendMsg(CAMERA, "CAMERA_initialize_marker", &marker_info);
  }
  */
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



void tcx_camera_find_marker(ALL_PARAMS, int camera)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: camera_find_marker.\n");
#endif

  /*
   * I doubt this is installed for the new framegrabber
   *

  if (program_state->tcx_connected_to_CAMERA){
    if (camera == 0)
      G_display_switch(LEFT_CAMERA_BUTTON, 1);
    else
      G_display_switch(RIGHT_CAMERA_BUTTON, 1);

    tcxSendMsg(CAMERA, "CAMERA_find_marker_query", &camera);
  }
  */
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_camera_image(ALL_PARAMS, int camera)
{
  CAMERA_image_query_type data;

#ifdef TCX_debug
  fprintf(stderr, "TCX: camera_grey_image.\n");
#endif
  
  if (program_state->tcx_connected_to_CAMERA){
    data.numGrabber    = 0;	/* always use first grabber */
    data.xsize         = PSEUDO_PICTURE_SIZE_HORIZONTAL;
    data.ysize         = PSEUDO_PICTURE_SIZE_VERTICAL;
    tcxSendMsg(CAMERA, "CAMERA_image_query", &data);
  }
}


/*********************************************************************\
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
\*********************************************************************/

void tcx_map_quit(ALL_PARAMS)
{

#ifdef TCX_debug
  fprintf(stderr, "TCX: map_quit.\n");
#endif

  if (program_state->tcx_connected_to_MAP)
    tcxSendMsg(MAP, "MAP_quit", NULL);
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_map_partial_map_query(ALL_PARAMS, int first_x, int first_y,
			       int size_x, int size_y)
{
  MAP_partial_map_query_type data;

#ifdef TCX_debug
  fprintf(stderr, "TCX: map_partial_map_query.\n");
#endif

  data.first_x   = first_x;
  data.first_y   = first_y;
  data.size_x    = size_x;
  data.size_y    = size_y;
  data.resolution = robot_specifications->map_resolution;
  
  if (program_state->tcx_connected_to_MAP)
    tcxSendMsg(MAP, "MAP_partial_map_query", &data);
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_map_register_auto_update(ALL_PARAMS, 
				  int subscribe_maps,
				  int subscribe_position_correction)
{
  MAP_register_auto_update_type data;

#ifdef TCX_debug
  fprintf(stderr, "TCX: map_register_auto_update.\n");
#endif


  data.subscribe_maps               = subscribe_maps;
  data.subscribe_position_correction = subscribe_position_correction;

  if (program_state->tcx_connected_to_MAP)
    tcxSendMsg(MAP, "MAP_register_auto_update", &data);
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_map_dump(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: map_dump.\n");
#endif

  if (program_state->tcx_connected_to_MAP)
    tcxSendMsg(MAP, "MAP_dump", NULL);
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_map_correction_parameters_query(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: map_correction_parameters_query.\n");
#endif

  if (program_state->tcx_connected_to_MAP)
    tcxSendMsg(MAP, "MAP_correction_parameters_query", NULL);
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_map_enable_map_update(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: map_enable_map_update.\n");
#endif

  if (program_state->tcx_connected_to_MAP)
    tcxSendMsg(MAP, "MAP_enable_map_update", NULL);
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_map_disable_map_update(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: map_disable_map_update.\n");
#endif

  if (program_state->tcx_connected_to_MAP)
    tcxSendMsg(MAP, "MAP_disable_map_update", NULL);
}



void clear_all_sensor_maps(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: map_clear_all_sensor_maps.\n");
#endif

  if (program_state->tcx_connected_to_MAP)
    tcxSendMsg(MAP, "MAP_clear_all_sensor_maps", NULL);
}




/*********************************************************************\
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
\*********************************************************************/


void tcx_plan_register_auto_update(ALL_PARAMS, 
				   int subscribe_status)
{
  PLAN_register_auto_update_type data;
#ifdef TCX_debug
  fprintf(stderr, "TCX: plan_register_auto_update\n");
#endif

  data.subscribe_status = subscribe_status;

  if (program_state->tcx_connected_to_PLAN)
    tcxSendMsg(PLAN, "PLAN_register_auto_update", &data);
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



void tcx_plan_goal_message(float x, float y, int name, int add, ALL_PARAMS)
{
  PLAN_goal_message_type goal;
  
#ifdef TCX_debug
  if (add)
    fprintf(stderr, "TCX: plan_goal_message: add %g %g (%d)\n",
	    x, y, name);
  else
    fprintf(stderr, "TCX: plan_goal_message: remove %g %g (%d)\n",
	    x, y, name);
#endif
  
  if (program_state->tcx_connected_to_PLAN){
    goal.x = x;
    goal.y = y;
    goal.max_radius = 0.0;
    goal.reward = 0.0;
    goal.name = name;
    goal.add = add;
    
    tcxSendMsg(PLAN, "PLAN_goal_message", &goal);
  }
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


#ifdef TOURGUIDE_VERSION


void tcx_remove_all_goals(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: plan_remove_all_goals.\n");
#endif
  
  if (program_state->tcx_connected_to_PLAN)
    tcxSendMsg(PLAN, "PLAN_remove_all_goals", NULL);
}

#endif /* TOURGUIDE_VERSION */




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



void tcx_plan_start_autonomous(int expl, ALL_PARAMS)
{

#ifdef TCX_debug
  fprintf(stderr, "TCX: plan_start_autonomous.\n");
#endif


  if (program_state->tcx_connected_to_PLAN){
    tcxSendMsg(PLAN, "PLAN_start_autonomous_message", &expl);
    program_state->robot_pos_x_when_going_autonomous = robot_state->x;
    program_state->robot_pos_y_when_going_autonomous = robot_state->y;
    program_state->autonomous_mode = 1;
    G_display_switch(AUTONOMOUS_BUTTON, 1);
  }
  else
    fprintf(stderr, "Need a planner for autunomous motion.\n");
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



void tcx_plan_action_query(ALL_PARAMS)
{
  PLAN_action_query_type pos;
#ifdef TCX_debug
  fprintf(stderr, "TCX: plan_action_query.\n");
#endif

  if (program_state->tcx_connected_to_PLAN){
    pos.x = robot_state->x;
    pos.y = robot_state->y;
    pos.orientation = robot_state->orientation;
    pos.stuck = 0;
    tcxSendMsg(PLAN, "PLAN_action_query", &pos);
  }
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_plan_stop_autonomous(int stop_base, ALL_PARAMS)
{

#ifdef TCX_debug
  fprintf(stderr, "TCX: plan_stop_autonomous.\n");
#endif


  if (program_state->tcx_connected_to_PLAN){
    tcxSendMsg(PLAN, "PLAN_stop_autonomous_message", &stop_base);
    program_state->autonomous_mode = 0;
    G_display_switch(AUTONOMOUS_BUTTON, 0);
  }
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_plan_new_robot_pos(ALL_PARAMS)
{
  PLAN_new_robot_pos_message_type data;

#ifdef TCX_debug
  fprintf(stderr, "TCX: plan_new_robot_pos.\n");
#endif


  data.x            = robot_state->x;
  data.y            = robot_state->y;
  data.orientation  = robot_state->orientation;

  if (program_state->tcx_connected_to_PLAN)
    tcxSendMsg(PLAN, "PLAN_new_robot_pos_message", &data);
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



void tcx_plan_quit(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: plan_quit.\n");
#endif

  if (program_state->tcx_connected_to_PLAN)
    tcxSendMsg(PLAN, "PLAN_quit_message", NULL);
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



void tcx_plan_reset_exploration_table(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: plan_reset_exploration_table.\n");
#endif

  if (program_state->tcx_connected_to_PLAN)
    tcxSendMsg(PLAN, "PLAN_reset_exploration_table", NULL);
}


#ifdef UNIBONN

/*********************************************************************\
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
\*********************************************************************/


void tcx_arm_register_auto_update(ALL_PARAMS, 
				  int subscribe_status)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: arm_register_auto_update\n");
#endif
}




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_arm_move_out(ALL_PARAMS, float force)
{

#ifdef TCX_debug
  fprintf(stderr, "TCX: arm_move_out.\n");
#endif


  if (program_state->tcx_connected_to_ARM){
    if (!robot_state->arm_moved_out){
      tcxSendMsg(ARM, "ARM_move_out", &force);
      robot_state->arm_moved_out = 1;
      G_display_switch(ARM_MOVE_OUT_IN_BUTTON, 
		       robot_state->arm_moved_out);
    }
    else
      fprintf(stderr, "ARM is already outside. No action taken.\n");
  }

  else
    fprintf(stderr, "Connect to ARM first. No action taken.\n");
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_arm_move_in(ALL_PARAMS, float force)
{

#ifdef TCX_debug
  fprintf(stderr, "TCX: arm_move_in.\n");
#endif

  if (program_state->tcx_connected_to_ARM){
    if (robot_state->arm_moved_out){
      tcxSendMsg(ARM, "ARM_move_in", &force);
      robot_state->arm_moved_out = 0;
      G_display_switch(ARM_MOVE_OUT_IN_BUTTON, 
		       robot_state->arm_moved_out);
    }
    else
      fprintf(stderr, "MoveIn: ARM is not outside. No action taken.\n");
  }
  else
    fprintf(stderr, "Connect to ARM first. No action taken.\n");
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_arm_pickup(ALL_PARAMS)
{

#ifdef TCX_debug
  fprintf(stderr, "TCX: arm_pickup.\n");
#endif

  if (program_state->tcx_connected_to_ARM){
    if (!robot_state->arm_moved_out){
      robot_state->arm_moved_out = 1;
      G_display_switch(ARM_MOVE_OUT_IN_BUTTON, 
		       robot_state->arm_moved_out);
    }
   tcxSendMsg(ARM, "ARM_pickup_at_ground_query", NULL); 
   robot_state->pick_sequence = 1;
   G_display_switch(ARM_PICK_BUTTON, robot_state->pick_sequence);
   robot_state->arm_gripper_closed = 0;
   G_display_switch(ARM_CLOSE_OPEN_GRIPPER_BUTTON, robot_state->arm_gripper_closed);
  }
  else
    fprintf(stderr, "Connect to ARM first. No action taken.\n");
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/
void tcx_arm_lift(ALL_PARAMS)
{

#ifdef TCX_debug
  fprintf(stderr, "TCX: arm_pickup.\n");
#endif

  if (program_state->tcx_connected_to_ARM){
    if (robot_state->arm_moved_out){
      tcxSendMsg(ARM, "ARM_lift_object_query", NULL); 
      robot_state->pick_sequence = 2;
      G_display_switch(ARM_PICK_BUTTON, robot_state->pick_sequence);
      robot_state->arm_gripper_closed = 1;
      G_display_switch(ARM_CLOSE_OPEN_GRIPPER_BUTTON, robot_state->arm_gripper_closed);
    }
    else
      fprintf(stderr, "Lift: Move out the ARM first. No action taken. \n");
  }
  else
    fprintf(stderr, "Connect to ARM first. No action taken.\n");
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_arm_drop(ALL_PARAMS)
{

#ifdef TCX_debug
  fprintf(stderr, "TCX: arm_pickup.\n");
#endif

  if (program_state->tcx_connected_to_ARM){
    if (robot_state->arm_moved_out){
      tcxSendMsg(ARM, "ARM_drop_object_query", NULL); 
      robot_state->pick_sequence = 0;
      G_display_switch(ARM_PICK_BUTTON, robot_state->pick_sequence);
      robot_state->arm_gripper_closed = 0;
      G_display_switch(ARM_CLOSE_OPEN_GRIPPER_BUTTON, robot_state->arm_gripper_closed);
    }
    else
      fprintf(stderr, "Drop: Move out the ARM first. No action taken. \n");
  }
  else
    fprintf(stderr, "Connect to ARM first. No action taken.\n");
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_arm_open_gripper(ALL_PARAMS)
{

#ifdef TCX_debug
  fprintf(stderr, "TCX: arm_open_gripper.\n");
#endif


  if (program_state->tcx_connected_to_ARM){
    if (robot_state->arm_gripper_closed){
      tcxSendMsg(ARM, "ARM_gripper_open", NULL);
      robot_state->arm_gripper_closed = 0;
      G_display_switch(ARM_CLOSE_OPEN_GRIPPER_BUTTON, 
		       robot_state->arm_gripper_closed);
    }
    else
      fprintf(stderr, "GRIPPER is already open. No action taken.\n");
  }

  else
    fprintf(stderr, "Connect to ARM first. No action taken.\n");
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_arm_close_gripper(ALL_PARAMS)
{

#ifdef TCX_debug
  fprintf(stderr, "TCX: arm_close_gripper.\n");
#endif


  if (program_state->tcx_connected_to_ARM){
    if (!robot_state->arm_gripper_closed){
      tcxSendMsg(ARM, "ARM_gripper_close", NULL);
      robot_state->arm_gripper_closed = 1;
      G_display_switch(ARM_CLOSE_OPEN_GRIPPER_BUTTON, robot_state->
		       arm_gripper_closed);
    }
    else
      fprintf(stderr, "GRIPPER is already open. No action taken.\n");
  }

  else
    fprintf(stderr, "Connect to ARM first. No action taken.\n");
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_arm_park_gripper(ALL_PARAMS)
{

#ifdef TCX_debug
  fprintf(stderr, "TCX: arm_park_gripper.\n");
#endif

  
  if (program_state->tcx_connected_to_ARM){
    tcxSendMsg(ARM, "ARM_gripper_park", NULL);
    robot_state->arm_gripper_closed = 0;
    G_display_switch(ARM_CLOSE_OPEN_GRIPPER_BUTTON, robot_state->
		     arm_gripper_closed);
  }
  
  else
    fprintf(stderr, "Connect to ARM first. No action taken.\n");
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_arm_set_mast_absolute(ALL_PARAMS, float height)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: arm_set_mast_absolute %g.\n", height);
#endif

  G_display_value(ARM_MAST_POSITION_BUTTON, height);

  if (program_state->tcx_connected_to_ARM){
    tcxSendMsg(ARM, "ARM_mast_move_to_percentage", &height);
  }
  else
    fprintf(stderr, "Connect to ARM first. No action taken.\n");
}
  

 
/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_arm_set_gripper_orientation_absolute(ALL_PARAMS, float orientation)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: arm_set_gripper_orientation_absolute %g.\n", 
	  orientation);
#endif

  G_display_value(ARM_GRIPPER_ORIENTATION_BUTTON, orientation);

  if (program_state->tcx_connected_to_ARM){
    tcxSendMsg(ARM, "ARM_wrist_rotate_to", &orientation);
  }
  else
    fprintf(stderr, "Connect to ARM first. No action taken.\n");
}
 

/*********************************************************************\
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
\*********************************************************************/


void tcx_sunvis_register_auto_update(int objects, ALL_PARAMS)
{
  SUNVIS_register_auto_update_type msg;

#ifdef TCX_debug
  fprintf(stderr, "TCX: sunvis_register_auto_update.\n");
#endif
  
  if (program_state->tcx_connected_to_SUNVIS){
    msg.subscribe_objects = objects;
    if (objects)
      G_display_switch(CAMERA_OBJECTS_BUTTON, 1);
    else
      G_display_switch(CAMERA_OBJECTS_BUTTON, 0);
    tcxSendMsg(SUNVIS, "SUNVIS_register_auto_update", &msg);
  }
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_sunvis_send_occupancy_maps_on(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: sunvis_send_occupancy_maps_on.\n");
#endif
  
  if (program_state->tcx_connected_to_SUNVIS){
    G_display_switch(CAMERA_MAPS_BUTTON, 1);
    tcxSendMsg(SUNVIS, "SUNVIS_send_occupancy_maps_on", NULL);
  }
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_sunvis_send_occupancy_maps_off(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: sunvis_send_occupancy_maps_off.\n");
#endif
  
  if (program_state->tcx_connected_to_SUNVIS){
    G_display_switch(CAMERA_MAPS_BUTTON, 0);
    tcxSendMsg(SUNVIS, "SUNVIS_send_occupancy_maps_off", NULL);
  }
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_sunvis_send_collision_info_on(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: sunvis_send_collision_info_on.\n");
#endif
  
  if (program_state->tcx_connected_to_SUNVIS){
    G_display_switch(CAMERA_COLLI_LINES_BUTTON, 1);
    tcxSendMsg(SUNVIS, "SUNVIS_send_collision_info_on", NULL);
  }
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_sunvis_send_collision_info_off(ALL_PARAMS)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: sunvis_send_collision_info_off.\n");
#endif
  
  if (program_state->tcx_connected_to_SUNVIS){
    G_display_switch(CAMERA_COLLI_LINES_BUTTON, 0);
    tcxSendMsg(SUNVIS, "SUNVIS_send_collision_info_off", NULL);
  }
}







/****************************************************************************
 * tracker_Order                                                            *
 ****************************************************************************/

void tracker_Order (int Order)
{
  EZR_order_type o;
  
  o.SenderID = 42;		/* magic number */
  o.Order    = Order;
  
  tcxSendMsg (TRACKER, "EZR_order", &o);
}

/****************************************************************************
 * tracker_Command                                                          *
 ****************************************************************************/


void tracker_Command(int SenderID,
		     int MainCommand,
		     int SubCommand, 
		     int Parameter1,
		     int Parameter2,
		     float Parameter3,
		     float Parameter4)
{
  EZR_command_type c;
  
  
  c.SenderID = SenderID;
  c.MainCommand = MainCommand;
  c.SubCommand = SubCommand;
  c.Parameter1 = Parameter1;
  c.Parameter2 = Parameter2;
  c.Parameter3 = Parameter3;
  c.Parameter4 = Parameter4;
  
  tcxSendMsg (TRACKER, "EZR_command", &c);
}


/****************************************************************************
 * tracker_start_tracking                                                   *
 ****************************************************************************/


void tracker_init(ALL_PARAMS)
{
  tracker_Order(T_BASE_STATUS);	/* initializes the tracker */
  tracker_Order(T_RPT_STATUS);	/* initializes the tracker */
  tracker_Order(T_RPT_FORWARD);	/* initializes the tracker */
  tracker_Order(T_TRANSLATION);	/* initializes the tracker */
  tracker_Order(T_ROTATION);	/* initializes the tracker */
  tracker_Order(T_Y_LOCKING);	/* initializes the tracker */
}



void tracker_start(ALL_PARAMS)
{
  tracker_Order(T_INIT);	/* initializes the tracker */
  tracker_Order(T_START);	/* initializes the tracker */
}


void tracker_stop(ALL_PARAMS)
{
  tracker_Order(T_STOP);	/* initializes the tracker */
}



#endif /* UNIBONN */


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


#ifdef TOURGUIDE_VERSION

void CD_play_track(ALL_PARAMS, int track_no, 
		   int initial_sound, 
		   int final_sound)
{
#ifdef CD_VERSION
  CD_play_part_query_type dat;
  CD_play_msg_query_type  dat2;
  int msgs[3], i;
  int button_nr;

  if (!program_state->tcx_connected_to_CD){
    program_state->do_connect_to_CD = 1;
    connect_to_tcx(ALL);
  }

  if (program_state->tcx_connected_to_CD){
#ifdef MANUAL_SELECTION
    dat.start_frame = dat.end_frame = 0;
    for (;;){
      printf("Start track? ");
      fflush(stdout);
      scanf("%d", &(dat.start_track));
      printf("Start time?  ");
      fflush(stdout);
      scanf("%d", &(dat.start_time));
      /* printf("Start frame? ");
	 fflush(stdout);
	 scanf("%d", &(dat.start_frame)); */
      dat.end_track = dat.start_track;
      /* printf("End track?   ");
	 fflush(stdout);
	 scanf("%d", &(dat.end_track)); */
      printf("End time?    ");
      fflush(stdout);
      scanf("%d", &(dat.end_time));
      /*printf("End frame?   ");
	fflush(stdout);
	scanf("%d", &(dat.end_frame));*/
      dat.initial_gong = 0;
      printf("{%d, %d, %d, %d, %d, %d, 0, \"\", \"\"},\n",
	     dat.start_track, dat.start_time, 
	     dat.start_frame, dat.end_track, dat.end_time, dat.end_frame);
      
      tcxSendMsg(CD, "CD_play_part_query", &dat);
    }
#endif
    dat2.num_msgs = 0;
    if (initial_sound > -1)
      msgs[dat2.num_msgs++] = initial_sound;
    msgs[dat2.num_msgs++] = track_no;
    if (final_sound > -1)
      msgs[dat2.num_msgs++] = final_sound;

    dat2.msgs = &(msgs[0]);
    
    if (program_state->last_cd_phrase != -1)
      G_display_switch(CD_BUTTONS[program_state->last_cd_phrase], 0);
    button_nr = -1;
    for (i = 0; i < NUM_CD_ROWS && button_nr == -1; i++)
      if (CD_TEXTS[i].msg == track_no)
	button_nr = i;
    if (button_nr >= 0 && button_nr < NUM_CD_ROWS){
      G_display_switch(CD_BUTTONS[button_nr], 1);
      program_state->last_cd_phrase = button_nr;
    }
    else
      program_state->last_cd_phrase = -1;
    
    tcxSendMsg(CD, "CD_play_msg_query", &dat2);
  }
#else
  fprintf(stderr, "Received CD_play_track message: %d %d %d\n",
	  track_no, initial_sound, final_sound);
#endif /* CD_VERSION */
}
#endif /* TOURGUIDE_VERSION */




#ifdef CD_VERSION



void CD_play_part_reply_handler(TCX_REF_PTR ref,
                                CD_play_part_reply_ptr data)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a CD_play_part_reply message.\n");
#endif

  tcxFree("CD_play_part_reply", data);
}






void CD_total_track_number_reply_handler(TCX_REF_PTR ref,
                                         CD_total_track_number_reply_ptr data)
{
  fprintf(stderr, "TCX: Received a CD_total_track_number_reply message.\n");

  tcxFree("CD_total_track_number_reply", data);
}
                                         
void CD_track_length_reply_handler(TCX_REF_PTR ref,
                                   CD_track_length_reply_ptr data)
{
  fprintf(stderr, "TCX: Received a CD_track_length_reply message.\n");

  tcxFree("CD_track_length_reply", data);
}
                                   
void CD_actual_position_reply_handler(TCX_REF_PTR ref,
                                      CD_actual_position_reply_ptr data)                                         
{
  fprintf(stderr, "TCX: Received a CD_actual_position_reply message.\n");

  tcxFree("CD_actual_position_reply", data);
}

void CD_play_track_reply_handler(TCX_REF_PTR ref,
                                 CD_play_track_reply_ptr data)
{
  fprintf(stderr, "TCX: Received a CD_play_track_reply message.\n");

  tcxFree("CD_play_track_reply", data);
}


void CD_play_msg_reply_handler(TCX_REF_PTR ref,
		       CD_play_msg_reply_ptr data)
{
  PROGRAM_STATE_PTR        program_state        = allGlobal.program_state;


#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a CD_play_msg_reply message.\n");
#endif

  if (program_state->last_cd_phrase != -1)
    G_display_switch(CD_BUTTONS[program_state->last_cd_phrase], 0);
  program_state->last_cd_phrase = -1;

  tcxFree("CD_play_msg_reply", data);
}
#endif /* CD_VERSION */



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef UNIBONN



void flow_request_info(ALL_PARAMS)
{
  if (program_state->tcx_connected_to_FLOW)
    tcxSendMsg(FLOW, "FLOW_tracking_info_query", NULL);
}


#endif /* UNIBONN */




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void buttons_init(ALL_PARAMS)
{
  int nstatus = 1;

  if (program_state->tcx_connected_to_BUTTONS){
    tcxSendMsg(BUTTONS, "BUTTONS_register_auto_update", &nstatus);

  }
}



void set_buttons(ALL_PARAMS, int new) /* new: if -1, all
				       * internal values will be
				       * set. 
				       * 0=red, 1=yellow, 2=green, 3=blue */
{
  BUTTONS_set_lights_type data;

  
  if (new < 0 || new == 0){
    if (robot_state->red_light_status == 0)
      data.red_light_status = BUTTON_LIGHT_STATUS_OFF;
    else if (robot_state->red_light_status == 1)
      data.red_light_status = BUTTON_LIGHT_STATUS_ON;
    else if (robot_state->red_light_status == 2)
      data.red_light_status = BUTTON_LIGHT_STATUS_FLASHING;
    else
      fprintf(stderr, "STRANGE: Unknown modus %s in red_light_status.\n",
	      robot_state->red_light_status);
    G_display_switch(RED_LIGHT_BUTTON, robot_state->red_light_status);
  }
  else
    data.red_light_status = BUTTON_LIGHT_STATUS_DONT_CHANGE;
  
  
  if (new < 0 || new == 1){
    if (robot_state->yellow_light_status == 0)
      data.yellow_light_status = BUTTON_LIGHT_STATUS_OFF;
    else if (robot_state->yellow_light_status == 1)
      data.yellow_light_status = BUTTON_LIGHT_STATUS_ON;
    else if (robot_state->yellow_light_status == 2)
      data.yellow_light_status = BUTTON_LIGHT_STATUS_FLASHING;
    else
      fprintf(stderr, "STRANGE: Unknown modus %s in yellow_light_status.\n",
	      robot_state->yellow_light_status);
    G_display_switch(YELLOW_LIGHT_BUTTON, robot_state->yellow_light_status);
  }
  else
    data.yellow_light_status = BUTTON_LIGHT_STATUS_DONT_CHANGE;
  
  
  if (new < 0 || new == 2){
    if (robot_state->green_light_status == 0)
      data.green_light_status = BUTTON_LIGHT_STATUS_OFF;
    else if (robot_state->green_light_status == 1)
      data.green_light_status = BUTTON_LIGHT_STATUS_ON;
    else if (robot_state->green_light_status == 2)
      data.green_light_status = BUTTON_LIGHT_STATUS_FLASHING;
    else
      fprintf(stderr, "STRANGE: Unknown modus %s in green_light_status.\n",
	      robot_state->green_light_status);
    G_display_switch(GREEN_LIGHT_BUTTON, robot_state->green_light_status);
  }
  else
    data.green_light_status = BUTTON_LIGHT_STATUS_DONT_CHANGE;
  
  
  if (new < 0 || new == 3){
    if (robot_state->blue_light_status == 0)
      data.blue_light_status = BUTTON_LIGHT_STATUS_OFF;
    else if (robot_state->blue_light_status == 1)
      data.blue_light_status = BUTTON_LIGHT_STATUS_ON;
    else if (robot_state->blue_light_status == 2)
      data.blue_light_status = BUTTON_LIGHT_STATUS_FLASHING;
    else
      fprintf(stderr, "STRANGE: Unknown modus %s in blue_light_status.\n",
	      robot_state->blue_light_status);
    G_display_switch(BLUE_LIGHT_BUTTON, robot_state->blue_light_status);
  }
  else
    data.blue_light_status = BUTTON_LIGHT_STATUS_DONT_CHANGE;


  data.left_kill_switch_light_status  = BUTTON_LIGHT_STATUS_OFF;
  data.right_kill_switch_light_status = BUTTON_LIGHT_STATUS_OFF;

  if (program_state->tcx_connected_to_BUTTONS)
    tcxSendMsg(BUTTONS, "BUTTONS_set_lights", &data);
  
}

void 
buttons_simulate_button_press(ALL_PARAMS, int number)
{
  int n;

  n = number;

  if (program_state->tcx_connected_to_BUTTONS)
    tcxSendMsg(BUTTONS, "BUTTONS_simulate_button_press", &n);
  
}


void
buttons_effect(ALL_PARAMS)
{
  robot_state->red_light_status    = 2;
  robot_state->green_light_status  = 2;
  robot_state->yellow_light_status = 2;
  robot_state->blue_light_status   = 2;


  if (program_state->tcx_connected_to_BUTTONS)
    tcxSendMsg(BUTTONS, "BUTTONS_effect", NULL);

  set_buttons(ALL, -1);

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         BASE_robot_position_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				       BASE_robot_position_reply_ptr pos)
{

  allGlobal.program_state->something_happened = 1; /* important for main loop!
						    * must be in every hadler*/
#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a BASE_robot_position_reply message.\n");
  fprintf(stderr, "contents: %g %g %g\n", pos->x, pos->y, pos->orientation);
#endif

  tcxFree("BASE_robot_position_reply", pos);
}






/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         BASE_update_status_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_update_status_reply_handler(TCX_REF_PTR                   ref,
				      BASE_update_status_reply_ptr status)
{
  ROBOT_SPECIFICATIONS_PTR robot_specifications = 
    allGlobal.robot_specifications;
  PROGRAM_STATE_PTR        program_state        = allGlobal.program_state;
  ROBOT_STATE_PTR          robot_state          = allGlobal.robot_state;
  ACTION_PTR               action               = allGlobal.action;
  SENSATION_PTR            sensation            = allGlobal.sensation;
  static int first_status_report_received = 0;
  static int prev_path_def = 0;
  static float prev_path_x, prev_path_y;

  int add_a_marker, change_pos_text, i;
  char pos_txt[80];
  float prev_robot_state_orientation = robot_state->orientation;
  float corr_robot_x, corr_robot_y, corr_robot_orientation;
  float advancement, worldsize;

#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a BASE_update_status_reply message.\n");
  fprintf(stderr, "time stamp: %g robot: %g %g %g\n", status->time,
	  status->pos_x, status->pos_y, status->orientation);
#endif
  
    /*... position, corrected */

    compute_forward_correction(status->pos_x, 
			       status->pos_y, 
			       90.0 - status->orientation,
			       robot_state->correction_parameter_x,
			       robot_state->correction_parameter_y,
			       robot_state->correction_parameter_angle,
			       robot_state->correction_type,
			       &corr_robot_x,
			       &corr_robot_y,
			       &corr_robot_orientation);


  if (!robot_state->state_is_known ||
      fabs(robot_state->x - corr_robot_x)
      + fabs(robot_state->y - corr_robot_y) < 300.0){ /* how could we go more
							* than that in a second
							* ??*/
    
    add_a_marker = (robot_state->x != status->pos_x ||
		    robot_state->y != status->pos_y);

    robot_state->x                        = corr_robot_x;
    robot_state->y                        = corr_robot_y;
    robot_state->orientation              = corr_robot_orientation;
    
    /* center the robot location */
    if (!first_status_report_received){
      worldsize = robot_specifications->global_worldsize_x;
      if (robot_specifications->global_worldsize_y < worldsize)
	worldsize = robot_specifications->global_worldsize_y;
      autoshift_display(robot_state->x, 
			robot_state->y, 
			0.5 * worldsize,
		      0.5 * worldsize,
			ALL);
      first_status_report_received = 1;
    }

    /* ... floats */
    robot_state->last_update_time         = status->time;
    robot_state->rot_set_acceleration     = status->rot_acceleration;
    robot_state->rot_velocity             = status->rot_current_speed;
    robot_state->rot_set_velocity         = status->rot_set_speed;
    robot_state->rot_position             = status->rot_position;
    robot_state->trans_set_acceleration   = status->trans_acceleration;
    robot_state->trans_velocity           = status->trans_current_speed; 
    robot_state->trans_set_velocity       = status->trans_set_speed;
    robot_state->trans_position           = status->trans_position;

    
    /* ... ints */
    robot_state->trans_direction          = (int) status->trans_direction;
    robot_state->trans_set_direction      = (int) status->trans_set_direction;
    robot_state->rot_direction            = (int) status->rot_direction;
    robot_state->rot_set_direction        = (int) status->rot_set_direction;
    robot_state->rot_moving               = (int) status->rot_moving;
    robot_state->trans_moving             = (int) status->trans_moving;
    robot_state->bumpers                  = (int) status->bumpers;
    robot_state->bump                     = (int) status->bump;
    robot_state->emergency                = (int) status->emergency;
    robot_state->emergencyProcedure       = (int) status->emergencyProcedure;
    robot_state->state_is_known           = (int) 1;
    
    for (;robot_state->orientation > 360.0;)
      robot_state->orientation -= 360.0;
    for (;robot_state->orientation < 0.0;)
      robot_state->orientation += 360.0;
    
    change_pos_text = (add_a_marker || 
		       robot_state->orientation 
		       != prev_robot_state_orientation);
    
    
    
    if (change_pos_text){
      sprintf(pos_txt, "[x:%5.1f y:%5.1f o:%4.1f]",
	      robot_state->x, robot_state->y, robot_state->orientation);
      G_set_new_text(POSITION_TEXT_BUTTON, pos_txt, 0);
    }
    
    
    

    if (change_pos_text || add_a_marker)
      display_robot_window(ALL);
    
    if (add_a_marker){
      if (prev_path_def &&
	  sqrt(((robot_state->x - prev_path_x) * 
		(robot_state->x - prev_path_x)) +
	       ((robot_state->y - prev_path_y) * 
		(robot_state->y - prev_path_y)) > 300.0)){
	G_clear_markers(PATH);
      }
      prev_path_def = 1;
      prev_path_x = robot_state->x;
      prev_path_y = robot_state->y;
      G_add_marker(PATH, robot_state->x, robot_state->y, 0);
    }
    

    /*
     * check, if we terminate an action here
     */

#ifdef junk
    target_dist = sqrt(((action->target_pos_x - robot_state->x)
			* (action->target_pos_x - robot_state->x))
		       + ((action->target_pos_y - robot_state->y)
			  * (action->target_pos_y - robot_state->y)));
    if (action->type == 4 && action->status == 2){
      if (target_dist <= robot_specifications->dist_action_achieved){
	action->status          = 1; /* 1=new command issued */
	action->type            = 1; /* 1=stop */
	action->continuous_ping = 0; /* 0=ping not necessary */
	initiate_action(ALL);	/* stops the robot */

	program_state->approaching_mode = 0; /*! hack */

      }
    }
#endif
    /*
     * update statistics
     */
    if (!last_uncorr_robot_def)
      advancement = 0.0;
    else
      advancement = sqrt(((last_uncorr_robot_x - status->pos_x) 
			* (last_uncorr_robot_x - status->pos_x))
			 + ((last_uncorr_robot_y - status->pos_y) 
			    * (last_uncorr_robot_y - status->pos_y)));

    update_internal_statistics(0, advancement, ALL);

    last_uncorr_robot_def = 1;
    last_uncorr_robot_x = status->pos_x;
    last_uncorr_robot_y = status->pos_y;

#ifdef big_
    /*!!! BIG HACK */
    if (program_state->autonomous_mode && robot_state->state_is_known){/*!*/
      dist = 
	sqrt(((program_state->robot_pos_x_when_going_autonomous 
	       - robot_state->x)
	      * (program_state->robot_pos_x_when_going_autonomous
		 - robot_state->x))
	     + ((program_state->robot_pos_y_when_going_autonomous
		 - robot_state->y)
		* (program_state->robot_pos_y_when_going_autonomous
		   - robot_state->y)));
      if (dist > 400.0 && dist < 700.0 && program_state->autonomous_mode &&
	  robot_state->base_mode != DEFAULT_MODE){
	tcx_base_set_mode(DEFAULT_MODE, ALL);
	putc(7, stderr);
      }
      if (dist <= 400.0 && program_state->autonomous_mode &&
	  robot_state->base_mode != FIND_DOOR_MODE){
	tcx_base_set_mode(FIND_DOOR_MODE, ALL);
	putc(7, stderr);
      }
    }
#endif    
  }
  else{
    fprintf(stderr, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! %g %g !!!!!!!!!!\n",
	    status->pos_x, status->pos_y);
    robot_state->state_is_known = 0;
  }

  program_state->something_happened = 1; /* important for main loop!
					  * must be in every hadler*/
    
  tcxFree("BASE_update_status_reply", status);

}




/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         BASE_action_executed_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
					BASE_action_executed_reply_ptr data)
{
  ROBOT_SPECIFICATIONS_PTR robot_specifications = 
    allGlobal.robot_specifications;
  PROGRAM_STATE_PTR        program_state        = allGlobal.program_state;
  ROBOT_STATE_PTR          robot_state          = allGlobal.robot_state;
  ACTION_PTR               action               = allGlobal.action;
  SENSATION_PTR            sensation            = allGlobal.sensation;
  int i;

#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a BASE_action_executed message.\n");
#endif
  /*  putc(7, stderr); */

  action->status           = 0;	/* no command active */
  action->type             = 0;
  robot_state->in_motion   = 0;
  action->continuous_ping  = 0;
  G_display_switch(IN_MOTION_BUTTON, robot_state->in_motion);

#ifdef UNIBONN
  if (program_state->approaching_mode)
    finish_approaching_mode(ALL);
#endif /* UNIBONN */
#ifdef TOURGUIDE_VERSION
  if (program_state->giving_tour && program_state->tour_modus == 3)
    reached_a_tour_goal(ALL);
#endif /* TOURGUIDE_VERSION */


  /*
   * remove target points
   */
  for (i = 0; i < G_return_num_markers(TARGET_POINT_LOCAL, 1); i++)
    G_undisplay_markers(TARGET_POINT_LOCAL, i, C_GREY90);
  G_clear_markers(TARGET_POINT_LOCAL);

  for (i = 0; i < G_return_num_markers(TARGET_POINT_GLOBAL, 1); i++)
    G_undisplay_markers(TARGET_POINT_GLOBAL, i, C_GREY90);
  if (i > 0){
    putc(7, stderr);
    G_clear_markers(TARGET_POINT_GLOBAL);
    display_global_robot(ALL);
  }



  tcxFree("BASE_action_executed_reply", data);
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

void SONAR_sonar_reply_handler(TCX_REF_PTR          ref,
			      SONAR_sonar_reply_ptr sonar)
{
  ROBOT_SPECIFICATIONS_PTR robot_specifications = 
    allGlobal.robot_specifications;
  PROGRAM_STATE_PTR        program_state        = allGlobal.program_state;
  ROBOT_STATE_PTR          robot_state          = allGlobal.robot_state;
  ACTION_PTR               action               = allGlobal.action;
  SENSATION_PTR            sensation            = allGlobal.sensation;

  int i;

#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a SONAR_sonar_reply message.\n");
  for (i = 0; i < robot_specifications->num_sonar_sensors; i++)
    fprintf(stderr, " %5.2f", sonar->values[i]);
  fprintf(stderr, "\n");
#endif

  for (i = 0; i < robot_specifications->num_sonar_sensors; i++){
    if (sonar->values[i] < 0.0)
      sensation->sonar_values[i] = robot_specifications->robot_size;
    else if (sonar->values[i] > robot_specifications->max_sonar_range - 
	     robot_specifications->robot_size ||
	sonar->values[i] < robot_specifications->min_sonar_range)
      sensation->sonar_values[i] = robot_specifications->max_sonar_range;
    else
      sensation->sonar_values[i] = sonar->values[i]
	+ robot_specifications->robot_size;
  }

#ifdef UNIBONN
  if (!robot_state->sonar_on){
    robot_state->sonar_on = 1;
    G_display_switch(SONAR_BUTTON, robot_state->sonar_on);
  }
#endif /* UNIBONN */

  tcxFree("SONAR_sonar_reply", sonar);

  display_robot_window(ALL);


  program_state->something_happened = 1; /* important for main loop!
					  * must be in every hadler*/
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
  PROGRAM_STATE_PTR        program_state        = allGlobal.program_state;



#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a SONAR_ir_reply message.\n");
#endif

  tcxFree("SONAR_ir_reply", ir);

  program_state->something_happened = 1; /* important for main loop!
					  * must be in every hadler*/
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

void LASER_laser_reply_handler(TCX_REF_PTR          ref,
			      LASER_laser_reply_ptr laser)
{
  ROBOT_SPECIFICATIONS_PTR robot_specifications = 
    allGlobal.robot_specifications;
  PROGRAM_STATE_PTR        program_state        = allGlobal.program_state;
  ROBOT_STATE_PTR          robot_state          = allGlobal.robot_state;
  ACTION_PTR               action               = allGlobal.action;
  SENSATION_PTR            sensation            = allGlobal.sensation;

  int i, j;
  float value;


#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a LASER_laser_reply message: %d+%d.\n",
	  laser->f_numberOfReadings, laser->r_numberOfReadings);
#endif


  for (i = 0; i < robot_specifications->num_laser_sensors; i++){
    j = i % (robot_specifications->num_laser_sensors / 2);
    if (i < robot_specifications->num_laser_sensors / 2){
      if (laser->f_numberOfReadings > 0)
	value = laser->f_reading[j];
      else
	value = -1.0;
    }
    else{
      if (laser->r_numberOfReadings > 0)
	value = laser->r_reading[j];
      else
	value = -1.0;
    }

    if (value < 0.0)
      sensation->laser_values[i] = robot_specifications->robot_size;
    else if (value > robot_specifications->max_laser_range - 
	     robot_specifications->robot_size ||
	     value < robot_specifications->min_laser_range)
      sensation->laser_values[i] = robot_specifications->max_laser_range;
    else
      sensation->laser_values[i] = value + robot_specifications->robot_size;
  }


  tcxFree("LASER_laser_reply", laser);

  display_robot_window(ALL);

  program_state->something_happened = 1; /* important for main loop!
					  * must be in every hadler*/

}


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         IR_ir_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void IR_ir_reply_handler(TCX_REF_PTR                ref,
			 IR_ir_reply_ptr      data)
{
  fprintf(stderr, "TCX: Received a IR_ir_reply message\n");
  tcxFree("IR_ir_reply", data);
}


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         COLLI_colli_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void COLLI_colli_reply_handler(TCX_REF_PTR                ref,
			       COLLI_colli_reply_ptr      data)
{
  fprintf(stderr, "TCX: Received a COLLI_colli_reply message\n");
  tcxFree("COLLI_colli_reply", data);
}

#ifdef UNIBONN

/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         SPEECH_init_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SPEECH_init_reply_handler(TCX_REF_PTR   ref,
			       int          *err_value)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a SPEECH_init_reply message: %d.\n",
	  *err_value);
#endif
}




/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         SPEECH_is_speaking_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SPEECH_is_speaking_reply_handler(TCX_REF_PTR   ref,
				      int          *speaking)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a SPEECH_is_speaking_reply message: %d.\n",
	  *speaking);
#endif
}

#endif



/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         PANTILT_init_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void PANTILT_init_reply_handler(TCX_REF_PTR   ref,
				int          *data)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: Received PANTILT_init_reply a message.\n");
  fprintf(stderr, "contents: %d\n", *data);
#endif

  if (!(*data))
    disconnect_PANTILT(allGlobal.robot_state, allGlobal.action, 
		       allGlobal.sensation, allGlobal.program_state,
		       allGlobal.robot_specifications);

  tcxFree("PANTILT_init_reply", data);
}



/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         PANTILT_position_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void PANTILT_position_reply_handler(TCX_REF_PTR                ref,
				    PANTILT_position_reply_ptr data)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a PANTILT_position_reply message.\n");
  fprintf(stderr, "contents: %g %g\n", data->pan_pos, data->tilt_pos);
#endif

  tcxFree("PANTILT_position_reply", data);
}



/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         PANTILT_limits_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void PANTILT_limits_reply_handler(TCX_REF_PTR              ref,
				  PANTILT_limits_reply_ptr data)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a PANTILT_limits_reply message.\n");
  fprintf(stderr, "contents: %g %g %g %g %g %g\n", 
	  data->pan_min_angle, data->pan_max_angle, data->tilt_min_angle, 
	  data->tilt_max_angle, data->pan_max_velocity, 
	  data->tilt_max_velocity);
#endif

  tcxFree("PANTILT_limits_reply", data);
}



/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         PANTILT_status_update_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void PANTILT_status_update_handler(TCX_REF_PTR              ref,
				  PANTILT_status_update_ptr data)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a PANTILT_status_update_handler message.\n");
  fprintf(stderr, "contents: %g %g %g %g %g %g %d\n", 
	  data->pan_pos, data->tilt_pos, data->pan_velocity,
	  data->tilt_velocity, data->pan_acceleration, data->tilt_acceleration,
	  data->error);
#endif
  allGlobal.sensation->pan =  data->pan_pos; /* update internal values */
  allGlobal.sensation->tilt = data->tilt_pos;
  G_display_value(CAMERA_PAN, -allGlobal.sensation->pan); /* update display */
  G_display_value(CAMERA_TILT, allGlobal.sensation->tilt);

  tcxFree("PANTILT_status_update", data);
}


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         PANTILT_status_update_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/




void PLAN_status_reply_handler(TCX_REF_PTR              ref,
			       PLAN_status_reply_ptr    status)
{
  ROBOT_SPECIFICATIONS_PTR robot_specifications = 
    allGlobal.robot_specifications;
  PROGRAM_STATE_PTR        program_state        = allGlobal.program_state;
  ROBOT_STATE_PTR          robot_state          = allGlobal.robot_state;
  ACTION_PTR               action               = allGlobal.action;
  SENSATION_PTR            sensation            = allGlobal.sensation;

  int i;
  static int prev_num_goals = 0;

#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a PLAN_status_reply message. %d goals\n",
	  status->goal_name);
#endif

#ifdef UNIBONN
  if (program_state->graphics_initialized)
    G_undisplay_markers(GOALS, -1, C_GREY90);

  G_clear_markers(GOALS);
  
  fprintf(stderr, "\n\t\tPLANNER: %d -> %d goals.\n\n",
	  prev_num_goals, status->num_goals);
  
  if (status->num_goals < prev_num_goals
      && program_state->hunting_mode 
      && program_state->looking_for_trash_bin_mode)
    dump_object(ALL);


  
  for (i = 0; i < status->num_goals; i++)
    if (status->goal_mode && status->goal_visible[i])
      G_add_marker(GOALS, status->goal_x[i], status->goal_y[i], 
		   status->goal_name[i]);

  if (program_state->graphics_initialized)
    G_display_markers(GOALS);
#endif /* UNIBONN */

#ifdef TOURGUIDE_VERSION
  /*fprintf(stderr, "\n\t\tPLANNER: %d -> %d goals.\n\n",
	  prev_num_goals, status->num_goals);*/
  

  if (program_state->giving_tour && status->num_goals == 0 &&
      program_state->tour_modus == 2)
    reached_a_tour_goal(ALL);


  prev_num_goals = status->num_goals;
#endif /* TOURGUIDE_VERSION */

     /*
     G_display_switch(AUTONOMOUS_BUTTON, status->autonomous);
     fprintf(stderr, "$$$$$ %d $$$$$$$$\n", status->autonomous);
     */

  G_display_switch(AUTONOMOUS_BUTTON, status->autonomous);

  if (status->autonomous && !program_state->autonomous_mode)
    program_state->autonomous_mode=1;
  else if (!status->autonomous && program_state->autonomous_mode)
    program_state->autonomous_mode=0;
  
  tcxFree("PLAN_status_reply", status);
}





/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         TCX_close_handler
 *                 
 *   FUNCTION:     handles a close message
 *                 
 *   PARAMETERS:   char *name       Name of the module that shut down
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void tcx_close_handler(char *name)
{
  ROBOT_SPECIFICATIONS_PTR robot_specifications = 
    allGlobal.robot_specifications;
  PROGRAM_STATE_PTR        program_state        = allGlobal.program_state;
  ROBOT_STATE_PTR          robot_state          = allGlobal.robot_state;
  ACTION_PTR               action               = allGlobal.action;
  SENSATION_PTR            sensation            = allGlobal.sensation;




  if (!strcmp(name, TCX_BASE_MODULE_NAME)){ /* BASE shut down */
    EZX_bell();
    /*stop_hunting(ALL);*/
    program_state->tcx_connected_to_BASE = 0;
    program_state->do_connect_to_BASE = 0;
    G_display_switch(CONNECT_BASE_BUTTON, 
		     program_state->tcx_connected_to_BASE);
    robot_state->sonar_on = 0;
#ifdef UNIBONN
    G_display_switch(SONAR_BUTTON, robot_state->sonar_on);
#endif /* UNIBONN */
  }

  else if (!strcmp(name, TCX_PANTILT_MODULE_NAME)){ /* PANTILT shut down */
    EZX_bell();
    program_state->tcx_connected_to_PANTILT = 0;
    program_state->do_connect_to_PANTILT = 0;
    G_display_switch(CONNECT_PANTILT_BUTTON, 
		     program_state->tcx_connected_to_PANTILT);
  }

#ifdef UNIBONN
  else if (!strcmp(name, TCX_SPEECH_MODULE_NAME)){ /* SPEECH shut down */
    EZX_bell();
    program_state->tcx_connected_to_SPEECH = 0;
    program_state->do_connect_to_SPEECH = 0;
    G_display_switch(CONNECT_SPEECH_BUTTON, 
		     program_state->tcx_connected_to_SPEECH);
  }
#endif

  else if (!strcmp(name, TCX_CAMERA_MODULE_NAME)){ /* CAMERA shut down */
    EZX_bell();
    program_state->tcx_connected_to_CAMERA = 0;
    program_state->do_connect_to_CAMERA = 0;
    G_display_switch(CONNECT_CAMERA_BUTTON, 
		     program_state->tcx_connected_to_CAMERA);
    G_display_switch(ACQUIRE_CAMERA_IMAGE_BUTTON, 0);
    G_display_switch(CONTINUOUS_CAMERA_IMAGE_BUTTON, 0);
#ifdef UNIBONN
    G_display_switch(CAMERA_X_DISPLAY_BUTTON, 0);
    G_display_switch(LEFT_CAMERA_BUTTON, 0);
    G_display_switch(RIGHT_CAMERA_BUTTON, 0);
#endif /* UNIBONN */
  }
  else if (!strcmp(name, TCX_BUTTONS_MODULE_NAME)){ /* BUTTONS shut down */
    program_state->tcx_connected_to_BUTTONS = 0;
    program_state->do_connect_to_BUTTONS = 0;
    G_display_switch(CONNECT_BUTTONS_BUTTON, 
		     program_state->tcx_connected_to_BUTTONS);
  }

#ifdef CD_VERSION
  else if (!strcmp(name, TCX_CD_MODULE_NAME)){ /* CD shut down */
    if (program_state->last_cd_phrase != -1)
      G_display_switch(CD_BUTTONS[program_state->last_cd_phrase], 0);
    program_state->last_cd_phrase = -1;
    program_state->tcx_connected_to_CD = 0;
    program_state->do_connect_to_CD = 0;
    G_display_switch(CONNECT_CD_BUTTON, 
		     program_state->tcx_connected_to_CD);
  }
#endif /* CD_VERSION */
#ifdef UNIBONN
  else if (!strcmp(name, TCX_SUNVIS_MODULE_NAME)){ /* SUNVIS shut down */
    EZX_bell();
    /*stop_hunting(ALL);*/
    program_state->tcx_connected_to_SUNVIS = 0;
    program_state->do_connect_to_SUNVIS = 0;
    G_display_switch(CONNECT_SUNVIS_BUTTON, 
		     program_state->tcx_connected_to_SUNVIS);
    G_display_switch(CAMERA_OBJECTS_BUTTON, 0);
    G_display_switch(CAMERA_COLLI_LINES_BUTTON, 2);
    G_display_switch(CAMERA_MAPS_BUTTON, 2);
  }
  else if (!strcmp(name, TCX_TRACKER_MODULE_NAME)){ /* TRACKER shut down */
    EZX_bell();
    program_state->tcx_connected_to_TRACKER = 0;
    program_state->do_connect_to_TRACKER = 0;
    G_display_switch(TRACKER_BUTTON, 0);
    G_display_switch(CONNECT_TRACKER_BUTTON, 
		     program_state->tcx_connected_to_TRACKER);
  }
  else if (!strcmp(name, TCX_FLOW_MODULE_NAME)){ /* FLOW shut down */
    program_state->tcx_connected_to_FLOW = 0;
    program_state->do_connect_to_FLOW = 0;
    G_display_switch(CONNECT_FLOW_BUTTON, 
		     program_state->tcx_connected_to_FLOW);
  }
  else if (!strcmp(name, TCX_ARM_MODULE_NAME)){ /* ARM shut down */
    EZX_bell();
    program_state->tcx_connected_to_ARM = 0;
    program_state->do_connect_to_ARM = 0;
    robot_state->arm_moved_out = 0;
    G_display_switch(ARM_MOVE_OUT_IN_BUTTON, 
		     robot_state->arm_moved_out);
    robot_state->arm_gripper_closed        = 0;
    G_display_switch(ARM_CLOSE_OPEN_GRIPPER_BUTTON, 
		     robot_state->arm_gripper_closed);
    robot_state->pick_sequence=0;
    G_display_switch(ARM_PICK_BUTTON,
                     robot_state->pick_sequence);
    G_display_switch(CONNECT_ARM_BUTTON, 
		     program_state->tcx_connected_to_ARM);
  }

#endif /* UNIBONN */

  else if (!strcmp(name, TCX_MAP_MODULE_NAME)){ /* MAP shut down */
    EZX_bell();
    program_state->tcx_connected_to_MAP = 0;
    program_state->do_connect_to_MAP = 0;
    clear_maps(ALL);
    G_display_switch(CONNECT_MAP_BUTTON, 
		     program_state->tcx_connected_to_MAP);
  }

  else if (!strcmp(name, TCX_PLAN_MODULE_NAME)){ /* PLAN shut down */
    EZX_bell();
    /*stop_hunting(ALL);*/
    program_state->tcx_connected_to_PLAN = 0;
    program_state->do_connect_to_PLAN = 0;
    G_display_switch(CONNECT_PLAN_BUTTON, 
		     program_state->tcx_connected_to_PLAN);
    /*
       if (program_state->???){
       tcx_base_stop_robot(ALL);
       usleep(100000);
       tcx_base_stop_robot(ALL);
       }
       */
  }

  else if (!strcmp(name, TCX_BASESERVER_MODULE_NAME)){ /* BASESERVER shut down */
    EZX_bell();
    program_state->tcx_connected_to_BASESERVER = 0;
    program_state->do_connect_to_BASESERVER = 0;
    G_display_switch(CONNECT_BASESERVER_BUTTON, 
		     program_state->tcx_connected_to_BASESERVER);
  }

  else if (!strcmp(name, TCX_SIMULATOR_MODULE_NAME)){ /* SIMULATOR shut down */
    EZX_bell();
    program_state->tcx_connected_to_SIMULATOR = 0;
    program_state->do_connect_to_SIMULATOR = 0;
    G_display_switch(CONNECT_SIMULATOR_BUTTON, 
		     program_state->tcx_connected_to_SIMULATOR);
  }

  else if (!strcmp(name, TCX_SONARINT_MODULE_NAME)){ /* SONARINT shut down */
    EZX_bell();
    program_state->tcx_connected_to_SONARINT = 0;
    program_state->do_connect_to_SONARINT = 0;
    G_display_switch(CONNECT_SONARINT_BUTTON, 
		     program_state->tcx_connected_to_SONARINT);
  }

  else if (!strcmp(name, TCX_LASERINT_MODULE_NAME)){ /* LASERINT shut down */
    EZX_bell();
    program_state->tcx_connected_to_LASERINT = 0;
    program_state->do_connect_to_LASERINT = 0;
    G_display_switch(CONNECT_LASERINT_BUTTON, 
		     program_state->tcx_connected_to_LASERINT);
  }


  else if (!strcmp(name, "TCX Server")){ /* or TCX shut down */
    EZX_bell();
    interrupt_handler(-1);	/* quit! */
  }

  else 
    printf("TCX: shut down %s, no action taken.\n", name);
}
     


#ifdef UNIBONN

/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         ARM_position_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void ARM_position_reply_handler(TCX_REF_PTR            ref,
				ARM_position_reply_ptr pos)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: Received an ARM_position_reply message.\n");
#endif


  tcxFree("ARM_position_reply", pos);
}

/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         ARM_update_status_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void ARM_update_status_reply_handler(TCX_REF_PTR                   ref,
				     ARM_update_status_reply_ptr   status)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: Received an ARM_update_status_reply message.\n");
#endif

  tcxFree("ARM_update_status_reply", status);
}

/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         ARM_action_executed_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void ARM_action_executed_reply_handler(TCX_REF_PTR                   ref,
				       ARM_action_executed_reply_ptr data)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: Received an ARM_action_executed_reply message.\n");
#endif

  tcxFree("ARM_action_executed_reply", data);
}




/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         ARM_pickup_at_ground_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void ARM_pickup_at_ground_reply_handler(TCX_REF_PTR                   ref,
				       ARM_pickup_at_ground_reply_ptr data)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: Received an ARM_pickup_at_ground_reply message.\n");
#endif

  tcxFree("ARM_pickup_at_ground_reply", data);
}




/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         ARM_move_in_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void ARM_move_in_reply_handler(TCX_REF_PTR                   ref,
				       ARM_move_in_reply_ptr data)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: Received an ARM_move_in_reply message.\n");
#endif

  tcxFree("ARM_move_in_reply", data);
}




/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         ARM_drop_object_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void ARM_drop_object_reply_handler(TCX_REF_PTR                   ref,
				       ARM_drop_object_reply_ptr data)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: Received an ARM_drop_object_reply message.\n");
#endif

  tcxFree("ARM_drop_object_reply", data);
}




/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         ARM_lift_object_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void ARM_lift_object_reply_handler(TCX_REF_PTR                   ref,
				       ARM_lift_object_reply_ptr data)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: Received an ARM_lift_object_reply message.\n");
#endif

  tcxFree("ARM_lift_object_reply", data);
}


#endif /* UNIBONN */




/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         CAMERA_image_reply_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void CAMERA_image_reply_handler(TCX_REF_PTR             ref,
				CAMERA_image_reply_ptr data)
{
  int i, j, index1, index2;
  ROBOT_SPECIFICATIONS_PTR robot_specifications = 
    allGlobal.robot_specifications;
  PROGRAM_STATE_PTR        program_state        = allGlobal.program_state;
  ROBOT_STATE_PTR          robot_state          = allGlobal.robot_state;
  ACTION_PTR               action               = allGlobal.action;
  SENSATION_PTR            sensation            = allGlobal.sensation;

#ifdef TCX_debug
  fprintf(stderr, "TCX: Received CAMERA_grey_image_reply message.\n");
#endif

/*   fprintf( stderr,"data->size %d\n", data->size ); */
/*   fprintf( stderr,"data->xsize %d\n", data->xsize ); */
/*   fprintf( stderr,"data->ysize %d\n" , data->ysize); */
/*   fprintf( stderr,"data->numGrabber %d\n",data->numGrabber ); */

  if (data->xsize != PSEUDO_PICTURE_SIZE_HORIZONTAL ||
      data->ysize != PSEUDO_PICTURE_SIZE_VERTICAL)
    fprintf(stderr, "ERROR: data format mismatch in grey_image (%d %d).\n",
	    data->xsize, data->ysize);
  
  else{
    for (i = 0; i < PSEUDO_PICTURE_SIZE_HORIZONTAL; i++)
      for (j = 0; j < PSEUDO_PICTURE_SIZE_VERTICAL; j++){
	index1 = (i+1) * PSEUDO_PICTURE_SIZE_VERTICAL - j - 1;
	index2 = j * PSEUDO_PICTURE_SIZE_HORIZONTAL + i;
	pseudo_picture[index1] = 
	  (((float) data->red[index2]) +
	   ((float) data->green[index2]) +
	   ((float) data->blue[index2])) / 256.0 / 3.0;
	if (pseudo_picture[index1] < 0.0)
	  pseudo_picture[index1] = 0.0;
	if (pseudo_picture[index1] > 1.0)
	  pseudo_picture[index1] = 1.0;
      }
    G_display_matrix(PSEUDO_PICTURE);

    sensation->last_camera_defined     = 1;
    sensation->last_camera_robot_x     = robot_state->x;
    sensation->last_camera_robot_y     = robot_state->y;
    sensation->last_camera_robot_orientation = robot_state->orientation;
  }

  G_display_switch(ACQUIRE_CAMERA_IMAGE_BUTTON, 0);


#ifdef UNIBONN
  if (program_state->camera_continuous_grabbing == -1){
    G_display_switch(RIGHT_CAMERA_BUTTON, 0);
    G_display_switch(LEFT_CAMERA_BUTTON, 0);
  }
  else if (program_state->camera_continuous_grabbing == 0){
    G_display_switch(RIGHT_CAMERA_BUTTON, 0);
    G_display_switch(LEFT_CAMERA_BUTTON, 2);
    tcx_camera_image(ALL, 0);
  }
  else{
    G_display_switch(RIGHT_CAMERA_BUTTON, 2);
    G_display_switch(LEFT_CAMERA_BUTTON, 0);
    tcx_camera_image(ALL, 2);
  }
#endif /* UNIBONN */


  tcxFree("CAMERA_image_reply", data);
}



void CAMERA_shmid_reply_handler( TCX_REF_PTR             ref,
				 CAMERA_shmid_reply_ptr shmid)
{
  fprintf(stderr, "Received a CAMERA_shmid_reply -> why???\n");
}

void CAMERA_load_reply_handler( TCX_REF_PTR           ref,
				CAMERA_load_reply_ptr data)
{
  fprintf(stderr, "Received a CAMERA_load_reply -> why???\n");
}



/************************************************************************
 *
 *
 *   NAME:         update_staistics
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void update_internal_statistics(int   n_new_grid_cells, 
				float robot_advancement,
				ALL_PARAMS)
{
  struct timeval this_time;
  float  time_difference;
  float  decay_factor;
  float  new_area;
  float  new_area_per_sec;
  char   button_text[128];
  float  advancement_per_sec;
  float  prev_value;

  /*
   * Compute time difference and decay factor
   */

  gettimeofday(&this_time, NULL);

  if (last_statistics_update.tv_sec == (long) 0 &&
      last_statistics_update.tv_usec == (long) 0){
    time_difference = 0.0;
    decay_factor = 0.0;
  }
  
  else{
    time_difference = 
      ((float) (this_time.tv_sec - last_statistics_update.tv_sec))
	+ (((float) (this_time.tv_usec 
		       - last_statistics_update.tv_usec))
	   /  1000000.0);
      
    decay_factor = pow(robot_specifications->statistics_time_decay_rate,
		       time_difference);
  }  

  last_statistics_update.tv_sec = this_time.tv_sec;
  last_statistics_update.tv_usec = this_time.tv_usec;


  /*
   * STATUS REPORT
   */


  program_state->total_distance_traveled += robot_advancement;


  if (time_difference > 0.0){
    advancement_per_sec = robot_advancement / time_difference;
    
    prev_value = program_state->advance_rate;
    program_state->advance_rate = 
      (program_state->advance_rate * decay_factor) + 
	(advancement_per_sec * (1.0 -  decay_factor));
    
    if (program_state->advance_rate > 300.0)
      program_state->advance_rate = 0.0; /* just to make sure BASE did not
					  * send us something terrible */
    
    
    if (program_state->advance_rate != prev_value){
      sprintf(button_text,  "advance (%5.2f)", program_state->advance_rate);
      G_set_new_text(STATUS_ADVANCEMENT_BUTTON, button_text, 0);
      G_display_value(STATUS_ADVANCEMENT_BUTTON,  
		      program_state->advance_rate);
    }
  }
  
  
  /*
   * MAP
   */

  new_area = ((float) n_new_grid_cells) 
    * robot_specifications->map_resolution 
      * robot_specifications->map_resolution;
  
  program_state->total_area_covered += new_area;
  
  if (time_difference > 0.0){
    new_area_per_sec = new_area / time_difference;
    
    prev_value = program_state->d_total_area_covered;
    program_state->d_total_area_covered = 
      (program_state->d_total_area_covered * decay_factor) + 
	(new_area_per_sec * (1.0 -  decay_factor));
    
    if (prev_value != program_state->d_total_area_covered){
      sprintf(button_text,  "new area (%5.2f)", 
	      program_state->d_total_area_covered);
      G_set_new_text(STATUS_NEW_AREA_BUTTON, button_text, 0);
      G_display_value(STATUS_NEW_AREA_BUTTON,  
		      program_state->d_total_area_covered);
    }
    
    if (n_new_grid_cells > 0){
      sprintf(button_text,  "total area (%5.2f)", 
	      program_state->total_area_covered);
      G_set_new_text(STATUS_TOTAL_AREA_BUTTON, button_text, 0);
      G_display_value(STATUS_TOTAL_AREA_BUTTON,  
		      program_state->total_area_covered);
    }
      
    /*printf("@1: %d %g %g %g %g %g %g\n", n_new_grid_cells,
    time_difference,
    new_area, new_area_per_sec, decay_factor,
    program_state->total_area_covered, 
    program_state->d_total_area_covered);
    */  
  }
}


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         MAP_partial_map_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void MAP_partial_map_reply_handler(TCX_REF_PTR                 ref,
				   MAP_partial_map_reply_ptr partial_map)
{
  ROBOT_SPECIFICATIONS_PTR robot_specifications = 
    allGlobal.robot_specifications;
  PROGRAM_STATE_PTR        program_state        = allGlobal.program_state;
  ROBOT_STATE_PTR          robot_state          = allGlobal.robot_state;
  ACTION_PTR               action               = allGlobal.action;
  SENSATION_PTR            sensation            = allGlobal.sensation;

  int x, y, global_x, global_y, partial_map_index, global_map_index;
  float *global_map;
  int   *global_map_active;
  unsigned char char_value;
  int   n_new_grid_cells;
  struct timeval this_time;
  float  time_difference;

#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a MAP_partial_map_reply message.\n");
#endif

  if (partial_map->resolution != robot_specifications->map_resolution){
    fprintf(stderr, "ERROR: Resolution mismatch. Message ignored.\n");
  }
  else{

    
    global_map         = robot_specifications->occupancy_values;
    global_map_active  = robot_specifications->map_active;
    n_new_grid_cells   = 0;
    
    if (partial_map->delete_previous_map)
      clear_maps(ALL);



    autoshift_display((((float) partial_map->first_x)
		       * robot_specifications->map_resolution)
		      + robot_specifications->max_sonar_range,
		      (((float) partial_map->first_y)
		       * robot_specifications->map_resolution)
		      + robot_specifications->max_sonar_range,
		      robot_specifications->safety_margin, 
		      robot_specifications->autoshift_distance, 
		      ALL);

    autoshift_display((((float) (partial_map->first_x + partial_map->size_x))
		       * robot_specifications->map_resolution)
		      - robot_specifications->max_sonar_range,
		      (((float) (partial_map->first_y + partial_map->size_y))
		       * robot_specifications->map_resolution)
		      - robot_specifications->max_sonar_range,
		      robot_specifications->safety_margin, 
		      robot_specifications->autoshift_distance, 
		      ALL);



    for (x = 0, global_x = x + partial_map->first_x 
	   + robot_specifications->autoshifted_int_x;
	 x < partial_map->size_x; x++, global_x++)
      for (y = 0, global_y = y + partial_map->first_y 
	     + robot_specifications->autoshifted_int_y;
	   y < partial_map->size_y; y++, global_y++){
	/* ----------- compute index ---------- */
	partial_map_index = x * partial_map->size_y + y;
	global_map_index  = global_x * 
	  robot_specifications->global_map_dim_y + global_y;
	/* ----------- update global map ---------- */
	if (global_x >= 0 &&
	    global_x < robot_specifications->global_map_dim_x &&
	    global_y >= 0 &&
	    global_y < robot_specifications->global_map_dim_y){

	  /* 
	 * old data format
	 * 
	 * global_map_active[global_map_index]
	 * = partial_map->active[partial_map_index];    
	 * if (global_map_active[global_map_index]){
	 * global_map[global_map_index] = 
	 * partial_map->values[partial_map_index];
	 */
	  char_value = partial_map->char_values[partial_map_index];
	  if (char_value == (unsigned char) 0){
	    if (global_map_active[global_map_index])
	      n_new_grid_cells--;
	    global_map_active[global_map_index] = 0;
	  }
	  else if (char_value == (unsigned char) 255){
	    if (!global_map_active[global_map_index])
	      n_new_grid_cells++;
	    global_map_active[global_map_index] = 1;
	    global_map[global_map_index] = 1.0;
	  }
	  else{
	    if (!global_map_active[global_map_index])
	      n_new_grid_cells++;
	    global_map_active[global_map_index] = 1;
	    global_map[global_map_index] = ((float) (char_value-1)) / 253.0;
	  }
	}
      }


	

    /*
     * update statistics
     */
    
    update_internal_statistics(n_new_grid_cells, 0.0, ALL);

	

    /*
     * map display update
     */


    gettimeofday(&this_time, NULL);
    time_difference = 
      ((float) (this_time.tv_sec - last_map_update.tv_sec))
      + (((float) (this_time.tv_usec 
		   - last_map_update.tv_usec))
	 /  1000000.0);
    if (program_state->map_regular_update_on &&
	time_difference > robot_specifications->map_update_interval){

      display_global_robot(ALL);
    
    }



    else if (partial_map->delete_previous_map){
      G_display_partial_matrix(OCCUPANCY_MAP, partial_map->first_x
			       + robot_specifications->autoshifted_int_x, 
			       partial_map->size_x, partial_map->first_y 
			       + robot_specifications->autoshifted_int_y, 
			       partial_map->size_y);
      G_display_markers(PATH);
      G_display_markers(TARGET_POINT_GLOBAL);
      G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y, 
		      robot_state->orientation, 0, dummy_sensors);
    }
  }
  tcxFree("MAP_partial_map_reply", partial_map);

}




/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         MAP_correction_parameters_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void MAP_correction_parameters_reply_handler(TCX_REF_PTR                 ref,
				     MAP_correction_parameters_reply_ptr corr)
{
  ROBOT_STATE_PTR          robot_state          = allGlobal.robot_state;


  float uncorr_robot_x, uncorr_robot_y, uncorr_robot_orientation;



#ifdef TCX_debug
  fprintf(stderr, 
	  "TCX: Received a MAP_correction_parameters_reply message.\n");
#endif


  compute_backward_correction(robot_state->x, robot_state->y, 
			      robot_state->orientation,
			      robot_state->correction_parameter_x,
			      robot_state->correction_parameter_y,
			      robot_state->correction_parameter_angle,
			      robot_state->correction_type,
			      &uncorr_robot_x, &uncorr_robot_y,  
			      &uncorr_robot_orientation);
  

  robot_state->correction_parameter_x     = corr->parameter_x;
  robot_state->correction_parameter_y     = corr->parameter_y;
  robot_state->correction_parameter_angle = corr->parameter_angle;
  robot_state->correction_type            = corr->type;

  compute_forward_correction(uncorr_robot_x, 
			     uncorr_robot_y, 
			     uncorr_robot_orientation,
			     robot_state->correction_parameter_x,
			     robot_state->correction_parameter_y,
			     robot_state->correction_parameter_angle,
			     robot_state->correction_type,
			     &(robot_state->x),
			     &(robot_state->y),
			     &(robot_state->orientation));


  tcxFree("MAP_correction_parameters_reply", corr);
}




/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         PLAN_action_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void PLAN_action_reply_handler(TCX_REF_PTR              ref,
			       PLAN_action_reply_ptr    plan)
{

#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a PLAN_action_reply.\n");
#endif

  tcxFree("PLAN_action_reply", plan);
}

#ifdef UNIBONN

/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         SUNVIS_object_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SUNVIS_object_reply_handler(TCX_REF_PTR                ref,
				 SUNVIS_object_reply_ptr    object_msg)
{
  ROBOT_SPECIFICATIONS_PTR robot_specifications = 
    allGlobal.robot_specifications;
  PROGRAM_STATE_PTR        program_state        = allGlobal.program_state;
  ROBOT_STATE_PTR          robot_state          = allGlobal.robot_state;
  ACTION_PTR               action               = allGlobal.action;
  SENSATION_PTR            sensation            = allGlobal.sensation;

  float uncorr_obj_x, uncorr_obj_y, *corr_obj_x, *corr_obj_y;
  float corr_robot_x, corr_robot_y, corr_robot_orientation;
  float corr_obj_orientation;
  int i;


#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a SUNVIS_object_reply message. %d objects.\n",
	  object_msg->num_objects);
  /*  fprintf(stderr, "\tnum_objects\t%d\n", object_msg->num_objects);
      for (i = 0; i < object_msg->num_objects; i++)
      fprintf(stderr, "\t\t%6.4f %6.4f %d\n",
      object_msg->obj_x[i], object_msg->obj_y[i],
      object_msg->type[i]);
      fprintf(stderr, "\n");
*/
#endif

  

  /*
   * correct alleged robot location
   */


  compute_forward_correction(object_msg->robot_x, 
			     object_msg->robot_y, 
			     object_msg->robot_orientation,
			     robot_state->correction_parameter_x,
			     robot_state->correction_parameter_y,
			     robot_state->correction_parameter_angle,
			     robot_state->correction_type,
			     &corr_robot_x, 
			     &corr_robot_y, 
			     &corr_robot_orientation);



  

  /*
   * if not, let's allocate memory
   */

  corr_obj_x = (float *) malloc(sizeof(float) * object_msg->num_objects);
  corr_obj_y = (float *) malloc(sizeof(float) * object_msg->num_objects);


  for (i = 0; i < object_msg->num_objects; i++){

    /*
     * Compute absolute cooredinates in the robot's coordinate system
     */
    
    uncorr_obj_x = object_msg->robot_x 
      + (object_msg->obj_x[i]
	 * sin(object_msg->robot_orientation * M_PI / 180.0))
	+ (object_msg->obj_y[i]
	   * cos(object_msg->robot_orientation * M_PI / 180.0));
    
    uncorr_obj_y = object_msg->robot_y 
      + (object_msg->obj_y[i]
	 * sin(object_msg->robot_orientation * M_PI / 180.0))
	- (object_msg->obj_x[i]
	   * cos(object_msg->robot_orientation * M_PI / 180.0));
    
    /*
     * Correct the coordinates
     */
    
    compute_forward_correction(uncorr_obj_x, 
			       uncorr_obj_y, 
			       0.0,
			       robot_state->correction_parameter_x,
			       robot_state->correction_parameter_y,
			       robot_state->correction_parameter_angle,
			       robot_state->correction_type,
			       &(corr_obj_x[i]),
			       &(corr_obj_y[i]),
			       &corr_obj_orientation);
    /*
     * print out the results
     */
    
    fprintf(stderr, "OBJ:\t%5.3f %5.3f type %d\n\t-> %5.3f %5.3f\n\t-> %5.3f %5.3f\n",
	    object_msg->obj_x[i], object_msg->obj_y[i],
	    object_msg->type[i],
	    uncorr_obj_x, uncorr_obj_y,
	    corr_obj_x[i], corr_obj_y[i]);
    
  }
  
  /*
   * And add the information into the database of objects
   */

  process_object_message(corr_robot_x, 
			 corr_robot_y, 
			 corr_robot_orientation,
			 object_msg->pan_angle,
			 object_msg->tilt_angle,
			 object_msg->num_objects,
			 corr_obj_x,
			 corr_obj_y,
			 object_msg->type,
			 0,
			 ALL);

  free(corr_obj_x);
  free(corr_obj_y);

  tcxFree("SUNVIS_object_reply", object_msg);
}



/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         SUNVIS_check_object_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SUNVIS_check_object_reply_handler(TCX_REF_PTR                   ref,
				       SUNVIS_check_object_reply_ptr data)
{
#ifdef TCX_debug
  fprintf(stderr, "TCX: Received an SUNVIS_check_object_reply message.\n");
#endif

  tcxFree("SUNVIS_check_object_reply", data);
}




/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         TRACKER-Handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void EZR_order_handler(TCX_REF_PTR ref,
		       EZR_order_type     *theOrder)
{
  fprintf(stderr, "TRACKER handler: EZR_order\n");

  tcxFree("EZR_order", theOrder);
}


void EZR_command_handler(TCX_REF_PTR ref,
			 EZR_command_type   *theCommand)
{
  fprintf(stderr, "TRACKER handler: EZR_command\n");

  tcxFree("EZR_command", theCommand);
}


void EZR_char_array_transmit_handler(TCX_REF_PTR ref,
				     EZR_char_array_transmit_type *theArray)
{
  fprintf(stderr, "TRACKER handler: EZR_char_array_transmit\n");

  tcxFree("EZR_char_array_transmit", theArray);
}


void EZR_int_array_transmit_handler(TCX_REF_PTR ref,
				    EZR_int_array_transmit_type  *theArray)
{
  fprintf(stderr, "TRACKER handler: EZR_int_array_transmit: %d\n",
	  theArray->DataType);

  tcxFree("EZR_int_array_transmit", theArray);
}


void EZR_float_array_transmit_handler(TCX_REF_PTR ref,
				      EZR_float_array_transmit_type *theArray)
{
  fprintf(stderr, "TRACKER handler: EZR_float_array_transmit\n");

  tcxFree("EZR_float_array_transmit", theArray);
}


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         FLOW_tracking_info_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void FLOW_tracking_info_reply_handler(TCX_REF_PTR     ref,
				       int            *info)
{
  ROBOT_SPECIFICATIONS_PTR robot_specifications = 
    allGlobal.robot_specifications;
  PROGRAM_STATE_PTR        program_state        = allGlobal.program_state;
  ROBOT_STATE_PTR          robot_state          = allGlobal.robot_state;
  ACTION_PTR               action               = allGlobal.action;
  SENSATION_PTR            sensation            = allGlobal.sensation;


#ifdef TCX_debug
  fprintf(stderr, "TCX: Received an FLOW_tracking_info_reply message: %d.\n",
	  *info);
#endif

  sensation->flow_result = *info;
  if (program_state->giving_tour)
    handle_flow_reply_in_tour(ALL);
}


#endif /* UNIBONN */

/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         BUTTONS_status_reply_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void BUTTONS_status_reply_handler(TCX_REF_PTR              ref,
				  BUTTONS_status_reply_ptr data)
{
  ROBOT_SPECIFICATIONS_PTR robot_specifications = 
    allGlobal.robot_specifications;
  PROGRAM_STATE_PTR        program_state        = allGlobal.program_state;
  ROBOT_STATE_PTR          robot_state          = allGlobal.robot_state;
  ACTION_PTR               action               = allGlobal.action;
  SENSATION_PTR            sensation            = allGlobal.sensation;



#ifdef TCX_debug
  fprintf(stderr, "TCX: Received BUTTONS_status_reply.\n");
#endif

  if (data != NULL){
    if (data->red_light_status == BUTTON_LIGHT_STATUS_ON ||
	data->red_light_status == BUTTON_LIGHT_STATUS_ON_TILL_PRESSED ||
	data->red_light_status == BUTTON_LIGHT_STATUS_TOGGLE_ON)
      robot_state->red_light_status = 1;
    else if (data->red_light_status == BUTTON_LIGHT_STATUS_FLASHING ||
	     data->red_light_status == 
	     BUTTON_LIGHT_STATUS_FLASHING_TILL_PRESSED)
      robot_state->red_light_status = 2;
    else
      robot_state->red_light_status = 0;
    
    
    if (data->red_button_pressed)
      sensation->red_button_status = 1;
    else
      sensation->red_button_status = 0;
    sensation->red_button_changed = data->red_button_changed;
    
    
    
    if (data->yellow_light_status == BUTTON_LIGHT_STATUS_ON ||
	data->yellow_light_status == BUTTON_LIGHT_STATUS_ON_TILL_PRESSED ||
	data->yellow_light_status == BUTTON_LIGHT_STATUS_TOGGLE_ON)
      robot_state->yellow_light_status = 1;
    else if (data->yellow_light_status == BUTTON_LIGHT_STATUS_FLASHING ||
	     data->yellow_light_status == 
	     BUTTON_LIGHT_STATUS_FLASHING_TILL_PRESSED)
      robot_state->yellow_light_status = 2;
    else
      robot_state->yellow_light_status = 0;
    
    
    if (data->yellow_button_pressed)
      sensation->yellow_button_status = 1;
    else
      sensation->yellow_button_status = 0;
    sensation->yellow_button_changed = data->yellow_button_changed;
    
    
    
    if (data->green_light_status == BUTTON_LIGHT_STATUS_ON ||
	data->green_light_status == BUTTON_LIGHT_STATUS_ON_TILL_PRESSED ||
	data->green_light_status == BUTTON_LIGHT_STATUS_TOGGLE_ON)
      robot_state->green_light_status = 1;
    else if (data->green_light_status == BUTTON_LIGHT_STATUS_FLASHING ||
	     data->green_light_status == 
	     BUTTON_LIGHT_STATUS_FLASHING_TILL_PRESSED)
      robot_state->green_light_status = 2;
    else
      robot_state->green_light_status = 0;
    
    
    if (data->green_button_pressed)
      sensation->green_button_status = 1;
    else
      sensation->green_button_status = 0;
    sensation->green_button_changed = data->green_button_changed;
    
    
    
    if (data->blue_light_status == BUTTON_LIGHT_STATUS_ON ||
	data->blue_light_status == BUTTON_LIGHT_STATUS_ON_TILL_PRESSED ||
	data->blue_light_status == BUTTON_LIGHT_STATUS_TOGGLE_ON)
      robot_state->blue_light_status = 1;
    else if (data->blue_light_status == BUTTON_LIGHT_STATUS_FLASHING ||
	     data->blue_light_status == 
	     BUTTON_LIGHT_STATUS_FLASHING_TILL_PRESSED)
      robot_state->blue_light_status = 2;
    else
      robot_state->blue_light_status = 0;
    
    
    if (data->blue_button_pressed)
      sensation->blue_button_status = 1;
    else
      sensation->blue_button_status = 0;
    sensation->blue_button_changed = data->blue_button_changed;
  }  

  /***********************************************************************/

  G_display_switch(RED_LIGHT_BUTTON, robot_state->red_light_status);
  G_display_switch(RED_PUSHED_BUTTON, sensation->red_button_status);
  G_display_switch(YELLOW_LIGHT_BUTTON, robot_state->yellow_light_status);
  G_display_switch(YELLOW_PUSHED_BUTTON, sensation->yellow_button_status);
  G_display_switch(GREEN_LIGHT_BUTTON, robot_state->green_light_status);
  G_display_switch(GREEN_PUSHED_BUTTON, sensation->green_button_status);
  G_display_switch(BLUE_LIGHT_BUTTON, robot_state->blue_light_status);
  G_display_switch(BLUE_PUSHED_BUTTON, sensation->blue_button_status);


#ifdef TOURGUIDE_VERSION
  if (program_state->learning_tour || program_state->giving_tour)
    handle_button_in_tour(ALL); 
#endif /* TOURGUIDE_VERSION */

  if (data != NULL)
    tcxFree("BUTTONS_status_reply", data);

}



void SIMULATOR_message_to_base_handler(TCX_REF_PTR   ref,
				       char **message)
{
}

void SIMULATOR_message_to_baseServer_handler(TCX_REF_PTR   ref,
					     char **message)
{
}



void SIMULATOR_message_to_sonar_handler(TCX_REF_PTR   ref,
					char **message)
{
}

void SIMULATOR_message_to_laser_handler(TCX_REF_PTR   ref,
					SIMULATOR_message_to_laser_ptr	data)
{
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/************************************************************************
 *
 *   NAME:         initiate_action()
 *                 
 *   FUNCTION:     if an action is specified, this routine initiates it
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 1, if some action was taken, 0 otherwise
 *                 
 ************************************************************************/


int initiate_action(ALL_PARAMS)
{
  int i;
  
  if (action->type == 1){	/* stop command. Do it, regardless of status */

#ifdef TCX_debug
    printf("### Initiate action: Stop!\n");
#endif
    tcx_base_stop_robot(ALL);
    tcx_base_set_velocity(ALL, 0.0, 0.0);
    action->type   = 0;
    action->status = 0;
    robot_state->in_motion = 0;
    G_display_switch(IN_MOTION_BUTTON, robot_state->in_motion);
    return 1;
  }

  


  if (action->status == 1){	/* just a new action arrived! */
    
    /*! took this out
      if (robot_state->trans_set_acceleration != action->trans_acceleration ||
      robot_state->rot_set_acceleration != action->rot_acceleration)
      tcx_base_set_acceleration(ALL, action->trans_acceleration, 
      action->rot_acceleration);
      */
    
    /*====
     *==== 2: velocity action =========*
     *====*/
    
    if (action->type == 2){	/* ACTION: velocities */

#ifdef TCX_debug
      printf("### Initiate action: VELOCITY %g %g\n", 
	     action->trans_velocity, action->rot_velocity);
#endif
      if (robot_state->trans_set_velocity != action->trans_velocity ||
	  robot_state->rot_set_velocity != action->rot_velocity)
	tcx_base_set_velocity(ALL, action->trans_velocity,
			      action->rot_velocity);

      if (action->trans_direction)
	tcx_base_translate_backward(ALL);
      else
	tcx_base_translate_forward(ALL);

      if (action->rot_direction)
	tcx_base_rotate_anticlockwise(ALL);
      else
	tcx_base_rotate_clockwise(ALL);

    }

   
    /*====
     *==== 3: absolutely addressed action =========*
     *====*/
 
    else if (action->type == 3){


#ifdef TCX_debug
      printf("### Initiate action: ABSOLUTE %g %g\n", 
	     action->target_pos_x, action->target_pos_y);
#endif

      tcx_base_goto_relative(ALL);
      /* tcx_base_approach_two_points_absolute(1, ALL);*/
      action->status = 2;		/* means action is in progress */
      
      for (i = 0; i < G_return_num_markers(TARGET_POINT_GLOBAL, 1); i++)
	G_undisplay_markers(TARGET_POINT_GLOBAL, i, C_GREY90);
      G_clear_markers(TARGET_POINT_GLOBAL);
      G_add_marker(TARGET_POINT_GLOBAL, 
		   robot_state->x 
		   + (sin(robot_state->orientation * M_PI / 180.0)
		      * action->target_pos_x)
		   + (cos(robot_state->orientation * M_PI / 180.0)
		      * action->target_pos_y),
		   robot_state->y 
		   - (cos(robot_state->orientation * M_PI / 180.0)
		      * action->target_pos_x)
		   + (sin(robot_state->orientation * M_PI / 180.0)
		      * action->target_pos_y), 0);
      if (i > 0)
	display_global_robot(ALL);
      else
	G_display_markers(TARGET_POINT_GLOBAL);
    }

   
    /*====
     *==== 4: absolutely addressed action =========*
     *====*/
 
    else if (action->type == 4){


#ifdef TCX_debug
      printf("### Initiate action: ABSOLUTE %g %g\n", 
	     action->target_pos_x, action->target_pos_y);
#endif

      tcx_base_approach_two_points_absolute(1, ALL);

      for (i = 0; i < G_return_num_markers(TARGET_POINT_GLOBAL, 1); i++)
	G_undisplay_markers(TARGET_POINT_GLOBAL, i, C_GREY90);
      G_clear_markers(TARGET_POINT_GLOBAL);
      G_add_marker(TARGET_POINT_GLOBAL, action->target_pos_x,
		   action->target_pos_y, 0);
      if (i > 0)
	display_global_robot(ALL);
      else
	G_display_markers(TARGET_POINT_GLOBAL);
    }
    

    else
      fprintf(stderr, "ERROR: Unknown action type: %d\n", action->type);
    
    
    action->status = 2;		/* means action is in progress */
    
    robot_state->in_motion = 1;
    G_display_switch(IN_MOTION_BUTTON, robot_state->in_motion);
    
    
    gettimeofday(&(action->last_ping_time), NULL);
    return 1;

  }
  return 0;
}




/************************************************************************
 *
 *   NAME:         reset_robot()
 *                 
 *   FUNCTION:     resets robot position, PATH and logs a "break"
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void reset_robot(ALL_PARAMS)
{
  /* if (robot_state->in_motion) */
  tcx_reset_joystick(ALL); 
  
  tcx_base_stop_robot(ALL);

}






/************************************************************************
 *
 *   NAME:         disconnect_BASE
 *                 
 *   FUNCTION:     disconnects existing TCX connection to BASE
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void disconnect_BASE(ALL_PARAMS)
{
  if (!program_state->tcx_connected_to_BASE)
    fprintf(stderr,
	    "WARNING: not connected to BASE, disconnect ignored.\n");

  else{

#ifdef UNIBONN
    stop_hunting(ALL);
#endif /* UNIBONN */

    tcx_base_subscribe(ALL, 0, 0, 0, 0, 0);

    tcx_base_disconnect(ALL); /* there is no explicit
			       * disconnect message in TCX... */
   

    program_state->tcx_connected_to_BASE = 0;
    program_state->do_connect_to_BASE = 0;
    G_display_switch(CONNECT_BASE_BUTTON, 
		     program_state->tcx_connected_to_BASE);
    robot_state->sonar_on = 0;
#ifdef UNIBONN
    G_display_switch(SONAR_BUTTON, robot_state->sonar_on);
#endif /* UNIBONN */
  }
}
  


/************************************************************************
 *
 *   NAME:         disconnect_PANTILT
 *                 
 *   FUNCTION:     disconnects existing TCX connection to PANTILT
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void disconnect_PANTILT(ALL_PARAMS)
{
  if (!program_state->tcx_connected_to_PANTILT)
    fprintf(stderr,
	    "WARNING: not connected to PANTILT, disconnect ignored.\n");

  else{
    tcx_pantilt_disconnect(ALL); /* there is no explicit
				  * disconnect message in TCX... */
    
    program_state->tcx_connected_to_PANTILT = 0;
    program_state->do_connect_to_PANTILT = 0;
    G_display_switch(CONNECT_PANTILT_BUTTON, 
		     program_state->tcx_connected_to_PANTILT);
     
  }
}
 


/************************************************************************
 *
 *   NAME:         disconnect_CAMERA
 *                 
 *   FUNCTION:     disconnects existing TCX connection to CAMERA
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void disconnect_CAMERA(ALL_PARAMS)
{
  if (!program_state->tcx_connected_to_CAMERA)
    fprintf(stderr,
	    "WARNING: not connected to CAMERA, disconnect ignored.\n");

  else{
    tcx_camera_x_display_off(ALL);
    tcx_camera_disconnect(ALL); /* there is no explicit
				  * disconnect message in TCX... */
    
    program_state->tcx_connected_to_CAMERA = 0;
    program_state->do_connect_to_CAMERA = 0;
    G_display_switch(ACQUIRE_CAMERA_IMAGE_BUTTON, 0);
    G_display_switch(CONTINUOUS_CAMERA_IMAGE_BUTTON, 0);
#ifdef UNIBONN
    G_display_switch(CAMERA_X_DISPLAY_BUTTON, 0);
    G_display_switch(LEFT_CAMERA_BUTTON, 0);
    G_display_switch(RIGHT_CAMERA_BUTTON, 0);
#endif /* UNIBONN */
    G_display_switch(CONNECT_CAMERA_BUTTON, 
		     program_state->tcx_connected_to_CAMERA);
     
  }
}

#ifdef UNIBONN

/************************************************************************
 *
 *   NAME:         disconnect_SUNVIS
 *                 
 *   FUNCTION:     disconnects existing TCX connection to SUNVIS
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void disconnect_SUNVIS(ALL_PARAMS)
{
  if (!program_state->tcx_connected_to_SUNVIS)
    fprintf(stderr,
	    "WARNING: not connected to SUNVIS, disconnect ignored.\n");

  else{

    stop_hunting(ALL);
    program_state->tcx_connected_to_SUNVIS = 0;
    program_state->do_connect_to_SUNVIS = 0;
    G_display_switch(CONNECT_SUNVIS_BUTTON, 
		     program_state->tcx_connected_to_SUNVIS);
    G_display_switch(CAMERA_OBJECTS_BUTTON, 0);
    G_display_switch(CAMERA_COLLI_LINES_BUTTON, 2);
    G_display_switch(CAMERA_MAPS_BUTTON, 2);

     
  }
}

/************************************************************************
 *
 *   NAME:         disconnect_TRACKER
 *                 
 *   FUNCTION:     disconnects existing TCX connection to TRACKER
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void disconnect_TRACKER(ALL_PARAMS)
{
  if (!program_state->tcx_connected_to_TRACKER)
    fprintf(stderr,
	    "WARNING: not connected to TRACKER, disconnect ignored.\n");

  else{
    program_state->tcx_connected_to_TRACKER = 0;
    program_state->do_connect_to_TRACKER = 0;
    G_display_switch(CONNECT_TRACKER_BUTTON, 
		     program_state->tcx_connected_to_TRACKER);
    G_display_switch(TRACKER_BUTTON, 0);
     
  }
}
#endif /* UNIBONN */
#ifdef CD_VERSION

/************************************************************************
 *
 *   NAME:         disconnect_CD
 *                 
 *   FUNCTION:     disconnects existing TCX connection to CD
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void disconnect_CD(ALL_PARAMS)
{
  if (!program_state->tcx_connected_to_CD)
    fprintf(stderr,
	    "WARNING: not connected to CD, disconnect ignored.\n");

  else{
    if (program_state->last_cd_phrase != -1)
      G_display_switch(CD_BUTTONS[program_state->last_cd_phrase], 0);
    program_state->last_cd_phrase = -1;
    program_state->tcx_connected_to_CD = 0;
    program_state->do_connect_to_CD = 0;
    G_display_switch(CONNECT_CD_BUTTON, program_state->tcx_connected_to_CD);
    /* G_display_switch(CD_BUTTON, 0); */
  }
}
#endif /* CD_VERSION */




/************************************************************************
 *
 *   NAME:         disconnect_BUTTONS
 *                 
 *   FUNCTION:     disconnects existing TCX connection to BUTTONS
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void disconnect_BUTTONS(ALL_PARAMS)
{
  if (!program_state->tcx_connected_to_BUTTONS)
    fprintf(stderr,
	    "WARNING: not connected to BUTTONS, disconnect ignored.\n");

  else{
    program_state->tcx_connected_to_BUTTONS = 0;
    program_state->do_connect_to_BUTTONS = 0;
    G_display_switch(CONNECT_BUTTONS_BUTTON,
		     program_state->tcx_connected_to_BUTTONS);
  }
}

#ifdef UNIBONN

/************************************************************************
 *
 *   NAME:         disconnect_FLOW
 *                 
 *   FUNCTION:     disconnects existing TCX connection to FLOW
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void disconnect_FLOW(ALL_PARAMS)
{
  if (!program_state->tcx_connected_to_FLOW)
    fprintf(stderr,
	    "WARNING: not connected to FLOW, disconnect ignored.\n");

  else{
    program_state->tcx_connected_to_FLOW = 0;
    program_state->do_connect_to_FLOW = 0;
    G_display_switch(CONNECT_FLOW_BUTTON,
		     program_state->tcx_connected_to_FLOW);
  }
}
  

/************************************************************************
 *
 *   NAME:         disconnect_ARM
 *                 
 *   FUNCTION:     disconnects existing TCX connection to ARM
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void disconnect_ARM(ALL_PARAMS)
{
  if (!program_state->tcx_connected_to_ARM)
    fprintf(stderr,
	    "WARNING: not connected to ARM, disconnect ignored.\n");

  else{
    program_state->tcx_connected_to_ARM = 0;
    program_state->do_connect_to_ARM = 0;
    robot_state->arm_moved_out = 0;
    G_display_switch(ARM_MOVE_OUT_IN_BUTTON, 
		     robot_state->arm_moved_out);
    robot_state->arm_gripper_closed        = 0;
    G_display_switch(ARM_CLOSE_OPEN_GRIPPER_BUTTON, 
		     robot_state->arm_gripper_closed);
    G_display_switch(CONNECT_ARM_BUTTON, 
		     program_state->tcx_connected_to_ARM);
  }
}


/************************************************************************
 *
 *   NAME:         disconnect_SPEECH
 *                 
 *   FUNCTION:     disconnects existing TCX connection to SPEECH
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void disconnect_SPEECH(ALL_PARAMS)
{

  if (!program_state->tcx_connected_to_SPEECH)
    fprintf(stderr,
	    "WARNING: not connected to SPEECH, disconnect ignored.\n");
  
  else{
    tcx_speech_disconnect(ALL); /* there is no explicit
				 * disconnect message in TCX... */
    
    program_state->tcx_connected_to_SPEECH = 0;
    program_state->do_connect_to_SPEECH = 0;
    G_display_switch(CONNECT_SPEECH_BUTTON, 
		     program_state->tcx_connected_to_SPEECH);
    
  }

}
#endif


/************************************************************************
 *
 *   NAME:         disconnect_MAP
 *                 
 *   FUNCTION:     disconnects existing TCX connection to MAP
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void disconnect_MAP(ALL_PARAMS)
{
  if (!program_state->tcx_connected_to_MAP)
    fprintf(stderr,
	    "WARNING: not connected to MAP, disconnect ignored.\n");

  else{
    tcx_map_register_auto_update(ALL, 0, 0);/* there is no explicit
					     * disconnect message in TCX... */
    program_state->tcx_connected_to_MAP = 0;
    program_state->do_connect_to_MAP = 0;
    clear_maps(ALL);
    G_display_switch(CONNECT_MAP_BUTTON, 
		     program_state->tcx_connected_to_MAP);
     
  }
}
  

/************************************************************************
 *
 *   NAME:         disconnect_PLAN
 *                 
 *   FUNCTION:     disconnects existing TCX connection to PLAN
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void disconnect_PLAN(ALL_PARAMS)
{
  if (!program_state->tcx_connected_to_PLAN)
    fprintf(stderr,
	    "WARNING: not connected to PLAN, disconnect ignored.\n");

  else{

    /*
       if (program_state->exploration_mode){
       tcx_base_stop_robot(ALL);
       usleep(100000);
       tcx_base_stop_robot(ALL);
       }
       */
#ifdef UNIBONN
    stop_hunting(ALL);
#endif /* UNIBONN */
    program_state->tcx_connected_to_PLAN = 0;
    program_state->do_connect_to_PLAN = 0;
    G_display_switch(CONNECT_PLAN_BUTTON, 
		     program_state->tcx_connected_to_PLAN);
  }
}






/************************************************************************
 *
 *   NAME:         disconnect_BASESERVER
 *                 
 *   FUNCTION:     disconnects existing TCX connection to BASESERVER
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void disconnect_BASESERVER(ALL_PARAMS)
{
  if (!program_state->tcx_connected_to_BASESERVER)
    fprintf(stderr,
	    "WARNING: not connected to BASESERVER, disconnect ignored.\n");

  else{
    program_state->tcx_connected_to_BASESERVER = 0;
    program_state->do_connect_to_BASESERVER = 0;
    G_display_switch(CONNECT_BASESERVER_BUTTON, 
		     program_state->tcx_connected_to_BASESERVER);
     
  }
}
  


/************************************************************************
 *
 *   NAME:         disconnect_SIMULATOR
 *                 
 *   FUNCTION:     disconnects existing TCX connection to SIMULATOR
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void disconnect_SIMULATOR(ALL_PARAMS)
{
  if (!program_state->tcx_connected_to_SIMULATOR)
    fprintf(stderr,
	    "WARNING: not connected to SIMULATOR, disconnect ignored.\n");

  else{
    program_state->tcx_connected_to_SIMULATOR = 0;
    program_state->do_connect_to_SIMULATOR = 0;
    G_display_switch(CONNECT_SIMULATOR_BUTTON, 
		     program_state->tcx_connected_to_SIMULATOR);
     
  }
}
  


/************************************************************************
 *
 *   NAME:         disconnect_SONARINT
 *                 
 *   FUNCTION:     disconnects existing TCX connection to SONARINT
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void disconnect_SONARINT(ALL_PARAMS)
{
  if (!program_state->tcx_connected_to_SONARINT)
    fprintf(stderr,
	    "WARNING: not connected to SONARINT, disconnect ignored.\n");

  else{
    program_state->tcx_connected_to_SONARINT = 0;
    program_state->do_connect_to_SONARINT = 0;
    G_display_switch(CONNECT_SONARINT_BUTTON, 
		     program_state->tcx_connected_to_SONARINT);
     
  }
}
  


/************************************************************************
 *
 *   NAME:         disconnect_LASERINT
 *                 
 *   FUNCTION:     disconnects existing TCX connection to LASERINT
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void disconnect_LASERINT(ALL_PARAMS)
{
  if (!program_state->tcx_connected_to_LASERINT)
    fprintf(stderr,
	    "WARNING: not connected to LASERINT, disconnect ignored.\n");

  else{
    program_state->tcx_connected_to_LASERINT = 0;
    program_state->do_connect_to_LASERINT = 0;
    G_display_switch(CONNECT_LASERINT_BUTTON, 
		     program_state->tcx_connected_to_LASERINT);
     
  }
}
  


/************************************************************************
 *
 *   NAME:         refresh_action()
 *                 
 *   FUNCTION:     will be called regularily
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


int refresh_action(ALL_PARAMS)
{
  struct timeval tp1;

  
  /*======================================================*\
   ******* CHECK, IF WE NEED TO SEND ACTION REFRESH *******|
   *======================================================*/
  
  if (action->status == 2 &&	/* action in progress */
      action->continuous_ping){	/* action requires regular refresh */

    gettimeofday(&tp1, NULL);	/* check if time over */
    
    if (tp1.tv_sec > action->last_ping_time.tv_sec + 
	robot_specifications->refresh_interval  ||
	(tp1.tv_sec == action->last_ping_time.tv_sec + 
	 robot_specifications->refresh_interval &&
	 tp1.tv_usec > action->last_ping_time.tv_usec)){ /* time over? */
      
      if (action->type == 4){	/* this is the propper treatment
				 * of a refresh */
	tcx_base_approach_two_points_absolute(0, ALL);
	action->last_ping_time.tv_sec  = tp1.tv_sec;
	action->last_ping_time.tv_usec = tp1.tv_usec;
      }
      
      
      else
	action->status = 1;	/* this is a hack: begin action
				 * from scratch. */
#if 0      
      initiate_action(ALL);	/* and go for it */
#endif      
      return 1;
    }
  }
  return 0;
}




/************************************************************************
 *
 *   NAME:         display_global_robot()
 *                 
 *   FUNCTION:     updates right window
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
display_global_robot(ALL_PARAMS)
{
  int i;

  G_display_matrix(OCCUPANCY_MAP);
  G_display_markers(PATH);
  G_display_markers(TARGET_POINT_GLOBAL);
  G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y, 
		  robot_state->orientation, 0, dummy_sensors);
  gettimeofday(&last_map_update, NULL);

#ifdef TOURGUIDE_VERSION
  for (i = 0; i < MAX_NUM_TOURS; i++){
    G_display_markers(TOUR_GOALS[i]);
    G_display_markers(TOUR_ACTIVE_GOALS[i]);
    G_display_markers(TOUR_OBJECTS[i]);
  }
#endif /* TOURGUIDE_VERSION */
  
}
