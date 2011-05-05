
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
#include "all.h" 



/************************************************************************
 *
 *   NAME:         test_mouse
 *                 
 *   FUNCTION:     tests mouse and executes whatever we'd like
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 1, if meaningful mouse event found, 0 otherwise
 *                 
 ************************************************************************/


int test_mouse(ALL_PARAMS)
{
  G_mouse_ptr mouse_events;
  int num_mouse_events, button, goal_number, n, k;
  float mouse_x, mouse_y, goal_x, goal_y, goal_orientation;
  int i, number, file_is_open, backup;
  int return_value = 0;
  int action_nr, is_random;
  int mouse_event_detected;
  int cd_event;

  if (program_state->do_Xdisplay){
    
    /****************** CHECK FOR MOUSE EVENT *******************/

    mouse_event_detected = G_test_mouse(0);
    if (mouse_event_detected){


      mouse_events = G_evaluate_mouse(&mouse_x, &mouse_y, 
				      &button, &num_mouse_events);
      
      if (mouse_event_detected == 1){ /* reglar mouse press */
	G_display_switch(BUSY_BUTTON, 1);
	
	
	/****************** EVALUATE MOUSE EVENT *******************/

	cd_event = 0;
#ifdef CD_VERSION
	for (k = 0; k < NUM_CD_TEXTS; k++)
	  if (G_mouse_event_at(CD_BUTTONS[k], mouse_events, &number))
	    cd_event = 1;
#endif /* CD_VERSION */
	


	/*====
	 *==== RIGHT: stop =========*
	 *====*/
	
	if (button == RIGHT_BUTTON ||
	    G_mouse_event_at(STOP_ROBOT, mouse_events, &number)){ 
	  action->status          = 1; /* 1=new command issued */
	  action->type            = 1; /* 1=stop */
	  action->continuous_ping = 0; /* 1=ping not necessary */
	  action->target_pos_x    = 0.0;
	  action->target_pos_y    = 0.0;
	  action->trans_velocity  = robot_state->trans_set_velocity;
	  action->rot_velocity    = robot_state->rot_set_velocity;
	  action->trans_acceleration = robot_state->trans_set_acceleration;
	  action->rot_acceleration = robot_state->rot_set_acceleration;
	}	  
	

	/********* BUTTONS ON THE ROBOT / LIGHTS *********/
	
	
	else if (G_mouse_event_at(RED_LIGHT_BUTTON, mouse_events, &number)){
	  if (button == MIDDLE_BUTTON)
	    robot_state->red_light_status = 2;
	  else if (!robot_state->red_light_status)
	    robot_state->red_light_status = 1;
	  else
	    robot_state->red_light_status = 0;
	  set_buttons(ALL, 0);
	}

	else if (G_mouse_event_at(YELLOW_LIGHT_BUTTON, mouse_events, &number)){
	  if (button == MIDDLE_BUTTON)
	    robot_state->yellow_light_status = 2;
	  else if (!robot_state->yellow_light_status)
	    robot_state->yellow_light_status = 1;
	  else
	    robot_state->yellow_light_status = 0;
	  set_buttons(ALL, 1);
	}

	else if (G_mouse_event_at(GREEN_LIGHT_BUTTON, mouse_events, &number)){
	  if (button == MIDDLE_BUTTON)
	    robot_state->green_light_status = 2;
	  else if (!robot_state->green_light_status)
	    robot_state->green_light_status = 1;
	  else
	    robot_state->green_light_status = 0;
	  set_buttons(ALL, 2);
	}

	else if (G_mouse_event_at(BLUE_LIGHT_BUTTON, mouse_events, &number)){
	  if (button == MIDDLE_BUTTON)
	    robot_state->blue_light_status = 2;
	  else if (!robot_state->blue_light_status)
	    robot_state->blue_light_status = 1;
	  else
	    robot_state->blue_light_status = 0;
	  set_buttons(ALL, 3);
	}

	else if (G_mouse_event_at(RED_PUSHED_BUTTON, mouse_events, &number)){
	  if (program_state->tcx_connected_to_BUTTONS)
	    buttons_simulate_button_press(ALL, BUTTON_RED);
	  else{
	    sensation->yellow_button_changed   = 0;
	    sensation->green_button_changed    = 0;
	    sensation->blue_button_changed     = 0;
	    backup                             = sensation->red_button_status;
	    sensation->red_button_status       = 1;
	    sensation->red_button_changed      = 1;
	    BUTTONS_status_reply_handler(NULL, NULL);
	    usleep(500000);
	    sensation->red_button_changed      = 0;
	    sensation->red_button_status       = backup;
	    BUTTONS_status_reply_handler(NULL, NULL);
	  }
	}
	else if (G_mouse_event_at(YELLOW_PUSHED_BUTTON, mouse_events,&number)){
	  if (program_state->tcx_connected_to_BUTTONS)
	    buttons_simulate_button_press(ALL, BUTTON_YELLOW);
	  else{
	    sensation->red_button_changed      = 0;
	    sensation->green_button_changed    = 0;
	    sensation->blue_button_changed     = 0;
	    backup                             = 
	      sensation->yellow_button_status;
	    sensation->yellow_button_status       = 1;
	    sensation->yellow_button_changed      = 1;
	    BUTTONS_status_reply_handler(NULL, NULL);
	    usleep(500000);
	    sensation->yellow_button_changed      = 0;
	    sensation->yellow_button_status       = backup;
	    BUTTONS_status_reply_handler(NULL, NULL);
	  }
	}
	else if (G_mouse_event_at(GREEN_PUSHED_BUTTON, mouse_events, &number)){
	  if (program_state->tcx_connected_to_BUTTONS)
	    buttons_simulate_button_press(ALL, BUTTON_GREEN);
	  else{
	    sensation->red_button_changed      = 0;
	    sensation->yellow_button_changed   = 0;
	    sensation->blue_button_changed     = 0;
	    backup                             = 
	      sensation->green_button_status;
	    sensation->green_button_status       = 1;
	    sensation->green_button_changed      = 1;
	    BUTTONS_status_reply_handler(NULL, NULL);
	    usleep(500000);
	    sensation->green_button_changed      = 0;
	    sensation->green_button_status       = backup;
	    BUTTONS_status_reply_handler(NULL, NULL);
	  }
	}
	else if (G_mouse_event_at(BLUE_PUSHED_BUTTON, mouse_events, &number)){
	  if (program_state->tcx_connected_to_BUTTONS)
	    buttons_simulate_button_press(ALL, BUTTON_BLUE);
	  else{
	    sensation->red_button_changed      = 0;
	    sensation->yellow_button_changed   = 0;
	    sensation->green_button_changed    = 0;
	    backup                             = sensation->blue_button_status;
	    sensation->blue_button_status       = 1;
	    sensation->blue_button_changed      = 1;
	    BUTTONS_status_reply_handler(NULL, NULL);
	    usleep(500000);
	    sensation->blue_button_changed      = 0;
	    sensation->blue_button_status       = backup;
	    BUTTONS_status_reply_handler(NULL, NULL);
	  }
	}

	/* ******* ACTIONS: MOUSE EVENT IN LEFT CONTROL WINDOW ******** */

	else if (G_mouse_event_at(SONAR_ROBOT, mouse_events, &number)){
	  /*====
	   *==== goto relative coordinates =========*
	   *====*/
	  if (!program_state->control_window_mode){ /* relative coordinates */
	    
	    action->status          = 1; /* 1=new command issued */
	    action->type            = 4; /* 4=absolute target coordinates */
	    action->continuous_ping = 1; /* 1= regular refresh necessary */
	    action->target_pos_x    = robot_state->x 
	      + (sin(robot_state->orientation * M_PI / 180.0)
		 * mouse_events[number].value_x)
		+ (cos(robot_state->orientation * M_PI / 180.0)
		   * mouse_events[number].value_y);
	    action->target_pos_y    = robot_state->y 
		   - (cos(robot_state->orientation * M_PI / 180.0)
		      * mouse_events[number].value_x)
		   + (sin(robot_state->orientation * M_PI / 180.0)
		      * mouse_events[number].value_y);
	    action->trans_velocity  = robot_state->trans_set_velocity;
	    action->rot_velocity    = robot_state->rot_set_velocity;
	    action->trans_acceleration = robot_state->trans_set_acceleration;
	    action->rot_acceleration = robot_state->rot_set_acceleration;
	    
#ifdef JUNK
	    tcx_pantilt_set_velocity(ALL, PAN_HIGH_SPEED, TILT_HIGH_SPEED);
	    tcx_pantilt_track_point(ALL, 
				    robot_state->x 
				    + (cos((robot_state->orientation - 90.0)
					   * M_PI / 180.0)
				       * action->target_pos_x) 
				    - (sin((robot_state->orientation - 90.0)
					   * M_PI / 180.0)
				       * action->target_pos_y),
				    robot_state->y
				    + (sin((robot_state->orientation - 90.0)
					   * M_PI / 180.0)
				       * action->target_pos_x) 
				    + (cos((robot_state->orientation - 90.0)
					   * M_PI / 180.0)
				       * action->target_pos_y));
#endif
	  }
	  
	  else{
	    /*====
	     *==== pick target velocity =========*
	     *====*/
	    
	    action->status          = 1; /* 1=new command issued */
	    action->type            = 2; /* 2=target velocities */
	    action->continuous_ping = 0; /* 0=ping not necessary */
	    action->trans_velocity  = mouse_events[number].value_y 
	      / (robot_specifications->max_sonar_range*1.05)
		* MAX_TRANS_VELOCITY;
	    action->rot_velocity    = mouse_events[number].value_x
	      / (robot_specifications->max_sonar_range*1.05)
		* MAX_ROT_VELOCITY * 0.5;
	    action->rot_direction   = action->rot_velocity < 0.0;
	    action->trans_direction = action->trans_velocity < 0.0;
	    action->rot_velocity    = fabs(action->rot_velocity)
	      - (robot_specifications->robot_size 
		 / (robot_specifications->max_sonar_range*1.05)
		 * MAX_ROT_VELOCITY * 0.5);
	    if (action->rot_velocity < 0.0)
	      action->rot_velocity = 0.0;
	    action->trans_velocity  = fabs(action->trans_velocity)
	      - (robot_specifications->robot_size 
		 / (robot_specifications->max_sonar_range*1.05)
		 * MAX_TRANS_VELOCITY);
	    if (action->trans_velocity < 0.0)
	      action->trans_velocity = 0.0;
	    action->trans_acceleration = robot_state->trans_set_acceleration;
	    action->rot_acceleration = robot_state->rot_set_acceleration;
	    action->target_pos_x    = 0.0;
	    action->target_pos_y    = 0.0;
	  }
	}
	


	/* ******* ACTIONS: MOUSE EVENT IN RIGHT CONTROL WINDOW ******** */
		
#ifdef UNIBONN
	/*====
	 *==== MIDDLE: fixate pantilt
	 *====*/
	
	
	else if (button == MIDDLE_BUTTON && /* emergency stop */
	    G_mouse_event_at(TARGET_OBJECTS, mouse_events, &number)){ 


	  tcx_pantilt_track_point(ALL, mouse_events[number].value_x, 
				  mouse_events[number].value_y);

	  /*
	  check_if_point_visible(robot_state->x, 
				     robot_state->y,
				     robot_state->orientation,
				     mouse_events[number].value_x,
				     mouse_events[number].value_y,
				     0, ALL);

				     */
	  ;
	  /*
	   check_and_add_object(mouse_events[number].value_x,
				mouse_events[number].value_y,
				0, ALL);*/
	}



#endif /* UNIBONN */
#ifdef CD_VERSION
	else if (cd_event){
	  for (k = 0; k < NUM_CD_TEXTS; k++)
	    if (G_mouse_event_at(CD_BUTTONS[k], mouse_events, &number))
	      CD_play_track(ALL, CD_TEXTS[k].msg, 
			    CD_TEXTS[k].initial_gong, 
			    -1);
	  
	}	  
#endif /* CD_VERSION */
	
	/*====
	 *==== LEFT: goto absolute coordinates =========*
	 *====*/

	else if (G_mouse_event_at(GLOBAL_ROBOT, mouse_events, &number)){
	  
	  action->status          = 1; /* 1=new command issued */
	  action->type            = 4; /* 4=absolute target coordinates */
	  action->continuous_ping = 1; /* 1= regular refresh necessary */
	  action->target_pos_x    = mouse_events[number].value_x;
	  action->target_pos_y    = mouse_events[number].value_y;
	  action->trans_velocity  = robot_state->trans_set_velocity;
	  action->rot_velocity    = robot_state->rot_set_velocity;
	  action->trans_acceleration = robot_state->trans_set_acceleration;
	  action->rot_acceleration = robot_state->rot_set_acceleration;
	  

	  tcx_pantilt_track_point(ALL, mouse_events[number].value_x, 
				  mouse_events[number].value_y);
	}

	


	/* ******* BUTTONS ******** */


	
	/* ******* QUIT ******** */
	/* CHANGED TITLE_BUTTON -> QUIT_BUTTON */
	else if (G_mouse_event_at(QUIT_BUTTON, mouse_events, &number)){
	  G_display_switch(mouse_events[number].name,
			   mouse_events[number].actual_text+1);
	  G_display_switch(QUIT_BUTTON, 1);
	  program_state->quit = 1;
	  if (program_state->tcx_connected_to_BUTTONS){
	    robot_state->red_light_status    = BUTTON_LIGHT_STATUS_OFF;
	    robot_state->yellow_light_status = BUTTON_LIGHT_STATUS_OFF;
	    robot_state->green_light_status  = BUTTON_LIGHT_STATUS_OFF;
	    robot_state->blue_light_status   = BUTTON_LIGHT_STATUS_OFF;
	    set_buttons(ALL, -1);
	  }
	  tcx_base_stop_robot(ALL);
	  if (program_state->tcx_connected_to_BASE) /* already connected? */
	    disconnect_BASE(ALL);
	  if (program_state->tcx_connected_to_PANTILT) /* already connected? */
	    disconnect_PANTILT(ALL);
#ifdef UNIBONN
	  if (program_state->tcx_connected_to_CAMERA) /* already connected? */
	    disconnect_CAMERA(ALL);
	  if (program_state->tcx_connected_to_FLOW) /* already connected? */
	    disconnect_FLOW(ALL);
	  if (program_state->tcx_connected_to_TRACKER) /* already connected? */
	    disconnect_TRACKER(ALL);
	  if (program_state->tcx_connected_to_ARM) /* already connected? */
	    disconnect_ARM(ALL);
	  if (program_state->tcx_connected_to_SPEECH) /* already connected? */
	    disconnect_SPEECH(ALL);
#endif /* UNIBONN */
#ifdef CD_VERSION
	  if (program_state->tcx_connected_to_CD) /* already connected? */
	    disconnect_CD(ALL);
#endif /* CD_VERSION */
	  if (program_state->tcx_connected_to_MAP) /* already connected? */
	    disconnect_MAP(ALL);
	  if (program_state->tcx_connected_to_PLAN) /* already connected? */
	    disconnect_PLAN(ALL);
	  if (program_state->tcx_connected_to_BUTTONS) /* already connected? */
	    disconnect_BUTTONS(ALL);
	  if (program_state->tcx_connected_to_BASESERVER) /* already connected? */
	    disconnect_BASESERVER(ALL);
	  if (program_state->tcx_connected_to_SIMULATOR) /* already connected? */
	    disconnect_SIMULATOR(ALL);
	  if (program_state->tcx_connected_to_SONARINT) /* already connected? */
	    disconnect_SONARINT(ALL);
	  if (program_state->tcx_connected_to_LASERINT) /* already connected? */
	    disconnect_LASERINT(ALL);
	  action->status = 0;
	  action->type = 0;
	  return_value = 1;
	}
	
	
	/* ******* WINDOW MODES ******** */
	else if (G_mouse_event_at(SONAR_ROBOT_SUBTITLE, mouse_events, 
				  &number)){
	  program_state->control_window_mode ^= 1;
	  G_display_switch(SONAR_ROBOT_SUBTITLE, 
			   program_state->control_window_mode);
	}

	else if (G_mouse_event_at(PANTILT_SUBTITLE, mouse_events, 
				  &number)){
	  program_state->pantilt_window_mode = 
	    (program_state->pantilt_window_mode + 1) % 3;
	  if (program_state->pantilt_window_mode == 0)
	    tcx_pantilt_set_velocity(ALL, PAN_LOW_SPEED, TILT_LOW_SPEED);
	  else if (program_state->pantilt_window_mode == 1)
	    tcx_pantilt_set_velocity(ALL, PAN_NORMAL_SPEED, TILT_NORMAL_SPEED);
	  else
	    tcx_pantilt_set_velocity(ALL, PAN_HIGH_SPEED, TILT_HIGH_SPEED);
	  G_display_switch(PANTILT_SUBTITLE, 
			   program_state->pantilt_window_mode);

	  
	}

	/* ******* REFRESH ******** */
	else if (G_mouse_event_at(REFRESH_BUTTON, mouse_events, &number) ||
		 G_mouse_event_at(TITLE_BUTTON, mouse_events, &number)){
	  G_display_switch(REFRESH_BUTTON, 1);
	  G_display_all();
	  G_display_switch(REFRESH_BUTTON, 0);
	}

	/* ******* MAP_UPDATE ******** */
	else if (G_mouse_event_at(MAP_UPDATE_BUTTON, mouse_events, &number) ||
		 G_mouse_event_at(TITLE_BUTTON, mouse_events, &number)){
	  program_state->map_regular_update_on ^= 1;
	  G_display_switch(MAP_UPDATE_BUTTON, 
			   program_state->map_regular_update_on);
	  if (program_state->map_regular_update_on){
	    G_display_matrix(OCCUPANCY_MAP);
	    G_display_markers(PATH);
	    G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y, 
			    robot_state->orientation, 0, dummy_sensors);
	    gettimeofday(&last_map_update, NULL);
	  }
	}

	
	/* ******* VELOCITY AND ACCELERATION ******** */
	else if (G_mouse_event_at(ROBOT_TRANS_SET_VELOCITY, mouse_events, 
				  &number))
	  tcx_base_set_velocity(ALL, mouse_events[number].value,
				robot_state->rot_set_velocity);
	
	else if (G_mouse_event_at(ROBOT_ROT_SET_VELOCITY, mouse_events,
				  &number))
	  tcx_base_set_velocity(ALL, robot_state->trans_set_velocity,
				mouse_events[number].value);
	
	else if (G_mouse_event_at(ROBOT_TRANS_SET_ACCELERATION, mouse_events,
				  &number))
	  tcx_base_set_acceleration(ALL, mouse_events[number].value,
				    robot_state->rot_set_acceleration);
	
	else if (G_mouse_event_at(ROBOT_ROT_SET_ACCELERATION, mouse_events,
				  &number))
	  tcx_base_set_acceleration(ALL, robot_state->trans_set_acceleration,
				    mouse_events[number].value);
	
	
	/* ******* SET PAN/TILT HEAD ******** */
	
	else if (G_mouse_event_at(CAMERA_TILT, mouse_events, &number)){
	  if (program_state->pantilt_window_mode == 0)
	    tcx_pantilt_set_velocity(ALL, PAN_LOW_SPEED, TILT_LOW_SPEED);
	  else if (program_state->pantilt_window_mode == 1)
	    tcx_pantilt_set_velocity(ALL, PAN_NORMAL_SPEED, TILT_NORMAL_SPEED);
	  else
	    tcx_pantilt_set_velocity(ALL, PAN_HIGH_SPEED, TILT_HIGH_SPEED);
	  tcx_pantilt_move_absolute(ALL, sensation->pan, 
				    mouse_events[number].value);
	}	    
	
	
	
	
	else if (G_mouse_event_at(CAMERA_PAN, mouse_events, &number)){
	  if (program_state->pantilt_window_mode == 0)
	    tcx_pantilt_set_velocity(ALL, PAN_LOW_SPEED, TILT_LOW_SPEED);
	  else if (program_state->pantilt_window_mode == 1)
	    tcx_pantilt_set_velocity(ALL, PAN_NORMAL_SPEED, TILT_NORMAL_SPEED);
	  else
	    tcx_pantilt_set_velocity(ALL, PAN_HIGH_SPEED, TILT_HIGH_SPEED);
	  tcx_pantilt_move_absolute(ALL, -mouse_events[number].value, /* !!! */
				    sensation->tilt);
	}	    
	
	
	
	else if (G_mouse_event_at(PSEUDO_PICTURE, mouse_events, &number)){
	  if (program_state->pantilt_window_mode == 0)
	    tcx_pantilt_set_velocity(ALL, PAN_LOW_SPEED, TILT_LOW_SPEED); 
	  else if (program_state->pantilt_window_mode == 1)
	    tcx_pantilt_set_velocity(ALL, PAN_NORMAL_SPEED, TILT_NORMAL_SPEED);
	  else
	    tcx_pantilt_set_velocity(ALL, PAN_HIGH_SPEED, TILT_HIGH_SPEED);
	  tcx_pantilt_move_absolute(ALL, mouse_events[number].rel_x
				    * (MIN_PAN_ANGLE - MAX_PAN_ANGLE)
				    + MAX_PAN_ANGLE, mouse_events[number].rel_y
				    * (MAX_TILT_ANGLE - MIN_TILT_ANGLE)
				    + MIN_TILT_ANGLE);
	}

	
	/******** BASE_MODE_BUTTON ***************/

	
	else if (G_mouse_event_at(BASE_MODE_BUTTON, mouse_events, &number)){
	  if (button == LEFT_BUTTON)
	    tcx_base_set_mode(robot_state->base_mode + 1, ALL);
	  else if (button == MIDDLE_BUTTON)
	    tcx_base_set_mode(robot_state->base_mode - 1, ALL);
	}

#ifdef UNIBONN
	/* ******* SONAR ******** */
	
	else if (G_mouse_event_at(SONAR_BUTTON, mouse_events, &number)){
	  if (program_state->tcx_connected_to_BASE){
	    if (mouse_events[number].actual_text == 0){
	      tcx_sonar_switch_on(ALL);
	      robot_state->sonar_on = 1;
	      G_display_switch(SONAR_BUTTON, robot_state->sonar_on);
	    }
	    else{
	      tcx_sonar_switch_off(ALL);
	    robot_state->sonar_on = 0;
	      G_display_switch(SONAR_BUTTON, robot_state->sonar_on);
	    }
	  }
	  else{
	    fprintf(stderr, "WARNING: Must be connected with BASE ");
	    fprintf(stderr, "to activate sonar. Ignored.\n");
	  }
	}

	/* ******* CAMERA ******** */
	
	else if (G_mouse_event_at(LEFT_CAMERA_BUTTON, mouse_events, &number)){
	  if (button == LEFT_BUTTON){
	    G_display_switch(LEFT_CAMERA_BUTTON, 1);
	    tcx_camera_image(ALL, 0);
	  }
	  else if (button == MIDDLE_BUTTON){
	    program_state->camera_continuous_grabbing = 0;
	    G_display_switch(LEFT_CAMERA_BUTTON, 2);
	    tcx_camera_image(ALL, 0);
	  }
	  else{
	    program_state->camera_continuous_grabbing = -1;
	    G_display_switch(LEFT_CAMERA_BUTTON, 0);
	  }
	  /*tcx_camera_find_marker(ALL, 0);*/
	}
	else if (G_mouse_event_at(RIGHT_CAMERA_BUTTON, mouse_events, &number)){
	  if (button == LEFT_BUTTON){
	    G_display_switch(RIGHT_CAMERA_BUTTON, 1);
	    tcx_camera_image(ALL, 2);
	  }
	  else if (button == MIDDLE_BUTTON){
	    program_state->camera_continuous_grabbing = 2;
	    G_display_switch(RIGHT_CAMERA_BUTTON, 2);
	    tcx_camera_image(ALL, 2);
	  }
	  else{
	    program_state->camera_continuous_grabbing = -1;
	    G_display_switch(RIGHT_CAMERA_BUTTON, 0);
	  }
	  /*tcx_camera_find_marker(ALL, 2);*/
	}
#endif

	/* ******* CAMERA ******** */
	
	else if (G_mouse_event_at(ACQUIRE_CAMERA_IMAGE_BUTTON,
				  mouse_events, &number)){
	  if (button == LEFT_BUTTON){
	    G_display_switch(ACQUIRE_CAMERA_IMAGE_BUTTON, 1);
	    tcx_camera_image(ALL, 0);
	  }
	}
	else if (G_mouse_event_at(CONTINUOUS_CAMERA_IMAGE_BUTTON,
				  mouse_events, &number)){
	  program_state->camera_continuous_grabbing = 
	    1 - program_state->camera_continuous_grabbing;
	  G_display_switch(CONTINUOUS_CAMERA_IMAGE_BUTTON, 
			   program_state->camera_continuous_grabbing);
	  if (program_state->camera_continuous_grabbing)
	    tcx_camera_register_auto_update(ALL, 50);
	  else
	    tcx_camera_register_auto_update(ALL, 0);
	}

#ifdef UNIBONN
	else if (G_mouse_event_at(CAMERA_X_DISPLAY_BUTTON,
				  mouse_events, &number)){
	  if (mouse_events[number].actual_text == 0)
	    tcx_camera_x_display_on(ALL);
	  else
	    tcx_camera_x_display_off(ALL);
	}
	
	else if (G_mouse_event_at(CAMERA_MAPS_BUTTON,
				  mouse_events, &number)){
	  if (mouse_events[number].actual_text == 0)
	    tcx_sunvis_send_occupancy_maps_on(ALL);
	  else
	    tcx_sunvis_send_occupancy_maps_off(ALL);
	}

	else if (G_mouse_event_at(CAMERA_COLLI_LINES_BUTTON,
				  mouse_events, &number)){
	  if (mouse_events[number].actual_text == 0)
	    tcx_sunvis_send_collision_info_on(ALL);
	  else
	    tcx_sunvis_send_collision_info_off(ALL);
	}
	
	else if (G_mouse_event_at(CAMERA_OBJECTS_BUTTON, 
				  mouse_events, &number)){
	  if (mouse_events[number].actual_text == 0)
	    tcx_sunvis_register_auto_update(1, ALL);
	  else
	    tcx_sunvis_register_auto_update(0, ALL);
	}

	


	/* ******* CONNECT/DISCONNECT (TCX) ******** */
	
	else if (G_mouse_event_at(CONNECT_BASE_BUTTON, mouse_events, &number)){
	  if (program_state->tcx_connected_to_BASE !=
	      mouse_events[number].actual_text)
	    fprintf(stderr, 
		    "ERROR: something wrong with CONNECT_BASE_BUTTON.\n");
	  if (program_state->tcx_connected_to_BASE) /* already connected? */
	    disconnect_BASE(ALL);
	  else{
	    program_state->do_connect_to_BASE = 1; /* connect! */
	    connect_to_tcx(ALL);	/* and do it! */
	  }
	}
	
	else if (G_mouse_event_at(CONNECT_PANTILT_BUTTON, 
				  mouse_events, &number)){
	  if (program_state->tcx_connected_to_PANTILT !=
	      mouse_events[number].actual_text)
	    fprintf(stderr, 
		    "ERROR: something wrong with CONNECT_PANTILT_BUTTON.\n");
	  if (program_state->tcx_connected_to_PANTILT) /* already connected? */
	    disconnect_PANTILT(ALL);
	  else{
	    program_state->do_connect_to_PANTILT = 1; /* connect! */
	    connect_to_tcx(ALL);	/* and do it! */
	  }
	}
	
	else if (G_mouse_event_at(CONNECT_MAP_BUTTON, 
				  mouse_events, &number)){
	  if (program_state->tcx_connected_to_MAP !=
	      mouse_events[number].actual_text)
	    fprintf(stderr, 
		    "ERROR: something wrong with CONNECT_MAP_BUTTON.\n");
	  if (program_state->tcx_connected_to_MAP) /* already connected? */
	    disconnect_MAP(ALL);
	  else{
	    program_state->do_connect_to_MAP = 1; /* connect! */
	    connect_to_tcx(ALL);	/* and do it! */
	  }
	}
	
	else if (G_mouse_event_at(CONNECT_PLAN_BUTTON, 
				  mouse_events, &number)){
	  if (program_state->tcx_connected_to_PLAN !=
	      mouse_events[number].actual_text)
	    fprintf(stderr, 
		    "ERROR: something wrong with CONNECT_PLAN_BUTTON.\n");
	  if (program_state->tcx_connected_to_PLAN) /* already connected? */
	    disconnect_PLAN(ALL);
	  else{
	    program_state->do_connect_to_PLAN = 1; /* connect! */
	    connect_to_tcx(ALL);	/* and do it! */
	  }
	}
	
	else if (G_mouse_event_at(CONNECT_BASESERVER_BUTTON, 
				  mouse_events, &number)){
	  if (program_state->tcx_connected_to_BASESERVER !=
	      mouse_events[number].actual_text)
	    fprintf(stderr, 
		    "ERROR: something wrong with CONNECT_BASESERVER_BUTTON.\n");
	  if (program_state->tcx_connected_to_BASESERVER) /* already connected? */
	    disconnect_BASESERVER(ALL);
	  else{
	    program_state->do_connect_to_BASESERVER = 1; /* connect! */
	    connect_to_tcx(ALL);	/* and do it! */
	  }
	}
	
	
	else if (G_mouse_event_at(CONNECT_SIMULATOR_BUTTON, 
				  mouse_events, &number)){
	  if (program_state->tcx_connected_to_SIMULATOR !=
	      mouse_events[number].actual_text)
	    fprintf(stderr, 
		    "ERROR: something wrong with CONNECT_SIMULATOR_BUTTON.\n");
	  if (program_state->tcx_connected_to_SIMULATOR) /* already connected? */
	    disconnect_SIMULATOR(ALL);
	  else{
	    program_state->do_connect_to_SIMULATOR = 1; /* connect! */
	    connect_to_tcx(ALL);	/* and do it! */
	  }
	}
	
	
	else if (G_mouse_event_at(CONNECT_SONARINT_BUTTON, 
				  mouse_events, &number)){
	  if (program_state->tcx_connected_to_SONARINT !=
	      mouse_events[number].actual_text)
	    fprintf(stderr, 
		    "ERROR: something wrong with CONNECT_SONARINT_BUTTON.\n");
	  if (program_state->tcx_connected_to_SONARINT) /* already connected? */
	    disconnect_SONARINT(ALL);
	  else{
	    program_state->do_connect_to_SONARINT = 1; /* connect! */
	    connect_to_tcx(ALL);	/* and do it! */
	  }
	}
	
	
	else if (G_mouse_event_at(CONNECT_LASERINT_BUTTON, 
				  mouse_events, &number)){
	  if (program_state->tcx_connected_to_LASERINT !=
	      mouse_events[number].actual_text)
	    fprintf(stderr, 
		    "ERROR: something wrong with CONNECT_LASERINT_BUTTON.\n");
	  if (program_state->tcx_connected_to_LASERINT) /* already connected? */
	    disconnect_LASERINT(ALL);
	  else{
	    program_state->do_connect_to_LASERINT = 1; /* connect! */
	    connect_to_tcx(ALL);	/* and do it! */
	  }
	}
	
	
	

	else if (G_mouse_event_at(CONNECT_ARM_BUTTON, 
				  mouse_events, &number)){
	  if (program_state->tcx_connected_to_ARM !=
	      mouse_events[number].actual_text)
	    fprintf(stderr, 
		    "ERROR: something wrong with CONNECT_ARM_BUTTON.\n");
	  if (program_state->tcx_connected_to_ARM) /* already connected? */
	    disconnect_ARM(ALL);
	  else{
	    program_state->do_connect_to_ARM = 1; /* connect! */
	    connect_to_tcx(ALL);	/* and do it! */
	  }
	}
	
	else if (G_mouse_event_at(CONNECT_CAMERA_BUTTON, 
				  mouse_events, &number)){
	  if (program_state->tcx_connected_to_CAMERA !=
	      mouse_events[number].actual_text)
	    fprintf(stderr, 
		    "ERROR: something wrong with CONNECT_CAMERA_BUTTON.\n");
	  if (program_state->tcx_connected_to_CAMERA) /* already connected? */
	    disconnect_CAMERA(ALL);
	  else{
	    program_state->do_connect_to_CAMERA = 1; /* connect! */
	    connect_to_tcx(ALL);	/* and do it! */
	  }
	}
	
	else if (G_mouse_event_at(CONNECT_SUNVIS_BUTTON, 
				  mouse_events, &number)){
	  if (program_state->tcx_connected_to_SUNVIS !=
	      mouse_events[number].actual_text)
	    fprintf(stderr, 
		    "ERROR: something wrong with CONNECT_SUNVIS_BUTTON.\n");
	  if (program_state->tcx_connected_to_SUNVIS) /* already connected? */
	    disconnect_SUNVIS(ALL);
	  else{
	    program_state->do_connect_to_SUNVIS = 1; /* connect! */
	    connect_to_tcx(ALL);	/* and do it! */
	  }
	}

	else if (G_mouse_event_at(CONNECT_TRACKER_BUTTON, 
				  mouse_events, &number)){
	  if (program_state->tcx_connected_to_TRACKER !=
	      mouse_events[number].actual_text)
	    fprintf(stderr, 
		    "ERROR: something wrong with CONNECT_TRACKER_BUTTON.\n");
	  if (program_state->tcx_connected_to_TRACKER) /* already connected? */
	    disconnect_TRACKER(ALL);
	  else{
	    program_state->do_connect_to_TRACKER = 1; /* connect! */
	    connect_to_tcx(ALL);	/* and do it! */
	  }
	}

	else if (G_mouse_event_at(CONNECT_FLOW_BUTTON, 
				  mouse_events, &number)){
	  if (program_state->tcx_connected_to_FLOW !=
	      mouse_events[number].actual_text)
	    fprintf(stderr, 
		    "ERROR: something wrong with CONNECT_FLOW_BUTTON.\n");
	  if (program_state->tcx_connected_to_FLOW) /* already connected? */
	    disconnect_FLOW(ALL);
	  else{
	    program_state->do_connect_to_FLOW = 1; /* connect! */
	    connect_to_tcx(ALL);	/* and do it! */
	  }
	}


	else if (G_mouse_event_at(CONNECT_SPEECH_BUTTON, 
				  mouse_events, &number)){
	  if (program_state->tcx_connected_to_SPEECH !=
	      mouse_events[number].actual_text)
	    fprintf(stderr, 
		    "ERROR: something wrong with CONNECT_SPEECH_BUTTON.\n");
	  if (program_state->tcx_connected_to_SPEECH) /* already connected? */
	    disconnect_SPEECH(ALL);
	  else{
	    program_state->do_connect_to_SPEECH = 1; /* connect! */
	    connect_to_tcx(ALL);	/* and do it! */
	  }
	}
	

	else if (G_mouse_event_at(CONNECT_BUTTONS_BUTTON, 
				  mouse_events, &number)){
	  if (program_state->tcx_connected_to_BUTTONS !=
	      mouse_events[number].actual_text)
	    fprintf(stderr, 
		    "ERROR: something wrong with CONNECT_BUTTONS_BUTTON.\n");
	  if (program_state->tcx_connected_to_BUTTONS) /* already connected? */
	    disconnect_BUTTONS(ALL);
	  else{
	    program_state->do_connect_to_BUTTONS = 1; /* connect! */
	    connect_to_tcx(ALL);	/* and do it! */
	  }
	}


#endif /* UNIBONN */	
#ifdef CD_VERSION

	else if (G_mouse_event_at(CONNECT_CD_BUTTON, 
				  mouse_events, &number)){
	  if (program_state->tcx_connected_to_CD !=
	      mouse_events[number].actual_text)
	    fprintf(stderr, 
		    "ERROR: something wrong with CONNECT_CD_BUTTON.\n");
	  if (program_state->tcx_connected_to_CD) /* already connected? */
	    disconnect_CD(ALL);
	  else{
	    program_state->do_connect_to_CD = 1; /* connect! */
	    connect_to_tcx(ALL);	/* and do it! */
	  }
	}
#endif /* CD_VERSION */


#ifdef xxx
	/* ******* LOGGING_BUTTON ******** */
	else if (G_mouse_event_at(LOGGING_BUTTON, mouse_events, &number)){
	  robot_specifications->log_file_unopenable = 0; /* give it a try */
	  robot_specifications->do_log = 1 - mouse_events[number].actual_text;
	  file_is_open = 0;
	  if (!robot_specifications->do_log){
	     if (robot_specifications->log_file_open) 
	       close_log_file(ALL); 
	  } 
	  else if (open_logfile(ALL))
	    file_is_open = 1;
	  G_display_switch(LOGGING_BUTTON, file_is_open);
	}
#endif

	/* ******* RESET_BUTTON ******** */
	else if (G_mouse_event_at(RESET_ROBOT_BUTTON, mouse_events, &number)){
	  G_display_switch(RESET_ROBOT_BUTTON, 1);
	  tcx_reset_joystick(ALL); 
	  usleep(600000);
	  G_display_switch(RESET_ROBOT_BUTTON, 0);
	}


#ifdef junk /*!!*/
	/* ****** EXPL_RESET_BUTTON ***** */


	
	else if (G_mouse_event_at(EXPL_RESET_BUTTON, 
				  mouse_events, &number)){
	  if (button == RIGHT_BUTTON)
	    tcx_plan_reset_exploration_table(ALL);
	  else{
	    program_state->auto_reset_expl_mode ^= 1;
	    G_display_switch(EXPL_RESET_BUTTON,
			    program_state->auto_reset_expl_mode);
	  }
	}
#endif	

#ifdef UNIBONN
	/* ****** ARM_MOVE_OUT_IN_BUTTON ***** */

	else if (G_mouse_event_at(ARM_MOVE_OUT_IN_BUTTON, 
				  mouse_events, &number)){
	  if (robot_state->arm_moved_out)
	    tcx_arm_move_in(ALL, robot_specifications->arm_move_out_in_force);
	  else
	    tcx_arm_move_out(ALL, robot_specifications->arm_move_out_in_force);
	}

        /* ****** ARM_PICK_BUTTON ***** */

	else if (G_mouse_event_at(ARM_PICK_BUTTON, 
				  mouse_events, &number)){
	  if (robot_state->pick_sequence==0)
	    {
	    tcx_arm_pickup(ALL);
	    fprintf(stderr,"PICKUP aufrufen");
	    }
	  else 
	    {
	    if (robot_state->pick_sequence==1)
	      {
	      tcx_arm_lift(ALL);
	      fprintf(stderr,"LIFT aufrufen");
	      }
	    else
	      {
	      tcx_arm_drop(ALL);
	      fprintf(stderr,"DROP aufrufen");
	      }
	    }
	}
	

	/* ****** ARM_CLOSE_OPEN_GRIPPER_BUTTON ***** */

	else if (G_mouse_event_at(ARM_CLOSE_OPEN_GRIPPER_BUTTON,
				  mouse_events, &number)){
	  if (button == MIDDLE_BUTTON)
	    tcx_arm_park_gripper(ALL);
	  else if (!robot_state->arm_gripper_closed)
	    tcx_arm_close_gripper(ALL);
	  else
	    tcx_arm_open_gripper(ALL);
	}


	/* ****** ARM_MAST_POSITION_BUTTON ***** */

	else if (G_mouse_event_at(ARM_MAST_POSITION_BUTTON, mouse_events, 
				  &number))
	  tcx_arm_set_mast_absolute(ALL, mouse_events[number].value);
	
	/* ****** ARM_GRIPPER_ORIENTATION_BUTTON ***** */
	else if (G_mouse_event_at(ARM_GRIPPER_ORIENTATION_BUTTON, 
				  mouse_events, &number)){
	  int value = (int) (mouse_events[number].value * 4.0 + 0.5);
	  if (button == LEFT_BUTTON)
	    tcx_arm_set_gripper_orientation_absolute(ALL, 
						     ((float) (value * 0.25)));
	  else
	    tcx_arm_set_gripper_orientation_absolute(ALL, 
						     mouse_events[number].value);
	}
	
	/* ****** ARM_GRIPPER_PAD_POSITION_BUTTON ***** */

	else if (G_mouse_event_at(ARM_GRIPPER_PAD_POSITION_BUTTON,
				  mouse_events, 
				  &number)){
	  int value = (int) (mouse_events[number].value + 0.5);
	  fprintf(stderr, "Not implemented.\n");
	  putc(7, stderr);
	  if (button == LEFT_BUTTON)
	    G_display_value(ARM_GRIPPER_PAD_POSITION_BUTTON, 
			    ((float) value));
	  else
	    G_display_value(ARM_GRIPPER_PAD_POSITION_BUTTON, 
			    mouse_events[number].value);
	}


	/* ****** ARM_IN_OUT_BUTTON ***** */

	else if (G_mouse_event_at(ARM_IN_OUT_BUTTON, mouse_events, 
				  &number)){
	  int value = (int) (mouse_events[number].value + 0.5);
	  fprintf(stderr, "Not implemented.\n");
	  putc(7, stderr);
	  if (button == LEFT_BUTTON)
	    G_display_value(ARM_IN_OUT_BUTTON, 
			    ((float) value));
	  else
	    G_display_value(ARM_IN_OUT_BUTTON, mouse_events[number].value);
	}
#endif /* UNIBONN */

	/* ******* DUMP_BUTTON ******** */
	
	else if (G_mouse_event_at(DUMP_BUTTON, mouse_events, &number)){
	  G_display_switch(DUMP_BUTTON, 1);
	  tcx_map_dump(ALL);
	  G_display_switch(DUMP_BUTTON, 0);
	}
	
	
	/* ******* AUTONOMOUS_BUTTON ******** */
	
	else if (G_mouse_event_at(AUTONOMOUS_BUTTON, mouse_events, &number)){
	  if (program_state->tcx_connected_to_PLAN){
	    if (!mouse_events[number].actual_text){ /* not on yet */
	      tcx_plan_start_autonomous(-1, ALL);
	    } 
	    else 
	      tcx_plan_stop_autonomous(1, ALL);
	  }
	  else{
	    putc(7,stderr);
	    fprintf(stderr, "You have to connect to a planner for going autonomously. Sorry.\n");
	  }
	}
	
#ifdef UNIBONN
	/* ******* HUNTING_BUTTON ******** */
	
	else if (G_mouse_event_at(HUNTING_BUTTON, mouse_events, &number)){
	  if (!program_state->hunting_mode)
	    start_hunting(ALL);
	  else
	    stop_hunting(ALL);
	}

	/* ******* NUM_OBJECTS_BUTTON ******** */
	
	else if (G_mouse_event_at(NUM_OBJECTS_BUTTON, mouse_events, &number)){
	  if (button == LEFT_BUTTON && program_state->n_carried_objetcs < 3)
	     program_state->n_carried_objetcs++;
	  else if (button == MIDDLE_BUTTON && 
		   program_state->n_carried_objetcs >= 1)
	    program_state->n_carried_objetcs--;
	  G_display_switch(NUM_OBJECTS_BUTTON, 
			   program_state->n_carried_objetcs);
	}

	/* ******* TRACKER_BUTTON ******** */
	
	else if (G_mouse_event_at(TRACKER_BUTTON, mouse_events, &number)){
	  if (program_state->tcx_connected_to_TRACKER){
	    if (mouse_events[number].actual_text){
	      tracker_stop(ALL);
	      G_display_switch(TRACKER_BUTTON, 0);
	    }
	    else{
	      G_display_switch(TRACKER_BUTTON, 1);
	      tracker_start(ALL);
	    }	    
	  }
	}
	  


#endif /* UNIBONN */
#ifdef TOURGUIDE_VERSION

	/* ******* TOUR_LEARN_BUTTON ******** */
	

	else if (G_mouse_event_at(TOUR_LEARN_BUTTON, mouse_events, &number)){
	  if (!program_state->learning_tour)
	    start_learning_tour(ALL);
	  else
	    finish_learning_tour(ALL);
	}
	

	/* ******* TOUR_GIVE_BUTTON ******** */
	

	else if (G_mouse_event_at(TOUR_GIVE_BUTTON, mouse_events, &number)){
	  if (!program_state->giving_tour)
	    start_giving_tour(ALL);
	  else
	    stop_giving_tour(ALL, 0);
	}
	

	/* ******* TOUR_SAVE_BUTTON ******** */
	

	else if (G_mouse_event_at(TOUR_SAVE_BUTTON, mouse_events, &number)){
	  save_tour(ALL);
	}

	/* ******* TOUR_LOAD_BUTTON ******** */

	else if (G_mouse_event_at(TOUR_LOAD_BUTTON, mouse_events, &number)){
	  load_tour(ALL);
	}
	
#endif /* TOURGUIDE_VERSION */

	
#ifdef xxx
/**/	/* ******* SCRIPT_BUTTON (SEVERAL) ******** */
/**/	else if (G_mouse_event_at(SCRIPT_BUTTON, mouse_events, &number)){
/**/	  if (!program_state->script_on){ /* then do open! */
/**/	    if (button == MIDDLE_BUTTON)
/**/	      open_and_read_script_file(ALL, 0); /*don't generate model files*/
/**/	    else
/**/	      open_and_read_script_file(ALL, 1);
/**/	    display_script_event(ALL, 0, 0); /* initial display */
/**/	    printf("");
/**/	    fflush(stdout);
/**/	  }
/**/	  else
/**/	    close_script_file(ALL);
/**/	}
/**/	else if (G_mouse_event_at(INCREASE_EPISODE_BUTTON, 
/**/				  mouse_events, &number)){
/**/	  if (program_state->script_on){ /* otherwise ignore! */
/**/	    if (program_state->current_episode_displayed <
/**/		program_state->script_number_episodes - 1)
/**/	      display_script_event(ALL, 
/**/				   program_state->current_episode_displayed
/**/				   + 1, 0);
/**/	  }
/**/	}
/**/	else if (G_mouse_event_at(DECREASE_EPISODE_BUTTON, 
/**/				  mouse_events, &number)){
/**/	  if (program_state->script_on){ /* otherwise ignore! */
/**/	    if (program_state->current_episode_displayed > 0)
/**/	      display_script_event(ALL, 
/**/				   program_state->current_episode_displayed
/**/				   - 1, 0);
/**/	  }
/**/	}
/**/	else if (G_mouse_event_at(INCREASE_EVENT_BUTTON, 
/**/				  mouse_events, &number)){
/**/	  if (program_state->script_on){ /* otherwise ignore! */
/**/	    if (program_state->current_event_displayed < program_state->
/**/		script_nevents_per_episode[program_state->
/**/					   current_episode_displayed] - 1)
/**/	      display_script_event(ALL, 
/**/				   program_state->current_episode_displayed,
/**/				   program_state->current_event_displayed + 1);
/**/	    else if (program_state->current_episode_displayed <
/**/		     program_state->script_number_episodes - 1)
/**/	      display_script_event(ALL, 
/**/				   program_state->current_episode_displayed
/**/				   + 1, 0);
/**/	  }
/**/	}
/**/	else if (G_mouse_event_at(DECREASE_EVENT_BUTTON, 
/**/				  mouse_events, &number)){
/**/	  if (program_state->script_on){ /* otherwise ignore! */
/**/	    if (program_state->current_event_displayed > 0)
/**/	      display_script_event(ALL, 
/**/				   program_state->current_episode_displayed,
/**/				   program_state->current_event_displayed - 1);
/**/	    else if (program_state->current_episode_displayed > 0)
/**/	      display_script_event(ALL, 
/**/				   program_state->current_episode_displayed
/**/				   - 1, program_state->
/**/				   script_nevents_per_episode[program_state->
/**/							      current_episode_displayed - 1] - 1);
/**/	  }
/**/	}
/**/	
/**/	else if (G_mouse_event_at(EPISODE_DIAL, mouse_events, &number)){
/**/	  display_script_event(ALL, (int)
/**/			       (mouse_events[number].value * 
/**/				((float)
/**/				 program_state->script_number_episodes)),
/**/			       0);
/**/	}
/**/	
/**/	else if (G_mouse_event_at(EVENT_DIAL, mouse_events, &number)){
/**/	  display_script_event(ALL, 
/**/			       program_state->current_episode_displayed, (int)
/**/			       (mouse_events[number].value * 
/**/				((float) program_state->
/**/				 script_nevents_per_episode
/**/				 [program_state->current_episode_displayed])));
/**/	}
/**/	
#endif


	
	/********* JUST UPDATE DISPLAY *********/
	
	else if (button == RIGHT_BUTTON){
	  G_display_all();
	}

	G_display_switch(BUSY_BUTTON, 0);

      }





      if (mouse_event_detected == 2 &&
	  action->status == 2 && /* action command is in execution */
	  action->type == 2){	/* velocity-type action */

	G_display_switch(BUSY_BUTTON, 1);

	/*====
	 *==== DEFAULT in window: pick target velocity =========*
	 *====*/
	if (G_mouse_event_at(SONAR_ROBOT, mouse_events, &number)){
	  action->status          = 1; /* 1=new command issued */
	  action->type            = 2; /* 2=target velocities */
	  action->continuous_ping = 0; /* 0=ping not necessary */
	  action->trans_velocity  = mouse_events[number].value_y 
	    / (robot_specifications->max_sonar_range*1.05)
	      * MAX_TRANS_VELOCITY;
	  action->rot_velocity    = mouse_events[number].value_x
	    / (robot_specifications->max_sonar_range*1.05)
	      * MAX_ROT_VELOCITY * 0.5;
	  action->rot_direction   = action->rot_velocity < 0.0;
	  action->trans_direction = action->trans_velocity < 0.0;
	  action->rot_velocity    = fabs(action->rot_velocity)
	    - (robot_specifications->robot_size 
	     / (robot_specifications->max_sonar_range*1.05)
	     * MAX_ROT_VELOCITY * 0.5);
	  if (action->rot_velocity < 0.0)
	    action->rot_velocity = 0.0;
	  action->trans_velocity  = fabs(action->trans_velocity)
	    - (robot_specifications->robot_size 
	       / (robot_specifications->max_sonar_range*1.05)
	       * MAX_TRANS_VELOCITY);
	  if (action->trans_velocity < 0.0)
	    action->trans_velocity = 0.0;
	  action->trans_acceleration = robot_state->trans_set_acceleration;
	  action->rot_acceleration = robot_state->rot_set_acceleration;
	}
	

	/*====
	 *==== NOT IN WINDOW: stop =========*
	 *====*/
	
	else{
	  action->status          = 1; /* 1=new command issued */
	  action->type            = 1; /* 1=stop */
	  action->continuous_ping = 0; /* 1=ping not necessary */
	  action->target_pos_x    = 0.0;
	  action->target_pos_y    = 0.0;
	  action->trans_velocity  = robot_state->trans_set_velocity;
	  action->rot_velocity    = robot_state->rot_set_velocity;
	  action->trans_acceleration = robot_state->trans_set_acceleration;
	  action->rot_acceleration = robot_state->rot_set_acceleration;
	}	  
	
	G_display_switch(BUSY_BUTTON, 0);

      }
    }

  }    
  return return_value;
}

