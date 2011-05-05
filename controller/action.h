
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






extern struct timeval last_map_update;





/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_base_subscribe(ALL_PARAMS, 
			int subscribe_status_report,
			int subscribe_sonar_report,
			int subscribe_colli_report,
			int subscribe_ir_report,
			int subscribe_laser_report);
void tcx_base_get_robot_position(ALL_PARAMS);
void tcx_base_stop_robot(ALL_PARAMS);
void tcx_reset_joystick(ALL_PARAMS);
void tcx_base_update_status(ALL_PARAMS);
void tcx_base_set_velocity(ALL_PARAMS, float trans_speed, float rot_speed);
void tcx_base_set_acceleration(ALL_PARAMS, float trans_accel, float rot_accel);
void tcx_base_rotate_clockwise(ALL_PARAMS);
void tcx_base_rotate_anticlockwise(ALL_PARAMS);
void tcx_base_translate_forward(ALL_PARAMS);
void tcx_base_translate_backward(ALL_PARAMS);
void tcx_base_goto_relative(ALL_PARAMS);
void tcx_base_goto_absolute(int new_target, ALL_PARAMS);
void tcx_base_approach_absolute(int new_target, ALL_PARAMS);
void tcx_base_approach_two_points_absolute(int new_target, ALL_PARAMS);
void tcx_base_disconnect(ALL_PARAMS);
void tcx_base_set_mode(int mode, ALL_PARAMS);

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_sonar_get_values(ALL_PARAMS);
void tcx_sonar_switch_on(ALL_PARAMS);
void tcx_sonar_switch_off(ALL_PARAMS);

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_speech_talk_text(ALL_PARAMS, char *text);
void tcx_speech_disconnect(ALL_PARAMS);

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void tcx_pantilt_init(ALL_PARAMS);
void tcx_pantilt_move_absolute(ALL_PARAMS, float pan, float tilt);
void tcx_pantilt_set_velocity(ALL_PARAMS, float pan_speed, float tilt_speed);
void tcx_pantilt_set_acceleration(ALL_PARAMS, 
				  float pan_accel, float tilt_accel);
void tcx_pantilt_terminate(ALL_PARAMS);
void tcx_pantilt_disconnect(ALL_PARAMS);
void tcx_pantilt_track_point(ALL_PARAMS, 
			     float x, float y);
void tcx_pantilt_stop_tracking(ALL_PARAMS);


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_camera_register_auto_update(ALL_PARAMS, int subscribe_image);
void tcx_camera_x_display_on(ALL_PARAMS);
void tcx_camera_x_display_off(ALL_PARAMS);
void tcx_camera_initialize_marker(ALL_PARAMS);
void tcx_camera_find_marker(ALL_PARAMS, int camera);
void tcx_camera_image(ALL_PARAMS, int camera);

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



void tcx_map_quit(ALL_PARAMS);
void tcx_map_partial_map_query(ALL_PARAMS, int first_x, int first_y,
			       int size_x, int size_y);
void tcx_map_register_auto_update(ALL_PARAMS, 
				  int subscribe_maps,
				  int subscribe_position_correction);
void tcx_map_dump(ALL_PARAMS);
void tcx_map_correction_parameters_query(ALL_PARAMS);
void tcx_map_enable_map_update(ALL_PARAMS);
void tcx_map_disable_map_update(ALL_PARAMS);
void clear_all_sensor_maps(ALL_PARAMS);

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_plan_register_auto_update(ALL_PARAMS, 
				   int subscribe_status);
void tcx_plan_goal_message(float x, float y, int name, int add, ALL_PARAMS);
void tcx_remove_all_goals(ALL_PARAMS);
void tcx_plan_start_autonomous(int expl, ALL_PARAMS);
void tcx_plan_stop_autonomous(int stop_base, ALL_PARAMS);
void tcx_plan_new_robot_pos(ALL_PARAMS);
void tcx_plan_quit(ALL_PARAMS);
void tcx_plan_action_query(ALL_PARAMS);
void tcx_plan_reset_exploration_table(ALL_PARAMS);


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_arm_register_auto_update(ALL_PARAMS, 
				  int subscribe_status);
void tcx_arm_move_out(ALL_PARAMS, float force);
void tcx_arm_move_in(ALL_PARAMS, float force);
void tcx_arm_close_gripper(ALL_PARAMS);
void tcx_arm_open_gripper(ALL_PARAMS);
void tcx_arm_park_gripper(ALL_PARAMS);
void tcx_arm_set_mast_absolute(ALL_PARAMS, float height);
void tcx_arm_set_gripper_orientation_absolute(ALL_PARAMS, float orientation);
void tcx_arm_pickup(ALL_PARAMS);
void tcx_arm_lift(ALL_PARAMS);
void tcx_arm_drop(ALL_PARAMS);

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_sunvis_register_auto_update(int objects, ALL_PARAMS);
void tcx_sunvis_send_occupancy_maps_on(ALL_PARAMS);
void tcx_sunvis_send_occupancy_maps_off(ALL_PARAMS);
void tcx_sunvis_send_collision_info_on(ALL_PARAMS);
void tcx_sunvis_send_collision_info_off(ALL_PARAMS);

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tracker_Order(int Order);
void tracker_Command(int SenderID,
		     int MainCommand,
		     int SubCommand, 
		     int Parameter1,
		     int Parameter2,
		     float Parameter3,
		     float Parameter4);
void tracker_init(ALL_PARAMS);
void tracker_start(ALL_PARAMS);
void tracker_stop(ALL_PARAMS);


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void CD_play_track(ALL_PARAMS, int track_no, int initial_sound, int final_sound);


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void buttons_init(ALL_PARAMS);
void set_buttons(ALL_PARAMS, int new);
void buttons_simulate_button_press(ALL_PARAMS, int number);
void buttons_effect(ALL_PARAMS);

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void disconnect_FLOW(ALL_PARAMS);
void flow_request_info(ALL_PARAMS);

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
				       BASE_robot_position_reply_ptr pos);







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
				      BASE_update_status_reply_ptr status);




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
					BASE_action_executed_reply_ptr data);




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
			      SONAR_sonar_reply_ptr sonar);



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
			       int          *err_value);




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
				      int          *speaking);




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
				int          *data);




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
				    PANTILT_position_reply_ptr data);




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
				  PANTILT_limits_reply_ptr data);




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
				  PANTILT_status_update_ptr data);



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
			       PLAN_status_reply_ptr    status);


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


void tcx_close_handler(char *name);

     
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
				ARM_position_reply_ptr pos);


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
				     ARM_update_status_reply_ptr   status);


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
				       ARM_action_executed_reply_ptr data);




/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         CAMERA_init_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void CAMERA_init_reply_handler(TCX_REF_PTR   ref,
				int          *data);





/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         CAMERA_grey_image_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void CAMERA_image_reply_handler(TCX_REF_PTR             ref,
				CAMERA_image_reply_ptr image);


#endif /* UNIBONN */



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
				     MAP_partial_map_reply_ptr map);




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
				     MAP_correction_parameters_reply_ptr corr);




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
			       PLAN_action_reply_ptr    action);



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
				 SUNVIS_object_reply_ptr    object_msg);

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#endif /* UNIBONN */


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


int initiate_action(ALL_PARAMS);





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


void reset_robot(ALL_PARAMS);







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


void disconnect_BASE(ALL_PARAMS);

  


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


void disconnect_PANTILT(ALL_PARAMS);

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


void disconnect_CD(ALL_PARAMS);
#endif /* CD_VERSION */
 
#ifdef UNIBONN

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


void disconnect_CAMERA(ALL_PARAMS);


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


void disconnect_TRACKER(ALL_PARAMS);


#endif  

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


void disconnect_SPEECH(ALL_PARAMS);


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
#ifdef UNIBONN

void disconnect_SUNVIS(ALL_PARAMS);

#endif /* UNIBONN */
 

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

void disconnect_MAP(ALL_PARAMS);


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


void disconnect_PLAN(ALL_PARAMS);



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


void disconnect_ARM(ALL_PARAMS);



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


int refresh_action(ALL_PARAMS);


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
				ALL_PARAMS);


void disconnect_BASESERVER(ALL_PARAMS);

void disconnect_SIMULATOR(ALL_PARAMS);

void disconnect_SONARINT(ALL_PARAMS);

void disconnect_LASERINT(ALL_PARAMS);

void
display_global_robot(ALL_PARAMS);
