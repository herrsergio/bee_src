
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





#include "collisionIntern.h"


/* The strcuctures determine the behaviour in the
 * different modes.
 */
mode_structure **mode_structure_array;
mode_structure *ACTUAL_MODE = NULL;
int mode_number;
velocities actual_velocities;

/* If this flag is set the collision avoidance moves backward. */
BOOLEAN colli_go_backward = FALSE;



/**********************************************************************
 **********************************************************************
 *               Functions for the different modes.                   *
 **********************************************************************
 **********************************************************************/


/**********************************************************************
 * Set the direction to move to the next target point.
 **********************************************************************/
void
COLLI_GoForward()
{
#ifdef UNIBONN
  if (use_vision)
    tcxSendMsg(SUNVIS, "SUNVIS_look_forward", NULL);
#endif
    colli_go_backward = FALSE;
}

void
COLLI_GoBackward()
{
#ifdef UNIBONN   
  if (use_vision)
    tcxSendMsg(SUNVIS, "SUNVIS_look_backward", NULL);
#endif
  colli_go_backward = TRUE;
}



void COLLI_SetMode( int mode)
{
  int intMode = (int) mode;
  
  if (intMode >= NUMBER_OF_MODES || intMode < 0) {
    fprintf(stderr, "Wrong mode number (%d). Use default mode.\n", intMode);
    ACTUAL_MODE = mode_structure_array[DEFAULT_MODE];
    mode_number = DEFAULT_MODE;
  }
  else {
    ACTUAL_MODE = mode_structure_array[intMode];
    mode_number = intMode;
    if ( mode_number == ARM_OUT_MODE) {
      armState = OUTSIDE;
/*       COLLI_GoBackward(); */
    }
  }

  fprintf(stderr, "\n\nmode: %d\n", intMode);
 
  fprintf (stderr, "tv      : %f  ", (ACTUAL_MODE->target_max_trans_speed));
  fprintf (stderr, "ta      : %f\n", (ACTUAL_MODE->target_trans_acceleration));
  fprintf (stderr, "rv      : %f  ", (RAD_TO_DEG(ACTUAL_MODE->target_max_rot_speed)));
  fprintf (stderr, "ra      : %f\n", (RAD_TO_DEG(ACTUAL_MODE->target_rot_acceleration)));
  fprintf (stderr, "velocity: %f \n", (ACTUAL_MODE->velocity_factor));
  fprintf (stderr, "angle   : %f\n", (ACTUAL_MODE->angle_factor));
  fprintf (stderr, "distance: %f\n", (ACTUAL_MODE->distance_factor));

  /* Set the corresponding values. */
  BASE_TranslateVelocity(             ACTUAL_MODE->target_max_trans_speed);
  BASE_RotateVelocity( RAD_TO_DEG(    ACTUAL_MODE->target_max_rot_speed));
  BASE_TranslateAcceleration(         ACTUAL_MODE->target_trans_acceleration);
  BASE_RotateAcceleration(RAD_TO_DEG( ACTUAL_MODE->target_rot_acceleration));
}


/************************************************************************
 * Sets all modes to the default value.
************************************************************************/
void
setAllModesToDefault()
{
    int i;

    for (i = 0; i < NUMBER_OF_MODES; i++) 
	mode_structure_array[i]         = default_mode_structure();
}


    

/************************************************************************
 * Copies the value of the default mode into all other modes. To do this
 * the DEFAULT_MODE must be allocated. Memory for the other modes will
 * be allocated in this function.
 ************************************************************************/
void
copyDefaultMode()
{
    int i;
    mode_structure* mode = mode_structure_array[DEFAULT_MODE];
    
    /* First set the default values. */
    VELOCITY_FACTOR               = mode->velocity_factor;
    ANGLE_FACTOR                  = mode->angle_factor;
    DISTANCE_FACTOR               = mode->distance_factor;

    NUMBER_OF_RVELS               = mode->number_of_rvels;
    NUMBER_OF_TVELS               = mode->number_of_tvels;

    TARGET_TRANS_ACCELERATION     = mode->target_trans_acceleration;
    TARGET_ROT_ACCELERATION       = mode->target_rot_acceleration;
    TARGET_MAX_ROT_SPEED          = mode->target_max_rot_speed;
    TARGET_MAX_TRANS_SPEED        = mode->target_max_trans_speed;

    EXCEPTION_TRANS_ACCELERATION  = mode->exception_trans_acceleration;
    EXCEPTION_TRANS_VELOCITY      = mode->exception_trans_velocity;
    EXCEPTION_ROT_ACCELERATION    = mode->exception_rot_acceleration;
    EXCEPTION_ROT_VELOCITY        = mode->exception_rot_velocity;

    MIN_DIST                      = mode->min_dist;
    SMOOTH_WIDTH                  = mode->smooth_width;
    SECURITY_DIST                 = mode->security_dist;;
    MIN_DIST_FOR_TARGET_WAY_FREE  = mode->min_dist_for_target_way_free;
    MAX_COLLISION_LINE_LENGTH     = mode->max_collision_line_length;
    MAX_RANGE                     = mode->max_range;

    EDGE_PORTION                  = mode->edge_portion;
    MAX_SECURITY_SPEED            = mode->max_security_speed;
    MIN_SECURITY_SPEED            = mode->min_security_speed;
    MAX_SECURITY_DIST             = mode->max_security_dist;

    SECURITY_ANGLE                = mode->security_angle;
    ROTATE_AWAY_PERSISTENCE       = mode->rotate_away_persistence;
    
    /* Now we can allocate memory for the other modes and initialize them
     * with the default values.
     */
    for (i = 0; i < NUMBER_OF_MODES; i++)
	if ( i != DEFAULT_MODE)
	    *(mode_structure_array[i])         = *mode;
}




/************************************************************************
 * Returns a structure for a mode initialized with the default values. *
 ************************************************************************/
mode_structure*
default_mode_structure(void)
{
  mode_structure *mode;

  mode = (mode_structure *) malloc (sizeof(mode_structure));

  mode->velocity_factor = VELOCITY_FACTOR;
  mode->angle_factor = ANGLE_FACTOR;
  mode->distance_factor = DISTANCE_FACTOR;

  mode->number_of_rvels = NUMBER_OF_RVELS;
  mode->number_of_tvels = NUMBER_OF_TVELS;

  mode->target_trans_acceleration = TARGET_TRANS_ACCELERATION;
  mode->target_rot_acceleration = TARGET_ROT_ACCELERATION;
  mode->target_max_rot_speed = TARGET_MAX_ROT_SPEED;
  mode->target_max_trans_speed = TARGET_MAX_TRANS_SPEED;

  mode->exception_trans_acceleration = EXCEPTION_TRANS_ACCELERATION;
  mode->exception_trans_velocity = EXCEPTION_TRANS_VELOCITY;
  mode->exception_rot_acceleration = EXCEPTION_ROT_ACCELERATION;
  mode->exception_rot_velocity = EXCEPTION_ROT_VELOCITY;

  mode->min_dist = MIN_DIST;
  mode->smooth_width = SMOOTH_WIDTH;
  mode->security_dist = SECURITY_DIST;;
  mode->min_dist_for_target_way_free = MIN_DIST_FOR_TARGET_WAY_FREE;
  mode->max_collision_line_length = MAX_COLLISION_LINE_LENGTH;
  mode->max_range = MAX_RANGE;

  mode->edge_portion = EDGE_PORTION;
  mode->max_security_speed = MAX_SECURITY_SPEED;
  mode->min_security_speed = MIN_SECURITY_SPEED;
  mode->max_security_dist = MAX_SECURITY_DIST;

  /* ARM PRAKTIKUM */
  mode->security_angle = SECURITY_ANGLE;
  mode->rotate_away_persistence = ROTATE_AWAY_PERSISTENCE;
  
  return mode;
}






/**********************************************************************/
void COLLI_get_parameters(COLLI_parameter_ptr parameters)
{
  
  if (dumpInfo)
    fprintf( dumpFile, "Received new parameters: VELOCITY_FACTOR : %f   ANGLE_FACTOR: %f\n",
	    parameters->velocity_factor, parameters->angle_factor);
  fprintf(stderr, "Received new parameters: VELOCITY_FACTOR : %f   ANGLE_FACTOR: %f\n",
	  parameters->velocity_factor, parameters->angle_factor);
  
  if (parameters->velocity_factor >= 0.0) 
    ACTUAL_MODE->velocity_factor = parameters->velocity_factor;
  if (parameters->angle_factor >= 0.0) 
    ACTUAL_MODE->angle_factor = parameters->angle_factor;
  if (parameters->distance_factor >= 0.0) 
    ACTUAL_MODE->distance_factor = parameters->angle_factor;
  if (parameters->target_max_trans_speed >= 0.0) {
    ACTUAL_MODE->target_max_trans_speed = parameters->target_max_trans_speed;
    BASE_TranslateVelocity(ACTUAL_MODE->target_max_trans_speed);
  }
  if (parameters->target_max_rot_speed >= 0.0)  {
    ACTUAL_MODE->target_max_rot_speed =  DEG_TO_RAD(parameters->target_max_rot_speed);
    BASE_RotateVelocity(RAD_TO_DEG(ACTUAL_MODE->target_max_rot_speed));
  }
  if (parameters->target_trans_acceleration >= 0.0)  {
    ACTUAL_MODE->target_trans_acceleration = parameters->target_trans_acceleration;
    BASE_TranslateCollisionAcceleration(ACTUAL_MODE->target_trans_acceleration);
  }
  if (parameters->target_rot_acceleration >= 0.0) {
    ACTUAL_MODE->target_rot_acceleration = DEG_TO_RAD(parameters->target_rot_acceleration);
    BASE_RotateCollisionAcceleration(RAD_TO_DEG(ACTUAL_MODE->target_rot_acceleration));
  } 
}





















