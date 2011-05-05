
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

float targetArriveSpeed = 0.0;
float approachDist = 24.8;
BOOLEAN target_flag = FALSE;
BOOLEAN newTargetPoint = FALSE;
BOOLEAN driveThroughPoint = FALSE;
Point nextTarget;
Point target;

/* If this flag is set the robot will use the sequences given in
 * colliPickup. */
BOOLEAN pickupObjects = TRUE;

/* From this distance the vision tries to get the exact position of the
 * object. */
const float distForApproachObject = 150.0;
/* From this distance the robot tries to pick up an object. */
const float distForTrashBin = 80.0;

/**********************************************************************
 **********************************************************************
 *                     Collision functions                            *
 **********************************************************************
 **********************************************************************/

static float
maxTargetArriveSpeed(Point rpos, Point target, Point nextTarget);

static void
checkIfDriveThroughPoint();


/**********************************************************************
 * Sets the approach distance used for the next call of
 * COLLI_TranslateRelative().
 **********************************************************************/
void
COLLI_SetApproachDist( double dist)
{
  approachDist = dist;
}




/**********************************************************************
 * Set the next target point
 **********************************************************************/
void
COLLI_TranslateRelative(double forward, double side)
{
  Point rpos; 
  float rrot;
  
  newTargetPoint = TRUE;  
  

  /* SOUND_talk_text("see you there."); */

  if (dumpInfo)
    fprintf( dumpFile, "See you there.\n");
  fprintf(stderr, "See you there.\n");
  rwi_base.collision_state.target_reached = FALSE;
  
  rpos.x = (float) rwi_base.pos_x;
  rpos.y = (float) rwi_base.pos_y;
  rrot   = (float) DEG_TO_RAD(90.0 - rwi_base.rot_position);
  
  norm_angle(&rrot);
  
  target_flag = TRUE;
  
  target.x = (rpos.x + fcos(rrot) * (float) forward + fsin(rrot) * (float) side);
  target.y = (rpos.y + fsin(rrot) * (float) forward - fcos(rrot) * (float) side);

  approachDist = 2.0 * ROB_RADIUS;
  driveThroughPoint = FALSE;
}


/**********************************************************************/
void
COLLI_GotoAbsolute( float x, float y, BOOLEAN new_target)
{
  COLLI_ApproachAbsolute_TwoPoints( x, y,
				    x, y,
				    2.0 * ROB_RADIUS,
				    new_target,
				    NULL);
}


/**********************************************************************
 * Sets the target point on x, y. The mode determines the behaviour of 
 * the robot on it's way to this point. If the distance to the target
 * is less than dist the robot stops and sets target_reached to TRUE.
 **********************************************************************/
void
COLLI_ApproachAbsolute( float x, float y, float dist,
			BOOLEAN new_target, int mode)
{
  if ( mode_number != mode) 
    COLLI_SetMode( (double) mode);
  
  COLLI_ApproachAbsolute_TwoPoints(x, y,
				   x, y,
				   dist,
				   new_target,
				   NULL);
}


/**********************************************************************
 * Sets the target point on x, y. The mode determines the behaviour of 
 * the robot on it's way to this point. If the distance to the target
 * is less than dist the robot stops and sets target_reached to TRUE.
 **********************************************************************/
void
COLLI_ApproachAbsolute_TwoPoints( float x1, float y1, float x2, float y2,
				  float dist, BOOLEAN new_target ,
				  TCX_REF_PTR TCX_sender)
{
  Point rpos;
  rpos.x = (float) rwi_base.pos_x;
  rpos.y = (float) rwi_base.pos_y;
  
  TCX_sender_of_action = TCX_sender;

  /* The robot will try to rotate to a new target before moving towards it. */
  newTargetPoint = ( ! target_flag || new_target);
  target_flag = TRUE;
  
  approachDist = dist;

  if ( newTargetPoint) {

    /* It is the first or a new pair of target points. We store the points
     * in the corresponding variables.
     */
    
    rwi_base.collision_state.target_reached = FALSE;
    emergencyStop = FALSE;
    target.x = x1;
    target.y = y1;

    /* To force the robot to move towards the target. */
    approachDist = MIN( approachDist,
			compute_distance( rpos, target) / 2.0);
    
    nextTarget.x = x2;
    nextTarget.y = y2;
    
    /* How fast shall the robot approach the first target point. */
    targetArriveSpeed = maxTargetArriveSpeed(rpos, target, nextTarget);
    
#ifdef BASE_debug
    fprintf(stderr,
	    "Got new target points at %f %f and %f %f. Approach dist is %f.\n",
	    x1, y1, x2, y2, approachDist);
    fprintf(stderr, "arrive: %f\n", targetArriveSpeed);
#endif
    if (dumpInfo) {
      fprintf( dumpFile,
	       "Got new target points at %f %f and %f %f. Approach dist is %f.\n",
	       x1, y1, x2, y2, approachDist);
      fprintf( dumpFile, "arrive: %f\n", targetArriveSpeed);
    }
  }
  else if ( rwi_base.collision_state.target_reached) {
    
    /* This is just the correction of an old target pair. But we have reached the
     * first target and use the nextTarget until the planner sends a new target.
     */
    target.x = nextTarget.x = x2;
    target.y = nextTarget.y = y2;
    targetArriveSpeed = 0.0;
    
    /* If this double target was not a drive through point the second
     * target will be handled like a new target point. */
    newTargetPoint = driveThroughPoint ? FALSE : TRUE;
    
    fprintf(stderr, "Reached the first target. Use the second one.\n");
  }
  else {
    
     /* Not much to do. */
     target.x = x1;
     target.y = y1;
     
     nextTarget.x = x2;
     nextTarget.y = y2;
     
     if (dumpInfo) {
	fprintf( dumpFile, "Correction of old target points ");
	fprintf( dumpFile, "(%f %f) (%f %f)\n", x1, y1, x2, y2);
	fprintf(stderr, "Correction of old target points ");
	fprintf(stderr, "(%f %f) (%f %f)\n", x1, y1, x2, y2);
     }
  }

  /* If the target arrive speed is big enough the pair of targets
   * is called a drive through point. */
  checkIfDriveThroughPoint();
}


/**********************************************************************
 * The maximal velocity at the arrival at the target point. It is determined
 * by the relative position of the nextTarget to the target and the robot
 * position.
 **********************************************************************/
static float
maxTargetArriveSpeed(Point rpos, Point target, Point nextTarget)
{
  float angle, distance, factor, max_brake_vel;
  
  distance = compute_distance(target, nextTarget);
  
  if (distance < 20.0) {
    return(0.0);
  }
  
  else {
    
    /* We want to be sure that the robot can stop at the nextTarget. */
    max_brake_vel = maxVelForEmergencyStop( distance,
					   ACTUAL_MODE->target_trans_acceleration);
    
    angle = compute_angle_3p(target, rpos, nextTarget);

    if (angle > DEG_180)
      angle = DEG_360 - angle;
    
    /* Now we have the angle the robot has to rotate after having reached the 
     * the first taget. If this angle is bigger than 90 degrees the robot
     * has to stop.
     */
    
    factor = MAX( 0.0, (angle-DEG_90) / DEG_90);
    
    return(SQR(factor) * ACTUAL_MODE->target_max_trans_speed);
  }
}


/**********************************************************************
 * If <newTargetPoint> is TRUE the robot will try to rotate to
 * the target until the conditions are not met any longer.
 **********************************************************************/
BOOLEAN
checkIfRotateToTargetPoint( float targetRot)
{
  /* If these conditions are met the robot will use a special
   * evaluation function which emphasizes the importance of
   * the target angle.
   */
    static BOOLEAN lastCycleWasRotatePoint = FALSE;
    
    static float headingToTarget = DEG_TO_RAD(60.0);

    BOOLEAN rotateToPoint = FALSE;
    
    /* If it is a new target point or the robot already rotates to the
     * actual target point we check the angle. */
    if ( newTargetPoint || lastCycleWasRotatePoint) {

	rotateToPoint =
	    /* The angle to the target has to be big enough. */
	    ( fabs( targetRot) > headingToTarget &&
	      /* In the RANDOM_MODE the target is not interesting. */
	      mode_number != RANDOM_MODE &&
	      /* In the RANDOM_MODE the target is not interesting. */
	      mode_number != ARM_OUT_RANDOM_MODE);
    }

    if ( rotateToPoint) {
	if ( ! lastCycleWasRotatePoint) {
	    fprintf( stderr, "Turn to the target ... ");
	    lastCycleWasRotatePoint = TRUE;
	}
    }
    else if ( lastCycleWasRotatePoint) {
	lastCycleWasRotatePoint = FALSE;
	fprintf( stderr, "... finished.\n");
    }

    newTargetPoint = FALSE;

    return lastCycleWasRotatePoint;
}


/**********************************************************************
 * Checks wether the robot has to stop when it reaches the actual
 * target point or wether it has to use the next target point.
 **********************************************************************/
void
checkIfDriveThroughPoint()
{
   if ( targetArriveSpeed > 0.0)
     driveThroughPoint = TRUE;
   else
     driveThroughPoint = FALSE;
}


/**********************************************************************
 * What to do when the robot has reached a target depends on the
 * current mode.
 * The function returns TRUE if this cycle of the collision avoidance
 * should be ended.
 * If we use a new target we adapt <targetDist> and <targetRot>.
 **********************************************************************/
BOOLEAN
handleArrivalAtTarget( Point rpos,
		       float rrot,
		       float* targetDist,
		       float* targetRot)
{     

  if ( *targetDist < fabs( approachDist)) {

    if (! rwi_base.collision_state.target_reached) {
      if (dumpInfo)
	fprintf( dumpFile, "I am %f cm away from the target (approach dist: %f)!\n",
		 *targetDist, approachDist);
      fprintf(stderr, "I am %f cm away from the target (approach dist: %f)!\n",
	      *targetDist, approachDist);
      
      /* SOUND_talk_text("I am there"); */
    }
    
    /*---------------------------------------------------------------*
     * Now we have to check what to do according to the actual mode. *
     *---------------------------------------------------------------*/
    if ( mode_number == APPROACH_OBJECT_MODE ||
	 mode_number == APPROACH_TRASH_BIN_MODE) {
      
      BASE_TranslateHalt();
      BASE_RotateHalt();
      target_flag = FALSE;
      rwi_base.collision_state.target_reached = TRUE;
      return TRUE;
    }
      
    else {

      /* Inform the other modules that the robot reached the target. */
      send_automatic_action_executed_message();
      
      /* We have just reached a temporary target and try to get to the
       * next point.
       */
      if ( driveThroughPoint) {

	fprintf( stderr, "Reached drive through point.\n");
	if ( dumpInfo) {
	  fprintf( dumpFile, "Reached drive through point.\n");
	}
	/* The state is target_reached. */
	rwi_base.collision_state.target_reached = TRUE;
	
	/* We use the next target and have to recompute the values. */
	target.x = nextTarget.x;
	target.y = nextTarget.y;
	*targetDist = compute_distance(rpos, target);
	*targetRot = compute_robot_angle_to_point(rpos, rrot, target);
	
	return FALSE;
      }
      
      /* This is the normal thing to do. */
      else {
	
	if ( dumpInfo) {
	  fprintf( dumpFile, "Reached target point.\n");
	}

	target_flag = FALSE;
	BASE_TranslateHalt();
	BASE_RotateHalt();
	
	/* Let's inform the planner that we need a new target point. */
	if ( ! rwi_base.collision_state.target_reached && (TCX_sender_of_action != NULL)) {
	  BASE_action_executed_reply_type state; 
	  state.x = (float) rwi_base.pos_x;
	  state.y = (float) rwi_base.pos_y;
	  state.orientation = (float) rwi_base.rot_position;
	  fprintf(stderr, "Send executed message.\n");
	  tcxReply(TCX_sender_of_action, "BASE_action_executed_reply", &state);
	  TCX_sender_of_action = NULL;
	}

	rwi_base.collision_state.target_reached = TRUE;

	return TRUE;
      }
    }
  }
  return FALSE;
}
   


