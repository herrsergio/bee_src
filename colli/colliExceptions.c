
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

/* These values determine the maximum velocities during 
   exceptions (rotate_away and achieve_distance).
   They are allowed to be above the MAX_... values. */
float EXCEPTION_TRANS_ACCELERATION = 40.0;
float EXCEPTION_TRANS_VELOCITY = 40.0;
float EXCEPTION_ROT_ACCELERATION = DEG_TO_RAD(40.0);
float EXCEPTION_ROT_VELOCITY = DEG_TO_RAD(40.0);


BOOLEAN achieveDistanceFlag = FALSE;
BOOLEAN rotateAwayFlag = FALSE;


/**********************************************************************
 **********************************************************************
 *                     Collision functions                            *
 **********************************************************************
 **********************************************************************/

/* This routine is called when the robot is too close to an obstacle
 * to translate forward.
 * In this case the robot just turns in the free direction that is closer 
 * to the target. */
static void
rotateAwayTrajectory(Point rpos, float rrot);

/* This routine is callled when the robot is too close to an obstacle.
 * In this case the robot just turns in the direction that is closer 
 * to free space than the other.*/
static void
achieveDistanceTrajectory(Point rpos, float rrot);


/**********************************************************************
 * In the random mode we prefer straight motion. 
 **********************************************************************/
BOOLEAN
straightMotionIsPossible( Point rpos, float rrot)
{
    float cDist, tDist;

#define MIN_DISTANCE_FOR_STRAIGHT_MOTION 150.0
#define MIN_DISTANCE_FOR_STRAIGHT_MOTION_WITH_ARM 150.0
    
    /* Is straight motion possible? */
    if ( actual_velocities.min_rvel > 0.001 || actual_velocities.max_rvel < 0.001)
	return FALSE;
    
    /* Is the motion allowed? */
    if ( admissible( rpos,
		     rrot, 
		     ACTUAL_MODE->target_max_trans_speed, 
		     0.0,
		     0, 0,
		     &cDist,
		     &tDist)) {
	/* Is enough space to the next obstacle? */
	if ( mode_number == RANDOM_MODE &&
	     cDist > MIN_DISTANCE_FOR_STRAIGHT_MOTION)
	    return TRUE;
	if ( mode_number == ARM_OUT_RANDOM_MODE &&
	     cDist > MIN_DISTANCE_FOR_STRAIGHT_MOTION_WITH_ARM)
	    return TRUE;
    } 
    return FALSE;
}



/**********************************************************************
 * Starts special modes to deal with exceptions. If rotation is allowed
 * we do this.
 **********************************************************************/
void
startExceptionHandling( Point rpos,
			float rrot,
			VelocityCombination bestCombination)
{
   /* Rotate away only when arm is not outside. */
   if ( bestCombination.rvel != 0.0 && armState == INSIDE) {
      /* There is at least one admissible trajectory. But the
       * robot is only allowed to rotate. */
      fprintf(stderr, "No translation allowed.\n");
      
      rotateAwayFlag = TRUE;
      newTargetPoint = FALSE;
      rotateAwayTrajectory(rpos, rrot);
   }
   else {
      fprintf( stderr, "No trajectory admissible.\n");
      if ( armState != INSIDE)
	 fprintf( stderr, "Just got in translate backwards because of arm outside.\n");
      /* No trajectory is admissible. */
      achieveDistanceFlag = TRUE;
      newTargetPoint = FALSE;
      achieveDistanceTrajectory( rpos, rrot);
   }
}

/**********************************************************************
 * The rotation or translation has to be finished.
 **********************************************************************/
BOOLEAN
keepOnWithExceptionHandling( Point rpos,
			     float rrot)
{
    if ( !achieveDistanceFlag && !rotateAwayFlag)
	return FALSE;
    
    if ( achieveDistanceFlag) {
	/* The robot has to translate backward and there has no more collision
	 * avoidance to be done.
	 */
	achieveDistanceTrajectory(rpos, rrot);
	COLLI_update_tcx_trajectory(rpos, rrot, -30.0, 0.0, 0.0);
    }
    else if (rotateAwayFlag){ 
	/* The robot has to turn and there has no more collision avoidance
	 * to be done.
	 */
	rotateAwayTrajectory(rpos, rrot);
	COLLI_update_tcx_trajectory(rpos, rrot, 0.0, 1.0, 0.0);
    }
    
    COLLI_send_colli_update();
    return TRUE;
}
    




/**********************************************************************
 * This routine is called when the robot is too close to an obstacle
 * to translate forward.
 * In this case the robot just turns in the free direction that is closer 
 * to the target.
 **********************************************************************/

void
rotateAwayTrajectory(Point rpos, float rrot)
{

  int i;
  float dist, targetAngle, targetDist, angle = DEG_180;
  float leftDist, rightDist, securityDist;
  static BOOLEAN first_time=TRUE;
  static int cnt = 0;

  if ( armState != INSIDE) {
    fprintf( stderr, "Can't rotate away when arm is out.\n");
    return;
  }

  if (first_time) {

    if (dumpInfo)
      fprintf( dumpFile, "Rotate away");
    fprintf(stderr, "Rotate away");
    
    securityDist = 2.0 * ACTUAL_MODE->min_dist;
    
    targetAngle = compute_angle_2p(rpos, target);
    
    dist = computeCollisionDistance( rpos, targetAngle,
				     ACTUAL_MODE->target_max_trans_speed,
				     0.0, 
				     ROB_RADIUS+ACTUAL_MODE->security_dist,
				     0.0,
				     0.0,
				     &targetDist);
    
    if (dist < securityDist) {
      
      /* The direct way to the target is not free so we have to look
       * for the best direction. 
       */

      for (i=10; i<180 && dist<securityDist; i+=10) {

	leftDist = computeCollisionDistance(
					    rpos, 
					    targetAngle + DEG_TO_RAD((float)i),
					    ACTUAL_MODE->target_max_trans_speed,
					    0.0, 
					    ROB_RADIUS+ACTUAL_MODE->security_dist,
					    0.0,
					    0.0,
					    &targetDist);
	
	rightDist = computeCollisionDistance(
					     rpos,
					     targetAngle - DEG_TO_RAD((float)i),
					     ACTUAL_MODE->target_max_trans_speed,
					     0.0, 
					     ROB_RADIUS+ACTUAL_MODE->security_dist,
					     0.0,
					     0.0,
					     &targetDist);
	
	if ( leftDist > securityDist || rightDist > securityDist) {
	  if ( leftDist > rightDist) {
	    dist = leftDist;
	    angle = targetAngle + DEG_TO_RAD((float) i) + DEG_TO_RAD(10.0);
	    if (0) fprintf(stderr, "\nlefttar %f  --> ", RAD_TO_DEG( rrot - targetAngle));
	  }
	  else {
	    dist = rightDist;
	    angle = targetAngle - DEG_TO_RAD((float) i) - DEG_TO_RAD(10.0);
	    if (0) fprintf(stderr, "\nrighttar %f  --> ", RAD_TO_DEG( rrot - targetAngle));
	  }
	  angle = normed_angle( rrot - angle);
	  if (0) fprintf(stderr, "%f\n", RAD_TO_DEG(angle));
	}
      }      
      if ( angle == DEG_180)
	fprintf( stderr, "nothing found\n");
    }
    else {
      angle = normed_angle( rrot - targetAngle);
    }
    
    /* Choose the shortest direction. */    
    if (angle > DEG_180) 
      angle -= DEG_360;

    if (0) fprintf(stderr, "finally tar %f  --> %f\n", RAD_TO_DEG( rrot - targetAngle), RAD_TO_DEG(angle));
    
    BASE_TranslateCollisionAcceleration(ACTUAL_MODE->exception_trans_acceleration);
    BASE_RotateCollisionAcceleration( RAD_TO_DEG( ACTUAL_MODE->exception_rot_acceleration));
    BASE_TranslateHalt();
    BASE_RotateHalt();

    /* In the stop commands the target flag is set to FALSE. 
     * We still want to be in the target mode.
     */
    target_flag = TRUE;
    BASE_TranslateVelocity (0.0);
    BASE_RotateVelocity ( RAD_TO_DEG( ACTUAL_MODE->exception_rot_velocity));
    BASE_TranslateCollisionVelocity (0.0);
    BASE_RotateCollisionVelocity ( RAD_TO_DEG( ACTUAL_MODE->exception_rot_velocity));

    BASE_Rotate((double)RAD_TO_DEG(angle));
    if (dumpInfo)
      fprintf( dumpFile, " (%f) ", RAD_TO_DEG(angle));
    fprintf(stderr, " (%f) ", RAD_TO_DEG(angle));
    first_time = FALSE;
    cnt = 0;
  }
  
  /* We have to wait until the rotation is finished. */
  else if ( ! stillInRotation()) {
    if (dumpInfo)
      fprintf( dumpFile, "done.\n");
    fprintf(stderr, "done.\n");
    rotateAwayFlag = FALSE;
    first_time = TRUE;
    BASE_TranslateCollisionAcceleration(ACTUAL_MODE->target_trans_acceleration);
    BASE_RotateCollisionAcceleration(RAD_TO_DEG(ACTUAL_MODE->target_rot_acceleration));
    BASE_TranslateCollisionVelocity (0.0);
    BASE_RotateCollisionVelocity (0.0);
  }
}

/**********************************************************************
 * This routine is callled when the robot is too close to an obstacle.
 * In this case the robot just turns in the direction that is closer 
 * to free space than the other.
 **********************************************************************/
void
achieveDistanceTrajectory(Point rpos, float rrot)
{
  
  float backupDist, moveOnDist, target_dist;
  static BOOLEAN first_time = TRUE;
  static Point start_pos;
  static int cnt;

    
  /* Maybe this was a wrong reading and the way is free again? */
  moveOnDist = computeCollisionDistance(rpos, rrot,
					ACTUAL_MODE->target_max_trans_speed, 0.0, 
					ROB_RADIUS+ACTUAL_MODE->security_dist,
					0.0,0.0,
					&target_dist);

  if ( moveOnDist > 2.0 * ACTUAL_MODE->min_dist) {
    if (dumpInfo)
      fprintf( dumpFile, "free again.\n");
    fprintf(stderr, "free again.\n");
    BASE_TranslateCollisionAcceleration(ACTUAL_MODE->target_trans_acceleration);
    BASE_RotateCollisionAcceleration(RAD_TO_DEG(ACTUAL_MODE->target_rot_acceleration));
    achieveDistanceFlag = FALSE;
    first_time = TRUE;
    return;
  }    

  /* Well there is still something in the way and we have to back up. */
  else {
    
    /* Can the robot drive backward? */
    backupDist = computeCollisionDistance(rpos, rrot+DEG_180,
					  ACTUAL_MODE->target_max_trans_speed, 0.0, 
					  ROB_RADIUS+ACTUAL_MODE->security_dist,
					  0.0,0.0,
					  &target_dist);
    
    
    if (backupDist < 2.0 * ACTUAL_MODE->security_dist) {
      
      if ( armState != INSIDE) {
	fprintf( stderr, "Normally would try to rotate but better try to translate");
	fprintf( stderr, "when arm is out.\n");
      }
      else {
	
	/* Rotation is the only thing we can do. */
	if (first_time) {
	  if (dumpInfo)
	    fprintf( dumpFile, "Can't translate backward. Try to rotate.\n");
	  fprintf(stderr, "Can't translate backward. Try to rotate.\n");
	}
	else {
	  if (dumpInfo)
	    fprintf( dumpFile, "Can't translate backward any more. Try to rotate.\n");
	  fprintf(stderr, "Can't translate backward any more. Try to rotate.\n");
	  BASE_TranslateVelocity (0.0);
	  BASE_TranslateHalt();
	  /* In the stop commands the target flag is set to FALSE. 
	   * We still want to be in the target mode.
	   */
	  target_flag = TRUE;
	}
	achieveDistanceFlag = FALSE;
	rotateAwayFlag = TRUE;
	first_time = TRUE;
	rotateAwayTrajectory(rpos, rrot);
	return;
      }
    }
    else {
      if (first_time) {
	if (dumpInfo)
	  fprintf( dumpFile, "Translate backward ");
	fprintf(stderr, "Translate backward ");
	start_pos = rpos;
	
	BASE_TranslateCollisionAcceleration( ACTUAL_MODE->exception_trans_acceleration);
	BASE_RotateCollisionAcceleration( RAD_TO_DEG( ACTUAL_MODE->exception_rot_acceleration));
	BASE_TranslateHalt();
	BASE_RotateHalt();
	
	/* In the stop commands the target flag is set to FALSE. 
	 * We still want to be in the target mode.
	 */
	target_flag = TRUE;
	BASE_TranslateVelocity (ACTUAL_MODE->exception_trans_velocity);
	BASE_RotateVelocity (0.0);
	BASE_TranslateCollisionVelocity (ACTUAL_MODE->exception_trans_velocity);
	BASE_RotateCollisionVelocity (0.0);
	
#define BACKUP_WITH_ARM_OUT 40.0
#define BACKUP 15.0
	
	if ( ! colli_go_backward) {
	  if ( armState != INSIDE)
	    BASE_Translate( - BACKUP_WITH_ARM_OUT);
	  else
	    BASE_Translate(- BACKUP);
	}
	else {
	  if ( armState != INSIDE)
	    BASE_Translate(  BACKUP);
	  else
	    BASE_Translate(  BACKUP);
	}
	first_time = FALSE;
	cnt = 0;
      }
      
      /* We have to wait until the translation is finished. */
      else if ( ! stillInTranslation()) {
	if (dumpInfo)
	  fprintf( dumpFile, "done.\n");
	fprintf(stderr, "done.\n");
	BASE_TranslateCollisionAcceleration(ACTUAL_MODE->target_trans_acceleration);
	BASE_RotateCollisionAcceleration(RAD_TO_DEG(ACTUAL_MODE->target_rot_acceleration));
	achieveDistanceFlag = FALSE;
	first_time = TRUE;
      }
    }
  }
}

