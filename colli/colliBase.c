
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

velocities actual_velocities;
BOOLEAN rot_wanted = FALSE;

/**********************************************************************
 **********************************************************************
 *       Functions especially for collision avoidance                 *
 **********************************************************************
 **********************************************************************/

/**********************************************************************
 * The actual values are set. The rotation is set such that the
 * common mathematical functions can be applied to it.
 * At the same time the velocities are updated and stored in
 * the global structure "actual_velocities".  
 **********************************************************************/
void
updateActualPosition( Point* rpos, float* rrot,
		      BOOLEAN considerDirection)
{
  rpos->x = (float) rwi_base.pos_x;
  rpos->y = (float) rwi_base.pos_y;
  
   /* If the robot should move backwards we just turn it by 180.0
    * degrees. The rest of the computation stays the same.
    */
   if ( !considerDirection || !colli_go_backward)
      *rrot   = normed_angle( (float) DEG_TO_RAD(90.0 -  rwi_base.rot_position)); 
   else
     *rrot   = normed_angle( (float) DEG_TO_RAD(270.0 -  rwi_base.rot_position));
}


/**********************************************************************
 * The actual values are set. The rotation is set such that the
 * common mathematical functions can be applied to it.
 * At the same time the velocities are updated and stored in
 * the global structure "actual_velocities".  
 **********************************************************************/
void
updateActualConfiguration( Point* rpos, float* rrot)
{
  updateActualPosition( rpos, rrot, CONSIDER_DIRECTION);
  
  compute_current_velocities();
  compute_possible_velocities( fabs( rwi_base.trans_acceleration),
			       DEG_TO_RAD( fabs( rwi_base.rot_acceleration)));
  compute_possible_trajectories();
}



/**********************************************************************
 * If the robot should move backward we have to swap the velocities.
 **********************************************************************/
void
compute_current_velocities(void)
{
  
  actual_velocities.current_tvel = (float) rwi_base.trans_current_speed; 
  actual_velocities.current_rvel = (float) DEG_TO_RAD(rwi_base.rot_current_speed);

  if (actual_velocities.current_tvel < MIN_TRANS_SPEED) {
    if (rwi_base.trans_set_direction == NEGATIVE)
      actual_velocities.current_tvel *= -1.0;
  }
  else
    if (rwi_base.trans_direction == NEGATIVE)
      actual_velocities.current_tvel *= -1.0;
  
  if (actual_velocities.current_rvel < MIN_ROT_SPEED) {
    if (rwi_base.rot_set_direction == NEGATIVE)
      actual_velocities.current_rvel *= -1.0;
  }
  else
    if (rwi_base.rot_direction == NEGATIVE)
      actual_velocities.current_rvel *= -1.0;

  /* If the robot should go backward, we change the sign of the velocity.
   * Now we can do the same computations as for the forward direction.
   */
  if ( colli_go_backward) 
      actual_velocities.current_tvel *= -1.0;
  
  return;
}



/**********************************************************************
 * Compute maximal allowed velocities.
 * The maximal velocity is set in the desired_trajectory. If the coll_dist
 * is too small to brake desired_trajectory.admissible is set to FALSE.
 **********************************************************************/
void
setVelocities(Point rpos, float coll_dist, trajectory desired_trajectory)
{
  float tvel, rvel, tmp_tvel;

  compute_max_velocity(coll_dist, &desired_trajectory);

  if ( !desired_trajectory.admissible) {
    /* trajectory not allowed ---> emergencyStop */
    tvel = 0.0;  
    rvel = 0.0;  
    if (!emergencyStop) {
      BASE_TranslateCollisionAcceleration(50.0);
      emergencyStop = TRUE;
      if (dumpInfo)
	fprintf( dumpFile, "Oops!\n");
      fprintf( stderr, "Oops!\n");
    }
  }
  
  else {
    if (target_flag) {

      /* Only positive tvel. 0.8 because the robot cannot stop immediately. */
      tmp_tvel = 0.8 * fsqrt( SQR( targetArriveSpeed) +
		       2.0 * ACTUAL_MODE->target_trans_acceleration *
		       compute_distance(rpos, target));

      /* The targetArriveSpeed depends on the position of the nextTarget. */
      desired_trajectory.max_tvel = MIN(desired_trajectory.max_tvel, tmp_tvel);
    }
    
    if ( emergencyStop && desired_trajectory.tvel != 0.0) {
      BASE_TranslateCollisionAcceleration(ACTUAL_MODE->target_trans_acceleration);
      emergencyStop = FALSE;
      if (dumpInfo)
	fprintf( dumpFile, "ok\n");
      fprintf(stderr, "ok\n");
    }
        
    if ( desired_trajectory.tvel > 0.0) 
      tvel = MIN(desired_trajectory.tvel, desired_trajectory.max_tvel);
    else 
      tvel = MAX(desired_trajectory.tvel, desired_trajectory.min_tvel);
    
    
    if ( fabs(desired_trajectory.tvel) > EPSILON){
      rvel = RAD_TO_DEG(desired_trajectory.rvel * tvel / desired_trajectory.tvel); 
    }
    else {
      rvel = RAD_TO_DEG(desired_trajectory.rvel);
    }
  }
  

  /* We have to change the direction of the translational movement
   * according to colli_go_backward.
   */
  if ( !colli_go_backward)
      rwi_base.trans_set_direction = (tvel >= 0.0) ? 
	  POSITIVE : NEGATIVE;
  else
      rwi_base.trans_set_direction = (tvel < 0.0) ? 
	  POSITIVE : NEGATIVE;
  
  rwi_base.rot_set_direction = (rvel >= 0.0) ? 
      POSITIVE : NEGATIVE;

  BASE_TranslateCollisionVelocity ((double) fabs(tvel));
  BASE_RotateCollisionVelocity ((double) fabs(rvel));
}	




/**********************************************************************
 * We have to change the direction of the translational movement
 * according to colli_go_backward.
 **********************************************************************/
void
setDirections( trajectory desiredTrajectory)
{
  if ( colli_go_backward) {
    if (desiredTrajectory.tvel < 0.0)
      BASE_TranslateForward();
    else
      BASE_TranslateBackward();
  }
  else {
    if (desiredTrajectory.tvel < 0.0)
      BASE_TranslateBackward();
    else
      BASE_TranslateForward();
  }
  if (desiredTrajectory.rvel > 0.0)
    BASE_RotateClockwise();
  else
    BASE_RotateAnticlockwise();
}




