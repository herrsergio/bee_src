
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


/**********************************************************************
 **********************************************************************
 *               Functions for data transfer via TCX                  *
 **********************************************************************
 **********************************************************************/

void
initTcxStructures()
{
    
  /* These values are only set once and must no be changed. */
  colli_tcx_status.rememberInterval = REMEMBER_INTERVAL;
  
  /* Initialize the vision lines and points */
  colli_tcx_status.no_of_external_points = 0;
  colli_tcx_status.external_points = NULL;
  
  colli_tcx_status.no_of_bumper_points = 0;
  colli_tcx_status.bumper_points = NULL;
  
  colli_tcx_status.no_of_ir_points = 0;
  colli_tcx_status.ir_points = NULL;
  
  /* Initialize the laser points */
  colli_tcx_status.no_of_laser_points = 0;
  colli_tcx_status.laser_points = NULL;
}

/**********************************************************************
 * Updates the values in colli_tcx_status. These values are shown in the 
 * trajectory of the robot in the collision graphics.
**********************************************************************/
void
COLLI_update_tcx_trajectory(Point rpos, float rrot,
			    float tvel, float rvel,
			    float dist)
{
  
  if ( n_auto_colli_update_modules > 0) {

      BOOLEAN lineFlag = FALSE;

      float addsize,armsize;
      float size;
      Point tmp;
      
      colli_tcx_status.rpos.x = casted_float(rpos.x);
      colli_tcx_status.rpos.y = casted_float(rpos.y);
      colli_tcx_status.rrot   = casted_float(RAD_TO_DEG(rrot));
      colli_tcx_status.tvel   = casted_float(tvel);
      colli_tcx_status.rvel   = casted_float(RAD_TO_DEG(rvel));
      colli_tcx_status.dist   = casted_float(dist);
      
      size = ROB_RADIUS + ACTUAL_MODE->security_dist;
      
      if ( armState != INSIDE)
	  armsize = computeArmOffset( tvel, rvel);
      else 
	  armsize = 0.0;
  
      addsize = speed_dependent_security_dist(tvel);

      
      if (target_flag) {
	  colli_tcx_status.targetpoint.x = casted_float(target.x); 
	  colli_tcx_status.targetpoint.y = casted_float(target.y); 
      }  
      else
	  colli_tcx_status.targetpoint.x = I_ERROR;
      
      /* Check for the kind of the trajectory and store the values
       * in the corresponding integers. */
      
      if (fabs(rvel) < EPSILON) 
	  lineFlag = TRUE;
      else if (fabs(tvel/rvel) > MAX_CURVE_RADIUS) 
	  lineFlag = TRUE;
      
      if ( lineFlag) {
	  LineSeg lline, rline;                    /* Collision area (line)    */
	  position_to_lines(rpos, rrot, tvel, size+0.5*addsize, &lline, &rline); 
	  
	  colli_tcx_status.leftLine.pt1.x = casted_float(lline.pt1.x); 
	  colli_tcx_status.leftLine.pt1.y = casted_float(lline.pt1.y); 
	  colli_tcx_status.leftLine.pt2.x = casted_float(lline.pt2.x); 
	  colli_tcx_status.leftLine.pt2.y = casted_float(lline.pt2.y); 
	  
	  colli_tcx_status.rightLine.pt1.x = casted_float(rline.pt1.x); 
	  colli_tcx_status.rightLine.pt1.y = casted_float(rline.pt1.y); 
	  colli_tcx_status.rightLine.pt2.x = casted_float(rline.pt2.x); 
	  colli_tcx_status.rightLine.pt2.y = casted_float(rline.pt2.y); 
	  
	  colli_tcx_status.innerCircle.M.x = I_ERROR;
	  colli_tcx_status.outerCircle.M.x = I_ERROR;
      }
      else {
	 Circle_trajectory circ =
	    velocities_to_circle_trajectory( rpos, rrot, tvel, rvel, size, addsize,armsize);
	 
	 colli_tcx_status.outerCircle.rad = casted_float( circ.big_rad);
	 colli_tcx_status.outerCircle.M.x = casted_float( circ.M.x);
	 colli_tcx_status.outerCircle.M.y = casted_float( circ.M.y);
	 
	 colli_tcx_status.innerCircle.rad = casted_float( circ.small_rad);
	 colli_tcx_status.innerCircle.M.x = casted_float( circ.M.x);
	 colli_tcx_status.innerCircle.M.y = casted_float( circ.M.y);

	 if ( armState != INSIDE) {
	    colli_tcx_status.armCircle.rad = casted_float( circ.arm_rad);
	    colli_tcx_status.armCircle.M.x = casted_float( circ.M.x);
	    colli_tcx_status.armCircle.M.y = casted_float( circ.M.y);
	    tmp = computeInnerArmPoint(rpos,rrot,rvel,tvel);
	    colli_tcx_status.innerArmPoint.x = casted_float(tmp.x);
	    colli_tcx_status.innerArmPoint.y = casted_float(tmp.y);
	    tmp = computeOuterArmPoint(rpos,rrot,rvel,tvel);
	    colli_tcx_status.outerArmPoint.x = casted_float(tmp.x);
	    colli_tcx_status.outerArmPoint.y = casted_float(tmp.y);
	 }

      }
  }
}



/**********************************************************************
 * Updates the values in colli_tcx_status. These lines are shown in the 
 * collision graphics in red.
**********************************************************************/
void
COLLI_update_tcx_CollLines(void)
{
  int i;
  LineSeg tmp;

  if ( n_auto_colli_update_modules > 0) {
    for (i = 0; i < bRobot.sonar_cols[0]; i++) {
      if ( use_sonar) {
	tmp = CollLines[i][next_CollLine_reading]; 
	(colli_tcx_status.sonar_lines)[i].pt1.x = casted_float(tmp.pt1.x); 
	(colli_tcx_status.sonar_lines)[i].pt1.y = casted_float(tmp.pt1.y); 
	(colli_tcx_status.sonar_lines)[i].pt2.x = casted_float(tmp.pt2.x); 
	(colli_tcx_status.sonar_lines)[i].pt2.y = casted_float(tmp.pt2.y); 
      }
      else
	colli_tcx_status.sonar_lines[i].pt1.x = I_ERROR;
    }
  }
}

/**********************************************************************
 * Updates the values in colli_tcx_status. These lines are shown in the 
 * collision graphics in red.
**********************************************************************/
void
COLLI_update_tcx_LaserPoints(void)
{
  int i;

  if ( n_auto_colli_update_modules > 0) {

    /* If the number of points has changed we allocate new memory. */
    if ( colli_tcx_status.no_of_laser_points != Laser_CollPoints.numberOfPoints) {

      if ( colli_tcx_status.no_of_laser_points > 0)
	free (colli_tcx_status.laser_points);
      
      colli_tcx_status.no_of_laser_points = Laser_CollPoints.numberOfPoints;

      if (colli_tcx_status.no_of_laser_points > 0) 
	colli_tcx_status.laser_points = (iPoint *) 
	  malloc( colli_tcx_status.no_of_laser_points * sizeof(struct iPoint));
      else
	colli_tcx_status.laser_points = (iPoint *) NULL; 
    }

    for (i=0; i < colli_tcx_status.no_of_laser_points; i++) {
      (colli_tcx_status.laser_points)[i].x =
	casted_float(Laser_CollPoints.points[i].x); 
      (colli_tcx_status.laser_points)[i].y =
	casted_float(Laser_CollPoints.points[i].y); 
    }
  }
}


/**********************************************************************
 * Updates the values in colli_tcx_status. These points are shown in the 
 * collision graphics in blue.
**********************************************************************/
void
COLLI_update_tcx_ExternalObstaclePoints(void)
{
  int i;
  Point tmp;

  
  if (n_auto_colli_update_modules > 0) {
    if (colli_tcx_status.no_of_external_points > 0) 
      free (colli_tcx_status.external_points);
    
    colli_tcx_status.no_of_external_points = External_Obstacle_Points.no_of_points;

    if (colli_tcx_status.no_of_external_points > 0) {
      
      colli_tcx_status.external_points = (iPoint *) 
	malloc( colli_tcx_status.no_of_external_points * sizeof(struct iPoint));
      
      for (i=0; i<colli_tcx_status.no_of_external_points; i++) {
 	tmp = External_Obstacle_Points.points[i];
	(colli_tcx_status.external_points)[i].x = casted_float(tmp.x); 
	(colli_tcx_status.external_points)[i].y = casted_float(tmp.y); 
      }
    }
    else 
      colli_tcx_status.external_points = NULL;
  }
  else {
    colli_tcx_status.no_of_external_points = 0; 
    colli_tcx_status.external_points = NULL;
  }
}


/**********************************************************************
 * Updates the values in colli_tcx_status. These points are shown in the 
 * collision graphics in blue.
**********************************************************************/
void
COLLI_update_tcx_BumperPoints(void)
{
  int i;
  Point tmp;

  if (n_auto_colli_update_modules > 0) {
    if (colli_tcx_status.no_of_bumper_points > 0) 
      free (colli_tcx_status.bumper_points);
    
    colli_tcx_status.no_of_bumper_points = Bumper_Obstacle_Points.no_of_points;

    if (colli_tcx_status.no_of_bumper_points > 0) {
     
      colli_tcx_status.bumper_points = (iPoint *) 
	malloc( colli_tcx_status.no_of_bumper_points * sizeof(struct iPoint));
      
      for (i=0; i<colli_tcx_status.no_of_bumper_points; i++) {
 	tmp = Bumper_Obstacle_Points.points[i];
	(colli_tcx_status.bumper_points)[i].x = casted_float(tmp.x); 
	(colli_tcx_status.bumper_points)[i].y = casted_float(tmp.y); 
      }
    }
    else 
      colli_tcx_status.bumper_points = NULL;
  }
  else {
    colli_tcx_status.no_of_bumper_points = 0; 
    colli_tcx_status.bumper_points = NULL;
  }
}


/* Updates the values in colli_tcx_status. These points are shown in the 
 * collision graphics in green. */
void
COLLI_update_tcx_IrPoints(void)
{
  int i;
  Point tmp;

  if (n_auto_colli_update_modules > 0) {

    if (colli_tcx_status.no_of_ir_points > 0) 
      free (colli_tcx_status.ir_points);
    
    colli_tcx_status.no_of_ir_points = Ir_Obstacle_Points.no_of_points;

    if (colli_tcx_status.no_of_ir_points > 0) {
      
      colli_tcx_status.ir_points = (iPoint *) 
	malloc( colli_tcx_status.no_of_ir_points * sizeof(struct iPoint));
      
      for (i=0; i<colli_tcx_status.no_of_ir_points; i++) {
 	tmp = Ir_Obstacle_Points.points[i];
	(colli_tcx_status.ir_points)[i].x = casted_float(tmp.x); 
	(colli_tcx_status.ir_points)[i].y = casted_float(tmp.y); 
      }
    }
    else 
      colli_tcx_status.ir_points = NULL;
  }
  else {
    colli_tcx_status.no_of_ir_points = 0; 
    colli_tcx_status.ir_points = NULL;
  }
}

