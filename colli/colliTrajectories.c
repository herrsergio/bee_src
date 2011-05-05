
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

BOOLEAN use_external = TRUE;

/**********************************************************************
 **********************************************************************
 *                         Help functions                            *
 **********************************************************************
 **********************************************************************/


/**********************************************************************
 * The lines do always lead from the robot to MAX_RANGE              *
 **********************************************************************/
void
position_to_lines(Point rpos, float rrot, float tvel, float size,
		  LineSeg *lline, LineSeg *rline)
{
  float dir;
  float x_offset, y_offset;
  float lineLength = MAX_RANGE + ROB_RADIUS;
  
  dir = (tvel >= 0.0) ? 1.0 : -1.0;
  
  x_offset = fcos( rrot+DEG_90) * 2.0 * size;
  y_offset = fsin( rrot+DEG_90) * 2.0 * size;
  
  lline->pt1.x = rpos.x + dir*(fcos( rrot+DEG_90) * size);
  lline->pt1.y = rpos.y + dir*(fsin( rrot+DEG_90) * size);
  
  lline->pt2.x = lline->pt1.x + dir*(fcos( rrot) * lineLength);
  lline->pt2.y = lline->pt1.y + dir*(fsin( rrot) * lineLength);
  
  rline->pt1.x = lline->pt1.x - dir*x_offset;
  rline->pt1.y = lline->pt1.y - dir*y_offset;
  rline->pt2.x = lline->pt2.x - dir*x_offset;
  rline->pt2.y = lline->pt2.y - dir*y_offset;
}


/**********************************************************************
* this procedure computes the circle-trajectories.
************************************************************************/

Circle_trajectory
velocities_to_circle_trajectory(Point rpos, float rrot, float tvel,
				float rvel, float size, float addsize,float armsize)
{
  Circle_trajectory tmp;
  float rad_rot, rdir, tdir;
   
  if (rvel != 0.0) {

    rdir = (rvel >= 0.0) ? 1.0 : -1.0;
    tdir = (tvel >= 0.0) ? 1.0 : -1.0;
      
    rad_rot = rrot - (rdir*tdir) * DEG_90;
    tmp.middle_rad = fabs (tvel / rvel);
      
    tmp.small_rad = MAX( 0.0,
			 tmp.middle_rad - size - (ACTUAL_MODE->edge_portion) * addsize);
    tmp.big_rad =  tmp.middle_rad
      + (size + (1.0 - ACTUAL_MODE->edge_portion) * addsize);
    tmp.arm_rad =  tmp.big_rad+armsize ;

    tmp.M.x = rpos.x + fcos( rad_rot) * tmp.middle_rad;
    tmp.M.y = rpos.y + fsin( rad_rot) * tmp.middle_rad;
      
  }
  else {
    tmp.M.x = tmp.M.y = 0.0;
    tmp.middle_rad = tmp.small_rad = tmp.big_rad = tmp.arm_rad = 0.0;
  }

  return(tmp);
}


/**********************************************************************
* For a given point and ring this function determines wether the point
* is in the ring.
**********************************************************************/
static BOOLEAN
is_point_in_ring_area(Point M, float r1, float r2, Point pt)

{
  float tmp; 

  tmp = compute_distance(M, pt);
  return (tmp < r2 && tmp > r1);
}

  


/**********************************************************************
 * Given the area in front of the robot (by two lines) the function 
 * checks wether the point is in this area.
 **********************************************************************/
static BOOLEAN
is_point_in_line_area( LineSeg *lline, LineSeg *rline, Point pt)
{
  float sign;
  BOOLEAN in=TRUE;

  /* The lines are parallel and the orthogonal lines are 
   * defined by the end points.
   */

  /* (pt.x - x1) * (-y2 + y1) + (pt.y - y1) * (x2 - x1) is positive
   * if the point is on the left side of the line. */


  /* The point has to be on the left side of the right line. */
  sign = (pt.x - rline->pt1.x) * (-rline->pt2.y + rline->pt1.y) +
    (pt.y - rline->pt1.y) * (rline->pt2.x - rline->pt1.x);

  if ((in = (in && (sign >= 0.0)))) {
      
    /* The point has to be on the left side of the end line. */
    sign = (pt.x - rline->pt2.x) * (-lline->pt2.y + rline->pt2.y) + 
      (pt.y - rline->pt2.y) * (lline->pt2.x - rline->pt2.x);
    if ((in = ( in && (sign>=0.0)))) {
	  
      /* The point has to be on the left side of the left line (with swapped points). */
      sign = (pt.x - lline->pt2.x) * (-lline->pt1.y + lline->pt2.y) +
	(pt.y - lline->pt2.y) * (lline->pt1.x - lline->pt2.x);
      if ((in = (in && (sign>=0.0)))) {
	      
	/* The point has to be on the left side of the robot line. */
	sign = (pt.x - lline->pt1.x) * (-rline->pt1.y + lline->pt1.y) + 
	  (pt.y - lline->pt1.y) * (rline->pt1.x - lline->pt1.x);
	in = (in && (sign>=0.0));
      }
    }
  }
  
  return (in);
}

static float
angleToCutPoint( Point cutPointOfRobot,
		 float rvel,
		 Point referencePoint,
		 Point centerPoint)
{
  if (rvel > 0.0) 
    return compute_angle_3p( centerPoint, referencePoint, cutPointOfRobot);
  else
    return compute_angle_3p( centerPoint, cutPointOfRobot, referencePoint);
}



/* Computes the angle until the robot collides with the given collPoint.
 * May also be used to find out the distance to the target. */
static float 
computeAngleToPoint( Point rpos, float rrot, float rvel,
		     Point collPoint,
		     Circle_trajectory* circ)
{
  float distanceFromRobot = compute_distance( rpos, collPoint);
  float virtualRobRadius, distanceFromTrajectoryCenter;

  /* --------------------------------------------------------------------
   * Check for all the cases whihc need special treatment.
   * -------------------------------------------------------------------- */
  
  /* Is the point in the ring? */
  if ( ! is_point_in_ring_area( circ->M,
				circ->small_rad,
				circ->big_rad,
				collPoint))
    return DEG_360;
  

  /* The point is in the robot --> no distance at all. */
  if ( distanceFromRobot <= ROB_RADIUS)
    return 0.0;

  distanceFromTrajectoryCenter = compute_distance( circ->M, collPoint);
  
  /* This is the radius of the robot PLUS the corresponding security
   * distance. */
  virtualRobRadius =
    ( distanceFromTrajectoryCenter < circ->middle_rad)
    ?
    ( circ->middle_rad - circ->small_rad + 0.1)
    :
    ( circ->big_rad - circ->middle_rad + 0.1);

  /* We distinct between trajectories with the center of the trajectory
   * circle within the robot and bigger circle trajectories. */
  if ( circ->small_rad <= 0.1) {
    
    /* Let's be conservative:
     * If the point is in the virtual robot no motion is allowed in the
     * small circle. */
    if ( distanceFromRobot <= virtualRobRadius)
      return 0.0;
  }
  
  /* The circle of the trajectory is big enough so that the center of
   * this circle is outside the robot. */
  else {
    
    /* Here we don't have to be as conservative. If the point is behind
     * the robot there is nothing special to be done. */
    if ( distanceFromRobot <= virtualRobRadius
	 && ! behindPoint( rpos, rrot, collPoint))
      return 0.0;
  }
  
  /* --------------------------------------------------------------------
   * When we get here everything is fine.
   * -------------------------------------------------------------------- */
  {
    
    /* Create a circle through the target point and cut it
     * with the robot. */
    Circle robotCircle            = { virtualRobRadius, rpos};
    Circle circleThroughCollPoint = { distanceFromTrajectoryCenter, circ->M};
    float angle;
    float minAngle = DEG_360;
    
    Point cutPoints[2];
    int cnt, cutCnt;
  
    cutCnt = cut_circle_and_circle( &robotCircle,
				    &circleThroughCollPoint,
				    cutPoints);

    for (cnt=0; cnt<cutCnt; cnt++) {
      
      angle = angleToCutPoint( cutPoints[cnt], rvel,
			       collPoint, circ->M);
      
      if (angle < minAngle) 
	minAngle = angle;
    }
    if ( cutCnt == 0) {
      /* Just a check. This must not happen. */
      putc( 7, stderr);
      fprintf(stderr,"Warning!\n");
      fprintf( stderr, "Circles: M %f %f  Radii: %f %f %f\n",
	       circ->M.x, circ->M.y, circ->small_rad, 
	       circ->middle_rad, circ->big_rad);	       
      minAngle = 0.0;
    }
    
    return minAngle;
  }
}


/**********************************************************************
 * The next functions return the distance / angle to the closest collision
 * point. They differ only in the representation of the points (as two dimensional
 * or one dimensional array of lines or as one dimensional array of points) and
 * in the collision area (ring or rectangle). 
 **********************************************************************/
static float
minAngleToCollisionLineOnTrajectory( Point rpos, float rrot, float rvel,float tvel, 
				     LineSeg **lines, int first_dim, int second_dim, 
				     Circle_trajectory *circ, float rob_angle)
{
  int i, j, k, cnt;
  float angle, min_angle=DEG_360;
  float arm_angle;
  Point actualPoint;
  Point arm_pt;
  Circle traj_circle;
  Circle arm_circle,big_circle;

  /* especially for arm-collision */
  Point armCollisionPoints[4];  /* structure for points of intersection */
  int numberOfArmCollisionPoints = 0; /* number of intersections */
  BOOLEAN pointInTrajectory;

  /* calculates the minimal angle when the arm is outside */
  if (armState != INSIDE)
    {
      /* The part of arm reaching out of the trajectory. */
      arm_circle.M = circ->M;
      arm_circle.rad = circ->arm_rad;
      big_circle.M = circ->M;
      big_circle.rad = circ->big_rad;

      /* Crucial point of the arm. */
      arm_pt = computeInnerArmPoint(rpos,rrot,rvel,tvel);
      arm_angle = compute_angle_2p(circ->M, arm_pt);
    
      traj_circle.M = circ->M;
      traj_circle.rad = circ->middle_rad;

      /* Each line is represented by three points. */
      for (i=0; i<first_dim; i++) {
	for (j=0; j < second_dim; j++) {
	  if (lines[i][j].pt1.x != F_ERROR) {
	    pointInTrajectory = FALSE;
	    for (k=0; k<2; k++) {
	      switch (k)
		{
		case (0): 
		  actualPoint = lines[i][j].pt1;
		break;
		case (1): 
		  actualPoint = lines[i][j].pt2;	
		break;
		case (2): 
		  actualPoint.x = (lines[i][j].pt1.x + lines[i][j].pt2.x) * 0.5;
		  actualPoint.y = (lines[i][j].pt1.y + lines[i][j].pt2.y) * 0.5;
		break;
		}

	      /* Is this point in the biggest ring: [innerCircle : armCircle]? */
	      if ( is_point_in_ring_area(circ->M,
					 circ->small_rad,
					 circ->arm_rad,
					 actualPoint)) {
		
		/* Now compute the angle to this point within the robot ring. */
		pointInTrajectory = TRUE;
		
		/* Now compute the angle to this point within the robot ring. */
		angle = computeAngleToPoint( rpos, rrot, rvel,
					     actualPoint, circ);
		
		if ( angle < min_angle)
		  min_angle = angle;
		
	      }
	    } /* End for k */
	    
	    if ( pointInTrajectory)
	      {
		/* Now check wether the line reaches into the area of the arm. */
		numberOfArmCollisionPoints =
		  get_collpoints_circle( arm_circle,
					 big_circle,
					 lines[i][j],
					 armCollisionPoints);
		
		if ( numberOfArmCollisionPoints != 0)
		  {
		    for (cnt=0; cnt<numberOfArmCollisionPoints; cnt++)
		      {
			if (rvel < 0.0) 
			  angle =
			    normed_angle( arm_angle
					  - compute_angle_2p(circ->M,
							     armCollisionPoints[cnt]));
			else
			  angle =
			    normed_angle( compute_angle_2p( circ->M,
							    armCollisionPoints[cnt])
					  - arm_angle);
			  
			if (angle < min_angle) 
			  min_angle = angle; 
		      } /* End for cnt */   
		  } /* End if */
	      } /* End if pointInTrajectory */
	  } /* End if (F_ERROR) */
	} /* End for j */
      } /* End for i */
      if (dumpInfo)
	fprintf( dumpFile, "minangle: %f\n", RAD_TO_DEG(min_angle));
      return(min_angle);
    }
  
  /* calculates the minimal angle when the arm is inside */
  else 
    {
      big_circle.M = circ->M;
      big_circle.rad = circ->big_rad;
      
      traj_circle.M = circ->M;
      traj_circle.rad = circ->middle_rad;
      
      
      for (i=0; i<first_dim; i++) {
	for (j=0; j < second_dim; j++) {
	  if (lines[i][j].pt1.x != F_ERROR) {
	    
	    for (k=0; k<2; k++) {
	      switch (k) {
	      case (0): 
		actualPoint = lines[i][j].pt1;
	      break;
	      case (1): 
		actualPoint = lines[i][j].pt2;	
	      break;
	      case (2): 
		actualPoint.x = (lines[i][j].pt1.x + lines[i][j].pt2.x) * 0.5;
	      actualPoint.y = (lines[i][j].pt1.y + lines[i][j].pt2.y) * 0.5;
	      break;
	      }
	      
	      /* Now compute the angle to this point within the robot ring. */
	      angle = computeAngleToPoint( rpos, rrot, rvel,
					   actualPoint, circ);
	      
	      if ( angle < min_angle)
		min_angle = angle;
	      
	    } /* End for k */
	  } /* End if ... != F_ERROR */
	} /* End for j (second_dim) */
      } /* End for i (first_dim) */
      if (dumpInfo)
	fprintf( dumpFile, "minangle: %f\n", RAD_TO_DEG(min_angle));
      return(min_angle);
    } /* End else */
}




static float
minDistToCollisionLineOnTrajectory( Point rpos, float rrot,
				    float robSize,
				    LineSeg **lines, int first_dim, 
				    int second_dim, LineSeg *lline, LineSeg *rline)
{
  int i, j, k;
  Point actualPoint;
  float dist, min_dist=1000.0;
    
  for (i=0; i<first_dim; i++) {
    for (j=0; j < second_dim; j++) {
      if (lines[i][j].pt1.x != F_ERROR) {
	for (k=0; k<2; k++) {
	  switch (k) {
	  case (0): 
	    actualPoint = lines[i][j].pt1;
	  break;
	  case (1): 
	    actualPoint = lines[i][j].pt2;	
	  break;
	  case (2): 
	    actualPoint.x = (lines[i][j].pt1.x + lines[i][j].pt2.x) * 0.5;
	    actualPoint.y = (lines[i][j].pt1.y + lines[i][j].pt2.y) * 0.5;
	  break;
	  }
	  
	  if ( is_point_in_line_area( lline, rline, actualPoint)) {
	    dist = compute_distance(rpos, actualPoint);
	    if (dist < min_dist) 
	      min_dist = dist;
	  }
	}
      }
    }
  }
  return( MAX( 0.0, min_dist - robSize));
}

/**********************************************************************
 * The next functions return the distance / angle to the closest collision
 * point. They differ only in the representation of the points (as two dimensional
 * or one dimensional array of lines or as one dimensional array of points) and
 * in the collision area (ring or rectangle). 
 **********************************************************************/
static float
minAngleToCollisionPointOnTrajectory( Point rpos, float rrot, float rvel,float tvel, 
				      Point *points, int numberOfPoints, 
				      Circle_trajectory *circ, float rob_angle)
{
  int i;
  float angle, min_angle=DEG_360;
  Point actualPoint;
  Circle traj_circle;

  /* calculates the minimal angle when the arm is outside */
  if (armState != INSIDE)
  {
    /* especially for arm-collision */
    float arm_angle;
    Point arm_pt;
     
    /* Crucial point of the arm. */
    arm_pt = computeInnerArmPoint(rpos,rrot,rvel,tvel);
    arm_angle = compute_angle_2p(circ->M, arm_pt);
    
    traj_circle.M = circ->M;
    traj_circle.rad = circ->middle_rad;

    for (i=0; i<numberOfPoints; i++) {

      if (points[i].x != F_ERROR) {

	actualPoint = points[i];
	
	/* Is this point in the robot ring: [innerCircle : outerCircle]? */
	if (is_point_in_ring_area( circ->M,
				   circ->small_rad,
				   circ->big_rad,
				   actualPoint)) {

	  /* Now compute the angle to this point within the robot ring. */
	  angle = computeAngleToPoint( rpos, rrot, rvel,
				       actualPoint, circ);
	  
	  if ( angle < min_angle)
	    min_angle = angle;
	}
	
	/* Otherwise check wether the point is in the arm area. */
	else if ( is_point_in_ring_area( circ->M,
					 circ->big_rad,
					 circ->arm_rad,
					 actualPoint)) {
	  if (rvel > 0.0) 
	    angle =
	      normed_angle( arm_angle
			    - compute_angle_2p(circ->M,
					       actualPoint));
	  else
	    angle =
	      normed_angle( compute_angle_2p( circ->M,
					      actualPoint)
			    - arm_angle);
	  
	  if (angle < min_angle) 
	    min_angle = angle;
	  
	} /* End if */
      } /* End if (F_ERROR) */
    } /* End for i */
    if (dumpInfo)
      fprintf( dumpFile, "minangle: %f\n", RAD_TO_DEG(min_angle));
    return(min_angle);
  }
  
  /* calculates the minimal angle when the arm is inside */
  else {

    traj_circle.M = circ->M;
    traj_circle.rad = circ->middle_rad;
        
    for (i=0; i < numberOfPoints; i++) {
      
      if (points[i].x != F_ERROR) {
	  
	/* Now compute the angle to this point within the robot ring. */
	angle = computeAngleToPoint( rpos, rrot, rvel,
				     points[i], circ);
	
	if ( angle < min_angle)
	  min_angle = angle;
      }  
    } /* End for i (first_dim) */
    if (dumpInfo)
      fprintf( dumpFile, "minangle: %f\n", RAD_TO_DEG(min_angle));
    return(min_angle);
  } /* End else */
}



static float
minDistToCollisionPointOnTrajectory( Point rpos, float rrot,
				     float robSize,
				     Point *points, int numberOfPoints,
				     LineSeg *lline, LineSeg *rline)
  {
  int i;
  Point actualPoint;
  float dist, min_dist=1000.0;

  for (i = 0; i < numberOfPoints; i++) {

    if ( points[i].x != F_ERROR) {

      actualPoint = points[i];
      
      if ( is_point_in_line_area( lline, rline, actualPoint)) {
	dist = compute_distance(rpos, actualPoint);

	if (dist < min_dist) 
	  min_dist = dist;
      }
    }
  }
  return( MAX( 0.0, min_dist - robSize));
}

static float
minDistToCollisionLine( Point rpos,
			LineSeg* lines, int numberOfLines)
{
  int i;
  float minDist = MAX_RANGE, tmp;
  
  for (i=0; i < numberOfLines; i++) {
    if (lines[i].pt1.x != F_ERROR) 
      if ( (tmp = compute_distance(rpos, lines[i].pt1)) < minDist) 
	minDist = tmp;
  }
  return minDist;
}

			  
static float
minDistToCollisionPoint( Point rpos,
			 Point* points, int numberOfPoints)
{
  int i;
  float minDist = MAX_RANGE, tmp;
  
  for (i=0; i < numberOfPoints; i++) {
    if ( (tmp = compute_distance(rpos, points[i])) < minDist) 
      minDist = tmp;
  }  
  return minDist;
}

			  

/**********************************************************************
 *   Computes the time until the robot reaches the closest obstacle   *
 *   which is in the tajectory area given through the velocities and  *
 *   the size of the area.                                            *
 *   If the target_flag is true the distance to the target point is   *
 *   also computed and set in *target_dist.
 *   This function uses all different sonar, external lines and points.
 *   Instead of using lines we only consider three points of a line.
 *********************************************************************/
float
computeCollisionDistance( Point rpos, float rrot, float tvel, float rvel,
			  float size, float addsize, float armsize,
			  float *target_dist)
{
  float colldist;               /* Distance till next collision */
  Circle_trajectory noTransCirc;
  float noTransAngle = DEG_360;
  Circle_trajectory circ;       /* Collision area (circle)  */
  LineSeg lline, rline;         /* Collision area (line)    */
  float min_angle=DEG_360;
  float min_dist=MAX_RANGE;
  float rob_angle=0.0;
  float tmp;
  BOOLEAN line_flag=FALSE;

  /* If the robot wants to rotate we first have to check wether the next
   * collision line is not in the security area. In this case the security
   * dist is set to 2.0 centimeter.
   */
  if (fabs(tvel) < EPSILON) {
    int i, j;
    
    if ((tmp = compute_distance(target, rpos)) <= ROB_RADIUS)
      *target_dist = tmp;
    else 
      *target_dist = MAX_RANGE;
    
    /*--------------------------------------------------
     * SONARS
     *--------------------------------------------------*/
    if ( use_sonar) {
      for (i = 0; i < bRobot.sonar_cols[0]; i++) {
	if (( tmp = minDistToCollisionLine( rpos, CollLines[i], REMEMBER_INTERVAL))
	    < min_dist)
	  min_dist = tmp;
      }
    }
    
    /*--------------------------------------------------
     * LASER
     *--------------------------------------------------*/
    if ( use_laser) {
      if ((tmp = minDistToCollisionPoint( rpos, Laser_CollPoints.points,
					  Laser_CollPoints.numberOfPoints)) < min_dist)
	min_dist = tmp;
    }
    
    /*--------------------------------------------------
     * BUMPER
     *--------------------------------------------------*/
    if ( use_bumper) {
      if ((tmp = minDistToCollisionPoint( rpos, Bumper_Obstacle_Points.points,
					  Bumper_Obstacle_Points.no_of_points)) < min_dist)
	min_dist = tmp;
    }
    
    /*--------------------------------------------------
     * Ir
     *--------------------------------------------------*/
    if ( use_ir) {
      if ((tmp = minDistToCollisionPoint( rpos, Ir_Obstacle_Points.points,
					  Ir_Obstacle_Points.no_of_points)) < min_dist)
	min_dist = tmp;
    }
    
#ifdef UNIBONN

    /*--------------------------------------------------
     * EXTERNAL OBSTACLE INFORMATION
     *--------------------------------------------------*/
    if ( use_external) {

      if (( tmp = minDistToCollisionPoint( rpos, External_Obstacle_Points.points,
					   External_Obstacle_Points.no_of_points))
	  < min_dist)
	min_dist = tmp;
    }
#endif
    
    colldist = MAX( 0.0, min_dist - size);  
    
    /* In the case of rotation (tvel == 0), we compute the minimal angle
       to the next obstacle. If the obstacle is inside the area of big_rad
       then we will return zero as angle( = distance).
       Otherwise the distance is computed as follows:
       distance = minimal angle * MAX_RANGE / 360 rad */
    
    if ( armState != INSIDE)
      {
	if( colldist <= (noTransCirc.big_rad-ROB_RADIUS))
	
	  {
	    return 0.0;
	  }
	else
	  {
	    noTransCirc =
	      velocities_to_circle_trajectory(rpos, rrot, 0.0,
					      rvel, size, addsize, armsize);
	
	    /*--------------------------------------------------
	     * SONAR
	     *--------------------------------------------------*/
	    if ( use_sonar) 
	      noTransAngle =
		minAngleToCollisionLineOnTrajectory( rpos, rrot, rvel,0.0, 
						     CollLines,NO_OF_SONARS,
						     REMEMBER_INTERVAL, 
						     &noTransCirc, rob_angle);
	    
	    /*--------------------------------------------------
	     * LASER
	     *--------------------------------------------------*/
	    if ( use_laser) {
	      tmp = minAngleToCollisionPointOnTrajectory( rpos, rrot, rvel,0.0, 
							  Laser_CollPoints.points,
							  Laser_CollPoints.numberOfPoints,
							  &noTransCirc, rob_angle);
	      if ( tmp < noTransAngle) 
		noTransAngle = tmp;
	    }
	
#ifdef UNIBONN
	    /*--------------------------------------------------
	     * EXTERNAL
	     *--------------------------------------------------*/
	    if ( use_external) {
	      /* The external are only one dimensional. So we cast them to a two
	       * dimensional structure with size 1 in the first dimension. */
	      tmp = minAngleToCollisionPointOnTrajectory( rpos, rrot, rvel,0.0, 
							  External_Obstacle_Points.points,
							  External_Obstacle_Points.no_of_points,
							  &noTransCirc, rob_angle);
	      if ( tmp < noTransAngle) 
		noTransAngle = tmp;
	    }
#endif
	    /*--------------------------------------------------
	     * BUMPER INFORMATION
	     *--------------------------------------------------*/
	    
	    if ( use_bumper) {
	      tmp = minAngleToCollisionPointOnTrajectory( rpos, rrot, rvel,0.0, 
							  Bumper_Obstacle_Points.points,
							  Bumper_Obstacle_Points.no_of_points,
							  &noTransCirc, rob_angle);
	      if ( tmp < noTransAngle) 
		noTransAngle = tmp;
	    }

	    /*--------------------------------------------------
	     * Ir INFORMATION
	     *--------------------------------------------------*/
	    
	    if ( use_ir) {
	      tmp = minAngleToCollisionPointOnTrajectory( rpos, rrot, rvel,0.0, 
							  Ir_Obstacle_Points.points,
							  Ir_Obstacle_Points.no_of_points,
							  &noTransCirc, rob_angle);
	      if ( tmp < noTransAngle) 
		noTransAngle = tmp;
	    }

	    return ((noTransAngle * MAX_RANGE) / DEG_360);
	  }
      }
    else
      return((colldist > 0.0) ? MAX_RANGE : 0.0); 
  }
  
  /* Check for the kind of the collision area. */
  if (fabs(rvel) < EPSILON) {
    rvel = 0.0; 
    line_flag = TRUE;
  }
  else if (fabs(tvel/rvel) > MAX_CURVE_RADIUS) {
    rvel = 0.0; 
    line_flag = TRUE;
  }
  
  
  /* Compute the collision area if rvel too small (i.e. we get two lines) */
  if (line_flag) 
    position_to_lines( rpos, rrot, tvel, size+0.5*addsize, &lline, &rline); 
  else {
    /* Else the collision area is a ring (defined by two circles) */
    circ = velocities_to_circle_trajectory( rpos,
					    rrot,
					    tvel, rvel,
					    size,
					    addsize,
					    armsize);
    
    rob_angle = compute_angle_2p(circ.M, rpos);			
  }
  
  if (line_flag) {
    
    /*--------------------------------------------------
     * SONAR
     *--------------------------------------------------*/
    if ( use_sonar) 
      min_dist = minDistToCollisionLineOnTrajectory( rpos, rrot,
						     size + 0.5 * addsize,
						     CollLines,
						     NO_OF_SONARS, REMEMBER_INTERVAL, 
						     &lline, &rline);

    /*--------------------------------------------------
     * LASER
     *--------------------------------------------------*/
    if ( use_laser) {
      tmp = minDistToCollisionPointOnTrajectory( rpos, rrot,
						 size + 0.5 * addsize,
						 Laser_CollPoints.points,
						 Laser_CollPoints.numberOfPoints,
						 &lline, &rline);
      if ( tmp < min_dist) 
	min_dist = tmp;
    }

    
#ifdef UNIBONN
    /*--------------------------------------------------
     * EXTERNAL OBSTACLE INFORMATION
     *--------------------------------------------------*/
    if ( use_external) {
      tmp = minDistToCollisionPointOnTrajectory( rpos, rrot,
						 size + 0.5 * addsize,
						 External_Obstacle_Points.points,
						 External_Obstacle_Points.no_of_points,
						 &lline, &rline);
      if ( tmp < min_dist) 
	min_dist = tmp;
    }
#endif
    
    /*--------------------------------------------------
     * BUMPER INFORMATION
     *--------------------------------------------------*/
    if ( use_bumper) {

      tmp = minDistToCollisionPointOnTrajectory( rpos, rrot,
						 size + 0.5 * addsize,
						 Bumper_Obstacle_Points.points,
						 Bumper_Obstacle_Points.no_of_points,
						 &lline, &rline);
      if ( tmp < min_dist) {
	min_dist = tmp;
      }
    }

    /*--------------------------------------------------
     * IR INFORMATION
     *--------------------------------------------------*/
    if ( use_ir) {

      tmp = minDistToCollisionPointOnTrajectory( rpos, rrot,
						 size + 0.5 * addsize,
						 Ir_Obstacle_Points.points,
						 Ir_Obstacle_Points.no_of_points,
						 &lline, &rline);
      if ( tmp < min_dist) {
	min_dist = tmp;
      }
    }

    colldist = MIN(MAX_RANGE, min_dist);
  }
  
  else {
    
    /*--------------------------------------------------
     * SONAR
     *--------------------------------------------------*/
    if ( use_sonar) {
      min_angle = minAngleToCollisionLineOnTrajectory(rpos, rrot, rvel, tvel, CollLines,
						      NO_OF_SONARS, REMEMBER_INTERVAL, 
						      &circ, rob_angle);
    }
    
    /*--------------------------------------------------
     * LASER
     *--------------------------------------------------*/
    if ( use_laser) { 
      tmp = minAngleToCollisionPointOnTrajectory(  rpos, rrot, rvel, tvel,
						   Laser_CollPoints.points,
						   Laser_CollPoints.numberOfPoints,
						   &circ, rob_angle);
      if ( tmp < min_angle) 
	min_angle = tmp;
    }
    
#ifdef UNIBONN
    /*--------------------------------------------------
     * EXTERNAL
     *--------------------------------------------------*/
    if ( use_external) {
      tmp = minAngleToCollisionPointOnTrajectory(  rpos, rrot, rvel, tvel,
						   External_Obstacle_Points.points,
						   External_Obstacle_Points.no_of_points,
						   &circ, rob_angle);
      if ( tmp < min_angle) 
	min_angle = tmp;
    }
#endif
    
    /*--------------------------------------------------
     * BUMPER
     *--------------------------------------------------*/
    if ( use_bumper) {
      tmp = minAngleToCollisionPointOnTrajectory(  rpos, rrot, rvel, tvel,
						   Bumper_Obstacle_Points.points,
						   Bumper_Obstacle_Points.no_of_points,
						   &circ, rob_angle);
      if ( tmp < min_angle) 
	min_angle = tmp;
    }

    /*--------------------------------------------------
     * IR
     *--------------------------------------------------*/
    if ( use_ir) {
      tmp = minAngleToCollisionPointOnTrajectory(  rpos, rrot, rvel, tvel,
						   Ir_Obstacle_Points.points,
						   Ir_Obstacle_Points.no_of_points,
						   &circ, rob_angle);
      if ( tmp < min_angle) 
	min_angle = tmp;
    }

    if ( min_angle >= DEG_180) 
      /* The average between the circle and MAX_RANGE. */
      colldist =  DEG_180 * circ.middle_rad;
    else
      colldist = min_angle * circ.middle_rad;
    
    /* Now cut collDist. */
    colldist = MIN( colldist, MAX_RANGE);

    if ( min_angle < DEG_360 && circ.small_rad == 0.0) 
      colldist = 0.0;
  }  
  
  if (target_flag) {

    /* Now we compute the distance to the target point if it is on the trajectory */
    if ((*target_dist = compute_distance(rpos, target)) <= ROB_RADIUS)
      return(colldist);
    else
      *target_dist = MAX_RANGE;
     
    if (line_flag) {
      if (is_point_in_line_area(&lline, &rline, target)) 
	*target_dist = MIN( MAX_RANGE, compute_distance(rpos, target)-ROB_RADIUS);
    }
    else {
      /* Now we know that the  the target point is in the collision area
       * and compute the distance to the target.
       * If the target point is behind the robot, i.e. the angle is bigger
       * than DEG_180 we return MAX_RANGE.
       */
      float angleToTarget = computeAngleToPoint( rpos, rrot, rvel, target, &circ);
      if ( angleToTarget > DEG_180)
	*target_dist = MAX_RANGE; 
      else
	*target_dist = MIN( MAX_RANGE, angleToTarget * circ.middle_rad);
    }
  }
  
  return( colldist);
}



/**********************************************************************
 * Computes the distance to the next obstacle on the direct way to the
 * target.  Radius is the size of the collision area.
 **********************************************************************/
float
collDistInTargetDirection( Point rpos, float tvel,
			   Point target, float radius)
{
  float target_dist;

  /* when the arm is outside we consider the part of the arm which stands
     out from the normal trajectories */
  if ( armState != INSIDE)
    return
      computeCollisionDistance(rpos,
			       compute_angle_2p(rpos, target),
			       tvel,
			       0.0,
			       radius,
			       speed_dependent_security_dist(tvel),
			       computeArmOffset(tvel,0.0),		       
			       &target_dist);
  else
    return
      computeCollisionDistance(rpos,
			       compute_angle_2p(rpos, target),
			       tvel,
			       0.0,
			       radius,
			       speed_dependent_security_dist(tvel),
			       0.0,
			       &target_dist);
}





/**********************************************************************
 **********************************************************************
 *       Functions especially for determination of target trajectory  *
 **********************************************************************
 **********************************************************************/

/**********************************************************************
 * Computes the angle to the target point for a given trajectory.
 * If the robot passes the target angle within the time the 
 * returned angle is negative.
 **********************************************************************/
float
lookaheadTargetAngle(Point target, Point rpos, float rrot, 
		     float tvel, float rvel,
		     float tacc, float racc,
		     float time)
{
  float lookahead_angle;
  Point lookahead_pos;
  float lookahead_rot;
   
  /* IMPORTANT: The time is just the next interval. We have to add 
   * the distance needed to stop the robot to the time. */
   
  if (fabs(rvel) < EPSILON)
    rvel = 0.0;
   
  if (tvel == 0.0)  {
    float brakeAngle = brakeDistance( rvel, racc);
    lookahead_pos = rpos;
    lookahead_rot = rrot - rvel * time - brakeAngle;
  }

  else if (rvel == 0.0) {
    float brakeDist = brakeDistance( tvel, tacc);
    lookahead_pos.x = rpos.x + fcos( rrot) * (tvel * time + brakeDist);
    lookahead_pos.y = rpos.y + fsin( rrot) * (tvel * time + brakeDist);
    lookahead_rot = rrot;
  }
  
  else {
    Circle tmp;
    float rad_rot;
    float brakeAngle = brakeDistance( rvel, racc);

    /* We start at the actual position and go to the midpoint of the circle. */
    rad_rot = rrot - ((rvel >= 0.0) ? 1.0 : -1.0) * DEG_90;
     
    tmp.rad = fabs (tvel / rvel);
    tmp.M.x = rpos.x + fcos( rad_rot) * tmp.rad;
    tmp.M.y = rpos.y + fsin( rad_rot) * tmp.rad;

    /* We change the rotation and go back to the circumference. */
    rad_rot = (rad_rot - DEG_180) - rvel * time - brakeAngle;
     
    lookahead_pos.x = tmp.M.x + fcos( rad_rot) * tmp.rad;
    lookahead_pos.y = tmp.M.y + fsin( rad_rot) * tmp.rad;
    lookahead_rot = rrot - rvel * time - brakeAngle;  
  }
  
  lookahead_angle = compute_robot_angle_to_point(lookahead_pos, lookahead_rot, target);

  return(lookahead_angle);
}



/**********************************************************************
 * Given the minimal and maximal velocities we compute the extreme traj
 * and store them as angles in the rvel - tvel space.
 */ 
void
compute_possible_trajectories(void)
{
  int min_tv_sgn, max_tv_sgn, min_rv_sgn, max_rv_sgn;

  if ( actual_velocities.min_tvel == 0.0 &&
       actual_velocities.min_rvel == 0.0 &&
       actual_velocities.max_tvel == 0.0 &&
       actual_velocities.max_rvel == 0.0) {
    actual_velocities.min_ratio = 0.0;
    actual_velocities.max_ratio = 0.0;
    return;
  }

  min_tv_sgn = (actual_velocities.min_tvel < 0.0) ? NEGATIVE : POSITIVE;
  max_tv_sgn = (actual_velocities.max_tvel < 0.0) ? NEGATIVE : POSITIVE;
  min_rv_sgn = (actual_velocities.min_rvel < 0.0) ? NEGATIVE : POSITIVE;
  max_rv_sgn = (actual_velocities.max_rvel < 0.0) ? NEGATIVE : POSITIVE;

  /* I'm sorry but I think that this is necessary */

  /* In the first four cases all possible velocities lie in
   * the same quadrant.
   */
  if ((min_tv_sgn == POSITIVE) && (max_tv_sgn == POSITIVE) && 
      (min_rv_sgn == POSITIVE) && (max_rv_sgn == POSITIVE)) {
    actual_velocities.min_ratio = normed_angle( fatan2(actual_velocities.min_tvel, 
						       actual_velocities.max_rvel));
    actual_velocities.max_ratio = normed_angle( fatan2(actual_velocities.max_tvel, 
						       actual_velocities.min_rvel));
  }
  else if ((min_tv_sgn == POSITIVE) && (max_tv_sgn == POSITIVE) && 
	   (min_rv_sgn == NEGATIVE) && (max_rv_sgn == NEGATIVE)) {
    actual_velocities.min_ratio = normed_angle( fatan2(actual_velocities.max_tvel, 
						       actual_velocities.max_rvel));
    actual_velocities.max_ratio = normed_angle( fatan2(actual_velocities.min_tvel,
						       actual_velocities.min_rvel));
  }
  else if ((min_tv_sgn == NEGATIVE) && (max_tv_sgn == NEGATIVE) && 
	   (min_rv_sgn == NEGATIVE) && (max_rv_sgn == NEGATIVE)) {
    actual_velocities.min_ratio = normed_angle( fatan2(actual_velocities.max_tvel, 
						       actual_velocities.min_rvel));
    actual_velocities.max_ratio = normed_angle( fatan2(actual_velocities.min_tvel, 
						       actual_velocities.max_rvel));
  }
  else if ((min_tv_sgn == NEGATIVE) && (max_tv_sgn == NEGATIVE) && 
	   (min_rv_sgn == POSITIVE) && (max_rv_sgn == POSITIVE)) {
    actual_velocities.min_ratio = normed_angle( fatan2(actual_velocities.min_tvel,
						       actual_velocities.min_rvel));
    actual_velocities.max_ratio = normed_angle( fatan2(actual_velocities.max_tvel,
						       actual_velocities.max_rvel));
  }


  /* In the next four cases the sgn is different in exactly one dimension */
  else if ((min_tv_sgn == POSITIVE) && (max_tv_sgn == POSITIVE) && 
	   (min_rv_sgn == NEGATIVE) && (max_rv_sgn == POSITIVE)) {
    actual_velocities.min_ratio = normed_angle( fatan2(actual_velocities.min_tvel,
						       actual_velocities.max_rvel));
    actual_velocities.max_ratio = normed_angle( fatan2(actual_velocities.min_tvel,
						       actual_velocities.min_rvel));
  }
  else if ((min_tv_sgn == NEGATIVE) && (max_tv_sgn == NEGATIVE) && 
	   (min_rv_sgn == NEGATIVE) && (max_rv_sgn == POSITIVE)) {
    actual_velocities.min_ratio = normed_angle( fatan2(actual_velocities.max_tvel,
						       actual_velocities.min_rvel));
    actual_velocities.max_ratio = normed_angle( fatan2(actual_velocities.max_tvel, 
						       actual_velocities.max_rvel));
  }
  else if ((min_tv_sgn == NEGATIVE) && (max_tv_sgn == POSITIVE) && 
	   (min_rv_sgn == POSITIVE) && (max_rv_sgn == POSITIVE)) {
    actual_velocities.min_ratio = normed_angle( fatan2(actual_velocities.min_tvel,
						       actual_velocities.min_rvel));
    actual_velocities.max_ratio = DEG_360 +
      normed_angle( fatan2(actual_velocities.max_tvel, 
			   actual_velocities.min_rvel));
  }
  else if ((min_tv_sgn == NEGATIVE) && (max_tv_sgn == POSITIVE) && 
	   (min_rv_sgn == NEGATIVE) && (max_rv_sgn == NEGATIVE)) {
    actual_velocities.min_ratio = normed_angle( fatan2(actual_velocities.max_tvel,
						       actual_velocities.max_rvel));
    actual_velocities.max_ratio = normed_angle( fatan2(actual_velocities.min_tvel,
						       actual_velocities.max_rvel));
  }
  
  /* All trajectories are reachable */
  else {
    actual_velocities.min_ratio = 0.0; 
    actual_velocities.max_ratio = DEG_360;
  }
}



/******************************************************************************
 * Given the values for the desired velocities the function returns a trajectory
 * with the minimal and maximal possible velocities on this traj with respect to
 * the actual velocities. If the traj is not possible the function returns a 
 * traj with the closest velocities.
 */

trajectory
closest_possible_trajectory( float tvel, float rvel)
{
  trajectory traj;
  float ratio;
  
  
  /* First we test wether the traj is reachable. */
  if (tvel == 0.0) {
    if (rvel >= 0.0)
      ratio = 0.0;
    else 
      ratio = DEG_180;
  }
  else 
    ratio = normed_angle( fatan2(tvel, rvel)); 

  if (actual_velocities.max_ratio > DEG_360)
    ratio += DEG_360;

  if (ratio >= actual_velocities.min_ratio-EPSILON &&
      ratio <= actual_velocities.max_ratio+EPSILON)
    traj.admissible = TRUE;
  else {
    if (actual_velocities.current_tvel > MIN_TRANS_SPEED ||
	actual_velocities.current_rvel > MIN_ROT_SPEED) 

      traj.admissible = FALSE;
  }
  
  /* In this case the trajectory is reachable. */
  if (traj.admissible) {
    traj.tvel = tvel;
    traj.rvel = rvel;
      
    if (rvel == 0.0) {
      traj.min_tvel = actual_velocities.min_tvel;
      traj.max_tvel = actual_velocities.max_tvel;
    }
    else {
      ratio = tvel / rvel;
      if (ratio >= 0.0) {
	traj.max_tvel = MIN( actual_velocities.max_tvel, 
			     ratio * actual_velocities.max_rvel);
	traj.min_tvel = MAX( actual_velocities.min_tvel,
			     ratio * actual_velocities.min_rvel);
      }
      else {
	traj.max_tvel = MIN( actual_velocities.max_tvel, 
			     ratio * actual_velocities.min_rvel);
	traj.min_tvel = MAX( actual_velocities.min_tvel,
			     ratio * actual_velocities.max_rvel);
      }
    }
  }
    
  /* The traj is not reachable. We use the closest possible velocities. */
  else {
    if (rvel < actual_velocities.min_rvel)
      traj.rvel = actual_velocities.min_rvel;
    else if (rvel < actual_velocities.max_rvel)
      traj.rvel = rvel;
    else 
      traj.rvel = actual_velocities.max_rvel;
      
    if (tvel < actual_velocities.min_tvel)
      traj.tvel = actual_velocities.min_tvel;
    else if (tvel < actual_velocities.max_tvel)
      traj.tvel = tvel;
    else 
      traj.tvel = actual_velocities.max_tvel;
    traj.min_tvel = traj.max_tvel = traj.tvel;
  }
    
  return( traj);
}
  
  
  
/******************************************************************************
 * this function computes the distance to the next obstacle by calling
 * computeCollisionDistance.
 ******************************************************************************/
float
collisionDistance( Point rpos,
		   float rrot,
		   float tvel,
		   float rvel,
		   float* targetDist)
{
  float addsize,armsize;
   
  /* here we determine the armsize */
  if ( armState != INSIDE)
    armsize = computeArmOffset( tvel, rvel);
  else 
    armsize = 0.0;
   
  addsize = speed_dependent_security_dist(tvel);
    
  return computeCollisionDistance( rpos,
				   rrot,
				   tvel,
				   rvel, 
				   ROB_RADIUS+ACTUAL_MODE->security_dist,
				   addsize,
				   armsize,
				   targetDist);
}


/******************************************************************************
 * Auxiliary function for checkWetherOnlyRotationOrOnlyTranslation().
 ******************************************************************************/
static void
checkWetherPointsInsideAndBehind( Point rpos, float rrot,
				  float minDist,
				  Point* points, int numberOfPoints,
				  int* foundPointInside,
				  int* allPointsBehind)
{
  int pt;

  for (pt=0; pt < numberOfPoints; pt++) {
    if ( compute_distance(rpos, points[pt]) < minDist) {
      *foundPointInside = TRUE;
      if ( ! behindPoint( rpos, rrot, points[pt])) {
	*allPointsBehind = FALSE;
	return;
      }
    }
  }

  return;
}


static void
checkWetherLinesInsideAndBehind( Point rpos, float rrot,
				 float minDist,
				 LineSeg* lines, int numberOfLines,
				 int* foundPointInside,
				 int* allPointsBehind)
{
  int i;

  for (i=0; i < numberOfLines; i++) {
    if (lines[i].pt1.x != F_ERROR) {
      if ( compute_distance(rpos, lines[i].pt1) < minDist) {
	*foundPointInside = TRUE;
	if ( ! behindPoint( rpos, rrot, lines[i].pt1)) {
	  *allPointsBehind = FALSE;
	  return;
	}
      }
      if ( compute_distance(rpos, lines[i].pt2) < minDist) {
	*foundPointInside = TRUE;
	if ( ! behindPoint( rpos, rrot, lines[i].pt2)) {
	  *allPointsBehind = FALSE;
	  return;
	}
      }
    }
  }
  return;
}


/* Check for the current situation wether only translation or only
 * rotation is allowed. */
void
checkWetherOnlyRotationOrOnlyTranslation(  Point rpos, float rrot,
					   int* onlyTranslation,
					   int* onlyRotation)
{
  /* If any obstacle is very close to the robot:
   * only translation is allowed if all obstacles are behind the robot
   * only rotation is allowed if NOT all obstacles are behind the robot.
   */

  int i, j;
  int foundPointInside = FALSE;
  int allPointsBehind = TRUE;
    
  float minDist = ROB_RADIUS+ACTUAL_MODE->security_dist;

  *onlyRotation = FALSE;
  *onlyTranslation = FALSE;

  /*------------------------------------------------------------------
   *------------------------------------------------------------------
   * Check all points and lines. As soon as we find an obstacle closer
   * than minDist and this obstacle is not behind the robot we can set
   * onlyRotation. *
   *------------------------------------------------------------------
   *------------------------------------------------------------------*/
  
  /*--------------------------------------------------
   * SONARS
   *--------------------------------------------------*/
  if ( use_sonar) {
    for (i = 0; i < bRobot.sonar_cols[0]; i++) {
      checkWetherLinesInsideAndBehind( rpos, rrot, minDist,
				       CollLines[i], REMEMBER_INTERVAL,
				       &foundPointInside,
				       &allPointsBehind);
      
      if ( foundPointInside && ! allPointsBehind) {
	*onlyRotation = TRUE;
	return;
      }
    }
  }

  /*--------------------------------------------------
   * LASER
   *--------------------------------------------------*/
  if (use_laser) {
    checkWetherPointsInsideAndBehind( rpos, rrot,
				      minDist,
				      Laser_CollPoints.points,
				      Laser_CollPoints.numberOfPoints,
				      &foundPointInside,
				      &allPointsBehind);
    
    if ( foundPointInside && ! allPointsBehind) {
      *onlyRotation = TRUE;
      return;
    }
  }
    
  
#ifdef UNIBONN
  
  /*--------------------------------------------------
   * EXTERNAL OBSTACLE INFORMATION
   *--------------------------------------------------*/
  if ( use_external) {
    checkWetherPointsInsideAndBehind( rpos, rrot,
				      minDist,
				      External_Obstacle_Points.points,
				      External_Obstacle_Points.no_of_points,
				      &foundPointInside,
				      &allPointsBehind);
    
    if ( foundPointInside && ! allPointsBehind) {
      *onlyRotation = TRUE;
      return;
    }
  }
#endif
  
  /*--------------------------------------------------
   * BUMPER INFORMATION
   *--------------------------------------------------*/
  if ( use_bumper) {
    checkWetherPointsInsideAndBehind( rpos, rrot,
				      minDist,
				      Bumper_Obstacle_Points.points,
				      Bumper_Obstacle_Points.no_of_points,
				      &foundPointInside,
				      &allPointsBehind);
    
    if ( foundPointInside && ! allPointsBehind) {
      *onlyRotation = TRUE;
      return;
    }
  }

  /*--------------------------------------------------
   * IR INFORMATION
   *--------------------------------------------------*/
  if ( use_ir) {
    checkWetherPointsInsideAndBehind( rpos, rrot,
				      minDist,
				      Ir_Obstacle_Points.points,
				      Ir_Obstacle_Points.no_of_points,
				      &foundPointInside,
				      &allPointsBehind);
    
    if ( foundPointInside && ! allPointsBehind) {
      *onlyRotation = TRUE;
      return;
    }
  }

  /* In this case there are points inside the close area but all
   * points are behind the robot ---> only straight motion is allowed. */
  if ( foundPointInside)
    *onlyTranslation = TRUE;
  
  return;
}
