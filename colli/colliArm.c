
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




/* Computes the coordinates of the inner armpoint |___!<-   */
/*                                                  |       */
/* For detailed information see figure 9 (documentation)    */
  
Point computeInnerArmPoint(Point rpos,float rrot,float rvel,float tvel)
{
   
   float angle,rdir,tdir;
   Point result;

   rdir = (rvel >= 0.0) ? 1.0 : -1.0;
   tdir = (tvel >= 0.0) ? 1.0 : -1.0;
   angle = rrot + DEG_180 - (rdir*tdir) * ADDANGLE;
  
   norm_angle(&angle);
   result.x = rpos.x + fcos(angle) * ARMLENGTH;
   result.y = rpos.y + fsin(angle) * ARMLENGTH;
   return(result);
}
/*                                                     .<-   */
/* Computes the coordinates of the outer armpoint \___/      */
/*                                                  |        */
/* For detailed information see figure 9 (documentation)     */

Point computeOuterArmPoint(Point rpos,float rrot,float rvel,float tvel)
{
   
   float angle,rdir,tdir;
   Point result;

   rdir = (rvel >= 0.0) ? 1.0 : -1.0;
   tdir = (tvel >= 0.0) ? 1.0 : -1.0;
   angle = rrot + DEG_180 - (rdir*tdir) * ADDANGLE;
  
   norm_angle(&angle);
   result.x = rpos.x + fcos(angle) * (ARMLENGTH+22.0);
   result.y = rpos.y + fsin(angle) * (ARMLENGTH+22.0);
   return(result);
}

/* Computes how much the arm stands out from the circle defined by big_rad */
float
computeArmOffset( float tvel, float rvel)
{
  Circle_trajectory circ;
  LineSeg armLine;
  Point cutPoints[2];
  Circle outerCircle;
  float rdir, tdir, armRot;
  int cnt;
  float additionalDist;
  Point rPos;
  float rRot;

/* Compare with figure 10 (documentation) */  
#define ARM_ANGLE (DEG_TO_RAD(25.0))
#define ARM_LENGTH 69.0

  /* We don't need these values. */
  rPos.x = rPos.y = rRot = 0.0;
  
  /* If we have straight motion there is no additional size necessary. */
  if (fabs(rvel) < EPSILON) 
    return 0.0;
  if (fabs(tvel/rvel) > MAX_CURVE_RADIUS)
    return 0.0;
  if ( fabs(tvel) < MIN_TRANS_SPEED)
    return ARM_LENGTH - ACTUAL_MODE->security_dist - ROB_RADIUS; 
  
  /* The collision area is a ring (defined by two circles). We are only interested
   * in the outer circle. */
  circ = velocities_to_circle_trajectory(rPos, rRot, tvel, rvel,
					 ROB_RADIUS+ACTUAL_MODE->security_dist,
					 0.0,0.0);
  outerCircle.M = circ.M;
  outerCircle.rad = circ.big_rad;

  /* Now let's compute wether we need the left gripper of the arm or the
   * right one.
   */
  rdir = (rvel >= 0.0) ? 1.0 : -1.0;
  tdir = (tvel >= 0.0) ? 1.0 : -1.0;
  armRot = rRot + DEG_180 - (rdir*tdir) * ARM_ANGLE;

  /* This line represents the edge of the gripper of the arm. */
  armLine.pt1 = rPos;
  armLine.pt2.x = armLine.pt1.x + fcos( armRot) * ARM_LENGTH;
  armLine.pt2.y = armLine.pt1.y + fsin( armRot) * ARM_LENGTH;

  if ( (cnt = cut_circle_and_line( outerCircle, armLine, cutPoints)) == 0)  {
    fprintf( stderr, "Huch. Kein Schnittpunkt.\n");
    additionalDist = ARM_LENGTH;
  }
  else if ( cnt == 1)
  {
    additionalDist = compute_distance( armLine.pt2, cutPoints[0]);
  }
  else
  {
    if ( point_on_LineSeg( cutPoints[0], armLine))
    {
      additionalDist = compute_distance( armLine.pt2, cutPoints[0]);
    }
    else {
      if ( ! point_on_LineSeg( cutPoints[1], armLine))
      {
	additionalDist = 0.0;
      }
      else
      {
	additionalDist = compute_distance( armLine.pt2, cutPoints[1]);
      }
    }
  }
  /* To get the ADDITIONAL distance given by the arm we have to substract
   * the radius and the security dist.
   */

  return additionalDist;
}






