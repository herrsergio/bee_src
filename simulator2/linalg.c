
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




#include <X11/Intrinsic.h>
#include <math.h>
#include "trigofkt.h"
#include "linalg.h"

#define EPSILON  0.00001
#define mySQR(x)        ((x) * (x))
#define myABS(x)  ((x) < 0 ? -(x) : (x))

/**************************************************************************
 * Norms the angle between [0:360].
 **************************************************************************/

float 
normedAngle(float angle)
{
  while (angle < 0.0)
    (angle) += 360.0;
  
  while (angle >= 360)
    (angle) -= 360;

  return(angle);
}


typedef struct Point {
  float x;
  float y;
} Point;


/* --------------- some linear algebra -------------------------- */

int
point_on_left_side(float px, float py,
		   float x1, float y1,
		   float x2, float y2)
{
    return (px - x1) * ( -y2 + y1) + (py - y1) * (x2 - x1) > 0 ? 1 : -1;
}

void
line_min_dist_point(float x1, float y1,
			float x2, float y2,
			float xp, float yp,
			 float *xo, float *yo)
{
    float x,y,xmax,ymax,xmin,ymin;
    if(x1 > x2) {
	xmin = x2;
	xmax = x1;
    }
    else {
	xmin = x1;
	xmax = x2;
    }      
    if(y1 > y2) {
	ymin = y2;
	ymax = y1;
    }
    else {
	ymin = y1;
	ymax = y2;
    }      
    if( x1 == x2) {
	x = x1;
	if(yp < ymin) 
	    y = ymin;
	else if( yp > ymax)
	y = ymax;
	else
	    y = yp;
    }
    else if(y1 == y2) {
	y = y1;
	if(xp < xmin) 
	    x = xmin;
	else if( xp > xmax)
	    x = xmax;
	else
	    x = xp;
    }
    else {
	float m = (y2 - y1) / (x2 - x1);
	x = m*m*x1+xp-m*(y1-yp) / (m*m+1);
	if( x < xmin) {
	    x = xmin;
	    y = ymin;
	}
	else if(x > xmax) {
	    x = xmax;
	    y = ymax;
	}
	else
	    y = m*(x-x1)+y1;
    }
    *xo = x;
    *yo = y;
}

float line_min_distance(float x1, float y1,
			float x2, float y2,
			float xp, float yp)
{
    float x,y,dx,dy;
    line_min_dist_point(x1,y1,x2,y2,xp,yp,&x,&y);
    dx = xp - x;
    dy = yp - y;
    return (float) sqrt(dx*dx+dy*dy);
}

Boolean inhalfplane(float xp, float yp, float x1, float y1, float x2, float y2)
{
    float m,c;

  if(x1 == x2) {
    if(y1 < y2)
      return (xp <= x1);
    else
      if(y1 > y2) 
	return (xp >= x1);
      else
	return FALSE;
  }
  else {
    if( x1 < x2 ) {
      m  = (y2 - y1) / (x2 - x1);
      c = y1 - m*x1;
      return (yp >= m*xp+c);
    }
    else {
      m  = (y1 - y2) / (x1 - x2);
      c = y1 - m*x1;
      return (yp <= m*xp+c);
    }
  }
}

/* ------------------------------------------------------------------------- */

float
Rad2Deg(float angle)
{
  return angle * 180.0 / M_PI;
}


/**********************************************************************
 *   Computes the angle from pt1 to pt2                               *
 **********************************************************************/

float 
angle_2p(Point pt1, Point pt2)
{
  pt2.x -= pt1.x;
  pt2.y -= pt1.y;
  
  if (fabs( pt2.x) < EPSILON && fabs( pt2.y) < EPSILON)
    return(0.0);
  else {
    return normedAngle(
		       Rad2Deg((float) atan2((double) pt2.y,
					     (double) pt2.x)));
  }
}

float 
angle_3p(Point M, Point pt1, Point pt2)
{
  return normedAngle(angle_2p(M, pt2) - angle_2p(M, pt1));
}

void
fswap(float *x, float *y)
{
	float	swap;

	swap = *x;
	*x = *y;
	*y = swap;

} /* fswap() */



float
SQR(float x)
{
  return x*x;
}


float
PointDistance(float x1, float y1, float x2, float y2)
{
  return (float) sqrt((double) (SQR(x1-x2) + SQR(y1-y2)));

} /* PointDistance() */

int 
check_intersection(float p_x1, float p_y1, float p_x2, float p_y2,
		     float q_x1, float q_y1, float q_x2, float q_y2,
		     float *x, float *y)
{
  float mu, lambda;

  float p_dx;
  float p_dy;
  float q_dx;
  float q_dy;
  float n;
  float d;

  p_dx = p_x2 - p_x1;
  p_dy = p_y2 - p_y1;
  q_dx = q_x2 - q_x1;
  q_dy = q_y2 - q_y1;

  n = (q_x1 * q_dy) - (q_y1 * q_dx) - (p_x1 * q_dy) + (p_y1 * q_dx);
  d = (p_dx * q_dy) - (p_dy * q_dx);

  if (d == 0.0)			/* collinear */
    return 0;

  lambda = n/d;
  
  n = (p_x1 * p_dy) - (p_y1 * p_dx) - (q_x1 * p_dy) + (q_y1 * p_dx);
  d = (q_dx * p_dy) - (q_dy * p_dx);
  
  if (d == 0.0)			/* ...shouldn't happen here! */
    return 0;

  mu = n/d;


  /*  fprintf(stderr, "### P=(%6.4f %6.4f ->%6.4f %6.4f) Q=(%6.4f %6.4f ->%6.4f %6.4f)   %6.4f %6.4f, lambda=%6.4f mu=%6.4f -> %d\n",
	  p_x1,  p_y1,  p_x2,  p_y2, q_x1,  q_y1,  q_x2,  q_y2,
	  p_x1 + (lambda * p_dx) - q_x1 - (mu * q_dx),
	  p_y1 + (lambda * p_dy) - q_y1 - (mu * q_dy), 
	  lambda, mu, 
	  (lambda >= 0.0 && lambda <= 1.0 && mu >= 0.0 && mu <= 1.0));
	  */

  if (lambda >= 0.0 && lambda <= 1.0 && mu >= 0.0 && mu <= 1.0){
    *x = p_x1 + (lambda * p_dx);
    *y = p_y1 + (lambda * p_dy);
    return 1;
  }
  else
    return 0;



}

int
cut_line_and_line(float line1x1, float line1y1, 
		  float line1x2, float line1y2, 
		  float line2x1, float line2y1, 
		  float line2x2, float line2y2, 
		  float *dist, float *angle)
{
  
  Point cutPoint;
  int cut;

  Point pt1 = {line1x1,line1y1}, pt2 = {line2x2, line2y2};
  
  cut = check_intersection(line1x1, line1y1, line1x2, line1y2, 
			   line2x1, line2y1, line2x2, line2y2,
			   &cutPoint.x, &cutPoint.y);

  
  if (cut){
    *dist = PointDistance(line1x1, line1y1, cutPoint.x, cutPoint.y);
    *angle = angle_3p(cutPoint, pt1, pt2);
    if (*angle >= 180.0)
      *angle -= 180.0;
    if (*angle > 90)
      *angle = 180.0 - *angle;
  }

  return cut;

} /* cut_line_and_line() */


float		
CalculateAngle(float x1, float y1, float x2, float y2, float x3, float y3)
/*** needed for sonar angle calculation ***/
/*** returns angle in deg ***/
{ 
  float	m1, m2, 
  	tanAngle, Angle;


  /*** calculate Steigung(???) ***/
  if (x2 == x1)	{
  	if (y2 == y1)					/*** this should not happen ! ***/
  		m1 = 0.0;
  	else
  		m1 = 9.9e20;				/*** infinite ***/
  } else
  	m1 = (y2 - y1) / (x2 - x1);

  if (x2 == x3)	{
  	if (y2 == y3)					/*** this should not happen ! ***/
  		m2 = 0.0;
  	else
  		m2 = 9.9e20;				/*** infinite ***/
  } else
  	m2 = (y2 - y3) / (x2 - x3);

  if (m1*m2 == -1.0)
    Angle = 90.0;
  else
    {
      tanAngle = (m2 - m1) / (1 + (m1*m2));
      Angle    = (float)atan((double)tanAngle);
      Angle    = 180.0 * Angle / (float)M_PI;   /*** rad -> deg ***/

      Angle = myABS(Angle);

      if (Angle > 90.0)
  	Angle = 180.0 - Angle;
    }

  return Angle;

} /* CalculateAngle() */


int 
cut_circle_and_line(float cx, float cy, float r, float lx1, float ly1,
		    float lx2, float ly2, 
		    float *dist, float *angle)
{
  float 	dx, dy, mue, lambda, tmp, norm;
  float         S0x, S0y, S1x, S1y;
  float		swap;

  float		PointDistance(float x1, float y1, float x2, float y2);

  void          fswap(float *x, float *y);

  dx = lx2 - lx1;
  dy = ly2 - ly1;

  norm = (float) (sqrt((double) (mySQR(dx) + mySQR(dy))));

  if (norm < EPSILON)		/*** the two points of the line are too close to each other ! ***/
  	return 0;

  dx /= norm;
  dy /= norm;

  /*** First compute the distance between midpoint of the circle
   *** and the line.
   *** The following formulas are the solution to:
   *** line.pt1.x + lambda*dx = M.x + mue*dy   and
   *** line.pt1.y + lambda*dy = M.y - mue*dx.
   *** In this solution the minimal distance between M and line is mue.
   ***/

  mue = (dy * (lx1 - cx) + dx * (cy - ly1)) / (mySQR(dy) + mySQR(dx));

  if (mySQR(r)-mySQR(mue) < 0.0)
    return 0;

  /*** Now compute the two points of intersection ***/  
  
  tmp = (float) (sqrt((double) (mySQR(r) - mySQR(mue))));

  if (dy != 0.0) {
    lambda = (cy - mue*dx - ly1) / dy;

    S0x = lx1 + (lambda - tmp) * dx;
    S0y = ly1 + (lambda - tmp) * dy;
    S1x = lx1 + (lambda + tmp) * dx;
    S1y = ly1 + (lambda + tmp) * dy;
  }
  else { 
    S0x = cx - tmp;
    S0y = ly1;
    S1x = cx + tmp;
    S1y = ly1;
  }

  /*** only used for sonar :  dist, angle ***/

  *dist = PointDistance(lx1, ly1, S0x, S0y);

  swap =  PointDistance(lx1, ly1, S1x, S1y);
  if (swap < *dist) {
  	*dist = swap;
  	*angle = CalculateAngle(lx1, ly1, S1x, S1y, cx, cy);
  } else
  	*angle = CalculateAngle(lx1, ly1, S0x, S0y, cx, cy);

  if (lx1 > lx2) 
  	fswap(&lx1, &lx2);

  if (ly1 > ly2) 
  	fswap(&ly1, &ly2);

  if (((S0x >= lx1) && (S0x <= lx2) && (S0y >= ly1) && (S0y <= ly2)) ||	    /*** infinit length of line ***/
      ((S1x >= lx1) && (S1x <= lx2) && (S1y >= ly1) && (S1y <= ly2))) 
      	return 1;

  return 0;

} /* cut_circle_and_line() */
