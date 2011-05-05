
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


/* Time struct for setStartTime() and timeExpired(). */
static struct timeval startTime[MAX_NUMBER_OF_TIMERS];

/* Lookup tables */
static float table_sin[62832];
static float table_cos[62832];


/**********************************************************************
 **********************************************************************
 *                   Mathemetical help functions                      *
 **********************************************************************
 **********************************************************************/


/**********************************************************************/
float 
  fsin( float x)
{
  for (; x < 0.0        ;) x += DEG_360;
  for (; x >= DEG_360;) x -= DEG_360;
  
  x = MAX(0.0, x);
  x = MIN(2.0 * M_PI, x);

  return(table_sin[(int) (x * 10000.0)]);
}


/**********************************************************************/
float 
fcos( float x)
{
  for (; x < 0.0        ;) x += DEG_360;
  for (; x >= DEG_360;) x -= DEG_360;
  
  x = MAX(0.0, x);
  x = MIN(2.0 * M_PI, x);

  return(table_cos[(int) (x * 10000.0)]);
}




/**********************************************************************/
float 
fsqrt( float x)
{
  return((float) sqrt((double) x));
}


/**********************************************************************/
float 
fatan2( float x, float y)
{
  return((float) atan2((double) x, (double) y));
}


void
setStartTime( int i)
{
   if ( i < MAX_NUMBER_OF_TIMERS)
      gettimeofday( &(startTime[i]), 0);
   else
      fprintf( stderr, "setStartTime: index (%d) too high!\n", i);
}


float
timeExpired( int i)
{
   static struct timeval now;
   
   if ( i < MAX_NUMBER_OF_TIMERS) {
     gettimeofday( &now, 0);
     return timeDiff( &now, &(startTime[i]));
   }
   else {
      fprintf( stderr, "setStartTime: index (%d) too high!\n", i);
      return 0.0;
   }
}

void
printExpiredTime( int i, BOOLEAN longFormat)
{
  if (longFormat)
    fprintf( stderr, "%f seconds of interval %d expired.\n", timeExpired( i), i);
  else
    fprintf( stderr, "%f\n", timeExpired( i)); 
}


float
timeDiff( struct timeval* t1, struct timeval* t2)
{
   float diff;

   diff =  (float) (t1->tv_usec - t2->tv_usec) / 1000000.0;
   diff += (float) (t1->tv_sec - t2->tv_sec);

   return diff;
}


/*****************************************************************************/
/*   float-Normierung: b = norm (a,...)                                      */
/*****************************************************************************/
float 
fnorm(a,amin,amax,bmin,bmax)
     float a,amin,amax,bmin,bmax;
{
  if (amin==amax) {
    fprintf( stderr, "Warning: cannot norm from empty interval.\n");
    return( a < amin ? bmin : bmax);
  }
  else
    if (bmin==bmax)
      return(bmax);
    else
      return( (a-amin) / (amax-amin) * (bmax-bmin) + bmin);
}



/**********************************************************************/
void 
norm_angle(float *angle)
{
  while (*angle < 0.0)
    (*angle) += DEG_360;
  
  while (*angle >= DEG_360)
    (*angle) -= DEG_360;
}


/**********************************************************************/
int 
casted_float(float f)
{
  
  if (f > (float) MAXINT)
    return (MAXINT);
  else if (f < (float) -MAXINT)
    return (-MAXINT);
  else
    return ((int) f);
}



/**********************************************************************/
float 
normed_angle(float angle)
{
  while (angle < 0.0)
    (angle) += DEG_360;
  
  while (angle >= DEG_360)
    (angle) -= DEG_360;

  return(angle);
}


/**********************************************************************/
BOOLEAN 
smaller(Point pt1, Point pt2)
{
  if ( (fabs( pt1.x-pt2.x) < EPSILON))
    if (pt1.y < pt2.y)
      return(TRUE);
    else return(FALSE);
  
  if (pt1.x < pt2.x)
    return(TRUE);
  else 
    return(FALSE);
}


/**********************************************************************/
void 
swap(LineSeg *line)
{
  Point tmp;
  
  tmp = line->pt1;
  line->pt1 = line->pt2;
  line->pt2 = tmp;
}


/**********************************************************************
  * Given a one dimensional histogram the function smoothes the values.
  * SMOOTH_WIDTH gives the distance of influence of the different values
  * on each other.
**********************************************************************/
void 
smoothHistogram(float* hist, int size, int smoothWidth)
{
   static int histSize = 0;
   static float* tmpHist = NULL;
   
   int i, j;
   int cnt;
   
   /* If the dimensions have changed we have to reallocate memory. */
   if ( histSize != size) {
       
       if ( tmpHist != NULL)
	   free( tmpHist);

      tmpHist = (float *) malloc( size * sizeof(float));
      histSize = size;
   }

   
   for (i=0; i < histSize; i++) {
     
      cnt = smoothWidth;
      
      /* Undefined values will be set to zero. */
      if (hist[i] == UNDEFINED) 
	  tmpHist[i] = 0.0;
      
      else {
	
	tmpHist[i] = smoothWidth * hist[i]; 
	for (j=1; j < smoothWidth; j++) {
	  if (i+j < histSize) {
	    if ( hist[i+j] != UNDEFINED)
	      tmpHist[i] += (float) (smoothWidth-j) * hist[i+j]; 
	      cnt += smoothWidth-j;
	  }
	  if (i-j >= 0) {
	    if ( hist[i-j] != UNDEFINED)
	      tmpHist[i] += (float) (smoothWidth-j) * hist[i-j]; 
	      cnt += smoothWidth-j;
	  }
	}
	if (cnt != 0)
	  tmpHist[i] /= (float) cnt;
	else 
	  fprintf(stderr, "Smooth histogram: something's wrong.\n");
      }
   }
   
   for (i=0; i < histSize; i++) 
     hist[i] = tmpHist[i];
   
}


/**********************************************************************
 * Given a two dimensional grid the function smoothes the values.
 * SMOOTH_WIDTH gives the distance of influence of the different values
 * on each other.
 **********************************************************************/
void
smoothGrid( float** grid, int iDim, int jDim, int smoothWidth)
{
   static int iDimension = 0;
   static int jDimension = 0;
   static float** tmpGrid = NULL;
   
   int i, j;
/* #define SEARCH_BEST */
#ifdef SEARCH_BEST
   int bestIBefore=-1, bestJBefore=-1, bestIAfter=-1, bestJAfter=-1;
   float bestBefore=-1.0, bestAfter=-1.0;

#endif

   /* If the dimensions have changed we have to reallocate memory. */
   if ( iDimension != iDim || jDimension != jDim) {
      
      if ( tmpGrid != NULL)
	 free( tmpGrid);

      tmpGrid = (float **) malloc( iDim * sizeof(float *));
      for (i=0; i < iDim; i++) 
      tmpGrid[i] = (float *) malloc( jDim * sizeof(float));

      iDimension = iDim;
      jDimension = jDim;
   }
   
   /* In the outer loops we go through the tmpGrid. For each element we have
    * to get the weighted sum of the surrounding values. This is done
    * via iOffset and jOffset.
    */
   for (i=0; i < iDimension; i++) {
      
      for (j=0; j < jDimension; j++) {
	 
	 int weightCnt = smoothWidth;

#ifdef SEARCH_BEST
	 if ( grid[i][j] != UNDEFINED && grid[i][j] > bestBefore) {
	    bestBefore = grid[i][j];
	    bestIBefore = i;
	    bestJBefore = j;
	 }
#endif
	 
	 /* Undefined values will be set to zero. */
	 if ( grid[i][j] == UNDEFINED) 
	    tmpGrid[i][j] = 0.0;
	 else {
	    
	    int iOffset, jOffset;
	    
	    tmpGrid[i][j] = smoothWidth * grid[i][j]; 
	    
	    for ( iOffset=1; iOffset < smoothWidth; iOffset++) {
	       
	       /* Is the shifted i-index in the defined values? */
	       if (i+iOffset < iDimension) {
		  
		  int shiftedI = i + iOffset;
		  
		  for ( jOffset=1; jOffset < smoothWidth; jOffset++) {
		     
		     float weight = (float) (smoothWidth - MAX( iOffset, jOffset));
		     
		     /* Is the shifted j-index in the defined values? */
		     if (j+jOffset < jDimension) {

			int shiftedJ = j + jOffset;
			
			if ( grid[shiftedI][shiftedJ] != UNDEFINED) 
			   tmpGrid[i][j] += weight * grid[shiftedI][shiftedJ]; 
			weightCnt += (int) weight;
		     }


		     /* Is the shifted j-index in the defined values? */
		     if (j >= jOffset) {

			int shiftedJ = j - jOffset;
			
			if ( grid[shiftedI][shiftedJ] != UNDEFINED) 
			   tmpGrid[i][j] += weight * grid[shiftedI][shiftedJ]; 
			weightCnt += (int) weight;
		     }
		  }
	       }

	       /* Is the shifted i-index in the defined values? */
	       if (i >= iOffset) {
		  
		  int shiftedI = i - iOffset;
		  
		  for ( jOffset=1; jOffset < smoothWidth; jOffset++) {
		     
		     float weight = (float) (smoothWidth - MAX( iOffset, jOffset));
		     
		     /* Is the shifted j-index in the defined values? */
		     if (j+jOffset < jDimension) {

			int shiftedJ = j + jOffset;
			
			if ( grid[shiftedI][shiftedJ] != UNDEFINED) 
			   tmpGrid[i][j] += weight * grid[shiftedI][shiftedJ]; 
			weightCnt += (int) weight;
		     }


		     /* Is the shifted j-index in the defined values? */
		     if (j >= jOffset) {

			int shiftedJ = j - jOffset;
			
			if ( grid[shiftedI][shiftedJ] != UNDEFINED) 
			   tmpGrid[i][j] += weight * grid[shiftedI][shiftedJ]; 
			weightCnt += (int) weight;
		     }
		  }
	       }
	    }
	 }
	 if ( weightCnt != 0)
	     tmpGrid[i][j] /= (float) weightCnt;
	 else 
	    fprintf(stderr, "Smooth grid: something's wrong.\n");
      }
   }

   for (i=0; i < iDimension; i++) 
       for (j=0; j < jDimension; j++) {
	  grid[i][j] = tmpGrid[i][j];
#ifdef SEARCH_BEST
	 if ( grid[i][j] != UNDEFINED && grid[i][j] > bestAfter) {
	    bestAfter = grid[i][j];
	    bestIAfter = i;
	    bestJAfter = j;
	 }
#endif
       }

#ifdef SEARCH_BEST
   fprintf( stderr, "before: %d  %d  after: %d %d\n", bestIBefore, bestJBefore,
	   bestIAfter, bestJAfter);
#endif
}



/**********************************************************************
 **********************************************************************
 *               Functions for analytical geometrie                   *
 **********************************************************************
 **********************************************************************/

/**********************************************************************/
float 
max_norm_distance(Point pt1, Point pt2)
{
  float dist1, dist2;
  
  dist1 = fabs(pt1.x-pt2.x);
  dist2 = fabs(pt1.y-pt2.y);
  return(MAX(dist1, dist2));
}

/**********************************************************************/
float 
compute_distance(Point pt1, Point pt2)
{
  return( fsqrt(SQR(pt1.x-pt2.x) + SQR(pt1.y-pt2.y)));
}




/**********************************************************************
 *   Computes the angle from pt1 to pt2                               *
 **********************************************************************/
float 
compute_angle_2p(Point pt1, Point pt2)
{
  float angle;
  
  pt2.x -= pt1.x;
  pt2.y -= pt1.y;
  
  if (fabs( pt2.x) < EPSILON && fabs( pt2.y) < EPSILON)
    return(0.0);
  else {
    angle = fatan2(pt2.y, pt2.x);
    norm_angle(&angle); 
    return(angle);
  }
}


/**********************************************************************
 *   Computes the angle from pt1 to pt2 (counterclockwise) given the  *
 *   midpoint M of the circle. The angle is in [0.0 : 2 pi]           *
 **********************************************************************/
float 
compute_angle_3p(Point M, Point pt1, Point pt2)
{
  float tmp = compute_angle_2p(M, pt2) - compute_angle_2p(M, pt1);
  norm_angle(&tmp);
  
  return(tmp);
}



/************************************************************************
 *  Computes a circle given two points and the angle of the tangent      *
 * through the first point .                                             *
 * If the solution is a line we return a circle with midpoint.x = F_ERROR.*
 *************************************************************************/
Circle
compute_circle(Point pt1, float pt1_angle, Point pt2)
{
  Circle circ;
  LineSeg line1, line2;
  float angle;
  
  /* First we compute the "Spiegelachse" for pt1 and pt2 */
  line1.pt1.x = (pt1.x + pt2.x) / 2.0; 
  line1.pt1.y = (pt1.y + pt2.y) / 2.0; 
  
  angle = compute_angle_2p(pt2, pt1) + DEG_90;
  line1.pt2.x = line1.pt1.x + fcos(angle) * 10.0;
  line1.pt2.y = line1.pt1.y + fsin(angle) * 10.0;
  
  /* Now we compute the radius line of the circle */
  line2.pt1 = pt1; 
  line2.pt2.x = pt1.x + fcos(pt1_angle+DEG_90) * 10.0;
  line2.pt2.y = pt1.y + fsin(pt1_angle+DEG_90) * 10.0;
  
  circ.M = cut_line_and_line(line1, line2);
  
  if (circ.M.x != F_ERROR)
    circ.rad = compute_distance(circ.M, pt1);
  else 
    circ.rad = 0.0;
  
  return(circ);
}


/**********************************************************************/
/*  If the lines are parallel the coordinates of the point are F_ERROR */
/**********************************************************************/
Point 
cut_line_and_line(LineSeg line1, LineSeg line2)
{
  float dx1, dy1, dx2, dy2, lambda2, tmp1, tmp2;
  Point point;
  
  dx1 = line1.pt2.x - line1.pt1.x;
  dy1 = line1.pt2.y - line1.pt1.y;
  dx2 = line2.pt2.x - line2.pt1.x;
  dy2 = line2.pt2.y - line2.pt1.y;
  
  if (fabs(dx1) < EPSILON) dx1 = 0.0;
  if (fabs(dy1) < EPSILON) dy1 = 0.0;
  if (fabs(dx2) < EPSILON) dx2 = 0.0;
  if (fabs(dy2) < EPSILON) dy2 = 0.0;
  
  if (dy1 == 0.0) {
    if (dy2 == 0.0) {
      point.x = point.y = F_ERROR;
    }
    else {  
      point.y = line1.pt1.y;
      if (dx2 == 0.0)
	point.x = line2.pt1.x;
      else
	point.x = line2.pt1.x + (point.y - line2.pt1.y) * dx2 / dy2;
    }
    return(point);
  }
  
  if (dy2 == 0.0) {
    point.y = line2.pt1.y;
    if (dx1 == 0.0)
      point.x = line1.pt1.x;
    else
      point.x = line1.pt1.x + (point.y - line1.pt1.y) * dx1 / dy1;
    return(point);
  }
  
  tmp1 = dx1 / dy1;
  tmp2 = dx2 / dy2;
  
  /* First look if the lines are parallel */
  if (tmp1 == tmp2) {
    point.x = point.y = F_ERROR;
    return(point);
  }
  
  /* The following formulas are the solution to:
   * line1.pt1.x + lambda1*dx1 = line2.pt1.x + lambda2*dx2 and 
   * line1.pt1.y + lambda1*dy1 = line2.pt1.y + lambda2*dy2
   */
  
  lambda2 = ( (line1.pt1.y-line2.pt1.y) * tmp1 + (line2.pt1.x - line1.pt1.x)) /
    (dy2*tmp1 - dx2);
  
  point.x = line2.pt1.x + lambda2*dx2;
  point.y = line2.pt1.y + lambda2*dy2;
  
  return(point);
}


/**********************************************************************
 *  If the two points of the line are too close to each other, the    *
 *  function returns -1                                               *
 **********************************************************************/
int
cut_circle_and_line(Circle circle, LineSeg line, Point S[2])
{
  float dx, dy, mue, lambda, tmp, norm;
  Point M;
  
  M = circle.M;
  
  dx = line.pt2.x - line.pt1.x;
  dy = line.pt2.y - line.pt1.y;
  
  norm = fsqrt(SQR(dx) + SQR(dy));
  
  if (norm < EPSILON) 
    return(-1);

  dx /= norm;
  dy /= norm;
  
  /* First compute the distance between midpoint of the circle
   * and the line.
   * The following formulas are the solution to:
   * line.pt1.x + lambda*dx = M.x + mue*dy   and
   * line.pt1.y + lambda*dy = M.y - mue*dx.
   * In this solution the minimal distance between M and line is mue.
   */
  
  mue = (dy * (line.pt1.x - M.x) + dx * (M.y - line.pt1.y)) / 
    (SQR(dy) + SQR(dx));

  if (fabs(mue) > circle.rad)
    return(0);
  
  /* Now compute the two points of intersection */  
  
  tmp = fsqrt(SQR(circle.rad) - SQR(mue));
  
  
  if (dy != 0.0) {
    lambda = (M.y - mue*dx - line.pt1.y) / dy;
    
    S[0].x = line.pt1.x + (lambda - tmp) * dx;
    S[0].y = line.pt1.y + (lambda - tmp) * dy;
    if (tmp == 0.0) 
      return(1);
    else {
      S[1].x = line.pt1.x + (lambda + tmp) * dx;
      S[1].y = line.pt1.y + (lambda + tmp) * dy;
      return(2);
    }
  }
  else { 
    S[0].x = circle.M.x - tmp;
    S[0].y = S[1].y = line.pt1.y;
    if (tmp == 0.0) 
      return(1);
    else {
      S[1].x = circle.M.x + tmp;
      return(2);
    }
  }
}


/**********************************************************************/
int 
cut_circle_and_circle(Circle *c1, Circle *c2, Point *pt)
{
  float D, d1, h, angle;

  D = compute_distance (c1->M, c2->M);

  /* The distance of the circles is too big. */
  if ( D > c1->rad + c2->rad) {
    fprintf(stderr,
	    "Circles too small to cut: D %f c1 %f c2 %f  --> D - c1 - c2 %g\n",
	    D, c1->rad, c2->rad, D - c1->rad - c2->rad);
    return (0);
  }
  /* One circle is contained in the other. */
  else if ( D < fabs(c1->rad - c2->rad)) {
    if ( c1->rad < c2->rad)
      fprintf( stderr, "Circle 1 is contained in circle 2: ");
    else
      fprintf( stderr, "Circle 2 is contained in circle 1: ");
    fprintf( stderr, " D %f c1 %f c2 %f  --> c1 - c2 %g\n",
	     D, c1->rad, c2->rad, c1->rad - c2->rad);
    return (0);
  }
  
  d1 = -(SQR(c2->rad) - SQR(c1->rad) - SQR(D)) / (2.0 * D);
  
  /*  if (d1 > c1->rad) {
      fprintf(stderr, "no cut %f %f \n", d1, c1->rad);
      return(0);
      }
      */
  
  h = SQR(c1->rad) - SQR(d1);
  
  if (fabs(h) < 1.0)
    h = 0.0;
  else if (h < 0.0) {
    fprintf(stderr, "h error: d1 %f D %f c1 %f %f %f c2 %f %f %f\n", d1,D,c1->M.x, c1->M.y, c1->rad,c2->M.x, c2->M.y, c2->rad);
    return (0); }
  else
    h = fsqrt(h);

  angle = compute_angle_2p(c1->M, c2->M);

  pt[0].x = c1->M.x + fcos(angle) * d1 + fcos(DEG_90+angle) * h;
  pt[0].y = c1->M.y + fsin(angle) * d1 + fsin(DEG_90+angle) * h;


  if (h==0.0)
    /* Only one point. */
    return(1);
  
  pt[1].x = c1->M.x + fcos(angle) * d1 + fcos(-DEG_90+angle) * h;
  pt[1].y = c1->M.y + fsin(angle) * d1 + fsin(-DEG_90+angle) * h;

  return (2);
}


/**********************************************************************
 * Tests wether a point on a line specified by two points is in between
 * the two points. This works only if the point is on the line!!!!
 **********************************************************************/
BOOLEAN 
point_on_LineSeg(Point pt, LineSeg line)
{

  if (smaller( line.pt2, line.pt1))
    swap(&line);

  if (smaller( line.pt1, pt) && smaller( pt, line.pt2))
    return TRUE;
  else 
    return FALSE;
}



int
get_collpoints_circle( Circle circle_big, Circle circle_small, 
		       LineSeg line, Point coll_pt[])
{
  
  int cnt, index, coll_cnt=0; 
  Point intersect_small[2], intersect_big[2];      /* Points of intersections  */
  int no_of_intersect_small, no_of_intersect_big;  /* Number of intersections  */
  Point intersections[4];                          /* Inte. of line and circle */
  
  
  no_of_intersect_big = cut_circle_and_line(circle_big, line, intersect_big);
  
  if (no_of_intersect_big == 2) {
    if (smaller( line.pt2, intersect_big[0]) ||
	smaller( intersect_big[1], line.pt1))
      return(0);                              /* The LineSeg is not in the circle */
    
    no_of_intersect_small = cut_circle_and_line(circle_small, line, intersect_small);
             
                     
    /* The whole segment between the intersections with the big circle is in
     * the collision area
     */
    if (no_of_intersect_small < 2) {
      
      coll_pt[coll_cnt++] = (smaller( line.pt1, intersect_big[0])) ? 
	intersect_big[0] : line.pt1; 
      coll_pt[coll_cnt++] = (smaller( intersect_big[1], line.pt2)) ? 
	intersect_big[1] : line.pt2;

      return(coll_cnt);
    }

    /* The part of the line which is in the small circle is not in the 
     * collision area and we compute the collision points from the 4 intersections
     */
    intersections[0] = intersect_big[0];	
    intersections[1] = intersect_small[0];	
    intersections[2] = intersect_small[1];	
    intersections[3] = intersect_big[1];	
    
    for (cnt=0; (cnt<4) && (smaller( intersections[cnt], line.pt1)); cnt++) ;
    index = 10 * cnt;
    for ( ; (cnt<4) && (smaller( intersections[cnt], line.pt2)); cnt++) ;
    index += cnt;
    
    switch (index) {
    case 01: 
      coll_pt[coll_cnt++] = intersections[0];
      coll_pt[coll_cnt++] = line.pt2;
      break;
    case 02:
      coll_pt[coll_cnt++] = intersections[0];
      coll_pt[coll_cnt++] = intersections[1];
      break;
    case 03:
      coll_pt[coll_cnt++] = intersections[0];
      coll_pt[coll_cnt++] = intersections[1];
      coll_pt[coll_cnt++] = intersections[2];
      coll_pt[coll_cnt++] = line.pt2;
      break;
    case 04:
      coll_pt[coll_cnt++] = intersections[0];
      coll_pt[coll_cnt++] = intersections[1];
      coll_pt[coll_cnt++] = intersections[2];
      coll_pt[coll_cnt++] = intersections[3];
      break;
    case 11:
      coll_pt[coll_cnt++] = line.pt1;
      coll_pt[coll_cnt++] = line.pt2;
      break;
    case 12:
      coll_pt[coll_cnt++] = line.pt1;
      coll_pt[coll_cnt++] = intersections[1];
      break;
    case 13:
      coll_pt[coll_cnt++] = line.pt1;
      coll_pt[coll_cnt++] = intersections[1];
      coll_pt[coll_cnt++] = intersections[2];
      coll_pt[coll_cnt++] = line.pt2;
      break;
    case 14:
      coll_pt[coll_cnt++] = line.pt1;
      coll_pt[coll_cnt++] = intersections[1];
      coll_pt[coll_cnt++] = intersections[2];
      coll_pt[coll_cnt++] = intersections[3];
      break;
    case 22:
      return(0);
    case 23:
      coll_pt[coll_cnt++] = intersections[2];
      coll_pt[coll_cnt++] = line.pt2;
      break;
    case 24:
      coll_pt[coll_cnt++] = intersections[2];
      coll_pt[coll_cnt++] = intersections[3];
      break;
    case 33:
      coll_pt[coll_cnt++] = line.pt1;
      coll_pt[coll_cnt++] = line.pt2;
      break;
    case 34:
      coll_pt[coll_cnt++] = line.pt1;
      coll_pt[coll_cnt++] = intersections[3];
      break;
    }
  }
  return(coll_cnt);
} 




/**********************************************************************
 **********************************************************************
 *                         Init functions                            *
 **********************************************************************
 **********************************************************************/



/**********************************************************************/
void 
init_fast_cos(void)
{
  int i;
  double x;
  
  for (i = 0; i < 62832; i++){
    x = ((double) i) / 10000.0;
    table_cos[i] = (float) cos(x);
  }
}

/**********************************************************************/
void 
init_fast_sin(void)
{
  int i;
  double x;
  
  for (i = 0; i < 62832; i++){
    x = ((double) i) / 10000.0;
    table_sin[i] = sin(x);
  }
}


