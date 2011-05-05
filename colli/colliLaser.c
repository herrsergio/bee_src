
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
#include "laser_interface.h"

LaserPoints Laser_CollPoints;

#define MAX_NUMBER_OF_LASER_POINTS 500
#define MAX_REMEMBER_TIME 10.0

#define NUMBER_OF_COLLPOINTS_PER_LASER 45

float LASER_UPDATE_TIME = 2.5;

static int laserVerbose = FALSE;

/***************************************************************************
  * forward declarations.
 ***************************************************************************/
static Point
laserCenter( Point rpos, float rRot,
	     float laserOffset, float laserAngle);

static void
setLaserPoints( float laserOffset, float laserAngle,
		LASER_reading readings);

/***************************************************************************
  * Initializes all values necessary in the structure for the laser points. 
 ***************************************************************************/
void
initLaserPointsStructure()
{
  int i;
  struct timeval now;

  gettimeofday( &now, 0);
    
  Laser_CollPoints.maxNumberOfPoints = MAX_NUMBER_OF_LASER_POINTS;
  Laser_CollPoints.numberOfPoints    = 0;
  Laser_CollPoints.maxRememberTime = MAX_REMEMBER_TIME;
  
  Laser_CollPoints.points = (struct Point*)
    malloc( MAX_NUMBER_OF_LASER_POINTS * sizeof(struct Point));
  
  Laser_CollPoints.times = (struct timeval*)
    malloc( MAX_NUMBER_OF_LASER_POINTS * sizeof(struct timeval));
  
  for ( i = 0; i < MAX_NUMBER_OF_LASER_POINTS; i++) {
    Laser_CollPoints.points[i].x = F_ERROR;
    Laser_CollPoints.times[i] = now;
  }
}


/**********************************************************************
 * Updates the structure containing the points of the obstacles as detected
 * by the laser range finders. 
 **********************************************************************/
void
update_LaserPoints( Pointer callback_data, Pointer client_data)
{

  if ( frontLaserReading.new) {

    if ( laserVerbose) {
      struct timeval now;
      static struct timeval last;

      gettimeofday( &now, 0);

      fprintf(stderr, "td %f\n", timeDiff( &now, &last));
      gettimeofday( &last, 0);
    }
      
    setLaserPoints( FRONT_LASER_OFFSET, FRONT_LASER_ANGLE,
		    frontLaserReading);

    frontLaserReading.new = FALSE;
    
    Last_FrontLaserPoint_Update = frontLaserReading.time;
  }
  
  if ( rearLaserReading.new) {
    
    setLaserPoints( REAR_LASER_OFFSET, REAR_LASER_ANGLE,
		    rearLaserReading);

    rearLaserReading.new = FALSE;
    
    Last_RearLaserPoint_Update = rearLaserReading.time;
  }

  COLLI_update_tcx_LaserPoints();
}

/***************************************************************************
 * The center points of the two laser scanners given the robot's position.
 ***************************************************************************/
static Point
laserCenter( Point rPos, float rRot,
	     float laserOffset, float laserAngle)
{
  Point center;

  center.x = rPos.x + (fcos( rRot + laserAngle) * laserOffset);
  center.y = rPos.y + (fsin( rRot + laserAngle) * laserOffset);

  return center;
}


/******************************************************************
 * Converts the distances in <readings> into absolute points, given
 * the robot position and the position of the laser.
 ******************************************************************/
static void
setLaserPoints( float laserOffset, float laserAngle,
		LASER_reading readings)
{
  int actualCnt, combineCnt, minIndex = -1;
  int lastDefined;
  struct timeval now;
  BOOLEAN verbose = FALSE;
  Point lCenter;
  float lRot, minReading = LASER_MAX_RANGE;
  int pointsToCombine = readings.numberOfReadings / NUMBER_OF_COLLPOINTS_PER_LASER;

  /* No time check in the first call. */
  static BOOLEAN firstTime = TRUE;

  /* Delete all points in the actual scan area. This area is in front of
   * the laser center position. We substract one cm because of rounding
   * errors. */
  Point scanAreaCenter   = laserCenter( readings.rPos, readings.rRot,
					laserOffset - 1.0, laserAngle);
  
  float scanAreaRotation = readings.rRot + laserAngle;

  gettimeofday( &now, 0);

  /* First check which points to delete:
   * Delete all points in the actual scan area
   * Delete all points which are too old.
   */
  for ( actualCnt = 0, lastDefined = 0;
	actualCnt < Laser_CollPoints.numberOfPoints;
	actualCnt++) {
  
    /* Too old? */
    if ( firstTime
	 || timeDiff( &now, &(Laser_CollPoints.times[actualCnt])) 
	 <= Laser_CollPoints.maxRememberTime) {

      /* All points behind the laser center have to be remembered. */
      if ( behindPoint( scanAreaCenter, scanAreaRotation,
			Laser_CollPoints.points[actualCnt])) {
	Laser_CollPoints.points[lastDefined] = Laser_CollPoints.points[actualCnt];
	Laser_CollPoints.times[lastDefined]  = Laser_CollPoints.times[actualCnt];
	lastDefined++;
	if (verbose) fprintf( stderr, "keep   : %d\n", actualCnt);
      }
      else
	if (verbose) fprintf( stderr, "Remove : %d\n", actualCnt);
    }
    else {
      if (verbose) fprintf( stderr, "Too old: %d\n", actualCnt);
    }
  }

  firstTime = FALSE;

  /* Now insert the actual readings. */
  lCenter = laserCenter( readings.rPos, readings.rRot,
			     laserOffset, laserAngle);
  
  lRot    = readings.rRot;
  
  /* actualCnt points to the actual readings.
   * lastDefined points to the laser points. */   
  for ( actualCnt = 0, combineCnt = 0;
	actualCnt < readings.numberOfReadings;
	actualCnt++, combineCnt++) {

    if ( (combineCnt > 0) && combineCnt % pointsToCombine == 0) {

      /* We found a minimal reading in the last combineCnt readings. */
      if ( minIndex > 0) {
	
	float beamRot = 
	  lRot + readings.startAngle + readings.angleResolution * minIndex;

	if ( ( lastDefined) < Laser_CollPoints.maxNumberOfPoints) {
	  Laser_CollPoints.points[lastDefined].x =
	    lCenter.x + (fcos( beamRot) * readings.reading[minIndex]);
	  Laser_CollPoints.points[lastDefined].y =
	    lCenter.y + (fsin( beamRot) * readings.reading[minIndex]);
	  Laser_CollPoints.times[lastDefined] = now;
	  lastDefined++;
	}
	else {
	  fprintf( stderr, "Too many readings (max: %d).\n",
		   Laser_CollPoints.maxNumberOfPoints);
	  break;
	}
      }
      minIndex = -1;
      minReading = LASER_MAX_RANGE;
    }
    
    if ( readings.reading[actualCnt] > 0.0
	 && readings.reading[actualCnt] <= minReading) {
      minIndex = actualCnt;
      minReading = readings.reading[actualCnt];
    }
  }

  Laser_CollPoints.numberOfPoints = lastDefined ;
  if ( verbose) fprintf( stderr, "remembered points %d\n", lastDefined);
}

