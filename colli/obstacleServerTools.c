
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


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>

#include "obstacleServer.h"
#ifdef UNIBONN
#include "localize.h"
#endif
#include "fakeSensors.hh"

#define BEAM_LENGTH 500.0


/***************************************************************************
 * Some auxiliary functions. 
 ***************************************************************************/
float
DegToRad(float x)
{
  return x * 0.017453293;
}


float
RadToDeg(float x)
{
  return x * 57.29578;
}

float
fSqr( float x)
{
  return x * x;
}

/***************************************************************************
 * Checks wether an obstacle is in the given direction. If so the
 * collision point with the obstacle is stored in the point. *
 ***************************************************************************/
int
obstacleInDirection( realPosition mapPos,
		     realPosition robPos,
		     float relativeAngle,
		     obstaclePoint* point,
		     correctionParameter* corr)
{

  /* Compute the end point of the 'beam'. */
  float absoluteAngle = mapPos.rot + relativeAngle;
  float endX = mapPos.x + BEAM_LENGTH * cos( absoluteAngle);
  float endY = mapPos.y + BEAM_LENGTH * sin( absoluteAngle);

  float distance;

  if ( getDistance( mapPos.x, mapPos.y, 35.0, endX, endY, &distance)) { 
    

    /* Set the corresponding point relative to the robot's current position. */
    point->x = robPos.x + distance * cos( robPos.rot + relativeAngle);
    point->y = robPos.y + distance * sin( robPos.rot + relativeAngle);

#ifdef SERVER_debug
    fprintf( stderr, "dist %f %f %f  --> %f ( %f %f %f)  --> %d %d\n",
	     mapPos.x, mapPos.y, RadToDeg(mapPos.rot),
	     distance,
	     robPos.x, robPos.y, robPos.rot,
	     point->x, point->y);

#endif
    
    return TRUE;
  }
  else
    return FALSE;
}
     










