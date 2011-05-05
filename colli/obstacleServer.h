
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



#ifndef OBSTACLE_SERVER_INCLUDE
#define OBSTACLE_SERVER_INCLUDE

#include "BASE-messages.h"

#define TRUE 1
#define FALSE 0

typedef struct {
  float x;
  float y;
  float rot;
} realPosition;

typedef struct {
  float x;
  float y;
  float rot;
  int type;
} correctionParameter;

#define DEG_90 (1.5707963705)

float
DegToRad(float x);

float
RadToDeg(float x);

float
fSqr( float x);

int
obstacleInDirection( realPosition mapPos,
		     realPosition robPos,
		     float relativeAngle,
		     obstaclePoint* point,
		     correctionParameter* correction);

extern realPosition robotPosition;

#endif


