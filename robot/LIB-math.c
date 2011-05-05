
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




#ifdef VMS
#include "vms.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Common.h"

/***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include "beeSoftVersion.h"

int  librobot_major      = BEESOFT_VERSION_MAJOR;
int  librobot_minor      = BEESOFT_VERSION_MINOR;
int  librobot_robot_type = BEESOFT_VERSION_ROBOT_TYPE;
char librobot_date[80]   = BEESOFT_VERSION_DATE;


/***************************************************************************
 ***************************************************************************
 ***************************************************************************/




#define _2pi (double)(2.0 * PI)

double _0_to_2pi (double rad)
{
  for(;;) {
    if (rad >= _2pi)
      rad -= _2pi;
    else if (rad < 0.0)
      rad += _2pi;
    else
      return (rad);
  }
}

double  _pi_to_pi (double rad)
{
  for(;;) {
    if (rad > PI)
      rad -= _2pi;
    else if (rad <= - PI)
      rad += _2pi;
    else
      return (rad);
  }
}

double _0_to_360 (double deg)
{
  for(;;) {
    if (deg >= 360.0)
      deg -= 360.0;
    else if (deg < 0.0)
      deg += 360.0;
    else
      return (deg);
  }
}


/*
  double _180_to_180 (double deg)
  {
  for(;;) {
  if (deg > 180.0)
  deg -= 360.0;
  else if (deg <= -180.0)
  deg += 360.0;
  else
  return (deg);
  }
  }
  */
double floatMod (double num1, double num2)
{
  if( num2 == 0 ) {
    fprintf( stderr, "floatMod: taking mod by 0\n" );
    return(0.0);
  }
  return (num1 - ((int)(num1/num2)) * num2);
}
