
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


#include <math.h>

float sin_tab[720];
float cos_tab[720];
float tan_tab[720];

float _mySIN(float r)
{
  float d = r * 180.0 / M_PI;
  int d2;
  d2 = (int)(2*d);
  return sin_tab[d2];
}

float _myCOS(float r)
{
  float d = r * 180 /M_PI;
  int d2 = (int)(2*d);
  return cos_tab[d2];
}

void initTrigFkt()
{
  float f;
  for(f = 0; f< 360; f += 0.5 ) {
    sin_tab[(int)(f*2)] = sin(f*M_PI/180);
    cos_tab[(int)(f*2)] = cos(f*M_PI/180);
  }
  for(f = -90; f <90; f+=0.5) {
    tan_tab[(int)(2*(270+f))] =
      tan_tab[(int)(2*(90+f))] = tan(f*M_PI/180);
  }
}

float _myTAN(float r)
{
  float d = r * 180/M_PI;
  int d2 = (int)(2*(90+d));
  return tan_tab[d2];
}

