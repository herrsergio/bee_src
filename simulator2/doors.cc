
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

#include "doors.hh"
#include "simxw.h"
#include "linalg.h"

t_door::t_door(char* tagName, float wi, float di, float hi, float apxi, float apyi, float apzi, float anglei)
{
    type = 2;
    tag = new char[strlen(tagName)+1];
    tag = strcpy( tag, tagName);
    w = fabs(wi);
    d = di;
    h = fabs(hi);
    apx = apxi;
    apy = apyi;
    apz = apzi;
    angle = anglei;
    cz = apzi;
    absolute_points();
    color = "green"; 
    //    active = TRUE;
}

void
t_door::move(float diffX, float diffY)
{
    apx += diffX;
    apy += diffY;
}


  
#define rotpnt(angle,x,y) \
{ \
      float t = x; \
    x = x * cos(angle) - y * sin(angle); \
    y = t  * sin(angle) + y * cos(angle); \
					      }
void t_door::absolute_points()
{
    if(d >= 0) {
	x[0] = w;
	y[0] = 0;
	rotpnt(angle,x[0],y[0]);
	x[1] = w;
	y[1] = d;
	rotpnt(angle,x[1],y[1]);
	x[2] = 0;
	y[2] = d;
	rotpnt(angle,x[2],y[2]);
	x[3] = 0;
	y[3] = 0;
	rotpnt(angle,x[3],y[3]);
    }
    else {
	x[0] = w;
	y[0] = d;
	rotpnt(angle,x[0],y[0]);
	x[1] = w;
	y[1] = 0;
	rotpnt(angle,x[1],y[1]);
	x[2] = 0;
	y[2] = 0;
	rotpnt(angle,x[2],y[2]);
	x[3] = 0;
	y[3] = d;
	rotpnt(angle,x[3],y[3]);
    }
	for(int i = 0; i <4; i++) {
	x[i] += apx;
	y[i] += apy;
    }
}


void t_door::save(FILE *fd)
{
  if ( strcmp( tag, dummyTag)) {
    if(cz == DEF_Z && h == DEF_H)
      fprintf(fd, "Door %f %f %f %f %f %s\n", apx, apy, w, d, angle, tag);
    else
      fprintf(fd, "3D-Door %f %f %f %f %f %f %f %s\n", apx, apy, apz, w, d, h, angle,tag);
  }
  else {
    if(cz == DEF_Z && h == DEF_H)
      fprintf(fd, "Door %f %f %f %f %f\n", apx, apy, w, d, angle);
    else
      fprintf(fd, "3D-Door %f %f %f %f %f %f %f\n", apx, apy, apz, w, d, h, angle);
  }
}

void t_door::mouse_action(float x, float y)
{
    float m;
    last_angle = angle;    
    if( x == apx ) {
	angle = -M_PI_2;
        if(y > apy) angle *= -1;
    }
    else {
	m = (apy - y) / (apx-x);
	angle = atan(m);
	if(x < apx) angle += M_PI;
    }
    redraw();
}
