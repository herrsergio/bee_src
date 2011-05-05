
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
#include "circle.hh"
#include "linalg.h"
#include "simxw.h"

extern t_surface *default_surface;

void
t_circle::bounds(float *xi1, float *yi1, float *xi2, float *yi2)
{
  *xi1 = cx - r;
  *yi1 = cy - r;
  *xi2 = cx + r;
  *yi2 = cy + r;
}

Boolean
t_circle::inside(float x, float y)
{
  float dx = cx - x;
  float dy = cy - y;
  return( (dx*dx + dy*dy) <= r*r );
}

void
t_circle::expose()
{
  myfig = expose_circle(cx,cy,r, color);  
}

void
t_circle::toggle()
{
    if(enabled) {
	RemoveFigure(myfig);
	myfig = expose_disabled_circle(cx,cy,r,color);
	enabled = FALSE;
    }
    else {
	RemoveFigure(myfig);
	myfig = expose_circle(cx,cy,r,color);
	enabled = TRUE;
    }
}

void
t_circle::redraw()
{
  redraw_circle(myfig,cx,cy,r);           
}

void
t_circle::save(FILE *fd)
{
  if ( strcmp( tag, dummyTag))
    fprintf(fd, "CYLINDER %f %f %f %f %f %s\n",cx,cy,cz,r,h,tag);
  else
    fprintf(fd, "CYLINDER %f %f %f %f %f\n",cx,cy,cz,r,h);
}

t_circle::t_circle(char* tagName, float xi1, float yi1, float zi, float ir, float hi)
{
  type = 1;
  tag = new char[strlen(tagName)+1];
  strcpy( tag, tagName);
  cx = xi1;
  cy = yi1;
  cz = zi;
  r = ir;
  h = hi;
  my_surface = default_surface;
  color = WALLCOLOR;
  enabled = TRUE;
  active = FALSE;
}

Boolean
t_circle::distance(float xs, float ys, float zs, float open_angle,
			   float xe, float ye,
			   float *dist, float *angle, t_surface **surface)
{
    Boolean C_HIT;
    *surface = my_surface;
    C_HIT = cut_circle_and_line(cx,cy,r,xs,ys,xe,ye,dist,angle);
    if(C_HIT) {                                      
	if(!( cz-h/2 <= zs && cz+h/2 >= zs)) {
	    if(*dist == 0.0) {
		C_HIT = FALSE;
	    }
	    else {
		float dz, ta;
		if(cz+h/2 < zs) 
		    dz = zs - (cz+h/2);
		else
		    dz = (cz-h/2) - zs;
		ta = dz / *dist;
		if(ta > tan(open_angle)) {
		    C_HIT = FALSE;
		}
		else {
		    *dist = sqrt(dz*dz+ *dist * *dist);
		}
	    }
	}
    }
    return C_HIT;
}

float
t_circle::min_distance(float xp, float yp)
{
  return 3000;
}



