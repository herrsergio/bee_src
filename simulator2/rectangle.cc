
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
#include <values.h>
#include "rectangle.hh"
#include "simxw.h"
#include "linalg.h"
#include "trigofkt.h"

extern Boolean mark_option;
extern t_surface *default_surface;

void
t_rectangle::newBounds(float *xi1, float *yi1, float *xi2, float *yi2)
{
  float min_x = MAXFLOAT;
  float max_x = MINFLOAT;
  float min_y = MAXFLOAT;
  float max_y = MINFLOAT;
  absolute_points();
  for(int i = 0; i< 3; i++) {
    if( x[i] < min_x) min_x = x[i];
    if( x[i] > max_x) max_x = x[i];
    if( y[i] < min_y) min_y = y[i];
    if( y[i] > max_y) max_y = y[i];
  }
  last_ori = ori;
  *xi1 = x1 = min_x;
  *yi1 = y1 = min_y;
  *xi2 = x2 = max_x;
  *yi2 = y2 = max_y;
}

void
t_rectangle::bounds(float *xi1, float *yi1, float *xi2, float *yi2)
{
  if( ori != last_ori) {
    newBounds(&x1, &y1, &x2, &y2);
  }
  *xi1 = x1;
  *xi2 = x2;
  *yi1 = y1;
  *yi2 = y2;
}


void
t_rectangle::move(float dx, float dy)
{
  cx += dx;
  cy += dy;
  last_ori  = abs_ori = ori - 1;
}

static void
rotpnt(float angle, float &x, float &y) 
{ 
  float t = x;
  x = x * cos(angle) - y * sin(angle);
  y = t  * sin(angle) + y * cos(angle); 
}

void t_rectangle::absolute_points()
{
  if(ori == abs_ori) return;
    x[0] = w/2;
    y[0] = -d/2;
    rotpnt(ori,x[0],y[0]);
    x[1] = w/2;
    y[1] = +d/2;
    rotpnt(ori,x[1],y[1]);
    x[2] = -w/2;
    y[2] = d/2;
    rotpnt(ori,x[2],y[2]);
    x[3] = -w/2;
    y[3] = -d/2;
    rotpnt(ori,x[3],y[3]);
    for(int i = 0; i <4; i++) {
	x[i] += cx;
	y[i] += cy;
    }
  abs_ori = ori;
}

Boolean t_rectangle::inside(float xi, float yi)
{
    int i;
    absolute_points();
    for(i = 0; i < 4; i++) {
	if(!inhalfplane(xi,yi,x[i],y[i],x[(i+1)%4],y[(i+1)%4]))
	    return FALSE;
    }
    return TRUE;
}

void t_rectangle::expose()
{
    absolute_points();
    myfig = expose_polygon(4,x,y,color);
    if(mark_option)
      place_dot(cx, cy, d, ori);
}

void t_rectangle::toggle()
{
    if(enabled) {
	RemoveFigure(myfig);
	absolute_points();
	myfig = expose_disabled_polygon(4,x,y,color);
	enabled = FALSE;
    }
    else {
	RemoveFigure(myfig);
	absolute_points();
	myfig = expose_polygon(4,x,y,color);
	enabled = TRUE;
    }
}

void t_rectangle::save(FILE *fd)
{
  if ( strcmp( tag, dummyTag))
    fprintf(fd, "CUBE %f %f %f %f %f %f %f %s\n",cx,cy,cz,w,d,h,ori,tag);
  else
    fprintf(fd, "CUBE %f %f %f %f %f %f %f\n",cx,cy,cz,w,d,h,ori);
}

void t_rectangle::redraw()
{
    absolute_points();
    if(enabled)
	myfig = redraw_polygon(myfig,4,x,y);
    else
	myfig = redraw_disabled_polygon(myfig,4,x,y);
}

t_rectangle::t_rectangle()
{
    my_surface = default_surface;
    enabled = TRUE;
    active = FALSE;
}

t_rectangle::t_rectangle( char* tagName, float cxi, float cyi, float czi,
			  float wi, float di, float hi, float orii)
{
  type = 0;
  tag = new char[strlen(tagName)+1];
  strcpy( tag, tagName);
  w = fabs(wi);
  d = fabs(di);
  h = fabs(hi);
  cx = cxi;
  cy = cyi;
  cz = czi;
  x1 = cx - w/2;
  y1 = cy - d/2;  
  x2 = cx + w/2;
  y2 = cy + d/2;  
  ori = last_ori = orii;
  abs_ori = ori+1;		// ensure to be unequal  
  color = WALLCOLOR;
  my_surface = default_surface;
  enabled = TRUE;
  active = FALSE;
}

Boolean t_rectangle::distance(float xr, float yr, float zr, float open_angle,
			      float xe, float ye,
			   float *dist, float *angle, t_surface **surface)
{
    Boolean Hit = FALSE,C_Hit = FALSE;
    float c_dist,c_angle;
    absolute_points();
    for(int i = 0; i < 4; i++) {
    //check if this wall is seen by the sensor at (xr,yr)	
	if( ! (point_on_left_side(cx, cy, x[i],y[i], x[(i+1)%4],y[(i+1)%4]) ==
	       point_on_left_side(xr, yr, x[i],y[i], x[(i+1)%4],y[(i+1)%4]))) {
	    C_Hit = cut_line_and_line(xr,yr,
				      xe, ye,
				      x[i],y[i],
				      x[(i+1)%4],y[(i+1)%4],
				      &c_dist, &c_angle);
	}
	else {
	    C_Hit = FALSE;
	}
	if(C_Hit) {                                      
	    // check if we still hit the obstacle if it is not directly infront
	    // of the sensor
	    if(!( cz-h/2 <= zr && cz+h/2 >= zr)) {
		if(c_dist == 0.0) {
		    C_Hit = FALSE;
		}
		else {
		    float dz, ta;
		    if(cz+h/2 < zr) 
			dz = zr - (cz+h/2);
		    else
			dz = (cz-h/2) - zr;
		    ta = dz/c_dist;
		    if(ta > tan(open_angle)) {
			C_Hit = FALSE;
		    }
		    else {
			c_dist = sqrt(dz*dz+c_dist*c_dist);
		    }
		}
	    }
	}
	if(C_Hit) {
	    if(Hit) {
		if (c_dist < *dist) {
		    *dist = c_dist;
		    *angle = c_angle;
		}
	    }
	    else {
		*dist = c_dist;
		*angle = c_angle;
		Hit = TRUE;
	    }
	}
	
    }
    *surface = my_surface;
    return Hit;   
}

float t_rectangle::min_distance(float xi, float yi)
{
    float dist, tmp;
    dist = MAXFLOAT;
    absolute_points();
    for(int i = 0; i < 4; i++) {
	tmp = line_min_distance(x[i],y[i],x[(i+1)%4],y[(i+1)%4],xi,yi);
	if(tmp < dist) dist = tmp;
    }
    return dist;
}    
