
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
#include <sys/time.h>

#include "simxw.h"
#include "human.hh"
#include "schedule.hh"
#include "store.hh"
#include "playground.hh"

extern t_obstgrid *obstore;

#define HUMAN_H 180.0
#define HUMAN_Z (HUMAN_H/2.0)

t_human::t_human(char* tagName, float px, float py, float s, float a)
  : t_circle(tagName, px, py, HUMAN_Z, HUMAN_RADIUS, HUMAN_H)
{
  type = 3;
  speed = s;
  angle = a;
  active = TRUE;
  color = "pink";
  start_pos_x = px;
  start_pos_y = py;
  stopped = 0;
  gettimeofday(&last_update, NULL);
}

void
t_human::update(struct timeval *now)
{
  struct timeval tdiff;
  float secs, trans;
  float nx,ny, ex,ey,dist;
  
  if(stopped) return;
  
  time_subtract(&tdiff, now, &last_update);
  secs = (float) timeval_to_usec(&tdiff) / 1000000.0;
  trans = secs * speed;
  nx = cx + trans*cos(angle);
  ny = cy + trans*sin(angle);
  ex = nx + HUMAN_RADIUS*cos(angle);
  ey = ny + HUMAN_RADIUS*sin(angle);
  if(get_distance(EXACT_SENSOR,
		  base_x(cx),base_y(cy),
		  0.0, M_PI - 0.00001,
		  base_x(ex),base_y(ey),&dist)) {
      cx = cx+(dist-HUMAN_RADIUS)*cos(angle);
      cy = cy+(dist-HUMAN_RADIUS)*sin(angle);
      angle += M_PI;  // turn around
      if(angle > 2*M_PI) angle -= 2*M_PI;
  }
  else {
      cx = nx;
      cy = ny;
  }
  last_update.tv_sec = now->tv_sec;
  last_update.tv_usec = now->tv_usec;  
}

void
t_human::save(FILE *fd)
{
  if ( strcmp( tag, dummyTag))
    fprintf(fd, "Human %f %f %f %f %s\n", cx, cy, speed, angle,tag);
  else
    fprintf(fd, "Human %f %f %f %f\n", cx, cy, speed, angle);
}
