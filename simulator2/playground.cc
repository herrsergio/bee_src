
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


#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "playground.hh"
#include "obstacles.hh"
#include "surface.hh"
#include "store.hh"
#include "robot.h"
#include "simxw.h"

t_obstgrid* obstore;
t_playground* map;
extern t_surface *default_surface;

#define POSSIBLE_FILES 3 
char *p_filename[] = {"sim.map","test.map","default.map"};


/*
 * This defines the size of the environment. Increase, if you need larger
 * maps.
 */
#define BORDER_SPAREROOM 2500.0

t_playground::t_playground(char *fname, int argc, char **argv)
{
    int i;
    float xin,yin,ori_in;
    char def_mapfile[80];
  FILE* mapfd = NULL;
  enabled = FALSE;

  x1 = 0;                           // default playground bounds
  y1 = 0;
  x2 = 0;
  y2 = 0;

  xin = 0;                       // default robot position
  yin = 0;
  ori_in = 0;
                                    // mapfile specified on commandline
  if(strlen(fname) != 0) {        
      mapfd = fopen(fname, "r");
                                    // not in pwd look in data dir.
      if(!mapfd) {                
	  sprintf(def_mapfile, "%s/bee/data/simulator/%s",
		  getenv("HOME"),fname);
	  mapfd = fopen(def_mapfile, "r");
	  if(mapfd) strcpy(filename, def_mapfile);
      }
      else strcpy(filename,fname);
      if(!mapfd) fprintf(stderr, "Could not find mapfile: %s\n", fname);
  }
				// no filename given, try default maps
  else {
				// try pwd first
    for(i = 0; i < POSSIBLE_FILES; i++) {
      if((mapfd = fopen(p_filename[i], "r")))
	  {   strcpy(filename, p_filename[i]);
	      printf("Loaded default map %s\n",filename);
	      break;
	  }
    }
				// not in pwd try data directory
    if(!mapfd) {
	for(i = 0; i < POSSIBLE_FILES; i++) {
	    sprintf(def_mapfile, "%s/bee/data/simulator/%s",
		    getenv("HOME"), p_filename[i]);
	    if((mapfd = fopen(def_mapfile, "r")) != 0) {
		strcpy(filename, def_mapfile);
		printf("Loaded default map %s\n",filename);
		break;
	    }
	}
    }
				// still not found, print files tried
    if(!mapfd) {
	fprintf(stderr, "Simulator: Could not find any map!\n");
	fprintf(stderr, "I tried\n");
	for(i = 0; i < POSSIBLE_FILES; i++) {
	    fprintf(stderr, "\t%s\n", p_filename[i]);
	}      
	for(i = 0; i < POSSIBLE_FILES; i++) {
	    fprintf(stderr, "\t%s/bee/data/simulator/%s\n",
		    getenv("HOME"),p_filename[i]);
	}      
    }
  }
				// try to start with map
  if(mapfd) {
      i = fscanf(mapfd, "MAP %f %f %f %f\n", &x1, &y1, &x2, &y2);
      if( i < 4) {
	  fprintf(stderr, "Simulator: Mapfile corrupted!\n");
	  exit(0);
      }
      i = fscanf(mapfd, "ROBOT %f %f %f\n", &xin, &yin,
                 &ori_in);
      if( i < 3) {
	  fprintf(stderr, "Simulator: Mapfile corrupted!\n");
	  exit(0);
      }
      x1 -= BORDER_SPAREROOM;
      y1 -= BORDER_SPAREROOM;
      y2 += BORDER_SPAREROOM;
      x2 += BORDER_SPAREROOM;
      obstore = new t_obstgrid(x1, y1, x2, y2);  
      obstore->read_obstacles(mapfd);
      fclose(mapfd);
  }
				// start without map
  else {
      x1 -= BORDER_SPAREROOM;
      y1 -= BORDER_SPAREROOM;
      y2 += BORDER_SPAREROOM;
      x2 += BORDER_SPAREROOM;      
      obstore = new t_obstgrid(x1,y1,x2,y2);  
  }
  InitPlayground();
  setRobotPosition(xin-x1, yin-y1, ori_in);
  printf("Number of Obstacles: %d\n",obstore->n_obstacles);
  return;
}

void t_playground::InitPlayground()
{
    activated_obstacle = NULL;
  InitPlaygroundGraphics(filename,x1,y1,x2,y2);
  obstore->ExposeObstacles();
  return;
}

Boolean SavePlayground(char *fname)
{
    map->_SavePlayground(fname);
}


Boolean t_playground::_SavePlayground(char *fname)
{
    FILE *fd;
    float rx,ry,rori;
    float nx1,ny1,nx2,ny2;
    getRobotPosition(&rx,&ry,&rori);
    rx = playground_x(rx);
    ry = playground_y(ry);
    strcpy(filename, fname);
    obstore->move(0.0,0.0);              // force update of bounding box 
    obstore->bounds(&nx1, &ny1, &nx2, &ny2);
    if(nx1 > rx) nx1 = rx;
    if(ny1 > ry) ny1 = ry;
    fprintf(stderr, "bounds are: %g %g %g %g\n",nx1,ny1,nx2,ny2);
    obstore->move(-nx1,-ny1);
    rx = rx - nx1;
    ry = ry - ny1;    
    obstore->bounds(&nx1, &ny1, &nx2, &ny2);
    if((fd = fopen(filename, "w"))) {
	fprintf(fd, "MAP %f %f %f %f\n",
		nx1,
		ny1,
		nx2,
		ny2);
	fprintf(fd, "ROBOT %f %f %f\n",
		rx, ry, rori);
	obstore->SaveObstacles(fd);
	fclose(fd);
	_LoadPlayground(fname);
	return TRUE;
    }
    return FALSE;
}

void LoadPlayground(char *fname)
{
    map->_LoadPlayground(fname);
}

void
t_playground::_LoadPlayground(char *fname)
{
    int i;
    float xin = 0, yin = 0, ori_in = 0;
    t_obstgrid *obstore2;
    FILE *mapfd = fopen(fname, "r");
    x1 = 0;                           // default playground bounds
    y1 = 0;
    x2 = 2000;
    y2 = 2000;
    if(!mapfd) {
	printf("Could not open mapfile, keeping old map\n");
	return;
    }
    strcpy(filename, fname);
    fprintf(stderr,"opened mapfile %s\n",fname);
    i = fscanf(mapfd, "MAP %f %f %f %f\n", &x1, &y1, &x2, &y2);
    if( i < 4) {
	fprintf(stderr, "Simulator: Mapfile corrupted, keeping old map\n");
	return;
    }
    i = fscanf(mapfd, "ROBOT %f %f %f\n", &xin, &yin, &ori_in);
    if( i < 3) {
	fprintf(stderr, "Simulator: Mapfile corrupted, keeping old map\n");
	return;
    }
    x1 -= BORDER_SPAREROOM;
    y1 -= BORDER_SPAREROOM;
    y2 += BORDER_SPAREROOM;
    x2 += BORDER_SPAREROOM;
    obstore2 = new t_obstgrid(x1,y1,x2,y2);  
    if(!obstore2->read_obstacles(mapfd)) {
	delete obstore2;
	fprintf(stderr, "Simulator: Mapfile corrupted, keeping old map\n");
	return;
    };
    fclose(mapfd);
    delete obstore;
    obstore = obstore2;
    InitPlayground();
    setRobotPosition(xin-x1, yin-y1, ori_in);
    DrawRobot(xin, yin);
    printf("Number of Obstacles: %d\n",obstore->n_obstacles);
}    

Boolean t_playground::_get_distance(float PosX, float PosY, float PosZ,
				    float open_angle,
				    float EndX, float EndY, 
				    float *dist, float *angle, t_surface **surface)
{
    t_obst* obst;
    return obstore->distance(playground_x(PosX),
			     playground_y(PosY),
			     PosZ,
			     open_angle,
			     playground_x(EndX), 
			     playground_y(EndY),
			     dist, angle, surface);
}

void t_playground::set_active_obstacle(t_obst* obst)
{
    activated_obstacle = obst;
}
t_obst* t_playground::get_active_obstacle()
{
    return activated_obstacle;
}

Boolean InsideObstacle(float x, float y)
{
  if(obstore->InsideObstacle(x,y)) return TRUE;
  else return FALSE;
}

Boolean InsideAnyObstacle(float x, float y)
{
  if(obstore->InsideAnyObstacle(x,y)) return TRUE;
  else return FALSE;
}

void
RemoveObstacle(float x, float y)
{
    t_obst* obst = obstore->InsideAnyObstacle(x,y);
    if(obst) { 
	obstore->RemoveObstacle(obst);    
    }
}

void
ToggleObstacle(float x, float y)
{
    t_obst* obst = obstore->InsideAnyObstacle(x,y);
    if(obst) 
	obst->toggle();    
}

void
ObstaclePushAction(float x, float y)
{
    t_obst* obst;
    map->set_active_obstacle(NULL);
    if((obst = obstore->InsideAnyObstacle(x,y))) {
	map->set_active_obstacle(obst);
	obst->mouse_action(x, y);
	return;
    }
}
void
ObstacleDragAction(float x, float y)
{
    t_obst* obst;
    if((obst = map->get_active_obstacle())) {
	obst->mouse_action(x,y);
    }
}

void new_rectangle(float x1, float y1, float x2, float y2)
{
    obstore->new_rectangle(x1,y1,x2,y2);
}
void new_door(float x1, float y1, float x2, float y2)
{
    obstore->new_door(x1,y1,x2,y2);
}
void new_circle(float x1, float y1, float r)
{
    obstore->new_circle(x1,y1,r);
}
void new_human(float x1, float y1, float s, float a)
{
    obstore->new_human(x1,y1,s,a);
}

float obstacles_min_distance(float x, float y)
{
    return obstore->min_distance(x,y);
}

Boolean get_distance(int sensor_type,
		     float PosX, float PosY,
		     float PosZ,
		     float open_angle,
		     float EndX, float EndY, float *dist)
{
    float angle;
    t_surface *surface;
    float tdist;
    Boolean Hit = map->_get_distance(PosX, PosY, PosZ,
				     open_angle,
				     EndX, EndY, &tdist,
				     &angle, &surface);
    if(!Hit) return FALSE;
    *dist = surface->add_error(sensor_type, tdist, angle);
    return TRUE;
}

Boolean map_enabled()
{
    return map->enabled;
}

float playground_x(float xin)
{
    return xin+map->x1;
}
float playground_y(float yin)
{
    return yin+map->y1;
}
float base_x(float xin)
{
    return xin-map->x1;
}
float base_y(float yin)
{
    return yin-map->y1;
}
