
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


#include "simxw.h"

#include "obstacles.hh"
#include "rectangle.hh"
#include "circle.hh"
#include "doors.hh"
#include "human.hh"

#define N_OBSTACLE_TYPES 7
char* o_types[N_OBSTACLE_TYPES] ={
    "RECTANGLE",
    "CIRCLE",
    "Door",
    "Human",
    "CUBE",
    "CYLINDER",
    "3D-Door"
};

#define MAX_LINE_LENGTH 255
char* dummyTag = "UNKNOWN";
char commentSign = '#';

t_obst* install_obstacle(int type, FILE* mapfd)
{
    int k;
    float x,y,z,w,d,h,r,angle;
    char line[MAX_LINE_LENGTH];
    char tagString[MAX_LINE_LENGTH];
    char* tagName = tagString;

    // Read in the whole line. 
    fgets( line, sizeof( line), mapfd);
    switch(type) {
    case 0:			// RECTANGLE
	k = sscanf( line, "%f %f %f %f %f %s",&x,&y,&w,&d,&angle, tagName);
	if(k < 4) { printf("mapfile corrupted\n"); exit(0);}
	if(k < 6 || *tagName == commentSign) tagName = dummyTag;
	return new t_rectangle(tagName,x,y,DEF_Z,w,d,DEF_H);
    case 1:			// CIRCLE
	k = sscanf( line, "%f %f %f %s", &x,&y,&r,tagName);
	if(k < 3) { printf("mapfile corrupted\n"); exit(0);}
	if(k < 4 || *tagName == commentSign) tagName = dummyTag;
	return new t_circle(tagName,x,y,DEF_Z,r,DEF_H);
    case 2:
    {
	k = sscanf( line, "%f %f %f %f %f %s", &x, &y, &w, &d, &angle, tagName);
	if(k < 5) { printf("mapfile corrupted\n"); exit(0);}
	if(k < 6 || *tagName == commentSign) tagName = dummyTag;
	return new t_door(tagName,w,d,DEF_H,x,y,DEF_Z,angle);
    }
    case 3:
    {
	float s,a;
	k = sscanf( line, "%f %f %f %f %s", &x, &y, &s, &a, tagName);
	if(k < 4) { printf("mapfile corrupted\n"); exit(0);}
	if(k < 5 || *tagName == commentSign) tagName = dummyTag;
	return new t_human(tagName,x,y,s,a);
    }
    case 4:			// CUBE
      k = sscanf( line, "%f %f %f %f %f %f %f %s",&x,&y,&z,&w,&d,&h,&angle,tagName);
      if(k < 6) { printf("mapfile corrupted\n"); exit(0);}
      if(k < 8 || *tagName == commentSign) tagName = dummyTag;
      if(k >= 7)
	return new t_rectangle(tagName,x,y,z,w,d,h,angle);
      else
	return new t_rectangle(tagName,x,y,z,w,d,h);
    case 5:			// CYLINDER
	k = sscanf( line, "%f %f %f %f %f %s", &x,&y,&z,&r,&h, tagName);
	if(k < 5) { printf("mapfile corrupted\n"); exit(0);}
	if(k < 6 || *tagName == commentSign) tagName = dummyTag;
	return new t_circle(tagName,x,y,z,r,h);
    case 6:                     // 3D-DOOR
    {
	float w,h,angle;
	k = sscanf( line, "%f %f %f %f %f %s", &x, &y, &z, &w, &d, &h, &angle, tagName);
	if(k < 5) { printf("mapfile corrupted\n"); exit(0);}
	if(k < 6 || *tagName == commentSign) tagName = dummyTag;
	return new t_door(tagName,w,d,h,x,y,z,angle);
    }
    }
    return NULL;
}

void t_obst::new_color(char *color)
{
  change_color(myfig, color);  
}

t_obst::t_obst()
{
  active = FALSE;
}

t_obst::~t_obst()
{
    RemoveFigure(myfig);
}

void t_obst::mouse_action(float x, float y)
{
    return;
}

void t_obst::update(struct timeval* now)
{
  return;
}

void
t_obst::move(float diffX, float diffY)
{
    cx += diffX;
    cy += diffY;
}    
