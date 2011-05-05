
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

#ifndef SIM_OBSTACLES_H
#define SIM_OBSTACLES_H

#include <X11/Intrinsic.h>
#include <stdio.h>
#include <sys/time.h>
#include "Drawing.h"
#include "surface.hh"

class t_obst {
public:
    int number;		   // ID 
    int type;
    char* tag;
    int active;
    Boolean enabled;
    void new_color(char*);
    virtual void move(float, float);
    virtual Boolean distance(float, float, float, float,
			     float, float,
			     float*, float *, t_surface**)   = 0;
    virtual float min_distance(float, float)                 = 0;
    virtual Boolean inside(float, float)                     = 0;
    virtual void bounds(float*,float*,float*,float*)         = 0;
    virtual void expose()                                    = 0;
    virtual void toggle()                                    = 0;    
    virtual void save(FILE*)                                 = 0;    
    virtual void redraw()                                    = 0;
    virtual void update(struct timeval*);        
    virtual void mouse_action(float,float);
    ~t_obst();
    t_obst();
protected:
    float cx, cy, cz;        // position e.g. center of bounding box 
    float x1,y1,x2,y2;       // static bounding box ensured to be correct 
    Figure myfig;            //Canvas Figure 
    char* color;
    t_surface *my_surface;
};

#define N_OBSTACLE_TYPES 7
extern char* o_types[N_OBSTACLE_TYPES];

extern char* dummyTag;
extern char commentSign;

#define DEF_H 300.0
#define DEF_Z (DEF_H / 2.0)

#endif
