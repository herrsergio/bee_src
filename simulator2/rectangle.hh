
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

#ifndef SIM_RECTANGLE_H
#define SIM_RECTANGLE_H

#include "obstacles.hh"
#include "surface.hh"

class t_rectangle : public t_obst {
protected:
    float ori;
    float last_ori;
    float abs_ori;
    float w,d,h;
    float x[4],y[4];    
    virtual void absolute_points();
    virtual void newBounds(float*,float*,float*,float*);
    char* color;
public:
    t_rectangle();
    t_rectangle(char*, float, float, float, float, float, float, float = 0);
    virtual Boolean distance(float, float, float, float, float, float, float*, float *, t_surface**);
    virtual float min_distance(float, float);
    virtual void move(float dx, float dy);
    virtual void bounds(float*,float*,float*,float*);
    virtual Boolean inside(float, float);
    virtual void expose();
    virtual void toggle();    
    virtual void save(FILE*);    
    virtual void redraw();        
};


#endif
