
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
#ifndef T_CIRCLE_HH
#define T_CIRCLE_HH

#include "obstacles.hh"
#include "surface.hh"

class t_circle : public t_obst {
private:
    float r;      /* circles radius */
    float h;
public:
    t_circle(char*, float, float, float, float, float);
    virtual Boolean distance(float, float, float, float, float, float, float*, float *, t_surface**);
    virtual float min_distance(float, float);
    virtual void bounds(float*,float*,float*,float*);
    virtual Boolean inside(float, float);
    virtual void expose();
    virtual void toggle();    
    virtual void redraw();    
    virtual void save(FILE*);
};

#endif
