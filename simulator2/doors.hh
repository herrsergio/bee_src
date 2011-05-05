
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
#include <X11/Intrinsic.h>
#include "rectangle.hh"
#include "Drawing.h"

class t_door : public t_rectangle {
  private:
    float apx,apy,apz;
    float angle;
    float last_angle;
    void absolute_points();
    virtual void move(float, float);
  public:
    t_door(char*, float, float, float, float, float, float, float);
    virtual void save(FILE*);
    virtual void mouse_action(float,float);
};
