
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
#include "obstacles.hh"
#include "circle.hh"
#include "surface.hh"

#define HUMAN_RADIUS 20

class t_human : public t_circle {
public:
    t_human(char* tag, float x, float y, float speed, float angle);
    virtual void update(struct timeval *now);
    virtual void save(FILE*);
private:
  struct timeval last_update;
  int stopped;
  float speed;
  float angle;
  float start_pos_x;
  float start_pos_y;  
};
