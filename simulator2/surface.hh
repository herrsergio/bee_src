
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

#ifndef SURFACE_HH 
#define SURFACE_HH

#define SONAR_SENSOR 0
#define LASER_SENSOR 1
#define EXACT_SENSOR 2
#define IR_SENSOR    3

#ifdef __cplusplus

class t_surface {
public:
    t_surface(char*, char*);
    float add_error(int, float, float);
    void init_serror(float, float, float);
private:
    char *name;
    char *color;    
    float S_MIN_DIST;
    float S_MAX_DIST; 
    float S_MINDIST_EANGLE;
    float S_MAXDIST_EANGLE;
    float S_ANGLERANGE;
    float ran();
    float sonar_error(float, float);    
    float ir_error(float, float);    
};

class t_surflist_entry {
public:
    t_surflist_entry();
    t_surface *surface;
    t_surflist_entry *next;
};

class t_surflist {
public:
    t_surflist();
    void new_entry(char*, char*, float, float, float);
private:
    t_surflist_entry *first;
    t_surflist_entry *last;    
};


#endif

#ifdef __cplusplus
extern "C" {
#endif

void new_surf_entry(char*, char*, float, float, float);    
    
#ifdef __cplusplus
}
#endif

#endif

