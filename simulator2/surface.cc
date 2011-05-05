
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
#include "sundefines.h"
#include "surface.hh"
#include "sonar.h"

t_surface *default_surface;
t_surflist *surfaces;


// returns a random number drawn to a normalized gaussian distribution
// that is zero mean and unit variance
float
random_gauss()
{
    static int iset = 0;
    static float gset;
    float fac, rsq, v1, v2;
    if(iset == 0) {
	do {
	    v1 = 2.0 * rand()/(RAND_MAX+1.0) - 1.0;
	    v2 = 2.0 * rand()/(RAND_MAX+1.0) - 1.0;	    
	    rsq = v1*v1 + v2*v2;
	} while(rsq >= 1.0 || rsq == 0.0);
	fac = sqrt(-2.0*log(rsq)/rsq);
	gset = v1*fac;
	iset = 1;
	return v2*fac;
    }
    else {
	iset = 0;
	return gset;
    }
}

float
random_uniform(float diff)
{
    return diff * rand()/((float)RAND_MAX + 1.0);
}

t_surface::t_surface(char *n, char *c)
{
    name = new char[strlen(n)+1];
    strcpy(name, n);
    color = new char[strlen(c)+1];
    strcpy(color, c);
    S_MIN_DIST = 5;
    S_MAX_DIST = sonar_range; 
    S_MINDIST_EANGLE = (60/180*M_PI);
    S_MAXDIST_EANGLE = (20/180*M_PI);
    S_ANGLERANGE = (30.0/180*M_PI);
}

void
t_surface::init_serror(float min_angle, float max_angle, float range)
{
    S_MINDIST_EANGLE = (min_angle/180*M_PI);
    S_MAXDIST_EANGLE = (max_angle/180*M_PI);
    S_ANGLERANGE = (range/180*M_PI);
}

float
t_surface::add_error(int sensor_type, float dist, float angle)
{
    switch(sensor_type) {
    case SONAR_SENSOR: return sonar_error(dist, angle);
    case IR_SENSOR: return ir_error(dist, angle);
    case LASER_SENSOR: return  dist + random_gauss();
    default:    return dist;
    }
}

float
t_surface::ran()
{
    float r;
    do {
	r = (float) rand() / ( (float) RAND_MAX + 1.0 );
    } while(r == 0.0);
    return r;
}

float
t_surface::sonar_error(float dist, float angle)
{
    float errorStartAngle;
    float errorProb;
    angle = 90 - angle;
    angle *= M_PI/180;
    if(dist < S_MIN_DIST)	// No response if distance < 5cm
	return sonar_range;
    errorStartAngle =
	S_MINDIST_EANGLE
	- (S_MINDIST_EANGLE-S_MAXDIST_EANGLE)
	/ (S_MAX_DIST-S_MIN_DIST)
	* (dist - S_MIN_DIST);
    if(angle < errorStartAngle) // correct measurement with certainty
	return dist;
				// error prob. increases to certainty
				// within 20 deg (linear approx.)
    errorProb =
	(angle - errorStartAngle)
	/ S_ANGLERANGE;
    if(errorProb > 1) errorProb = 1;
    if(errorProb > ran()) return dist + random_uniform(sonar_range-dist);
    else return dist;    
}

float
t_surface::ir_error(float dist, float angle)
{
    float value;

    angle = 90.0 - angle;
    angle *= M_PI/180.0;

    if(dist < 1) {
      dist = 1;
    }

    value = 40.0*40.0*fabs(cos(angle))/(dist*dist);
    if (value>100.0) {
      value = 100.0;
    }
    
    return(value);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

void
new_surf_entry(char *n, char *c,
	       float min_angle,
	       float max_angle,
	       float angle_range)
{
    surfaces->new_entry(n, c, min_angle, max_angle, angle_range);
}

void
t_surflist::new_entry(char *n, char *c,
	       float min_angle,
	       float max_angle,
	       float angle_range)
{
    t_surflist_entry *entry = new t_surflist_entry();
    if(!first) first = last = entry;
    else {
	last->next = entry;
	last = entry;
    }
    entry->surface = new t_surface(n, c);
    entry->surface->init_serror(min_angle, max_angle, angle_range);
    entry->next = NULL;
    if(first && (default_surface != first->surface) ) {
	delete default_surface;
	default_surface = first->surface;
    }
}

t_surflist::t_surflist()
{
    first = NULL;
    last = NULL;
    
    default_surface = new t_surface("default","grey80");
    default_surface->init_serror(60, 20, 30); 

}

t_surflist_entry::t_surflist_entry()
{
    surface= NULL;
    next = NULL;
}
