
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
#include "compound.hh"

t_compobst::t_compobst(int n, float x, float y, float iori)
{
    type = 4;
    components = NULL;
    ori = iori;
    number = n;
    cx = x;
    cy = y;
}

Boolean t_compobst::distance(float x, float y,
				 float xe, float ye,
				 float* dist, float* ang)
{
    float tdist, tang;
    Boolean ret = FALSE;
    t_obstlist* trav = components;
    *dist = MAXFLOAT;
    while(trav) {
	ret |= trav->obstacle->distance(x,y,xe,ye,&tdist,&tang);
	if(tdist < *dist) {
	    *dist = tdist;
	    *ang = tang;
	}
	trav = trav->next;
    }
    return ret;
}

float t_compobst::min_distance(float x, float y)
{
    float tdist,dist;
    t_obstlist* trav = components;
    dist = MAXFLOAT;
    while(trav) {
	tdist = trav->obstacle->min_distance(x,y);
	if(tdist < dist)
	    dist = tdist;
	trav = trav->next;
    }
    return dist;
}

void t_compobst::bounds(float* xo1, float* yo1, float* xo2, float* yo2)
{
    float tx1,ty1,tx2,ty2;
    t_obstlist* trav = components;
    *xo1=MAXFLOAT;
    *yo1=MAXFLOAT;
    *xo2=MINFLOAT;
    *yo2=MINFLOAT;
    while(trav) {
	trav->obstacle->bounds(&tx1,&ty1,&tx2,&ty2);
	if(tx1 < *xo1)
	    *xo1 = tx1;
	if(ty1 < *yo1)
	    *yo1 = ty1;
	if(tx2 < *xo2)
	    *xo2 = tx2;
	if(ty2 < *yo2)
	    *yo2 = ty2;
	trav = trav->next;
    }
    return;
}

Boolean t_compobst::inside(float x, float y)
{
    t_obstlist* trav = components;
    while(trav) {
	if(trav->obstacle->inside(x,y))
	    return TRUE;
	trav = trav->next;
    }
    return FALSE;
}

void t_compobst::expose()
{
    t_obstlist* trav = components;
    while(trav) {
	trav->obstacle->expose();
	trav = trav->next;
    }
}
