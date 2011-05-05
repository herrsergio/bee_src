
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
#include <stdio.h>
#include <math.h>

#include "sundefines.h"
#include "store.hh"
#include "obstacles.hh"
#include "rectangle.hh"
#include "circle.hh"
#include "doors.hh"
#include "human.hh"

extern t_obst* install_obstacle(int,FILE*);

Boolean
t_obstws::check_room(int n)
{
    return n < maxsize;
}

t_obstws::~t_obstws()
{
    delete[] workingset;
    delete[] members;
}

t_obstws::t_obstws(int n)
{
    maxsize = n+WS_SPAREROOM;
    workingset = new (t_obst*)[n+WS_SPAREROOM];
    members = new Boolean[n+WS_SPAREROOM];
    top = 0;
    travptr = 0;
}

void
t_obstws::add(t_obst *obst)
{
    if(obst->number >= maxsize) {
	fprintf(stderr, "t_obstws::add() : set overflow (%d) !\n",obst->number);
	exit(-1);
    }
    if(!obst->enabled || members[obst->number]) return;
    workingset[top++] = obst;
    members[obst->number] = TRUE;
}

int
t_obstws::size()
{
    return top;
}

void
t_obstws::clear()
{
  int i;
  for(i = 0; i < (maxsize); i++) members[i] = FALSE;
  top = 0;
  travptr = 0;
}

t_obst*
t_obstws::traverse()
{
  if(travptr < top)
    return workingset[travptr++];
  else {
    travptr = 0;
    return NULL;
  }
}

/* ------------------------------------------------------------------------- */

t_obstlist*
t_obstgrid::get(int i, int j)
{
#ifdef DEBUG
    if( i < 0 ) printf("t_obstgrid::get row_underflow: %d %d\n", i, j);
    if( j < 0 ) printf("t_obstgrid::get col_underflow: %d %d\n", i, j);
    if( i >= rows ) printf("t_obstgrid::get row_overflow: %d %d\n", i, j);
    if( j >= cols ) printf("t_obstgrid::get col_overflow: %d %d\n", i, j);
#endif
    return grid[i*cols+j];
}

Boolean
t_obstgrid::off_ground(float x, float y)
{
    if(x < lx || y < ly || x > hx || y > hy)
	return TRUE;
    else return FALSE;
}

t_obstgrid::~t_obstgrid()
{
    int i,j;
    t_obstlist_entry *tmp, *del_entry = all_obstacles.first;
    while(del_entry) {
	tmp = del_entry;
	del_entry = del_entry->next;
	delete tmp->obstacle;
    }
    for(i = 0; i < rows*cols; i++) {
	delete grid[i];
    }
    delete ws;
}

t_obstgrid::t_obstgrid(float x1, float y1, float x2, float y2)
{
  int i;
  lx = x1;
  ly = y1;
  hx = x2;
  hy = y2;
  float w = x2 - x1+200;            // leave some room for a bounding box
  float d = y2 - y1+200;
  t_obst *new_obstacle;
  xoff = x1-100;
  yoff = y1-100;
  cols = (int)( (w) / OBST_GRID_SIZE + 2.0);
  rows = (int)( (d) / OBST_GRID_SIZE + 2.0);
  grid = new (t_obstlist*)[cols*rows];
  for(i=0; i < cols*rows; i++) {
      grid[i] = new t_obstlist();
  }
  ws = new t_obstws(0);
  all_obstacles.first = NULL;
  all_obstacles.last = NULL;  
  active_obstacles.first = NULL;
  active_obstacles.last = NULL;
  n_obstacles = 0;
  new_obstacle = new t_rectangle(dummyTag,x1-50,y1+d/2,DEF_Z,100,d,DEF_H);
  new_obstacle->number = n_obstacles++;
  insert_grid(new_obstacle);
  new_obstacle = new t_rectangle(dummyTag,x2+50,y1+d/2,DEF_Z,100,d,DEF_H);
  new_obstacle->number = n_obstacles++;  
  insert_grid(new_obstacle);
  new_obstacle = new t_rectangle(dummyTag,x1+w/2,y1-50,DEF_Z,w,100,DEF_H);
  new_obstacle->number = n_obstacles++;
  insert_grid(new_obstacle);
  new_obstacle = new t_rectangle(dummyTag,x1+w/2,y2+50,DEF_Z,w,100,DEF_H);   
  n_obstacles = n_obstacles++;
  new_obstacle->number = n_obstacles++;  
  insert_grid(new_obstacle);
}

void
t_obstgrid::insert_grid(t_obst *obst)
{
    t_obstlist_entry *entry;
    t_obstlist *list;
    int minx,miny,maxx,maxy;
    float x1,y1,x2,y2;
    int i,j;

    if(ws) {
	if(!ws->check_room(n_obstacles)) {
	    delete ws;
	    ws = new t_obstws(n_obstacles);
	}
    }

    obst->bounds(&x1,&y1,&x2,&y2);
    minx = (int)( (x1-xoff) / OBST_GRID_SIZE);
    miny = (int)( (y1-yoff) / OBST_GRID_SIZE);
    maxx = (int)( (x2-xoff) / OBST_GRID_SIZE + 1);
    maxy = (int)( (y2-yoff) / OBST_GRID_SIZE + 1);

    for(i = miny; i < maxy; i++)
	for(j = minx; j < maxx; j++) {
	  entry = new t_obstlist_entry;
	  entry->obstacle = obst;
	    list = get(i,j);
	    entry->next = list->first;
	    list->first = entry;
	}
}

t_obstlist::~t_obstlist()
{
    t_obstlist_entry *tmp, *del_entry = first;
    while(del_entry) {
	tmp = del_entry;
	del_entry = del_entry->next;
	delete tmp;
    }
}

t_obstlist::t_obstlist()
{
    first = last = NULL;
}

t_obst*
t_obstlist::insert(t_obst *obst)
{
  t_obstlist_entry *new_obstacle = new t_obstlist_entry;
  new_obstacle->obstacle = obst;
  if(last) { 
    last->next = new_obstacle;	
    last = new_obstacle;
  }
  else {
    first = last = new_obstacle;
  }
  last->next = NULL;
}


t_obst*
t_obstgrid::insert(t_obst* obst)
{
  all_obstacles.insert(obst);
  obst->number = n_obstacles++;
  if(obst->active) {
      active_obstacles.insert(obst);
  }
  insert_grid(obst);
  return obst;
}

void
t_obstgrid::fill_ws_ray(float x1, float y1,float x2, float y2)
{
  float m;
  float tx,ty,inc;
  int i,j;
  t_obstlist_entry *entry;
  ws->clear();
  if( fabs(x1 - x2) < 0.01 ) {
    if(y2 < y1) {
      m = y1;
      y1 = y2;
      y2 = m;
    }
    j = (int)( (x1-xoff) / OBST_GRID_SIZE);
    if(j < 0) return;
    if(j >= cols) return;  
    for(ty = y1; ty <= (y2+OBST_GRID_SIZE); ty+=OBST_GRID_SIZE) {
      i = (int)( (ty-yoff) / OBST_GRID_SIZE);
      if(i < 0) continue;
      if(i >= rows) continue;  
      entry = get(i,j)->first;
      while(entry) {
	ws->add(entry->obstacle);
	entry = entry->next;
      }
    }
    return;
  }
  if(x1 > x2) {
    tx = x2;
    ty = y2;
    x2 = x1;
    y2 = y1;
    x1 = tx;
    y1 = ty;
  }
  if( fabs(y2 - y1) < 0.01) {
      m = 0;
      inc = OBST_GRID_SIZE;
  }
  else {
    m = (y2-y1)/(x2-x1);
    inc = fabs((float)OBST_GRID_SIZE / m);
    if(inc > OBST_GRID_SIZE) inc = OBST_GRID_SIZE;
  }
  for(tx = x1; tx <= (x2+OBST_GRID_SIZE); tx += inc) {
    ty = m*(tx-x1) + y1;
    j = (int)( (tx-xoff) / OBST_GRID_SIZE);
    i = (int)( (ty-yoff) / OBST_GRID_SIZE);
    if(j < 0) continue;
    if(i < 0) continue;
    if(j >= cols) continue;
    if(i >= rows) continue;  
    entry = get(i,j)->first;
    while(entry) {
      ws->add(entry->obstacle);
      entry = entry->next;
    }
  }
}


void t_obstgrid::fill_ws(float x, float y,float dist)
{
  int minx = (int)( (x-xoff-dist) / OBST_GRID_SIZE);
  int miny = (int)( (y-yoff-dist) / OBST_GRID_SIZE);
  int maxx = (int)( (x-xoff+dist) / OBST_GRID_SIZE + 1);
  int maxy = (int)( (y-yoff+dist) / OBST_GRID_SIZE + 1);
  int i,j;
  t_obstlist_entry *entry;
#ifdef DEBUG
  color_ws("grey50");
#endif
  ws->clear();
  if(minx < 0) minx = 0;
  if(miny < 0) miny = 0;
  if(maxx >= cols) maxx = cols-1;
  if(maxy >= rows) maxy = rows-1;  
  for(i = miny; i < maxy; i++)
    for(j = minx; j < maxx; j++) {
      entry = get(i,j)->first;
      while(entry) {
	ws->add(entry->obstacle);
	entry = entry->next;
      }
    }
#ifdef DEBUG
  color_ws("red");
  printf("num_vis_obstacles: %d\n",ws->size());
#endif
}

int
t_obstgrid::read_obstacles(FILE *mapfd)
{
    char otype[80];
    int i;
    Boolean found;
    t_obst *new_obstacle;
    while(!feof(mapfd)) {
	found = FALSE;    
	i = fscanf(mapfd,"%s",&otype);
	if( i < 1)
	  continue;
	else if ( otype[0] == commentSign) { // comment sign --> ignore end of line
	  fgets( otype, sizeof(otype),mapfd);
	  continue;
	}	
	for(i = 0; i < N_OBSTACLE_TYPES; i++) {
	    if(strcmp(o_types[i], otype) == 0) {
		found = TRUE;
		new_obstacle = install_obstacle(i,mapfd);
		insert(new_obstacle);
		break;
	    }
	}
    }
    if(ws) delete ws;
    ws = new t_obstws(n_obstacles);
    return n_obstacles;   
}

void t_obstgrid::ExposeObstacles()
{
    t_obstlist_entry *to_expose = all_obstacles.first;
    int i,j;
    while(to_expose) {
	to_expose->obstacle->expose();
	to_expose = to_expose->next;
    }
	
}

void t_obstgrid::SaveObstacles(FILE *fd)
{
    t_obstlist_entry *to_expose = all_obstacles.first;
    int i,j;
    while(to_expose) {
	to_expose->obstacle->save(fd);
	to_expose = to_expose->next;
    }
	
}

void
t_obstgrid::bounds(float *minX, float *minY, float *maxX, float *maxY)
{
    float tx1,ty1,tx2,ty2;
    t_obstlist_entry* trav_obst = all_obstacles.first;
    if(trav_obst) {
	trav_obst->obstacle->bounds(minX,minY,maxX,maxY);
	trav_obst = trav_obst->next;
    }
    else {
	*minX = 0;
	*maxX = 0;	
	*minY = 0;
	*maxY = 0;
	return;
    }
    while(trav_obst) {
	trav_obst->obstacle->bounds(&tx1,&ty1,&tx2,&ty2);
	if(tx1 < *minX) *minX = tx1;
	if(tx2 > *maxX) *maxX = tx2;	
	if(ty1 < *minY) *minY = ty1;
	if(ty2 > *maxY) *maxY = ty2;
	trav_obst = trav_obst->next;
    }
}

void
t_obstgrid::move(float diffX, float diffY)
{
    t_obstlist_entry *trav_obst = all_obstacles.first;
    while(trav_obst) {
	trav_obst->obstacle->move(diffX, diffY);
	trav_obst = trav_obst->next;
    }
}

t_obst*
t_obstgrid::InsideObstacle(float x, float y)
{
    t_obst *inside_test_obstacle;    
    fill_ws(x,y,OBST_GRID_SIZE);    
    while((inside_test_obstacle = ws->traverse())) {
	if(inside_test_obstacle->inside(x,y))
	    return inside_test_obstacle;
    }
    return NULL;
}

t_obst*
t_obstgrid::InsideAnyObstacle(float x, float y)
{
    t_obstlist_entry *inside_test_obstacle = all_obstacles.first;    
    while(inside_test_obstacle) {
	if(inside_test_obstacle->obstacle->inside(x,y))
	    return inside_test_obstacle->obstacle;
	inside_test_obstacle = inside_test_obstacle->next;
    }
    return NULL;
}

void
t_obstlist::remove(t_obst *remove)
{
    t_obstlist_entry *rem_test = first, *last_one = NULL;
	while(rem_test && rem_test->obstacle != remove) {
	last_one = rem_test;
	rem_test = rem_test->next;
    }
    if(!rem_test) {
	fprintf(stderr, "t_obstlist::remove() : obstacle not in list\n");
	exit(0);
    }
    if(last_one) {
	last_one->next = rem_test->next;
	delete rem_test;
	if(last_one->next == NULL) last = last_one;    
    }
    else {
	if( first->next == NULL ) first = last = NULL; 
	else first = first->next;
	delete rem_test;
    }

}

void
t_obstgrid::remove_from_grid(t_obst *to_remove)
{
  float x1,y1,x2,y2;
  int minx, maxx, miny, maxy;
  int i,j;
  t_obstlist *list, *dummy;
  to_remove->bounds(&x1,&y1,&x2,&y2);
  minx = (int)( (x1-xoff) / OBST_GRID_SIZE);
  miny = (int)( (y1-yoff) / OBST_GRID_SIZE);
  maxx = (int)( (x2-xoff) / OBST_GRID_SIZE + 1);
  maxy = (int)( (y2-yoff) / OBST_GRID_SIZE + 1);
  for(i = miny; i < maxy; i++)
    for(j = minx; j < maxx; j++) {
      list = get(i,j);
      list->remove(to_remove);
    }
}

void
t_obstgrid::Update()
{
  t_obstlist_entry *traverse = active_obstacles.first;
  struct timeval now;
  gettimeofday(&now, NULL);
  while(traverse) {
    remove_from_grid(traverse->obstacle);
    traverse->obstacle->update(&now);
    insert_grid(traverse->obstacle);
    traverse = traverse->next;
  }
}

void
t_obstgrid::Redraw()
{
  t_obstlist_entry *traverse = active_obstacles.first;
  while(traverse) {
    remove_from_grid(traverse->obstacle);
    traverse->obstacle->redraw();
    insert_grid(traverse->obstacle);
    traverse = traverse->next;
  }
}

void
t_obstgrid::RemoveObstacle(t_obst *remove)
{
    if(!remove) return;
    printf("removing from all obstacles\n");
    all_obstacles.remove(remove);
    if(remove->active) {
	printf("removing from active obstacles\n");	
	active_obstacles.remove(remove);
    }
    remove_from_grid(remove);
    delete remove;
}

void
t_obstgrid::new_rectangle(float x1, float y1, float x2, float y2)
{
  float w = x2-x1;
  float d = y2-y1;
  insert(new t_rectangle(dummyTag,x1+w/2,y1+d/2,DEF_Z,w,d,DEF_H))->expose();
}

static void
swapFloats(float *f1, float *f2)
{
    float t = *f1;
    *f1 = *f2;
    *f2 = t;
}

void
t_obstgrid::new_door(float apx, float apy, float w, float d)
{
  float tmp,angle;
  if(fabs(w) < fabs(d)) {        // doors orientation is vertical
      swapFloats(&w,&d);
      if(w < 0) {  // door drawn in negative y-direction
	  angle = -M_PI/2;
      }
      else {
	  angle = M_PI/2;
	  d *= -1;
      }
  }
  else {             // doors orientation is horizontal
      if(w < 0) {  // door drawn in negative x-direction
	  angle = M_PI;
	  d *= -1;
      }
      else angle = 0;      
  }
  insert(new t_door(dummyTag,w,d,DEF_H,apx,apy,DEF_Z,angle))->expose();    
}

void
t_obstgrid::new_circle(float x1, float y1, float r)
{
  insert(new t_circle(dummyTag,x1,y1,DEF_Z,r,DEF_H))->expose();
}

void
t_obstgrid::new_human(float x1, float y1, float s, float a)
{
  insert(new t_human(dummyTag,x1,y1,s,a))->expose();
}

Boolean t_obstgrid::distance(float xr, float yr, float zr,
			     float open_angle,
			     float xe, float ye,
			     float *dist, float *angle, t_surface **surface)
{
  float tangle,tmp;
  Boolean Hit = FALSE, C_Hit;
  t_obst *c_obstacle;
  t_surface *tsurface;
  fill_ws_ray(xr,yr,xe,ye); 
  while((c_obstacle = ws->traverse())) {
    C_Hit = FALSE;
    C_Hit = c_obstacle->distance(xr,yr,zr, open_angle,
				 xe,ye,&tmp,&tangle,&tsurface);
    if(C_Hit) {
	tangle = fabs(tangle);
	if(tangle > M_PI) tangle -= M_PI;    
	if(Hit) {
	    if(tmp < *dist) {
		*dist = tmp;
		*angle = tangle;
		*surface = tsurface;
	    }
	}
	else {
	    Hit = TRUE;
	    *dist = tmp;
	    *angle = tangle;
	    *surface = tsurface;
	}
    }
  }
  return Hit;
}

float t_obstgrid::min_distance(float x, float y)
{
  float tmp, dist = MAXFLOAT;
  t_obst *c_obstacle;

  fill_ws(x,y,500);

  while((c_obstacle = ws->traverse())) {
    tmp = c_obstacle->min_distance(x, y);
    if(tmp < dist) dist = tmp;
  }
  return dist;
}

void t_obstgrid::color_ws(char *color)
{
  t_obst *c_obstacle;
  while((c_obstacle = ws->traverse())) {
    c_obstacle->new_color(color);
  }
}





