
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
#include "surface.hh"

class t_obstws {
private:
    int top;
    int travptr;
    int maxsize;
    t_obst** workingset;
    Boolean* members;
public:
    t_obstws(int);
    ~t_obstws();
    Boolean check_room(int);
    void add(t_obst*);
    void clear();
    int size();
    t_obst* traverse();
};

class t_obstlist_entry {
public:
    t_obst *obstacle;
    t_obstlist_entry *next;
};

class t_obstlist {
public:
    t_obstlist();
    ~t_obstlist();
    t_obst* insert(t_obst*);
    void remove(t_obst*);
    t_obstlist_entry *first;
    t_obstlist_entry *last;
};

class t_obstgrid {
public:
    int n_obstacles;
    t_obstgrid(float,float,float,float);
    ~t_obstgrid();
    int read_obstacles(FILE*);
    void fill_ws(float,float,float);
    void fill_ws_ray(float,float,float,float);    
    void ExposeObstacles();
    void RemoveObstacle(t_obst*);
    void Update();              // active obstacle state
    void Redraw();              // active obstacles only    
    void SaveObstacles(FILE*);    
    void new_rectangle(float,float,float,float);
    void new_door(float,float,float,float);
    void new_circle(float,float,float);
    void new_human(float,float,float,float);        
    void bounds(float*,float*,float*,float*);
    void move(float,float);
    t_obst* InsideObstacle(float,float);          // only visible obstacles
    t_obst* InsideAnyObstacle(float,float);       
    Boolean distance(float, float, float, float, float, float, float*, float*, t_surface**);
    float min_distance(float, float);
    t_obst *insert(t_obst*);
private:
    float xoff,yoff;
    float lx,ly,hx,hy;
    int cols,rows;
    t_obstlist all_obstacles;
    t_obstlist active_obstacles;
    t_obstlist** grid;
    t_obstws* ws;
    void set(int,int,t_obstlist*);
    void remove_from_grid(t_obst*);
    void remove_from_obstlist(t_obstlist**, t_obstlist**, t_obst*);
    Boolean off_ground(float, float);
    t_obstlist* get(int,int);
    void color_ws(char*);
    void insert_grid(t_obst*);
};

#define WS_SPAREROOM 200        /* room for N additional obstacles */
#define OBST_GRID_SIZE 100      /* size of a grid cell in cm */
