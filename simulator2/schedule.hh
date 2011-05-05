
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
#include <sys/time.h>
#define REGULAR 0

class t_event {
public:
    int delay;
    struct timeval *duration;
    struct timeval insertion_time;    
    struct timeval start_time;
    struct timeval *delta_start;
    int priority;
    void (*job)();
    int due();
    int sooner(struct timeval*, t_event*);
    t_event(int typ, void (*)(), int priority, int delay,
	    struct timeval* dur, struct timeval* delta );
    t_event *next;
    t_event *prev;
    int type;
    friend int operator<(t_event, t_event);
    friend int operator<(struct timeval, struct timeval);
};

class t_eventlist {
public:
    void insert(t_event*);
    t_event* pop();
    void remove(t_event*);
    t_event* next();
    void print_list();
    t_event* first;
    t_event* last;    
private:
    friend int timeval_to_usec(struct timeval*);
};


class t_scheduler {
public:
    t_scheduler();
    void schedule( int type, void (*)(),
		   int priority, int delay,
		   struct timeval* dur,
		   struct timeval* delta);
    int triggerNextEvent();    
private:
    t_eventlist queue;
};

int timeval_to_usec(struct timeval* t);
void time_subtract(struct timeval *res, struct timeval *t1, struct timeval *t2);
