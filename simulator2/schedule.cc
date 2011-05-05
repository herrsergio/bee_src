
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
#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>
#include "simxw.h"
#include "schedule.hh"


extern "C" {
    int block_wait(struct timeval *timeout, int tcx_initialized,
	       int X_initialized);
}

// ---------------------------------------------------------------------------

int
timeval_to_usec(struct timeval* t)
{
    return t->tv_sec * 1000000 + t->tv_usec;
}

void
time_subtract(struct timeval *res, struct timeval *t1, struct timeval *t2)
{
    res->tv_sec = t1->tv_sec - t2->tv_sec;
    res->tv_usec = t1->tv_usec - t2->tv_usec;
    if(res->tv_usec < 0) {
	res->tv_usec += 1000000;
	res->tv_sec -= 1;
    }
}

int operator<(struct timeval first, struct timeval second)
{
    int sec;
    int usec;
    sec = first.tv_sec - second.tv_sec;
    usec = first.tv_usec - second.tv_usec;
    if(usec < 0) 
	sec -= 1;
    return sec < 0;
}

//---------------------------------------------------------------------------


t_scheduler::t_scheduler()
{
    queue.first = queue.last = NULL;
}
void
t_scheduler::schedule(int type, void (*job)(), int priority, int delay,
		      struct timeval* dur, struct timeval* delta)
{
#if 0
  fprintf(stderr, "%10s:%5d:%14s()\n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif
    t_event *new_event = new t_event(type, job, priority, delay, dur, delta);
//    printf("created new event\n");
    queue.insert(new_event);
//    printf("event inserted\n");

#if 0
  fprintf(stderr, "%10s:%5d:%14s() - return\n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif
}

int
t_scheduler::triggerNextEvent()
{
    int sec, delay;
    int usec,delta;
    struct timeval now, before, waiting_time,late;
    t_event *next, *current;
    while(1) {
	current = queue.pop();
	while(current->due()) {
	    gettimeofday(&before,NULL);
	    current->job();
	    gettimeofday(&now,NULL);
	    time_subtract(&waiting_time, &now, &before);
	    time_subtract(&late, &now, &current->start_time);
	    current->duration->tv_sec = waiting_time.tv_sec;
	    current->duration->tv_usec = waiting_time.tv_usec;	    
	    current->delta_start->tv_sec = late.tv_sec/2;
	    current->delta_start->tv_usec =
		late.tv_usec/2+500000*(late.tv_sec%2);
	    delete current;
	    current = queue.pop();	    
	}
	queue.insert(current);

	next = queue.next();
	gettimeofday(&now,NULL);	
	time_subtract(&waiting_time, &next->start_time, &now);
	time_subtract(&late, &waiting_time, next->delta_start);
	if(timeval_to_usec(&late) > 500) {
	    return block_wait(&late, 1, 1);
	}
    }
}


//---------------------------------------------------------------------------

t_event::t_event(int typ, void (*myjob)(), int mypriority, int mydelay,
		 struct timeval* dur, struct timeval* delta )
{
    type = typ;
    gettimeofday( &insertion_time, NULL);
    job = myjob;
    delay = mydelay;
    priority = mypriority;
    start_time.tv_usec = insertion_time.tv_usec + delay; 
    start_time.tv_sec = insertion_time.tv_sec;
    start_time.tv_sec += start_time.tv_usec / 1000000;
    start_time.tv_usec %= 1000000;
    duration = dur;
    delta_start = delta;
}

int
operator<(t_event first, t_event second)
{
    struct timeval t1, t2;
    time_subtract(&t1, &first.start_time, first.delta_start);
    time_subtract(&t2, &second.start_time, second.delta_start);    
    return t1 < t2;
}

//  sooner returns true, if second should start earlier then first finishs  
int
t_event::sooner(struct timeval *now, t_event *first)
{
    int sec;
    int usec;
    struct timeval timediff, timediff2;
    time_subtract(&timediff2, &start_time, now);
    time_subtract(&timediff, &timediff2, delta_start);
    timediff2.tv_usec = first->duration->tv_usec + 500;
    timediff2.tv_sec = first->duration->tv_sec +
	               timediff2.tv_usec / 1000000;    
    timediff2.tv_usec %= 1000000;
    return timediff < timediff2;
}

// check if event is due

int
t_event::due()
{
    int sec;
    int usec;
    struct timeval now,timediff, timediff2;
    gettimeofday(&now, NULL);
    time_subtract(&timediff2, &start_time, &now);
    time_subtract(&timediff, &timediff2, delta_start);
    return (timediff.tv_sec < 0 ||
	    timediff.tv_sec == 0 && timediff.tv_usec < 5000);
}

// for debugging purposes

void
t_eventlist::print_list()
{
    t_event* current = first;
    struct timeval now;
    gettimeofday(&now,NULL);
    printf("current queue: %d %d\n", now.tv_sec, now.tv_usec);
    while(current) {
	printf("\t%x: %d, %d %d \n",
	       current,
	       current->priority,
	       current->start_time.tv_sec,
	       current->start_time.tv_usec);
	current = current->next;
    }
}


// insert into ordered list sort on time

void
t_eventlist::insert(t_event* new_event)
{
    t_event* current;
#if 0
  fprintf(stderr, "%10s:%5d:%14s()\n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

    if(!first) {
	first = last = new_event;
	new_event->prev = new_event->next = NULL;
    }
    else {
	current = first;
	while( current && ( *current < *new_event) ) {
	  if( new_event->type != REGULAR &&
	      current->type == new_event->type) {
	    fprintf(stderr, "Warning: dropping event\n");
	    delete new_event;
	    return;
	  }
	  current = current->next;
	}
	if(current) {
	    if(current->prev)
		current->prev->next = new_event;
	    else
		first = new_event;
	    new_event->prev = current->prev;
	    current->prev = new_event;
	    new_event->next = current;
	}
	else {
	    last->next = new_event;
	    new_event->next = NULL;
	    new_event->prev = last;
	    last = new_event;
	}
    }
    //    print_list();
}


// which event is triggered next

t_event*
t_eventlist::next()
{
    struct timeval now;
    t_event *current = first;
    gettimeofday(&now,NULL);
    while( current->next &&
	   current->priority > current->next->priority &&
	   current->next->sooner( &now, current) ) {
	current = current->next;
    }
    return current;
}

void
t_eventlist::remove(t_event* current)
{
    if(current->prev)
	current->prev->next = current->next;
    else
	first = current->next;
    if(current->next)
	current->next->prev = current->prev;
    else 
	last = current->prev;
}


// pop the event which is to be triggered next,
// drop unimportant event, which can not be handled in time.

t_event*
t_eventlist::pop()
{
    struct timeval now;
    t_event *to_remove, *current = first;
//    print_list();
    gettimeofday(&now,NULL);
    while( current->next &&
	   current->priority > current->next->priority &&
	   current->next->sooner( &now, current) ) {
	remove(current);
	to_remove = current;
	current = current->next;
//	printf("dropping one event\n");
	if(to_remove->type == REGULAR) {

	    int delta = timeval_to_usec(current->duration);
	    if(to_remove->priority > 0) to_remove->priority-=1;
	    to_remove->delta_start->tv_sec = 0;
	    to_remove->delta_start->tv_usec = 0;	    
	    t_event* new_event = new t_event(to_remove->type,
					     to_remove->job,
					     to_remove->priority-1,
					     to_remove->delay+delta,
					     to_remove->duration,
					     to_remove->delta_start);
	    delete to_remove;
	    insert(new_event);
	}
	else {
	    delete to_remove;
	}
    }
    remove(current);
    return current;
}

/*
struct timeval test_dur;

t_scheduler *test_scheduler;

void
proc1()
{
    struct timeval now;
    gettimeofday(&now,NULL);
    printf("proc1: time %d, duration (%d %d)\n", now.tv_sec,
	   test_dur.tv_sec, test_dur.tv_usec); 
    test_scheduler->schedule(REGULAR, proc1, 1, 200000, &test_dur);
}

void
proc2()
{
    struct timeval now;
    gettimeofday(&now,NULL);
    printf("proc2: time %d, duration (%d %d)\n", now.tv_sec,
	   test_dur.tv_sec, test_dur.tv_usec); 
    test_scheduler->schedule(REGULAR, proc2, 1, 100000, &test_dur);
}

void
proc3()
{
    struct timeval now;
    gettimeofday(&now,NULL);
    printf("proc3: time %d, duration (%d %d)\n", now.tv_sec,
	   test_dur.tv_sec, test_dur.tv_usec); 
    test_scheduler->schedule(REGULAR, proc3, 1, 128509, &test_dur);
}


main()
{
    test_dur.tv_sec = 0;
    test_dur.tv_usec = 600;
    test_scheduler = new t_scheduler();
    test_scheduler->schedule(REGULAR, proc1, 2, 200000, &test_dur);
    test_scheduler->schedule(REGULAR, proc2, 1, 100000, &test_dur);
    test_scheduler->schedule(REGULAR, proc3, 1, 128509, &test_dur);        
    test_scheduler->triggerNextEvent();
}

*/

