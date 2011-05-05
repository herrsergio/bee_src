
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







#if defined(i386)

/*
 * ualarm - schedule signal after it_interval in microseconds
 *
 * 940619 ws
 *
 * "You do not want to use ualarm()"  ... and wouldn't you know it ...
 * Sorry Sebastian, couldn't resist :-)))
 */

#include <sys/time.h>

#define	WS_TIME_MICRO	1000*1000

unsigned int ualarm(unsigned int it_value, unsigned int it_interval)
{
 struct itimerval	value,ovalue;

 value.it_interval.tv_usec = it_interval % WS_TIME_MICRO;
 value.it_interval.tv_sec = it_interval / WS_TIME_MICRO;
 value.it_value.tv_usec = it_value % WS_TIME_MICRO;
 value.it_value.tv_sec = it_value / WS_TIME_MICRO;

 if (-1 == setitimer(ITIMER_REAL,&value,&ovalue))
     return -1;

 return ovalue.it_value.tv_sec * WS_TIME_MICRO + ovalue.it_value.tv_usec;
}

#endif
