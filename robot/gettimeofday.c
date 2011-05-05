
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



/* Improve it as necessary.  - Tyson */

#ifdef VMS

#include "vms.h"
#include "vmstime.h"
   
#include <stdio.h>
#include <sys/time.h>

int gettimeofday( struct timeval *tp, struct timezone *tzp) {
    struct timeb theTime;    

    ftime(&theTime);
    if (tp) {
	tp->tv_sec=theTime.time;
	tp->tv_usec=theTime.millitm*1000;
    }
    return 0;
}
#endif
