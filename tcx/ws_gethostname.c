
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

#include <netdb.h>
#if defined(sun) && defined(__svr4__)
#include <sys/systeminfo.h>
#endif
#include <stdio.h>

int
ws_gethostname(char *hostname,int len)
{
 struct hostent         *hp;

#if defined(sun) && defined(__svr4__)
 if (-1 == sysinfo(SI_HOSTNAME,hostname,len-1))
     return -1;
#else
 if (-1 == gethostname(hostname,len-1))
     return -1;
#endif

 hp = gethostbyname(hostname);
 if (hp)
     (void)strncpy(hostname,hp->h_name,len-1);
 hostname[len-1] = '\0';

/* 
 if (!strchr(hostname,'.'))
    fprintf(stderr,"gethostbyname(hostname)->h_name != official hostname\n");
 */

 return 0;
}
