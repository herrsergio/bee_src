
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



/* some of the information in this file was originally distributed with the
   following and similar copyrights                                         */

/*      @(#)types.h 2.31 89/11/09 SMI; from UCB 7.1 6/4/86      */

/*
 * Copyright (c) 1982, 1986 Regents of the University of California.
 * All rights reserved.  The Berkeley software License Agreement
 * specifies the terms and conditions for redistribution.
 */
/****************************************************************************/ 

#ifdef VMS

#ifndef _VMS_H
#define _VMS_H

#include <unixio.h>
#include <unixlib.h>

#define NBBY    8               /* number of bits in a byte */
/*
 * Select uses bit masks of file descriptors in longs.
 * These macros manipulate such bit fields (the filesystem macros use chars).
 * FD_SETSIZE may be defined by the user, but the default here
 * should be >= NOFILE (param.h).
 */
#ifndef FD_SETSIZE
#define FD_SETSIZE      256
#endif

typedef long    fd_mask;
#define NFDBITS (sizeof (fd_mask) * NBBY)       /* bits per mask */
#ifndef howmany
#ifdef  sun386
#define howmany(x, y)   ((((u_int)(x))+(((u_int)(y))-1))/((u_int)(y)))
#else
#define howmany(x, y)   (((x)+((y)-1))/(y))
#endif
#endif

typedef struct fd_set {
        fd_mask fds_bits[howmany(FD_SETSIZE, NFDBITS)];
} fd_set;


#define FD_SET(n, p)    ((p)->fds_bits[(n)/NFDBITS] |= (1 << ((n) % NFDBITS)))
#define FD_CLR(n, p)    ((p)->fds_bits[(n)/NFDBITS] &= ~(1 << ((n) % NFDBITS)))
#define FD_ISSET(n, p)  ((p)->fds_bits[(n)/NFDBITS] & (1 << ((n) % NFDBITS)))
#define FD_ZERO(p)      bzero((char *)(p), sizeof (*(p)))

#define index(a, b)             strchr(a, b)
#define rindex(a, b)            strrchr(a, b)
#define bzero(a, b)             memset(a, 0, b)
#define bcmp(a, b, n)           memcmp(a, b, n)
#define bcopy(a, b, n)          memmove(b, a, n)

#endif  /* _VMS_H */
#endif  /* VMS */

