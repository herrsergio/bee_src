
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



/*
 * Portions of this code were created by and are
 * copyrighted by Real World Interface Inc. (RWI)
 *
 * The creation and continued develoment of this software
 * is sponsored and directed by RWI in the interest of
 * providing the mobile robotics and AI research communities
 * with a well designed and robust Robot Applications
 * Interface (RAI) for the complete line of RWI mobile robots. 
 *
 *   ==Contact  support@rwii.com  for further information==
 */

/*
 * Copyright 1994, Brown University, Providence, RI
 *
 * Permission to use and modify this software and its documentation for
 * any purpose other than its incorporation into a commercial product is
 * hereby granted without fee.  Permission to copy and distribute this
 * software and its documentation only for non-commercial use is also
 * granted without fee, provided, however, that the above copyright notice
 * appear in all copies, that both that copyright notice and this permission
 * notice appear in supporting documentation, that the name of Brown
 * University not be used in advertising or publicity pertaining to
 * distribution of the software without specific, written prior permission,
 * and that the person doing the distribution notify Brown University of
 * such distributions outside of his or her organization. Brown University
 * makes no representations about the suitability of this software for
 * any purpose.  It is provided "as is" without express or implied warranty.
 * Brown University requests notification of any modifications to this
 * software or its documentation.
 *
 * Send the following redistribution information:
 *
 *	Name:
 *	Organization:
 *	Address (postal and/or electronic):
 *
 * To:
 *	Software Librarian
 *	Computer Science Department, Box 1910
 *	Brown University
 *	Providence, RI 02912
 *
 *		or
 *
 *	brusd@cs.brown.edu
 *
 * We will acknowledge all electronic notifications.
 */

#ifndef SONAR_H
#define SONAR_H

#ifdef __cplusplus
extern "C" {
#endif


#include <sys/time.h>
#include <bUtils.h>

#define BOGUS_THRESHOLD 0
#define NO_RETURN 0x7FFF

/*
 * These structs may be expanded to support more features
 */

typedef struct sonarType {
  int value;			/* the distance */
  int mostRecent;		/* was this one in most recent return? */
  struct timeval time;		/* time of capture.  not yet precise */
} sonarType;

typedef  int (*sonarCallbackType)     (sonarType *sonar);

extern sonarType   sonars[];

void sonarInit(void);
void sonarStart(void);
void sonarStop(void);
void registerSonarCallback(sonarCallbackType);


#ifdef __cplusplus
}
#endif

#endif /* SONAR_H */
