
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

#ifndef MSP_MODULE_H
#define MSP_MODULE_H

#include <time.h>
#include <bUtils.h>
#include <sonarClient.h>     /* The generic APIs for the sensors we support*/
#include <tactileClient.h>
#include <irClient.h>
#include <acb/abus.h>
#include <msp.h>

/*
 * This MSP RAI device module supports only one client.
 * It perhaps should support the possibility of more
 * than one fd.  That can come later.
 *
 * This modules abstracts the multiple MSPs with different
 * sensor types into linear arrays of sensors.  The
 * setting of the tables to translate the sensors
 * of each MSP to the linear array for each device 
 * type will be a topic of continued research.
 * One goal is to be as independent if MSPs as reasonable.
 */

#define MAX_SONAR_CALLBACKS 1

#define ABUS_FILE "/dev/abus"

#define  NUM_BASE_MSPS     (4)
#define  NUM_ENCL_MSPS     (3)
#define  NUM_MSPS          (NUM_BASE_MSPS + NUM_ENCL_MSPS)

#define  CHAINS_PER_MSP    (2)
#define  SONARS_PER_CHAIN  (4)
#define  SONARS_PER_MSP    (CHAINS_PER_MSP * SONARS_PER_CHAIN)

/* More  IR, Sonar, Tactile info defined on a per machine basis in Robot.h */

#define SONARS_PER_SET 6
#define NUM_SETS 4		/* How many sets for full table */
#define BOGUS_THRESHOLD 0
#define NO_RETURN 0x7FFF
#define MM_PER_CLICK 0.568

extern int normalizeIr;

#endif MSP_MODULE_H
