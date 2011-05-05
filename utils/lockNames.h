
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

/******************************************************/
/*                                                    */
/* LockNames.h                                        */
/*                                                    */
/* Names for the lock files created for each system   */
/* program.  These are used in conjunction with       */
/* makeLock from Utils.h to make sure that a          */
/* program cannot create init the base module while   */
/* another is running a base module, and so on.       */     
/*                                                    */
/* James Kurien   jmk@cs.brown.edu		      */	
/*                                                    */
/******************************************************/

#ifndef _LOCKS_H
#define _LOCKS_H

#define LOCK_PREFIX "/usr/local/rwi/locks/_+RWI+_"

#define BASE_LOCK "B21Base"
#define SONAR_LOCK "Sonar"
#define PAN_LOCK "PanTilt"
#define MAST_LOCK "Mast"
#define GRIP_LOCK "Grip"
#define MSP_LOCK  "Msp"
#define COMPASS_LOCK "Compass"

#define ARM_SERVER_LOCK "ArmServer"
#define SPEECH_SERVER_LOCK "SpeechServer"
#define DISPLAY_SERVER_LOCK "DisplayServer"
#define PAN_SERVER_LOCK "PanServer"
#define BASE_SERVER_LOCK "BaseServer"

/* obsolete */
#define DRIVE_SERVER_LOCK "DriveServer"
#define VGA_SERVER_LOCK "VgaServer"

#endif
