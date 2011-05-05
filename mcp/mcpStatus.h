
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

#ifndef MCPSTATUS_HEADER
#define MCPSTATUS_HEADER


/* Status reports for Base MCP */
typedef struct  {
  unsigned long Request;
  unsigned long Clock;
  unsigned long GeneralStatus;
  unsigned long Xpos;
  unsigned long Ypos;  
  unsigned long Heading;
  unsigned long BaseRelativeHeading;
  unsigned long TranslateError;
  unsigned long TranslateVelocity;
  unsigned long TranslateStatus;
  unsigned long RotateError;
  unsigned long RotateVelocity;
  unsigned long RotateStatus;
} statusReportType;


/* what bits of the TranslateStatus is the phase number, the most */ 
/* interesting part of the Translate and Rotate Status fields     */
/* so if statusPhase(report->RotateStatus) == PHASE_ACCEL, the    */
/* motor on the rotate axis is accelerating */

#define PHASE_MASK 0xE000
#define statusPhase(status)  ( (status & PHASE_MASK) >> 13)
#define statusDirectionPositive(status)  ( !((status >> 11) & 1))
#define phaseMotionless(x) ((x==PHASE_STOP)||(x==PHASE_HALT)||(x==PHASE_LIMP))

#define PHASE_STOP  0
#define PHASE_ACCEL 1
#define PHASE_VELOC 2
#define PHASE_DECEL 3
#define PHASE_HALT  4
#define PHASE_LIMP  5
#define PHASE_POWER  6
#define PHASE_TORQUE  7


/*******************************************************************/
/*
    Support for specifying what a status report should include
*/ 
/*******************************************************************/

/* These are the bits you send to statusData or statusReport in  */
/* order to have the corresponding piece of data sent back. That */
/* is, to get Xpos and Ypos back, you send (REPORT_X | REPORT_Y) */

#define REPORT_STATUS_DATA	  	(1<<0)  /* allways set automatically */
#define REPORT_BASE_CLOCK		(1<<1) 
#define REPORT_GENERAL_STATUS		(1<<2) 
#define REPORT_X  			(1<<4)
#define REPORT_Y  			(1<<5)
#define REPORT_HEADING  		(1<<6)
#define REPORT_BASE_RELATIVE_HEADING	(1<<7)
#define REPORT_TRANSLATE_ERROR  	(1<<8)
#define REPORT_TRANSLATE_VELOCITY  	(1<<9)
#define REPORT_TRANSLATE_STATUS  	(1<<11)
#define REPORT_ROTATE_ERROR	  	(1<<16)
#define REPORT_ROTATE_VELOCITY  	(1<<17)
#define REPORT_ROTATE_STATUS  	        (1<<19)

/* half the stuff the status report could send is not really of interest */
/* so to get everything useful use REPORT_EVERYTHING  */ 

#define REPORT_EVERYTHING ( \
 REPORT_STATUS_DATA 		| \
 REPORT_BASE_CLOCK 		| \
 REPORT_GENERAL_STATUS 		| \
 REPORT_X  			| \
 REPORT_Y  			| \
 REPORT_HEADING  		| \
 REPORT_BASE_RELATIVE_HEADING 	| \
 REPORT_TRANSLATE_ERROR  	| \
 REPORT_TRANSLATE_VELOCITY 	| \
 REPORT_TRANSLATE_STATUS  	| \
 REPORT_ROTATE_ERROR		| \
 REPORT_ROTATE_VELOCITY		| \
 REPORT_ROTATE_STATUS  	\
)




/* The status report can also report this stuff, which seemed fairly */
/* obsolete now that the programming interface is so different       */
/* 3 = base bump switches, not used */ 
/* 10-15 A/D and motor stuff */
/* 18-31 More A/D, motor, radio stuff */

#endif


