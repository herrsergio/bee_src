
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
#ifndef _ARM_OPCODES_H
#define _ARM_OPCODES_H

#define OP_ERROR_ACKNOWLEDGE 	0x0B
#define OP_INDEX_MASK           0x65

/* setting index mask 3 allows the mcp to return some bits whenever */
/* an axis goes by the index */
/*

  On the mast, the mast is the 2nd bit, the boom is the first.
  The bit is set when the sensor hits the index and when it leaves it.

  On the gripper, the wrist is the second bit and works as above.
  The gripper tactiles are backwards (0 is open, 1 is closed)
  and you get a signal whenever state  chagnes.

*/ 

/**********************************************************/

/*** putting the boom out ***/

#define OP_BOOM_HALT		0x22
#define OP_BOOM_LIMP		0x23
#define OP_BOOM_EXTEND		0x26
#define OP_BOOM_RETRACT		0x27

#define OP_BOOM_TORQUE 		0x2D
#define OP_BOOM_VEL		0x20
#define OP_BOOM_GET_WHERE	0x3C


/**** MAST ***/

#define OP_MAST_RAISE		0x46
#define OP_MAST_DROP		0x47
#define OP_MAST_LIMP		0x43
#define OP_MAST_HALT		0x42
#define OP_MAST_REL_UP		0x44
#define OP_MAST_REL_DOWN	0x45
#define OP_MAST_TO_POS		0x5B
#define OP_MAST_FIND_INDEX 	0x13
#define OP_MAST_LOAD_HEADING 	0x15


#define OP_MAST_VEL		0x40
#define OP_MAST_TORQUE		0x4D
#define OP_MAST_SLOPE		0x4E


/* getting mast info */
#define OP_MAST_GET_WHERE	0x5C

/******************************************/
/*  
GRIP STUFF
*/
/******************************************/

#define OP_GRIP_VEL		0x20
#define OP_GRIP_ACCEL		0x21
#define OP_GRIP_HALT		0x22
#define OP_GRIP_LIMP		0x23
#define OP_GRIP_REL_OPEN	0x24
#define OP_GRIP_REL_CLOSE	0x25
#define OP_GRIP_OPEN		0x26
#define OP_GRIP_CLOSE		0x27

#define OP_GRIP_TORQUE 		0x2D


#define OP_GRIP_TO_POS		0x3B
#define OP_GRIP_GET_WHERE	0x3C

/******************************************/
/*  
WRIST STUFF
*/
/******************************************/

#define OP_WRIST_LOAD_HEADING 	0x15
#define OP_WRIST_HALT		0x42
#define OP_WRIST_LIMP		0x43

/* NOTE: These are backwards wrt the base.  Using the opcodes this */
/* way allows the wrist to turn CW from the perspective of the robot */
/* for positive */

/*
 * These are no longer backwards do to gear changes
 */

#define OP_WRIST_REL_NEG	0x45
#define OP_WRIST_REL_POS	0x44

#define OP_WRIST_TORQUE		0x4D
#define OP_WRIST_VEL		0x40

#define OP_WRIST_FIND_INDEX 	0x13

#define OP_WRIST_GET_WHERE	0x5C

#define OP_WRIST_TO_POS		0x5B


#endif
