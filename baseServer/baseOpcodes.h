
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
#ifndef _BASE_OPCODES_H
#define _BASE_OPCODES_H

#include <mcpIO.h>  /*  opcodes common to all MCP applications*/

#define OP_GET_BATTERY_CURRENT 	0x61
#define OP_GET_BATTERY_VOLTAGE 	0x60
#define OP_FIND_ROT_INDEX 	0x13
#define OP_LOAD_POSITION	0x14
#define OP_LOAD_HEADING		0x15


#define OP_BUMPS_ENABLE		0x64

#define OP_ROTATE_TO_POS	0x5B
#define OP_ROTATE_HALT		0x42
#define OP_ROTATE_LIMP		0x43
#define OP_ROTATE_REL_POS	0x44
#define OP_ROTATE_REL_NEG	0x45
#define OP_ROTATE_VEL_POS	0x46
#define OP_ROTATE_VEL_NEG	0x47
#define OP_ROTATE_TRQ_NEG	0x48
#define OP_ROTATE_TRQ_POS	0x49
#define OP_ROTATE_PWR_NEG	0x4B
#define OP_ROTATE_PWR_POS	0x4A

#define OP_GET_ROTATE_CURRENT	0x5A
#define OP_GET_ROTATE_WHERE	0x5C

#define OP_SET_ROTATE_VEL	0x40
#define OP_SET_ROTATE_ACCEL	0x41
#define OP_SET_ROTATE_ZERO	0x50
#define OP_SET_ROTATE_TORQUE	0x4D
#define OP_SET_ROTATE_SLOPE	0x4E
#define OP_SET_ROTATE_FRICTION	0x4F

#define OP_TRANS_LIMP		0x23
#define OP_TRANS_HALT		0x22

#define OP_GET_TRANS_WHERE	0x3C
#define OP_GET_TRANS_CURRENT	0x3A

#define OP_SET_TRANS_VEL	0x20
#define OP_SET_TRANS_ACCEL	0x21
#define OP_SET_TRANS_TORQUE 	0x2D
#define OP_SET_TRANS_SLOPE	0x2E
#define OP_SET_TRANS_ZERO	0x30

#define OP_TRANS_REL_POS	0x24
#define OP_TRANS_REL_NEG	0x25
#define OP_TRANS_VEL_POS	0x26
#define OP_TRANS_VEL_NEG	0x27
#define OP_TRANS_TRQ_NEG	0x28
#define OP_TRANS_TRQ_POS	0x29
#define OP_TRANS_PWR_NEG	0x2B
#define OP_TRANS_PWR_POS	0x2A
#define OP_TRANS_TO_POS		0x3B


#endif
