
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



#ifndef _MCP_IO_H
#define  _MCP_IO_H

#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <utils.h>
#include <rai.h>

#ifdef __cplusplus
extern "C" {
#endif



typedef struct
{
  int fd;
  int state;
  char* lockname;
  char* return_string;
  int start_in_stream;
  int pos;
  char buff[512];
  int  buffLen;
  int  buffPos;
}  mcpIOState;

void mcpSendCmd(mcpIOState*, unsigned char opcode, unsigned long param);
char* mcpSelect(mcpIOState*);
mcpIOState* openMcp(const char* devicename, char* lockname,speed_t baudRate);
void resetMcpComm(mcpIOState* state);
void  closeMcp(mcpIOState*);




/***** Binary mode opcodes which are common to all MCP usages.   ****/
/***** The semantics of other MCP opcodes depend on what the MCP ****/
/*****  is hooked up to  */

#define OP_GET_VERSION_REPORT 	0x01
#define OP_MCP_KILL 		0x02
#define OP_MCP_DELAY 		0x05
#define OP_GET_USER_MESSAGE     0x08
#define OP_LEAVE_DIRECT_MODE 	0x09
#define OP_ERROR_DELAY 		0x0A
#define OP_ERROR_ACKNOWLEDGE 	0x0B
#define OP_FULL_DUPLEX 		0x0C
#define OP_HALF_DUPLEX 		0x0D
#define OP_JOYSTICK_DISABLE	0x0E
#define OP_RADIO_DISABLE	0x0F
#define OP_GET_ONE_REPORT       0x10
#define OP_SET_STATUS_DATA      0x11
#define OP_SET_STATUS_PERIOD    0x12
#define OP_WATCH_DOG_TIMER	0x16
#define OP_ADJUST_HEADING	0x17   /* delta is sent as signed short */

#define OP_GET_MCP_CLOCK	0x63
#define OP_INDEX_REPORT         0x65

#ifdef __cplusplus
}
#endif

#endif
