
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

/****************************************************************/
/*    FILE: Arm.h                                              */
/****************************************************************/

#ifndef _ARM_H
#define _ARM_H

#include <termios.h>
#include <armMessages.h>  
#include <bUtils.h>

/* ARM EXTENTS */

#define MAST_ENCODERS_PER_MM 56.0
#define MAST_ENCODERS_AT_INDEX 0x8600
#define MAST_ENCODERS_AT_STOW 0x911D
#define MAST_TOP_MM 1040

#define GRIP_ENCODERS_PER_MM 154.624
#define GRIP_TRAVEL 0xAB00
#define GRIP_OPEN_MM 250

#define ARM_CONFIG_VARIABLE "ARMHOST"

#ifdef  __cplusplus
extern "C" {
#endif

  typedef void (*armCallback)(armMessage, unsigned long);
  void registerArmEventHandler(armCallback cb);

  /*****/

  void ArmInit(const char *mastDev, speed_t mastBaud,
	       const char *gripDev, speed_t gripBaud);
  void ArmShutdown(void);
  void deployArm();
  void stowArm();
  void armLimp();

  /* Mast */
  void mastLimp();  /* in case of emergency or CTRL-C */
  void mastHalt();  
  void mastRelativeUp(unsigned long amount);
  void mastRelativeDown(unsigned long amount);
  void mastToPos(unsigned long pos);
  void mastVelocityDown();
  void mastVelocityUp();
  void mastWhere(void);

  /* gripper */
  void gripLimp();
  void gripHalt();
  void gripToPos(unsigned long amount);
  void gripRelativeOpen(unsigned long amount);
  void gripRelativeClose(unsigned long amount);
  void gripWhere(void);

  /* wrist */
  void wristLimp(void);
  void wristHalt(void);
  void wristToPos(unsigned long pos);
  void wristRelativePos(unsigned long amount);
  void wristRelativeNeg(unsigned long amount);

  void wristWhere(void);



/* The types of errors the arm can generate */
/* NOTE: THESE ARE NOT AS SPECIFIED IN THE RWI MANUAL.*/
/* the MCP returns 04 on both rotate and translate errors */

#define MAST_EXTEND_ERROR 0x04
#define TC   0x02
#define MAST_RAISE_ERROR 0x01
#define RC   0x08
#define BHI  0x10
#define BLO  0x20

#define GRIP_ROTATE_ERROR 0x04



#ifdef  __cplusplus
}
#endif


#endif

