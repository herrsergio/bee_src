
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



#ifndef _ARM_MESSAGES_H
#define _ARM_MESSAGES_H

typedef enum {

  /* these are used for both library and server */
  ARM_mastError,  /* 0 */
  ARM_gripError,	
  ARM_wristError,
  ARM_deployArm,
  ARM_stowArm,
  ARM_mastWhere,  /* 5 */
  ARM_gripWhere,
  ARM_wristWhere,
  ARM_wristStopped,
  ARM_mastStopped,
  ARM_gripStopped, /* 10 */
  ARM_gripEngaged,

  /* this is used to make TCX calls to server */
  ARM_subscribe,
  ARM_armLimp,
  ARM_mastLimp,
  ARM_mastHalt,        /* 15 */
  ARM_mastRelativeUp,
  ARM_mastRelativeDown,
  ARM_mastToPos,
  ARM_mastVelocityDown,
  ARM_mastVelocityUp,   /* 20 */
  ARM_gripLimp,
  ARM_gripHalt,
  ARM_gripToPos,
  ARM_gripRelativeOpen,
  ARM_gripRelativeClose, /* 25 */
  ARM_wristLimp,
  ARM_wristHalt,
  ARM_wristToPos,
  ARM_wristRelativePos,
  ARM_wristRelativeNeg   /* 30 */
} armMessage;

#endif
