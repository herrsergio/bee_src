
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
 * rwiab.h   Standard RWI A.bus defines and structs.
 *
 * Tyson D. Sawyer
 * tyson@rwii.com
 * Real World Interface, Inc.
 *
 * The creation and continued develoment of this software
 * is sponsored and directed by RWI in the interest of
 * providing the mobile robotics and AI research communities
 * with a well designed and robust Robot Applications
 * Interface (RAI) for the complete line of RWI mobile robots. 
 */

/*
 * Copyright 1995, Real World Interface, Inc.
 *
 * Permission to use and modify this software is herby granted by
 * Real World Interface, Inc. (RWI) for non-commercial uses without
 * fee, provided, however, that the above copyright notice appear in
 * all copies, that both the copyright notice and this permission
 * notice appear in supporting documentation.  This permission may be
 * extended to commercial use of this software with specific written
 * permision from RWI.  RWI makes no representations about the
 * suitability of this software for any purpose.  It is provided
 * "as is" without express or implied warranty.  RWI requests
 * notification of any modifications to this software or its
 * documentation.
 *
 *   ==Contact  support@rwii.com  for further information==
 */

/********************************************************************/
/*                             rwiab.h                              */
/********************************************************************/

/* Warning:
 * This file will change considerably.
 *   -Tyson
 */

#ifndef _RWIAB_H_
#define _RWIAB_H_

#define AB_RWI_STDERR       1
#define AB_RWI_STDERR_ASYNC 2
#define AB_RWI_STDREP       3
#define AB_RWI_STDREP_ASYNC 4

/**********************
*** RWI defined opcodes
**********************/

#define AB_RWI_OPCODE      0x7E /* RWI opcode */
#define AB_MSP_OPCODE      0x77 /* RWI MSP opcode */

#define AB_RWI_DBG_STR     0x01 /* Debug Output */
#define AB_RWI_DBG_BIN     0x02 /* Debug Code (binary) */
#define AB_RWI_AB_TX       0x03 /* Ab Trasmit */
#define AB_RWI_CONFIG_REQ  0x04 /* System Configuration */
#define AB_RWI_CONFIG_RPL  0x05 /* System Configuration Reply */
#define AB_RWI_VR_REQ      0x06 /* Version Request */
#define AB_RWI_VR_RPL      0x07 /* Version Reply */
#define AB_RWI_RD_MEM_REQ  0x08 /* Read Memory request */
#define AB_RWI_RD_MEM_RPL  0x09 /* Read Memory reply */
#define AB_RWI_WR_MEM      0x0A /* Write Memory */
#define AB_RWI_EXEC        0x0B /* Execute at Address */
#define AB_RWI_SET_FD      0x0C	/* send async reports to this port */

/*******************
*** RWI Message body
*******************/

typedef struct {
   unsigned char major;
   unsigned char minor;
} RWI_BodyHdr;

#define   RWI_DATA_LEN     (MSG_DATA_LEN-sizeof(RWI_BodyHdr))

typedef struct {
   RWI_BodyHdr   rwihdr;
   unsigned char data[RWI_DATA_LEN];
} RWI_Body;

typedef struct {
   RWI_BodyHdr   rwihdr;
} RWI_ReqBody;


/************************
*** RWI Protocol messages
************************/

typedef struct {
   RWI_BodyHdr   rwihdr;
   char          str[RWI_DATA_LEN];
} RWI_DbgStrBody;

typedef struct {
   RWI_BodyHdr   rwihdr;
   unsigned char data[RWI_DATA_LEN];
} RWI_DbgBinBody;

typedef struct {
   RWI_BodyHdr   rwihdr;        /* Danger!  This structure is */
                                /* larger than the standard   */
                                /* AbMsg structure, because of*/
   BUSMSG        msg;           /* the nested AbMsg type.     */
} RWI_AbTxBody;

typedef RWI_ReqBody RWI_ConfigReqBody;

typedef struct {
   RWI_BodyHdr   rwihdr;
   short         config;
} RWI_ConfigRplBody;

typedef RWI_ReqBody RWI_VrReqBody;

typedef struct {
   RWI_BodyHdr   rwihdr;
   char          ver[RWI_DATA_LEN];
} RWI_VrRplBody;

typedef struct {
  RWI_BodyHdr   rwihdr;
#if 0
  unsigned long addr;
#else
  unsigned short addr0;
  unsigned short addr1;
#endif
  unsigned char len;
} RWI_RdMemReqBody;

typedef struct {
   RWI_BodyHdr   rwihdr;
#if 0
   unsigned long addr;
#else
   unsigned short addr0;
   unsigned short addr1;
#endif
   unsigned char data[RWI_DATA_LEN-sizeof(unsigned long)];
} RWI_RdMemRplBody;

typedef struct {
   RWI_BodyHdr   rwihdr;
#if 0
   unsigned long addr;
#else
   unsigned short addr0;
   unsigned short addr1;
#endif
   unsigned char data[RWI_DATA_LEN-sizeof(unsigned long)];
} RWI_WrMemBody;

typedef struct {
   RWI_BodyHdr   rwihdr;
   unsigned long addr;
} RWI_ExecBody;

typedef struct {
   RWI_BodyHdr   rwihdr;
   unsigned char fdId;
   char          dev[RWI_DATA_LEN-sizeof(unsigned char)];
} RWI_SetFdBody;

#endif /* _RWIAB_H_ */
