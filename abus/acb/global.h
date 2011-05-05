
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
 * global.h  ACCESS.bus manager
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
 * Portions of this code were created by and are
 * copyrighted by Computer Access Technology Corp.
 *
 * Permission to use this software is herby granted by
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
/*                             abus.h                               */
/********************************************************************/

#ifndef _ABUS_H_
#define _ABUS_H_

#define     MC_ADDR     0x50
#define     MGR_ADDR    0x51
#define     ABDEF_ADDR  0x6E

/* some useful self test constants */

#define ST_SUCCESS      0x00
#define ST_ROM_CHECKSUM 0x01
#define ST_INT_RAM      0x02
#define ST_EXT_RAM      0x04
#define ST_POWER_DOWN   0x08
#define ST_DEVICE       0x10

/* Status bits */

#define AB_STATUS_LINKED    0x80
#define AB_STATUS_GOOD      0x08
#define AB_STATUS_ACTIVE    0x04
#define AB_STATUS_PHYS      0x02
#define AB_STATUS_ENABLE    0x01

/* Required Control/Status opcodes */
/*   Protocol bit == 1 */

/* 0x00 - 0x7F are reserved for vendor usage */

/* ACCESS.bus defined opcodes */

#define AB_APPL_HDW_SIG   0xA0 /* Application Hdw Signal; optional */
#define AB_APPL_TST_RPL   0xA1 /* Application Test Reply */
#define AB_APPL_STAT_MSG  0xA2 /* Application Status Message */

#define AB_APPL_TST       0xB1 /* Application Test */

/* 0xC0 - 0xC8 are reserved for vendor usage */

#define AB_ATTN           0xE0 /* Attention */
#define AB_IDENT_RPL      0xE1 /* Identification Reply */
#define AB_RES_E2         0xE2 /* Reserved */
#define AB_CAP_RPL        0xE3 /* Capabilities Reply */
#define AB_RES_E4         0xE4 /* Reserved */
#define AB_RSRC_REQ       0xE5 /* Resource Request; optional */
#define AB_PWR_USG_RPL    0xE6 /* Power Usage Reply; optional */
#define AB_RES_E7         0xE7 /* Reserved */
#define AB_BW_USG_RPL     0xE8 /* Bandwidth Usage Reply; optional */

#define AB_RESET          0xF0 /* Reset */
#define AB_IDENT_REQ      0xF1 /* Identification Request */
#define AB_ASGN_ADDR      0xF2 /* Assign Address */
#define AB_CAP_REQ        0xF3 /* Capabilities Request */
#define AB_RSRC_GRANT     0xF4 /* Resource Grant; optional */
#define AB_EN_AP_REP      0xF5 /* Enable Application Report */
#define AB_PWR_MGMT       0xF6 /* Power Management; optional */
#define AB_PRES_CHK       0xF7 /* Presence Check; optional */
#define AB_DEV_BW_MGMT    0xF8 /* Device Bandwidth Mngmnt; optional */


/********************************************************************
* Driver to manager commands
*********************************************************************/
/*
* External interface
*/
#define     D2M_EXT_START          0x20
#define     D2M_RESET              0x20
#define     D2M_LINK_REQUEST       0x21
#define     D2M_LINK_APPROVE       0x22
#define     D2M_GET_LONG_ID        0x23
#define     D2M_GET_ID             0x24
#define     D2M_GET_TYPE           0x25
#define     D2M_GET_STATUS         0x26
#define     D2M_ENABLE             0x27
#define     D2M_MSG_TO_DEVICE      0x28
#define     D2M_DISCONNECT         0x29
#define     D2M_EXT_END            0x29

/********************************************************************
* Manager to driver commands
*********************************************************************/
#define     M2D_START              0x40
#define     M2D_LINK_REPLY         0x40
#define     M2D_LONG_ID            0x41
#define     M2D_ID                 0x42
#define     M2D_TYPE               0x43
#define     M2D_STATUS             0x44
#define     M2D_MSG_TO_DRIVER      0x45
#define     M2D_DISCONNECT         0x46
#define     M2D_PDT                0x47
#define     M2D_ERROR              0x48
#define     M2D_LINK_APPROVE_ACK   0x49
#define     M2D_END                0x49


/********************************************************************
* Message Types
*********************************************************************/

/*******************************************************************
* MsgHdr
*
*     byte   field
*       1 - minor op-code
*       2 - major op-code
*       3 - device id
*       4 - message length
*
*     on little endian architectures (i.e. Intel):
*
*	First word (2 bytes) :  LSB - minor opcode
*				MSG - major opcode
*	Second word (2 bytes):	LSB - device id
*				MSB - message length
*
*********************************************************************/

typedef struct {
   unsigned char minor;
   unsigned char major;
   unsigned char devId;
   unsigned char msgLen;
} MsgHdr;


/********************************************************************
* BusMsgHdr
*
*       byte    field
*          1    source address
*          2    destination address
*          3    length (plus high bit protocol)
*
********************************************************************/

typedef struct {
  unsigned char  dest;
  unsigned char  src;
  unsigned char  len;		/* length with protocol */
} BusMsgHdr;


#define     MSG_DATA_LEN              (128)
#define     ABMSG_LEN                 (MSG_DATA_LEN+sizeof(MsgHdr))
#define     BUSMSG_LEN                (MSG_DATA_LEN+sizeof(BusMsgHdr))


/*
 * Ab Manager types
 *
 */

typedef struct {
   unsigned char protRev;
   unsigned char moduleRev[7];
   unsigned char vendorName[8];
   unsigned char moduleName[8];
   long          devNum;
} DevId;

typedef struct {
   unsigned char prot[9];
   unsigned char type[9];
   unsigned char model[9];
} DevProt;

typedef union {
   struct {
      unsigned char enable :1;
      unsigned char real   :1;
      unsigned char active :1;
      unsigned char good   :1;
      unsigned char kbd    :1;
      unsigned char dummy  :2;
      unsigned char inuse  :1;
   } flags;
   unsigned char byte;
} DevStatus;

#if 0
typedef enum {
   ST_SUCCESS      = 0x00,
   ST_ROM_CHECKSUM = 0x01,
   ST_INT_RAM      = 0x02,
   ST_EXT_RAM      = 0x04,
   ST_POWER_DOWN   = 0x08,
   ST_DEVICE       = 0x10
} DevSelfTest;
#endif

/*
 * Composite message types
 */

typedef struct {
   DevId     devId;
   DevProt   devProt;
   DevStatus devStatus;
} DevLongId;

typedef union {
   MsgHdr         hdr;
   unsigned char  data[ABMSG_LEN];
   unsigned short word[(ABMSG_LEN+1)/2]; /* +1 assures round up */
} ABMSG;

typedef union {
   BusMsgHdr      hdr;
   unsigned char  data[BUSMSG_LEN];
   unsigned short word[(BUSMSG_LEN+1)/2]; /* +1 assures round up */
} BUSMSG;


/********************************************************************
* Message element offset #defines
* 
*  The A.b messages are defined by byte offsets in a byte buffer.
*  This is not assured to be cross-machine compatible with structs
*  which may be padded in places to achieve word or long word
*  alignment.  For this reason, the use of structs for other than
*  byte elements at the beginning of a union is questionable.
*  These #defines are an alternative that is assured to match
*  the A.b message definitions and can be used on ABMSG.data[].
*  The use of the ABMSG.hdr element seems quite safe, though
*  defines are given anyway.
*
*********************************************************************/

/*
 * The M2D/D2M header
 */

#define ABMSG_MINOR_OFFSET      (0)
#define ABMSG_MINOR_LEN         (1)

#define ABMSG_MAJOR_OFFSET      (1)
#define ABMSG_MAJOR_LEN         (1)

#define ABMSG_DEVID_OFFSET      (2)
#define ABMSG_DEVID_LEN         (1)

#define ABMSG_MSGLEN_OFFSET     (3)
#define ABMSG_MSGLEN_LEN        (1)

/*
 * Long Id
 *    Used for M2D_LINK_REPLY (0x40) and M2D_LONG_ID (0x41)
 */

#define LONGID_PROT_REV_OFFSET  (4)
#define LONGID_PROT_REV_LEN     (1)

#define LONGID_MOD_REV_OFFSET   (5)
#define LONGID_MOD_REV_LEN      (7)

#define LONGID_VENDOR_OFFSET   (12)
#define LONGID_VENDOR_LEN       (8) 

#define LONGID_MOD_NAME_OFFSET (20)
#define LONGID_MOD_NAME_LEN     (8)

#define LONGID_DEV_NUM_OFFSET  (28)
#define LONGID_DEV_NUM_LEN      (4)

#define LONGID_PROT_OFFSET     (32)
#define LONGID_PROT_LEN         (9)

#define LONGID_TYPE_OFFSET     (41)
#define LONGID_TYPE_LEN         (9)

#define LONGID_MODEL_OFFSET    (50)
#define LONGID_MODEL_LEN        (9)

#define LONGID_STATUS_OFFSET   (59)
#define LONGID_STATUS_LEN       (1)

/*
 * Dev Id
 *    Used for M2D_ID (0x42)
 */

#define DEVID_PROT_REV_OFFSET   (4)
#define DEVID_PROT_REV_LEN      (1)

#define DEVID_MOD_REV_OFFSET    (5)
#define DEVID_MOD_REV_LEN       (7)

#define DEVID_VENDOR_OFFSET    (12)
#define DEVID_VENDOR_LEN        (8) 

#define DEVID_MOD_NAME_OFFSET  (20)
#define DEVID_MOD_NAME_LEN      (8)

#define DEVID_DEV_NUM_OFFSET   (28)
#define DEVID_DEV_NUM_LEN       (4)

/*
 * Dev Type
 *   Used for D2M_LINK_REQ (0x21) and M2D_TYPE (0x43)
 */

#define DEVTYPE_PROT_OFFSET     (4)
#define DEVTYPE_PROT_LEN        (9)

#define DEVTYPE_TYPE_OFFSET    (13)
#define DEVTYPE_TYPE_LEN        (9)

#define DEVTYPE_MODEL_OFFSET   (22)
#define DEVTYPE_MODEL_LEN       (9)


#endif /* _ABUS_H_ */

