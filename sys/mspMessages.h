
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



/********************************************************************/
/*                    msp/include/messages.h                        */
/********************************************************************/

#ifndef _MSP_MESSAGES_H_
#define _MSP_MESSAGES_H_

#include <acb/global.h>		/* standard A.b defines and typedefs */
#include <acb/rwiab.h>		/* standard RWI defines and typedefs */

/* 0x00 - 0x7F are reserved for vendor usage */
/* 0xC0 - 0xC8 are reserved for vendor usage */

/* MSP Control/Status opcodes */
/*   Protocol bit == 1 */

#define MSP_AD_REQ         0x11
#define MSP_AD_RPL         0x12
#define MSP_AD_PARMS       0x13	/* not yet supported */
#define MSP_AD_PARMS_RPL   0x14	/* not yet supported */

#define MSP_BMP_REQ        0x21
#define MSP_BMP_RPL        0x22
#define MSP_BMP_PARMS      0x23	/* not yet supported */
#define MSP_BMP_PARMS_RPL  0x24	/* not yet supported */

#define MSP_IR_REQ         0x31
#define MSP_IR_RPL         0x32
#define MSP_IR_PARMS       0x33
#define MSP_IR_PARMS_RPL   0x34

#define MSP_SON_REQ        0x41
#define MSP_SON_RPL        0x42
#define MSP_SON_PARMS      0x43
#define MSP_SON_PARMS_RPL  0x44
#define MSP_SON_TABLE      0x45
#define MSP_SON_TABLE_RPL  0x46
#define MSP_SON_START      0x47

/* MSP needs space in its BusMsg's to record Rx time... */
/* ... so we have yet an other typdef...               */

typedef struct {
  BusMsgHdr hdr;
  unsigned char data[MSG_DATA_LEN];
  unsigned short time;
} MSP_Msg;

/* AD converter messages */

#define MSP_AD_NUM 8

typedef RWI_ReqBody MSP_AdReqBody;

typedef struct {
  RWI_BodyHdr   rwihdr;
  unsigned char AdVals[MSP_AD_NUM];
} MSP_AdRplBody;


/* Bump messags */

#define MSP_BMP_NUM 1

typedef RWI_ReqBody MSP_BmpReqBody;

typedef struct {
  RWI_BodyHdr   rwihdr;
  unsigned char BmpVals[MSP_BMP_NUM];
} MSP_BmpRplBody;

/* IR messages */

#define MSP_IR_NUM 8

typedef RWI_ReqBody MSP_IrReqBody;

typedef struct {
  RWI_BodyHdr    rwihdr;
  unsigned char IrVals[MSP_IR_NUM];
} MSP_IrRplBody;

typedef struct {
  RWI_BodyHdr   rwihdr;
  unsigned short parms[RWI_DATA_LEN/sizeof(short)];
} MSP_IrParmsBody;

typedef struct {
  RWI_BodyHdr   rwihdr;
  unsigned short irInterval;
} MSP_IrParmsRplBody;

#define MSP_IR_PARM_INTERVAL 1


/* Sonar messages */

typedef struct {
  RWI_BodyHdr    rwihdr;
  unsigned short sonAddr[RWI_DATA_LEN/sizeof(short)];
  /* MSB of sonAddr is MSP number (i.e. dipswitches) */
  /* MSNibble of LSB is chain numer */
  /* LSNibble of LSB is transducer number */
} MSP_SonReqBody;

typedef struct {
  RWI_BodyHdr    rwihdr;
  unsigned short echoNum;
  unsigned short data[RWI_DATA_LEN/sizeof(short)];
} MSP_SonRplBody;

typedef struct {
  RWI_BodyHdr    rwihdr;
  unsigned short parms[RWI_DATA_LEN/sizeof(short)];
} MSP_SonParmsBody;

typedef struct {
  RWI_BodyHdr   rwihdr;
  unsigned short fireInterval;
  unsigned short echoCount;
  unsigned short echoTimeout;
  unsigned short initBlank;
  unsigned short echoBlank;
  unsigned short startDelay;
  unsigned short fireDelay;
} MSP_SonParmsRplBody;

#define MSP_SON_PARM_FIRE_INTERVAL  1
#define MSP_SON_PARM_ECHO_NUMBER    2
#define MSP_SON_PARM_ECHO_TIMEOUT   3
#define MSP_SON_PARM_INIT_BLNK_TIME 4
#define MSP_SON_PARM_ECHO_BLNK_TIME 5
#define MSP_SON_PARM_START_DELAY    6
#define MSP_SON_PARM_FIRE_DELAY     7

typedef struct {
  RWI_BodyHdr    rwihdr;
  unsigned short table[RWI_DATA_LEN/sizeof(short)];
} MSP_SonTableBody;

typedef struct {
  RWI_BodyHdr    rwihdr;
  unsigned short table[RWI_DATA_LEN/sizeof(short)];
} MSP_SonTableRplBody;

typedef struct {
  RWI_BodyHdr    rwihdr;
  unsigned char  op;
} MSP_SonStartBody;

#define MSP_SON_START_START 1
#define MSP_SON_START_STOP  2

/*

MSP_SON_RPL
  short   data
    0      sonar address
    1      first return value
    2      second return value
    N      Nth return value
    ...
 echoNum-1 return value
 echoNum   second sonar address
 echoNum+1 first return value of second sonar
    ...
      
MSP_SON_TABLE
  short   data
    0     first sonar address
    1     second sonar address
    ...
    0000  indicating end of first sonar list
    ...
    0000  end of second list
    ...
      
*/
      
#endif _MSP_MESSAGES_H_
