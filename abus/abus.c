
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
 * libabdriver.c   library for abus device drivers
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


/*****************************************************************
 *
 *  Debuging utilites
 *
 *****************************************************************/

#define DEBUG_ALERT   /* Problem with driver or manager */
#define DEBUG_WARN    /* Problem with client */

#undef  DEBUG_PGM_TRACE
#undef  DEBUG_CMD     /* Commands sent to manager */

#undef  DEBUG_MSG_1
#undef  DEBUG_MSG_2
#undef  DEBUG_MSG_3   /* trace device status byte */

#define DBG_PRE_FMT "%s:%14s:%5d:%14s(): "
#define DBG_PRE_ARGS dateStr(), __FILE__, __LINE__, __FUNCTION__

#define _pgmTrace() \
  { \
    fprintf(stderr, DBG_PRE_FMT "\n", DBG_PRE_ARGS); \
  }

#define _return_d(x) \
  { \
    fprintf(stderr, DBG_PRE_FMT "return(0x%08X, %d);\n", \
	   DBG_PRE_ARGS, (unsigned)x, (int)x); \
    return(x); \
  }

#define _return_v \
  { \
    fprintf(stderr, DBG_PRE_FMT "return(void);\n", \
           DBG_PRE_ARGS); \
    return; \
  }

#ifdef DEBUG_PGM_TRACE

#define pgmTrace() \
  { \
    fprintf(stderr, DBG_PRE_FMT "\n", DBG_PRE_ARGS); \
  }

#define return_d(x) \
  { \
    fprintf(stderr, DBG_PRE_FMT "return(0x%08X, %d);\n", \
	   DBG_PRE_ARGS, (unsigned)x, (int)x); \
    return(x); \
  }

#define return_v \
  { \
    fprintf(stderr, DBG_PRE_FMT "return(void);\n", \
           DBG_PRE_ARGS); \
    return; \
  }

#else /* DEBUG_PGM_TRACE */

#define pgmTrace() {}
#define return_d(x) return(x)
#define return_v return

#endif /* DEBUG_PGM_TRACE */

#ifdef DEBUG_ALERT
#define dbgAlert(format, args...) fprintf(stderr, \
                                        DBG_PRE_FMT "ALERT:" format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgAlert(A...) {}
#endif /* DEBUG_ALERT */

#ifdef DEBUG_WARN
#define dbgWarn(format, args...) fprintf(stderr, \
                                        DBG_PRE_FMT "Warn:" format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgWarn(A...) {}
#endif /* DEBUG_WARN */

#ifdef DEBUG_CMD
#define dbgCmd(format, args...) fprintf(stderr, \
                                        DBG_PRE_FMT "Cmd:" format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgCmd(A...) {}
#endif /* DEBUG_CMD */

#ifdef DEBUG_MSG_1
#define dbgMsg1(format, args...) fprintf(stderr, \
                                        DBG_PRE_FMT "dbgMsg1:" format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgMsg1(A...) {}
#endif /* DEBUG_MSG_1 */

#ifdef DEBUG_MSG_2
#define dbgMsg2(format, args...) fprintf(stderr, \
                                        DBG_PRE_FMT "dbgMsg2:" format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgMsg2(A...) {}
#endif /* DEBUG_MSG_2 */

#ifdef DEBUG_MSG_3
#define dbgMsg3(format, args...) fprintf(stderr, \
                                        DBG_PRE_FMT "dbgMsg3" format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgMsg3(A...) {}
#endif /* DEBUG_MSG_1 */

/*****************************************************************
 *
 * End of debug utilities
 *
 *****************************************************************/


#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

/* #include <curses.h> */

#include <netinet/in.h>  /* for byteorder */
#include <time.h>

#include "acb/global.h"
#include "acb/abus.h"

abdDevType abdDev[ABD_MAX_DEVS];

static int (*msgToDriverCB)(ABMSG *msg);
static int fd1;

const char *dateStr(void) {
  time_t timeValue;

  char *datePtr = ctime((time(&timeValue), &timeValue));
  datePtr[strlen(datePtr)-1] = 0; /* strip the '\n' */
  return(datePtr);
}

int
abDriverLibInit (int fd, int (*M2D_CB)(ABMSG *msg)) {
  int ii;
  
  pgmTrace();

  fd1 = fd;
  msgToDriverCB = M2D_CB;
  
  for (ii=0; ii<ABD_MAX_DEVS; ii++) {
    abdDev[ii].state = ABD_DEV_STATE_NOT;
    abdDev[ii].busId = 0;
    abdDev[ii].fd = 0;
  }
  
  return_d(0);
}

int
msgToMgr(const ABMSG *msg) {
  ABMSG msg2;
  int devId;
  int busId;
  int fd;
  int retval;

  pgmTrace();

  devId = msg->hdr.devId;
  if ((devId<0) || (devId>ABD_MAX_DEVS)) {
    dbgWarn("devId out of range\n");
    return_d(-1);
  }

  if ((abdDev[devId].state == ABD_DEV_STATE_NOT) && (devId!=0)) {
    dbgWarn("non-existent devId %d\n", devId);
    return_d(-1);
  }

  if (devId==0) {
    busId = 0;
    fd = fd1;
  }
  else {
    busId = abdDev[devId].busId;
    if ((busId<1) || (busId>255) || (busId&1)) {
      dbgWarn("illegal busId: %d, devId = %d\n", busId, devId);
      return_d(-1);
    }
    fd = abdDev[devId].fd;
  }

  if ((fd<1) || (fd>32)) {
    dbgWarn("illegal fd: %d\n", fd);
    return_d(-1);
  }

  memcpy(&msg2, msg, sizeof(msg2));

  msg2.hdr.devId = busId;

  retval = write(fd, (char *)&msg2, sizeof(msg2));
  if (retval>=0) {
    retval = 0;
    return_d(retval);
  }

  dbgWarn("write() failed - 1\n");

  usleep((unsigned long)(.01*1000000));

  retval = write(fd, (char *)&msg2, sizeof(msg2));
  if (retval>=0) {
    retval = 0;
    return_d(retval);
  }

  dbgAlert("write() failed - 2\n");

  usleep((unsigned long)(.05*1000000));

  retval = write(fd, (char *)&msg2, sizeof(msg2));
  if (retval>=0) {
    retval = 0;
    return_d(retval);
  }

  dbgAlert("write() failed - 3\n");

  return_d(retval);
}


/***************************************************
 * send messages and data to the abus manager
 **************************************************/

int
D2M_reset (void) {
  ABMSG msg;

  pgmTrace();

  msg.hdr.major  = D2M_RESET;
  msg.hdr.minor  = 0;
  msg.hdr.devId  = 0;
  msg.hdr.msgLen = 0;
  
  return_d(msgToMgr(&msg));
}

int
D2M_linkRequest (const char *prot, const char *type, const char *model) {
  ABMSG msg;

  pgmTrace();

  msg.hdr.major = D2M_LINK_REQUEST;
  msg.hdr.minor = 0;
  msg.hdr.devId = 0;
  msg.hdr.msgLen = 0x1b;

  strcpy(&msg.data[DEVTYPE_PROT_OFFSET], prot);
  strcpy(&msg.data[DEVTYPE_TYPE_OFFSET], type);
  strcpy(&msg.data[DEVTYPE_MODEL_OFFSET], model);

  return_d(msgToMgr(&msg));
}

int
D2M_linkApprove (int devId, int approve) {
  ABMSG msg;
  int retval;

  pgmTrace();

  msg.hdr.major = D2M_LINK_APPROVE;
  msg.hdr.minor = approve;
  msg.hdr.devId = devId;
  msg.hdr.msgLen = 0;

  retval = msgToMgr(&msg);
  if (retval) {
    return_d(retval);
  }

  if (approve && (devId != 0)) abdDev[devId].state = ABD_DEV_STATE_ACK;
  else abdDev[devId].state = ABD_DEV_STATE_NOT;

  return_d(retval);
}

int
D2M_getLongId (int devId) {
  ABMSG msg;

  pgmTrace();

  msg.hdr.major = D2M_GET_LONG_ID;
  msg.hdr.minor = 0;
  msg.hdr.devId = devId;
  msg.hdr.msgLen = 0;

  return_d(msgToMgr(&msg));
}

int
D2M_getId (int devId) {
  ABMSG msg;

  pgmTrace();

  msg.hdr.major = D2M_GET_ID;
  msg.hdr.minor = 0;
  msg.hdr.devId = devId;
  msg.hdr.msgLen = 0;

  return_d(msgToMgr(&msg));
}

int
D2M_getType (int devId) {
  ABMSG msg;

  pgmTrace();

  msg.hdr.major = D2M_GET_TYPE;
  msg.hdr.minor = 0;
  msg.hdr.devId = devId;
  msg.hdr.msgLen = 0;

  return_d(msgToMgr(&msg));
}

int
D2M_getStatus (int devId) {
  ABMSG msg;

  pgmTrace();

  msg.hdr.major = D2M_GET_STATUS;
  msg.hdr.minor = 0;
  msg.hdr.devId = devId;
  msg.hdr.msgLen = 0;

  return_d(msgToMgr(&msg));
}

int
D2M_enable (int devId, int enable) {
  ABMSG msg;
  int retval;

  pgmTrace();

  msg.hdr.major = D2M_ENABLE;
  msg.hdr.minor = enable;
  msg.hdr.devId = devId;
  msg.hdr.msgLen = 0;

  retval = msgToMgr(&msg);

  /*
   * _HACK_ !!!!  There is a race with the manager here!
   * _GROSS_ !!!
   *
   * This message goes to the manager which does 
   * some book keeping and then sets the enable 
   * flages in the kernel and then sends a message
   * to the controller card.  If this client attempts
   * to send a message to the device on the bus before
   * the manager has finished its work, the send will fail.
   */

  usleep((unsigned long)(.05*1000000));

  return_d(retval);
}

int
D2M_msgToDevice (ABMSG *msg) {

  pgmTrace();

  return_d(msgToMgr(msg));
}

int
D2M_disconnect (int devId) {
  ABMSG msg;

  pgmTrace();

  abdDev[devId].state = ABD_DEV_STATE_NOT;

  msg.hdr.major = D2M_DISCONNECT;
  msg.hdr.minor = 0;
  msg.hdr.devId = devId;
  msg.hdr.msgLen = 0;

  return_d(msgToMgr(&msg));
}



/***************************************************
 * handle messages and data from the abus manager
 **************************************************/

int
M2D_longId (ABMSG *msg) {
  abdDevType *dev;
  int major;
  int minor;
  int devId;
  int msgLen;

  pgmTrace();

  major = msg->hdr.major;
  minor = msg->hdr.minor;
  devId = msg->hdr.devId;
  msgLen = msg->hdr.msgLen;

  
  if (minor!=0xFF) {		/* if dev available */

    /* copy vital stats into abdDev structure */
    dev = &abdDev[devId];

    dev->devId.protRev = msg->data[LONGID_PROT_REV_OFFSET];
    dev->devId.devNum =
      ntohl(*(long *)(&(msg->data[LONGID_DEV_NUM_OFFSET])));

    memcpy(dev->devId.moduleRev,
           &msg->data[LONGID_MOD_REV_OFFSET],
           LONGID_MOD_REV_LEN);
    memcpy(dev->devId.vendorName,
           &msg->data[LONGID_VENDOR_OFFSET],
           LONGID_VENDOR_LEN);
    memcpy(dev->devId.moduleName,
           &msg->data[LONGID_MOD_NAME_OFFSET],
           LONGID_MOD_NAME_LEN);
     
    strcpy(dev->devProt.prot,  &(msg->data[LONGID_PROT_OFFSET]));
    strcpy(dev->devProt.type,  &(msg->data[LONGID_TYPE_OFFSET]));
    strcpy(dev->devProt.model, &(msg->data[LONGID_MODEL_OFFSET]));
     
    dev->devStatus.byte = msg->data[LONGID_STATUS_OFFSET];

    dbgMsg3("abDev[%02X].devStatus=0x%02X\n", \
	    dev->devId.devNum, dev->devStatus.byte);

  }
  return_d((*msgToDriverCB)(msg));
}

int
M2D_id (ABMSG *msg) {

  pgmTrace();

  return_d((*msgToDriverCB)(msg));
}

int
M2D_type (ABMSG *msg) {

  pgmTrace();

  return_d((*msgToDriverCB)(msg));
}

int
M2D_status (ABMSG *msg) {

  pgmTrace();

  return_d((*msgToDriverCB)(msg));
}

int
M2D_msgToDriver (ABMSG *msg) {

  pgmTrace();

  return_d((*msgToDriverCB)(msg));
}
    
int
M2D_disconnect (ABMSG *msg) {
  int devId;

  pgmTrace();

  devId = msg->hdr.devId;
  abdDev[devId].state = ABD_DEV_STATE_NOT;
  return_d((*msgToDriverCB)(msg));
}
    
int
M2D_linkApproveAck (ABMSG *msg) {
  int minor;
  int devId;

  pgmTrace();

  minor = msg->hdr.minor;
  devId = msg->hdr.devId;
  if (minor == 0) {
    abdDev[devId].state = ABD_DEV_STATE_RDY;
    abdDev[devId].devStatus.byte |= AB_STATUS_LINKED;
  }
  else {
    abdDev[devId].state = ABD_DEV_STATE_NOT;
  }
  return_d((*msgToDriverCB)(msg));
}

/*****************************************/

int
abDriverLibSelect (int fd) {
  ABMSG msg;
  int major;
  int minor;
  int msgLen;
  int busId;
  int devId;
  int count;

  pgmTrace();

  count = read (fd, (char *)&msg, sizeof (msg));
  if (count != sizeof(msg)) {
    dbgAlert("read() != sizeof(msg)\n");
    return_d(-1);
  }

  major  = msg.hdr.major;
  minor  = msg.hdr.minor;
  msgLen = msg.hdr.msgLen;
  busId  = msg.hdr.devId;

  devId = 0;

  if (busId) {
    for (devId = 1; devId<ABD_MAX_DEVS; devId++) {
      if ((abdDev[devId].state != ABD_DEV_STATE_NOT) &&
	  (abdDev[devId].busId == busId) &&
	  (abdDev[devId].fd == fd)) break;
    }
    
    if (devId == ABD_MAX_DEVS) {	/* find empty abdDev slot */
      for (devId = 1; devId<ABD_MAX_DEVS; devId++) {
	if (abdDev[devId].state == ABD_DEV_STATE_NOT) {
	  memset(&abdDev[devId], 0, sizeof(abdDev[0]));
	  abdDev[devId].state = ABD_DEV_STATE_NEW;
	  abdDev[devId].busId  = busId;
	  abdDev[devId].fd = fd;
	  break;
	}
      }
      
      if (devId == ABD_MAX_DEVS) {
	dbgWarn("No space for new dev\n");
	return_d(0);
      }
    }
  }

  msg.hdr.devId = devId;

  switch (major) {

  case M2D_LINK_REPLY:       return_d(M2D_longId(&msg));
  case M2D_LONG_ID:          return_d(M2D_longId(&msg));
  case M2D_ID:               return_d(M2D_id    (&msg));
  case M2D_TYPE:             return_d(M2D_type  (&msg));
  case M2D_STATUS:           return_d(M2D_status(&msg));
  case M2D_MSG_TO_DRIVER:    return_d(M2D_msgToDriver(&msg));
  case M2D_DISCONNECT:       return_d(M2D_disconnect(&msg));
  case M2D_LINK_APPROVE_ACK: return_d(M2D_linkApproveAck(&msg));
  default:
    dbgWarn("mspM2D - unhandled major\n\n");
    return_d(-1);
  }
}
