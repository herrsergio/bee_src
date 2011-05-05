
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
 * abusd.c   ACCESS.bus manager/daemon
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


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#if defined(linux) || defined(BSD)
#include <sys/ioctl.h>
#endif /* linux || BSD */
#include <ctype.h>
#include <string.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <errno.h>
#include <time.h>
#include <syslog.h>

#include "acb/global.h"
#include "acb/catc.h"


/*****************************************************************
 *
 *  Debuging utilites
 *
 *****************************************************************/

#define DEBUG_INFORM
#define DEBUG_ALERT   /* Problem with driver or manager */
#define DEBUG_WARN    /* Problem with client */

#undef  DEBUG_PGM_TRACE
#undef  DEBUG_CMD     /* Commands sent to manager */

#undef  DEBUG_MSG_1
#undef  DEBUG_MSG_2
#undef  DEBUG_MSG_3   /* trace device status byte */

#define DBG_PRE_FMT "%10s:%5d:%14s(): "
#define DBG_PRE_ARGS __FILE__, __LINE__, __FUNCTION__
#define LOG_PRIORITY LOG_DAEMON | LOG_NOTICE

#define _pgmTrace() \
  { \
    syslog(LOG_PRIORITY, DBG_PRE_FMT"\n", DBG_PRE_ARGS); \
  }

#define _return_d(x) \
  { \
    syslog(LOG_PRIORITY, DBG_PRE_FMT "return(0x%08X, %d);\n", \
	   DBG_PRE_ARGS, (unsigned)x, (int)x); \
    return(x); \
  }

#define _return_v \
  { \
    syslog(LOG_PRIORITY, DBG_PRE_FMT "return(void);\n", \
           DBG_PRE_ARGS); \
    return; \
  }

#ifdef DEBUG_PGM_TRACE

#define pgmTrace() \
  { \
    syslog(LOG_PRIORITY, DBG_PRE_FMT "\n", DBG_PRE_ARGS); \
  }

#define return_d(x) \
  { \
    syslog(LOG_PRIORITY, DBG_PRE_FMT "return(0x%08X, %d);\n", \
	   DBG_PRE_ARGS, (unsigned)x, (int)x); \
    return(x); \
  }

#define return_v \
  { \
    syslog(LOG_PRIORITY, DBG_PRE_FMT "return(void);\n", \
           DBG_PRE_ARGS); \
    return; \
  }

#else /* DEBUG_PGM_TRACE */

#define pgmTrace() {}
#define return_d(x) return(x)
#define return_v return

#endif /* DEBUG_PGM_TRACE */

#ifdef DEBUG_INFORM
#define dbgInform(format, args...) syslog(LOG_PRIORITY, \
                                        DBG_PRE_FMT format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgInform(A...) {}
#endif /* DEBUG_WARN */

#ifdef DEBUG_ALERT
#define dbgAlert(format, args...) syslog(LOG_PRIORITY, \
                                        DBG_PRE_FMT "ALERT: " format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgAlert(A...) {}
#endif /* DEBUG_ALERT */

#ifdef DEBUG_WARN
#define dbgWarn(format, args...) syslog(LOG_PRIORITY, \
                                        DBG_PRE_FMT "Warn: " format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgWarn(A...) {}
#endif /* DEBUG_WARN */

#ifdef DEBUG_CMD
#define dbgCmd(format, args...)  syslog(LOG_PRIORITY, \
                                        DBG_PRE_FMT "Cmd: " format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgCmd(A...) {}
#endif /* DEBUG_CMD */

#ifdef DEBUG_MSG_1
#define dbgMsg1(format, args...) syslog(LOG_PRIORITY, \
                                        DBG_PRE_FMT "dbgMsg1: " format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgMsg1(A...) {}
#endif /* DEBUG_MSG_1 */

#ifdef DEBUG_MSG_2
#define dbgMsg2(format, args...) syslog(LOG_PRIORITY, \
                                        DBG_PRE_FMT "dbgMsg2: " format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgMsg2(A...) {}
#endif /* DEBUG_MSG_2 */

#ifdef DEBUG_MSG_3
#define dbgMsg3(format, args...) syslog(LOG_PRIORITY, \
                                        DBG_PRE_FMT "dbgMsg3: " format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgMsg3(A...) {}
#endif /* DEBUG_MSG_1 */

/*****************************************************************
 *
 * End of debug utilities
 *
 *****************************************************************/


#define AB_DVR_TYPE_NONE   0
#define AB_DVR_TYPE_USER   1
#define AB_DVR_TYPE_KERNEL 2

#define AB_DEV_STATE_NOT   0
#define AB_DEV_STATE_BAD   1
#define AB_DEV_STATE_CAP   2
#define AB_DEV_STATE_RDY   3
#define AB_DEV_STATE_LNK   4

#define LINK_WILDCARD   "*"

/* the magic numbers */
#define MAX_DEVS    (20)
#define MAX_CLIENTS (20)

/* Link status of a driver */
#define AB_DVR_LINK_IDLE     0
#define AB_DVR_LINK_PENDING  1
#define AB_DVR_LINK_REQ      2

typedef struct {
   unsigned char dvrType;	/* DVR_TYPE_USER/KERNEL */
   unsigned char linkState;	/* idle/pending/offered */
   unsigned char opFlg;		/* use LINK_REPLY or DEV_ID? */

   char prot[9];                /* protocol used by driver */
   char type[9];                /* type used by driver */
   char model[9];		/* model used by driver */
} abDvrType;

typedef struct {
   int state;			/* not, bad_dev, cap, ready, linked */
   time_t timeout;		/* Used as timeout to reset a bad_dev */
   int clientNum;		/* num of client dev is linked to */

   DevId     devId;		/* prot, rev, vendor, module, devNum */
   DevProt   devProt;		/* prot, type, model */
   DevStatus devStatus;		/* status */
  
   char  refused[MAX_CLIENTS];
} abDevType;

abDevType abDev[MAX_DEVS];
abDvrType abDvr[MAX_CLIENTS];
unsigned char abMonNum;


const char *dateStr(void) {
  time_t timeValue;

  char *datePtr = ctime((time(&timeValue), &timeValue));
  datePtr[strlen(datePtr)-1] = 0; /* strip the '\n' */
  return(datePtr);
}

static abDevType* getDev (int devId)
{
  pgmTrace();

  if (devId > 0 && devId < MAX_DEVS) {
    return_d(&abDev[devId]);
  }
  return_d(NULL);
}

int abusdInit (char *dev, int irq, int port) {
   int ii;
   int fd;
   ABMSG msg;
   
   struct {
      long irq_;
      long port_;
   } connection_ = {irq,port};
   
   pgmTrace();

   abMonNum = 0;

   for(ii=0; ii<MAX_DEVS; ii++)
      abDev[ii].state=AB_DEV_STATE_NOT;
   
   for(ii=0; ii<MAX_CLIENTS; ii++)
      abDvr[ii].dvrType=AB_DVR_TYPE_NONE;

   fd = open(dev, O_RDWR);
   if (fd < 0) {
     return_d(fd);
   }
   
   if (ioctl(fd, ABIO_I_AM_MANAGER, NULL) <0) {
     dbgAlert("ioctl(ABIO_I_AM_MANAGER) %s %m\n", dev);
     exit(1);
   }

   while(1) {
     int count=0;
     
     if (ioctl(fd, ABIO_CONNECT_TO_CONTROLLER, (char*)&connection_) >  -1) {
       break;
     }

     if (count++ > 11) {
       count=0;
       dbgAlert("ioctl(ABIO_CONNECT_TO_CONTROLLER) failed: "
		"%s port=0x%04X,irq=%d: %m\n",
		dev, port, irq);
     }
     sleep(5);
   }
   
   /* Get device table */
   msg.hdr.major = M2C_GET_TABLE;
   msg.hdr.minor = 0;
   msg.hdr.devId = 0x50;
   msg.hdr.msgLen = 0;
   
   dbgMsg1("write(M2C_GET_TABLE)\n");

   if (write(fd, (char*)&msg, sizeof(msg)) < 0) {
      dbgAlert("write(M2C_GET_TABLE): %s\n",
              strerror(errno));
      exit(1);
   }
   return_d(fd);
}

void showDvr (int dvrNum) {

#ifdef DEBUG_MSG_1
  abDvrType *dvr = &abDvr[dvrNum];
#endif
  
  pgmTrace();

  dbgMsg1("\tdvrNum: %d  dvrType: %d  linkState: %d  opFlg: %d\n", 
	  dvrNum, dvr->dvrType, dvr->linkState, dvr->opFlg);
  dbgMsg1("\tprot: '%s'  type: '%s'  model: '%s'\n",
	  dvr->prot, dvr->type, dvr->model);
  return_v;
}

void showDev (int devId) {
   abDevType *dev;
#if 0
   int ii;
#endif /* 0 */

   pgmTrace();

   dev = getDev(devId);

   if (dev == NULL) {
      dbgWarn("\tdevId = %d  doesn't exist\n", devId);
      return_v;
   }
     
#if 0
   dbgMsg1("\tdevId = %d  clientNum = %d  state = %d\n",
           devId, dev->clientNum, dev->state);
     
   dbgMsg1("\tProt ver: %c\n", dev->devId.protRev);
     
   dbgMsg1("\tMod rev : ");
     
   for (ii=0; ii<7; ii++) {
      dbgMsg1("%c", dev->devId.moduleRev[ii]);
   }
     
   dbgMsg1("\n");
     
   dbgMsg1("\tVendor  : ");

   for (ii=0; ii<8; ii++) {
      dbgMsg1("%c", dev->devId.vendorName[ii]);
   }

   dbgMsg1("\n");
      
   dbgMsg1("\tMod name: ");
   for (ii=0; ii<8; ii++) {
      dbgMsg1("%c", dev->devId.moduleName[ii]);
   }

   dbgMsg1("\n");
      
   dbgMsg1("\tdev num : %08lX\n", ntohl(dev->devId.devNum));
      
   dbgMsg1("\tprot    : %s\n", dev->devProt.prot);
   dbgMsg1("\ttype    : %s\n", dev->devProt.type);
   dbgMsg1("\tmodel   : %s\n", dev->devProt.model);
    
   dbgMsg1("\tstatus  : %02X\n", dev->devStatus.byte);
   dbgMsg1("\n");
#else
   dbgMsg1( \
	   "   devId:%d clientNum:%d state:%d status:%02X devNum:%08lX\n",
           devId, dev->clientNum, dev->state, \
	   dev->devStatus.byte, ntohl(dev->devId.devNum));
#endif /* 0 */
   return_v;
}

static int
devDvrMatch (abDevType *dev, abDvrType *dvr) {

  /*
   * This is a non-case sensitive compare
   * as per ABIG specs.
   */
  
  if (strncasecmp(dvr->prot, LINK_WILDCARD, sizeof(dvr->prot)) != 0) {
    /* if prot isn't wildcarded */
    if (strncasecmp(dvr->prot, dev->devProt.prot, sizeof(dvr->prot))
	!= 0)
      return(0);              /* we don't match */
  }
  
  if (strncasecmp(dvr->type, LINK_WILDCARD, sizeof(dvr->type)) != 0) {
    /* if type isn't wildcarded */
    if (strncasecmp(dvr->type, dev->devProt.type, sizeof(dvr->type))
	!= 0)
      return(0);              /* we don't match */
  }
  
  if (strncasecmp(dvr->model, LINK_WILDCARD, sizeof(dvr->model)) != 0) {
    /* if model isn't wildcarded */
    if (strncasecmp(dvr->model, dev->devProt.model, sizeof(dvr->model))
	!= 0) {
      return_d(0);              /* we don't match */
    }
  }
  
  /* if we get here, we're golden */
  return_d(1);
}

void
abusdRead (int fd) {
   ABKMSG   kmsg;
   ABMSG   *msg = &kmsg.msg;
   ABKMSG kmsg2;
   int count;
   int ii;
   int clientNum;
   abDevType *dev = NULL;
   abDvrType *dvr = NULL;
   unsigned char minor;
   unsigned char major;
   unsigned char msgLen;
   unsigned char devId;
  
   pgmTrace();

   count = read(fd, (char*)&kmsg, sizeof(kmsg));

   if (count == 0) {
     dbgAlert("no data\n");
     return_v;
   }

   if (count < 0) {
      dbgAlert("read(): %m");
      return_v;
   }
  
   minor = msg->hdr.minor;
   major = msg->hdr.major;
   msgLen = msg->hdr.msgLen;
   devId = msg->hdr.devId;
  
   /*
    * clientNum is obtained from the AbmKernelMsg
    * this will only be valid in D2M and M2D messages
    */

   clientNum = kmsg.clientNum;
   
   
   switch (msg->hdr.major) {

   case D2M_RESET: {

      dev = getDev(devId);
      if (dev == NULL) {
         dbgAlert("bad devId 0x%02X\n",
                 (int)devId);
         return_v;
      }

      dvr = &abDvr[clientNum];
    
      dbgCmd("D2M_RESET: \tclientNum = %d, devId = %d\n", \
	     clientNum, devId);
     
      /* send reset msg to card */
      msg->hdr.major = M2C_RESET; /* D2M -> M2C */

      dbgMsg1("write(M2C_RESET)\n");

      if (write(fd, msg, sizeof(*msg)) < 0) {
         dbgAlert("write(M2C_RESET): %m\n");
      }

      return_v;
   }

   case D2M_LINK_REQUEST: {
#if 0
      abDvrType *dvr = &abDvr[clientNum];
#endif
      dvr = &abDvr[clientNum];
    
      dbgMsg1("D2M_LINK_REQUEST: clientNum = %d\n", clientNum);
     
      if (dvr->dvrType == AB_DVR_TYPE_NONE) {
         dbgAlert("This client isn't connected ?!?\n");
         return_v;
      }
         
      memcpy(dvr->prot, &msg->data[DEVTYPE_PROT_OFFSET],
             DEVTYPE_PROT_LEN);
      memcpy(dvr->type, &msg->data[DEVTYPE_TYPE_OFFSET],
             DEVTYPE_TYPE_LEN);
      memcpy(dvr->model, &msg->data[DEVTYPE_MODEL_OFFSET],
             DEVTYPE_MODEL_LEN);
         
      abDvr[clientNum].linkState = AB_DVR_LINK_REQ;

      /* Is this a monitor request? */
      if (!strcasecmp(dvr->prot, "abmon")) {
         if (!abMonNum) {
            long newMonNum;

            abDvr[clientNum].linkState = AB_DVR_LINK_IDLE;
            abMonNum = clientNum;
            newMonNum = abMonNum;
            ioctl(fd, ABIO_ASSGN_MONITOR, &newMonNum);
         }
         else {
            /*
             * YYY
             * The monitor may use the monitor pid/fd
             * only as a monitor.  Abusd will send a
             * LONG_ID for each existing device.  Abmon will
             * then automatically receive a copy of
             * LONG_IDs as devs are hot-plugged and
             * removed.  If the monitor wishes
             * to link to a dev, it must open an other fd.
             * When the kernel detects closure of the monitor
             * pid/fd, it will report a D2M_DISCONNECT
             * from the monitor client number.  Regardless
             * of the devId, the monitor's fd will then lose
             * monitor status.  Any D2M_DISCONNECT from the
             * monitor will have the same results.  Abusd
             * should (will) refuse any attempt by the
             * monitor to actually accept a device and
             * will send it M2D_DISCONNECT for each dev it
             * may have linked to before requesting monitor
             * status.
             *
             * Summary: ABIG's specs for the abmon are skimpy.
             *
             * The previous abterm was device sensitive
             * and was designed to accept handlers for
             * mulitple dev types.  Abmon will not do
             * this.  Since the current A.b manager
             * (abus.o/abusd) supports multiple clients,
             * an ASCII command interface may be written
             * specific for each dev type.
             */
         }
      }

      return_v;
   }

   case D2M_LINK_APPROVE: {
      struct {
         long devId;
         long arg;
      } ioctlData;
    
      dbgCmd("D2M_LINK_APPROVE\n");

      dev = getDev(devId);

      if ((dev == NULL) && (devId != 0)) {
         dbgWarn("bad device id 0x%02X\n", (int)devId);
         return_v;
      }
       
      dvr = &abDvr[clientNum];

      /*
       * YYY
       * This message should be accepted somewhat asyncronously.
       * This would be more robust than requiring a specific
       * sequence and the docs suggest that the driver may be able
       * to ask for specs on specific devs without doing a link request
       * and then approve a link to the dev.  Though the docs are not
       * clear.
       */


      if (minor == 0) {		/* refuse */
         if (dev) dev->refused[clientNum] = 1;
         dvr->linkState = AB_DVR_LINK_PENDING;
         return_v;
      }

      kmsg2.msg.hdr.major = M2D_LINK_APPROVE_ACK;
      kmsg2.msg.hdr.minor = 0xFF; /* 0xFF = dev not available */
      kmsg2.msg.hdr.msgLen = 0;
      kmsg2.msg.hdr.devId = devId;
      kmsg2.clientNum = clientNum;

      if (dev) {
	if (dev->state != AB_DEV_STATE_RDY) {
	  dvr->linkState = AB_DVR_LINK_PENDING;
	  
	  dbgMsg1("write(M2D_LINK_APPROVE_ACK)\n");

	  if (write (fd, (char *)&kmsg2, sizeof(kmsg2)) < 0) {
	    dbgAlert("write(M2D_LINK_APPROVE_ACK): %m\n");
	  }

	  return_v;
	}
      }
      else {
	dvr->linkState = AB_DVR_LINK_PENDING;
	return_v;
      }
      
      /* tell abusd */
    
      dev->state = AB_DEV_STATE_LNK;

      dbgMsg3("abDev[%02X].devStatus=0x%02X\n", \
	      dev->devId.devNum, dev->devStatus.byte);

      dev->devStatus.byte |= AB_STATUS_LINKED;

      dbgMsg3("abDev[%02X].devStatus=0x%02X\n", \
	      dev->devId.devNum, dev->devStatus.byte);

      dev->clientNum = clientNum;
    
      /* let kernel know */
      ioctlData.devId = devId;
      ioctlData.arg = clientNum;
      ioctl(fd, ABIO_ASSGN_DEV_TO_CLIENT, (char *)&ioctlData);
    
      ioctlData.devId = devId;

      dbgMsg3("abDev[%02X].devStatus=0x%02X\n", \
	      dev->devId.devNum, dev->devStatus.byte);

      ioctlData.arg = abDev[devId].devStatus.byte;
      ioctl(fd, ABIO_SET_DEV_STATUS, &ioctlData);

      /* tell client that link is approved */
      kmsg2.msg.hdr.minor = 0x00; /* 0x00 = dev available */

      dbgMsg1("write(M2D_LINK_APPROVE_ACK)\n");

      if (write (fd, (char *)&kmsg2, sizeof(kmsg2)) < 0) {
	dbgAlert("write(M2D_LINK_APPROVE_ACK): %s\n", strerror(errno));
      }

      showDev(devId);

      return_v;
   }

   case D2M_GET_LONG_ID: {

      dbgMsg1("D2M_GET_LONG_ID: clientNum = %d, devId = %d\n", \
	      clientNum, devId);
     
      /* send message */
      return_v;
   }

   case D2M_GET_ID: {

      dev = getDev(devId);
      if (dev == NULL) {
         dbgWarn("bad device id 0x%02X\n", (int)devId);
         return_v;
      }

      dvr = &abDvr[clientNum];
    
      dbgMsg1("D2M_GET_ID: clientNum = %d, devId = %d\n", clientNum, devId);
     
      /* send message */
      return_v;
   }

   case D2M_GET_TYPE: {

      dev = getDev(devId);
      if (dev == NULL) {
         dbgWarn("bad device id 0x%02X\n",
                 (int)devId);
         return_v;
      }

      dvr = &abDvr[clientNum];
    
      dbgCmd("D2M_GET_TYPE: clientNum = %d, devId = %d\n", clientNum, devId);
     
      /* send message */
      return_v;
   }

   case D2M_GET_STATUS: {

      dev = getDev(devId);
      if (dev == NULL) {
         dbgWarn("bad device id 0x%02X\n", (int)devId);
         return_v;
      }

      dvr = &abDvr[clientNum];
    
      dbgCmd("D2M_GET_STATUS: clientNum = %d, devId = %d\n", clientNum, devId);
     
      /* send message */
      return_v;
   }

   case D2M_ENABLE: {
      struct {
         long devId;
         long arg;
      } ioctlData;

      int retval;

      dbgCmd("D2M_ENABLE: clientNum = %d, devId = %d\n", clientNum, devId);

      pgmTrace();

      dev = getDev(devId);
      if (dev == NULL) {
         dbgWarn("bad device id 0x%02X\n", (int)devId);
         return_v;
      }

      dvr = &abDvr[clientNum];

      /* tell abusd */


      if (minor) {
	dev->devStatus.byte |= AB_STATUS_ENABLE;
	dbgMsg2("ENABLE %02X devStatus=0x%02X\n", \
		dev->devId.devNum, dev->devStatus.byte);
      }
      else {
	dev->devStatus.byte &= ~AB_STATUS_ENABLE;
	dbgMsg2("DISABLE %02X devStatus=0x%02X\n", \
		dev->devId.devNum, dev->devStatus.byte);
      }

      /* tell kernel */
      ioctlData.devId = devId;
      ioctlData.arg = abDev[devId].devStatus.byte;

      retval=ioctl(fd, ABIO_SET_DEV_STATUS, &ioctlData);

      if (retval != 0) {
	dbgAlert("ioctl(ABIO_SET_DEV_STATUS)=%d: %m\n", retval);
      }

      dbgMsg2("abDev[%02X].devStatus=0x%02X\n", \
	      dev->devId.devNum, dev->devStatus.byte);

      /* tell card */
      kmsg2.msg.hdr.major = M2C_ENABLE;
      kmsg2.msg.hdr.minor = minor;
      kmsg2.msg.hdr.msgLen = 0;
      kmsg2.msg.hdr.devId = devId;

      dbgMsg2("write(M2C_ENABLE)\n");

      if (write(fd, (char*)&kmsg2, sizeof(kmsg2)) < 0) {
	dbgAlert("write(M2C_ENABLE): %s\n", strerror(errno));
	if (write(fd, (char*)&kmsg2, sizeof(kmsg2)) < 0) {
	  dbgAlert("write(M2C_ENABLE): %s\n", strerror(errno));
	  if (write(fd, (char*)&kmsg2, sizeof(kmsg2)) < 0) {
	    dbgAlert("write(M2C_ENABLE): %s\n", strerror(errno));
	  }
	}
      }

      /* tell device */
      kmsg2.msg.hdr.major = M2C_MSG_TO_DEVICE;
      kmsg2.msg.hdr.minor = 0x01; /* protocol */
      kmsg2.msg.hdr.devId = msg->hdr.devId;
      kmsg2.msg.data[4] = AB_EN_AP_REP;
      kmsg2.msg.data[5] = minor;
      kmsg2.msg.hdr.msgLen = 2;

      dbgMsg2("write(M2C_MSG_TO_DEVICE)\n");

      if (write(fd, (char*)&kmsg2, sizeof(kmsg2)) < 0) {
	 dbgAlert("write(M2C_MSG_TO_DEVICE): %m\n");
      }

      return_v;
   }

   case D2M_MSG_TO_DEVICE: {

      dev = getDev(devId);
      if (dev == NULL) {
         dbgWarn("bad device id 0x%02X\n", (int)devId);
         return_v;
      }

      dvr = &abDvr[clientNum];

      /* should not get this message */
      dbgAlert("D2M_MSG_TO_DEVICE ?!?: clientNum = %d, devId = %d\n", 
	       clientNum, devId);
     
      return_v;
   }

   case D2M_DISCONNECT: {
      struct {
         long devId;
         long arg;
      } ioctlData;
    

      dbgCmd("D2M_DISCONNECT: clientNum = %d, devId = %d\n", 
	     clientNum, devId);
     
      dev = getDev(devId);
      if (dev == NULL) {
         dbgCmd("bad device id 0x%02X\n", (int)devId);
         return_v;
      }

      dvr = &abDvr[clientNum];
    
      /* unlink drvr from dev.  disable dev. */

      /* tell dev */
      kmsg2.msg.hdr.major = M2C_MSG_TO_DEVICE;
      kmsg2.msg.hdr.minor = 0x01; /* protocol */
      kmsg2.msg.hdr.devId = msg->hdr.devId;
      kmsg2.msg.data[4] = AB_EN_AP_REP;
      kmsg2.msg.data[5] = 0;    /* 0 = disable */
      kmsg2.msg.hdr.msgLen = 2;

      dbgMsg1("write(M2C_MSG_TO_DEVICE)\n");

      if (write(fd, (char*)&kmsg2, sizeof(kmsg2)) < 0) {
         dbgAlert("write(M2C_MSG_TO_DEVICE): %m\n");
      }

      /* tell card */
      kmsg2.msg.hdr.major = M2C_ENABLE;
      kmsg2.msg.hdr.minor = 0;
      kmsg2.msg.hdr.msgLen = 0;
      kmsg2.msg.hdr.devId = devId;

      dbgMsg1("write(M2C_ENABLE)\n");

      if (write(fd, (char*)&kmsg2, sizeof(kmsg2)) < 0) {
         dbgAlert("write(M2C_ENABLE): %m\n");
      }

      /* tell abusd */

      dbgMsg3("abDev[%02X].devStatus=0x%02X\n", \
	      dev->devId.devNum, dev->devStatus.byte);

      dev->devStatus.byte &= ~AB_STATUS_ENABLE;

      dev->state = AB_DEV_STATE_RDY;
      dev->clientNum = 0;

      /* tell kernel */
      ioctlData.devId = devId;
      ioctlData.arg = 0;
      ioctl(fd, ABIO_ASSGN_DEV_TO_CLIENT, (char *)&ioctlData);
    
      ioctlData.devId = devId;
      ioctlData.arg = abDev[devId].devStatus.byte;
      ioctl(fd, ABIO_SET_DEV_STATUS, &ioctlData);

      dbgMsg3("abDev[%02X].devStatus=0x%02X\n", \
	      dev->devId.devNum, dev->devStatus.byte);

      return_v;
   }

   case C2M_LONG_ID: {
      struct {
         long devId;
         long arg;
      } ioctlData;
    
      dbgCmd("C2M_LONG_ID\n");
     
      dev = getDev(devId);
      if (dev == NULL) {
         dbgWarn("bad device id 0x%02X\n", (int)devId);
         return_v;
      }

      /* if AB_DEV_STATE_NOT, init new dev struct */

      if (abDev[devId].state == AB_DEV_STATE_NOT) {
         abDev[devId].state = AB_DEV_STATE_BAD; /* a safe default */
         abDev[devId].clientNum = 0;
         abDev[devId].timeout = 0;

	 memset(&abDev[devId].devStatus, 0, sizeof(abDev[devId].devStatus));

         for (ii = 0; ii<MAX_CLIENTS; ii++) {
            abDev[devId].refused[ii] = 0;
         }
      }
      
      /* copy vital stats into abDev structure */
     
      memset(&abDev[devId].devId,     0, sizeof(abDev[devId].devId));
      memset(&abDev[devId].devProt,   0, sizeof(abDev[devId].devProt));

      dev->devId.protRev = msg->data[LONGID_PROT_REV_OFFSET];
      dev->devId.devNum = *(long *)(&(msg->data[LONGID_DEV_NUM_OFFSET]));

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
     
      dev->devStatus.byte = (dev->devStatus.byte & 0xF0) | 
         (msg->data[LONGID_STATUS_OFFSET] & 0x0F);
     
      dbgMsg3("abDev[%02X].devStatus=0x%02X\n", \
	      dev->devId.devNum, dev->devStatus.byte);

      ioctlData.devId = devId;
      ioctlData.arg = dev->devStatus.byte;
      ioctl(fd, ABIO_SET_DEV_STATUS, &ioctlData);

      /*
       * _HACK_
       * {RWI_dev, *, *} -> {RWI-DEV, MSP, 1}
       * This may be removed when all initial release
       * MSP ROMs have been removed from use.
       */

      if (strcmp(dev->devProt.prot, "RWI_dev") == 0) {
         strcpy(dev->devProt.prot, "RWI-DEV");
         strcpy(dev->devProt.type, "MSP");
         strcpy(dev->devProt.model, "1");
      }

      /*
       * end _HACK_
       */

      dbgMsg3("abDev[%02X].devStatus=0x%02X\n", \
	      dev->devId.devNum, dev->devStatus.byte);

      if ((!strcasecmp(dev->devProt.prot, "bad_dev")) ||
          (!(dev->devStatus.byte & AB_STATUS_GOOD))) {
         if ((dev->state==AB_DEV_STATE_LNK) && (dev->clientNum)) {
            /*
             * unlink driver
             * This shouldn't happen.  We should get a disc
             * msg from the controller if a dev goes bad.
             */

            dbgAlert("linked dev changed to bad_dev?!?\n");
         }
       
         dev->timeout = time(NULL)+10; /* allow 10 sec to change to good */
         dev->state = AB_DEV_STATE_BAD;

	 dbgMsg3("abDev[%02X].devStatus=0x%02X\n", \
		 dev->devId.devNum, dev->devStatus.byte);

         if (!dev->devStatus.byte) dev->state = AB_DEV_STATE_NOT;
      }
      else if (dev->state==AB_DEV_STATE_LNK) {
         if (dev->clientNum) {
            /* send update to driver */
	    kmsg.msg.hdr.major = M2D_LONG_ID;
	    kmsg.clientNum = dev->clientNum;


	    dbgMsg3("abDev[%02X].devStatus=0x%02X\n", \
		    dev->devId.devNum, dev->devStatus.byte);

	    msg->data[LONGID_STATUS_OFFSET] = dev->devStatus.byte;
     
	    dbgMsg1("write(M2D_LONG_ID)\n");

            if (write(fd, (char *)&kmsg, sizeof(kmsg)) < 0) {
               dbgAlert("write(M2D_LONG_ID, size=%d): %s\n",
                       sizeof(kmsg), strerror(errno));
	    }
         }
         else {
            dbgAlert("dev is _STATE_LNK but has no clientNum?!?\n");
         }
      }
      else {
         /* dev is now ready */
         dev->state = AB_DEV_STATE_RDY;
      }
     
      /* display vital stats */
     
      showDev(devId);
      return_v;
   }
    
   case C2M_ID: {
      dbgCmd("C2M_ID\n");
      return_v;
   }
    
   case C2M_TYPE: {
      dev = getDev(devId);
      if (dev == NULL) {
         dbgWarn("bad device id 0x%02X\n", (int)devId);
         return_v;
      }
      
      dbgCmd("C2M_TYPE: clientNum = %d, devId = %d\n", 
	     dev->clientNum, devId);

      return_v;
   }
    
   case C2M_SELF_TEST: {
      dev = getDev(devId);
      if (dev == NULL) {
         dbgWarn("bad device id 0x%02X\n", (int)devId);
         return_v;
      }

      dbgCmd("C2M_SELF_TEST: clientNum = %d, devId = %d\n", 
	     dev->clientNum, devId);
   }
    
   case C2M_STATUS: {
      dev = getDev(devId);
      if (dev == NULL) {
         dbgWarn("bad device id 0x%02X\n", (int)devId);
         return_v;
      }

      dbgCmd("C2M_STATUS: clientNum = %d, devId = %d\n", 
	     dev->clientNum, devId);

      return_v;
   }
    
   case C2M_END_TABLE: {
      /*
       * There is nothing to do with this.
       */

      dbgMsg1("C2M_END_TABLE\n");
      return_v;
   }
    
   case C2M_DISCONNECT: {

      struct {
         long devId;
         long arg;
      } ioctlData;
    
      dev = getDev(devId);
      if (dev == NULL) {
         dbgWarn("bad device id 0x%02X\n", (int)devId);
         return_v;
      }

      dvr = &abDvr[dev->clientNum];

      dbgCmd("C2M_DISCONNECT: clientNum = %d, devId = %d\n", \
	     dev->clientNum, devId);

      /* let client know */
      kmsg.msg.hdr.major = M2D_DISCONNECT; /* translate C2M to M2D */
      kmsg.clientNum = dev->clientNum; /* tell kernel which client */

      dbgMsg1("write(M2D_DISCONNECT)\n");

      if (write(fd, (char *)(&kmsg), sizeof(kmsg)) < 0) { /* tell driver */
         dbgAlert("write(M2D_DISCONNECT): %m\n");
      }

      /* let kernel know */
      ioctlData.devId = devId;
      ioctlData.arg = 0;
      ioctl(fd, ABIO_ASSGN_DEV_TO_CLIENT, (char *)&ioctlData);

      ioctlData.devId = devId;
      ioctlData.arg = 0;
      ioctl(fd, ABIO_SET_DEV_STATUS, &ioctlData);
      
      /* let abusd know */
      dev->state = AB_DEV_STATE_NOT;
      dev->timeout = 0;
      dev->clientNum = 0;
      dev->devStatus.byte = 0;
      if (dvr) dvr->linkState = AB_DVR_LINK_PENDING;
      
      dbgMsg3("abDev[%02X].devStatus=0x%02X\n", \
	      dev->devId.devNum, dev->devStatus.byte);
      
      return_v;
   }
   
   case C2M_AB_EMULATION: {
     dev = getDev(devId);
     if (dev == NULL) {
       dbgWarn("bad device id 0x%02X\n", (int)devId);
       return_v;
     }
     
     dbgCmd("C2M_AB_EMULATION: clientNum = %d, devId = %d\n", 
	    dev->clientNum, devId);

     return_v;
   }
   
   case C2M_MSG_TO_DRIVER: {
     int opcode;
     
     dev = getDev(devId);
     if (dev == NULL) {
       dbgWarn("bad device id 0x%02X\n", (int)devId);
       return_v;
     }
     
     dbgCmd("C2M_MSG_TO_DRIVER: clientNum = %d, devId = %d\n",
	    dev->clientNum, devId);
     
     opcode = kmsg.msg.data[4];
     
     switch (opcode) {		/* switch on A.b opcode */
     case AB_APPL_HDW_SIG:
     case AB_APPL_TST_RPL:
     case AB_APPL_STAT_MSG:
     case AB_APPL_TST:
     case AB_ATTN:
     case AB_IDENT_RPL:
     case AB_CAP_RPL:		/* send msg to driver */
     case AB_RSRC_REQ:		/* MSPs will require time support */
     case AB_PWR_USG_RPL:
     case AB_BW_USG_RPL:
     default:
       dbgAlert("unhandled opcode: %02X\n", opcode);
       return_v;
     }
     
     return_v;
   }
   
   case C2M_MSG_TO_MONITOR: {
     dev = getDev(devId);
     if (dev == NULL) {
       dbgWarn("bad device id 0x%02X\n", (int)devId);
       return_v;
     }
     
     dbgCmd("C2M_MSG_TO_MONITOR: clientNum = %d, devId = %d\n", 
	    dev->clientNum, devId);

     return_v;
   }
   
   case K2D_CLIENT_CONNECT: {
     
     dvr = &abDvr[clientNum];
     
     if (minor) {
       dbgCmd("K2D_CLIENT_CONNECT: clientNum = %d  - connect\n",
	      clientNum);
     }
     else {
       dbgCmd("K2D_CLIENT_CONNECT: "
	       "clientNum = %d  - disconnect\n", clientNum);
     }

     if (minor == 0) {		/* this is disconnect */
       ABKMSG msg2;
       
       for (ii=0; ii<MAX_DEVS; ii++) {
	 if (abDev[ii].clientNum == clientNum) {
	   
	   /* unlink drvr from dev.  disable dev. */
	   
	   /* tell dev */
	   msg2.msg.hdr.major = M2C_MSG_TO_DEVICE;
	   msg2.msg.hdr.minor = 0x01; /* protocol */
	   msg2.msg.hdr.devId = ii;
	   msg2.msg.data[4] = AB_EN_AP_REP;
	   msg2.msg.data[5] = 0; /* 0 = disable */
	   msg2.msg.hdr.msgLen = 2;
	   
	   dbgMsg1("write(M2C_MSG_TO_DEVICE)\n");

	   if (write(fd, (char *)&msg2, sizeof(msg2)) < 0) {
	     dbgAlert("write(M2C_MSG_TO_DEVICE): %m\n");
	   }

	   /* tell card */
	   msg2.msg.hdr.major = M2C_ENABLE;
	   msg2.msg.hdr.minor = 0;
	   msg2.msg.hdr.msgLen = 0;
	   msg2.msg.hdr.devId = ii;
	   
	   dbgMsg1("write(M2C_ENABLE)\n");

	   if (write(fd, (char *)&msg2, sizeof(msg2)) < 0) {
	     dbgAlert("write(M2C_ENABLE): %m\n");
	   }	   
	   /* tell abusd */
	   
	   dbgMsg3("abDev[%02X].devStatus=0x%02X\n", \
		   abDev[ii].devId.devNum, abDev[ii].devStatus.byte);
	   
	   abDev[ii].devStatus.byte &= ~AB_STATUS_ENABLE;
	   
	   dbgMsg3("abDev[%02X].devStatus=0x%02X\n", \
		   abDev[ii].devId.devNum, abDev[ii].devStatus.byte);
	   
	   abDev[ii].state = AB_DEV_STATE_RDY;
	   abDev[ii].clientNum = 0;
	 }
       }
       dvr->dvrType = AB_DVR_TYPE_NONE;
       dvr->linkState = AB_DVR_LINK_IDLE;
       return_v;
     }
     
     if (dvr->dvrType == AB_DVR_TYPE_NONE) {
       /* client is just connecting */
       memset((void*)dvr, 0, sizeof(*dvr));
       for (ii=0; ii<MAX_DEVS; ii++)
	 abDev[ii].refused[clientNum] = 0;
       dvr->dvrType = AB_DVR_TYPE_USER;
       abDvr[clientNum].linkState = AB_DVR_LINK_IDLE;
     }
     else {
       dbgAlert("Client %d is already connected ?!?\n", clientNum);
     }
     
     return_v;
   }
     
   default:
     dev = getDev(devId);
     if (dev == NULL) {
       dbgWarn("bad device id 0x%02X\n", (int)devId);
       return_v;
     }
     
     dbgWarn("packet w/o handler! : clientNum = %d, devId = %d\n", 
	     dev->clientNum, devId);
     return_v;
   }
   return_v;
}

int abusdPoll(int fd) {
   int devId;
   int clientNum;
   ABKMSG kmsg;
   int pollNext;

   /*
    * Check for bad_dev timeouts, reset dev.
    * Check for drivers that need to be offered devs.
    */

   pgmTrace();

   pollNext = 0;

   for (devId=0; devId<MAX_DEVS; devId++) {
      if (abDev[devId].state == AB_DEV_STATE_BAD) {
         pollNext = 1;
         if (abDev[devId].timeout-time(NULL)<0) {

            dbgMsg1("resetting dev #%02X\n", devId);

            kmsg.msg.hdr.major = M2C_MSG_TO_DEVICE;
            kmsg.msg.hdr.minor = 1;
            kmsg.msg.hdr.msgLen = 1;
            kmsg.msg.hdr.devId = devId;
            kmsg.msg.data[4] = AB_RESET;

	    dbgMsg1("write(M2C_MSG_TO_DEVICE)\n");

            if (write (fd, (char *)(&kmsg), sizeof(kmsg)) < 0) {
               dbgAlert("write(M2C_MSG_TO_DEVICE): %m\n");
	    }
         }
      }
   }

   /*
    * For each dvr that has a pending LINK_REQUEST,
    * find the first matching dev and offer it.
    * If there are no matches, indicate end of list.
    */

   for (clientNum = 0; clientNum<MAX_CLIENTS; clientNum++) {
    
      if ((abDvr[clientNum].linkState == AB_DVR_LINK_PENDING) ||
          (abDvr[clientNum].linkState == AB_DVR_LINK_REQ)) {

         for (devId = 1; devId<MAX_DEVS; devId++) {

            if ((abDev[devId].refused[clientNum] == 0) &&
                (devDvrMatch(&abDev[devId], &abDvr[clientNum])) &&
                (abDev[devId].state == AB_DEV_STATE_RDY)) {
               abDevType *dev = getDev(devId);
	    
               /* offer dev to dvr */

               abDvr[clientNum].linkState = AB_DVR_LINK_IDLE;

               if (abDvr[clientNum].opFlg == 0)
                  kmsg.msg.hdr.major = M2D_LINK_REPLY;
               else
                  kmsg.msg.hdr.major = M2D_LONG_ID;

               kmsg.msg.hdr.minor = 0; /* matching dev */
               kmsg.msg.hdr.msgLen = 0x38;
               kmsg.msg.hdr.devId = devId;
               kmsg.clientNum = clientNum;

               kmsg.msg.data[LONGID_PROT_REV_OFFSET] = dev->devId.protRev;
               *(long *)&kmsg.msg.data[LONGID_DEV_NUM_OFFSET] =
                  dev->devId.devNum;
	  
               memcpy(&kmsg.msg.data[LONGID_MOD_REV_OFFSET],
                      dev->devId.moduleRev,
                      LONGID_MOD_REV_LEN);
               memcpy(&kmsg.msg.data[LONGID_VENDOR_OFFSET],
                      dev->devId.vendorName,
                      LONGID_VENDOR_LEN);
               memcpy(&kmsg.msg.data[LONGID_MOD_NAME_OFFSET],
                      dev->devId.moduleName,
                      LONGID_MOD_NAME_LEN);
	  
               memcpy(&kmsg.msg.data[LONGID_PROT_OFFSET], 
                      dev->devProt.prot,
                      LONGID_PROT_LEN);
               memcpy(&kmsg.msg.data[LONGID_TYPE_OFFSET], 
                      dev->devProt.type,
                      LONGID_TYPE_LEN);
               memcpy(&kmsg.msg.data[LONGID_MODEL_OFFSET], 
                      dev->devProt.model,
                      LONGID_MODEL_LEN);
	  
	       dbgMsg3("abDev[%02X].devStatus=0x%02X\n", \
		       dev->devId.devNum, dev->devStatus.byte);

               kmsg.msg.data[LONGID_STATUS_OFFSET] = dev->devStatus.byte;
               
	       dbgMsg1("write(M2D_LONG_ID | M2D_LINK_REPLY)\n");

               if (write(fd, (char*)(&kmsg), sizeof(kmsg)) < 0) {
                  dbgAlert("write(LINK_REPLY): %s\n", strerror(errno));
	       }

               dbgMsg1("offering dev %d to client %d\n",
                       devId, clientNum);
	  
               break;
            }
         }

         if (abDvr[clientNum].linkState == AB_DVR_LINK_REQ) {
            /*
             * if we get here, then we didn't find a matching device.
             * Send a non-matching link reply; end of table
             */
	
            dbgMsg1("no matching dev for client %d\n", clientNum);

            kmsg.msg.hdr.major = M2D_LINK_REPLY;
            kmsg.msg.hdr.minor = 0xFF; /* no matching dev */
            kmsg.msg.hdr.msgLen = 0;
            kmsg.msg.hdr.devId = 0;
            kmsg.clientNum = clientNum;

	    dbgMsg1("write(M2D_LINK_REPLY)\n");

            if (write(fd, (char*)(&kmsg), sizeof(kmsg)) < 0) {
	      dbgAlert("write(M2D_LINK_REPLY): %s\n", strerror(errno));
	    }

            abDvr[clientNum].linkState = AB_DVR_LINK_IDLE;
            abDvr[clientNum].opFlg = 1;
         }
      }
   }
   return_d(pollNext);
}

#define   ABDEV_TIMEOUT_SECS    (1)
#define   ABDEV_TIMEOUT_USECS   (0)

int main (int argc, char *argv[])
{
   fd_set real_rset, rset;
   struct timeval zztime;
   int fd;
   int rc;
   int pollNext;
   int irq, port;
   int retval;
   int help=0;
   int no_fork=0;
   int argn;
   
   pgmTrace();

   pollNext = 1;

   irq=0;
   port=MC_DEFPORT;

   /*
    * parse command line args
    */

   for (argn=1; argn<argc; argn++) {
     if (!strcmp("-irq", argv[argn])) {
       argn++;
       if (argc>argn) {
	 irq = strtol(argv[argn], (char **)NULL, 0);
       }
       else {
	 help=1;
	 break;
       }
     }
     else if (!strcmp("-port", argv[argn])) {
       argn++;
       if (argc>argn) {
	 port = strtol(argv[argn], (char **)NULL, 0);
       }
       else {
	 help=1;
	 break;
       }
     }
     else if (!strcmp("-no-fork", argv[argn])) {
       no_fork=1;
     }
     else {
       help=1;
       break;
     }
   }

   if (!irq) {
     help=1;
   }

   if (help) {
     fprintf(stderr, "\n");
     fprintf(stderr, "Usage:\n%s -irq irq_number [-port port_number] "
	     "[-no-fork]\n", argv[0]);
     fprintf(stderr, "\n");
     fprintf(stderr, "   -irq irq_number\n");
     fprintf(stderr, "      Tell driver to use irq irq_number.  This\n");
     fprintf(stderr, "      value can not be probed and must be given.\n");
     fprintf(stderr, "      Legal values are 10, 11 or 12.\n");
     fprintf(stderr, "\n");
     fprintf(stderr, "   -port port_number\n");
     fprintf(stderr, "      Tell driver to use IO port port_number.  If\n");
     fprintf(stderr, "      this value is not given, it will be probed.\n");
     fprintf(stderr, "      Legal values are 0x250, 0x260, 0x350.\n");
     fprintf(stderr, "\n");
     fprintf(stderr, "   -no-fork\n");
     fprintf(stderr, "      Normally abusd will fork and the parent\n");
     fprintf(stderr, "      process will exit leaving the daemon in\n");
     fprintf(stderr, "      the background.  This option prevents this\n");
     fprintf(stderr, "      behavior.\n");
     fprintf(stderr, "\n");
     exit(1);
   }

   /*
    * become a daemon
    */

   if (!no_fork) {
     retval = fork();
     
     if (retval < 0) {
       fprintf(stderr, "%s: fork() failed - %s\n", argv[0], strerror(errno));
       exit(1);
     }
     
     if (retval != 0) exit(0);
     
     fclose(stdin);
     fclose(stdout);
     fclose(stderr);
   }

   /*
    * attempt to connect to controller
    */

   openlog(argv[0], LOG_CONS | LOG_NDELAY, LOG_DAEMON);

   fd=abusdInit("/dev/abus", irq, port);

   if (fd<0) {
     dbgAlert("%s: Unable to initialize %s using "
	     "port 0x%03X, IRQ %d\n  - %s\n", 
	     argv[0], "/dev/abus", port, irq, strerror(errno));
     exit(1);
   }

   /*
    * Announce to the world...
    */

   dbgInform("abusd 0.9.0 3/20/96 for CATC A.bus card - "
		   "using port 0x%03X, IRQ %d\n", port, irq);

   /*
    * setup fd read set 
    */

   FD_ZERO(&real_rset);
   FD_SET(fd, &real_rset);	/* do this for every fd */

   /*
    * start main loop
    */

   while (1) {
      /* setup ab device timeout */
      zztime.tv_sec = ABDEV_TIMEOUT_SECS;
      zztime.tv_usec = ABDEV_TIMEOUT_USECS;
   
      rset = real_rset;		/* must be reset */
      
      if (pollNext) rc = select(6, &rset, NULL, NULL, &zztime);
      else rc = select(6, &rset, NULL, NULL, NULL);

      if (rc < 0) {
	dbgAlert("select() failed - %m");
      }
      else if (rc > 0) {
         /* fds are selected */
         if (FD_ISSET(fd, &rset)) { /* do this for every fd */
            abusdRead(fd);
	 }
      }
    
      pollNext = abusdPoll(fd); /* do this for every fd */
   }
   return_d(0);
}
