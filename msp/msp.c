
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
 *  libmsp.c  - msp support library.  Uses libabdriver
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#include <netinet/in.h>  /* for byteorder */

#include <acb/global.h>
#include <acb/rwiab.h>
#include <acb/abus.h>
#include <msp.h>
#include <mspMessages.h>


/*****************************************************************
 *
 *  Debuging utilites
 *
 *****************************************************************/

#undef DEBUG

#define DEBUG_ALERT   /* Problem with driver or manager */
#define DEBUG_WARN    /* Problem with client */

#undef DEBUG_PGM_TRACE
#undef DEBUG_CMD     /* Commands sent to manager */

#undef DEBUG_MSG_1
#undef DEBUG_MSG_2
#undef DEBUG_MSG_3   /* trace device status byte */

#define DBG_PRE_FMT "%s:%10s:%5d:%14s(): "
#define DBG_PRE_ARGS dateStr(), __FILE__, __LINE__, __FUNCTION__

#define _pgmTrace() \
  { \
    fprintf(stderr, DBG_PRE_FMT "\n", DBG_PRE_ARGS); \
  }

#define _return_d(x) \
  { \
    unsigned int retval; \
    retval = (x); \
    fprintf(stderr, DBG_PRE_FMT "return(0x%08X, %d);\n", \
	   DBG_PRE_ARGS, (unsigned)retval, (int)retval); \
    return(retval); \
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
    unsigned int retval; \
    retval = (x); \
    fprintf(stderr, DBG_PRE_FMT "return(0x%08X, %d);\n", \
	   DBG_PRE_ARGS, (unsigned)retval, (int)retval); \
    return(retval); \
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

typedef struct {
  unsigned char op;
  int (*func)(ABMSG *msg);
} mspLibFuncList;

static int fd;
static mspOpsType mspOps;
static int msgToDriverCB (ABMSG *msg);


/**********************************
 *  Internal utility functions
 **********************************/

int
mspNum2devId (long mspNum) {
  int ii;
  int devId;

  pgmTrace();

  devId = -1;
  for (ii=1; ii<ABD_MAX_DEVS; ii++) {
    if ((abdDev[ii].devId.devNum == mspNum) &&
	(abdDev[ii].state != ABD_DEV_STATE_NOT)) {
      devId = ii;
      break;
    }
  }
  return_d(devId);
}


/*********************************
 *  Manager to Driver commands
 *  These call the callbacks
 *********************************/

static int
M2D_verRpl (ABMSG *msg) {
  RWI_VrRplBody *body = (void*)&msg->data[4];
  long mspNum;
  
  pgmTrace();

  mspNum = devId2mspNum(msg->hdr.devId);
   
  if (msg->hdr.msgLen > sizeof(body->rwihdr)) {
    /* Verify NULL termination */
    body->ver[msg->hdr.msgLen - sizeof(body->rwihdr)] = 0;

    if (mspOps.verRep) {
      return_d(mspOps.verRep(mspNum, body->ver));
    }

    fprintf(stderr, "No handler for mspOps.verRep\n");
    return_d(0);
  }
  else {
    fprintf(stderr, "{%08X} bad ver\n", (unsigned)mspNum);
    return_d(-1);
  }
}

static int
M2D_dbgStr (ABMSG * msg) {
  RWI_DbgStrBody *body = (void*)&msg->data[4];
  long mspNum;
  
  pgmTrace();

  mspNum = devId2mspNum(msg->hdr.devId);
   
  fprintf(stderr, "{%08X} ", (unsigned)mspNum);
   
  if (msg->hdr.msgLen > sizeof(body->rwihdr)) {
    /* Verify NULL termination */
    body->str[msg->hdr.msgLen - sizeof(body->rwihdr)] = 0;
    fprintf(stderr, "%s", body->str);

    if (mspOps.dbgRepStr) {
      return_d(mspOps.dbgRepStr(mspNum, body->str));
    }

    return_d(0);
  }
  else {
    fprintf(stderr, "bad dbgStr\n");
    return_d(-1);
  }
}

static int
M2D_dbgBin (ABMSG * msg) {
  RWI_DbgBinBody *body = (void*)&msg->data[4];
  unsigned char *dat;
  int len;
  long mspNum;
  
  pgmTrace();

  mspNum = devId2mspNum(msg->hdr.devId);

  dat = body->data;
  len = msg->hdr.msgLen - sizeof(body->rwihdr);
  fprintf(stderr, "{%08X} ", (unsigned)mspNum);
  while (len--) {
    fprintf(stderr, "[%02X]", (unsigned int)*dat++);
  }
  fprintf(stderr, "\n");

  dat = body->data;
  len = msg->hdr.msgLen - sizeof(body->rwihdr);
  if (mspOps.dbgRepBin) {
    return_d(mspOps.dbgRepBin(mspNum, len, dat));
  }
  return_d(0);
}

static int
M2D_bmpRep (ABMSG *msg) {
  MSP_BmpRplBody *body = (void*)&msg->data[4];
  unsigned long val;
  long mspNum;
  
  pgmTrace();

  mspNum = devId2mspNum(msg->hdr.devId);
  val = body->BmpVals[0];
  
  if (mspOps.bmpRep) {
    return_d(mspOps.bmpRep(mspNum, val));
  }

  fprintf(stderr, "No handler for mspOps.bmpRep\n");
  return_d(0);
}

static int
M2D_irRep (ABMSG *msg) {
  MSP_IrRplBody *body = (void*)&msg->data[4];
  unsigned long irVal[MSP_IR_NUM+1];
  int ii;
  long mspNum;
  
  pgmTrace();

  mspNum = devId2mspNum(msg->hdr.devId);

  for (ii=0; ii<MSP_IR_NUM; ii++) {
    irVal[ii] = body->IrVals[ii];
  }

  irVal[ii] = 0;

  if (mspOps.irRep) {
    return_d(mspOps.irRep(mspNum, irVal));
  }

  fprintf(stderr, "No handler for mspOps.irRep\n");
  return_d(0);
}

static int
M2D_irParmsRpl (ABMSG *msg) {
  mspIrParmsType parms;
  MSP_IrParmsRplBody *body = (void*)&msg->data[4];
  long mspNum;

  pgmTrace();

  mspNum = devId2mspNum(msg->hdr.devId);
  parms.interval = ntohs(body->irInterval);

  if (mspOps.irParms) {
    return_d(mspOps.irParms(mspNum, &parms));
  }

  fprintf(stderr, "No handler for mspOps.irParms\n");
  return_d(0);
}

static int
M2D_sonRep (ABMSG *msg) {
  MSP_SonRplBody *body = (void*)&msg->data[4];
  unsigned int count;
  unsigned int echos;
  unsigned short *msgdat;
  unsigned int ii;
  long mspNum;
  unsigned int set, echo;
  const unsigned long *table[10];
  unsigned long table0[10][10];
  
  pgmTrace();

  for (set = 0; set<10; set++) table[set] = table0[set];
  
  mspNum = devId2mspNum(msg->hdr.devId);
  count = msg->hdr.msgLen-sizeof(body->rwihdr)-sizeof(body->echoNum);
  echos = ntohs(body->echoNum);
  msgdat = body->data;
   
  count /= (echos+1)*sizeof(short);
   
  set = 0;
  while (count--) {
    /* sonar number */
    table0[set][0] = ntohs(*msgdat++);
    echo = 1;
    for (ii=0; ii<echos; ii++) {
      /* echo return */
      table0[set][echo] = ntohs(*msgdat++);
      echo++;
    }
    table0[set][echo] = 0;
    set++;
  }

  table[set] = NULL;

  if (mspOps.sonRep) {
    return_d(mspOps.sonRep(mspNum, table));
  }

  fprintf(stderr, "No handler for mspOps.sonRep\n");
  return_d(0);
}

static int
M2D_sonParmsRpl (ABMSG *msg) {
  MSP_SonParmsRplBody *body = (void*)&msg->data[4];
  long mspNum;
  mspSonParmsType parms;

  pgmTrace();

  memset(&parms, 0xFF, sizeof(parms));

  mspNum = devId2mspNum(msg->hdr.devId);
  
  parms.fireInterval =    ntohs(body->fireInterval);
  parms.echoCount        = ntohs(body->echoCount);
  parms.echoTimeout      = ntohs(body->echoTimeout);
  parms.initialBlankTime = ntohs(body->initBlank);
  parms.echoBlankTime    = ntohs(body->echoBlank);
  parms.fireDelay        = ntohs(body->fireDelay);
  parms.startDelay       = ntohs(body->startDelay);
  
  if (mspOps.sonParms) {
    return_d(mspOps.sonParms(mspNum, &parms));
  }

  fprintf(stderr, "No handler for mspOps.sonParms\n");
  return_d(0);
}

static int
M2D_sonTableRpl (ABMSG *msg) {
  MSP_SonTableRplBody *body = (void*)&msg->data[4];
  unsigned int sonar;
  unsigned int count, num;
  long mspNum;
  unsigned int set, xducer;
  const unsigned long *table[10];
  unsigned long table0[10][10];
  
  pgmTrace();

  for (set = 0; set<10; set++) table[set] = table0[set];

  mspNum = devId2mspNum(msg->hdr.devId);

  num = (msg->hdr.msgLen-sizeof(body->rwihdr))/sizeof(short);
  set = 0;
  xducer = 0;

  for (count=0; count<num; count++) {
    sonar = ntohs(body->table[count]);
    table0[set][xducer++] = sonar;
    if (sonar == 0) {
      set++;
      xducer = 0;
    }
  }
   
  table0[set][xducer] = 0;
  if (xducer) {
    set++;
  }
  table[set] = NULL;	/* NULL to end table */

  if (mspOps.sonTable) {
    return_d(mspOps.sonTable(mspNum, table));
  }

  fprintf(stderr, "No handler for mspOps.sonTable\n");
  return_d(0);
}

/* M2D functions table */

static const mspLibFuncList M2D_MSP_lst[] = {
  {MSP_BMP_RPL,        M2D_bmpRep      },
  {MSP_IR_RPL,         M2D_irRep       },
  {MSP_IR_PARMS_RPL,   M2D_irParmsRpl  },
  {MSP_SON_RPL,        M2D_sonRep      },
  {MSP_SON_PARMS_RPL,  M2D_sonParmsRpl },
  {MSP_SON_TABLE_RPL,  M2D_sonTableRpl },
  
  {0, NULL}
};

static const mspLibFuncList M2D_RWI_lst[] = {
  {AB_RWI_VR_RPL,      M2D_verRpl    },
  {AB_RWI_DBG_STR,     M2D_dbgStr   },
  {AB_RWI_DBG_BIN,     M2D_dbgBin   },
  
  {0, NULL}
};

static const mspLibFuncList M2D_AB_lst[] = {
  
  {0, NULL}
};


static int
msgToDriverCB (ABMSG *msg) {
  int major;
  int minor;
  int msgLen;
  int devId;
  abdDevType *dev;
  int ii;
  int approve;

  pgmTrace();

  major  = msg->hdr.major;
  minor  = msg->hdr.minor;
  msgLen = msg->hdr.msgLen;
  devId  = msg->hdr.devId;

  dev = &abdDev[devId];

  switch (major) {

  case M2D_LINK_REPLY:
  case M2D_LONG_ID: {
    
    if (minor==0xFF) {		/* No devs available */
#ifdef DEBUG
      fprintf(stderr, "M2D_LINK_REPLY or M2D_LONG_ID: "
	      "No free devs available at this time.\n");
#endif

      D2M_linkApprove(0, 1);	/* approve for new devs */
      return_d(0);
    }

    approve = 1;
    if (mspOps.validMspId) {
      approve = 0;
      for (ii=0; mspOps.validMspId[ii]; ii++) {
	if (mspOps.validMspId[ii]==devId2mspNum(devId)) approve = 1;
      }
    }

    if (abdDev[devId].state != ABD_DEV_STATE_RDY)
      D2M_linkApprove (devId, approve);	/* reply with a link approval */

    /* display vital stats */
    
#ifdef DEBUG
    fprintf(stderr, "libmsp: M2D_LINK_REPLY or M2D_LONG_ID - "
	    "mspNum %08X devId %02X status %02X\n",
	    (unsigned)devId2mspNum(devId), devId, dev->devStatus.byte);
#endif

    dbgMsg3("abDev[%d].devStatus=0x%02X  MSP %02X\n", \
	    devId, dev->devStatus.byte, (unsigned)devId2mspNum(devId));
    
    return_d(0);
  }

  case M2D_ID: {
    fprintf(stderr,"libmsp: mspM2D - unhandled major M2D_ID\n");
    return_d(-1);
  }

  case M2D_TYPE: {
    fprintf(stderr,"libmsp: mspM2D - unhandled major M2D_TYPE\n");
    return_d(-1);
  }

  case M2D_STATUS: {
    fprintf(stderr,"libmsp: mspM2D - unhandled major M2D_STATUS\n");
    return_d(-1);
  }

  case M2D_MSG_TO_DRIVER: {
    RWI_Body *body = (void*)&msg->data[4];
    const mspLibFuncList *lst;
    int cmd_num, op;
     
    pgmTrace();

    if (minor == 0) {		/* protocol bit set */
      return_d(-1);		/* not handled */
    }
    else {

      pgmTrace();

      switch (body->rwihdr.major) {
      case AB_MSP_OPCODE:
	lst = M2D_MSP_lst;
	op = body->rwihdr.minor;
	break;
      case AB_RWI_OPCODE:
	lst = M2D_RWI_lst;
	op = body->rwihdr.minor;
	break;
      default:
	lst = M2D_AB_lst;
	op = body->rwihdr.major;
	break;
      }
    }
           
    for (cmd_num=0; lst[cmd_num].op && lst[cmd_num].op != op;
	 cmd_num++) ;
     
    if (!lst[cmd_num].func) {
      fprintf(stderr, "msplib: M2D_MSG_TO_DRIVER - "
	      "unhandled A.b major %02X\n", op);
      return_d(-1);
    }
    
    pgmTrace();
    return_d((*(lst[cmd_num].func))(msg));
  }
    
  case M2D_DISCONNECT: {

    fprintf(stderr, "libmsp: M2D_DISCONNECT - "
	    "mspNum %08X\n", (unsigned)devId2mspNum(devId));

    if (mspOps.mspConnect) {
      return_d(mspOps.mspConnect(devId2mspNum(devId), 0));
    }
    else {
      return_d(0);
    }
  }
    
  case M2D_LINK_APPROVE_ACK: {
    
    
    if (minor == 0) {
      D2M_enable(devId, 1);	/* enable dev */
#ifdef DEBUG
      fprintf(stderr, "libmsp: M2D_LINK_APPROVE_ACK - "
	      "mspNum %08X is ready\n",
	      (unsigned)devId2mspNum(devId));
#endif

      /* Get back into request queue */
      D2M_linkRequest("RWI-DEV", "MSP", "1");

      /* notify client */
      if (mspOps.mspConnect) {
	return_d(mspOps.mspConnect(devId2mspNum(devId), 1));
      }
      else {
	return_d(0);
      }
    }
    else {
      fprintf(stderr, "libmsp: M2D_LINK_APPROVE_ACK - "
	      "mspNum %08X is not available\n",
	      (unsigned)devId2mspNum(devId));
      return_d(0);
    }
  }

  default:
    fprintf(stderr, "libmsp: unknown major opcode %02X\n", major);
    return_d(-1);
  }
}

/**************************************
 *  Begin public interface
 **************************************/

int
mspLibInit (const char *devFile, const mspOpsType *ops) {
  int retval;
  
  pgmTrace();

  memset(&mspOps, 0, sizeof(mspOps));
  if (ops) memcpy(&mspOps, ops, sizeof(mspOps));

  fd = open(devFile, O_RDWR);
  if (fd<0) {
    return_d(-1);
  }

  abDriverLibInit(fd, &msgToDriverCB);
  retval = D2M_linkRequest("RWI-DEV", "MSP", "1");
  if (retval<0) {
    return_d(retval);
  }

  return_d(fd);
}

int
mspLibSelect (void) {
  pgmTrace();

  return_d(abDriverLibSelect(fd));
}


/****************************************
 * functions to send messages to an MSP
 ****************************************/

int
mspReqReset (long mspNum) {
  ABMSG  msg;
  int devId;

  pgmTrace();

  devId = mspNum2devId(mspNum);
  if (devId<1) {
    return_d(-1);
  }

  msg.hdr.major = D2M_MSG_TO_DEVICE;
  msg.hdr.minor = 1;
  msg.hdr.devId = devId;
  msg.data[4] = AB_RESET;
  msg.hdr.msgLen = 1;

  D2M_msgToDevice(&msg);
  return_d(0);
}

int
mspReqVer (long mspNum) {
  ABMSG        msg;
  int devId;
  RWI_VrReqBody *body = (void*)&msg.data[4];

  pgmTrace();

  devId = mspNum2devId(mspNum);
  if (devId<1) {
    return_d(-1);
  }

  msg.hdr.major = D2M_MSG_TO_DEVICE;
  msg.hdr.minor = 1;
  msg.hdr.devId = devId;
  msg.hdr.msgLen = 2;

  body->rwihdr.major = AB_RWI_OPCODE;
  body->rwihdr.minor = AB_RWI_VR_REQ;
   
  D2M_msgToDevice(&msg);
  return_d(0);
}

int
mspReqIrParms (long mspNum) {
  ABMSG        msg;
  int devId;
  MSP_IrParmsBody *body = (void*)&msg.data[4];

  pgmTrace();

  devId = mspNum2devId(mspNum);
  if (devId<1) {
    return_d(-1);
  }

  msg.hdr.major = D2M_MSG_TO_DEVICE;
  msg.hdr.minor = 1;
  msg.hdr.devId = devId;
  msg.hdr.msgLen = 2;

  body->rwihdr.major = AB_MSP_OPCODE;
  body->rwihdr.minor = MSP_IR_PARMS;
   
  D2M_msgToDevice(&msg);
  return_d(0);
}

int
mspSetIrParms (long mspNum, const mspIrParmsType *parms) {
  ABMSG        msg;
  int devId;
  MSP_IrParmsBody *body = (void*)&msg.data[4];

  pgmTrace();

  dbgMsg1("iri %d\n", parms->interval);

  devId = mspNum2devId(mspNum);
  if (devId<1) {
    return_d(-1);
  }

  msg.hdr.major = D2M_MSG_TO_DEVICE;
  msg.hdr.minor = 1;
  msg.hdr.devId = devId;
  msg.hdr.msgLen = 6;

  body->rwihdr.major = AB_MSP_OPCODE;
  body->rwihdr.minor = MSP_IR_PARMS;
  body->parms[0] = htons(MSP_IR_PARM_INTERVAL);
  body->parms[1] = htons((unsigned short)parms->interval);
   
  D2M_msgToDevice(&msg);
  return_d(0);
}

int
mspReqSonTable (long mspNum) {
  ABMSG msg;
  MSP_SonTableBody *body = (void*)&msg.data[4];
  int devId;

  pgmTrace();

  devId = mspNum2devId(mspNum);
  if (devId<1) {
    return_d(-1);
  }

  msg.hdr.minor = 0x01;
  msg.hdr.major = D2M_MSG_TO_DEVICE;
  msg.hdr.devId = devId;
  
  body->rwihdr.major = AB_MSP_OPCODE;
  body->rwihdr.minor = MSP_SON_TABLE;
  
  msg.hdr.msgLen= sizeof(body->rwihdr);
  D2M_msgToDevice(&msg);
  return_d(0);
}

int
mspSetSonTable (long mspNum, const unsigned long *table[]) {
  ABMSG msg;
  MSP_SonTableBody *body = (void*)&msg.data[4];
  int count, set, xducer;
  int devId;

  pgmTrace();

  devId = mspNum2devId(mspNum);
  if (devId<1) {
    return_d(-1);
  }
  
  count = 0;
  set = 0;
  while (table[set]) {
    xducer = 0;
    while (table[set][xducer]) {
      body->table[count++] = htons(table[set][xducer++]);
    }
    body->table[count++] = 0;
    set++;
  }

  msg.hdr.minor = 0x01;
  msg.hdr.major = D2M_MSG_TO_DEVICE;
  msg.hdr.devId = devId;
  
  body->rwihdr.major = AB_MSP_OPCODE;
  body->rwihdr.minor = MSP_SON_TABLE;
  
  msg.hdr.msgLen= sizeof(body->rwihdr) + (count-1)*sizeof(short);

  D2M_msgToDevice(&msg);
  return_d(0);
}

int
mspReqSon (long mspNum, const unsigned long *table) {
  ABMSG msg;
  MSP_SonReqBody *body = (void*)&msg.data[4];
  int count;
  int devId;

  pgmTrace();

  devId = mspNum2devId(mspNum);
  if (devId<1) {
    return_d(-1);
  }
  
  msg.hdr.minor = 0x01;
  msg.hdr.major = D2M_MSG_TO_DEVICE;
  msg.hdr.devId = devId;
  
  body->rwihdr.major = AB_MSP_OPCODE;
  body->rwihdr.minor = MSP_SON_REQ;
  
  count = 0;
  while ((body->sonAddr[count] = htons(table[count]))) count++;

  msg.hdr.msgLen = sizeof(body->rwihdr) + (count)*sizeof(short);
  D2M_msgToDevice(&msg);
  return_d(0);
}

int
mspReqSonStart (long mspNum) {
  int devId;
  ABMSG msg;
  MSP_SonStartBody *body = (void*)&msg.data[4];
  
  pgmTrace();

  devId = mspNum2devId(mspNum);
  if (devId<1) {
    return_d(-1);
  }

  msg.hdr.minor = 0x01;
  msg.hdr.major = D2M_MSG_TO_DEVICE;
  msg.hdr.devId = devId;
  
  body->rwihdr.major = AB_MSP_OPCODE;
  body->rwihdr.minor = MSP_SON_START;
  
  body->op = MSP_SON_START_START;
  
  msg.hdr.msgLen= sizeof(*body);
  D2M_msgToDevice(&msg);
  return_d(0);
}

int
mspReqSonStop (long mspNum) {
  int devId;
  ABMSG msg;
  MSP_SonStartBody *body = (void*)&msg.data[4];
  
  pgmTrace();

  devId = mspNum2devId(mspNum);
  if (devId<1) {
    return_d(-1);
  }

  msg.hdr.minor = 0x01;
  msg.hdr.major = D2M_MSG_TO_DEVICE;
  msg.hdr.devId = devId;
  
  body->rwihdr.major = AB_MSP_OPCODE;
  body->rwihdr.minor = MSP_SON_START;
  
  body->op = MSP_SON_START_STOP;
  
  msg.hdr.msgLen= sizeof(*body);
  D2M_msgToDevice(&msg);
  return_d(0);
}

int
mspReqSonParms (long mspNum) {
  int devId;
  ABMSG msg;
  MSP_SonParmsBody *body = (void*)&msg.data[4];
  
  pgmTrace();

  devId = mspNum2devId(mspNum);
  if (devId<1) {
    return_d(-1);
  }

  msg.hdr.minor = 0x01;
  msg.hdr.major = D2M_MSG_TO_DEVICE;
  msg.hdr.devId = devId;
  
  body->rwihdr.major = AB_MSP_OPCODE;
  body->rwihdr.minor = MSP_SON_PARMS;
  
  msg.hdr.msgLen = sizeof(body->rwihdr);
  D2M_msgToDevice(&msg);
  return_d(0);
}

int
mspSetSonParms (long mspNum, const mspSonParmsType *parms) {
  ABMSG msg;
  MSP_SonParmsBody *body = (void*)&msg.data[4];
  int count;
  int devId;

  pgmTrace();

  devId = mspNum2devId(mspNum);
  if (devId<1) {
    return_d(-1);
  }

  msg.hdr.minor = 0x01;
  msg.hdr.major = D2M_MSG_TO_DEVICE;
  msg.hdr.devId = devId;
  
  body->rwihdr.major = AB_MSP_OPCODE;
  body->rwihdr.minor = MSP_SON_PARMS;
  
  count = 0;

  if (parms->echoCount >= 0) {
    body->parms[count++] = htons(MSP_SON_PARM_ECHO_NUMBER);
    body->parms[count++] = htons(parms->echoCount);
  }

  if (parms->echoTimeout >= 0) {
    body->parms[count++] = htons(MSP_SON_PARM_ECHO_TIMEOUT);
    body->parms[count++] = htons(parms->echoTimeout);
  }

  if (parms->fireDelay >= 0) {
    body->parms[count++] = htons(MSP_SON_PARM_FIRE_DELAY);
    body->parms[count++] = htons(parms->fireDelay);
  }

  if (parms->fireInterval >= 0) {
    body->parms[count++] = htons(MSP_SON_PARM_FIRE_INTERVAL);
    body->parms[count++] = htons(parms->fireInterval);
  }

  if (parms->echoBlankTime >= 0) {
    body->parms[count++] = htons(MSP_SON_PARM_ECHO_BLNK_TIME);
    body->parms[count++] = htons(parms->echoBlankTime);
  }

  if (parms->initialBlankTime >= 0) {
    body->parms[count++] = htons(MSP_SON_PARM_INIT_BLNK_TIME);
    body->parms[count++] = htons(parms->initialBlankTime);
  }

  if (parms->startDelay >= 0) {
    body->parms[count++] = htons(MSP_SON_PARM_START_DELAY);
    body->parms[count++] = htons(parms->startDelay);
  }

  msg.hdr.msgLen= sizeof(body->rwihdr) + count*sizeof(short);

  D2M_msgToDevice(&msg);
  return_d(0);
}
