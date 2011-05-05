
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
 * abusk.c  ACCESS.bus manager/kernel module for CATC ab125i
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

/*****************************************************************
 * 	                     abus.c                              *
 ****************************************************************/

#define MODULE
#define __KERNEL__

#include <linux/config.h>

#ifdef CONFIG_MODVERSIONS
#undef  MODVERSIONS
#define MODVERSIONS
#endif /* CONFIG_MODVERISONS */

#include <linux/version.h>
#include <linux/module.h>

#if (LINUX_VERSION_CODE > 66304)
#define NEW_MODULES
#define IRQ_FILLER , NULL
#else
#define IRQ_FILLER
#endif

#ifdef MODVERSIONS
#ifdef NEW_MODULES
#include <linux/modversions.h>
#else /* !NEW_MODULES */
char kernel_version[] = UTS_RELEASE;
#endif
#endif /* MODVERSIONS */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/malloc.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <asm/io.h>

extern void enable_irq(int irq);
extern void disable_irq(int irq);

#include "acb/global.h"
#include "acb/catc.h"

static unsigned int port;	/* base port address */
static unsigned int irq;	/* hardware interrupt number */

#define AB_MSG_NUM 32		/* AB_MSG_NUM*sizeof(ABKMSG) */
#define AB_CLIENT_NUM 0x20
#define AB_DEV_NUM 256
#define AB_MON_NUM 4;

typedef struct {
   int pid;			/* to pair read/write with buffer */
   struct file *file;		/* to pair read/write with buffer */
   ABKMSG *buffer;		/* points to read() buffer */
   unsigned int bufferIn;	/* next empty spot */
   unsigned int bufferOut;	/* last spot read */
   unsigned char select_wait;	/* is client sleeping in select?  */
   struct wait_queue *queue;	/* queue entry used for select */
} abClientType;

typedef struct {
  int clientNum;
  unsigned char status;
} abDevType;

static abClientType abClient[AB_CLIENT_NUM];
static abDevType abDev[AB_DEV_NUM];

static unsigned char abMgrNum;
static unsigned char abMonNum;

static ABKMSG writeMsg;

static int abErrno;

static void MsgEnableIrq(void);
static void MsgDisableIrq(void);
static void MsgBusy(void);
static void MsgIdle(void);
static int  MsgWaitForInit(void);	
static int  MsgInit(void);
static int  MsgSend(ABMSG *msg);
static int  MsgRecv(ABMSG *msg);

static void abInterrupt();

static int abInit();

/*****************************************************************
 *
 *  Debuging utilites
 *
 *****************************************************************/

#undef DEBUG_PGM_TRACE
#undef DEBUG_PORT_IO

#define DEBUG_ALERT   /* Problem with hardware or driver */
#define DEBUG_WARN    /* Problem with manager or other client */

#undef  DEBUG_CMD     /* Commands sent to driver, ioctl and opcodes */
#undef  DEBUG_SYSCALL /* read, open, release, etc */

#undef  DEBUG_MSG_TRACE

#undef  DEBUG_MSG_1
#undef  DEBUG_MSG_2
#undef  DEBUG_MSG_3   /* trace device status byte */

#define DBG_PRE_FMT "%10s:%5d:%14s():"
#define DBG_PRE_ARGS __FILE__, __LINE__, __FUNCTION__

#define _pgmTrace() \
  { \
    printk(DBG_PRE_FMT "\n", DBG_PRE_ARGS); \
  }

#define _return_d(x) \
  { \
    printk(DBG_PRE_FMT "return(0x%08X, %d);\n", \
	   DBG_PRE_ARGS, (unsigned)x, (int)x); \
    return(x); \
  }

#define _return_v \
  { \
    printk(DBG_PRE_FMT "return(void);\n", \
           DBG_PRE_ARGS); \
    return; \
  }

#ifdef DEBUG_PGM_TRACE

#define pgmTrace() \
  { \
    printk(DBG_PRE_FMT "\n", DBG_PRE_ARGS); \
  }

#define return_d(x) \
  { \
    printk(DBG_PRE_FMT "return(0x%08X, %d);\n", \
	   DBG_PRE_ARGS, (unsigned)x, (int)x); \
    return(x); \
  }

#define return_v \
  { \
    printk(DBG_PRE_FMT "return(void);\n", \
           DBG_PRE_ARGS); \
    return; \
  }

#else /* DEBUG_PGM_TRACE */

#define pgmTrace() {}
#define return_d(x) return(x)
#define return_v return

#endif /* DEBUG_PGM_TRACE */



#ifdef DEBUG_PORT_IO

static inline void outb_d(unsigned int portValue, unsigned int portAddress) {
  printk("ab: IO_TRACE 0x%02X->0x%03X\n", portValue, portAddress);
  outb(portValue, portAddress);
  return;
}

static inline unsigned char inb_d(unsigned int port) {
  unsigned char val;
 
  val = inb(port);
  printk("ab: IO_TRACE 0x%03X=0x%02X\n", port, val);
  return(val);
}

#else /* !DEBUG_PORT_IO */

#define outb_d(val, port) outb(val, port)
#define inb_d(port) inb(port)

#endif /* DEBUG_PORT_IO */


#ifdef DEBUG_MSG_TRACE
#define msgTrace(format, args...) { \
  int ii; \
\
  printk(DBG_PRE_FMT "msgTrace:" format, \
	    DBG_PRE_ARGS , ## args); \
\
  for (ii=0; ii<3; ii++) { \
    printk("    msgTrace:"  \
	   " current pointers - client %d, in=%03d, out=%03d\n", \
	   ii, abClient[ii].bufferIn, abClient[ii].bufferOut); \
  } \
}

#else /* DEBUG_MSG_TRACE */
#define msgTrace(A...) {}
#endif /* DEBUG_MSG_TRACE */

#ifdef DEBUG_ALERT
#define dbgAlert(format, args...) printk(DBG_PRE_FMT "ALERT: " format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgAlert(A...) {}
#endif /* DEBUG_ALERT */

#ifdef DEBUG_WARN
#define dbgWarn(format, args...) printk(DBG_PRE_FMT "Warn: " format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgWarn(A...) {}
#endif /* DEBUG_WARN */

#ifdef DEBUG_CMD
#define dbgCmd(format, args...) printk(DBG_PRE_FMT "Cmd: " format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgCmd(A...) {}
#endif /* DEBUG_CMD */

#ifdef DEBUG_SYSCALL
#define dbgSyscall(format, args...) printk(DBG_PRE_FMT "dbgSyscall: " format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgSyscall(A...) {}
#endif /* DEBUG_SYSCALL */

#ifdef DEBUG_MSG_1
#define dbgMsg1(format, args...) printk(DBG_PRE_FMT "dbgMsg1: " format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgMsg1(A...) {}
#endif /* DEBUG_MSG_1 */

#ifdef DEBUG_MSG_2
#define dbgMsg2(format, args...) printk(DBG_PRE_FMT "dbgMsg2: " format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgMsg2(A...) {}
#endif /* DEBUG_MSG_2 */

#ifdef DEBUG_MSG_3
#define dbgMsg3(format, args...) printk(DBG_PRE_FMT "dbgMsg3: " format, \
					DBG_PRE_ARGS , ## args)
#else
#define dbgMsg3(A...) {}
#endif /* DEBUG_MSG_1 */

/*****************************************************************
 *
 * End of debug utilities
 *
 *****************************************************************/

/******************************************************************
* NAME        : MsgBusy
* DESCRIPTION : Send busy signal to MC
*/
static void
MsgBusy(void) {
   OUTSTAT((MSG_BUSY << 4) | ACK_ERR); /* set BUSY signal */
  return;
}

/******************************************************************
* NAME        : MsgIdle
* DESCRIPTION : Send idle signal to MC
*/
static void
MsgIdle(void) {
   OUTSTAT(ACK_IDLE); /* set IDLE sugnal */
  return;
}

/*******************************************************************
* NAME        : MsgEnableIrq
* DESCRIPTION : Enables AB interrupts
*/
static void
MsgEnableIrq(void) {
  enable_irq(irq);
  MsgIdle();
  return;
}

/*******************************************************************
* NAME        : MsgDisableIrq
* DESCRIPTION : Disbles AB interrupts
*               To notify the controller we put
*               BUSY signal in the status register
*/
static void
MsgDisableIrq(void) {
  MsgBusy();
  disable_irq(irq);
  return;
}

/******************************************************************
* NAME        : MsgWaitForInit
* DESCRIPTION : Waits for MC to finish initialization
* RETURN      : ABOK or ABERR and errno is set
*/

static int
MsgWaitForInit(void) {
  unsigned char inpStatus;
  int timeout;
  
#ifdef BROKEN_FOR_SMP
  timeout = (int)jiffies+TIMEOUT_LIMIT;
  
  do {
    /* Just poll and poll */
    if ((int)jiffies-timeout>0) {
      abErrno = ERR_AB_NOT_READY;
      return(ABERR);
    }
    inpStatus=INPSTAT();
  } while (inpStatus != ACK_IDLE);

#else

  timeout = AB_RETRIES;
  
  do {
    /* Just poll and poll */
    if (!timeout--) {
      abErrno = ERR_AB_NOT_READY;
      return(ABERR);
    }
    udelay(AB_UDELAY);
    inpStatus=INPSTAT();
  } while (inpStatus != ACK_IDLE);

#endif

  return(ABOK);
}

/*******************************************************************
* NAME        : MsgInit
* DESCRIPTION : Initialize the message module
* ALGORITHM   : Checks for idle card
* RETURN      : ABOK or ABERR
* NOTES       : This routine must be called first
*/
static int
MsgInit(void) {

   /* wait for hardware to become idle */
   if (MsgWaitForInit() != ABOK)
     return(ABERR);

   /* Clear pending interrupts, signal IDLE */
   CLRIRQ();
   OUTSTAT(ACK_IDLE);
   return(ABOK);
}

/*******************************************************************
* NAME        : WaitForSend
* DESCRIPTION : Wait until sending possible
* ALGORITHM   : Wait for status register to turn IDLE. On timeout
*               exit
* RETURN      : ABOK or ABERR
*/
static int
WaitForSend(void)
{
  unsigned inpStatus;
  int timeout;
  
#ifdef BROKEN_FOR_SMP

  timeout = (int)jiffies+TIMEOUT_LIMIT;
  
  /* Wait for status register to IDLE */
  do {
    if ((int)jiffies-timeout>0) {
      OUTSTAT(ACK_IDLE);
      abErrno = ERR_AB_NOT_READY;
      return(ABERR);
    }
    inpStatus=INPSTAT();
  } while (inpStatus != ACK_IDLE);

  return(ABOK);

#else 

  timeout = AB_RETRIES;
  
  /* Wait for status register to IDLE */
  do {
    if (!timeout--) {
      OUTSTAT(ACK_IDLE);
      abErrno = ERR_AB_NOT_READY;
      return(ABERR);
    }
    udelay(AB_UDELAY);
    inpStatus=INPSTAT();
  } while (inpStatus != ACK_IDLE);

  return(ABOK);

#endif


#if 0 /* dead code? */
  for (i=0; i<TIMEOUT_LIMIT; i++) {
     inpStatus=INPSTAT();
     if (ABMSG_ACK(inpStatus) == ACK_IDLE) {
       return(ABOK);
     }
  }

  /* return timeout */
  OUTSTAT(ACK_IDLE);
  abErrno=ERR_AB_NOT_READY;
  return(ABERR);
#endif
}

/*******************************************************************
* NAME        : SendWord
* DESCRIPTION : Send 16 bit word to AB
* ALGORITHM   : Wait for AB ack signal and send word
* RETURN      : ABOK or ABERR
* NOTES       : This routine must be called first
*/
static int
SendWord(
	 unsigned int val,        /* value to send */
	 unsigned int outStatus, /* context (for status register) */
	 int intr)            /* interrupt AB ? */
{
  unsigned int inpStatus;
  int timeout;

  /* send data */
  OUTDATA(val);
  
  /* set status word (might use interrupt status register) */
  if (intr)
    OUTSTATI(outStatus);
  else
    OUTSTAT(outStatus);
  
  
#ifdef BROKEN_FOR_SMP

  /* wait for AB ack. The AB must return the same status as sent */
  timeout = (int)jiffies+TIMEOUT_LIMIT;

  do {
    
    /* Read status register */
    inpStatus=INPSTAT();
    
    /* If same value, ABOK */
    if (ABMSG_ACK(inpStatus) == outStatus) { return(ABOK); }
    
    /* If ERR signal came back, send ERR signal to AB and wait for
     * IDLE signal  */
    if (ABMSG_ACK(inpStatus) == ACK_ERR) {
      inpStatus=INPSTAT();                 /* try again */
      if (ABMSG_ACK(inpStatus) == ACK_ERR) {
	abErrno=ERR_START + ABMSG_ERR(inpStatus);
	OUTSTAT(ACK_ERR);
	
	timeout = (int)jiffies+TIMEOUT_LIMIT;
	do {
	  inpStatus=INPSTAT();
	  if (ABMSG_ACK(inpStatus) == ACK_IDLE)
	    break;
	} while ((int)jiffies-timeout < 0);
	OUTSTAT(ACK_IDLE);
	return(ABERR);
      }
    }

  } while ((int)jiffies-timeout < 0);
  
  OUTSTAT(ACK_IDLE);
  abErrno=ERR_TIMEOUT;
  return(ABERR);

#else

  /* wait for AB ack. The AB must return the same status as sent */
  timeout = AB_RETRIES;

  do {
    
    /* Read status register */
    inpStatus=INPSTAT();
    
    /* If same value, ABOK */
    if (ABMSG_ACK(inpStatus) == outStatus) { return(ABOK); }
    
    /* If ERR signal came back, send ERR signal to AB and wait for
     * IDLE signal  */
    if (ABMSG_ACK(inpStatus) == ACK_ERR) {
      inpStatus=INPSTAT();                 /* try again */
      if (ABMSG_ACK(inpStatus) == ACK_ERR) {
	abErrno=ERR_START + ABMSG_ERR(inpStatus);
	OUTSTAT(ACK_ERR);
	
	timeout = AB_RETRIES;
	do {
	  inpStatus=INPSTAT();
	  if (ABMSG_ACK(inpStatus) == ACK_IDLE)
	    break;
	  udelay(AB_UDELAY);
	} while (timeout--);
	OUTSTAT(ACK_IDLE);
	return(ABERR);
      }
    }

    udelay(AB_UDELAY);
  } while (timeout--);
  
  OUTSTAT(ACK_IDLE);
  abErrno=ERR_TIMEOUT;
  return(ABERR);

#endif

}


/*******************************************************************
* NAME        : MsgSend
* DESCRIPTION : Send message to AB controller
* ALGORITHM   : - Wait for AB IDLE
*               - Send first word with interrupt
*               - Send second word
*               - Send message data
*               - Signal host IDLE
* RETURN      : ABOK or ABERR
*/
static int
MsgSend(ABMSG *msg)    /* message to send */
{
  unsigned outStatus,i,n;
  unsigned short   *msg_word = (unsigned short*)msg;

  pgmTrace();

  if ((irq==0) || (port==0)) {
    abErrno = ERR_AB_NOT_READY;
    return_d(ABERR);
  }
  
  /* wait for AB IDLE */
  if (WaitForSend() == ABERR) {
    msg_word[2]=INPSTAT();
    msg_word[3]=0;
    return_d(ABERR);
  }

  /* Send first word with interrupt */
  if (SendWord(msg_word[0],0,1) == ABERR) {
    msg_word[2]=INPSTAT();
    msg_word[3]=1;
    return_d(ABERR);
  }

  /* send second word */
  if (SendWord(msg_word[1],1,0) == ABERR) {
    msg_word[2]=INPSTAT();
    msg_word[3]=2;
    return_d(ABERR);
  }

  /* send message data */
  n=2 + ((msg->hdr.msgLen+1) >> 1);
  if (n != 2) {
    outStatus=2;
    for (i=2; i<n; i++) {
      if (SendWord(msg_word[i],outStatus,0) == ABERR) {
	msg_word[2]=INPSTAT();
	msg_word[3]=i+1;
	return_d(ABERR);
      }
      NEXTCNT(outStatus);
    }
  }

  /* signal host idle */
  OUTSTAT(ACK_IDLE);
  return_d(ABOK);
}


/*******************************************************************
* NAME        : RecvWord
* DESCRIPTION : Read one word
* ALGORITHM   : Wait until input status matchs the expected status
*               and then read data and ack AB
* RETURN      : ABOK or ABERR
*/
static int 
RecvWord(unsigned short *val, unsigned outStatus) {
  unsigned inpStatus;
  int timeout, timeout2;
  
#ifdef BROKEN_FOR_SMP

  timeout = jiffies+TIMEOUT_LIMIT;
  do {
    
    /* read status */
    inpStatus=INPSTAT();
    
    /* if match expected status, ack AB and exit */
    if (ABMSG_ACK(inpStatus) == outStatus) {
      *val=INPDATA();
      OUTSTAT(inpStatus);
      return(ABOK);
    }
    
    /* If error detected, ack and wait for IDLE signal */
    if (ABMSG_ACK(inpStatus) == ACK_ERR) {
      inpStatus=INPSTAT();       /* try again */
      if (ABMSG_ACK(inpStatus) == ACK_ERR) {
	dbgAlert("errno: 0x%x \n",
	       ERR_START + ABMSG_ERR(inpStatus));
	abErrno = ERR_START + ABMSG_ERR(inpStatus);
	OUTSTAT(ACK_ERR);

	timeout2 = jiffies+TIMEOUT_LIMIT;
	while ((int)jiffies-timeout < 0) {
	  inpStatus=INPSTAT();
	  if (ABMSG_ACK(inpStatus) == ACK_IDLE)
	    break;
	}
	OUTSTAT(ACK_IDLE);
	return(ABERR);
      }
    }
  } while ((int)jiffies-timeout < 0);
  
  /* exit on timeout */
  dbgAlert(": INPSTAT = %04X\n", inpStatus);
  OUTSTAT(ACK_IDLE);
  abErrno=ERR_TIMEOUT;
  return(ABERR);

#else
  timeout = AB_RETRIES;

  do {
    
    /* read status */
    inpStatus=INPSTAT();
    
    /* if match expected status, ack AB and exit */
    if (ABMSG_ACK(inpStatus) == outStatus) {
      *val=INPDATA();
      OUTSTAT(inpStatus);
      return(ABOK);
    }
    
    /* If error detected, ack and wait for IDLE signal */
    if (ABMSG_ACK(inpStatus) == ACK_ERR) {
      inpStatus=INPSTAT();       /* try again */
      if (ABMSG_ACK(inpStatus) == ACK_ERR) {
	dbgAlert("errno: 0x%x \n",
	       ERR_START + ABMSG_ERR(inpStatus));
	abErrno = ERR_START + ABMSG_ERR(inpStatus);
	OUTSTAT(ACK_ERR);

	timeout2 = AB_RETRIES;
	
	do {
	  inpStatus=INPSTAT();
	  if (ABMSG_ACK(inpStatus) == ACK_IDLE)
	    break;
	  udelay(AB_UDELAY);
	} while (timeout2--);

	OUTSTAT(ACK_IDLE);
	return(ABERR);
      }
    }
    udelay(AB_UDELAY);
  } while (timeout--);
  
  /* exit on timeout */
  dbgAlert(": INPSTAT = %04X\n", inpStatus);
  OUTSTAT(ACK_IDLE);
  abErrno=ERR_TIMEOUT;
  return(ABERR);

#endif

}




/*******************************************************************
* NAME        : MsgRecv
* DESCRIPTION : Read message from AB
* ALGORITHM   : - read first word
*               - read second word
*               - read data
*               - wait for AB idle
*               - signal idle
* RETURN      : ABOK or ABERR
*/
static int
MsgRecv(ABMSG *msg)    /* message buffer */
{
  unsigned i,n,outStatus;
  unsigned short   *msg_word = (unsigned short*)msg;
  
  pgmTrace();

  if ((irq==0) || (port==0)) {
    abErrno = ERR_AB_NOT_READY;
    return_d(ABERR);
  }
  
  /* read first word */
  if (RecvWord(msg_word,0) == ABERR) {
    msg_word[2]=INPSTAT();
    msg_word[3]=1;
    return_d(ABERR);
  }
  
  /* read second word */
  if (RecvWord(msg_word+1,1) == ABERR) {
    msg_word[2]=INPSTAT();
    msg_word[3]=2;
    return_d(ABERR);
  }
  
  /* read message data */
  n=2 + ((msg->hdr.msgLen+1) >> 1);
  if (n != 2) {
    outStatus=2;
    for (i=2; i<n; i++) {
      if (RecvWord(msg_word+i,outStatus) == ABERR) {
	msg_word[2]=INPSTAT();
	msg_word[3]=i+2;
	return_d(ABERR);
      }
      NEXTCNT(outStatus);
    }
  }
  
  /* wait for AB idle */
  if (WaitForSend() == ABERR) {
    msg_word[2]=INPSTAT();
    msg_word[3]=0;
    return_d(ABERR);
  }
  
  /*
   * _HACK_
   * card does not seem to set this
   * in the case of C2M_MSG_TO_DRIVER.
   * Since all messages currently in use
   * set this bit, I will set it manually
   * as a temporary hack.
   */
  
  if (msg->hdr.major == C2M_MSG_TO_DRIVER) msg->hdr.minor = 1;
  
  OUTSTAT(ACK_IDLE);
  return_d(ABOK);
}

static void
abPerror(char* text) {

  pgmTrace();

   if(text) printk(DBG_PRE_FMT "%s : ", DBG_PRE_ARGS, text);
   else     printk(DBG_PRE_FMT, DBG_PRE_ARGS);

   switch (abErrno) {
    case ERR_UNKNOWN_OPCODE:
      printk("unknown opcode\n");
      break;
    case ERR_UNKNOWN_DEVID:
      printk("unknown device id\n");
      break;
    case ERR_RESEND:
      printk("resend message\n");
      break;
    case ERR_BUFFER_FULL:
      printk("AB buffer full\n");
      break;
    case ERR_LEN_NOT_MATCH:
      printk("message length not match\n");
      break;
    case ERR_BUSY:
      printk("device busy\n");
      break;
    case ERR_TIMEOUT:
      printk("timeout\n");
      break;
    case ERR_INVALID_PORT:
      printk("invalid port\n");
      break;
    case ERR_INVALID_IRQ:
      printk("invalid irq\n");
      break;
    case ERR_AB_NOT_READY:
      printk("MC not ready\n");
      break;
    case ERR_NOMEM:
      printk("no memory\n");
      break;
    case ERR_NO_DRIVER:
      printk("no driver handler\n");
      break;
    case ERR_TOO_MANY_DEVICES:
      printk("too many devices\n");
      break;
    case ERR_DEVICE_EXISTS:
      printk("device already exists\n");
      break;
    case ERR_NOT_IMPLEMENTED:
      printk("not implemented yet\n");
      break;
    case ERR_DRIVER_EXISTS:
      printk("driver exists\n");
      break;
    case ERR_NO_AB_MANAGER:
      printk("ab manager not installed\n");
      break;
    case ERR_INTR_USED:
      printk("ab interrupt used\n");
      break;
    case ERR_SELF_TEST:
      printk("ab self test error\n");
      break;
    default:
      printk("**UNKNOWN** abErrno\n");
   }
  return_v;
}

static int
abConnect (int newIrq, int newPort) {

  pgmTrace();

  if (irq) free_irq(irq IRQ_FILLER); /* ZZZ */
  if (port) release_region(port, 12);

  irq = newIrq;
  port = newPort;

  /* sanity check */
  if ((port != MC_PORT1) && (port != MC_PORT2) && (port != MC_PORT3)) {
    irq = 0;
    port = 0;
    abErrno = ERR_INVALID_PORT;
    return_d(ABERR);
  }
  if ((irq != MC_IRQ1) && (irq != MC_IRQ2) && (irq != MC_IRQ3)) {
    irq = 0;
    port = 0;
    abErrno = ERR_INVALID_IRQ;
    return_d(ABERR);
  }
  
  if(request_irq(irq, &abInterrupt, 0, AB_MODULE_NAME IRQ_FILLER) < 0) { /* ZZZ */
    printk("%s can't get IRQ %d\n", __FILE__, irq);
    irq = 0;
    port = 0;
    abErrno = ERR_INTR_USED;
    return_d(ABERR);
  }

  if (check_region(port, 12)) {
    free_irq(irq IRQ_FILLER); /* ZZZ */
    printk("%s IO port not available\n", __FILE__);
    abErrno = ERR_INVALID_PORT;
    return_d(ABERR);
  }

  request_region(port, 12, AB_MODULE_NAME);
  
  /* see if the CATC card exists */
  
  if(MsgInit() == ABERR) {
    printk("%s cannot find card\n", __FILE__);
    free_irq(irq IRQ_FILLER); /* ZZZ */
    release_region(port, 12);
    irq=0;
    port=0;
    abErrno = ERR_AB_NOT_READY;
    return_d(ABERR);
  }
  printk("abus found CATC card at 0x%03X, using IRQ %d\n", port, irq);
  return_d(ABOK);
}

static int
abDisconnect (unsigned int oldIrq, unsigned int oldPort) {

  pgmTrace();

  if ((oldIrq==irq) && (oldPort==port)) {
    MsgBusy();
    if (irq) free_irq(irq IRQ_FILLER); /* ZZZ */
    if (port) release_region(port, 12);
    irq = 0;
    port = 0;
    dbgSyscall("\n");
    return_d(ABOK);
  }
  else {
    dbgAlert("trying to disconnect from wrong port or irq\n");
    return_d(ABERR);
  }
}

static int abFindClientByPid(int pid, struct file *file) {
  int ii;

  pgmTrace();

  ii = 1;
  while ((ii<AB_CLIENT_NUM) && (abClient[ii].pid != pid)
	 && (abClient[ii].file != file)) ii++;

  if (ii<AB_CLIENT_NUM) {
    if (file->private_data != &abClient[ii]) {
      dbgAlert("file->private_data != &abClient[%d]", ii);
    }
    return_d(ii);
  }

  return_d(-1);			/* couldn't find match */
}

static int
abFindClientByDevId(unsigned char devId,
		    unsigned char major,
		    unsigned char minor,
		    unsigned char opcode) {

  pgmTrace();

  switch (major) {
  case C2M_LONG_ID:
  case C2M_ID:
  case C2M_TYPE:
  case C2M_SELF_TEST:
  case C2M_STATUS:
  case C2M_END_TABLE:
  case C2M_DISCONNECT:
  case C2M_AB_EMULATION:
  case C2M_MSG_TO_MONITOR:
    if (abMgrNum) {
      return_d(abMgrNum);
    }
    return_d(-1);

  case C2M_MSG_TO_DRIVER:
    if (minor) {
      switch (opcode) {
      case AB_APPL_HDW_SIG:
      case AB_APPL_TST_RPL:
      case AB_APPL_STAT_MSG:
      case AB_APPL_TST:
      case AB_ATTN:
      case AB_IDENT_RPL:
      case AB_RES_E2:
      case AB_CAP_RPL:
      case AB_RES_E4:
      case AB_RSRC_REQ:
      case AB_PWR_USG_RPL:
      case AB_RES_E7:
      case AB_BW_USG_RPL:
	if (abMgrNum) {
	  return_d(abMgrNum);
	}
	return_d(-1);

      default:
	if (abDev[devId].clientNum) {
	  return_d(abDev[devId].clientNum);
	}
	return_d(-1);
      }
    }
  default:
    return_d(-1);
  }
}

static int
abMsgToClient (int destNum, ABKMSG *srcPtr) {
   int count;
   ABKMSG *destPtr;
   int destIn;
   int destOut;

   pgmTrace();

   msgTrace("clienNum %02X  major %02X minor %02X  devId %02X\n",
	    destNum, srcPtr->msg.hdr.major,
	    srcPtr->msg.hdr.minor, srcPtr->msg.hdr.devId);

   if ((destNum>AB_CLIENT_NUM) || (destNum<0) ||
       (abClient[destNum].pid==0)) {
     return_d(-1);
   }
   
   destIn = abClient[destNum].bufferIn;
   destOut = abClient[destNum].bufferOut;
   
   destPtr = &abClient[destNum].buffer[destIn];
   
   destIn += 1;
   if (destIn == AB_MSG_NUM) destIn = 0;
   
   if (destIn == destOut) {
#ifdef I_LIKE_CRASHED_KERNELS
     /* This can suck up all CPU time and cause buffer overruns */
      dbgAlert("buffer overrun on client #%02X\n",  destNum);
#endif
      destOut += 1;
      if (destOut == AB_MSG_NUM) destOut = 0;     
   }
   
   abClient[destNum].bufferIn = destIn;
   abClient[destNum].bufferOut = destOut;
   
   count = ((*srcPtr).msg.hdr.msgLen & 0x7F) + 4;
   memcpy(destPtr, srcPtr, count);

   /* copy client number */
   destPtr->clientNum = srcPtr->clientNum;
   
   if ((destNum>0) && (abClient[destNum].select_wait)) {
     abClient[destNum].select_wait=0;
     wake_up_interruptible(&abClient[destNum].queue);
   }

   return_d(0);
}

static void abAsyncFlush (void) {
  int count;
  ABKMSG *srcPtr;
  unsigned int srcOut;
  int destNum;
  int major;
  int minor;
  int devId;
  unsigned char opcode;

  pgmTrace();

  while (1) {
    srcOut = abClient[0].bufferOut;
    if (srcOut == abClient[0].bufferIn) break;
    
    srcPtr = &abClient[0].buffer[srcOut];
    
    srcOut += 1;
    if (srcOut == AB_MSG_NUM) srcOut = 0;
    abClient[0].bufferOut = srcOut;
    
    count = (*srcPtr).msg.hdr.msgLen;
    major = (*srcPtr).msg.hdr.major;
    minor = (*srcPtr).msg.hdr.minor;
    devId = (*srcPtr).msg.hdr.devId;
    opcode= (*srcPtr).msg.data[4];

    /* put it in the correct client's space */
    destNum = abFindClientByDevId(devId, major, minor, opcode);

    if ((destNum != abMgrNum) && (major == C2M_MSG_TO_DRIVER))
       (*srcPtr).msg.hdr.major = M2D_MSG_TO_DRIVER;
    
    if (abMonNum && (destNum != abMonNum)) {
       abMsgToClient(abMonNum, srcPtr);
    }
    
    if (destNum>0) {
       abMsgToClient(destNum, srcPtr);
    }
  }
  return_v;
}

static void abInterrupt() {
  int bufferIn;
  int bufferOut;
  ABKMSG *msgPtr;
  int major;
  int minor;
  int devId;
  int count;
  int abClientNum;
  unsigned char opcode;

  MsgDisableIrq();
  CLRIRQ();

  pgmTrace();

  bufferIn = abClient[0].bufferIn;
  bufferOut = abClient[0].bufferOut;
  msgPtr = &abClient[0].buffer[bufferIn];

  if (MsgRecv(&msgPtr->msg) == ABERR) {
     abPerror("abInterrupt() - ABERR");
     MsgEnableIrq();
     return_v;
  }
  
  bufferIn += 1;
  if (bufferIn == AB_MSG_NUM) bufferIn = 0;
  
  if (bufferIn == bufferOut) {
#ifdef I_LIKE_CRASHED_KERNELS
    /*
     * This can suck up all CPU time and cause buffer overruns
     * Additionally, in an interrupt like this it seems to cause
     * kernel corruption.
     */
    dbgAlert("buffer overrun on client #%02X\n", 0);

    bufferOut += 1;
    if (bufferOut == AB_MSG_NUM) bufferOut = 0;
#else
    MsgEnableIrq();
    return_v;
#endif
  }
  
  count = msgPtr->msg.hdr.msgLen;
  major = msgPtr->msg.hdr.major;
  minor = msgPtr->msg.hdr.minor;
  devId = msgPtr->msg.hdr.devId;
  opcode= msgPtr->msg.data[0];

  abClientNum = abFindClientByDevId(devId, major, minor, opcode);

  abClient[0].bufferIn = bufferIn;
  abClient[0].bufferOut = bufferOut;

  if ((abClientNum>0) && (abClient[abClientNum].select_wait)) {
     abClient[abClientNum].select_wait=0;
     wake_up_interruptible(&abClient[abClientNum].queue);
  }

  MsgEnableIrq();
  return_v;
}

static int abOpen(struct inode* inode,
		     struct file* file)
{
  ABKMSG kmsg;
  int abClientNum;
  
  pgmTrace();

  abAsyncFlush();

  abClientNum=1;
  while ((abClientNum<AB_CLIENT_NUM) && (abClient[abClientNum].pid !=0))
    abClientNum++;
  
  if(!(abClientNum<AB_CLIENT_NUM)) { /* too many clients */
    return_d(-EBUSY);
  }
  
  if (abClient[abClientNum].buffer) {
    dbgAlert(" abClient.buffer != NULL ?!?  Attempting to free it.\n");

    vfree (abClient[abClientNum].buffer);
    abClient[abClientNum].buffer = NULL;
  }
  
  abClient[abClientNum].pid = current->pid;
  abClient[abClientNum].file = file;
  abClient[abClientNum].bufferIn = 0;
  abClient[abClientNum].bufferOut = 0;
  abClient[abClientNum].select_wait = 0;
  abClient[abClientNum].queue = NULL;
  
  abClient[abClientNum].buffer =
     vmalloc (AB_MSG_NUM * sizeof(*abClient[abClientNum].buffer));

  if (!abClient[abClientNum].buffer) {
    abClient[abClientNum].pid = 0;
    abClient[abClientNum].file = NULL;
    abClient[abClientNum].buffer = NULL;
    dbgAlert(" vmalloc failed\n");
    return_d(-ENOMEM);
  }
  
  kmsg.msg.hdr.major  = K2D_CLIENT_CONNECT;
  kmsg.msg.hdr.minor  = 1;		/* 1 is connect */
  kmsg.msg.hdr.msgLen = 0;
  kmsg.msg.hdr.devId  = 0;
  kmsg.clientNum  = abClientNum;
  
  if (abMgrNum) abMsgToClient(abMgrNum, &kmsg);
  if (abMonNum) abMsgToClient(abMonNum, &kmsg);

  file->private_data = &abClient[abClientNum];

  dbgSyscall("pid %d, file* 0x%08X, abClientNum %d\n",
	 current->pid, (unsigned int)file, abClientNum);
  
  MOD_INC_USE_COUNT;

  return_d(0);
}
		     
static void abRelease(struct inode* inode,
			 struct file* file)
{
  ABKMSG kmsg;
  int abClientNum;
  int ii;

  pgmTrace();

  abAsyncFlush();

  abClientNum = abFindClientByPid(current->pid, file);

  if (abClientNum<0) {		/* This shouldn't happen */
    dbgAlert(" Can't match call to an existing client!\n");
    return_v;
  }

  dbgSyscall("pid %d, file *0x%08X, abClientNum %d\n",
	 current->pid, (unsigned int)file, abClientNum);
  
  /* cleanup for the next user */
  abClient[abClientNum].pid = 0;
  abClient[abClientNum].select_wait = 0;
  abClient[abClientNum].file = NULL;

  vfree(abClient[abClientNum].buffer);
  abClient[abClientNum].buffer = NULL;

  if (abClientNum == abMonNum) abMonNum = 0;

  if (abClientNum==abMgrNum) {
    abMgrNum = 0;

#if 0 
     /*
      * If there are still enabled devices this causes
      * the CATC card to stall the bus.  All devices must
      * be disabled (with an M2C_ENABLE(0) message) before
      * silencing the CATC card.  Since I don't want to
      * implement all this in the kernel and we can't really
      * count on abusd (as it may have been SIG_KILL'ed) I
      * am avoiding the problem for now and not disconnecting
      * from the CATC card.  The only side effect I can think
      * of at this time is a bit of cpu time is lost and some
      * of the enable/disable type state is not quite right.
      */
     
     abDisconnect(irq, port);	/* disconnect from card */
#endif
     
  }
  else {
    kmsg.msg.hdr.major  = K2D_CLIENT_CONNECT;
    kmsg.msg.hdr.minor  = 0;		/* 0 is disconnect */
    kmsg.msg.hdr.msgLen = 0;
    kmsg.msg.hdr.devId  = 0;
    kmsg.clientNum  = abClientNum;
    
    if (abMgrNum) abMsgToClient(abMgrNum, &kmsg);
    if (abMonNum) abMsgToClient(abMonNum, &kmsg);
    
    for (ii=0; ii<AB_DEV_NUM; ii++) {
      if (abDev[ii].clientNum == abClientNum) {
	abDev[ii].clientNum = 0;
	dbgMsg3("abDev[%d].status=0x%02X\n", ii, abDev[ii].status);
	abDev[ii].status &= ~AB_STATUS_LINKED;
	dbgMsg3("abDev[%d].status=0x%02X\n", ii, abDev[ii].status);
      }
    }
  }

  MOD_DEC_USE_COUNT;

  return_v;
}

static int abRead(struct inode* inode,
		     struct file* file,
		     char* buf,
		     int count)
{
  ABKMSG *msgPtr;
  unsigned int bufferOut;
  int abClientNum;
  int msgLen;

  pgmTrace();

  abAsyncFlush();

  abClientNum = abFindClientByPid(current->pid, file);

  if (abClientNum<0) {		/* This shouldn't happen */
    dbgAlert("Can't match call to an existing client!\n");
    return_d(-EINVAL);
  }

  if((count != sizeof(ABMSG)) && (count != sizeof(ABKMSG))) {
    dbgWarn("Wrong buffer size for read.  "
	   "count = %d, sizeof(ABKMSG)=%d\n", count, sizeof(ABKMSG));
    return_d(-EINVAL);
  }
  
  bufferOut = abClient[abClientNum].bufferOut;
  if (bufferOut == abClient[abClientNum].bufferIn) {
    return_d(0);
  }

  msgPtr = &abClient[abClientNum].buffer[bufferOut];

  bufferOut += 1;
  if (bufferOut == AB_MSG_NUM) bufferOut = 0;
  abClient[abClientNum].bufferOut = bufferOut;

  if (count == sizeof(ABKMSG)) {
    /* user wants clientNum */
    put_fs_long(msgPtr->clientNum, &((ABKMSG*)buf)->clientNum);
  }

  /* put message in the user's space */
  msgLen = ((*msgPtr).msg.hdr.msgLen & 0x7F) + 4;
  memcpy_tofs(buf, msgPtr, msgLen);

  return_d(count);
}


static int abWrite(struct inode* inode,
		      struct file* file,
		      const char* buf,
		      int count)
{
  int major;
  int minor;
  int devId;
  int msgLen;
  int bufferOut;
  int bufferIn;
  ABKMSG *msgPtr;
  int abClientNum;

 if((count != sizeof(ABMSG)) && (count != sizeof(ABKMSG))) {
     dbgWarn("Wrong buffer size for write: count=%d, "
	    "sizeof(ABMSG)=%d, sizeof(ABKMSG)=%d\n", count, sizeof(ABMSG), 
	    sizeof(ABKMSG));
     abAsyncFlush();
     return_d(-EINVAL);
  }
  
  pgmTrace();

  abAsyncFlush();

  if ((irq==0) || (port==0) || (abMgrNum==0)) {
    dbgWarn("Not attached or no deamon\n");
    return_d(-EUNATCH);
  }

  abClientNum = abFindClientByPid(current->pid, file);

  if (abClientNum<0) {		/* This shouldn't happen */
    dbgAlert("Can't match call to an existing client!\n");
    return_d(-EINVAL);
  }

 if((count != sizeof(ABMSG)) && (count != sizeof(ABKMSG))) {
     dbgWarn("Wrong buffer size for write: count=%d, "
	    "sizeof(ABMSG)=%d, sizeof(ABKMSG)=%d\n", count, sizeof(ABMSG), 
	    sizeof(ABKMSG));
     return_d(-EINVAL);
  }
  
  if (count == sizeof(ABKMSG)) {
    /* user wants to add a clientNum */
    writeMsg.clientNum = get_fs_long(&((ABKMSG*)buf)->clientNum);
  }

  /* get the message */
  memcpy_fromfs(&writeMsg, buf, 4); /* get the header */
  msgLen = writeMsg.msg.hdr.msgLen;
  major = writeMsg.msg.hdr.major;
  minor = writeMsg.msg.hdr.minor;
  devId = writeMsg.msg.hdr.devId;

  msgTrace("clienNum %02X  major %02X minor %02X  devId %02X\n",
	   abClientNum, major, minor, devId);
  
  if (msgLen>132) {
    return_d(-EINVAL);
  }
  if (msgLen)			/* get the rest */
    memcpy_fromfs((unsigned char *)(&writeMsg)+4, buf+4, msgLen);
  
  switch (major) {

  case M2C_GET_TABLE:
  case M2C_GET_LONG_ID:
  case M2C_GET_ID:
  case M2C_GET_TYPE:
  case M2C_GET_SELF_TEST:
  case M2C_GET_STATUS: {
    if ((abClientNum != abMgrNum) && (abClientNum != abMonNum)) {
        return_d(-EPERM);
    }
     
     if (MsgSend(&writeMsg.msg) == ABERR) {
        abPerror("write - 1 - MsgSend");
        if (abErrno == ERR_TIMEOUT) {
           return_d(-EBUSY);
	}
	return_d(-EIO);
     }
     
     bufferIn = abClient[abClientNum].bufferIn;
     bufferOut = abClient[abClientNum].bufferOut;
     
     do {
        msgPtr = &abClient[abClientNum].buffer[bufferIn];
        if (MsgRecv(&msgPtr->msg) == ABERR) {
           abPerror("M2C_GET_COMP_DEV_TBL - MsgRecv");
           if (abErrno == ERR_TIMEOUT) {
              return_d(-ENODATA);
	   }
	   return_d(-EIO);
        }
        
        if (abMonNum && (abMonNum != abClientNum)) {
           abMsgToClient (abMonNum, msgPtr);
        }
        
        bufferIn += 1;
        if (bufferIn == AB_MSG_NUM) bufferIn = 0;
        
        if (bufferIn == bufferOut) {
           dbgAlert("buffer overrun on client #%02X\n", 0);
           bufferOut += 1;
           if (bufferOut == AB_MSG_NUM) bufferOut = 0;     
        }
        
        abClient[abClientNum].bufferIn = bufferIn;
        abClient[abClientNum].bufferOut = bufferOut;
        
     } while (((*msgPtr).msg.hdr.major != C2M_END_TABLE) &&
              (major == M2C_GET_TABLE));
     
     return_d(count);
  }
     
  case M2C_RESET:
    dbgCmd("XXX M2C_RESET !!!\n");

  case M2C_AB_EMULATION:
  case M2C_ENABLE:
  case M2C_MSG_TO_DEVICE: {
    if ((abClientNum != abMgrNum) && (abClientNum != abMonNum)) {
      dbgWarn(" illegal write - EPERM\n");
      return_d(-EPERM);
    }

    if (MsgSend(&writeMsg.msg) == ABERR) {
      _pgmTrace();
      abPerror("write 2");
      if (abErrno == ERR_TIMEOUT) {
	_return_d(-EBUSY);
      }
      else {
	_return_d(-EIO);
      }
    }
    return_d(count);
  }

  case M2C_KBD_EMULATION:
  case M2C_KBD_TABLE:
  case M2C_KBD_LOOPBACK:
    if ((abClientNum != abMgrNum) && (abClientNum != abMonNum)) {
      return_d(-EPERM);
    }

    return_d(-EINVAL);

  case M2D_LINK_REPLY:
  case M2D_LONG_ID:
  case M2D_ID:
  case M2D_TYPE:
  case M2D_STATUS:
  case M2D_MSG_TO_DRIVER:
  case M2D_DISCONNECT:
  case M2D_LINK_APPROVE_ACK:
    {
      /* clientNum is stored in ABKMSG */
      int clientNum;
      
      dbgCmd("M2D_LINK_REPLY, M2D_LONG_ID, M2D_ID, M2D_TYPE, M2D_STATUS\n");
      dbgCmd("M2D_MSG_TO_DRIVER, M2D_DISCONNECT or M2D_LINK_APPROVE_ACK\n");
      
      clientNum = writeMsg.clientNum;
      
      if ((abClientNum != abMgrNum) && (abClientNum != abMonNum)) {
	return_d(-EPERM);
      }
      
      if (count != sizeof(ABKMSG)) {
	return_d(-EINVAL);
      }
      
      abMsgToClient (clientNum, &writeMsg);
      
      if (abMonNum && (abMonNum != abDev[devId].clientNum)) {
	abMsgToClient (abMonNum, &writeMsg);
      }
      return_d(count);
    }
    
  case D2M_LINK_REQUEST:
  case D2M_LINK_APPROVE:
  case D2M_GET_LONG_ID:
  case D2M_GET_ID:
  case D2M_GET_TYPE:
  case D2M_GET_STATUS:
  case D2M_ENABLE:
  case D2M_DISCONNECT:
  case D2M_RESET:
     {
        /* clientNum is passed in ABKMSG */

        if (!abMgrNum) {
	  return_d(-EUNATCH);
	}

        writeMsg.clientNum = abClientNum;
        abMsgToClient (abMgrNum, &writeMsg);
        
        if (abMonNum && (abMonNum != abMgrNum)) {
           abMsgToClient (abMonNum, &writeMsg);
        }
        
        return_d(count);
     }

  case D2M_MSG_TO_DEVICE:
     {

        pgmTrace();

        /* Need to make sure that client is mgr, mon, or owner */

        if ((abDev[devId].clientNum != abClientNum) &&
            (abClientNum != abMgrNum) &&
            (abClientNum != abMonNum)) {
	  _return_d(-EPERM);
	}
        
        /* check that the dev is enabled if not mgr or mon */

	dbgMsg3("abDev[%d].status=0x%02X\n", devId, abDev[devId].status);

        if ((abClientNum!=abMgrNum) && (abClientNum != abMonNum) && 
	    !(abDev[devId].status & AB_STATUS_ENABLE)) {
	  _return_d(-EBUSY);
	}

	writeMsg.msg.hdr.major = M2C_MSG_TO_DEVICE; /* translate opcode */
        if (MsgSend(&writeMsg.msg) == ABERR) {
           abPerror("write - 3 - MsgSend");
           if (abErrno == ERR_TIMEOUT) {
              _return_d(-EBUSY);
	   }
           else {
              _return_d(-EIO);
	   }
        }
        return_d(count);
     }
     
  default:
     return_d(-EINVAL);
  }
  return_d(-EINVAL);
}


static int abIoctl(struct inode* inode,
		      struct file* file,
		      unsigned int cmd,
		      unsigned long data)
{
  int abClientNum;

  pgmTrace();

  abAsyncFlush();
  
  abClientNum = abFindClientByPid(current->pid, file);

  if (abClientNum<0) {		/* This shouldn't happen */
    dbgAlert("Can't match call to an existing client!\n");
    return_d(-EINVAL);
  }

  if ((abMgrNum) && (abClientNum != abMgrNum) &&
      (abClientNum != abMonNum)) {
    return_d(-EPERM);
  }

  switch (cmd) {
  case ABIO_I_AM_MANAGER: {
    dbgCmd("ABIO_I_AM_MANAGER\n");
    if (!abMgrNum) {
      abMgrNum = abClientNum;
      /* XXX inform daemon of already connected clients */
      return_d(0);
    }
    return_d(-EBUSY);
  }

  case ABIO_CONNECT_TO_CONTROLLER: {
    unsigned long *dataPtr;
    unsigned long newIrq;
    unsigned long newPort;

    dbgCmd("ABIO_CONNECT_TO_CONTROLLER\n");

    if (abClientNum != abMgrNum) {
      return_d(-EPERM);
    }

    dataPtr = (unsigned long *)data;
    newIrq  = get_fs_long(dataPtr++);
    newPort = get_fs_long(dataPtr++);

    if (abConnect(newIrq, newPort)==ABOK) {
      return_d(0);
    }
    return_d(-ENODEV);
  }

  case ABIO_DISCONNECT_FROM_CONTROLLER: {
    unsigned long *dataPtr;
    unsigned long oldIrq;
    unsigned long oldPort;

    dbgCmd("ABIO_DISCONNECT_FROM_CONTROLLER\n");

    if (abClientNum != abMgrNum) {
      return_d(-EPERM);
    }

    dataPtr = (unsigned long *)data;
    oldIrq  = get_fs_long(dataPtr++);
    oldPort = get_fs_long(dataPtr++);

    if (abDisconnect(oldIrq, oldPort)==ABOK) {
      return_d(0);
    }

    return_d(-ENODEV);
  }
    
  case ABIO_ASSGN_MONITOR: {
    unsigned long *dataPtr;
    unsigned long clientNum;

    dbgCmd("ABIO_ASSGN_MONITOR\n");

    if (abClientNum != abMgrNum) {
      return_d(-EPERM);
    }

    dataPtr = (unsigned long *)data;
    clientNum = get_fs_long(dataPtr++);

    if ((clientNum == 0) ||
	((abClient[clientNum].pid != 0) &&
	(abClient[clientNum].file != NULL))) {
      abMonNum = clientNum;
      return_d(0);
    }
    return_d(-EINVAL);
  }

  case ABIO_ASSGN_DEV_TO_CLIENT: {
    unsigned long *dataPtr;
    unsigned long clientNum;
    unsigned long devNum;

    dbgCmd("ABIO_ASSGN_DEV_TO_CLIENT\n");

    if (abClientNum != abMgrNum) {
      return_d(-EPERM);
    }

    dataPtr = (unsigned long *)data;
    devNum = get_fs_long(dataPtr++);
    clientNum = get_fs_long(dataPtr++);

    dbgMsg3("abDev[%d].status=0x%02X\n", (int)devNum, abDev[devNum].status);

    if ((abDev[devNum].status != 0) &&
	(abClient[clientNum].pid != 0) &&
	(abClient[clientNum].file != NULL)) {
      abDev[devNum].clientNum = clientNum;
      return_d(0);
    }
    return_d(-EINVAL);
  }

  case ABIO_SET_DEV_STATUS: {
    unsigned long *dataPtr;
    unsigned long devNum;
    unsigned long devStatus;

    pgmTrace();

    dbgCmd("ABIO_SET_DEV_STATUS\n");

    if (abClientNum != abMgrNum) {
      return_d(-EPERM);
    }

    dataPtr = (unsigned long *)data;
    devNum = get_fs_long(dataPtr++);
    devStatus = get_fs_long(dataPtr++);

    abDev[devNum].status = devStatus;

    dbgMsg2("abDev[%d].status=0x%02X\n", (int)devNum, abDev[devNum].status);

    return_d(0);
  }

  case ABIO_ENABLE_IRQ:
    dbgCmd("MsgEnableIrq\n");
    MsgEnableIrq();
    return_d(0);

  case ABIO_DISABLE_IRQ:
    dbgCmd("MsgDisableIrq\n");
    MsgDisableIrq();
    return_d(0);

  case ABIO_MSG_BUSY:
    dbgCmd("MsgBusy\n");
    MsgBusy();
    return_d(0);

  case ABIO_MSG_IDLE:
    dbgCmd("MsgIdle\n");
    MsgIdle();
    return_d(0);
    
  case ABIO_SEND_TIME:
    dbgCmd("ABIO_SEND_TIME\n");

    /*
     * Send kernel struct timeval xtime
     * to device in A.b time resource format.
     */

    return_d(0);

  default:
    dbgWarn("invalid cmd %04X\n", cmd);
    return_d(-EINVAL);
  }
  return_d(0);
}

static int abSelect(struct inode *inode, struct file *file,
                         int sel_type, select_table *wait) {
  int abClientNum;

  pgmTrace();

  abAsyncFlush();

  abClientNum = abFindClientByPid(current->pid, file);
  
  if (sel_type != SEL_IN) {
    return_d(0);
  }

  if (abClientNum<0) {		/* This shouldn't happen */
    dbgAlert("Can't match call to an existing client!\n");
    return_d(-EINVAL);
  }

  cli();
  if (abClient[abClientNum].bufferIn != abClient[abClientNum].bufferOut) {
    sti();
    return_d(1);
  }

  abClient[abClientNum].select_wait = 1;
  select_wait(&abClient[abClientNum].queue, wait);
  sti();

  return_d(0);
}

static struct file_operations fop_abus;
   
static int
abInit() {
  int i;

  pgmTrace();

  memset (&fop_abus, 0, sizeof (fop_abus));
  
  fop_abus.read=abRead;
  fop_abus.write=abWrite;
  fop_abus.select=abSelect; 
  fop_abus.ioctl=abIoctl;
  fop_abus.open=abOpen;
  fop_abus.release=abRelease;

  printk("ACCESS.bus host controller module for CATC A.b125i\n");
  printk("abusk.c: 0.9.2 Oct 24, 1997 Tyson Sawyer,  tyson@rwii.com\n");
  
  abClient[0].pid = 0;
  abClient[0].file = NULL;
  abClient[0].bufferIn = 0;
  abClient[0].bufferOut = 0;
  abClient[0].select_wait = 0;
  abClient[0].queue = NULL;
  
  abClient[0].buffer = vmalloc (AB_MSG_NUM * sizeof(*abClient[0].buffer));
  
  if (!abClient[0].buffer) {
    dbgAlert("vmalloc failed\n");
    return_d(-1);
  }
  
  for (i=1; i<AB_CLIENT_NUM; i++) {
    abClient[i].pid = 0;
    abClient[i].file = NULL;
    abClient[i].buffer = NULL;
    abClient[i].bufferIn = 0;
    abClient[i].bufferOut = 0;
    abClient[i].select_wait = 0;
    abClient[i].queue = NULL;
  }

  for (i=0; i<AB_DEV_NUM; i++) {
    abDev[i].clientNum = 0;
    abDev[i].status = 0;
    dbgMsg3("abDev[%d].status=0x%02X\n", i, abDev[i].status);
  }

  abMgrNum = 0;			/* no manager yet */
  abMonNum = 0;			/* no monitor yet */

  if(register_chrdev(AB_MAJOR, AB_MODULE_NAME, &fop_abus)) {
     vfree(abClient[0].buffer);
     abClient[0].buffer = NULL;
     printk("abInit: cannot register character device\n");
     return_d(-1);
  }

  return_d(0);
}

int init_module(void) {
  pgmTrace();
  return_d(abInit());
}

void cleanup_module(void) {
  int clientNum;

  pgmTrace();

  printk("unloading ab module\n");
  abDisconnect(irq, port);

  for (clientNum = 0; clientNum<AB_CLIENT_NUM; clientNum++) {
    if (abClient[clientNum].buffer) {
      vfree(abClient[clientNum].buffer);
    }
  }

  unregister_chrdev(AB_MAJOR, AB_MODULE_NAME);
  return_v;
}
