
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



/**********************************************************************
 *
 *  PROJECT:  Task Control
 *
 * PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
 * (c) Copyright 1993 Christopher Fedor. All rights reserved.
 *
 *  MODULE: Message
 *
 *  FILE: msg.c
 *
 *  ABSTRACT: Message and Receive Queue Routines.
 *
 *  EXPORTS:
 *
 *  HISTORY:
 *  $Log: msg.c,v $
 *  Revision 1.6  1997/04/11 18:56:57  tyson
 *  minor fixes and chasing TCX segv
 *
 *  Revision 1.5  1997/03/26 00:50:13  tyson
 *  #$%^&^$ SunOS doesnt support strerror()
 *
 *  Revision 1.4  1997/03/25 21:44:52  tyson
 *  Many bug fixes.
 *
 *  Revision 1.3  1997/03/11 17:16:47  tyson
 *  added IR simulation and other work
 *
 *  Revision 1.2  1997/02/22 15:43:01  thrun
 *  Fixed some problems that caused compiler warnings.
 *
 *  Revision 1.1.1.1  1996/09/22 16:46:01  rhino
 *  General reorganization of the directories/repository, fusion with the
 *  RWI software.
 *
 *  Revision 1.2  1994/10/22 18:47:38  tyson
 *  VMS version. Fixed structure indexing (computation of the offsets
 *  in a struct). Added signal handlers to a1, b1 tcxServer.
 *
 * Revision 1.1.1.1  1994/03/31  08:35:05  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:30  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:28  rhino
 * test
 *
 * Revision 1.29  1993/03/12  20:58:58  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.28  1993/03/03  20:18:42  fedor
 * Added tcxNRecvMsgE start of new message interface.
 * Added tcxRegisterCloseHnd - generic all module close.
 * Added tcxCurrentModuleName - give module name - change to pass NULL to
 * tcxModuleName?
 *
 * Revision 1.27  1993/03/02  19:46:34  fedor
 * Updates for pbk to convert eddie code. Added connect/disconnect.
 *
 * Revision 1.26  1992/11/30  02:33:39  fedor
 * Flaged items for deletion inside findItem for handler run items.
 * This avoids a list iteration problem for recusively deleting items from a list.
 * Also moved handler masks for tcxRecvData to tcx.h from tcxP.h
 *
 * Revision 1.25  1992/11/13  14:47:20  fedor
 * Update archive with a working version and partially working server route
 *
 * Revision 1.24  1992/10/27  03:42:36  fedor
 * Added tcxCloseAll (VxWorks) and changed tcxInitialized to be called afterwards.
 *
 * Revision 1.23  1992/10/26  22:59:13  fedor
 * Added tcxSetAutoReconnect and tcxTestActiveModule
 *
 * Revision 1.22  1992/10/23  20:15:30  fedor
 * Redid method of connecting to modules. Added the notion of a connection.
 * Added some auto-reconnect for those modules that are probably listening.
 * See detail notes.
 *
 * Revision 1.21  1992/10/16  18:30:52  fedor
 * Changed messageDefinition hashTableFind to look up name and not &name.
 *
 * Revision 1.20  1992/10/14  18:31:02  fedor
 * Changed tcxRecvData to execute at least a single handler from one connection or
 * at most one handler from each connection and return.
 * Changed tcxRecvLoop to loop here instead of in tcxRecvData - needs clean up.
 *
 * Revision 1.19  1992/10/11  18:29:54  fedor
 * Added tcxRecvMsgNoHnd as a quick fix so that connect handler completes
 * before messages get sent to user handlers.
 *
 * Revision 1.18  1992/10/10  03:04:37  fedor
 * Moving toward basing initializing connections on moduleInfo messages.
 *
 * Revision 1.17  1992/10/09  15:37:34  fedor
 * Added som test code for TCX_REF_PTR to test if freed.
 *
 * Revision 1.16  1992/10/08  14:38:10  fedor
 * Remove oops error by adding connection confirmation.
 * Establish all pending connections in tcxInitialize by waiting for a
 * confirmation of connection before sending version information.
 *
 * Revision 1.15  1992/09/14  21:24:36  fedor
 * Changed message hash function to ignore case.
 *
 * Revision 1.14  1992/09/12  21:53:11  fedor
 * Connect when first message received. Moving towards version 5.x
 *
 * Revision 1.13  1992/08/08  00:28:21  jwest
 * Separated out stuff that only tcx server needs from library.  Lots more
 * work to be done.
 *
 * Revision 1.12  1992/08/06  09:39:13  fedor
 * Prevent multiple calls to register messages.
 *
 * Revision 1.11  1992/08/04  14:45:54  fedor
 * Debugging fprintf for VxWorks.
 * Added checks for using NULL formatters.
 *
 * Revision 1.10  1992/07/27  07:24:00  fedor
 * Forwarding of messages on tcxSend(NULL - does not handle tcxReply
 *
 * Revision 1.9  1992/07/27  04:02:36  fedor
 * Added registration of handlers to server. Started message forwarding.
 *
 * Revision 1.8  1992/07/23  15:26:32  fedor
 * Changed tcxQuery and tcxReply to take a message for the reply format
 *
 * Revision 1.7  1992/07/20  00:31:54  fedor
 * Added receive message style calls. Added data freeing calls.
 *
 * Revision 1.6  1992/07/08  15:20:26  fedor
 * Changed message registration to use server generated id values.
 * Added a message handler registration - name may change.
 * Now there are two kinds of handlers one for messages of the form (ref,
 * data, cdata) and the many parameter one (mod, id, ref, data, cdata)
 *
 * Revision 1.5  1992/07/05  15:41:48  fedor
 * Added random makefile.NeXT for NeXT machine use.
 * Changed incorrect use of perror to fprintf(stderr,
 *
 * Revision 1.4  1992/07/05  14:06:47  fedor
 * Added code to call moduleClose on a write fail.
 *
 * Revision 1.3  1992/07/05  13:06:58  fedor
 * Changed printf to fprintf(stderr,
 * Removed old list. Some file reorganization.
 *
 * Revision 1.2  1992/07/03  10:13:26  fedor
 * New module definition and routines
 *
 * Revision 1.1.1.1  1992/06/16  17:22:02  fedor
 * Import TCX 2.0 16-Jun-92
 *
 * Revision 1.4  1992/04/20  10:37:56  fedor
 * *** empty log message ***
 *
 * Revision 1.3  1992/04/20  10:30:26  fedor
 * test file
 *
 * Revision 1.3  1992/04/20  10:30:26  fedor
 * test file
 *
 * Revision 1.2  1992/03/30  07:39:30  fedor
 * new data.c and msg.c
 *
 * Revision 1.1  1992/03/26  06:44:20  fedor
 * Initial revision
 *
 *  
 *  $Revision: 1.6 $
 *  $Date: 1997/04/11 18:56:57 $
 *  $Author: tyson $
 *
 *********************************************************************/

#ifdef VMS
#include "vms.h"
#endif

#ifdef VXWORKS
#include "/usr/vxworks/vx5.0.2b/h/vxWorks.h"
#include "/usr/vxworks/vx5.0.2b/h/stdioLib.h"
#include "/usr/vxworks/vx5.0.2b/h/ctype.h"
#else
#include "stdio.h" 
#include "ctype.h"
#include <string.h>
#endif

#include "formatters.h"
#include "tcx.h"
#include "tcxP.h"
#include "list.h"
#include "hash.h"
#include "msg.h"

#include "global.h"

extern void *malloc();
extern void moduleClose();

#if 0
#define pgmTrace() fprintf(stderr, "%s:%6d:%s()\n", \
			   __FILE__, __LINE__, __FUNCTION__)

#define pgmReturn() fprintf(stderr, "%s:%6d:%s() return\n", \
			   __FILE__, __LINE__, __FUNCTION__)
#else
#define pgmTrace() {}
#define pgmReturn() {}
#endif

/*****************************************************************************/

int nameHashFunc(char *str)
{
  int i, sum;

  for(i=0, sum=0;str[i] != '\0';i++) {
    if (isupper(str[i])) {
      sum += (int)(tolower(str[i]));
    } else {
      sum += (int)str[i];
    }
  }

  return sum;
}

/******************************************************************************
*
* FUNCTION: int nameEqFunc(a, b)
*
* DESCRIPTION: 
* Return 1 (TRUE) if strings a and b are equal. Returns 0 (FALSE) otherwise.
* Case insensitive compare.
*
*
******************************************************************************/

int nameEqFunc(strA, strB)
char *strA, *strB;
{
  int i;
  char a1, b1;

  i = 0;
  while (strA[i] != '\0' && strB[i] != '\0') {
    if (isupper(strA[i]))
      a1 = tolower(strA[i]);
    else 
      a1 = strA[i];
    if (isupper(strB[i]))
      b1 = tolower(strB[i]);
    else 
      b1 = strB[i];
    
    if (a1 != b1)
      return FALSE;

    i++;
  }

  if (strA[i] == strB[i])
    return TRUE;
  else
    return FALSE;
}

/***************************************************************************/

int idHashFunc(id)
int *id;
{
  return *id;
}

int idEqFunc(a, b)
int *a, *b;
{
  return(*a == *b);
}

/***************************************************************************/

void messageInitialize()
{
  pgmTrace();

#ifndef VXWORKS  
  if (Global->msgInitFlagGlobal) {
    fprintf(stderr, "ERROR: messageInitialize: Called more than once!\n");
    return;
  }
#endif

  Global->msgQGlobal = listCreate();
  Global->msgFreeRefS = listCreate();

  Global->msgIdTableGlobal = hashTableCreate(10, idHashFunc, idEqFunc);
  Global->msgNameTableGlobal = hashTableCreate(10, nameHashFunc, nameEqFunc);

  Global->msgInitFlagGlobal = 1;
  Global->msgRefGlobal = 1;
}

/******************************************************/

TCX_REF_PTR messageRefCreate(module, id, ref)
MODULE_PTR module;
int id, ref;
{
  TCX_REF_PTR refPtr;

  if (!listLength(Global->msgFreeRefS)) {
    refPtr = (TCX_REF_TYPE *)malloc(sizeof(TCX_REF_TYPE));
  } else {
    refPtr = (TCX_REF_PTR)listPopItem(Global->msgFreeRefS);
  }

  refPtr->module = module;
  refPtr->id = id;
  refPtr->ref = ref;
  refPtr->free = 0;

  return refPtr;
}

/******************************************************/

void messageRefFree(ref)
TCX_REF_PTR ref;
{
  pgmTrace();

  if (ref) {
    if (ref->free) {
      fprintf(stderr, "WARNING: messageRefFree: ref already freed!\n");
    }

    ref->free = 1;
    listInsertItem(ref, Global->msgFreeRefS);
  }

  pgmReturn();
  return;
}

/******************************************************/

TCX_REF_PTR tcxRefCopy(TCX_REF_PTR refPtr)
{
  TCX_REF_PTR copy;

  pgmTrace();

  copy = NULL;
  
  if (refPtr) {
    if (refPtr->free) {
      fprintf(stderr, "ERROR: tcxRefCopy: ref is marked freed!\n");
    }
    copy = messageRefCreate(refPtr->module, refPtr->id, refPtr->ref);
  }

  return copy;
}

/******************************************************/

void tcxRefFree(TCX_REF_PTR ref)
{
  pgmTrace();
  messageRefFree(ref);
  pgmReturn();
}

/******************************************************/

void messageDefinition(int id, char *msgName, char *msgForm)
{
  MSG_DEF_PTR msg;

  pgmTrace();

  msg = (MSG_DEF_PTR)hashTableFind(msgName, Global->msgNameTableGlobal);

  if (!msg) {
    msg = (MSG_DEF_TYPE *)malloc(sizeof(MSG_DEF_TYPE));
    msg->id = id;
    msg->name = msgName; 
    msg->msgForm = msgForm;
    msg->msgFormat = tcxParseFormat(msgForm);

    msg->hndList = NULL;

    hashTableInsert(&id, sizeof(int), msg, Global->msgIdTableGlobal);
    hashTableInsert(msgName, strlen(msgName)+1, msg, Global->msgNameTableGlobal);

  } else {
    /* update values */
    hashTableRemove(&msg->id, Global->msgIdTableGlobal);

    msg->id = id;
    msg->name = msgName; 
    msg->msgForm = msgForm;

    /* 7-Jul-92: fedor: should free old memory for parse info */
    msg->msgFormat = tcxParseFormat(msgForm);

    hashTableInsert(&id, sizeof(int), msg, Global->msgIdTableGlobal);
  }
}

/******************************************************/

MSG_DEF_PTR messageFindByName(char *name)
{

  if (!name) {
    return NULL;
  }

  return((MSG_DEF_PTR)hashTableFind(name, Global->msgNameTableGlobal));
}

/******************************************************/

MSG_DEF_PTR messageFindById(int id)
{
  pgmTrace();

  return((MSG_DEF_PTR)hashTableFind(&id, Global->msgIdTableGlobal));
}

/******************************************************/

MSG_INS_PTR messageCreateInstance(TCX_MODULE_PTR module, int id, int len, 
				  void *encodedData)
{
  MSG_INS_PTR msgInstance;

  pgmTrace();

  msgInstance = (MSG_INS_TYPE *)malloc(sizeof(MSG_INS_TYPE));

  msgInstance->msg = NULL;
  msgInstance->id = id;
  msgInstance->len = len;
  msgInstance->module = module;
  msgInstance->encodedData = encodedData;

  return(msgInstance);
}

/******************************************************/

MSG_INS_PTR messageCreateInsFromMsg(TCX_MODULE_PTR module, 
				    char *name, void *data, int ref)
{
  MSG_DEF_PTR msg;
  MSG_INS_PTR item;

  pgmTrace();

  msg = (MSG_DEF_PTR)messageFindByName(name);

  if (!msg) {
    fprintf(stderr, "ERROR: tcxSendMsg: No Registered Message: %s\n", name);
    return NULL;
  }
  
  if (!ref) {
    if (!Global->msgRefGlobal) {
      Global->msgRefGlobal++;
    }
    ref = Global->msgRefGlobal;
    Global->msgRefGlobal++;
  }

  item = (MSG_INS_TYPE *)malloc(sizeof(MSG_INS_TYPE));
  item->msg = msg;
  item->id = msg->id;
  item->module = module;
  item->len = bufferSize(msg->msgFormat, data);
  item->encodedData = (char *)malloc(item->len);
  encodeData(msg->msgFormat, data, item->encodedData, 0);

  return item;
}

/******************************************************/

void messageInsertQueue(MSG_INS_PTR msgInstance)
{
  pgmTrace();

  listInsertItemLast(msgInstance, Global->msgQGlobal);
}

/******************************************************/

void messageAddNewInstance(TCX_MODULE_PTR module, int id, int len, 
			   void *encodedData)
{
  MSG_INS_PTR msgInstance;

  pgmTrace();

  msgInstance = messageCreateInstance(module, id, len, encodedData);
  messageInsertQueue(msgInstance);
}

/******************************************************/

int tcxTestActiveModule(TCX_MODULE_PTR module)
{
  pgmTrace();

  if (module && (module->status & STATUS_ACTIVE)) {
    return 1;
  } else {
    return 0;
  }
    
}

/******************************************************/

int tcxSendMsg(TCX_MODULE_PTR module, char *msgName, void *data)
{
  int ref;
  MSG_DEF_PTR msg;

  ref = -1;

  if (!module) {
    ref = tcxSendDoubleMsg(Global->tcxServerGlobal, "TCXforw", NULL, msgName, data);
    return ref;
  }

  if (module->status & STATUS_INACTIVE) {
    fprintf(stderr, "ERROR: tcxSendMsg: Module %s Not Connected.\n",
	    module->moduleInfo->name);
    return(-1);
  }

  if ((module->moduleInfo->status & STATUS_SERVER_ROUTE) && 
      Global->tcxModuleGlobal != Global->tcxServerGlobal) {
   
    ref = tcxSendDoubleMsg(Global->tcxServerGlobal, "TCXserverRoute", 
			   module->moduleInfo, msgName, data);

    return ref;
  }

  msg = (MSG_DEF_PTR)messageFindByName(msgName);

  if (msg) {
    if (!Global->msgRefGlobal) {
      Global->msgRefGlobal++;
    }
    ref = Global->msgRefGlobal;
    Global->msgRefGlobal++;
    tcxSendData(module, msg->id, ref, data, 0, msg->msgFormat);
  } else {
    fprintf(stderr, "ERROR: tcxSendMsg: No Registered Message: %s\n", msgName);
  }

  return ref;
}

/******************************************************/

int tcxSendDoubleMsgRef(TCX_MODULE_PTR module, int refSent, 
			char *name1, void *data1,
			char *name2, void *data2)
{
  int ref;
  int total;
  char *sbuf;
  HDR_TYPE hdr;
  MSG_DEF_PTR msg1, msg2;

  pgmTrace();

  ref = -1;

  if (!module) {
    fprintf(stderr, "ERROR: tcxSendDoubleMsg: Module is NULL\n");
    return(-1);
  }

  if (module->status & STATUS_INACTIVE) {
    fprintf(stderr, "ERROR: tcxSendDoubleMsg: Module %s Not Connected.\n",
	    module->moduleInfo->name);
    return(-1);
  }

  /***
    if (!module) {
    fprintf(stderr, "ERROR: tcxSendDoubleMsg: Module is NULL\n");
    (*(Global->tcxExitHndGlobal))();
    }
    ***/

  if (!name1) {
    if (name2) {
      /* no first message - let first message be second message */
      name1 = name2;
      data1 = data2;
      
      name2 = NULL;
      data2 = NULL;
    } else {
      /* no messages - return(-1) */
      return(-1);
    }
  }

  msg1 = NULL;
  msg2 = NULL;
  
  hdr.msg1 = 0;
  hdr.msg2 = 0;
  hdr.size1 = 0;
  hdr.size2 = 0;

  if (name1) {
    msg1 = messageFindByName(name1);
    if (!msg1) {
      fprintf(stderr, "ERROR: tcxSendDoubleMsg: No Registered Message: %s.\n",
	      name1);
      return(-1);
    }
    hdr.msg1 = msg1->id;
  }

  if (name2) {
    msg2 = messageFindByName(name2);
    if (!msg2) {
      fprintf(stderr, "ERROR: tcxSendDoubleMsg: No Registered Message: %s.\n",
	      name2);
      return(-1);
    }
    hdr.msg2 = msg2->id;
  }

  if (msg1 && data1 && msg1->msgFormat) {
    hdr.size1 = bufferSize(msg1->msgFormat, data1);
  }

  if (msg2 && data2 && msg2->msgFormat) {
    hdr.size2 = bufferSize(msg2->msgFormat, data2);
  }

  total = hdr.size1+hdr.size2+sizeof(HDR_TYPE);

  if (Global->dataBufGlobal.len < total) {
    free(Global->dataBufGlobal.buf);
    sbuf = (char *)malloc(total);
    Global->dataBufGlobal.len = total;
    Global->dataBufGlobal.buf = sbuf;
  } else {
    sbuf = Global->dataBufGlobal.buf;
  }

  if (!refSent) {
    if (!Global->msgRefGlobal) {
      Global->msgRefGlobal++;
    }
  
    hdr.ref1 = Global->msgRefGlobal;
    hdr.ref2 = Global->msgRefGlobal;

    ref = Global->msgRefGlobal;
    Global->msgRefGlobal++;
  } else {
    hdr.ref1 = refSent;
    hdr.ref2 = refSent;
  }

  bcopy(&hdr, sbuf, sizeof(HDR_TYPE));
  sbuf += sizeof(HDR_TYPE);

  if (msg1 && msg1->msgFormat) {
    encodeData(msg1->msgFormat, data1, sbuf, 0);
  }

  sbuf += hdr.size1;

  if (msg2 && msg2->msgFormat) {
    encodeData(msg2->msgFormat, data2, sbuf, 0);
  }



  if (writeAll(module->sd, Global->dataBufGlobal.buf, total) < 0) {
    fprintf(stderr, "%s:%6d:%s() 1 - ERROR: writeAll()=%d\n",
	    __FILE__, __LINE__, __FUNCTION__, errno);
    perror("");
    if (Global->closeModGlobal || errno == EPIPE) {
      moduleClose(module);
      Global->closeModGlobal = 0;
    } else {
      (*(Global->tcxExitHndGlobal))();
    }
  }

  return ref;
}

/******************************************************/

int tcxSendDoubleMsg(TCX_MODULE_PTR module,
			char *name1, void *data1,
			char *name2, void *data2)
{
  pgmTrace();

  return(tcxSendDoubleMsgRef(module, 0, name1, data1, name2, data2));
}

/******************************************************/

void tcxSendDoubleIns(TCX_MODULE_PTR module, MSG_INS_PTR ins1, 
		      MSG_INS_PTR ins2)
{
  int ref;
  int total;
  char *sbuf;
  HDR_TYPE hdr;
  MSG_DEF_PTR msg1, msg2;


  pgmTrace();

  if (!module) {
    fprintf(stderr, "ERROR: tcxSendDoubleIns: Module is NULL\n");
    return;
  }

  if (module->status & STATUS_INACTIVE) {
    fprintf(stderr, "ERROR: tcxSendDoubleIns: Module %s Not Connected.\n",
	    module->moduleInfo->name);
    return;
  }

  hdr.msg1 = ins1->id;
  hdr.msg2 = ins2->id;
  hdr.size1 = ins1->len;
  hdr.size2 = ins2->len;

  total = hdr.size1+hdr.size2+sizeof(HDR_TYPE);

  if (Global->dataBufGlobal.len < total) {
    free(Global->dataBufGlobal.buf);
    sbuf = (char *)malloc(total);
    Global->dataBufGlobal.len = total;
    Global->dataBufGlobal.buf = sbuf;
  } else {
    sbuf = Global->dataBufGlobal.buf;
  }

  hdr.ref1 = ins1->ref;
  hdr.ref2 = ins2->ref;

  bcopy(&hdr, sbuf, sizeof(HDR_TYPE));
  sbuf += sizeof(HDR_TYPE);

  bcopy(ins1->encodedData, sbuf, ins1->len);
  sbuf += ins1->len;

  bcopy(ins2->encodedData, sbuf, ins2->len);
  sbuf += ins2->len;

  if (writeAll(module->sd, Global->dataBufGlobal.buf, total) < 0) {
    fprintf(stderr, "%s:%6d:%s() 1 - ERROR: writeAll()=%d\n",
	    __FILE__, __LINE__, __FUNCTION__, errno);
    perror("");
    if (Global->closeModGlobal || errno == EPIPE) {
      moduleClose(module);
      Global->closeModGlobal = 0;
    } else {
      (*(Global->tcxExitHndGlobal))();
    }
  }
}

/******************************************************/

/******************************************************/
/*****************
void tcxReplyData(TCX_MODULE_PTR module, int id, int ref, void *reply)
{
  MSG_DEF_PTR msg;

  pgmTrace();

  msg = messageFindById(id);

  if (msg) {
    tcxSendData(module, msg->id, ref, reply, 0, msg->resFormat);
  } else {
    fprintf(stderr, "ERROR: tcxReply: No Registered Message Id: %d\n", id);
    return;
  }
}
********/
/******************************************************/

void tcxReply(TCX_REF_PTR ref, char *replyMsgName,  void *reply)
{
  MSG_DEF_PTR msg;

  pgmTrace();

  msg = (MSG_DEF_PTR)messageFindByName(replyMsgName);

  if ((ref->module->moduleInfo->status & STATUS_SERVER_ROUTE) && 
      Global->tcxModuleGlobal != Global->tcxServerGlobal) {

    tcxSendDoubleMsgRef(Global->tcxServerGlobal, ref->ref,
			"TCXserverRoute", ref->module->moduleInfo,
			replyMsgName, reply);
  } else {
    if (ref && msg) {
      tcxSendData(ref->module, msg->id, ref->ref, reply, 0, msg->msgFormat);
      /** tcxReplyData(ref->module, msg->id, ref->ref, reply);**/
    }
  }
}

/******************************************************/

char *tcxMessageName(int id)
{
  MSG_DEF_PTR msg;
  
  pgmTrace();

  msg = messageFindById(id);

  if (!msg)
    return NULL;
  else
    return msg->name;
}

/******************************************************/

int tcxMessageId(char *name)
{
  MSG_DEF_PTR msg;

  pgmTrace();

  msg = messageFindByName(name);
  if (msg) {
    return(msg->id);
  }

  return 0;
}
    

/******************************************************/

tcxRegMsgSend(TCX_REG2_MSG_TYPE *msgArray, int size)
{
    int i;
    
    MSG_REG2_TYPE m;
    MSG_REG2_REPLY_TYPE mr;

    pgmTrace();

    printf("tcxRegMsgSend: start\n");

    m.num = size;
    m.msgs = msgArray;

    tcxQuery(Global->tcxServerGlobal, "TCXRegMsgSend", &m, "TCXmsgReg2Reply", &mr);

    for(i=0;i < mr.num; i++) {
	printf("%d\n", mr.ids[i]);
    }

    printf("tcxRegMsgSend: done\n");
}

/******************************************************/

void tcxRegisterMessages(TCX_REG_MSG_TYPE *msgArray, int size)
{
  int i;

  MSG_REG_TYPE m;
  MSG_REG_REPLY_TYPE mr;

  MSG_DEF_PTR msg;

  pgmTrace();

#ifndef VXWORKS
  if (Global->tcxRegMsgFlagG) {
   fprintf(stderr, "WARNING: Multiple call to tcxRegisterMessages ignored!\n");
    return;
  }

  Global->tcxRegMsgFlagG = 1;
#endif  

  m.num = size;
  m.msgs = msgArray;

  tcxQuery(Global->tcxServerGlobal, "TCXmsgReg", &m, "TCXmsgRegReply", &mr);

  for(i=0; i < mr.num; i++) {
    /* printf("%d\n", mr.ids[i]);*/
    messageDefinition(mr.ids[i], msgArray[i].msgName, msgArray[i].msgFormat);
  }
}

/***************************************************************************/

TCX_MODULE_PTR tcxRefModule(ref)
TCX_REF_PTR ref;
{
  pgmTrace();

  if (ref)
    return(ref->module);
  else 
    return NULL;
}

int tcxRefId(ref)
TCX_REF_PTR ref;
{
  pgmTrace();

  if (ref)
    return(ref->id);
  else
    return 0;
}

int tcxRefIns(ref)
TCX_REF_PTR ref;
{
  pgmTrace();

  if (ref)
    return(ref->ref);
  else
    return 0;
}

/***************************************************************************/

/* 11-Oct-92: fedor: may need to make a distinction between user handlers
   and internal handlers */

int tcxRecvMsgNoHnd(char *msgName, int *ref, void *data, void *timeout)
{
  int ret = 0;
  MSG_DEF_PTR msg;
  int searchRef = 0;

  pgmTrace();

  msg = messageFindByName(msgName);

  if (msg) {
    ret = tcxRecvData(NULL, &msg->id, ref, data, 
		      msg->msgFormat, TCX_HND, timeout);
  }

  return ret;
}

/***************************************************************************/

int tcxRecvMsg(char *msgName, int *ref, void *data, void *timeout)
{
  int ret = 0;
  MSG_DEF_PTR msg;
  int searchRef = 0;

  pgmTrace();

  msg = messageFindByName(msgName);

  if (msg) {
    ret = tcxRecvData(NULL, &msg->id, ref, data, 
		      msg->msgFormat, ALL_HND, timeout);
  }

  return ret;
}

/***************************************************************************/

int tcxNRecvMsgE(TCX_MODULE_PTR *module, char **msgName, void *data,
		 void *timeout, int *inst, int hndMask)
{
  char *buf, *r2;

  int id, ret;
  MSG_DEF_PTR msg;

  pgmTrace();

  ret = 0;

  if (*msgName) {
    msg = messageFindByName(*msgName);

    if (msg) {
      ret = tcxRecvData(module, &msg->id, inst, data,
			msg->msgFormat, hndMask, timeout);
    }
    
    return ret;
  }
    
  id = 0;
  buf = NULL;
  ret = tcxRecvData(module, &id, inst, &buf, NULL, hndMask, timeout);
  
  if (ret < 0) {
    return ret;
  }

  msg = messageFindById(id);

  if (!msg) {
    /* blah! data may be using memory and should be freed! */
    *msgName = NULL;
    
    return ret;
  }

  if (msg->msgFormat) {
    if (buf) {
      r2 = (char *)decodeData(msg->msgFormat, buf, 0);
      bcopy(&r2, data, sizeof(char *));
    }
  } else {
    data = NULL;
  }

  *msgName = msg->name;

  return ret;
}

/***************************************************************************/

/*** int tcxNRecvMsg(TCX_MODULE_PTR *module***/


/***************************************************************************/

void tcxQuery(TCX_MODULE_PTR module, char *msgName, void *data,
	      char *replyMsgName, void *reply)
{
  int ins;
  MSG_DEF_PTR msg, replyMsg;

  pgmTrace();

  msg = (MSG_DEF_PTR)messageFindByName(msgName);
  replyMsg = (MSG_DEF_PTR)messageFindByName(replyMsgName);

  if (!msg) {
    fprintf(stderr, "ERROR: tcxQuery: No Registered Message: %s\n", msgName);
    return;
  }

  if (!replyMsg) {
    fprintf(stderr, "ERROR: tcxQuery: No Registered Message: %s\n", 
	    replyMsgName);
    return;
  }

#if 0  
  if (!Global->msgRefGlobal) {
    Global->msgRefGlobal++;
  }

  ins = Global->msgRefGlobal;
  Global->msgRefGlobal++;
  
  /* fprintf(stderr, "id: %d: ins: %d\n", msg->id, ins);*/

  tcxSendData(module, msg->id, ins, data, 0, msg->msgFormat);

  tcxRecvData(&module, &replyMsg->id, &ins, reply, 
	      replyMsg->msgFormat, ALL_HND, NULL);

#endif

  ins = tcxSendMsg(module, msgName, data);
  tcxRecvMsg(replyMsgName, &ins, reply, NULL);
}

/***************************************************************************/

int tcxRecvMsgE(TCX_MODULE_PTR *module, char *msgName, int *ref, 
		       void *data, int allowHnd, void *timeout)
{
  int ret = 0;
  MSG_DEF_PTR msg;

  pgmTrace();

  msg = messageFindByName(msgName);

  if (msg) {
    ret = tcxRecvData(module, &msg->id, ref, data, 
		      msg->msgFormat, allowHnd, timeout);
  }

  return ret;
}

/***************************************************************************/
/*****************
int tcxRecvResp(char *msgName, int *ref, void *data, void *timeout)
{
  int ret = 0;
  MSG_DEF_PTR msg;
 
  pgmTrace();

  msg = messageFindByName(msgName);

  if (msg) {
     ret = tcxRecvData(NULL, &msg->id, ref, data, 
		      msg->resFormat, ALL_HND, timeout);
  }

  return ret;
}
********/
/***************************************************************************/
/*******************
int tcxRecvRespE(TCX_MODULE_PTR *module, char *msgName, int *ref, 
		       void *data, int allowHnd, void *timeout)
{
  int ret = 0;
  MSG_DEF_PTR msg;

  pgmTrace();

  msg = messageFindByName(msgName);

  if (msg) {
    ret = tcxRecvData(module, &msg->id, ref, data, 
		      msg->resFormat, allowHnd, timeout);
  }

  return ret;
}
**************/
