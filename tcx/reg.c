
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
 *  MODULE: Registration
 *
 *  FILE: reg.c
 *
 *  ABSTRACT: Register Messages
 *
 *  EXPORTS:
 *
 *  HISTORY:
 *  $Log: reg.c,v $
 *  Revision 1.3  1997/04/11 18:56:57  tyson
 *  minor fixes and chasing TCX segv
 *
 *  Revision 1.2  1997/03/11 17:16:48  tyson
 *  added IR simulation and other work
 *
 *  Revision 1.1.1.1  1996/09/22 16:46:01  rhino
 *  General reorganization of the directories/repository, fusion with the
 *  RWI software.
 *
 *  Revision 1.2  1994/10/22 18:47:43  tyson
 *  VMS version. Fixed structure indexing (computation of the offsets
 *  in a struct). Added signal handlers to a1, b1 tcxServer.
 *
 * Revision 1.1.1.1  1994/03/31  08:35:05  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:30  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:29  rhino
 * test
 *
 * Revision 1.11  1993/03/12  20:59:08  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.10  1992/10/23  20:15:33  fedor
 * Redid method of connecting to modules. Added the notion of a connection.
 * Added some auto-reconnect for those modules that are probably listening.
 * See detail notes.
 *
 * Revision 1.9  1992/08/08  00:28:23  jwest
 * Separated out stuff that only tcx server needs from library.  Lots more
 * work to be done.
 *
 * Revision 1.8  1992/07/27  07:24:03  fedor
 * Forwarding of messages on tcxSend(NULL - does not handle tcxReply
 *
 * Revision 1.7  1992/07/27  04:02:40  fedor
 * Added registration of handlers to server. Started message forwarding.
 *
 * Revision 1.6  1992/07/20  00:32:03  fedor
 * Added receive message style calls. Added data freeing calls.
 *
 * Revision 1.5  1992/07/08  15:20:33  fedor
 * Changed message registration to use server generated id values.
 * Added a message handler registration - name may change.
 * Now there are two kinds of handlers one for messages of the form (ref,
 * data, cdata) and the many parameter one (mod, id, ref, data, cdata)
 *
 * Revision 1.4  1992/07/05  15:41:50  fedor
 * Added random makefile.NeXT for NeXT machine use.
 * Changed incorrect use of perror to fprintf(stderr,
 *
 * Revision 1.3  1992/07/05  13:07:07  fedor
 * Changed printf to fprintf(stderr,
 * Removed old list. Some file reorganization.
 *
 * Revision 1.2  1992/07/03  10:13:32  fedor
 * New module definition and routines
 *
 * Revision 1.1.1.1  1992/06/16  17:22:08  fedor
 * Import TCX 2.0 16-Jun-92
 *
 *  
 *  $Revision: 1.3 $
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
#else
#include "stdio.h" 
#endif

#include "tcx.h"
#include "tcxP.h"

#include "list.h"
#include "msg.h"

#include "hash.h"

#include "global.h"

#if 0
#define pgmTrace() fprintf(stderr, "%s:%6d:%s()\n", \
			   __FILE__, __LINE__, __FUNCTION__)

#define pgmReturn() fprintf(stderr, "%s:%6d:%s() return\n", \
			   __FILE__, __LINE__, __FUNCTION__)
#else
#define pgmTrace() {}
#define pgmReturn() {}
#endif

/**********************************************************************
 *
 *  FUNCTION:  void tcxRegisterExitHnd(void (*proc)())
 *
 *  DESCRIPTION: Provide a procedure other than exit to major errors.
 *
 *
 *********************************************************************/

void tcxRegisterExitHnd(void (*proc)())
{
  pgmTrace();

  Global->tcxExitHndGlobal = proc;
}


/**********************************************************************
 *
 *  FUNCTION:  void tcxRegisterDHnd(int id, TCX_FMT_PTR fmt, 
 *                           int (*proc)(), int control, int mask,
 *                           void *callData)
 *
 *  DESCRIPTION: 
 * Associate a procedure with an id value, format and hndData.
 * The control value effects future calls to the registered procedure.
 * RECV_ALL  - future calls to tcxRecvData will allow registered procedures.
 * RECV_NONE - future calls to tcxRecvData will not allow registered procedures
 * RECV_ONCE - All procedures except the current one will be allowed to
 *             execute from future calls to tcxRecvData.
 *
 *********************************************************************/

void tcxRegisterDHnd(int id, TCX_FMT_PTR fmt, void (*proc)(), 
		    int control, int mask, void *callData)
{
  int test;
  HND_PTR hnd;

  pgmTrace();

  if (!proc) {
    fprintf(stderr, "ERROR: tcxRegisterDHnd: No procedure!\n");
    (*(Global->tcxExitHndGlobal))();
  }
  
  if (!(control == TCX_RECV_ALL || 
	control == TCX_RECV_NONE || 
	control == TCX_RECV_ONCE)){
    fprintf(stderr, "ERROR: tcxRegisterDHnd: Invalid control: %d\n", control);
    (*(Global->tcxExitHndGlobal))();    
  }

  if (id == 0) {
    fprintf(stderr, "ERROR: tcxRegisterDHnd: Id value can not be 0.\n");
    (*(Global->tcxExitHndGlobal))();
  }

  hnd = (HND_PTR)hashTableFind(&id, Global->hndIdTable);

  if (!hnd)
    hnd = (HND_TYPE *)malloc(sizeof(HND_TYPE));

  hnd->proc = proc;
  hnd->control = control;
  hnd->status = HND_EXEC;
  hnd->id = id;
  hnd->mask = 0;
  hnd->fmt = fmt;
  hnd->callData = callData;
  hnd->type = DataHnd;

  hashTableInsert(&id, sizeof(int), hnd, Global->hndIdTable);
}

/***************************/

void tcxRegisterMHnd(char *msgName, void (*proc)(), int control, int mask,
		     void *callData)
{
  int test;
  HND_PTR hnd;
  MSG_DEF_PTR msg;

  pgmTrace();

  if (!proc) {
    fprintf(stderr, "ERROR: tcxRegisterMHnd: No procedure!\n");
    (*(Global->tcxExitHndGlobal))();
  }
  
  if (!(control == TCX_RECV_ALL || 
	control == TCX_RECV_NONE || 
	control == TCX_RECV_ONCE)){
    fprintf(stderr, "ERROR: tcxRegisterMHnd: Invalid control: %d\n", control);
    (*(Global->tcxExitHndGlobal))();
  }

  msg = (MSG_DEF_PTR)messageFindByName(msgName);
  
  if (!msg) {
    fprintf(stderr, "ERROR: tcxRegisterMHnd: Missing message def: %s\n",
	    msgName);
    (*(Global->tcxExitHndGlobal))();
  }

  hnd = (HND_PTR)hashTableFind(&msg->id, Global->hndIdTable);

  if (!hnd) {
    hnd = (HND_TYPE *)malloc(sizeof(HND_TYPE));
    hashTableInsert(&msg->id, sizeof(int), hnd, Global->hndIdTable);
  }

  hnd->proc = proc;
  hnd->control = control;
  hnd->status = HND_EXEC;
  hnd->id = msg->id;
  hnd->fmt = msg->msgFormat;
  hnd->callData = callData;
  hnd->type = MsgHnd;
  hnd->mask = mask;
  hnd->module = NULL;
}

/***********************************/


void tcxRegisterHandlers(TCX_REG_HND_TYPE *hndArray, int size)
{
  int i;
  HND_REG_TYPE hnds;
  
  pgmTrace();

  for(i=0;i < size;i++) {
    tcxRegisterMHnd(hndArray[i].msgName, hndArray[i].hndProc, 
		    hndArray[i].hndControl, USER_HND, 
		    hndArray[i].hndData);
  }

  hnds.num = size;
  hnds.hnds = hndArray;
  tcxSendMsg(Global->tcxServerGlobal, "TCXhndReg", &hnds);
}
