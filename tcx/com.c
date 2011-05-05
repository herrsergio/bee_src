
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



/******************************************************************************
*
* PROJECT: TCX
*
* PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
* (c) Copyright 1993 Christopher Fedor. All rights reserved.
*
* MODULE: communications
*
* FILE: com.c
*
* ABSTRACT:
* 
* Communications
*
* REVISION HISTORY
*
* $Log: com.c,v $
* Revision 1.8  1997/04/11 18:56:56  tyson
* minor fixes and chasing TCX segv
*
* Revision 1.7  1997/04/01 22:30:35  tyson
* bugs and stuff
*
* Revision 1.6  1997/02/22 00:59:15  thrun
* Introduced version number support
*
* Revision 1.5  1997/02/21 22:40:20  thrun
* Changed tcxInitialize so that it checks if a module with the same name
* is already up and running. If this is the case, it prints out an error
* message and exists.
* This way we should now have an effective mechanisms that prevents from
* running multiple copies of the same module - without changing a line
* of code outside tcx.
* I also fixed a bug introduced by bUtils, that made tcxServer crash on
* my SUN.
*
* Revision 1.4  1997/01/28 20:39:38  tyson
* daemonize COLLI, other tweaks
*
* Revision 1.3  1996/11/14 12:59:23  ws
* ridiculous 32 byte limitation for length of hostname replaced by 256 bytes
*
* Revision 1.2  1996/10/28 15:34:46  ws
* Now TCX clients and server can connect even when located in different
* internet domains.
*
* Revision 1.1.1.1  1996/09/22 16:46:00  rhino
* General reorganization of the directories/repository, fusion with the
* RWI software.
*
* Revision 1.4  1996/08/22 21:13:34  fox
* Added a define for SA_RESTART.
*
* Revision 1.3  1996/05/16 20:14:14  tyson
* Added support for RAI watchdog - TDS
*
 * Revision 1.2  1994/10/22  18:47:23  tyson
 * VMS version. Fixed structure indexing (computation of the offsets
 * in a struct). Added signal handlers to a1, b1 tcxServer.
 *
 * Revision 1.1.1.1  1994/03/31  08:35:04  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:28  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:28  rhino
 * test
 *
 * Revision 1.38  1993/03/12  20:58:08  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.37  1993/03/03  20:18:37  fedor
 * Added tcxNRecvMsgE start of new message interface.
 * Added tcxRegisterCloseHnd - generic all module close.
 * Added tcxCurrentModuleName - give module name - change to pass NULL to
 * tcxModuleName?
 *
 * Revision 1.36  1993/01/15  03:37:37  fedor
 * Added tcxRegisterConnectHnd for setting an open and close connection hnd
 *
 * Revision 1.35  1992/11/24  04:38:44  fedor
 * The module initialize message in tcxConnectModule is overloaded.
 * Make sure we get the right module - if not finish the other module connection.
 *
 * Revision 1.34  1992/11/19  03:01:57  fedor
 * Remove module from a pending sets if it crashes before a confirming connection.
 *
 * Revision 1.33  1992/11/16  15:50:58  fedor
 * Rewrote initializing connections and auto reconnect.
 *
 * Revision 1.32  1992/11/13  14:47:06  fedor
 * Update archive with a working version and partially working server route
 *
 * Revision 1.31  1992/10/27  03:42:30  fedor
 * Added tcxCloseAll (VxWorks) and changed tcxInitialized to be called afterwards.
 *
 * Revision 1.30  1992/10/26  23:16:42  fedor
 * Fixed tcxConnectModule to be called to reconnect a dead module.
 *
 * Revision 1.29  1992/10/26  22:59:10  fedor
 * Added tcxSetAutoReconnect and tcxTestActiveModule
 *
 * Revision 1.28  1992/10/23  22:35:19  fedor
 * Optional connect was using the wrong message id value!
 *
 * Revision 1.27  1992/10/23  21:56:09  fedor
 * Added tcxConnectOptional. More thought on this needs to be done.
 *
 * Revision 1.26  1992/10/23  20:15:08  fedor
 * Redid method of connecting to modules. Added the notion of a connection.
 * Added some auto-reconnect for those modules that are probably listening.
 * See detail notes.
 *
 * Revision 1.25  1992/10/14  20:26:40  fedor
 * Fixed tcxRecvLoop to return on timeout.
 *
 * Revision 1.24  1992/10/14  18:30:49  fedor
 * Changed tcxRecvData to execute at least a single handler from one connection or
 * at most one handler from each connection and return.
 * Changed tcxRecvLoop to loop here instead of in tcxRecvData - needs clean up.
 *
 * Revision 1.23  1992/10/11  18:29:51  fedor
 * Added tcxRecvMsgNoHnd as a quick fix so that connect handler completes
 * before messages get sent to user handlers.
 *
 * Revision 1.22  1992/10/10  03:04:26  fedor
 * Moving toward basing initializing connections on moduleInfo messages.
 *
 * Revision 1.21  1992/10/08  18:05:08  fedor
 * tcxInitialize was ignoring the server host parameter
 *
 * Revision 1.20  1992/10/08  14:47:47  fedor
 * Replace com.c with com2.c and remove com2.c
 *
 * Revision 1.1  1992/10/08  14:39:03  fedor
 * Using com2 instead of com.c in Makefile - should clean this up!
 *
 * Revision 1.18  1992/09/14  21:24:30  fedor
 * Changed message hash function to ignore case.
 *
 * Revision 1.17  1992/09/12  21:53:04  fedor
 * Connect when first message received. Moving towards version 5.x
 *
 * Revision 1.16  1992/08/08  00:28:18  jwest
 * Separated out stuff that only tcx server needs from library.  Lots more
 * work to be done.
 *
* 23-Nov-91 Christopher Fedor, School of Computer Science, CMU
* created.
*
******************************************************************************/

/*****************************************************************************
 * INCLUDES
 *****************************************************************************/

#ifdef VMS
#include "vms.h"
#endif

#ifdef VXWORKS
#include "/usr/vxworks/vx5.0.2b/h/vxWorks.h"
#include "/usr/vxworks/vx5.0.2b/h/stdioLib.h"
#include "/usr/vxworks/vx5.0.2b/h/ctype.h"
#include "/usr/vxworks/vx5.0.2b/h/socket.h"
#include "/usr/vxworks/vx5.0.2b/h/in.h"
#include "/usr/vxworks/vx5.0.2b/h/sigLib.h"
#include "/usr/vxworks/vx5.0.2b/h/errno.h"
#else
#include "stdio.h"
#include "ctype.h"
#include "signal.h"
#include "sys/time.h"
#include "sys/types.h"
#include "sys/socket.h"
#include "netinet/in.h"
#include "netinet/tcp.h"
#include "netdb.h"
#include "errno.h"
#endif

#include "tcx.h"
#include "tcxP.h"

#include "list.h"
#include "hash.h"
#include "key.h"
#include "formatters.h"
#include "msg.h"
#include "com.h"
#include "module.h"

#include "global.h"

/*****************************************************************************
 * EXTERNS
 *****************************************************************************/

extern void exit();
extern char *malloc();

extern FORMAT_PTR parseFormatString();
extern int idEqFunc();
extern int idHashFunc();

extern int nameHashFunc();
extern int nameEqFunc();

/*****************************************************************************
 * DEFINES
 *****************************************************************************/

#if 0
#define pgmTrace() fprintf(stderr, "%s:%6d:%s()\n", \
			   __FILE__, __LINE__, __FUNCTION__)

#define pgmReturn() fprintf(stderr, "%s:%6d:%s() return\n", \
			   __FILE__, __LINE__, __FUNCTION__)
#else
#define pgmTrace() {}
#define pgmReturn() {}
#endif

#define TCP 6      /* from etc/protocols or getprotoent call */
#define BACKLOG 5  /* backlog for listen is pretty much limited to 5 anyway */

/*****************************************************************************
 * TYPEDEFS
 *****************************************************************************/

/*****************************************************************************
 * GLOBAL VARIABLES
 *****************************************************************************/

TCX_FMT_PTR tcxCreateFMT; /** special value **/

int beeSoftMaj = -1;
int beeSoftMin = -1;
int beeSoftRobotType = -1;

/*****************************************************************************
 * STATIC VARIABLES
 *****************************************************************************/

/***************************************************************************/

int findMod(char *name, MODULE_PTR module)
{
  return(strKeyEqFunc(name, module->moduleInfo->name));
}

/**********************************************************************
 *
 *  FUNCTION:  void sigHnd(int sig, int code, struct sigcontext *scp, 
 *                         char *addr)
 *
 *  DESCRIPTION: 
 *
 *		Random Signal Handler to ignore random SIGPIPE errors.
 * Should expand to handle other signals - EINT?
 *
 *********************************************************************/

#ifdef VXWORKS
void sigHnd(int sig, int code, SIGCONTEXT *scp)
#else
     /* void sigHnd(int sig, int code, struct sigcontext *scp, char *addr) */
void sigHnd(int sig)
#endif
{
  pgmTrace();

  fprintf(stderr, "%s:%6d:%s() - WARNING: Got signal: %d\n",
	  __FILE__, __LINE__, __FUNCTION__, sig);

  if (sig == 13)
    Global->closeModGlobal = 1;

  Global->sigFlagGlobal = 1;
  Global->sigerrorGlobal = errno;
}


/***********************/

void defaultExit()
{
  pgmTrace();

  exit(1);
}

/***********************/

tcaModError(s)
char *s;
{
  pgmTrace();

  fprintf(stderr, "ERROR: tcaModError: %s\n", s);
  (*(Global->tcxExitHndGlobal))();
}

/******************************************************************************
*
* FUNCTION: int strKeyEqFunc(a, b)
*
* DESCRIPTION: 
* Return 1 (TRUE) if strings a and b are equal. Returns 0 (FALSE) otherwise.
* Case insensitive compare.
*
* INPUTS: char *a, *b
*
* OUTPUTS: int
*
******************************************************************************/

int strKeyEqFunc(a, b)
char *a, *b;
{
  int i;
  char a1, b1;

  if (a == b)
    return TRUE;

  if (!a || !b)
    return FALSE;

  i = 0;
  while (a[i] != '\0' && b[i] != '\0') {
    if (isupper(a[i]))
      a1 = tolower(a[i]);
    else 
      a1 = a[i];
    if (isupper(b[i]))
      b1 = tolower(b[i]);
    else 
      b1 = b[i];
    
    if (a1 != b1)
      return FALSE;

    i++;
  }

  if (a[i] == b[i])
    return TRUE;
  else
    return FALSE;
}

/***************************************************************************/
/*****************************/

int findPend(name, pending)
char *name;
PEND_CONNECT_PTR pending;
{
  pgmTrace();

  return(strKeyEqFunc(name, pending->name));
}

/***************************************************************************/

void connectModHndX(MODULE_PTR refModule, MODULE_INFO_PTR moduleInfo)
{
  MODULE_PTR module;
  HND_CONNECT_PTR hndConnect;

  pgmTrace();

/*****
  printf("connectModHnd: start\n");

  printf("%d: %s: %s\n", moduleInfo->port, moduleInfo->name, 
    moduleInfo->host);

  printf("press return.\n");
  getchar();
****/

  module = (MODULE_PTR)listMemReturnItem(findMod, moduleInfo->name, 
					 Global->moduleListGlobal);

  /* 16-Nov-92: fedor: make sure sd set in tcxConnectionListGlobal */

  if (module) {
    /* reconnect a dead module */
    module->sd = refModule->sd;
    free(module->moduleInfo);
    module->moduleInfo = moduleInfo;

    listDeleteItem(refModule, Global->moduleListGlobal); /* remove old */
    free(refModule);    /* free one created by accept */
    refModule = module; /* reset ref->module */

    module->status = STATUS_CONNECTED;

  } else {
    /* new module created by accept - update it */    
    module = refModule;

    free(module->moduleInfo);
    module->moduleInfo = moduleInfo;
  }

  fprintf(stderr, "New Connection Detected From: %d: %d: %s: %s\n", 
	  module->sd, module->moduleInfo->port, 
	  module->moduleInfo->name, module->moduleInfo->host);

  /* send confirm */
  tcxSendMsg(Global->tcxServerGlobal, "TCXConfirmINFO", &(module->moduleInfo->name));

  if (Global->hndConnectTable) {
    hndConnect = (HND_CONNECT_PTR)hashTableFind(module->moduleInfo->name,
						Global->hndConnectTable);
    if (hndConnect && hndConnect->openHnd) {
      (*hndConnect->openHnd)(module->moduleInfo->name); 
    }
  }

  /*** printf("connectModHnd: end\n");***/
}

/***************************************************************************/

void connectModHnd(TCX_REF_PTR ref, MODULE_INFO_PTR moduleInfo, void *data)
{
  connectModHndX(ref->module, moduleInfo);
}

/***************************************************************************/

comInit()
{
#ifdef VXWORKS
  SIGVEC pVec;

  pgmTrace();

  pVec.sv_handler = sigHnd;
  pVec.sv_mask = 0;
  pVec.sv_flags = 0;

  sigvec(SIGPIPE, &pVec, NULL);
#else
  /* 27-Nov-95: kurien: signal only handles the signal one time.
     That is when the signal is caught, the handler for the signal
     is reset to the default.  So, the second SIGPIPE terminates
     the program.  We could call signal() again in the handler,
     but the right thing to do is use sigaction, which causes the
     signal handler to be reinstalled automagically after the signal
     is caught. */
#ifdef SA_RESTART
  struct sigaction newAction;
  newAction.sa_handler = (void*) sigHnd;
  sigemptyset(&newAction.sa_mask);
  newAction.sa_flags = SA_RESTART;
  sigaction(SIGPIPE,&newAction,NULL);
#else
  signal(SIGPIPE, (void *) sigHnd);
#endif
#endif

  globalInit();

  Global->sigFlagGlobal = 0;

  Global->tcxExitHndGlobal = defaultExit;

  Global->recvUseHndGlobal = 1;

  messageInitialize();

  Global->hndIdTable = hashTableCreate(10, idHashFunc, idEqFunc);

  tcxRegisterExitHnd(exit);

  formatInitialize();

  tcxCreateFMT = NEW_FORMATTER(); /* give it a value */

  messageDefinition(5, "TCXmsgReg", MSG_REG_FORM);
  messageDefinition(6, "TCXmsgRegReply", MSG_REG_REPLY_FORM);
  messageDefinition(7, "TCXhndReg", HND_REG_FORM);

  messageDefinition(8, "TCXforw", MSG_FORW_FORM);

  messageDefinition(9,"TCXmsgReg2Reply", MSG_REG2_REPLY_FORM);

  messageDefinition(10, "TCXRegMsgSend", MSG_REG2_FORM);

  messageDefinition(11, "TCXinitialize", MODULE_INFO_FORMAT);
  messageDefinition(12, "TCXversionINFO", VERSION_FORM);
  messageDefinition(13, "TCXconfirmINFO", "string");
  messageDefinition(14, "TCXconnectRQST", "string");
  messageDefinition(15, "TCXconnectToModRQST", MODULE_INFO_FORMAT);

  messageDefinition(16, "TCXrecvLoopINFO", "string");

  messageDefinition(17, "TCXconnectOptionalRQST", "string");

  messageDefinition(18, "TCXserverRoute", MODULE_INFO_FORMAT);

  messageDefinition(19, "TCXinitializeServerRoute", MODULE_INFO_FORMAT);

  Global->dataListGlobal = listCreate();

  Global->dataBufGlobal.len = 1024;
  Global->dataBufGlobal.buf = (char *)malloc(1024);

  Global->moduleListGlobal = listCreate();
  FD_ZERO(&(Global->tcxConnectionListGlobal));
}

/***************************************************************************/

char *tcxModuleName(module)
MODULE_PTR module;
{
  pgmTrace();

  if (module && module->moduleInfo)
    return(module->moduleInfo->name);
  else
    return NULL;
}

/**********************************************************************
 *
 *  FUNCTION:  
 *
 *  DESCRIPTION: 
 *
 *		Combine connectAtPort with sending module info.
 *
 *********************************************************************/

MODULE_PTR initiateConnection(MODULE_PTR module, char *name,
      char *host, int port)
{
  int sd;
  MODULE_PTR newModule;

  pgmTrace();

  sd = connectToModule(name, host, port);

  newModule = moduleCreate(sd, port, name, host, STATUS_CONNECTED);

  tcxSendMsg(newModule, "TCXinitialize", module->moduleInfo);

  listInsertItemLast(newModule, module->connections);

  return(newModule);
}

/***************************************************************************/

void connectToModHnd(TCX_REF_PTR ref, MODULE_INFO_PTR moduleInfo, void *data)
{
  int sd;
  MODULE_PTR module;
  HND_CONNECT_PTR hndConnect;

  pgmTrace();

  /*** printf("connectToModHnd: start\n");***/

  sd = connectToModule(moduleInfo->name, moduleInfo->host, moduleInfo->port);

  module = (MODULE_PTR)listMemReturnItem(findMod, moduleInfo->name,
					 Global->moduleListGlobal);

  if (module) {
    /* module already created  - update it */
    /* 23-Oct-92: fedor: may never be executed on a restart 
       - perhaps vxworks */
    module->sd = sd;
    FD_SET(module->sd, &(Global->tcxConnectionListGlobal));    
    free(module->moduleInfo);
    module->moduleInfo = moduleInfo;
    module->status = STATUS_CONNECTED;
  } else {
    module = moduleCreate2(sd, moduleInfo, STATUS_CONNECTED);
  }

  fprintf(stderr, "New Connection Created For: %d: %d: %s: %s\n", 
	  module->sd, module->moduleInfo->port, 
	  module->moduleInfo->name, module->moduleInfo->host);

  /*************
  printf("press return\n");
  getchar();
  *************/

  pgmTrace();

  tcxSendMsg(module, "TCXinitialize", Global->tcxModuleGlobal->moduleInfo);

  if (Global->hndConnectTable) {
    hndConnect = (HND_CONNECT_PTR)hashTableFind(module->moduleInfo->name, 
						Global->hndConnectTable);
    if (hndConnect && hndConnect->openHnd) {
      (*hndConnect->openHnd)(module->moduleInfo->name); 
    }
  }

  /*** printf("connectToModHnd: done\n");***/
}

/***************************************************************************/

void tcxInitializeX(module, server, serverRoute)
char *module;
char *server;
int serverRoute;
{
  static char *tcxVersion = TCX_VERSION;

  int ref;
  char *host;
  HDR_TYPE hdr;
  int port, total, modLength, hostLength, adv;

  int id, len;
  MODULE_PTR mod;
  VERSION_TYPE version;

  int sd;
  MODULE_INFO_TYPE moduleInfo;

  int status;

  pgmTrace();

  if (!Global) {
    globalInit();
  }

  if (Global->tcxInitFlagG) {
    fprintf(stderr, "WARNING: Multiple tcxInitialize: Ignore message init\n");
  } else {
    comInit();
    tcxRegisterMHnd("TCXinitialize", connectModHnd, TCX_RECV_ALL, 
		    TCX_HND, NULL);
    tcxRegisterMHnd("TCXconnectToModRQST", connectToModHnd, TCX_RECV_ALL, 
		    TCX_HND, NULL);
  }

#ifdef VXWORKS 
  Global->tcxInitFlagG = 1;
#endif

  /* connect to server */
  if (!server) {
    fprintf(stderr, "ERROR: tcxInitialize: No Server Specified!\n");
    (*(Global->tcxExitHndGlobal))();
  }
  if (!module) {
    fprintf(stderr, "ERROR: tcxInitialize: No Module Specified!\n");
    (*(Global->tcxExitHndGlobal))();
  }

  port = 0;
  if (!listenAtPort(&port, &sd)) {
    fprintf(stderr, 
	    "ERROR: tcxInitialize: Failure to create module listen socket.\n");
    (*(Global->tcxExitHndGlobal))();
  }

  host = (char *)malloc(sizeof(char)*256); /* _ws_ 32 -> 256 bytes */
/*  gethostname(host, 32); */
  if (-1 == ws_gethostname(host, 256)) /* _ws_ */
     {
      fprintf(stderr,"ws_gethostname() failed\n");
      exit(-1);
     }

  if (serverRoute) {
    status = STATUS_CONNECTED | STATUS_SINGLE_CONN;
  } else {
    status = STATUS_CONNECTED;
  }
    
  Global->tcxModuleGlobal = moduleCreate(sd, port, module, host, status);

  Global->tcxServerGlobal = initiateConnection(Global->tcxModuleGlobal, 
					       SERVER_NAME, 
					       server, 
					       SERVER_PORT);

  tcxRecvMsg("TCXversionINFO", NULL, &version, NULL);

  /*
     fprintf(stderr, "BeeSoft Version : %d.%d (B%d)\n", 
     version.beeSoftMaj,
     version.beeSoftMin,
     version.beeSoftRobotType);
     */

  beeSoftMaj = version.beeSoftMaj;
  beeSoftMin = version.beeSoftMin;
  beeSoftRobotType = version.beeSoftRobotType;

  if (version.vMaj != TCX_VERSION_MAJOR) {
      fprintf(stderr, "*** ERROR ***\n");
      fprintf(stderr, 
	      "Major Version Number is different from Task Control Server.\n");
      fprintf(stderr, "Module Version     : %d.%d\n", 
	     TCX_VERSION_MAJOR, TCX_VERSION_MINOR);
      fprintf(stderr, 
	      "Task Control Server: %d.%d\n", version.vMaj, version.vMin);
      (*(Global->tcxExitHndGlobal))();
    } else if (version.vMin != TCX_VERSION_MINOR) {
      fprintf(stderr, "*** WARNING ***\n");
      fprintf(stderr, 
	      "Minor Version Number is different from Task Control Server.\n");
      fprintf(stderr, "Module Version     : %d.%d\n", 
	     TCX_VERSION_MAJOR, TCX_VERSION_MINOR);
      fprintf(stderr, 
	      "Task Control Server: %d.%d\n", version.vMaj, version.vMin);
    } else {
      fprintf(stderr, 
	      "Task Control Connected %d.%d\n", version.vMaj, version.vMin);
    }

  /* fprintf(stderr, "done.\n");*/
}

/***************************************************************************/

void tcxInitializeInternal(char *module, char *server)
{
  pgmTrace();

  tcxInitializeX(module, server, 0);
}

void tcxInitializeServerRoute(char *module, char *server)
{
  pgmTrace();

  tcxInitializeX(module, server, 1);
}

/***************************************************************************/

MODULE_PTR tcxConnectModuleX(char *name, int optional)
{
  int sd;
  MSG_DEF_PTR msg;
  MODULE_PTR module, new;
  MODULE_INFO_TYPE moduleInfo;
  MODULE_INFO_PTR mInfo;

  HND_CONNECT_PTR hndConnect;

  pgmTrace();

  if (!name) {
    fprintf(stderr, "ERROR: tcxConnectModule: Missing module name\n");
    (*(Global->tcxExitHndGlobal))();
  }

  module = (MODULE_PTR)listMemReturnItem(findMod, name, Global->moduleListGlobal);

  if (module) {
    /* already connected - return */
    /* 22-Oct-92: fedor: only part of code that prevents double connections */
    if (tcxTestActiveModule(module)) {
      return module;
    } /* 16-Nov-92:fedor  can not garbage collect dead module because
	 a data item may be pointing to it */
  }
  
  /***********
  printf("press return\n");
  getchar();
  ***********/

  if (optional) {
    tcxSendMsg(Global->tcxServerGlobal, "TCXconnectOptionalRQST", &name);
  } else {
    tcxSendMsg(Global->tcxServerGlobal, "TCXconnectRQST", &name);
  }

  do {
    new = NULL;
    
    pgmTrace();

    msg = messageFindByName("TCXinitialize");
    tcxRecvData(&new, &msg->id, NULL, &moduleInfo, msg->msgFormat, TCX_HND, 
		NULL);

    /***********
      printf("tcxConnectModule: %s:%s\n", name, moduleInfo.name);
      printf("press return\n");
      getchar();
      ***********/

    if (optional && new == Global->tcxServerGlobal) {
      return NULL;
    }
    
    if (!strKeyEqFunc(moduleInfo.name, name)) {
      mInfo = (MODULE_INFO_TYPE *)malloc(sizeof(MODULE_INFO_TYPE));
      mInfo->port = moduleInfo.port;
      mInfo->status = moduleInfo.status;
      mInfo->name = moduleInfo.name;
      mInfo->host = moduleInfo.host;
      mInfo->vmajor = moduleInfo.vmajor;
      mInfo->vminor = moduleInfo.vminor;
      connectModHndX(new, mInfo);
      new = NULL;
    }

  } while (!new);

  if (moduleInfo.status & STATUS_SERVER_ROUTE) {
    module = moduleCreate(0, moduleInfo.port, moduleInfo.name,
			  moduleInfo.host, (STATUS_SERVER_ROUTE |
					    STATUS_CONNECTED));
  } else {
    if (!module) {
      module = new;
    } else {
      /* old module re-initialize and remove newly created module */
      listDeleteItem(new, Global->moduleListGlobal);
      module->sd = new->sd;
      module->status = new->status;
      listFree(new->itemQ);
      listFree(new->pending);
      listFree(new->connections);
      free(new); /* should be ok since all data items should be cleared out */
    }
    module->moduleInfo->port = moduleInfo.port;
    module->moduleInfo->name = moduleInfo.name;
    module->moduleInfo->host = moduleInfo.host;
    module->moduleInfo->vmajor = moduleInfo.vmajor;
    module->moduleInfo->vminor = moduleInfo.vminor;
  }

  tcxSendMsg(Global->tcxServerGlobal, "TCXconfirmINFO", &(module->moduleInfo->name));

  if (Global->hndConnectTable) {
    hndConnect = (HND_CONNECT_PTR)hashTableFind(module->moduleInfo->name,
						Global->hndConnectTable);
    if (hndConnect && hndConnect->openHnd) {
      (*hndConnect->openHnd)(module->moduleInfo->name); 
    }
  }
  
  return module;
}

/***************************************************************************/

MODULE_PTR tcxConnectOptional(char *name)
{
  pgmTrace();

  return(tcxConnectModuleX(name, 1));
}

/************************************************/

MODULE_PTR tcxConnectModule(char *name)
{
  pgmTrace();

  return(tcxConnectModuleX(name, 0));
}

/************************************************/

TCX_FMT_PTR tcxParseFormat(s)
char *s;
{
  pgmTrace();

  if (s) {
    return(parseFormatString(s));
  } else {
    return(NULL);
  }
}

/********************************/

void tcxFreeData(fmt, data)
TCX_FMT_PTR fmt;
void *data;
{
  pgmTrace();

  if (fmt && data) {
    freeDataStructure(fmt, data);
  }
}

/********************************/

void tcxFree(char *name, void *data)
{
  MSG_DEF_PTR msg;

  pgmTrace();

  msg = messageFindByName(name);
  
  if (data && msg) {
    freeDataStructure(msg->msgFormat, data);
  }
}

/********************************/
/********
void tcxFreeReply(char *name, void *data)
{
  MSG_DEF_PTR msg;

  pgmTrace();

  msg = messageFindByName(name);
  
  if (data && msg) {
    freeDataStructure(msg->resFormat, data);
  }
}
*********/
/********************************/

void tcxFreeByRef(TCX_REF_PTR ref, void *data)
{
  MSG_DEF_PTR msg;

  pgmTrace();

  if (!ref)
    return;

  msg = messageFindById(ref->id);
  
  if (data && msg) {
    freeDataStructure(msg->msgFormat, data);
  }
}


/********************************/

void tcxDecodeData(fmt, buf, data)
TCX_FMT_PTR fmt;
void *buf, *data;
{
  char *r2;
  int amount;

  pgmTrace();

  if (fmt && buf) {
    r2 = (char *)decodeData(fmt, buf, 0);
    amount = dataStructureSize(fmt);

    bcopy(r2, data, amount);
    free(r2);
  }
}

/********************************/

void tcxSetAutoReconnect()
{
  pgmTrace();

  tcxSendMsg(Global->tcxServerGlobal, "TCXrecvLoopINFO", 
	     &(Global->tcxModuleGlobal->moduleInfo->name));
  Global->tcxModuleGlobal->recv = 1;
}

/********************************/

int tcxRecvLoop(void *timeout)
{
  int id, ref, len;
  TCX_MODULE_PTR module;

  pgmTrace();

  if (Global->tcxModuleGlobal->recv == 0) {
    tcxSetAutoReconnect();
  }

  while (1) {
    id = 0;
    module = NULL;
    len = tcxRecvData(&module, &id, NULL, NULL, NULL, ALL_HND, timeout);

    if (len == -1) {
      return -1; /* timeout */
    }

    if (len != -2) {
      fprintf(stderr, "ERROR: tcxRecvLoop: Unhandled Message Id %d from %s\n", 
	      id, module->moduleInfo->name);
    }
  }
}

/********************************/

/* 13-Jan-93: fedor: may be able to simplify this by including open and close
   handlers within module structure. Avoided this for now because it may 
   confuse auto reconnect code. */

void tcxRegisterConnectHnd(char *name, void (*openHnd)(), void (*closeHnd)())
{
  HND_CONNECT_PTR hndConnect;

  pgmTrace();

  if (!name) {
    return;
  }

  if (!openHnd && !closeHnd) {
    return;
  }

  if (!Global) {
    globalInit();
  }

  /* 14-Jan-93: fedor: support a call here before tcxInitialize */
  if (!Global->hndConnectTable) {
    Global->hndConnectTable = hashTableCreate(10, nameHashFunc, nameEqFunc);
    hndConnect = NULL;
  } else {
    hndConnect = (HND_CONNECT_PTR)hashTableFind(name, Global->hndConnectTable);
  }

  if (hndConnect) {
    hndConnect->openHnd = openHnd;
    hndConnect->closeHnd = closeHnd;
  } else {
    hndConnect = (HND_CONNECT_TYPE *)malloc(sizeof(HND_CONNECT_TYPE));
    hndConnect->name = name;
    hndConnect->openHnd = openHnd;
    hndConnect->closeHnd = closeHnd;
    hashTableInsert(name, strlen(name)+1, hndConnect, Global->hndConnectTable);
  }
}

void tcxRegisterCloseHnd(void (*closeHnd)())
{
  pgmTrace();

  Global->tcxCloseHndG = closeHnd;
}

char *tcxCurrentModuleName(void)
{
  pgmTrace();

  return(tcxModuleName(Global->tcxModuleGlobal));
}
