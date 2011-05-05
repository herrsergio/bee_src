
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

#include <bUtils.h>

#include "global.h"

#include "tcx.h"
#include "tcxP.h"

#include "list.h"
#include "hash.h"
#include "key.h"
#include "formatters.h"
#include "msg.h"
#include "com.h"

#include "module.h"

#include "beeSoftVersion.h"

/*****************************************************************************
 * EXTERNS
 *****************************************************************************/

extern MSG_INS_PTR messageNextInstance();
extern int findMod(char *name, MODULE_PTR module);

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

/*****************************************************************************
 * STATIC VARIABLES
 *****************************************************************************/

/***************************************************************************/

int messageInsSend(item, hnd)
MSG_INS_PTR item;
HND_PTR hnd;
{

  pgmTrace();

  /* 27-Jul-92: fedor: should add hnd for tcxServerGlobal - should place
     item on dataListGlobal and return but messageForwHnd frees item! */
  /*********
    printf("Forward: %d To: %s:%s\n", item->id, 
    hnd->module->name, hnd->module->host);
    **********/

  tcxSendData(hnd->module, item->id, item->ref, 
	      item->encodedData, item->len, NULL);

  return 1; /* continue for all handlers for this message */
}

/***************************************************************************/

void handlerRegisterHnd(TCX_REF_PTR ref, HND_REG_PTR hndList, void *data)
{
  int i;
  HND_PTR hnd;
  MSG_DEF_PTR msg;
  
  pgmTrace();

  for(i=0; i < hndList->num; i++) {
    /***
      printf("%d: %s: %s\n", i, hndList->hnds[i].msgName,
      hndList->hnds[i].hndName);
      *****/

    msg = (MSG_DEF_PTR)messageFindByName(hndList->hnds[i].msgName);
  
    if (!msg) {
      fprintf(stderr, "ERROR: handlerRegisterHnd: Missing message def: %s\n",
	      hndList->hnds[i].msgName);
      break;
    }

    hnd = (HND_PTR)hashTableFind(&msg->id, Global->hndIdTable);

    if (!hnd) {
      hnd = (HND_TYPE *)malloc(sizeof(HND_TYPE));
      hashTableInsert(&msg->id, sizeof(int), hnd, Global->hndIdTable);
      listInsertItemLast(hnd, msg->hndList);
    }

    hnd->proc = NULL;
    hnd->control = hndList->hnds[i].hndControl;
    hnd->status = HND_EXEC;
    hnd->id = msg->id;
    hnd->fmt = msg->msgFormat;
    hnd->callData = NULL;
    hnd->type = MsgHnd;
    hnd->module = tcxRefModule(ref);
  }
}

/***************************************************************************/

void messageForwHnd(TCX_REF_PTR ref, void *data)
{
  MSG_DEF_PTR msg;
  MSG_INS_PTR item;

  pgmTrace();

  item = messageNextInstance();

  /*********
    if (!item) {
    printf("No Item!\n");
    return;
    }

    printf("id: %d: ref: %d: len: %d\n", item->id, item->ref, item->len);
    *********/

  msg = messageFindById(item->id);

  if (msg != NULL)		/* S.Thrun 94-7-11. Dirty Hack!! */
    listIterateFromFirst(messageInsSend, item, msg->hndList);

  if (item->encodedData) {
    free(item->encodedData);
  }
  free(item);
}

/******************************************************/

void regMsgSendHnd(TCX_REF_PTR ref, MSG_REG2_PTR msgList, void *data)
{
  int len;
  char *str;

  int i, *ids;
  MSG_DEF_PTR msg;
  MSG_REG2_REPLY_TYPE idList;

  pgmTrace();

  ids = (int *)malloc(sizeof(int)*msgList->num);

  for (i=0; i<msgList->num;i++) {

    msg = (MSG_DEF_PTR)hashTableFind(msgList->msgs[i].name, 
				     Global->msgNameTableGlobal);

    if (!msg) {
      msg = (MSG_DEF_TYPE *)malloc(sizeof(MSG_DEF_TYPE));
      if (!Global->msgIdS) {
	fprintf(stderr, 
		"ERROR: messageRegisterHnd: Message Id Generation Wrapped!");
	Global->msgIdS = 1000;
      }

      msg->id = Global->msgIdS++;
      msg->msgFormat = NULL;
      msg->hndList = listCreate();

      /*** form bcopy ***/

      if (msgList->msgs[i].format) {
	str = msgList->msgs[i].format;
	len = strlen(str);

	msg->msgForm = (char *)malloc(sizeof(char)*len+1);
	
	bcopy(str, msg->msgForm, len);
	msg->msgForm[len] = '\0';
      }

      /*** name bcopy ***/

      if (msgList->msgs[i].name) {
	str = msgList->msgs[i].name;
	len = strlen(str);

	msg->name = (char *)malloc(sizeof(char)*len+1);

	bcopy(str, msg->name, len);
	msg->name[len] = '\0';

	/* hashTableInsert(&msg->id, sizeof(int), msg, Global->msgIdTableGlobal);*/
	/* hashTableInsert(str, sizeof(char)*len+1, msg, Global->msgNameTableGlobal);*/
      } else {
	fprintf(stderr, "messageRegisterHnd: NULL Message Name: %d\n", i);
      }
    }
    ids[i] = msg->id;

    printf("%d: %s ", msgList->msgs[i].connections, msgList->msgs[i].name);

    if (msgList->msgs[i].format) {
	printf("%s ", msgList->msgs[i].format);
    } else {
	printf("NULL ");
    }

    printf("\n");

  }

  idList.num = msgList->num;
  idList.ids = ids;

  tcxReply(ref, "TCXmsgReg2Reply", &idList);

  free(ids);
  tcxFreeByRef(ref, msgList);
}

/*****************/

void messageRegisterHnd(TCX_REF_PTR ref, MSG_REG_PTR msgList, void *data)
{
  int len;
  char *str;

  int i, *ids;
  MSG_DEF_PTR msg;
  MSG_REG_REPLY_TYPE idList;

  pgmTrace();

  ids = (int *)malloc(sizeof(int)*msgList->num);

  for (i=0; i<msgList->num;i++) {

    msg = (MSG_DEF_PTR)hashTableFind(msgList->msgs[i].msgName, 
				     Global->msgNameTableGlobal);

    if (!msg) {
      msg = (MSG_DEF_TYPE *)malloc(sizeof(MSG_DEF_TYPE));
      if (!Global->msgIdS) {
	fprintf(stderr, 
		"ERROR: messageRegisterHnd: Message Id Generation Wrapped!");
	Global->msgIdS = 1000;
      }

      msg->id = Global->msgIdS++;
      msg->msgFormat = NULL;
      msg->hndList = listCreate();

      /*** form bcopy ***/

      if (msgList->msgs[i].msgFormat) {
	str = msgList->msgs[i].msgFormat;
	len = strlen(str);

	msg->msgForm = (char *)malloc(sizeof(char)*len+1);
	
	bcopy(str, msg->msgForm, len);
	msg->msgForm[len] = '\0';
      }

      /*** name bcopy ***/

      if (msgList->msgs[i].msgName) {
	str = msgList->msgs[i].msgName;
	len = strlen(str);

	msg->name = (char *)malloc(sizeof(char)*len+1);

	bcopy(str, msg->name, len);
	msg->name[len] = '\0';

	hashTableInsert(&msg->id, sizeof(int), msg, Global->msgIdTableGlobal);
	hashTableInsert(str, sizeof(char)*len+1, msg, Global->msgNameTableGlobal);
      } else {
	fprintf(stderr, "messageRegisterHnd: NULL Message Name: %d\n", i);
      }
    }
    ids[i] = msg->id;

    /*************
      printf("%d: %s ", msgList->msgs[i].id, msgList->msgs[i].msgName);

      if (msgList->msgs[i].msgFormat) {
      printf("%s ", msgList->msgs[i].msgFormat);
      } else {
      printf("NULL ");
      }
      if (msgList->msgs[i].resFormat) {
      printf("%s\n", msgList->msgs[i].resFormat);
      } else {
      printf("NULL\n");
      }
      *************/
  }

  idList.num = msgList->num;
  idList.ids = ids;

  tcxReply(ref, "TCXmsgRegReply", &idList);

  free(ids);
  tcxFreeByRef(ref, msgList);
}

/***************************************************************************/

void sendConnectToModRqst(MODULE_PTR to, MODULE_PTR info)
{
  if (to->moduleInfo->status & STATUS_SERVER_ROUTE) {

    pgmTrace();

    tcxSendMsg(info, "TCXinitialize", to->moduleInfo);
  } else {
    tcxSendMsg(to, "TCXconnectToModRQST", info->moduleInfo);
  }
}

/***************************************************************************/

int iteratePending2(MODULE_PTR module, MODULE_PTR pending)
{
  if (MODULE_IS_ACTIVE(pending)) {
    sendConnectToModRqst(module, pending);
  } else {
    listDeleteItem(pending, module->pending);
    listInsertItem(module, pending->pending);
  }

  return 1;
}

/***************************************************************************/

void connectOptionalHnd(TCX_REF_PTR ref, char **name, void *data)
{
  MODULE_PTR module, requestMod;
  MODULE_INFO_TYPE moduleInfo;

  pgmTrace();


#ifdef VERBOSE
  printf("connectOptionalHnd: start\n");
  printf("%s\n", *name);
#endif

  requestMod = ref->module;

  module = (MODULE_PTR)listMemReturnItem(findMod, *name, Global->moduleListGlobal);

  if (!module || (module->status & STATUS_INACTIVE)) {
    printf("Module Not Found: %s\n", *name);

    pgmTrace();

    tcxSendMsg(requestMod, "TCXinitialize", Global->tcxServerGlobal->moduleInfo);
#ifdef VERBOSE
    printf("connectOptionalHnd: done\n"); 
#endif
    return;
  } 

  tcxFree("TCXconnectOptionalRQST", name);

  sendConnectToModRqst(module, requestMod);

#ifdef VERBOSE
  printf("connectOptionalHnd: done\n"); 
#endif

}

/***************************************************************************/

void connectHnd(TCX_REF_PTR ref, char **name, void *data)
{
  MODULE_PTR module, requestMod;
  MODULE_INFO_TYPE moduleInfo;

  pgmTrace();

#ifdef VERBOSE
  printf("connectHnd: start\n");
  printf("%s\n", *name);
#endif

  requestMod = ref->module;

  module = (MODULE_PTR)listMemReturnItem(findMod, *name, Global->moduleListGlobal);

  if (!module) {
    module = moduleCreate(0, 0, *name, "Unknown", STATUS_PENDING);
  } else {
    tcxFree("TCXconnectRQST", name);
  }
  
  if (MODULE_IS_ACTIVE(module)) {
    sendConnectToModRqst(module, requestMod);
  } else {
    listInsertUnique(requestMod, module->pending);
  }

#ifdef VERBOSE
  printf("connectHnd: done\n"); 
#endif

}

/***************************************************************************/

void confirmHnd(TCX_REF_PTR ref, char **name, void *data)
{
  VERSION_TYPE version;

  MODULE_PTR pending, module;

  pgmTrace();

  pending = ref->module;
  module = (MODULE_PTR)listMemReturnItem(findMod, *name, Global->moduleListGlobal);

  if (!module) {
    fprintf(stderr, "ERROR: confirmHnd: Missing Module: %s\n", *name);
    return;
  }

  tcxFreeByRef(ref, name);

  printf("%s confirms connection to %s\n", 
	 pending->moduleInfo->name, module->moduleInfo->name);

  if (listMemberItem(pending, module->pending)) {
    listDeleteItem(pending, module->pending);
    if (!listLength(module->pending)) {
      version.vMaj = TCX_VERSION_MAJOR;
      version.vMin = TCX_VERSION_MINOR;
      version.beeSoftMaj = BEESOFT_VERSION_MAJOR;
      version.beeSoftMin = BEESOFT_VERSION_MINOR;
      version.beeSoftRobotType = BEESOFT_VERSION_ROBOT_TYPE;
#ifdef VERBOSE
      printf("c: send version info: %s\n", module->moduleInfo->name);
#endif
      tcxSendMsg(module, "TCXversionInfo", &version);
    }
  } /* else - simply confirming a connection */

  /* store connections */
  listInsertUnique(pending, module->connections);
  listInsertUnique(module, pending->connections);
}

/***************************************************************************/

void recvLoopHnd(TCX_REF_PTR ref, char **name, void *data)
{
  MODULE_PTR module;

  pgmTrace();

  module = (MODULE_PTR)listMemReturnItem(findMod, *name, Global->moduleListGlobal);

  if (module) {
    module->recv = 1;
  }

#ifdef VERBOSE
  if (name != NULL && *name != NULL)
    printf("recvLoopHnd: %s\n", *name);
  else
    printf("recvLoopHnd: (NULL)\n");
#endif

  tcxFreeByRef(ref, name);
}

/***************************************************************************/

void initializeHnd(TCX_REF_PTR ref, MODULE_INFO_PTR moduleInfo, void *data)
{
  MODULE_PTR module;
  VERSION_TYPE version;

  pgmTrace();

  version.vMaj = TCX_VERSION_MAJOR;
  version.vMin = TCX_VERSION_MINOR;
  version.beeSoftMaj = BEESOFT_VERSION_MAJOR;
  version.beeSoftMin = BEESOFT_VERSION_MINOR;
  version.beeSoftRobotType = BEESOFT_VERSION_ROBOT_TYPE;
  
  printf("Initialize Connection: %d: %s: %s\n", 
	 moduleInfo->port, moduleInfo->name, moduleInfo->host);


  module = (MODULE_PTR)listMemReturnItem(findMod, moduleInfo->name,
					 Global->moduleListGlobal);

  if (module) {
    /* module already created by tcxConnectModule 
       or dead module restarting - update it */
    module->sd = ref->module->sd;
    free(module->moduleInfo);
    module->moduleInfo = moduleInfo;

    listDeleteItem(ref->module, Global->moduleListGlobal); /* remove old */

#if 0
    /*
     * XXX
     * I dont know how this is supposed to be but ref->module
     * still gets used somehow.  This works OK until the memory
     * free()d here gets reused.  As a gross hack to avoid segv's I
     * am naively just commenting out this free() call.  Hopefully
     * it will result in nothing worse than a small memory leak.
     *     -tds
     */
    
    free(ref->module);    /* free one created by accept */
#else
    fprintf(stderr, "%s:%6d:%s() - "
	    "skipping free(ref->module); module = 0x%X\n",
	    __FILE__, __LINE__, __FUNCTION__, ref->module);
#endif

    ref->module = module; /* reset ref->module */

  } else {
    /* new module created by accept - update it */
    module = ref->module;

    free(module->moduleInfo);
    module->moduleInfo = moduleInfo;
  }

  module->status = STATUS_ACTIVE;

  reconnectModule2(module);

  if (listLength(module->pending)) {
    listIterate(iteratePending2, module, module->pending);
    if (!listLength(module->pending)) {
      printf("d: send version info: %s\n", module->moduleInfo->name);
      tcxSendMsg(module, "TCXversionInfo", &version);
    }
  } else {
#ifdef VERBOSE
    printf("b: send version info: %s\n", module->moduleInfo->name);
#endif
    tcxSendMsg(module, "TCXversionInfo", &version);
  }
}

/***************************************************************************/

extern MSG_INS_PTR messageCreateInsFromMsg();
extern void tcxSendDoubleIns();

void serverRouteHnd(TCX_REF_PTR ref, MODULE_INFO_PTR moduleInfo, void *data)
{
  MSG_DEF_PTR msg;
  MSG_INS_PTR item1, item2;

  MODULE_PTR module;

  pgmTrace();

  item1 = messageNextInstance();

  module = (MODULE_PTR)listMemReturnItem(findMod, moduleInfo->name,
					 Global->moduleListGlobal);
  msg = messageFindById(item1->id);

  if (module->status & STATUS_ACTIVE) {
    printf("%s -> %s: %s: %d\n", ref->module->moduleInfo->name,
	   module->moduleInfo->name, msg->name, item1->ref);

    pgmTrace();
  
    item2 = messageCreateInsFromMsg(Global->tcxServerGlobal, "TCXinitialize", 
				    ref->module->moduleInfo, item1->ref);

    tcxSendDoubleIns(module, item1, item2);
    /********
      tcxSendData(module, item->id, item->ref, 
      item->encodedData, item->len, NULL);
      *********/

    if (item2->encodedData) {
      free(item2->encodedData);
    }
    free(item2);

    if (item1->encodedData) {
      free(item1->encodedData);
    }
    free(item1);
  } else {
    listInsertItem(item1, module->itemQ);
  }

  tcxFreeByRef(ref, moduleInfo);
}


/***************************************************************************/

tcxInit()
{
  char *host;
  int sd, port;

  pgmTrace();

/*  host = (char *)malloc(sizeof(char)*32); */
  host = (char *)malloc(sizeof(char)*256); /* _ws_ 32 -> 256 bytes */
/*  gethostname(host, 32); */
  if (-1 == ws_gethostname(host, 256)) /* _ws_ */
     {
      fprintf(stderr,"ws_gethostname() failed\n");
      exit(-1);
     }

  port = SERVER_PORT;

  if (!listenAtPort(&port, &sd)) {
    fprintf(stderr, "ERROR: tcxServer already running on this machine.\n");
#ifndef VXWORKS
    if (Global != NULL)
      (*(Global->tcxExitHndGlobal))();
    exit(-1);
#else
    fprintf(stderr, "Restarting - assume some memory loss with restart\n");
#endif
    fflush (stderr);
  } else {
    comInit();
  }

/*****************
    printf("ERROR: tcxlistenAtPort: Assuming restart - lost mem\n");
    FD_SET(tcaServerModGlobal->sd, &(gM->tcaConnectionListGlobal));
    
    modKey.sd = tcaServerModGlobal->sd;
    modKey.name = tcaServerModGlobal->modData->modName;

    hashTableInsert(&modKey, sizeof(MODULE_KEY_TYPE), tcaServerModGlobal, 
		    moduleTable);

    listInsertItem(tcaServerModGlobal, moduleList);

    * return FALSE; *
***********************/

  Global->tcxServerGlobal = moduleCreate(sd, port, SERVER_NAME, host, 
				 STATUS_CONNECTED);

  /* trigger for central server */
  Global->tcxModuleGlobal = Global->tcxServerGlobal; 

  tcxRegisterMHnd("TCXmsgReg", messageRegisterHnd, TCX_RECV_ALL, 
		  TCX_HND, NULL);

  tcxRegisterMHnd("TCXhndReg", handlerRegisterHnd, TCX_RECV_ALL,
		  TCX_HND, NULL);

  tcxRegisterMHnd("TCXforw", messageForwHnd, TCX_RECV_ALL,
		  TCX_HND, NULL);

  tcxRegisterMHnd("TCXRegMsgSend", regMsgSendHnd, TCX_RECV_ALL,
		  TCX_HND, NULL);

  tcxRegisterMHnd("TCXinitialize", initializeHnd, TCX_RECV_ALL,
		  TCX_HND, NULL);

  tcxRegisterMHnd("TCXconnectRQST", connectHnd, TCX_RECV_ALL,
		  TCX_HND, NULL);

  tcxRegisterMHnd("TCXconnectOptionalRQST", connectOptionalHnd, TCX_RECV_ALL,
		  TCX_HND, NULL);

  tcxRegisterMHnd("TCXconfirmINFO", confirmHnd, TCX_RECV_ALL,
		  TCX_HND, NULL);

  tcxRegisterMHnd("TCXrecvLoopINFO", recvLoopHnd, TCX_RECV_ALL,
		  TCX_HND, NULL);

  tcxRegisterMHnd("TCXserverRoute", serverRouteHnd, TCX_RECV_ALL,
		  TCX_HND, NULL);
}

/***************************************************************************/

execMsg(module, id, data, length)
MODULE_PTR module;
int id;
char *data;
int length;
{
  pgmTrace();

  if (id) {
    fprintf(stderr, "ERROR: execMssg: Unknown id: %d\n", id);
   /* (*(Global->tcxExitHndGlobal))();*/
  }
  pgmReturn();
  return;
}

/***************************************************************************/

serverLoop()
{
  char *data;
  int id, amount;
  TCX_FMT_PTR fmt;
  TCX_MODULE_PTR module;

  pgmTrace();

  fprintf(stderr, "TCX Server %d.%d (%s)\n", 
	 TCX_VERSION_MAJOR, TCX_VERSION_MINOR, VERSION_DATE);
  
  for (;;) {
    id = 0;
    data = NULL;
    module = NULL;
    amount = tcxRecvData(&module, &id, NULL, &data, tcxCreateFMT, 
			 ALL_HND, NULL);
    execMsg(module, id, data, amount);
  }
}

/***************************************************************************/

void Quit(void) {
    printf ("Exiting on SIGINT ...\n");
    exit(0);
}

#ifdef VXWORKS
tcx()
#else
int
main(int argc, char *argv[])
#endif
{
#ifdef VXWORKS
  bDaemonize("tcxServer");
#else
#ifdef B_UTILS_H
  int forkProcess;
  struct bParamList * paramList = NULL;

  paramList = bParametersAddEntry(paramList, "", "fork", "yes");

  /* add some parameter files */
  paramList = bParametersAddFile(paramList, "etc/beeSoft.ini");

  /* add command line arguements */
  paramList = bParametersAddArray(paramList, "", argc, argv);

  /* extract parameters from the list */
  forkProcess = bStrToTruth(bParametersGetParam(paramList, "", "fork"));

#endif
#endif

  signal(SIGINT, (void *)Quit);

  tcxInit();


#ifndef VXWORKS
#ifdef B_UTILS_H
  if (forkProcess) 
    bDaemonize("tcxServer");
#endif
#endif

  serverLoop();
  exit(0);
}
