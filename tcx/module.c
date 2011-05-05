
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
 *  MODULE: Module
 *
 *  FILE: module.c
 *
 *  ABSTRACT: Module Tracking Routines
 *
 *  EXPORTS:
 *
 *  HISTORY:
 *  
 *  $Log: module.c,v $
 *  Revision 1.6  1997/04/27 11:39:11  fox
 *  Removed debug output.
 *
 *  Revision 1.5  1997/04/12 06:43:58  tyson
 *  TCX segv hacked plus clean-up
 *
 *  Revision 1.4  1997/04/01 22:30:36  tyson
 *  bugs and stuff
 *
 *  Revision 1.3  1997/02/22 05:35:41  thrun
 *  Version numbers are now also checked for libraries.
 *
 *  Revision 1.2  1997/02/22 00:59:15  thrun
 *  Introduced version number support
 *
 *  Revision 1.1.1.1  1996/09/22 16:46:01  rhino
 *  General reorganization of the directories/repository, fusion with the
 *  RWI software.
 *
 *  Revision 1.2  1994/10/22 18:47:36  tyson
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
 * Revision 1.19  1993/03/12  20:58:51  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.18  1993/03/03  20:18:40  fedor
 * Added tcxNRecvMsgE start of new message interface.
 * Added tcxRegisterCloseHnd - generic all module close.
 * Added tcxCurrentModuleName - give module name - change to pass NULL to
 * tcxModuleName?
 *
 * Revision 1.17  1993/01/15  03:37:41  fedor
 * Added tcxRegisterConnectHnd for setting an open and close connection hnd
 *
 * Revision 1.16  1992/11/19  03:02:03  fedor
 * Remove module from a pending sets if it crashes before a confirming connection.
 *
 *  $Revision: 1.6 $
 *  $Date: 1997/04/27 11:39:11 $
 *  $Author: fox $
 *
 *
 *********************************************************************/


#ifdef VMS
#include "vms.h"
#endif

#ifdef VXWORKS
#include "/usr/vxworks/vx5.0.2b/h/vxWorks.h"
#include "/usr/vxworks/vx5.0.2b/h/stdioLib.h"
#include "/usr/vxworks/vx5.0.2b/h/types.h"
#else
#include "stdio.h" 
#include "sys/types.h"
#endif

#include "tcx.h"
#include "tcxP.h"

#include "list.h"

#include "module.h"

#include "hash.h"

#include "global.h"

#include "beeSoftVersion.h"

extern int findMod();
extern int findPend();

/*************************************/

MODULE_INFO_PTR moduleInfoCreate(int status, int port, char *name, char *host)
{
  MODULE_INFO_PTR moduleInfo;

  moduleInfo = (MODULE_INFO_TYPE *)malloc(sizeof(MODULE_INFO_TYPE));

  moduleInfo->port = port;

  moduleInfo->status = status;

  moduleInfo->name = name;
  moduleInfo->host = host;

  moduleInfo->vmajor = TCX_VERSION_MAJOR;
  moduleInfo->vminor = TCX_VERSION_MINOR;

  return moduleInfo;
}

/*************************************/

MODULE_PTR moduleCreate(int sd, int port, char *name, char *host, int status)
{
  MODULE_PTR module;

  module = (MODULE_TYPE *)malloc(sizeof(MODULE_TYPE));

  module->sd = sd;

  module->recv = 0;

  module->moduleInfo = moduleInfoCreate(status, port, name, host);

  module->status = status;

  module->pending = listCreate();
  module->connections = listCreate();
  module->oldConnections = listCreate();

  module->itemQ = listCreate();

  listInsertItem(module, Global->moduleListGlobal);

  if (status & STATUS_CONNECTED) {
    FD_SET(module->sd, &(Global->tcxConnectionListGlobal));
  }

  return module;
}

/*************************************/

MODULE_PTR moduleCreate2(int sd, MODULE_INFO_PTR moduleInfo, int status)
{
  MODULE_PTR module;

  module = (MODULE_TYPE *)malloc(sizeof(MODULE_TYPE));

  module->sd = sd;

  module->recv = 0;

  module->moduleInfo = moduleInfo;

  module->status = status;

  module->pending = listCreate();
  module->connections = listCreate();
  module->oldConnections = listCreate();

  module->itemQ = listCreate();

  listInsertItem(module, Global->moduleListGlobal);

  if (status & STATUS_CONNECTED) {
    FD_SET(module->sd, &(Global->tcxConnectionListGlobal));
  }

  return module;
}

/*************************************/

#if 0

void moduleOpen(int sd)
{
  char *buf;
  HDR_TYPE hdr;
  MODULE_PTR module;
  NEW_CON_TYPE newCon;

  if (readAll(sd, &hdr, sizeof(HDR_TYPE)) < 0) {
    fprintf(stderr, "ERROR: iterateActiveMods: readAll 1\n");
    (*(Global->tcxExitHndGlobal))();
  }

  buf = (char *)malloc(ntohInt(hdr.size1));

  if (readAll(sd, buf, ntohInt(hdr.size1)) < 0) {
    fprintf(stderr, "ERROR: iterateActiveMods: readAll 2\n");
    (*(Global->tcxExitHndGlobal))();
  }

  tcxDecodeData(newConFMT, buf, &newCon);
  free(buf);

  module = (MODULE_PTR)listMemReturnItem(findMod, newCon.name, 
					 Global->moduleListGlobal);

  if (!module) {
    module = moduleCreate(sd, newCon.port, newCon.name, newCon.host, 
			  STATUS_CONNECTED);
    listInsertItem(module, Global->moduleListGlobal);
  } else {
    module->sd = sd;
    module->moduleInfo->port = newCon.port;
    module->moduleInfo->host = newCon.host;
    module->status = STATUS_CONNECTED;
  }

  FD_SET(module->sd, &(Global->tcxConnectionListGlobal));

  fprintf(stderr, "New Connection Detected from: %d: %d: %s: %s\n", 
	  module->sd, module->moduleInfo->port, 
	  module->moduleInfo->name, module->moduleInfo->host);
}

#endif

/*****************************/

int iteratePendingDelete(MODULE_PTR module, MODULE_PTR connection)
{
  VERSION_TYPE version;

  if (listMemberItem(module, connection->pending)) {
    listDeleteItem(module, connection->pending);
    if (!listLength(connection->pending)) {    
      printf("e: send version info: %s\n", connection->moduleInfo->name);
      version.vMaj = TCX_VERSION_MAJOR;
      version.vMin = TCX_VERSION_MINOR;
      version.beeSoftMaj = BEESOFT_VERSION_MAJOR;
      version.beeSoftMin = BEESOFT_VERSION_MINOR;
      version.beeSoftRobotType = BEESOFT_VERSION_ROBOT_TYPE;
      tcxSendMsg(connection, "TCXversionInfo", &version);
    }
  }

  return 1;
}


/*****************************/

void moduleClose(MODULE_PTR module)
{
  HND_CONNECT_PTR hndConnect;

/*   fprintf(stderr, "%s:%6d:%s() - Closed Connection Detected from: %s: %s\n", */
/* 	  __FILE__, __LINE__, __FUNCTION__, */
/* 	  module->moduleInfo->name, module->moduleInfo->host); */
  
  if (Global->hndConnectTable) {
    hndConnect = (HND_CONNECT_PTR)hashTableFind(module->moduleInfo->name,
						Global->hndConnectTable);
    if (hndConnect && hndConnect->closeHnd) {
      (*hndConnect->closeHnd)(module->moduleInfo->name); 
    }
   }

  if (Global->tcxCloseHndG) {
    (*(Global->tcxCloseHndG))(module->moduleInfo->name, module); /* need module
								  * S.Thrun
								  * 94-1-12*/
  }

  FD_CLR(module->sd, &(Global->tcxConnectionListGlobal));
  shutdown(module->sd, 2);
  close(module->sd);

  /* 17-Oct-92: fedor: remove from moduleListGlobal? 
     17-Nov-92: fedor: nope - save for reconnects */
  module->status = STATUS_CLOSED;
  module->recv = 0;

  /* 16-Nov-92: fedor attempt stuff for reconnect */
  listFree(module->oldConnections);
  module->oldConnections = module->connections;
  module->connections = listCreate();

  listIterate(iteratePendingDelete, module, module->oldConnections);

  return;
}

/*****************************/

int iterateCloseAll(void *param, MODULE_PTR module)
{
  fprintf(stderr, "Closing Connection: %s: %s\n", 
	  module->moduleInfo->name, module->moduleInfo->host);	  
  FD_CLR(module->sd, &(Global->tcxConnectionListGlobal));
  shutdown(module->sd, 2);
  close(module->sd);

  /* free up some memory - for VxWorks */
  free(module->moduleInfo);

  free(module);

  return 1;
}

void tcxCloseAll()
{
  listIterateFromFirst(iterateCloseAll, NULL, Global->moduleListGlobal);
  listFree(Global->moduleListGlobal);
  Global->moduleListGlobal = listCreate(); /* for restart */
}

/*****************************/

int iterateReconnect2(MODULE_PTR module, MODULE_PTR old)
{
  if (!listMemberItem(old, module->pending) &&
      MODULE_IS_ACTIVE(old) && old->recv) {
    listInsertItem(old, module->pending);
  }

  return 1;
}

void reconnectModule2(MODULE_PTR module)
{
  if (listLength(module->oldConnections)) {
    listIterate(iterateReconnect2, module, module->oldConnections);
  }
}
