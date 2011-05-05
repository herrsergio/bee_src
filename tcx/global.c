
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
* MODULE: globals
*
* FILE: global.c
*
* ABSTRACT:
* 
* A Temporary Global Solution
*
* REVISION HISTORY
*
* $Log: global.c,v $
* Revision 1.2  1997/02/22 15:43:01  thrun
* Fixed some problems that caused compiler warnings.
*
* Revision 1.1.1.1  1996/09/22 16:46:01  rhino
* General reorganization of the directories/repository, fusion with the
* RWI software.
*
* Revision 1.2  1994/10/22 18:47:30  tyson
* VMS version. Fixed structure indexing (computation of the offsets
* in a struct). Added signal handlers to a1, b1 tcxServer.
*
 * Revision 1.1.1.1  1994/03/31  08:35:04  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:29  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:28  rhino
 * test
 *
 * Revision 1.1  1993/03/12  20:58:29  fedor
 * Updated test cases and hid global variables for vxworks
 *
*
******************************************************************************/

#ifdef VMS
#include "vms.h"
#endif

#include "global.h"

extern char *malloc();

G_PTR Global = NULL;

void globalInit()
{
  /* support some random calls for handler connects/disconnects 
   before tcxInitialize */
  /* 12-Mar-93: fedor: this may be a very bad idea - blah1 */

  if (Global) {
    return;
  }

  Global = (G_TYPE *)malloc(sizeof(G_TYPE));

#ifdef VXWORKS
  if (taskVarAdd(0, (int *)&Global) != OK) {
    printErr("taskVarAdd failed\n");
  }
#endif  

  /* formatters.c */
  Global->formatNamesTable = NULL;

  /* lex.c */
  Global->currentLocationGlobal = 0;
  Global->currentStringGlobal = NULL;
  Global->ungetTokenGlobal = NULL;

  /* list.c */
  Global->listFreeListGlobal = NULL;
  Global->listCellFreeListGlobal = NULL;

  /* tcaMem.c */
  Global->totalMemRequest = 0;
  Global->freeMemRetryAmount = 0;
  Global->tcaFreeMemoryHnd = NULL;
  Global->mallocMemRetryAmount = 1;
  Global->tcaMallocMemHnd = (void *) malloc;

  /* parseFmttrs.c */
  Global->parseString = NULL;
  Global->searchFormat = NULL;
  Global->foundKey = NULL;

  /* com.c */
  /* Global->tcxCreateFMT = NULL; ** special */
  Global->closeModGlobal = 0;
  Global->tcxCloseHndG = NULL;
  Global->hndIdTable = NULL;
  Global->hndConnectTable = NULL;
  Global->tcxInitFlagG = 0;
  Global->sigFlagGlobal = 0;
  Global->sigerrorGlobal = 0;
  Global->tcxServerGlobal = NULL;
  Global->tcxModuleGlobal = NULL;
  Global->dataBufGlobal.len = 0;
  Global->dataBufGlobal.buf = NULL;
  Global->dataListGlobal = NULL;
  FD_ZERO(&(Global->tcxConnectionListGlobal));

  /* data.c */
  Global->recvUseHndGlobal = 1;

  /* module.c */
  Global->moduleListGlobal = NULL; 

  /* msg.c */
  Global->tcxRegMsgFlagG = 0;
  Global->msgRefGlobal = 1;
  Global->msgInitFlagGlobal = 0;
  Global->msgQGlobal = NULL;
  Global->msgFreeRefS = NULL;
  Global->msgIdTableGlobal = NULL;
  Global->msgNameTableGlobal = NULL;

  /* reg.c */
  Global->tcxExitHndGlobal = NULL;

  /* tcxServer.c */
  Global->msgIdS = 1000; /* 0 not used start user messages at 1000 */
}

