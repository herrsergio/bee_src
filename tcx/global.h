
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
* FILE: global.h
*
* ABSTRACT:
* 
* A Temporary Global Solution
*
* REVISION HISTORY
*
* $Log: global.h,v $
* Revision 1.1.1.1  1996/09/22 16:46:02  rhino
* General reorganization of the directories/repository, fusion with the
* RWI software.
*
* Revision 1.3  1994/10/22 18:47:07  tyson
* VMS version. Fixed structure indexing (computation of the offsets
* in a struct). Added signal handlers to a1, b1 tcxServer.
*
 * Revision 1.2  1994/05/31  21:00:47  rhino
 * General reorganization. New header file. New makefiles.
 *
 * Revision 1.1.1.1  1994/03/31  08:35:03  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:26  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:27  rhino
 * test
 *
 * Revision 1.1  1993/03/12  20:58:32  fedor
 * Updated test cases and hid global variables for vxworks
 *
*
******************************************************************************/

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
#include "sys/timeb.h"
#include "sys/types.h"
#include "sys/socket.h"
#include "netinet/in.h"
#include "netinet/tcp.h"
#include "netdb.h"
#include "errno.h"

extern int errno;

#endif  /* !VXWORKS */

#ifdef i386
#include <malloc.h>
#endif


#include "tcx.h"
#include "tcxP.h"

#include "hash.h"
#include "lex.h"
#include "list.h"
#include "formatters.h"

/***********************************************/

typedef struct {
  /* formatters.c */
  HASH_TABLE_PTR formatNamesTable;

  /* lex.c */
  int currentLocationGlobal;
  char *currentStringGlobal;
  TOKEN_PTR ungetTokenGlobal;

  /* list.c */
  LIST_PTR listFreeListGlobal;
  LIST_ELEM_PTR listCellFreeListGlobal;

  /* tcaMem.c */
  unsigned totalMemRequest;
  int freeMemRetryAmount;
  void (*tcaFreeMemoryHnd)();
  int mallocMemRetryAmount;
  char *(*tcaMallocMemHnd)();

  /* parseFmttrs.c */
  char *parseString;
  FORMAT_PTR searchFormat;
  char *foundKey;

  /* com.c */
  /** TCX_FMT_PTR tcxCreateFMT; ** special value */
  void (*tcxCloseHndG)();
  HASH_TABLE_PTR hndIdTable;
  HASH_TABLE_PTR hndConnectTable;
  int sigFlagGlobal;
  int sigerrorGlobal;
  int closeModGlobal;
  TCX_MODULE_PTR tcxServerGlobal;
  TCX_MODULE_PTR tcxModuleGlobal;
  DATA_BUF_TYPE	dataBufGlobal;
  LIST_PTR dataListGlobal;
  fd_set tcxConnectionListGlobal;
  int tcxInitFlagG;

  /* module.c */
  int recvUseHndGlobal;

  /* data.c */
  LIST_PTR moduleListGlobal;
  int tcxRegMsgFlagG;
  int msgRefGlobal;
  int msgInitFlagGlobal;
  LIST_PTR msgQGlobal;
  LIST_PTR msgFreeRefS;
  HASH_TABLE_PTR msgIdTableGlobal;
  HASH_TABLE_PTR msgNameTableGlobal;
  
  /* reg.c */
  void (*tcxExitHndGlobal)();

  /* tcxServer.c */
  int msgIdS;

} G_TYPE, *G_PTR;

extern G_PTR Global;
extern void globalInit();
