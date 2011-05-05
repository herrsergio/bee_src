
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
 *  MODULE: Data
 *
 *  FILE: data.c
 *
 *  ABSTRACT: Data sending and receiving routines.
 *
 *  EXPORTS:
 *
 *  HISTORY:
 *  $Log: data.c,v $
 *  Revision 1.7  1997/04/12 06:43:57  tyson
 *  TCX segv hacked plus clean-up
 *
 *  Revision 1.6  1997/04/11 18:56:56  tyson
 *  minor fixes and chasing TCX segv
 *
 *  Revision 1.5  1997/04/01 22:30:36  tyson
 *  bugs and stuff
 *
 *  Revision 1.4  1997/03/26 00:50:12  tyson
 *  #$%^&^$ SunOS doesnt support strerror()
 *
 *  Revision 1.3  1997/03/25 21:44:52  tyson
 *  Many bug fixes.
 *
 *  Revision 1.2  1997/02/22 15:43:01  thrun
 *  Fixed some problems that caused compiler warnings.
 *
 *  Revision 1.1.1.1  1996/09/22 16:46:01  rhino
 *  General reorganization of the directories/repository, fusion with the
 *  RWI software.
 *
 *  Revision 1.2  1994/10/22 18:47:26  tyson
 *  VMS version. Fixed structure indexing (computation of the offsets
 *  in a struct). Added signal handlers to a1, b1 tcxServer.
 *
 * Revision 1.1.1.1  1994/03/31  08:35:04  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:29  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:28  rhino
 * test
 *
 * Revision 1.23  1993/03/16  20:10:41  fedor
 * Another bad fix to tcxRecvData
 *
 * Revision 1.22  1993/03/16  16:39:29  fedor
 * Quick fix for tcxRecvData - if an item is found with quick data q check
 * then return it. This will need lots more work soon!
 *
 * Revision 1.21  1993/03/12  20:58:21  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.20  1992/11/30  02:33:35  fedor
 * Flaged items for deletion inside findItem for handler run items.
 * This avoids a list iteration problem for recusively deleting items from a list.
 * Also moved handler masks for tcxRecvData to tcx.h from tcxP.h
 *
 * Revision 1.19  1992/11/13  14:47:12  fedor
 * Update archive with a working version and partially working server route
 *
 * Revision 1.18  1992/10/23  20:15:14  fedor
 * Redid method of connecting to modules. Added the notion of a connection.
 * Added some auto-reconnect for those modules that are probably listening.
 * See detail notes.
 *
 * Revision 1.17  1992/10/14  22:15:48  fedor
 * Error when checking item q and when to do single pass for timeouts and no tests
 * in tcxRecvData. Much more cleaning up needs to be done. Quick fix.
 *
 * Revision 1.16  1992/10/14  18:30:56  fedor
 * Changed tcxRecvData to execute at least a single handler from one connection or
 * at most one handler from each connection and return.
 * Changed tcxRecvLoop to loop here instead of in tcxRecvData - needs clean up.
 *
 * Revision 1.15  1992/10/10  03:04:31  fedor
 * Moving toward basing initializing connections on moduleInfo messages.
 *
 * Revision 1.14  1992/09/14  21:24:33  fedor
 * Changed message hash function to ignore case.
 *
 * Revision 1.13  1992/08/06  12:58:56  fedor
 * Quick reorder of checking queue before checking pending.
 *
 * Revision 1.12  1992/08/04  14:45:51  fedor
 * Debugging fprintf for VxWorks.
 * Added checks for using NULL formatters.
 *
 * Revision 1.11  1992/07/27  07:23:57  fedor
 * Forwarding of messages on tcxSend(NULL - does not handle tcxReply
 *
 * Revision 1.10  1992/07/27  04:02:33  fedor
 * Added registration of handlers to server. Started message forwarding.
 *
 * Revision 1.9  1992/07/20  00:53:45  fedor
 * Redefined tcxRecvData to check pending first. Commented out some debugging.
 *
 * Revision 1.8  1992/07/20  00:31:37  fedor
 * Added receive message style calls. Added data freeing calls.
 *
 * Revision 1.7  1992/07/10  16:04:50  fedor
 * Changes to compile for VxWorks. Also some changes to avoid compiler warnings.
 *
 * Revision 1.6  1992/07/08  15:20:19  fedor
 * Changed message registration to use server generated id values.
 * Added a message handler registration - name may change.
 * Now there are two kinds of handlers one for messages of the form (ref,
 * data, cdata) and the many parameter one (mod, id, ref, data, cdata)
 *
 * Revision 1.5  1992/07/05  15:41:38  fedor
 * Added random makefile.NeXT for NeXT machine use.
 * Changed incorrect use of perror to fprintf(stderr,
 *
 * Revision 1.4  1992/07/05  14:06:43  fedor
 * Added code to call moduleClose on a write fail.
 *
 * Revision 1.3  1992/07/05  13:06:40  fedor
 * Changed printf to fprintf(stderr,
 * Removed old list. Some file reorganization.
 *
 * Revision 1.2  1992/07/03  10:13:14  fedor
 * New module definition and routines
 *
 * Revision 1.1.1.1  1992/06/16  17:22:00  fedor
 * Import TCX 2.0 16-Jun-92
 *
 * Revision 1.3  1992/04/20  10:37:56  fedor
 * *** empty log message ***
 *
 * Revision 1.2  1992/04/20  10:30:26  fedor
 * test file
 *
 * Revision 1.2  1992/04/20  10:30:26  fedor
 * test file
 *
 * Revision 1.1  1992/03/30  07:39:30  fedor
 * Initial revision
 *
 *  
 *  $Revision: 1.7 $
 *  $Date: 1997/04/12 06:43:57 $
 *  $Author: tyson $
 *
 *********************************************************************/

#ifdef VMS
#include "vms.h"
#endif

#ifdef VXWORKS
#include "/usr/vxworks/vx5.0.2b/h/vxWorks.h"
#include "/usr/vxworks/vx5.0.2b/h/stdioLib.h"
#include "/usr/vxworks/vx5.0.2b/h/socket.h"
#include "/usr/vxworks/vx5.0.2b/h/in.h"
#include "/usr/vxworks/vx5.0.2b/h/sigLib.h"
#include "/usr/vxworks/vx5.0.2b/h/errno.h"
#else
#include "stdio.h"
#include "signal.h"
#include "sys/time.h"
#include "sys/types.h"
#include "sys/socket.h"
#include "netinet/in.h"
#include "netinet/tcp.h"
#include "netdb.h"
#include "errno.h"
#include "string.h"
#endif

#include "tcx.h"
#include "tcxP.h"

#include "list.h"
#include "hash.h"
#include "key.h"

#include "formatters.h"

#include "msg.h"
#include "module.h"

#include "global.h"

extern int findMod();
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

void **testPtr=NULL;

/************************************************/

MSG_INS_PTR messageNextInstance() 
{
  MSG_INS_PTR item, item2;

  item2 = NULL;

  item = (MSG_INS_PTR)listFirst(Global->dataListGlobal);

  if (item) {
    item2 = (MSG_INS_TYPE *)malloc(sizeof(MSG_INS_TYPE));
    item2->id = item->id;
    item2->ref = item->ref;
    item2->len = item->len;
    item2->module = item->module;
    item2->encodedData = item->encodedData;

    /* flag item for removal */
    item->id = 0;
  }

  return item2;
}

/************************************************/

void tcxSendData(module, id, ref, data, len, fmt)
TCX_MODULE_PTR module;
int id, ref;
void *data;
int len;
TCX_FMT_PTR fmt;
{
  int total;
  char *sbuf;
  HDR_TYPE hdr;

  if (!module) {
    fprintf(stderr, "ERROR: tcxSendMsg: Module is NULL\n");
    return;
  }

  if (module->status & STATUS_INACTIVE) {
    fprintf(stderr, "ERROR: tcxSendMsg: Module %s Not Connected.\n",
	    module->moduleInfo->name);
    return;
  }
  
  hdr.msg1 = id;
  hdr.ref1 = ref;

  hdr.msg2 = 0;
  hdr.ref2 = 0;
  hdr.size2 = 0;

  if (id == 0) {
    fprintf(stderr, "ERROR: tcxSendData: id value is 0\n");
    (*(Global->tcxExitHndGlobal))();
  }
  
  /*******
    if (!module) {
    fprintf(stderr, "ERROR: tcxSendData: Module is NULL\n");
    (*(Global->tcxExitHndGlobal))();
    }
    *********/

#if (defined(i386) || defined(VMS))
  hdr.msg1 = htonInt(hdr.msg1);	/* (st) S.Thrun 94-1-5 */
  hdr.msg2 = htonInt(hdr.msg2);	/* Conversion for i386 machines */
  hdr.ref1 = htonInt(hdr.ref1);
  hdr.ref2 = htonInt(hdr.ref2);
#endif

  if (!data) {
    hdr.size1 = 0;

    if (writeAll(module->sd, &hdr, sizeof(HDR_TYPE)) < 0) {
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

    return;
  }

  if (fmt) {
    hdr.size1 = bufferSize(fmt, data);
  } else {
    hdr.size1 = len;
  }

  total = hdr.size1+sizeof(HDR_TYPE);

  if (Global->dataBufGlobal.len < total) {
    free(Global->dataBufGlobal.buf);
    sbuf = (char *)malloc(total);
    Global->dataBufGlobal.len = total;
    Global->dataBufGlobal.buf = sbuf;
  } else {
    sbuf = Global->dataBufGlobal.buf;
  }



#if (defined(i386) || defined(VMS))
  hdr.size1 = htonInt(hdr.size1);
  hdr.size2 = htonInt(hdr.size2);
#endif
#if 0
  {				
    printf("#1# %d=[%.2X%.2X%.2X%.2X] %d=[%.2X%.2X%.2X%.2X]"
	    " %d=[%.2X%.2X%.2X%.2X] %d=[%.2X%.2X%.2X%.2X]"
	    " %d=[%.2X%.2X%.2X%.2X] %d=[%.2X%.2X%.2X%.2X]", 
	   hdr.msg1, 
	   *((unsigned char *) (&(hdr.msg1))),
	   *(((unsigned char *) (&(hdr.msg1)))+1),
	   *(((unsigned char *) (&(hdr.msg1)))+2),
	   *(((unsigned char *) (&(hdr.msg1)))+3),
	   hdr.ref1 , 
	   *((unsigned char *) (&(hdr.ref1))),
	   *(((unsigned char *) (&(hdr.ref1)))+1),
	   *(((unsigned char *) (&(hdr.ref1)))+2),
	   *(((unsigned char *) (&(hdr.ref1)))+3),
	   hdr.size1 , 
	   *((unsigned char *) (&(hdr.size1))),
	   *(((unsigned char *) (&(hdr.size1)))+1),
	   *(((unsigned char *) (&(hdr.size1)))+2),
	   *(((unsigned char *) (&(hdr.size1)))+3),
	   hdr.msg2, 
	   *((unsigned char *) (&(hdr.msg2))),
	   *(((unsigned char *) (&(hdr.msg2)))+1),
	   *(((unsigned char *) (&(hdr.msg2)))+2),
	   *(((unsigned char *) (&(hdr.msg2)))+3),
	   hdr.ref2,
	   *((unsigned char *) (&(hdr.ref2))),
	   *(((unsigned char *) (&(hdr.ref2)))+1),
	   *(((unsigned char *) (&(hdr.ref2)))+2),
	   *(((unsigned char *) (&(hdr.ref2)))+3),
	   hdr.size2 , 
	   *((unsigned char *) (&(hdr.size2))),
	   *(((unsigned char *) (&(hdr.size2)))+1),
	   *(((unsigned char *) (&(hdr.size2)))+2),
	   *(((unsigned char *) (&(hdr.size2)))+3));
	   
    fflush(stdout);
  }   /* (S.Thrun 93-5-1) */
#endif

  bcopy(&hdr, sbuf, sizeof(HDR_TYPE));
  sbuf += sizeof(HDR_TYPE);

  if (fmt) {
    encodeData(fmt, data, sbuf, 0);    
  } else {
    bcopy(data, sbuf, len);
/*    printf(" *"); (S.Thrun 93-5-1) */
  }
/*  printf ("\n"); */

  if (writeAll(module->sd, Global->dataBufGlobal.buf, total) < 0) {
#if 0
    fprintf(stderr, "%s:%6d:%s() 2 - ERROR: writeAll() : total=%d errno=%d ",
	    __FILE__, __LINE__, __FUNCTION__, errno, total);
    perror("");
#endif
    if (Global->closeModGlobal || errno == EPIPE) {
      moduleClose(module);
      Global->closeModGlobal = 0;
    } else {
      (*(Global->tcxExitHndGlobal))();
    }
  }
}

/*****************************************************/

MODULE_PTR referenceInformation()
{
  MSG_DEF_PTR msg;
  MSG_INS_PTR item;
  MODULE_PTR module;
  MODULE_INFO_PTR moduleInfo;

  item = messageNextInstance();

  msg = messageFindById(item->id);

  moduleInfo = (MODULE_INFO_PTR)decodeData(msg->msgFormat, 
					   item->encodedData, 0);

  free(item->encodedData);

  module = (MODULE_PTR)listMemReturnItem(findMod, moduleInfo->name,
					 Global->moduleListGlobal);

  if (!module) {
    module = moduleCreate2(0, moduleInfo, STATUS_SERVER_ROUTE);
  } else {
    tcxFreeData(msg->msgFormat, moduleInfo);
  }

  return(module);
}

/*****************************************************/

int findItem(test, item)
ITEM_TEST_PTR test;
MSG_INS_PTR item;
{
  char *data;
  HND_PTR hnd;
  MODULE_PTR module;
  int id, ref, modTest, idTest, refTest, idMatch;

  TCX_REF_PTR refPtr;

  pgmTrace();

  /* 26-Jul-92: fedor */
  /* special cleanup item that has been flagged for deletion */
  /* this is needed because we wish to delete the forwarded message
     from the message queue but list iteration still points to it */
  /* encoded data already assumed to have been freed or in use */

  if (item->id == 0) {
    listDeleteItem(item, Global->dataListGlobal);
    free(item);

    pgmReturn();
    return 0; /* continue through list */
  }
    

  /* initialize test criteria so that we can return with a partial test */
  idTest = 1;
  modTest = 1;
  refTest = 1;

  /* check that we are testing for id for handler execution */
  idMatch = 0;

  if (test->mask & MODULE_TEST) {
    modTest = strKeyEqFunc(test->module->moduleInfo->name, 
			   item->module->moduleInfo->name);
  }

  if (test->mask & ID_TEST) {
    idTest = (test->id == item->id);
    idMatch = idTest;
  }

  if (test->mask & REF_TEST) {
    refTest = (test->ref == item->ref);
  }

  if (!idMatch && Global->recvUseHndGlobal) {

    if (test->hnd && (hnd = (HND_PTR)hashTableFind(&item->id, 
						   Global->hndIdTable))
	&& (test->hnd & hnd->mask) &&  hnd->status != HND_NO_EXEC) {

      if (hnd->control == TCX_RECV_ONCE) {
	hnd->status = HND_NO_EXEC;
      }
      if (hnd->control == TCX_RECV_NONE) {
	Global->recvUseHndGlobal = 0;
      }

      if (!hnd->fmt || hnd->fmt == tcxCreateFMT) {
	data = item->encodedData;
      } else {
	data = (char *)decodeData(hnd->fmt, item->encodedData, 0);
	if (item->encodedData)
	  free(item->encodedData);
      }

      id = item->id;
      ref = item->ref;
      module = item->module;

      /* delete within a list op should still work because of the adt list */ 

      /* 29-Nov-92: fedor:*** this does not work with recusive
	 recvData calls - listMemReturn	will iterate over an already freed
	 item !!!!  

      listDeleteItem(item, Global->dataListGlobal); 
      free(item);
      *************/

      /* 29-Nov-92: fedor - mark item for deletion */
      item->id = 0;

      /* make sure final return test fails on this item */
      idTest = 0;

      /* get correct reference if this is a forwarded msg */
      if (module == Global->tcxServerGlobal && !(hnd->mask & TCX_HND)) {
	module = referenceInformation();

	/* flag server forwarded */
	module->moduleInfo->status |= STATUS_SERVER_FORW;
      }

      if (hnd->type == DataHnd) {
	(*hnd->proc)(module, id, ref, data, hnd->callData);
      }
      else {
	refPtr = messageRefCreate(module, id, ref);
	(*hnd->proc)(refPtr, data, hnd->callData);

	testPtr = (void **)&(module->moduleInfo);

	messageRefFree(refPtr);
      }

      if (module->moduleInfo->status & STATUS_SERVER_FORW) {
	/* remove flag */
	module->moduleInfo->status |= STATUS_SERVER_FORW;
      }

      hnd->status = HND_EXEC;
      Global->recvUseHndGlobal = 1;
      
    }
  }

  return(idTest && modTest && refTest);
}

/******************************************************/

int tcxRecvData(TCX_MODULE_PTR *module,
		int *id, 
		int *ref,
		void *data,
		TCX_FMT_PTR fmt,
		int allowHnd,
		void *timeout)
{
  fd_set readMask;
  MSG_INS_PTR item;
  ITEM_TEST_TYPE test;
  int stat, mask, testMask;

  char *r2;
  int amount;

  pgmTrace();

  /*   printf("{");fflush(stdout);(S.Thrun 93-5-1) */

  test.hnd = allowHnd;

  mask = 0;
  testMask = 0;
  /* printf("-a-");fflush(stdout);(S.Thrun 93-5-1) */

  if (module) {
    if (*module) {
      mask |= MODULE_TEST;
      testMask |= mask;
      test.module = *module;
    }
    else {
      mask |= MOD_RETURN;
    }
  }
  /* printf("-b-");fflush(stdout);(S.Thrun 93-5-1) */

  if (id) {
    if (*id) {
      mask |= ID_TEST;
      testMask |= mask;
      test.id = *id;
    }
    else {
      mask |= ID_RETURN;
    }
  }

  if (ref) {
    if (*ref) {
      mask |= REF_TEST;
      testMask |= mask;
      test.ref = *ref;
    }
    else {
      mask |= REF_RETURN;
    }
  }

  test.mask = mask;
  /* printf("-c-");fflush(stdout);(S.Thrun 93-5-1) */


  /* 23-Oct-92: fedor: all of this is silly special case code which needs
     to be rethought and documented */

  if (testMask) {
    /* printf("-d-");fflush(stdout);(S.Thrun 93-5-1) */
    item = (MSG_INS_PTR)listMemReturnItem(findItem, &test, 
					  Global->dataListGlobal);
    /* printf("-e-");fflush(stdout);(S.Thrun 93-5-1) */

    while (!item) {
      stat = singlePassActiveModules(timeout);
      /* printf("-f-");fflush(stdout);(S.Thrun 93-5-1) */
      if (stat == 0) {
	/* 	printf("a}");fflush(stdout);(S.Thrun 93-5-1) */
	pgmReturn();
	return -1; /* timeout */
      }
      
      if (stat < 0) {
	/* printf("-g-");fflush(stdout);(S.Thrun 93-5-1) */
	fprintf(stderr, "ERROR: tcxRecvData: singlePassActiveModules\n");
	(*(Global->tcxExitHndGlobal))();
      }
      
      /* printf("-h-");fflush(stdout);(S.Thrun 93-5-1) */
      item = (MSG_INS_PTR)listMemReturnItem(findItem, &test, 
					    Global->dataListGlobal);
    }

    /* printf("-i-");fflush(stdout);(S.Thrun 93-5-1) */
    listDeleteItem(item, Global->dataListGlobal);

  } else {
    /* printf("-j-");fflush(stdout);(S.Thrun 93-5-1) */

    item = NULL;

    /* printf("-k-");fflush(stdout);(S.Thrun 93-5-1) */
    if (timeout) {
      /* run q once */
      /* printf("-l-");fflush(stdout);(S.Thrun 93-5-1) */
      item = (MSG_INS_PTR)listMemReturnItem(findItem, &test, 
					    Global->dataListGlobal);
      /* printf("-m-");fflush(stdout);(S.Thrun 93-5-1) */
    } else if (listLength(Global->dataListGlobal)) {
      /* null timeout but make sure we can run items still in q */
      /* printf("-n-");fflush(stdout);(S.Thrun 93-5-1) */
      item = (MSG_INS_PTR)listMemReturnItem(findItem, &test, 
					    Global->dataListGlobal);
      /* printf("-o-");fflush(stdout);(S.Thrun 93-5-1) */
    }


    /* 16-Mar-93: fedor: more of a mess - another quick fix to return item
       if found even though this was orginally only going to run some 
       handlers. */

    if (!item) {
      /* printf("-p-");fflush(stdout);(S.Thrun 93-5-1) */
      stat = singlePassActiveModules(timeout);
      /* printf("-q-");fflush(stdout);(S.Thrun 93-5-1) */
      if (stat == 0) {
	/* 	printf("b}");fflush(stdout);(S.Thrun 93-5-1) */
	pgmReturn();
	return -1; /* timeout */
      }
      
      if (stat < 0) {
	/* printf("-r-");fflush(stdout);(S.Thrun 93-5-1) */
	fprintf(stderr, "ERROR: tcxRecvData: singlePassActiveModules\n");
	/* printf("-s-");fflush(stdout);(S.Thrun 93-5-1) */
	(*(Global->tcxExitHndGlobal))();
      }
      
      /* printf("-t-");fflush(stdout);(S.Thrun 93-5-1) */
      item = (MSG_INS_PTR)listMemReturnItem(findItem, &test, 
					    Global->dataListGlobal);
      
      /* printf("-u-");fflush(stdout);(S.Thrun 93-5-1) */
      listDeleteItem(item, Global->dataListGlobal);
      
      if (!item) {
	/* 	printf("c}");fflush(stdout);(S.Thrun 93-5-1) */
	pgmReturn();
	return -2; /* executed a handler for a message and return */
      }
    } else {
      /* 16-Mar-93: fedor: eeeks this is awful stuff!! */
      /* printf("-v-");fflush(stdout);(S.Thrun 93-5-1) */
      listDeleteItem(item, Global->dataListGlobal);      
    }
  }
  /* printf("-w-");fflush(stdout);(S.Thrun 93-5-1) */


  if (mask & MOD_RETURN)
    *module = item->module;

  if (mask & ID_RETURN)
    *id = item->id;

  if (mask & REF_RETURN)
    *ref = item->ref;

/*printf(" <id:%d ref:%d len:%d>\n", item->id,  item->ref, item->len);(S.Thrun 93-5-1) */
  if (!data) {
    amount = item->len;
    free(item);
    /*     printf("e}");fflush(stdout);(S.Thrun 93-5-1) */
    pgmReturn();
    return(amount);
  }
  if (fmt == tcxCreateFMT) {
    bcopy(&(item->encodedData), data, sizeof(char *));
    amount = item->len;
    free(item);
    /*     printf("f}");fflush(stdout);(S.Thrun 93-5-1) */
    pgmReturn();
    return(amount);
  }

  if (fmt) {
    if (item->encodedData) {
      r2 = (char *)decodeData(fmt, item->encodedData, 0);

      if (mask & ID_RETURN) {
	bcopy(&r2, data, sizeof(char *));
      } else {
	amount = dataStructureSize(fmt);
	bcopy(r2, data, amount);
	free(r2);
      }

      if (item->encodedData)
	free(item->encodedData);

    } else {
      data = NULL;
    }
  } else {

    if (mask & ID_RETURN) {
      bcopy(&(item->encodedData), data, sizeof(char *)); 
    } else {
      bcopy(item->encodedData, data, item->len);
      if (item->encodedData)
	free(item->encodedData);
    }
  }

  amount = item->len;
  free(item);

    
  /*   printf("g}");fflush(stdout);(S.Thrun 93-5-1) */
  pgmReturn();
  return amount;
}

/*****************************************************/

int freeItem(param, item)
char *param;
MSG_INS_PTR item;
{
  if (item) {
    if (item->encodedData)
      free(item->encodedData);
    free(item);
  }

  return 1;
}

/********************************/

void tcxRecvFlush(module, id)
MODULE_PTR module;
int id;
{
  int mask, tmp;
  MSG_INS_PTR item;
  ITEM_TEST_TYPE test;

  mask = 0;

  if (module)
    mask |= MODULE_TEST;

  if (id)
    mask |= ID_TEST;

  if (mask) {
    test.module = module;
    test.id = id;
    test.mask = mask;

    tmp = Global->recvUseHndGlobal;
    Global->recvUseHndGlobal = 0; /* force recv handlers off for flushing */

    item = (MSG_INS_PTR)listMemReturnItem(findItem, &test, Global->dataListGlobal);
    while (item) {
      listDeleteItem(item, Global->dataListGlobal);
      if (item->encodedData)
	free(item->encodedData);
      free(item);      
      item = (MSG_INS_PTR)listMemReturnItem(findItem, &test, Global->dataListGlobal);
    }

    Global->recvUseHndGlobal = tmp; /* reset recv handlers */

  } else {
    /* free the entire recv queue */
    listIterateFromFirst(freeItem, NULL, Global->dataListGlobal);
    listFree(Global->dataListGlobal);
    Global->dataListGlobal = listCreate();
  }
}

/********************************/
