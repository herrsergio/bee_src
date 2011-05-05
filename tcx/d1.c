
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
* PROJECT: Carnegie Mellon Erebus Project
*          Task Control Architecture (TCX)
*
* PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
* (c) Copyright 1993 Christopher Fedor. All rights reserved.
* 
* MODULE: d1
*
* FILE: d1.c
*
* ABSTRACT:
* 
* Try porblems with modules crashing d <-> b <-> a
*
* REVISION HISTORY:
*
* $Log: d1.c,v $
* Revision 1.2  1996/12/03 05:38:34  thrun
* If TCXHOST is not se, the software will now assume "localhost" and
* won't terminate.
*
* Revision 1.1.1.1  1996/09/22 16:46:01  rhino
* General reorganization of the directories/repository, fusion with the
* RWI software.
*
* Revision 1.1.1.1  1994/03/31 08:35:04  rhino
*
 * Revision 1.1.1.1  1994/03/17  14:22:28  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:30  rhino
 * test
 *
 * Revision 1.2  1993/03/12  20:58:19  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.1  1992/10/10  03:04:28  fedor
 * Moving toward basing initializing connections on moduleInfo messages.
 *
*
* NOTES:
*  none
*
******************************************************************************/

#include "stdio.h" 
#include "tcx.h"

#include "sampleTest.h"

extern char *getenv(char *);

void charMsgHnd(ref, sampleChar)
TCX_REF_PTR ref;
char *sampleChar;
{
  printf("sampleCharHnd: Start.\n");

  printf("ins: %d\n", tcxRefIns(ref));

  printf("sampleChar: %c\n", *sampleChar);

  printf("sampleCharHnd: End.\n\n");
}

/*************/

TCX_REG_HND_TYPE handlersArray[] = {
  {"charMsg", "charMsgHnd", charMsgHnd, TCX_RECV_ALL, NULL},
};

main()
{
  char *tcxMachine = NULL;
  
  TCX_MODULE_PTR module;

  printf("Connect ...\n");

  tcxMachine = getenv("TCXHOST");   

  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize("d1", tcxMachine);

  /* module = tcxConnectModule("b1");*/

  tcxRegisterMessages(messageArray, 
		      sizeof(messageArray)/sizeof(TCX_REG_MSG_TYPE));

  tcxRegisterHandlers(handlersArray,
		      sizeof(handlersArray)/sizeof(TCX_REG_HND_TYPE));
  
  tcxRecvLoop(NULL);
}

