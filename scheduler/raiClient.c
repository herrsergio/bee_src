
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


 
#ifndef lint
static char rcsid[] = "$Id: raiClient.c,v 1.8 1998/01/17 01:06:21 swa Exp $";
static void *const use_rcsid = ((void)&use_rcsid, (void)&rcsid, 0);
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <tcx.h>
#include <rai.h>
#include <raiClient.h>
#include <bUtils.h>

#include "beeSoftVersion.h"


extern void tcxRegisterCloseHnd(void (*closeHnd)());

/* change this to linked list later */
#define X_LIST_SIZE 200
TCX_REG_MSG_TYPE  registeredMessages[X_LIST_SIZE];
TCX_REG_HND_TYPE  registeredHandlers[X_LIST_SIZE];
int nextMessage = 0;
int nextHandler = 0;


void
registerInterface(char* name, int numMsgs,TCX_REG_MSG_TYPE messages[],
		  int numHnds,TCX_REG_HND_TYPE handlers[])
{
  int i;

  for(i=0;i<numMsgs;i++) {
    if (nextMessage >= X_LIST_SIZE){
      fprintf(stderr, 
	      "Too many entries in raiClient.c. Change X_LIST_SIZE.\n");
      exit(-1);
    }      

    registeredMessages[nextMessage].msgName = strdup(messages[i].msgName);
    
    if (messages[i].msgFormat) {
      registeredMessages[nextMessage].msgFormat =
	strdup(messages[i].msgFormat);
    }
    else {
      registeredMessages[nextMessage].msgFormat = NULL;
    }
    nextMessage++;
  }
  
  for(i=0;i<numHnds;i++) {
    if (nextHandler >= X_LIST_SIZE){
      fprintf(stderr, 
	      "Too many entries in raiClient.c. Change X_LIST_SIZE.\n");
      exit(-1);
    }      
    registeredHandlers[nextHandler].msgName =	strdup(handlers[i].msgName);
    registeredHandlers[nextHandler].hndName = strdup(handlers[i].hndName);
    registeredHandlers[nextHandler].hndProc = handlers[i].hndProc;
    registeredHandlers[nextHandler].hndControl = handlers[i].hndControl;
    registeredHandlers[nextHandler].hndData = handlers[i].hndData;
    nextHandler++;
  }
/*   fprintf(stderr, "[[ %d handlers ]]\n", nextHandler); */
}

void closeClient()
{
 tcxCloseAll();
}


void
shutdownClient()
{
  tcxCloseAll();	
}


void
initClient(char* name,clientCloseFcn clientClose)
{

  if(nextMessage == 0) {
    fprintf(stderr, "RaiClient: initClient(): "
	    "No interface messages registered.\n");  
    return;
  }

  if (!bRobot.TCXHOST) {
    fprintf(stderr, "RaiClient: initClient(): "
	    "TCXHOST not specified.  Assuming 'localhost'.\n");
    tcxInitialize(name, "localhost");
    check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
			 BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
			 NULL, 1);
  

  }
  else {
    tcxInitialize(name,(char *)bRobot.TCXHOST);
    check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
			 BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE, 
			 NULL, 1);
  }

  tcxRegisterMessages(registeredMessages,nextMessage);



  if(nextHandler >0) {
/*     int i; */
/*     for(i=0;i<nextHandler;i++) { */
/*       fprintf(stderr, "<%d: [%s] [%s] %p %p %p>\n", */
/* 	      i,  */
/* 	      registeredHandlers[i].msgName, */
/* 	      registeredHandlers[i].hndName, */
/* 	      registeredHandlers[i].hndProc, */
/* 	      registeredHandlers[i].hndControl, */
/* 	      registeredHandlers[i].hndData); */
/*     } */
/*     fprintf( stderr,"-f-\n"); */
    tcxRegisterHandlers(registeredHandlers,nextHandler);
  }

  tcxRegisterCloseHnd(clientClose);
}

void 
clientTCXCallback(RaiModule * mod)
{
  struct timeval TCX_waiting_time = {0, 10};
  tcxRecvLoop((void *) &TCX_waiting_time); 
}
 
void
initClientModules()
{
  RaiModule *client_module;
  client_module = makeModule("ClientTCXModule",NULL);
  addPolling(client_module,clientTCXCallback,TCX_CLIENT_POLLING_INTERVAL);
}
