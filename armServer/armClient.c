
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
static char rcsid[] =
"$Id: armClient.c,v 1.5 1998/01/14 18:59:37 swa Exp $";
static void *const use_rcsid = ((void)&use_rcsid, (void)&rcsid, 0);
#endif

#include <stdio.h>

#include <stdlib.h>
#include <unistd.h>

#include "armServer.h"

#include <rai.h>
#include <tcx.h>
#include <armClient.h>

extern void tcxRegisterCloseHnd(void (*closeHnd)());

#define ARM_CLIENT_HANDLERS  {   \
  {ARM_FIXED_MESSAGE,"armFixed",handleArmClientFixed, TCX_RECV_ALL,NULL}}

TCX_MODULE_PTR  armServer  = NULL;

clientCallback  armClientFcn = NULL;

#define VERBOSE FALSE



/****************** FUNCTIONS FOR SENDING TO THE SERVER *************/

void armSendServerFixed(int operation,unsigned long arg)
{
  RAI_FixedMsgType command;

  if (armConnected) {
    command.operation = operation;
    command.parameter = arg;
    tcxSendMsg(armServer,ARM_FIXED_MESSAGE,&command);
  } else {
    fprintf(stderr,
	    "%s(%s): armServer is not connected. \n", __FILE__, __FUNCTION__);
    armConnect( 0 );
    if (armConnected) {
      command.operation = operation;
      command.parameter = arg;
      tcxSendMsg(armServer,ARM_FIXED_MESSAGE,&command);
    }
  }
}
 
void subscribeToArm(clientCallback fcn)
{
  armSendServerFixed(ARM_subscribe,0);
  if ( armConnected )
    armClientFcn = fcn;
}

void deployArm()
{  
  armSendServerFixed(ARM_deployArm,0);
}

void stowArm()                                               
{ 
  armSendServerFixed(ARM_stowArm,0);
}

void armLimp()
{ 
  armSendServerFixed(ARM_armLimp,0);
}

/*** MAST  **/
void mastLimp()
{ 
  armSendServerFixed(ARM_mastLimp,0);
}

void mastHalt()
{ 
  armSendServerFixed(ARM_mastHalt,0);
}

void mastRelativeUp(unsigned long amount)
{ 
  armSendServerFixed(ARM_mastRelativeUp,amount);
}

void mastRelativeDown(unsigned long amount)
{ 
  armSendServerFixed(ARM_mastRelativeDown,amount);
}

void mastToPos(unsigned long pos)
{ 
  armSendServerFixed(ARM_mastToPos,pos);
}

void mastVelocityDown(void)
{
  armSendServerFixed(ARM_mastVelocityDown,0);
}

void mastVelocityUp(void)
{ 
  armSendServerFixed(ARM_mastVelocityUp,0);
}

void mastWhere(void)
{ 
  armSendServerFixed(ARM_mastWhere,0);
}

/* gripper */
void gripLimp(void)
{ 
  armSendServerFixed(ARM_gripLimp,0);
}

void gripWhere(void)
{ 
  armSendServerFixed(ARM_gripWhere,0);
}

void gripHalt(void)
{
  armSendServerFixed(ARM_gripHalt,0);
}

void gripToPos(unsigned long amount)
{ 
  armSendServerFixed(ARM_gripToPos,amount);
}

void gripRelativeOpen(unsigned long amount)
{ 
  armSendServerFixed(ARM_gripRelativeOpen,amount);
}

void gripRelativeClose(unsigned long amount)
{ 
  armSendServerFixed(ARM_gripRelativeClose,amount);
}

/*** wrist ***/
void wristLimp(void)
{ 
  armSendServerFixed(ARM_wristLimp,0);
}

void wristHalt(void)
{ 
  armSendServerFixed(ARM_wristHalt,0);
}

void wristToPos(unsigned long pos)
{ 
  armSendServerFixed(ARM_wristToPos,pos);
}

void wristRelativePos(unsigned long amount)
{ 
  armSendServerFixed(ARM_wristRelativePos,amount);
}

void wristRelativeNeg(unsigned long amount)
{ 
  armSendServerFixed(ARM_wristRelativeNeg,amount);
}

/****************** HANDLING RESPONSES FROM THE SERVER *************/


void armDispatchFixedMessage(int operation, unsigned long param)
{
  if (armClientFcn != NULL)
    armClientFcn(operation,param);

  if (VERBOSE)
    fprintf(stderr,"armClient received %d,%lu from server\n",operation,param);
}

void handleArmClientFixed(TCX_REF_PTR message,RAI_FixedMsgType *msg_ptr)
{
  int operation;
  unsigned long param;

  operation = msg_ptr->operation;
  param = msg_ptr->parameter;

  if (VERBOSE)
    fprintf(stderr,"armClient fixed msg %d %lu\n",operation,param); 
  armDispatchFixedMessage(operation,param);
  tcxFreeByRef(message,msg_ptr);
}

/* DO NOT USE ANYMORE */
void registerArmClient()
{
  int numberOfMessages;
  int numberOfHandlers;

  TCX_REG_MSG_TYPE messages[] = ARM_MESSAGES; 
  TCX_REG_HND_TYPE handlers[] = ARM_CLIENT_HANDLERS;
  
  numberOfMessages = sizeof(messages)/sizeof(TCX_REG_MSG_TYPE); 
  numberOfHandlers = sizeof(handlers)/sizeof(TCX_REG_HND_TYPE); 

  registerInterface(ARM_SERVER_NAME,numberOfMessages,messages,
		    numberOfHandlers,handlers);
}

/******************* SETUP ***************************/

void armRegister()
{
  int numberOfMessages;
  int numberOfHandlers;

  TCX_REG_MSG_TYPE messages[] = ARM_MESSAGES; 
  TCX_REG_HND_TYPE handlers[] = ARM_CLIENT_HANDLERS;
  
  numberOfMessages = sizeof(messages)/sizeof(TCX_REG_MSG_TYPE); 
  numberOfHandlers = sizeof(handlers)/sizeof(TCX_REG_HND_TYPE); 

  registerInterface(ARM_SERVER_NAME,numberOfMessages,messages,
		    numberOfHandlers,handlers);
}

/* DO NOT USE THIS FUNCTION ANYMORE, use armConnect(); */
void findArmServer()
{
  fprintf(stderr, "ArmClient: Connecting to ArmServer...\n");
  armServer = tcxConnectModule(ARM_SERVER_NAME);
  fprintf(stderr, "ArmClient: Connected.\n");
}

/* ---------------------------------------------------------
 *
 * hook up to armServer. if armServer dies, try reconnecting 
 * every three seconds
 *
 * --------------------------------------------------------*/
int
armConnect( int wait_till_established )
{
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference;

#if ( defined(G_DEBUG_TCX) )
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);

  if (time_difference < 3.0)
    return -1;

  if ( wait_till_established ) { /* 1 */

    fprintf(stderr, "ArmClient: Connecting to Arm server...\n");
    armServer = tcxConnectModule(ARM_SERVER_NAME);
    armConnected = 1;
    fprintf(stderr, "ArmClient: Connected.\n");

  } else {			/* 0 */

/*     fprintf( stderr,"armConnected==%d armServer==%p\n",  */
/* 	     armConnected, armServer ); */

    if ( armConnected == 0 || !armServer ) { /* doppelt haelt besser */
      fprintf(stderr, "ArmClient: Connecting to Arm server...\n");
      armServer  = tcxConnectOptional(ARM_SERVER_NAME);
      if( armServer ) {
	armConnected = 1;
	/* there is no ARM_register_auto_update that we have to send */
	/* arm_subscribe(); */  
	fprintf(stderr, "ArmClient: Connected.\n");
      } else {
	armConnected = 0;
      }
    }

  }

  last_time.tv_sec  = this_time.tv_sec;
  last_time.tv_usec = this_time.tv_usec;

  return 0;
}
