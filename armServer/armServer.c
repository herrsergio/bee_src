
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
"$Id: armServer.c,v 1.15 1998/01/14 18:59:38 swa Exp $";
static void *const use_rcsid = ((void)&use_rcsid, (void)&rcsid, 0);
#endif

/*
 * Copyright 1994, Brown University, Providence, RI
 *
 * Permission to use and modify this software and its documentation for
 * any purpose other than its incorporation into a commercial product is
 * hereby granted without fee.  Permission to copy and distribute this
 * software and its documentation only for non-commercial use is also
 * granted without fee, provided, however, that the above copyright notice
 * appear in all copies, that both that copyright notice and this permission
 * notice appear in supporting documentation, that the name of Brown
 * University not be used in advertising or publicity pertaining to
 * distribution of the software without specific, written prior permission,
 * and that the person doing the distribution notify Brown University of
 * such distributions outside of his or her organization. Brown University
 * makes no representations about the suitability of this software for
 * any purpose.  It is provided "as is" without express or implied warranty.
 * Brown University requests notification of any modifications to this
 * software or its documentation.
 *
 * Send the following redistribution information:
 *
 *	Name:
 *	Organization:
 *	Address (postal and/or electronic):
 *
 * To:
 *	Software Librarian
 *	Computer Science Department, Box 1910
 *	Brown University
 *	Providence, RI 02912
 *
 *		or
 *
 *	brusd@cs.brown.edu
 *
 * We will acknowledge all electronic notifications.
 */
#include <signal.h>
#include <rai.h>
#include <arm.h>
#include <utils.h>
#include <bUtils.h>
#include <lockNames.h>
#include <armServer.h>

#include "beeSoftVersion.h"
#include "libbUtils.h"
#include "libutils.h"
#include "libmcp.h"
#include "librai.h"



#define ARM_HANDLERS {{ARM_FIXED_MESSAGE,ARM_FIXED_MESSAGE,handleArmCmd,TCX_RECV_ALL,NULL}}

/**********************************************************************        
                                                                               
  Right now, we are assuming there is only one client, and when                
  it subscribes we record its TCX module so we can send messages back.         
                                                                               
**********************************************************************/        
#define TCX_MAX_CLIENTS 100

TCX_MODULE_PTR CLIENT[TCX_MAX_CLIENTS];

extern void tcxRegisterCloseHnd(void (*closeHnd)());

unsigned char verbose = FALSE;


void sendClientFixed(int operation, unsigned long arg)           
{                                                                
  RAI_FixedMsgType command;                                       
  int ii;
  
  if (verbose) {
    fprintf(stderr,"Arm server sending %d, %ld\n",operation,arg);
  }
  
  for (ii=0; ii<TCX_MAX_CLIENTS; ii++) {
    if (CLIENT[ii] != NULL) {
      command.operation = operation;                                  
      command.parameter = arg;                                        
      tcxSendMsg(CLIENT[ii], ARM_FIXED_MESSAGE, &command);                 
    }
  }
}                                                                


void armTCXCallback(RaiModule * mod)
{
   struct timeval TCX_waiting_time = {0, 10};
   tcxRecvLoop((void *) &TCX_waiting_time); 
 }

void initArmServerModule()
{
  RaiModule * arm_module;
  arm_module = makeModule("armServer",NULL);
  addPolling(arm_module,armTCXCallback,150);
}


void armClose(char *moduleName, TCX_MODULE_PTR module)
{
  int ii;

  fprintf( stderr, "%s(%s): %s died. \n", 
	   __FILE__, __FUNCTION__, moduleName );

  if (!strcmp(moduleName, "TCX Server")){ /* TCX shut down */
    fprintf( stderr, "Exiting.\n" );
    armLimp();
    RaiShutdown();
    exit(0);
  }

  for (ii=0; ii<TCX_MAX_CLIENTS; ii++) {
    if (CLIENT[ii] == module) {
      CLIENT[ii] = NULL;
      fprintf(stderr, "Client %s was dropped from the list.\n",
	      moduleName);
    }
  }

  for (ii=0; ii<TCX_MAX_CLIENTS; ii++) {
    if (CLIENT[ii] != NULL) {
      return;
    }
  }

  fprintf(stderr, "%s:%6d:%s() - shutting off arm motors.\n",
          __FILE__, __LINE__, __FUNCTION__);
  
  armLimp();
}

void dispatchArmCmd(int cmd,int parameter) 
{
  switch(cmd)
   {
 
    case ARM_deployArm: deployArm(); break;
    case ARM_stowArm: stowArm(); break;
    case ARM_armLimp: armLimp(); break;

    case ARM_mastLimp: mastLimp(); break;
    case ARM_mastHalt: mastHalt(); break;
    case ARM_mastRelativeUp:  mastRelativeUp(parameter); break;
    case ARM_mastRelativeDown:  mastRelativeDown(parameter); break;
    case ARM_mastToPos:  mastToPos(parameter); break;
    case ARM_mastVelocityDown:  mastVelocityDown(parameter); break;
    case ARM_mastVelocityUp: mastVelocityUp(parameter); break; 
    case ARM_mastWhere: mastWhere(); break; 

    case ARM_gripLimp: gripLimp(); break; 
    case ARM_gripHalt: gripHalt(); break; 
    case ARM_gripToPos: gripToPos(parameter); break; 
    case ARM_gripRelativeOpen: gripRelativeOpen(parameter); break; 
    case ARM_gripRelativeClose: gripRelativeClose(parameter); break; 
    case ARM_gripWhere: gripWhere(); break; 

    case ARM_wristLimp: wristLimp(); break; 
    case ARM_wristHalt: wristHalt(); break; 
    case ARM_wristToPos: wristToPos(parameter); break; 
    case ARM_wristRelativePos: wristRelativePos(parameter); break; 
    case ARM_wristRelativeNeg: wristRelativeNeg(parameter); break; 
    }

}


void handleArmCmd(TCX_REF_PTR message,RAI_FixedMsgType *message_ptr)
{
  int operation;
  unsigned long parameter;
  int ii;

  operation = message_ptr->operation;
  parameter = message_ptr->parameter;

  if (verbose)
    fprintf(stderr,"arm received %d %lu\n", operation,parameter);

  if(operation== ARM_subscribe) {
    for (ii=0; ii<TCX_MAX_CLIENTS; ii++) {
      if (CLIENT[ii] == NULL) {
        CLIENT[ii]=message->module;
        break;
      }
    }
  }
  else {
    dispatchArmCmd(operation, parameter);
  }

  tcxFreeByRef(message,message_ptr);
}

void initArmServerTCX(const char *tcxMachine)
{
  TCX_REG_MSG_TYPE messageArray[] = ARM_MESSAGES;
  TCX_REG_HND_TYPE handlerArray[] = ARM_HANDLERS;
  int numberOfHandlers;
  int numberOfMessages;  
  
  numberOfHandlers = sizeof(handlerArray)/sizeof(TCX_REG_HND_TYPE); 
  numberOfMessages = sizeof(messageArray)/sizeof(TCX_REG_MSG_TYPE); 

  tcxInitialize(ARM_SERVER_NAME,(char *)tcxMachine);
  check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
		       BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
		       NULL, 0);
  check_version_number(librai_major, librai_minor,
		       librai_robot_type, librai_date,
		       "librai", 0);
  check_version_number(libmcp_major, libmcp_minor,
		       libmcp_robot_type, libmcp_date,
		       "libmcp", 0);
  check_version_number(libutils_major, libutils_minor,
		       libutils_robot_type, libutils_date,
		       "libutils", 0);
  check_version_number(libbUtils_major, libbUtils_minor,
		       libbUtils_robot_type, libbUtils_date,
		       "libbUtils", 1);







  tcxRegisterMessages(messageArray,numberOfMessages);
  tcxRegisterHandlers(handlerArray,numberOfHandlers);
  tcxRegisterCloseHnd(armClose);
  if (verbose)
    fprintf(stderr,"armServer Registered\n");

}

void dispatchArmResponse(armMessage event, unsigned long value)
{
  sendClientFixed(event,value);                          	
}

void ctrlcShutdown() {

  fprintf( stderr, "%s(%s): %s died. \n", 
	   __FILE__, __FUNCTION__, "Ctrl-C" );

  fprintf( stderr, "Exiting.\n" );

  RaiShutdown();

  exit( -1 );

}

int main(int argc, char* argv[]) 
{
  struct bParamList * paramList = NULL;
  const char *mastDev = NULL;
  const char *gripDev = NULL;
  speed_t mastBaudRate = 0;
  speed_t gripBaudRate = 0;
  const char *strPtr;
  int ii;

  for (ii=0; ii<TCX_MAX_CLIENTS; ii++) {
    CLIENT[ii] = NULL;
  }

  /*
   * Set some parameters
   */

  paramList = bParametersAddEntry(paramList, "", "TCXHOST", "localhost");
  paramList = bParametersAddEntry(paramList, "", "fork", "yes");
  paramList = bParametersAddEntry(paramList, "*", "arm.mast.bps", "9600");
  paramList = bParametersAddEntry(paramList, "*", "arm.mast.dev", "/dev/cur4");
  paramList = bParametersAddEntry(paramList, "*", "arm.grip.bps", "9600");
  paramList = bParametersAddEntry(paramList, "*", "arm.grip.dev", "/dev/cur5");

  paramList = bParametersAddFile(paramList, "etc/beeSoft.ini");

  paramList = bParametersAddEnv(paramList, "", "TCXHOST");

  paramList = bParametersAddArray(paramList, "", argc, argv);

  bParametersFillParams(paramList);

  strPtr = bParametersGetParam(paramList, "robot", "arm");

  if (!strPtr) {
    fprintf(stderr,
	    "Arm name not set.  Check the robot.arm\n"
	    "value in beeSoft.ini.\n");
    exit(1);
  }
  else if (!strcmp("none", strPtr)) {
    fprintf(stderr,
	    "Arm name set to 'none'.  Check the robot.arm\n"
	    "value in beeSoft.ini.\n");
    exit(1);
  }

  strPtr = bParametersGetParam(paramList, strPtr, "arm.type");

  if (!strPtr) {
    fprintf(stderr,
	    "Arm type not set.  Check the \n"
	    "value in beeSoft.ini.\n");
    exit(1);
  }
  else if (strcmp("B21-arm", strPtr)) {
    fprintf(stderr,
	    "Arm type set to unrecognized type '%s'.  Check the\n"
	    "value in beeSoft.ini.\n", strPtr);
    exit(1);
  }
  
  mastBaudRate = bRobot.arm_mast_bps;
  gripBaudRate = bRobot.arm_grip_bps;
  mastDev      = bRobot.arm_mast_dev;
  gripDev      = bRobot.arm_grip_dev;

  /*
   *
   */

  if (makeLock(ARM_SERVER_LOCK)<0)
    {
      fprintf(stderr, "%s: Already running ArmServer.\n", argv[0]);
      exit(1);
    }
  if (argc > 1)
    {
      verbose = TRUE;
      fprintf(stderr,"armServer started in verbose mode\n");
    }

  if (bRobot.fork) {
    bDaemonize("armServer");
  }

  initArmServerTCX(bRobot.TCXHOST);
  RaiInit();
  catchInterrupts();
  signal( SIGINT, &ctrlcShutdown ); /* whenever user hits CTRL-C */
  initArmServerModule();		
  ArmInit(mastDev, mastBaudRate, gripDev, gripBaudRate);
  registerArmEventHandler(dispatchArmResponse);
  printf("Arm server initialized\n");

  RaiStart();
  exit(0);
}
