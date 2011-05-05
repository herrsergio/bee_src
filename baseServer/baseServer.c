
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
static char rcsid[] = "$Id: baseServer.c,v 1.26 1997/07/30 23:27:43 thrun Exp $";
static void *const use_rcsid = ((void)&use_rcsid, (void)&rcsid, 0);
#endif

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <string.h>
#include <sys/time.h>

#include <bUtils.h>
#include <rai.h>
#include <base.h>
#include <lockNames.h>
#include <utils.h>
#include <sonarClient.h>
#include <irClient.h>
#include <tactileClient.h>
#include <baseServer.h>
#include <tcx.h>
#include <tcxP.h>

#define TCX_define_variables
#include <SIMULATOR-messages.h>
#undef  TCX_define_variables

#include "beeSoftVersion.h"
#include "libbUtils.h"
#include "libabus.h"
#include "libutils.h"
#include "libmsp.h"
#include "libmcp.h"
#include "libbase.h"
#include "librai.h"



#define BASE_HANDLERS  {   \
  {BASE_FIXED_MESSAGE, \
     "BaseFxd", \
     handleBaseFixed, \
     TCX_RECV_ALL, \
     NULL}, \
  {BASE_VARIABLE_MESSAGE, \
     "BaseVar", \
     handleBaseVariable, \
     TCX_RECV_ALL, \
     NULL}, \
  {"SIMULATOR_message_to_baseServer", \
     "SIMULATOR_message_to_baseServer_handler", \
     simulatorBaseServerHandler, \
     TCX_RECV_ALL, \
     NULL}, \
  {"SIMULATOR_message_to_sonarServer", \
     "SIMULATOR_message_to_sonarServer_handler", \
     simulatorSonarServerHandler, \
     TCX_RECV_ALL, \
     NULL}, \
  {"SIMULATOR_message_to_tactileServer", \
     "SIMULATOR_message_to_tactileServer_handler", \
     simulatorTactileServerHandler, \
     TCX_RECV_ALL, \
     NULL}, \
  {"SIMULATOR_message_to_irServer", \
     "SIMULATOR_message_to_irServer_handler", \
     simulatorIrServerHandler, \
     TCX_RECV_ALL, \
     NULL} \
}

int VERBOSE=FALSE;
int IRS=TRUE;            /* do we use irs or not */
int TACTILES=TRUE;       /* do we use irs or not */
int SONARS=TRUE;         
int useSimulator=FALSE;

extern void tcxRegisterCloseHnd(void (*closeHnd)());


/* Globals for the odometry offsets */
float bOriginX = 0.0;
float bOriginY = 0.0;
float bOriginHeading = 0.0;
int bOdometryLock = -1;
unsigned short bOdometryLockPriority = 0;
int bRequesterIndex;

void changeOdometry(unsigned long *data);

void requestOdometryLock(unsigned short priority);
void releaseOdometryLock();

void confirmCommandToClient(int operation, unsigned long param);

/**********************************************************************

  Right now, we are assuming there is only one client, and when
  it subscribes we record its TCX module so we can send messages back.

**********************************************************************/

#define TCX_MAX_CLIENTS 100

TCX_MODULE_PTR CLIENT[TCX_MAX_CLIENTS];

TCX_MODULE_PTR simulatorHandle = NULL;

/*------------------------------------------------------------*/

void
SIM_sonarStart(void)
{
  if (useSimulator==FALSE) {
    sonarStart();
    return;
  }
  else if (simulatorHandle) {
    char message[132];
    
    message[0] = 1;
#if 0
    {struct timeval TCX_no_time = {0, 0};tcxRecvLoop((void *)(&TCX_no_time));}
#endif
    tcxSendMsg(simulatorHandle, 
	       "SIMULATOR_message_from_sonarServer", 
	       message);
    return;
  }
} 

/*------------------------------------------------------------*/

void
SIM_sonarStop(void)
{
  if (useSimulator==FALSE) {
    sonarStop();
    return;
  }
  else if (simulatorHandle) {
    char message[132];
    
    message[0] = 0;
#if 0
    {struct timeval TCX_no_time = {0, 0};tcxRecvLoop((void *)(&TCX_no_time));}
#endif
    tcxSendMsg(simulatorHandle, 
	       "SIMULATOR_message_from_sonarServer", 
	       message);
    return;
  }
}

/*------------------------------------------------------------*/

void
SIM_irStart(void)
{
  if (useSimulator==FALSE) {
    irStart();
    return;
  }
  else if (simulatorHandle) {
    char message[132];
    
    message[0] = 1;
#if 0
    {struct timeval TCX_no_time = {0, 0};tcxRecvLoop((void *)(&TCX_no_time));}
#endif
    tcxSendMsg(simulatorHandle, 
	       "SIMULATOR_message_from_irServer", 
	       message);
    return;
  }
} 

/*------------------------------------------------------------*/

void
SIM_irStop(void)
{
  if (useSimulator==FALSE) {
    irStop();
    return;
  }
  else if (simulatorHandle) {
    char message[132];
    
    message[0] = 0;
#if 0
    {struct timeval TCX_no_time = {0, 0};tcxRecvLoop((void *)(&TCX_no_time));}
#endif
    tcxSendMsg(simulatorHandle, 
	       "SIMULATOR_message_from_irServer", 
	       message);
    return;
  }
}

/*------------------------------------------------------------*/

void
SIM_tactileStart(void)
{
  if (useSimulator==FALSE) {
    tactileStart();
    return;
  }
  else if (simulatorHandle) {
    char message[132];
    
    message[0] = 1;
#if 0
    {struct timeval TCX_no_time = {0, 0};tcxRecvLoop((void *)(&TCX_no_time));}
#endif
    tcxSendMsg(simulatorHandle, 
	       "SIMULATOR_message_from_tactileServer", 
	       message);
    return;
  }
} 

/*------------------------------------------------------------*/

void
SIM_tactileStop(void)
{
  if (useSimulator==FALSE) {
    tactileStop();
    return;
  }
  else if (simulatorHandle) {
    char message[132];
    
    message[0] = 0;
#if 0
    {struct timeval TCX_no_time = {0, 0};tcxRecvLoop((void *)(&TCX_no_time));}
#endif
    tcxSendMsg(simulatorHandle, 
	       "SIMULATOR_message_from_tactileServer", 
	       message);
    return;
  }
}

/*------------------------------------------------------------*/

void
addBaseClient(TCX_MODULE_PTR newClient)
{
  int ii;

  if (SONARS) {
    SIM_sonarStart();
  }
  
  if (IRS) {
    SIM_irStart();
  }

  if (!strcmp(tcxModuleName(newClient), "RHINO")) {
    fprintf(stderr, "Connection from CONTROLLER\n");
  }
  else {
    for (ii=0; ii<TCX_MAX_CLIENTS; ii++) {
      if (CLIENT[ii] == NULL) {
	CLIENT[ii]=newClient;
	break;
      }
    }
  }
}

/*------------------------------------------------------------*/

void
dropBaseClient(TCX_MODULE_PTR dropClient)
{
  int ii;

  fprintf(stderr, "%s:%6d:%s()\n", __FILE__, __LINE__, __FUNCTION__);

  for (ii=0; ii<TCX_MAX_CLIENTS; ii++) {
    if (CLIENT[ii] == dropClient) {
      CLIENT[ii] = NULL;
      fprintf(stderr, "Client was dropped from the list.\n");

      /* If the dropped client was holding the odometry lock, then */
      /* free it up for someone else to grab.                      */
      if (ii == bOdometryLock)
        releaseOdometryLock();
    }
  }

  fprintf(stderr, "%s:%6d:%s()\n", __FILE__, __LINE__, __FUNCTION__);

  for (ii=0; ii<TCX_MAX_CLIENTS; ii++) {
    if (CLIENT[ii] != NULL) {
      return;
    }
  }

  fprintf(stderr, "%s:%6d:%s() - shutting off sensors and base\n",
	  __FILE__, __LINE__, __FUNCTION__);

  watchdogTimer(0); /* turn off timer */
  rotateLimp(); 
  translateLimp(); 
  SIM_sonarStop();
  SIM_irStop();
}

/*------------------------------------------------------------*/

void
baseTCXCallback(RaiModule * mod)
{
  struct timeval TCX_waiting_time = {0, 10};
  tcxRecvLoop((void *) &TCX_waiting_time); 
}
 

/*------------------------------------------------------------*/

void
shutdownBaseServer(RaiModule* mod)
{
  fprintf(stderr, "BaseServer: Shutting down...\n");
  watchdogTimer(0); /* turn off timer */
  rotateLimp(); 
  translateLimp(); 
  SIM_sonarStop();
  SIM_irStop();
}

/*------------------------------------------------------------*/

void
initBaseServerModules()
{
  RaiModule * base_module;

  base_module = makeModule(BASE_SERVER_NAME,shutdownBaseServer);
  addPolling(base_module,baseTCXCallback,TCX_SERVER_POLLING_INTERVAL);
}

/*------------------------------------------------------------*/

/***  FOR SENDING MESSAGES TO CLIENT ***/

void
sendClientFixed(int operation, unsigned long arg)
{
  RAI_FixedMsgType command;
  int ii;

  command.operation = operation;
  command.parameter = arg;
#if 0
  {struct timeval TCX_no_time = {0, 0};tcxRecvLoop((void *)(&TCX_no_time));}
#endif
  for (ii=0; ii<TCX_MAX_CLIENTS; ii++) {
    if (CLIENT[ii] != NULL) {
      tcxSendMsg(CLIENT[ii], BASE_FIXED_MESSAGE, &command);
    }
  }
}

/*------------------------------------------------------------*/

void
sendClientVariable(int operation, int bufsize, char buffer[])
{
  RAI_VariableMsgType command;
  int ii;

  if (VERBOSE) {
    fprintf(stderr,"Sending op %d to client\n",operation);
  }
  
  command.operation = operation;
  command.bufsize = bufsize;
  command.buffer = buffer;
#if 0
  {struct timeval TCX_no_time = {0, 0};tcxRecvLoop((void *)(&TCX_no_time));}
#endif
  for (ii=0; ii<TCX_MAX_CLIENTS; ii++) {
    if (CLIENT[ii] != NULL) {
      tcxSendMsg(CLIENT[ii], BASE_VARIABLE_MESSAGE, &command);
    }
  }
}

/*------------------------------------------------------------*/

/***************** FOR RESPONDING TO MESSAGES FROM CLIENT *************/

void dispatchFixedMessage(int operation, unsigned long param)
{
  if (VERBOSE) {
    fprintf(stderr,"Calling base operation %d\n",operation);
  }

  confirmCommandToClient(operation, param); /* sends a message back to all
					     * baseClients, confirming the
					     * command, sebastian 7-97 */

  switch(operation) {
    
    /**** GENERAL ****/ 
  case BASE_baseKill:		baseKill(); break;	
  case BASE_loadHeading:	loadHeading(param); break;	
  case BASE_loadPosition:	loadPosition(param); break;	
  case BASE_statusReportData:	statusReportData(param);break;
  case BASE_statusReportPeriod:	statusReportPeriod(param);break;
  case BASE_batteryVoltage:	batteryVoltage();break;
  case BASE_batteryCurrent:	batteryCurrent();break;

  case BASE_watchdogTimer:	watchdogTimer(param);break;
  case BASE_assumeWatchdog:	assumeWatchdog();break;


    /**** ROTATION ****/ 
  case BASE_rotateLimp:		rotateLimp(); break;
  case BASE_rotateHalt:		rotateHalt(); break;
  case BASE_rotateVelocityPos:	rotateVelocityPos(); break;	
  case BASE_rotateVelocityNeg:  rotateVelocityNeg(); break;	
  case BASE_rotateRelativePos:  rotateRelativePos(param); break;	
  case BASE_rotateRelativeNeg: 	rotateRelativeNeg(param); break;	
  case BASE_rotateTorquePos:	rotateTorquePos(param); break;	
  case BASE_rotateTorqueNeg: 	rotateTorqueNeg(param); break;	
  case BASE_rotatePowerPos:	rotatePowerPos(param); break;	
  case BASE_rotatePowerNeg: 	rotatePowerNeg(param); break;	
  case BASE_rotateToPosition: 	rotateToPosition(param); break;	
  case BASE_findRotIndex:
    if (bRobot.base_hasIndex) {
      findRotIndex();
    }
    else {
      sendClientFixed(BASE_indexReport, 0xFFFFFF);
    }
    break;

  case BASE_setRotateVelocity:	setRotateVelocity(param); break;	
  case BASE_setRotateAcceleration: setRotateAcceleration(param); break;	
  case BASE_setRotateFriction: 	setRotateFriction(param); break;	
  case BASE_setRotateSlope: 	setRotateSlope(param); break;	
  case BASE_setRotateTorque: 	setRotateTorque(param); break;	
  case BASE_setRotateZero: 	setRotateZero(param); break;	

  case BASE_rotateCurrent:	rotateCurrent(); break;	
  case BASE_rotateWhere:	rotateWhere(); break;	


    /**** TRANSLATION ****/ 
  case BASE_translateLimp:		translateLimp(); break;
  case BASE_translateHalt:		translateHalt(); break;
  case BASE_translateVelocityPos:  	translateVelocityPos(); break;	
  case BASE_translateVelocityNeg:  	translateVelocityNeg(); break;	
  case BASE_translateRelativePos:  	translateRelativePos(param); break;
  case BASE_translateRelativeNeg:  	translateRelativeNeg(param); break;
  case BASE_translateTorquePos:		translateTorquePos(param); break;
  case BASE_translateTorqueNeg: 	translateTorqueNeg(param); break;
  case BASE_translatePowerPos:		translatePowerPos(param); break;
  case BASE_translatePowerNeg: 		translatePowerNeg(param); break;
  case BASE_translateToPosition: 	translateToPosition(param); break;
  case BASE_setTranslateVelocity:	setTranslateVelocity(param); break;
  case BASE_setTranslateAcceleration: 	setTranslateAcceleration(param); break;
  case BASE_setTranslateSlope: 		setTranslateSlope(param); break;
  case BASE_setTranslateTorque: 	setTranslateTorque(param); break;
  case BASE_setTranslateZero: 		setTranslateZero(param); break;

  case BASE_translateCurrent:		translateCurrent(); break;	
  case BASE_translateWhere:		translateWhere(); break;	

  /**** SONARS ****/
  case BASE_sonarStart:                 SIM_sonarStart(); break;
  case BASE_sonarStop:                  SIM_sonarStop(); break;

  /**** Odometry stuff ****/
  case BASE_odometryChangeX:
    ntohf(param, bOriginX);
    sendClientFixed(BASE_odometryChangeX, param);
    break;
  case BASE_odometryChangeY:
    ntohf(param, bOriginY);
    sendClientFixed(BASE_odometryChangeY, param);
    break;
  case BASE_odometryChangeH:
    ntohf(param, bOriginHeading);
    sendClientFixed(BASE_odometryChangeH, param);
    break;

  /**** Odometry lock requests ****/
  case BASE_requestOdometryLock:
    requestOdometryLock((unsigned short)param);
    break;
  case BASE_releaseOdometryLock:
    releaseOdometryLock();
    break;

  default: 
    fprintf(stderr, "BaseServer: Operation %d not yet implemented.\n",
            operation); 	
  }
  
  if (VERBOSE) {
    fprintf(stderr,"Operation %d complete\n",operation);
  }
}

/*------------------------------------------------------------*/

void
dispatchVariableMessage(int operation, int bufsize, char buffer[])
{
  fprintf(stderr,
	  "BaseServer: Variable length message %d not yet implemented.\n",
          operation);
}

/*------------------------------------------------------------*/

void
handleBaseFixed(TCX_REF_PTR message,RAI_FixedMsgType *msg_ptr)
{
  int operation;
  unsigned long param;
  TCX_MODULE_PTR ptr;
  int i;

  operation = msg_ptr->operation;
  param = msg_ptr->parameter;

  if (VERBOSE) {
    fprintf(stderr,"baseServer fixed msg %d %lu\n",operation,param); 
  }

  /* Get the index in the client module list to enable odometry locking */
  ptr = tcxRefModule(message);
  for(i = 0; i < TCX_MAX_CLIENTS; i++)
    if (CLIENT[i] == ptr) {
      bRequesterIndex = i;
      break;
    }

  if (operation== BASE_subscribe) {
    /* Later, update this so multiple modules can subscribe, */
    /* specify what they want to hear about */
    addBaseClient(message->module);
  }
  else {
    dispatchFixedMessage(operation,param);
  }
  tcxFree(BASE_FIXED_MESSAGE, msg_ptr);
}

/*------------------------------------------------------------*/

void
handleBaseVariable(TCX_REF_PTR message, RAI_VariableMsgType *msg_ptr)
{
  fprintf(stderr, "handleBaseVariable() not yet implemented.\n");
}

/*------------------------------------------------------------*/

void
simulatorBaseServerHandler(TCX_REF_PTR message, char *msg_ptr)
{
  extern void handleBaseOutput(unsigned char *);

  handleBaseOutput(msg_ptr);
  tcxFree("SIMULATOR_message_to_baseServer", msg_ptr);
}

/*------------------------------------------------------------*/

void
simulatorSonarServerHandler(TCX_REF_PTR message, sonarType *msg_ptr)
{
  int sonarToClient(sonarType* sonars);

  sonarToClient(msg_ptr);
  tcxFree("SIMULATOR_message_to_sonarServer", msg_ptr);
}

/*------------------------------------------------------------*/

void
simulatorIrServerHandler(TCX_REF_PTR message,
			 SIMULATOR_message_to_irServer_type *msg_ptr)
{
  int bufsize;
  long *values;
  int ii;

  bufsize = sizeof(long) * msg_ptr->count;
  values = malloc(bufsize * sizeof(*values));

  if (!values) {
    fprintf(stderr, "%s:%6d:%s() - malloc() failed\n",
	    __FILE__, __LINE__, __FUNCTION__);
    return;
  }

  for (ii=0; ii<bufsize; ii++) {
    values[ii] = htonl(msg_ptr->values[ii]);
  }

  sendClientVariable(IR_newValues, bufsize, (char*)values);
  free(values);
  values = NULL;

  tcxFree("SIMULATOR_message_to_irServer", msg_ptr);
  return;
}

/*------------------------------------------------------------*/

void
simulatorTactileServerHandler(TCX_REF_PTR message,
			 SIMULATOR_message_to_tactileServer_type *msg_ptr)
{
  int bufsize;
  long *values;
  int ii;

  bufsize = sizeof(long) * msg_ptr->count;
  values = malloc(bufsize * sizeof(*values));

  if (!values) {
    fprintf(stderr, "%s:%6d:%s() - malloc() failed\n",
	    __FILE__, __LINE__, __FUNCTION__);
    return;
  }

  for (ii=0; ii<bufsize; ii++) {
    values[ii] = htonl(msg_ptr->values[ii]);
  }

  sendClientVariable(TACTILE_newValues, bufsize, (char*)values);
  free(values);
  values = NULL;

  tcxFree("SIMULATOR_message_to_tactileServer", msg_ptr);
}

/*------------------------------------------------------------*/

void
baseClose(char *moduleName, TCX_MODULE_PTR module)
{
  fprintf(stderr, "TCX close connection detected: %s\n",
	  moduleName);
  
  if (!strcmp(moduleName, "TCX Server")){ /* TCX shut down */
    fprintf(stderr, "TCX Server died, and so will I.\n");
    RaiShutdown();
    exit(0);
  }

  if (!strcmp(moduleName, TCX_SIMULATOR_MODULE_NAME)){
    fprintf(stderr, "Simulator died, and so will I.\n");
    RaiShutdown();
    exit(0);
  }

  dropBaseClient(module);
}

/*------------------------------------------------------------*/

void
initBaseServerTCX(const char *tcxHost)
{
  TCX_REG_MSG_TYPE messageArray[] = {
    BASE_MESSAGES,
    SIMULATOR_messages
  };

  TCX_REG_HND_TYPE handlerArray[] = BASE_HANDLERS;
  int numberOfHandlers;
  int numberOfMessages;  
  
  numberOfHandlers = sizeof(handlerArray)/sizeof(TCX_REG_HND_TYPE); 
  numberOfMessages = sizeof(messageArray)/sizeof(TCX_REG_MSG_TYPE); 

  tcxInitialize(BASE_SERVER_NAME, (char *)tcxHost);
  check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
		       BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
		       NULL, 0);
  check_version_number(librai_major, librai_minor,
		       librai_robot_type, librai_date,
		       "librai", 0);
  check_version_number(libbase_major, libbase_minor,
		       libbase_robot_type, libbase_date,
		       "libbase", 0);
  check_version_number(libmcp_major, libmcp_minor,
		       libmcp_robot_type, libmcp_date,
		       "libmcp", 0);
  check_version_number(libmsp_major, libmsp_minor,
		       libmsp_robot_type, libmsp_date,
		       "libmsp", 0);
  check_version_number(libutils_major, libutils_minor,
		       libutils_robot_type, libutils_date,
		       "libutils", 0);
  check_version_number(libabus_major, libabus_minor,
		       libabus_robot_type, libabus_date,
		       "libabus", 0);
  check_version_number(libbUtils_major, libbUtils_minor,
		       libbUtils_robot_type, libbUtils_date,
		       "libbUtils", 1);

  tcxRegisterMessages(messageArray, numberOfMessages);
  tcxRegisterHandlers(handlerArray, numberOfHandlers);
  tcxRegisterCloseHnd(baseClose);

  if (VERBOSE) {
    fprintf(stderr,"baseServer Registered\n");
  }
}

/*------------------------------------------------------------*/

void
baseToClient(unsigned long opcode, unsigned long value)
{
  if (VERBOSE) {
    fprintf(stderr,
	    "%s:%6d:%s() - Sending opcode %d, value %lu to client\n",
	    __FILE__, __LINE__, __FUNCTION__, opcode, value);
  }
  sendClientFixed(opcode,value);
}

/*------------------------------------------------------------*/

void
statusToClient(statusReportType* status)
{
  statusReportType nstatus;

  if(VERBOSE) {
    fprintf(stderr,"Sending status report to client\n");
  }

  /*
   * NOTE!
   *
   * This only works because the entire struct
   * is longs and not a mix of types.  If the struct becomes
   * other than all longs this will likely fail do to different
   * architectures packing structures differently.  -tds
   */

  nstatus.Request              = htonl(status->Request);
  nstatus.Clock                = htonl(status->Clock);
  nstatus.GeneralStatus        = htonl(status->GeneralStatus);
  nstatus.Xpos                 = htonl(status->Xpos);
  nstatus.Ypos                 = htonl(status->Ypos);
  nstatus.Heading              = htonl(status->Heading);
  nstatus.BaseRelativeHeading  = htonl(status->BaseRelativeHeading);
  nstatus.TranslateError       = htonl(status->TranslateError);
  nstatus.TranslateVelocity    = htonl(status->TranslateVelocity);
  nstatus.TranslateStatus      = htonl(status->TranslateStatus);
  nstatus.RotateError          = htonl(status->RotateError);
  nstatus.RotateVelocity       = htonl(status->RotateVelocity);
  nstatus.RotateStatus         = htonl(status->RotateStatus);

  sendClientVariable(BASE_statusReport, sizeof(statusReportType),
		     (char*)(&nstatus));

  return;
}

/*------------------------------------------------------------*/

void
watchdogToClient()
{
  fprintf(stderr,
	  "%s:%6d:%s() - *** watchdog timeout being sent to client\n",
	  __FILE__, __LINE__, __FUNCTION__);
  baseToClient(BASE_watchdogTimeout,0);
  return;
}

/*------------------------------------------------------------*/

int
sonarToClient(sonarType* sonars)
{
  int index;
  int value_index= 0;
  int bufsize;
  long values[B_MAX_SENSOR_COLS*2]; 

  /* we pack the index and sonar value up for each new reading */
  /* and terminate it by a sonar index < 0, which client */
  /* code checks for */

  for(index=0; index<bRobot.sonar_cols[0]; index++) {
    if (sonars[index].mostRecent) {
      values[value_index++]=htonl(index);
      values[value_index++]=htonl(sonars[index].value);
    }
  }

  values[value_index++] = htonl(-1);
  bufsize = sizeof(long) * value_index;
  sendClientVariable(SONAR_newValues, bufsize, (char*)values);
  return(0);
}

/*------------------------------------------------------------*/

int
irToClient(irType **irTable)
{
  int row;
  int col;
  int value_index= 0;
  int bufsize;
  long values[B_MAX_SENSOR_ROWS * B_MAX_SENSOR_ROWS * 3];

  /* we pack the index and sonar value up for each new reading */
  /* and terminate it by an index < 0, which client */
  /* code checks for */

  for(row=0; row<bRobot.ir_rows; row++) {
    for(col=0; col<bRobot.ir_cols[row]; col++) {
      if (irTable[row][col].mostRecent) {
	values[value_index++]=htonl(row);
	values[value_index++]=htonl(col);
	values[value_index++]=htonl(irTable[row][col].value);
      }
    }
  }

  values[value_index++] = htonl(-1);
  bufsize = sizeof(long) * value_index;
  sendClientVariable(IR_newValues, bufsize, (char*)values);
  return(0);
}

/*------------------------------------------------------------*/

int
tactileToClient(tactileType **tactileTable)
{
  int row;
  int col;
  int value_index= 0;
  int bufsize;
  long values[B_MAX_SENSOR_ROWS * B_MAX_SENSOR_ROWS * 3];

#ifdef DEBUG
  fprintf(stderr, "%s:%6d:%s() -\n", __FILE__, __LINE__, __FUNCTION__);

  row=0;
  for(col=0; col<bRobot.tactile_cols[row]; col++) {
    fprintf(stderr, "\tvalue[%3d, %3d] = %5d\n",
	    row, col, tactileTable[row][col].value);
  }

  fprintf(stderr, "%s:%6d:%s() -\n", __FILE__, __LINE__, __FUNCTION__);
#endif

  for(row=0; row<bRobot.tactile_rows; row++) {
    for(col=0; col<bRobot.tactile_cols[row]; col++) {
      values[value_index++]=htonl(row);
      values[value_index++]=htonl(col);
      values[value_index++]=htonl(tactileTable[row][col].value);

#ifdef DEBUG
      if (row==0) {
	fprintf(stderr, "\tvalue[%3d, %3d] = %5d\n",
		row, col, tactileTable[row][col].value);
      }
#endif

    }
  }

#ifdef DEBUG
  fprintf(stderr, "%s:%6d:%s() -\n", __FILE__, __LINE__, __FUNCTION__);

  row=0;
  for(col=0; col<bRobot.tactile_cols[row]; col++) {
    fprintf(stderr, "\tvalue[%3d, %3d] = %5d\n",
	    row, col, tactileTable[row][col].value);
  }
#endif

  values[value_index++] = htonl(-1);

  bufsize = sizeof(long) * value_index;
  sendClientVariable(TACTILE_newValues, bufsize, (char*)values);
  return(0);
}

/*------------------------------------------------------------*/

/*
 * This is a routine for sending an echo of each motion command
 * back to the baseClient. It is used for logging the commands that
 * are sent to the baseServer. 
 *
 * by Sebastian Thrun, July 1997
 */

void
confirmCommandToClient(int operation, unsigned long param)
{
  confirmCommandDataType confirmData;


  if (operation != BASE_baseKill             &&	/* 6 in baseMessages.h */
      operation != BASE_rotateHalt           &&	/* 16 */
      operation != BASE_rotateVelocityPos    &&	/* 17 */
      operation != BASE_rotateVelocityNeg    &&	/* 18 */
      operation != BASE_setRotateVelocity    &&	/* 31 */
      operation != BASE_translateHalt        &&	/* 33 */
      operation != BASE_translateVelocityPos &&	/* 34 */
      operation != BASE_translateVelocityNeg &&	/* 35 */
      operation != BASE_setTranslateVelocity  ) /* 47 */
    return;


  confirmData.operation =  htonl((unsigned long int) operation);
  confirmData.param     =  htonl((unsigned long int) param);

  sendClientVariable(BASE_confirmCommand, sizeof(confirmCommandDataType),
		     (char*)(&confirmData));

  /*fprintf(stderr, " [%d:%d] ", (int) operation, (int) param); */
}

/*------------------------------------------------------------*/

int
findArg(int argc, char* argv[], char* value)
{
  int index;
  for(index=1;index<argc;index++) {
    if (strcmp(value,argv[index])==0) {
      return TRUE;
    }
  }

  return(FALSE);
}

/*------------------------------------------------------------*/

int main(int argc, char* argv[]) {
  struct bParamList * paramList = NULL;
  const char *strPtr;

  int ii;

  for (ii=0; ii<TCX_MAX_CLIENTS; ii++) {
    CLIENT[ii] = NULL;
  }

  /*
   * Set some parameters
   */

  /* add some defaults */
  paramList = bParametersAddEntry(paramList, "*", "base.bps", "9600");
  paramList = bParametersAddEntry(paramList, "*", "base.dev", "/dev/cur0");
  paramList = bParametersAddEntry(paramList, "", "TCXHOST", "localhost");
  paramList = bParametersAddEntry(paramList, "", "fork", "yes");

  /* add some parameter files */
  paramList = bParametersAddFile(paramList, "etc/beeSoft.ini");

  /* add some enviroment variables */
  paramList = bParametersAddEnv(paramList, "", "TCXHOST");

  /* add command line arguements */
  paramList = bParametersAddArray(paramList, "", argc, argv);

  /* here is where we should add non "parameter format" options */


  bParametersFillParams(paramList);

  strPtr = bParametersGetParam(paramList, "robot", "name");
  if (!strPtr) {
    fprintf(stderr,
	    "Robot name not set.  Check the value in beeSoft.ini.\n");
    exit(1);
  }
  else if (!strcmp("none", strPtr)) {
    fprintf(stderr,
	    "Robot name set to 'none'.  Check value in beeSoft.ini.\n");
    exit(1);
  }

  strPtr = bParametersGetParam(paramList, strPtr, "base.type");
  if (!strPtr) {
    fprintf(stderr,
	    "Base type not set.  Check the value in beeSoft.ini.\n");
    exit(1);
  }
  else if (
	   strcmp("B14", strPtr) &&
	   strcmp("B21", strPtr) &&
	   strcmp("Ramona", strPtr)
	   ) {
    fprintf(stderr,
	    "Arm type set to unrecognized type '%s'.  Check the\n"
	    "value in beeSoft.ini.\n", strPtr);
    exit(1);
  }
  
  /*
   *
   */

  argv0 = argv[0];

  if (makeLock(BASE_SERVER_LOCK)<0) {
    fprintf(stderr,"%s:  Already running base server\n",argv[0]);
    exit(-1);
  }

  if (findArg(argc,argv,"-v")) {
    VERBOSE = TRUE;
    fprintf(stderr,"baseServer started in verbose mode\n");
  }

  if (findArg(argc,argv,"-noIrs")) {
    IRS = FALSE;
    fprintf(stderr,"baseServer started without IRs\n");
  }

  if (findArg(argc,argv,"-noTactiles")) {
    TACTILES = FALSE;
    fprintf(stderr,"baseServer started without Tactiles\n");
  }

  if (findArg(argc,argv,"-noSonars")) {
    SONARS= FALSE;
    fprintf(stderr,"baseServer started without Sonars\n");
  }

  if (findArg(argc,argv,"-simulator")) {
    useSimulator=TRUE;
    fprintf(stderr,"baseServer connecting to simulator.\n");
  }

  
  RaiInit();
  BaseInit(bRobot.base_dev, bRobot.base_bps);
  catchInterrupts();

  initBaseServerTCX(bRobot.TCXHOST);

  if (useSimulator==TRUE) {
    simulatorHandle = tcxConnectModule(TCX_SIMULATOR_MODULE_NAME);
  }

  registerWatchdogCallback(watchdogToClient);

  /*
   * GRUMBLE - It is going to be a bit ugly to get the sensor
   * structures initialized without opening /dev/abus.  tds
   */

  if (SONARS && !useSimulator) {
    sonarInit();
    registerSonarCallback(sonarToClient);
    SIM_sonarStop();  /* turn off sonar until we have a client */
  }

  if (IRS && !useSimulator) {
    irInit();
    registerIrCallback(irToClient);
    SIM_irStop();  /* turn off ir until we have a client */
  }

  if (TACTILES && !useSimulator) {
    tactileInit();
    registerTactileCallback(tactileToClient);
    tactileStop();
  }

  registerStatusCallback(statusToClient);
  registerBaseCallback(baseToClient);
  initBaseServerModules();

  if (bRobot.fork) {
    bDaemonize("baseServer.log");
  }

  rotateLimp(); 
  translateLimp(); 

  RaiStart();

  exit(0);
}



void requestOdometryLock(unsigned short priority) {
  RAI_FixedMsgType command;
  int i;

  if ((bOdometryLock == -1) || (priority > bOdometryLockPriority)) {
    bOdometryLock = bRequesterIndex;
    bOdometryLockPriority = priority;

    /* tell the requesting module that they have the lock */
    command.operation = BASE_odometryLockNotify;
    command.parameter = B_ODOMETRY_I_HAVE_LOCK;
    tcxSendMsg(CLIENT[bRequesterIndex], BASE_FIXED_MESSAGE, &command);

    /* tell everyone else that they can't do anything */
    command.parameter = B_ODOMETRY_OTHER_HAS_LOCK;
    for(i = 0; i < TCX_MAX_CLIENTS; i++)
      if ((CLIENT[i] != NULL) && (i != bOdometryLock)) {
        tcxSendMsg(CLIENT[i], BASE_FIXED_MESSAGE, &command);
      }
  }
}
  


void releaseOdometryLock() {
  if (bOdometryLock == bRequesterIndex) {
    bOdometryLock = -1;
    bOdometryLockPriority = 0;
    
    sendClientFixed(BASE_odometryLockNotify, 
                    (unsigned long)B_ODOMETRY_OTHER_HAS_LOCK);
  }
}
