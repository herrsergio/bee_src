
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
#include <stdio.h>
#include <unistd.h>	
#include <armClient.h>
#include <speechClient.h>
#include <baseClient.h>
#include <rai.h>
#include <bUtils.h>

int demoState = 0;

void
commShutdown(char *name, TCX_MODULE_PTR module)
{

  fprintf( stderr, "%s(%s): %s died. Closing. \n", 
	   __FILE__, __FUNCTION__, name );

  module = module;		/* prevents gcc from barking with `-Wall' */

  RaiShutdown();
}

void
armMessageHandler(armMessage event, unsigned long value)
{
  switch(demoState) 
    {
    case 0: 
      if (event==ARM_deployArm) {
	printf("Arm deployed.\n");
	sendSpeech("Arm deployed.\n");
	mastToPos(200);
	wristToPos(0x100);
	demoState = 1;
      }
      break;

    case 1:
      if (event == ARM_wristStopped) {
	gripToPos(200);
	demoState = 2;
      }
      break;

    case 2:
      if (event == ARM_gripStopped) {
	printf("grip stopped\n");
	mastToPos(0);
	demoState = 3;
      }
      break;

    case 3:
      if (event == ARM_mastStopped) {
	printf("grabbing\n");
	sendSpeech("grabbing\n");
	gripToPos(10);
	demoState = 4;
      }
      break;

    case 4:
      if (event == ARM_gripStopped) {
	printf("Didn't get anything\n");
	sendSpeech("Did not get anything\n");
	printf("stowing arm\n");
	sendSpeech("stowing arm\n");
	stowArm();
	demoState = 9;
      }
      else if ((event == ARM_gripError) || (event == ARM_gripEngaged)) {
	gripHalt();
	if (event == ARM_gripError) {
	  sendSpeech("Grip stalled.\n");
	}
	else {
	  sendSpeech("grip engaged.\n");
	}
	printf("Grabbed something.\n");
	mastToPos(200);
	demoState = 5;
      }
      break;

    case 5:
      if (event == ARM_mastStopped) {
	wristToPos(0x300);
	demoState = 6;
      }
      break;

    case 6:
      if (event == ARM_wristStopped) {
	mastToPos(0);
	sendSpeech("Dropping\n");
	demoState = 7;
      }
      break;

    case 7:
      if (event == ARM_mastStopped) {
	gripToPos(200);
	demoState = 8;
      }
      break;

    case 8:
      if (event == ARM_gripStopped) {
	printf("stowing arm\n");
	sendSpeech("stowing arm\n");
	stowArm();
	demoState = 9;
      }
      break;
      
    case 9:
      if (event == ARM_stowArm) {
	sendSpeech("Arm stowed\n"); 
	RaiShutdown();
      }
      break;

    default: break;
    }
}

void
moveArm(RaiModule * mod)
{
  subscribeToArm((clientCallback)armMessageHandler);
  sendSpeech("Deploying Arm\n");
  deployArm();
}

void
initMyModule()
{
  RaiModule * mover;
  mover = makeModule("Mover",NULL);
  addTimeout(mover,moveArm,1000);
}


int
main(int argc, char *argv[])
{
  struct bParamList * params = NULL;

  /* add some parameter files */
  params = bParametersAddFile(params, "etc/beeSoft.ini");

  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(params);

  printf("Registering clients\n");

  registerArmClient();
  registerSpeechClient();

  initClient("Test",commShutdown);
  printf("Finding Servers\n");
  findSpeechServer();
  findArmServer();

  RaiInit();
  initClientModules();
  initMyModule();
  RaiStart();
  exit(0);
}
