
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
#include <baseClient.h>
#include <speechClient.h>
#include <rai.h>
#include <bUtils.h>

void speechTimer(RaiModule * mover);

int speechBusy = 0;

int activeTactileRow;
int activeTactileCol;
int tactileVal = 0;

RaiModule * newMod;

/*------------------------------------------------------------*/

/*
 * this is called if a server exits.
 */

void
commShutdown(char *name, TCX_MODULE_PTR module)
{

  fprintf( stderr, "%s(%s): %s died. Closing. \n", 
	   __FILE__, __FUNCTION__, name );

  module = module;		/* prevents gcc from barking with `-Wall' */

  shutdownClient();

  RaiShutdown();

  exit(0);
}

/*------------------------------------------------------------*/

/*
 * this is called if Rai scheduler shuts down (ie if user interrupts)
 */

void
raiClose()
{
  printf("Scheduler shutting down\n");
  shutdownClient();
  /* Rai shutdown should not be called */
}

/*------------------------------------------------------------*/

int
handleTactiles(tactileType** tactileMatrix)
{
 int index,row;
 
 tactileVal = 0;

 for(row=0 ;row < bRobot.tactile_rows; row++) {
   for(index = 0; index< bRobot.tactile_cols[row]; index++) {
     if (tactileMatrix[row][index].value) {
       activeTactileRow = row;
       activeTactileCol = index;
       tactileVal = 1;
     }
   }
 }

 return 0;
}

/*------------------------------------------------------------*/

void
speechTimer(RaiModule * mover)
{
  static int first = 1;

  speechBusy = 0;

  if (first) {
    first=0;
    sendSpeech("Ready to rock!\n");
    addTimeout(newMod,speechTimer,0500);
    speechBusy = 1;
  }
}

/*------------------------------------------------------------*/

void
speechPolling(RaiModule * mover)
{
  char buffer[400];
  
  if (speechBusy) {
    return;
  }

  if (tactileVal) {
    printf("bump %d %d\n", activeTactileCol, activeTactileRow);
    speechBusy=1;
    sprintf(buffer,"bump %d %d\n", activeTactileCol, activeTactileRow);
    sendSpeech(buffer);
    addTimeout(newMod, speechTimer, 1700);
    return;
  }

  tactileVal = 0;

  return;
}  

/*------------------------------------------------------------*/

void
initMyModule()
{
  
  /* make a module that would do something useful */
  /* in 1/2 second, if this were not a demo */
  newMod = makeModule("newMod",raiClose);
  speechBusy=1;
  addTimeout(newMod, speechTimer, 0500);
  addPolling(newMod, speechPolling, 0250);
  
  registerTactileCallback(handleTactiles);
}

/*------------------------------------------------------------*/

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

  registerSpeechClient();
  registerBaseClient();

  initClient("Test",commShutdown);
  
  baseConnect(1);
  findSpeechServer();
  
  RaiInit();
  catchInterrupts();
  
  initClientModules();
  initMyModule();

  RaiStart();
  exit(0);
}
