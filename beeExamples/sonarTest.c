
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

#define SONAR_THRESH 1000

void speechTimer(RaiModule * mover);

int speechBusy = 0;

int lastSonar;
int closeSonar;
int sonarVal = -1;

RaiModule * newMod;

/*------------------------------------------------------------*/

/*
 * this is called if a server bags exits
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
handleSonars(sonarType* sonarList)
{
  int index;
  
  sonarVal = -1;

  for(index = 0; index < bRobot.sonar_cols[0]; index++) {
    if ((sonars[index].mostRecent &&
	 (sonars[index].value < SONAR_THRESH)) && 
	((sonarVal==-1) ||
	 (sonars[index].value < sonars[closeSonar].value))) {
      
      closeSonar = index;
      sonarVal = sonars[index].value;
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

  if (sonarVal != -1) {
    printf("sonar %d %d\n", closeSonar, sonarVal);
    speechBusy=1;
    if (closeSonar != lastSonar) {
      sprintf(buffer,"sonar %d %d\n", closeSonar, sonarVal);
      sendSpeech(buffer);
      addTimeout(newMod, speechTimer, 5500);
    }
    else {
      sprintf(buffer,"%d\n", sonarVal);
      sendSpeech(buffer);
      addTimeout(newMod, speechTimer, 4000);
    }

    sonarVal = -1;
    lastSonar=closeSonar;
    return;
  }

  lastSonar = -1;  

  return;
}  

/*------------------------------------------------------------*/

void initMyModule()
{
  
  /* make a module that would do something useful */
  /* in 1/2 second, if this were not a demo */
  newMod = makeModule("newMod",raiClose);
  speechBusy=1;
  addTimeout(newMod, speechTimer, 0500);
  addPolling(newMod, speechPolling, 0250);
  
  registerSonarCallback(handleSonars);
}

/*------------------------------------------------------------*/

int main(int argc, char *argv[])
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
