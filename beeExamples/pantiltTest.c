
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
#include <stdlib.h>
#include <math.h>

#include <baseClient.h>
#include <pantiltClient.h>
#include <rai.h>
#include <bUtils.h>

RaiModule* demoModule;

void
demoPoll(RaiModule * demoModule)
{
  float newTilt, newPan;
  int ii, closest;

  closest = bRobot.sonar_cols[0]/2;

  for (ii=0; ii<bRobot.sonar_cols[0]; ii++) {
    if (sonars[ii].value < sonars[closest].value) {
      closest = ii;
    }
  }

  newTilt = atan2(40.0, (float)sonars[closest].value / 10.0);
  newPan = bSonarAngle(0, closest);
  ptMoveTo(newPan, newTilt);

  fprintf(stderr, "%s:%s() - sonar %3d = %5.1f, newPos = (%6.1f,%6.1f)\n",
	  __FILE__, __FUNCTION__,
	  closest, (float)sonars[closest].value/10.0,
	  newPan * 180.0 / M_PI, newTilt * 180.0 / M_PI);

  return;
}

void
ptStatusUpdate()
{
  struct timeval time;
  int time2;

  gettimeofday(&time, NULL);
  time2 = (time.tv_sec % 3600) * 10 + time.tv_usec/100000;

  fprintf(stderr, "%s:%6d:%s() - [%6d][%5.2f:%5.2f]\n",
	  __FILE__, __LINE__, __FUNCTION__, time2,
	  ptStatusPanPos*180.0/M_PI, ptStatusTiltPos*180.0/M_PI);
}

void
createDemoModule()
{
  /* ask that your polling function be run every 500msec */
  demoModule=makeModule("panTest",NULL);
  addPolling(demoModule,demoPoll,500);

  ptStatusCB(ptStatusUpdate);
}

void
commShutdown(char *name, TCX_MODULE_PTR module)
{

  fprintf( stderr, "%s(%s): %s died. Closing. \n", 
	   __FILE__, __FUNCTION__, name );

  module = module;		/* prevents gcc from barking with `-Wall' */

  RaiShutdown();
}
  
int
main(int argc, char** argv )
{
  struct bParamList * params = NULL;

  fprintf(stderr, "starting main\n");

  /* add some parameter files */
  params = bParametersAddFile(params, "etc/beeSoft.ini");

  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(params);

  registerBaseClient();
  ptRegister();
  initClient("pantiltTest",commShutdown); /* close function called if */
                                     /* the base server dies */
  baseConnect(1);                  /* hook up to running server*/
  ptConnect(1);

  RaiInit();                            /* init (but not start) scheduler*/
  catchInterrupts();
  initClientModules();                  /* set up Rai modules to do      */
                                        /* communication for you */
  createDemoModule();                   /* set up your module to move robot*/
  RaiStart();
  /* this will only return when RaiShutdown is called or ^C is used */
  return(0);
}
