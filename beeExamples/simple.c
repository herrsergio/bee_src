
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
#include <rai.h>
#include <bUtils.h>

RaiModule* demoModule;

static int front;
static float distance;
static float maxSpeed;
static int limp;

void
demoPoll(RaiModule * demoModule)
{
  float vel;
  static int count= 100;

  vel = ((float)sonars[front].value/10.0 - distance)/2.0;

  if (vel > maxSpeed) {
    vel = maxSpeed;
  }

  if (vel < -maxSpeed) {
    vel = -maxSpeed;
  }

  if (count++>10) {
    printf("vel = %8.2f  sonars[%d]=%8.2f\n",
	   vel, front, (float)sonars[front].value/10.0);
    count=0;
  }

  if (limp) {
    translateLimp();
    rotateLimp();
  }
  else {
    bSetVel(0.0, vel);
  }

  return;
}

void
createDemoModule()
{
  int ii;

  front = 0;

  /* find the sonar sensor that is closest to the front */
  for (ii=1; ii<bRobot.sonar_cols[0]; ii++) {
    if (fabs(bSonarAngle(0, ii)) < fabs(bSonarAngle(0, front))) {
      front = ii;
    }
  }

  /* ask that your polling function be run every 100msec */
  demoModule=makeModule("simple", NULL);
  addPolling(demoModule, demoPoll, 100);
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

  /* put some default values in the list */
  params = bParametersAddEntry(params, "", "distance", "100");
  params = bParametersAddEntry(params, "", "maxSpeed", "30");
  params = bParametersAddEntry(params, "", "limp", "no");

  /* add the parameter file */
  params = bParametersAddFile(params, "etc/beeSoft.ini");

  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* This puts the information in the global bRobot struct. */
  bParametersFillParams(params);

  /* get some custom values from the parameter list */
  distance = atof(bParametersGetParam(params, "", "distance"));
  maxSpeed = atof(bParametersGetParam(params, "", "maxSpeed"));
  limp = bStrToTruth(bParametersGetParam(params, "", "limp"));

  /* Tell the user what the values are */
  printf("\n***************************\n\n");
  printf("-distance=%f\n", distance);
  printf("-maxSpeed=%f\n", maxSpeed);
  printf("-limp=%d\n",    limp);
  printf("\n***************************\n\n");

  /* This will initialize the scheduler */
  RaiInit();

  /* initialize the baseServer client library */
  registerBaseClient();

  /*
   * Connect this process to tcxServer.  Call commShutdown
   * if any of the connected TCX processes exit.
   */

  initClient("simple",commShutdown);

  /* Connect the baseServer client library to baseServer */
  baseConnect(1);

  /* install some default signal handlers */
  catchInterrupts();

  /* sets up some scheduling stuff for TCX communications */
  initClientModules();

  /* Setup this program */
  createDemoModule();

  /*
   * Start the scheduler.
   * this will only return when RaiShutdown is called.
   */

  RaiStart();

  return(0);
}
