
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
#include <colliClient.h>
#include <rai.h>
#include <bUtils.h>

int gotOrigin = FALSE;
int statusCount = 0;

int
tactileCallback(tactileType ** newTactile)
{
  int i;
  int j;

  for(i=0;i<bRobot.tactile_rows;i++) {
    for(j=0;j<bRobot.tactile_cols[i];j++) {
      if (tactiles[i][j].value) {
	printf("**** Touched on %d,%d\n",i,j);
	bSetVel(0.0, 0.0);
	return;
      }
    }
  }

  return;
}


void
demoPoll(RaiModule * demoModule)
{
  static time = 0;

  if (gotOrigin) {
    if (time == 0) {
      colliApproachAbsolute(-100.0, 0.0, 50.0, 1);
    }
    
    if (time>300) {
      RaiShutdown();
    }
    
    time++;
  }
  
  return;
}

void
statusCallback(statusReportType * newStatus)
{

  statusCount++;

  if ((!gotOrigin) && (statusCount<10)) {
    bSetPosition(0.0, 0.0, 0.0);
    gotOrigin=TRUE;
  }
}

void
createDemoModule()
{
  RaiModule* demoModule;
  int ii;

  printf("Setting up robot program\n"); 

  loadHeading(0);                   /* units = radians*512/M_PI    */
  loadPosition(0x80008000);         /* units = encoder counts/256  */
  statusReportPeriod(100*256/1000); /* Units = seconds*256         */
  registerStatusCallback(statusCallback);
  registerTactileCallback(tactileCallback);

  colliSetVelocity(20.0, M_PI/2.0);
  colliSetAcceleration(60.0, 20.*M_PI);

  /* ask that your polling function be run every 100msec */
  demoModule=makeModule("Demo",NULL);
  addPolling(demoModule,demoPoll,100);
  fprintf(stderr, "done.\n");
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

  /* add some parameter files */
  params = bParametersAddFile(params, "etc/beeSoft.ini");

  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(params);

  registerBaseClient();
  colliRegister();

  initClient("colliTest",commShutdown); /* close function called if */
                                     /* the base server dies */

  baseConnect(1);                       /* hook up to running server*/
  colliConnect(1);

  RaiInit();                            /* init (but not start) scheduler*/
  catchInterrupts();
  initClientModules();                  /* set up Rai modules to do      */
                                        /* communication for you */
  createDemoModule();                   /* set up your module to move robot*/
  RaiStart();
  /* this will only return when RaiShutdown is called or ^C is used */
  return;
}
