
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
#include <time.h>

#include <baseClient.h>
#include <rai.h>

#include <bUtils.h>

/*------------------------------------------------------------*/

void
baseCallback(unsigned long opcode, unsigned long value)
{

  switch (opcode) {

   /* error conditions */
  case BASE_translateError:
  case BASE_rotateError:
  case BASE_batteryHigh:
  case BASE_batteryLow:
    fprintf(stderr, "%s:%6d:%s() - error code = %d\n",
	    __FILE__, __LINE__, __FUNCTION__, opcode);
    break;

  case BASE_rotateHalt:
    fprintf(stderr, "%s:%6d:%s() - BASE_rotateHalt\n",
	    __FILE__, __LINE__, __FUNCTION__);
    break;

  case BASE_translateHalt:
    fprintf(stderr, "%s:%6d:%s() - BASE_translateHalt\n",
	    __FILE__, __LINE__, __FUNCTION__);
    break;

   /* commands that return values, message is used when returning value too */
  case BASE_batteryCurrent:
    fprintf(stderr, "%s:%6d:%s() - BASE_batteryCurrent = %5.1f\n",
	    __FILE__, __LINE__, __FUNCTION__, (float)value/10.0);
    break;

  case BASE_batteryVoltage:
    fprintf(stderr, "%s:%6d:%s() - BASE_batteryVoltage = %5.1f\n",
	    __FILE__, __LINE__, __FUNCTION__, (float)value/10.0);
    break;

  case BASE_rotateCurrent:
    fprintf(stderr, "%s:%6d:%s() - BASE_rotateCurrent = %d\n",
	    __FILE__, __LINE__, __FUNCTION__, value);
    break;

  case BASE_rotateWhere:
    fprintf(stderr, "%s:%6d:%s() - BASE_rotateWhere = %d\n",
	    __FILE__, __LINE__, __FUNCTION__, value);
    break;

  case BASE_translateCurrent:
    fprintf(stderr, "%s:%6d:%s() - BASE_translateCurrent = %d\n",
	    __FILE__, __LINE__, __FUNCTION__, value);
    break;

  case BASE_translateWhere:
    fprintf(stderr, "%s:%6d:%s() - BASE_translateWhere = %d\n",
	    __FILE__, __LINE__, __FUNCTION__, value);
    break;


  case BASE_indexReport:
    fprintf(stderr, "%s:%6d:%s() - BASE_indexReport = %d\n",
	    __FILE__, __LINE__, __FUNCTION__, value);
    break;

  default:
    fprintf(stderr, "%s:%6d:%s() - unexpected opcode = %d, value= %d\n",
	    __FILE__, __LINE__, __FUNCTION__, opcode, value);
  }

  return;
}

/*------------------------------------------------------------*/

void
demoPoll(RaiModule * demoModule)
{
  static int direction = 0;

  fprintf(stderr, "%s:%6d:%s() - direction = %d\n",
	  __FILE__, __LINE__, __FUNCTION__, direction);

  if (direction) {
    rotateRelativeNeg(0x40);
    translateRelativeNeg(1000);
    direction = 0;
  }
  else {
    rotateRelativePos(0x40);
    translateRelativePos(1000);
    direction = 1;
  }
    
  return;
}

/*------------------------------------------------------------*/

void
createDemoModule()
{
  RaiModule* demoModule;

  statusReportPeriod(100*256/1000); /* Units = seconds*256         */
  registerBaseCallback(baseCallback);

  /* ask that your polling function be run every 5000msec */
  demoModule=makeModule("Demo", NULL);
  addPolling(demoModule, demoPoll, 5000);

  return;
}

/*------------------------------------------------------------*/

void
commShutdown(char *name, TCX_MODULE_PTR module)
{

  fprintf( stderr, "%s(%s): %s died. Closing. \n", 
	   __FILE__, __FUNCTION__, name );
  fflush(NULL);
  module = module;		/* prevents gcc from barking with `-Wall' */
  RaiShutdown();
  exit(0);
}
  
/*------------------------------------------------------------*/

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

  initClient("test", commShutdown); /* close function called if */
                                      /* the base server dies */

  baseConnect(1);                   /* hook up to running server*/

  RaiInit();                          /* init (but not start) scheduler*/
  catchInterrupts();

  initClientModules();                /* set up Rai modules to do      */
                                      /* communication for you */

  createDemoModule();                 /* set up your module to move robot*/

  RaiStart(); /* This will not return */
  exit(0);
}
