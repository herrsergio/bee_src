
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

enum squareEvent {SQE_START, SQE_END, SQE_ROT_HALT, SQE_TRAN_HALT};

enum squareState {SQS_START, SQS_END,
		  SQS_S1, SQS_S2, SQS_S3, SQS_S4,
		  SQS_C1, SQS_C2, SQS_C3, SQS_C4};

/*------------------------------------------------------------*/

void
squareStateMachine(enum squareEvent event)
{
  static enum squareState state = SQE_START;

  switch(event) {
  case SQE_START:
    switch(state) {
    case SQS_START:
      translateRelativePos(1000);
      fprintf(stderr, "%s:%6d:%s() - Starting side 1\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_S1;
      break;

    case SQS_END:
    case SQS_S1:
    case SQS_S2:
    case SQS_S3:
    case SQS_S4:
    case SQS_C1:
    case SQS_C2:
    case SQS_C3:
    case SQS_C4:
      translateLimp();
      rotateLimp();
      fprintf(stderr, "%s:%6d:%s() - Unexpected SQE_START\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_END;

    default:
      translateLimp();
      rotateLimp();
      fprintf(stderr, "%s:%6d:%s() - Unexpected state\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_END;
      break;
    }
    break;

  case SQE_END:
    switch(state) {
    case SQS_START:
    case SQS_END:
    case SQS_S1:
    case SQS_S2:
    case SQS_S3:
    case SQS_S4:
    case SQS_C1:
    case SQS_C2:
    case SQS_C3:
    case SQS_C4:
      translateLimp();
      rotateLimp();
      state = SQS_END;
      break;

    default:
      translateLimp();
      rotateLimp();
      fprintf(stderr, "%s:%6d:%s() - Unexpected state\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_END;
      break;
    }
    break;
    
  case SQE_ROT_HALT:
    switch(state) {
    case SQS_START:
    case SQS_END:
    case SQS_S1:
    case SQS_S2:
    case SQS_S3:
    case SQS_S4:
      translateLimp();
      rotateLimp();
      fprintf(stderr, "%s:%6d:%s() - Unexpected event\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_END;
      break;

    case SQS_C1:
      translateRelativePos(1000);
      fprintf(stderr, "%s:%6d:%s() - Starting side 2\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_S2;
      break;

    case SQS_C2:
      translateRelativePos(1000);
      fprintf(stderr, "%s:%6d:%s() - Starting side 3\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_S3;
      break;

    case SQS_C3:
      translateRelativePos(1000);
      fprintf(stderr, "%s:%6d:%s() - Starting side 4\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_S4;
      break;

    case SQS_C4:
      translateLimp();
      rotateLimp();
      fprintf(stderr, "%s:%6d:%s() - Done.\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_END;
      break;

    default:
      translateLimp();
      rotateLimp();
      fprintf(stderr, "%s:%6d:%s() - Unexpected state\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_END;
      break;
    }
    break;
    
  case SQE_TRAN_HALT:
    switch(state) {
    case SQS_START:
    case SQS_END:
      translateLimp();
      rotateLimp();
      fprintf(stderr, "%s:%6d:%s() - Unexpected event\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_END;
      break;

    case SQS_S1:
      rotateRelativePos(0x100);
      fprintf(stderr, "%s:%6d:%s() - Starting corner 1\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_C1;
      break;

    case SQS_S2:
      rotateRelativePos(0x100);
      fprintf(stderr, "%s:%6d:%s() - Starting corner 2\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_C2;
      break;

    case SQS_S3:
      rotateRelativePos(0x100);
      fprintf(stderr, "%s:%6d:%s() - Starting corner 3\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_C3;
      break;

    case SQS_S4:
      rotateRelativePos(0x100);
      fprintf(stderr, "%s:%6d:%s() - Starting corner 4\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_C4;
      break;

    case SQS_C1:
    case SQS_C2:
    case SQS_C3:
    case SQS_C4:
      translateLimp();
      rotateLimp();
      fprintf(stderr, "%s:%6d:%s() - Unexpected event\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_END;
      break;

    default:
      translateLimp();
      rotateLimp();
      fprintf(stderr, "%s:%6d:%s() - Unexpected state\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_END;
      break;
    }
    break;
    
  default:
      translateLimp();
      rotateLimp();
      fprintf(stderr, "%s:%6d:%s() - Unexpected event\n",
	      __FILE__, __LINE__, __FUNCTION__);
      state = SQS_END;
      break;
  }

  if (state == SQS_END) {
    RaiShutdown();
    exit(0);
  }

  return;
}

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
    squareStateMachine(SQE_ROT_HALT);
    break;

  case BASE_translateHalt:
    fprintf(stderr, "%s:%6d:%s() - BASE_translateHalt\n",
	    __FILE__, __LINE__, __FUNCTION__);
    squareStateMachine(SQE_TRAN_HALT);
    break;

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
  /*
   * Could put some time based events here.
   */

  return;
}

/*------------------------------------------------------------*/

void
demoStart(RaiModule * demoModule)
{

  setTranslateVelocity(500);       /* units = mm/sec    */
  setTranslateAcceleration(1000);  /* units = mm/sec^2  */

  setRotateVelocity(0x40);          /* units = radians*512/PI/sec  */
  setRotateAcceleration(0x100);     /* units = radians*1024/2PI/sec  */
  
  /*
   * Send a start event to the state machine.
   */

  squareStateMachine(SQE_START);

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
  addTimeout(demoModule, demoStart, 1000);
  return;
}

/*------------------------------------------------------------*/

void
commShutdown(char *name, TCX_MODULE_PTR module)
{

  fprintf( stderr, "%s(%s): %s died. Closing. \n", 
	   __FILE__, __FUNCTION__, name );

  module = module;		/* prevents gcc from barking with `-Wall' */
  fflush(NULL);
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
