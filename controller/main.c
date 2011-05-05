
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






#include <ctype.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#ifndef i386
#include <sys/unistd.h>
#endif
#include <signal.h>
#include "all.h"
#include "tcx.h"
#include "tcxP.h"
#define MAX_RANDOM 1073741823.5
#define RAND() ((((float) random())/MAX_RANDOM)-1.0)
#define RAND_POS() ((((float) random())/MAX_RANDOM)/2.0)
#define pi  3.14159265358979323846
#define pi2 6.28318530717958647692


ALL_TYPE                 allGlobal;

/**********8**********8**********8**********8**********8*********/

int block_wait(struct timeval *timeout, int tcx_initialized,
	       int X_initialized);


struct timeval block_waiting_time = {1, 0};


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

struct bParamList * bParamList = NULL;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



/************************************************************************
 *
 *   NAME:          main routine
 *                 
 *   FUNCTION:     controls everything
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


int main(int argc, char **argv)
{
  ROBOT_SPECIFICATIONS     robot_specifications_data;
  PROGRAM_STATE            program_state_data;
  ROBOT_STATE              robot_state_data;
  ACTION                   action_data;
  SENSATION                sensation_data;

  ROBOT_SPECIFICATIONS_PTR robot_specifications = &robot_specifications_data;
  PROGRAM_STATE_PTR        program_state        = &program_state_data;
  ROBOT_STATE_PTR          robot_state          = &robot_state_data;
  ACTION_PTR               action               = &action_data;
  SENSATION_PTR            sensation            = &sensation_data;

  struct timeval TCX_waiting_time = {0, 0};

  robot_specifications->is_initialized = 0;
  program_state->is_initialized        = 0;
  robot_state->is_initialized          = 0;
  action->is_initialized               = 0;
  sensation->is_initialized            = 0;
  allGlobal.is_initialized             = 0;

  signal(SIGTERM, &interrupt_handler); /* kill interupt handler */
  signal(SIGINT,  &interrupt_handler); /* control-C interupt handler */
#if 0
  signal(SIGALRM, &alarm_handler);	/* handler for regular interrupts */
#endif  

  bParamList = bParametersAddEntry(bParamList, "robot", "name", "B21");
  bParamList = bParametersAddEntry(bParamList, "", "TCXHOST", "localhost");
  bParamList  = bParametersAddEntry(bParamList, "", "fork", "yes");

  /* add some parameter files */
  bParamList = bParametersAddFile (bParamList, "etc/beeSoft.ini");

  /* add some enviroment variables */
  bParamList = bParametersAddEnv(bParamList, "", "TCXHOST");

  bParamList = bParametersAddArray(bParamList, "", argc, argv);

  bParametersFillParams(bParamList);

  check_commandline_parameters(argc, argv, ALL);
  init_program(ALL);
  if (!load_parameters(RHINO_INIT_NAME, ALL))
    exit(-1);
  allocate_memory(ALL);
  /*save_parameters(RHINO_INIT_NAME, ALL);*/
  init_graphics(ALL);
  connect_to_tcx(ALL);
  G_display_switch(TITLE_BUTTON, 1);
#if 0
  alarm((unsigned) robot_specifications->alarm_interval); /* set up alarm */
#endif
  tcx_reset_joystick(ALL); 


  for (;!program_state->quit;){
    program_state->something_happened = 0;
    
    if (program_state->tcx_autoconnect)
      connect_to_tcx(ALL);

    if (test_mouse(ALL))       program_state->something_happened = 1;
    
    if (refresh_action(ALL))   program_state->something_happened = 1;

    if (initiate_action(ALL))  program_state->something_happened = 1;

    /* if (terminate_action(ALL)) program_state->something_happened = 1; */
    

    if (program_state->tcx_initialized){
      TCX_waiting_time.tv_sec   = 0;
      TCX_waiting_time.tv_usec  = 0;
      tcxRecvLoop((void *) &TCX_waiting_time);
    }

    if (!program_state->something_happened){
      block_waiting_time.tv_sec  = 1;
      block_waiting_time.tv_usec = 0;
      block_wait(&block_waiting_time, program_state->tcx_initialized,
		 program_state->graphics_initialized);  
    }

#ifdef TOURGUIDE_VERSION
    tourguide_check_for_timeout(ALL);
#endif /* TOURGUIDE_VERSION */

  }



  /* close_log_file(ALL);*/
  /*  if (program_state->tcx_initialized)
      tcxCloseAll();*/
  G_display_switch(TITLE_BUTTON, 2);
  fprintf(stderr, "\nGood-bye.\n");
  sleep(1);
  exit(0);

}
