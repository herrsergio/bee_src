
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
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <math.h>

#include "tcx.h"
#include "tcxP.h"
#include "Common.h"
#include "libc.h"
#include "LIB-math.h"
#include "ROBOT.h"
#include <bUtils.h>

#if defined(__STDC__) && defined(__GCC_NEW_VARARGS__)
typedef void (*VOID_FN)(void);
#elif  defined(__STDC__)
typedef void (*VOID_FN)(void);
#else /* __STDC__ */
typedef void (*VOID_FN)();
#endif

typedef struct command {
  char *name;
  VOID_FN func;
  int  n_parameters;
} command;


BOOLEAN use_collision;
BOOLEAN use_simulator = FALSE;


extern int sonar_on;
extern int status_on;
extern int colli_on;
extern int laser_on;
extern int verbose_on;

struct bParamList * bParamList = NULL;

/***************************************************************************
 * main                                                                    *
 *                                                                         *
 * Parameters : argc, *argv[]                                              *
 * Returns    : Nothing                                                    *
 * Does       : Checking options, starts up initialization , starts router *
 * Called     : Well it's - main.                                          *
 ***************************************************************************/
int main(int argc, char *argv[])
{
  char    machine[20];
  char    log_file[20];
  int     i;


  /* add some parameter files */
  bParamList = bParametersAddFile(bParamList, "etc/beeSoft.ini");
  
  /* add some enviroment variables */
  bParamList = bParametersAddEnv(bParamList, "", "TCXHOST");
  
  /* add command line arguements */
  bParamList = bParametersAddArray(bParamList, "", argc, argv);
  
  /* Fill the global parameter structure */
  bParametersFillParams(bParamList);
  
  for (i=1; i<argc; i++) {
    if ((strcmp(argv[i],"-nostatus")==0))
      status_on = 0;
    else if ((strcmp(argv[i],"-nosonar")==0))
      sonar_on = 0;
    else if ((strcmp(argv[i],"-usecolli")==0))
      colli_on = 1;
    else if ((strcmp(argv[i],"-uselaser")==0))
      laser_on = 1;
    else if ((strcmp(argv[i],"-verbose")==0))
      verbose_on = 1;
    else {
      fprintf (stderr, "\nusage: ROUTER [-nostatus (no status autoreply)]");
      fprintf (stderr, "\n              [-nosonar  (no sonar autoreply)]");
      fprintf (stderr, "\n              [-usecolli (starts colli autoreply)]");
      fprintf (stderr, "\n              [-uselaser (starts laser autoreply)]");
      fprintf (stderr, "\n              [-verbose  (displays some information)]\n");
 
#if 0
      exit(-1);
#endif
    }
  }

  init_tcx();

  if (bRobot.fork) {
    bDaemonize("router.log");
  }
  
  MainControlLoop();
  /*  signal(SIGINT,  (void *) Quit); */
  /*  signal(SIGBUS,  (void *) Quit);*/
  /*  signal(SIGSEGV, (void *) Quit);*/


  exit(0);
}
