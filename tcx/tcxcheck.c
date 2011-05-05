
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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <signal.h>

#include "tcx.h"
#include "tcxP.h"
#include "global.h"

/*
 * Routine connects to TCX, and checks if a module with the name
 * moduleName is already running. If so it prints an error message
 * and returns 0 - otherwise 1.
 *
 * The purpose of this command is to detect + prevent having multiple
 * modules of the same type running at the same time. Uff.
 */

void
tcxInitialize(char *moduleName, char *tcxHost)
{
  char tcxName[128], hostname[128], pidname[128];

  /*
   * create a unique name (using the PID and the hostname)
   */

  strcpy(tcxName, moduleName);
  sprintf(pidname, "_%d_", getpid());
  strcat(tcxName, pidname);
  gethostname(hostname, 32);
  strcat(tcxName, hostname); 

  /*
   * initialize TCX
   */

  tcxInitializeInternal(tcxName, tcxHost);
  
  /*
   * and check if the module is already running
   */


  if (tcxConnectOptional(moduleName) == NULL){
    tcxCloseAll();
    Global->msgInitFlagGlobal = 0;
    tcxInitializeInternal(moduleName, tcxHost);
  }
  else{
    fprintf(stderr, "ERROR: You cannot run multiple copies of the same module.\n");
    exit(-1);
  }
}
