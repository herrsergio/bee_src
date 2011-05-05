
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






/*****************************************************************************
 * PROJECT: Rhino
 *
 * FILE: ROBOT.c
 *
 * ABSTRACT:
 * 
 * This file provides a set for routines for interfacing to the robot.
 *
 *****************************************************************************/

#ifdef VMS
#include "vms.h"
#include "vmstime.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#ifndef VMS
#include <sys/ioctl.h>
#endif
#include <sys/time.h>

#include "tcx.h"
#include "tcxP.h"
#include "Common.h"
#include "libc.h"
#include "ROBOT.h"

/*****************************************************************************
 *
 * FUNCTION: void signal_base(void);
 *
 * DESCRIPTION:
 *
 * Interrupt handler for the base.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

void signal_base(void)
{
  fprintf(stderr,"shutting down the base\n");
  stop_robot();
}


/*****************************************************************************
 *
 * FUNCTION: BOOLEAN start_robot()
 *
 * DESCRIPTION:
 *
 * This routine starts base device on the robot.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/


BOOLEAN start_robot(void)
{
  BOOLEAN result=TRUE;

  
  /* do any device specific set up here. */
  
  /* Start up base device */
  result = BASE_init();

  /* connect up the signal handler. */
  if (!base_device.dev.use_simulator && !base_device.dev.use_rwi_server)
    base_device.dev.sigHnd = signal_base;
  
  return result;
}


void stop_robot()
{

  BASE_RotateHalt();
  BASE_TranslateHalt();
  
  fprintf(stderr, "Someone stopped the robot!\n");
}




