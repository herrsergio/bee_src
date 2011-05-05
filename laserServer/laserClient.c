
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
#ifndef lint
static char rcsid[] =
"$Id";
static void *const use_rcsid = ((void)&use_rcsid, (void)&rcsid, 0);
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <ctype.h>
#include <sys/time.h>
#include <tcx.h>
#include <tcxP.h>
#include <rai.h>
#define TCX_define_variables
#define DEFINE_REPLY_HANDLERS
#include "devUtils.h"
#include "io.h"
#include <LASER_SERVER-messages.h>
#include <laserClient.h>
#include <raiClient.h>

#define LASER_CLIENT_NAME "laserclient"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

struct timeval LASER_last_send_message_time = {0, 0};

int laserConnected = 0;

static int sweep0_subscription = 0;
static int sweep1_subscription = 0;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

laserSweepCallbackType userLaserFcn=NULL;

void registerLaserSweepCallback(laserSweepCallbackType fcn)
{
  userLaserFcn=fcn;
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/
void LASER_SERVER_sweep_reply_handler( TCX_REF_PTR           ref,
				       LASER_SERVER_sweep_reply_ptr data ) {
  laserSweepType info;

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  info.numberLasers = data->numberLasers;
  info.numLaser     = data->numLaser;
  info.value        = data->value;
  
  if (userLaserFcn != NULL) {
    userLaserFcn( &info );
  } else {
    fprintf( stderr,"%s: no callback registered for %s from %s\n",
	     __FILE__, __FUNCTION__, tcxModuleName(ref->module) );
  }

  tcxFree("LASER_SERVER_sweep_reply", data );

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/
void laserRequestSweep( int numLaser ) {

  LASER_SERVER_sweep_query_type specs;

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if (laserConnected){
    specs.numLaser = numLaser;
    tcxSendMsg(LASER_SERVER, "LASER_SERVER_sweep_query", &specs);
  }
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/
void laser_subscribe() {

  LASER_SERVER_register_auto_update_type subscribe;

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if ( laserConnected ){
    subscribe.sweep0        = sweep0_subscription;
    subscribe.sweep1        = sweep1_subscription;
    tcxSendMsg(LASER_SERVER, "LASER_SERVER_register_auto_update", &subscribe);
  } else {
    fprintf(stderr,
	    "%s(%s): laserServer is not connected. \n", __FILE__, __FUNCTION__);
    laserConnect( 0 );
  }

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/
void laserSubscribeSweep( int numLaser, int number )
{
  if ( numLaser == 0 ) {

    sweep0_subscription = number;
    
  } else if ( numLaser == 1 ) {

    sweep1_subscription = number;

  } else {
    fprintf( stderr,"%s Error in line %d.\n", __FILE__, __LINE__ );
    return;
  } 
  
#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  laser_subscribe();
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* ---------------------------------------------------------
 *
 * hook up to laserServer. if laserServer dies, try reconnecting 
 * every three seconds
 *
 * --------------------------------------------------------*/
int
laserConnect( int wait_till_established )
{
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference;

#if ( defined(G_DEBUG_TCX) )
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);

  if (time_difference < 3.0)
    return -1;

  if ( wait_till_established ) { /* 1 */

    fprintf(stderr, "LaserClient: Connecting to Laser server...\n");
    LASER_SERVER = tcxConnectModule(TCX_LASER_SERVER_MODULE_NAME);
    laserConnected = 1;
    fprintf(stderr, "LaserClient: Connected.\n");

  } else {			/* 0 */

    if ( laserConnected == 0 || !LASER_SERVER ) { /* doppelt haelt besser */
      fprintf(stderr, "LaserClient: Connecting to Laser server...\n");
      LASER_SERVER  = tcxConnectOptional(TCX_LASER_SERVER_MODULE_NAME);
      if( LASER_SERVER ) {
	laserConnected = 1;
	laser_subscribe();  
	fprintf(stderr, "LaserClient: Connected.\n");
      } else {
	laserConnected = 0;
      }
    }

  }

  last_time.tv_sec  = this_time.tv_sec;
  last_time.tv_usec = this_time.tv_usec;

  return 0;
}

int _laserConnect(int wait_till_established) {

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if (wait_till_established){
    fprintf(stderr, "LaserClient: Connecting to Laser Server...\n");
    LASER_SERVER = tcxConnectModule(TCX_LASER_SERVER_MODULE_NAME);
  } else {
    LASER_SERVER = tcxConnectOptional(TCX_LASER_SERVER_MODULE_NAME);
  }
  
  if ( LASER_SERVER ){

    laser_subscribe();  
    
    laserConnected = 1;

    fprintf(stderr, "LaserClient: Connected.\n");

  } else 
    laserConnected = 0;
  
  return 0;

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/
void laserRegister() {

  int numberOfMessages;
  int numberOfHandlers;

  TCX_REG_MSG_TYPE messages[] = { LASER_SERVER_messages }; 

#if ( defined(TCX_debug) )
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  numberOfMessages = sizeof(messages)/sizeof(TCX_REG_MSG_TYPE); 

  numberOfHandlers = sizeof(LASER_SERVER_reply_handler_array)
    /sizeof(TCX_REG_HND_TYPE); 

  registerInterface( "", 
		     numberOfMessages, 
		     messages,
		     numberOfHandlers, 
		     LASER_SERVER_reply_handler_array );

}

/*
 * $Log: laserClient.c,v $
 * Revision 1.5  1998/01/15 15:29:30  swa
 * Changed the laserClient lib to allow reconnects and modified laserExample to
 * automatically reconnect to the laserServer once the server dies.
 *
 * Revision 1.4  1998/01/13 22:41:41  thrun
 * resolved a naming conflict. CHanged LASER to LASER_SERVER.
 *
 * Revision 1.3  1997/08/07 15:42:52  swa
 * First working version of the laserServer. Seems to run fine. Has support for
 * a second laser clientwise, yet the device stuff for the second one is not
 * implemented.
 *
 * Revision 1.2  1997/08/07 03:50:21  swa
 * Fixed a bunch of bugs. Still not working.
 *
 * Revision 1.1  1997/08/07 02:45:51  swa
 * First version of a laserServer. Compiles nicely, yet produces nothing useful. ;)
 * Just to copy things to a safe place ...
 *
 *
 */
