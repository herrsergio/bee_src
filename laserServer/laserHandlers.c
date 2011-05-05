
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
#include <stdlib.h>
#include <stdio.h>
#include <rai.h>
#include <bUtils.h>
#include "tcx.h"
#include "robot_specifications.h"
#include "tcxP.h"
#include "global.h"
#define TCX_define_variables  /* this makes sure variables are installed */
#include "devUtils.h"
#include "io.h"
#include "LASER_SERVER-messages.h"
#include <signal.h>
#include <sys/mman.h>
#include <Common.h>
#include "laserHandlers.h"
#include "laserClient.h"
#include "beeSoftVersion.h"
#include "raiClient.h"
#include "mainlaser.h"

void commShutdown( );

TCX_REG_HND_TYPE LASER_SERVER_handler_array[] = {

  {"LASER_SERVER_sweep_query", "LaserSweepQueryHandler",
   LaserSweepQueryHandler, TCX_RECV_ALL, NULL},
  
  {"LASER_SERVER_register_auto_update", "LaserRegisterAutoUpdateHandler",
   LaserRegisterAutoUpdateHandler, TCX_RECV_ALL, NULL}

};

/******* auto-reply stuff, data definitions ***************/


#define MAX_N_AUTO_UPDATE_MODULES 100


int n_auto_update_modules = 0; /* number of processes to whom
				* position should be forwarded
				* automatically upon change */

typedef struct{
  TCX_MODULE_PTR module;	/* pointer to TCX module */
  int            sweep0;	/* >=1=subscribed to regular sweep updates */
  int            last_sweep0;
  int            sweep1;	/* >=1=subscribed to regular sweep updates */
  int            last_sweep1;
} auto_update_type;


auto_update_type auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];
                                /* collection of TCX-module ptrs */

/************************************************************************
 *   FUNCTION:     Counts, how many modules of the different types require
 *                 auto-updates
 *   RETURN-VALUE: 1, if successful, 0 if not
 ************************************************************************/
void count_auto_update_modules() {

  int i;

  n_auto_sweep0_update_modules      = 0;
  n_auto_sweep1_update_modules      = 0;

  for (i = 0; i < n_auto_update_modules; i++) {

    if (auto_update_modules[i].sweep0)
      n_auto_sweep0_update_modules++;

    if (auto_update_modules[i].sweep1)
      n_auto_sweep1_update_modules++;

  }

}

/************************************************************************
 *   FUNCTION:     Adds a module to the list of modules that
 *                 get automatical map updates
 *   RETURN-VALUE: 1, if successful, 0 if not
 ************************************************************************/
static int add_auto_update_module( TCX_MODULE_PTR                 module,
				   LASER_SERVER_register_auto_update_ptr data)
{
  int i;

  if (n_auto_update_modules >= MAX_N_AUTO_UPDATE_MODULES){

    fprintf(stderr, "ERROR: auto-module memory overflow. Request rejected.\n");

    return 0;

  } else for (i = 0; i < n_auto_update_modules; i++)

    if (auto_update_modules[i].module == module) {
      fprintf( stderr, 
	       "Module %s already known. Subscription modified: %d %d\n",
	       tcxModuleName(module), 
	       data->sweep0,
	       data->sweep1 );
      auto_update_modules[i].sweep0      = data->sweep0; /* subsrc? */
      auto_update_modules[i].last_sweep0 = -1;
      auto_update_modules[i].sweep1      = data->sweep1; /* subsrc? */
      auto_update_modules[i].last_sweep1 = -1;
      count_auto_update_modules();
      return 1;
    }

  fprintf( stderr, "Add %s to auto-reply list: %d %d.\n",
	   tcxModuleName(module), 
	   data->sweep0,
	   data->sweep1 );

  auto_update_modules[n_auto_update_modules].module     = module; /* pointer*/
  auto_update_modules[n_auto_update_modules].sweep0      = data->sweep0;
  auto_update_modules[n_auto_update_modules].last_sweep0 = -1;
  auto_update_modules[n_auto_update_modules].sweep1      = data->sweep1; 
  auto_update_modules[n_auto_update_modules].last_sweep1 = -1;

  n_auto_update_modules++;
  count_auto_update_modules();

  return 1;
}

/************************************************************************
 *   FUNCTION:     Attempts to remove a module from the list of 
 *                 modules that get automatical map updates 
 *   RETURN-VALUE: 1, if successful, 0 if not
 ************************************************************************/
static int remove_auto_update_module(TCX_MODULE_PTR module) {     

  int i, j, found = 0;

  for (i = 0; i < n_auto_update_modules; i++) /* search for module */

    if (auto_update_modules[i].module == module){ /* if module found */

      fprintf( stderr, "Remove %s from auto-reply list.\n", tcxModuleName(module));

      found++;

      n_auto_update_modules--;	/* remove that entry, one less now */

      for (j = i; j < n_auto_update_modules; j++){
	auto_update_modules[j].module = auto_update_modules[j+1].module;

	auto_update_modules[j].sweep0 =  auto_update_modules[j+1].sweep0;
	auto_update_modules[j].last_sweep0 = auto_update_modules[j+1].last_sweep0;

	auto_update_modules[j].sweep1 =  auto_update_modules[j+1].sweep1;
	auto_update_modules[j].last_sweep1 = auto_update_modules[j+1].last_sweep1;

      }
    }

  if (!found)
    fprintf(stderr, "(%s: no auto-replies removed)\n", tcxModuleName(module));

  count_auto_update_modules();

  return found;

}

/************************************************************************
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 ************************************************************************/
void send_automatic_sweep_update( int numLaser, float *values ) {

  int i;

#if ( defined(TCX_debug) )
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if ( numLaser == 0 ) { 

    for (i = 0; i < n_auto_update_modules; i++){
      if (auto_update_modules[i].sweep0 > 0){
	
	auto_update_modules[i].last_sweep0++;
	if (auto_update_modules[i].last_sweep0 %
	    auto_update_modules[i].sweep0 == 0){
#ifdef LASER_debug	
	  fprintf(stderr, "Send sweep from laser 0 to %s.\n", 
		  tcxModuleName(auto_update_modules[i].module));
#endif
	  auto_update_modules[i].last_sweep0 = 0;
	  
	  LaserSendSweepTo( auto_update_modules[i].module, 0, values );
	}
      }
    }

  } else if ( numLaser == 1 ) {

    for (i = 0; i < n_auto_update_modules; i++){
      if (auto_update_modules[i].sweep1 > 0){
	
	auto_update_modules[i].last_sweep1++;
	if (auto_update_modules[i].last_sweep1 %
	    auto_update_modules[i].sweep1 == 0){
#ifdef LASER_debug	
	  fprintf(stderr, "Send sweep from laser 1 to %s.\n", 
		  tcxModuleName(auto_update_modules[i].module));
#endif
	  auto_update_modules[i].last_sweep1 = 0;
	  
	  LaserSendSweepTo( auto_update_modules[i].module, 1, values );
	}
      }
    }

  } else {

    /* ouch! */

  }

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void LaserRegisterAutoUpdateHandler( TCX_REF_PTR  ref,
					 LASER_SERVER_register_auto_update_ptr data)
{
  fprintf(stderr, "Received an auto update request from %s: %d %d.",
	tcxModuleName(ref->module), data->sweep0, data->sweep1);

  add_auto_update_module(ref->module, data);
  
  if (data != NULL){
    tcxFree("LASER_SERVER_register_auto_update", data);
    data = NULL;
  }
}

/************************************************************************
 *   FUNCTION:     general initialization routine - must be called before
 *                 anything else. Returns error value (0=success, 1=error)
 ************************************************************************/
int LaserInitializeTCX() {
  
  /* messages used by LASER */
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    LASER_SERVER_messages,
  };
  
  /* TCX */
  
  fprintf(stderr, "Connecting to TCX...");

  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize(TCX_LASER_SERVER_MODULE_NAME, tcxMachine);

  check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
		       BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
		       NULL, 0);

  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));
  
  /* Handlers */
  
  tcxRegisterHandlers(LASER_SERVER_handler_array, 
		      sizeof(LASER_SERVER_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterCloseHnd(LaserCloseHandler);
  
  /* Return */
  return (1) ; 
  
}

/************************************************************************
 *   FUNCTION:     handles a close message (special case)
 ************************************************************************/
void LaserCloseHandler(char *name, TCX_MODULE_PTR module) {

#ifdef LASER_debug
  fprintf(stderr, "LASER: closed connection detected: %s\n", name);
#endif

  remove_auto_update_module(module);

  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    commShutdown(NULL);
  }

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void LaserSweepQueryHandler( TCX_REF_PTR            ref,
			     LASER_SERVER_sweep_query_ptr  data ) {

#ifdef TCX_debug
  fprintf(stderr, "Received a  LASER_SERVER_sweep_query_message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  if ( data->numLaser == 0 ) 
    LaserSendSweepTo( ref->module, 0, LaserSensors.values );

  else if ( data->numLaser == 1 ) 
    fprintf( stderr,"%s: laser 1 not yet supported\n", __FILE__ );

  else {
    /* ouch */
  }

  tcxFree( "LASER_SERVER_sweep_query", data );

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void LaserSendSweepTo( TCX_MODULE_PTR module, int numLaser, float *values ) {

  LASER_SERVER_sweep_reply_type sweep;
  int i;
  
  /* get the current reading and store the stuff in sweep.value[i] */
  sweep.numberLasers = NUMBER_LASERS;

  sweep.numLaser = numLaser;

  sweep.value = (float *) malloc(sizeof(float) * NUMBER_LASERS);
  
  for (i=0; i<NUMBER_LASERS; i++)
    sweep.value[i] = values[i];

#ifdef TCX_debug 
  fprintf(stderr, "TCX: Sending LASER_SERVER_sweep_reply.\n");
#endif

  tcxSendMsg(module, "LASER_SERVER_sweep_reply", &sweep);

  free( sweep.value );

}

/*
 * $Log: laserHandlers.c,v $
 * Revision 1.6  1998/01/13 23:13:38  thrun
 * .
 *
 * Revision 1.5  1998/01/13 22:41:41  thrun
 * resolved a naming conflict. CHanged LASER to LASER_SERVER.
 *
 * Revision 1.4  1997/11/06 17:56:21  swa
 * Fixed a bug with TCXHOST. If the environment variable TCXHOST is set,
 * take it. If not, take the TCXHOST from beeSoft.ini. If this doesn't
 * work, take localhost. If this doesn't work: bummer.
 *
 * Revision 1.3  1997/08/07 15:42:53  swa
 * First working version of the laserServer. Seems to run fine. Has support for
 * a second laser clientwise, yet the device stuff for the second one is not
 * implemented.
 *
 * Revision 1.2  1997/08/07 03:50:21  swa
 * Fixed a bunch of bugs. Still not working.
 *
 * Revision 1.1  1997/08/07 02:45:52  swa
 * First version of a laserServer. Compiles nicely, yet produces nothing useful. ;)
 * Just to copy things to a safe place ...
 *
 *
 */
