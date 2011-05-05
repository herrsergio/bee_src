
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

#include <tcx.h>
#include <tcxP.h>
#include <rai.h>
#include <raiClient.h>
#include <bUtils.h>

#include <signal.h>
#include <laserClient.h>
#include <Common.h>

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void commShutdown(char *name, TCX_MODULE_PTR module)
{

#if ( defined(G_DEBUG_WHERE_AM_I) )
  fprintf( stderr, "%s: %s\n", __FILE__, __FUNCTION__ );
#endif

  fprintf( stderr, "%s(%s): %s died. \n", 
	   __FILE__, __FUNCTION__, name );

  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    fprintf( stderr, "Exiting.\n" );
    RaiShutdown();
    exit( -1 );
  }

  if (!strcmp(name, "laserServer")){
    laserConnected = 0;
  }

  module = module;		/* gcc's -Wall */

}

/* ---------------------------------------------------------
 *
 * 
 *
 * --------------------------------------------------------*/
void ctrlcShutdown() 
{
  commShutdown("Ctrl-C",NULL);

  RaiShutdown();

  exit( -1 );

}

/* ---------------------------------------------------------
 *
 * pretty dumb callback function that simply prints out
 * all values within the sweep
 *
 * --------------------------------------------------------*/
int myLaserSweepCallback ( laserSweepType *data ) {

  int i;

  fprintf( stderr, "%s: %s\n", __FILE__, __FUNCTION__ );

  fprintf( stderr,"This reading comes from laser num %d.\n", data->numLaser );

  for ( i=0; i<data->numberLasers; i++ ) 
    fprintf( stderr,"%5.0f ", data->value[i] );
  
  fprintf( stderr,"\n" );

  return 0;
}

/* ---------------------------------------------------------
 *
 * is called every 200ms from the polling routine
 *
 * --------------------------------------------------------*/
void CommandLoop( RaiModule *mod )
{

  mod = mod;

  if ( laserConnected == 0 ) {
  
    laserConnect( 0 );		/* try connecting  */
    laserSubscribeSweep( 0, 1 ); /* 0 == first laser, 1 = every reading*/

  }

}

/* ---------------------------------------------------------
 *
 * 
 *
 * --------------------------------------------------------*/
void createLaserExampleModule() {
 
  RaiModule *LaserExampleModule;
 
  LaserExampleModule = makeModule( "laserExample", NULL) ;  

  registerLaserSweepCallback( myLaserSweepCallback );
  
  addPolling( LaserExampleModule, CommandLoop, 200 );
  
  fprintf( stderr, "Done.\n" );
  
}

/* ---------------------------------------------------------
 *
 * 
 *
 * --------------------------------------------------------*/
int main( int argc, char** argv ) {

  struct bParamList * params = NULL;
/*   struct timeval TCX_waiting_time = {0, 0}; */

  /* add some parameter files */
  params = bParametersAddFile(params, "etc/beeSoft.ini");
  
  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(params);

  laserRegister();

  initClient( "laserExample", commShutdown); /* close function called if */
                                             /* the server dies          */

  laserConnect( 0 );		/* 1 -> wait until connection has been established */

  RaiInit();			/* init (but not start) scheduler   */

  catchInterrupts();

  initClientModules();		/* set up Rai modules to do         */
				/* communication for you            */

  createLaserExampleModule();

  /* whenever user hits CTRL-C */
  signal( SIGINT, &ctrlcShutdown );

  /* laserRequestSweep( 0 ); */  /*  0 == first laser */
  
  laserSubscribeSweep( 0, 1 );	/* 0 == first laser, 1 = every reading*/

  RaiStart();

  return 0;
}

/*
 * $Log: laserExample.c,v $
 * Revision 1.7  1998/02/03 18:06:29  swa
 * Updated documentation to include reconnection. Removed some comments.
 *
 * Revision 1.6  1998/01/15 15:29:30  swa
 * Changed the laserClient lib to allow reconnects and modified laserExample to
 * automatically reconnect to the laserServer once the server dies.
 *
 * Revision 1.5  1998/01/14 16:45:11  swa
 * Changed clientCloseFcn to take two arguments, the name of the module
 * that died and the TCX_MODULE_PTR module. Now one can distinguish in the
 * close function between modules that die.
 *
 * Revision 1.4  1997/08/07 15:46:47  swa
 * Slightly changed the example so that it displays the laser number in the
 * callback.
 *
 * Revision 1.3  1997/08/07 15:42:53  swa
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
 */
