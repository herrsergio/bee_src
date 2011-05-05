
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
#include <buttonClient.h>
#include <Common.h>

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void commShutdown(char *name, TCX_MODULE_PTR module)
{

#if ( defined(BUTTON_DEBUG) )
  fprintf( stderr, "%s: %s\n", __FILE__, __FUNCTION__ );
#endif

  fprintf( stderr, "%s(%s): %s died. \n", 
	   __FILE__, __FUNCTION__, name );

  RaiShutdown();

  exit( -1 );

}

/* ---------------------------------------------------------
 *
 * 
 *
 * --------------------------------------------------------*/
void ctrlcShutdown() 
{
  commShutdown("Ctrl-C",NULL);
}

/* ---------------------------------------------------------
 *
 * 
 *
 * --------------------------------------------------------*/
int myButtonStatusCallback ( buttonStatusType *data ) {

#if ( defined(TCX_DEBUG) )
  fprintf( stderr,"\n----------- status --------------------\n" );
  fprintf( stderr,"red_light_status ............... : %d\n",
	   data->red_light_status );
  fprintf( stderr,"yellow_light_status ............ : %d\n",
	   data->yellow_light_status );
  fprintf( stderr,"green_light_status ............. : %d\n",
	   data->green_light_status );
  fprintf( stderr,"blue_light_status .............. : %d\n",
	   data->blue_light_status );
  fprintf( stderr,"left_kill_switch_light_status .. : %d\n",
	   data->left_kill_switch_light_status );
  fprintf( stderr,"right_kill_switch_light_status . : %d\n",
	   data->right_kill_switch_light_status );
  fprintf( stderr,"red_button_pressed ............. : %d\n",
	   data->red_button_pressed );
  fprintf( stderr,"red_button_changed ............. : %d\n",
	   data->red_button_changed );
  fprintf( stderr,"yellow_button_pressed .......... : %d\n",
	   data->yellow_button_pressed );
  fprintf( stderr,"yellow_button_changed .......... : %d\n",
	   data->yellow_button_changed );
  fprintf( stderr,"green_button_pressed ........... : %d\n",
	   data->green_button_pressed );
  fprintf( stderr,"green_button_changed ........... : %d\n",
	   data->green_button_changed );
  fprintf( stderr,"blue_button_pressed ............ : %d\n",
	   data->blue_button_pressed );
  fprintf( stderr,"blue_button_changed ............ : %d\n",
	   data->blue_button_changed );
#endif
  
  return 0;
}

/* ---------------------------------------------------------
 *
 * 
 *
 * --------------------------------------------------------*/
int main( int argc, char** argv ) {

  struct bParamList * params = NULL;
  struct timeval TCX_waiting_time = {0, 0};

  /* add some parameter files */
  params = bParametersAddFile(params, "etc/beeSoft.ini");
  
  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(params);

  buttonRegister();

  initClient( "buttonExample", commShutdown); /* close function called if */
                                              /* the server dies          */

  buttonConnect( 1 );  /* 1 -> wait until connection has been established */

  RaiInit();                            /* init (but not start) scheduler   */

  catchInterrupts();

  initClientModules();                  /* set up Rai modules to do         */
                                        /* communication for you            */

  /* whenever user hits CTRL-C */
  signal( SIGINT, &ctrlcShutdown );

  /* we wanna know when the buttons' status change ... */
  registerButtonStatusCallback( myButtonStatusCallback );
  
  buttonSubscribeStatus( 1 );

  /* and flash some buttons ... */
  buttonStartCuteThing();
  
  while ( 1 ) {
      
    TCX_waiting_time.tv_sec  = 0;
    TCX_waiting_time.tv_usec = 0;
    tcxRecvLoop((void *) &TCX_waiting_time);
    
  }

  RaiStart();

  return 0;
}

/*
 * $Log: example.c,v $
 * Revision 1.2  1998/01/14 16:45:38  swa
 * Changed clientCloseFcn to take two arguments, the name of the module
 * that died and the TCX_MODULE_PTR module. Now one can distinguish in the
 * close function between modules that die.
 *
 * Revision 1.1  1997/06/29 21:58:28  thrun
 * Added a ton of files for a client/server button thing.                (swa)
 *
 */
