
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
#include <rai.h>
#include <string.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <math.h>
#include <sys/time.h>

#include "tcx.h"
#include "tcxP.h"
#include "global.h"

#include <bUtils.h>

#include <Common.h>
#include "devUtils.h"
#include "EZX11.h"
#include "o-graphics.h"
#include "io.h"
#include "mainlaser.h"
#include "LASER_SERVER-messages.h"
#include "laserHandlers.h"

int ROBOT_BACKGROUND;
int ROBOT;
int INIT_BUTTON;
extern int listen_for_tcx_events;
int display;

const char *laserConfig;
const char *laserType;
const char *laserHost;
const char *laserDev;
const char *laserBps;

int block_wait_with_serial_ports(struct timeval *timeout, int tcx_initialized,
				 int X_initialized);

/* ---------------------------------------------------------
 *
 * is called when connecting modules die
 *
 * --------------------------------------------------------*/
void commShutdown( ) {

  fprintf( stderr, "%s: TCX Server died. Exiting.\n", __FILE__ );
  exit(-1);

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void init_graphics(int display)
{
  static char *myfonts[] = {"5x8", "6x10", "7x13bold", 
			    "9x15", "10x20", "12x24", 
			    "lucidasans-bold-18"};

  G_set_display(display);

  G_initialize_fonts(7, myfonts);
  G_initialize_graphics("LASER", 70.0, 10.0, C_STEELBLUE4);
  G_set_matrix_display_style(1);
  
  
  /******** ROBOT_BACKGROUND **************************************/
  {
    int switch_num                      = 1;
    static float switch_pos[]           = {0.9, 3.0, 6.0, 9.9};
    static char *switch_texts[]         = {""};
    static int switch_fonts[]           = {2};
    static int switch_background_color[]= {C_GREY70};
    static int switch_frame_color[]     = {C_GREY40};
    static int switch_text_color[]      = {NO_COLOR};
    
    ROBOT_BACKGROUND = G_create_switch_object(switch_pos, switch_num, 
						    switch_texts,
						    switch_background_color,
						    switch_frame_color, 
						    switch_text_color,
						    switch_fonts);
    
  }
  
  /******** ROBOT *************************************/
  {
    static float pos_r[]                 = {0.0, 3.9, 6.0, 9.9};
    static char *text_r                  = "ROBOT";
    static int robot_font                = 4;
    static int colors_r[]                = {NO_COLOR, NO_COLOR, C_GREY90,
					      C_BLACK, C_TURQUOISE4, C_GREY40,
					      C_GREY90, NO_COLOR};
    
    ROBOT =  G_create_robot_object(pos_r, text_r, 
				   -(MAX_LASER_RANGE) * 1.05,
				   (MAX_LASER_RANGE) *1.05,
				   -(MAX_LASER_RANGE) *1.05,
				   (MAX_LASER_RANGE) *1.05,
				   0.0, 0.0, 90.0, 
				   ROBOT_RADIUS,
				   NUMBER_LASERS,
				   MAX_LASER_RANGE,
				   LaserSensors.values,
				   LaserSensors.angles,
				   colors_r, robot_font);
  }    
  /******** INIT_BUTTON **************************************/
  {
    
    int switch_num                      = 2;
    static float switch_pos[]           = {0.0, 3.9, 5.5, 5.9};
    static char *switch_texts[]         = 
      {"initialize laser", "initialize laser"};
    static int switch_fonts[]           = {2,2};
    static int switch_background_color[]= {C_GREY70, C_RED};
    static int switch_frame_color[]     = {C_GREY40, C_GREY40};
    static int switch_text_color[]      = {C_BLACK, C_BLACK};
      
    INIT_BUTTON = G_create_switch_object(switch_pos, switch_num, 
						switch_texts,
						switch_background_color,
						switch_frame_color, 
						switch_text_color,
						switch_fonts);
  } 
  
  G_display_all();

}

/************************************************************************
 *   FUNCTION:     tests mouse and executes whatever we'd like
 *   RETURN-VALUE: 1, if meaningful mouse event found, 0 otherwise
 ************************************************************************/
int test_mouse(int display) {

  G_mouse_ptr mouse_events;
  int num_mouse_events, button;
  float mouse_x, mouse_y;
  int  number;
  int return_value = 0;
  int mouse_event_detected;


  if (display){
    /****************** CHECK FOR MOUSE EVENT *******************/

    mouse_event_detected = G_test_mouse(0);
    if (mouse_event_detected){


      mouse_events = G_evaluate_mouse(&mouse_x, &mouse_y, 
				      &button, &num_mouse_events);
      
      if (mouse_event_detected == 1){ /* reglar mouse press */

	return_value = 1;
	
	
	/****************** EVALUATE MOUSE EVENT *******************/

	/* ******* INIT_BUTTON ******** */
	
	if (G_mouse_event_at(INIT_BUTTON, mouse_events, &number)){ 
	  LaserInit();
	}	  

	/********* JUST UPDATE DISPLAY *********/
	
	else if (button == RIGHT_BUTTON){
	  G_display_all();
	}
	
      }
    }
  }
  return return_value;

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int displayParameters() {

  fprintf(stderr, "********  OPTIONS  **********\n");
  fprintf(stderr, "-display ............... = %d\n", display );
  fprintf(stderr, "laserConfig ............ = [%s.laser]\n", laserConfig );
  fprintf(stderr, "laserType .............. = %s\n", laserType );
  fprintf(stderr, "laserHost .............. = %s\n", laserHost );
  fprintf(stderr, "laserDev ............... = %s\n", laserDev );
  fprintf(stderr, "laserBps ............... = %s\n", laserBps );
  fprintf(stderr, "*****************************\n");

  return 0;

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int setupLaser() {

  int i;

  devInit();

  for (i = 0; i < NUMBER_LASERS; i++){
    LaserSensors.angles[i] = -90.0 + (((float)i) *180.0 /
				      ((float) (NUMBER_LASERS-1)));
    LaserSensors.values[i] = MAX_LASER_RANGE * 0.5;
  }

  init_graphics( display );

  /* ------------------------------------------------------------- */

/*     laserType = bParametersGetParam( params, dummy, "type" ); */
/*     laserHost = bParametersGetParam( params, dummy, "host" ); */
/*     laserDev = bParametersGetParam( params, dummy, "dev" ); */
/*     laserBps = bParametersGetParam( params, dummy, "bps" ); */

  /*   next line was apparently a bad idea */
  /*   sprintf( LaserDevice.dev.ttydev.ttyPort, "%s", laserDev ); */
  LaserDevice.dev.ttydev.ttyPort = (char*) laserDev;

  if ( !strcmp(laserBps,"9600") ) {
    
    LaserDevice.dev.ttydev.baudCode = 13;

  } else if ( !strcmp( laserBps,"19200") ) {

    LaserDevice.dev.ttydev.baudCode = 14;
    fprintf( stderr, "%s: %s\n", __FILE__, __FUNCTION__ );
    fprintf( stderr, "NOT YET IMPLEMENTED!!!\n" );
    commShutdown();

  } else {

    fprintf( stderr, "%s: %s\n", __FILE__, __FUNCTION__ );
    fprintf( stderr, "Invalid baud rate in beeSoft.ini! \n" );
    commShutdown();

  }

  /* ------------------------------------------------------------- */

  connectTotty( &LaserDevice.dev );

  if ( LaserDevice.dev.fd == -1 ) {
    fprintf( stderr,"%s: Error in line %d.\n", __FILE__, __LINE__ );
    commShutdown();
  }

  connectDev( &LaserDevice.dev );
  
  /* Sebastian, in an attempt to get the laser running, 97-8-6 */
  m_setparms( LaserDevice.dev.fd, "9600", "EVEN", "8", 0, 0 );

  LaserDevice.dev.sigHnd = LaserShutdownLaser;

  LaserInit();

  return 0;

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/



/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int main( int argc, char *argv[] ) {

  struct bParamList * params = NULL;
  struct timeval TCX_waiting_time = {0, 0};
  char dummy[256];

  params = bParametersAddEntry( params, "", "display", "0");

  /* add some parameter files */
  params = bParametersAddFile(params, "etc/beeSoft.ini");

  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(params);

  display = atoi(bParametersGetParam(params, "", "display"));

  tcxMachine = (char*)bParametersGetParam(params, "", "TCXHOST");

  /* find out the current laser config in the robot 
     section should be one of { none, SICK }        */
  laserConfig = bParametersGetParam( params, "robot", "laser" );

  if ( !strcmp( laserConfig, "none" ) ) {
    fprintf( stderr,"Your config in beeSoft.ini says, that you\n" );
    fprintf( stderr,"don't have a laser. What am I supposed to do?\n" );
    commShutdown();
  } else {
    /* get variables from the section `[<laserConfig>.laser]' */
    sprintf( dummy, "%s.%s", laserConfig, "laser" );
    laserType = bParametersGetParam( params, dummy, "type" );
    laserHost = bParametersGetParam( params, dummy, "host" );
    laserDev = bParametersGetParam( params, dummy, "dev" );
    laserBps = bParametersGetParam( params, dummy, "bps" );
    if ( !laserType || !laserHost || !laserDev || !laserBps ) {
      fprintf( stderr,"Couldn't parse beeSoft.ini properly.\n");
      commShutdown();
    }
  }

  displayParameters();

  LaserInitializeTCX();
  
  setupLaser();

  listen_for_tcx_events = 1;

  for (;;) {
    


    if ( LaserSensors.defined ) {
      if ( n_auto_sweep0_update_modules )
	send_automatic_sweep_update( 0, LaserSensors.values );
      G_display_robot( ROBOT, 0.0, 0.0, 90.0, NUMBER_LASERS,
		       &(LaserSensors.values[0]));
      LaserSensors.defined = 0;
    }

    TCX_waiting_time.tv_sec  = 1000;
    TCX_waiting_time.tv_usec = 0;
    block_wait_with_serial_ports(&TCX_waiting_time, 1, display);

    
    ProcessDevices();

    test_mouse( display );

    
  }

}

/*
 * $Log: mainlaser.c,v $
 * Revision 1.11  1998/01/14 00:27:29  thrun
 * laserServer quits now when TCX Server dies.
 *
 * Revision 1.10  1998/01/13 23:55:59  thrun
 * .
 *
 * Revision 1.9  1998/01/13 23:53:22  thrun
 * changed timing/blocking, with the following effects:
 *   - less data gets lost
 *   - processing time down by a factor of 1000 or so
 *   - performance will not degrade as machine gets busier
 *
 * Revision 1.8  1998/01/13 22:41:41  thrun
 * resolved a naming conflict. CHanged LASER to LASER_SERVER.
 *
 * Revision 1.7  1997/11/06 17:56:21  swa
 * Fixed a bug with TCXHOST. If the environment variable TCXHOST is set,
 * take it. If not, take the TCXHOST from beeSoft.ini. If this doesn't
 * work, take localhost. If this doesn't work: bummer.
 *
 * Revision 1.6  1997/09/19 23:31:32  swa
 * Fixed a typo.
 *
 * Revision 1.5  1997/09/19 00:59:22  swa
 * The laserServer now honours the entries in beeSoft.ini, that is device and
 * baudrate. Internally however, it only works with 9600 baud. It does some
 * moderate error checking on the entries.
 *
 * Revision 1.4  1997/08/07 15:42:53  swa
 * First working version of the laserServer. Seems to run fine. Has support for
 * a second laser clientwise, yet the device stuff for the second one is not
 * implemented.
 *
 * Revision 1.3  1997/08/07 03:50:21  swa
 * Fixed a bunch of bugs. Still not working.
 *
 * Revision 1.2  1997/08/07 02:45:52  swa
 * First version of a laserServer. Compiles nicely, yet produces nothing useful. ;)
 * Just to copy things to a safe place ...
 *
 * Revision 1.1  1997/08/06 15:12:33  swa
 * Very first and incomplete version of a laserServer. No TCX comm yet.
 *
 *
 */
