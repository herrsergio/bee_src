
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
#include <cameraClient.h>
#include <Common.h>
#include <EZX11.h>
#include <display.h>

#include "tk.h"
#include "tcl.h"
#include "tix.h"

#define MISCERROR fprintf( stderr, "\n>>>>>> %s: %s(%d) error\n", __FILE__, __FUNCTION__, __LINE__ )

#define STARTUP_FILE "camControl.tcl"

#ifndef DEBUG_CAMERA
static Tcl_Interp *tcl_interp;
#endif

extern int Stefan_Init (Tcl_Interp *);

int useGrabber;			/* which grabber do we use? */

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void commShutdown(char *name, TCX_MODULE_PTR module)
{

#if ( defined(G_DEBUG_WHERE_AM_I) )
  fprintf( stderr, "%s: %s\n", __FILE__, __FUNCTION__ );
#endif

  fprintf( stderr, "%s(%s): %s died. Closing. \n", 
	   __FILE__, __FUNCTION__, name );

  module = module;

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
 * --------------------------------------------------------*/
int My_Tcl_Eval(Tcl_Interp *interp, char *s)
{
  int result;

  result = Tcl_Eval(interp, s);

  if (result == TCL_ERROR) {
    fprintf(stderr, "Tcl Error: %s\n", interp->result);
    fprintf(stderr, "         : Command was \"%s\"\n", s);
  }
  
  return result;
}

/* ---------------------------------------------------------
 *
 * 
 *
 * --------------------------------------------------------*/
int myCameraFileCallback ( cameraFileType *data ) {

#if (defined(sun))
  int i;
#endif

  data = data;			/* -Wall */

  fprintf( stderr,"Received CameraFileCallback, which means reading has finished.\n" );

  return 0;
}

/*
 *----------------------------------------------------------------------
 *
 * Tcl_AppInit --
 *
 *	This procedure performs application-specific initialization.
 *	Most applications, especially those that incorporate additional
 *	packages, will have their own version of this procedure.
 *
 * Results:
 *	Returns a standard Tcl completion code, and leaves an error
 *	message in interp->result if an error occurs.
 *
 * Side effects:
 *	Depends on the startup script.
 *
 *----------------------------------------------------------------------
 */

int Tcl_AppInit(Tcl_Interp *interp) {

#define REALLYLONG 200

  char reallyLongString[REALLYLONG]; /* tcl_eval might not like this, but who cares? */
  char myTclAndStuffDir[200];

  if (Tcl_Init(interp) == TCL_ERROR) {
    return TCL_ERROR;
  }

  if (Tk_Init(interp) == TCL_ERROR) {
    return TCL_ERROR;
  }

  if (Tix_Init(interp) == TCL_ERROR) {
    return TCL_ERROR;
  }

  if (Stefan_Init(interp) == TCL_ERROR) {
    return TCL_ERROR;
  }

  /* get the name of the executable, ie where Tcl thinks the binary is*/
  /* set i_am_this [ eval  info nameofexecutable ] */
  /*   My_Tcl_Eval( interp, "info nameofexecutable" ); */
  My_Tcl_Eval( interp, "file dirname [eval info nameofexecutable]" ); 
  sprintf( myTclAndStuffDir, "%s", interp->result ); /*  /home/swa/bee/bin  */

  /* set the bitmap directory to bin/ */
  sprintf( reallyLongString, "tix addbitmapdir %s", myTclAndStuffDir );
  My_Tcl_Eval( interp, reallyLongString );

  /* add a selection for the framegrabber */
  if ( useGrabber == 3 ) {
    sprintf( reallyLongString, "set source_options {0 1}" );
    My_Tcl_Eval( interp, reallyLongString );
    sprintf( reallyLongString, "set source_labels(0) \"One\"" );
    My_Tcl_Eval( interp, reallyLongString );
    sprintf( reallyLongString, "set source_labels(1) \"Two\"" );
    My_Tcl_Eval( interp, reallyLongString );
  } else if ( useGrabber == 2 ) {
    sprintf( reallyLongString, "set source_options {1}" );
    My_Tcl_Eval( interp, reallyLongString );
    sprintf( reallyLongString, "set source_labels(1) \"Two\"" );
    My_Tcl_Eval( interp, reallyLongString );
  } else if ( useGrabber == 1 || useGrabber == 0 ) {
    sprintf( reallyLongString, "set source_options {0}" );
    My_Tcl_Eval( interp, reallyLongString );
    sprintf( reallyLongString, "set source_labels(0) \"One\"" );
    My_Tcl_Eval( interp, reallyLongString );
  } else {
    /* duh? there should be no `else' */
  }
  
  /* and source that baby */
  sprintf( reallyLongString, "source %s/%s", myTclAndStuffDir, STARTUP_FILE);
  My_Tcl_Eval( interp, reallyLongString );
  
#if 0
#if ((TCL_MAJOR_VERSION > 7) || ((TCL_MAJOR_VERSION == 7) && (TCL_MINOR_VERSION >= 5)))
  /* Starting from TCL 7.5, the symbol tcl_rcFileName is no longer
   * exported by libtcl.a. Instead, this variable must be set using
   * a TCL global variable
   */
  Tcl_SetVar(interp, "tcl_rcFileName", "~/.tixwishrc", TCL_GLOBAL_ONLY);
#else
  tcl_RcFileName = "~/.tixwishrc";
#endif
#endif
  
  return TCL_OK;
}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int parseParameters( struct bParamList *params ) {
  
  const char *grabberConfig;
  const char *grabberType;
  const char *grabberWhichGrabber;
  const char *grabberDev0;
  const char *grabberDev1;
  char dummy[256];

  /* find out the current grabber config in the robot 
     section should be one of { none, matrox }        */
  grabberConfig = bParametersGetParam( params, "robot", "framegrabber" );

  if ( !strcmp( grabberConfig, "none" ) ) {
    useGrabber = 0;
  } else {
    /* get variables from the section `[<grabberConfig>.framegrabber]' */
    sprintf( dummy, "%s.%s", grabberConfig, "framegrabber" );
    grabberType = bParametersGetParam( params, dummy, "type" );
    grabberDev0 = bParametersGetParam( params, dummy, "dev1" );
    grabberDev1 = bParametersGetParam( params, dummy, "dev2" );
    grabberWhichGrabber = bParametersGetParam( params, dummy, "usegrabber" );
    if ( !grabberType || !grabberDev0 || !grabberDev1 || !grabberWhichGrabber ) {
      fprintf( stderr,"Couldn't parse beeSoft.ini properly.\n");
      commShutdown("",NULL);
    }
    if ( strcmp( grabberType, "matrox-meteor" ) ) {
      fprintf( stderr,"I cannot handle framegrabbers other than the ");
      fprintf( stderr,"Matrox Meteor at this time.\n");
      commShutdown("",NULL);
    }
    useGrabber = atoi( grabberWhichGrabber );
    if ( useGrabber<0 || useGrabber>3 ) {
      MISCERROR;
      return -1;
    }
  }

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

  if( parseParameters( params ) != 0 ) {
    MISCERROR;
    commShutdown("",NULL);
  }

  cameraRegister();

  initClient( "camControl", commShutdown); /* close function called if */
                                         /* the server dies          */

  cameraConnect( 1 );  /* 1 -> wait until connection has been established */

  RaiInit();                            /* init (but not start) scheduler   */

  catchInterrupts();

  initClientModules();                  /* set up Rai modules to do         */
                                        /* communication for you            */
  /* whenever user hits CTRL-C */
  signal( SIGINT, &ctrlcShutdown );

  registerCameraFileCallback( myCameraFileCallback );

#ifdef DEBUG_CAMERA

  Tk_Main( argc, argv, Tcl_AppInit );

#else  

  Tcl_FindExecutable( argv[0] );

  /* For normal usage, use the Tk main event loop (no prompt) */
  tcl_interp = Tcl_CreateInterp();

  Tcl_AppInit(tcl_interp);

  Tk_MainLoop();

#endif

  while ( 1 ) {

    TCX_waiting_time.tv_sec  = 1;
    TCX_waiting_time.tv_usec = 0;
    tcxRecvLoop((void *) &TCX_waiting_time);
    
  }

  return 0;

}

/*
 * $Log: camControl.c,v $
 * Revision 1.7  1998/01/14 16:44:22  swa
 * Changed clientCloseFcn to take two arguments, the name of the module
 * that died and the TCX_MODULE_PTR module. Now one can distinguish in the
 * close function between modules that die.
 *
 * Revision 1.6  1998/01/13 00:35:12  swa
 * Added two new functions -- start and stop. They can start and stop either
 * of the two frame grabbers.
 *
 * Revision 1.5  1997/11/11 20:24:42  swa
 * Changed Tcl_Eval to My_Tcl_Eval, help debugging. Removed the Tcl command prompt
 * if we don't debug.
 *
 * Revision 1.4  1997/11/10 22:44:59  swa
 * Some changes to the Tcl/Tk interface have been made (user now selects framegrabber
 * and frames, he doesn't have to enter them). About dialog pepped up. Now, we
 * require Tix to be installed for the Tcl/Tk program.
 *
 * Revision 1.3  1997/10/04 00:13:05  swa
 * First working version of the cameraServer that supports two (count'em
 * two!) framegrabbers.  Although the server seems to work just fine, it
 * has not yet been fully tested.
 *
 * Revision 1.2  1997/08/05 17:41:09  swa
 * It is no longer req. to be in the same directory of camControl.tcl when you
 * start camControl.
 *
 * Revision 1.1  1997/07/23 22:43:31  swa
 * This is the first version of camControl, a Tcl program that remotely controls
 * the cameraServer. The user can save and read frames. recorder was renamed
 * to camControl, since the program will not only record but also read frames.
 *
 *
 */
