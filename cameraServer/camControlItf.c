
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
#include <tcl.h>
#include <cameraClient.h>

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int ProcedureToCallCmd (ClientData clientData, Tcl_Interp *interp, 
			int argc, char *argv[]) {

  clientData = clientData;	/* -Wall */

  if ( argc < 2 )  {
    interp->result = "wrong # args";
    return TCL_ERROR;
  }

  /* ------------------------------------------------------------------
   *
   * argv[1] = startc, argv[2] = filename, argv[3] = numGrabber
   *
   * ------------------------------------------------------------------- */
  else if (strcmp (argv[1], "startc") == 0) {

    fprintf( stderr,"Received startc (%s) (%s).\n", argv[2], argv[3] );

    cameraStartSaving( atoi(argv[3]), argv[2] );

    if ( argc != 4 )
      return TCL_ERROR;
    else
      return TCL_OK;
  }

  /* ------------------------------------------------------------------
   *
   * argv[1] = startc, argv[2] = filename, argv[3] = numGrabber
   *
   * ------------------------------------------------------------------- */
  else if (strcmp (argv[1], "stopc") == 0) {

/*     fprintf( stderr,"Received stopc %s.\n", argv[2] ); */

    cameraStopSaving( atoi(argv[3]) );

    if ( argc != 4 )
      return TCL_ERROR;
    else
      return TCL_OK;
  }

  /* ------------------------------------------------------------------
   *
   * argv[1] = startc, argv[2] = filename, argv[3] = num_frames, argv[4] = numGrabber
   *
   * ------------------------------------------------------------------- */
  else if (strcmp (argv[1], "save") == 0) {

/*     fprintf( stderr,"Received save %d frames to %s.\n", atoi(argv[3]), argv[2] ); */

    cameraSaveFile( atoi(argv[4]), argv[2], atoi(argv[3]) );

    if ( argc != 5 )
      return TCL_ERROR;
    else
      return TCL_OK;
  }

  /* ------------------------------------------------------------------
   *
   * argv[1] = startc, argv[2] = filename, argv[3] = numGrabber
   *
   * ------------------------------------------------------------------- */
  else if (strcmp (argv[1], "re_ad") == 0) {

/*     fprintf( stderr,"Received read %s.\n", argv[2] ); */

    cameraLoadFile( atoi(argv[3]), argv[2] );

    if ( argc != 4 )
      return TCL_ERROR;
    else
      return TCL_OK;
  }

  /* ------------------------------------------------------------------
   *
   *
   * ------------------------------------------------------------------- */
  else {
    sprintf(interp->result, "ERROR!! Unknown Command !!! \n");
    return TCL_ERROR;
  }

  return TCL_OK;
}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int stefanInitCmd (ClientData clientData, Tcl_Interp *interp, 
		   int argc,  char *argv[]) {

  if (argc != 2) {
    interp->result = "wrong # args";
    return TCL_ERROR;
  }

  clientData = clientData;	/* -Wall */
  
  Tcl_CreateCommand (interp, argv[1], ProcedureToCallCmd, (ClientData) NULL, NULL);

  return TCL_OK;

}


/* ---------------------------------------------------------
 *
 * called from the main program camControl.c
 *
 * --------------------------------------------------------*/
int Stefan_Init (Tcl_Interp *interp) {

  Tcl_CreateCommand ( interp, 
		      "camControl", 
		      stefanInitCmd, 
		      (ClientData) NULL,
		      (Tcl_CmdDeleteProc *)NULL
		      );
  return TCL_OK;
  
}

/*
 * $Log: camControlItf.c,v $
 * Revision 1.4  1997/10/04 00:13:05  swa
 * First working version of the cameraServer that supports two (count'em
 * two!) framegrabbers.  Although the server seems to work just fine, it
 * has not yet been fully tested.
 *
 * Revision 1.3  1997/07/24 18:48:00  swa
 * Renamed two internal functions. Tested it again with the Tcl program. Works
 * fine. :)
 *
 * Revision 1.2  1997/07/24 00:54:24  swa
 * Fixed some minor bugs, extended the README file and tested the version. The
 * cameraAttachExample does not yet die when the cameraServer dies. Will be
 * fixed in future versions.
 *
 * Revision 1.1  1997/07/23 22:43:32  swa
 * This is the first version of camControl, a Tcl program that remotely controls
 * the cameraServer. The user can save and read frames. recorder was renamed
 * to camControl, since the program will not only record but also read frames.
 *
 */
