
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

/*     fprintf( stderr,"Received startc (%s) (%s).\n", argv[2], argv[3] ); */
/*     cameraStartSaving( atoi(argv[3]), argv[2] ); */

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
int consoleInitCmd( ClientData clientData, 
		    Tcl_Interp *interp, 
		    int argc, 
		    char *argv[]) {

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
int Console_Init( Tcl_Interp *interp ){

  Tcl_CreateCommand ( interp,
		      "console",
		      consoleInitCmd,
		      (ClientData) NULL,
		      (Tcl_CmdDeleteProc *)NULL
		      );
  return TCL_OK;
  
}

/*
 * $Log: consoleItf.c,v $
 * Revision 1.1.1.1  1997/11/10 23:10:36  swa
 * Stefan's new console manager.
 *
 * Revision 1.1.1.1  1997/11/07 22:31:36  swa
 * imported
 *
 */
