
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
#include <Common.h>

#include "tk.h"
#include "tcl.h"
#include "tix.h"
#include "blt.h"

#include "console.h"

#define STARTUP_FILE "console.tcl"

#ifndef DEBUG
static Tcl_Interp *tcl_interp;
#endif

extern int Console_Init (Tcl_Interp *);

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void commShutdown() {

#if ( defined(G_DEBUG_WHERE_AM_I) )
  fprintf( stderr, "%s: %s\n", __FILE__, __FUNCTION__ );
#endif

  fprintf( stderr, "%s: Somebody died. Exiting.\n", __FILE__ );

  RaiShutdown();

  exit( -1 );

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
  int i, j;

  if (Tcl_Init(interp) == TCL_ERROR) {
    return TCL_ERROR;
  }

  if (Tk_Init(interp) == TCL_ERROR) {
    return TCL_ERROR;
  }

  if (Tix_Init(interp) == TCL_ERROR) {
    return TCL_ERROR;
  }

  if (Blt_Init(interp) == TCL_ERROR) {
    return TCL_ERROR;
  }
  
  if (Console_Init (interp) == TCL_ERROR) {
    return TCL_ERROR;
  }

  /* get the name of the executable, ie where Tcl thinks the binary is*/
  /* set i_am_this [ eval  info nameofexecutable ] */
  /*   My_Tcl_Eval( interp, "info nameofexecutable" ); */
  My_Tcl_Eval( interp, "file dirname [eval info nameofexecutable]" ); 
  sprintf( myTclAndStuffDir, "%s", interp->result );

  /* set the bitmap directory to bin/ */
  sprintf( reallyLongString, "tix addbitmapdir %s", myTclAndStuffDir );
  My_Tcl_Eval( interp, reallyLongString );

  /* now eval all the stuff we sucked out of the .ini files */
  for ( i=0; i<numMyPrograms; i++ ) {
    sprintf( reallyLongString, "%s \"%s\"", 
	     "lappend programID", ConsoleDep[i].programID );
    My_Tcl_Eval( interp, reallyLongString );
    sprintf( reallyLongString, "%s \"%s\"", 
	     "lappend programName", ConsoleDep[i].programName );
    My_Tcl_Eval( interp, reallyLongString );
    sprintf( reallyLongString, "%s \"%s\"", 
	     "lappend howStart1", ConsoleDep[i].howStart1 );
    My_Tcl_Eval( interp, reallyLongString );
    sprintf( reallyLongString, "%s \"%s\"", 
	     "lappend howStart2", ConsoleDep[i].howStart2 );
    My_Tcl_Eval( interp, reallyLongString );
    sprintf( reallyLongString, "%s \"%s\"", 
	     "lappend howStart3", ConsoleDep[i].howStart3 );
    My_Tcl_Eval( interp, reallyLongString );
    sprintf( reallyLongString, "%s \"%s\"", 
	     "lappend howKill1", ConsoleDep[i].howKill1 );
    My_Tcl_Eval( interp, reallyLongString );
    sprintf( reallyLongString, "%s \"%s\"", 
	     "lappend howKill2", ConsoleDep[i].howKill2 );
    My_Tcl_Eval( interp, reallyLongString );
    sprintf( reallyLongString, "%s \"%s\"", 
	     "lappend howKill3", ConsoleDep[i].howKill3 );
    My_Tcl_Eval( interp, reallyLongString );
    sprintf( reallyLongString, "%s \"%s\"", 
	     "lappend programOnHost", ConsoleDep[i].programOnHost );
    My_Tcl_Eval( interp, reallyLongString );
    if ( ConsoleDep[i].numDependencies ) 
      for (j=0; j<ConsoleDep[i].numDependencies; j++) {
	sprintf( reallyLongString, "%s%d%s \"%s\"", 
		 "lappend dep(", i, ")", ConsoleDep[i].dep[j] );
	My_Tcl_Eval( interp, reallyLongString );
      }
    else {			/* this means no dep => empty list */
      sprintf( reallyLongString, "set dep(%d) {}", i );
      My_Tcl_Eval( interp, reallyLongString );
    }
  }

  /* pass on pathToMyPrograms to tcl */
  sprintf( reallyLongString, "set pathToMyPrograms %s", pathToMyPrograms );
  My_Tcl_Eval( interp, reallyLongString );

  /* pass on scrollauto to tcl */
  if ( scrollauto )
    sprintf( reallyLongString, "set scrollauto 1" );
  else
    sprintf( reallyLongString, "set scrollauto 0" );
  My_Tcl_Eval( interp, reallyLongString );

  /* and eval the .tcl file */
  sprintf( reallyLongString, "source %s/%s", myTclAndStuffDir, STARTUP_FILE );
  My_Tcl_Eval( interp, reallyLongString );
  
  return TCL_OK;
}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int printDependencies() {

  int i, j;

  for (i=0; i<numMyPrograms; i++ ) {
    fprintf( stderr, "%2d.programName .... %s\n", i, ConsoleDep[i].programName );
    fprintf( stderr, "%2d.howStart1....... %s\n", i, ConsoleDep[i].howStart1 );
    fprintf( stderr, "%2d.howStart2....... %s\n", i, ConsoleDep[i].howStart2 );
    fprintf( stderr, "%2d.howStart3....... %s\n", i, ConsoleDep[i].howStart3 );
    fprintf( stderr, "%2d.howKill1........ %s\n", i, ConsoleDep[i].howKill1 );
    fprintf( stderr, "%2d.howKill2........ %s\n", i, ConsoleDep[i].howKill2 );
    fprintf( stderr, "%2d.howKill3........ %s\n", i, ConsoleDep[i].howKill3 );
    fprintf( stderr, "%2d.programOnHost... %s\n", i, ConsoleDep[i].programOnHost );
    fprintf( stderr, "%2d.numDependencies. %d\n", i, ConsoleDep[i].numDependencies );
    if ( ConsoleDep[i].numDependencies ) {
      fprintf( stderr, "%2d.dep ............ \n", i );
      for (j=0; j<ConsoleDep[i].numDependencies; j++ )
	fprintf( stderr, "%2d                  %s\n", i, ConsoleDep[i].dep[j] );
    }
    fprintf( stderr, "\n" );
  }
  
  return 0;

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int printList( char **progs, int numCurrentElements ) {

  int i;

  fprintf( stderr,"List:\n" );

  for (i=0; i<numCurrentElements; i++)
    fprintf( stderr, "%d:%s\n", i, progs[i] );

  return 0;

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int unionList( char **progs, int *numCurrentElements, char *element ) {

  int i;
  int alreadyInList = 0;

  for (i=0; i<*numCurrentElements; i++)
    if ( !strcmp(progs[i],element ) ) 
      alreadyInList = 1;
  
  if ( alreadyInList == 0 ) {
    if ( *numCurrentElements == MAXCONSOLEPROGRAMS ) {
      fprintf( stderr,"Too many dependencies.\n");
      MISCERROR;
      return -1;
    }
    sprintf( progs[*numCurrentElements], "%s", element );
    (*numCurrentElements)++;
  }

  return 0;
}

/* ---------------------------------------------------------
 *
 * is recursively called and adds programs with its dependencies to the list
 *
 * --------------------------------------------------------*/
int addProgram( struct bParamList *params,
		char             **progs,
		int               *numCurrentProgs,
		char              *newelement ) {

  const char *string;
  int index[10], ii;
  char *element;

  /* union element with the list */
  if ( unionList( progs, numCurrentProgs, newelement) != 0 ) {
    MISCERROR;
    return -1;
  }

  /* find the `dep' line and add all its dependencies */
  string = bParametersGetParam( params, newelement, "dep" );
  ii=0;
  while ( index[0] = ii++, (element = bGetArrayElementM(string, 1, index)) ) {
    if ( strcmp(element,"") )
      if ( addProgram( params, myProgram, &numMyPrograms, element ) !=0 ) {
	MISCERROR;
	return -1;
      }
    if (element)
      free(element);
  }

  return 0;

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int parseParameters( struct bParamList *params ) {

  const char *string;
  int index[10], ii, h, j, dummy;
  char *element;
  char *dumm1;
  char  dumm2[256];		/* ugly hack */
  char *dumm3;

  tcxMachine = (char*)bParametersGetParam(params, "", "TCXHOST");

  pathToMyPrograms = (char*)bParametersGetParam(params, "", "pathToMyPrograms");

  if ( !pathToMyPrograms ) { 
    fprintf( stderr, "pathToMyPrograms not set. Check console.ini.\n");
    commShutdown();
  }

  if ( !tcxMachine ) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  myProgram = (char**) malloc( MAXCONSOLEPROGRAMS * sizeof(char*) );
  if ( !myProgram ) {
    MISCERROR;
    commShutdown();
  }
  for( j=0; j<MAXCONSOLEPROGRAMS; j++) {
    myProgram[j] = (char*) malloc( MAXCONSOLESTRINGLENGTH * sizeof(char) );
    if ( !myProgram[j] ) {
      MISCERROR;
      commShutdown();
    }
  }

  /* parse `myPrograms', that means add all entries in     */
  /* myPrograms and then recursively all it's dependencies */
  /* Fill array myProg[] and numMyPrograms accordingly.    */

  /* ok, here's something new: first add all user programs */
  /* then add all of the dependent programs to the list    */
  numMyPrograms = 0;

  string = bParametersGetParam( params, "", "myPrograms" );
  ii=0; h=0;
  while ( index[0] = ii++, (element = bGetArrayElementM(string, 1, index)) ) {
    if ( strcmp(element,"") )
      if ( unionList( myProgram, &numMyPrograms, element) != 0 ) {
	MISCERROR;
	return -1;
      }
    if (element)
      free(element);
  }

  /* at this point we have all the of the user programs         */
  /* and now we add the dependencies. because we union the list */
  /* the user's programs won't get added twice.                 */
  dummy = numMyPrograms;
  for ( h=0; h<dummy; h++ )
    if ( addProgram( params, myProgram, &numMyPrograms, myProgram[h] ) !=0 ) {
      MISCERROR;
      commShutdown();
    }

  for ( j=0; j<numMyPrograms; j++ ) {
    sprintf( ConsoleDep[j].programID, 
	     "%s", myProgram[j] );
    sprintf( ConsoleDep[j].programName, 
	     "%s", bParametersGetParam(params,myProgram[j],"name") );
    sprintf( ConsoleDep[j].howStart1, 
	     "%s", bParametersGetParam(params,myProgram[j],"howStart1") );
    sprintf( ConsoleDep[j].howStart2, 
	     "%s", bParametersGetParam(params,myProgram[j],"howStart2") );
    sprintf( ConsoleDep[j].howStart3, 
	     "%s", bParametersGetParam(params,myProgram[j],"howStart3") );
    sprintf( ConsoleDep[j].howKill1, 
	     "%s", bParametersGetParam(params,myProgram[j],"howKill1") );
    sprintf( ConsoleDep[j].howKill2, 
	     "%s", bParametersGetParam(params,myProgram[j],"howKill2") );
    sprintf( ConsoleDep[j].howKill3, 
	     "%s", bParametersGetParam(params,myProgram[j],"howKill3") );
    sprintf( ConsoleDep[j].programOnHost, 
	     "%s", bParametersGetParam(params,myProgram[j],"programOnHost") );
    string = bParametersGetParam( params, myProgram[j], "dep" );
    ii=0; h=0;
    while ( index[0] = ii++, (element = bGetArrayElementM(string, 1, index)) ) {
      if ( strcmp(element,"") )
	sprintf( ConsoleDep[j].dep[h++], "%s", element );
      if (element)
	free(element);
    }
    ConsoleDep[j].numDependencies = h;
  }

  /* now check all the programOnHost variables. if you find an entry with `***' */
  /* pick the hardware definition from beeSoft.ini */
  for ( j=0; j<numMyPrograms; j++ ) {

    if ( !strcmp(ConsoleDep[j].programOnHost,"***") ) {

      /* ---------------------------------------------------------------- */

      if ( !strcmp( ConsoleDep[j].programID, "tcxServer" ) ) {
	/* thats easy :) */
	sprintf( ConsoleDep[j].programOnHost, "%s", tcxMachine ); 

      /* ---------------------------------------------------------------- */

      } else if ( !strcmp( ConsoleDep[j].programID, "armServer" ) ) {
	/* get definition of [robot]arm */
	/* get definition of mast.host in [[robot].arm].arm] */
	dumm1 = (char*)bParametersGetParam(params,"robot","arm");
	if ( !dumm1 ) {
	  fprintf( stderr,"Your section robot.arm is missing from beeSoft.ini.\n");
	  MISCERROR;
	  commShutdown();
	}
	sprintf( dumm2, "%s.arm", dumm1 );
	dumm3 = (char*)bParametersGetParam(params,dumm2,"mast.host");
	if ( !dumm3 ) {
	  fprintf( stderr,"Your entry mast.host is missing from the arm section in beeSoft.ini.\n");
	  MISCERROR;
	  commShutdown();
	}
	sprintf( ConsoleDep[j].programOnHost, "%s", dumm3 );


      /* ---------------------------------------------------------------- */


      } else if ( !strcmp( ConsoleDep[j].programID, "baseServer" ) ) {
	/* get definition of [robot]name */
	/* get definition of host in [[robot]name.base] */
	dumm1 = (char*)bParametersGetParam(params,"robot","name");
	if ( !dumm1 ) {
	  fprintf( stderr,"Your section robot.name is missing from beeSoft.ini.\n");
	  MISCERROR;
	  commShutdown();
	}
	sprintf( dumm2, "%s.base", dumm1 );
	dumm3 = (char*)bParametersGetParam(params,dumm2,"host");
	if ( !dumm3 ) {
	  fprintf( stderr,"Your host entry is missing from the base section in beeSoft.ini.\n");
	  MISCERROR;
	  commShutdown();
	}
	sprintf( ConsoleDep[j].programOnHost, "%s", dumm3 );


      /* ---------------------------------------------------------------- */


      } else if ( !strcmp( ConsoleDep[j].programID, "buttonServer" ) ) {
	/* bail out (for now) */
	fprintf( stderr,"There is no hardware info on the buttons in the file beeSoft.ini.\n");
	MISCERROR;
	commShutdown();


      /* ---------------------------------------------------------------- */


      } else if ( !strcmp( ConsoleDep[j].programID, "cameraServer" ) ) {
	/* get definition of [robot]framegrabber */
	/* get definition of host in [[robot]framegrabber.framegrabber] */
	dumm1 = (char*)bParametersGetParam(params,"robot","framegrabber");
	if ( !dumm1 ) {
	  fprintf( stderr,"Your section robot.framegrabber is missing from beeSoft.ini.\n");
	  MISCERROR;
	  commShutdown();
	}
	sprintf( dumm2, "%s.framegrabber", dumm1 );
	dumm3 = (char*)bParametersGetParam(params,dumm2,"host");
	if ( !dumm3 ) {
	  fprintf( stderr,"Your framegrabber entry is missing from the framegrabber section in beeSoft.ini.\n");
	  MISCERROR;
	  commShutdown();
	}
	sprintf( ConsoleDep[j].programOnHost, "%s", dumm3 );


      /* ---------------------------------------------------------------- */


      } else if ( !strcmp( ConsoleDep[j].programID, "colliServer" ) ) {
	/* get definition of [robot]name */
	/* get definition of host in [[robot]name.base] */
	dumm1 = (char*)bParametersGetParam(params,"robot","name");
	if ( !dumm1 ) {
	  fprintf( stderr,"Your section robot.name is missing from beeSoft.ini.\n");
	  MISCERROR;
	  commShutdown();
	}
	sprintf( dumm2, "%s.base", dumm1 );
	dumm3 = (char*)bParametersGetParam(params,dumm2,"host");
	if ( !dumm3 ) {
	  fprintf( stderr,"Your host entry is missing from the base section in beeSoft.ini.\n");
	  MISCERROR;
	  commShutdown();
	}
	sprintf( ConsoleDep[j].programOnHost, "%s", dumm3 );


      /* ---------------------------------------------------------------- */


      } else if ( !strcmp( ConsoleDep[j].programID, "laserServer" ) ) {
	/* get definition of [robot]laser */
	/* get definition of host in [[robot]laser.laser] */
	dumm1 = (char*)bParametersGetParam(params,"robot","laser");
	if ( !dumm1 ) {
	  fprintf( stderr,"Your section robot.laser is missing from beeSoft.ini.\n");
	  MISCERROR;
	  commShutdown();
	}
	sprintf( dumm2, "%s.laser", dumm1 );
	dumm3 = (char*)bParametersGetParam(params,dumm2,"host");
	if ( !dumm3 ) {
	  fprintf( stderr,"Your host entry is missing from the laser section in beeSoft.ini.\n");
	  MISCERROR;
	  commShutdown();
	}
	sprintf( ConsoleDep[j].programOnHost, "%s", dumm3 );


      /* ---------------------------------------------------------------- */


      } else if ( !strcmp( ConsoleDep[j].programID, "pantiltServer" ) ) {
	/* get definition of [robot]pantilt */
	/* get definition of host in [[robot]pantilt.pantilt] */
	dumm1 = (char*)bParametersGetParam(params,"robot","pantilt");
	if ( !dumm1 ) {
	  fprintf( stderr,"Your section robot.pantilt is missing from beeSoft.ini.\n");
	  MISCERROR;
	  commShutdown();
	}
	sprintf( dumm2, "%s.pantilt", dumm1 );
	dumm3 = (char*)bParametersGetParam(params,dumm2,"host");
	if ( !dumm3 ) {
	  fprintf( stderr,"Your host entry is missing from the pantilt section in beeSoft.ini.\n");
	  MISCERROR;
	  commShutdown();
	}
	sprintf( ConsoleDep[j].programOnHost, "%s", dumm3 );


      /* ---------------------------------------------------------------- */


      } else if ( !strcmp( ConsoleDep[j].programID, "speechServer" ) ) {
	/* get definition of [robot]speech */
	/* get definition of host in [[robot]speech.speech] */
	dumm1 = (char*)bParametersGetParam(params,"robot","speech");
	if ( !dumm1 ) {
	  fprintf( stderr,"Your section robot.speech is missing from beeSoft.ini.\n");
	  MISCERROR;
	  commShutdown();
	}
	sprintf( dumm2, "%s.speech", dumm1 );
	dumm3 = (char*)bParametersGetParam(params,dumm2,"host");
	if ( !dumm3 ) {
	  fprintf( stderr,"Your host entry is missing from the speech section in beeSoft.ini.\n");
	  MISCERROR;
	  commShutdown();
	}
	sprintf( ConsoleDep[j].programOnHost, "%s", dumm3 );


      /* ---------------------------------------------------------------- */


      } else {
	/* bail out, one of the hosts is still `***' which is a no no. */
	fprintf( stderr,"There are host entries with `***' which don't have a hardware configuration in beeSoft.ini.\n");
	MISCERROR;
	commShutdown();
      }
    }
  }
  
  return 0;
  
}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int main( int argc, char** argv ) {

  struct bParamList * params = NULL;
  int j;
  
  /* add some parameter files */
  params = bParametersAddFile( params, "etc/beeSoft.ini" );

  /* add some parameter files */
  params = bParametersAddFile( params, "etc/console.ini" );
  
  /* add some enviroment variables */
  params = bParametersAddEnv( params, "", "TCXHOST" );

  /* add command line arguements */
  params = bParametersAddArray( params, "", argc, argv );

  /* Fill the global parameter structure */
  bParametersFillParams( params );

  scrollauto = 1;		/* default */

  scrollauto = atoi(bParametersGetParam(params, "", "scrollauto"));

  parseParameters( params );

/*   printDependencies(); */

#ifdef DEBUG

  Tk_Main( argc, argv, Tcl_AppInit );

#else  

  Tcl_FindExecutable( argv[0] );

  /* For normal usage, use the Tk main event loop (no prompt) */
  tcl_interp = Tcl_CreateInterp();

  Tcl_AppInit(tcl_interp);

  Tk_MainLoop();

#endif

  /* clean up after the fact */
  for( j=0; j<MAXCONSOLEPROGRAMS; j++)
    if ( myProgram[j] ) 
      free( myProgram[j] );

  if ( myProgram ) 
    free( myProgram );

  return 0;

}

/*
 * $Log: console.c,v $
 * Revision 1.7  1998/02/09 22:59:29  swa
 * Added a parameter that controls if we automatically scroll to the end of
 * each log window.
 *
 * Revision 1.6  1997/11/29 16:58:59  swa
 * Friendlier error messages.
 *
 * Revision 1.5  1997/11/29 00:13:10  swa
 * Moved all stuff from console.ini into beeSoft.ini. Moved console2.ini to console.ini.
 * Consider other definitions in beeSoft.ini if programOnHost is `***', so that exisiting
 * definitions are re-used.
 *
 * Revision 1.4  1997/11/28 18:26:00  swa
 * Added a variable to replace absolute paths. More user friendly. Use only
 * ssh for simplicity and security. Update TODO with tyson's thoughts.
 *
 * Revision 1.3  1997/11/11 20:27:08  swa
 * Fixed a debug flag.
 *
 * Revision 1.2  1997/11/11 20:20:36  swa
 * Added a -DDEBUG flag so that we suppress the Tcl prompt if we don't want
 * to debug. User cannot enter Tcl stuff at the prompt then. Removed some
 * stuff in Tcl_AppInit. For the debug stuff to work we use a different setup,
 * and we needed (!) to call Tcl_FindExecutable first. Tcl--.
 *
 * Revision 1.1.1.1  1997/11/10 23:10:36  swa
 * Stefan's new console manager.
 *
 * Revision 1.9  1997/11/09 18:34:45  swa
 * Minor stuff.
 *
 * Revision 1.8  1997/11/09 18:25:10  swa
 * All programs and their dependencies are now taken care of. A second,
 * user-specific ini file was added, console2.ini. All dependencies in
 * console.ini should eventually go into beeSoft.ini and console2.ini
 * should be renamed console.ini.
 *
 * Revision 1.7  1997/11/08 18:01:07  swa
 * Programs which depend on other programs are now taken care of.
 *
 * Revision 1.6  1997/11/08 17:27:12  swa
 * Completed console.ini. Now parse all servers.
 *
 * Revision 1.5  1997/11/08 04:14:58  swa
 * Use a single button for both, start and stop. Next fix must be,
 * that the button goes back to start iff a program exits and the
 * user didn't click the button stop.
 *
 * Revision 1.4  1997/11/08 03:51:05  swa
 * Most variables are no longer set in the .tcl script, instead they are
 * read from the .ini file and evaluated by our Tcl interpreter.
 *
 * Revision 1.3  1997/11/08 01:09:24  swa
 * Start and Stop icons added.
 *
 * Revision 1.2  1997/11/07 22:41:36  swa
 * Compiles and runs nicely.
 *
 * Revision 1.1.1.1  1997/11/07 22:31:36  swa
 * imported
 *
 */
