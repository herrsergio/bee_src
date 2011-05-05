
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

#if (!defined(sun))
#include <sys/ipc.h> /* shm stuff */
#include <sys/shm.h> /* shm stuff */
#endif

#include <signal.h>
#include <cameraClient.h>
#include <Common.h>
#include <EZX11.h>
#include <display.h>
#include <imagesize.h>
#include <misc.h>

/* **************************************************************************
 *
 * This little program demonstrates the use of shared memory. Start
 * the cameraServer and use this program to display the grabbed image(s) 
 * in a new window or in several windows (if you are fortunate enough to own
 * more than one frame grabber.
 *
 * **************************************************************************/

char *pCameraImage[2];
int shmid[2];			/* the id of the shared mem segment */
int shmid_attached[2];		/* did we receive the id, yet? */
int window[2];

const char *grabberConfig;
const char *grabberType;
const char *grabberWhichGrabber;
const char *grabberDev0;
const char *grabberDev1;

int useGrabber;

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
 *
 * --------------------------------------------------------*/
int myCameraShmIdCallback ( cameraShmIdType *gestureShmId ) {

  int numGrabber;

#if (defined(sun))
  int i;
#endif

  fprintf( stderr, 
	   "%s: Attaching to cameraServer's shmid %d (for grabber %d).\n", 
	   __FILE__, gestureShmId->shmid, gestureShmId->numGrabber );

  numGrabber = gestureShmId->numGrabber;

  if ( numGrabber!=0 && numGrabber!=1 ) {
    MISCERROR;
    return -1;
  }

#if (defined(sun))
  pCameraImage[numGrabber] = (char *) malloc( sizeof(char)*ROWS*COLS*4 );
  i = ROWS*COLS*4 - 1;
  do {
    pCameraImage[numGrabber][i] = 5;
  } while ( --i>=0 );
#else
  /* so that we can access the shared chunk of memory */
  pCameraImage[numGrabber] = (char *) shmat( gestureShmId->shmid, (char *) 0, 0);
  if ( pCameraImage[numGrabber] == (char*)(-1) ) {
    perror( "Attaching failed: shmat's error is " );
    return -1;
  }
#endif

  shmid_attached[numGrabber] = 1;
      
  return 0;
}


/* ---------------------------------------------------------
 *
 * dumps the ppm image. note, that the source data is BGRX
 * and NOT RGBX.
 *
 * --------------------------------------------------------*/
int writePPMimage( int num ) {

  FILE *fp;
  char name[50];
  char dump[COLS*ROWS*4];
  char realdump[COLS*ROWS*3];
  int i,j;

  /* dump the current image into a .ppm file */
  sprintf( name, "image-%02d.ppm", num );
  fprintf( stderr, "Writing image into %s for later processing\n", name);
  fp = fopen( name, "w" );
  if ( !fp ) {
    perror("Error opening filename");
    return -1;
  }
  memcpy( dump, pCameraImage[num], (size_t) COLS*ROWS*4 );

  i=0; j=0;
  do {
    realdump[j+0] = dump[i+2];
    realdump[j+1] = dump[i+1];
    realdump[j+2] = dump[i+0];
    i+=4; j+=3;
  } while (i<ROWS*COLS*4);

  fprintf(fp,"P6\n%d %d\n255\n",COLS,ROWS);
  if ( ROWS*COLS*3 != fwrite( realdump,sizeof(char),ROWS*COLS*3,fp ) ) {
    perror("writePPMimage");
    exit(-1);
  }
	
  fclose( fp );

  return 0;
}

/* ---------------------------------------------------------
 *
 * 
 *
 * --------------------------------------------------------*/
int DoTheJob() {

  struct timeval TCX_waiting_time = {0, 0};
  XEvent theEvent;

  if ( shmid_attached[0] || shmid_attached[1] ) {
    if (XCheckMaskEvent(theDisplay, ButtonPressMask, &theEvent)) { 
      XButtonEvent theButtonEvent = theEvent.xbutton;
      int xx = theButtonEvent.x; 
      int yy = theButtonEvent.y; 
      theButtonEvent.button--;
      /* boy, we really hope to have short circuit boolean evaluation */
      if ( shmid_attached[0] && theButtonEvent.window == w_id[window[0]]->w ) {
	if ( theButtonEvent.button == LEFT_BUTTON ) {
	  writePPMimage( 0 );
	} else if ( theButtonEvent.button == RIGHT_BUTTON ) {
	  fprintf( stderr,"Right clicked at (%d,%d)\n", xx, yy );
	} else if ( theButtonEvent.button == MIDDLE_BUTTON ) {
	  fprintf( stderr,"Middle clicked at (%d,%d)\n", xx, yy );
	}
      } else if ( shmid_attached[1] && theButtonEvent.window == w_id[window[1]]->w ) {
	if ( theButtonEvent.button == LEFT_BUTTON ) {
	  writePPMimage( 1 );
	} else if ( theButtonEvent.button == RIGHT_BUTTON ) {
	  fprintf( stderr,"Right clicked at (%d,%d)\n", xx, yy );
	} else if ( theButtonEvent.button == MIDDLE_BUTTON ) {
	  fprintf( stderr,"Middle clicked at (%d,%d)\n", xx, yy );
	}
      }
    }
  }

  TCX_waiting_time.tv_sec  = 0;
  TCX_waiting_time.tv_usec = 0;
  tcxRecvLoop((void *) &TCX_waiting_time);

  if ( shmid_attached[0] )
    displayImage4Byte( window[0], pCameraImage[0], COLS, ROWS );

  if ( shmid_attached[1] )
    displayImage4Byte( window[1], pCameraImage[1], COLS, ROWS );

  return 0;
}

/* ---------------------------------------------------------
 *
 *
 *
 * --------------------------------------------------------*/
int displaybParameters() {

  fprintf(stderr, "********  OPTIONS  **********\n");
  fprintf(stderr, "grabberConfig ......... = [%s.framegrabber]\n", grabberConfig );
  fprintf(stderr, "grabberType ........... = %s\n", grabberType );
  fprintf(stderr, "grabberWhichGrabber ... = %s\n", grabberWhichGrabber );
  fprintf(stderr, "grabberDev1 ........... = %s\n", grabberDev0 );
  fprintf(stderr, "grabberDev2 ........... = %s\n", grabberDev1 );
  fprintf(stderr, "*****************************\n");

  return 0;
}

/* ---------------------------------------------------------
 *
 * 
 *
 * --------------------------------------------------------*/
int main( int argc, char** argv ) {

  struct bParamList * params = NULL;
  char dummy[256];

  pCameraImage[0] = NULL;
  pCameraImage[1] = NULL;
  shmid[0] = 0;
  shmid[1] = 0;
  shmid_attached[0] = 0;
  shmid_attached[1] = 0;
  window[0] = 0;
  window[1] = 0;

  /* add some parameter files */
  params = bParametersAddFile(params, "etc/beeSoft.ini");
  
  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(params);

  /* find out the current grabber config in the robot 
     section should be one of { none, SICK }        */
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

  displaybParameters();

  cameraRegister();

  /* close function called if the server dies */
  initClient( "attach", commShutdown); 

  cameraConnect( 1 );		/* 1 -> wait until connection has been established */

  RaiInit();			/* init (but not start) scheduler   */

  catchInterrupts();

  initClientModules();		/* set up Rai modules to do         */
				/* communication for you            */

  signal( SIGINT, &ctrlcShutdown ); /* whenever user hits CTRL-C */

  registerCameraShmIdCallback( myCameraShmIdCallback );

  if ( useGrabber&1 || useGrabber == 0 ) {
    cameraRequestShmId( 0 );
    window[0] = createDisplay( "window0", COLS, ROWS, 100, 100 ); 
  }

  if ( useGrabber&2 ) {
    cameraRequestShmId( 1 );
    window[1] = createDisplay( "window1", COLS, ROWS, 100+10+COLS, 100 ); 
  }

  fprintf( stderr,
	   "\n\n Left click into the window writes the current image into file\n\n");

  for (;;)
    DoTheJob();
  
  killDisplay( window[0] );

  killDisplay( window[1] );

  RaiStart();

  return 0;
}

/*
 * $Log: mainatt.c,v $
 * Revision 1.11  1998/01/14 16:44:23  swa
 * Changed clientCloseFcn to take two arguments, the name of the module
 * that died and the TCX_MODULE_PTR module. Now one can distinguish in the
 * close function between modules that die.
 *
 * Revision 1.10  1997/10/04 01:06:47  swa
 * Fixed some bugs and inconsistencies wrt to both frame grabbers.
 *
 * Revision 1.9  1997/10/04 00:13:09  swa
 * First working version of the cameraServer that supports two (count'em
 * two!) framegrabbers.  Although the server seems to work just fine, it
 * has not yet been fully tested.
 *
 * Revision 1.8  1997/07/24 16:31:07  swa
 * Forgot to note, that the example program now dies when the cameraServer
 * dies.
 *
 * Revision 1.7  1997/07/24 16:28:36  swa
 * Removed .png stuff, Left click on the window in the example program
 * now saves a .ppm file, which can be read using xv. The order of the
 * bytes is surprisingly BGRX and NOT RGBX; i.e. all other algorithms
 * that access the data have to pay attention to that.
 *
 * Revision 1.6  1997/07/24 00:54:25  swa
 * Fixed some minor bugs, extended the README file and tested the version. The
 * cameraAttachExample does not yet die when the cameraServer dies. Will be
 * fixed in future versions.
 *
 * Revision 1.5  1997/07/22 22:47:17  swa
 * Added file handling (reading/saving). Renamed two (three?) functions to
 * be consistent.
 *
 * Revision 1.4  1997/07/04 17:28:31  swa
 * Left and right mouseclicks have been added to the example-program. Left
 * mouse saves the current cameraimage to a 4 byte .ppm format, right mouse
 * shows the current coordinates.
 *
 * Revision 1.3  1997/06/30 00:14:33  thrun
 * This new version allows saving of images (in .ppm 4-(four!)-byte format), by
 * left-clicking in cameraAttachExample's window . The cameraServer can then
 * later read that image and display it (instead of using the framegrabber). (swa)
 *
 * Revision 1.2  1997/06/25 18:46:50  thrun
 * Added support for tcx messages. It is for mainatt no longer necessary to
 * use argv[1]. It now sends out messages. (swa)
 *
 * Revision 1.1  1997/06/24 16:02:51  thrun
 * shared memory.
 *
 *
 */
