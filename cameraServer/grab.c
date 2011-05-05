
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
#include <rai.h>
#include <bUtils.h>

#include "tcx.h"
#include "robot_specifications.h"
#include "tcxP.h"
#include "global.h"
#include "CAMERA-messages.h"	/* messages for VISION   */

/* #include "baseClient.h" */
/* #include "pantiltClient.h" */

#include <sys/time.h>
#include <sys/types.h>
#include <sys/ipc.h>		/* shm stuff */
#include <sys/shm.h>		/* shm stuff */

#include <signal.h>
#include <sys/mman.h>

#if (!defined(sun))
#include <sys/fcntl.h> 
#include <ioctl_meteor.h> 
#endif

#include <Common.h>
#include <EZX11.h>
#include "display.h"
#include "imagesize.h"

#include "grab.h"
#include "misc.h"
#include "handlers.h"

int display;
int tcx;
int color;

const char *grabberConfig;
const char *grabberType;
const char *grabberWhichGrabber;
const char *grabberDev0;
const char *grabberDev1;

/* ---------------------------------------------------------
 *
 * whenever user hits CTRL-C
 *
 * --------------------------------------------------------*/
void abortMeteor( int sig ) {

  fprintf( stderr, "\nCaught interrupt signal %d - cleaning up...\n", sig );
  fflush( stdout );
  fflush( stderr );

#if ( !defined(sun) )
  if ( useGrabber&1 )
    signal( SIGUSR1, SIG_IGN );

  if ( useGrabber&2 )
    signal( SIGUSR2, SIG_IGN );

  if ( useGrabber&1 )
    close( meteor[0] );

  if ( useGrabber&2 )
    close( meteor[1] );
#endif

  if ( useGrabber&1 || useGrabber==0 )
    shmdt( location_shmem[0] );

  if ( useGrabber&2 )
    shmdt( location_shmem[1] );

  if ( useGrabber&1 || useGrabber==0 )
    shmctl( shmid[0], IPC_RMID, NULL );

  if ( useGrabber&2 )
    shmctl( shmid[1], IPC_RMID, NULL );

  exit( 0 );
}


/* ---------------------------------------------------------
 *
 * is called when i dies.
 *
 * --------------------------------------------------------*/
void commShutdown( ) {

  fprintf( stderr, "%s: Somebody died. Exiting.\n", __FILE__ );

  RaiShutdown();

  abortMeteor( 0 );

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int openAndReadFile( int numGrabber, char *filename ) {

  FILE *fp;
  int done = 0;
  int frame;
  char dump[ROWS*COLS*4];
  struct timeval TCX_waiting_time = {0, 0};

  if ( useGrabber==0 && numGrabber==1 ) {
    fprintf( stderr,"%s: Using wrong frame grabber.\n", __FUNCTION__ );
    return -1;
  }
  
  if ( !useGrabber ) {		/* no framegrabber? -> use only first slot */
    numGrabber = 0;
  }

  if ( numGrabber!=0 && numGrabber!=1 ) {
    MISCERROR;
    return -1;
  }

  /* trying to use frame which is not in use */
  if ( useGrabber && !((numGrabber+1)&useGrabber) ) {
    fprintf( stderr,"%s: Using wrong frame grabber.\n", __FUNCTION__ );
    return -1;
  }

#if ( !defined(sun) )
  stopMeteor( numGrabber );
#endif

  /* open the file and read the frames */
  fprintf( stderr, "Reading %s.\n", filename);
  fp = fopen( filename, "rb" );
  if ( !fp ) {
    perror("\nError opening filename");
    return -1;
  }

  frame = -1;

  while ( !done ) {

    frame++;

/*     fprintf( stderr,"." ); */

    fprintf( stderr,"%3d \n", frame );

    if ( fread(dump,1,ROWS*COLS*4,fp)!=ROWS*COLS*4 ) {

      fprintf( stderr, "\nError reading frame %d from file, eof?\n", frame+1);
      fclose( fp );
      done = 1;

    } else {

      memcpy( location_shmem[numGrabber], dump, (size_t) COLS*ROWS*4 );

      /* the line below segfaults, cant write there! */
/*       memcpy( pCameraImage[numGrabber], dump, (size_t) COLS*ROWS*4 ); */

      if ( display ) {
	
	if ( color )
	  displayImage4ByteColor( CameraWindow[numGrabber], 
				  location_shmem[numGrabber], 
				  COLS, ROWS );
	else
	  displayImage4Byte( CameraWindow[numGrabber], 
			     location_shmem[numGrabber], 
			     COLS, ROWS );

      }	
	
      TCX_waiting_time.tv_sec  = 0;
      TCX_waiting_time.tv_usec = 0;
#ifdef TCX_debug
      fprintf( stderr,"(" );
#endif
      tcxRecvLoop((void *) &TCX_waiting_time);
#ifdef TCX_debug
      fprintf( stderr,")" );
#endif

      send_automatic_image_update( numGrabber, location_shmem[numGrabber] );

#ifdef SEBASTIAN		/* no clue what this is for ... */
      connect_to_Base();
      connect_to_Pantilt();
#endif

    }

  }

  fprintf( stderr,"\n" );

  return 0;
}

/* ---------------------------------------------------------
 *
 * this function is called everytime the frame grabber
 * receives a new frame
 *
 * --------------------------------------------------------*/
void gotframe( int signalnumber ) {

  int numGrabber;
  static int framecounter[2] = { 0, 0 };
  static int semaphore[2] = { 0, 0 };
  struct timeval TCX_waiting_time = {0, 0};

#if ( defined(G_DEBUG_PERFORMANCE) )
  static int frame_count[2] = { 0, 0 };
  static struct timeval tp1[2], tp2[2];
  int time_elapsed[2];
#endif

  if ( signalnumber == 10 )	/* first framegrabber */
    numGrabber = 0;
  else if ( signalnumber == 12 ) /* second framegrabber */
    numGrabber = 1;
  else {
    fprintf( stderr,"%s: You have to change the signal numbers in %s.\n",
	     __FILE__, __FUNCTION__ );
    return;
  }

  /* -----  Poor man's semaphore implementation -------- */
  if ( semaphore[numGrabber] ) {

    fprintf( stderr,"i'm too fast\n");
    return;

  } else semaphore[numGrabber] = 1;


#if ( defined(G_DEBUG_PERFORMANCE) )
  if ( ++frame_count[numGrabber] == MAXFPS ) {
    gettimeofday( &tp2[numGrabber], NULL );
    time_elapsed[numGrabber] = (int) 
      (((tp2[numGrabber].tv_sec-tp1[numGrabber].tv_sec)*1000)+
       ((tp2[numGrabber].tv_usec-tp1[numGrabber].tv_usec)/1000));
    if ( 1 ) {
      fprintf ( stderr,
	        "%s: Caught %d frames with fg %d in %.1f sec, ie %.2f fps.\n",
		__FILE__,
		frame_count[numGrabber],
		numGrabber,
		((float)time_elapsed[numGrabber])/1000.0,
		((float)frame_count[numGrabber]) / 
		(((float)time_elapsed[numGrabber])/1000.0)
		);
    }
    tp1[numGrabber] = tp2[numGrabber];
    frame_count[numGrabber] = 0;
  }
#endif

  memcpy( location_shmem[numGrabber], 
	  pCameraImage[numGrabber], (size_t) COLS*ROWS*PIXELSIZE );

  /* did we receive a save message via TCX */
  if ( saving[numGrabber] ) {
    
    /* if this is our first frame, open videofp.  */
    /* videofp will be closed upon receiving 0 in */
    /* handlers */
    if ( !savefileopen[numGrabber] ) {
      videofp[numGrabber] = fopen( filename[numGrabber] , "wb" ); /* filename is set in CAM_save_handler */
      if ( !videofp[numGrabber] ) {
	fprintf( stderr,"Error with %s.", filename[numGrabber] );
	perror( "Error" );
	saving[numGrabber]     = 0;
	saveframes[numGrabber] = 0;
	return;
      }
      savefileopen[numGrabber] = 1;
      framecounter[numGrabber]    = 0;
    }

    if ( ROWS*COLS*4 != fwrite( pCameraImage[numGrabber],sizeof(char),ROWS*COLS*4,videofp[numGrabber] ) ) {
      perror("fwrite()");
      fclose( videofp[numGrabber] );
      commShutdown(NULL);
    } else { 
#ifdef DO_DA_BEEP
      fprintf( stderr, "\007" );
#endif
#ifdef DO_DA_COUNTER
      fprintf( stderr, "%d\n", framecounter[numGrabber] );
#else
      fprintf( stderr, "." );
#endif
    }

    framecounter[numGrabber]++;

    if ( saveframes[numGrabber] > 0 && framecounter[numGrabber] == saveframes[numGrabber] ) {
      saving[numGrabber]       = 0;
      savefileopen[numGrabber] = 0;
      saveframes[numGrabber]   = 0;
      fclose( videofp[numGrabber] );
      fprintf( stderr, "\n" );
    }

  }

  if ( display ) {

    if ( color )
      displayImage4ByteColor( CameraWindow[numGrabber], pCameraImage[numGrabber], COLS, ROWS );
    else
      displayImage4Byte( CameraWindow[numGrabber], pCameraImage[numGrabber], COLS, ROWS );

    TCX_waiting_time.tv_sec  = 0;
    TCX_waiting_time.tv_usec = 0;
#ifdef TCX_debug
    fprintf( stderr,"(" );
#endif
    tcxRecvLoop((void *) &TCX_waiting_time);
#ifdef TCX_debug
    fprintf( stderr,")" );
#endif

  }
  
  send_automatic_image_update( numGrabber, pCameraImage[numGrabber] );

#ifdef SEBASTIAN		/* no clue what this is for ... */
  connect_to_Base();
  connect_to_Pantilt();
#endif

  /* and let me in again */
  semaphore[numGrabber] = 0;

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int initMeteor( int numGrabber ) {

#if ( !defined(sun) )
  int c;
  const char *dev = NULL;
#endif

  if ( numGrabber!=0 && numGrabber!=1 ) {
    MISCERROR;
    return -1;
  }

#if ( !defined(sun) )

  if ( useGrabber&(numGrabber+1) ) {
    if ( numGrabber == 0 )
      dev = grabberDev0;
    else if ( numGrabber == 1 )
      dev = grabberDev1;
    if ((meteor[numGrabber] = open( dev, O_RDONLY)) < 0) {
      fprintf( stderr,"open failed on device %s\n", dev );
      return -1;
    }
  }

  geo[numGrabber].rows    = ROWS;
  geo[numGrabber].columns = COLS;
  geo[numGrabber].frames  = 1;
  /* geo.oformat = METEOR_GEO_RGB24 | METEOR_GEO_ODD_ONLY;*/
  
  geo[numGrabber].oformat = METEOR_GEO_RGB24;
  if ( useGrabber&(numGrabber+1) ) {
    if (ioctl(meteor[numGrabber], METEORSETGEO, &geo[numGrabber]) < 0) {
      fprintf( stderr,"ioctl SetGeometry failed\n");
      return -1;
    }
  }

  if ( useGrabber&(numGrabber+1) ) {
    pCameraImage[numGrabber]=(char *) mmap ((caddr_t)0, PIXELSIZE*ROWS*COLS, 
					    PROT_READ,MAP_FILE|MAP_PRIVATE,
					    meteor[numGrabber], (off_t)0);
  } else {
    pCameraImage[numGrabber]=(char *) malloc( PIXELSIZE*ROWS*COLS*sizeof(char) );
    if ( pCameraImage[numGrabber] == NULL ) {
      MALLOC;
      return -1;
    }
  }

  if ( pCameraImage[numGrabber]==(char *)(-1) ) { 
    fprintf( stderr,"mmap failed\n"); 
    return -1;
  }

#else
  
  pCameraImage[numGrabber]=(char *) malloc( PIXELSIZE*ROWS*COLS*sizeof(char) );
  if ( pCameraImage[numGrabber] == NULL ) {
    MALLOC;
    return -1;
  }

#endif

  shmid[numGrabber] = shmget( IPC_PRIVATE, PIXELSIZE*ROWS*COLS, 0600 | IPC_CREAT );
  if ( shmid[numGrabber] == -1 ) {
    perror("shmid's error" );
    fprintf( stderr, "shmid is %d\n", shmid[numGrabber] );
    return -1;
  }
  location_shmem[numGrabber] = (char*)shmat( shmid[numGrabber], (char*)NULL, (int)0 ); 
  if ( location_shmem[numGrabber] == (char*)(-1) ) {
    perror("shmat's error" );
    return -1;
  }

#if ( !defined(sun) )

  if ( useGrabber&(numGrabber+1) ) {
    c = DEFAULT_FORMAT;
    if (ioctl(meteor[numGrabber], METEORSFMT, &c) < 0) {
      fprintf( stderr,"ioctl SetFormat failed\n");
      return -1;
    }
    c = METEOR_INPUT_DEV0;
    if (ioctl(meteor[numGrabber], METEORSINPUT, &c) < 0) {
      fprintf( stderr,"ioctl Setinput failed\n");
      return -1;
    }

    sigact.sa_handler  = gotframe;
    sigact.sa_flags    = 0;
    sigact.sa_restorer = NULL;
    sigemptyset(&sigact.sa_mask);


    /* register gotframe() */
    if ( numGrabber == 0 ) {
      if ( sigaction(SIGUSR1, &sigact, NULL) ) {
	fprintf( stderr,"sigaction0 failed\n");
	return -1;
      }
    } else if ( numGrabber == 1 ) {
      if ( sigaction(SIGUSR2, &sigact, NULL) ) {
	fprintf( stderr,"sigaction1 failed\n");
	return -1;
      }
    }
    if ( numGrabber==0 )
      c = SIGUSR1;
    else if ( numGrabber==1 )
      c = SIGUSR2;
    if (ioctl(meteor[numGrabber], METEORSSIGNAL, &c) < 0) {
      fprintf( stderr,"ioctl SetSignal failed\n");
      return -1;
    }
  }

#endif
    
  signal(SIGTERM, &abortMeteor);
    
  signal(SIGINT,  &abortMeteor);
    
  return 0;
}   

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int stopMeteor( int numGrabber ) {

#if ( !defined(sun) )
  int c;

  /* stop the framegrabber from grabbing images */
  if ( useGrabber )
    fprintf( stderr, "Stopping frame grabber %d.\n", numGrabber );

  /* first disable application signals */
  if ( useGrabber ) {
    c = 0;
    if (ioctl(meteor[numGrabber], METEORSSIGNAL, &c) < 0) {
      perror("ioctl 0 failed" );
      return -1;
    }
    
    c = METEOR_CAP_STOP_CONT; 
    if (ioctl(meteor[numGrabber], METEORCAPTUR, &c)) {
      fprintf( stderr,"ioctl METEOR_CAP_STOP_CONT failed\n");
      return -1;
    }
  }

#endif

  return 0;

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int startMeteor( int numGrabber, int how ) {

#if ( !defined(sun) )
  int c;
#endif

  if ( !useGrabber ) {
    return 0;
  }

  if ( numGrabber!=0 && numGrabber!=1 ) {
    MISCERROR;
    return -1;
  }

  if ( useGrabber && !((numGrabber+1)&useGrabber) ) {	/* wrong frame grabber */
    fprintf( stderr,"%s: Using wrong frame grabber.\n", __FUNCTION__ );
    return -1;
  }

  if ( how == 0 ) {		/* the default */
    
#if ( !defined(sun) )
    c = METEOR_INPUT_DEV0;
    if (ioctl(meteor[numGrabber], METEORSINPUT, &c) < 0) {
      fprintf( stderr, "Meteor: METEORSINPUT ioctl failed: %d\n", errno);
      return -1;
    }
    c  = METEOR_CAP_CONTINOUS; 
    if (ioctl(meteor[numGrabber], METEORCAPTUR, &c)) {
      fprintf( stderr,"ioctl CaptContinuous failed\n");
      return -1;
    }
    fprintf( stderr,"Frame grabber %d started.\n", numGrabber );
#endif

  } else {			/* restart */

#if ( !defined(sun)  )
    fprintf( stderr, "Starting frame grabber %d.\n", numGrabber );
    if ( numGrabber==0 ) {
      c = SIGUSR1;
    } else if ( numGrabber==1 ) {
      c = SIGUSR2;
    } 
    if (ioctl(meteor[numGrabber], METEORSSIGNAL, &c) < 0) {
      fprintf( stderr,"ioctl SIGUSR1 or 2 failed\n");
      return -1;
    }
    c = METEOR_CAP_CONTINOUS;
    if (ioctl(meteor[numGrabber], METEORCAPTUR, &c)) {
      fprintf( stderr,"ioctl METEOR_CAP_CONTINOUS failed\n");
      return -1;
    }
    fprintf( stderr,"Frame grabber %d restarted.\n", numGrabber );
#endif
    
  }

  return 0;

}

/* ---------------------------------------------------------
 *
 *
 *
 * --------------------------------------------------------*/
int displaybParameters() {

  fprintf(stderr, "********  OPTIONS  **********\n");
  fprintf(stderr, "display ............... = %d\n", display );
  fprintf(stderr, "tcx ................... = %d\n", tcx );
  fprintf(stderr, "color ................. = %d\n", color );
  fprintf(stderr, "grabberConfig ......... = [%s.framegrabber]\n", grabberConfig );
  fprintf(stderr, "grabberType ........... = %s\n", grabberType );
  fprintf(stderr, "grabberWhichGrabber ... = %s\n", grabberWhichGrabber );
  fprintf(stderr, "grabberDev1 ........... = %s\n", grabberDev0 );
  fprintf(stderr, "grabberDev2 ........... = %s\n", grabberDev1 );

#if ( defined(sun) )
  fprintf(stderr, "I will not use the frame grabber.\n");
#else
  if ( !useGrabber )
    fprintf(stderr, "I will not use the frame grabber.\n");
#endif
  fprintf(stderr, "*****************************\n");

  return 0;
}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int main(int argc, char** argv ) {
  struct bParamList * params = NULL;
  struct timeval TCX_waiting_time = {0, 0};
  char dummy[256];
  
  saving[0]       = 0;
  saving[1]       = 0;
  savefileopen[0] = 0;
  savefileopen[1] = 0;
  saveframes[0]   = 0;
  saveframes[1]   = 0;
  
  /* set defaults */
  params = bParametersAddEntry(params, "", "display", "0");
  params = bParametersAddEntry(params, "", "tcx", "1");
  params = bParametersAddEntry(params, "", "color", "0");
 
  /* add some parameter files */
  params = bParametersAddFile(params, "etc/beeSoft.ini");
  
  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  params = bParametersAddEntry(params, "", "fork", "no");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(params);

  tcxMachine = (char*)bParametersGetParam(params, "", "TCXHOST");
  display = atoi(bParametersGetParam(params, "", "display"));
  tcx     = atoi(bParametersGetParam(params, "", "tcx"));
  color   = atoi(bParametersGetParam(params, "", "color"));

  check_commandline_parameters(argc, argv);

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
      commShutdown();
    }
    if ( strcmp( grabberType, "matrox-meteor" ) ) {
      fprintf( stderr,"I cannot handle framegrabbers other than the ");
      fprintf( stderr,"Matrox Meteor at this time.\n");
      commShutdown();
    }
    useGrabber = atoi( grabberWhichGrabber );
    if ( useGrabber<0 || useGrabber>3 ) {
      MISCERROR;
      return -1;
    }
  }

  displaybParameters();

  /* ---------------------- tcx ---------------------------------- */
  if (tcx)
    CAMERA_initialize_tcx();

  /* ------------------------------------------------------------------------ */
  if ( display == 1 ) {
    if ( useGrabber&1 )
      CameraWindow[0] = createDisplay("camera0", COLS, ROWS, 300, 100 );
    if ( useGrabber&2 )
      CameraWindow[1] = createDisplay("camera1", COLS, ROWS, 300+10+COLS, 100 );
    if ( color ) {		/* Make private color map. */
      if ( (useGrabber&1) && EZX_LoadBestColorMap( w_id[CameraWindow[0]] ) == 0 ) {
	MISCERROR;
	return -1;
      }
      if ( (useGrabber&2) && EZX_LoadBestColorMap( w_id[CameraWindow[1]] ) == 0 ) {
	MISCERROR;
	return -1;
      }
    }
  }

  if ( bRobot.fork )
    bDaemonize("cameraServer.log");

  if ( useGrabber&1 || useGrabber == 0 )
    if ( initMeteor( 0 ) != 0 ) {
      fprintf( stderr,"initMeteor failed for frame grabber 0\n");
      return -1;
    }

  if ( useGrabber&2 )
    if ( initMeteor( 1 ) != 0 ) {
      fprintf( stderr,"initMeteor failed for frame grabber 1\n");
      return -1;
    }
  
  if ( useGrabber&1 )
    if ( startMeteor( 0, 0 ) != 0 ) {
      fprintf( stderr,"startMeteor failed for frame grabber 0\n");
      return -1;
    }

  if ( useGrabber&2 )
    if ( startMeteor( 1, 0 ) != 0 ) {
      fprintf( stderr,"startMeteor failed for frame grabber 1\n");
      return -1;
    }
  
  for (;;){			/* loops forever */

    if (tcx) {

      TCX_waiting_time.tv_sec  = 1;
      TCX_waiting_time.tv_usec = 0;
#ifdef TCX_debug
      fprintf( stderr,"{" );
#endif
      tcxRecvLoop((void *) &TCX_waiting_time);
#ifdef TCX_debug
      fprintf( stderr,"}" );
#endif
    }

  } 

  return 0;

}

/*
 * $Log: grab.c,v $
 * Revision 1.32  1998/02/08 00:13:13  swa
 * now works with redhat5.0
 *
 * Revision 1.31  1998/01/13 00:35:13  swa
 * Added two new functions -- start and stop. They can start and stop either
 * of the two frame grabbers.
 *
 * Revision 1.30  1997/11/06 18:15:22  swa
 * Grrr. Fixed a -Wall error.
 *
 * Revision 1.29  1997/11/06 17:54:24  swa
 * Fixed a bug with TCXHOST. If the environment variable TCXHOST is set,
 * take it. If not, take the TCXHOST from beeSoft.ini. If this doesn't
 * work, take localhost. If this doesn't work: bummer.
 *
 * Revision 1.28  1997/10/04 18:01:06  swa
 * Fixed a bug in CAMERA-messages so that sending images over TCX works again.
 *
 * Revision 1.27  1997/10/04 14:58:15  swa
 * Changed some things in an attempt to compile the stuff on a SUN. There are
 * still some `implicit declaration' warnings that I cannot get rid of.
 *
 * Revision 1.26  1997/10/04 01:06:46  swa
 * Fixed some bugs and inconsistencies wrt to both frame grabbers.
 *
 * Revision 1.25  1997/10/04 00:13:07  swa
 * First working version of the cameraServer that supports two (count'em
 * two!) framegrabbers.  Although the server seems to work just fine, it
 * has not yet been fully tested.
 *
 * Revision 1.24  1997/09/20 22:39:37  swa
 * added parameter and ini section to the documentation. The compiler flag NO_GRABBER
 * is not longer necessary, since we get that info now from beeSoft.ini.
 *
 * Revision 1.23  1997/09/20 00:47:42  swa
 * You can now use the cameraServer on a Linux box without a
 * framegrabber. This no longer requires recompiling the stuff but is set
 * in an entry in beeSoft.ini. Speaking of beeSoft.ini: the devices and
 * other stuff is not hardcoded anymore but read from the .ini file.
 *
 * Revision 1.22  1997/09/19 23:03:17  swa
 * Image-over-TCX works now correctly when using file as the source. A bunch
 * of function calls were missing in the load-file function.
 *
 * Revision 1.21  1997/09/17 15:48:41  swa
 * Added a Makefile flag to display the framenumber while saving.
 *
 * Revision 1.20  1997/08/11 15:52:57  swa
 * Changed the name of the module to cameraServer (looks nicer), removed
 * libcameraClient.h from the Makefile. Added beeps.
 *
 * Revision 1.19  1997/08/09 18:41:20  swa
 * Changed the cameraServer so that it beeps (can be changed thru compile flag,
 * default is off) whenever it saved a frame. This can be used to verify that
 * the harddisk is actually doing something while saving images (for some reason,
 * the harddisk on bo sometimes stops for 2-8s, which is very annoying.
 * Especially odd, because reading seems to be almost perfectly linear.)
 *
 * Revision 1.18  1997/08/02 18:50:54  swa
 * Support for file-only mode (SUN and Linux) added.
 *
 * Revision 1.17  1997/07/25 16:29:12  swa
 * Now compiles under SUN.
 *
 * Revision 1.16  1997/07/24 00:54:24  swa
 * Fixed some minor bugs, extended the README file and tested the version. The
 * cameraAttachExample does not yet die when the cameraServer dies. Will be
 * fixed in future versions.
 *
 * Revision 1.15  1997/07/23 22:43:32  swa
 * This is the first version of camControl, a Tcl program that remotely controls
 * the cameraServer. The user can save and read frames. recorder was renamed
 * to camControl, since the program will not only record but also read frames.
 *
 * Revision 1.14  1997/07/22 23:00:12  swa
 * The little recorder program now receives the callback once the file has
 * been completely loaded.
 *
 * Revision 1.13  1997/07/22 22:47:16  swa
 * Added file handling (reading/saving). Renamed two (three?) functions to
 * be consistent.
 *
 * Revision 1.12  1997/06/30 00:14:33  thrun
 * This new version allows saving of images (in .ppm 4-(four!)-byte format), by
 * left-clicking in cameraAttachExample's window . The cameraServer can then
 * later read that image and display it (instead of using the framegrabber). (swa)
 *
 * Revision 1.11  1997/06/25 18:46:50  thrun
 * Added support for tcx messages. It is for mainatt no longer necessary to
 * use argv[1]. It now sends out messages. (swa)
 *
 * Revision 1.10  1997/06/24 22:48:13  thrun
 * checking of the command line arguments.
 *
 * Revision 1.9  1997/06/24 22:28:24  thrun
 * forking + fixed the tcx option.
 *
 * Revision 1.8  1997/06/24 16:46:50  thrun
 * Fixed some typos. (swa)
 *
 * Revision 1.7  1997/06/23 23:48:43  thrun
 * shared memory and other stuff (swa)
 *
 * Revision 1.6  1997/06/23 02:36:00  thrun
 * Stefan's new camera server with shared memory
 *
 * Revision 1.5  1997/06/21 22:36:22  thrun
 * Improved interface, cleaner
 *
 * Revision 1.4  1997/06/20 01:09:29  thrun
 * Renamed function names to be more consistent. Currently the cameraServer
 * can track various object based on its color information. The color info
 * is supplied in a seperate textfile and is loaded into the server upon the
 * first request. (swa)
 *
 * Revision 1.3  1997/06/18 15:58:13  thrun
 * now with auto-update
 *
 * Revision 1.2  1997/06/17 18:22:17  thrun
 * Compiles now on a sun as well.
 *
 * Revision 1.1.1.1  1997/06/16 22:32:20  thrun
 * New Camera Server, much faster, more reliable, works already with
 * commander.
 *
 */
