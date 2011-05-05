
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
#ifndef lint
static char rcsid[] =
"$Id: cameraClient.c,v 1.17 1998/01/15 00:23:03 swa Exp $";
static void *const use_rcsid = ((void)&use_rcsid, (void)&rcsid, 0);
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <ctype.h>
#include <sys/time.h>

#include <tcx.h>
#include <rai.h>

#define TCX_define_variables
#define DEFINE_REPLY_HANDLERS
#include <CAMERA-messages.h>

#include <cameraClient.h>
#include <raiClient.h>

#ifndef PI
#define PI M_PI
#endif

/*  extern void tcxRegisterCloseHnd(void (*closeHnd)()); */

/*
 * This should include hostname and
 * PID or something like that
 */

#define CAMERA_CLIENT_NAME "cameraclient"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


struct timeval CAMERA_last_send_message_time = {0, 0};

int cameraConnected = 0;

static int image_subscription_numGrabber = 0;
static int image_subscription = 0;
static int image_subscription_xsize = 1;
static int image_subscription_ysize = 1;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

cameraImageCallbackType userCameraFcn=NULL;

void registerCameraImageCallback(cameraImageCallbackType fcn)
{
  userCameraFcn=fcn;
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

cameraFileCallbackType userFileFcn=NULL;

void registerCameraFileCallback(cameraFileCallbackType fcn)
{
  userFileFcn=fcn;
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

cameraShmIdCallbackType userShmIdFcn=NULL;

void registerCameraShmIdCallback(cameraShmIdCallbackType fcn)
{
  userShmIdFcn=fcn;
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void CAMERA_image_reply_handler(TCX_REF_PTR             ref,
				CAMERA_image_reply_ptr cameraImage)
{
  cameraImageType camera_image;

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  ref = ref;			/* -Wall */

  camera_image.xsize = cameraImage->xsize;
  camera_image.ysize = cameraImage->ysize;
  camera_image.red   = cameraImage->red;
  camera_image.green = cameraImage->green;
  camera_image.blue  = cameraImage->blue;
  camera_image.numGrabber = cameraImage->numGrabber;

  if (userCameraFcn != NULL) {
    userCameraFcn(&camera_image);
  }

  tcxFree("CAMERA_image_reply", cameraImage);
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void CAMERA_load_reply_handler(TCX_REF_PTR           ref,
			       CAMERA_load_reply_ptr cameraFile)
{
  cameraFileType data;

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif
  
  ref = ref;			/* -Wall */

  data.dummy = cameraFile->dummy;

  if (userFileFcn != NULL) {
    userFileFcn( &data );
  }

  tcxFree("CAMERA_load_reply", cameraFile);
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void CAMERA_shmid_reply_handler( TCX_REF_PTR            ref,
				 CAMERA_shmid_reply_ptr shmid )
{
  cameraShmIdType ShmId;

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

#ifdef TCX_debug
  fprintf(stderr, "Received a CAMERA_shmid_reply message: %d %d\n",
	  shmid->shmid, shmd->numGrabber );
#endif

  ref = ref;			/* -Wall */

  ShmId.shmid  = shmid->shmid; /* this line sucks */
  ShmId.numGrabber  = shmid->numGrabber; 

  if (userShmIdFcn != NULL) {
    userShmIdFcn(&ShmId);
  }

  tcxFree("CAMERA_shmid_reply", shmid);
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void
cameraRequestImage( int numGrabber, int xsize, int ysize )
{
  CAMERA_image_query_type specs;

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if (cameraConnected){

    if ( numGrabber!=0 && numGrabber!=1 ) {
      fprintf( stderr,"%s: numGrabber is not in {0,1}.\n", __FUNCTION__ );
      return;
    }

    specs.numGrabber = numGrabber;
    specs.xsize = xsize;
    specs.ysize = ysize;
    tcxSendMsg(CAMERA, "CAMERA_image_query", &specs);
  } else {
    fprintf(stderr,
	    "%s(%s): cameraServer is not connected. \n", __FILE__, __FUNCTION__);
    cameraConnect( 0 );
  }


}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void
cameraRequestShmId( int numGrabber )
{

  CAMERA_shmid_query_type specs;

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if (cameraConnected){

    if ( numGrabber!=0 && numGrabber!=1 ) {
      fprintf( stderr,"%s: numGrabber is not in {0,1}.\n", __FUNCTION__ );
      return;
    }
    
    specs.numGrabber = numGrabber;
    tcxSendMsg(CAMERA, "CAMERA_shmid_query", &specs);
  } else {
    fprintf(stderr,
	    "%s(%s): cameraServer is not connected. \n", __FILE__, __FUNCTION__);
    cameraConnect( 0 );
  }

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void 
camera_subscribe()
{
  CAMERA_register_auto_update_type subscribe;

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if (cameraConnected){
    subscribe.numGrabber  = image_subscription_numGrabber;
    subscribe.image       = image_subscription;
    subscribe.image_xsize = image_subscription_xsize;
    subscribe.image_ysize = image_subscription_ysize;
    tcxSendMsg(CAMERA, "CAMERA_register_auto_update", &subscribe);
  } else {
    fprintf(stderr,
	    "%s(%s): cameraServer is not connected. \n", __FILE__, __FUNCTION__);
    cameraConnect( 0 );
  }

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void 
cameraSubscribeImage( int numGrabber, int number, int xsize, int ysize )
{

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if ( numGrabber!=0 && numGrabber!=1 ) {
    fprintf( stderr,"%s: numGrabber is not in {0,1}.\n", __FUNCTION__ );
    return;
  }

  image_subscription_numGrabber = numGrabber;
  image_subscription = number;
  image_subscription_xsize = xsize;
  image_subscription_ysize = ysize;


  camera_subscribe();
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* ---------------------------------------------------------
 *
 * hook up to cameraServer. if cameraServer dies, try reconnecting 
 * every three seconds
 *
 * --------------------------------------------------------*/
int
cameraConnect( int wait_till_established )
{
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference;

#if ( defined(G_DEBUG_TCX) )
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);

  if (time_difference < 3.0)
    return -1;

  if ( wait_till_established ) { /* 1 */

    fprintf(stderr, "CameraClient: Connecting to Camera server...\n");
    CAMERA = tcxConnectModule(TCX_CAMERA_MODULE_NAME);
    cameraConnected = 1;
    fprintf(stderr, "CameraClient: Connected.\n");

  } else {			/* 0 */

    if ( cameraConnected == 0 || !CAMERA ) { /* doppelt haelt besser */
      fprintf(stderr, "CameraClient: Connecting to Camera server...\n");
      CAMERA  = tcxConnectOptional(TCX_CAMERA_MODULE_NAME);
      if( CAMERA ) {
	cameraConnected = 1;
	camera_subscribe();  
	fprintf(stderr, "CameraClient: Connected.\n");
      } else {
	cameraConnected = 0;
      }
    }

  }

  last_time.tv_sec  = this_time.tv_sec;
  last_time.tv_usec = this_time.tv_usec;

  return 0;
}


int
_cameraConnect(int wait_till_established)
{

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if (wait_till_established){
    fprintf(stderr, "CameraClient: Connecting to Camera Server...\n");
    CAMERA = tcxConnectModule(TCX_CAMERA_MODULE_NAME);
  }
  else{
    CAMERA = tcxConnectOptional(TCX_CAMERA_MODULE_NAME);
  }

  if (CAMERA){
    camera_subscribe();  
    
    cameraConnected = 1;

    fprintf(stderr, "CameraClient: Connected.\n");
    return 1;
  }
  else{
    cameraConnected = 0;
    return 0;
  }
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void 
cameraStartSaving( int numGrabber, char *filename ) {

  CAMERA_save_type data;
    
#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if (cameraConnected){

    if ( numGrabber!=0 && numGrabber!=1 ) {
      fprintf( stderr,"%s: numGrabber is not in {0,1}.\n", __FUNCTION__ );
      return;
    }
    
    sprintf( data.filename, "%s", filename );
    data.frames = -1;		/* continous */
    data.numGrabber = numGrabber; 
    
    tcxSendMsg(CAMERA, "CAMERA_save", &data);
  } else {
    fprintf(stderr,
	    "%s(%s): cameraServer is not connected. \n", __FILE__, __FUNCTION__);
    cameraConnect( 0 );
  }

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void 
cameraStopSaving( int numGrabber ) {

  CAMERA_save_type data;

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if (cameraConnected){

    if ( numGrabber!=0 && numGrabber!=1 ) {
      fprintf( stderr,"%s: numGrabber is not in {0,1}.\n", __FUNCTION__ );
      return;
    }
    
    data.frames = 0;		/* stop */
    data.numGrabber = numGrabber;
    
    sprintf( data.filename, "%s", "hi" );
    
    tcxSendMsg(CAMERA, "CAMERA_save", &data);
  } else {
    fprintf(stderr,
	    "%s(%s): cameraServer is not connected. \n", __FILE__, __FUNCTION__);
    cameraConnect( 0 );
  }

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void 
cameraSaveFile( int numGrabber, char *filename, int num ) {

  CAMERA_save_type data;

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif
  if (cameraConnected){

    if ( numGrabber!=0 && numGrabber!=1 ) {
      fprintf( stderr,"%s: numGrabber is not in {0,1}.\n", __FUNCTION__ );
      return;
    }
    
    data.numGrabber = numGrabber;
    
    sprintf( data.filename, "%s", filename );
    data.frames = num; /* stop */
    
    tcxSendMsg(CAMERA, "CAMERA_save", &data);
  } else {
    fprintf(stderr,
	    "%s(%s): cameraServer is not connected. \n", __FILE__, __FUNCTION__);
    cameraConnect( 0 );
  }

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void 
cameraLoadFile( int numGrabber, char *filename ) {

  CAMERA_load_type data;

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if (cameraConnected){
    
    if ( numGrabber!=0 && numGrabber!=1 ) {
      fprintf( stderr,"%s: numGrabber is not in {0,1}.\n", __FUNCTION__ );
      return;
    }
    
    data.numGrabber= numGrabber;
    sprintf( data.filename, "%s", filename );
  
    tcxSendMsg(CAMERA, "CAMERA_load", &data);
  } else {
    fprintf(stderr,
	    "%s(%s): cameraServer is not connected. \n", __FILE__, __FUNCTION__);
    cameraConnect( 0 );
  }

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void
cameraRegister()
{
  int numberOfMessages;
  int numberOfHandlers;

  TCX_REG_MSG_TYPE messages[] = { CAMERA_messages }; 
  
  numberOfMessages = sizeof(messages)/sizeof(TCX_REG_MSG_TYPE); 
  numberOfHandlers = 
    sizeof(CAMERA_reply_handler_array)/sizeof(TCX_REG_HND_TYPE); 

  registerInterface("", numberOfMessages, messages,
		    numberOfHandlers, CAMERA_reply_handler_array);
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void
cameraRestartGrabber( int numGrabber )
{
  CAMERA_startstop_type data;

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if (cameraConnected){
    
    if ( numGrabber!=0 && numGrabber!=1 ) {
      fprintf( stderr,"%s: numGrabber is not in {0,1}.\n", __FUNCTION__ );
      return;
    }
    
    data.numGrabber = numGrabber;
    data.action     = 1;
    tcxSendMsg(CAMERA, "CAMERA_startstop", &data);
  } else {
    fprintf(stderr,
	    "%s(%s): cameraServer is not connected. \n", __FILE__, __FUNCTION__);
    cameraConnect( 0 );
  }

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void
cameraStopGrabber( int numGrabber )
{

  CAMERA_startstop_type data;

#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if (cameraConnected){

    if ( numGrabber!=0 && numGrabber!=1 ) {
      fprintf( stderr,"%s: numGrabber is not in {0,1}.\n", __FUNCTION__ );
      return;
    }
    
    data.numGrabber = numGrabber;
    data.action     = 0;
    tcxSendMsg(CAMERA, "CAMERA_startstop", &data);
  } else {
    fprintf(stderr,
	    "%s(%s): cameraServer is not connected. \n", __FILE__, __FUNCTION__);
    cameraConnect( 0 );
  }

}

/*
 * $Log: cameraClient.c,v $
 * Revision 1.17  1998/01/15 00:23:03  swa
 * cameraClient handles restarts of the server nicely.
 *
 * Revision 1.16  1998/01/13 00:35:12  swa
 * Added two new functions -- start and stop. They can start and stop either
 * of the two frame grabbers.
 *
 * Revision 1.15  1997/10/04 00:13:05  swa
 * First working version of the cameraServer that supports two (count'em
 * two!) framegrabbers.  Although the server seems to work just fine, it
 * has not yet been fully tested.
 *
 * Revision 1.14  1997/07/24 18:48:00  swa
 * Renamed two internal functions. Tested it again with the Tcl program. Works
 * fine. :)
 *
 * Revision 1.13  1997/07/23 22:43:32  swa
 * This is the first version of camControl, a Tcl program that remotely controls
 * the cameraServer. The user can save and read frames. recorder was renamed
 * to camControl, since the program will not only record but also read frames.
 *
 * Revision 1.12  1997/07/22 22:47:15  swa
 * Added file handling (reading/saving). Renamed two (three?) functions to
 * be consistent.
 *
 * Revision 1.11  1997/06/25 18:46:49  thrun
 * Added support for tcx messages. It is for mainatt no longer necessary to
 * use argv[1]. It now sends out messages. (swa)
 *
 * Revision 1.10  1997/06/24 17:05:43  thrun
 * Fixed some debug flags (swa)
 *
 * Revision 1.9  1997/06/24 16:46:50  thrun
 * Fixed some typos. (swa)
 *
 * Revision 1.8  1997/06/23 23:48:42  thrun
 * shared memory and other stuff (swa)
 *
 * Revision 1.7  1997/06/21 22:36:21  thrun
 * Improved interface, cleaner
 *
 * Revision 1.6  1997/06/20 01:09:28  thrun
 * Renamed function names to be more consistent. Currently the cameraServer
 * can track various object based on its color information. The color info
 * is supplied in a seperate textfile and is loaded into the server upon the
 * first request. (swa)
 *
 * Revision 1.5  1997/06/19 21:43:38  thrun
 * nothing major, just a printf
 *
 * Revision 1.4  1997/06/19 21:24:29  thrun
 * nothing, really.
 *
 * Revision 1.3  1997/06/19 21:16:53  thrun
 * a little cleanup
 *
 * Revision 1.2  1997/06/19 21:06:57  thrun
 * Added various stuff for faceInfo (callbacks, etc)
 *
 * Revision 1.1  1997/06/19 03:54:17  thrun
 * client library created
 *
 */
