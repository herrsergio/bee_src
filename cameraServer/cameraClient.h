
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

#ifndef CAMERA_H
#define CAMERA_H

#ifdef __cplusplus
extern "C" {
#endif


  /* ====================================================================== *
   *
   * PART 1:   Procedures concerned with registration/connection
   *
   * ====================================================================== */


void cameraRegister();

int cameraConnect(int wait_till_established); /* if parameter is 1, then
					       * this will wait until
					       * connection has been 
					       * established */

void tcxRegisterCloseHnd(void (*closeHnd)());


int cameraConnected;		/* 1, if there is a connection to *xs
				 * the camera server, 0 if not    */


  /* ====================================================================== *
   *
   * PART 2:   Procedures and data structures 
   *           concerned with the transmission of raw images
   *
   * ====================================================================== */


typedef struct cameraImageType {
  int                xsize, ysize; /* image size (number of pixels)*/
  unsigned char     *red;	/* image data                   */
  unsigned char     *green;	/* image data                   */
  unsigned char     *blue;	/* image data                   */
  int                numGrabber; /* zero for first graber, ... */
} cameraImageType;

typedef int (*cameraImageCallbackType) (cameraImageType *cameraImage);

typedef struct cameraReqShmIdType {
  int numGrabber;		/* zero for first grabber, ... */
} cameraReqShmIdType;

typedef struct cameraShmIdType {
  int shmid;			/* id of the shm segment */
  int numGrabber;		/* zero for first grabber, ... */
} cameraShmIdType;

typedef int (*cameraShmIdCallbackType) (cameraShmIdType *cameraShmId);

typedef struct cameraFileType {
  int dummy;
} cameraFileType;

typedef int (*cameraFileCallbackType) (cameraFileType *cameraFile);

void registerCameraImageCallback( cameraImageCallbackType fcn );

void registerCameraShmIdCallback( cameraShmIdCallbackType fcn );

void registerCameraFileCallback( cameraFileCallbackType fcn );

void cameraRequestShmId( int numGrabber );

void cameraRequestImage( int numGrabber, int xsize, int ysize );

void cameraSubscribeImage( int numGrabber,
			   int number,
			   int xsize, 
			   int ysize ); /* 0=unsubscribe, n>0 = every n-th */

void cameraStartSaving( int numGrabber, char *filename );

void cameraStopSaving( int numGrabber );

void cameraSaveFile( int numGrabber, char *filename, int num );

void cameraLoadFile( int numGrabber, char *filename );

void cameraRestartGrabber( int numGrabber ); /* zero for first grabber, ... */

void cameraStopGrabber( int numGrabber ); /* zero for first grabber, ... */

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_H */

/*
 * $Log: cameraClient.h,v $
 * Revision 1.13  1998/01/13 00:35:13  swa
 * Added two new functions -- start and stop. They can start and stop either
 * of the two frame grabbers.
 *
 * Revision 1.12  1997/10/04 00:13:06  swa
 * First working version of the cameraServer that supports two (count'em
 * two!) framegrabbers.  Although the server seems to work just fine, it
 * has not yet been fully tested.
 *
 * Revision 1.11  1997/07/24 18:48:01  swa
 * Renamed two internal functions. Tested it again with the Tcl program. Works
 * fine. :)
 *
 * Revision 1.10  1997/07/23 22:43:32  swa
 * This is the first version of camControl, a Tcl program that remotely controls
 * the cameraServer. The user can save and read frames. recorder was renamed
 * to camControl, since the program will not only record but also read frames.
 *
 * Revision 1.9  1997/07/22 22:47:16  swa
 * Added file handling (reading/saving). Renamed two (three?) functions to
 * be consistent.
 *
 * Revision 1.8  1997/06/25 18:46:49  thrun
 * Added support for tcx messages. It is for mainatt no longer necessary to
 * use argv[1]. It now sends out messages. (swa)
 *
 * Revision 1.7  1997/06/23 23:48:43  thrun
 * shared memory and other stuff (swa)
 *
 * Revision 1.6  1997/06/23 02:35:59  thrun
 * Stefan's new camera server with shared memory
 *
 * Revision 1.5  1997/06/21 22:36:22  thrun
 * Improved interface, cleaner
 *
 * Revision 1.4  1997/06/20 01:09:28  thrun
 * Renamed function names to be more consistent. Currently the cameraServer
 * can track various object based on its color information. The color info
 * is supplied in a seperate textfile and is loaded into the server upon the
 * first request. (swa)
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
