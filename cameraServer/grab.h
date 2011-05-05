
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
#ifndef GRAB_H_LOADED
#define GRAB_H_LOADED

#define DEFAULT_FORMAT METEOR_FMT_NTSC
#define MAXFPS 30
#define RGBSIZE 3
#define PIXELSIZE (RGBSIZE+1)
#define MAXSIZE (PIXELSIZE*ROWS*COLS)

char *tcxMachine;		/* which tcxhost do we use? */

int useGrabber;			/* which grabber do we use? */

extern int display;		/* display images at all? */

extern int tcx;			/* use tcx */

extern int color;		/* display images in color */

char *location_shmem[2];	/* shared memory segment  */

int shmid[2];			/* id of the shared memory segment */

char *pCameraImage[2];		/* camera image */

int saveframes[2];		/* frames to save (set by TCX message) */

int saving[2];			/* flag that will be set by a TCX message */

FILE *videofp[2];		/* the filehandle for saving frames */

int savefileopen[2];		/* state of the file */

char filename[2][128];		/* the name of the file, where we save the frames */

#if (!defined(sun) )
static int meteor[2];		/* handle for the framegrabber */
#endif

/* extern int errno; */		/* what is this for??? */

void gotframe( int signalnumber ); /* function prototype */

void commShutdown( );		/* when we shutdown we call this function */

int openAndReadFile( int numGrabber, char *filename );

int initMeteor( int numGrabber );

int startMeteor( int numGrabber, int how );

int stopMeteor( int numGrabber );

#if (!defined(sun) )

static struct meteor_geomet geo[2];

static struct sigaction sigact;

#endif

#endif

/* $Log: grab.h,v $
 * Revision 1.17  1998/02/08 00:13:14  swa
 * now works with redhat5.0
 *
 * Revision 1.16  1998/01/13 00:35:14  swa
 * Added two new functions -- start and stop. They can start and stop either
 * of the two frame grabbers.
 *
 * Revision 1.15  1997/11/06 17:54:38  swa
 * Fixed a bug with TCXHOST. If the environment variable TCXHOST is set,
 * take it. If not, take the TCXHOST from beeSoft.ini. If this doesn't
 * work, take localhost. If this doesn't work: bummer.
 *
 * Revision 1.14  1997/10/04 18:01:06  swa
 * Fixed a bug in CAMERA-messages so that sending images over TCX works again.
 *
 * Revision 1.13  1997/10/04 00:13:07  swa
 * First working version of the cameraServer that supports two (count'em
 * two!) framegrabbers.  Although the server seems to work just fine, it
 * has not yet been fully tested.
 *
 * Revision 1.12  1997/09/20 22:39:37  swa
 * added parameter and ini section to the documentation. The compiler flag NO_GRABBER
 * is not longer necessary, since we get that info now from beeSoft.ini.
 *
 * Revision 1.11  1997/09/20 00:47:42  swa
 * You can now use the cameraServer on a Linux box without a
 * framegrabber. This no longer requires recompiling the stuff but is set
 * in an entry in beeSoft.ini. Speaking of beeSoft.ini: the devices and
 * other stuff is not hardcoded anymore but read from the .ini file.
 *
 * Revision 1.10  1997/09/19 23:03:18  swa
 * Image-over-TCX works now correctly when using file as the source. A bunch
 * of function calls were missing in the load-file function.
 *
 * Revision 1.9  1997/08/02 18:50:54  swa
 * Support for file-only mode (SUN and Linux) added.
 *
 * Revision 1.8  1997/07/24 00:54:24  swa
 * Fixed some minor bugs, extended the README file and tested the version. The
 * cameraAttachExample does not yet die when the cameraServer dies. Will be
 * fixed in future versions.
 *
 * Revision 1.7  1997/07/23 22:43:33  swa
 * This is the first version of camControl, a Tcl program that remotely controls
 * the cameraServer. The user can save and read frames. recorder was renamed
 * to camControl, since the program will not only record but also read frames.
 *
 * Revision 1.6  1997/07/22 22:47:16  swa
 * Added file handling (reading/saving). Renamed two (three?) functions to
 * be consistent.
 *
 * Revision 1.5  1997/06/30 00:14:33  thrun
 * This new version allows saving of images (in .ppm 4-(four!)-byte format), by
 * left-clicking in cameraAttachExample's window . The cameraServer can then
 * later read that image and display it (instead of using the framegrabber). (swa)
 *
 * Revision 1.4  1997/06/24 22:48:13  thrun
 * checking of the command line arguments.
 *
 * Revision 1.3  1997/06/23 23:48:43  thrun
 * shared memory and other stuff (swa)
 *
 * Revision 1.2  1997/06/23 02:36:00  thrun
 * Stefan's new camera server with shared memory
 *
 * Revision 1.1.1.1  1997/06/16 22:32:20  thrun
 * New Camera Server, much faster, more reliable, works already with
 * commander.
 *
 */
