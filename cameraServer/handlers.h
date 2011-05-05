
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
void CAMERA_close_handler(char *name, TCX_MODULE_PTR module);

#ifdef SEBASTIAN
void connect_to_Base(void);
void connect_to_Pantilt(void);
void tcx_pantilt_init();
void tcx_base_subscribe();
#endif

int CAMERA_initialize_tcx();

void send_automatic_image_update( int numGrabber, char *pCameraImage );

int CAMERA_initialize_tcx();

void CAMERA_register_auto_update_handler( TCX_REF_PTR                     ref,
					  CAMERA_register_auto_update_ptr data);

/* ----------------------------------------------------------------------- */

void CAMERA_targetCoord_query_handler( TCX_REF_PTR ref );

void CAMERA_send_targetCoord_to( TCX_MODULE_PTR module );

/* ----------------------------------------------------------------------- */

void CAMERA_image_query_handler( TCX_REF_PTR            ref,
				 CAMERA_image_query_ptr data );

void CAMERA_shmid_query_handler( TCX_REF_PTR             ref,
				 CAMERA_shmid_query_ptr  data);

void CAMERA_send_camera_image_to( TCX_MODULE_PTR module,
				  int numGrabber,
				  char *pCameraImage,
				  int image_xsize,
				  int image_ysize );

/* ----------------------------------------------------------------------- */

void check_commandline_parameters(int argc, char **argv);

/* ----------------------------------------------------------------------- */

void CAMERA_save_handler( TCX_REF_PTR     ref,
			  CAMERA_save_ptr data );

void CAMERA_load_handler( TCX_REF_PTR     ref,
			  CAMERA_load_ptr data );

void CAMERA_startstop_handler( TCX_REF_PTR          ref,
			       CAMERA_startstop_ptr data );

/* ----------------------------------------------------------------------- */

/*
 * $Log: handlers.h,v $
 * Revision 1.12  1998/01/13 00:35:14  swa
 * Added two new functions -- start and stop. They can start and stop either
 * of the two frame grabbers.
 *
 * Revision 1.11  1997/10/04 00:13:08  swa
 * First working version of the cameraServer that supports two (count'em
 * two!) framegrabbers.  Although the server seems to work just fine, it
 * has not yet been fully tested.
 *
 * Revision 1.10  1997/09/19 23:03:18  swa
 * Image-over-TCX works now correctly when using file as the source. A bunch
 * of function calls were missing in the load-file function.
 *
 * Revision 1.9  1997/07/24 18:48:01  swa
 * Renamed two internal functions. Tested it again with the Tcl program. Works
 * fine. :)
 *
 * Revision 1.8  1997/07/22 22:47:16  swa
 * Added file handling (reading/saving). Renamed two (three?) functions to
 * be consistent.
 *
 * Revision 1.7  1997/06/25 18:46:50  thrun
 * Added support for tcx messages. It is for mainatt no longer necessary to
 * use argv[1]. It now sends out messages. (swa)
 *
 * Revision 1.6  1997/06/24 22:48:14  thrun
 * checking of the command line arguments.
 *
 * Revision 1.5  1997/06/23 23:48:44  thrun
 * shared memory and other stuff (swa)
 *
 * Revision 1.4  1997/06/20 01:09:29  thrun
 * Renamed function names to be more consistent. Currently the cameraServer
 * can track various object based on its color information. The color info
 * is supplied in a seperate textfile and is loaded into the server upon the
 * first request. (swa)
 *
 * Revision 1.3  1997/06/19 21:06:58  thrun
 * Added various stuff for faceInfo (callbacks, etc)
 *
 * Revision 1.2  1997/06/18 15:58:14  thrun
 * now with auto-update
 *
 * Revision 1.1.1.1  1997/06/16 22:32:20  thrun
 * New Camera Server, much faster, more reliable, works already with
 * commander.
 *
 */
