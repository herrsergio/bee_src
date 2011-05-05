
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
#ifndef DISPLAY_H_INCLUDED
#define DISPLAY_H_INCLUDED

typedef struct iPoint {
  int x;
  int y;
} iPoint, *iPointPtr;

typedef struct iRect {
  iPoint p1;
  iPoint p2;
} iRect, *iRectPtr;

#define D_PANEL                 1
#define D_CAMERA                2
#define D_FACEGAUSSIAN          4
#define D_GREEN                 8
#define D_RED                  16
#define D_FACEGAUSSIANTOTAL    32
#define D_SHIRTGAUSSIAN        64
#define D_SHIRTGAUSSIANTOTAL  128
#define D_MOTION              256
#define D_MISC1               512
#define D_UPDATE             1024
#define D_MISC2              2048
#define D_GRIDLOCK           4096
int displayWhat;


int CameraWindow[2];

/* max windows to be popped up simultaneously */
#define MAXWIN 9

/* maxrows and columns */
#define MAXROWS 500
#define MAXCOLS 500

EZXW_p w_id[MAXWIN];

int marked[MAXWIN][MAXROWS*MAXCOLS];

#define MINGREY    C_GREY0
#define MAXGREY    C_GREY100
#define GREYRANGE  ((MAXGREY)-(MINGREY)+1)

/* ---------------------------------------------------------------- */

int createDisplay( char *name, int x, int y, int dx, int dy );

int killDisplay( int w );

/* ---------------------------------------------------------------- */

int displayImage4Byte( int w, char *buffer, int bufferwidth, int bufferheight );

int displayImage4ByteColor( int w, char *buffer, int bufferwidth, int bufferheight );

int displayImage1Byte( int w, char *buffer, int bufferwidth, int bufferheight );

/* ---------------------------------------------------------------- */

int markRectangle( int w, int c, iRectPtr rect );

int markPoint( int w, int c, iPointPtr pp );

int markPoint2( int w, int c, int x, int y );

int makeGridLock( int w );

/* ---------------------------------------------------------------- */

int convertIntoInt( double *source, 
		    char   *target, 
		    int     reso, 
		    int     bufferwidth, 
		    int     bufferheight );

int sanityCheckWindow( iRectPtr pWindow, int rows, int cols );

/* int testColors( int w, char *image ); */

/* ---------------------------------------------------------------- */

int initRect( iRectPtr w );

int printRect( iRectPtr w );

int printPoint( iPointPtr p );

#endif

/*
 * $Log: display.h,v $
 * Revision 1.3  1997/10/04 00:13:06  swa
 * First working version of the cameraServer that supports two (count'em
 * two!) framegrabbers.  Although the server seems to work just fine, it
 * has not yet been fully tested.
 *
 * Revision 1.2  1997/06/23 02:35:59  thrun
 * Stefan's new camera server with shared memory
 *
 * Revision 1.1.1.1  1997/06/16 22:32:20  thrun
 * New Camera Server, much faster, more reliable, works already with
 * commander.
 *
 */
