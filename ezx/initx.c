
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

#ifdef VMS
#include "vms.h"
struct timeval {
        long    tv_sec;         /* seconds */
        long    tv_usec;        /* and microseconds */
};
#endif

#include "EZX11.h"

int	    	theScreen;		/* Screen number */
Display	       *theDisplay;		/* X server connection */
int	     	theDepth;		/* 1 if monochrome */
Colormap	theColormap;
unsigned char	theBlackPixel;
unsigned char	theWhitePixel;


static int 	theFirstTime = True;


void EZX_InitX(char *display, char *program)
{

   if (! theFirstTime) return;
   /* establish a connection to X server */
   if ( display == NULL )  display = getenv("DISPLAY");
   if ( (theDisplay = XOpenDisplay(display)) == NULL ) {
      fprintf(stderr, "\n%s: could not open display %s.\n",
	      program, XDisplayName(display));
      exit(1);
   }
   /* check for the default screen and color plane depth, etc */
   theScreen     = XDefaultScreen( theDisplay );
   theDepth      = XDefaultDepth( theDisplay, theScreen );
   theWhitePixel = XWhitePixel( theDisplay, theScreen );
   theBlackPixel = XBlackPixel( theDisplay, theScreen );
   theColormap   = XDefaultColormap( theDisplay, theScreen );

   EZX_InitDefaultColors();
   EZX_InitPatterns(8);
   theFirstTime = False;
}



void EZX_EndX(void)
{
   if (theFirstTime) return;
   XFlush( theDisplay );
   XCloseDisplay( theDisplay );
   theFirstTime = True;
}
