
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







/*
 *  colorx.c	-   set up colors
 *  Created by Long-Ji Lin (ljl) at Aug 6, 1991.
 */

#ifdef VMS
#include "vms.h"
struct timeval {
        long    tv_sec;         /* seconds */
        long    tv_usec;        /* and microseconds */
};
#endif

#include "EZX11.h"
#include <stdio.h>
#include <stdlib.h>

static EZX_color = C_BLACK;

unsigned long	thePixels[MAXCOLORS];


static char	*theColorNames[] = {
   "white", "black", "red", "seagreen2", "blue", 
   "yellow", "gold", "violet", "pink",
   "grey0", "grey5",
   "grey10", "grey15", 
   "grey20", "grey25", 
   "grey30", "grey35", 
   "grey40", "grey45", 
   "grey50", "grey55", 
   "grey60", "grey65", 
   "grey70", "grey75", 
   "grey80", "grey85", 
   "grey90", "grey95", "grey100",
   "mediumvioletred", "mediumpurple3", "palegreen4", "cyan", "steelblue4",
   "orangered4", "khaki4", "darkturquoise", "forestgreen", "oldlace",
   "lightslategrey", "slategrey", "darkslategrey", "paleturquoise4",
   "limegreen", "cadetblue", "deeppink", "magenta2", "sienna4", 
   "pink1", "turquoise4", "royalblue", "yellow2", "#bdc1ff",

   /* 101 grey values for nicer maps */

   "grey0", "grey1", "grey2", "grey3", "grey4", "grey5", "grey6",
   "grey7", "grey8", "grey9", "grey10", "grey11", "grey12", "grey13",
   "grey14", "grey15", "grey16", "grey17", "grey18", "grey19", "grey20",
   "grey21", "grey22", "grey23", "grey24", "grey25", "grey26", "grey27",
   "grey28", "grey29", "grey30", "grey31", "grey32", "grey33", "grey34",
   "grey35", "grey36", "grey37", "grey38", "grey39", "grey40", "grey41",
   "grey42", "grey43", "grey44", "grey45", "grey46", "grey47", "grey48",
   "grey49", "grey50", "grey51", "grey52", "grey53", "grey54", "grey55",
   "grey56", "grey57", "grey58", "grey59", "grey60", "grey61", "grey62",
   "grey63", "grey64", "grey65", "grey66", "grey67", "grey68", "grey69",
   "grey70", "grey71", "grey72", "grey73", "grey74", "grey75", "grey76",
   "grey77", "grey78", "grey79", "grey79", "grey81", "grey82", "grey83",
   "grey84", "grey85", "grey86", "grey87", "grey88", "grey89", "grey90",
   "grey91", "grey92", "grey93", "grey94", "grey95", "grey96", "grey97",
   "grey98", "grey99", "grey100",


   "white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white","white","white","white","white","white","white",
   "white","white"  
   };


void EZX_InitDefaultColors(void)
{
   XColor	theRGBColor, theHardwareColor;
   int		theStatus;
   int		i;
   
   if (theDepth==1) {
      /* monochrome system */
      thePixels[C_WHITE] = theWhitePixel;
      thePixels[C_BLACK] = theBlackPixel;
   } else {
      for(i=0;i<MAXCOLORS;++i) {
	 theStatus = XLookupColor(theDisplay, theColormap, theColorNames[i],
				  &theRGBColor, &theHardwareColor);
	 if (theStatus != 0) {
	    theStatus = XAllocColor(theDisplay, theColormap, 
				    &theHardwareColor);
	    if (theStatus != 0)
	       thePixels[i] = theHardwareColor.pixel;
	    else
	       thePixels[i] = theBlackPixel;
	 }
      }
   }
}



int  EZX_SetColor(int color)			/* set foreground color */
{
   int save = EZX_color;
   if (color >= MAXCOLORS || color < 0) {
      printf("Wrong color: %d\n", color);
      return save;
   }
   XSetForeground(theDisplay, theGC, thePixels[EZX_color = color]);
   return save;
}


int EZX_RequestColor ( int color )
{
  return thePixels[color];
}


int EZX_SetBackgroundColor ( int color )
{
   int save = EZX_color;
   if (color >= MAXCOLORS || color < 0) {
      printf("Wrong color: %d\n", color);
      return save;
   }
  
   XSetBackground(theDisplay, theGC, thePixels[EZX_color = color]);
/*     fprintf (stderr,"\nHere is no .."); */
   return save;
}
  

int EZX_GetColor(void)
{
   return EZX_color;
}

