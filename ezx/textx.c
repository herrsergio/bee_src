
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






#ifdef VMS
#include "vms.h"
struct timeval {
        long    tv_sec;         /* seconds */
        long    tv_usec;        /* and microseconds */
};
#endif

#include "EZX11.h"

XFontStruct	*theFont = NULL;

/* Changes by Sebastian 94-6-10: If a font has not been freed, it will
 * be freed automatically before allocating memory for a new font. This
 * prevents a memory leak.
 */

void EZX_UseFont(GC theNewGC, char fontname[])
{
  char *tryFontName;
  char **useFontName = NULL;	/*  makes the compiler happy */
  int foundFont = 0;
  int tryingCourier = 0;

  if (theFont != NULL)
    XFreeFont(theDisplay, theFont);
  theFont = XLoadQueryFont(theDisplay, fontname);

  /* font name was not found, try to find one similar */
  /* Added 13 Oct 1994 - Tyson D Sawyer               */

  if (!theFont) {
    printf ("EZX_UseFont: Font not found - %s\n", fontname);
    tryFontName = malloc(256);
    strcpy (tryFontName+1, fontname);
    tryFontName[0] = '*';
    tryFontName[strlen(fontname)+1] = '*';
    tryFontName[strlen(fontname)+2] = 0;

    while (!foundFont) {
	printf ("EZX_UseFont: looking for font - %s\n", tryFontName);
	useFontName = XListFonts(theDisplay, tryFontName, 1, &foundFont);
	tryFontName[strlen(tryFontName)-2] = '*';
	tryFontName[strlen(tryFontName)-1] = 0;
	if (!tryingCourier && strlen(tryFontName)<5) {
	    strcpy (tryFontName, "*courier*");
	    tryingCourier = 1;
	}
    }
    printf ("EZX_UseFont: Using font - %s\n", useFontName[0]);
    theFont = XLoadQueryFont(theDisplay, useFontName[0]);
    XFreeFontNames(useFontName);
    free (tryFontName);
    if (!theFont) printf("EZX_UseFont: failed - theFont=NULL\n");
  }
  
  if (theFont != 0)
    XSetFont(theDisplay, theNewGC, theFont->fid);
}

void EZX_FreeFont(void)
{
   XFreeFont(theDisplay, theFont);
   theFont = NULL;
}




void EZX_DrawString(EZXW_p w, int x, int y, char *string)
{
   XDrawString(theDisplay, w->w, theGC, x, y, string, strlen(string));
}


void EZX_DrawText(EZXW_p w, int x, int y, char *string)
{
   XDrawImageString(theDisplay, w->w, theGC, x, y, string, strlen(string));
}


static void format_spec(int x, int y, char string[], char style, 
			int *newx, int *newy, int *width)
{
   *width = XTextWidth(theFont, string, strlen(string));
   *newx  = x;
   *newy  = y;
   switch( style ) {
        case 'l':	/* Left Alignment */
	case 'L':
	    break;
	case 'R':	/* Right Alignment */
	case 'r':
/*	    *newx = x + WSizeHints.width - *width;  */
	    *newx = x - *width;
	    break;
	case 'c':	/* Centered */ 
	case 'C':
/*	    *newx = x + (WSizeHints.width - *width) / 2; */
	    *newx = x - *width / 2;
/*	    *newy = y - EZX_GetFontHeight()/2; */
	    break;
	default:	/* use left alignment as default */
	    format_spec(x,y, string, 'L', newx,newy,width);
	 }
}


void EZX_DrawTextAt(EZXW_p w, int x, int y, char *string, char style)
{
   int	width,newx,newy;

   format_spec(x,y, string, style, &newx, &newy, &width);
   EZX_DrawText(w, newx, newy, string );
}


void EZX_DrawStringAt(EZXW_p w, int x, int y, char *string, char style)
{
   int	width,newx,newy;

   format_spec(x,y, string, style, &newx, &newy, &width);
   EZX_DrawString(w, newx, newy, string );
}


void EZX_FormatAt(EZXW_p w, int x, int y, char *string, char style,
		  int background_filled, int underlied)
{
   int	width,newx,newy;

   format_spec(x,y, string, style, &newx, &newy, &width);
   if( background_filled )
      EZX_DrawText(w, newx, newy, string );
   else
      EZX_DrawString(w, newx, newy, string );

   if( underlied )
      EZX_DrawLine(w, newx, newy+2, x+width, newy+2 );
}


int  EZX_GetFontHeight(void)
{
    if (theFont) return( theFont->ascent + theFont->descent );
    else return (0);
}


int EZX_GetTextWidth(char string[])
{
   if (theFont) return( XTextWidth(theFont, string, strlen(string)) );
   else return (0);
}

int EZX_GetCharWidth ( int which )
{
  if ( which >= (int)theFont->min_char_or_byte2 )
    return theFont->per_char[which-theFont->min_char_or_byte2].width;
  
  return 0;
}
