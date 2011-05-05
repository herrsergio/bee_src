
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
#include <Common.h>
#include <EZX11.h>
#include "display.h"
#include "misc.h"

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int createDisplay(char *name, int x, int y, int dx, int dy) {

  static BOOLEAN initialized = FALSE;
  char string[40];
  int w;
  int i;
  
  if ( !initialized ) {

    EZX_InitX( NULL, "EZX" );
    for (w=0; w<MAXWIN; w++)
      w_id[w] = NULL;
    initialized = TRUE;
  }
  
  for (w=0; w<MAXWIN; w++)

    if ( w_id[w] == NULL ) {

      sprintf(string,"+%d+%d", dx, dy);
      w_id[w] = EZX_MakeWindow( name, x, y, string );
      for (i=0; i<x*y; i++) 
	marked[w][i]=0;

      return w;

    }
  
  return -1;
}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int killDisplay( int w ) {

  if ( w_id[w] != NULL  ) {
    EZX_EndWindow( w_id[w] );
    free( w_id[w] );
  } else {
  }

  /* EZX_Flush(); */

  return 0;
}

/* ---------------------------------------------------------
 *
 * allocates and returns a chunk of memory
 *
 * --------------------------------------------------------*/
void *getmem(size_t bytes) {
  void *ptr;

  if ((ptr = malloc(bytes)) == (void *) NULL)
    MISCERROR;

  return ptr;
}

/* -------------------------------------------------------------------------

   Display the contents of a memory buffer, "buf", presumably
   containing an image.  

   ------------------------------------------------------------------------- */
int displayImage4ByteColor( int w, char *buffer, int bufferwidth, int bufferheight ) {

  char *ximage_buf;
  XImage *ximage;
  int i, j;
  double red, green, blue;
  unsigned long pixel_value;

  if ( w_id[w] == NULL || buffer == NULL )
    return -1;

  ximage_buf = getmem( w_id[w]->width * w_id[w]->height );

  if ( ximage_buf == NULL ) {
    NULLERR;
    return -1;
  }

  ximage = XCreateImage( theDisplay,
			 theVisual,
			 theDepth,
			 ZPixmap,
			 0,
			 ximage_buf,
			 w_id[w] -> width,
			 w_id[w] -> height,
			 32,
			 w_id[w] -> width );

  if ( ximage == NULL ) {
    MISCERROR;
    return -1;
  }

  XInitImage( ximage );

  for ( i = 0; i < bufferheight; i++)
    for ( j = 0; j < bufferwidth; j++) {
      red = buffer[4 * (j + i * bufferwidth) + 2] / 255.0;
      green =  buffer[4 * (j + i * bufferwidth) + 1] / 255.0;
      blue = buffer[4 * (j + i * bufferwidth)] / 255.0;
      pixel_value = theBestMapInfo->base_pixel +
	((unsigned long) ((0.5 + red * theBestMapInfo->red_max)) *
	 theBestMapInfo->red_mult) +
	((unsigned long) ((0.5 + green * theBestMapInfo->green_max)) *
	 theBestMapInfo->green_mult) +
	((unsigned long) ((0.5 + blue * theBestMapInfo->blue_max)) *
	 theBestMapInfo->blue_mult);
      XPutPixel( ximage, j, i, pixel_value );
    }

  XPutImage( theDisplay, w_id[w]->w, 
	     theGC, ximage, 
	     0, 0, 
	     0, 0, 
	     w_id[w]->width,
	     w_id[w]->height );

  XDestroyImage(ximage);

  EZX_Flush();

  return 0;

}

/* ---------------------------------------------------------
 *
 * Displays buffer in the window with window_id w. if buffer-
 * height or -width is larger than the window, the image gets
 * clipped. If it is smaller it will be padded by black pixels.
 *
 * --------------------------------------------------------*/
int displayImage4Byte( int w, char *buffer, int bufferwidth, int bufferheight ) {

  int     slookup, tlookup, mlookup, color, x, y, xmax, ymax;
  XImage *ximage = NULL;
  char   *ximage_buf = NULL;

  if ( buffer == NULL || w_id[w] == NULL ) {
    NULLERR;
    return -1;
  }

  ximage_buf = getmem( w_id[w]->width * w_id[w]->height );

  if ( ximage_buf == NULL ) {
    NULLERR;
    return -1;
  }

  XSetForeground(theDisplay, theGC,theBlackPixel);
  XSetBackground(theDisplay, theGC,theWhitePixel);

  /* -------- determine max height and width -------------- */
  if ( bufferwidth < w_id[w]->width )
    xmax = bufferwidth;
  else
    xmax = w_id[w]->width;

  if ( bufferheight < w_id[w]->height )
    ymax = bufferheight;
  else
    ymax = w_id[w]->height;

  /* ----------- and copy the buffer ---------------------- */
  for ( y=0; y<ymax; y++ ) {

    slookup = y * bufferwidth * 4;
    mlookup = y * bufferwidth;
    tlookup = y * w_id[w]->width;

    for ( x=0; x<xmax; x++ ) {

      if ( marked[w][mlookup+x]!=0 ) {

	ximage_buf[tlookup+x]  = thePixels[marked[w][mlookup+x]];
	marked[w][mlookup+x] = 0;
	
      } else {
	
	color = ( ((unsigned char)buffer[slookup+x*4+0]+
		   (unsigned char)buffer[slookup+x*4+1]+
		   (unsigned char)buffer[slookup+x*4+2]) * 0.026 ) + MINGREY;
	
	if ( theDepth==1 && color != C_BLACK ) 
	  color = C_WHITE;

	ximage_buf[tlookup+x] = thePixels[color];

      }

    }

  }

  /* ----------- and pad with black pixels ---------------- */
  for ( y=0; y<w_id[w]->height; y++ ) {
    tlookup = y*w_id[w]->width;
    for ( x=xmax; x<w_id[w]->width; x++ )
      ximage_buf[tlookup+x] = thePixels[ C_BLACK ];
  }
  /* ----------- and pad with black pixels ---------------- */
  for ( y=ymax; y<w_id[w]->height; y++ ) {
    tlookup = y*w_id[w]->width;
    for ( x=0; x<w_id[w]->width; x++ )
      ximage_buf[tlookup+x] = thePixels[ C_BLACK ];
  }

  ximage = XCreateImage( theDisplay, DefaultVisual(theDisplay, theScreen),
			 theDepth, ZPixmap,
			 0, 
			 ximage_buf,
			 w_id[w]->width, 
			 w_id[w]->height, 
			 8, 
			 w_id[w]->width );

  if ( ximage==NULL) {
    NULLERR;
    return -1;
  }

  ximage->byte_order = XImageByteOrder( theDisplay );

  XPutImage( theDisplay, w_id[w]->w, theGC, ximage, 
	     0, 0, 
	     0, 0, 
	     w_id[w]->width,
	     w_id[w]->height );

  free( ximage_buf );

  ximage->data = NULL;

  XDestroyImage( ximage );

  EZX_Flush();

  return 0;

}

/* ---------------------------------------------------------
 *
 * Displays buffer in the window with window_id w. if buffer-
 * height or -width is larger than the window, the image gets
 * clipped. If it is smaller it will be padded by black pixels.
 *
 * --------------------------------------------------------*/
int displayImage1Byte( int w, char *buffer, int bufferwidth, int bufferheight ) {

  int     slookup, tlookup, color, x, y, xmax, ymax;
  XImage *ximage = NULL;
  char   *ximage_buf = NULL;

  if ( buffer == NULL || w_id[w] == NULL ) {
    NULLERR;
    return -1;
  }

  ximage_buf = getmem( w_id[w]->width * w_id[w]->height );

  if ( ximage_buf == NULL ) {
    NULLERR;
    return -1;
  }
  
  XSetForeground( theDisplay, theGC,theBlackPixel );
  XSetBackground( theDisplay, theGC,theWhitePixel );
  
  /* -------- determine max height and width -------------- */
  if ( bufferwidth < w_id[w]->width )
    xmax = bufferwidth;
  else
    xmax = w_id[w]->width;

  if ( bufferheight < w_id[w]->height )
    ymax = bufferheight;
  else
    ymax = w_id[w]->height;

  /* ----------- and copy the buffer ---------------------- */
  for ( y=0; y<ymax; y++ ) {

    slookup = y * bufferwidth;
    tlookup = y * w_id[w]->width;

    for ( x=0; x<xmax; x++ ) {

      if ( marked[w][slookup+x]!=0 ) {

	ximage_buf[tlookup+x] = thePixels[marked[w][slookup+x]];
	marked[w][slookup+x] = 0;

      } else {

	color = buffer[slookup+x] + MINGREY;
	if ( theDepth==1 && color != C_BLACK ) 
	  color = C_WHITE;

	ximage_buf[tlookup+x] = thePixels[color];
      }

    }

  }

  /* ----------- and pad with black pixels ---------------- */
  for ( y=0; y<w_id[w]->height; y++ ) {
    tlookup = y*w_id[w]->width;
    for ( x=xmax; x<w_id[w]->width; x++ )
      ximage_buf[tlookup+x] = thePixels[ C_BLACK ];
  }
  /* ----------- and pad with black pixels ---------------- */
  for ( y=ymax; y<w_id[w]->height; y++ ) {
    tlookup = y*w_id[w]->width;
    for ( x=0; x<w_id[w]->width; x++ )
      ximage_buf[tlookup+x] = thePixels[ C_BLACK ];
  }

  ximage = XCreateImage( theDisplay, DefaultVisual(theDisplay, theScreen),
			 theDepth, ZPixmap,
			 0, 
			 ximage_buf,
			 w_id[w]->width, 
			 w_id[w]->height, 
			 8, 
			 w_id[w]->width );

  /*
  fprintf( stderr,"w:%d,title:%s,id:%d,width:%d,height:%d\n",
	   w,
	   w_id[w]->title, 
	   w_id[w]->w, 
	   w_id[w]->width,
	   w_id[w]->height );
  
  fprintf( stderr,"theDisp:%d,DefVis:%d,theDe:%d,ZPix:%d,width:%d,height:%d\n",
	   theDisplay, DefaultVisual(theDisplay, theScreen),
	   theDepth, ZPixmap, w_id[w]->width, w_id[w]->height );

  fprintf( stderr, "--0--\n");
  */

  if ( ximage == NULL) {
    NULLERR;
    return -1;
  }

  ximage->byte_order = XImageByteOrder( theDisplay );
  
  XPutImage( theDisplay, w_id[w]->w, theGC, ximage,
	     0, 0,
	     0, 0,
	     w_id[w]->width,
	     w_id[w]->height );

  free( ximage_buf );

  ximage->data = NULL;

  XDestroyImage( ximage );

  EZX_Flush();

  return 0;
}


/* ---------------------------------------------------------
 *
 * Put a color c rectangle into window w
 *
 * --------------------------------------------------------*/
int markRectangle( int w, int cc, iRectPtr rect ) {

  int x,y;
  char c = (char)cc;

  if ( rect == NULL || w_id[w] == NULL ) {
    NULLERR;
    return -1;
  }
  
  if ( rect->p1.x < 0 || rect->p1.x >= w_id[w]->width || 
       rect->p2.x < 0 || rect->p2.x >= w_id[w]->width ||
       rect->p1.y < 0 || rect->p1.y >= w_id[w]->height || 
       rect->p2.y < 0 || rect->p2.y >= w_id[w]->height ) {
    fprintf( stderr, "----------------------------------------------\n");
    fprintf( stderr, "bullshit in markrectangle:\n");
    fprintf( stderr, "(%d %d),(%d,%d)\n",rect->p1.x,rect->p1.y,rect->p2.x,rect->p2.y);
    return -1;
  }
  
  if ( 0 ) {
    fprintf( stderr, "(%d %d),(%d,%d)\n",rect->p1.x,rect->p1.y,rect->p2.x,rect->p2.y);
    fprintf( stderr, "w %d\n", w);
  }
  
  for (x=rect->p1.y*w_id[w]->width+rect->p1.x;
       x<=rect->p1.y*w_id[w]->width+rect->p2.x; x++ )
    marked[w][x]=c;

  for (x=rect->p2.y*w_id[w]->width+rect->p1.x;
       x<=rect->p2.y*w_id[w]->width+rect->p2.x; x++ )
    marked[w][x]=c;

  for (y=rect->p1.y*w_id[w]->width+rect->p1.x;
       y<=rect->p2.y*w_id[w]->width+rect->p1.x; y+=w_id[w]->width ) 
    marked[w][y]=c;

  for (y=rect->p1.y*w_id[w]->width+rect->p2.x;
       y<=rect->p2.y*w_id[w]->width+rect->p2.x; y+=w_id[w]->width ) 
    marked[w][y]=c;
  
  return 0;

}

/* ---------------------------------------------------------
 *
 * Put a color c point into window w
 *
 * --------------------------------------------------------*/
int markPoint( int w, int cc, iPointPtr pp ) {

  iRect rect;
  int x,y;
  char c = (char)cc;
  
  if ( pp == NULL || w_id[w] == NULL ) {
    NULLERR;
    return -1;
  }

  rect.p1.x = pp->x - 1;
  rect.p2.x = pp->x + 1;
  rect.p1.y = pp->y - 1;
  rect.p2.y = pp->y + 1;

  if (rect.p1.x<0) rect.p1.x = 0;
  if (rect.p1.x>w_id[w]->width-1) rect.p1.x = w_id[w]->width-1;
  if (rect.p2.x<0) rect.p2.x = 0;
  if (rect.p2.x>w_id[w]->width-1) rect.p2.x = w_id[w]->width-1;
  
  if (rect.p1.y<0) rect.p1.y = 0;
  if (rect.p1.y>w_id[w]->height-1) rect.p1.y = w_id[w]->height-1;
  if (rect.p2.y<0) rect.p2.y = 0;
  if (rect.p2.y>w_id[w]->height-1) rect.p2.y = w_id[w]->height-1;
  
  if ( rect.p1.x < 0 || rect.p1.x >= w_id[w]->width ||
       rect.p2.x < 0 || rect.p2.x >= w_id[w]->width ||
       rect.p1.y < 0 || rect.p1.y >= w_id[w]->height ||
       rect.p2.y < 0 || rect.p2.y >= w_id[w]->height ) {
    fprintf( stderr, "----------------------------------------------\n");
    fprintf( stderr, "bullshit in markpoint :\n");
    fprintf( stderr, "(%d %d),(%d,%d)\n",rect.p1.x,rect.p1.y,rect.p2.x,rect.p2.y);
    return -1;
  }
  
  for (x=rect.p1.y*w_id[w]->width+rect.p1.x; x<=rect.p1.y*w_id[w]->width+rect.p2.x; x++ ) 
    marked[w][x]=c;

  for (x=rect.p2.y*w_id[w]->width+rect.p1.x; x<=rect.p2.y*w_id[w]->width+rect.p2.x; x++ ) 
    marked[w][x]=c;

  for (y=rect.p1.y*w_id[w]->width+rect.p1.x; y<=rect.p2.y*w_id[w]->width+rect.p1.x; y+=w_id[w]->width )
    marked[w][y]=c;

  for (y=rect.p1.y*w_id[w]->width+rect.p2.x; y<=rect.p2.y*w_id[w]->width+rect.p2.x; y+=w_id[w]->width )
    marked[w][y]=c;

  return 0;
}

/* ---------------------------------------------------------
 *
 * Put a color c point into window w
 *
 * --------------------------------------------------------*/
int markPoint2( int w, int cc, int x, int y ) {
  iRect rect;
  char c = (char)cc;

  if ( w_id[w] == NULL ) {
    NULLERR;
    return -1;
  }

  rect.p1.x = x - 1;
  rect.p2.x = x + 1;
  rect.p1.y = y - 1;
  rect.p2.y = y + 1;

  if (rect.p1.x<0) rect.p1.x = 0;
  if (rect.p1.x>w_id[w]->width-1) rect.p1.x = w_id[w]->width-1;
  if (rect.p2.x<0) rect.p2.x = 0;
  if (rect.p2.x>w_id[w]->width-1) rect.p2.x = w_id[w]->width-1;

  if (rect.p1.y<0) rect.p1.y = 0;
  if (rect.p1.y>w_id[w]->height-1) rect.p1.y = w_id[w]->height-1;
  if (rect.p2.y<0) rect.p2.y = 0;
  if (rect.p2.y>w_id[w]->height-1) rect.p2.y = w_id[w]->height-1;

  if ( rect.p1.x < 0 || rect.p1.x >= w_id[w]->width ||
       rect.p2.x < 0 || rect.p2.x >= w_id[w]->width ||
       rect.p1.y < 0 || rect.p1.y >= w_id[w]->height ||
       rect.p2.y < 0 || rect.p2.y >= w_id[w]->height ) {
    fprintf( stderr, "----------------------------------------------\n");
    fprintf( stderr, "bullshit in markpoint :\n");
    fprintf( stderr, "(%d %d),(%d,%d)\n",rect.p1.x,rect.p1.y,rect.p2.x,rect.p2.y);
    return -1;
  }

  for (x=rect.p1.y*w_id[w]->width+rect.p1.x; x<=rect.p1.y*w_id[w]->width+rect.p2.x; x++ ) 
    marked[w][x]=c;

  for (x=rect.p2.y*w_id[w]->width+rect.p1.x; x<=rect.p2.y*w_id[w]->width+rect.p2.x; x++ )
    marked[w][x]=c;

  for (y=rect.p1.y*w_id[w]->width+rect.p1.x; y<=rect.p2.y*w_id[w]->width+rect.p1.x; y+=w_id[w]->width )
    marked[w][y]=c;

  for (y=rect.p1.y*w_id[w]->width+rect.p2.x; y<=rect.p2.y*w_id[w]->width+rect.p2.x; y+=w_id[w]->width )
    marked[w][y]=c;

  return 0;
}


/* ---------------------------------------------------------
 *
 * Put a color c gridlock into the window w
 *
 * --------------------------------------------------------*/
int makeGridLock( int w ) {

  int x,y;

  if ( w_id[w] == NULL ) {
    NULLERR;
    return -1;
  }

  y = (w_id[w]->height/2)*w_id[w]->width;
  for ( x=0; x<w_id[w]->width; x++)
    marked[w][y+x] = C_YELLOW;
  
  
  x = w_id[w]->width/2;
  for ( y=0; y<w_id[w]->height; y++)
    marked[w][y*w_id[w]->width+x] = C_YELLOW;

  return 0;

}

/* ---------------------------------------------------------
 *
 * Converts double source into char target (resolution reso)
 *
 * --------------------------------------------------------*/
int convertIntoInt( double *source, 
		    char   *target, 
		    int     reso, 
		    int     bufferwidth, 
		    int     bufferheight ) {
  int x,y,lookup;
  double max;
  double min;

  WHERE;

  if ( source == NULL || target == NULL ) {
    NULLERR;
    return -1;
  }
  
  max = source[0]; /*swa*/
  min = source[0]; /*swa*/
  
  /* ----------------- find maximum ---------------------- */
  for (y=0; y<bufferheight; y+=reso) {

    lookup = y*bufferwidth;

    for (x=0; x<bufferwidth; x+=reso) {

      if ( lookup+x < 0 || lookup+x > bufferheight*bufferwidth-1 ) {
	fprintf( stderr," bound error == %d (reso=%d,width=%d,height=%d)\n",
		 lookup+x, reso, bufferwidth, bufferheight );
	return -1;
      }
      if ( source[lookup+x] < 0.0 || source[lookup+x] > 1.0 ) {
	fprintf( stderr, "convertIntoInt ohh boy - another bug detected: %e\n",
		 source[lookup+x] );
	return -1;
      }

      if ( source[lookup+x] > max )
	max = source[lookup+x];

      if ( source[lookup+x] < min )
	min = source[lookup+x];

    }
  }

  /* ---------------- target black --------------------- */
  x = bufferheight*bufferwidth-1;
  do {
    target[x] = 0;
  } while (--x>=0 );

  /* ----------------- und rueberkopieren --------------- */
  for (y=0; y<bufferheight; y+=reso) {
    lookup=y*bufferwidth;
    for (x=0; x<bufferwidth; x+=reso) {
      target[lookup+x]= (char)( 
			       ((source[lookup+x]-min)/max) * (double)GREYRANGE
			       );
    }
  }

  return 0;
}

/* ---------------------------------------------------------
 *
 * truncates a window to fit in (0,0) (rows,cols)
 *
 * --------------------------------------------------------*/
int sanityCheckWindow( iRectPtr pWindow, int rows, int cols ) {

  if ( pWindow == NULL ) {
    NULLERR;
    return -1;
  }

  if ( pWindow->p1.x  < 0 )
    pWindow->p1.x = 0;
  
  if ( pWindow->p1.y  < 0 )
    pWindow->p1.y = 0;

  if ( pWindow->p1.x  >= cols )
    pWindow->p1.x = cols-1;
  
  if ( pWindow->p1.y  >= rows )
    pWindow->p1.y = rows-1;

  if ( pWindow->p2.x  < 0 )
    pWindow->p2.x = 0;
  
  if ( pWindow->p2.y  < 0 )
    pWindow->p2.y = 0;

  if ( pWindow->p2.x  >= cols )
    pWindow->p2.x = cols-1;
  
  if ( pWindow->p2.y  >= rows )
    pWindow->p2.y = rows-1;
  
  return 0;

}

/* ---------------------------------------------------------
 *
 *
 *
 * --------------------------------------------------------*/
/* int testColors( int w, char *image ) { */

/*   int i,x,y; */
/*   int color; */

/*   if ( image == NULL || w_id[w] == NULL ) { */
/*     NULLERR; */
/*     return -1; */
/*   } */

/*   color = 8; */

/*   for ( x=0; x<cols; x++ ) { */
    
/*     color++;  */

/*     if (color==30) */
/*       color=9; */

/*     for (y=0; y<rows; y++) */
/*         image[y*cols+x]=color; */


/*   } */

/*   displayImage1Byte( w, image, cols, rows ); */

/*   getchar(); */

/*   return 0; */
/* } */


/* ---------------------------------------------------------
 *
 *
 *
 * --------------------------------------------------------*/
int printRect( iRectPtr w ) {
  
  if ( w == NULL ) {
    NULLERR;
    return -1;
  }
   
  fprintf( stderr," ---------------------- \n" );
  fprintf( stderr,"(%d,%d) -- (%d,%d)\n", w->p1.x, w->p1.y, w->p2.x, w->p2.y );
  
  return 0;

}

/* ---------------------------------------------------------
 *
 *
 *
 * --------------------------------------------------------*/
int printPoint( iPointPtr p ) {

  if ( p == NULL ) {
    NULLERR;
    return -1;
  }
   
  fprintf( stderr," ---------------------- \n" );
  fprintf( stderr,"(%d,%d)\n", p->x, p->y );
  
  return 0;

}

/* ---------------------------------------------------------
 *
 *
 *
 * --------------------------------------------------------*/
int initRect( iRectPtr w ) {

  w->p1.x = 0;
  w->p1.y = 0;
  w->p2.x = 0;
  w->p2.y = 0;

  return 0;
}

/*
 * $Log: display.c,v $
 * Revision 1.4  1997/10/04 00:13:06  swa
 * First working version of the cameraServer that supports two (count'em
 * two!) framegrabbers.  Although the server seems to work just fine, it
 * has not yet been fully tested.
 *
 * Revision 1.3  1997/06/25 18:46:49  thrun
 * Added support for tcx messages. It is for mainatt no longer necessary to
 * use argv[1]. It now sends out messages. (swa)
 *
 * Revision 1.2  1997/06/23 02:35:59  thrun
 * Stefan's new camera server with shared memory
 *
 * Revision 1.1.1.1  1997/06/16 22:32:20  thrun
 * New Camera Server, much faster, more reliable, works already with
 * commander.
 *
 */
