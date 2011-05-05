
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
 *
 *  modified a little bit by hermann@holmium.informatik.uni-bonn.de
 *  modified another bit by trouvain@uran.informatik.uni-bonn.de
 */

#ifdef VMS
#include "vms.h"
struct timeval {
        long    tv_sec;         /* seconds */
        long    tv_usec;        /* and microseconds */
};
#endif

#include "EZX11.h"

#define  BORDER_WIDTH	2

#define  DEFAULT_GC     0
#define  PRIVATE_GC     1


GC		theGC,theDefaultGC;

XGCValues	theGCValues;

static int	theFirstWindow = True;
static int      whichGC = DEFAULT_GC;

Visual *theVisual;
XStandardColormap *theBestMapInfo;
unsigned long theEventMask;

static int      theWinGravity=NorthWestGravity;
static int      theBitGravity=NorthWestGravity;
static int      theBorderPixel=-1;
static int      theBackgroundPixel=-1;
static int      theBorderWidth=5;
static char *   theStandardIcon=0;
static int      theStandardIcon_width;
static int      theStandardIcon_height;
int      GetMotionEvent = 1;          /* gets motion events always */

int EZX_CreateGC(Window theNewWindow, GC *theNewGC);


static Window EZX_OpenWindow(char title[], int x, int y, 
			     int width, int height, int flag, 
			     GC *theNewGC, char *position)
/*
     int x,y;				 * window position.
					 * if negative, specified at runtime
					 *
     int width, height;			 * window size  *
     int flag;				 * if > 0, window is a pop-up *
     GC  *theNewGC;
     char *position;
*/
{
   XSetWindowAttributes	theWindowAttributes;
   XSizeHints		theSizeHints;
   unsigned long	theWindowMask;
   Window		theNewWindow;
   static XWMHints theWMHints = {	/* Let WM know how to handle window */
      (InputHint|StateHint),		/* flags */
      False,				/* input */
      NormalState,			/* initial_state */
      0,				/* icon pixmap */
      0,				/* icon window */
      0, 0,				/* icon location */
      0,				/* icon mask */
      0,				/* window group */
   };

   /* set up the attributes for the window.
    * Window manager (WM) may deny some of these resources.  
    * Setting override_redirect to True will tell WM to leave the window
    * alone.
    */
   theWindowAttributes.backing_store = Always;
   theWindowAttributes.backing_planes = 3;
   theWindowAttributes.event_mask = ButtonPressMask | ButtonReleaseMask | StructureNotifyMask | PointerMotionMask;
   if ( theBorderPixel == -1 )
     {
       theWindowAttributes.border_pixel  = theBlackPixel;
       theWindowAttributes.background_pixel = theWhitePixel;
     }
   else
     {
       theWindowAttributes.border_pixel  = theBorderPixel;
       theWindowAttributes.background_pixel = theBackgroundPixel;
     }

   /* CHANGE */
   theWindowAttributes.win_gravity = theWinGravity;
   theWindowAttributes.bit_gravity = theBitGravity;


   theWindowMask = CWBackingStore | CWEventMask |
      		   CWBackPixel | CWBorderPixel | 
		   CWBitGravity | CWWinGravity |
		   CWBackingPlanes | CWBackingPixel;

   if (flag > 0) {
      theWindowAttributes.override_redirect = True;
      theWindowAttributes.save_under	    = True;
      theWindowMask |= (CWSaveUnder |  CWOverrideRedirect);
   } else {
     theWindowAttributes.override_redirect = False;
     theWindowMask |= CWOverrideRedirect;
   }
   

   /* Open a window on the display */
   theNewWindow = XCreateWindow(theDisplay,
				XRootWindow(theDisplay, theScreen),
				x,y,
				width,height,
				theBorderWidth,
				theDepth,
				InputOutput,
				CopyFromParent,
				theWindowMask,
				&theWindowAttributes);

   /* Send "Hints" to WM, such as where the window should go. */
   XSetWMHints(theDisplay, theNewWindow, &theWMHints);

   /* Tell WM about the size and location for the window */

if (position)
   theSizeHints.flags	= (USPosition | USSize); /* User spec'd Size */
else
   theSizeHints.flags	= PPosition | PSize;

   theSizeHints.x	= x;
   theSizeHints.y	= y;
   theSizeHints.width	= width;
   theSizeHints.height	= height;

   if ( theStandardIcon == 0 )
     {
       XSetStandardProperties(theDisplay, theNewWindow,
			      title, 	/* window_name */
			      title,	/* icon_name */
			      None,		/* icon_pixmap */
			      0,		/* argv */
			      0,		/* argc */
			      &theSizeHints);
     }
   else
     {
       XSetStandardProperties(theDisplay, theNewWindow,
			      title, 	/* window_name */
			      title,	/* icon_name */
			      XCreateBitmapFromData(theDisplay,
						    theNewWindow,
						    theStandardIcon,
						    theStandardIcon_width,
						    theStandardIcon_height),
			      0,		/* argv */
			      0,		/* argc */
			      &theSizeHints);
     }

   /* Create a graphic context (GC) for the window, only for the first time */
   if (theFirstWindow) {
      if (EZX_CreateGC(theNewWindow, theNewGC) == 0) {
	 XDestroyWindow(theDisplay, theNewWindow);
	 return((Window)0);
      }
      theFirstWindow = False;
   }

   if ( GetMotionEvent == 1 )
     {
       theEventMask = KeyPressMask | ButtonPressMask | ButtonReleaseMask |
	 PointerMotionMask | Button1MotionMask | Button2MotionMask |
	   Button3MotionMask | Button4MotionMask | Button5MotionMask |
	     StructureNotifyMask | ExposureMask | EnterWindowMask;
     }
   else
     {
       theEventMask = KeyPressMask | ButtonPressMask | ButtonReleaseMask |
	 /* PointerMotionMask | Button1MotionMask | Button2MotionMask |
	   Button3MotionMask | Button4MotionMask | Button5MotionMask | */
	     StructureNotifyMask | ExposureMask | EnterWindowMask;
     }
   
   XSelectInput ( theDisplay, theNewWindow , theEventMask );
   
   /*
   theEventMask = KeyPressMask | ButtonPressMask | ButtonReleaseMask |
     PointerMotionMask | Button1MotionMask | Button2MotionMask |
       Button3MotionMask | Button4MotionMask | Button5MotionMask |
	 StructureNotifyMask | ExposureMask ;
   XSelectInput ( theDisplay, theNewWindow , theEventMask );
   */

/* Ask X to place the window visibly on the screen */
   XMapWindow(theDisplay, theNewWindow);
   XFlush(theDisplay);

/* Wait until X sends MapNotify for theNewWindow !! */
   {
   XEvent rep;
 
   while(1) {
      XNextEvent(theDisplay, &rep);
      if (rep.type == MapNotify)
        break;
      }
   }

   return(theNewWindow);
}


/* Returns 0 if error, 1 if Ok */

int EZX_CreateGC(Window theNewWindow, GC *theNewGC)
{
  *theNewGC = XCreateGC(theDisplay, theNewWindow, 
			(unsigned long)0, &theGCValues);
  if (*theNewGC == 0)			/* unable to create a GC */
    return(0);
  
  if ( theBackgroundPixel == -1 )
    {
       XSetForeground(theDisplay, *theNewGC, theBlackPixel);
       XSetBackground(theDisplay, *theNewGC, theWhitePixel);
     }
  else
    {
      XSetForeground(theDisplay, *theNewGC, theBorderPixel);
      XSetBackground(theDisplay, *theNewGC, theBackgroundPixel);
    }
  EZX_UseFont(*theNewGC, FONT_NAME);
  return 1;
}

EZXW_p EZX_MakeWindow(char *title, int width, int height, char *position)
{
  int left=-1, top=-1;
  char geometry[32];
  EZXW_p W;
  
  EZX_InitX( NULL, "EZX");
  W = (EZXW_p)malloc(sizeof(EZXW_t));
  strcpy(W->title, title);
  W->width  = width;
  W->height = height;
  sprintf(geometry, "=%dx%d", width, height);
  if (position) {
    strcat(geometry, position);
    XParseGeometry(geometry, &left, &top, &width, &height);
  } 
  W->w = EZX_OpenWindow(title,left,top,width,height, 
			0,		/* pop-up window ? */
			&theGC,position);
  W->gc=theGC;
  return(W);
}

void EZX_EndWindow(EZXW_p w)
{
    XFlush( theDisplay );
    XDestroyWindow( theDisplay, w->w );
    XFlush( theDisplay );
}


void EZX_ClearWindow(EZXW_p w)
{
    XClearWindow( theDisplay, w->w );
    XFlush( theDisplay );
}

void EZX_Flush(void)
{
   XFlush( theDisplay );
}

int EZX_LoadBestColorMap(EZXW_p w)
{
  int planes, i;
  XVisualInfo info;
  int count;
  unsigned long foreground_pixel, background_pixel;

  EZX_InitX(NULL, "Video");
  
  planes = DisplayPlanes(theDisplay, theScreen);
  if (XMatchVisualInfo(theDisplay, theScreen, planes, PseudoColor, &info) == 0)
    printf("PseudoColor graphics not supported!\n");
  theVisual = info.visual;
  
  if (theVisual != DefaultVisual(theDisplay, theScreen))
    printf("Error using color map!\n");
  
  /* Lifted from O'Reilly & Associates, vol 1, p 225 */
  if( XGetRGBColormaps(theDisplay, RootWindow(theDisplay,theScreen),
		       &theBestMapInfo, &count, XA_RGB_BEST_MAP) == 0 ){
    printf( "RGB_BEST_MAP colormap property not set.\n" );
    printf( "run `xstdcmap -best'\n" );
    return(0);
  } 
  else if( count != 1 ){
    printf( "count = %d\n", count );
    return(0);
  } 
  else if( theBestMapInfo->colormap ){
    /* created */
    if( theBestMapInfo->red_max == 0 ){
      printf( "RGB_BEST_MAP colormap property is set\n" );
      printf( "but is missing data.\n" );
      printf( "run `xstdcmap -best'\n" );
      return(0);
    } 
    else if( theBestMapInfo->visualid == 0 ){
      printf( "Standard colormap property is set\n" );
      printf( "but is missing data.\n" );
      printf( "run `xstdcmap -best'\n" );
      return(0);
    }
  }
  else {
    printf( "Got info, but the described colormap has not been created.\n" );
    return(0);
  }    
  
  if (w != NULL){
    XSetWindowColormap(theDisplay, w->w, theBestMapInfo->colormap);
    
    foreground_pixel = theBestMapInfo->base_pixel +
      (theBestMapInfo->red_max * theBestMapInfo->red_mult) +
	(theBestMapInfo->green_max * theBestMapInfo->green_mult) +
	  (theBestMapInfo->blue_max * theBestMapInfo->blue_mult);
    background_pixel = theBestMapInfo->base_pixel;
    XSetBackground(theDisplay, theGC,background_pixel);
    XSetForeground(theDisplay, theGC,foreground_pixel);
    EZX_Flush();
  }
  return (1);
}

/*
 * EZX_SetWinGravitiy 
 * accepts : ForgetGravity,StaticGravity,
 * NorthWestGravity,NorthGravity,NorthEastGravity,
 * WestGravity,CenterGravity,EastGravity,
 * SouthWestGravity,SouthGravity,SouthEastGravity
 */
void EZX_SetWinGravity ( int which_gravity )
{
  theWinGravity=which_gravity;
}
/*
 * EZX_SetBitGravity
 * accepts : same as EZX_SetWinGravity
 */
void EZX_SetBitGravity ( int which_gravity )
{
  theBitGravity=which_gravity;
}


void EZX_SetBorderWidth ( int BorderWidth )
{
  theBorderWidth=BorderWidth;
}

/*
 * EZX_ResizeWindow changes the size of the choosen window
 * mode : 0 => x and y are absolute
 *        1 => x and y are added to the current size
 *        2 => x and y are subtracted from the current size
 * BEWARE : Set correct Bitgravity, partial loss of graphics if 
 *          windowsize is reduced may be possible
 */
void EZX_ResizeWindow (EZXW_p W, int x, int y,int mode)
{
  XWindowAttributes win_attr;
  XSizeHints size_hints;

  long user_supplied_mask;

  /* Get original position and size */
  XGetWindowAttributes(theDisplay,W->w,&win_attr);
  
  /* Get size hints for the Window */
  XGetWMNormalHints(theDisplay,W->w, &size_hints, &user_supplied_mask);

  switch (mode)
    {
    case  0:
      XResizeWindow(theDisplay,W->w,x,y);
      break;
    case 1:
      XResizeWindow(theDisplay,W->w,win_attr.width+x,win_attr.height+y);    
      break;
    case 2:
      XResizeWindow(theDisplay,W->w,win_attr.width-x,win_attr.height-y);
      break;
    }
}

void EZX_MoveWindow (EZXW_p W,int x,int y,int mode)
{
  XWindowAttributes win_attr;
  XSizeHints size_hints;
  
  long user_supplied_mask;
  
  /* Get original position and size */
  XGetWindowAttributes(theDisplay,W->w,&win_attr);
  
  /* Get size hints for the Window */
  XGetWMNormalHints(theDisplay,W->w, &size_hints, &user_supplied_mask);
    
  switch (mode)
    {
    case 0:
      XMoveWindow (theDisplay,W->w,win_attr.x+x,win_attr.y+y);
      break;
    case 1:
      XMoveWindow (theDisplay,W->w,x,y);
      break;
    }

}

void EZX_LowerWindow (EZXW_p W)
{
  XLowerWindow (theDisplay,W->w);
}
void EZX_RaiseWindow (EZXW_p W)
{
  XRaiseWindow (theDisplay,W->w);
}

void EZX_SetGeneralWindowIcon (char *icon ,int width,int height)
{
  theStandardIcon=icon;
  theStandardIcon_width=width;
  theStandardIcon_height=height;
}

void EZX_SetWindowColor ( long foreground,long background )
{
  EZX_SetBackgroundPixel ( background );
  EZX_SetBorderPixel ( foreground );
}

void EZX_SetWindowBackground ( EZXW_p W,long color )
{
  XSetWindowBackground ( theDisplay,W->w,color );
}
void EZX_SetWindowBorder ( EZXW_p W, long color )
{
  XSetWindowBorder ( theDisplay,W->w,color );
}
void EZX_SetWindowBorderWidth ( EZXW_p W, unsigned int width )
{
  XSetWindowBorderWidth ( theDisplay,W->w,width );
}
void EZX_IconifyWindow ( EZXW_p W )
{
  XIconifyWindow ( theDisplay,W->w,theScreen );
}

void EZX_SetBorderPixel ( int  BorderPixel )
{
  theBorderPixel=EZX_RequestColor (BorderPixel);
}

void EZX_SetBackgroundPixel ( int BackgroundPixel )
{
  theBackgroundPixel=EZX_RequestColor (BackgroundPixel);
}


   

     


/* 
 * EZX_CopyArea
 */
void EZX_CopyArea ( EZXW_p src,EZXW_p dest,int src_x,int src_y,
		   unsigned int width,unsigned int height,
		   int dest_x,int dest_y )
{
  XCopyArea ( theDisplay,src->w,dest->w,theGC,src_x,src_y,width,height,
	     dest_x,dest_y );

}
   
/* Limits the output to the given rectangle of the window */
void EZX_ClipRectangle ( EZXW_p W,int x,int y,unsigned int width,
			unsigned int height)
{
  XRectangle XR;
  
  XR.x = x;
  XR.y = y;
  XR.width = width;
  XR.height = height;

  XSetClipRectangles ( theDisplay,theGC,0,0,&XR,1,0 );

  /*
  SetClipRectangles ordering 
    
  #define Unsorted		0
  #define YSorted		1
  #define YXSorted		2
  #define YXBanded		3
 */
}     

/* Limits the output to the full window */
void EZX_DisableClipping ( EZXW_p W )
{
  XWindowAttributes win_attr;
  XSizeHints size_hints;
  long user_supplied_mask;
  int width,height;


  /* Get original position and size */
  XGetWindowAttributes(theDisplay,W->w,&win_attr);
  
  /* Get size hints for the Window */
  XGetWMNormalHints(theDisplay,W->w, &size_hints, &user_supplied_mask);

  width = win_attr.width;
  height = win_attr.height;
  
  EZX_ClipRectangle ( W,0,0,width,height );
}

/* returns the default depth of the screen in pixels */
int EZX_GetScreenDepth ( void )
{
  return XDefaultDepth ( theDisplay , theScreen );
}

/* WARNING : The MotionEvent Flag has to be set before opening the window ! */
void EZX_NoMotionEvents ( void )
{
  GetMotionEvent = 0;
}
void EZX_AllMotionEvents ( void )
{
  GetMotionEvent = 1;
}
