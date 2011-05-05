
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
 *  EZX11.h - Header File for Multiple Window Graphics Interface to X11 (R3)
 *
 *  Created by gth.
 *  Modified by ljl (Aug 1, 1990).
 */

#ifndef EZX11_LOADED
#define EZX11_LOADED

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#ifndef __t800__

#ifdef VMS
#include <string.h>
#else
#include <strings.h>
#endif

#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#else
/* MINI-X definitions for usage with T800's */
typedef unsigned long XID;  
typedef XID Window;
typedef struct _XGC { void *ext_data; int gid; int rects; int dashes; 
		      unsigned long dirty; void * values; } *GC;
typedef struct { short x, y; } XPoint;
void *Display;
#endif

typedef struct {
   char title[64];
   int  width, height;
   Window w;
   GC gc;
#if defined(__t800__) || defined(__t800emu__)
   void *sunW_p;
#endif
}  EZXW_t, *EZXW_p;


typedef struct {
  XPoint start;		/* Starting Point Coordinate */
  XPoint *points;	/* Array of XPoints */
  int	 npoints;	/* No. of Points */
} BitMapPoints;

#define EZX_Event_was_Nothing          0
#define EZX_Event_was_Key_Press        1
#define EZX_Event_was_Button_Press     2
#define EZX_Event_was_Button_Release   3
#define EZX_Event_was_Motion           4

typedef struct {
  int type;
  int Key;
  int Button;
  int PointerX;
  int PointerY;
  long win;
} EZX_EventType,*EZX_EventPtr;



typedef XGCValues privateXGCValues;
typedef GC        privateGC;


/* #define FONT_NAME 	"fixed" */
/* #define FONT_NAME	"9x15" */
#define FONT_NAME	"9x15bold"

#define	LEFT_BUTTON	0	/* button detail */
#define MIDDLE_BUTTON	1
#define RIGHT_BUTTON	2
#define OTHER_BUTTON	3

#define NO_GREY_LEVELS	65
#define PATTERN_WIDTH	8
#define PATTERN_HEIGHT	8

#define PROMPT_AND_GET_RETURN(prompt) \
	XFlush( WDisplay); \
	printf("\n]]] Press Return to %s ...", prompt); \
	getchar();

/* typedef XRectangle : short x,y / unsigned short width ,height */

#ifndef __t800__
extern	int	    	theScreen;
extern	Display	       *theDisplay;
extern	int	     	theDepth;
extern	Colormap	theColormap;
extern	unsigned char	theBlackPixel;
extern	unsigned char	theWhitePixel;
extern	unsigned long	thePixels[];
extern  Visual *theVisual;
extern  XStandardColormap *theBestMapInfo;

extern	GC	     	theGC;
extern	XGCValues     	theGCValues;
extern	BitMapPoints 	theGrey[NO_GREY_LEVELS];
extern	XFontStruct    *theFont;
#endif

/* colors */


#define C_WHITE		0
#define C_BLACK		1
#define C_RED		2
#define C_LAWNGREEN	3
#define C_BLUE		4
#define C_YELLOW	5
#define C_GOLD		6
#define C_VIOLET	7
#define C_PINK		8
#define C_GREY          17
#define C_GREY0		9
#define C_GREY5         10
#define C_GREY10        11
#define C_GREY15        12
#define C_GREY20        13
#define C_GREY25        14
#define C_GREY30        15
#define C_GREY35        16
#define C_GREY40        17
#define C_GREY45        18
#define C_GREY50        19
#define C_GREY55        20
#define C_GREY60        21
#define C_GREY65        22
#define C_GREY70        23
#define C_GREY75        24
#define C_GREY80        25
#define C_GREY85        26
#define C_GREY90        27
#define C_GREY95        28
#define C_GREY100       29
#define C_MEDIUMVIOLETRED 30
#define C_MEDIUMPURPLE3   31
#define C_PALEGREEN4      32
#define C_CYAN            33
#define C_STEELBLUE4      34
#define C_ORANGERED4      35
#define C_KHAKI4          36
#define C_DARKTURQUOISE   37
#define C_FORESTGREEN     38
#define C_OLDLACE         39
#define C_LIGHTSLATEGREY  40
#define C_SLATEGREY       41
#define C_DARKSLATEGREY   42
#define C_PALETURQUOISE4  43
#define C_LIMEGREEN       44
#define C_CADETBLUE       45
#define C_DEEPPINK        46
#define C_MAGENTA2        47
#define C_SIENNA4         48
#define C_PINK1           49
#define C_TURQUOISE4      50
#define C_ROYALBLUE       51
#define C_YELLOW2         52
#define C_LIGHTVIOLET     53

#define MAXCOLORS	  256

#ifdef __cplusplus
extern "C"{
#endif

/* init routines */

void EZX_InitX		  (char *display, char *program);
void EZX_EndX		  (void);
void EZX_InitDefaultColors(void);
int  EZX_LoadBestColorMap(EZXW_p w);

/* window creation/destruction routines */


/* init routines */

void EZX_InitX		  (char *display, char *program);
void EZX_EndX		  (void);
void EZX_InitDefaultColors(void);
int  EZX_LoadBestColorMap(EZXW_p w);

/* window creation/destruction routines */

EZXW_p EZX_MakeWindow	(char *title, int width, int height, char *position);
void   EZX_EndWindow	(EZXW_p w);
void   EZX_SetGeneralWindowIcon (char *icon,int width,int height);
void   EZX_SetWindowBackground ( EZXW_p W , long  color );
void   EZX_SetWindowBorder ( EZXW_p W , long color );
void   EZX_SetWindowBorderWidth ( EZXW_p W , unsigned int width );
void   EZX_SetWinGravity  (int grav);
void   EZX_SetBitGravity  (int grav);
void   EZX_SetBorderPixel (int BorderPixel);
void   EZX_SetBackgroundPixel (int BackgroundPixel);
void   EZX_SetBorderWidth (int BorderWidth);
void   EZX_SetWindowColor ( long foreground,long background );
void   EZX_ResizeWindow (EZXW_p W, int x , int y , int mode );
void   EZX_MoveWindow (EZXW_p W, int x , int y , int mode);
void   EZX_LowerWindow (EZXW_p W);
void   EZX_RaiseWindow (EZXW_p W);
void   EZX_IconifyWindow ( EZXW_p W );
int    EZX_GetScreenDepth ( void );
void   EZX_NoMotionEvents ( void );
void   EZX_AllMotionEvents ( void );

/* cursor routines */

void EZX_bell         	(void);
int  EZX_GetCursor    	(int *xp, int *yp);
int  EZX_GetCursorw   	(int *xp, int *yp, Window *win);
int  EZX_TestGetCursor	(EZXW_p w, int *xp, int *yp);
int  EZX_TestGetCursorw (int *xp, int *yp, Window *win);
int  EZX_TestCursor   	(EZXW_p w);
int  EZX_GetCursors   	(int *xp, int *yp, int *xr, int *yr);
int  EZX_GetCursorsw  	(int *xp, int *yp, int *xr, int *yr, Window *win);
int  EZX_block_and_wait(struct timeval *timeout);
int EZX_GetEventT ( EZX_EventPtr EZX_Event );
int EZX_GetEventT2 ( EZX_EventPtr EZX_Event , long microseconds );
int EZX_GetEvent ( EZX_EventPtr EZX_Event );
/* basic drawing routines */

void EZX_ClearWindow   	(EZXW_p w);
void EZX_Flush         	(void);
void EZX_ClearRectangle	(EZXW_p w, int left, int top,int  width, int height);
void EZX_DrawPoint     	(EZXW_p w, int x, int y);
void EZX_DrawPoints    	(EZXW_p w, int npoints, XPoint *points);
void EZX_DrawLine      	(EZXW_p w, int x1, int y1, int x2, int y2);
void EZX_DrawLines     	(EZXW_p w, int npoints, XPoint *points);
void EZX_DrawRectangle 	(EZXW_p w, int x, int y, 
		    	unsigned int width, unsigned int height);
void EZX_FillRectangle 	(EZXW_p w, int x, int y, 
		   	 unsigned int width, unsigned int height);
void EZX_DrawCircle    	(EZXW_p w, int x, int y, int r);
void EZX_DrawEllipse    (EZXW_p w, int x, int y, int width, int height);
void EZX_FillCircle    	(EZXW_p w, int x, int y, int r);
void EZX_DrawString    	(EZXW_p w, int x, int y, char *string);
void EZX_DrawText      	(EZXW_p w, int x, int y, char *string);
void EZX_DrawArc        (EZXW_p w,int x ,int y, unsigned int width,unsigned int height,
		         int angle1, int angle2 );
void EZX_DrawArcPiece ( EZXW_p w, int x, int y, unsigned int width, 
		       unsigned int height, int size, int angle1,int angle2 );
void EZX_FillArc(EZXW_p w,int x ,int y, unsigned int width,unsigned int height,
		 int angle1, int angle2 );
/* Image routines */
void EZX_ScrollUp (EZXW_p w,int x1,int y1,int width,int height,int lines);
void EZX_TransImage (EZXW_p W1,EZXW_p W2,int x1,int y1,int width,int height);
void EZX_CopyArea ( EZXW_p src,EZXW_p dest,int src_x,int src_y,
		   unsigned int width,unsigned int height,
		   int dest_x,int dest_y );
void EZX_ClipRectangle ( EZXW_p W,int x,int y,unsigned int width,
			unsigned int height);
void EZX_DisableClipping ( EZXW_p W );

void EZX_Scroll (EZXW_p W,int x1,int y1,int width,int height,int xl,int yl,int bg);
int  EZX_GetPixel (EZXW_p W,int x, int y);

/* advanced drawing routines */

void EZX_DrawPolygon  	(EZXW_p w, int npoints, int x[], int y[]);
void EZX_FillPolygon	(EZXW_p w, int npoints, XPoint *points);
void EZX_DrawTextAt   	(EZXW_p w, int x, int y, char *string, char style);
void EZX_DrawStringAt 	(EZXW_p w, int x, int y, char *string, char style);
void EZX_FormatAt     	(EZXW_p w, int x, int y, char *string, char style,
		         int background_filled, int underlined);

void EZX_InitPatterns 	(int scale);
void EZX_CreatePattern	(unsigned int pattern[PATTERN_HEIGHT], 
		         BitMapPoints *mapPoints, int scale);
void EZX_DrawPattern  	(EZXW_p w, int x, int y, BitMapPoints *pattern);
void EZX_DrawGrey     	(EZXW_p w, int x, int y, int g);

/* GC changes --> return old settings */

int  EZX_SetColor	(int color);		/* set foreground color */
int  EZX_SetBackgroundColor ( int color );
int  EZX_RequestColor ( int color );
int  EZX_SetLineWidth	(int line_width);
int  EZX_SetLineStyle	(int line_style);
int  EZX_SetFillStyle	(int style);
int  EZX_SetMode	(int mode);
void EZX_SetDashes	(unsigned char l, unsigned char h);
int  EZX_SetCoordMode	(int mode);  /* CoordModePrevious or CoordModeOrigin */
int  EZX_SetJoinStyle   (int join);
int  EZX_SetCapStyle    (int cap);

/* GC info */

int  EZX_GetColor	(void);
int  EZX_GetLineWidth	(void);
int  EZX_GetLineStyle	(void);
int  EZX_GetFillStyle	(void);
int  EZX_GetMode	(void);
int  EZX_GetCoordMode	(void);
int  EZX_GetJoinStyle   (void);
int  EZX_GetCapStyle    (void);


/* misc */

void EZX_UseFont	(GC theNewGC, char fontname[]);
void EZX_FreeFont	(void);
int  EZX_GetFontHeight  (void);
int  EZX_GetTextWidth	(char string[]);
int  EZX_GetCharWidth   (int which);


#define EZX_SetToEraseMode()	EZX_SetColor(C_WHITE)
#define EZX_SetToDrawMode()	EZX_SetColor(C_BLACK)

#ifdef __cplusplus
}; /* extern "C" */
#endif

#endif
