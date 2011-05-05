
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
 *  drawx.c
 *
 *  Created by Goang-Tay Hsu (gth).
 *  Modified by Long-Ji Lin (ljl) at Aug 1, 1990.
 *
 */     

#ifdef VMS
#include "vms.h"
struct timeval {
        long    tv_sec;         /* seconds */
        long    tv_usec;        /* and microseconds */
};
#endif

#include "EZX11.h"

static EZX_line_width = 1;
static EZX_line_style = LineSolid;
static EZX_fill_style = FillSolid;
static EZX_mode       = GXcopy;
static EZX_JoinStyle  = JoinBevel;
static EZX_CapStyle   = CapButt;
static EZX_CoordMode  = CoordModePrevious; /* or CoordModeOrigin */


void EZX_ClearRectangle(EZXW_p w, int left, int top,int  width, int height)
{
   XClearArea(theDisplay, w->w, left, top, width, height, False);
}


void EZX_DrawPoint(EZXW_p w, int x, int y)
{
   XDrawPoint(theDisplay, w->w, theGC, x, y);
}


void EZX_DrawPoints(EZXW_p w, int npoints, XPoint *points)
{
   XDrawPoints(theDisplay, w->w, theGC, points, npoints, EZX_CoordMode);
}


void EZX_DrawLine(EZXW_p w, int x1, int y1, int x2, int y2)
{
   XDrawLine(theDisplay, w->w, theGC, x1, y1, x2, y2);
}


void EZX_DrawLines(EZXW_p w, int npoints, XPoint *points)
{
  XDrawLines(theDisplay, w->w, theGC, points, npoints, EZX_CoordMode);
}


void EZX_DrawRectangle(EZXW_p w, int x, int y, 
		       unsigned int width, unsigned int height)
{
   XDrawRectangle( theDisplay, w->w, theGC, x, y, width, height );
}

/* EZX_DrawArc and EZX_FillArc use angle as 0 - 360 * 64
 * 12 o'clock = 90 , 3 o'clock = 0, 6 o'clock = 270, 9 o'clock = 180
 */ 
void EZX_DrawArc(EZXW_p w,int x ,int y, unsigned int width,unsigned int height,
		 int angle1, int angle2 )
{
  XDrawArc (theDisplay,w->w,theGC,x-width,y-height,width<<1,height<<1,
	    angle1,angle2);
}

void EZX_DrawArcPiece ( EZXW_p w, int x, int y, unsigned int width, 
		       unsigned int height, int size, int angle1,
		       int angle2 )
{
  int result_width,result_height;
  int line_width_buffer;
   
  result_width = width + size/2;
  result_height= height + size/2;

  if ( size < 0 )
    size = -size;

  
  line_width_buffer = EZX_GetLineWidth ();
  EZX_SetLineWidth ( size );
  
  XDrawArc (theDisplay,w->w,theGC,x-result_width,y-result_height,
	    result_width<<1,result_height<<1,angle1,angle2);
  EZX_SetLineWidth ( line_width_buffer );
}

void EZX_FillArc(EZXW_p w,int x ,int y, unsigned int width,unsigned int height,
		 int angle1, int angle2 )
{
  XFillArc (theDisplay,w->w,theGC,x-width,y-height,width<<1,height<<1,
	    angle1,angle2);
}

	  
void EZX_FillRectangle(EZXW_p w, int x, int y, 
		       unsigned int width, unsigned int height)
{
   XFillRectangle( theDisplay, w->w, theGC, x, y, width, height );
}


void EZX_DrawCircle(EZXW_p w, int x, int y, int r)
{
   XDrawArc( theDisplay, w->w, theGC, x - r, y - r, r<<1, r<<1, 0, 360*64);
}

void EZX_DrawEllipse ( EZXW_p w, int x, int y, int width, int height )
{
  XDrawArc ( theDisplay, w->w, theGC, x-width, y-height, width<<1,height<<1,
	    0,360*64 );
}


void EZX_FillCircle(EZXW_p w, int x, int y, int r)
{
    int  r2 = (int) (r  /* / 1.41421356 */ + 0.5); /*(st) */
    unsigned int wh = 2 * r2;

    XFillArc( theDisplay, w->w, theGC, x - r2, y - r2, wh, wh, 0, 360*64);
}

void EZX_DrawPolygon(EZXW_p w, int npoints, int x[], int y[])
{
  int x0, y0, i;
  
  x0 = x[npoints-1];
  y0 = y[npoints-1];
  
  for(i = 0; i < npoints; i++) {
    EZX_DrawLine(w, x0,y0, x[i], y[i]); 
    x0 = x[i];
    y0 = y[i];
  }
}

void EZX_FillPolygon(EZXW_p w, int npoints, XPoint *points)
{
  XFillPolygon(theDisplay, w->w, theGC, points, npoints, EZX_fill_style, EZX_CoordMode);
}

int EZX_SetLineWidth(int line_width)
{
  int save = EZX_line_width;
  theGCValues.line_width = EZX_line_width = line_width;
  XChangeGC(theDisplay, theGC, (GCLineWidth), &theGCValues);
  return save;
}

/* line_stype include:
 *   LineSolid, LineOnOffDash, LineDoubledash
 */

int EZX_SetLineStyle(int line_style)
{
   int save = EZX_line_style;
   theGCValues.line_style = EZX_line_style = line_style;
   XChangeGC(theDisplay, theGC, (GCLineStyle), &theGCValues);
   return save;
}


/* possible styles include:
 *   FillSolid, FillTiled, FillStippled, FillOpaqueStippled
 */

int EZX_SetFillStyle(int style)
{
   int save = EZX_fill_style;
   theGCValues.fill_style = EZX_fill_style = style;
   XChangeGC(theDisplay, theGC, (GCFillStyle), &theGCValues);
   return save;
}


/* possible modes include:
 *   GXcopy, GXor, GXxor
 */

int EZX_SetMode(int mode)
{
   int save = EZX_mode;
   theGCValues.function = EZX_mode = mode;
   XChangeGC(theDisplay, theGC, (GCFunction), &theGCValues);
   return save;
}


void EZX_SetDashes(unsigned char l, unsigned char h)
{
   unsigned char dotted[2] = { l,h };
   XSetDashes(theDisplay, theGC, 0, dotted, 2);
}

/* CoordModeOrigin,CoordModePrevious */
int EZX_SetCoordMode(int mode)
{
  int save = EZX_CoordMode;
  EZX_CoordMode = mode;
  return save;
}

/* JoinRound , JoinMiter , JoinBevel */
int EZX_SetJoinStyle ( int join )
{
  int save = EZX_JoinStyle;
  theGCValues.join_style = EZX_JoinStyle = join;
  XChangeGC(theDisplay, theGC, (GCJoinStyle), &theGCValues);
  return save;
}

/* CapNotLast , CapButt , CapRound , CapProjecting */
int EZX_SetCapStyle ( int cap )
{
  int save = EZX_CapStyle;
  theGCValues.cap_style = EZX_JoinStyle = cap;
  XChangeGC(theDisplay, theGC, (GCCapStyle), &theGCValues);
  return save;
}

int EZX_GetCoordMode(void)
{
  return EZX_CoordMode;
}  

int EZX_GetLineWidth(void)
{
   return EZX_line_width;
}

int EZX_GetLineStyle(void)
{
   return EZX_line_style;
}

int EZX_GetFillStyle(void)
{
   return EZX_fill_style;
}

int EZX_GetMode(void)
{
   return EZX_mode;
}

int EZX_GetJoinStyle ( void )
{
  return EZX_JoinStyle;
}

int EZX_GetCapStyle ( void )
{
  return EZX_CapStyle;
}


void EZX_Scroll (EZXW_p W,int x1,int y1,int width,int height,int xl,int yl,int bg)
{
  XImage *xi;
  int newHeight=height,newWidth=width,newSrcx=0,newSrcy=0,newx1=x1,newy1=y1;
  int hrx1=x1,hrwidth=width,hry1=0,hrheight=yl;
  int vrx1=x1,vrheight=height,vry1=y1,vrwidth=xl;

  xi=XGetImage (theDisplay,W->w,x1,y1,width,height,AllPlanes,XYPixmap);
  
  if ( xl >  0 )
    {
      vrx1=x1;
      vry1=y1;
      vrheight=height;
      vrwidth=xl;

      newWidth-=xl;
      newx1+=xl;
    }
  else
    {
      vrx1=x1+width+xl;
      vry1=y1;
      vrwidth=-xl;
      vrheight=height;
      
      newSrcx-=xl;
    }

  if ( yl > 0 )
    {
      hrx1=x1;
      hry1=y1;
      hrwidth=width;
      hrheight=yl;

      newHeight-=yl;
      newy1+=yl;
    }
  else
    {
      hrx1=x1;
      hry1=y1+height+yl;
      hrwidth=width;
      hrheight=-yl;

      newSrcy+=yl;
    }

  XPutImage (theDisplay,W->w,theGC,xi,newSrcx,newSrcy,newx1,newy1,newWidth,newHeight);

  EZX_SetFillStyle (FillSolid);
  EZX_SetColor (bg);
  if ( xl != 0 )
    EZX_FillRectangle ( W , vrx1,vry1,vrwidth,vrheight);
  if ( yl != 0 )
    EZX_FillRectangle ( W , hrx1,hry1,hrwidth,hrheight);

}


int EZX_GetPixel (EZXW_p W, int x, int y)
{
  XImage *xi;
  
  xi = XGetImage (theDisplay,W->w,x,y,1,1,AllPlanes,XYPixmap);
  return (XGetPixel ( xi,x,y ));
}







