
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
 *  cursorx.c
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

static XButtonEvent theButtonEvent;
static XEvent theEvent;
static XMotionEvent theMotionEvent;
static XKeyEvent theKeyEvent;

                                    /* For holding the button event */
extern GetMotionEvent;

static struct timeval Event_timeout;

void EZX_bell(void)
{
putc(7,stderr);
}

/* get cursor position when button pressed */

int EZX_GetCursor(int *xp, int *yp)
{
return EZX_GetCursorw(xp,yp,NULL);
}

int EZX_GetCursorw(int *xp, int *yp, Window *win)
{
    do {
      XNextEvent(theDisplay, &theEvent);
      theButtonEvent=theEvent.xbutton;
    } while (theButtonEvent.type != ButtonPress);

    if (win!=NULL)  *win  = theButtonEvent.window;
    *xp = theButtonEvent.x;
    *yp = theButtonEvent.y;

    if (theButtonEvent.button == Button1) return(LEFT_BUTTON);
    else if (theButtonEvent.button == Button2) return(MIDDLE_BUTTON);
    else if (theButtonEvent.button == Button3) return(RIGHT_BUTTON);
    else return(OTHER_BUTTON);
}





/* get cursor position when button pressed */
int EZX_TestGetCursorw(int *xp, int *yp, Window *win)
{
  *xp = -1;
  *yp = -1;

  if (XCheckMaskEvent(theDisplay, ButtonPressMask, &theEvent)){
    
    theButtonEvent=theEvent.xbutton;
    if (win != NULL)
      *win = theButtonEvent.window; /* this is the window the button
				     * was pressed in */
    *xp = theButtonEvent.x;
    *yp = theButtonEvent.y;
    
    if (theButtonEvent.button == Button1) return(LEFT_BUTTON);
    else if (theButtonEvent.button == Button2) return(MIDDLE_BUTTON);
    else if (theButtonEvent.button == Button3) return(RIGHT_BUTTON);
    else return(OTHER_BUTTON);
  }
  
  else if (XCheckTypedEvent(theDisplay, MotionNotify, &theEvent)){

    do{
      theMotionEvent=theEvent.xmotion;
      if (win != NULL)
	*win = theMotionEvent.window; /* this is the window pointer
				       * was moved in */
      *xp = theMotionEvent.x;
      *yp = theMotionEvent.y;
    }
    while(XCheckTypedEvent(theDisplay, MotionNotify, &theEvent));

    return -1;
  }

  else{				/* ...nothing found here */
    *xp = -1;
    *yp = -1;
    return -1;
  } 
}



/* get cursor position when button pressed */
int EZX_TestGetCursor(EZXW_p w, int *xp, int *yp)
{
  Window win;
  int ret_value;

  ret_value = EZX_TestGetCursorw(xp, yp, &win);
  if (w != NULL && win != w->w)
    *xp = *yp = ret_value = -1;
  return  ret_value;
}





/* set flag when button pressed */
int EZX_TestCursor(EZXW_p w)
{
      

    XCheckMaskEvent(theDisplay, ButtonPressMask, &theEvent);
					
    if (w==NULL)     
      return (theEvent.xbutton.type == ButtonPress);
    else
      return ((theEvent.xbutton.type == ButtonPress) &&
              (theEvent.xbutton.window==w->w));
}


/* get cursor positions when button pressed and released */
int EZX_GetCursors(int *xp, int *yp, int *xr, int *yr)
{
return EZX_GetCursorsw(xp, yp, xr, yr, NULL);
}

int EZX_GetCursorsw(int *xp, int *yp, int *xr, int *yr, Window *win)
{
    do {
      XNextEvent(theDisplay, &theEvent);
      theButtonEvent=theEvent.xbutton;
    } while (theButtonEvent.type != ButtonPress);

    if (win!=NULL) *win=theButtonEvent.window;
    *xp = theButtonEvent.x;
    *yp = theButtonEvent.y;

    do {
	XNextEvent(theDisplay, &theEvent);
	theButtonEvent=theEvent.xbutton;
     } while (theButtonEvent.type != ButtonRelease);

    *xr = theButtonEvent.x;
    *yr = theButtonEvent.y;

    if (theButtonEvent.button == Button1) return(LEFT_BUTTON);
    else if (theButtonEvent.button == Button2) return(MIDDLE_BUTTON);
    else if (theButtonEvent.button == Button3) return(RIGHT_BUTTON);
    else return(OTHER_BUTTON);
}


int EZX_block_and_wait(struct timeval *timeout)
{
  int stat, xfd;
  fd_set readMask;
  
  xfd = XConnectionNumber(theDisplay);
  
  FD_ZERO(&readMask);

  FD_SET(xfd,&readMask);
      
  stat = select(FD_SETSIZE, &readMask, NULL, NULL, timeout);

  if (stat > 0) 
    {
      if (FD_ISSET(xfd,&readMask))
	return 1;
      else
	return 2;
    }
  else return 0;
}


/**************************************************************************
 * EZX_GetEvent                            Boris AT Trouvain              *
 **************************************************************************/
int EZX_GetEvent ( EZX_EventPtr EZX_Event )
{
  Window root,child;
  unsigned int keys_buttons;
  int pos_x,pos_y,root_x,root_y;
  char buffer[20];
  int bufsize = 20;
  KeySym keysym;
  XComposeStatus compose;
  int charcount;
  
  if ( EZX_Event != NULL )
    {
      EZX_Event->type = EZX_Event_was_Nothing;

      if ( XCheckMaskEvent ( theDisplay,ButtonPressMask | ButtonReleaseMask
			    | KeyPressMask | PointerMotionHintMask |
			    Button1MotionMask | Button2MotionMask |
			    Button3MotionMask | Button4MotionMask | 
			    Button5MotionMask , &theEvent ) )
	{
	  
	  switch ( theEvent.type )
	    {
	    case KeyPress:
	      theKeyEvent = theEvent.xkey;
	      EZX_Event->type = EZX_Event_was_Key_Press;
	      EZX_Event->win = theKeyEvent.window;
	      EZX_Event->PointerX = theKeyEvent.x;
	      EZX_Event->PointerY = theKeyEvent.y;
	      
	      charcount = XLookupString ( &theKeyEvent,buffer,bufsize,&keysym,
					 &compose );
	      EZX_Event->Key = keysym;  /* hopefully an int */
	      /* There is yet no access to composed characters such as
		 ctrl-a alt-t and so on but this will be solved soon */
	      break;
	    case ButtonPress:
	      theButtonEvent = theEvent.xbutton;
	      EZX_Event->type = EZX_Event_was_Button_Press;
	      EZX_Event->win = theButtonEvent.window;
	      EZX_Event->PointerX = theButtonEvent.x;
	      EZX_Event->PointerY = theButtonEvent.y;
	      EZX_Event->Button = theButtonEvent.button;
	      break;
	    case ButtonRelease:
	      theButtonEvent = theEvent.xbutton;
	      EZX_Event->type = EZX_Event_was_Button_Release;
	      EZX_Event->win = theButtonEvent.window;
	      EZX_Event->PointerX = theButtonEvent.x;
	      EZX_Event->PointerY = theButtonEvent.y;
	      EZX_Event->Button = theButtonEvent.button;
	      break;
	    case MotionNotify:
	      theMotionEvent = theEvent.xmotion;
	      EZX_Event->type = EZX_Event_was_Motion;
	      EZX_Event->win = theMotionEvent.window;
	      EZX_Event->PointerX = theMotionEvent.x;
	      EZX_Event->PointerY = theMotionEvent.y;
	      if ( XQueryPointer ( theDisplay,theMotionEvent.window,
				  &root,&child,&root_x,&root_y,
				  &pos_x,&pos_y,&keys_buttons ))
		{
		  EZX_Event->type = EZX_Event_was_Nothing;
		  switch ( keys_buttons )
		    {
		    case 256: 
		      EZX_Event->Button = 1;
		      EZX_Event->type = EZX_Event_was_Motion;
		      break;
		    case 512:
		      EZX_Event->Button = 2;
		      EZX_Event->type = EZX_Event_was_Motion;
		      break;
		    case 1024:
		      EZX_Event->Button = 3;
		      EZX_Event->type = EZX_Event_was_Motion;
		      break;
		    }
		}
	      break;
	      
	    }
	  
	}
    }
    return 0;
}

/**************************************************************************
 * EZX_GetEventT                           Boris AT Trouvain              *
 **************************************************************************/
int EZX_GetEventT ( EZX_EventPtr EZX_Event )
{

  while ( EZX_block_and_wait ( NULL ) == 0 )
    {
    }
  
  EZX_GetEvent ( EZX_Event );
    
}

/**************************************************************************
 * EZX_GetEventT2                           Boris AT Trouvain             *
 **************************************************************************/
int EZX_GetEventT2 ( EZX_EventPtr EZX_Event , long microseconds )
{
  Event_timeout.tv_usec = microseconds;
  Event_timeout.tv_sec = 0;
  while ( EZX_block_and_wait ( &Event_timeout ) == 0 )
    {
    }
  
  EZX_GetEvent ( EZX_Event );
    
}

