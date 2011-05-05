
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
#endif

#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <X11/Xlib.h>

#include "tcx.h"
#include "tcxP.h"
#include "global.h"
#include "EZX11.h"
#include "devUtils.h"


/* extern Display *theDisplay; */ /* get this from your EZX */

/* block till something happens
 * returns 1 if there is a waiting X11 event, 2 if there is a waiting TCX
 * event and 0 if there is no event 
 */
 

int block_wait(struct timeval *timeout, int tcx_initialized,
	       int X_initialized)
{
  extern void check_redraw_screen();
  XEvent ev;

/*  extern fd_set tcxConnectionListGlobal; */
  int stat, xfd = 0;
  fd_set readMask;
  
  if (tcx_initialized)
    readMask = (Global->tcxConnectionListGlobal);/* current tcx socket list */
  else
    FD_ZERO(&readMask);

  if (X_initialized){
    xfd = XConnectionNumber(theDisplay);
    FD_SET(xfd,&readMask);
  }
      
  stat = select(FD_SETSIZE, &readMask, NULL, NULL, timeout);

  if (stat > 0) 
    {
      if (!X_initialized)
	return 2;
      else if (FD_ISSET(xfd,&readMask)) {
        /* Do we need to grab the focus? */
        if (XCheckMaskEvent(theDisplay, EnterWindowMask, &ev)) {
          XSetInputFocus(theDisplay, ev.xcrossing.window, 
                         RevertToPointerRoot, CurrentTime);
        }

        /* Do we need to issue a redraw? */
        check_redraw_screen(theDisplay);

	return 1;
      }
      else
	return 2;
    }
  else return 0;
}


int block_wait_with_serial_ports(struct timeval *timeout, int tcx_initialized,
				 int X_initialized)
{
  int i;
  extern int maxDevNum;
  /*  extern fd_set tcxConnectionListGlobal; */
  int stat, xfd = 0;
  fd_set readMask;
  struct timeval TCX_waiting_time = {0, 0};

  if (tcx_initialized)
    readMask = (Global->tcxConnectionListGlobal);/* current tcx socket list */
  else
    FD_ZERO(&readMask);

  if (X_initialized){
    xfd = XConnectionNumber(theDisplay);
    FD_SET(xfd,&readMask);
  }

  for (i = 0; i < maxDevNum; i++) /* add all devices */
    if (devices[i] != NULL)
      FD_SET(i, &readMask);

  stat = select(FD_SETSIZE, &readMask, NULL, NULL, timeout);

  tcxRecvLoop((void *)(&TCX_waiting_time));

}
