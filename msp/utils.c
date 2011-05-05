
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



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <curses.h>

#include "abterm.h"
#include "msputils.h"

void
abtPrintTitles (void)
{
  int col;
  
  /* print the cool looking separators */
  standout();
  move (0, 0);
  for (col = 0; col != COLS; col++) addch (' ');
  move (LINES/2-3, 0);
  for (col = 0; col != COLS; col++) addch (' ');
  move (LINES/2, 0);
  for (col = 0; col != COLS; col++) addch (' ');
  standend();
  
  /* print titles */
  standout();
  move (0, 2);         addstr (TERM_TITLE);
  move (LINES/2-3, 2);   addstr (DBG_TITLE);
  move (LINES/2, 2);   addstr (DEVICE_TITLE);
  standend();
  
  /* display */
  refresh ();
}

void
abtCls (void)
{
  wprintw (termWin, "\n");	/* hack to clear remaining prompt */
  wprintw (devWin, "\n");
  wprintw (dbgWin, "\n");

  werase (stdscr);
  werase (devWin);
  werase (termWin);
  werase (dbgWin);
  clearok (stdscr, 1);
  
  wrefresh (stdscr);
  wrefresh (devWin);
  wrefresh (dbgWin);
  wrefresh (termWin);
  
  /* redraw seperators */
  abtPrintTitles ();
  
  return;
}

void
abtPrintMem (WINDOW *w, unsigned char *dat, int count)
{
  int ii;
  int i2_last = ii + 16;
  int i2;
    
  for (ii=0; ii<count; ii += 16) {
    /* start line with whitespace */
    wprintw (w, "   [");

    i2_last = ii + 16;		/* print in 16 byte chunks */

    /* print hex values */
    for (i2=ii; i2<count && i2<i2_last; i2++) {
      wprintw (w, "%02X", dat[i2]);
    }
    /* print extra space if necessary */
    for (; i2<i2_last; i2++) {
      wprintw (w, "  ");
    }
    
    /* end hex section and start char section*/
    wprintw (w, "]  [");

    /* print char values */
    for (i2=ii; i2<count && i2<i2_last; i2++) {
      wprintw (w, "%c", (isprint(dat[i2])?dat[i2]:'.'));
    }
    /* print extra space if necessary */
    for (; i2<i2_last; i2++) {
      wprintw (w, " ");
    }

    /* end char section and end line with newline */
    wprintw (w, "]\n");
  }

  wrefresh (w);
}

void
abtPrintPacket (WINDOW *w, ABMSG *msg)
{
  /* standard header */
  wprintw (w, "MAJ 0x%02X ", msg->hdr.major);
  wprintw (w, "MIN 0x%02X ", msg->hdr.minor);
  wprintw (w, "LEN 0x%02X ", msg->hdr.msgLen);
  wprintw (w, "DEV 0x%02X ", msg->hdr.devId);
  wprintw (w, "\n");

  /* data */
  abtPrintMem (w, &msg->data[4], msg->hdr.msgLen);
}
