
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



/* abterm.h */

#ifndef _ABTERM_H
#define _ABTERM_H

#include <sys/types.h>
#include <termios.h>
#include <curses.h>

typedef struct {
  char fName[10];
  int (*func)(int argc, char *argV[], long mspNum);
} funcList;

extern WINDOW  *devWin, *termWin, *dbgWin;

#define  DEVICE_TITLE   "DEVICE OUTPUT"
#define  TERM_TITLE     "TERMINAL"
#define  DBG_TITLE      "DEBUG OUTPUT"
#define  PROMPT         "{%04X}> "

#endif _ABTERM_H
