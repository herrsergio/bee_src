
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



#ifndef _UTILS_H_
#define _UTILS_H_

#include <termios.h>
#include <curses.h>
#include <acb/global.h>

void abtPrintTitles (void);
void abtCls (void);
void abtPrintMem (WINDOW *w, unsigned char *dat, int count);
void abtPrintPacket (WINDOW *w, ABMSG *msg);

#ifndef  MAX
#define  MAX(a,b)  ((a)>(b)?(a):(b))
#endif
#ifndef  MIN
#define  MIN(a,b)  ((a)<(b)?(a):(b))
#endif

#endif /* _UTILS_H_ */
