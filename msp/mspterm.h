
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
   msp.h
*/


#ifndef _DEV_H_
#define _DEV_H_

extern int mspU2D (int argc, char *argv[],
		   int devId);	        /* User 2 Driver */
extern int mspSelect(int fd);
extern int mspPoll(int fd);		/* do poll */
extern int mspExit(int fd);		/* clean up for exit */
extern int mspInit(char *fileName);	/* init (really!) */

#endif _DEV_H_
