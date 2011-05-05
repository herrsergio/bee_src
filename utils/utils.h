
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
 * Copyright 1994, Brown University, Providence, RI
 *
 * Permission to use and modify this software and its documentation for
 * any purpose other than its incorporation into a commercial product is
 * hereby granted without fee.  Permission to copy and distribute this
 * software and its documentation only for non-commercial use is also
 * granted without fee, provided, however, that the above copyright notice
 * appear in all copies, that both that copyright notice and this permission
 * notice appear in supporting documentation, that the name of Brown
 * University not be used in advertising or publicity pertaining to
 * distribution of the software without specific, written prior permission,
 * and that the person doing the distribution notify Brown University of
 * such distributions outside of his or her organization. Brown University
 * makes no representations about the suitability of this software for
 * any purpose.  It is provided "as is" without express or implied warranty.
 * Brown University requests notification of any modifications to this
 * software or its documentation.
 *
 * Send the following redistribution information:
 *
 *	Name:
 *	Organization:
 *	Address (postal and/or electronic):
 *
 * To:
 *	Software Librarian
 *	Computer Science Department, Box 1910
 *	Brown University
 *	Providence, RI 02912
 *
 *		or
 *
 *	brusd@cs.brown.edu
 *
 * We will acknowledge all electronic notifications.
 */

/******************************************************/
/*                                                    */
/* util.h                                             */
/*                                                    */
/* Include file for random utilities                  */
/* version 0.8                                        */
/* James Kurien   jmk@cs.brown.edu                    */
/*                                                    */
/*                                                    */
/*                                                    */
/* revision history:                                  */
/*  9/17/94 jmk       Carved this out of rai.h where  */
/*		      it did not belong.              */
/* 2/3/95   jmk       Added makeLock, releaseLock     */
/*                                                    */
/* 9/15/95  jmk       Added hasResource               */
/******************************************************/

#include <stdio.h>
#include <fcntl.h> 
#include <termios.h>

#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TRUE
#define TRUE 1
#define FALSE !TRUE
#define ERROR -1
#endif

int openRaw(const char* filename, mode_t io_flags);
int openCooked(char* filename, mode_t io_flags);

int setBaudRate(int fd, speed_t baudRate);

int makeLock(char * name);
int releaseLock(char * name);

/* These check the host names associated with each resource (like an */
/* arm, a pan/tilt unit, etc                                         */

int localTo(const char * resourceName);
int hasResource(const char* resourceName);

/* fopenEtcFile() will search in standard locations for a config file */
FILE * fopenEtcFile(const unsigned char *fileName);

extern char *argv0; /* fopenEtcFile() needs this to get to some places */

/* I doubt this stuff (which certain people insisted was critical :)*/
/* has ever been used.  It should be removed at some point          */
/* -----------------------------------------------------------------*/

/*********************************************************/
/* Command line/conditional compile utilities            */
/*                                                       */
/* As described in the README file, these are usefule for*/
/* conditionally compiling modules or turning them on or */
/* off based upon the command line, assume you use the   */
/* command line form described in README                 */ 
/*                                                       */
/*********************************************************/

/* return non-zero if module_name appears turned on in */
/* the passed args */

int turnedOn(int argc, char** argv, char* module_name);

#define warnUndefined(name) \
 printf("Warning: failed to init uncompiled %s module\n",name)

#define usage(exec_name) \
 printf("Usage: %s [+|- <module>+]\n",exec_name);


#ifdef __cplusplus
}
#endif

#endif /* UTILS_H */
