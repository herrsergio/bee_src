
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








/*****************************************************************************
 * PROJECT: Rhino
 *
 * FILE: devUtils.h
 *
 * ABSTRACT:
 * 
 * This file provides a set for routines for use with device interfaces.
 * A device interface is a set of routines that provides high level access
 * to a device through its device driver.  All device interfaces are required
 * to be able to connect to a socket rather than the device driver. The routine
 * provided in the file are to help with connecting to a simulator.
 *
 * Note : ANSI C headers are used.
 *
 *****************************************************************************/

#ifndef DEVUTIL_LOADED
#define DEVUTIL_LOADED

#include <stdio.h>
#include <sys/types.h>
#include <sys/time.h>
#include "Common.h"
#include "devNames.h"
#include <limits.h>
#define USE_OLD_TTY
/*#include <sys/ttydev.h>*/


/* declare some constants that should be in a system header file. */
#define stdin_fd  0
#define stdout_fd 1
#define stderr_fd 2

/* define any device constants */

/* defaults for communication */
/*#include <sys/ttydev.h>*/

#define DEFAULT_BAUD    B9600
#define DEFAULT_PORT    1621

/* Type declarations */

/*****************************************************************************
 * TYPE : DEV_INTERFACE_TYPE, DEV_INTERFACE_PTR
 * 
 * Usage : this structure is used to pass information to open a socket to
 * a simulated device.  The BOOLEAN field simulator will be non-null if the
 * simulator is to be used.  In such a case, the machine field gives the 
 * name of the machine running the simulator and the portNumber gives the number
 * of the socket.
 *****************************************************************************/

/* A device output handler receives the socket descriptor and the number
 * of bytes to be read.
 */

typedef struct _dev *DEV_PTR;

typedef void (*DEVICE_OUTPUT_HND)(int, long );
typedef void (*DEVICE_SET_TIMEOUT)(DEV_PTR, int);
typedef void (*DEVICE_CANCEL_TIMEOUT)(DEV_PTR);

typedef struct _dev {
  BOOLEAN use_simulator;
  struct {
    char *machine;
    int portNumber;
  } sim_dev;
  struct {
    char *ttyPort;
    int baudCode;   /* one of the codes B0 .. B9600 in ttydev.h */
  } ttydev;
  char *devName;
  int fd;
  BOOLEAN listen;
  BOOLEAN debug;
  FILE *debug_file;
  fd_set *readMask;
  DEVICE_OUTPUT_HND outputHnd;
  void (* timeoutHnd)(void);  
  DEVICE_SET_TIMEOUT  setTimeout;  
  DEVICE_CANCEL_TIMEOUT cancelTimeout;
  void (* pollHnd)(void);
  struct timeval pollInterval;
  struct timeval pollTime;
  struct timeval timeOutTime;
  void (* sigHnd)(void);
  BOOLEAN use_rwi_server;
}  DEV_TYPE;

/* forward declaration for default stdin handler */

void stdin_defaultInputHnd(int fd, long chars_available);

#ifdef DECLARE_DEVUTILS_VARS

DEV_PTR devices[FD_SETSIZE];
fd_set devConnections;

DEV_TYPE    stdin_device = 
{ FALSE,
    { "", DEFAULT_PORT},
    { "Standard in", DEFAULT_BAUD},
    "Standard in",
    stdin_fd,
    TRUE,
    FALSE,
    (FILE *) NULL,
    &devConnections,
    (DEVICE_OUTPUT_HND) stdin_defaultInputHnd,
    (void (*)(void)) NULL,  
    (DEVICE_SET_TIMEOUT)  NULL,  
    (DEVICE_CANCEL_TIMEOUT) NULL,
    (void (*)(void)) NULL,  
    {0, 0},
    {LONG_MAX, 0},
    {LONG_MAX, 0},
    (void (*)(void)) NULL
    };

#else

extern DEV_PTR devices[FD_SETSIZE];
extern fd_set devConnections;

extern DEV_TYPE  stdin_device;

#endif
/* Utility routine */

int connectToSocket(DEV_PTR);

int connectSimulator(DEV_PTR);

int connectTotty(DEV_PTR dev);

void connectDev(DEV_PTR dev);

void disconnectDev(DEV_PTR dev);

void flushChars(DEV_PTR);

BOOLEAN writeN(DEV_PTR, char *, int);

int readN(DEV_PTR, char *, int);

long numChars(int);

void cancelTimeout (DEV_PTR);
void setTimeout (DEV_PTR dev, int seconds);

void devMainLoop(void);

void ProcessDevices(void);

void ProcessSingleDevice(DEV_PTR dev);

void devInit();

void devStartPolling(DEV_PTR dev,
		     struct timeval *interval,
		     void (* handler)(void));

typedef struct { char *buffer;
		 int length;
		 int nextChar;
		 char delimChar;
		 void (*processRoutine)(char *);
	       } LINE_BUFFER_TYPE, *LINE_BUFFER_PTR;

LINE_BUFFER_PTR createLineBuffer(int lineLength, char delimChar,
				 void (*processRoutine)(char *));

void processOutput (DEV_PTR device, LINE_BUFFER_PTR lineBuffer, 
		    int chars_available);

BOOLEAN WaitForResponse(DEV_PTR dev, int *doneFlag, long timeout);

void ExitFromWait(void);

BOOLEAN noinput(void);

/* Enhanced setting possibilities for the devices. Necessary for the
 * laser range finders. */
void m_setrts( int fd);
void m_setparms( int fd,
		char *baudr,
		char *par,
		char *bits,
		int hwf,
		int swf);

#endif
