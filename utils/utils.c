
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

#include <unistd.h>
#include <stdlib.h> 
#include <string.h>
#include <termios.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <pwd.h>
#include <rai.h>      /* for ERROR */
#include <utils.h>
#include <lockNames.h>

#define HOSTNAME_LENGTH 512   /* how long of a string to pass to gethostname */

char *argv0 = NULL;

/**********************************************************/
/*                                                        */
/*  openRaw						  */	
/*                                                        */
/*  Opens passed filename and sets its device to raw mode.*/
/*  Returns file descriptor.                              */
/*                                                        */
/*  raw mode is defined as:                               */
/*  Off: echo, canonical input, extended processing,      */
/*       signals, break key, parity, 8th bit strip,       */
/*       flow control, output post processing             */
/*   On: 8 bit size                                       */
/*                                                        */
/**********************************************************/
 
int openRaw(const char * filename, mode_t io_flags)
{
  struct termios term_info;
  int fd;

  fd = open(filename,io_flags);
  if (fd == -1)
    {
      /* maybe complain if TRACE is on */
      return ERROR;
    }
  
  if(tcgetattr(fd,&term_info) <0)
    {
      /* complain - fd is not a terminal */
      return ERROR;
    }

  /* turn off echo, canonical mode, extended processing, signals */
  term_info.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  
  /* turn off break sig, cr->nl, parity off, 8 bit strip, flow control */
  term_info.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

  /* clear size, turn off parity bit */
  term_info.c_cflag &= ~(CSIZE | PARENB);

  /* set size to 8 bits */
  term_info.c_cflag |= CS8;

  /* turn output processing off */
  term_info.c_oflag &= ~(OPOST);

  /* Set time and bytes to read at once */
  term_info.c_cc[VTIME] = 0;
  term_info.c_cc[VMIN] = 0;

  if(tcsetattr(fd,TCSAFLUSH,&term_info) <0)
    return -1;

  return fd;
}

/**********************************************************/
/*                                                        */
/*  openCooked						  */	
/*                                                        */
/*  Opens passed filename and sets its device to cooked   */
/*  mode. Returns file descriptor.                        */
/*                                                        */
/*  cooked mode is defined as:                            */
/*  On: echo, canonical input, extended processing,       */
/*       signals, break key, parity, 8th bit strip,       */
/*       flow control, output post processing             */
/*                                                        */
/*  Everything else is left as previously set on the      */
/*  device.                                               */	
/*                                                        */
/**********************************************************/ 

int openCooked(char* filename, mode_t io_flags)
{

  struct termios term_info;

  int fd;

  fd = open(filename,io_flags);
  if (fd == -1)
    {
      /* maybe complain if TRACE is on */
      return ERROR;
    }
  /*  I am not really sure what to set for cooked */

  if(tcgetattr(fd,&term_info) <0)
    {
      /* complain - fd is not a terminal */
      return ERROR;
    }

  /* turn on echo, canonical mode, extended processing, signals */
  term_info.c_lflag |= ECHO | ICANON | IEXTEN | ISIG;
  
  /* turn on break sig, cr->nl, parity, 8 bit strip, flow control */
  term_info.c_lflag |= BRKINT | ICRNL | INPCK | ISTRIP | IXON;

  /* turn output processing on */
  term_info.c_lflag |= OPOST;

  if(tcsetattr(fd,TCSAFLUSH,&term_info) <0)
    return -1;

  return fd;

}
 

/**********************************************************/
/*                                                        */
/*  setBaudRate						  */	
/*                                                        */
/*  Set the baud rate of an open device.                  */
/*                                                        */
/*  Baud rate constants are found in <termios.h>.         */
/*  B9600 == 9600 baud                                    */
/*                                                        */
/**********************************************************/

int setBaudRate (int fd, speed_t baudRate)
{
   struct termios terminfo;
   int error;
     
   error = tcgetattr(fd, &terminfo);
   if (error)
      {
         perror("tcgetattr()");
         return(FALSE);
      }
   error = cfsetospeed(&terminfo, baudRate);
   if (error)
      {
         fprintf(stderr, "cfsetospeed(%ld): ",
                 (long)baudRate);
         perror(NULL);
         return(FALSE);
      }
   error = cfsetispeed(&terminfo, baudRate);
   if (error)
      {
         fprintf(stderr, "cfsetispeed(%ld): ",
                 (long)baudRate);
         perror(NULL);
         return(FALSE);
      }
   error = tcsetattr(fd, TCSANOW, &terminfo);
   if (error)
      {
         perror("tcsetattr()");
         return(FALSE);
      }
   return(TRUE);
}

/************************************************************
 *
 * openEtcFile()
 *
 * will search in standard locations for a config file
 * and return a FILE * for it.
 *
 ************************************************************/

FILE * fopenEtcFile(const unsigned char *fileName) {
  FILE *fp;
  char fullName[256];
  struct passwd *passwdent;

  /*
   * First try the current directory
   */
#ifdef VERBOSE
  fprintf (stderr, "Trying to open %s ...\n", fileName);
#endif
  fp = fopen(fileName, "r");
  
  /*
   * If that fails try dirname(argv[0])/etc
   * This would work to find the etc dir in the local tree
   * if the binary is run from the top directory.
   */
  
  if ((fp==NULL) && argv0) {
    fullName[255]=0;
    strncpy(fullName, argv0, 255);
    
    while(strlen(fullName) && (fullName[strlen(fullName)-1]!='/')) {
      fullName[strlen(fullName)-1]=0;
    }
    
    strncat(fullName, "etc/", 255-strlen(fullName));
    strncat(fullName, fileName, 255-strlen(fullName));
#ifdef VERBOSE
    fprintf(stderr, "Trying to open %s ...\n", fullName);
#endif
    fp = fopen (fullName, "r");
  }
  
  /*
   * If that fails try dirname(argv[0])/../etc
   * This would work to find the etc dir in the local tree
   * if the binary is run from bin.
   */
  
  if ((fp==NULL) && argv0) {
    fullName[255]=0;
    strncpy(fullName, argv0, 255);
    
    while(strlen(fullName) && (fullName[strlen(fullName)-1]!='/')) {
      fullName[strlen(fullName)-1]=0;
    }
    
    strncat(fullName, "../etc/", 255-strlen(fullName));
    strncat(fullName, fileName, 255-strlen(fullName));
#ifdef VERBOSE
    fprintf(stderr, "Trying to open %s ...\n", fullName);
#endif
    fp = fopen (fullName, "r");
  }
  
  /*
   * If that fails try `dirname argv[0]`/../../etc
   * This would work to find the etc dir in the local tree
   * if the binary is run from its src directory.
   */
  
  if (fp==NULL) {
    fullName[255]=0;
    strncpy(fullName, argv0, 255);
    
    while(strlen(fullName) && (fullName[strlen(fullName)-1]!='/')) {
      fullName[strlen(fullName)-1]=0;
    }
    
    strncat(fullName, "../../etc/", 255-strlen(fullName));
    strncat(fullName, fileName, 255-strlen(fullName));
#ifdef VERBOSE
    fprintf(stderr, "Trying to open %s ...\n", fullName);
#endif
    fp = fopen (fullName, "r");
  }
  
  /*
   * If that fails try ~bee/etc
   */

  if (fp==NULL) {
    fullName[255]=0;

    passwdent = getpwnam("bee");

    if (passwdent) {
      strncpy(fullName, passwdent->pw_dir, 255);
      strncat(fullName, "/etc/", 255-strlen(fullName));
      strncat(fullName, fileName, 255-strlen(fullName));
#ifdef VERBOSE
      fprintf(stderr, "Trying to open %s ...\n", fullName);
#endif
      fp = fopen (fullName, "r");
    }
  }
    
  /*
   * If that fails try ~orion/etc
   */

  if (fp==NULL) {
    fullName[255]=0;

    passwdent = getpwnam("orion");

    if (passwdent) {
      strncpy(fullName, passwdent->pw_dir, 255);
      strncat(fullName, "/etc/", 255-strlen(fullName));
      strncat(fullName, fileName, 255-strlen(fullName));
#ifdef VERBOSE
      fprintf(stderr, "Trying to open %s ...\n", fullName);
#endif
      fp = fopen (fullName, "r");
    }
  }
    
  /*
   * If that fails try ~rhino/etc
   */

  if (fp==NULL) {
    fullName[255]=0;

    passwdent = getpwnam("rhino");

    if (passwdent) {
      strncpy(fullName, passwdent->pw_dir, 255);
      strncat(fullName, "/etc/", 255-strlen(fullName));
      strncat(fullName, fileName, 255-strlen(fullName));
#ifdef VERBOSE
      fprintf(stderr, "Trying to open %s ...\n", fullName);
#endif
      fp = fopen (fullName, "r");
    }
  }
    
  /*
   * If that fails try /usr/local/etc
   */

  if (fp==NULL) {
    fullName[255]=0;
    strncpy(fullName, "/usr/local/etc/", 255);
    strncat(fullName, fileName, 255-strlen(fullName));
#ifdef VERBOSE
    fprintf(stderr, "Trying to open %s ...\n", fullName);
#endif
    fp = fopen (fullName, "r");
  }
      
  /*
   * If that fails try /usr/etc
   */

  if (fp==NULL) {
    fullName[255]=0;
    strncpy(fullName, "/usr/etc/", 255);
    strncat(fullName, fileName, 255-strlen(fullName));
#ifdef VERBOSE
    fprintf(stderr, "Trying to open %s ...\n", fullName);
#endif
    fp = fopen (fullName, "r");
  }
      
  /*
   * If that fails try /etc
   */

  if (fp==NULL) {
    fullName[255]=0;
    strncpy(fullName, "/etc/", 255);
    strncat(fullName, fileName, 255-strlen(fullName));
#ifdef VERBOSE
    fprintf(stderr, "Trying to open %s ...\n", fullName);
#endif
    fp = fopen (fullName, "r");
  }
      
  if (fp!=NULL) {
    fprintf(stderr, "File opened.\n");
  }
  else {
    fprintf(stderr, "File %s not found.\n", fileName);
  }
  return(fp);
}



/**********************************************************/
/*                                                        */
/*  makeLock						  */	
/*                                                        */
/*  Problem:  How do we make sure two people do not start */
/*  base programs or the like at once?  Ideally, we could */  
/*  use flock to lock the file the base uses.  When a     */
/*  base program started we would try to lock it.  If we  */
/*  could not, we know someone is using it.  If the       */
/*  using it crashes or is terminated, the flock is       */
/*  removed automactically.                               */
/*                                                        */
/*  The only problem is that the base and so on only      */
/*  use device files, which you cannot flock.  So we      */
/*  create a dummy file and flock it.  Note, just because */
/*  the file exists does not mean something is locked.    */
/*  The file must exist and the OS must have an flock     */
/*  on it for the process.  If the process is terminated, */
/*  the flock is removed but the file stays.              */
/**********************************************************/ 

int makeLock(char* lockName)
{
#if 0
 int fd;
 int pid;
 int oldUmask;
 char name[HOSTNAME_LENGTH];
 char identification[HOSTNAME_LENGTH+20];

 sprintf(name,"%s%s",LOCK_PREFIX,lockName);

 oldUmask = umask(0);
 fd = open(name, (O_RDWR|O_CREAT|O_TRUNC|O_SYNC), 0666);
 umask(oldUmask); 

 if (fd<0)
   {
     fprintf(stderr,"Could not open lock file %s: ",name);
     perror(NULL);
     return ERROR;
   }
 if (flock(fd, (LOCK_EX|LOCK_NB)) <0)
   {
     fprintf(stderr,"Lock file %s seems to be already locked.\n",name);
     return ERROR;
   }
 
 gethostname(name, sizeof(name));
 pid = getpid(); 
 
 sprintf(identification,"%s,%d\n",name,pid);
 write(fd,identification,strlen(identification));

 /* do not close fd, or you will lose the lock */       
#endif
 return 0;
}

 
/**********************************************************/
/*                                                        */
/*  releaseLock						  */	
/*                                                        */
/*  If the lock file does not exist the lock is released. */
/*  If the lock file exists, then we release the lock on  */
/*  it.                                                   */
/*                                                        */
/*  Note that when a process terminates any locks is has  */ 
/*  are released automatically, so it is  not strictly    */
/*  necessary to call this.                               */
/*                                                        */
/**********************************************************/ 

int releaseLock(char* name)
{
#if 0
 int fd;
 char filename[256];

 /* this will blow up if name+LOCK_PREFIX is longer than 256 */
 sprintf(filename,"%s%s",LOCK_PREFIX,name);
 fd = open(filename,O_RDONLY);
 if (fd<0) return 0;

 if (flock(fd,LOCK_UN) <0)
   return ERROR;
#endif
 return 0;
}



/**********************************************************/
/*                                                        */
/*  Problem:  How do know where the pan/tilt, base or     */
/*  whatever is connected, so we can write programs which */
/*  bail if they are started on the wrong computer, or    */
/*  better yet, can decide whether to use remote or local */
/*  versions of a library?                                */
/*                                                        */
/*  We have a config file that the user must source with  */
/*  their shell before running any RAI programs.  This    */
/*  file gives the hostname of the computer with each     */
/*  robot resource.  This function returns TRUE if the    */
/*  named resource is on the current host, ERROR if the   */
/*  location of the named resource is not known, and      */
/*  FALSE if the resource is known to be on another host. */
/*                                                        */
/*  Each resource name is an environment variable whose   */
/*  value is the name of the computer the resource is on. */
/*  So we just compare the value to the current hostname  */  
/*  See the file utilities/config.sh for the defintion of */
/*  all the resource names */ 

int localTo(const char* resourceName)
{

  char* resourceHost;

  char currentHost[HOSTNAME_LENGTH];
  char errval;

#if 1
  return(1);
#else
  resourceHost = getenv(resourceName);

  if (resourceHost==NULL) {
    /* if env variable is undefined, just ignore test */
    return(1);
  }

  if (!strcmp("localhost", resourceHost)) return(1);

  errval = gethostname(currentHost,HOSTNAME_LENGTH);  
  if (errval) 
    {
      fprintf(stderr,"localTo failed due to long host name\n");
      return ERROR;
    }
  return (!strcmp(currentHost,resourceHost));
#endif
}


/**********************************************************/
/*                                                        */
/*  This just checks if the host name associated with     */
/*  a resource like an RWI arm is defined.  If not, we    */
/*  can assume this robot does not have the resource.     */
/**********************************************************/

int hasResource(const char* resourceName)
{
  char* resourceHost;
  resourceHost = getenv(resourceName);

  if (resourceHost == NULL) {
    return(TRUE);
  }
  else if (!strcmp("NONE",resourceHost)) {
    return FALSE;
  }
  else {
    return TRUE;
  }
}

