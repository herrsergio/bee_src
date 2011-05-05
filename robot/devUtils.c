
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
 * PROJECT: rhino
 *
 * FILE: devUtils.c
 *
 * ABSTRACT:
 * 
 * This file provides a set for routines for use with device interfaces.
 * A device interface is a set of routines that provides high level access
 * to a device through its device driver.  All device interfaces are required
 * to be able to connect to a socket rather than the device driver.  The 
 * routine provided in the file are to help with connecting to a simulator.
 *
 *
 *****************************************************************************/

#define USE_OLD_TTY

#ifdef i386
#include <sys/types.h>
#include <termios.h>

#if 0
/*
 * Linux glibc-2/libc-6 cleanely doesn't 
 * bother supporting this old waste of space.
 */

#ifndef TIOCGETP 
#define TIOCGETP        0x5481
#define TIOCSETP        0x5482
#define RAW             1
#define CBREAK          64

struct sgttyb
{
    unsigned short sg_flags;
    char sg_ispeed;
    char sg_ospeed;
    char sg_erase;
    char sg_kill;
    struct termios t;
    int check;
};
#endif
#endif /* TIOCGETP */

#else

#define BSD_COMP
#include <strings.h>
#include <sys/types.h>
#include <sys/termios.h>
#include <sys/ttold.h>
#include <sys/fcntl.h>
#include <sys/filio.h>
#include <sys/ioctl.h>
#include <sys/ttydev.h>
#include <sys/termios.h>
/* #include <sys/sunddi.h> */

#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <sys/errno.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <sys/time.h>
#include <sys/timeb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <signal.h>
#include <termios.h>
#include <sys/ioctl.h>

#include "Common.h"
/*#include "libc.h"*/
#include "handlers.h"
#define DECLARE_DEVUTILS_VARS
#include "devUtils.h"
#include <unistd.h>

#include "robot_specifications.h"
#include "tcx.h"
#include "tcxP.h"
#include "global.h"/*(st)*/

int listen_for_tcx_events = 0;	/* set this to 1 if you want to
				 * listen for devices and TCX events
				 * simultaneously. */

/*****************************************************************************
 * Global constants 
 *****************************************************************************/

#define WAITING -1 /* used to indicate a handler is waiting */

/*****************************************************************************
 * Global variables 
 *****************************************************************************/

static struct timeval MAX_TIMEOUT = {60,0};   /* one minute maximum timeout. */
static struct timeval ZERO_TIMEOUT = {0,0};   /* zero timeout. */

static int waiting = 0; /* the number of handlers waiting for returns */
static struct timeval global_time_out = {0,0};

int maxDevNum = 0; /* number of devices actually connected */

/*****************************************************************************
 *
 * FUNCTION: void devSignal(void);
 *
 * DESCRIPTION: Handles various signals.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

void devSignal(int s, int c)
{
  /* the default thing to do is call the exit routine for each device and 
   * then disconnect the device and exit
   */
  int i;
  
  fprintf(stderr,"\n Abort: caught a signal %d %d; cleaning up.\n",s,c);
  fflush(stderr);
  for(i=0; i<maxDevNum; i++){
    if (devices[i] != NULL) {
      if (devices[i]->sigHnd != NULL) {
	(* (devices[i]->sigHnd))();
      }
      if (devices[i]->fd != -1) {
	close(devices[i]->fd);
	devices[i]->fd = -1;
      }
      disconnectDev(devices[i]);
    }
  }
  exit(0);
}

/*****************************************************************************
 *
 * FUNCTION: void devInit(void);
 *
 * DESCRIPTION: Initialze the data structures used by devUtils.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

void devInit(void)
{
  static BOOLEAN firstTime = TRUE;

  if (firstTime) {
    fprintf(stderr, "-A-");
    firstTime = FALSE;
    fprintf(stderr, "-B-");
    FD_ZERO(&devConnections);
    fprintf(stderr, "-C-");
    bzero(devices, sizeof(devices));
    fprintf(stderr, "-D-");
    /* set up signal handlers */
/*    signal(SIGINT,  (void *) devSignal);*/
    /* signal(SIGQUIT, devSignal); */
    /* signal(SIGILL,  devSignal); */
    /* signal(SIGFPE,  devSignal); */
/*    signal(SIGBUS, (void *)  devSignal);
    signal(SIGSEGV, (void *) devSignal); */
    /* signal(SIGSYS,  devSignal); */
    /* signal(SIGTERM, devSignal); */
    fprintf(stderr, "-E-");
  }
  fprintf(stderr, "-F-");
}

/*****************************************************************************
 *
 * FUNCTION: void connectDev(DEV_PTR dev);
 *
 * DESCRIPTION:
 *
 * INPUTS:
 * DEV_PTR dev - structure for initializing connection.
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

void connectDev(DEV_PTR dev)
{
  if (dev->listen) 
    FD_SET(dev->fd, &devConnections);
  devices[dev->fd] = dev;
  if(dev->fd+1 > maxDevNum)
    maxDevNum = dev->fd+1;
}

/*****************************************************************************
 *
 * FUNCTION: void disconnectDev(DEV_PTR dev);
 *
 * DESCRIPTION:
 *
 * INPUTS:
 * DEV_PTR dev - structure for initializing connection.
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

void disconnectDev(DEV_PTR dev)
{
  if (dev->listen) 
    FD_CLR(dev->fd, &devConnections);
  devices[dev->fd] = NULL;
}


/*****************************************************************************
 *
 * FUNCTION: int connectToSocket(DEV_PTR dev)
 *
 * DESCRIPTION:
 *
 * INPUTS:
 * DEV_PTR dev - structure for initializing connection.
 *
 * OUTPUTS: Return socket descriptor number if successful, else -1.
 *
 * HISTORY:
 *
 *****************************************************************************/

int connectToSocket(dev)
     DEV_PTR dev;
{
  struct hostent *hp;
  int sd;
  
  struct sockaddr_in serverAddr;
  
  bzero((char *)&serverAddr, sizeof(struct sockaddr_in));
  serverAddr.sin_port = dev->sim_dev.portNumber;
  serverAddr.sin_family = AF_INET;
  
  if ((hp = gethostbyname(dev->sim_dev.machine)) == NULL)
    return -1;
  bcopy(hp->h_addr, &serverAddr.sin_addr, hp->h_length);
  
  if (((sd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) ||
      (connect(sd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)) {
    fprintf(stderr,"Unable to connect to socket\n");
    fprintf(stderr, "Errno: %d\n", errno);
    fprintf(stderr,"Make sure the server is running first\n");
    fprintf(stderr,"Also, after a crash it takes the system about 1 minute\n");
    fprintf(stderr,"to reset the socket.\n");
    exit(-1);
  } else {
    dev->fd = sd;
  }
  return sd;
}


/*****************************************************************************
 *
 * FUNCTION: void m_setrts(int fd)
 *
 * DESCRIPTION: Set RTS line. Sometimes dropped. Linux specific?
 *
 * HISTORY:
 *
 ****************************************************************************/
void m_setrts( int fd)
{
#if defined(TIOCM_RTS) && defined(TIOCMODG)
  int mcs;

  ioctl(fd, TIOCMODG, &mcs);
  mcs |= TIOCM_RTS;
  ioctl(fd, TIOCMODS, &mcs);
#endif
#ifdef _COHERENT
  ioctl(fd, TIOCSRTS, 0);
#endif
}

/*
 * Set baudrate, parity and number of bits.
 */

#define _POSIX

/*****************************************************************************
 *
 * FUNCTION: void m_setparms(....)
 *
 * DESCRIPTION: Set baudrate, parity and number of bits.
 *
 * HISTORY:
 *
 ****************************************************************************/
void m_setparms(int fd,
		char *baudr,
		char *par,
		char *bits,
		int hwf,
		int swf)
{
  int spd = -1;
  int newbaud;
  int bit = bits[0];

#ifdef _POSIX
  struct termios tty;

  tcgetattr(fd, &tty);
#else
  struct sgttyb tty;

  ioctl(fd, TIOCGETP, &tty);
#endif

  /* We generate mark and space parity ourself. */
  if (bit == '7' && (par[0] == 'M' || par[0] == 'S'))
	bit = '8';

  /* Check if 'baudr' is really a number */
  if ((newbaud = (atol(baudr) / 100)) == 0 && baudr[0] != '0') newbaud = -1;

  switch(newbaud) {
  	case 0:
#ifdef B0
			spd = B0;	break;
#else
			spd = 0;	break;
#endif
  	case 3:		spd = B300;	break;
  	case 6:		spd = B600;	break;
  	case 12:	spd = B1200;	break;
  	case 24:	spd = B2400;	break;
  	case 48:	spd = B4800;	break;
  	case 96:	spd = B9600;	break;
#ifdef B19200
  	case 192:	spd = B19200;	break;
#else
#  ifdef EXTA
	case 192:	spd = EXTA;	break;
#   else
	case 192:	spd = B9600;	break;
#   endif	
#endif	
#ifdef B38400
  	case 384:	spd = B38400;	break;
#else
#  ifdef EXTB
	case 384:	spd = EXTB;	break;
#   else
	case 384:	spd = B9600;	break;
#   endif
#endif	
#ifdef B57600
	case 576:	spd = B57600;	break;
#endif
#ifdef B115200
	case 1152:	spd = B115200;	break;
#endif
  }
  
#if defined (_BSD43) && !defined(_POSIX)
  if (spd != -1) tty.sg_ispeed = tty.sg_ospeed = spd;
  /* Number of bits is ignored */

  tty.sg_flags = RAW | TANDEM;
  if (par[0] == 'E')
	tty.sg_flags |= EVENP;
  else if (par[0] == 'O')
	tty.sg_flags |= ODDP;
  else
  	tty.sg_flags |= PASS8 | ANYP;

  ioctl(fd, TIOCSETP, &tty);

#  ifdef TIOCSDTR
  /* FIXME: huh? - MvS */
  ioctl(fd, TIOCSDTR, 0);
#  endif
#endif

#if defined (_V7) && !defined(_POSIX)
  if (spd != -1) tty.sg_ispeed = tty.sg_ospeed = spd;
  tty.sg_flags = RAW;
  if (par[0] == 'E')
	tty.sg_flags |= EVENP;
  else if (par[0] == 'O')
	tty.sg_flags |= ODDP;

  ioctl(fd, TIOCSETP, &tty);
#endif

#ifdef _POSIX

  if (spd != -1) {
	cfsetospeed(&tty, (speed_t)spd);
	cfsetispeed(&tty, (speed_t)spd);
  }

  switch (bit) {
  	case '5':
  		tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS5;
  		break;
  	case '6':
  		tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS6;
  		break;
  	case '7':
  		tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS7;
  		break;
  	case '8':
	default:
  		tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  		break;
  }		
  /* Set into raw, no echo mode */
#if !defined(_DGUX_SOURCE)
  tty.c_iflag &= ~(IGNBRK | IGNCR | INLCR | ICRNL | IUCLC | 
  	IXANY | IXON | IXOFF | INPCK | ISTRIP);
  tty.c_iflag |= (BRKINT | IGNPAR);
  tty.c_oflag &= ~OPOST;
  tty.c_lflag = ~(ICANON | ISIG | ECHO | ECHONL | ECHOE | ECHOK);
  tty.c_cflag |= CREAD | CRTSCTS;
#else /* Okay, this is better. XXX - Fix the above. */
  tty.c_iflag =  IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cflag |= CLOCAL | CREAD;
#endif
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 5;

  /* Flow control. */
  if (hwf) {
    tty.c_cflag |= CRTSCTS;
    tty.c_cflag &= ~CLOCAL;
  }
  else {
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CLOCAL;
  }

  if (swf) {
    tty.c_iflag |= (IXON | IXOFF);
  }
  else {
    tty.c_iflag &= ~(IXON | IXOFF);
  }

  tty.c_cflag &= ~(PARENB | PARODD);
  if (par[0] == 'E')
	tty.c_cflag |= PARENB;
  else if (par[0] == 'O')
	tty.c_cflag |= PARODD;

  tcsetattr(fd, TCSANOW, &tty);

  m_setrts(fd);
#  ifdef _DGUX_SOURCE
  m_sethwf(fd, hwf);
#  endif
#endif
}


/*****************************************************************************
 *
 * FUNCTION: int connectTotty(DEV_PTR dev)
 *
 * DESCRIPTION:
 *
 * INPUTS:
 * DEV_PTR dev - structure for initializing connection.
 *
 * OUTPUTS: Return tty descriptor number if successful, else -1.
 *
 * HISTORY:
 *
 ****************************************************************************/
int connectTotty(DEV_PTR dev)
{
  
  /*.. Open the port for both ReaDing and WRiting ..*/

  fprintf(stderr, "-+1-");
  if ((dev->fd = open(dev->ttydev.ttyPort, O_RDWR, 0)) < 0) 
    {
      fprintf(stderr,"could not open port %s\n", dev->ttydev.ttyPort); 
      fflush(stderr);
      exit(-1);
    };

  fprintf(stderr, "-+2-");
  if (ioctl(dev->fd, TIOCNXCL, NULL) < 0) {
    fprintf(stderr, "Warning, can't set exclusive access for %s\n",
	    dev->ttydev.ttyPort);
  };
  fprintf(stderr, "-+3-");


/*
  ioctl(dev->fd, TIOCMODS, TIOCM_LE);
  ioctl(dev->fd, TIOCMODS, TIOCM_DTR);
  ioctl(dev->fd, TIOCMODS, TIOCM_RTS);
  ioctl(dev->fd, TIOCMODS, TIOCM_CTS);
  ioctl(dev->fd, TIOCMODS, TIOCM_DSR);
*/

#ifdef i386  /* Linux prefers standard, proper POSIX */
  {
    struct termios term_info;

    if(tcgetattr(dev->fd,&term_info) <0) {
      /* complain - fd is not a terminal */
      fprintf(stderr, "%s:%6d:%s() - ERROR: Can't get control bits.\n", 
	      __FILE__, __LINE__, __FUNCTION__);
      exit(-1);
    }

    /* turn on echo, canonical mode, extended processing, signals */
    term_info.c_lflag |= IEXTEN | ISIG;
  
    /* turn on break sig, cr->nl, parity, 8 bit strip, flow control */
    term_info.c_lflag |= BRKINT | ICRNL | INPCK | ISTRIP;
    
    /* turn off echo, canonical mode, extended processing, signals */
    term_info.c_lflag &= ~(ECHO | ICANON | IXON);
    
    /* clear size, turn off parity bit */
    term_info.c_cflag &= ~(CSIZE | PARENB);
    
    /* set size to 8 bits */
    term_info.c_cflag |= CS8;
    
    /* Set time and bytes to read at once */
    term_info.c_cc[VTIME] = 1;
    term_info.c_cc[VMIN] = 0;
    
    if (cfsetospeed(&term_info, dev->ttydev.baudCode)) {
      fprintf(stderr, "cfsetospeed(%ld): ",
	      (long)dev->ttydev.baudCode);
      perror(NULL);
      exit(-1);
    }

    if (cfsetispeed(&term_info, dev->ttydev.baudCode)) {
      fprintf(stderr, "cfsetispeed(%ld): ",
	      (long)dev->ttydev.baudCode);
      perror(NULL);
      exit(-1);
    }

    if(tcsetattr(dev->fd,TCSAFLUSH,&term_info) <0) {
      fprintf(stderr, "%s:%6d:%s() - ERROR: Can't initialize control bits.\n", 
	      __FILE__, __LINE__, __FUNCTION__);
      exit(-1);
    }
    
  }
#else  /* Linux no longer supports as much old incompatible BSD stuff */
  {
    struct sgttyb mode;

    ioctl(dev->fd, TIOCGETP, &mode);
    mode.sg_ispeed=dev->ttydev.baudCode;
    mode.sg_ospeed=dev->ttydev.baudCode;
    
    mode.sg_flags |= RAW;
    mode.sg_flags |= CBREAK;	/* _ws_ 940519 */
    
    if (ioctl(dev->fd, TIOCSETP, &mode) < 0) {
      fprintf(stderr, "ERROR: Can not initialize control bits.\n");
      exit(-1);
    };
  }
#endif

  flushChars(dev);
  return(dev->fd);
}

/******************************************************************************
 *
 * FUNCTION: BOOLEAN writeN(DEV_PTR, buf, nChars)
 *
 * DESCRIPTION: This routine primarily calls the system function write.
 *
 * INPUTS: 
 * DEV_PTR dev;
 * char *buf;
 * int nChars;
 *
 * OUTPUTS: BOOLEAN 
 *
 *****************************************************************************/

BOOLEAN writeN(DEV_PTR dev, char *buf, int nChars)
{
  int amountWritten = 0;
  
  if (dev->debug)
    if (dev->debug_file)
      fprintf(dev->debug_file, "%s sending: [%s]\n",dev->devName, buf);
    else
      printf("%s sending: [%s]\n",dev->devName, buf);
  
  while (nChars > 0) {
    usleep(1);
    amountWritten = write(dev->fd, buf, nChars);
    if (amountWritten < 0) {
      if (errno == EWOULDBLOCK) {
	fprintf(stderr,"\nWARNING: writeN: EWOULDBLOCK: trying again!\n");
	usleep(1);
      } else {
	return FALSE;
      }
    } else {
      nChars -= amountWritten;
      buf += amountWritten;
    }
  }
  return TRUE;
}

/******************************************************************************
 *
 * FUNCTION: long numChars(sd)
 *
 * DESCRIPTION:
 * Find out how many characters are available to be read.
 *
 * INPUTS: 
 * int sd;
 *
 * OUTPUTS: 
 *
 * NOTES:
 *
 *****************************************************************************/

long numChars(int sd)
{
  long available=0;
  
  if (ioctl(sd, FIONREAD, &available) == 0)
    return available;
  else
    return -1;
}    

/******************************************************************************
 *
 * FUNCTION: int readN(dev, buf, nchars)
 *
 * DESCRIPTION:
 * Read nchars of data from sd into buf. Continue to read until 
 * nchars have been read.  Return 0 if end of file reached and -1 if
 * there was an error.
 *
 * INPUTS: 
 * DEV_PTR dev;
 * char *buf;
 * int nchars;
 *
 * OUTPUTS: int numRead.
 *
 * NOTES:
 *
 * buf is a preallocated pointer to storage equal to nchars.
 * Propagation of low level errors - failures on read/writes still an issue.
 *
 *****************************************************************************/

int readN(DEV_PTR dev, char *buf, int nchars)
{
  int amountRead, amountToRead;
  
  amountToRead = nchars;
  for(;;){
    amountRead = read(dev->fd, buf, amountToRead);
    if (amountRead == 0)
      { /* just got an end of file indication. 
	 * close the socket and remove it from the list of active
	 * connections 
	 */
	/* remove the connection from the read mask */
	printf("Connection closed %s\n", dev->devName);
	FD_CLR(dev->fd,dev->readMask);
	close(dev->fd);
	return 0;
      } else if (amountRead < 0)
	/* some other problem.  Try to continue?
	 */
	return amountRead;
    amountToRead -= amountRead;
    if (amountToRead == 0)
      return amountRead;
    buf += amountRead;
  }
}

/******************************************************************************
 *
 * FUNCTION: void flushChars(DEV_PTR dev)
 *
 * DESCRIPTION:
 * Flush any characters from the input of the file descriptor.
 *
 * INPUTS: 
 * DEV_PTR dev;
 *
 * OUTPUTS: 
 *
 * NOTES:
 *
 * This could be done using FIONFLUSH for files and tty devices, but not for
 * sockets.  
 *
 *****************************************************************************/

void flushChars(DEV_PTR dev)
{
  long available=0;
  char inbuf[DEFAULT_LINE_LENGTH];
  
  while ((available=numChars(dev->fd)) > 0)
    readN(dev, inbuf, MIN(available,DEFAULT_LINE_LENGTH));
}    

/*****************************************************************************
 *
 * FUNCTION: int connectSimulator (DEV_PTR dev);
 *
 * DESCRIPTION:
 *
 * INPUTS:
 *
 * OUTPUTS: Return socket descriptor number if successful, else -1.
 *
 * HISTORY:
 *
 *****************************************************************************/

int connectSimulator (DEV_PTR dev)
{
  int sd;
  char inbuf[DEFAULT_LINE_LENGTH+1];
  long chars_available=0;
  fd_set readSet;
  int result=0;
  
  fprintf(stderr, "Trying to open simulated device %s\n",dev->devName);
  sd = connectToSocket(dev);
  if (sd != -1) {
    /* write the device name to the socket to allow the simulator 
     * to know what device we wanted to open.
     */
    
    fprintf(stderr, "Opened socket to simulated device %s\n",dev->devName);
    
    do {
      /* wait for the name of the simulator */
      FD_ZERO(&readSet);
      FD_SET(sd,&readSet);
      select(FD_SETSIZE, &readSet, NULL, NULL, NULL);
      
      chars_available = numChars(sd);
      if (chars_available == -1)
	fprintf(stderr,"POLL ERROR on socket\n");
    } while (chars_available <= 0);
    
    bzero(inbuf,DEFAULT_LINE_LENGTH+1);
    result = readN(dev,inbuf,MIN(chars_available,DEFAULT_LINE_LENGTH));
    
    if (result <= 0) 
      { /* cound not read the socket */
	return -1;
      }
    
    fprintf(stderr, "Connected to server %s\n",inbuf);
    
    writeN(dev,dev->devName,strlen(dev->devName));
    fprintf(stderr, "Opened simulated device %s\n",dev->devName);
    readN(dev,inbuf,strlen(dev->devName));
  }
  
  return sd;
}

/******************************************************************************
 *
 * FUNCTION: void setTimeout (DEV_PTR dev, int seconds)
 * 
 *
 * DESCRIPTION: 
 * Set a timeout for some device interface.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 *****************************************************************************/

void setTimeout (DEV_PTR dev, int seconds)
{
  gettimeofday (&(dev->timeOutTime), 0);
  dev->timeOutTime.tv_sec += seconds;
}

/******************************************************************************
 *
 * FUNCTION: void cancelTimeout (DEV_PTR dev)
 *
 *
 * DESCRIPTION: 
 * Cancel a timeout for some device interface.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 *****************************************************************************/

void cancelTimeout (DEV_PTR dev)
{
  dev->timeOutTime.tv_sec = LONG_MAX;
  dev->timeOutTime.tv_usec = 0;
}


/******************************************************************************
 *
 * FUNCTION: lessTime (struct timeval *t1, struct timeval *t2)
 *
 * DESCRIPTION: 
 * Returns true if t1 is less than t2.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 *****************************************************************************/
INLINE
static BOOLEAN lessTime (struct timeval *t1, struct timeval *t2)
{
  return((t1->tv_sec < t2->tv_sec ||
	  (t1->tv_sec == t2->tv_sec &&
	   t1->tv_usec < t2->tv_usec)) ? TRUE : FALSE);
}

/******************************************************************************
 *
 * FUNCTION: void addTime (struct timeval *t1,
 *                         struct timeval *t2,
 *                         struct timeval *result)
 *
 * DESCRIPTION: 
 * Adds t1 to t2 and stores the result in  result.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 *****************************************************************************/

static void addTime(struct timeval *t1,
		    struct timeval *t2,
		    struct timeval *result)
{
  result->tv_usec = t1->tv_usec + t2->tv_usec;
  result->tv_sec = t1->tv_sec + t2->tv_sec;
  /* do a carry if needed */
  if (result->tv_usec > 1000000) {
    result->tv_usec -= 1000000;
    result->tv_sec += 1;
  }
}

/******************************************************************************
 *
 * FUNCTION: void subTime (struct timeval *t1, struct timeval *t2)
 *
 * DESCRIPTION: 
 * subtracts t2 from t1.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 *****************************************************************************/

static void subTime(struct timeval *t1, struct timeval *t2)
{
  t1->tv_usec -= t2->tv_usec;
  t1->tv_sec -= t2->tv_sec;
  /* do a borrow if needed */
  if (t1->tv_usec < 0) {
    t1->tv_usec += 1000000;
    t1->tv_sec -= 1;
  }
}


/******************************************************************************
 *
 * FUNCTION: void devStartPolling(DEV_PTR dev,
 *                                struct timeval *interval,
 *                                void (* handler)(void))
 *
 * DESCRIPTION: 
 * Start polling the device by calling the given hander every interval 
 * milliseconds.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 *****************************************************************************/

void devStartPolling(DEV_PTR dev,
		     struct timeval *interval,
		     void (* handler)(void))
{
  struct timeval now;
  
  dev->pollInterval = *interval;
  gettimeofday (&now, 0); 
  addTime(&now, &(dev->pollInterval), &(dev->pollTime));
  dev->pollHnd = handler;
}



/******************************************************************************
 *
 * FUNCTION: void devMainLoop(void)
 *
 * DESCRIPTION: 
 * This is the main loop for collecting and processing input from all the
 * devices.  This routine should never return.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 *****************************************************************************/

void devMainLoop(void)
{
  for(;;)
    ProcessDevices();
}

/******************************************************************************
 *
 * FUNCTION: void ProcessDevices(void)
 *
 * DESCRIPTION: 
 * Loop listening to all the open connections.  Call the device output 
 * handling routines and timeout routines as needed.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 *****************************************************************************/

void ProcessDevices(void)
{
  int i, numReady;
  struct timeval timeout;
  struct timeval now;
  fd_set readMask;
  struct timeval TCX_waiting_time = {0, 0};


  /*
    int loop;
    struct timeval idle_time;
    */
  /* set timeout */
  timeout.tv_sec = LONG_MAX;
  timeout.tv_usec = 0;
  for(i=0; i<maxDevNum; i++){
    if (devices[i] != NULL) {
      if (lessTime(&(devices[i]->timeOutTime),&timeout)) {
	timeout = devices[i]->timeOutTime;
      }
      if (lessTime(&(devices[i]->pollTime),&timeout)) {
	timeout = devices[i]->pollTime;
      }
    }
  }

  gettimeofday (&now, 0);
  /*
    idle_time = now;
    */
  if(lessTime(&now,&timeout)) 
    {
      /* have time to do a select */
      subTime(&timeout,&now);
      if(lessTime(&MAX_TIMEOUT, &timeout))
	timeout = MAX_TIMEOUT;
    }
  else 
    timeout = ZERO_TIMEOUT;




  /*====================================================================
   *
   * Sebastian 94-1-13: This code takes tcx events into account. 
   * (warning: it might not work for the simulator...)
   */
  

  if (!listen_for_tcx_events || Global == NULL){
    /***** previous version *******************/
    bcopy(&devConnections,&readMask, sizeof(fd_set));
    numReady = select(maxDevNum, &readMask, NULL, NULL, &timeout);
  }
  else{
    readMask = (Global->tcxConnectionListGlobal);/* current tcx socket list */
    for (i = 0; i < maxDevNum; i++) /* add all devices */
      if (devices[i] != NULL)
	FD_SET(i, &readMask);
    numReady = select(FD_SETSIZE, &readMask, NULL, NULL, &timeout);
    tcxRecvLoop((void *) &TCX_waiting_time); /* check for TCX messages */
  }

  /*
   *====================================================================
   */


  /*
    gettimeofday (&now, 0);
    printf("%d.%d", now.tv_sec, now.tv_usec);
    subTime(&now, &idle_time);
    printf(" %f\n", now.tv_sec + (now.tv_usec / 1000000.0));
    */
  if (numReady < 0) {
    fprintf(stderr, "Error on select %d\n", errno);
  } else {
    if (numReady > 0)
      { 
	for(i=0; i<maxDevNum; i++) {
	  if (FD_ISSET(i,&readMask)) {
	    long chars_available;
	    chars_available = numChars(i);
	    (* (devices[i]->outputHnd))(i, chars_available);
	  }
	}
      }
  }

  /* handle any timeouts */
  gettimeofday (&now, 0);
  for(i=0; i<maxDevNum; i++)
    if (devices[i] != NULL) {
      if (lessTime(&(devices[i]->timeOutTime),&now)) {
	(* (devices[i]->timeoutHnd))();
      }
      if (lessTime(&(devices[i]->pollTime),&now)) {
	(* (devices[i]->pollHnd))();
	/* set next time to poll */
	addTime(&now, &(devices[i]->pollInterval), 
		&(devices[i]->pollTime));
      }
    }
}


/******************************************************************************
 *
 * FUNCTION: void ProcessSingleDevice(int i)
 *
 * DESCRIPTION: 
 * Loop listening to a single open connection.  Call the device output 
 * handling routines. This procedure is NOT blocking!!!
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 *****************************************************************************/

void ProcessSingleDevice(DEV_PTR dev)
{
  int numReady;
  int dev_no;
  struct timeval no_timeout = {0, 0};
  fd_set readMask;

  dev_no = dev->fd;

  FD_ZERO(&readMask);
  FD_SET(dev_no, &readMask);
  numReady = select(maxDevNum, &readMask, NULL, NULL, &no_timeout);

  if (FD_ISSET(dev_no,&readMask)) {
    long chars_available;
    chars_available = numChars(dev_no);
    (* (devices[dev_no]->outputHnd))(dev_no, chars_available);
  }
}


/*****************************************************************************
 *
 * FUNCTION: void stdin_defaultInputHnd(int fd, int chars_available)
 *
 * DESCRIPTION: Handles character typed from stdin.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

void stdin_defaultInputHnd(int fd, long chars_available)
{
  static char buffer[DEFAULT_LINE_LENGTH+1];
  int numRead=0, i;
  
  bzero(buffer, DEFAULT_LINE_LENGTH+1);
  
  numRead = readN(&stdin_device, buffer, 
		  MIN(chars_available,DEFAULT_LINE_LENGTH));
  for (i=0; i<numRead; i++)
    if (buffer[i] == 'q')
      {
	printf("Quit command from console.\n");
	exit(0);
      }
}

/*****************************************************************************
 *
 * FUNCTION: LINE_BUFFER_PTR createLineBuffer(int lineLength, char delimChar,
 *				 void (*processRoutine)(char *))
 *
 * DESCRIPTION: Creates a buffer data structure that is used to collect
 * input until one of the delimit characters is read.  The processing 
 * routine is then called.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

LINE_BUFFER_PTR createLineBuffer(int lineLength, char delimChar,
				 void (*processRoutine)(char *))
{
  int length;
  LINE_BUFFER_PTR lineBuffer;
  
  length = lineLength*2;
  lineBuffer = (LINE_BUFFER_PTR)calloc((size_t) sizeof(LINE_BUFFER_TYPE),1);
  lineBuffer->buffer = calloc((size_t) length,1);
  bzero(lineBuffer->buffer, length);
  lineBuffer->length = length;
  lineBuffer->nextChar = 0;
  lineBuffer->delimChar = delimChar;
  lineBuffer->processRoutine = processRoutine;
  
  return lineBuffer;
}

/*****************************************************************************
 *
 * FUNCTION: void processOutput (DEV_PTR device, LINE_BUFFER_PTR lineBuffer, 
 *                               int chars_available)
 *
 * DESCRIPTION:
 *  Collects characters from the device into the line buffer.
 *  Calls the processRoutine if the delimChar is found, with the delimChar 
 *  character replaced by \0, to indicate end of line.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

void processOutput (DEV_PTR device, LINE_BUFFER_PTR lineBuffer, 
		    int chars_available)
{
  char *lineEnd;
  int numRead, remainChar;
  
  while (chars_available > 0) {
    /* read in the output */
    numRead = readN(device, &(lineBuffer->buffer[lineBuffer->nextChar]),
		    MIN(chars_available,
			(lineBuffer->length - lineBuffer->nextChar)));
    if (numRead == 0) {
      /* Handle error here. The port is closed */
      exit (-1);
    } else {
      lineBuffer->nextChar += numRead;
      for(lineEnd = lineBuffer->buffer; *lineEnd != lineBuffer->delimChar;
	  lineEnd++)
	if (*lineEnd == '\0') {
	  lineEnd = NULL;
	  break;
	}
      /* lineEnd = strpbrk(lineBuffer->buffer, lineBuffer->delimSet); */
      while (lineEnd != NULL) {
	*lineEnd = '\0';
	if (device->debug) {
	  fprintf(stderr, "Received: %s\n", lineBuffer->buffer);
	}
	(lineBuffer->processRoutine)(lineBuffer->buffer);
	remainChar = lineBuffer->nextChar - (lineEnd+1 - lineBuffer->buffer);
	bcopy(&(lineEnd[1]), lineBuffer->buffer, remainChar);
	bzero(&(lineBuffer->buffer[remainChar]),
	      lineBuffer->nextChar - remainChar);
	lineBuffer->nextChar = remainChar;
	for(lineEnd = lineBuffer->buffer; *lineEnd != lineBuffer->delimChar;
	    lineEnd++)
	  if (*lineEnd == '\0') {
	    lineEnd = NULL;
	    break;
	  }
	/* lineEnd = strpbrk(lineBuffer->buffer, lineBuffer->delimSet); */
      }
      chars_available -= numRead;
    }
  }
}

/*****************************************************************************
 *
 * FUNCTION: BOOLEAN WaitForResponse(DEV_PTR dev, int *doneFlag, long time_out)
 *
 * DESCRIPTION:
 * The following function allows to wait within an output handler
 * and keep on reading the devices. 
 *
 * IMPORTANT!!!: The code of the output handler that is waiting must 
 * be reentrant for the function to work. Otherwise, the device corresponding
 * to that output handler won't process properly the requests until
 * the waiting ends.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

BOOLEAN WaitForResponse(DEV_PTR dev, int *doneFlag, long time_out)
{
  struct timeval now;
  
  if (time_out != 0) {
    gettimeofday (&now, 0);
    now.tv_sec += time_out;
    if (global_time_out.tv_sec == 0)
      global_time_out = now;
    else {
      if (lessTime(&global_time_out, &now))
	global_time_out = now;
    }
  }
  *doneFlag = WAITING;
  waiting++;
  while (*doneFlag == WAITING) {
    ProcessDevices();
    if (global_time_out.tv_sec != 0) {
      gettimeofday (&now, 0);
      if (lessTime(&global_time_out, &now)) {
	printf ("%s:%6d:%s() - Time out\n",
		__FILE__, __LINE__, __FUNCTION__);
	waiting--;
	if (waiting == 0)
	  global_time_out.tv_sec = 0;
	return(FALSE);
      }
    }
  }
  waiting--;
  if (dev->debug) fprintf(stderr, "Exiting from waiting\n");
  if (waiting == 0)
    global_time_out.tv_sec = 0;
  return(TRUE);
}

/*****************************************************************************
 *
 * FUNCTION: BOOLEAN noinput()
 *
 * DESCRIPTION:
 * A routine to check to see if there is any input available on stdin.
 * Returns TRUE if there is, FALSE otherwise.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

BOOLEAN noinput(void)
{
  long n=0;
  if(ioctl(fileno(stdin), FIONREAD, &n) == 0 ) {
    return( n==0 );
  }
  perror("ioctl");
  return FALSE;
}
