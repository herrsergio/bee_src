
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



/**********************************************************************
 *
 *  PROJECT:  TCX
 *
 * PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
 * (c) Copyright 1993 Christopher Fedor. All rights reserved.
 *
 *  MODULE: Communications
 *
 *  FILE: comA.c
 *
 *  ABSTRACT: Unix Communications  
 *
 *  EXPORTS:
 *
 *  HISTORY:
 *  $Log: comA.c,v $
 *  Revision 1.9  1998/02/27 15:34:52  arbuckle
 *  *** empty log message ***
 *
 *  Revision 1.8  1998/02/07 23:22:31  swa
 *  2nd attempt.
 *
 *  Revision 1.7  1998/02/07 23:17:59  swa
 *  now works with redhat5.0
 *
 *  Revision 1.6  1997/04/11 18:56:56  tyson
 *  minor fixes and chasing TCX segv
 *
 *  Revision 1.5  1997/04/01 22:30:36  tyson
 *  bugs and stuff
 *
 *  Revision 1.4  1997/03/11 17:16:46  tyson
 *  added IR simulation and other work
 *
 *  Revision 1.3  1997/02/22 15:43:00  thrun
 *  Fixed some problems that caused compiler warnings.
 *
 *  Revision 1.2  1996/10/28 15:34:47  ws
 *  Now TCX clients and server can connect even when located in different
 *  internet domains.
 *
 *  Revision 1.1.1.1  1996/09/22 16:46:00  rhino
 *  General reorganization of the directories/repository, fusion with the
 *  RWI software.
 *
 *  Revision 1.5  1994/10/22 18:47:24  tyson
 *  VMS version. Fixed structure indexing (computation of the offsets
 *  in a struct). Added signal handlers to a1, b1 tcxServer.
 *
 * Revision 1.4  1994/07/26  03:52:17  thrun
 * Lots of improvements. Several bugs left.
 *
 * Revision 1.3  1994/07/10  18:24:22  thrun
 * changed it so that is runs with Linux, too.
 *
 * Revision 1.2  1994/07/10  16:37:53  thrun
 * Introduced buffer for messages, asynchroneous communication.
 * Thanks to Wolli!
 *
 * Revision 1.1.1.1  1994/03/31  08:35:04  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:28  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:28  rhino
 * test
 *
 * Revision 1.15  1993/03/12  20:58:17  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.14  1992/11/13  14:47:09  fedor
 * Update archive with a working version and partially working server route
 *
 * Revision 1.13  1992/10/23  20:15:11  fedor
 * Redid method of connecting to modules. Added the notion of a connection.
 * Added some auto-reconnect for those modules that are probably listening.
 * See detail notes.
 *
 * Revision 1.12  1992/10/14  18:30:53  fedor
 * Changed tcxRecvData to execute at least a single handler from one connection or
 * at most one handler from each connection and return.
 * Changed tcxRecvLoop to loop here instead of in tcxRecvData - needs clean up.
 *
 * Revision 1.11  1992/10/08  14:38:03  fedor
 * Remove oops error by adding connection confirmation.
 * Establish all pending connections in tcxInitialize by waiting for a
 * confirmation of connection before sending version information.
 *
 * Revision 1.10  1992/09/12  23:12:26  fedor
 * Fix inifinte loop problem with EWOULDBLOCK in writeAll. Still a mess.
 *
 * Revision 1.9  1992/09/12  21:53:06  fedor
 * Connect when first message received. Moving towards version 5.x
 *
 * Revision 1.8  1992/08/04  14:45:49  fedor
 * Debugging fprintf for VxWorks.
 * Added checks for using NULL formatters.
 *
 * Revision 1.7  1992/07/20  00:53:42  fedor
 * Redefined tcxRecvData to check pending first. Commented out some debugging.
 *
 * Revision 1.6  1992/07/20  00:31:34  fedor
 * Added receive message style calls. Added data freeing calls.
 *
 * Revision 1.5  1992/07/10  16:04:48  fedor
 * Changes to compile for VxWorks. Also some changes to avoid compiler warnings.
 *
 * Revision 1.4  1992/07/05  15:41:33  fedor
 * Added random makefile.NeXT for NeXT machine use.
 * Changed incorrect use of perror to fprintf(stderr,
 *
 * Revision 1.3  1992/07/05  13:06:38  fedor
 * Changed printf to fprintf(stderr,
 * Removed old list. Some file reorganization.
 *
 * Revision 1.2  1992/07/03  10:13:09  fedor
 * New module definition and routines
 *
 * Revision 1.1.1.1  1992/06/16  17:22:00  fedor
 * Import TCX 2.0 16-Jun-92
 *
 * Revision 1.4  1992/04/28  02:14:38  fedor
 * module socket iteration
 *
 * Revision 1.3  1992/04/20  10:37:56  fedor
 * *** empty log message ***
 *
 * Revision 1.2  1992/04/20  10:30:26  fedor
 * test file
 *
 * Revision 1.2  1992/04/20  10:30:26  fedor
 * test file
 *
 * Revision 1.1  1992/04/20  07:05:42  fedor
 * Initial revision
 *                  
 *  
 *  $Revision: 1.9 $
 *  $Date: 1998/02/27 15:34:52 $
 *  $Author: arbuckle $
 *
 *********************************************************************/

#if 0
#define pgmTrace() fprintf(stderr, "%s:%6d:%s()\n", \
			   __FILE__, __LINE__, __FUNCTION__)

#define pgmReturn() fprintf(stderr, "%s:%6d:%s() return\n", \
			   __FILE__, __LINE__, __FUNCTION__)
#else
#define pgmTrace() {}
#define pgmReturn() {}
#endif

#define TCP 6      /* from etc/protocols or getprotoent call */
#define BACKLOG 5  /* backlog for listen is pretty much limited to 5 anyway */


#include <fcntl.h>

#ifdef VMS
#include "vms.h"
#endif

#ifdef VXWORKS
#include "/usr/vxworks/vx5.0.2b/h/vxWorks.h"
#include "/usr/vxworks/vx5.0.2b/h/stdioLib.h"
#include "/usr/vxworks/vx5.0.2b/h/ctype.h"
#include "/usr/vxworks/vx5.0.2b/h/socket.h"
#include "/usr/vxworks/vx5.0.2b/h/in.h"
#include "/usr/vxworks/vx5.0.2b/h/sigLib.h"
#include "/usr/vxworks/vx5.0.2b/h/errno.h"
#else
#include "stdio.h"
#include "ctype.h"
#include "signal.h"
#include "sys/time.h"
#include "sys/types.h"
#include "sys/socket.h"
#include "netinet/in.h"
#include "netinet/tcp.h"
#include "netdb.h"
#include "errno.h"
#endif



#include "tcx.h"
#include "tcxP.h"

#include "com.h"
#include "list.h"

#include "msg.h"
#include "module.h"

#include "global.h"

extern MODULE_PTR moduleCreate();

static int warning_printed = 0;

#ifndef i386
extern char* sys_errlist[];	/* for redhat 5.0, swa */
#endif

/**********************************************************************
 *
 *  FUNCTION:  int readAll(int sd, void *buf, int nbytes)
 *
 *  DESCRIPTION: 
 *
 *		Read the specified number of bytes, nbytes, from the
 * socket descriptor sd into the preallocated buffer, buf.  
 *
 * Return 1 if successful. Otherwise < 0.
 *
 *********************************************************************/

int readAll(int sd, char *buf, int nbytes)
{
  int amountRead = 0;

  /* printf("=1(%d)=",nbytes);fflush(stdout);(S.Thrun 93-5-1) */

  while (nbytes > 0) {

    /* printf("=2=");fflush(stdout);(S.Thrun 93-5-1) */

    do {
      amountRead = read(sd, buf, nbytes);
    } while (amountRead < 0);

    /* printf("=3=");fflush(stdout);(S.Thrun 93-5-1) */

    if (amountRead < 0) {
      return -2; /* ERROR */
    }

    /* printf("=4=");fflush(stdout);(S.Thrun 93-5-1) */
    if (!amountRead) {
      return -1; /* EOF */
    }

    /* printf("=5=");fflush(stdout);(S.Thrun 93-5-1) */

    nbytes -= amountRead;
    buf += amountRead;

    /* printf("=6=");fflush(stdout);(S.Thrun 93-5-1) */
  }

  /* printf("=7=");fflush(stdout);(S.Thrun 93-5-1) */
  
  return 1; /* OK */
}    

/**********************************************************************
 *
 *  FUNCTION:  int writeAll(int sd, void *buf, nbytes)
 *
 *  DESCRIPTION: 
 *
 *		Write the amount, nbytes of buf to socket descriptor sd.
 * writeAll will attempt to write everything to the socket sd, there may be 
 * a problem with interrupt signals.
 *
 * Returns 1 on success. Otherwise < 0. 
 *
 * NOTES:
 * 12-Sep-92: fedor: Be careful that EWOULDBLOCK error does not cause
 * a negative value to be subtracted from nbytes. Loop on problem for now.
 *
 * 20-Apr-92: fedor: EWOULDBLOCK error looks like a problem in general.
 *    if (amountWritten < 0 && errno != EWOULDBLOCK)
 *      return -1;
 *
 * Do not like this yet.
 *
 *********************************************************************/

int writeAll(int sd, char *buf, int nbytes)
{
  int amountWritten = 0;

#if 0
  fprintf(stderr, "%s:%6d:%s()\n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif
#if 0
  {
    int i;
    unsigned char *buff;
    buff = (unsigned char *) buf;
    printf("[");
    for (i = 0; i < nbytes; i++) {
      printf("%.2x",  buff[i]);
      if ((i + 1) % 4 == 0)
	printf(" ");
    }
  }
  printf("]\n");fflush(stdout); /* (S.Thrun 93-5-1) */
#endif

  while (nbytes > 0) {

    amountWritten = write(sd, buf, nbytes);
    if (amountWritten < 0) {
#if 0
      fprintf(stderr, "%s:%6d:%s() - amountWritten: %d: sd: %d: nbytes: %d\n",
	      __FILE__, __LINE__, __FUNCTION__,
	      amountWritten, sd, nbytes);
#endif
      if (errno == EWOULDBLOCK) {
#if 0
	fprintf(stderr, "%s:%6d:%s() - EWOULDBLOCK: Trying Again!\n",
		__FILE__, __LINE__, __FUNCTION__);
#endif
      }
      else {
#if 0
	fprintf(stderr, "%s:%6d:%s() - ERROR: errno: %d\n", 
		__FILE__, __LINE__, __FUNCTION__, errno);
#endif
	return -1;
      }
    } else {
      nbytes -= amountWritten;
      buf += amountWritten;
    }
  }
  
  return 1;
}

/**********************************************************************
 *
 *  FUNCTION:  int connectAtPort(char *machine, int port, int *sd)
 *
 *  DESCRIPTION: 
 *
 *		Establish a socket connection to the specified machine
 * at a specific port and store the created socket descriptor sd.
 *
 * Return 1 on success, else 0. 
 *
 *********************************************************************/

int connectAtPort(char *machine, int port, int *sd)
{
#ifndef VXWORKS
  struct hostent *hp;
#endif


  int value;
  struct sockaddr_in server;

  pgmTrace();

  bzero((char *)&server, sizeof(struct sockaddr_in));

  server.sin_family = AF_INET;
  server.sin_port = htons(port);

  /* fprintf(stderr, "connectAtPort: look for machine: %d:%s\n", 
     port, machine);*/

  if (isdigit(machine[0])) {
    /* printf("digit! \n");*/
    server.sin_addr.s_addr = inet_addr(machine);
  } else {
#ifndef VXWORKS
    if ((hp = gethostbyname(machine)) == NULL) {
      /*  fprintf(stderr, "connectAtPort: gethostbyname: %s\n", machine); */
      return 0;
    }
    bcopy(hp->h_addr, &server.sin_addr, hp->h_length);
#else
    if ((server.sin_addr.s_addr = hostGetByName(machine)) == ERROR) {
      /*  fprintf(stderr, "connectAtPort: hostGetByName: %s\n", machine); */
      return 0;
    }
#endif
  }

  /* fprintf(stderr, "connectAtPort: found machine: %d:%s\n", port, machine);*/

  if ((*sd = socket(AF_INET, SOCK_STREAM, TCP)) < 0) {
    /*  fprintf(stderr, "connectAtPort: ERROR: socket\n");*/
    return 0;
  }

  /******* 17-aug-91: fedor: causes NeXT machine to fail connecting **
  if ((setsockopt(*sd, SOL_SOCKET, SO_REUSEADDR, (char *)0, 0)) < 0)
    return 0;
    *****/
 value = 50000;
 if (setsockopt(*sd,SOL_SOCKET,SO_SNDBUF,(char*)&value,sizeof(int)) <0 ){
    fprintf(stderr, "\nWOLLI-ERROR 1 [%s]", sys_errlist[errno]);
     return 0;
  }
 if (setsockopt(*sd,SOL_SOCKET,SO_RCVBUF,(char*)&value,sizeof(int)) <0 ){
    fprintf(stderr, "\nWOLLI-ERROR 2  [%s]", sys_errlist[errno]);
     return 0;
  }
#ifdef old_stuff_wont_work
  if (fcntl(*sd, F_SETFL, O_NDELAY | FASYNC) < 0){
    fprintf(stderr, "\nWOLLI-ERROR 3  [%s]", sys_errlist[errno]);
    return 0;			/* Wolli */
  }
#endif
#ifndef VXWORKS
  value = 1;
  if ((setsockopt(*sd, TCP, TCP_NODELAY, (char*)&value, sizeof(int))) < 0)
    if (errno != 95)		/* 95 means only: not supported */
      return 0;
    else if (!warning_printed){
      fprintf(stderr, "WARNING: TCP protocol option TCP_NODELAY not supported.\n");
      warning_printed = 1;
    }
#endif

  /* 24-Aug-90: fedor: VxWorks sockaddr_in is defined differently
     this should be checked to make a VxWorks version. */
  if (connect(*sd, (struct sockaddr *)&server, sizeof(server)) < 0) {
    /* fprintf(stderr, "connectAtPort: ERROR: connect\n");*/
    return 0;
  }

  return 1;
}

/**********************************************************************
 *
 *  FUNCTION:  int listenAtPort(int *port, int *sd)
 *
 *  DESCRIPTION: 
 *
 *		Create a socket for listening for new connections at a given
 * port value. If the value of port is 0, return the value chosen in port.
 *
 * Return 1 on success, else 0.
 *
 * NOTES:
 * 7-Jun-91: fedor: may need some changes for vxworks 
 *
 *********************************************************************/

int listenAtPort(int *port, int *sd)
{
  int value, value2;
  int reuse, length;

  struct sockaddr_in server;

  pgmTrace();

  /* printf("listenAtPort: %d\n", *port);*/

  bzero((char *)&server,  sizeof(struct sockaddr_in));  


  server.sin_port = htons(*port);

  server.sin_family = AF_INET;
  server.sin_addr.s_addr = INADDR_ANY;

  if ((*sd = socket(AF_INET, SOCK_STREAM, TCP)) < 0)
    return 0;

  reuse = 1;

  if ((setsockopt(*sd, SOL_SOCKET, SO_REUSEADDR, (char*)&reuse, sizeof(int))) < 0)
    return 0;
  
  value2 = 50000;		/* Wolli */

  if (setsockopt(*sd,SOL_SOCKET,SO_SNDBUF,(char*)&value2,sizeof(int)) <0 ){
    fprintf(stderr, "\nWOLLI-ERROR 6 [%s]", sys_errlist[errno]);
    return 0;		/* Wolli */
  }
   
  if (setsockopt(*sd,SOL_SOCKET,SO_RCVBUF,(char*)&value2,sizeof(int)) <0 ){
    fprintf(stderr, "\nWOLLI-ERROR 7  [%s]", sys_errlist[errno]);
    return 0;	 		/* Wolli */
  }
   
#ifdef old_stuff_wont_work
  if (fcntl(*sd, F_SETFL, O_NDELAY | FASYNC) < 0){
    fprintf(stderr, "\nWOLLI-ERROR 8  [%s]", sys_errlist[errno]); 
    return 0;			/* Wolli */
  }
#endif
  reuse = 1;

#ifndef VXWORKS
  value = 1;
  if ((setsockopt(*sd, TCP, TCP_NODELAY, (char*)&value, sizeof(int))) < 0) {
    if (errno != 95) {		/* 95 means only: not supported */
      fprintf(stderr, "\nERROR - setsockopt: [%s]\n", sys_errlist[errno]);
      return 0;
    }
    else if (!warning_printed){
      fprintf(stderr, "WARNING: TCP protocol option TCP_NODELAY not supported.\n");
      warning_printed = 1;
    }
  }
#endif
  
  if ((bind(*sd, (struct sockaddr *)&server, sizeof(struct sockaddr_in))) < 0) {
    fprintf(stderr, "\nERROR - bind [%s]\n", sys_errlist[errno]);
    return 0;
  }

  if (listen(*sd, BACKLOG) < 0) {
    fprintf(stderr, "\nERROR - listen: [%s]\n", sys_errlist[errno]);
    return 0;
  }

  /* find what port is if not set */
  if (!*port) {
    length = sizeof(server);
    if (getsockname(*sd, (struct sockaddr *) &server, &length) < 0) {
      fprintf(stderr, "\nERROR - getsockname: [%s]\n", sys_errlist[errno]);
      return 0;
    }
    *port = ntohs(server.sin_port);
  }

  return 1;
}

/**********************************************************************
 *
 *  FUNCTION:  int connectToModule(char *mod, char *host, int port)
 *
 *  DESCRIPTION: 
 *
 *             Iterate connectAtPort
 *
 * Return the socket descriptor of the new connection.
 *
 *********************************************************************/

int connectToModule(char *mod, char *host, int port)
{
  int sd;

  pgmTrace();

  if (!connectAtPort(host, port, &sd)) {
    fprintf(stderr, "Looking for %s on %s ", mod, host);
    fflush(stderr);

    while (!connectAtPort(host, port, &sd)) {
      fprintf(stderr, ".");
      fflush(stderr);
#ifdef VXWORKS
      taskDelay(60*2);
#else
      sleep(2);
#endif
    }
    fprintf(stderr, "found.\n");
  }

  return sd;
}

/**********************************************************************
 *
 *  FUNCTION:  iterateActiveMods
 *
 *  DESCRIPTION: 
 *
 *		Handle new connections and message reading / queueing.
 *
 *********************************************************************/

int iterateActiveMods(readMask, module)
fd_set *readMask;
MODULE_PTR module;
{
  char *buf;
  HDR_TYPE hdr;
  int sd, stat, total;
  MODULE_PTR newModule;
  MSG_INS_PTR item1, item2;

  /* printf("=A=");fflush(stdout);(S.Thrun 93-5-1) */
  if ((module->status & STATUS_ACTIVE) && FD_ISSET(module->sd, readMask)) {

    /* fprintf(stderr, "active module: %s\n", module->name);*/

    /* printf("=B=");fflush(stdout);(S.Thrun 93-5-1) */
    if (module == Global->tcxModuleGlobal) {
      sd = accept(Global->tcxModuleGlobal->sd, NULL, NULL);
      module = moduleCreate(sd, 0, NULL, NULL, STATUS_CONNECTED);
    } else {
      sd = module->sd;
    }


    /* printf("=C=");fflush(stdout);(S.Thrun 93-5-1) */
    if ((stat = readAll(sd, (char *) &hdr, sizeof(HDR_TYPE))) < 0) {
      moduleClose(module);

      return TRUE;
    }

    /* printf("=D=");fflush(stdout);(S.Thrun 93-5-1) */
    total = ntohInt(hdr.size1)+ntohInt(hdr.size2);

    if (!total) {
      /* printf("=E=");fflush(stdout);(S.Thrun 93-5-1) */
      buf = NULL;
    } else {
      /* printf("=F=");fflush(stdout);(S.Thrun 93-5-1) */
      buf = (char *)malloc(total);
      /* printf("=FF=");fflush(stdout);(S.Thrun 93-5-1) */
      if (readAll(sd, buf, total) < 0) {
	/* printf("=FFF=");fflush(stdout);(S.Thrun 93-5-1) */
	fprintf(stderr, "ERROR: iterateActiveM: readAll 3\n");
	(*(Global->tcxExitHndGlobal))();
      }
    }

    /* printf("=G=");fflush(stdout);(S.Thrun 93-5-1) */
    item1 = (MSG_INS_TYPE *)malloc(sizeof(MSG_INS_TYPE));
    item1->id = ntohInt(hdr.msg1);
    item1->ref = ntohInt(hdr.ref1);
    item1->len = ntohInt(hdr.size1);
    item1->module = module;
    
    /* printf("=H=");fflush(stdout);(S.Thrun 93-5-1) */
    if (!ntohInt(hdr.msg2)) {
      /* printf("=I=");fflush(stdout);(S.Thrun 93-5-1) */
      item1->encodedData = buf;
      listInsertItemLast(item1, Global->dataListGlobal);
    } else {
      /* printf("=J=");fflush(stdout);(S.Thrun 93-5-1) */
      item2 = (MSG_INS_TYPE *)malloc(sizeof(MSG_INS_TYPE));
      item2->id = ntohInt(hdr.msg2);
      item2->ref = ntohInt(hdr.ref2);
      item2->len = ntohInt(hdr.size2);
      item2->module = module;
      
      if (!ntohInt(hdr.size2)) {
	/* printf("=K=");fflush(stdout);(S.Thrun 93-5-1) */
	item1->encodedData = buf;
	item2->encodedData = NULL;
      } else {
	/* printf("=L=");fflush(stdout);(S.Thrun 93-5-1) */
	item1->encodedData = (char *)malloc(ntohInt(hdr.size1));
	bcopy(buf, item1->encodedData, ntohInt(hdr.size1));
	
	item2->encodedData = (char *)malloc(ntohInt(hdr.size2));
	bcopy(buf+ntohInt(hdr.size1), item2->encodedData, ntohInt(hdr.size2));
	/* printf("=M=");fflush(stdout);(S.Thrun 93-5-1) */
	  
	free(buf);
      }
      /* printf("=N=");fflush(stdout);(S.Thrun 93-5-1) */

      listInsertItemLast(item1, Global->dataListGlobal);
      /* printf("=O=");fflush(stdout);(S.Thrun 93-5-1) */
      listInsertItemLast(item2, Global->dataListGlobal);
      /* printf("=P=");fflush(stdout);(S.Thrun 93-5-1) */
    }
  }

  return 1;
}

/**********************************************************************
 *
 *  FUNCTION:  int singlePassActiveModules(struct timeval *timeout)
 *
 *  DESCRIPTION: 
 *
 *            Single Pass on module list reading in active messages.
 *
 * Return 1 on success for reading in at least one message.
 * Return 0 on timeout, if timeout was not NULL.
 * Return -1 on error.
 *
 *  NOTES:
 * 
 * 20-Apr-92: fedor: 
 * I wonder how often more than one module will have something pending.
 *
 *********************************************************************/

int singlePassActiveModules(void *timeout)
{
  int stat;
  fd_set readMask;

  pgmTrace();

  readMask = (Global->tcxConnectionListGlobal);
      
  do {
    stat = select(FD_SETSIZE, &readMask, NULL, NULL, timeout);
  } while (stat < 0 && errno == EINTR);
  
  if (stat <= 0){
    pgmReturn();
    return stat;
  }
      
  listIterateFromFirst(iterateActiveMods, &readMask, Global->moduleListGlobal);

  pgmReturn();

  return 1;
}
