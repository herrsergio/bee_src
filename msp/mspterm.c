
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



/*********************************************
 *
 *   mspterm.c  - msp section for mspterm
 *
 *********************************************/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#include <termios.h>
#include <curses.h>
#include <netinet/in.h>  /* for byteorder */

#include <acb/abus.h>

#include "abterm.h"
#include "msputils.h"

#include "msp.h"

static mspOpsType mspOps;


/**************************
 *    User commands
 **************************/

static int
reqVer (int argc, char *argv[], long mspNum) {

  mspReqVer(mspNum);
  return 0;
}

static int
setIri (int argc, char *argv[], long mspNum) {
  mspIrParmsType parm;

  memset(&parm, 0xFF, sizeof(parm)); /* initialize all elements to -1 */

  if (argc == 1) {		/* if no args, request parms */
    mspReqIrParms(mspNum);
    return(0);
  }

  parm.interval = strtoul (argv[1], NULL, 16);

  mspSetIrParms(mspNum, &parm);
  return (0);
}

static int
reqSon (int argc, char *argv[], long mspNum) {
  unsigned long table[20];
  int count;
  
  if (argc<2) {
    wprintw (termWin,
	     "usage: sonreq <sonar> [sonar] [sonar] ...\n");
    
    wrefresh(termWin);
    return (0);
  }
  
  for (count=1; count<argc; count++) {
    table[count-1] = strtoul(argv[count], NULL, 16);
  }
  table[argc] = 0;

  mspReqSon(mspNum, table);
  return (0);
}

static int
sonTable (int argc, char *argv[], long mspNum) {
  unsigned int set, xducer;
  int count;
  const unsigned long *table[10];
  unsigned long table0[10][10];
  int ii;
  
  for (ii = 0; ii<10; ii++) table[ii] = &table0[ii][0];

  if (argc<2) {
    mspReqSonTable(mspNum);
    return (0);
  }
   
  if ((!strcmp(argv[1], "-h")) || (!strcmp(argv[1], "--help"))) {
    wprintw (termWin,
	     "usage: sontable [sonar] [sonar] "
	     "[...] [: sonar] [sonar] [...]\n");
    wprintw (termWin, "\tNo args will return current table.\n");
    wrefresh(termWin);
    return (0);
  }
   
  set = 0;
  xducer = 0;
  for (count=1; count<argc; count++) {
    if (strcmp (argv[count], ":") == 0) {
      table0[set++][xducer] = 0;
      xducer = 0;
    }
    else {
      table0[set][xducer++] = strtoul (argv[count], NULL, 16);
    }
  }
  table0[set][xducer] = 0;
  if (xducer) {
    set++;
  }
  table[set] = NULL;	/* NULL to end table */

  mspSetSonTable(mspNum, table);
  return (0);
}

static int
reqSonStart (int argc, char *argv[], long mspNum) {

  mspReqSonStart(mspNum);
  return (0);
}

static int
reqSonStop (int argc, char *argv[], long mspNum) {

  mspReqSonStop(mspNum);
  return (0);
}

typedef struct {
  char *arg;
  unsigned short parm;
} _sp_cmdarg;

#define MSPT_EC    0
#define MSPT_ETO   1
#define MSPT_FD    2
#define MSPT_FI    3
#define MSPT_EBT   4
#define MSPT_IBT   5
#define MSPT_SD    6

static _sp_cmdarg sonparms_[] = {
  {"-ec",     MSPT_EC},
  {"-eto",    MSPT_ETO},
  {"-fd",     MSPT_FD},
  {"-fi",     MSPT_FI},
  {"-ebt",    MSPT_EBT},
  {"-ibt",    MSPT_IBT},
  {"-sd",     MSPT_SD},
  
  {NULL, 0}
};


static int
_sonparms_getparm(char *arg)
{
  int count;
  for (count=0; sonparms_[count].arg != NULL; count++)
    if (strcmp(sonparms_[count].arg, arg)==0)
      return sonparms_[count].parm;
  return -1;
}

static int
setSonParms (int argc, char *argv[], long mspNum) {
  mspSonParmsType parm;
  int parmNum;
  int count, argcount;
  
  if (argc<2) {
    mspReqSonParms(mspNum);
    return (0);
  }
  
  if ((!strcmp(argv[1], "-h")) || (!strcmp(argv[1], "--help"))) {
    wprintw(termWin,
	    "usage: sonparms [-ec echo count] [-eto echo timeout]\n"
	    "                [-fd fire delay] [-fi fire interval]\n"
	    "                [-ebt echo blanking time]\n"
	    "                [-ibt init blanking time]\n"
	    "                [-sd start delay]\n"
	    "       No args will return current parms.\n");
    wrefresh(termWin);
    return (0);
  }
  
  memset(&parm, 0xFF, sizeof(parm)); /* initialize all elements to -1 */

  for (count=0, argcount=1; argcount<argc; argcount++) {
    parmNum = _sonparms_getparm(argv[argcount]);
    if (parmNum < 0) {
      wprintw (termWin,
	       "unknown parameter: %s\n",
	       argv[argcount]);
      wrefresh (termWin);
      return (-1);
    }
    if (++argcount >= argc) {
      wprintw (termWin,
	       "parameter %s requires an argument\n",
	       argv[argcount-1]);
      wrefresh (termWin);
      return (-1);
    }
    
    switch (parmNum) {
    case MSPT_EC:
      parm.echoCount = strtoul(argv[argcount], NULL, 16);
      break;
    case MSPT_ETO:
      parm.echoTimeout = strtoul(argv[argcount], NULL, 16);
      break;
    case MSPT_FD:
      parm.fireDelay = strtoul(argv[argcount], NULL, 16);
      break;
    case MSPT_FI:
      parm.fireInterval = strtoul(argv[argcount], NULL, 16);
      break;
    case MSPT_EBT:
      parm.echoBlankTime = strtoul(argv[argcount], NULL, 16);
      break;
    case MSPT_IBT:
      parm.initialBlankTime = strtoul(argv[argcount], NULL, 16);
      break;
    case MSPT_SD:
      parm.startDelay = strtoul(argv[argcount], NULL, 16);
      break;
    }
  } 
  mspSetSonParms(mspNum, &parm);
  return (0);
}

int
mspCls (int argc, char *argv[], long mspNum) {

  abtCls();
  return (0);
}

int
mspLsdevs (int argc, char *argv[], long mspNum) {
  int devId;
  int defId;

  defId = mspNum2devId(mspNum);

  wprintw (termWin,
	   "   mspNum  status  state devId  busId fd\n");

  for (devId=0; devId<ABD_MAX_DEVS; devId++) {
    
    if (abdDev[devId].state) {

      if (devId == defId)
	wprintw (termWin, "* ");
      else
	wprintw (termWin, "  ");

      wprintw (termWin, "%08X   %02X     %3d  %3d     %02X   %d\n",
	       abdDev[devId].devId.devNum,
	       abdDev[devId].devStatus.byte,
	       abdDev[devId].state,
	       devId,
	       abdDev[devId].busId,
	       abdDev[devId].fd);
    }
  }
  return (0);
}

/*************************************************
 *      msp message callback handlers
 *************************************************/

int mspConnectHandler (long mspNum, int flag) {

  if (flag) {
    wprintw (devWin, "{%08X} Ready\n", mspNum);
  }
  else {
    wprintw (devWin, "{%08X} Disconnected\n", mspNum);
  }
  wrefresh(devWin);
  return(0);
}

int
verRepHandler (long mspNum, const char *verString) {

  wprintw (devWin, "{%08X} VERSION : %s\n", (unsigned)mspNum, verString);
  wrefresh (devWin);
  return (0);
}

static int
bmpRepHandler (long mspNum, unsigned long bumps) {
  char buf[40];
  int ii;
   
  for (ii=0; ii<32; ii++) buf[ii] = ((bumps>>(31-ii))&0x01)+'0';
  buf[ii] = 0;
  wprintw(devWin, "{%08X} BMP : %s\n", (unsigned)mspNum, buf);
  wrefresh (devWin);
  return (0);
}

static int
irRepHandler (long mspNum, const unsigned long *irs) {
  int ii;

  wprintw (devWin, "{%08X} IR : ", (unsigned)mspNum);
  
  for (ii=0; irs[ii]; ii++) {
    wprintw (devWin, "0x%04X ", (unsigned)irs[ii]);
  }
  wprintw (devWin, "\n");
  
  wrefresh (devWin);
  return 0;
}

static int
irParmsHandler (long mspNum, const mspIrParmsType *parms) {
   
  wprintw (devWin, "{%08X} IR PARMS : interval : 0x%04X\n",
	   (unsigned)mspNum, parms->interval);
  wrefresh (devWin);
  return 0;
}

static int
sonRepHandler (long mspNum, const unsigned long *table[]) {
  int xducer, echo;

  wprintw (devWin, "{%08X} Sonar Reply:\n", (unsigned)mspNum);

  xducer = 0;
  while (table[xducer]) {
    wprintw (devWin, "\t%04X:\t", table[xducer][0]);
    echo = 1;
    while (table[xducer][echo]) {
      wprintw (devWin, " %04X", table[xducer][echo++]);
    }
    wprintw (devWin, "\n");
    xducer++;
  }
   
  wprintw (devWin, "\n");
  wrefresh(devWin);
  return (0);
}

static int
sonParmsHandler (long mspNum, const mspSonParmsType *parms) {
   
  wprintw (devWin, "{%08X} Sonar Parameters:\n", (unsigned)mspNum);
 
  wprintw (devWin, "\tfire interval : 0x%02X\n", parms->fireInterval);
  wprintw (devWin, "\techo count    : 0x%02X\n", parms->echoCount);
  wprintw (devWin, "\techo timeout  : 0x%02X\n", parms->echoTimeout);
  wprintw (devWin, "\tinit blank    : 0x%02X\n", parms->initialBlankTime);
  wprintw (devWin, "\techo blank    : 0x%02X\n", parms->echoBlankTime);
  wprintw (devWin, "\tfire delay    : 0x%02X\n", parms->fireDelay);
  wprintw (devWin, "\tstart delay   : 0x%02X\n", parms->startDelay);
  wprintw (devWin, "\n");
   
  wrefresh(devWin);
  return (0);
}

static int
sonTableHandler (long mspNum, const unsigned long *table[]) {
  int set, xducer;

  wprintw (devWin, "{%08X} Sonar Table:\n", (unsigned)mspNum);

  set = 0;
  while (table[set] && (set<6)) {
    wprintw (devWin, "\t");
    xducer = 0;
    while (table[set][xducer] && (xducer<10)) {
      wprintw (devWin, " %04X", table[set][xducer++]);
      wrefresh(devWin);
    }
    wprintw (devWin, "\n");
    wrefresh(devWin);
    set++;
  }
   
  wprintw (devWin, "\n");
  wrefresh(devWin);
  return (0);
}

/***************************************
 *   Other stuff
 ***************************************/

static const funcList U2D_lst[] = {
  {"ver",          reqVer           },
  {"iri",          setIri           },
  {"sonreq",       reqSon           },
  {"sontable",     sonTable         },
  {"sonstart",     reqSonStart      },
  {"sonstop",      reqSonStop       },
  {"sonparms",     setSonParms      },

  {"cls",          mspCls           },
  {"lsdevs",       mspLsdevs        },
  {"!U Cmd!\0", NULL}
};

int				/* User to Driver message */
mspU2D (int argc, char *argv[], long mspNum) {
  int (*foo)(int argc, char *argv[], long mspNum);
  int cmd_num;
  
  for (cmd_num = 0; U2D_lst[cmd_num].func != NULL
       && strcmp (argv[0], U2D_lst[cmd_num].fName) != 0;
       cmd_num++) ;

  if (U2D_lst[cmd_num].func == NULL) {
    wprintw(termWin, "Unknown command\n");
    wrefresh(termWin);
    return (-1);
  }
  else {
    foo = U2D_lst[cmd_num].func;
    (*foo)(argc, argv, mspNum);
    return (0);
  }
}


int
mspSelect(void) {
  return(mspLibSelect());
}

int
mspInit (char *fileName) {
  
  memset(&mspOps, 0, sizeof(mspOps));

  mspOps.validMspId = NULL;	/* use NULL to accept all MSPs */

  /* uncomment below and edit to select specific MSPs */
  /* mspOps.validMspId = {0x20, 0x21, 0x22, 0x30, 0x31, 0x32, 0x33}; */

  mspOps.mspConnect = mspConnectHandler;
  mspOps.verRep     = verRepHandler;
  mspOps.bmpRep     = bmpRepHandler;
  mspOps.irRep      = irRepHandler;
  mspOps.irParms    = irParmsHandler;
  mspOps.sonRep     = sonRepHandler;
  mspOps.sonTable   = sonTableHandler;
  mspOps.sonParms   = sonParmsHandler;

  return(mspLibInit(fileName, &mspOps));
}

int mspExit (int fd){
  /*
   * Technically, we should notify the manager that
   * the MSP driver is disconnecting... but lets
   * test to see if the manager can handle an unexpected
   * close for now.
   */
  close(fd);
  return 0;
}
