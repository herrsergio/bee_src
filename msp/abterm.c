
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



#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include <sys/types.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <termios.h>
#include <curses.h>

#include "abterm.h"
#include "msputils.h"
#include "version.h"
#include "mspterm.h"

char *usage =
"Yeah!  Right!  ...options coming soon to an mspterm near you.\n";
/* "usage: %s [-s <serial_dev>] [-a <abus_dev>]\n" */

WINDOW *termWin;
WINDOW *devWin;
WINDOW *dbgWin;

int destId;
int fd;

void initWins()
{
  termWin = devWin = dbgWin = NULL;

  stdscr = initscr ();
  termWin = subwin (stdscr, LINES/2-4, COLS, 1, 0);
  dbgWin  = subwin (stdscr, 3, COLS, LINES/2-2, 0);
  devWin  = subwin (stdscr, LINES/2-1, COLS, LINES/2+1, 0);
  if (devWin == NULL) {
    endwin ();
    fprintf (stderr, "unable to initialize devWin\n");
    exit (1);
  } else if (termWin == NULL) {
    endwin ();
    fprintf (stderr, "unable to initialize termWin\n");
    exit (1);
  } else if (dbgWin == NULL) {
    endwin ();
    fprintf (stderr, "unable to initialize dbgWin\n");
    exit (1);
  }

  /* make all windows scrolling */
  scrollok (termWin, TRUE);
  scrollok (devWin, TRUE);
  scrollok (dbgWin, TRUE);

  /* screen setup */
  abtPrintTitles ();
}

void quit (int rc) {

   mspExit(fd);

   if (termWin != NULL) {
      /* little hack to make the prompt go away */
      wprintw (termWin, "\n");

      refresh ();
      move (LINES-1, 0);
      addch ('\n');
      refresh ();
      endwin ();
   }

   exit (rc);
}

int parse_line (char *line) {
  int argc;
  char *argv[100];
  
  if ((line!=(char *)EOF) && (line!=NULL)) {
    for (argc=0; argc<10; argc++) argv[argc]=NULL;
    argc=0;
    while (isspace(*line)) line++;
    if (*line != 0) {
      if (isdigit (*line)) {
	destId = strtoul(line, &line, 16);

	/* XXX check on destId, devId, busId */

      }
      while (isspace(*line)) line++;
      argv[0]=line;
      argc=0;
      if (*line) argc=1;

      do {
	while ((isspace(*line)==0) && *line) line++;
	if (*line==0) break;
	*(line++)=0;
	while (isspace(*line)) line++;
	if (*line) {
	  argv[argc++]=line;
	}
      } while (*line);
      
      if (!(strcmp(argv[0], "quit"))) quit(0);
      
      if (argc) mspU2D(argc, argv, destId);
      wrefresh (termWin);
    }
    return (1);
  }
  quit(0);
  return(0);
}

void
abtSelect (int fd) {
  mspSelect(fd);
  return;
#if 0
  ABMSG msg;
  int count;

  count = read (fd, (char *)&msg, sizeof (msg));
  if (count != sizeof(msg)) {
    wprintw(dbgWin, "select: read() != sizeof(msg)\n\n");
    wrefresh(dbgWin);
    return;
  }

  if (mspM2D(fd, &msg)) defM2D(fd, &msg);
#endif /* 0 */
}

int
main (int argc, char *argv[])
{
  int argn;
  int termFd;
  char cmdline[256];

  /* this is kinda pointless, cause we're probably
     not going to be getting input from anything
     but stdin */
  termFd = 0;
  
  initWins();  /* set up curses interface */
  
  fd = mspInit("/dev/abus");
  if (fd < 0) exit (0);
  
  wprintw (termWin, "mspterm - %s\n", version_date);
  
  /* we also need to cause devId 0x51 to be abmgr, 0x50 to be ab, */
  /* and 0x52 to be serial. Real devs will be numbered after that */
  /* probably this should happen in abmgrInit().                  */
  
  /* handle commandline args */
  
  for (argn=1; argn!=argc; argn++) {
    if (*argv[argn] == '-') {
      if (strcmp (argv[argn], "-h") == 0) {
	fprintf (stderr, usage, argv[0]);
	quit (0);
      }
    }
  }
  
  signal (SIGTERM, quit);
  signal (SIGINT, quit);
  signal (SIGHUP, quit);
  
  
  do {
    fd_set realset, rset;

    wprintw (termWin, PROMPT, destId);
    wrefresh (termWin);
    
    /* we need to reset this each time through */
    FD_ZERO (&realset);
    FD_SET (fd, &realset);
    FD_SET (termFd, &realset);
    
    do {
      /* has to be reset, cause select unsets it */
      rset = realset;
      
      /* block until read ready on one of msps or termWin */
      select (10, &rset, NULL, NULL, NULL);
      
      /* deal with everyone that needs to be read */
      if (FD_ISSET (fd, &rset) && fd != termFd) {
	abtSelect(fd);
	wrefresh(termWin);
      }
      /* feed the user from time to time */
    } while (!FD_ISSET (termFd, &rset));
    wgetstr (termWin, cmdline);
  } while (parse_line (cmdline));
  
  /* exit gracefully */
  quit (0);
  return (0);
}
