
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
#ifndef CONSOLE_H_LOADED
#define CONSOLE_H_LOADED

#define MAXCONSOLEPROGRAMS (8)

#define MAXCONSOLESTRINGLENGTH (255) /* be safe */

#define MISCERROR fprintf( stderr, "\n>>>>>> %s: %s(%d) error\n", __FILE__, __FUNCTION__, __LINE__ )

/* yeah, i know, it's static. */
typedef struct {
  char programID[MAXCONSOLESTRINGLENGTH];
  char programName[MAXCONSOLESTRINGLENGTH];
  char howStart1[MAXCONSOLESTRINGLENGTH];
  char howStart2[MAXCONSOLESTRINGLENGTH];
  char howStart3[MAXCONSOLESTRINGLENGTH];
  char howKill1[MAXCONSOLESTRINGLENGTH];
  char howKill2[MAXCONSOLESTRINGLENGTH];
  char howKill3[MAXCONSOLESTRINGLENGTH];
  char programOnHost[MAXCONSOLESTRINGLENGTH];
  char dep[MAXCONSOLEPROGRAMS-1][MAXCONSOLESTRINGLENGTH];
  int numDependencies;
} ConsoleDepType, *ConsoleDepPtr;

ConsoleDepType ConsoleDep[MAXCONSOLEPROGRAMS];

char *tcxMachine;		/* which tcxhost do we use? */

char *pathToMyPrograms;		/* where to find beeSoft and user programs */

char **myProgram;		/* [MAXCONSOLEPROGRAMS][MAXCONSOLESTRINGLENGTH] */

int numMyPrograms;		/* ! */

int scrollauto;			/* scroll automatically to the end of the window */
#endif

/*
 * $Log: console.h,v $
 * Revision 1.3  1998/02/09 22:59:29  swa
 * Added a parameter that controls if we automatically scroll to the end of
 * each log window.
 *
 * Revision 1.2  1997/11/28 18:26:00  swa
 * Added a variable to replace absolute paths. More user friendly. Use only
 * ssh for simplicity and security. Update TODO with tyson's thoughts.
 *
 * Revision 1.1.1.1  1997/11/10 23:10:36  swa
 * Stefan's new console manager.
 *
 * Revision 1.4  1997/11/09 18:25:11  swa
 * All programs and their dependencies are now taken care of. A second,
 * user-specific ini file was added, console2.ini. All dependencies in
 * console.ini should eventually go into beeSoft.ini and console2.ini
 * should be renamed console.ini.
 *
 * Revision 1.3  1997/11/08 18:01:07  swa
 * Programs which depend on other programs are now taken care of.
 *
 * Revision 1.2  1997/11/08 17:27:12  swa
 * Completed console.ini. Now parse all servers.
 *
 * Revision 1.1  1997/11/08 03:51:05  swa
 * Most variables are no longer set in the .tcl script, instead they are
 * read from the .ini file and evaluated by our Tcl interpreter.
 *
 *
 */
