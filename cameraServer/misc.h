
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
#ifndef MISC_H_LOADED
#define MISC_H_LOADED

#define EPS (1.0E-06)

/*#define MAX_RANDOM 1073741823.5*/
/*#define RAND() ((((double) random())/MAX_RANDOM)-1.0)*/

#define DEF_STRING_SIZE (200)

#define WHERE if ( 0 ) fprintf( stderr, "%s: %s\n", __FILE__, __FUNCTION__ )

#define MALLOC  fprintf( stderr, ">>>>>> %s: %s(%d) malloc failed\n", __FILE__, __FUNCTION__, __LINE__  )

#define NULLERR   fprintf( stderr, ">>>>>> %s: %s(%d) pointer NULL\n", __FILE__,  __FUNCTION__, __LINE__ )

#define DIVZERO fprintf( stderr, ">>>>>> %s: %s(%d) div by zero error\n",  __FILE__, __FUNCTION__, __LINE__ )

#define MISCERROR fprintf( stderr, ">>>>>> %s: %s(%d) error\n", __FILE__, __FUNCTION__, __LINE__ )

#define KILL    fprintf( stderr, ">>>>>> %s: %s(%d) couldn't kill/free pointer \n", __FILE__,  __FUNCTION__, __LINE__ )

#define ZEROZERO fprintf( stderr, ">>>>>> %s: %s(%d) zero/zero error! \n", __FILE__,  __FUNCTION__, __LINE__ )

int inRegion( int x, int y, iRectPtr w);

int fillFileName( char *filename );

int openFileForWrite( FILE **fp, const char *filename );

int openFileForWrite2( FILE **fp, char *filename );

int openFileForRead( FILE **fp, const char *filename );

double minDouble( double *tmp, int num );

double maxDouble( double *tmp, int num );

int fireupGnuplot( char         *datafilename,    /* contains the data */
		   char         *title,           /* graphtitle */
		   int           numColumns,      /* numgraphs in file */
		   char         **titlePerColumn, /* title per graph */
		   int           numCommands,     /* additional ... */
		   char         **commands,       /*    commands    */
		   int           grid,            /* 1 = yes */
		   int           formatForPrinter,/* 1 = create .ps file */
		   int           showFilename,
		   int           use_minmax,
		   double        ymin,
		   double        ymax);  /* display commandfilename */

#define FT_RAWDATA                1
#define FT_OBSERVATIONSET         2
#define FT_PROTOTYPE              3
#define FT_ARMPOSITIONDATABASE    4

int determineFileType( char *filename );

#define D_PERFORMANCE           1
#define D_INFO                  2
#define D_WHEREAMI              4
#define D_DEBUG                 8
#define D_TCX                  16
int debug;

#ifndef RAND_MAX
#define RAND_MAX  2147483647.0
#endif

#endif

/* $Log: misc.h,v $
 * Revision 1.3  1997/10/04 00:13:09  swa
 * First working version of the cameraServer that supports two (count'em
 * two!) framegrabbers.  Although the server seems to work just fine, it
 * has not yet been fully tested.
 *
 * Revision 1.2  1997/06/23 02:36:00  thrun
 * Stefan's new camera server with shared memory
 *
 * Revision 1.1.1.1  1997/06/16 22:32:20  thrun
 * New Camera Server, much faster, more reliable, works already with
 * commander.
 */
