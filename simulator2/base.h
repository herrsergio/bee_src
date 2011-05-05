
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



#ifdef __cplusplus
extern "C" {
#endif

#include "tcx.h"
#include "rwibase_interface.h"
    
#define BATTERY_VOLTAGE		16

/* #define COUNTS_IN_360_DEGREES  		1024 */
#define COUNTS_PER_DEG      		(float) COUNTS_PER_DEGREE

/* internal BASE variables */

struct BaseVar {

  float         time;                

  float   	RotateAcceleration;
  float   	RotateWhere;
  float    	RotateWantedVelocity;
  float    	RotateCurrentVelocity;

  int		RotateDirection;
  float   	RotateRelativePos;
  char		RotateRelativeFlag;
  char		RotateHaltFlag;

  float   	TranslateAcceleration;
  float   	TranslateWhere;			
  float    	TranslateWantedVelocity;
  float    	TranslateCurrentVelocity;

  int		TranslateDirection;
  float   	TranslateRelativePos;
  char		TranslateRelativeFlag;
  char		TranslateHaltFlag;

/*
  int           pos_x;               
  int 		pos_y;               
*/

  char		bump;
  int		watch_dog;

  long  	StatusReportPeriod;

};

extern struct BaseVar BaseVariables;

extern int	retvalHandleMove;



extern struct 	timeval		tv_start;		/* tv_base - tv_start  =  running time */
extern struct 	timeval		tv_base;


/**********************************************************************************************************/
/* PROCEDURE :                  base()                                  ***********************************/
/* Parameter :                  command                                 ***********************************/
/*                                                                      ***********************************/
/* Change internal base variables in dependencie if Parameter command.  ***********************************/
/* This parameters are used by routine HandleMove().                    ***********************************/
/* Sets timer for status report update.                                 ***********************************/
/**********************************************************************************************************/

void	base(unsigned char command[]);


/**********************************************************************************************************/
/* PROCEDURE :                  HandleMove()                            ***********************************/
/* Parameter :                  none					***********************************/
/*                                                                      ***********************************/
/* Using internal base variables to change robtot parameters like       ***********************************/
/* acceleration, velocity, position ...                                 ***********************************/
/* Returns 1, when move can be performed, 0 otherwise                   ***********************************/
/**********************************************************************************************************/

void	HandleMove();


/**********************************************************************************************************/
/* PROCEDURE :                  InitBaseVar()                           ***********************************/
/* Parameter :                  none                                    ***********************************/
/*                                                                      ***********************************/
/* Initialization of global Base Variables                              ***********************************/
/**********************************************************************************************************/

void    InitBaseVar();


/**********************************************************************************************************/
/* PROCEDURE :                  statusReport()                          ***********************************/
/* Parameter :                  none                                    ***********************************/
/*                                                                      ***********************************/
/* sends status report via TCX to MODULE_BASE                           ***********************************/
/**********************************************************************************************************/


void statusReport();

/*** only used for status report ***/

/*** some help routines ***/
#define mySGN(x)  ((x) >= 0 ? (1) : (-1))
#define myABS(x)  ((x) < 0 ? -(x) : (x))
#define mySQR(x)  ((x) * (x))
#define myMAX(x, y)     ((x) > (y) ? (x) : (y))
#define myMIN(x, y)     ((x) < (y) ? (x) : (y))
#define ctoi(x)         ((x) - '0')

/*** tables with trigonometric values -> used for speed up ***/
/* who needs speedup ? */

/* extern float SIN_TAB[]; */
/* #define mySIN(x) sin(x * M_PI / 180) */
/* #define myCOS(x) cos(x * M_PI / 180) */

/* #define mySIN(x) (mySGN((x))*SIN_TAB[(int)(0.5 + myABS((x))) % 360]) */
/* #define myCOS(x) (mySIN((x)+90)) */
/*** atan from table doesn't speed up !?! ***/

void base_sendTarget(float,float);
void InitBaseVar();

float base_coord_x();
float base_coord_y();
float base_coord_rot();

struct mcpPacket {
  unsigned char size;
  unsigned char chksum;
  unsigned char data1[255];
  unsigned char data[255];
};

typedef struct mcpPacket mcpPacketType;

#ifdef __cplusplus
}
#endif
