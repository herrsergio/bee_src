
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



#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <netinet/in.h>

#include "robot.h"
#include "sonar.h"
#include "tactile.h"
#include "base.h"
#include "trigofkt.h"
#include "surface.hh"
#include "playground.hh"

/* the following define is needed because someone included this nasty
   little handcrafted libc.h */
#define _UNISTD_H
#include "baseOpcodes.h"

extern Boolean use_baseServer;


#define MAXARITY 1		                /* maximum arity a BASE commands can have */
#define TCX_define_variables        		/*** this makes sure variables are installed ***/
/*
#define BASE_DEBUG
*/

#include "SIMULATOR-messages.h"

TCX_MODULE_PTR MODULE_BASE = NULL;


int	 retvalHandleMove = 1;
static	 int KI_OR_SH = 0;			/*** when base command "KI" or "SH" --> 		***/
						/*** HandleMove() should not try to make next move !	***/

static   char	slowDownRot   = 0;			/*** used for smooth stop after RR, ... ***/
static   char	slowDownTrans = 0;			/*** used for smooth stop after TR, ... ***/

struct BaseVar BaseVariables;

struct   timeval         tv_start;              /*** tv_base - tv_start  =  running time ***/
struct   timeval         tv_base;
struct 	 itimerval	 itv;			/*** used for status report update ***/



char *command;                                  /* current command string */

#define COUNTS_PER_CM   BASE_COUNTS_PER_CM

#define CLEANUP 1


typedef void (*VOID_FN)();

typedef struct {
  unsigned char *cmd;
  unsigned char opCode;
  VOID_FN cmdFunc;
  unsigned int numParams;
  unsigned char *cmdDesc;
  unsigned char *paramsDesc;
} cmddef;

/* simulated commands sofar */

void SIM_StatusReportPeriod();
void SIM_WatchDog();
void SIM_Clock();
void SIM_BatteryVoltage();
void SIM_RotateRelPositive();
void SIM_RotateRelNegative();
void SIM_RotateHalt();
void SIM_RotateAcceleration();
void SIM_RotatePosition();
void SIM_RotateVelocity();
void SIM_RotateClockwise();
void SIM_RotateAnticlockwise();
void SIM_RotateWhere();
void SIM_Kill();
void SIM_TranslateHalt();
void SIM_TranslateAcceleration();
void SIM_TranslateVelocity();
void SIM_TranslateForward();
void SIM_TranslateBackward();
void SIM_TranslatePosition();
void SIM_TranslateWhere();
void SIM_TranslateRelNegative();
void SIM_TranslateRelPositive();
void SIM_SetReference();
void SIM_LoadHeading(long l);
void SIM_Help();
void SIM_Dummy();
void SIM_Dummy1();

static cmddef Base_CmdTable[] = {
  {"SP", OP_SET_STATUS_PERIOD, (VOID_FN) SIM_StatusReportPeriod,1,
     "SIM: Set Report Interval",NULL},
  {"WD", OP_WATCH_DOG_TIMER, (VOID_FN) SIM_WatchDog,1,
     "SIM: Set watchdog Timer",NULL},
  {"CL", OP_GET_MCP_CLOCK, (VOID_FN) SIM_Clock,0,
     "SIM: read Clock (equals noop)",NULL},
  {"BV", OP_GET_BATTERY_VOLTAGE, (VOID_FN) SIM_BatteryVoltage,0,
     "read battery voltage (equlas noop)",NULL},
  {"RH", OP_ROTATE_HALT, (VOID_FN) SIM_RotateHalt, 0,
     "Rotate halt",NULL},
  {"RL", OP_ROTATE_LIMP, (VOID_FN) SIM_RotateHalt, 0,
     "Rotate Limp",NULL},
  {"RA", OP_SET_ROTATE_ACCEL, (VOID_FN) SIM_RotateAcceleration, 1,
     "Set rotation acceleration","acceleration"},
  {"RV", OP_SET_ROTATE_VEL, (VOID_FN) SIM_RotateVelocity, 1,
     "Set rotation velocity",NULL},
  {"RP", OP_ROTATE_TO_POS, (VOID_FN) SIM_RotatePosition, 1,
     "Rotate to direction",NULL},
  {"R+", OP_ROTATE_VEL_POS, (VOID_FN) SIM_RotateClockwise, 0,
     "Rotate clockwise",NULL},
  {"R-", OP_ROTATE_VEL_NEG, (VOID_FN) SIM_RotateAnticlockwise, 0,
     "Rotate anticlockwise",NULL},
  {"R>", OP_ROTATE_REL_POS, (VOID_FN) SIM_RotateRelPositive, 1,
     "Rotate relative positive",NULL},
  {"R<", OP_ROTATE_REL_NEG, (VOID_FN) SIM_RotateRelNegative, 1,
     "Rotate relative negative",NULL},
  {"RW", OP_GET_ROTATE_WHERE, (VOID_FN) SIM_RotateWhere, 0,
     "Query Direction",NULL},
  {"KI", OP_MCP_KILL, (VOID_FN) SIM_Kill, 0,
     "Panic halt ?",NULL},
  {"SH", 0x03, (VOID_FN) SIM_Kill, 0,
     "Panic halt ?",NULL},
  {"TH", OP_TRANS_HALT, (VOID_FN) SIM_TranslateHalt, 0,
     "Translate halt",NULL},
  {"TL", OP_TRANS_LIMP, (VOID_FN) SIM_TranslateHalt, 0,
     "Translate limp",NULL},
  {"TA", OP_SET_TRANS_ACCEL, (VOID_FN) SIM_TranslateAcceleration, 1,
     "Set translation acceleration","acceleration"},
  {"TV", OP_SET_TRANS_VEL, (VOID_FN) SIM_TranslateVelocity, 1,
     "Set translation velocity","velocity"},
  {"T+", OP_TRANS_VEL_POS, (VOID_FN) SIM_TranslateForward, 0,
     "Translate forward",NULL},
  {"T-", OP_TRANS_VEL_NEG, (VOID_FN) SIM_TranslateBackward, 0,
     "Translate backward",NULL},
  {"TP", OP_TRANS_TO_POS, (VOID_FN) SIM_TranslatePosition, 1,
     "Translate Position",NULL},
  {"TW", OP_GET_TRANS_WHERE, (VOID_FN) SIM_TranslateWhere, 0,
     "Query Position",NULL},
  {"T<", OP_TRANS_REL_NEG, (VOID_FN) SIM_TranslateRelNegative, 1,
     "Translate Relative negative",NULL},
  {"T>", OP_TRANS_REL_POS, (VOID_FN) SIM_TranslateRelPositive, 1,
     "Translate Relative positive",NULL},
  {"IX", OP_FIND_ROT_INDEX, (VOID_FN) SIM_SetReference, 0,
     "Set current pos as reference",NULL},
  {"LH", OP_LOAD_HEADING, (VOID_FN) SIM_LoadHeading, 1,
     "Set robot heading",NULL},
  { "?", 0x00, (VOID_FN) SIM_Help, 0,
     "Display this help message", NULL},
  {"JD", OP_JOYSTICK_DISABLE, (VOID_FN) SIM_Dummy1, 1,
     "Joystick disable", NULL},
  {"HD", OP_HALF_DUPLEX, (VOID_FN) SIM_Dummy, 0,
     "Half Duplex", NULL},
  {"TT", OP_SET_TRANS_TORQUE, (VOID_FN) SIM_Dummy1, 1,
     "Torque Limit", NULL},
  {"SD", OP_SET_STATUS_DATA, (VOID_FN) SIM_Dummy1, 1,
     "Status report data", NULL},  
  {"IM", OP_INDEX_REPORT, (VOID_FN) SIM_Dummy1, 1,
     "Index poll and mask set", NULL},  
  {(char *) NULL, 0x00, (VOID_FN) NULL, 0,NULL, NULL}
};

/* command dispatcher */

void base(unsigned char cmdline[]) {
  unsigned int
    i = 0,
    numParams;
  int j;
  unsigned char cmd[5];
  long p_ary[MAXARITY];
  cmddef *cmd_entry;

  command = cmdline;
  numParams = sscanf(cmdline, "%s %x",
		     &cmd,&p_ary[0]);
  numParams -= 1;
  /*this became necessary since BASE appears to send a 0D, at least
   *under SUN-OS 4.1, (st) Thrun 96-11-5*/
  for (j = 0; cmd[j] != '\0'; j++)
    if ((int) cmd[j] == 13)
      cmd[j] = '\0';

  /*doing linear search for the moment*/
  while(Base_CmdTable[i].cmd != NULL) {
    cmd_entry = &Base_CmdTable[i];
    if(strcmp(cmd_entry->cmd, cmd) == 0 ) {
      if(cmd_entry->cmdFunc == NULL) {
	fprintf(stderr,"Command not implemented: %s\n",cmd);
	return;
      }
      if( cmd_entry->numParams != numParams) {
	fprintf(stderr," base(): num arguments mismatch for cmd %s expected %d found %d !\n",
		cmd, cmd_entry->numParams, numParams); 
      }
      switch(numParams) {
      case 0:
	(* (cmd_entry->cmdFunc))();
	break;
      case 1:
	(* (cmd_entry->cmdFunc))(p_ary[0]);
	break;
      default:
	fprintf(stderr, "base(): Sorry %d arguments not yet supported, \
maximum is %d\n", numParams, MAXARITY);
      }
      return;
    } else {
      i++;
      if(Base_CmdTable[i].cmd == NULL) {
	fprintf(stderr,"base(): Unsupported cmd \"%s\" !\n",cmd);
	return;
      }
    }
  }
}

#define         RealRobotX       (robot.map_robot_x-robot.map_robot_start_x)
#define         RealRobotY       (robot.map_robot_y-robot.map_robot_start_y)

float base_init_rot;
float base_correct_x = 0.0;
float base_correct_y = 0.0;

float base_coord_x()
{
  float rot = M_PI/180 * base_init_rot;
  return RealRobotX*cos(rot)-RealRobotY*sin(rot)
    + ROBOT_INIT_POS_X + base_correct_x;
}
float base_coord_y()
{
  float rot = M_PI/180 * base_init_rot;
  return RealRobotX*sin(rot)+RealRobotY*cos(rot)
    + ROBOT_INIT_POS_Y + base_correct_y;
}

float base_coord_rot()
{
  float rot = (robot.robot_deg - robot.robot_start_deg
	       + ROBOT_INIT_ROT);
  while(rot >= 360) rot -= 360;
  while(rot < 0) rot += 360;
  rot = 360 - rot;
  rot += 90;
  while(rot >= 360) rot -= 360;
  return rot;
}

/**********************************************************************
 *
 * Routine to parse and dispatch messages from baseServer - TDS
 *
 **********************************************************************/

void baseServer(unsigned char *cmd) {
  unsigned int i = 0;
  unsigned char opCode;
  long cmdArg;
  char *cmdAry = (char*)&cmdArg;
  cmddef *cmd_entry;
  
  opCode = cmd[0];
  cmdAry[0] = cmd[1];             
  cmdAry[1] = cmd[2];
  cmdAry[2] = cmd[3];
  cmdAry[3] = cmd[4];  
  cmdArg = ntohl(cmdArg);
  while(Base_CmdTable[i].cmd != NULL) {
    cmd_entry = &Base_CmdTable[i];
    if(cmd_entry->opCode == opCode) {
      if(cmd_entry->cmdFunc == NULL) {
	fprintf(stderr,"Command not implemented: %s\n",cmd);
	return;
      }

      switch(cmd_entry->numParams) {
      case 0:
	(* (cmd_entry->cmdFunc))();
	break;
      case 1:
	(* (cmd_entry->cmdFunc))(cmdArg);
	break;
      default:
	fprintf(stderr, "base(): Sorry %d arguments not yet supported, "
		"maximum is %d\n", cmd_entry->numParams, MAXARITY);
      }

      return;
    } else {
      i++;
      if(Base_CmdTable[i].cmd == NULL) {
	fprintf(stderr,"base(): Unsupported opCode 0x%02X !\n", opCode);

	return;
      }
    }
  }
}

/* help_on_cmds() prints help message on available BASE commands */

void SIM_Help()
{
  unsigned int i = 0;
  while(Base_CmdTable[i].cmd != NULL) {
    printf("%s ", Base_CmdTable[i].cmd);
    if(Base_CmdTable[i].numParams > 0) 
      if(Base_CmdTable[i].paramsDesc != NULL) 
	printf("%s",Base_CmdTable[i].paramsDesc);
      else
	printf("%d param.",Base_CmdTable[i].numParams);
    else		/* don't get confused with tabs (hopefully)*/
      printf("       ");
    if(Base_CmdTable[i].cmdDesc != NULL) 
      printf("\t\t-- %s\n",Base_CmdTable[i].cmdDesc);
    else
      printf("\t\t-- Undocumented (ask someone competent)\n");
    i++;
  }
}

/* simulated BASE commands ... */

void SIM_RotateAcceleration( long l)
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): 0x%08X\n",
	  __FILE__, __LINE__, __FUNCTION__, l);
#endif
  BaseVariables.RotateAcceleration = l / COUNTS_PER_DEG;
}

void SIM_RotateHalt()
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  BaseVariables.RotateHaltFlag = 1;
  BaseVariables.RotateRelativeFlag = 0;
  slowDownRot = 0;
}

void SIM_RotateVelocity(long l)
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): 0x%08X\n",
	  __FILE__, __LINE__, __FUNCTION__, l);
#endif

  BaseVariables.RotateWantedVelocity = l / COUNTS_PER_DEG;
}

void SIM_RotateClockwise()
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  BaseVariables.RotateDirection = -1;
  BaseVariables.RotateHaltFlag = 0;
  BaseVariables.RotateRelativeFlag = 0;
  slowDownRot = 0;
}

void SIM_RotateAnticlockwise()
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  BaseVariables.RotateDirection = 1;
  BaseVariables.RotateHaltFlag = 0;
  BaseVariables.RotateRelativeFlag = 0;
  slowDownRot = 0;
}

void SIM_TranslateAcceleration(long l)
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): 0x%08X\n",
	  __FILE__, __LINE__, __FUNCTION__, l);
#endif

  BaseVariables.TranslateAcceleration = l / COUNTS_PER_CM;
}

void SIM_TranslateHalt()
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  BaseVariables.TranslateHaltFlag = 1;
  BaseVariables.TranslateRelativeFlag = 0;
  slowDownTrans = 0;
}

void SIM_TranslateVelocity(long l)
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): 0x%08X\n",
	  __FILE__, __LINE__, __FUNCTION__, l);
#endif

  BaseVariables.TranslateWantedVelocity = l / COUNTS_PER_CM;
}

void SIM_TranslateForward()
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  BaseVariables.TranslateDirection = 1;
  BaseVariables.TranslateRelativeFlag = 0;
  BaseVariables.TranslateHaltFlag = 0;
  slowDownTrans = 0;
}

void SIM_TranslateBackward()
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  BaseVariables.TranslateDirection = -1;
  BaseVariables.TranslateRelativeFlag = 0;
  BaseVariables.TranslateHaltFlag = 0;
  slowDownTrans = 0;
}

void SIM_RotateRelPositive(long l)
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): 0x%08X\n",
	  __FILE__, __LINE__, __FUNCTION__, l);
#endif

  BaseVariables.RotateRelativePos = -1.0 * l / COUNTS_PER_DEG;
  BaseVariables.RotateDirection = -1;
  BaseVariables.RotateHaltFlag = 0;
  BaseVariables.RotateRelativeFlag = 1;
  slowDownRot = 0;
}

void SIM_RotateRelNegative(long l)
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): 0x%08X\n",
	  __FILE__, __LINE__, __FUNCTION__, l);
#endif

  BaseVariables.RotateRelativePos = l / COUNTS_PER_DEG;
  BaseVariables.RotateDirection = 1;
  BaseVariables.RotateHaltFlag = 0;
  BaseVariables.RotateRelativeFlag = 1;
  slowDownRot = 0;
}

void SIM_Kill()
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  robot.runflag = 0;
  KI_OR_SH = 1;  
}

/* XXX The robot does not implement this function this way! - TDS */

void SIM_RotatePosition(long l)
{
  int deg;
  
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): 0x%08X\n",
	  __FILE__, __LINE__, __FUNCTION__, l);
#endif

  fprintf(stderr, "*** call to improperly implemented "\
	  "%s function ***", __FUNCTION__);
  
  deg = l / COUNTS_PER_DEG;
  while (deg < 0.0)
    deg += 360.0;
  while (deg >= 360.0)
    deg -= 360.0;
  
  /*** take shorter way ***/
  if ((robot.robot_start_deg - deg < 180.0) && (robot.robot_start_deg - deg > -180.0))
    BaseVariables.RotateRelativePos = robot.robot_start_deg - deg;
  else
    BaseVariables.RotateRelativePos = robot.robot_start_deg + deg;
  
  if (BaseVariables.RotateRelativePos < 0.0)
    BaseVariables.RotateRelativePos += 360.0;
  if (BaseVariables.RotateRelativePos >= 360.0)
    BaseVariables.RotateRelativePos -= 360.0;
  
  BaseVariables.RotateRelativeFlag = 1;
  BaseVariables.RotateHaltFlag = 0;
  slowDownRot = 0;
}

/* XXX This is NOT how this works on the robot! - TDS */

void SIM_RotateWhere()
{
  long l;
  char ret[10];

  fprintf(stderr, "*** call to improperly implemented "\
	  "%s function ***", __FUNCTION__);

  l = (long) (BaseVariables.RotateWhere * COUNTS_PER_DEG);
  sprintf(ret, " %x", l);
  strcat(command, ret);
}

void SIM_TranslatePosition(long l)
{
  float dist, actual_dist;
  
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): 0x%08X\n",
	  __FILE__, __LINE__, __FUNCTION__, l);
#endif

  dist = l / COUNTS_PER_CM;
  actual_dist =
    (float) (sqrt((double) (mySQR(robot.map_robot_x - robot.map_robot_start_x)
			    + mySQR(robot.map_robot_y
				    - robot.map_robot_start_y))));
  
  BaseVariables.TranslateRelativePos = (dist - actual_dist);
  BaseVariables.TranslateRelativeFlag = 1;
  BaseVariables.TranslateHaltFlag = 0;
  slowDownTrans = 0;
}

void SIM_TranslateWhere()
{
  char ret[10];
  long l = (long) (BaseVariables.TranslateWhere * COUNTS_PER_CM);
  sprintf(ret, " %x", l);
  strcat(command, ret);
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

}

void SIM_TranslateRelPositive(long l)
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): 0x%08X\n",
	  __FILE__, __LINE__, __FUNCTION__, l);
#endif

  BaseVariables.TranslateRelativePos = l / COUNTS_PER_CM;
  BaseVariables.TranslateRelativeFlag = 1;
  BaseVariables.TranslateDirection = 1;
  BaseVariables.TranslateHaltFlag = 0;
  slowDownTrans = 0;
}

void SIM_TranslateRelNegative(long l)
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): 0x%08X\n",
	  __FILE__, __LINE__, __FUNCTION__, l);
#endif

  BaseVariables.TranslateRelativePos = l / COUNTS_PER_CM;
  BaseVariables.TranslateRelativePos *= -1;
  BaseVariables.TranslateRelativeFlag = 1;
  BaseVariables.TranslateDirection = -1;
  BaseVariables.TranslateHaltFlag = 0;
  slowDownTrans = 0;
}

void SIM_SetReference()
{
  int distance;

#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

#if 0
  /*
   * This does not match robot behavior.
   * This functionallity should exist in LH and LP
   * commands.
   */
  robot.map_robot_start_x = robot.map_robot_x;
  robot.map_robot_start_y = robot.map_robot_y;
  robot.robot_start_deg = robot.robot_deg;
#endif

  fprintf(stderr, "%10s:%5d:%14s(): base_BRH = %d\n",
	  __FILE__, __LINE__, __FUNCTION__, robot.base_BRH);

  if (robot.base_BRH == 0.0) {
    srandom(time(NULL));
    robot.base_BRH = base_coord_rot() + (float)(random() & 0x3FF);

    while(robot.base_BRH > 360.0) {
      robot.base_BRH -= 360.0;
    }

    fprintf(stderr, "%10s:%5d:%14s(): base_BRH = %d\n",
	    __FILE__, __LINE__, __FUNCTION__, robot.base_BRH);

  }

  distance = (int)((robot.base_BRH-base_coord_rot())*COUNTS_PER_DEG) & 0x3FF;

  if (distance == 0) {
    distance = 0x400;
  }

  fprintf(stderr, "%10s:%5d:%14s(): rotating 0x%X\n",
	  __FILE__, __LINE__, __FUNCTION__, distance);

  SIM_RotateRelPositive(distance);
}

void SIM_LoadHeading(long l)
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): 0x%08X\n",
	  __FILE__, __LINE__, __FUNCTION__, l);
#endif

  robot.robot_deg=l;
}

void SIM_StatusReportPeriod(long l)
{
  int interval;
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): 0x%08X\n",
	  __FILE__, __LINE__, __FUNCTION__, l);
#endif

  BaseVariables.StatusReportPeriod = l * 1000;	/*** s -> ms ***/
  BaseVariables.StatusReportPeriod /= 256;	/*** 256 ticks per second ***/
  interval = BaseVariables.StatusReportPeriod;	
/*  schedule((void *) statusReport, interval, SCHEDULE_STATUS_REPORT); */
  schedule_statusReport(interval);
  return;
}

void SIM_WatchDog(long l)
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): 0x%08X\n",
	  __FILE__, __LINE__, __FUNCTION__, l);
#endif

  BaseVariables.watch_dog = (l * 1000)/256;	/*** s*256 -> ms ***/
}

void SIM_Clock() /* this command does nothing useful */
{
  long time_in_256_sec;
  char ret[10];
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  gettimeofday(&tv_base, NULL);
  time_in_256_sec = (tv_base.tv_sec - tv_start.tv_sec) * 256;
  time_in_256_sec += ((tv_base.tv_usec - tv_start.tv_usec) * 256) / 1000000;
  sprintf(ret, " %x", time_in_256_sec);
  strcat(command, ret); 
}

void SIM_BatteryVoltage() /* this command does nothing useful */
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  return;
}

void SIM_Dummy()
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  return;
}

void SIM_Dummy1(long dummy1)
{
#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  return;
}

/**********************************************************************************************************/
/* PROCEDURE :			HandleMove()				***********************************/
/* Parameter :			none					***********************************/
/*									***********************************/
/* Using internal base variables to change robtot parameters like       ***********************************/
/* acceleration, velocity, position ...					***********************************/
/* Returns 1, when move can be performed, 0 otherwise 			***********************************/
/**********************************************************************************************************/


void    
HandleMove()
{
  static float	x, y, oldx, oldy;
  float 		timeDiff;
  int			NotUsed = 0;
  float		NotUsedf = 0.0;
  float tdist,dx,dy,alpha;
  void	calculateRotVel(float timediff);
  void	calculateTransVel(float timediff);
  void	executeRot(float timediff);
  void	executeTrans(float timediff, float *x, float *y);
  
  if (!robot.placed) {		/*** robot not set  into window at begin ***/
    retvalHandleMove = 0;
    return;
  }
  
  if (robot.changePos) {	/*** do this only the first time because of inaccuracy when casting ***/
    x = robot.map_robot_x;	/*** the new position at the end of function			    ***/
    y = robot.map_robot_y;
    
    robot.changePos = FALSE;
  }

  oldx = x;
  oldy = y;
  
  timeDiff = -BaseVariables.time;		/*** calculate time to get correct velocities ***/
  
  gettimeofday(&tv_base, NULL);
  BaseVariables.time  = tv_base.tv_sec - tv_start.tv_sec;
  BaseVariables.time += (tv_base.tv_usec - tv_start.tv_usec) / 1e6;
  
  timeDiff += BaseVariables.time;

/*  
  if (timeDiff > 0.5)			
    timeDiff = 0.0;
    */
  
  /***** calculate velocities *****/
  
  calculateRotVel(timeDiff);
  calculateTransVel(timeDiff);
  
  
  /***** perform moves *****/
  
  executeRot(timeDiff);
  executeTrans(timeDiff, &x, &y);
  

  if (KI_OR_SH == 1) {
    robot.runflag = 0;			/*** SH or KI ***/
    retvalHandleMove = 0;
    return;
  }
  
  
  /***** don't run over obstacle *****/

  dx = x-oldx;
  dy = y - oldy;

  alpha = atan2(dy, dx);

  if(!get_distance(EXACT_SENSOR, oldx, oldy,
		   0.0, M_PI-0.00001,
		   x+robot.RobotRadius*cos(alpha),
		   y+robot.RobotRadius*sin(alpha), &tdist)
     &&
     (obstacles_min_distance(playground_x(x),
			    playground_y(y)) > robot.RobotRadius)) { 
      robot.map_robot_x = x;
      robot.map_robot_y = y;

#ifndef UNIBONN
      tactileReset();
#endif
      BaseVariables.bump = 0;

      retvalHandleMove = 1;
  }
  else {

#ifndef UNIBONN
    tactileHit(x, y); 
#endif
    x = oldx;	   
    y = oldy; 
#ifndef UNIBONN
    BaseVariables.bump = 1;    
#else
    BaseVariables.bump = 0;    
#endif
    retvalHandleMove = 0;
  } 
} /* HandleMove() */


/**********************************************************************************************************/
/* PROCEDURE :			calculateRotVel()			***********************************/
/*				calculateTransVel()			***********************************/
/* Parameter :			timeDiff				***********************************/
/*									***********************************/
/* Uses time difference to last call to calculate new velocity in       ***********************************/
/* dependency to acceleration						***********************************/
/**********************************************************************************************************/


void
calculateRotVel(float timeDiff)
{
  float		wanted_rvel;
  int			dir_r;
  
  
  wanted_rvel = (!BaseVariables.RotateHaltFlag) ? BaseVariables.RotateWantedVelocity : 0.0;
  dir_r	= BaseVariables.RotateDirection;
  wanted_rvel *= dir_r;
  
  if (slowDownRot) {
    wanted_rvel = 0.0;
  }
  
  if (BaseVariables.RotateCurrentVelocity < wanted_rvel) {
    BaseVariables.RotateCurrentVelocity += timeDiff * BaseVariables.RotateAcceleration;
    if (BaseVariables.RotateCurrentVelocity > wanted_rvel) 
      BaseVariables.RotateCurrentVelocity = wanted_rvel; 
  }
  if (BaseVariables.RotateCurrentVelocity > wanted_rvel) {
    BaseVariables.RotateCurrentVelocity -= timeDiff * BaseVariables.RotateAcceleration;
    if (BaseVariables.RotateCurrentVelocity < wanted_rvel) 
      BaseVariables.RotateCurrentVelocity = wanted_rvel; 
  }
  
  /*
     #ifdef BASE_DEBUG
     fprintf(stderr, "Actual Rotate Velocity : %f\n", BaseVariables.RotateCurrentVelocity);
     #endif
     */
  
  return;
  
} /* calculateRotVel() */



void
calculateTransVel(float timeDiff)
{
  float		wanted_tvel;
  int			dir_t;
  
  wanted_tvel = (!BaseVariables.TranslateHaltFlag) ? BaseVariables.TranslateWantedVelocity : 0.0;
  dir_t	= BaseVariables.TranslateDirection;
  wanted_tvel *= dir_t;
  
  
  if (slowDownTrans) {
    wanted_tvel = 0.0;
  }
  
  if (BaseVariables.TranslateCurrentVelocity < wanted_tvel) {
    BaseVariables.TranslateCurrentVelocity += timeDiff * BaseVariables.TranslateAcceleration;
    if (BaseVariables.TranslateCurrentVelocity > wanted_tvel) 
      BaseVariables.TranslateCurrentVelocity = wanted_tvel; 
  }
  if (BaseVariables.TranslateCurrentVelocity > wanted_tvel) {
    BaseVariables.TranslateCurrentVelocity -= timeDiff * BaseVariables.TranslateAcceleration;
    if (BaseVariables.TranslateCurrentVelocity < wanted_tvel) 
      BaseVariables.TranslateCurrentVelocity = wanted_tvel; 
  }
  
  /*
     #ifdef BASE_DEBUG
     fprintf(stderr, "Actual Translate Velocity : %f\n", BaseVariables.TranslateCurrentVelocity);
     #endif
     */
  
  return;
  
} /* calculateTransVel() */


/**********************************************************************************************************/
/* PROCEDURE :			executeRot()				***********************************/
/*				executeTrans()				***********************************/
/* Parameter :			timeDiff				***********************************/
/*									***********************************/
/* Performs move in dependency to internal base variables set in base() ***********************************/
/* and calculated in calculateRot/TransVel				***********************************/
/**********************************************************************************************************/


void
executeRot(float timeDiff)
{
  float  tmp;
  float  dist;
  
  if (BaseVariables.RotateRelativeFlag) {
    tmp = BaseVariables.RotateCurrentVelocity * timeDiff;
    dist = BaseVariables.RotateRelativePos - tmp;
    
    if ((mySGN(BaseVariables.RotateRelativePos) == mySGN(dist)) && 
	(BaseVariables.RotateRelativePos != 0)) {
      BaseVariables.RotateRelativePos -= tmp;
      BaseVariables.RotateWhere += tmp;
      /*** stop slowly ***/
      if ((BaseVariables.RotateCurrentVelocity != 0.0) && 
	  (myABS(BaseVariables.RotateRelativePos) >= (2 * BaseVariables.RotateAcceleration * 
						      mySQR(BaseVariables.RotateRelativePos / BaseVariables.RotateCurrentVelocity)))) {
	slowDownRot = 1;
      } else {
	if (BaseVariables.RotateCurrentVelocity == 0.0)
	  slowDownRot = 0;
      }
    } else {								    /*** turned too much ***/
      BaseVariables.RotateWhere += BaseVariables.RotateRelativePos;
      BaseVariables.RotateRelativePos = 0.0;
      BaseVariables.RotateRelativeFlag = 0;
      BaseVariables.RotateHaltFlag = 1;
      BaseVariables.RotateCurrentVelocity = 0.0;
      slowDownRot = 0;
    }
    
  } else {						 /*** Rotate Constant Velocity ***/
    BaseVariables.RotateWhere += BaseVariables.RotateCurrentVelocity * timeDiff;
  }
  
  if (BaseVariables.RotateWhere < 0.0)
    BaseVariables.RotateWhere += 360.0;
  if (BaseVariables.RotateWhere >= 360.0)
    BaseVariables.RotateWhere -= 360.0;
  
  robot.robot_deg = BaseVariables.RotateWhere;
  
} /* executeRot() */


void
executeTrans(float timeDiff, float *x, float *y)
{
  float 	diff;
  float 	dist;
  float 	tmp;
  
  if (BaseVariables.TranslateRelativeFlag) {	
    
    tmp = BaseVariables.TranslateCurrentVelocity * timeDiff;
    dist = BaseVariables.TranslateRelativePos - tmp;
    
    if ((mySGN(BaseVariables.TranslateRelativePos) == mySGN(dist)) && 
	(BaseVariables.TranslateRelativePos != 0)) {
      BaseVariables.TranslateRelativePos -= tmp;
      BaseVariables.TranslateWhere += tmp;
      diff = tmp;
      /*** stop slowly ***/
      if ((BaseVariables.TranslateCurrentVelocity != 0.0) && 
	  (myABS(BaseVariables.TranslateRelativePos) >= (2 * BaseVariables.TranslateAcceleration * 
							 mySQR(BaseVariables.TranslateRelativePos / BaseVariables.TranslateCurrentVelocity)))) {
	slowDownTrans = 1;
      } else {
	if (BaseVariables.TranslateCurrentVelocity == 0.0)
	  slowDownTrans = 0;
      }
    } else {								    /*** moved too much ***/
      BaseVariables.TranslateWhere += BaseVariables.TranslateRelativePos;
      diff = BaseVariables.TranslateRelativePos;
      BaseVariables.TranslateRelativePos = 0;
      BaseVariables.TranslateRelativeFlag = 0;
      BaseVariables.TranslateHaltFlag = 1;
      BaseVariables.TranslateCurrentVelocity = 0.0;
      slowDownTrans = 0;
    }
    
    *x += diff*myCOS(robot.robot_deg*M_PI/180);
    *y += diff*mySIN(robot.robot_deg*M_PI/180);
    
  } else {						 /*** Translate Constant Velocity ***/
    BaseVariables.TranslateWhere +=  BaseVariables.TranslateCurrentVelocity * timeDiff;
    diff = BaseVariables.TranslateCurrentVelocity * timeDiff;
    *x += diff*myCOS(robot.robot_deg*M_PI/180);
    *y += diff*mySIN(robot.robot_deg*M_PI/180);
  }
  
} /* executeTrans() */


/**********************************************************************
 *
 * Routines to generate binary MCP packets needed by baseServer - TDS
 *
 **********************************************************************/

static mcpPacketType *
mcpPacketNew(void) 
{
  mcpPacketType *new;

  new = malloc(sizeof(*new));

  if (!new) {
    return(NULL);
  }

  new->size = 0;
  new->chksum = 0;

  return(new);
}

static int
mcpPacketAddChar(mcpPacketType *this, unsigned char data)
{
  this->data1[this->size++]=data;

  if (!this->size) {
    this->size--;
    return(-1);
  }

  this->chksum ^= data;

  return(0);
}

static unsigned char *
mcpPacketData(mcpPacketType *this)
{
  int byteCount;
  int size;
  unsigned char *srcPtr;
  unsigned char *dstPtr;

  size = this->size;
  byteCount=0;

  srcPtr = this->data1;
  dstPtr = this->data;

  *dstPtr++ = 2;
  *dstPtr++ = 2;

  *dstPtr++ = (unsigned char)size;
  if ((size==2) || (size==3)) {
    *dstPtr++ = 0;
  }

  while(size--) {
    *dstPtr = *srcPtr++;

    if ((*dstPtr==2) || (*dstPtr==3)) {
      dstPtr++;
      *dstPtr = 0;
    }
    dstPtr++;
  }

  *dstPtr = this->chksum;
  if ((*dstPtr==2) || (*dstPtr==3)) {
    dstPtr++;
    *dstPtr = 0;
  }
  dstPtr++;

  *dstPtr++ = 3;
  *dstPtr++ = 3;

  return(this->data);
}

static int
mcpPacketDestroy(mcpPacketType *this)
{
  free(this);
  return(0);
}

/**********************************************************************************************************/
/* PROCEDURE :			statusReport()				***********************************/
/* Parameter :			none					***********************************/
/*									***********************************/
/* sends status report via TCX to MODULE_BASE				***********************************/
/**********************************************************************************************************/

void
statusReport()
{
  long 		time_in_256_sec = 0;
  char		ret[81];
  int		interval;
  mcpPacketType     *mcpPkt;
  
#ifdef BASE_DEBUG
      fprintf(stderr, "%10s:%5d:%14s()\n",
	      __FILE__, __LINE__, __FUNCTION__);
#endif

  if (MODULE_BASE != NULL) {
    if (BaseVariables.watch_dog) {

      /*
       * works only when update interval < watch dog interval
       */

      BaseVariables.watch_dog -= BaseVariables.StatusReportPeriod; 

      /*
       * Make sure that we don't happen to "switch off" the timer.
       */

      if (!BaseVariables.watch_dog) BaseVariables.watch_dog = -1;
    }

    if (use_baseServer == FALSE) {
      char *reply_msg;

      reply_msg = (char *) malloc(256);

#ifdef BASE_DEBUG
      fprintf(stderr, "%10s:%5d:%14s(): \n",
	      __FILE__, __LINE__, __FUNCTION__);
#endif

      if (BaseVariables.watch_dog < 0) {
	fprintf(stderr,"watch_dog timeout (%d)!\n",BaseVariables.watch_dog);
	BaseVariables.RotateHaltFlag = 1;
	BaseVariables.TranslateHaltFlag = 1;

	/*** send a message (which one ???) ***/
	strcpy(reply_msg, "%%WDT motors stopped");
	
#ifdef BASE_DEBUG
	fprintf(stderr, "%10s:%5d:%14s(): \n",
		__FILE__, __LINE__, __FUNCTION__);
#endif

	tcxSendMsg(MODULE_BASE, "SIMULATOR_message_to_base", &reply_msg); 
      }
    
      strcpy(reply_msg, "%PSR");
      tcxSendMsg(MODULE_BASE, "SIMULATOR_message_to_base", &reply_msg);
    
    
      strcpy(reply_msg, "CLK ");
      gettimeofday(&tv_base, NULL);
      time_in_256_sec = (tv_base.tv_sec - tv_start.tv_sec) * 256;
      time_in_256_sec += (tv_base.tv_usec - tv_start.tv_usec) * 256 / 1000000;
      sprintf(ret, "%8x", time_in_256_sec);
      strcat(reply_msg, ret);
      tcxSendMsg(MODULE_BASE, "SIMULATOR_message_to_base", &reply_msg);
    
    
      if (BaseVariables.bump == 1)
	strcpy(reply_msg, "BBS FFFF");
      else
	strcpy(reply_msg, "BBS 0000");
      tcxSendMsg(MODULE_BASE, "SIMULATOR_message_to_base", &reply_msg);
    
    
      strcpy(reply_msg, "BPX ");
      sprintf(ret, "%4x", (unsigned int) (base_coord_x() + 32768));
      strcat(reply_msg, ret);
      tcxSendMsg(MODULE_BASE, "SIMULATOR_message_to_base", &reply_msg);
    
    
      strcpy(reply_msg, "BPY ");
      sprintf(ret, "%4x", (unsigned int) (base_coord_y() + 32768));
      strcat(reply_msg, ret);
      tcxSendMsg(MODULE_BASE, "SIMULATOR_message_to_base", &reply_msg);
    
    
      strcpy(reply_msg, "TPH ");
      sprintf(ret, "%4x", (long) (base_coord_rot()*COUNTS_PER_DEG));
      strcat(reply_msg, ret);
      tcxSendMsg(MODULE_BASE, "SIMULATOR_message_to_base", &reply_msg);
    
    
      strcpy(reply_msg, "TVE ");
      sprintf(ret, "%4x", myABS ((int) (BaseVariables.TranslateCurrentVelocity*COUNTS_PER_CM)));
      strcat(reply_msg, ret);
      tcxSendMsg(MODULE_BASE, "SIMULATOR_message_to_base", &reply_msg);
    
      if (BaseVariables.TranslateDirection == -1)
	strcpy(reply_msg, "TSF FFFF");
      else
	strcpy(reply_msg, "TSF 0000");
    
      tcxSendMsg(MODULE_BASE, "SIMULATOR_message_to_base", &reply_msg);
    
    
      strcpy(reply_msg, "RVE ");
      sprintf(ret, "%4x", myABS ((int) (BaseVariables.RotateCurrentVelocity*COUNTS_PER_DEG)));
      strcat(reply_msg, ret);
      tcxSendMsg(MODULE_BASE, "SIMULATOR_message_to_base", &reply_msg);
    
    
      if (BaseVariables.RotateDirection != -1)
	strcpy(reply_msg, "RSF FFFF");
      else
	strcpy(reply_msg, "RSF 0000");

#ifdef BASE_DEBUG
      fprintf(stderr, "%10s:%5d:%14s(): \n",
	      __FILE__, __LINE__, __FUNCTION__);
#endif

      tcxSendMsg(MODULE_BASE, "SIMULATOR_message_to_base", &reply_msg);
      free(reply_msg);
    
    }
    else { /* *** use_baseServer *** */
      unsigned int temp;

#ifdef BASE_DEBUG
      fprintf(stderr, "%10s:%5d:%14s(): \n",
	      __FILE__, __LINE__, __FUNCTION__);
#endif

      if (BaseVariables.watch_dog < 0) {
	fprintf(stderr, "%s:%6d:%s() - watch_dog timeout (%d)!\n",
		__FILE__, __LINE__, __FUNCTION__, BaseVariables.watch_dog);
	BaseVariables.watch_dog = 0;
	BaseVariables.RotateHaltFlag = 1;
	BaseVariables.TranslateHaltFlag = 1;

	mcpPkt = mcpPacketNew();
	mcpPacketAddChar(mcpPkt, 0x16);

#ifdef BASE_DEBUG
	fprintf(stderr, "%10s:%5d:%14s(): \n",
		__FILE__, __LINE__, __FUNCTION__);
#endif

	tcxSendMsg(MODULE_BASE, "SIMULATOR_message_to_baseServer",
		   mcpPacketData(mcpPkt)); 
	mcpPacketDestroy(mcpPkt);
      }
    
      mcpPkt = mcpPacketNew();
      mcpPacketAddChar(mcpPkt, 0x11); /* PSR */

      mcpPacketAddChar(mcpPkt, 0x00); /* SRD 3 */
      mcpPacketAddChar(mcpPkt, 0x0A); /* SRD 2 */
      mcpPacketAddChar(mcpPkt, 0x0A); /* SRD 1 */
      mcpPacketAddChar(mcpPkt, 0xF3); /* SRD 0 */
    
      gettimeofday(&tv_base, NULL);
      time_in_256_sec = (tv_base.tv_sec - tv_start.tv_sec) * 256;
      time_in_256_sec += (tv_base.tv_usec - tv_start.tv_usec) * 256 / 1000000;

      temp = (unsigned int)time_in_256_sec;

#ifdef BASE_DEBUG
      fprintf(stderr, "%10s:%5d:%14s(): CLK = %8X\n",
	      __FILE__, __LINE__, __FUNCTION__, temp);
#endif

      mcpPacketAddChar(mcpPkt, (temp>>24) & 0xFF); /* CLK 3 */
      mcpPacketAddChar(mcpPkt, (temp>>16) & 0xFF); /* CLK 2 */
      mcpPacketAddChar(mcpPkt, (temp>> 8) & 0xFF); /* CLK 1 */
      mcpPacketAddChar(mcpPkt, (temp    ) & 0xFF); /* CLK 0 */

#if 0 /* baseServer doesn't expect BBS in status report - It should */
      if (BaseVariables.bump == 1) {
	temp = 0xFFFF;
      }
      else {
	temp = 0;
      }
      mcpPacketAddChar(mcpPkt, temp); /* BBS 1 */
      mcpPacketAddChar(mcpPkt, temp); /* BBS 0 */
#endif
    
      temp = (unsigned int)((base_coord_x()-(float)0x04BD)*
			    bRobot.base_posPerCm) + 0x8000;

#ifdef BASE_DEBUG
      fprintf(stderr, "%10s:%5d:%14s(): base-X = 0x%04X\n",
	      __FILE__, __LINE__, __FUNCTION__, temp);
#endif

      mcpPacketAddChar(mcpPkt, (temp>>8) & 0xFF); /* BPX 1 */
      mcpPacketAddChar(mcpPkt, (temp   ) & 0xFF); /* BPX 0 */
    
      temp = (unsigned int)((base_coord_y()-(float)0x04BD)*
			    bRobot.base_posPerCm) + 0x8000;

#ifdef BASE_DEBUG
      fprintf(stderr, "%10s:%5d:%14s(): base-Y = %4X\n",
	      __FILE__, __LINE__, __FUNCTION__, temp);
#endif

      mcpPacketAddChar(mcpPkt, (temp>>8) & 0xFF); /* BPY 1 */
      mcpPacketAddChar(mcpPkt, (temp   ) & 0xFF); /* BPY 0 */
    
      temp = (unsigned int)(base_coord_rot()*COUNTS_PER_DEG);

#ifdef BASE_DEBUG
      fprintf(stderr, "%10s:%5d:%14s(): base-Heading = %4X\n",
	      __FILE__, __LINE__, __FUNCTION__, temp);
#endif

      mcpPacketAddChar(mcpPkt, (temp>>8) & 0xFF); /* TPH 1 */
      mcpPacketAddChar(mcpPkt, (temp   ) & 0xFF); /* TPH 0 */

      temp = (unsigned int)(robot.base_BRH*COUNTS_PER_DEG);

#ifdef BASE_DEBUG
      fprintf(stderr, "%10s:%5d:%14s(): BRH = %4X\n",
	      __FILE__, __LINE__, __FUNCTION__, temp);
#endif

      mcpPacketAddChar(mcpPkt, (temp>>8) & 0xFF); /* BRH 1 */
      mcpPacketAddChar(mcpPkt, (temp   ) & 0xFF); /* BRH 0 */

      temp = myABS ((int)
		    (BaseVariables.TranslateCurrentVelocity*COUNTS_PER_CM));

#ifdef BASE_DEBUG
      fprintf(stderr, "%10s:%5d:%14s(): transVel = %4X\n",
	      __FILE__, __LINE__, __FUNCTION__, temp);
#endif

      mcpPacketAddChar(mcpPkt, (temp>>8) & 0xFF); /* TVE 1 */
      mcpPacketAddChar(mcpPkt, (temp   ) & 0xFF); /* TVE 0 */

      temp = 0;

      /*
       * _HACK_
       * Assume that calculated, torque, measured and PWM 
       * direction are all the same.
       */

      if (BaseVariables.TranslateDirection == -1) {
	temp |= 0x1D00;
      }

      /*
       * _HACK_
       * Assume constant vel if not halted.
       */

      if (BaseVariables.TranslateHaltFlag) {
	temp |= 0x0004;
      }
      else {
	temp |= 0x4002;
      }

#ifdef BASE_DEBUG
      fprintf(stderr, "%10s:%5d:%14s(): transStatus = %4X\n",
	      __FILE__, __LINE__, __FUNCTION__, temp);
#endif

      mcpPacketAddChar(mcpPkt, (temp>>8) & 0xFF); /* TSF 1 */
      mcpPacketAddChar(mcpPkt, (temp   ) & 0xFF); /* TSF 0 */

      temp = myABS((int)
		   (BaseVariables.RotateCurrentVelocity*COUNTS_PER_DEG));


#ifdef BASE_DEBUG
      fprintf(stderr, "%10s:%5d:%14s(): rotVel = %4X\n",
	      __FILE__, __LINE__, __FUNCTION__, temp);
#endif

      mcpPacketAddChar(mcpPkt, (temp>>8) & 0xFF); /* RVE 1 */
      mcpPacketAddChar(mcpPkt, (temp   ) & 0xFF); /* RVE 0 */
    

      temp = 0;

      /*
       * _HACK_
       * Assume that calculated, torque, measured and PWM 
       * direction are all the same.
       */

      if (BaseVariables.RotateDirection != -1) {
	temp |= 0x1D00;
      }

      /*
       * _HACK_
       * Assume constant vel if not halted.
       */

      if (BaseVariables.RotateHaltFlag) {
	temp |= 0x0004;
      }
      else {
	temp |= 0x4002;
      }

#ifdef BASE_DEBUG
      fprintf(stderr, "%10s:%5d:%14s(): rotStatus = %4X\n",
	      __FILE__, __LINE__, __FUNCTION__, temp);
#endif

      mcpPacketAddChar(mcpPkt, (temp>>8) & 0xFF); /* RSF 1 */
      mcpPacketAddChar(mcpPkt, (temp   ) & 0xFF); /* RSF 0 */

#ifdef BASE_DEBUG
      fprintf(stderr, "%10s:%5d:%14s(): \n", __FILE__, __LINE__, __FUNCTION__);
#endif

      tcxSendMsg(MODULE_BASE, "SIMULATOR_message_to_baseServer",
		 mcpPacketData(mcpPkt));

      mcpPacketDestroy(mcpPkt);
    } /* use_baseServer */
    
  }  /* if (MODULE_BASE != NULL) */
  
  return;
  
} /* statusReport() */




/**********************************************************************************************************/
/* PROCEDURE :			InitBaseVar()				***********************************/
/* Parameter :			none					***********************************/
/*									***********************************/
/* Initialization of global Base Variables				***********************************/
/**********************************************************************************************************/


void
InitBaseVar()
{
  BaseVariables.RotateWantedVelocity		= 0.0;
  BaseVariables.RotateCurrentVelocity		= 0.0;
  BaseVariables.RotateAcceleration		= 0.0;

  BaseVariables.RotateWhere			= robot.robot_deg; /* 0.0 */
  base_init_rot = (ROBOT_INIT_ROT - robot.robot_start_deg);
  BaseVariables.RotateDirection		= 0;
  BaseVariables.RotateRelativeFlag		= 0;
  BaseVariables.RotateHaltFlag			= 0;
  
  BaseVariables.TranslateWantedVelocity	= 0.0;
  BaseVariables.TranslateCurrentVelocity	= 0.0;
  BaseVariables.TranslateAcceleration		= 0.0;

  BaseVariables.TranslateWhere			= 0.0;

  BaseVariables.TranslateDirection		= 0;
  BaseVariables.TranslateRelativeFlag		= 0;
  BaseVariables.TranslateHaltFlag		= 0;
  
  BaseVariables.bump				= 0;
  
  BaseVariables.time				= 0.0;
  BaseVariables.watch_dog			= 1000;	/*** avoid error at begin ***/
  
   gettimeofday(&tv_start, NULL);		/*** tv_start needed to calculate time base is running ***/
  
  return;
} /* InitBaseVar() */

static struct {
  int n;
  float x;
  float y;
  } point;

void base_sendTarget(float x, float y)
{
  float tmp;
  float rot = M_PI/180 * base_init_rot;
  tmp = x-robot.map_robot_start_x;
  y = y-robot.map_robot_start_y;
  x = tmp*cos(rot)-y*sin(rot)+ROBOT_INIT_POS_X+base_correct_x;
  y = tmp*sin(rot)+y*cos(rot)+ROBOT_INIT_POS_Y+base_correct_y;
  point.n = 0;
  point.x = x;
  point.y = y;

#ifdef BASE_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif
  tcxSendMsg(MODULE_BASE, "BASE_goto_absolute", &point);
}
