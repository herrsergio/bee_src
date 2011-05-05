
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





#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <math.h>

#include "bUtils.h"
#include "tcx.h"
#include "tcxP.h"
#include "Common.h"
#include "libc.h"
#include "ROBOT.h"
#include "rwibase_interface.h"
#include "LIB-math.h"
#include "server.h"
#include "sonar_interface.h"
#include "laser_interface.h"

#include "robot_specifications.h"
#include "BASE-messages.h"
#include "SONAR-messages.h"
#include "COLLI-messages.h"
#ifdef UNIBONN
#include "SOUND-messages.h"
#include "SUNVIS-messages.h"
#include "LASER-messages.h"
#endif /* UNIBONN */
#include "LASER_SERVER-messages.h"

#include "SIMULATOR-messages.h"
/* #include "base-handlers.h" */
#include "colli-handlers.h"
#include "collisionIntern.h"

#ifdef CLEANUP
#include "colliCleanUp.h"
#endif /* CLEANUP */

#if defined(__STDC__) && defined(__GCC_NEW_VARARGS__)
typedef void (*VOID_FN)(void);
#elif  defined(__STDC__)
typedef void (*VOID_FN)(void);
#else /* __STDC__ */
typedef void (*VOID_FN)();
#endif

typedef struct command {
  char *name;
  VOID_FN func;
  int  n_parameters;
} command;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

struct bParamList * bParamList = NULL;
const char *bRobotType;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/* from server.c */
extern int DONE_INDEXING;

extern int  listen_for_tcx_events;	/* in devUtils.c */
/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

BOOLEAN use_sonar;
BOOLEAN use_laser;		/* 1, if we want to use laser */
BOOLEAN use_laser_server;	/* 1, if the laser data comes form laserserver */
BOOLEAN msp_sonar;

BOOLEAN use_collision;
BOOLEAN use_simulator;
BOOLEAN use_rwi_server;
BOOLEAN use_vision;
BOOLEAN use_bumper;
BOOLEAN use_ir;
BOOLEAN use_arm;
int use_router = 0;
int use_keyboard = 1;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

static void Help(void);
static void Quit(void);
static void CurrentPosition(void);
static void BroadcastStatusReport(Pointer callback_data, Pointer client_data);
static void StopHandler(int callback_data, Pointer client_data);
static void DebugOn(void);
static void DebugOff(void);
static void Rotate(double degrees);
static void Status(void);

static void setMode(double param);
static void SonarChangeMask(double param);
static void SonarLoopStart(double param);
static void BroadcastSonarReport(Pointer callback_data, Pointer client_data);

void stdin_inputHnd(int fd, long chars_available);

static command table_commands[] = {
  {"SLS", (VOID_FN) SonarLoopStart, 1},
  {"SLE", (VOID_FN) SONAR_LoopEnd, 0},
  {"SLI", (VOID_FN) SONAR_SetLoopIntervall, 1},
  {"SCM", (VOID_FN) SonarChangeMask, 1},
  {"HOME", (VOID_FN) BASE_SetRotationReference, 0},
  {"BQ", (VOID_FN) BASE_QueryBatteryStatus, 0},
  {"STAT", (VOID_FN) Status, 0},

  {"RH", (VOID_FN) BASE_RotateHalt, 0},
  {"RA", (VOID_FN) BASE_RotateAcceleration, 1},
  {"RV", (VOID_FN) BASE_RotateVelocity, 1},
  {"R+", (VOID_FN) BASE_RotateClockwise, 0},
  {"R-", (VOID_FN) BASE_RotateAnticlockwise, 0},
  {"RR", (VOID_FN) BASE_Rotate, 1},
  
  {"TH", (VOID_FN) BASE_TranslateHalt, 0},
  {"TA", (VOID_FN) BASE_TranslateAcceleration, 1},
  {"TV", (VOID_FN) BASE_TranslateVelocity, 1},
  {"T+", (VOID_FN) BASE_TranslateForward, 0},
  {"T-", (VOID_FN) BASE_TranslateBackward, 0},
  {"TR", (VOID_FN) BASE_Translate, 1},
  
  {"CTR", (VOID_FN) COLLI_TranslateRelative, 2},
     
  {"CSAD", (VOID_FN) COLLI_SetApproachDist, 1},
  {"CSM", (VOID_FN) setMode, 1},
  {"C+", (VOID_FN) COLLI_GoForward, 0},
  {"C-", (VOID_FN) COLLI_GoBackward, 0},

  {"CDUMP",(VOID_FN) COLLI_DumpInfo, 0},
  {"CMARK",(VOID_FN) COLLI_MarkInfo, 0},
  {"CGNU",(VOID_FN) COLLI_StartToDumpGnuPlot, 1},

  {"D", (VOID_FN)  DebugOn, 0},
  {"ND", (VOID_FN) DebugOff, 0},
  {"STOP", (VOID_FN) stop_robot, 0},
  {"?", (VOID_FN) Help, 0},
  {".", (VOID_FN) Quit, 0},

#ifdef CLEANUP
  /* Functions for clean up the office. */
  {"CSC", (VOID_FN) COLLI_StartCleaningUp, 0},
  {"CHC", (VOID_FN) COLLI_StopCleaningUp, 0},
  {"CFTB", (VOID_FN) COLLI_SimulateFarTrashBin, 2},
  {"CFO", (VOID_FN) COLLI_SimulateFarObject, 2},
  {"CCTB", (VOID_FN) COLLI_SimulateCloseTrashBin, 2},
  {"CCO", (VOID_FN) COLLI_SimulateCloseObject, 2},

  {"A1",(VOID_FN) ARM_moveToGround, 0}, 
  {"A2",(VOID_FN) ARM_liftObject, 0}, 
  {"A3",(VOID_FN) ARM_dropObject, 0},
  {"A4",(VOID_FN) ARM_moveIn, 0}, 

  {"A1R",(VOID_FN) ARM_moveToGroundReady, 1}, 
  {"A2R",(VOID_FN) ARM_liftObjectReady, 1}, 
  {"A3R",(VOID_FN) ARM_dropObjectReady, 1},
  {"A4R",(VOID_FN) ARM_moveInReady, 1}, 
#endif

  {(char *) NULL, (VOID_FN) NULL, 0}
};


static void Help(void)
{
  printf("BASE Commands:\n");
  printf("RP                      -- Rotation possible\n");
  printf("RB distance             -- RollBack\n");

  
  printf("HOME                    -- Rotation home\n");
  printf("BQ                      -- Battery query\n");
  printf("STAT                    -- Status report\n\n");
  printf("RH                      -- Rotate halt\n");
  printf("RA acceleration         -- Set rotation acceleratio\n");
  printf("RV velocity             -- Set rotation velocity\n");
  printf("R+                      -- Rotate clockwise\n");
  printf("R-                      -- Rotate anticlockwise\n");
  printf("RR degrees              -- Rotate degrees\n\n");
  printf("TH                      -- Translate halt\n");
  printf("TA acceleration         -- Set translation acceleration\n");
  printf("TV velocity             -- Set translation velocity\n");
  printf("T+                      -- Translate forward\n");
  printf("T-                      -- Translate backward\n");
  printf("TR cm                   -- Translate cm\n\n");
  
  printf("SONAR Commands:\n");
  printf("SLS maskno              -- Start loop to get sonarReadings (use maskno)\n");
  printf("SLE                     -- End loop to get sonarReadings\n");
  printf("SLI no                  -- Set number of RT commands to be sent at once\n");
  printf("SCM maskno              -- Change maskno for loop sequence\n");

  printf("COLLISION AVOIDANCE Commands:\n");
  printf("CSM                     -- Set mode for target trajectory\n");
  printf("                           0: DEFAULT_MODE\n");
  printf("                           1: FAST_TRAVEL_MODE\n");
  printf("                           2: FIND_DOOR_MODE\n");
  printf("                           3: SERVO_MODE\n");
  printf("                           4: ARM_OUT_MODE\n");
  printf("                           5: RANDOM_MODE\n");
  printf("                           6: APPROACH_OBJECT_MODE\n");
  printf("                           7: APPROACH_TRASH_BIN_MODE\n");
  printf("                           8: ARM_OUT_RANDOM_MODE\n");
  printf("CTR                     -- Translate relative (forward, sideward)\n");

#ifdef CLEANUP
  printf("CSC                     -- Colli start cleaning up\n");
  printf("CHC                     -- Colli stop cleaning up\n");
  printf("A1                      -- ARM move to ground.\n");
  printf("A2                      -- ARM lift object.\n");
  printf("A3                      -- ARM drop object.\n");
  printf("A4                      -- ARM move in.\n");
#endif
  
  printf("C+                      -- Go forward to target\n");
  printf("C-                      -- Go backward to target\n");
  printf("CDUMP                   -- Write collision information\n");
  printf("CMARK                   -- Write a mark in the dump file\n");
  printf("CGNU                    -- Write gnuplot file of evaluation function\n");
  
  printf("\nD                       -- Debug: set base debug on\n");
  printf("ND                      -- No Debug: set base debug off\n");
  printf("STOP                    -- Stop movement until next command\n");
  printf("?                       -- Help\n");
  printf(".                       -- Exit\n");
}


static void setMode(double param)
{
  if ( (int) param == FIND_DOOR_MODE) {
    fprintf(stderr, "Sonar off.\n");
    use_sonar = FALSE;
  }
  else if ( mode_number == FIND_DOOR_MODE)
    use_sonar = TRUE;
  COLLI_SetMode( (int) param);
}


static void SonarLoopStart(double param)
{
  int i;

  i = (int) param;

  if (i >= 0 && i < MAX_MASKS) 
    SONAR_LoopStart(sonar_mask_array[i]);
  else {
    fprintf(stderr, "wrong mask number (%i)! Use default number (0)\n", i);
    SONAR_LoopStart(sonar_mask_array[0]);
  }
}


static void SonarChangeMask(double param)
{
  int i;

  i = (int) param;

  if (i < MAX_MASKS) 
    SONAR_ChangeMask(sonar_mask_array[i]);
  else 
    fprintf(stderr, "Wrong mask number (%i)! Can't change mask\n", i);
}



static void BroadcastSonarReport(Pointer callback_data, Pointer client_data)
{
  int i, okCnt  = 0;

  for(i=0; i<24; i++) {
    sonar_tcx_status.values[i] = (float) sonar_readings[i];
    if ( sonar_tcx_status.values[i] > 0)
      okCnt++;
  }
  if ( okCnt < 5)
    fprintf(stderr, "Huch error already in send.\n");
  
  send_automatic_sonar_update();
}


static void BroadcastLaserReport(Pointer callback_data, Pointer client_data)
{
  int i;

  /* If the number of points has changed we reallocate memory. */
  if ( laser_tcx_status.f_numberOfReadings != frontLaserReading.numberOfReadings) {
    if ( laser_tcx_status.f_numberOfReadings > 0)
      free ( laser_tcx_status.f_reading);
    
    if (frontLaserReading.numberOfReadings > 0) 
      laser_tcx_status.f_reading = (int *) 
	malloc( frontLaserReading.numberOfReadings * sizeof(int));
    else
      laser_tcx_status.f_reading = (int *) NULL; 
  }

  /* Copy the readings. */
  laser_tcx_status.f_numberOfReadings = frontLaserReading.numberOfReadings;
  laser_tcx_status.f_startAngle       = frontLaserReading.startAngle;
  laser_tcx_status.f_angleResolution  = frontLaserReading.angleResolution;

  for ( i = 0; i < frontLaserReading.numberOfReadings; i++)
    laser_tcx_status.f_reading[i] = (int) frontLaserReading.reading[i];

  
  /* If the number of points has changed we reallocate memory. */
  if (laser_tcx_status.r_numberOfReadings != rearLaserReading.numberOfReadings) {
    if ( laser_tcx_status.r_numberOfReadings > 0)
      free (laser_tcx_status.r_reading);
    
    if ( rearLaserReading.numberOfReadings > 0) 
      laser_tcx_status.r_reading = (int *) 
	malloc( rearLaserReading.numberOfReadings * sizeof(int));
    else
      laser_tcx_status.r_reading = (int *) NULL; 
  }

  laser_tcx_status.r_numberOfReadings = rearLaserReading.numberOfReadings;
  laser_tcx_status.r_startAngle       = rearLaserReading.startAngle;
  laser_tcx_status.r_angleResolution  = rearLaserReading.angleResolution;

  for ( i = 0; i < rearLaserReading.numberOfReadings; i++)
    laser_tcx_status.r_reading[i] = (int) rearLaserReading.reading[i];

  laser_tcx_status.xPos   = frontLaserReading.rPos.x;
  laser_tcx_status.yPos   = frontLaserReading.rPos.y;
  laser_tcx_status.rotPos = frontLaserReading.rRot;

  send_automatic_laser_update();
}


static void BroadcastStatusReport(Pointer callback_data, Pointer client_data)
{

  base_tcx_status.time                = (float) rwi_base.time;
  base_tcx_status.rot_acceleration    = (float) rwi_base.rot_acceleration;
  base_tcx_status.rot_current_speed   = (float) rwi_base.rot_current_speed;
  base_tcx_status.rot_set_speed       = (float) rwi_base.rot_set_speed;
  base_tcx_status.rot_position        = (float) rwi_base.rot_position;
  base_tcx_status.trans_acceleration  = (float) rwi_base.trans_acceleration;
  base_tcx_status.trans_current_speed = (float) rwi_base.trans_current_speed;
  base_tcx_status.trans_set_speed     = (float) rwi_base.trans_set_speed;
  base_tcx_status.trans_position      = (float) rwi_base.trans_position;
  base_tcx_status.pos_x               = (float) rwi_base.pos_x;
  base_tcx_status.pos_y               = (float) rwi_base.pos_y;
  base_tcx_status.orientation         = (float) rwi_base.rot_position;
  base_tcx_status.trans_direction     =
    (unsigned char)   rwi_base.trans_direction;
  base_tcx_status.trans_set_direction =
    (unsigned char)   rwi_base.trans_set_direction;
  base_tcx_status.rot_direction       =
    (unsigned char)   rwi_base.rot_direction;
  base_tcx_status.rot_set_direction   =
    (unsigned char)   rwi_base.rot_set_direction;
  base_tcx_status.rot_moving          =
    (unsigned char)   rwi_base.rot_moving;
  base_tcx_status.trans_moving        =
    (unsigned char)   rwi_base.trans_moving;
  base_tcx_status.bumpers             =
    (unsigned char)   rwi_base.bumpers;
  base_tcx_status.bump                =
    (unsigned char)   rwi_base.bump;
  base_tcx_status.emergency           =
    (unsigned char)   rwi_base.emergency;
  base_tcx_status.emergencyProcedure  =
    (unsigned char)   rwi_base.emergencyProcedure;
  base_tcx_status.collision_state.target_reached =
    (unsigned char) (!target_flag || rwi_base.collision_state.target_reached);

  base_tcx_status.targetDefined = target_flag;
  base_tcx_status.targetX       = target.x;
  base_tcx_status.targetY       = target.y;
  
  send_automatic_status_update();
}



static void Status(void)
{
  fprintf(stderr, "\nstatus report       :\n");
  fprintf(stderr, "CLOCK                 : %g \n", rwi_base.time);
  fprintf(stderr, "TRANS_POS_X           : %g \n", rwi_base.pos_x);
  fprintf(stderr, "TRANS_POS_Y           : %g \n", rwi_base.pos_y);
  fprintf(stderr, "TRANS_VELOC           : %g \n", rwi_base.trans_current_speed);
  fprintf(stderr, "TRANS_DIRECTION       : %i \n", rwi_base.trans_direction);
  fprintf(stderr, "ROT_POS               : %g \n", rwi_base.rot_position);
  fprintf(stderr, "ROT_VELOC             : %g \n", rwi_base.rot_current_speed);
  fprintf(stderr, "ROT_DIRECTION         : %i \n", rwi_base.rot_direction);
  fprintf(stderr, "BUMP_SWITCHES         : %i \n", rwi_base.bumpers);
  fprintf(stderr, "BATTERY_VOLTAGE (time): %g (%g)\n", 
	  rwi_base.battery_state.voltage, rwi_base.battery_state.time);

}

static void Rotate(double degrees)
{
/*    BASE_InstallHandler(StopHandler, ROT_STOPPED, NULL);*/
/*    TurnRobot(degrees, NULL); */
}

static void CurrentPosition(void)
{
    printf("X: %f Y: %f Orientation: %f\n", rwi_base.pos_x + 43, rwi_base.pos_y + 12, rwi_base.orientation);
}

static void StopHandler(int callback_data, Pointer client_data)
{
    printf("STOPPED!!\n");
}

static void Quit(void) 
{
    BASE_RotateHalt();
    BASE_TranslateHalt();
    exit(0);
}

static void ActivateReactive(void)
{
/*  ActivateReactiveLevel(TRUE, TRUE);
*/
}

static void DebugOn(void)
{
  BASE_Debug(TRUE, "base.log");
  SONAR_Debug(TRUE, "sonar.log");
}

static void DebugOff(void)
{
    BASE_Debug(FALSE, NULL);
    SONAR_Debug(FALSE, NULL);
}

void ProcessCommand(char *buffer)
{
    int i = 0;
    int par1, par2, par3, par4;
    
    printf ("Processing %s\n", buffer);
    while (table_commands[i].name != NULL) {
	if (strncmp(buffer, 
		    table_commands[i].name, 
		    strlen(table_commands[i].name)) == 0) {
	    buffer += strlen(table_commands[i].name);
	    switch (table_commands[i].n_parameters) {
	      case 0:
		(* (table_commands[i].func))();
		break;
	      case 1: 
		sscanf(buffer, "%d", &par1);
		(* (VOID_FN1)(table_commands[i].func))((double)par1);
		break;
	      case 2:
		sscanf(buffer, "%d %d", &par1, &par2);
		(* (VOID_FN2)(table_commands[i].func))((double)par1,
						       (double)par2);
		break;
	      case 3:
		sscanf(buffer, "%d %d %d", &par1, &par2, &par3);
		(* (VOID_FN3)(table_commands[i].func))((double)par1, 
						       (double)par2, 
						       (double)par3);
		break;
	      case 4:
		sscanf(buffer, "%d %d %d %d", &par1, &par2, &par3, &par4);
		(* (VOID_FN4)(table_commands[i].func))((double)par1, 
						       (double)par2, 
						       (double)par3,
						       (double) par4);
		break;
	    }
	    printf (">");
	    return;
	}
	i++;
    }
    
    if (strncmp(buffer, "SP", 2) == 0) {
	sscanf (buffer, "SP %d", &par1);
	BASE_SetIntervalUpdates(par1);
	return;
	
    }
    
    fprintf (stderr, 
	     "I don't know how to process %s ('?' to see commands)",
	     buffer);
    printf ("\n>");
    return;
}


void stdin_inputHnd(int fd, long chars_available)
{
  static char buffer[DEFAULT_LINE_LENGTH+1];
  static char *startPos = buffer; /* position to start parsing from */
  static char *endPos = buffer; /* position to add more characters */
  char *lineEnd;
  int numRead = 0;
  /* should handle characters output by the base */
  
  /* never expect more than DEFAULT_LINE_LENGTH characters on a line.
   * read the first DEFAULT_LINE_LENGTH and let the function get called 
   * again for any remaining characters.  This can be changed.
   */
  
  /*printf ("Entering in stdin_inputHnd\n");*/
  if (startPos == endPos)
    { 
      startPos = endPos = buffer;
      bzero(buffer, DEFAULT_LINE_LENGTH+1);
    }
  
  /* read in the command. */
  numRead = readN(&stdin_device, endPos, 
		  MIN(chars_available,(DEFAULT_LINE_LENGTH - (endPos - startPos))));
  endPos += numRead;
  if (numRead == 0)
    { /* handle error here. The port is already closed. */
    }
  else {
    /* see if we have a \n */
    lineEnd = (char *) strpbrk(startPos,"\n");
    while (lineEnd != NULL)
      {/* found a string, pass it to the parsing routines. */
	*lineEnd = '\0';
	ProcessCommand(startPos);
	startPos = lineEnd+1;
	lineEnd = (char *) strpbrk(startPos,"\n");
      }
    /* Fix up the buffer. Throw out any consumed lines.*/
    if (startPos >= endPos) 
      { /* parsed the whole thing, just clean it all up */
	bzero(buffer, DEFAULT_LINE_LENGTH+1);
	startPos = endPos = buffer;
      }
    else if (startPos != buffer)
      { /* slide it back and wait for more characters */
	bcopy(startPos, buffer, (endPos - startPos));
	endPos = buffer + (endPos - startPos);
	startPos = buffer;
      }
  }
  /*printf ("Exiting from stdin_inputHnd\n");*/
}







int main(int argc, char *argv[])
{
  int     i;

  /* listen_for_tcx_events important for devUtils */
  listen_for_tcx_events = TRUE;
  use_sonar = TRUE;
  msp_sonar = TRUE;
  use_laser = FALSE;
  use_laser_server = FALSE;
  use_collision = TRUE;
  use_rwi_server = TRUE;
  use_simulator = FALSE;
  use_vision = FALSE;
  use_arm = FALSE;
  use_bumper = TRUE;
  use_ir = TRUE;
  LASER_SERVER = NULL;


  bParamList = bParametersAddEntry(bParamList, "robot", "name", "B21");
  bParamList = bParametersAddEntry(bParamList, "", "TCXHOST", "localhost");
#ifdef UNIBONN
  bParamList = bParametersAddEntry(bParamList, "", "fork", "no");
#else
  bParamList = bParametersAddEntry(bParamList, "", "fork", "yes");
#endif
  bParamList = bParametersAddFile (bParamList, "etc/beeSoft.ini");

  bRobotType = bParametersGetParam(bParamList, "robot", "name");
  fprintf(stderr, "*** Robot type: %s ***\n", bRobotType);
  fprintf(stdout, "*** Robot type: %s ***\n", bRobotType);

  /* add some enviroment variables */
  bParamList = bParametersAddEnv(bParamList, "", "TCXHOST");


  /* add command line arguements */
  bParamList = bParametersAddArray(bParamList, "", argc, argv);

  for (i=1; i<argc; i++) {
    if ((strcmp(argv[i],"-tcx")==0))
      listen_for_tcx_events = FALSE;
    else if (strcmp(argv[i],"-fork=no")==0) {
      bParamList = bParametersAddEntry(bParamList, "", "fork", "no");
    }
    else if (strcmp(argv[i], "-fork=yes")==0) {
      bParamList = bParametersAddEntry(bParamList, "", "fork", "yes");
    }
    else if ((strcmp(argv[i],"-sonar")==0)) {
      use_sonar = FALSE; }
    else if ((strcmp(argv[i],"-ir")==0)) {
      use_ir = FALSE; }
    else if ((strcmp(argv[i],"-noindex")==0)) {
      DONE_INDEXING = TRUE; }
    /*     else if ((strcmp(argv[i],"+laser")==0)) { */
    /*       use_laser = TRUE;  */
    /*       use_laser_server = FALSE; } */
    else if ((strcmp(argv[i],"+laser")==0)) {
      use_laser = TRUE; 
      use_laser_server = TRUE; }
    else if ((strcmp(argv[i],"-colli")==0))
      use_collision = FALSE;
    else if ((strcmp(argv[i],"+simulator")==0)) {
      use_simulator = TRUE;
      use_rwi_server = FALSE;
      DONE_INDEXING = TRUE;
    }      
    else if ((strcmp(argv[i],"-rwi")==0))
      use_rwi_server = FALSE;
    else if ((strcmp(argv[i],"-tactile")==0))
      use_bumper = FALSE;
    else if ((strcmp(argv[i],"-router")==0))
      use_router = TRUE;
    else if ((strcmp(argv[i],"-keyboard")==0))
      use_keyboard = 0;
#ifdef CLEANUP
    else if ((strcmp(argv[i],"+vision")==0))
      use_vision = TRUE;
    else if ((strcmp(argv[i],"+arm")==0))
      use_arm = TRUE;
#endif
    else {
      fprintf(stderr, "\nusage: COLLI [-tcx (if tcx not required)]\n");
      fprintf(stderr, "            [-fork=yes (do not use stdin handler and fork())]\n");
      fprintf(stderr, "            [-fork=no (use stdin handler and do not fork())]\n");
      fprintf(stderr, "            [-sonar (if sonar not required)]\n");
      fprintf(stderr, "            [-ir (if ir not required)]\n");
      fprintf(stderr, "            [-tactile (if tactiles not required)]\n");
      fprintf(stderr, "            [+laser (use laserserver)]\n");
      fprintf(stderr, "            [-colli (if collision avoidance not required)]\n");
      fprintf(stderr, "            [+simulator (use simulator)]\n");
      fprintf(stderr, "            [-rwi (do not use rwi modules)]\n");
      fprintf(stderr, "            [-noindex (do not use findRotIndex())]\n");
      fprintf(stderr, "            [-keyboard (do not accept keyboard input)]\n");
#ifdef CLEANUP
      fprintf(stderr, "            [+vision (use vision)]\n");
      fprintf(stderr, "            [+arm (use arm)]\n");
#endif
      /*      exit(-1); */
    }
  }

  if (use_simulator) {
    use_laser_server = FALSE;
    base_device.dev.use_simulator  = TRUE;
    sonar_device.dev.use_simulator = TRUE;
    frontLaserDevice.dev.use_simulator = TRUE;
    rearLaserDevice.dev.use_simulator = TRUE;
  }
  else {
    base_device.dev.use_simulator  = FALSE;
    sonar_device.dev.use_simulator = FALSE;
    frontLaserDevice.dev.use_simulator = FALSE;
    rearLaserDevice.dev.use_simulator = FALSE;
  }

  /* Laser device does not care about rwi server. */
  if (use_rwi_server){
    base_device.dev.use_rwi_server  = TRUE;
    sonar_device.dev.use_rwi_server = TRUE;
  }
  else{
    base_device.dev.use_rwi_server  = FALSE;
    sonar_device.dev.use_rwi_server = FALSE; 
  }

  /* The lasers never use the rwi server. */
  frontLaserDevice.dev.use_rwi_server = FALSE;
  rearLaserDevice.dev.use_rwi_server = FALSE;

  /* 
   * Important: If the simulator is used, the initialization of TCX must
   * precede the initialization of the (simulated) devices, since the
   * initialization of devices initiates a bunch of TCX messages.
   */

  bParametersFillParams(bParamList);

  if (bRobot.fork) { 
    bDaemonize("colliServer.log");
  }
  
  approachDist = ROB_RADIUS;
  
  if (use_simulator){
    init_tcx();
    fprintf(stderr, "Connecting to the SIMULATOR...");
    SIMULATOR = tcxConnectModule(TCX_SIMULATOR_MODULE_NAME);
    fprintf(stderr, "Connected.\n");
    sleep(2);
  }
  else if (use_rwi_server){
    /* First start the laser because it is not handled by baseServer. */
    if ( use_laser) {
      if ( ! start_laser())
	exit(-1); 
    }
    
    init_tcx();
    fprintf(stderr, "Connecting to the RWI SERVER...");
    SERVER = tcxConnectModule(TCX_SERVER_MODULE_NAME);
    fprintf(stderr, "Connected");
    init_server_connection();
    fprintf(stderr, ".\n");
  }
  
  if (use_laser_server){
    LASER_SERVER_register_auto_update_type subscribe;
    
    fprintf(stderr, "Connecting to TCX LASER_SERVER ...");
    LASER_SERVER = tcxConnectModule(TCX_LASER_SERVER_MODULE_NAME);
    fprintf(stderr, "Connected");
    subscribe.sweep0        = 1;
    subscribe.sweep1        = 1;
    tcxSendMsg(LASER_SERVER, "LASER_SERVER_register_auto_update", &subscribe);
    fprintf(stderr, "...send auto update request");
    fprintf(stderr, ".\n");
  }
  
  
  
  
#ifdef UNIBONN
  if (use_vision) {
    fprintf(stderr, "Connecting to SUNVIS ...");
    SUNVIS = tcxConnectModule(TCX_SUNVIS_MODULE_NAME);
    fprintf(stderr, "Connected");
  }
  else
    SUNVIS = NULL;
#ifdef CLEANUP
  if (use_arm) {
    fprintf(stderr, "Connecting to ARM ...");
    ARM = tcxConnectModule(TCX_ARM_MODULE_NAME);
    fprintf(stderr, "Connected");
  }
#endif /* CLEANUP */
#endif /* UNIBONN */
  
  BASE_Debug(FALSE, NULL);
  
  if (use_sonar) {
    start_sonar(); 
  }
  
  if ( use_laser) {
    if ( ! start_laser())
      exit(-1); 
  }

  start_robot();
  
  if (use_collision) 
    if (!use_sonar && !use_laser) {
      fprintf(stderr,
	      "Sorry can't perform collision avoidance without sonar or laser!\n");
      exit(0);
    }
    else
      COLLI_start_collision_avoidance(); 

  if (!bRobot.fork && use_keyboard) {
    /* connect up the handlers for standard in. */
    connectDev(&stdin_device);
    stdin_device.outputHnd = stdin_inputHnd;
  }

  /*
   * Important: If real I/O interfaces are used, the
   * initialization of TCX must succeed the initialization
   * of all devices. Otherwise, TCX alloctes a device number which 
   * confuses devUtils.c (ProcessDevices), I think. Sebastian.
   */
  
  if (!use_simulator && !use_rwi_server)
    if (listen_for_tcx_events)
      init_tcx();
  

  if (use_sonar) {
    SONAR_InstallHandler(BroadcastSonarReport, SONAR_REPORT, NULL);
  }

  if (use_laser) {
    LASER_InstallHandler(BroadcastLaserReport, COMPLETE_LASER_REPORT, NULL);
  }

  BASE_InstallHandler(BroadcastStatusReport, STATUS_REPORT, NULL);

#ifdef UNIBONN
  SOUND = NULL;
  if (listen_for_tcx_events)
    connect_to_SOUND();
#endif
  
  signal(SIGINT,  (void *) Quit);
  /*  signal(SIGBUS,  (void *) Quit);*/
  /*  signal(SIGSEGV, (void *) Quit);*/

  devMainLoop();
  
  /* should never reach here. */
  exit(0);
}

