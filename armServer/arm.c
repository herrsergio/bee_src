
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


#ifndef lint
static char rcsid[] = "$Id: arm.c,v 1.8 1997/04/17 16:10:30 tyson Exp $";
#endif

#include <stdio.h>
#include <stdlib.h>
#ifdef __LINUX__
#include <netinet/in.h>
#endif

#include <lockNames.h>
#include <rai.h>
#include <arm.h>
#include <mcpIO.h>
#include <mcpStatus.h>
#include <bUtils.h>

#include "statusReport.h"
 
#include "armOpcodes.h"

#define MAST 01
#define GRIP 02

enum armStates { UKN, DEPLOYING, DEPLOYED, STOWING, STOWED };

enum armEvents {
  BOOM_WHERE, BOOM_STOP, BOOM_STALL, BOOM_INDEX_SET, BOOM_INDEX_CLEAR,
  MAST_WHERE, MAST_STOP, MAST_STALL, MAST_INDEX_SET, MAST_INDEX_CLEAR,
  WRIST_WHERE, WRIST_STOP, WRIST_STALL, WRIST_INDEX_SET, WRIST_INDEX_CLEAR,
  GRIP_WHERE, GRIP_STOP, GRIP_STALL, GRIP_INDEX_SET, GRIP_INDEX_CLEAR
};

int armState = UKN;
int stowState = 0;

int boomState  = 0;
int mastState  = 0;
int wristState = 0;
int gripState  = 0;

unsigned long boomEncoder;
unsigned long mastEncoder;
unsigned long wristEncoder;
unsigned long gripEncoder;

/*
 *   We record the encoder positions of mast, wrist & gripper at known 
 *   positions so we can give offsets later for absolute commands
 */

unsigned long wristEncoderZero=0; 
unsigned long gripEncoderZero=0;
unsigned long mastEncoderZero=0;
unsigned long boomEncoderZero=0;
unsigned long boomEncoderStart=0;

int boomIndex  = 0;
int mastIndex  = 0;
int wristIndex = 0;
int gripIndex  = 0;

unsigned char boomStalled  = 0;
unsigned char mastStalled  = 0;
unsigned char wristStalled = 0;
unsigned char gripStalled  = 0;


mcpIOState* MAST_MCP = NULL;
mcpIOState* GRIP_MCP = NULL;
statusReportType* mastReport; 
statusReportType* gripReport; 

RaiModule* ArmModule = NULL;
RaiModule* GripModule = NULL;

armCallback userCallbackFunc = NULL;

unsigned long status_report_format = (REPORT_STATUS_DATA |
				      REPORT_HEADING |
				      REPORT_TRANSLATE_STATUS |
				      REPORT_ROTATE_STATUS);

#define DEBUG

void armModShutDown(RaiModule*);
void mastSelect(RaiModule*);
void gripSelect(RaiModule*);
void handleMastError(unsigned char opCode, char* data);
void handleGripError(unsigned char opCode, char* data);
void handleMastStatus(statusReportType* report);
void handleGripStatus(statusReportType* report);


#define ENSURE_DEPLOYED() { \
  if (armState !=DEPLOYED) { \
    fprintf(stderr,"%4d:%s() - ArmLibrary: arm is not yet deployed!\n", \
	    __LINE__, __FUNCTION__); \
    return; \
  } \
}

void armMastCmd(unsigned char opCode, unsigned long parameter) {
   mcpSendCmd(MAST_MCP, opCode, parameter);
}


void armGripCmd(unsigned char opCode, unsigned long parameter) {
   mcpSendCmd(GRIP_MCP, opCode, parameter);
}

/* these are not exported and are only used for initializing the arm */
/* they do not check if the arm it deployed, etc since they deploy it */ 

unsigned long mastPosMM(void) {
  float val;

  val = (mastEncoder-mastEncoderZero)/MAST_ENCODERS_PER_MM;
  return((unsigned long)val);
}

unsigned long mastPosEnc(unsigned long mm) {
  float val;

  val = mm*MAST_ENCODERS_PER_MM + mastEncoderZero;
  return((unsigned long)val);
}

unsigned long gripPosMM(void) {
  float val;

  val = (gripEncoder-gripEncoderZero)/GRIP_ENCODERS_PER_MM;
  return((unsigned long)val);
}

unsigned long gripPosEnc(unsigned long mm) {
  float val;

  val = mm*GRIP_ENCODERS_PER_MM + gripEncoderZero;
  return((unsigned long)val);
}

/*
 * boom
 * This is not user controllable except by stow/deploy
 */

static void _boomLimp(void) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_BOOM_LIMP,0);
}

static void _boomExtend(void) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_BOOM_EXTEND,0);
}

static void _boomRetract(void) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_BOOM_RETRACT,0);
}

static void _boomWhere(void) {
  /*  fprintf(stderr, "%s();\n", __FUNCTION__); */
  armMastCmd(OP_BOOM_GET_WHERE,0);
}

static void _boomTorque(unsigned long arg) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_BOOM_TORQUE,arg);
}

static void _boomVelocity(unsigned long arg) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_BOOM_VEL,arg);
}

static void _boomStop(void) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_BOOM_HALT, 0);
}

static void _boomToPos(unsigned long arg) {
  fprintf(stderr, "%s(0x%08X);  boomEncoder=0x%08X\n",
	  __FUNCTION__, arg, boomEncoder);
  armMastCmd(OP_GRIP_TO_POS, arg); /* XXX borrowing OP codes here */
}

/*
 * mast
 */

static void _mastToPos(unsigned long arg) {
  fprintf(stderr, "%s(0x%08X);  mastEncoderZero=0x%08X\n",
	  __FUNCTION__, arg, mastEncoderZero);
  armMastCmd(OP_MAST_TO_POS, arg);
}

static void _mastStop(void) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_MAST_HALT, 0);
}

static void _mastLimp(void) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_MAST_LIMP, 0);
}

static void _mastUp(void) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_MAST_RAISE, 0);
}

static void _mastDown(void) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_MAST_DROP, 0);
}

static void _mastUpBy(unsigned long arg) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_MAST_REL_UP, arg);
}

static void _mastDownBy(unsigned long arg) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_MAST_REL_UP,arg);
}

static void _mastWhere(void) {
#if 0
  fprintf(stderr, "%s();\n", __FUNCTION__);
#endif
  armMastCmd(OP_MAST_GET_WHERE,0);
}

static void _mastTorque(unsigned long arg) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_MAST_TORQUE,arg);
}

static void _mastVelocity(unsigned long arg) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_MAST_VEL,arg);
}

static void _mastIndex(void) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_MAST_FIND_INDEX,0);
}

/*
 * gripper
 */

static void _gripStop(void) {
  fprintf(stderr, "%s(void);\n", __FUNCTION__);
  armGripCmd(OP_GRIP_HALT, 0);
}

static void _gripLimp(void) {
  fprintf(stderr, "%s(void);\n", __FUNCTION__);
  armGripCmd(OP_GRIP_LIMP, 0);
}

static void _gripCloseBy(unsigned long arg) {
  fprintf(stderr, "%s(void);\n", __FUNCTION__);
  armGripCmd(OP_GRIP_REL_CLOSE, arg);
}

static void _gripClose(void) {
  fprintf(stderr, "%s(void);\n", __FUNCTION__);
  armGripCmd(OP_GRIP_CLOSE,0);
}

static void _gripOpenBy(unsigned long arg) {
  fprintf(stderr, "%s(%u);\n", __FUNCTION__, arg);
  armGripCmd(OP_GRIP_REL_OPEN, arg);
}

static void _gripOpen(void) {
  fprintf(stderr, "%s(void);\n", __FUNCTION__);
  armGripCmd(OP_GRIP_OPEN,0);
}

static void _gripToPos(unsigned long arg) {
  fprintf(stderr, "%s(0x%08X);  gripEncoderZero=0x%08X\n",
	  __FUNCTION__, arg, gripEncoderZero);
  armGripCmd(OP_GRIP_TO_POS, arg);
}

static void _gripWhere(void) {
#if 0
  fprintf(stderr, "%s();\n", __FUNCTION__);
#endif
  armGripCmd(OP_GRIP_GET_WHERE,0);
}

static void _gripTorque(unsigned long arg) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armGripCmd(OP_GRIP_TORQUE,arg);
}

static void _gripVelocity(unsigned long arg) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armGripCmd(OP_GRIP_VEL,arg);
}

/*
 * wrist
 */

static void _wristStop(void) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armGripCmd(OP_WRIST_HALT, 0);
}

static void _wristLimp(void) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armGripCmd(OP_WRIST_LIMP, 0);
}

static void _wristIndex(void) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armGripCmd(OP_WRIST_FIND_INDEX, 0);
}

static void _wristTorque(unsigned long arg) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armGripCmd(OP_WRIST_TORQUE, arg);
}

static void _wristVelocity(unsigned long arg) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armGripCmd(OP_WRIST_VEL, arg);
}

static void _wristWhere(void) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armGripCmd(OP_WRIST_GET_WHERE, 0);
}

static void _wristCW(unsigned long arg) {
  fprintf(stderr, "%s(%03X);\n", __FUNCTION__, arg);
  armGripCmd(OP_WRIST_REL_POS, arg);
}

static void _wristCCW(unsigned long arg) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armGripCmd(OP_WRIST_REL_NEG, arg);
}

static void _wristLoadHeading(unsigned long arg) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armGripCmd(OP_WRIST_LOAD_HEADING, arg);
}

static void _wristToPos(unsigned long arg) {
  long toNewPos;
  
  fprintf(stderr, "%s:%6d:%s() - Wrist from %X to %X : ",
	  __FILE__, __LINE__, __FUNCTION__, gripReport->Heading, arg);

  toNewPos = (arg - gripReport->Heading) & 0x3FF;
  
  if (toNewPos>512) {
    toNewPos -= 1024;
  }
  
  if (toNewPos<0) {
    fprintf(stderr, "rotating -%X\n", -toNewPos);
    _wristCCW((unsigned long)(-toNewPos));
  }
  else if (toNewPos>0) {
    fprintf(stderr, "rotating %X\n", toNewPos-1);
    _wristCW((unsigned long)toNewPos-1);
  }
  else {
    fprintf(stderr, "rotating 0\n");
  }
}

/*
 * status reports
 */

static void _gripStatusData(unsigned long arg) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armGripCmd(OP_SET_STATUS_DATA,arg);
}

static void _gripStatusPeriod(unsigned long arg) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armGripCmd(OP_SET_STATUS_PERIOD,arg);
}

static void _mastStatusData(unsigned long arg) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_SET_STATUS_DATA,arg);
}

static void _mastStatusPeriod(unsigned long arg) {
  fprintf(stderr, "%s();\n", __FUNCTION__);
  armMastCmd(OP_SET_STATUS_PERIOD,arg);
}

/************************************************************/
/************************************************************/

/*
 * Arm state machine
 */

static int stateMachine (int event) {
  int nextArmState   = armState;
  int nextBoomState  = boomState;
  int nextMastState  = mastState;
  int nextWristState = wristState;
  int nextGripState  = gripState;
  int error = 0;

#ifdef DEBUG
  if ((armState==DEPLOYING) || 
      ((armState>DEPLOYING) && (event!=MAST_WHERE) && 
       (event!=GRIP_WHERE) && (event!=BOOM_WHERE))) {
    fprintf(stderr,
	    "A armState = %2d : axis states  = %2d %2d %2d %2d : event = ",
	    armState, boomState, mastState,
	    wristState, gripState);
    switch (event) {
    case BOOM_WHERE:        fprintf(stderr, "BOOM_WHERE        "); break;
    case BOOM_STOP:         fprintf(stderr, "BOOM_STOP         "); break;
    case BOOM_STALL:        fprintf(stderr, "BOOM_STALL        "); break;
    case BOOM_INDEX_SET:    fprintf(stderr, "BOOM_INDEX_SET    "); break;
    case BOOM_INDEX_CLEAR:  fprintf(stderr, "BOOM_INDEX_CLEAR  "); break;
    case MAST_WHERE:        fprintf(stderr, "MAST_WHERE        "); break;
    case MAST_STOP:         fprintf(stderr, "MAST_STOP         "); break;
    case MAST_STALL:        fprintf(stderr, "MAST_STALL        "); break;
    case MAST_INDEX_SET:    fprintf(stderr, "MAST_INDEX_SET    "); break;
    case MAST_INDEX_CLEAR:  fprintf(stderr, "MAST_INDEX_CLEAR  "); break;
    case WRIST_WHERE:       fprintf(stderr, "WRIST_WHERE       "); break;
    case WRIST_STOP:        fprintf(stderr, "WRIST_STOP        "); break;
    case WRIST_STALL:       fprintf(stderr, "WRIST_STALL       "); break;
    case WRIST_INDEX_SET:   fprintf(stderr, "WRIST_INDEX_SET   "); break;
    case WRIST_INDEX_CLEAR: fprintf(stderr, "WRIST_INDEX_CLEAR "); break;
    case GRIP_WHERE:        fprintf(stderr, "GRIP_WHERE        "); break;
    case GRIP_STOP:         fprintf(stderr, "GRIP_STOP         "); break;
    case GRIP_STALL:        fprintf(stderr, "GRIP_STALL        "); break;
    case GRIP_INDEX_SET:    fprintf(stderr, "GRIP_INDEX_SET    "); break;
    case GRIP_INDEX_CLEAR:  fprintf(stderr, "GRIP_INDEX_CLEAR  "); break;
    default:                fprintf(stderr, "* UNKNOWN EVENT * ");
    }
    fprintf(stderr, "\n");
  }
#endif

  if (armState == DEPLOYING) {

    if ((boomState  == 0) &&
	(mastState  == 0) &&
	(wristState == 0) &&
	(gripState  == 0) &&
	(!error)) {
      if (event == MAST_STOP) {
	_boomWhere();
	nextBoomState = 1;
      }
    }

    if ((boomState  == 1) &&
	(mastState  == 0) &&
	(wristState == 0) &&
	(gripState  == 0) &&
	(!error)) {
      if (event == BOOM_WHERE) {
	boomEncoderStart = boomEncoder;
	_boomTorque(0x10);
	_boomExtend();
	nextBoomState = 2;
      }
      else {
	error = 1;
      }
    }

    if ((boomState  == 2) &&
	(mastState  == 0) &&
	(wristState == 0) &&
	(gripState  == 0) &&
	(!error)) {
      if (event == BOOM_STALL) {
	_boomLimp();
	_boomStop();
	if (boomIndex) {
	  _boomWhere();
	  nextBoomState = 3;
	}
	else {
	  error = 1;
	}
      }
      else {
	error = 1;
      }
    }

    if ((boomState  == 3) &&
	(mastState  == 0) &&
	(wristState == 0) &&
	(gripState  == 0) &&
	(!error)) {
      if (event == BOOM_WHERE) {
	nextBoomState = 15;
	nextMastState = 1;
	boomEncoderZero = boomEncoder;

	_mastTorque(0x40);

	if ((boomEncoder-boomEncoderStart>0x52DC) || mastIndex) {
	  _mastUpBy(0x30);
	}
	else {
	  _mastUpBy(0x230);
	}
      }
      else {
	error = 1;
      }
    }

    if ((boomState  ==15) &&
	(mastState  == 1) &&
	(wristState == 0) &&
	(gripState  == 0) &&
	(!error)) {
      if (event == MAST_INDEX_CLEAR) {
	_mastUpBy(0x30);
      }
      else if ((event == MAST_STALL) ||
	       (event == MAST_STOP)) {
	_mastIndex();
	_mastWhere();
	nextMastState = 2;
	_wristIndex();
	_wristWhere();
	nextWristState = 1;
      }	
      else {
	error = 1;
      }
    }

    if ((boomState  ==15) &&
	(mastState  == 2) &&
	(!error)) {
      if (event == MAST_STOP) { /* XXX */
	mastEncoderZero = mastEncoder-MAST_ENCODERS_AT_INDEX;
	nextMastState = 15;
      }	
    }

    if ((boomState  ==15) &&
	(mastState   > 1) &&
	(wristState == 1) &&
	(gripState  == 0) &&
	(!error)) {
      if (event == WRIST_STOP) { /* XXX */
	nextWristState = 15;
	_wristLoadHeading(0x0);
	_gripTorque(0x18);
	_gripOpenBy(0x1800);
	nextGripState = 1;
      }	
    }

    if ((boomState  ==15) &&
	(mastState   > 1) &&
	(wristState ==15) &&
	(gripState  == 1) &&
	(!error)) {
      if ((event == GRIP_STOP) ||
	  (event == GRIP_STALL)) {
	_gripStop();
	_gripTorque(0x30);
	_gripCloseBy(0x1500);
	nextGripState = 2;
      }	
    }

    if ((boomState  ==15) &&
	(mastState   > 1) &&
	(wristState ==15) &&
	(gripState  == 2) &&
	(!error)) {
      if (event == GRIP_STOP) {
	_gripTorque(0x10);
	_gripClose();
	nextGripState = 3;
      }	
    }

    if ((boomState  ==15) &&
	(mastState   > 1) &&
	(wristState ==15) &&
	(gripState  == 3) &&
	(!error)) {
      if ((event == GRIP_STALL) || (event == GRIP_INDEX_SET)) {
	_gripStop();
	_gripWhere();
	nextGripState = 4;
      }	
    }

    if ((boomState  ==15) &&
	(mastState   > 1) &&
	(wristState ==15) &&
	(gripState  == 4) &&
	(!error)) {
      if (event == GRIP_WHERE) {
	gripEncoderZero = gripEncoder;
	_gripTorque(0x18);
	_gripOpenBy(0x9500);
	nextGripState = 5;
      }	
    }

    if ((boomState  ==15) &&
	(mastState   > 1) &&
	(wristState ==15) &&
	(gripState  == 5) &&
	(!error)) {
      if (event == GRIP_STOP) {
	_gripWhere();
	nextGripState = 6;
      }	
    }

    if ((boomState  ==15) &&
	(mastState   > 1) &&
	(wristState ==15) &&
	(gripState  == 5) &&
	(!error)) {
      if (event == GRIP_STALL) {
	error = 1;
      }	
    }

    if ((boomState  ==15) &&
	(mastState   > 1) &&
	(wristState ==15) &&
	(gripState  == 6) &&
	(!error)) {
      if (event == GRIP_WHERE) {
	int gripTravel;

	gripTravel = gripEncoder - gripEncoderZero;

	if ((gripTravel < 0x9700) && (gripTravel > 0x9300)) {
	  nextGripState = 15;
	}
	else {
	  error = 1;
	}
      }	
    }

    if ((nextBoomState==15) && (nextMastState==15) && 
	(nextWristState==15) && (nextGripState==15)) {
      mastEncoderZero = mastEncoder-MAST_ENCODERS_AT_INDEX;
      nextArmState = DEPLOYED;
      if (userCallbackFunc) {
	userCallbackFunc(ARM_deployArm, 1);
      }
    }

  }

  if (armState==STOWING) {
    switch (stowState) {
    case 1: if (event == MAST_STOP) {
      _wristIndex();
      _wristWhere();
      stowState=2;
    }
    break;

    case 2: if (event == WRIST_STOP) { /* XXX */
      _gripTorque(0x20);
      _gripToPos(gripEncoderZero+GRIP_TRAVEL);
      stowState=3;
    }
    break;

    case 3: if ((event==GRIP_STOP) || (event==GRIP_STALL)) {
      _gripLimp();
      _wristLimp();
      _boomTorque(0x20);
      _boomToPos(boomEncoderZero-bRobot.arm_mast_boomTravel);
      stowState=4;
    }

    case 4: if ((event==BOOM_STOP) || (event==BOOM_STALL) ) {
      _wristLimp();
      _gripLimp();
      _boomLimp();
      _mastLimp();
      nextArmState=STOWED;
      if (userCallbackFunc) {
	userCallbackFunc(ARM_stowArm, 1);
      }
      fprintf(stderr, "*** arm stowed\n");
    }
    break;
    
    default:
    }
  }

#ifdef DEBUG
  if (armState==DEPLOYING) {
    fprintf(stderr,
	    "B NextArmState = %2d : axis states  = %2d %2d %2d %2d",
	     nextArmState, nextBoomState, nextMastState,
	    nextWristState, nextGripState);
    
    if (error) {
      fprintf(stderr, " *\n");
    }
    else {
      fprintf(stderr, "\n");
    }
  }

#endif

  armState   = nextArmState;
  boomState  = nextBoomState;
  mastState  = nextMastState;
  wristState = nextWristState;
  gripState  = nextGripState;

  return(0);
}

/***************************************************************************
 *
 *  Arm Module 
 *
 *  Handles output from base
 *
 ***************************************************************************/


void
getEncoders(RaiModule* mod)
{
  if (1) {
    _boomWhere();
    _mastWhere();
    _gripWhere();
    armMastCmd(OP_INDEX_REPORT,0xFF);
    armGripCmd(OP_INDEX_REPORT,0xFF);
  }
}

void ArmShutdown() {
  armLimp();
}

void
armStartup(RaiModule* mod)
{
  int statusPeriod = 0x10;
  armMastCmd(OP_SET_STATUS_DATA,status_report_format);
  armMastCmd(OP_SET_STATUS_PERIOD,statusPeriod);
  armMastCmd(OP_INDEX_REPORT,0xFF);
  armGripCmd(OP_SET_STATUS_DATA,status_report_format);
  armGripCmd(OP_SET_STATUS_PERIOD,statusPeriod);
  armGripCmd(OP_INDEX_REPORT,0xFF);

  _boomTorque(0x08);

  addPolling(ArmModule, getEncoders, 62);
}

void
ArmInit(const char *mastDev, speed_t mastBaud,
	const char *gripDev, speed_t gripBaud)
{
  static int STARTED=FALSE;
  int mastFd;
  int gripperFd;

  if (STARTED == FALSE)
    {
      STARTED=TRUE;
      MAST_MCP = openMcp(mastDev, MAST_LOCK, mastBaud);
      if (MAST_MCP == NULL)
         {
            fprintf(stderr, "ArmInit: openMcp(%s) failed.\nExiting.\n",
                    mastDev);
            exit(-1);
         }
      mastFd = MAST_MCP->fd;

      GRIP_MCP = openMcp(gripDev, GRIP_LOCK, gripBaud);
      if (GRIP_MCP == NULL)
         {
            fprintf(stderr, "ArmInit: openMcp(%s) failed.\nExiting.\n",
                    gripDev);
            exit(-1);
         }
      mastReport = initStatusStructure(); 
      gripReport = initStatusStructure(); 
      gripperFd = GRIP_MCP->fd;

      RaiInit();
      ArmModule = makeModule("Rai Arm module", armModShutDown);
      addSelect(ArmModule, mastSelect, mastFd);
      addTimeout(ArmModule,armStartup, 10);
      GripModule = makeModule("Rai Grip module", armModShutDown);
      addSelect(GripModule, gripSelect, gripperFd);

#ifdef DEBUG
      printf("Arm Module Initialized.\n"); 
      raiTrace(ArmModule);
#endif
    }
}


void
dispatchValue(int Axis, unsigned char opcode, long value)
{
  long userValue = -1;
  armMessage event = -1;

  if (Axis == MAST) {
    switch (opcode) {
    case OP_BOOM_GET_WHERE:
      boomEncoder = value;
      stateMachine(BOOM_WHERE);
      break;

    case  OP_MAST_GET_WHERE:
      mastEncoder = value;
      stateMachine(MAST_WHERE);
      break;

    case OP_BOOM_HALT:
      stateMachine(BOOM_STOP);
      break;

    case OP_MAST_HALT:
      stateMachine(MAST_STOP);
      event = ARM_mastStopped;
      userValue = 0;
      break;

    case OP_INDEX_REPORT:
      if (((value & 0x02) != 0) ^ (mastIndex != 0)) {
	if (value & 0x02) {
	  mastIndex = 1;
	  stateMachine(MAST_INDEX_SET);
	}
	else {
	  mastIndex = 0;
	  stateMachine(MAST_INDEX_CLEAR);
	}
      }

      if (((value & 01) != 0) ^ (boomIndex != 0)) {
	if (value & 0x01) {
	  boomIndex = 1;
	  stateMachine(BOOM_INDEX_SET);
	}
	else {
	  boomIndex = 0;
	  stateMachine(BOOM_INDEX_CLEAR);
	}
      }

      break;

    default:
    }
  }

  if (Axis == GRIP) {

    switch (opcode) {
    case OP_INDEX_REPORT:

      if (((value & 0x02) != 0) ^ (wristIndex != 0)) {
	if (value & 0x02) {
	  wristIndex = 1;
	  stateMachine(WRIST_INDEX_SET);
	}
	else {
	  wristIndex = 0;
	  stateMachine(WRIST_INDEX_CLEAR);
	}
      }

      if (((!(value & 01)) != 0) ^ (gripIndex != 0)) {
	if (!(value & 0x01)) {
	  gripIndex = 1;
	  stateMachine(GRIP_INDEX_SET);
	  event = ARM_gripEngaged;
	  userValue = TRUE;
	}
	else {
	  gripIndex = 0;
	  stateMachine(GRIP_INDEX_CLEAR);
	  event = ARM_gripEngaged;
	  userValue = FALSE;
	}
      }

      break;

    case OP_WRIST_GET_WHERE:
      wristEncoder = value;
      stateMachine(WRIST_WHERE);
      break;


    case OP_GRIP_GET_WHERE:
      gripEncoder = value;
      stateMachine(GRIP_WHERE);
      break;

    case OP_WRIST_HALT:
      stateMachine(WRIST_STOP);
      event = ARM_wristStopped;
      userValue = 0;
      break;

    case OP_GRIP_HALT:
      stateMachine(GRIP_STOP);
      event = ARM_gripStopped;
      userValue = 0;
      break;

    default:
    }
  }

  if (userCallbackFunc && (armState==DEPLOYED) && (event>0)) {
    userCallbackFunc(event, userValue);
  }

  return;
}

void
handleMastError (unsigned char opCode, char *data)
{
  unsigned char errorByte = data[4];

  if (errorByte & 0x01) {
    boomStalled = 1;
    stateMachine(BOOM_STALL);
  }

  if (errorByte & 0x04) {
    mastStalled = 1;
    stateMachine(MAST_STALL);
    if ((userCallbackFunc != NULL) && (armState == DEPLOYED)) {
      userCallbackFunc(ARM_mastError,0);
    }
  }
}

void
handleGripError (unsigned char opCode, char *data)
{
  unsigned char errorByte = data[4];

  if (errorByte & 0x01) {
    gripStalled = 1;
    stateMachine(GRIP_STALL);
    if ((userCallbackFunc != NULL) && (armState == DEPLOYED)) {
      userCallbackFunc(ARM_gripError,0);
    }
  }

  if (errorByte & 0x04) {
    wristStalled = 1;
    stateMachine(WRIST_STALL);
    if ((userCallbackFunc != NULL) && (armState==DEPLOYED)) {
      userCallbackFunc(ARM_wristError,0);
    }
  }
}

void
handleArmOutput(int axis, unsigned char* base_output)
{
  unsigned char* cur_byte = base_output;
  unsigned char* return_string = 0;
  unsigned char* cur_string;
  unsigned long* hack_long_ptr;
  unsigned char op_code;
  unsigned char  output_length = 0;
  unsigned long value = 0;
  int i,j;

  /* I have not had time to figure out what the F this code is doing :) */

  if ((*cur_byte == 2) && (*(cur_byte+1) == 2))
    {
      cur_byte += 2;
      output_length = *cur_byte;

      if ((*cur_byte == 3) || (*cur_byte == 2))
	cur_byte++;      

      cur_byte++;
      op_code = *cur_byte;

      /* why are  these checks here? - jmk */
      if ((*cur_byte == 3) || (*cur_byte == 2))
	cur_byte++;      

      output_length--;
      cur_byte++;
      
      return_string = (unsigned char *)
	calloc(output_length+20, sizeof(unsigned char));

      cur_string = return_string;
      if (output_length < 4)
	*(cur_string++) = 4;
      else
	*(cur_string++) = output_length;

      /* Note:
        We do not always want to put the length as the first byte,
	for example when we call setLValue below.  This should
        be rewritten to fix this, but for now I just fixed the code
        to pass the bytes after the length to setLValue 
      */

      hack_long_ptr = (unsigned long*) cur_string;

      for (j = 0; j < (4 - output_length); j++)
	{
	  *(cur_string) = 0;
	  cur_string++;
	}

      for (i = 0; i < output_length; i++) {
	*(cur_string + i) = *cur_byte;
	*(cur_string + i + 1) = '\0';
	
	if ((*cur_byte == 3) || (*cur_byte == 2))
	  cur_byte++;
	cur_byte++;
      }
      
#if 0
      if ((op_code != 17) && (op_code != 92) && (op_code !=60)) {
	fprintf(stderr,"Entering switch with axis %d, opcode %d\n",
		axis,op_code);
      }
#endif

      switch(op_code){
      case 0x00 :
	if (axis==MAST)
	  resetMcpComm(MAST_MCP);
	else
	  resetMcpComm(GRIP_MCP);
	break;

      case 0x01 :
	if (axis==MAST) {
	  handleMastError(op_code,return_string);
	}
	else {
	  handleGripError(op_code,return_string);
	}
	break;

      case 0x02 :
	/* error cleared */
	break;

      case 0x04 :
	printf("Should not receive WE in direct mode\n");
	break;

      case 0x10 :
      case 0x11 :
	/* parse report & call callbacks based on which axis */
	if (axis==MAST) {
	  parseStatusReport(mastReport,return_string); 
	  handleMastStatus(mastReport);
	}
	else {
	  parseStatusReport(gripReport,return_string); 
	  handleGripStatus(gripReport);
	}

	break;

      case 0x08:
	break;

      default:
	value = htonl(*hack_long_ptr);
	dispatchValue(axis,op_code,value);
      }
      free (return_string);
    }
  else {
    fprintf(stderr,"Packet with missing start sent to handleArmOutput\n");
  }
}


void
armModShutDown(RaiModule* base_module)
{
  ArmShutdown();
}


void
mastSelect(RaiModule* base_module)
{
  char* message;
  message = mcpSelect(MAST_MCP);
  if (message != NULL) {
    handleArmOutput(MAST,message);
    free(message);
  }
}

void
gripSelect(RaiModule* base_module)
{
  char* message;
  message = mcpSelect(GRIP_MCP);
  if (message != NULL) {
    handleArmOutput(GRIP,message);
    free(message);
  }
}

void
handleGripStatus(statusReportType* report)
{
  unsigned char rotatePhase;
  static unsigned char lastRotatePhase = PHASE_STOP;  
  unsigned char translatePhase;
  static unsigned char lastTranslatePhase = PHASE_STOP;  
  
  rotatePhase = statusPhase(report->RotateStatus);
  translatePhase = statusPhase(report->TranslateStatus);

  if (phaseMotionless(rotatePhase) && 
      !phaseMotionless(lastRotatePhase))
    dispatchValue(GRIP,OP_WRIST_HALT,0);
  
  if (phaseMotionless(translatePhase) && 
      !phaseMotionless(lastTranslatePhase))
    dispatchValue(GRIP,OP_GRIP_HALT,0);
  
  lastRotatePhase = rotatePhase;
  lastTranslatePhase = translatePhase;

  if (report->RotateStatus & 0x40) {
    if (!wristStalled) {
      wristStalled = 1;
      stateMachine(WRIST_STALL);
      if ((userCallbackFunc != NULL) && (armState==DEPLOYED)) {
	userCallbackFunc(ARM_wristError,0);
      }
    }
  }
  else {
    wristStalled = 0;
  }

  if (report->TranslateStatus & 0x40) {
    if (!gripStalled) {
      gripStalled = 1;
      stateMachine(GRIP_STALL);
      if ((userCallbackFunc != NULL) && (armState==DEPLOYED)) {
	userCallbackFunc(ARM_gripError,0);
      }
    }
  }
  else {
    gripStalled = 0;
  }

}

void
handleMastStatus(statusReportType* report)
{
  unsigned char rotatePhase;
  static unsigned char lastRotatePhase = PHASE_STOP;    
  unsigned char translatePhase;
  static unsigned char lastTranslatePhase = PHASE_STOP;    

  rotatePhase = statusPhase(report->RotateStatus);
  translatePhase = statusPhase(report->TranslateStatus);

#if 0
  fprintf(stderr, "boomPhase = %02X : mastPhase = %02X\n",
	  translatePhase, rotatePhase);
#endif

  if (phaseMotionless(rotatePhase) && 
      !phaseMotionless(lastRotatePhase))
    dispatchValue(MAST,OP_MAST_HALT,0);

  if (phaseMotionless(translatePhase) && 
      !phaseMotionless(lastTranslatePhase))
    dispatchValue(MAST,OP_BOOM_HALT,0);

  lastRotatePhase = rotatePhase;
  lastTranslatePhase = translatePhase;

  if (report->RotateStatus & 0x40) {
    if (!mastStalled) {
      mastStalled = 1;
      stateMachine(MAST_STALL);
      if ((userCallbackFunc != NULL) && (armState==DEPLOYED)) {
	userCallbackFunc(ARM_mastError,0);
      }
    }
  }
  else {
    mastStalled = 0;
  }

  if (report->TranslateStatus & 0x40) {
    if (!boomStalled) {
      boomStalled = 1;
      stateMachine(BOOM_STALL);
    }
  }
  else {
    boomStalled = 0;
  }

}


/************************************************************************/
/*
*/
/************************************************************************/

void registerArmEventHandler(armCallback  func) {
    userCallbackFunc = func;
}

void armLimp() {
  _boomLimp();
  _mastLimp();
  _gripLimp();
  _wristLimp(); 
  if (armState != DEPLOYED) {
    armState = UKN;
  }
}

void deployArm(void) {
  /* if we are sure the arm is already deployed, do not redeploy it */

  if (armState == DEPLOYED) {
    if (userCallbackFunc != NULL) {
      userCallbackFunc(ARM_deployArm,0);
    }
    return;
  }

  armState = DEPLOYING;
  boomState = 0;
  mastState = 0;
  wristState = 0;
  gripState = 0;

  _mastUpBy(0x03);
}

void stowArm(void) {

  /* if we are sure the arm is already stowed, do not stow it */
  if (armState == STOWED) {
    fprintf(stderr, "Arm already stowed, not doing it again\n");
    if (userCallbackFunc != NULL) {
      userCallbackFunc(ARM_stowArm,0);
    }
    return; 
  }

  fprintf(stderr, "Checking that arm is deployed.\n");
  ENSURE_DEPLOYED();
  fprintf(stderr, "Arm is deployed, stowing it.\n");

  _mastToPos(mastEncoderZero+bRobot.arm_mast_stow+50);
  armState = STOWING;
  stowState = 1;

  /* XXX
   * Here we should set up all motion parameters to 
   * good values incase the client has messed with them.
   */

}



/*************************************************************

  MAST COMMANDS

*************************************************************/

void mastLimp(void) {
  ENSURE_DEPLOYED();
  _mastLimp();
}

void mastHalt(void) {
  ENSURE_DEPLOYED();
  _mastStop();
}

void mastRelativeUp(unsigned long arg) {
  unsigned long newPos;

  ENSURE_DEPLOYED();

  newPos = mastPosMM()+arg;
  if (newPos>MAST_TOP_MM) {
    newPos = MAST_TOP_MM;
  }

  _mastToPos(mastPosEnc(newPos));
}

void mastRelativeDown(unsigned long arg) {
  unsigned long newPos;

  ENSURE_DEPLOYED();

  newPos = mastPosMM()-arg;
  if ((long)newPos<0) {
    newPos = 0;
  }

  _mastToPos(mastPosEnc(newPos));
}

void mastToPos(unsigned long arg) {
  unsigned long newPos;

  ENSURE_DEPLOYED();

  newPos = arg;

  if ((long)newPos<0) {
    newPos = 0;
  }

  if (newPos>MAST_TOP_MM) {
    newPos = MAST_TOP_MM;
  }

  _mastToPos(mastPosEnc(newPos));
}

void mastVelocityDown() {

  ENSURE_DEPLOYED();
  _mastToPos(mastPosEnc(0));
}

void mastVelocityUp() {
  ENSURE_DEPLOYED();
  _mastToPos(mastPosEnc(MAST_TOP_MM));
}

void mastWhere() {
  ENSURE_DEPLOYED();

  if (userCallbackFunc) {
    userCallbackFunc(ARM_mastWhere, mastPosMM());
  }
}


/*************************************************************/

void wristLimp(void) {
  ENSURE_DEPLOYED();
  _wristLimp();
}

void wristHalt(void) { 
  ENSURE_DEPLOYED();
  _wristStop();
}

void wristToPos(unsigned long arg) {
  ENSURE_DEPLOYED();
  _wristToPos(arg);
}

void wristRelativePos(unsigned long arg) {
  ENSURE_DEPLOYED();
  _wristCW(arg);
}

void wristRelativeNeg(unsigned long arg) {
  ENSURE_DEPLOYED();
  _wristCCW(arg);
}

void wristWhere() {
  ENSURE_DEPLOYED();

  if (userCallbackFunc) {
    userCallbackFunc(ARM_wristWhere, gripReport->Heading);
  }
}

/*************************************************************

  GRIP COMMANDS

*************************************************************/

void gripLimp(void) {
  ENSURE_DEPLOYED();
  _gripLimp();
}

void gripHalt(void) {

  ENSURE_DEPLOYED();
  _gripStop();
}

void gripRelativeClose(unsigned long arg) {
  unsigned long newPos;

  ENSURE_DEPLOYED();

  newPos = gripPosMM() - arg;

  if ((long)newPos<0) {
    newPos = 0;
  }

  if (newPos>GRIP_OPEN_MM) {
    newPos = GRIP_OPEN_MM;
  }

  _gripToPos(gripPosEnc(newPos));
}

void gripRelativeOpen(unsigned long arg) {
  unsigned long newPos;

  ENSURE_DEPLOYED();

  newPos = gripPosMM() + arg;

  if ((long)newPos<0) {
    newPos = 0;
  }

  if (newPos>GRIP_OPEN_MM) {
    newPos = GRIP_OPEN_MM;
  }

  _gripToPos(gripPosEnc(newPos));
}

void gripToPos(unsigned long arg) {

  ENSURE_DEPLOYED();

  if (arg>GRIP_OPEN_MM) {
    arg = GRIP_OPEN_MM;
  }

  _gripToPos(gripPosEnc(arg));
}

void gripWhere() {
  ENSURE_DEPLOYED();

  if (userCallbackFunc) {
    userCallbackFunc(ARM_gripWhere, gripPosMM());
  }
}
