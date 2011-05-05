
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
#define define_base_message_names
#include <tcx.h>
#include <bUtils.h>
#include "devUtils.h"
#include "sonar_interface.h"
#include "rwibase_interface.h"
#include "server.h"
#include "collision.h"
#include "collisionIntern.h"

/*#define VERBOSE*/
/*#define SERVERdebug */

/* At what position in the index */
float indexHeading = 0.0;          
int FOUND_INDEX = FALSE;
int INDEXING = FALSE;
int DONE_INDEXING= FALSE;     /* did we find index and turn back to start? */

#define raiVariableMsg "{int,int,<char: 2>}"

typedef struct 
   {int operation; int bufsize; unsigned char * buffer;} RAI_VariableMsgType;
  

#define raiFixedMsg "{int, long}"
typedef struct {int operation; unsigned long parameter;} RAI_FixedMsgType;


typedef void (*clientCallback)     (int,unsigned long);
/* irgendwo schon definiert ...
typedef  int (*sonarCallbackType)  (sonarType *sonar);
typedef  int (*irCallbackType)     (irType **irMatrix);
typedef  int (*tactileCallbackType) (tactileType **tactile);
*/
typedef void (*statusCallback)      (statusReportType*);  

clientCallback baseClientFcn=NULL;

sonarCallbackType userSonarFcn=NULL;

irCallbackType userIrFcn=NULL; 

tactileCallbackType userTactileFcn=NULL; 

statusCallback statusClientFcn=NULL;


#define SERVER_reply_handlers  {   \
  {"baseFixed","BaseFxd",handleBaseClientFixed, TCX_RECV_ALL,NULL}, \
  {"baseVar","BaseVar",handleBaseClientVariable, TCX_RECV_ALL,NULL}}


/***************************************************************/
/*
 
  Globals which are exported by the generic sensor API. We
  must replicate them here, and then get the actual info
  from the base server and fill them in
  
*/
/***************************************************************/

statusReportType  activeStatusReport;

sonarType sonars[B_MAX_SENSOR_COLS];

irType irs[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];

tactileType tactiles[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];

/* Pointer references to above for calling usr callbacks */
irType  * irPtr[B_MAX_SENSOR_ROWS];
tactileType  * tactilePtr[B_MAX_SENSOR_ROWS];

/****************** FUNCTIONS FOR SENDING TO THE SERVER *************/

void baseSendServerFixed(int operation,unsigned long arg)
{
  RAI_FixedMsgType command;
  command.operation = operation;
  command.parameter = arg;
#ifdef VERBOSE
  fprintf(stderr, "\tTCX: send to baseServer: %s %d\n",
	  baseMessageNamesXXX[(int) operation], (int) arg);
#endif /* SERVERdebug */
  tcxSendMsg( SERVER,"baseFixed",&command);
}


/*** General ****/


void registerBaseCallback(clientCallback fcn)
{
  baseClientFcn=fcn;
}

void registerStatusCallback(statusCallback fcn)
{
  statusClientFcn=fcn;
}

void registerSonarCallback(sonarCallbackType fcn)
{
 userSonarFcn=fcn;
}


void registerIrCallback(irCallbackType fcn)
{
  userIrFcn=fcn;
}


void registerTactileCallback(tactileCallbackType fcn)
{
 userTactileFcn=fcn;
}

/* Each member of the API on the client side consists of a function which */
/* just sends the corresponding message (fcn name with BASE_ prepended    */
/* and perhaps argument, to the base server.  So I wrote macros to form   */ 
/* the functions */

/* I have long suspected trained monkeys could write C :-) */  

#define serverCmd(name) void name(void) {baseSendServerFixed(BASE_ ## name,0);}
#define serverArgCmd(name) void name(unsigned long arg) \
 {baseSendServerFixed(BASE_ ## name,arg);}


/*** GENERAL ****/
serverCmd(baseKill)
serverArgCmd(loadHeading)
serverArgCmd(loadPosition)
serverArgCmd(statusReportData)
serverArgCmd(statusReportPeriod)

serverCmd(batteryVoltage);
serverCmd(batteryCurrent);

/*** ROTATION ****/
serverCmd(rotateLimp)
serverCmd(rotateHalt)
serverCmd(rotateVelocityPos)
serverCmd(rotateVelocityNeg)
serverArgCmd(rotateRelativePos)
serverArgCmd(rotateRelativeNeg)
serverArgCmd(rotateTorquePos)
serverArgCmd(rotateTorqueNeg)
serverArgCmd(rotatePowerPos)
serverArgCmd(rotatePowerNeg)
serverArgCmd(rotateToPosition) 
serverCmd(findRotIndex)

serverArgCmd(setRotateFriction)
serverArgCmd(setRotateVelocity)
serverArgCmd(setRotateAcceleration)
serverArgCmd(setRotateSlope)
serverArgCmd(setRotateTorque)
serverArgCmd(setRotateZero)

serverCmd(rotateCurrent)
serverCmd(rotateWhere)


/*** TRANSLATION ****/

serverCmd(translateLimp)
serverCmd(translateHalt)
serverCmd(translateVelocityPos)
serverCmd(translateVelocityNeg)
serverArgCmd(translateRelativePos)
serverArgCmd(translateRelativeNeg)
serverArgCmd(translateTorquePos)
serverArgCmd(translateTorqueNeg)
serverArgCmd(translatePowerPos)
serverArgCmd(translatePowerNeg)
serverArgCmd(translateToPosition) 

serverArgCmd(setTranslateVelocity)
serverArgCmd(setTranslateAcceleration)
serverArgCmd(setTranslateSlope)
serverArgCmd(setTranslateTorque)
serverArgCmd(setTranslateZero)

serverCmd(translateCurrent)
serverCmd(translateWhere)


/*------------------------------------------------------------*/

/****************** SUPPORT FUNCTIONS FOR SERVER RESPONSES *************/

/* sonarLongs alternate between the ping transducer index and ping value */
void
incorporateSonars(long sonarLongs[])
{
  int longNum=0;
  long index;

  for(index=ntohl(sonarLongs[longNum++]);
      index >= 0;
      index=ntohl(sonarLongs[longNum++]))
    {
      sonars[index].value = ntohl(sonarLongs[longNum++]);
      sonars[index].mostRecent = TRUE;
    }

  if (userSonarFcn != NULL) {
    userSonarFcn(sonars);
  }
}

/* irLongs are <row,col,value> .... */
void
incorporateIrs(unsigned long irLongs[])
{
  int longNum=0;
  int row;
  int col;

  /*
   * the server will pass a row number < 0 as a flag
   * after all real data
   */

  for(row = ntohl(irLongs[longNum++]);
      row >= 0;
      row= ntohl(irLongs[longNum++]))
    {
      col = ntohl(irLongs[longNum++]);
      irs[row][col].value = ntohl(irLongs[longNum++]);
      irs[row][col].mostRecent = TRUE;
    }
  if (userIrFcn != NULL) {
    userIrFcn(irPtr);
  }
}

/*------------------------------------------------------------*/

/* tactileLongs are <row,col,value> .... */
void
incorporateTactiles(long tLongs[])
{
  int longNum=0;
  int row;
  int col;

  /*
   * the server will pass a row number < 0 as a flag
   * after all real data
   */

#ifdef VERBOSE
  fprintf(stderr, "%s:%6d:%s() -\n", __FILE__, __LINE__, __FUNCTION__);
#endif
  
  for(row = ntohl(tLongs[longNum++]);
      row >= 0;
      row = ntohl(tLongs[longNum++]))
    {
      col = ntohl(tLongs[longNum++]);
      tactiles[row][col].value = ntohl(tLongs[longNum++]);

#ifdef VERBOSE
      fprintf(stderr, "\tvalue[%3d, %3d] = %5d\n",
	      row, col, tactiles[row][col].value);
#endif
    }

  if (userTactileFcn != NULL) {
    userTactileFcn(tactilePtr);
  }
}

/*------------------------------------------------------------*/

void
incorporateStatusReport(statusReportType* newReport)
{

  /*
   * NOTE!
   *
   * This only works because the entire struct
   * is longs and not a mix of types.  If the struct becomes
   * other than all longs this will likely fail do to different
   * architectures packing structures differently.  -tds
   */

  activeStatusReport.Request            = htonl(newReport->Request);
  activeStatusReport.Clock              = htonl(newReport->Clock);
  activeStatusReport.GeneralStatus      = htonl(newReport->GeneralStatus);
  activeStatusReport.Xpos               = htonl(newReport->Xpos);
  activeStatusReport.Ypos               = htonl(newReport->Ypos);
  activeStatusReport.Heading            = htonl(newReport->Heading);
  activeStatusReport.BaseRelativeHeading
                                   = htonl(newReport->BaseRelativeHeading);
  activeStatusReport.TranslateError     = htonl(newReport->TranslateError);
  activeStatusReport.TranslateVelocity  = htonl(newReport->TranslateVelocity);
  activeStatusReport.TranslateStatus    = htonl(newReport->TranslateStatus);
  activeStatusReport.RotateError        = htonl(newReport->RotateError);
  activeStatusReport.RotateVelocity     = htonl(newReport->RotateVelocity);
  activeStatusReport.RotateStatus       = htonl(newReport->RotateStatus);

  if (statusClientFcn != NULL) {
    statusClientFcn(&activeStatusReport);
  }
}

/*------------------------------------------------------------*/


/****************** HANDLING RESPONSES FROM THE SERVER *************/


void baseDispatchFixedMessage(int operation, unsigned long param)
{

  if (operation== BASE_rotateHalt && INDEXING) {

    if (!FOUND_INDEX) {

      indexHeading = DEG_TO_RAD( activeStatusReport.Heading *
				 360.0/(float)COUNTS_IN_360_DEGREES);
      FOUND_INDEX=TRUE;
      printf("Found index at %ld = %f deg.\n",
	     activeStatusReport.Heading,
	     RAD_TO_DEG(indexHeading));
      /* if by coincidence we started on the index  */
      /* we do not need to turn back. If we did not */
      /* start on the index, we need to start a turn*/
      /* back to the start position and wait for it */
      /* to end before starting navigation */
      if (indexHeading == 0.0) {
	INDEXING=FALSE;
	DONE_INDEXING=TRUE;
      } else
	rotateRelativeNeg(activeStatusReport.Heading);
    }
    else {
      printf("Ready to start, indexHeading = %f deg.\n",
	     RAD_TO_DEG(indexHeading));
      INDEXING = FALSE;
      DONE_INDEXING=TRUE;
    }
  }   
#define ROTATE_AXIS_STOP 16
#define TRANSLATE_AXIS_STOP 33
#define I_DONT_KNOW_WHAT 49
  if (operation==BASE_watchdogTimeout) {
    rwi_base.stopped_by_watch_dog = TRUE;
    if (dumpInfo) {
      fprintf(dumpFile, "***************************************\n");
      fprintf(dumpFile, "***************************************\n");
      fprintf(dumpFile, "WDT timed out.  Motors stopped by base.\n");
      fprintf(dumpFile, "***************************************\n");
      fprintf(dumpFile, "***************************************\n");
    }
    fprintf(stderr, "***************************************\n");
    fprintf(stderr, "***************************************\n");
    fprintf(stderr, "WDT timed out.  Motors stopped by base.\n");
    fprintf(stderr, "***************************************\n");
    fprintf(stderr, "***************************************\n");
  }
  else if(baseClientFcn!= NULL)
    baseClientFcn(operation,param);
  else if ( operation != ROTATE_AXIS_STOP && operation != TRANSLATE_AXIS_STOP &&
	    operation != I_DONT_KNOW_WHAT)
    fprintf(stderr,"Base client received unhandled %d,%lu from server\n",
	    operation,param);
}


void baseDispatchVariableMessage(int operation, int bufsize, char buffer[])
{
  switch(operation)
    {
    case TACTILE_newValues:
      {
	incorporateTactiles( (unsigned long*) buffer);
	break;
      }

    case SONAR_newValues:
      {
	incorporateSonars( (unsigned long*) buffer);
	break;
      }

    case IR_newValues:
      {
 	incorporateIrs( (unsigned long*) buffer);  
	break;
      }

    case BASE_statusReport:
      {
	incorporateStatusReport( (statusReportType*) buffer);
	break;
      }

    /*
     * The following message echoes all commands sent to the baseServer,
     * to every baseClient. It can be used for logging commands.
     * Implemented by Sebastian, 7-97
     */
    /*
    case BASE_confirmCommand:
      {
	break;			Just ignore 
      }
      */
    default:
      printf("Variable opn %d,%d not yet implemented \n",operation,bufsize); 	
      break;
    }
}


void handleBaseClientFixed(TCX_REF_PTR message,RAI_FixedMsgType *msg_ptr)
{
  int operation;
  unsigned long param;

  operation = msg_ptr->operation;
  param = msg_ptr->parameter;

#ifdef VERBOSE
    fprintf(stderr,"baseClient fixed msg %d %lu\n",operation,param); 
#endif
  baseDispatchFixedMessage(operation,param);
  tcxFreeByRef(message,msg_ptr);
}


void handleBaseClientVariable(TCX_REF_PTR message,RAI_VariableMsgType *msg_ptr)
{
  int operation;
  int bufsize;
  unsigned char* buffer;
  operation = msg_ptr->operation;
  bufsize = msg_ptr->bufsize;
  buffer = msg_ptr->buffer;

#ifdef VERBOSE
    fprintf(stderr,"baseClient received var msg %d %d\n",operation,bufsize); 
#endif
  baseDispatchVariableMessage(operation,bufsize,buffer);
  tcxFreeByRef(message,msg_ptr);
}


/******************* SETUP ***************************/

void registerBaseClient()
{
  int numberOfHandlers;
  int i;


  TCX_REG_HND_TYPE handlers[] = SERVER_reply_handlers;
  numberOfHandlers = sizeof(handlers)/sizeof(TCX_REG_HND_TYPE); 
  tcxRegisterHandlers(handlers, numberOfHandlers);


  /* so we can pass an  irType** to user functions */
  for(i = 0; i < bRobot.ir_rows; i++)
    irPtr[i]=irs[i];

  for(i = 0; i < bRobot.tactile_rows; i++)
    tactilePtr[i]=tactiles[i];

}


void findSERVER()
{
  printf("Connecting to Base Server \n");
  SERVER = tcxConnectModule(TCX_SERVER_MODULE_NAME);
  baseSendServerFixed(BASE_subscribe,0);  
  printf("Connected to Base Server\n");
}


/* ====================================================================== *\
 * ====================================================================== *
\* ====================================================================== */


static float local_IR_values[B_MAX_SENSOR_COLS];
static float local_Sonar_values[B_MAX_SENSOR_COLS];

int SonarCallbackRoutine(sonarType *sonar)
{
  static struct timeval last_sonar_report = {0,0};
  struct timeval now;
  int index, index2, newindex;
  static int fired[B_MAX_SENSOR_COLS];
  static int count = -1;
  static int count2 = 0;
  float dist;
  static int sonar_no[4];
  int i;

  if (count == -1){
    for(index2 = 0; index2 < bRobot.sonar_cols[0]; index2++)
      fired[index2] = 0;
    count = 0;
  }

  sonar_start_rot = (float) rwi_base.rot_position;
  sonar_start_pos_x = (float) rwi_base.pos_x;
  sonar_start_pos_y = (float) rwi_base.pos_y;

  
  for(index = 0; index < bRobot.sonar_cols[0]; index++)
    if (sonars[index].mostRecent){
#ifdef B21
      newindex = (index + 12) % bRobot.sonar_cols[0];
#else
      newindex = (index + 8) % bRobot.sonar_cols[0];
#endif
      sonars[index].mostRecent = 0;
      fired[newindex] += 1;
      sonar_no[count2] = newindex;

      dist = (float)sonars[index].value/10.0 - ROB_RADIUS;
      local_Sonar_values[newindex] = dist; 

      /*
       * Additional 2 cm are used to get distance
       * from base and not from enclosure.
       */

      count2++;
      if (count2 == 4){
	for (i = 0; i < bRobot.sonar_cols[0]; i++)
	  sonar_readings[i] = local_Sonar_values[i] - 2.0;
	FireHandler(sonar_handlers, SONAR_RT_COMPLETE, (Pointer) sonar_no);
	count2 = 0;
      }

      
      /*
       * Additional 2 cm are used to get distance
       * from base and not from enclosure.
       */

      count++;
      if (count == bRobot.sonar_cols[0]) {
	for (i = 0; i < bRobot.sonar_cols[0]; i++)
 	  sonar_readings[i] = local_Sonar_values[i] - 2.0;
	FireHandler(sonar_handlers, SONAR_REPORT, (Pointer) NULL);

	count = 0;
	gettimeofday(&now, NULL);

	last_sonar_report.tv_sec  = now.tv_sec;
	last_sonar_report.tv_usec = now.tv_usec;
      }
    }
  return 0;
}


/* ====================================================================== *\
 * ====================================================================== *
\* ====================================================================== */


/* ====================================================================== *\
 * ====================================================================== *
\* ====================================================================== */

#define IR_ROW_TO_BE_USED 1

/* ---------------------------------------------------------------------
 * Computes the angle of an ir sensor relative to the robot's heading
 * direction. 
 * ---------------------------------------------------------------------*/

float
angleOfIr( int row, int index, float indexRotation)
{
  static int firstTime = TRUE;
  static float angleLookup[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];
  float angle = 0.0;
  
  /* Initialize the relative positions at the first call. */
  if ( firstTime) {
    
    float startAngle = DEG_TO_RAD( 150.0);
    float angleStep  = DEG_360 / bRobot.ir_cols[IR_ROW_TO_BE_USED];
    int j;
    
    for ( j = 0; j < bRobot.ir_cols[IR_ROW_TO_BE_USED]; j++) {	
      angleLookup[IR_ROW_TO_BE_USED][j] = normed_angle( startAngle - j * angleStep);
    }
    firstTime = FALSE;
  }

  /* upper part */
  if ( row != IR_ROW_TO_BE_USED)
    fprintf( stderr, "Ir row not supported.\n");
  else if ( FOUND_INDEX) {    
    float currentRobotAngle = DEG_TO_RAD( activeStatusReport.Heading * 360.0
					  / (double) COUNTS_IN_360_DEGREES);
    if (0) fprintf(stderr, "index %f   rob %f   lookup %f\n",
		   RAD_TO_DEG(indexRotation),
		   RAD_TO_DEG(currentRobotAngle),
		   RAD_TO_DEG(angleLookup[row][index]));

    angle = normed_angle( indexRotation + angleLookup[row][index] +
			  currentRobotAngle);
  }
  else {
    angle = 0.0;
  }

#ifdef VERBOSE
  fprintf(stderr, "Ir on (%.2d,%.2d) at %f deg.\n",
	  row, index, DEG_TO_RAD(angle));
#endif	  
  return angle;
}


#define IR_THRESHOLD 93
float irSign[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0};

int IrCallbackRoutine( irType ** ir)
{
  static irInfo irs;
  int index, angleCnt = 0;

  for( index = 0; index <  bRobot.ir_cols[IR_ROW_TO_BE_USED]; index++)
    if (ir[IR_ROW_TO_BE_USED][index].mostRecent)
      angleCnt++;

  if ( angleCnt ==  bRobot.ir_cols[IR_ROW_TO_BE_USED]) {

    angleCnt = 0;
    
    for( index = 0; index <  bRobot.ir_cols[IR_ROW_TO_BE_USED]; index++) {

      ir[IR_ROW_TO_BE_USED][index].mostRecent = FALSE;
      ir[IR_ROW_TO_BE_USED][index].value *= irSign[index];
      
      if ( ir[IR_ROW_TO_BE_USED][index].value > IR_THRESHOLD) {
	irs.value[angleCnt] = ir[IR_ROW_TO_BE_USED][index].value;
	irs.angle[angleCnt++] = angleOfIr( IR_ROW_TO_BE_USED, index, indexHeading);
      }
    }
  
    irs.numberOfActivatedIrs = angleCnt;
    COLLI_IrHandler( &irs);
  }
  
  return 0;
}




/* ====================================================================== *\
 * ====================================================================== *
\* ====================================================================== */
void StatusCallbackRoutine(statusReportType *report)
{
  static struct timeval last_status_report = {0,0};
  struct timeval now;

  tmp_state.rot_position        = SafeSub((unsigned long)
					  report->Heading, 0.0) / COUNTS_PER_DEGREE; 

  tmp_state.time                = ((double) report->Clock) / 256.0;

  if (report->TranslateStatus & 0x100)
    tmp_state.trans_direction = NEGATIVE;
  else
    tmp_state.trans_direction = POSITIVE;

  if (report->RotateStatus & 0x100)
    tmp_state.rot_direction = NEGATIVE;
  else
    tmp_state.rot_direction = POSITIVE;

  tmp_state.trans_current_speed = ((double) report->TranslateVelocity) / 10.0;
  tmp_state.rot_current_speed = ((double) report->RotateVelocity) / COUNTS_PER_DEGREE;
  previous_state.pos_x = rwi_base.pos_x;
  rwi_base.pos_x =  0.1 * (((float) report->Xpos) -
                           (320.0*1024.0) / bRobot.base_posPerCm);
  previous_state.pos_y = rwi_base.pos_y;
  rwi_base.pos_y =  0.1 * (((float) report->Ypos) -
                           (320.0*1024.0) / bRobot.base_posPerCm);

  rwi_base.bump = FALSE;
  rwi_base.emergency = FALSE;
  rwi_base.battery_state.voltage = 0.0;
  rwi_base.battery_state.current = 0.0;
  rwi_base.battery_state.time = 0.0;
  BASE_waiting_for_battery_status = FALSE;

  gettimeofday(&now, NULL);

  last_status_report.tv_sec  = now.tv_sec;
  last_status_report.tv_usec = now.tv_usec;


  ReceivedStatusReport(FALSE);

}

/* ---------------------------------------------------------------------
 * Computes the angle of a tactile relative to the robot's heading
 * direction. 
 * ---------------------------------------------------------------------*/
float
angleOfTactile( int row, int index, float indexRotation)
{
  static int firstTime = TRUE;
  static float angleLookup[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];
  float angle;
  
  /* Initialize the relative positions at the first call. */
  if ( firstTime) {
    
    float top_lookup[] = { 180, 240, 240, 300, 300, 0,
			   0, 60, 60, 120, 120, 180 };
    float bottom_lookup[] = { 225, 225, 270, 270, 315, 315,
			      0, 0, 45, 45, 90, 90,
			      135, 135, 180, 180 };
    int i, j;
    
    for ( i = 0; i < bRobot.tactile_rows; i++) {      
      for ( j = 0; j < bRobot.tactile_cols[i]; j++) {	
	if ( i < 2) /* upper part */
	  angleLookup[i][j] = DEG_TO_RAD( top_lookup[j]);
	else        /* lower part */
	  angleLookup[i][j] = DEG_TO_RAD( bottom_lookup[j]);
      }
    }
    firstTime = FALSE;
  }
  
  
  /* upper part */
  if ( row < 2)
    angle = DEG_360 - angleLookup[row][index];
  else if (FOUND_INDEX) {
    float currentRobotAngle = DEG_TO_RAD( activeStatusReport.Heading * 360.0
					  / (double) COUNTS_IN_360_DEGREES);
    
    angle = normed_angle( indexRotation - angleLookup[row][index] +
			  currentRobotAngle);
  }
  else {
    angle = 0.0;
  }

#ifdef VERBOSE
  fprintf(stderr, "Contact on (%.2d,%.2d) at %f deg.\n",
	  row, index, RAD_TO_DEG(angle));
#endif	  
  return angle;
}


/* --------------------------------------------------------------------- */
/*                                                                       */
/* This function is called whenever the Robot receives tactile feedback  */
/*                                                                       */
/* WARNING: This function works properly only if a robot with            */
/*          following configuration is used:                             */
/*          o 2 upper rows of sensors (0,1)                              */
/*	    o 2 lower rows of sensors (2,3)                              */
/*	    o 12 columns of sensors in the upper part                    */
/*          o 16 columns of sensors in the lower part                    */
/*          e.g. Amelia meets these requirements.                        */
/* --------------------------------------------------------------------- */
int
TactileCallbackRoutine(tactileType **tactile) {

  int row;
  int index;
  double where = 0.0;
  double dummy = 0.0;

  static tactileInfo tactiles;
  static int firstTime = TRUE;
  int angleCnt = 0;
  
  if ( firstTime) {
    int numberOfTactiles = 0;

    firstTime = FALSE;
    
    for (row = 0; row < bRobot.tactile_rows; row++)
      numberOfTactiles += bRobot.tactile_cols[row];
  }
  
  for (row = 0, angleCnt = 0; row < bRobot.tactile_rows; row++) {
    for (index = 0; index < bRobot.tactile_cols[row]; index++)
      if ( tactile[row][index].value ) {
 	fprintf(stderr, "tac %d\n", index); 
	tactiles.angle[angleCnt++] = angleOfTactile( row, index, indexHeading);
      }
  }
  
  tactiles.numberOfActivatedTactiles = angleCnt;
  FireHandler( rwibase_handlers, BUMP, &tactiles);

  return 0;
}

/* ====================================================================== *\
 * ====================================================================== *
\* ====================================================================== */
void init_server_connection()
{
  registerBaseClient();
  
  findSERVER();

  if (use_sonar)
    registerSonarCallback(SonarCallbackRoutine);

  if (use_ir)
    registerIrCallback(IrCallbackRoutine);   

  statusReportData(REPORT_EVERYTHING);

  statusReportPeriod((unsigned long) 80);


  /* there's a problem if we go too fast, the robot will still find */
  /* its index, but it will skip the ROTATE_HALT message */
  setTranslateAcceleration(1000);
  setRotateAcceleration(150);
  setRotateVelocity(20);
  
  /* this should be improved at some point .. */
  if (!DONE_INDEXING) {
    if (!INDEXING) {
      printf("Finding rotate index, please wait ... \n");
      INDEXING = TRUE;
      rotateVelocityPos(); /* start it moving in case we are on the */
	                   /* index sensor */
      findRotIndex();
    }
    /* the base is indexing and we are waiting */
  }
  else {
    INDEXING = FALSE;
    FOUND_INDEX = TRUE;
    indexHeading = 0.0;
  }

  registerTactileCallback(TactileCallbackRoutine);

  registerStatusCallback(StatusCallbackRoutine);
}
