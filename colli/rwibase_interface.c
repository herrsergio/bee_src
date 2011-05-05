
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






/*****************************************************************************
 * PROJECT: Rhino
 *
 * FILE: rwibase_interface.c
 *
 * ABSTRACT:
 * 
 * Straightforward translation of the base interface commands to C.
 *
 *****************************************************************************/

#ifdef VMS
#include "vms.h"
#include "vmstime.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <sys/time.h>
#include "tcx.h"
#include "tcxP.h"
#include "SIMULATOR-messages.h"
#include "COLLI-messages.h"
#include "server.h"
#include "Common.h"
#include "libc.h"

#define  DECLARE_RWIBASE_VARS
#include "rwibase_interface.h"
#include "collision.h"
#include "collisionIntern.h"

#undef DEBUG
#undef DEBUG2

/*****************************************************************************
 * Global Constants
 *****************************************************************************/

#define COMMAND_TIMEOUT  10
#define RESPONSE_TIMEOUT 10
#define LINES_REPORT     9

/*****************************************************************************
 * Global variables 
 *****************************************************************************/

/* external variables */
_Base rwi_base;
_Base previous_state;
_Base tmp_state;

/* These values can be set by user processes */
float desired_rot_velocity;
float desired_trans_velocity;


/* private variables */

static char *report_data[LINES_REPORT] = { 
  "CLK",
  "BBS",
  "BPX",
  "BPY",
  "TPH",
  "TVE",
  "TSF",
  "RVE",
  "RSF",
};


static FILE *debug_file_in = NULL;

HandlerList rwibase_handlers;
static unsigned rotate_reference = 0;
static int translate_reference = 0;
BOOLEAN BASE_waiting_for_battery_status=FALSE;
static BOOLEAN BASE_waiting_for_report=FALSE;

/* Forward declaration of private procedures */
static void InitBase();
static BOOLEAN InitDevice();
static void JoystickDisable(void);
static void ResetJoystick(void);
static void HalfDuplex(void);
static void TorqueLimit (b8_integer limit);
static void InitializeTorsoPosition(void);
static void StatusReportData (b32_integer items);
static void SetRotationReference(Pointer rot_position, Pointer client_data);
static void ProcessBump(void);
static void FireEventHandlers(void);
static int WriteCommand (char *buffer, int nbytes, unsigned long l);
static void BaseParseReturn(char *start);
long SafeSub (unsigned long l1, unsigned long l2);
static void BASE_report_check(void);


/********************/
/* PUBLIC FUNCTIONS */
/********************/

/*	Function Name: BASE_init
 *	Arguments:     
 *	Description:   Open the base device and initialize it
 *	Returns:       Returns true if device opened successfully, false otherwise.
 *
 */

BOOLEAN BASE_init()
{
  struct timeval poll_interval;
  
  rwibase_handlers = CreateHandlerList(BASE_NUMBER_EVENTS);

  InitBase();

  if (!base_device.dev.use_simulator && !base_device.dev.use_rwi_server){
    devInit();
    if (!InitDevice())
      return FALSE;
  }
    

  if (!base_device.dev.use_rwi_server){	/*!*/
    JoystickDisable();
    HalfDuplex();
    TorqueLimit(150);
    BASE_RotateAcceleration(35.0);
    BASE_TranslateAcceleration(20.0);
    BASE_RotateVelocity(2.0);
    BASE_TranslateVelocity(5.0);
    StatusReportData(CLOCK          |
		     TRANS_POS_X    |
		     TRANS_POS_Y    |
		     TRANS_VELOC    |
		     TRANS_STATUS   |
		     ROT_POS        |
		     ROT_VELOC      |
		     ROT_STATUS     |
		   BUMP_SWITCHES);
  }
  
  /* Only temporary because simulator can handle only ONE BASE_SetIntervalUpdates().
   * The interval for the collision avoidance is set in init_collision_avoidance().
   */
  if (!base_device.dev.use_simulator || !use_collision)
    if (!base_device.dev.use_rwi_server)
      BASE_SetIntervalUpdates(STREAMING_MILLISECONDS);
  
  if (base_device.dev.use_rwi_server) {
    baseSendServerFixed(BASE_assumeWatchdog, 0);
  }

  poll_interval.tv_usec = 0;
  poll_interval.tv_sec = CHECKING_SECONDS; 
  
  if (!base_device.dev.use_simulator && !base_device.dev.use_rwi_server)
      devStartPolling(&base_device.dev, &poll_interval, 
		      BASE_report_check);
  return TRUE;
}

void BASE_JoystickDisable(void)
{
  JoystickDisable();
}


void BASE_ResetJoystick(void)
{
  ResetJoystick();
}

/*	Function Name: 
 *	Arguments:
 *	Description:
 *	Returns: 
 */

void BASE_SetRotationReference(void)
{
  InitializeTorsoPosition(); 
  BASE_QueryRotatePosition(SetRotationReference, NULL);
}


void BASE_SetPos(double x, double y, double orientation)
{
  rwi_base.pos_x = x;
  rwi_base.pos_y = y;
  rwi_base.orientation = orientation;
  
  previous_state.pos_x = x;
  previous_state.pos_y = y;
  previous_state.orientation = orientation;
}


/*	Function Name: BASE_outputHnd
 *	Arguments:     fd -- file descriptor
 *                     chars_available -- numbers of chars read
 *	Description:   Handles character output from the base
 *	Returns: 
 */

#define BASE_BUFFER_SIZE DEFAULT_LINE_LENGTH

void BASE_outputHnd(int fd, long chars_available)
{
  static char buffer[BASE_BUFFER_SIZE+1];
  static char *startPos = buffer; /* position to start parsing from */
  static char *endPos = buffer; /* position to add more characters */
  char *lineEnd;
  int numRead = 0;

  while (chars_available > 0) {
    if (startPos == endPos)
      { 
	startPos = endPos = buffer;
	bzero(buffer, BASE_BUFFER_SIZE+1);
      }
    
    /* read in the output. */
    numRead = readN(&base_device.dev, endPos, 
		    MIN(chars_available,(BASE_BUFFER_SIZE - 
					 (endPos - startPos))));
    endPos += numRead;
    if (numRead == 0)
      { /* handle error here. The port is already closed. */
      }
    else {
      /* see if we have a \n  or null character */
      lineEnd = (char *) strpbrk(startPos,"\n\r");
      while (lineEnd != NULL)
	{/* found a string, pass it to the parsing routines. */
	  *lineEnd = '\0';
	  BaseParseReturn(startPos);
	  startPos = lineEnd+1;
	  lineEnd = (char *) strpbrk(startPos,"\n\r");
	}
      /* Fix up the buffer. Throw out any consumed lines.*/
      if (startPos >= endPos) 
	{ /* parsed the whole thing, just clean it all up */
	  bzero(buffer, BASE_BUFFER_SIZE+1);
	  startPos = endPos = buffer;
	}
      else if (startPos != buffer)
	{ /* slide it back and wait for more characters */
	  bcopy(startPos, buffer, (endPos - startPos));
	  endPos = buffer + (endPos - startPos);
	  startPos = buffer;
	}
    }
    chars_available = numChars(fd);
  }
}


/*	Function Name: BASE_timeoutHnd
 *	Arguments:
 *	Description:   Called when there is a timeout
 *	Returns: 
 */

void BASE_timeoutHnd(void)
{
  /* stub : will be called when there is a timeout.
   */
}


/*	Function Name: BASE_terminate
 *	Arguments:      
 *	Description:   Close the base device
 *	Returns: 
 */

void BASE_terminate(void)
{
  close(base_device.dev.fd);
  base_device.dev.fd = -1;
}


/*	Function Name: BASE_Debug
 *	Arguments:     debug_flag -- boolean
 *	Description:   This function set or reset the base debug, which
 *                     will printout the characters send and received by
 *                     the base interface.
 *	Returns: 
 */

void BASE_Debug (BOOLEAN debug_flag, char *file_name)
{
  char file[30];
  
  base_device.dev.debug = debug_flag;
  if (debug_flag &&
      file_name != NULL) {
    strcpy (file, file_name);
    strcat (file, ".out");
    base_device.dev.debug_file = fopen(file, "w");
    strcpy (file, file_name);
    strcat (file, ".in");
    debug_file_in = fopen(file, "w");
  }
}



/*	Function Name: BASE_StatusReportData
 *	Arguments:     items -- items in the status report. 32 bit, 
 *                              with an 1 in the bits corresponding to
 *                              the data in which we are interested.
 *	Description:   Setup the data for periodic status reports
 *	Returns:       
 */

void BASE_StatusReportData (int items)
{
  if (base_device.dev.use_rwi_server)
    baseSendServerFixed(BASE_statusReportData, items);
  else
    WriteCommand ("SD", 32, items);
}


/*	Function Name: BASE_SetIntervalUpdates
 *	Arguments:     milliseconds -- rate of incoming status reports
 *	Description:   Setup the period for the status report
 *	Returns: 
 */

void BASE_SetIntervalUpdates(long milliseconds)
{
  long rwi_period;

  rwi_period = milliseconds * 256 / 1000;
  if (base_device.dev.use_rwi_server)
    baseSendServerFixed(BASE_statusReportPeriod, rwi_period);
  else
    WriteCommand("SP", 16, rwi_period);
}



/*	Function Name: 
 *	Arguments:
 *	Description:
 *	Returns: 
 */

void BASE_InstallHandler(Handler handler, int event, Pointer client_data)
{
  InstallHandler(rwibase_handlers, handler, event, client_data);
}



/*	Function Name: 
 *	Arguments:
 *	Description:
 *	Returns: 
 */

void  BASE_RemoveHandler(Handler handler, int event)
{
  RemoveHandler(rwibase_handlers, handler, event);
}


void  BASE_RemoveAllHandlers(int event)
{
  RemoveAllHandlers(rwibase_handlers, event);
}


/*******************************/
/* QUERIES and ANSWER-Handlers */
/*******************************/


/*	Function Name: BASE_QueryBatteryStatus
 *	Arguments:     
 *	Description:   This query returns the battery voltage and current
 *                     running to the base control computer and the top
 *                     plate connector. struct battery_state includes time 
 *                     of update.
 *	Returns:       Nothing
 */

void BASE_QueryBatteryStatus (void)
{
  BASE_waiting_for_battery_status = TRUE;
  if (base_device.dev.use_rwi_server){
    baseSendServerFixed(BASE_batteryVoltage, 0);
    /* clock is missing here */
  }
  else{
    WriteCommand("BV", 0, 0);
    WriteCommand("CL", 0, 0);
  }
}


/*	Function Name: BASE_QueryRotatePosition
 *	Arguments:     
 *	Description:   This query returns the torso orientation
 *	Returns:       Nothing
 */

void BASE_QueryRotatePosition (Handler handler, Pointer client_data)
{
  InstallHandler(rwibase_handlers, handler, ANSWER_ROTATE_POSITION, client_data);
  if (base_device.dev.use_rwi_server)
    baseSendServerFixed(BASE_rotateWhere, 0);
  else
    WriteCommand("RW", 0, 0);
}


/******************/
/* MISC. COMMANDS */
/******************/

/*	Function Name: BASE_WatchDogTimer
 *	Arguments:     WDT timeout in milliseconds
 *	Base manual:   This command sets the WDT in the base.  If
 *                     the timer counts down to zero because it was
 *                     not set again, or set to 0, the base will
 *                     RH and TH (Rotate Halt, Translate Halt) and
 *                     then issue a message that it has done so.
 *	Returns:       Nothing
 */

void BASE_WatchDogTimer(int msecs)
{
  if (base_device.dev.use_rwi_server) {
    baseSendServerFixed(BASE_watchdogTimer, msecs*256/1000);
  }
  else
    WriteCommand ("WD", 16, msecs*256/1000); 
}



/*	Function Name: BASE_Kill
 *	Arguments:     
 *	Base manual:   This command causes both motors to go limp. This
 *                     command does not observe acceleration and
 *                     velocity parameters.
 *	Returns:       Nothing
 */

void BASE_Kill(void)
{
  if (base_device.dev.use_rwi_server)
    baseSendServerFixed(BASE_baseKill, 0);
  else
    WriteCommand ("KI", 0, 0);
}


/*	Function Name: BASE_DisableEmergencyProcedure
 *	Arguments:
 *	Description:   Disable the emergency stop procedure
 *	Returns: 
 */

void BASE_DisableEmergencyProcedure(void)
{
  previous_state.emergency = rwi_base.emergency;
  previous_state.emergencyProcedure = rwi_base.emergencyProcedure;
  
  rwi_base.emergency = FALSE;
  rwi_base.emergencyProcedure = FALSE;
}


/*	Function Name: BASE_EnableEmergencyProcedure
 *	Arguments:
 *	Description:   Enable the emergency stop procedure
 *	Returns: 
 */

void BASE_EnableEmergencyProcedure(void)
{
  previous_state.emergencyProcedure = rwi_base.emergencyProcedure;
  
  rwi_base.emergencyProcedure = TRUE;
}



/*********************/
/* ROTATION COMMANDS */
/*********************/



/*	Function Name: BASE_RotateCollisionAcceleration
 *	Arguments:     acceleration -- double, in degrees/second squared
 *	Base manual:   This command sets the internal rotational
 *                     acceleration variable.  The parameter is in
 *                     encoder counts per second squared. This command
 *                     may affect motions already in progress, but it
 *                     alone does not cause the base to move.
 *	Returns:       Nothing
 */

void BASE_RotateCollisionAcceleration (double acceleration)
{
  previous_state.rot_acceleration = rwi_base.rot_acceleration;
  rwi_base.rot_acceleration = acceleration;

  if (base_device.dev.use_rwi_server)
    baseSendServerFixed(BASE_setRotateAcceleration, 
			abs(acceleration * COUNTS_PER_DEGREE));
  else
    WriteCommand ("RA", 16, abs(acceleration * COUNTS_PER_DEGREE));

}



/*	Function Name: BASE_RotateAcceleration
 *	In the target mode no other process is allowed to change the
 *      acceleration.
 */

void BASE_RotateAcceleration (double acceleration)
{
    /* If the target flag is set no one is allowed to set the acceleration
     * above the initial acceleration. */

  if (!use_collision || ACTUAL_MODE == NULL) 
    BASE_RotateCollisionAcceleration (acceleration);
  else 
    BASE_RotateCollisionAcceleration ( MIN( acceleration,
					   RAD_TO_DEG(ACTUAL_MODE->target_rot_acceleration)));
}



/*	Function Name: BASE_RotateHalt
 *	Arguments:
 *	Description:   Check the current velocity and only send
 *                     the command if needed
 *	Base manual:   This command decelerates and stops the rotation
 *                     motor. The rotational velocity and acceleration
 *                     values do not change.
 *	Returns:       Nothing
 */

void BASE_RotateHalt(void)
{
  target_flag = FALSE; 
  rot_wanted = FALSE;

  rwi_base.rotate_relative_mode = FALSE;

  if (base_device.dev.use_rwi_server)
    baseSendServerFixed(BASE_rotateHalt, 0);
  else
    WriteCommand ("RH", 0, 0);
}


/*	Function Name: BASE_RotateCollisionVelocity
 *	Arguments:     velocity -- double, in degrees/sec
 *	Description:   See RotateVelocity
 *	Base manual:   This command sets the internal rotational
 *                     velocity variable. This variable can be set only
 *                     by the collision avoidance. The user is only allowed to 
 *                     change the desired_rot_velocity. The velocity value is in
 *                     encoder counts per second. The rotational
 *                     velocity will never exceed this programmed
 *                     velocity unless the velocity is already over
 *                     it, in which case, the rotation will decelerate
 *                     until it is at the new velocity. This command
 *                     may affect motions already in progress, but it
 *                     alone does not cause the base to move.
 *	Returns:       Nothing
 */

void BASE_RotateCollisionVelocity (double velocity)
{
  if (velocity >=  0.0) {
    if (velocity != rwi_base.rot_set_speed) {
      previous_state.rot_set_speed = rwi_base.rot_set_speed;
      if (rwi_base.rot_set_direction == POSITIVE)
	rwi_base.rot_set_speed = velocity;
      else
	rwi_base.rot_set_speed = -velocity;
      if (base_device.dev.use_rwi_server)
	baseSendServerFixed(BASE_setRotateVelocity, 
			    velocity * COUNTS_PER_DEGREE);
      else
	WriteCommand("RV", 16, velocity * COUNTS_PER_DEGREE);
    }
  }
  else
    fprintf(stderr, "Error in BASE_RotateCollisionVelocity>> velocity must be positive\n");
}


/*	Function Name: BASE_RotateVelocity
 *	Arguments:     velocity -- double, in degrees/sec
 *	Description:   See RotateVelocity
 *	Base manual:   This command sets the by the user DESIRED rotational
 *                     velocity variable (see BASE_RotateCollisionVelocity). 
 *                    The velocity value is in
 *                     encoder counts per second. The rotational
 *                     velocity will never exceed this programmed
 *                     velocity unless the velocity is already over
 *                     it, in which case, the rotation will decelerate
 *                     until it is at the new velocity. This command
 *                     may affect motions already in progress, but it
 *                     alone does not cause the base to move.
 *	Returns:       Nothing
 */

void BASE_RotateVelocity (double velocity)
{
  if (use_collision){
    if (velocity >=  0.0) {
      if (velocity != desired_rot_velocity) {
	if (rwi_base.rot_set_direction == POSITIVE)
	  desired_rot_velocity = velocity;
	else
	  desired_rot_velocity = -velocity;
      }
    }
    else
      fprintf(stderr, "Error in BASE_RotateVelocity>> velocity must be positive\n");
  }
  else
    BASE_RotateCollisionVelocity(velocity);
}



/*	Function Name: BASE_RotateClockwise
 *	Arguments:     
 *	Description:   Check the current state of the base and only
 *                     send the command if needed
 *	Base manual:   This command cause the base to start rotating at
 *                     the programmed acceleration and velocity values
 *                     in the specified direction.  Positive stands
 *                     for clockwise and negative for anticlockwise.
 *                     The base will continue rotating until a new
 *                     rotation command is given.
 *	Returns:       Nothing
 */

void BASE_RotateClockwise(void)
{

  rot_wanted = TRUE;    /* for the collision avoidance */

  rwi_base.rotate_relative_mode = FALSE;

  /*  if ((rwi_base.rot_current_speed == 0.0 || 
      rwi_base.rot_direction != POSITIVE) &&
      rwi_base.rot_set_speed != 0.0)
      */

  /* We have to send the command to the BASE even if (rwi_base.rot_set_speed == 0.0)
   * because the collision avoidance will raise the velocity in the next cycle.
   */

  if (rwi_base.rot_current_speed == 0.0 || rwi_base.rot_direction != POSITIVE)
    {
      
      previous_state.rot_set_direction = rwi_base.rot_set_direction;
      rwi_base.rot_set_direction = POSITIVE;
      
      previous_state.rot_set_speed = rwi_base.rot_set_speed;
      desired_rot_velocity = fabs(desired_rot_velocity);
      rwi_base.rot_set_speed = fabs(rwi_base.rot_set_speed);
      
      if ( dumpInfo)
	fprintf( dumpFile, "Send R+ to the base\n");
      
      if (base_device.dev.use_rwi_server)
	baseSendServerFixed(BASE_rotateVelocityPos, 0); 
      /*baseSendServerFixed(BASE_rotateRelativePos, 10);*/
      else
	WriteCommand("R+", 0, 0);
    }
}



/*	Function Name: BASE_RotateAnticlockwise
 *	Arguments:     
 *	Description:   Check the current state of the base and only
 *                     send the command if needed
 *	Base manual:   This command cause the base to start rotating at
 *                     the programmed acceleration and velocity values
 *                     in the specified direction.  Positive stands
 *                     for clockwise and negative for anticlockwise.
 *                     The base will continue rotating until a new
 *                     rotation command is given.
 *	Returns:       Nothing
 */

void BASE_RotateAnticlockwise(void)
{

  rot_wanted = TRUE;

  rwi_base.rotate_relative_mode = FALSE;

  /*  if ((rwi_base.rot_current_speed == 0.0 || 
      rwi_base.rot_direction != NEGATIVE) &&
      rwi_base.rot_set_speed != 0.0)
      */

  /* We have to send the command to the BASE even if (rwi_base.rot_set_speed == 0.0)
   * because the collision avoidance will raise the velocity in the next cycle.
   */

  if ( rwi_base.rot_current_speed == 0.0 || rwi_base.rot_direction != NEGATIVE)

    {
      previous_state.rot_set_direction = rwi_base.rot_set_direction;
      rwi_base.rot_set_direction = NEGATIVE;
      
      previous_state.rot_set_speed = rwi_base.rot_set_speed;
      desired_rot_velocity = -fabs(desired_rot_velocity);
      rwi_base.rot_set_speed = -fabs(rwi_base.rot_set_speed)+0.0;
      
      if ( dumpInfo)
	fprintf( dumpFile, "Send R- to the base\n");

      if (base_device.dev.use_rwi_server)
	baseSendServerFixed(BASE_rotateVelocityNeg, 0);
      
      else
	WriteCommand("R-", 0, 0);
    }
}


/*	Function Name: BASE_Rotate
 *	Arguments:     position -- double with number of degrees
 *	Description:   Position may be positive or negative. See 
 *                     RotateRelativePositive and RotateRelativeNegative.
 *	Base manual:   This command cause the base to rotate to a new
 *                     position given as an offset of its present
 *                     position using the current acceleration and
 *                     velocity values. The parameter units are
 *                     encoder counts.
 *	Returns:       Nothing
 */

void BASE_Rotate (double position)
{
  rot_wanted = TRUE;

  /* The robot has to rotate to a certain position. */
  rwi_base.rotate_relative_mode = TRUE; 
  rwi_base.still_to_rotate = position;

  previous_state.rot_set_direction = rwi_base.rot_set_direction;
  if (position > 0.0) {
    rwi_base.rot_set_direction = POSITIVE;
    if (base_device.dev.use_rwi_server)
      baseSendServerFixed(BASE_rotateRelativePos, 
			  (position * COUNTS_PER_DEGREE));
    else
      WriteCommand("R>", 32, (int) (position * COUNTS_PER_DEGREE));
  }
  else {
    rwi_base.rot_set_direction = NEGATIVE; 
    if (base_device.dev.use_rwi_server)
      baseSendServerFixed(BASE_rotateRelativeNeg, 
			  (-position * COUNTS_PER_DEGREE));
    else
      WriteCommand("R<", 32, (int) (-position * COUNTS_PER_DEGREE));
  }
  
  previous_state.rot_set_speed = rwi_base.rot_set_speed;
  if (rwi_base.rot_set_direction == POSITIVE) {
    desired_rot_velocity = fabs(desired_rot_velocity);
    rwi_base.rot_set_speed = fabs(rwi_base.rot_set_speed);
  }
  else {
    desired_rot_velocity = - fabs(desired_rot_velocity);
    rwi_base.rot_set_speed = -fabs(rwi_base.rot_set_speed)+0.0;
  }
}


/*	Function Name: BASE_RotateTo
 *	Arguments:     position -- double with the degrees (0-north, 90-east)
 *	Description:   Rotate to the absolute position. For this command works
 *                     properly it is needed the information returned by the
 *                     status report.
 *	Base manual:   This command causes the base to rotate to a
 *                     specified absolute encoder position. This
 *                     rotation occurs at the programmed velocity and
 *                     acceleration. When switched on, the rotation
 *                     encoder's position is set to 0 and thereafter
 *                     tracks the cumulative rotational motions. Refer
 *                     to rotate relative position for more convenient
 *                     rotate commands.
 *	Returns:       Nothing
 */

void BASE_RotateTo (double position)
{
  /* fox: there is no support for recovery from watch dog stop (see BASE_Rotate). */

  int current_direction_in_counts;
  int wanted_direction_in_counts;
  unsigned long current_counts, increment;
  
  rot_wanted = TRUE;

  current_counts = rwi_base.rot_position * COUNTS_PER_DEGREE;
  current_direction_in_counts = current_counts % COUNTS_IN_360_DEGREES;
  wanted_direction_in_counts = position * COUNTS_PER_DEGREE;
  increment = (wanted_direction_in_counts - current_direction_in_counts) %
    COUNTS_IN_360_DEGREES;
  if (base_device.dev.use_rwi_server)
    baseSendServerFixed(BASE_rotateToPosition, 
			current_counts + increment + 0x80000000);
  else
    WriteCommand ("RP", 32, current_counts + increment + 0x80000000);
}




/************************/
/* TRANSLATION COMMANDS */
/************************/


/*	Function Name: BASE_TranslateCollisionAcceleration
 *	Arguments:     acceleration -- double, in cm/second squared
 *	Base manual:   This command sets the internal translational
 *                     acceleration variable.  The parameter is in
 *                     encoder counts per second squared. This command
 *                     may affect motions already in progress, but it
 *                     alone does not cause the base to move.
 *	Returns:       Nothing
 */

void BASE_TranslateCollisionAcceleration (double acceleration)
{
  previous_state.trans_acceleration = rwi_base.trans_acceleration;
  rwi_base.trans_acceleration = acceleration;
  if (base_device.dev.use_rwi_server)
    baseSendServerFixed( BASE_setTranslateAcceleration, 
			 (long) abs( acceleration * BASE_SERVER_COUNTS_PER_CM));
  else
    WriteCommand ("TA", 16, abs(acceleration * BASE_COUNTS_PER_CM));
}


/*	Function Name: BASE_TranslateAcceleration
 *	Arguments:     acceleration -- double, in cm/second squared
 *	Base manual:   This command sets the internal translational
 *                     acceleration variable.  The parameter is in
 *                     encoder counts per second squared. This command
 *                     may affect motions already in progress, but it
 *                     alone does not cause the base to move.
 *	Returns:       Nothing
 */

void BASE_TranslateAcceleration (double acceleration)
{
  /* If the target flag is set no one is allowed to set the acceleration
   * above the initial acceleration. */
  if ( !use_collision || ACTUAL_MODE == NULL) 
    BASE_TranslateCollisionAcceleration (acceleration);
  else 
    BASE_TranslateCollisionAcceleration ( MIN( acceleration,
					      ACTUAL_MODE->target_trans_acceleration));
}



/*	Function Name: BASE_TranslateHalt
 *	Arguments:
 *	Description:   Check the current state of the base and only
 *                     send the command if needed
 *	Base manual:   This command decelerates and stops the translation
 *                     motor. The translational velocity and acceleration
 *                     values do not change.
 *	Returns:       Nothing
 */

void BASE_TranslateHalt(void)
{
  target_flag = FALSE;  
  if (base_device.dev.use_rwi_server)
    baseSendServerFixed(BASE_translateHalt, 0);
  else
    WriteCommand ("TH", 0, 0);
}



/*	Function Name: BASE_TranslateCollisionVelocity
 *	Arguments:     velocity -- double, in cm/sec
 *	Base manual:   This command sets the internal translational velocity of the base.
 */

void BASE_TranslateCollisionVelocity (double velocity)
{
  if (velocity >= 0.0) {
    if (velocity != rwi_base.trans_set_speed) {
      previous_state.trans_set_speed = rwi_base.trans_set_speed;
      if (rwi_base.trans_set_direction == POSITIVE)
	rwi_base.trans_set_speed = velocity;
      else
	rwi_base.trans_set_speed = -velocity;
      if (base_device.dev.use_rwi_server)
	baseSendServerFixed(BASE_setTranslateVelocity, 
			    ((unsigned long) (velocity * BASE_SERVER_COUNTS_PER_CM)));
      else
	WriteCommand("TV", 16, velocity * BASE_COUNTS_PER_CM); 
    }
  }
  else
    fprintf(stderr, "Error in BASE_TranslateCollisionVelocity>> velocity must be positive\n");
}



/*	Function Name: BASE_TranslateVelocity
 *	Arguments:     velocity -- double, in cm/sec
 *	Base manual:   This command sets the internal DESIRED translational
 *                     velocity variable. The value used by the base can be set
 *                     only by the collision avoidance (see BASE_TranslateCollisionVelocity).
 *                     The velocity value is in
 *                     encoder counts per second. The translational
 *                     velocity will never exceed this programmed
 *                     velocity unless the velocity is already over
 *                     it, in which case, the translation will decelerate
 *                     until it is at the new velocity. This command
 *                     may affect motions already in progress, but it
 *                     alone does not cause the base to move.
 *	Returns:       Nothing
 */


void BASE_TranslateVelocity (double velocity)
{
  if (use_collision){
    if (velocity >= 0.0) {
      if (velocity != desired_trans_velocity) {
	if (rwi_base.trans_set_direction == POSITIVE)
	  desired_trans_velocity = velocity;
	else
	  desired_trans_velocity = -velocity;
      }
    }
    else
      fprintf(stderr, "Error in BASE_TranslateVelocity>> velocity must be positive\n");
  }
  else
    BASE_TranslateCollisionVelocity(velocity);
}

/*	Function Name: BASE_TranslateForward
 *	Arguments:     
 *	Description:   Check the current state of the base and only
 *                     send the command if needed
 *	Description:   This command cause the base to start translating at
 *                     the programmed acceleration and velocity values
 *                     in the specified direction.  Positive stands
 *                     for forward and negative for backward.
 *                     The base will continue translating until a new
 *                     translation command is given.
 *	Returns:       Nothing
 */

void BASE_TranslateForward(void)
{
  if (rwi_base.emergency)
    return;
  if (rwi_base.trans_current_speed == 0 ||
      (rwi_base.trans_direction != POSITIVE &&
       rwi_base.trans_set_speed != 0.0))
    {
      
      previous_state.trans_set_direction = rwi_base.trans_set_direction;
      rwi_base.trans_set_direction = POSITIVE;
      
      desired_trans_velocity = fabs(desired_trans_velocity);
      previous_state.trans_set_speed = rwi_base.trans_set_speed;
      rwi_base.trans_set_speed = fabs(rwi_base.trans_set_speed);
      
      if ( dumpInfo)
	fprintf( dumpFile, "Send T+ to the base\n");
      
      if (base_device.dev.use_rwi_server)
	baseSendServerFixed(BASE_translateVelocityPos, 0);
      else
	WriteCommand("T+", 0, 0);
    }
}


/*	Function Name: BASE_TranslateBackward
 *	Arguments:     
 *	Description:   Check the current state of the base and only
 *                     send the command if needed
 *	Base manual:   This command cause the base to start translating at
 *                     the programmed acceleration and velocity values
 *                     in the specified direction.  Positive stands
 *                     for forward and negative for backward.
 *                     The base will continue translating until a new
 *                     translation command is given.
 *	Returns:       Nothing
 */

void BASE_TranslateBackward(void)
{
  if (rwi_base.emergency)
    return;
  if (rwi_base.trans_current_speed == 0 ||
      (rwi_base.trans_direction != NEGATIVE &&
       rwi_base.trans_set_speed != 0.0)) {
    
    previous_state.trans_set_direction = rwi_base.trans_set_direction;
    previous_state.trans_set_speed = rwi_base.trans_set_speed;

    rwi_base.trans_set_speed = (double) -fabs((float) rwi_base.trans_set_speed);
    desired_trans_velocity = - fabs(desired_trans_velocity);
    rwi_base.trans_set_direction = NEGATIVE;

    if ( dumpInfo)
      fprintf( dumpFile, "Send T- to the base\n");
    
    if (base_device.dev.use_rwi_server)
      baseSendServerFixed(BASE_translateVelocityNeg, 0);
    else
      WriteCommand("T-", 0, 0);
  }
}


/*	Function Name: BASE_Translate
 *	Arguments:     position -- double with number of cm
 *	Description:   Position may be positive or negative. See 
 *                     TranslateRelativePositive and TranslateRelativeNegative.
 *	Base manual:   This command cause the base to translate to a new
 *                     position given as an offset of its present
 *                     position using the current acceleration and
 *                     velocity values. The parameter units are
 *                     encoder counts.
 *	Returns:       Nothing
 */

void BASE_Translate (double position)
{
  if (rwi_base.emergency)
    return;
  
  previous_state.trans_set_direction = rwi_base.trans_set_direction;
  if (position > 0.0) {
    rwi_base.trans_set_direction = POSITIVE;
    if (base_device.dev.use_rwi_server)
      baseSendServerFixed(BASE_translateRelativePos, 
			  (position * BASE_SERVER_COUNTS_PER_CM));
    else
      WriteCommand("T>", 32, position * BASE_COUNTS_PER_CM);
  }
  else {
    rwi_base.trans_set_direction = NEGATIVE;
    if (base_device.dev.use_rwi_server)
      baseSendServerFixed(BASE_translateRelativeNeg, 
			  (-position * BASE_SERVER_COUNTS_PER_CM));
    else
      WriteCommand("T<", 32, -position * BASE_COUNTS_PER_CM);
  }
  previous_state.trans_set_speed = rwi_base.trans_set_speed;
  if (rwi_base.trans_set_direction == POSITIVE) {
    desired_trans_velocity = fabs(desired_trans_velocity);
    rwi_base.trans_set_speed = fabs(rwi_base.trans_set_speed);
  }
  else {
    desired_trans_velocity = -fabs(desired_trans_velocity);
    rwi_base.trans_set_speed = -fabs(rwi_base.trans_set_speed)+0.0;
  }
}

void BASE_HyperJump (float x, float y, float o)
{
  if (base_device.dev.use_simulator) {
    char buffer[55];
    
    sprintf(buffer, "HY %f %f %f\r", x,y,o);
    
    /* set the timeout, if one is available */
    if (base_device.dev.setTimeout != NULL)
      (* base_device.dev.setTimeout)(&base_device.dev,COMMAND_TIMEOUT);
    
    flushChars(&(base_device.dev));
    writeN(&(base_device.dev), buffer,strlen(buffer));
  }
  else
    fprintf(stderr, "WHAT THE HELL IS A HYPERJUMP???\n");
}


/*********************/
/* PRIVATE FUNCTIONS */
/*********************/


static void InitBase()
{
  rwi_base.time               = previous_state.time               = 0;
  rwi_base.rot_acceleration   = previous_state.rot_acceleration   = 0.0;
  rwi_base.rot_position       = previous_state.rot_position       = 0.0;
  /*  rwi_base.rot_status         = previous_state.rot_status         = STOPPED; */
  rwi_base.trans_acceleration = previous_state.trans_acceleration = 0.0;
  rwi_base.trans_position     = previous_state.trans_position     = 0.0;
  /*  rwi_base.trans_status       = previous_state.trans_status       = STOPPED; */
  rwi_base.bumpers            = previous_state.bumpers            = 0;
  rwi_base.bump               = previous_state.bump               = FALSE;
  rwi_base.emergency          = previous_state.emergency          = FALSE;
  rwi_base.emergencyProcedure = previous_state.emergencyProcedure = TRUE;
  rwi_base.pos_x              = previous_state.pos_x              = 0.0;
  rwi_base.pos_y              = previous_state.pos_y              = 0.0;
  rwi_base.orientation        = previous_state.orientation        = 0.0;

  /* For recovery from watch dog stop. */
  rwi_base.rotate_relative_mode        = previous_state.rotate_relative_mode = FALSE;
  rwi_base.still_to_rotate             = previous_state.still_to_rotate = 0.0;
  rwi_base.stopped_by_watch_dog        = previous_state.stopped_by_watch_dog = FALSE;
}


static BOOLEAN InitDevice()
{
  /* set the correct name for the base */
  connectTotty(&base_device.dev);
  if( base_device.dev.fd == -1)
    return FALSE;
  else {
    connectDev(&base_device.dev);
    return TRUE;
  }
}



/*	Function Name: ReceivedStatusReport
 *	Arguments:     
 *	Description:   Function done each time that a new status
 *                     report arrives from the base
 *	Returns: 
 */

void ReceivedStatusReport(BOOLEAN report_corrupted)
{

  static int cnt=0;
  
  if (!report_corrupted){
    
    /* First let's look wether the robot is in the rotate_relative_mode.
     * In this case we count the degrees the robot still has to rotate.
     * This is necessary because we want to finish the rotation if the
     * watch dog stopped the robot.
     */
    if (rwi_base.rotate_relative_mode) {
      double tmp;
      
      tmp = tmp_state.rot_position - rwi_base.rot_position;
      /* This has to be done because the position is normed to [0:360]. */
      if ( (rwi_base.still_to_rotate > 0.0) && (tmp < 0.0)) tmp += 360.0;
      if ( (rwi_base.still_to_rotate < 0.0) && (tmp > 0.0)) tmp -= 360.0;
      
      /* If the robot moves a bit too far. */
      if (fabs(tmp) > fabs(rwi_base.still_to_rotate))
	rwi_base.still_to_rotate = 0.0;
      else
	rwi_base.still_to_rotate -= (tmp);

      if ( 0 && fabs( rwi_base.still_to_rotate) > 5.0)
	fprintf(stderr, " Have to rotate %f degrees.\n",
		rwi_base.still_to_rotate);
    }

    if ( rwi_base.stopped_by_watch_dog || rwi_base.stopped_by_colli) {
      
      if ( rwi_base.stopped_by_watch_dog) {
	fprintf( stderr, "Recovered from WDT.\n");
	rwi_base.stopped_by_watch_dog = FALSE;
      }
      else {
	fprintf( stderr, "Recovered from timeout.\n");
	rwi_base.stopped_by_colli = FALSE;
      }
      
      if (rwi_base.rotate_relative_mode) {
	fprintf( stderr, "Finish rotation. Rotate by %f degrees.\n",
		     rwi_base.still_to_rotate);
	BASE_Rotate( rwi_base.still_to_rotate);
      }
    }

    /* clear the waiting flag */
    
    BASE_waiting_for_report=FALSE;
    
    /* store the previous state */
    
    previous_state = rwi_base;
    
    /* commit the data in the temporal variable */
    rwi_base.time = tmp_state.time;
    rwi_base.trans_position = tmp_state.trans_position;
    /* rwi_base.trans_status  = tmp_state.trans_status; */
    rwi_base.trans_direction = tmp_state.trans_direction;
    rwi_base.rot_position = tmp_state.rot_position;
    /* rwi_base.rot_status = tmp_state.rot_status; */
    rwi_base.rot_direction = tmp_state.rot_direction;
    rwi_base.trans_current_speed = tmp_state.trans_current_speed;
    rwi_base.rot_current_speed = tmp_state.rot_current_speed;
    
/* #define TEST_ACCELERATIONS */
#ifdef TEST_ACCELERATIONS
  {
    static BOOLEAN first = TRUE;
    static float lastTV, lastRV;
    float dtv, drv;
    float dt;
    
    if ( first) {
      setStartTime( 6);
      first = FALSE;
      lastTV = tmp_state.trans_current_speed;
      lastRV = tmp_state.rot_current_speed;
    }
    
    dtv = tmp_state.trans_current_speed - lastTV;
    drv = tmp_state.rot_current_speed - lastRV;
    dt = timeExpired( 6);
    
    if (dt > 0.001)
      fprintf( stderr, "Trans: %.1f  %.1f   Rot: %.1f  %.1f\n", tmp_state.trans_current_speed, dtv / dt, tmp_state.rot_current_speed, drv / dt);
    
    lastTV = tmp_state.trans_current_speed;
    lastRV = tmp_state.rot_current_speed;
    
    setStartTime( 6);
  }
#endif
  
  FireEventHandlers();
  FireHandler(rwibase_handlers, STATUS_REPORT, (Pointer) NULL);
  
    /* Received a status report. Set the watchDog for the next interval.
     * If collision avoidance is used, it will set its own watchDog. */
    if ( ! use_collision)
      BASE_WatchDogTimer((int) STREAMING_MILLISECONDS * 2.0);
  }
  else
    fprintf(stderr, ":Bad status report:\n");

  if (++cnt == BATTERY_QUERY_INTERVAL && !base_device.dev.use_rwi_server){
    BASE_QueryBatteryStatus();
    cnt = 0;
  }
}


/*	Function Name: SetRotationReference
 *	Arguments:
 *	Description:
 *	Returns: 
 */

static void SetRotationReference(Pointer rot_position, Pointer client_data)
{
  rotate_reference = (unsigned) 0.0 /*rot_position*/;
}


/*	Function Name: FireEventHandlers
 *	Arguments:     
 *	Description:   Look at the current and previous base state, sending
 *                     all the events that have occured 
 *	Returns: 
 */


static void FireEventHandlers(void)
{
  if (previous_state.trans_moving &&
      !rwi_base.trans_moving)
    FireHandler(rwibase_handlers, TRANS_STOPPED, (Pointer) TRANS_STOPPED);
  
  if (previous_state.rot_moving &&
      !rwi_base.rot_moving)
    FireHandler(rwibase_handlers, ROT_STOPPED, (Pointer) ROT_STOPPED);
}


static void ProcessBump(void)
{
  rwi_base.emergency = TRUE;
  BASE_TranslateHalt();
}


/*	Function Name: StatusReportData
 *	Arguments:     items -- items in the status report. 32 bit, 
 *                              with an 1 in the bits corresponding to
 *                              the data in which we are interested.
 *	Description:   Setup the data for periodic status reports
 *	Returns:       
 */

static void StatusReportData (b32_integer items)
{
  if (base_device.dev.use_rwi_server)
    baseSendServerFixed(BASE_statusReportData, items);
  else
    WriteCommand("SD", 32, items);
}



/*	Function Name: TorqueLimit
 *	Arguments:     limit - limit of translate torque. 
 *	Description:   The translate torque is by default really strong, 
 *                     so the robot even can go throgh walls!. 
 *                     A good limit is 20.
 *	Returns:       
 */

static void TorqueLimit (b8_integer limit)
{
  if (base_device.dev.use_rwi_server)
    baseSendServerFixed(BASE_setTranslateTorque, limit);
  else
    WriteCommand("TT", 8, limit);
}


/*	Function Name: 
 *	Arguments:
 *	Description:
 *	Returns: 
 */

static void JoystickDisable(void)
{
  if (base_device.dev.use_rwi_server){
    fprintf(stderr, "JoystickDisable is not implemented in RWI's base server. Sorry.\n");
    putc(7, stderr);
  }
  else
    WriteCommand("JD", 8, 1);
}


static void ResetJoystick(void)
{
  if (base_device.dev.use_rwi_server){
    fprintf(stderr, "JoystickDisable is not implemented in RWI's base server. Sorry.\n");
    putc(7, stderr);
  }
  else{
    WriteCommand("JD", 8, 0);
    WriteCommand("JD", 8, 1);
  }
}



/*	Function Name: 
 *	Arguments:
 *	Description:
 *	Returns: 
 */

static void HalfDuplex(void)
{
  if (!base_device.dev.use_rwi_server)
    WriteCommand("HD", 0, 0);
}

/*	Function Name: 
 *	Arguments:
 *	Description:
 *	Returns: 
 */

static void InitializeTorsoPosition(void)
{
  if (base_device.dev.use_rwi_server)
    baseSendServerFixed(BASE_findRotIndex, 0);
  else
    WriteCommand("IX", 0, 0);
}


/*	Function Name: 
 *	Arguments:
 *	Description:
 *	Returns: 
 */

static int WriteCommand (char *command, int n_bits, unsigned long l)
{
  unsigned long mask;
  char buffer[DEFAULT_LINE_LENGTH];
  
  if (!base_device.dev.use_simulator && !base_device.dev.use_rwi_server &&
      base_device.dev.fd == -1) {
    fprintf (stderr, "Error in base_interface::WriteCommand>> device is not initialized\n");
    return -1;
  }
  
  /* check that l doesn't overflow nbits */
  
  switch (n_bits) {
  case 0:
    l = 0;
    mask = 0x0;
  case 8:
    mask = 0xFF;
    break;
  case 16:
    mask = 0xFFFF;
    break;
  case 32:
    mask = 0xFFFFFFFF;
    break;
  default:
    fprintf(stderr, "Error in base_interface::WriteCommand>> bad nbits %d\n", n_bits);
    return -1;
  }
  if (~mask & l) {
    fprintf(stderr, "Error in base_interface::WriteCommand>> overflow on argument\n");
    fprintf(stderr, "debug: mask = %lx l = %lx\n", mask, l);
    fprintf(stderr, "debug: ~mask = %lx ~mask | l = %lx\n", ~mask, (~mask | l));
    return -1;
  }

  
  /* set the timeout, if one is available */
  if (!base_device.dev.use_simulator && !base_device.dev.use_rwi_server){
    if (n_bits != 0)
      sprintf (buffer, "%s %lx\r", command, l);
    else
      sprintf (buffer, "%s\r", command);
    
    if (base_device.dev.setTimeout != NULL)
      (* base_device.dev.setTimeout)(&base_device.dev,COMMAND_TIMEOUT);
    
    flushChars(&(base_device.dev));
    
    return ((int) writeN(&(base_device.dev), buffer,strlen(buffer)));
  }
  else if(base_device.dev.use_simulator) { /* **** simulator: Use TCX ********* */
    char *message;
    int length;

    message = (char *) malloc (sizeof(char) * DEFAULT_LINE_LENGTH);
    if (n_bits != 0)
      sprintf (message, "%s %x\r", command, (unsigned int) l);
    else
      sprintf (message, "%s\r", command);
    tcxSendMsg(SIMULATOR, "SIMULATOR_message_from_base", &message);
    length = strlen(message);
    free(message);
    return length; 
  }
  return(-1);
}

static BOOLEAN in_report = FALSE;
static int report_line = 0;
static BOOLEAN report_corrupted = FALSE;


/*	Function Name: ProcessLine
 *	Arguments:     line -- output line from the base
 *	Description:   process the base output, updating the global
 *                     variable rwi_base and firing the installed handlers
 *	Returns: 
 */

static void ProcessLine(char *line) 
{
  double data_double;
  float data_float; 
  int    data_int, len;
  unsigned long data_unsigned;
  char  a;
  if (base_device.dev.debug)
    if (debug_file_in)
      fprintf(debug_file_in,"BaseInterface receiving: [%s]\n", line);
    else
      fprintf(stderr,"BaseInterface receiving: [%s]\n", line);
  
  /* all the streaming data come without any *, and
   * the first line with one star marks the end of the
   * status report
   */
  
  a = *line++;
  
  if (a != '*')
    line--;
  
  if (in_report && !report_corrupted) {
    
    /* See if the string that comes is the correct one by 
       testing the command and the length of the line */
    
    len = strlen(report_data[report_line]);
    
    if (strncmp(line, report_data[report_line++], len) != 0){
      report_corrupted = TRUE;
      fprintf(stderr, "Bad data: missed one carriage return %s\n", line);
      return;
    }
  }
  
  
  if (strncmp (line, "%PSR", 2) == 0) {
    if (report_corrupted)
      ReceivedStatusReport(report_corrupted);
    report_corrupted = FALSE;
    in_report = TRUE;
    report_line = 0;
  }
  
  
  else if (strncmp (line, "CLK", 3) == 0) {
    
    /* first line of status report */
    if (report_corrupted)
      ReceivedStatusReport(report_corrupted);
    report_corrupted = FALSE;
    in_report = TRUE;

    report_line = 1;
    
    if (strlen(line) != 12) {
      report_corrupted = TRUE;
      return;
    }
    
    /* report clock: first data in report */
    
    sscanf(line,"CLK %x", &data_int);
    /* the units are 1/256 secs. */
    tmp_state.time = data_int / 256.0;
  }
  

  else if (strncmp (line, "TSF", 3) == 0) {
    
    /* report translate flag */
    
    if (!report_corrupted) {
      if (strlen(line) != 8) {
	report_corrupted = TRUE;
	fprintf(stderr, "Bad status report data %s\n", line);
	return;
      }
      
      sscanf(line, "TSF %x", &data_int);
      /* tmp_state.trans_status = data_int; */
      if (data_int & 0x100)
	tmp_state.trans_direction = NEGATIVE;
      else
	tmp_state.trans_direction = POSITIVE;
    }
  }
  
  else if (strncmp (line, "TPH", 3) == 0) {
    
    /* report rotate position (Top Plate Heading) */
    
    if (!report_corrupted) {
      if (strlen(line) != 8) {
	report_corrupted = TRUE;
	fprintf(stderr, "Bad status report data %s\n", line);
	return;
      }
      sscanf(line, "TPH %x", (unsigned int) &data_unsigned);
      if (rotate_reference == 0)
	rotate_reference = data_unsigned;
      /* the units are counts */
      tmp_state.rot_position = SafeSub(data_unsigned, 0.0 /*rotate_reference changed by S.Thrun 94-2-1 */) / COUNTS_PER_DEGREE; 
    }
  }
  
  else if (strncmp (line, "RSF", 3) == 0) {
    
    /* report rotate flag */
    
    if (!report_corrupted) {
      if (strlen(line) != 8) {
	report_corrupted = TRUE;
	fprintf(stderr, "Bad status report data %s\n", line);
	return;
      }
      sscanf(line, "RSF %x", &data_int);
      /*  tmp_state.rot_status = data_int; */
      if (data_int & 0x100) 
	tmp_state.rot_direction = NEGATIVE;
      else
	tmp_state.rot_direction = POSITIVE;
    }
    /* This is the last line of a status report */
    in_report = FALSE;
    if (report_line != LINES_REPORT) 
      report_corrupted = TRUE;
    ReceivedStatusReport(report_corrupted);
    report_corrupted = FALSE;
  }
  
  else if (strncmp (line, "TVE", 3) == 0) {
    
    /* translate velocity */
    
    if (!report_corrupted) {
      if (strlen(line) != 8) {
	report_corrupted = TRUE;
	fprintf(stderr, "Bad status report data %s\n", line);
	return;
      }
      sscanf(line,"TVE %x", &data_int);
      data_double = data_int;
      data_double /= BASE_COUNTS_PER_CM;
      tmp_state.trans_current_speed = data_double;
    }
  }
  
  else if (strncmp (line, "RVE", 3) == 0) {
    
    /* rotate velocity */
    
    if (!report_corrupted) {
      if (strlen(line) != 8) {
	report_corrupted = TRUE;
	fprintf(stderr, "Bad status report data %s\n", line);
	return;
      }
      sscanf(line,"RVE %x", &data_int);
      tmp_state.rot_current_speed = data_int / COUNTS_PER_DEGREE;
    }
  }
  
  
  else if (strncmp (line, "BPX", 3) == 0) {
    if (strlen(line) != 8) {
      report_corrupted = TRUE;
      fprintf(stderr, "Bad status report data %s\n", line);
      return;
    }
    else {
      sscanf(line,"BPX %x", &data_int);
      previous_state.pos_x = rwi_base.pos_x;
      rwi_base.pos_x = data_int - 32768;
      /* central position is: 0 0 instead of 32768 32768 */ 
    }
  }
  
  else if (strncmp (line, "BPY", 3) == 0) {
    sscanf(line,"BPY %x", &data_int);
    if (strlen(line) != 8) {
      report_corrupted = TRUE;
      fprintf(stderr, "Bad status report data %s\n", line);
      return;
    }
    else {
      previous_state.pos_y = rwi_base.pos_y;
      rwi_base.pos_y = data_int - 32768;
      /* central position is: 0 0 instead of 32768 32768 */
    }
  }

  
  else if (strncmp (line, "BBS", 3) == 0) {
    
    /* report bump */
    
    if (strlen(line) != 8) {
      report_corrupted = TRUE;
      fprintf(stderr, "Bad status report data %s\n", line);
    }
    if (!report_corrupted) {
      sscanf(line, "BBS %x", &data_int);
      rwi_base.bumpers = data_int;
      if (rwi_base.bumpers != 0xFFFF &&
	  previous_state.bumpers == 0xFFFF) {
	rwi_base.bump = TRUE;
	if (rwi_base.emergencyProcedure) {
	  ProcessBump();
	}
      }
      if (rwi_base.bumpers == 0xFFFF &&
	  previous_state.bumpers != 0xFFFF) {
	rwi_base.bump = FALSE;
	if (rwi_base.emergencyProcedure) {
	  rwi_base.emergency = FALSE;
	}
      }
    }
  }
  /* answers to queries */
  
  else if (strncmp (line, "BV", 2) == 0) {
    
    /* answer battery voltage */
    
    sscanf(line,"BV %x", &data_int);
    /* the units are 1/10 volts */
    rwi_base.battery_state.voltage = ((double) data_int) / 10.0;
    
  }
    
  else if (strncmp (line, "BC", 2) == 0) {
    
    /* answer battery current */
    
    sscanf(line,"BC %x", &data_int);
    /* the units are 1/10 amps */
    rwi_base.battery_state.current = ((double) data_int) / 10.0;
  }
    
  else if (strncmp (line, "CL", 2) == 0) {
    
    /* clock (maybe time of battery_state?) */
    
    sscanf(line,"CL %x", &data_int);
    /* the units are 1/256 secs */
    if (BASE_waiting_for_battery_status) {
      rwi_base.battery_state.time = ((double) data_int) / 256.0;
      BASE_waiting_for_battery_status = FALSE;
    }
    else {
      /* handler dealing with the time to be fired */
    }
    
  }
    
  else if (strncmp (line, "RW", 2) == 0) {
    
    /* answer rotate position */
    
    sscanf(line,"RW %x", (unsigned int) &data_unsigned);
    
    FireHandler(rwibase_handlers, ANSWER_ROTATE_POSITION, (Pointer) data_unsigned);
    BASE_RemoveAllHandlers(ANSWER_ROTATE_POSITION);
  }

  /*
   *  If more needs to be done when the base stops the motors
   *  because the Watch Dog Timer timed out, do it here - Tyson
   */
  
  else if (strncmp (line, "%WDT motors stopped", 4) == 0) {
    rwi_base.stopped_by_watch_dog = TRUE;
    fprintf(stderr, "***************************************\n");
    fprintf(stderr, "***************************************\n");
    fprintf(stderr, "WDT timed out.  Motors stopped by base.\n");
    fprintf(stderr, "***************************************\n");
    fprintf(stderr, "***************************************\n");
  }

  else
    fprintf(stderr, "BASE: unknown data: %s\n", line);
}
  

/*	Function Name: SIMULATOR_message_to_base_handler
 *	Arguments:
 *	Description:   Pseudo-handler for simulator
 *	Returns: 
 */


void SIMULATOR_message_to_base_handler(TCX_REF_PTR   ref,
				       char        **message)
{
  /* fprintf(stderr, "Received a base-message <%s> from the simulator\n", 
   *message);  */
  ProcessLine(*message);
  tcxFree("SIMULATOR_message_to_base", message);
}



/*	Function Name: 
 *	Arguments:
 *	Description:
 *	Returns: 
 */

long SafeSub (unsigned long l1, unsigned long l2)
{
  if (l1 > l2)
    return l1-l2;
  else
    return - (long) (l2-l1);
}



/*	Function Name: BaseParseReturn
 *	Arguments:
 *	Description:   Used to parse the return from the base, a line at a time
 *	Returns: 
 */

static void BaseParseReturn(char *start)
{
  /* cancel the timeout. */
  if (base_device.dev.cancelTimeout != NULL)
    (* base_device.dev.cancelTimeout)(&(base_device.dev));
  
  /* parse the characters in the buffer and update state */
  
  if (strcmp(start, "") != 0)
    ProcessLine (start);
  
}

static void BASE_report_check(void)
{
  if (BASE_waiting_for_report)
    {/* missed some reports from the base */
      FireHandler(rwibase_handlers, BASE_REPORT_MISSED, (Pointer) NULL);
    }
  BASE_waiting_for_report=TRUE;
}


/*****************************************************************************
 *
 * FUNCTION: void BASE_missed(Handler handler, 
 *                             Pointer client_data)
 *
 * DESCRIPTION: register a function to handle cases where a good base report 
 * never in the time that 5 are expected.
 *
 *****************************************************************************/

void BASE_missed(Handler handler, Pointer client_data)
{
  BASE_InstallHandler(handler, BASE_REPORT_MISSED, client_data);
}

		
