
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




#ifdef VMS
#include "vms.h"
#include "vmstime.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "Common.h"
/*#include "libc.h"*/
/*#include "LIB-math.h"*/
#define DECLARE_PANTILT_VARS
#include "panTilt_interface.h"
#include "devUtils.h"

/*****************************************************************************
 * Global variables 
 *****************************************************************************/
char failureString[80];

static int statusOK;
static float responseValue;
static char *waitForCommand = NULL;
static int maxPanPos, minPanPos, maxTiltPos, minTiltPos;
static int maxPanVel, minPanVel, maxTiltVel, minTiltVel;
static float panDegreesPerCount, tiltDegreesPerCount;

extern float global_pan_correction;

float global_pan_correction = 0.0;

#define TIMEOUT 1 /* In seconds */

      
DEGREES _180_to_180 (DEGREES value)
{
  for(;;) {
    if (value > 180.0)
      value -= 360.0;
    else if (value <= -180.0)
      value += 360.0;
    else
      return (value);
  }
}


#define NO_ARG -99999

/*****************************************************************************
 *
 * FUNCTION: int writeCommand
 *
 * DESCRIPTION: Write a command to the pan/tilt unit
 *
 * INPUTS:
 *
 * OUTPUTS: Returns TRUE if successful write, otherwise FALSE
 *
 * HISTORY: Merger of function by same name in rwibase_interface.c
 *          and function written by Conrad Poelman for the pan/tilt unit
 *
 *****************************************************************************/

static BOOLEAN writeCommand (char *command, int arg)
{
  char buffer[DEFAULT_LINE_LENGTH];
  
  if (panTilt_device.dev.fd == -1) {
    fprintf (stderr, "WriteCommand: Error: device is not initialized\n");
    return TRUE;
  }
  
  if (arg != NO_ARG) {
    sprintf (buffer, " %s%d ", command, arg);
  } else {
    sprintf (buffer, " %s ", command);
  }

#if 0  
  fprintf(stderr, "%s:%6d:%s() - [%s]\n",
	  __FILE__, __LINE__, __FUNCTION__, buffer);
#endif
  return ((int) writeN(&(panTilt_device.dev), buffer, strlen(buffer)));
}

BOOLEAN panTiltQuery (char *command, int arg)
{
  waitForCommand = command;
  writeCommand(command, arg);
  return WaitForResponse(&(panTilt_device.dev), &statusOK, TIMEOUT);
}

void panTiltParse (char *line)
{
  int i;
  char *restLine;

#if 0
  fprintf(stderr, "%s:%6d:%s() - line = [%s:%s]\n",
	  __FILE__, __LINE__, __FUNCTION__, waitForCommand, line);
#endif  

#if 1
  restLine = line;

  while (*restLine) {

    if (waitForCommand &&
	strncmp(waitForCommand, restLine, strlen(waitForCommand)) == 0) {

      statusOK = 1;
      restLine += strlen(waitForCommand);

      while (restLine[0] != '\0' && restLine[0] != '*') {
	restLine++;
      }

      while (restLine[0] != '\0') {
	if (isdigit(restLine[0]) ||
	    restLine[0] == '-' ||
	    restLine[0] == '.') {
	    responseValue = atof(&(restLine[0]));
	    break;
	  }
	  restLine++;
	}
    }

    else if (*restLine == '!') {
      if (restLine[1] == 'T') {
	strcpy(failureString, "Tilt axis limit reached");
      } else if (restLine[1] == 'P') {
	strcpy(failureString, "Pan axis limit reached");
      } else {
	strcpy(failureString, &(restLine[2]));
      }

      fprintf(stderr, "%s:%d:%s() - %s\n",
	      __FILE__, __LINE__, __FUNCTION__, failureString);

      restLine += 2;
    }

    else if (*restLine == '*') {
      if (!waitForCommand) statusOK = 1;
      restLine++;
    }

    else {
      restLine++;
    }
  }

#else

  if (waitForCommand == NULL ||
      strncmp(waitForCommand, line, strlen(waitForCommand)) == 0) {
    restLine = strpbrk(line, "*!");
    if (!restLine) {
      /* Handle error in parsing */
      exit (-1);
    } else {
      statusOK = (restLine[0] == '*');

      if (statusOK) {
	i = 1;
	while (restLine[i] != '\0') {
	  if (isdigit(restLine[i]) ||
	      restLine[i] == '-' ||
	      restLine[i] == '.') {
	    responseValue = atof(&(restLine[i]));
	    break;
	  }
	  i++;
	}
      } else if (restLine[1] == 'T') {
	strcpy(failureString, "Tilt axis limit reached");
      } else if (restLine[1] == 'P') {
	strcpy(failureString, "Pan axis limit reached");
      } else {
	strcpy(failureString, &(restLine[2]));
      }
    }
  }
#endif

}

static void initPanTilt(void)
{
#ifndef UNIBONN
  writeCommand("R", NO_ARG);
  sleep(30);
#endif
  writeCommand("I", NO_ARG);
  writeCommand("I", NO_ARG);
  writeCommand("EE", NO_ARG);	/* Enable echo */
  writeCommand("FT", NO_ARG);	/* Feedback Terse */
  writeCommand("LE", NO_ARG);	/* Enable position limits */
  if (panTiltQuery("PX", NO_ARG)) maxPanPos = (int)responseValue;
  if (panTiltQuery("PN", NO_ARG)) minPanPos = (int)responseValue;
  if (panTiltQuery("PU", NO_ARG)) maxPanVel = (int)responseValue;
  if (panTiltQuery("PL", NO_ARG)) minPanVel = (int)responseValue;
  if (panTiltQuery("PR", NO_ARG)) panDegreesPerCount = responseValue/3600.0;
  
  if (panTiltQuery("TX", NO_ARG)) maxTiltPos = (int)responseValue;
  if (panTiltQuery("TN", NO_ARG)) minTiltPos = (int)responseValue;
  if (panTiltQuery("TU", NO_ARG)) maxTiltVel = (int)responseValue;
  if (panTiltQuery("TL", NO_ARG)) minTiltVel = (int)responseValue;
  if (panTiltQuery("TR", NO_ARG)) tiltDegreesPerCount = responseValue/3600.0;
  /*  
  panTiltQuery("PL", minPanVel);
  panTiltQuery("PB", minPanVel);

  panTiltQuery("TL", minTiltVel);
  panTiltQuery("TB", minTiltVel);
  */
  /* Move in "regular" power mode. */
  writeCommand("PMR", NO_ARG);
  writeCommand("TMR", NO_ARG);
  
  /* Set low hold power mode to save power. */
  writeCommand("PHL", NO_ARG);
  writeCommand("THL", NO_ARG);
}

/*****************************************************************************
 *
 * FUNCTION: BOOLEAN PANTILT_init(void);
 *
 * DESCRIPTION: Open the panTilt device and initialize it.
 *
 * INPUTS:
 * PANTILT_PTR init - structure describing panTilt device.
 *
 * OUTPUTS: Returns true if device opened successfully, false otherwise.
 *
 * HISTORY:
 *
 *****************************************************************************/
BOOLEAN PANTILT_init(void)
{
  devInit();

  if (panTilt_device.dev.use_simulator) {
    /* set up to use the simulator */
    panTilt_device.dev.fd = connectSimulator(&panTilt_device.dev);
  } else {
    /* code to open the real device here. */
    connectTotty(&panTilt_device.dev);
  }
  if( panTilt_device.dev.fd == -1)
    return FALSE;
  else {
    m_setparms(panTilt_device.dev.fd, "x", "NONE", "8", 0, 0);
    connectDev(&panTilt_device.dev);
    initPanTilt();
    return TRUE;
  }
}

/*****************************************************************************
 *
 * FUNCTION: void PANTILT_outputHnd(int fd, long chars_available)
 *
 * DESCRIPTION: Handles output from the pan tilt device.
 *
 * INPUTS:
 * PANTILT_PTR init - structure describing panTilt device.
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/
void PANTILT_outputHnd(int fd, long chars_available)
{
  static LINE_BUFFER_PTR lineBuffer = NULL;
  if (!lineBuffer) lineBuffer = createLineBuffer(80, '\n', panTiltParse);
  processOutput(&(panTilt_device.dev), lineBuffer, chars_available);
}

/*****************************************************************************
 *
 * FUNCTION: BOOLEAN PANTILT_terminate(void);
 *
 * DESCRIPTION: Close the panTilt device.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

void PANTILT_terminate(void)
{
  close(panTilt_device.dev.fd);
  panTilt_device.dev.fd = -1;
}

/* Set the pan and tilt velocity, in degrees per second */
BOOLEAN PANTILT_setVelocity (float panVelocity, float tiltVelocity)
{
  int panVel, tiltVel;
  
  panVel = (int)(0.5 + panVelocity/panDegreesPerCount);
  tiltVel = (int)(0.5 + tiltVelocity/tiltDegreesPerCount);
  if (panVel < minPanVel || panVel > maxPanVel ||
      panVel < minPanVel || panVel > maxPanVel) {
    return FALSE;
  } else {
    return (panTiltQuery("PS", panVel) && panTiltQuery("TS", tiltVel));
  }
}

/* Set the pan and tilt acceleration */
BOOLEAN PANTILT_setAcceleration (float panAcceleration, float tiltAcceleration)
{
  int panVel, tiltVel;
  static int lastPanVel = 0, lastTiltVel= 0;
  static BOOLEAN last_ret = 0;
  panVel = (int) panAcceleration;
  tiltVel = (int) tiltAcceleration;
  if (panVel != lastPanVel || lastTiltVel != tiltVel){
    lastTiltVel = tiltVel;
    lastPanVel = panVel;
    last_ret = (panTiltQuery("PA", panVel) && panTiltQuery("TA", tiltVel));
  }
  return last_ret;
}

/* Return the position of the pan and tilt axes, in degrees */
BOOLEAN PANTILT_position (DEGREES *panAngle, DEGREES *tiltAngle)
{
  if (!panTiltQuery("PP", NO_ARG)) return FALSE;
  *panAngle = responseValue * panDegreesPerCount;
  *panAngle += global_pan_correction;
  
  if (!panTiltQuery("TP", NO_ARG)) return FALSE;
  *tiltAngle = responseValue * tiltDegreesPerCount;
  
  return TRUE;
}

/* Move the pan axis to the given (absolute) angle, in degrees */
BOOLEAN PANTILT_pan (DEGREES panAngle)
{
  int panPos;
  
  panAngle -= global_pan_correction;
  panAngle = _180_to_180(panAngle);

  panPos = (int)(panAngle/panDegreesPerCount);
  if (panPos < minPanPos || panPos > maxPanPos) {
    return FALSE;
  } else {
    writeCommand("I", NO_ARG);
#ifdef UNIBONN
    return (panTiltQuery("PP", panPos) && panTiltQuery("A", NO_ARG));
#else
    return (panTiltQuery("PP", panPos));
#endif
  }
}

/* Move the pan axis relative to its current angle, in degrees */
BOOLEAN PANTILT_panRelative (DEGREES panAngle)
{
  int panPosRel, panPos;
  
  panPosRel = (int)(panAngle/panDegreesPerCount);
  if (!panTiltQuery("PP", NO_ARG)) return FALSE;
  panPos = responseValue + panPosRel;
  if (panPos < minPanPos || panPos > maxPanPos) {
    return FALSE;
  } else {
    writeCommand("I", NO_ARG);
    return (panTiltQuery("PO", panPosRel) && panTiltQuery("A", NO_ARG));
  }
}

/* Move the tilt axis to the given (absolute) angle, in degrees */
BOOLEAN PANTILT_tilt (DEGREES tiltAngle)
{
  int tiltPos;
  
  tiltAngle = _180_to_180(tiltAngle);

  tiltPos = (int)(tiltAngle/tiltDegreesPerCount);
  if (tiltPos < minTiltPos || tiltPos > maxTiltPos) {
    return FALSE;
  } else {
    writeCommand("I", NO_ARG);
#ifdef UNIBONN
    return (panTiltQuery("TP", tiltPos) && panTiltQuery("A", NO_ARG));
#else
    return (panTiltQuery("TP", tiltPos));
#endif
  }
}

/* Move the tilt axis relative to its current angle, in degrees */
BOOLEAN PANTILT_tiltRelative (DEGREES tiltAngle)
{
  int tiltPosRel, tiltPos;
  
  tiltPosRel = (int)(tiltAngle/tiltDegreesPerCount);
  if (!panTiltQuery("TP", NO_ARG)) return FALSE;
  tiltPos = responseValue + tiltPosRel;
  if (tiltPos < minTiltPos || tiltPos > maxTiltPos) {
    return FALSE;
  } else {
    writeCommand("I", NO_ARG);
    return (panTiltQuery("TO", tiltPosRel) && panTiltQuery("A", NO_ARG));
  }
}

/* Move the pan and tilt axes simultaneously to the 
   given (absolute) angle, in degrees */
BOOLEAN PANTILT_move (DEGREES panAngle, DEGREES tiltAngle)
{
  int panPos, tiltPos;
  
  panAngle -= global_pan_correction;
  panAngle = _180_to_180(panAngle);

  tiltAngle = _180_to_180(tiltAngle);

  panPos = (int)(panAngle/panDegreesPerCount);
  tiltPos = (int)(tiltAngle/tiltDegreesPerCount);
  if (panPos < minPanPos || panPos > maxPanPos ||
      tiltPos < minTiltPos || tiltPos > maxTiltPos) {
    return FALSE;
  } else {
#ifdef UNIBONN
    writeCommand("S", NO_ARG);
    return (panTiltQuery("PP", panPos) && panTiltQuery("TP", tiltPos)
	    && panTiltQuery("A", NO_ARG));
#else
    writeCommand("I", NO_ARG);
    return (panTiltQuery("PP", panPos) && panTiltQuery("TP", tiltPos));
#endif
  }
}

/* Move the pan and tilt axes simultaneously to positions relative to
   the current positions, in degrees */
BOOLEAN PANTILT_moveRelative (DEGREES panAngle, DEGREES tiltAngle)
{
  int panPosRel, panPos, tiltPosRel, tiltPos;
  
  panPosRel = (int)(panAngle/panDegreesPerCount);
  if (!panTiltQuery("PP", NO_ARG)) return FALSE;
  panPos = responseValue + panPosRel;
  tiltPosRel = (int)(tiltAngle/tiltDegreesPerCount);
  if (!panTiltQuery("TP", NO_ARG)) return FALSE;
  tiltPos = responseValue + tiltPosRel;
  
  if (panPos < minPanPos || panPos > maxPanPos ||
      tiltPos < minTiltPos || tiltPos > maxTiltPos) {
    return FALSE;
  } else {
    writeCommand("S", NO_ARG);
    return (panTiltQuery("PO", panPosRel) && panTiltQuery("TO", tiltPosRel) &&
	    panTiltQuery("A", NO_ARG));
  }
}

/* Reset the pan and tilt axes, and restore the stored settings */
BOOLEAN PANTILT_reset(void)
{
  fprintf(stderr, "Resetting Pan/Tilt head: Please wait\n");
  writeCommand("DR", NO_ARG);	/* Restore stored defaults */
  return panTiltQuery("R", NO_ARG);
}

/* 
 * Return the limits of the pan and tilt axes (min/max angles, max velocity)
 */
void PANTILT_limits (DEGREES *minPanAngle, DEGREES *maxPanAngle,
		     DEGREES *minTiltAngle, DEGREES *maxTiltAngle,
		     float *maxPanVelocity, float *maxTiltVelocity)
{
  *minPanAngle = minPanPos*panDegreesPerCount + global_pan_correction;
  *maxPanAngle = maxPanPos*panDegreesPerCount + global_pan_correction;
  *minTiltAngle = minTiltPos*tiltDegreesPerCount;
  *maxTiltAngle = maxTiltPos*tiltDegreesPerCount;
  *maxPanVelocity = maxPanVel*panDegreesPerCount;
  *maxTiltVelocity = maxTiltVel*tiltDegreesPerCount;
}
