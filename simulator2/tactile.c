
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


#include <tactileClient.h>

#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <bUtils.h>

#include "sundefines.h"
#include "playground.hh"
#include "robot.h"
#include "tactile.h"
#include "trigofkt.h"
#include "surface.hh"

#define TCX_define_variables
#include "SIMULATOR-messages.h"

float base_coord_rot();

TCX_MODULE_PTR MODULE_TACTILE = NULL;

int tactile_vals[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];
int tactile_lastVals[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];

static struct TactileVar {
  int tactileRunning;
} TactileVariables;

#define TACTILE_DEBUG


/**********************************************************************
 *
 * MSP type TACTILE support - tds
 *
 **********************************************************************/

/*------------------------------------------------------------*/

void
mspTactileStart(int value)
{
  extern void schedule_tactileServerReport(int delay);

#if 1
  fprintf(stderr, "%10s:%5d:%14s(): %d\n",
	  __FILE__, __LINE__, __FUNCTION__, value);
#endif

  TactileVariables.tactileRunning = value;
  if (value) {
    schedule_tactileServerReport(100);
  }
#if 1
  fprintf(stderr, "%10s:%5d:%14s() - return\n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  return;
}

/*------------------------------------------------------------*/

void
tactileServer(char *message)
{
  /*
   * XXX - This is a total gross hack right now. -tds
   */

  mspTactileStart(message[0]);
}

/*------------------------------------------------------------*/

void
tactileReset(void)
{
  int row, col;

#if 0
  fprintf(stderr, "%10s:%5d:%14s()\n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  for (row = 0; row < bRobot.tactile_rows; row++) {
    for (col = 0; col < bRobot.tactile_cols[row]; col++) {
      tactile_vals[row][col] = 0;
    }
  }

#if 0
  fprintf(stderr, "%10s:%5d:%14s() - return\n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  return;
}

/*------------------------------------------------------------*/

void
tactileInit()
{

#if 1
  fprintf(stderr, "%10s:%5d:%14s()\n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  tactileReset();
  mspTactileStart(1);

#if 1
  fprintf(stderr, "%10s:%5d:%14s() - return\n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  return;
}

/*------------------------------------------------------------*/

static void
_tactileHit(float angle)
{
  int row, col;
  float brh, heading;

#if 0
  fprintf(stderr, "%10s:%5d:%14s() - angle = %5.2f\n",
	  __FILE__, __LINE__, __FUNCTION__, angle*180.0/M_PI);
#endif

  heading = (base_coord_rot() + robot.robot_start_deg) * M_PI / 180.0;
  brh     = robot.base_BRH * M_PI / 180.0;

  angle = angle + brh - M_PI_2;

  while(angle>M_PI) {
    angle -= 2*M_PI;
  }
    
  while(angle<-M_PI) {
    angle += 2*M_PI;
  }
    
  if (!strcmp(bRobot.base_type, "B21")) {
    row = 2;
    col = (int)((M_PI - angle)*
		(float)bRobot.tactile_cols[row]/(2.0*M_PI));
    tactile_vals[row][col] = 1;
  }
  else if (!strcmp(bRobot.base_type, "B14")) {
    row = 2;
    col = (int)((M_PI - angle)*
		(float)bRobot.tactile_cols[row]/(2.0*M_PI));
    tactile_vals[row][col] = 1;
  }
  else {
    fprintf(stderr, "%s:%6d:%s() - ******** unknown base type [%s]\n",
	    __FILE__, __LINE__, __FUNCTION__, bRobot.base_type);
  }
  return;
}

/*------------------------------------------------------------*/

#define TACTILE_NUM_ANGLES 200

void
tactileHit(float x, float y)
{
  int angle;
  float alpha;
  float dist;

#if 0
  fprintf(stderr, "%10s:%5d:%14s()\n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  tactileReset();
  
  for (angle = 0; angle < TACTILE_NUM_ANGLES; angle++) {
    alpha = (float)angle * 2.0 * M_PI / TACTILE_NUM_ANGLES;

    while(alpha>M_PI) {
      alpha -= 2*M_PI;
    }
    
    while(alpha<-M_PI) {
      alpha += 2*M_PI;
    }
    
    if (get_distance(EXACT_SENSOR, x, y,
		     0.0, M_PI-0.00001,
		     x+bRobot.base_radius*cos(alpha)*1.05,
		     y+bRobot.base_radius*sin(alpha)*1.05, &dist)) {
      
      _tactileHit(alpha);
    }
  }
  
  return;
}

/*------------------------------------------------------------*/

/*
 * Get tactile data and send tactile messages
 */

int
tactileServerReport (void)
{
  int row;
  int col;
  int value_index= 0;
  int bufsize;
  long values[B_MAX_SENSOR_ROWS * B_MAX_SENSOR_COLS * 3];
  SIMULATOR_message_to_tactileServer_type message;

  int tactileNum, tactile;
  unsigned long value;
  struct timeval time;
  float angle;
  int changed = 0;

#if 0
  fprintf(stderr, "%10s:%5d:%14s()\n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif
#if 0
  if (!TactileVariables.tactileRunning) {
    return(0);
  }
#endif

  gettimeofday(&time, NULL);    /* get time first */

  /*
   * we pack the index and tactile value up for each new reading
   * and terminate it by a -1 tactile index which client code checks for 
   */

  for(row=0; row<bRobot.tactile_rows; row++) {
    for(col=0; col<bRobot.tactile_cols[row]; col++) {
      values[value_index++] = row;
      values[value_index++] = col;
      values[value_index++] = tactile_vals[row][col];
      
      if (tactile_vals[row][col] != tactile_lastVals[row][col]) {
	changed = 1;
	tactile_lastVals[row][col] = tactile_vals[row][col];
      }
    }
  }

  values[value_index++] = -1;

  message.values = values;
  message.count  = value_index;

  /*
   * Do TCX message
   */

  if (changed) {

#if 0
    fprintf(stderr, "%s:%6d:%s() - \n", __FILE__, __LINE__, __FUNCTION__);
    
    for(row=0; row<bRobot.tactile_rows; row++) {
      for(col=0; col<bRobot.tactile_cols[row]; col++) {
	fprintf(stderr, "[%3d] ",
		(int)tactile_vals[row][col]);
      }
      fprintf(stderr, "\n");
    }
#endif

    MODULE_TACTILE = MODULE_SONAR;
    
    if (MODULE_TACTILE) {
      tcxSendMsg(MODULE_TACTILE, "SIMULATOR_message_to_tactileServer",
		 &message);
    }
  }

  return(1);
}
