
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


#include <irClient.h>

#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <bUtils.h>

#include "sundefines.h"
#include "playground.hh"
#include "robot.h"
#include "ir.h"
#include "trigofkt.h"
#include "surface.hh"

#define TCX_define_variables
#include "SIMULATOR-messages.h"

float base_coord_rot();

TCX_MODULE_PTR MODULE_IR = NULL;

float ir_dists[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];

static struct IrVar {
  int irRunning;
  irType irs[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];
} IrVariables;

#define IR_DEBUG


/**********************************************************************
 *
 * MSP type IR support - tds
 *
 **********************************************************************/

/*------------------------------------------------------------*/

/*
 * NOTE!
 * 
 * There are assumptions here about the sensor geometry and
 * the robot type.
 *
 */

static float
simIrWorldAngle(int irRow, int irCol)
{
  float angle = 0.0;
  float heading, brh;

  if (irRow >= bRobot.ir_rows) {
    return(0.0);
  }

  if (irCol >= bRobot.ir_cols[irRow]) {
    return(0.0);
  }

  heading = (base_coord_rot() + robot.robot_start_deg) * M_PI / 180.0;
  brh     = robot.base_BRH * M_PI / 180.0;

  switch (irRow) {
  case 0:
    angle = M_PI + M_PI_2 - heading -
      ((float)irCol+.5) * (2.0 * M_PI/(float)bRobot.ir_cols[irRow]);
    break;

  case 1:
    angle = M_PI + M_PI_2 - brh -
      ((float)irCol+0.7) * (2.0 * M_PI /(float)bRobot.ir_cols[irRow]);
    break;

  case 2:
    angle = M_PI + M_PI_2 - brh -
      ((float)irCol+0.05) * (2.0 * M_PI /(float)bRobot.ir_cols[irRow]);
    break;
  }

  while (angle>M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle<-M_PI) {
    angle += 2.0 * M_PI;
  }

#if 0
  {
    static int count = 0;

    if (--count<0) {
      count=64;
      fprintf(stderr,
	      "%s:%6d:%s() - row=%3d col=%3d "
	      "head1=%f head=%f brh=%f angle=%f\n",
	      __FILE__, __LINE__, __FUNCTION__, irRow, irCol,
	      base_coord_rot(),
	      heading*180.0/M_PI, brh*180.0/M_PI, angle*180.0/M_PI);
    }
  }
#endif

  return(angle);
}

/*------------------------------------------------------------*/

void
mspIrStart(int value)
{
  extern void schedule_irServerReport(int delay);

#ifdef IR_DEBUG
  fprintf(stderr, "%10s:%5d:%14s(): %d\n",
	  __FILE__, __LINE__, __FUNCTION__, value);
#endif

  IrVariables.irRunning = value;
  if (value) {
    schedule_irServerReport(100);
  }
}

/*------------------------------------------------------------*/

void
irServer(char *message)
{
  /*
   * XXX - This is a total gross hack right now. -tds
   */

  mspIrStart(message[0]);
}

/*------------------------------------------------------------*/

/*
 * Get ir data and send ir messages
 */

int
irServerReport (void)
{
  int row;
  int col;
  int value_index= 0;
  int bufsize;
  long values[B_MAX_SENSOR_ROWS * B_MAX_SENSOR_COLS * 3];
  SIMULATOR_message_to_irServer_type message;

  int irNum, ir;
  unsigned long value;
  struct timeval time;
  float dist;

  static float GetDistance(int row, int col);

#if 0
  fprintf(stderr, "%10s:%5d:%14s()\n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  if (!IrVariables.irRunning) {
    return(0);
  }

  gettimeofday(&time, NULL);    /* get time first */

  /* make previous data old */
  for(row=0; row<bRobot.ir_rows; row++) {
    for(col=0; col<bRobot.ir_cols[row]; col++) {
      IrVariables.irs[row][col].mostRecent = FALSE;
    }
  }

  /* we pack the index and sonar value up for each new reading */
  /* and terminate it by a sonar index out of range, which client */
  /* code checks for */

  {
    static int count = 0;

    count = 10;

    if (!count) {
      fprintf(stderr, "%s:%6d:%s() - \n", __FILE__, __LINE__, __FUNCTION__);
    }

    for(row=0; row<bRobot.ir_rows; row++) {
      for(col=0; col<bRobot.ir_cols[row]; col++) {
	values[value_index++] = row;
	values[value_index++] = col;
	values[value_index++] = IrVariables.irs[row][col].value = 
	  GetDistance(row, col);
	IrVariables.irs[row][col].mostRecent = 1;
	if (!count) {
	  fprintf(stderr, "[%3d] ",
		  (int)IrVariables.irs[row][col].value);
	}
      }
      if (!count) {
	fprintf(stderr, "\n");
      }
    }

    if (--count < 0) count = 20;
  }

  values[value_index++] = -1;

  message.values = values;
  message.count  = value_index;

  /*
   * Do TCX message
   */

  if (MODULE_IR) {
    tcxSendMsg(MODULE_IR, "SIMULATOR_message_to_irServer",
	       &message);
  }

  return(1);
}

/*------------------------------------------------------------*/

void
InitIr()
{
  int ii, jj;

  for (jj=0; jj<B_MAX_SENSOR_ROWS; jj++) {
    for (ii=0; ii< B_MAX_SENSOR_COLS; ii++) {
      ir_dists[jj][ii] = 0.0;
    }
  }

  return;
}

/*------------------------------------------------------------*/

static float
GetDistance(int row, int col)
{
  float tdist, dist;
  float PosX, PosY, EndX, EndY;
  float rx, ry, rori;
  float worldAngle;
  float open_angle;
  float irZ;

  getRobotPosition(&rx,&ry,&rori);

  worldAngle = simIrWorldAngle(row, col);

  PosX = rx + bRobot.base_radius * cos(worldAngle);
  PosY = ry + bRobot.base_radius * sin(worldAngle);
  EndX = PosX + 50.0 * cos(worldAngle);
  EndY = PosY + 50.0 * sin(worldAngle);

  /*
   * open_angle determines the z-range which is hit depending on
   * the obstacles distance
   */

  open_angle = 5.0 * M_PI/180.0;
  irZ = 20.0;

  if (row > 1) {
    ir_dists[row][col] = dist = 30.0;
  }
  else if(get_distance(IR_SENSOR,
		       PosX, PosY, irZ,
		       open_angle,
		       EndX, EndY, &dist)) {
    ir_dists[row][col] = dist;
  }
  else {
    dist = ir_dists[row][col] = 0;
  }

  return(dist);
} /* GetDistance() */
