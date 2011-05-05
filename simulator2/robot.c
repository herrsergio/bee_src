
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
#include <bUtils.h>
#include "simxw.h"
#include "playground.hh"
#include "robot.h"
#include "base.h"
#include "sonar.h"
#include "trigofkt.h"

extern Boolean no_tcx;

struct robot_struct robot;  
extern float sonar_dists[];

void robot_HandleMove()
{
    HandleMove();
}

void DrawSonarBeams(float *dists)
{
  
    if(!sonarOn) return;
    else {
	int i;
	float f;
	float cos_f; 
	float sin_f;
	float r = robot.RobotRadius;
	float xr = playground_x(robot.map_robot_x);
	float yr = playground_y(robot.map_robot_y);
	float dist;
	f = robot.robot_deg - sonar_offset;
	f *= M_PI/180;
	automatic_redraw(FALSE);
	for(i = 0; i < bRobot.sonar_cols[0]; i++) {
	    cos_f = myCOS(f);
	    sin_f = mySIN(f);
	    dist = dists[i]+r;
	    if(dist >= sonar_infinity) expose_beam(i,xr,yr,xr,yr);
	    else {
		expose_beam(i, xr+r*cos_f, yr+r*sin_f,
			    xr+dist*cos_f, yr+dist*sin_f);

	    }
	    f -= 2*M_PI/bRobot.sonar_cols[0];
	    while(f > 2*M_PI) f -= 2*M_PI;
	    while(f < 0) f += 2*M_PI; 	    
	}
	automatic_redraw(TRUE);
    }
}

static float playground_old_x;
static float playground_old_y;
void DrawRobot()
{
  Boolean alarm;
  if(traceOn)  {
    drawTraceLine(playground_old_x,
		  playground_old_y,
		  playground_x(robot.map_robot_x),
		  playground_y(robot.map_robot_y));
    playground_old_x = playground_x(robot.map_robot_x);
    playground_old_y = playground_y(robot.map_robot_y);
  }
  alarm = ( obstacles_min_distance(playground_x(robot.map_robot_x),
				  playground_y(robot.map_robot_y))
	   <= robot.RobotRadius); 
  DrawRobotFigure(alarm,
		  playground_x(robot.map_robot_x),
		  playground_y(robot.map_robot_y),
		  robot.RobotRadius, 360-robot.robot_deg);
}

void MoveRobot()
{
  DrawSonarBeams(sonar_dists);
  DrawLasers();
  if(robot.map_robot_x == robot.map_old_robot_x &&  
     robot.map_robot_y == robot.map_old_robot_y &&
     robot.robot_deg == robot.old_robot_deg)
    return;  
  robot.map_old_robot_y = robot.map_robot_y;
  robot.map_old_robot_x = robot.map_robot_x;
  robot.old_robot_deg = robot.robot_deg;
  DrawRobot();
}

void BeamRobot(float x, float y)
{
  robot.map_robot_x = base_x(x);
  robot.map_robot_y = base_y(y);
  robot.changePos = 1;
  robot.placed = TRUE;
  MoveRobot();  
}

extern float base_init_rot;
extern float base_correct_x;
extern float base_correct_y;

void BeamRobot3(float x, float y, float rot)
{
  float oldx = base_coord_x();
  float oldy = base_coord_y();  
  robot.map_robot_start_x += base_x(x) - robot.map_robot_x;
  robot.map_robot_start_y += base_y(y) - robot.map_robot_y;
  robot.map_robot_x = base_x(x);
  robot.map_robot_y = base_y(y);
  robot.robot_start_deg += 180.0 / M_PI * rot - robot.robot_deg;  
  robot.robot_deg = 180.0 / M_PI * rot;
  base_init_rot = (ROBOT_INIT_ROT - robot.robot_start_deg);
  BaseVariables.RotateWhere = robot.robot_deg;
  base_correct_x += oldx - base_coord_x();
  base_correct_y += oldy - base_coord_y();
  robot.changePos = 1;
  robot.placed = TRUE;
  MoveRobot();  
}

void InitRobot()
{
  robot.placed = FALSE;
  robot.runflag = 1;                  	/*** is robot running ? ***/
  robot.map_robot_start_x = robot.map_robot_x;
  robot.map_robot_start_y = robot.map_robot_y;
  robot.robot_start_deg = robot.robot_deg;
  robot.map_old_robot_x = robot.map_robot_x;
  robot.map_old_robot_y = robot.map_robot_y;    

  robot.RobotRadius = bRobot.base_radius;               /*** cm ***/
  robot.placed = TRUE;
  robot.changePos = 1;                	/*** true, when robot position was changed manually ***/
  robot.base_BRH = 0.0;
  InitRobotFigure(playground_x(robot.map_robot_x),
		  playground_y(robot.map_robot_y),
		  robot.RobotRadius,robot.robot_deg);
}

void setRobotPosition(float x, float y, float a)
{
  robot.map_old_robot_x = robot.map_robot_x;
  robot.map_old_robot_y = robot.map_robot_y;  
  robot.map_robot_x = x;
  robot.map_robot_y = y;
  robot.robot_deg = a;
}

void getRobotPosition(float *x, float *y, float *a)
{
    *x = robot.map_robot_x;
    *y = robot.map_robot_y;
    *a = robot.robot_deg;
}

void robot_setTarget(float x, float y)
{
  if(!no_tcx)  
      base_sendTarget(base_x(x), base_y(y));
}
