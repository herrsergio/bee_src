
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
#include <X11/Intrinsic.h>

#ifdef __cplusplus
extern "C" {
#endif

struct robot_struct{
    Boolean placed;
    float map_robot_x,     		/*** robot position ***/ 
    map_robot_y,
    map_old_robot_x,   /*** to delete robot from screen when trace is off ***/ 
    map_old_robot_y,
    map_robot_start_x, /*** start position ***/
    map_robot_start_y;
    float robot_deg;              	/*** direction in degree ***/
    float old_robot_deg;
    float robot_start_deg;		/*** start direction ***/
    float	RobotRadius;
    int	changePos;			/*** true, when robot position was changed manually ***/
    int     runflag;                  	/*** is robot running ? ***/
  float base_BRH;          /* Base Relative Heading. To support indexing */
};

extern struct robot_struct robot;  

#define MOTION_UPDATE_INTERVAL 100 /* update robots pos every N millisecs */  


void InitRobot();
void MoveRobot();            /* move robot and draw it if neccessary */
void DrawRobot();            /* force drawing of robot */
void BeamRobot(float, float);
void BeamRobot3(float, float, float);
void DrawSonarBeams(float*);
void robot_setTarget(float, float);
void setRobotPosition(float, float, float);
void getRobotPosition(float*, float*, float*);

#ifdef __cplusplus
}
#endif





