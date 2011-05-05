
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




#ifndef COLLISION_LOADED
#define COLLISION_LOADED

#include "bUtils.h"
#include "COLLI-messages.h"

#define F_ERROR MAXFLOAT
#define I_ERROR (MAXINT)

#define ROB_RADIUS (bRobot.base_radius) 

#define COLLI_SONAR_RADIUS 2.48

#define SONAR_OPEN_ANGLE (DEG_TO_RAD(7.5))

#define MAX_TRANS_SPEED 90.0
#define MAX_ROT_SPEED (DEG_TO_RAD(60.0))
#define NO_OF_SONARS bRobot.sonar_cols[0]

#define DEG_90 (1.5707963705)
#define DEG_180 (3.1415927410)
#define DEG_270 (4.7123891115)
#define DEG_360 (6.2831854820)


#define SGN(x)    ((x) >= 0.0 ? (1) : (-1))

/**********************************************************************/

typedef struct Point {
  float x;
  float y;
} Point;

typedef struct VelocityCombination {
  float tvel;
  float rvel;
} VelocityCombination;

typedef struct Circle {
  float rad;
  Point M;
} Circle;

typedef struct LineSeg {
  Point pt1;
  Point pt2;
} LineSeg;

typedef struct ObstacleLines {
  int no_of_lines;
  LineSeg *lines;
} ObstacleLines;

typedef struct ObstaclePoints {
  int no_of_points;
  Point *points;
} ObstaclePoints;


typedef struct LaserPoints {
  int maxNumberOfPoints;
  int numberOfPoints;
  float maxRememberTime;
  Point* points;
  struct timeval* times;
} LaserPoints;


typedef struct mode_structure {
    float velocity_factor;
    float angle_factor;
    float distance_factor;
    
    int number_of_rvels;
    int number_of_tvels;
    
    float target_trans_acceleration;
    float target_rot_acceleration;
    float target_max_rot_speed;
    float target_max_trans_speed;

    float exception_trans_acceleration;
    float exception_trans_velocity;
    float exception_rot_acceleration;
    float exception_rot_velocity;

    float min_dist;
    int smooth_width;
    float security_dist;
    float min_dist_for_target_way_free;
    float max_collision_line_length;
    float max_range;

    float edge_portion;
    float max_security_speed;
    float min_security_speed;
    float max_security_dist;

    /* Parameter for exception handling with arm outisde. */
    
    float security_angle;
    int   rotate_away_persistence;
 } mode_structure;


void COLLI_DumpInfo();
void COLLI_MarkInfo();
void COLLI_init_Lines(void);
void COLLI_start_collision_avoidance();
void COLLI_update_tcx_status(LineSeg, LineSeg, Circle, BOOLEAN, Point, float, int*, int*);
void COLLI_SetApproachDist( double dist);
void COLLI_StartExploration();
void COLLI_PickupObject( double forward, double side);
void COLLI_TranslateRelative(double xdir, double ydir);
void COLLI_TranslateRelative_TwoPoints(double x1, double y1, double x2, double y2);
void COLLI_GotoAbsolute(float x, float y, BOOLEAN new_target );
void COLLI_ApproachAbsolute(float, float, float, BOOLEAN, int );
void COLLI_ApproachAbsolute_TwoPoints(float, float, float,
				      float, float, BOOLEAN, TCX_REF_PTR);
void COLLI_get_parameters(COLLI_parameter_ptr);
void COLLI_set_wall_orientation(float orient);
void COLLI_GoForward();
void COLLI_GoBackward();
void COLLI_BumpHandler( Pointer callback_data, Pointer client_data);

/* These two functions start to write the evaluation function into
 * files. */
void COLLI_StartToDumpGnuPlot( double step);
void COLLI_StartToDumpMathematica( double step);

#ifdef UNIBONN

void
COLLI_get_CollPoints_from_vision(COLLI_vision_point_ptr);

void
COLLI_get_CollLines_from_vision(COLLI_vision_line_ptr vision_lines);

#endif

/* For communication with the arm. These functions send commands to the arm. */

#ifdef CLEANUP
void ARM_moveToGround(void);
void ARM_liftObject(void);
void ARM_dropObject(void);
void ARM_moveIn(void);

/* These functions get information about the successfull executions of commands. */

void ARM_moveToGroundReady(int success);
void ARM_liftObjectReady(int success);
void ARM_dropObjectReady(int success);
void ARM_moveInReady(int success);


/* Just for testing. The next functions simulate the vision to send objects. */
void
COLLI_SimulateCloseObject( double forward, double side);

void
COLLI_SimulateFarObject( double forward, double side);

void
COLLI_SimulateCloseTrashBin( double forward, double side);

void
COLLI_SimulateFarTrashBin( double forward, double side);

#endif

/*****************************************************************************
 * External variables
 *****************************************************************************/

extern BOOLEAN use_sonar;
extern BOOLEAN use_laser;
extern BOOLEAN use_laser_server;
extern BOOLEAN msp_sonar;

extern BOOLEAN use_arm;
extern BOOLEAN use_bumper;
extern BOOLEAN use_ir;
extern BOOLEAN use_vision;
extern BOOLEAN use_collision;


extern BOOLEAN colli_go_backward;

/* This is used to write information relevant for collision
 * avoidance into a dump file.
 */
extern BOOLEAN dumpInfo;
extern FILE* dumpFile;

extern int mode_number;
extern int REMEMBER_INTERVAL;
extern float SECURITY_DIST;
extern float EDGE_PORTION;
extern LineSeg **CollLines;
extern ObstaclePoints External_Obstacle_Points;
extern ObstaclePoints Bumper_Obstacle_Points;
extern ObstaclePoints Ir_Obstacle_Points;
extern LaserPoints Laser_CollPoints;
extern int next_CollLine_reading;
extern float *histogram;
extern BOOLEAN target_flag;
extern BOOLEAN rot_wanted;
extern Point target;
extern mode_structure *ACTUAL_MODE;
extern mode_structure **mode_structure_array;

#endif /* COLLISION_LOADED */



