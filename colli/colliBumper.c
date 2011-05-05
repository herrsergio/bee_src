
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




#include "collisionIntern.h"

ObstaclePoints Bumper_Obstacle_Points;

#define BUMPER_REMEMBER_TIME 10.0
static float bumperTimeOut;


static void
update_BumperPoints( tactileInfo* tactiles);


void
updateBumperTimer()
{
  if ( Bumper_Obstacle_Points.no_of_points > 0)
    if ( timeExpired( BUMPER_TIMER) > BUMPER_REMEMBER_TIME) {
      Bumper_Obstacle_Points.no_of_points = 0;
      fprintf(stderr, "Bumper too old (remove it).\n");
      COLLI_update_tcx_BumperPoints();
    }
}


/**********************************************************************
 * Handles tacitle callback.
 **********************************************************************/
void
COLLI_BumpHandler( Pointer callback_data, Pointer client_data)
{
  int angleCnt = 0;
  int tmpTarget = target_flag;
  tactileInfo* tactiles = (tactileInfo*) callback_data;

  if ( (Bumper_Obstacle_Points.no_of_points +
	NUMBER_OF_POINTS_PER_BUMPER * tactiles->numberOfActivatedTactiles)
       > MAX_NUMBER_OF_BUMPERS) {
    fprintf(stderr, "Too many bumpers (%d, %d). Resize MAX_NUMBER_OF_BUMPERS.\n",
	    tactiles->numberOfActivatedTactiles, MAX_NUMBER_OF_BUMPERS);
    return;
  }
  
  if ( tactiles->numberOfActivatedTactiles < 1) {
    if ( 0) fprintf( stderr, "Bumper released.\n");
    return;
  }
  
  update_BumperPoints( tactiles);
  setStartTime( BUMPER_TIMER);

  BASE_TranslateCollisionAcceleration( ACTUAL_MODE->exception_trans_acceleration);
  BASE_RotateCollisionAcceleration( RAD_TO_DEG( ACTUAL_MODE->exception_rot_acceleration));

  BASE_TranslateHalt();
  BASE_RotateHalt();

  target_flag = tmpTarget;
}


/**********************************************************************
 * Updates the structure containing the points of the obstacles as detected
 * by the bumpers.
 **********************************************************************/
#define BUMPER_ANGLE (DEG_TO_RAD( 10.0))

static void
update_BumperPoints( tactileInfo* tactiles)
{
  int cnt, pointCnt; 
  int activeBumper = Bumper_Obstacle_Points.no_of_points;
  Point rpos;
  float rrot;

  float angle;
  float angleStep;

  /* We insert several points per bumper. */
  if ( NUMBER_OF_POINTS_PER_BUMPER == 1) {
    angle = 0.0;
    angleStep = 0.0;
  }
  else {
    angle     = - BUMPER_ANGLE / 2.0;
    angleStep = BUMPER_ANGLE / (float) (NUMBER_OF_POINTS_PER_BUMPER - 1);
  }
  
  updateActualPosition( &rpos, &rrot, DONT_CONSIDER_DIRECTION);

  for ( cnt = 0; cnt < tactiles->numberOfActivatedTactiles; cnt++) {
    
    for ( pointCnt = 0; pointCnt < NUMBER_OF_POINTS_PER_BUMPER; pointCnt++) {
      
      float pointAngle = rrot + tactiles->angle[cnt] + angle + pointCnt * angleStep;
      int index = activeBumper + NUMBER_OF_POINTS_PER_BUMPER * cnt + pointCnt;
      float translationTillStop;
      float distanceToObstacle;

      /* If it is a rear bumper we substract the distance till the robot
       * has stopped. Otherwise we add this distance. */
      translationTillStop = brakeDistance( rwi_base.trans_current_speed,
					   -ACTUAL_MODE->exception_trans_acceleration);
      if (tactiles->angle[cnt] > DEG_90 && tactiles->angle[cnt] < DEG_270)
	translationTillStop = - translationTillStop;
      
      distanceToObstacle = translationTillStop + ROB_RADIUS;
      
      Bumper_Obstacle_Points.points[index].x =
	rpos.x + (fcos( pointAngle) * distanceToObstacle);
      Bumper_Obstacle_Points.points[index].y =
	rpos.y + (fsin( pointAngle) * distanceToObstacle);
    }
  }

  Bumper_Obstacle_Points.no_of_points = activeBumper
    + NUMBER_OF_POINTS_PER_BUMPER * tactiles->numberOfActivatedTactiles;
  
  COLLI_update_tcx_BumperPoints();
}
