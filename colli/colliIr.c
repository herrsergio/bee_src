
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

#define IR_THRESHOLD_DIST 10.0
#define IR_REMEMBER_TIME 3.0

ObstaclePoints Ir_Obstacle_Points;

static void
update_IrPoints( irInfo* irs);

void
COLLI_IrHandler( irInfo* irs)
{
  if ( 0 && irs->numberOfActivatedIrs > 0)
    fprintf( stderr, "%d irs\n", irs->numberOfActivatedIrs);

  if ( irs->numberOfActivatedIrs > 0
       ||
       ( irs->numberOfActivatedIrs == 0
	&&
	Ir_Obstacle_Points.no_of_points > 0
	 &&
	 ( timeExpired( IR_TIMER) > IR_REMEMBER_TIME))
       )
    update_IrPoints( irs);
  
  COLLI_update_tcx_IrPoints();
}

static void
update_IrPoints( irInfo* irs)
{
  int cnt;
  Point rpos;
  float rrot;

  float distanceToObstacle = ROB_RADIUS + IR_THRESHOLD_DIST;
  
  updateActualPosition( &rpos, &rrot, DONT_CONSIDER_DIRECTION);

  for ( cnt = 0; cnt < irs->numberOfActivatedIrs; cnt++) {
    
    float angle = rrot + irs->angle[cnt];
    
    Ir_Obstacle_Points.points[cnt].x =
      rpos.x + (fcos( angle) * distanceToObstacle);
    Ir_Obstacle_Points.points[cnt].y =
      rpos.y + (fsin( angle) * distanceToObstacle);
  }

  if ( irs->numberOfActivatedIrs > 0)
    setStartTime( IR_TIMER);
  
  Ir_Obstacle_Points.no_of_points = irs->numberOfActivatedIrs;
}

