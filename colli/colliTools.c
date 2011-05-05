
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
#include "laser_interface.h"


/* These times are used to check wether the data is actual enough. */
struct timeval Last_CollLine_Update;
struct timeval Last_VisionLine_Update;
struct timeval Last_VisionPoint_Update;
struct timeval Last_FrontLaserPoint_Update;
struct timeval Last_RearLaserPoint_Update;


/* These values are used to check for time intervals. */
int timeInterval;
struct timeval beginTime;

int next_CollLine_reading = 0;
LineSeg **CollLines;

ObstacleLines External_Obstacle_Lines;
ObstaclePoints External_Obstacle_Points;

#define MAX_RECOVER_TIME 20.0

/**********************************************************************
 **********************************************************************
 *                         Init functions                            *
 **********************************************************************
 **********************************************************************/



/**********************************************************************/
void

COLLI_init_obstacle_structs(void)
{
  int i,j;
  void COLLI_reset_obstacle_structs(void);
  
  Bumper_Obstacle_Points.points = (struct Point*)
    malloc( NUMBER_OF_POINTS_PER_BUMPER * MAX_NUMBER_OF_BUMPERS * sizeof(struct Point));
  
  Ir_Obstacle_Points.points = (struct Point*)
    malloc( MAX_NUMBER_OF_IRS * sizeof(struct Point));

  COLLI_reset_obstacle_structs();
}

/**********************************************************************/
/* The following function is used by the BASE_reset_obstacle_line_field    */
/* TCX message (ds 14/01/98)
/**********************************************************************/

void
COLLI_reset_obstacle_structs(void)
{
  int i,j;
  
  next_CollLine_reading = 0;
  
  External_Obstacle_Lines.no_of_lines = 0;
  External_Obstacle_Points.no_of_points = 0;
  Bumper_Obstacle_Points.no_of_points = 0;

  Ir_Obstacle_Points.no_of_points = 0;

  for (i=0; i < bRobot.sonar_cols[0]; i++) {
    for (j=0; j<REMEMBER_INTERVAL; j++)
      CollLines[i][j].pt1.x = F_ERROR;
  }
  fprintf(stderr, "BASE: reseted obstacle line field\n");
}



/******************************************************************************
 * We set some default values and allocate memory for global variables        *
 ******************************************************************************/

void
init_collision_avoidance()
{
  int i;

  /* We use lookup tables for sin and cos for faster computation. */
  init_fast_sin();
  init_fast_cos();

  /* First we install an array of modes. All values in the modes are set to default. 
   * So every non specified value in colli_modes.ini will be ok.
   */
  mode_structure_array = (struct mode_structure **) 
    malloc( NUMBER_OF_MODES * sizeof(struct mode_structure *)); 
  
  /* Now we read the values for the differnt modes. */
  load_parameters("colli_modes.ini");

  /* If no mode is set we use the default mode. */
  ACTUAL_MODE = mode_structure_array[DEFAULT_MODE];
  mode_number = DEFAULT_MODE;

  BASE_SetIntervalUpdates((long) ( COLLISION_UPDATE_INTERVAL * 1000.0)); 
  
  BASE_RotateCollisionAcceleration(RAD_TO_DEG(ACTUAL_MODE->target_rot_acceleration));
  BASE_TranslateCollisionAcceleration(ACTUAL_MODE->target_trans_acceleration);
  BASE_RotateVelocity( RAD_TO_DEG( ACTUAL_MODE->target_max_rot_speed));
  BASE_TranslateVelocity( ACTUAL_MODE->target_max_trans_speed);

  /* No target specified */
  target_flag = FALSE;

  /* No target reached */
  rwi_base.collision_state.target_reached = FALSE;
  
  /* For recovery from sonar errors */
  gettimeofday(&Last_CollLine_Update, 0);
  
  /* For recovery from laser errors */
  gettimeofday(&Last_FrontLaserPoint_Update, 0);
  gettimeofday(&Last_RearLaserPoint_Update, 0);
  
  /* We want to delete old vision lines and points */
  gettimeofday(&Last_VisionLine_Update, 0);
  gettimeofday(&Last_VisionPoint_Update, 0);
  
  /* Allocate memory for CollLines*/
  CollLines = (struct LineSeg **) malloc(bRobot.sonar_cols[0] * sizeof(struct LineSeg *));
  for (i = 0; i < bRobot.sonar_cols[0]; i++)
    CollLines[i] = (struct LineSeg *) malloc( REMEMBER_INTERVAL * sizeof(struct LineSeg));

  /* Allocate memory for the sonar angles */
  SonarAngle = (float *) malloc( bRobot.sonar_cols[0] * sizeof(float));

  initTcxStructures();
  init_SonarAngle();
  initLaserPointsStructure();
  COLLI_init_obstacle_structs();
}


/**********************************************************************/
void
COLLI_start_collision_avoidance()
{
  
  init_collision_avoidance();

  BASE_InstallHandler(update_CollisionStatus, STATUS_REPORT, NULL);  
  
  if (use_bumper) 
    BASE_InstallHandler(COLLI_BumpHandler, BUMP, NULL);  

  if ( use_sonar) {
    SONAR_InstallHandler(update_CollLines, SONAR_RT_COMPLETE, NULL);   
    SONAR_InstallHandler(Inc_next_reading, SONAR_REPORT, NULL);
    SONAR_LoopStart(sonar_mask_array[0]);
  }


  /* Convert the distances into points after each scan. */
  if ( use_laser) { 
    LASER_InstallHandler( update_LaserPoints, SINGLE_LASER_REPORT, NULL);
  }
}



/**********************************************************************/
void
Inc_next_reading(Pointer callback_data, Pointer client_data)
{
  next_CollLine_reading = (next_CollLine_reading+1) % REMEMBER_INTERVAL;
}


/* We compute the time since the last call to this routine. 
 * When has the collision avoidance been updated the last time? */
static BOOLEAN
checkBaseUpdate( struct timeval* now)
{
  static struct timeval Last_Collision_Avoidance;
  float tDiff;

  static BOOLEAN firstTime = TRUE;
  if (firstTime) {
    gettimeofday(&Last_Collision_Avoidance,0);
    firstTime = FALSE;
  }

  tDiff = timeDiff( now, &Last_Collision_Avoidance);

  Last_Collision_Avoidance = *now;

  if ( tDiff > 2.0 * COLLISION_UPDATE_INTERVAL) {
    
    BOOLEAN targetFlagSave = target_flag;
    
    if ( dumpInfo) {
      fprintf(dumpFile, "No collision avoidance for (%f) seconds!\n", tDiff);
      fprintf(stderr, "Stop the robot\n");
    }
    
    fprintf(stderr, "No collision avoidance for (%f) seconds!\n", tDiff);
    fprintf(stderr, "Stop the robot\n");
    BASE_TranslateHalt();
    BASE_RotateHalt();
    target_flag = targetFlagSave;
    return FALSE;
  }
  else
    return TRUE;
}  


static BOOLEAN
checkSonarUpdate( struct timeval* now)
{
    
  float tDiff = timeDiff( now, &Last_CollLine_Update);
  
  if ( tDiff > SONAR_UPDATE_TIME) {
    
    if ( dumpInfo) {
      fprintf(dumpFile, "No sonar for %f seconds. ", tDiff);
    }
    fprintf( stderr, "No sonar for %f seconds. ", tDiff);

    /* Try to start the loop again. */
    if (base_device.dev.use_simulator) {
      if (dumpInfo)
	fprintf( dumpFile, "Try to start sonar simulation loop again.\n");
      SONAR_LoopStart(sonar_act_mask);
    }
    return FALSE;	 /* Sonar too old. */
  }
  else
    return TRUE;    /* Sonar up to date. */
}


#define GAP_BETWEEN_RESTARTS 0.3

static BOOLEAN
checkLaserUpdate( struct timeval* now)
{
  static struct timeval lastFrontRestart;
  static struct timeval lastRearRestart;
  
  float frontTDiff = timeDiff( now, &Last_FrontLaserPoint_Update);
  float rearTDiff = timeDiff( now, &Last_RearLaserPoint_Update);

  if ( ( USE_FRONT_LASER && (frontTDiff > LASER_UPDATE_TIME))
       || ( USE_REAR_LASER && (rearTDiff > LASER_UPDATE_TIME))) {
    
    if ( USE_FRONT_LASER && (frontTDiff > LASER_UPDATE_TIME)) {
      if ( timeDiff( now, &lastFrontRestart) > GAP_BETWEEN_RESTARTS) {
	gettimeofday( &lastFrontRestart, 0);
	requestNextLaserScan( FRONT_LASER);
	fprintf(stderr, "No laser for %f %f seconds. ", frontTDiff, rearTDiff);
      }
    }
    else {
      if ( timeDiff( now, &lastRearRestart) > GAP_BETWEEN_RESTARTS) {
	gettimeofday( &lastRearRestart, 0);
	requestNextLaserScan( REAR_LASER);
	fprintf(stderr, "No laser for %f %f seconds. ", frontTDiff, rearTDiff);
      }
    }
    
    if ( dumpInfo) {
      fprintf(dumpFile, "No laser for %f %f seconds. ", frontTDiff, rearTDiff);
    }
    
    return FALSE;  /* Laser too old. */
  }
  else
    return TRUE;   /* Laser up to date. */
}


/**********************************************************************
 * Checks wether the collision information (sonar lines, vision lines and
 * vision points) are up to date and wether the 
 * time since last collision avoidance is not too big.
 **********************************************************************/
BOOLEAN
update_check_ok(void)
{
  static BOOLEAN robotStoppedAlready = FALSE;
  static BOOLEAN targetFlagAtStop;
  static struct timeval timeOfStop; 
  BOOLEAN stopRobot = FALSE;
  
  struct timeval now;
  gettimeofday( &now, 0);

  /* We compute the time since the last call to this routine. */
  /* When has the collision avoidance been updated the last time? */
  if ( ! checkBaseUpdate( &now))
    return FALSE;
  
  updateBumperTimer();
  
  /*----------------------------------------------------------------
   * Check for proximity sensors.
   *----------------------------------------------------------------*/

  /* We compute the time since last data received from sonar. *
   * If the data is too old, we stop the robot.               */
  stopRobot = use_sonar && ! checkSonarUpdate( &now);
  
  /* We compute the time since last data received from laser. *
   * If the data is too old, we stop the robot.               */
  stopRobot = stopRobot || ( use_laser && ! checkLaserUpdate( &now));
  
  /* Stop the robot if the proximity information is too old. */
  if ( stopRobot) {

    /* Is it the first time to stop the robot?
     * Save the target flag and the current time */
    if ( ! robotStoppedAlready) {
      targetFlagAtStop    = target_flag;
      target_flag         = FALSE;
      robotStoppedAlready = TRUE;
      gettimeofday( &timeOfStop, 0);

      fprintf( stderr, "Stop the robot\n");
      BASE_TranslateHalt();
      BASE_RotateHalt();
    }
  }
  else {

    if ( robotStoppedAlready) {
      
      /* Recover from the stop if it is recent enough. */
      if ( timeDiff( &now, &timeOfStop) < MAX_RECOVER_TIME) {
	rwi_base.stopped_by_colli = TRUE;
	fprintf( stderr, "Activate target again.\n");
	target_flag = targetFlagAtStop;
      }
    }

    robotStoppedAlready = FALSE;
  }

  return ! stopRobot;
}



/**********************************************************************
 **********************************************************************
 *            Functions to check the state of the robot and
 *               the collision avoidance.
 **********************************************************************
 **********************************************************************/




/**********************************************************************
 * This routine is called to check wether the robot is still in rotation.
 **********************************************************************/
BOOLEAN 
stillInRotation()
{
  static int rotationCnt = 0; 
  static int noRotationCnt = 0; 

#define MIN_ROT_WAIT_CNT 2
#define MIN_NO_ROTATION_CNT 1


  /* We want to wait at least two cycles at the beginning. */ 
  if (rotationCnt++ > MIN_ROT_WAIT_CNT &&
      fabs(actual_velocities.current_rvel) < MIN_ROT_SPEED) { 
    if ( ++noRotationCnt > MIN_NO_ROTATION_CNT) {
      noRotationCnt = 0; 
      rotationCnt = 0; 
      return FALSE;
    }
    else {
      if (dumpInfo) {
	fprintf( stderr, "No rotation, but still wait.\n");
	fprintf( dumpFile, "No rotation, but still wait.\n");
      }
      return TRUE;
    }
  } 
  else { 
    if (dumpInfo) {
      fprintf ( dumpFile, "rv %d  %f\n", rotationCnt, 
	       RAD_TO_DEG(actual_velocities.current_rvel)); 
    }
    if (0) fprintf (stderr, "rv %d  %f\n", rotationCnt, 
		    RAD_TO_DEG(actual_velocities.current_rvel));
    return TRUE; 
  }
}

/**********************************************************************
 * This routine is called to check wether the robot is still in rotation.
 **********************************************************************/
BOOLEAN 
stillInTranslation()
{
  static int translationCnt = 0;
  static int noTranslationCnt = 0;

#define MIN_TRANS_WAIT_CNT 3
#define MIN_NO_TRANSLATION_CNT 0

  /* We want to wait at least two cycles at the beginning. */
  if (translationCnt++ > MIN_TRANS_WAIT_CNT &&
      fabs(actual_velocities.current_tvel) < MIN_TRANS_SPEED) {
    if ( ++noTranslationCnt > MIN_NO_TRANSLATION_CNT) {
      noTranslationCnt = 0; 
      translationCnt = 0; 
      return FALSE;
    }
    else {
      if (dumpInfo) {
	fprintf( stderr, "No translation, but still wait.\n");
	fprintf( dumpFile, "No translation, but still wait.\n");
      }
      return TRUE;
    }
  }
  else {
    if (dumpInfo) {
      fprintf ( dumpFile, "tv %d  %f\n", translationCnt,
	       actual_velocities.current_tvel);
      fprintf (stderr, "tv %d  %f\n", translationCnt,
	       actual_velocities.current_tvel);
    }
    return TRUE;
  }
}



/**********************************************************************
 * The next two functions are used to check for a time interval.
 **********************************************************************/

void
setTimer( int secs)
{
  fprintf( stderr, "Set timer to %d secs.\n", secs);
  timeInterval = secs;
  gettimeofday( &beginTime, 0);
}


BOOLEAN
stillInTimeInterval()
{
  struct timeval now;

  gettimeofday( &now, 0);
  now.tv_usec -= beginTime.tv_usec;
  now.tv_sec -= beginTime.tv_sec;
  if (now.tv_usec < 0) {
    now.tv_usec += 1000000;
    now.tv_sec -= 1;
  }
  
  if ( dumpInfo) {
    if ( (int) now.tv_sec % 10 == 0 && (int) now.tv_sec != 0) {
      fprintf( dumpFile, "Sec (%.4f)\n",
	      (float)now.tv_sec + (float)now.tv_usec / 1000000.0);
      fprintf( stderr, "Sec (%.4f)\n",
	      (float)now.tv_sec + (float)now.tv_usec / 1000000.0);
    }
  }
  if (now.tv_sec > timeInterval)
    return FALSE;
  else return TRUE;
}




/**********************************************************************
 **********************************************************************
 *            Functions to determine admissible velocities.
 **********************************************************************
 **********************************************************************/

     

/**********************************************************************
 *  Tests wether a point is behind the robot.
 **********************************************************************/
BOOLEAN
behindPoint( Point rpos, float rrot, Point pt)
{
  return ( fabs( compute_robot_angle_to_point( rpos, rrot, pt)) >=  DEG_90);
}


/**********************************************************************
 *   Computes the angle distance from the robot at rpos to pt.        *
 **********************************************************************/
float
compute_robot_angle_to_point(Point rpos, float rrot, Point target)
{
  float angle; 

  angle = rrot - compute_angle_2p(rpos, target);

  if (angle < -DEG_180)
    angle = DEG_360 + angle;
  else if (angle > DEG_180)
    angle -= DEG_360;

  return(angle);
}


/**********************************************************************
 * Computes the maximal velocity such that the robot can stop within the 
 * distance. 
 * The formula is the solution to:
 * coll_dist = -0.5 * a * SQR(t) + tvel * t      and
 * tvel = -a * t
 ***********************************************************************/
float
maxVelForEmergencyStop(float distance, float acceleration)
{
  return 0.5 * (fsqrt( 2.0 * distance * acceleration));
}


/**********************************************************************
 * Computes the distance covered by the robot for a given velocity
 * and acceleration until it stops.
 ***********************************************************************/
float
brakeDistance( float velocity, float acceleration)
{
  float time;

  if (fabs(acceleration) > EPSILON) {
    time = fabs(velocity / acceleration);
    return (velocity * time + 0.5 * acceleration * SQR(time));
  }
  else
    return(MAXFLOAT);
}

/**********************************************************************
 * Computes the time needed to stop.
 ***********************************************************************/
float
brakeTime( float velocity, float acceleration)
{
   if (fabs(acceleration) > EPSILON) 
      return fabs(velocity / acceleration);
  else
    return(MAXFLOAT);
}


/**********************************************************************
 **********************************************************************
 *            Functions to determine admissible velocities.
 **********************************************************************
 **********************************************************************/


/* Checks wether direct way to the target is free. */
BOOLEAN
checkForTargetWayFree( Point rpos, float rrot, float tvel,
		       float radius, Point target)
{
  float targetDist, collDist;
  
  /* The distance to the target. */
  targetDist = compute_distance(rpos, target);
  
  /* The distance to the next obstacle. */
  collDist = collDistInTargetDirection( rpos,
				       tvel,
				       target,
				       radius);
  
  if (collDist > targetDist) 
    return TRUE;
  else
    return FALSE;
}





/**********************************************************************
 * The security dist should depend on the velocity. 
 **********************************************************************/
float
speed_dependent_security_dist(float tvel)
{
  float tmp;

  /* If tvel is MIN_SECURITY_SPEED the additional dist should be 0.0.
   * If tvel is MAX_SECURITY_SPEED the additional dist should be MAX_SECURITY_DIST
   */

  tmp = fnorm( tvel,
	       ACTUAL_MODE->min_security_speed,
	       ACTUAL_MODE->max_security_speed, 0.0, 1.0);

  tmp = SQR( MAX(0.0, tmp));

  return( tmp * ACTUAL_MODE->max_security_dist);
}



/**********************************************************************
 * The maximal velocity is stored in traj->max_tvel if traj->tvel positive
 * (otherwise in traj->min_tvel).
 **********************************************************************/
void
compute_max_velocity( float coll_dist, trajectory *traj)
{
  
  float max_tvel_for_required_space;
  float min_tvel, max_tvel, tmp;

  traj->admissible = TRUE;

  /* This gives us the security that the robot has one more cycle
   * to stop.
   */
  if (fabs(traj->tvel) < EPSILON)
  {
     coll_dist = MAX( 0.0, (coll_dist*DEG_360/MAX_RANGE) - 
		      fabs( COLLISION_UPDATE_INTERVAL *
			   actual_velocities.current_rvel));
  }
  else
  {
     coll_dist = MAX( 0.0, coll_dist - 
		     fabs( COLLISION_UPDATE_INTERVAL *
			  actual_velocities.current_tvel));
  }
  /* If the robot is in the emergencyStop we are more conservative about
   * starting the robot again.
   */
  if ( emergencyStop) coll_dist *= 0.5;

  if( fabs(traj->tvel) < EPSILON)
  {
     if (coll_dist  <= DEG_TO_RAD(ACTUAL_MODE->security_angle))
     {
	traj->admissible = FALSE;
	return;
     }
  }
  else
  {
     if( coll_dist <= ACTUAL_MODE->min_dist)
     { 
       traj->admissible = FALSE;
	return;
     }
  }
  /* Which velocity is relevant? If tvel is negative the most safe velocity in this
   * direction is max_tvel, otherwise it is min_tvel. 
   */

  if (traj->tvel == 0.0) {
    /* No collision possible so we don't have to change any velocity. */
    traj->max_tvel = traj->min_tvel = 0.0;
    return;
  }
 
  if (traj->tvel < 0.0) {
    min_tvel = MAX(0.0, -traj->max_tvel);
    max_tvel = -traj->min_tvel;
  }
  else {
    min_tvel = MAX(0.0, traj->min_tvel);
    max_tvel = traj->max_tvel;
  }
    
    
  max_tvel_for_required_space =
     maxVelForEmergencyStop( coll_dist, ACTUAL_MODE->target_trans_acceleration);
  
  if (min_tvel >= max_tvel_for_required_space) {
    /* Trajectory not allowed. Emergency_stop not possible */
     traj->admissible = FALSE;
    return;
  }
  
  else  
    /* We use the biggest velocity without speed reduction */
    tmp = MIN(max_tvel, max_tvel_for_required_space);
  
  if (traj->tvel < 0.0) 
     traj->min_tvel = -tmp;
  else if (traj->tvel > 0.0) 
     traj->max_tvel = tmp;
}



/**********************************************************************
 * Given the actual velocities and accelerations this function computes
 * the possible velocities within the next ACCELERATION_ASSERTION_TIME
 * seconds. The values are stored in the structure actual_velocities.
 **********************************************************************/
void
compute_possible_velocities( float tacc, float racc)
{
  float delta_tv, delta_rv;
  float max_tvel, max_rvel;
  
  max_tvel = target_flag ? ACTUAL_MODE->target_max_trans_speed : fabs(desired_trans_velocity);
  max_rvel = target_flag ? ACTUAL_MODE->target_max_rot_speed : fabs(DEG_TO_RAD(desired_rot_velocity));

  if (tacc < 0.0) {
    if ( dumpInfo)
      fprintf(dumpFile, "Error. Negative trans acceleration.\n");
    fprintf(stderr, "Error. Negative trans acceleration.\n");
    tacc = -tacc;
  }

  if (racc < 0.0) {
    if (dumpInfo)
      fprintf(dumpFile, "Error. Negative rot acceleration.\n");
    fprintf(stderr, "Error. Negative rot acceleration.\n");
    racc = -racc;
  }

  delta_tv = tacc * ACCELERATION_ASSERTION_TIME;
  
  actual_velocities.max_tvel = MIN( max_tvel, actual_velocities.current_tvel + delta_tv);

  
  if (target_flag  && !rotatingAwayFlag && !haltingFlag && !achieveDistanceFlag && !rotateAwayFlag)
    actual_velocities.min_tvel = MAX(0.0, actual_velocities.current_tvel - delta_tv);
  else
    actual_velocities.min_tvel = MAX(-max_tvel, actual_velocities.current_tvel - delta_tv);

  
  delta_rv = racc * ACCELERATION_ASSERTION_TIME;
  
  actual_velocities.max_rvel = MIN( max_rvel, actual_velocities.current_rvel + delta_rv);
  actual_velocities.min_rvel = MAX( -max_rvel, actual_velocities.current_rvel - delta_rv);

  if (actual_velocities.max_tvel < actual_velocities.min_tvel){
    if (target_flag) {
      if (dumpInfo)
	fprintf(dumpFile, "Error, must increase max_tvel from %f to %f\n",
		actual_velocities.max_tvel,actual_velocities.min_tvel);
    }
    actual_velocities.max_tvel = actual_velocities.min_tvel;
  }
  if (actual_velocities.max_rvel < actual_velocities.min_rvel){ 
    if (target_flag) {
      if (dumpInfo)
	fprintf(dumpFile, "Error, must increase max_rvel from %f to %f\n",
		actual_velocities.max_rvel,actual_velocities.min_rvel);
    }
    actual_velocities.max_rvel = actual_velocities.min_rvel;
  }
}





/**********************************************************************
 * The matrizes contain for each trajectory the rvel / tvel
 * combination for DIFFERENT_VELOCITIES different tvels.
 **********************************************************************/
void
generateVelocityCombinations( VelocityCombination*** combinations,
			      int iDim, int jDim)
{
    int i,j;
    float tvelStep, rvelStep;
    float minTvel, minRvel;
    float rvelIterations = 0.0;
    float previousRvel;
    
    int sign;
    BOOLEAN foundStraightTrajectory = FALSE;
    
    static int iDimension = 0;
    static int jDimension = 0;
    static VelocityCombination** combi = NULL;
    
    /* Let's check wether the dimensions have changed. */
    if ( iDimension != iDim || jDimension != jDim) {
      
	if ( combi != NULL)
	   free( combi);
	
	combi = (VelocityCombination **)
	    malloc( iDim * sizeof(VelocityCombination *));
	for (i = 0; i < iDim; i++) 
	    combi[i] = (VelocityCombination *)
		malloc( jDim * sizeof(VelocityCombination));

	iDimension = iDim;
	jDimension = jDim;
    }
    
    minTvel = actual_velocities.min_tvel;
    minRvel = actual_velocities.min_rvel;
    
    tvelStep = (actual_velocities.max_tvel - minTvel) /
	(float) (ACTUAL_MODE->number_of_tvels -1);
    rvelStep = (actual_velocities.max_rvel - minRvel) /
	(float) (ACTUAL_MODE->number_of_rvels -1);

    for ( i = 0, sign = SGN( minRvel), previousRvel = minRvel; i < iDimension; i++) {
      
      float rvel = minRvel + i * rvelStep;
      
      /* If the sign of the rotation changes we insert straight translation. */
      if ( ! foundStraightTrajectory && (SGN( rvel) != sign)) {
	foundStraightTrajectory = TRUE;
	
	/* Set the rotational velocity, which is closer to zero, to zero. */
	if ( fabs( rvel) < fabs( previousRvel)) 
	  rvel = 0.0;
	else
	  /* The previous velocity was smaller. */
	  /* This can't happen for i == 0. */
	  for ( j = 0; j < jDimension; j++)  {
	    combi[i-1][j].rvel = 0.0;
	  }
      }
      for ( j = 0; j < jDimension; j++)  {
	combi[i][j].rvel = rvel;
	combi[i][j].tvel = minTvel + j * tvelStep;
      }
    }
    
    /* Now set the values. */
    *combinations = combi;
}


/**********************************************************************
 * The next matrizes are used to store the evaluation for the rvels
 * and tvels described above.
 **********************************************************************/
void
allocateEvaluations( float*** velocities, float*** distances,
		     float*** angles, float*** values,
		     int iDim, int jDim)
{
   static int iDimension = 0;
   static int jDimension = 0;
   
   static float** vels = NULL;
   static float** dists = NULL;
   static float** angs = NULL;
   static float** vals = NULL;

   /* Let' check wether the dimensions have changed. */
   if ( iDimension != iDim || jDimension != jDim) {

      int i;

      /* Velocities. */
      if ( vels != NULL)
	 free( vels);
      vels = (float **) malloc( iDim * sizeof(float *));
      for (i = 0; i < iDim; i++) 
	 vels[i] = (float *) malloc( jDim * sizeof(float));
      
    
      /* Distances. */
      if ( dists != NULL)
	 free( dists);
      
      dists = (float **) malloc( iDim * sizeof(float *));
      for (i = 0; i < iDim; i++) 
	 dists[i] = (float *) malloc( jDim * sizeof(float));
      
      
      /* Angles. */
      if ( angs != NULL) 
	 free( angs);
      
      angs = (float **) malloc( iDim * sizeof(float *));
      for (i = 0; i < iDim; i++) 
	 angs[i] = (float *) malloc( jDim * sizeof(float));


      /* Values. */
      if ( vals != NULL)
	 free( vals);
      
      vals = (float **) malloc( iDim * sizeof(float *));
      for (i = 0; i < iDim; i++) 
	 vals[i] = (float *) malloc( jDim * sizeof(float));
      
      iDimension = iDim;
      jDimension = jDim;
   }
   
   *velocities = vels;
   *distances = dists;
   *angles = angs;
   *values = vals;
}

   

