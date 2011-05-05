
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


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>

#include "tcx.h"
#include "tcxP.h"
#include "fakeSensors.hh" 
#ifdef UNIBONN
#include "localize.h"
#endif
#include "beeSoftVersion.h"


#define TCX_define_variables /* this makes sure variables are installed */
#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */
#include "obstacleServer.h"
#include "BASE-messages.h"
#ifdef UNIBONN
#include "LOCALIZE-messages.h"
#endif

#define TCX_USER_MODULE_NAME "OBSTACLE_SERVER"

BASE_register_auto_update_type baseUpdate;
LOCALIZE_register_auto_update_type localizeUpdate;

/************************************************************************
 * Global variables according to robot position.
 ************************************************************************/
realPosition mapPosition;
realPosition robotPosition;
correctionParameter correction;
int correctionParametersKnown = FALSE;
int robotPositionKnown = FALSE;

#define MOVEMENT_THRESHOLD 30.0
#define MAX_NUMBER_OF_OBSTACLE_POINTS 90

/************************************************************************
 * Checks wether the robot has moved enough since the last update and
 * sends the obstacles around the robot to the collision avoidance.
 ************************************************************************/
static void
deleteObstaclePoints()
{
  BASE_obstacle_points_type obstaclePoints;

  if ( BASE == NULL)
    return;
    
  obstaclePoints.points = NULL;
  obstaclePoints.no_of_points = 0;
  
  tcxSendMsg( BASE, "BASE_obstacle_points", &obstaclePoints);
}

/************************************************************************
 * Checks wether the robot has moved enough since the last update and
 * sends the obstacles around the robot to the collision avoidance.
 ************************************************************************/
static void
updateObstaclePoints()
{  
  static int movement = MOVEMENT_THRESHOLD;
  static realPosition previousPos;
  static int firstTime = TRUE;

  if ( BASE == NULL)
    return;
  
  /* Update the summed movement. */
  if ( firstTime) {
    firstTime = FALSE;
  }
  else
    movement += sqrt( fSqr( previousPos.x - robotPosition.x)
		      + fSqr( previousPos.y - robotPosition.y)
		      + fSqr( previousPos.rot - robotPosition.rot));

  previousPos = robotPosition;
  
  if ( movement >= MOVEMENT_THRESHOLD) {

    BASE_obstacle_points_type obstaclePoints;
    obstaclePoint points[MAX_NUMBER_OF_OBSTACLE_POINTS];
    int i, numberOfPoints = 0;

    realPosition robPosWithCorrectRotation = robotPosition;
    robPosWithCorrectRotation.rot = DEG_90 - DegToRad( robPosWithCorrectRotation.rot);
    
    for ( i = 0; i < MAX_NUMBER_OF_OBSTACLE_POINTS; i++) {

      float relativeAngle = DegToRad( (i - MAX_NUMBER_OF_OBSTACLE_POINTS / 2)
				      * 360 / MAX_NUMBER_OF_OBSTACLE_POINTS);

      if ( obstacleInDirection( mapPosition, robPosWithCorrectRotation,
				relativeAngle,
				&(points[numberOfPoints]),
				&correction))
	numberOfPoints++;
    }
      
    obstaclePoints.points = points;
    obstaclePoints.no_of_points = numberOfPoints;
    
    tcxSendMsg( BASE, "BASE_obstacle_points", &obstaclePoints);
    movement = 0.0;
  }
}

/************************************************************************
 *
 *   Name:         OBSTACLE_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
OBSTACLE_close_handler(char *name, TCX_MODULE_PTR module)
{

  fprintf( stderr, "OBSTACLE_SERVER: closed connection detected: %s\n", name);
  
  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    exit(0);
  }
  else if ( module == BASE) {
    fprintf( stderr, "BASE disconnected.\n");
    robotPositionKnown = FALSE;
    BASE = NULL;
  }
  else if ( module == LOCALIZE) {
    fprintf( stderr, "LOCALIZE disconnected.\n");
    correctionParametersKnown = FALSE;
    deleteObstaclePoints();
    LOCALIZE = NULL;
  }
}

/**********************************************************************
 **********************************************************************
 *
 *  BASE handlers
 *
 **********************************************************************
 **********************************************************************/
void
BASE_update_status_reply_handler( TCX_REF_PTR ref,
				  BASE_update_status_reply_ptr status)
{
#ifdef TCX_debug
  fprintf( stderr, "TCX: Received a BASE_update_status_reply message.\n");
  fprintf( stderr, "robot: %g %g %g\n", 
	   status->pos_x, status->pos_y, status->orientation);
#endif

  robotPositionKnown = TRUE;
  
  /* Set the new robot position. */
  robotPosition.x = status->pos_x;
  robotPosition.y = status->pos_y;
  robotPosition.rot = status->orientation;

  if ( correctionParametersKnown) {
    
    robotCoordinates2MapCoordinates( robotPosition.x,
				     robotPosition.y,
				     robotPosition.rot,
				     correction.x,
				     correction.y,
				     correction.rot,
				     correction.type,
				     &(mapPosition.x),
				     &(mapPosition.y),
				     &(mapPosition.rot));  
    
#ifdef TCX_debug
    fprintf( stderr, "New map position: %f %f %f\n",
	     mapPosition.x, mapPosition.y, RadToDeg(mapPosition.rot));
#endif

    updateObstaclePoints();
  }
  
  tcxFree("BASE_update_status_reply", status); /* don't remove this! */
}

void
BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				  BASE_robot_position_reply_ptr pos)
{;}


void
BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
				   BASE_action_executed_reply_ptr data)
{;}


/**********************************************************************
 **********************************************************************
 *
 *  LOCALIZE handlers
 *
 **********************************************************************
 **********************************************************************/
void
LOCALIZE_map_reply_handler( TCX_REF_PTR                 ref,
				 LOCALIZE_map_reply_ptr map)
{}

void
LOCALIZE_update_status_reply_handler( TCX_REF_PTR                 ref,
				      LOCALIZE_update_status_reply_ptr status)
{
#ifdef TCX_debug
  fprintf( stderr, "TCX: Received a LOCALIZE_update_status_reply message.\n");
  fprintf( stderr, "local: %d %f\n",
	   status->numberOfLocalMaxima, status->probOfGlobalMaximum);
#endif
  fprintf( stderr, "TCX: Received a LOCALIZE_update_status_reply message.\n");
  fprintf( stderr, "local: %d %f\n",
	   status->numberOfLocalMaxima, status->probOfGlobalMaximum);
  
  /* Set the new correction parameters. */
  correction.x = status->corrX;
  correction.y = status->corrY;
  correction.rot = status->corrRot;
  correction.type = status->corrType;
  
  if (status->numberOfLocalMaxima < 3) {  
    correctionParametersKnown = TRUE;

    if ( robotPositionKnown) {
      
      /* Get the new corrected position of the robot. */
      robotCoordinates2MapCoordinates( robotPosition.x,
				       robotPosition.y,
				       robotPosition.rot,
				       correction.x,
				       correction.y,
				       correction.rot,
				       correction.type,
				       &mapPosition.x, &mapPosition.y, &mapPosition.rot);  
      
      fprintf( stderr, "New map position: %f %f %f\n",
	       mapPosition.x, mapPosition.y, RadToDeg(mapPosition.rot));
    }
  }
  else
    correctionParametersKnown = FALSE;
  
  tcxFree("LOCALIZE_update_status_reply", status); /* don't remove this! */
}


void
initTcx()
{
  char *tcxMachine = NULL;
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    BASE_messages,
    LOCALIZE_messages
  };

  baseUpdate.subscribe_status_report = 1;
  baseUpdate.subscribe_sonar_report = 0;
  baseUpdate.subscribe_laser_report = 0;
  baseUpdate.subscribe_ir_report = 0;
  baseUpdate.subscribe_colli_report = 0;
  
  localizeUpdate.subscribe = 1;

  fprintf(stderr, "Connecting to TCX...");
  tcxMachine = getenv("TCXHOST");   

  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize(TCX_USER_MODULE_NAME, (void *) tcxMachine);
  fprintf(stderr, "done.\n");

  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));
  

  tcxRegisterHandlers(BASE_reply_handler_array,
		      sizeof(BASE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(LOCALIZE_reply_handler_array,
		      sizeof(LOCALIZE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterCloseHnd(OBSTACLE_close_handler); 
}


int
main( int argc, char** argv)
{
  float xrot, yrot;
  float distance;
  int i;
  int useBaseRouted = FALSE;
  struct timeval TCX_waiting_time = {0, 0};

  FILE *mapfd = NULL;

  if ( argc > 1) {
    for (i=1; i<argc; i++) {
      if ((strcmp(argv[i],"-router")==0))
	useBaseRouted = TRUE;
       else {
	fprintf( stderr, "Get map from %s\n", argv[i]);
	mapfd = fopen(argv[i], "r");
      }
    }    
  }
  
  if ( mapfd == NULL) {
    fprintf( stderr, "Get map from floor.sim\n");
    mapfd = fopen("floor.sim", "r");
  }

  installSimMap(mapfd); 

  initTcx();
  
  for (;;) {
    if ( LOCALIZE == NULL || BASE == NULL) {
      if ( LOCALIZE == NULL) {
	fprintf(stderr, "Connecting to %s...", TCX_LOCALIZE_MODULE_NAME);
	LOCALIZE = tcxConnectModule(TCX_LOCALIZE_MODULE_NAME);
	tcxSendMsg( LOCALIZE, "LOCALIZE_register_auto_update", &localizeUpdate);
	fprintf(stderr, "done.\n");
      }
      if ( BASE == NULL){
	fprintf(stderr, "Connecting to %s...", TCX_BASE_MODULE_NAME);
	if ( useBaseRouted)
	  BASE = tcxConnectModule("BASE_ROUTED");
	else
	  BASE = tcxConnectModule(TCX_BASE_MODULE_NAME);
	fprintf(stderr, "ok\n");
	tcxSendMsg(BASE, "BASE_register_auto_update", &baseUpdate);
	fprintf(stderr, "done.\n");
      }
    }
    else
      block_wait(NULL, 1, 0);
    
    tcxRecvLoop((void *) &TCX_waiting_time);
  }
  
  exit(0);			/* should never reach here! */
}

