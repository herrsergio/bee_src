
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



#define VERBOSE

typedef struct Object {
    float x;
    float y;
    int type;
} Object;


typedef struct TrashBin {
    float x;
    float y;
} TrashBin;


/* Different kinds of objects sent by the vision. */
#define EVERYTHING 0
#define TRASH_BIN 1
#define OBJECT 2

/* After a call to the clean up sequence do we need collision
 * avoidance? */
#define CONTINUE_WITH_COLLISION_AVOIDANCE 0
#define NO_MORE_COLLISION_AVOIDANCE 1

/* How long do we wait for the vision to give a position? */
#define MAX_WAITING_TIME_FOR_VISION 30


/* Different states of the clean up sequence. */

extern int cleanUpState;

#define NOT_RUNNING 0
#define RANDOM_WALK 1
#define APPROACH_FAR_OBJECT 2
#define GET_EXACT_OBJECT_POSITION 3
#define APPROACH_CLOSE_OBJECT 4
#define PICKUP_OBJECT 5
#define LOOK_FOR_TRASH_BIN 6
#define APPROACH_FAR_TRASH_BIN 7
#define GET_EXACT_TRASH_BIN_POSITION 8
#define APPROACH_CLOSE_TRASH_BIN 9
#define DROP_OBJECT 10

/* Vectors containing potential objects to pick up or trash bin
 * to drop the objects. */

#define MAX_NUMBER_OF_OBJECTS 2
#define MAX_NUMBER_OF_TRASH_BINS 2

/* Contain the absolute positions of the most recent objects / trash bins
 * from the vision. */
extern TrashBin farTrashBins[];
extern TrashBin closeTrashBins[];
extern Object   farObjects[];
extern Object   closeObjects[];

/* Number of the actual objects / trash bins. */
extern int numberOfFarObjects;
extern int numberOfFarTrashBins;
extern int numberOfCloseObjects;
extern int numberOfCloseTrashBins;

/* Actual points to be approached, picked up, ... */
extern Point actualFarObject;
extern Point actualCloseObject;
extern Point actualFarTrashBin;
extern Point actualCloseTrashBin;


/***************************************************************************
 ***************************************************************************
 *    FILE:     colliCleanUp.c
 *
 *    FUNCTION: special functions to pick up objects
 ***************************************************************************
 ***************************************************************************/

/* Starts the sequence to clean up. */
void
COLLI_StartCleaningUp();
/* Starts the sequence to clean up. */
void
COLLI_StopCleaningUp();

BOOLEAN
inCleanUpSequence( Point rPos, float rRot);


/***************************************************************************
 ***************************************************************************
 *    FILE:     colliCleanupArm.c
 *
 *    FUNCTION: special functions to pick up objects
 ***************************************************************************
 ***************************************************************************/

/* The next functions communicate with the arm module. */
void ARM_pickupAtGround(void);
void ARM_liftObject(void);
void ARM_dropObject(void);
void ARM_moveIn(void);
    
/* The next functions react to successful / unsuccessful execution
 * of arm commands. They change the state of the cleanup sequences. */
void ARM_pickupObjectReady( int success );
void ARM_liftObjectReady(int success);
void ARM_dropObjectReady(int success);
void ARM_moveInReady(int success);


/*************************************************************
 * colliCleanUpTools.c
 *************************************************************/

void
failSignal();
void
stateTransitionSignal();
void
infoSignal();

void
removeFarObject();
void
removeFarTrashBin();

void
removeAllFarObjects();
void
removeAllFarTrashBins();


/* Checks wether the clean up seqence accepts new objects from the vision. */
BOOLEAN
dontAcceptFarThings();

/* Checks wether the clean up seqence accepts new objects from the vision. */
BOOLEAN
dontAcceptCloseThings();


void
COLLI_turnToPoint( Point rPos,
		   float rRot,
		   Point object,
		   float rotateVelocity);


/* How long do we wait for the way to become free before giving up? */
#define MAX_WAITING_TIME_FOR_WAY_FREE 30

/* Translates to the point with the given velocity until the robot is
 * <distance> centimeters away from the point.
 * If <checkForWayFree> is TRUE at each call it is checked wether
 * the direct way to the point is free. */
int
COLLI_translateToPoint( Point rPos,
		       float rRot,
		       Point object,
		       float distance,
		       float translateVelocity,
		       float securityDist,
		       BOOLEAN checkForWayFree);

/* Different return values from the function above. */
#define POINT_IS_REACHED 0
#define POINT_IS_NOT_YET_REACHED 1
#define GAVE_UP 2

   
/*************************************************************
 * colliCleanUpArm.c
 *************************************************************/


/*************************************************************
 * colliCleanUpPick.c
 *************************************************************/

/* How long do we wait for the robot to reach the far object? */
#define MAX_WAITING_TIME_FOR_FAR_OBJECT_REACHED 60


/*************************************************************
 * colliCleanUpDrop.c
 *************************************************************/

/* How long do we wait for the robot to reach the far object? */
#define MAX_WAITING_TIME_FOR_FAR_TRASH_BIN_REACHED 60



/*************************************************************
 * colliCleanUpSunvis.c
 *************************************************************/

/* Sends SUNVIS a request for far objects of type <type>. */
void
sendRequestForFarObjects( int type);

/* Sends SUNVIS a request for exact position of an object of type <type>. */
void
sendRequestForExactPosition( int type);







