
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



/*************************************************************
 * colliCleanUpPick.c
 *************************************************************/

/* Different internal states of the main states. */
extern int randomWalkInternalState;
extern int approachFarObjectInternalState;
extern int getExactObjectPositionInternalState;
extern int approachCloseObjectInternalState;
extern int pickUpObjectInternalState;


/* Values for the internal states. */
#define Not_Active 0
#define Active 1
#define Wait_Until_Rotation_Finished 2
#define Wait_Until_Translation_Finished 3
#define Wait_For_Coarse_Position 4
#define Wait_For_Exact_Position 5

#define Wait_Until_Object_Reached 6
#define Wait_Until_Trash_Bin_Reached 7
#define Wait_Until_Facing_Trash_Bin 8

#define Wait_Until_Arm_Down 9
#define Wait_Until_Object_Lifted 10
#define Wait_Until_Object_Dropped 11
#define Wait_Until_Arm_In 12

/* Values for the internal states. */

/***************************************************************************
 * The robot moves randomly and waits until the vision sends an object.
 ***************************************************************************/
BOOLEAN
randomWalkState( Point rPos, float rRot);
BOOLEAN
startRandomWalkState();
BOOLEAN
quitRandomWalkState();
BOOLEAN
randomWalkStateFailed();


/***************************************************************************
 * The robot approaches a far object.
 ***************************************************************************/
BOOLEAN
approachFarObjectState( Point rPos, float rRot);
BOOLEAN
startApproachFarObjectState();
BOOLEAN
quitApproachFarObjectState();
BOOLEAN
approachFarObjectStateFailed();



/***************************************************************************
 * Waits for the vision to send the exact position of an object. 
 ***************************************************************************/
BOOLEAN
getExactObjectPositionState( Point rPos, float rRot);
BOOLEAN
startGetExactObjectPositionState();
BOOLEAN
quitGetExactObjectPositionState();
BOOLEAN
getExactObjectPositionStateFailed();



/***************************************************************************
 * Approaches a close object.
 ***************************************************************************/
BOOLEAN
approachCloseObjectState( Point rPos, float rRot);
BOOLEAN
startApproachCloseObjectState( Point rPos, float rRot);
BOOLEAN
quitApproachCloseObjectState();
BOOLEAN
approachCloseObjectStateFailed();

/***************************************************************************
 * Picks up an object.
 ***************************************************************************/
BOOLEAN
pickUpObjectState( Point rPos, float rRot);
BOOLEAN
startPickUpObjectState();
BOOLEAN
quitPickUpObjectState();
BOOLEAN
pickUpObjectStateFailed();






/***************************************************************************
 *
 ***************************************************************************/


/*************************************************************
 * colliCleanUpDrop.c
 *************************************************************/

extern int lookForTrashBinInternalState;
extern int approachFarTrashBinInternalState;
extern int getExactTrashBinPositionInternalState;
extern int approachCloseTrashBinInternalState;
extern int dropObjectInternalState;

/***************************************************************************
 * The robot moves randomly ( the arm is outside) and look for trash bins.
 ***************************************************************************/
BOOLEAN
lookForTrashBinState( Point rPos, float rRot);
BOOLEAN
startLookForTrashBinState();
BOOLEAN
quitLookForTrashBinState();
BOOLEAN
lookForTrashBinStateFailed();

/***************************************************************************
 * Approaches far trash bin.
 ***************************************************************************/
BOOLEAN
approachFarTrashBinState( Point rPos, float rRot);
BOOLEAN
startApproachFarTrashBinState();
BOOLEAN
quitApproachFarTrashBinState();
BOOLEAN
approachFarTrashBinStateFailed();



/***************************************************************************
 * Waits for the vision to send exact position of a trash bin.
 ***************************************************************************/
BOOLEAN
getExactTrashBinPositionState( Point rPos, float rRot);
BOOLEAN
startGetExactTrashBinPositionState();
BOOLEAN
quitGetExactTrashBinPositionState();
BOOLEAN
getExactTrashBinPositionStateFailed();


/***************************************************************************
 *  Approaches a close trash bin.
 ***************************************************************************/
BOOLEAN
approachCloseTrashBinState( Point rPos, float rRot);
BOOLEAN
startApproachCloseTrashBinState( Point rPos, float rRot);
BOOLEAN
quitApproachCloseTrashBinState();
BOOLEAN
approachCloseTrashBinStateFailed();



/***************************************************************************
 * Drops an object into the trash bin.
 ***************************************************************************/
BOOLEAN
dropObjectState( Point rPos, float rRot);
BOOLEAN
startDropObjectState();
BOOLEAN
quitDropObjectState();
BOOLEAN
dropObjectStateFailed();








