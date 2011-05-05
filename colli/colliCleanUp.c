
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



#ifdef CLEANUP


#include "collisionIntern.h"
#include "colliCleanUp.h"
#include "colliCleanUpStates.h"


/********************************************************************
 * Resets all flags and sets the robot into the default mode. 
 **********************************************************************/
void
resetCleanUpStates()
{
  cleanUpState = NOT_RUNNING;

  /* All internal states are not active.\n*/
  /* PICK */
  randomWalkInternalState = Not_Active;
  approachFarObjectInternalState = Not_Active;
  getExactObjectPositionInternalState = Not_Active;
  approachCloseObjectInternalState = Not_Active;
  pickUpObjectInternalState = Not_Active;

  /* DROP */
  lookForTrashBinInternalState = Not_Active;
  approachFarTrashBinInternalState = Not_Active;
  getExactTrashBinPositionInternalState = Not_Active;
  approachCloseTrashBinInternalState = Not_Active;
  dropObjectInternalState = Not_Active;

  /* We delete all stored objects except the far trash bins. */
  numberOfCloseObjects = 0;
  numberOfFarObjects = 0; 
  numberOfCloseTrashBins = 0;
  /* numberOfFarTrashBins = 0; */
  
  /* Reset the default mode. */
  COLLI_SetMode( (double) DEFAULT_MODE);
}
  

/**********************************************************************
 * The robot starts to clean up the ground and to put the objects in
 * trash bins.
 **********************************************************************/
void
COLLI_StartCleaningUp()
{
  /* Reset the finite automata. */
  resetCleanUpStates();

  BASE_TranslateVelocity (0.0);
  BASE_RotateVelocity ( 0.0);
  BASE_TranslateHalt();
  BASE_RotateHalt();

  COLLI_GoForward();
  
  /* The robot starts by walking around randomly. */
  cleanUpState = RANDOM_WALK;
}


/**********************************************************************
 * Stop is.
 **********************************************************************/
void
COLLI_StopCleaningUp()
{
  /* Reset the finite automata. */
  resetCleanUpStates();

  numberOfFarObjects = numberOfFarTrashBins = 0;
  
  COLLI_GoForward();
  
  BASE_TranslateHalt();
  BASE_RotateHalt();

  target_flag = FALSE;
  
  cleanUpState = NOT_RUNNING;
}
    


/**********************************************************************
 * Checks wether the robot is in a sequence of the following modes.
 **********************************************************************/
BOOLEAN
inCleanUpSequence( Point rPos, float rRot)
{
    switch (cleanUpState) {
    case NOT_RUNNING :
	return CONTINUE_WITH_COLLISION_AVOIDANCE;
    case RANDOM_WALK :
	return randomWalkState( rPos, rRot);
    case APPROACH_FAR_OBJECT :
      return approachFarObjectState( rPos, rRot);
    case GET_EXACT_OBJECT_POSITION :
	return getExactObjectPositionState( rPos, rRot);
    case APPROACH_CLOSE_OBJECT :
	return approachCloseObjectState( rPos, rRot);
    case PICKUP_OBJECT :
	return pickUpObjectState( rPos, rRot);
    case LOOK_FOR_TRASH_BIN :
	return lookForTrashBinState( rPos, rRot);
    case APPROACH_FAR_TRASH_BIN :
	return approachFarTrashBinState( rPos, rRot);
    case GET_EXACT_TRASH_BIN_POSITION :
	return getExactTrashBinPositionState( rPos, rRot);
    case APPROACH_CLOSE_TRASH_BIN :
	return approachCloseTrashBinState( rPos, rRot);
    case DROP_OBJECT :
	return dropObjectState( rPos, rRot);
    default :
       return CONTINUE_WITH_COLLISION_AVOIDANCE;
    }
}


#endif
