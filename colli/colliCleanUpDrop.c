
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


/* Different internal states of the main states above. */
int lookForTrashBinInternalState = Not_Active;
int approachFarTrashBinInternalState = Not_Active;
int getExactTrashBinPositionInternalState = Not_Active;
int approachCloseTrashBinInternalState = Not_Active;
int dropObjectInternalState = Not_Active;

TrashBin farTrashBins[MAX_NUMBER_OF_TRASH_BINS];
TrashBin closeTrashBins[MAX_NUMBER_OF_TRASH_BINS];

int numberOfFarTrashBins = 0;
int numberOfCloseTrashBins = 0;

/* Turn to drop the object. */
const float rotateToCloseTrashBinVelocity = 30.0;

/* Turn away from the trash bin. */
const float rotateFromCloseTrashBinVelocity = 30.0;

const float rotateToFarTrashBinVelocity = 30.0;

const float translateToCloseTrashBinVelocity = 30.0;
const float translateFromCloseTrashBinVelocity = 30.0;

const float distForApproachFarTrashBin = 200.0;
const float distForApproachCloseTrashBin = 40.0;
const float distForBackupFromCloseTrashBin = 20.0;
const float distForRotateFromCloseTrashBin = 180.0;

Point actualFarTrashBin;
Point actualCloseTrashBin;

/**********************************************************************
 **********************************************************************
 *           Functions for the different states                       *
 **********************************************************************
 **********************************************************************/


/**********************************************************************
 * Starts when the robot has picked up an object.
 * Ends when the vision sends the coarse position of a trash bin.
 **********************************************************************/
BOOLEAN
lookForTrashBinState( Point rPos, float rRot)
{
    switch (lookForTrashBinInternalState) {
      
    case (Not_Active) :

       /*---------------------------------------------*
	* Same as random walk.
	*---------------------------------------------*/
       
       SOUND_talk_text("Look for trash bin.");
       SOUND_play_message(18);
       return startLookForTrashBinState();
       
    case (Wait_For_Coarse_Position) :
       
       /*---------------------------------------------*
	* Just do nothing but wait for the vision to
	* find an object.
	*---------------------------------------------*/
	
	if ( numberOfFarTrashBins > 0) {
	    
	    /*---------------------------------------------*
	     * The vision found a trash bin. Quit this state
	     * and change to the next one.
	     *---------------------------------------------*/

	   SOUND_talk_text("Saw a trash bin.");
	   SOUND_play_message(17);
	   return quitLookForTrashBinState();
	}
   }
    return CONTINUE_WITH_COLLISION_AVOIDANCE;
}


    

/**********************************************************************
 * Starts the robot to go to an object.
 * Ends when the robot faces the object.
 **********************************************************************/
BOOLEAN
approachFarTrashBinState( Point rPos, float rRot)
{
    switch (approachFarTrashBinInternalState) {
       
    case (Not_Active) :

	/*---------------------------------------------*
	 * Start to approach the object.
	 *---------------------------------------------*/
	
	return startApproachFarTrashBinState();
      
    case (Wait_Until_Trash_Bin_Reached) :

	/*---------------------------------------------*
	 * Just do nothing but wait for the robot to
	 * get close enough to the trashBin.
	 *---------------------------------------------*/
	
	if ( rwi_base.collision_state.target_reached == TRUE) {
	    
	    /*---------------------------------------------*
	     * The robot is close enough. Turn to the trashBin.
	     *---------------------------------------------*/
	    
	    COLLI_turnToPoint( rPos, rRot, target, rotateToFarTrashBinVelocity);
	    
	    approachFarTrashBinInternalState = Wait_Until_Rotation_Finished;
	    
	    return NO_MORE_COLLISION_AVOIDANCE;
	}
	else {
	  if ( stillInTimeInterval())
	    /* The robot still has to approach the target. */
	    return CONTINUE_WITH_COLLISION_AVOIDANCE;
	  else {
	    /* That's too long. Forget the target. */
	    fprintf( stderr, "Can't reach the trash bin. ");
	    fprintf( stderr, "Start to look for other trash bins.\n");
	    return approachFarTrashBinStateFailed();
	  }
	}

    case (Wait_Until_Rotation_Finished) :

	/*---------------------------------------------*
	 * Just do nothing but wait for the robot to
	 * face the trashBin.
	 *---------------------------------------------*/
	
	if ( ! stillInRotation()) {
	    
	    /*---------------------------------------------*
	     * Rotation finished. Get into the next state.
	     *---------------------------------------------*/
	    
	    fprintf( stderr, "Reached the trash bin.\n");
	    return quitApproachFarTrashBinState( rPos, rRot);
	}
	else
	    return NO_MORE_COLLISION_AVOIDANCE;
    }
    return NO_MORE_COLLISION_AVOIDANCE;
}


/**********************************************************************
 * Starts when the robot faces an trash Bin. Sends the vision the information
 * to get the exact position of the trash Bin.
 * Ends when the position is known.
 **********************************************************************/
BOOLEAN
getExactTrashBinPositionState( Point rPos, float rRot)
{
   switch (getExactTrashBinPositionInternalState) {
		    
   case (Not_Active) :

      /*---------------------------------------------*
       * Send the vision the information to take
       * a closer look at the position of the trashBin.
       *---------------------------------------------*/

       fprintf(stderr, "Send request to the vision module.\n");
       return startGetExactTrashBinPositionState();
       
   case (Wait_For_Exact_Position) :

      /*---------------------------------------------*
       * Just wait. If interval too long we look for
       * far trash bins again.
       *---------------------------------------------*/
      
       if ( ! stillInTimeInterval()) {
	 fprintf(stderr, "Sorry, that's too long!\n");

	 return getExactTrashBinPositionStateFailed();
       }
	 
      if ( numberOfCloseTrashBins > 0) {
	 
	 /*---------------------------------------------*
	  * Got the information. Change state to pick up
	  * the trashBin and copy the trash bin to the
	  * next far trash bin.
	  *---------------------------------------------*/

	  return quitGetExactTrashBinPositionState();
      }
      return NO_MORE_COLLISION_AVOIDANCE;
   }
   return NO_MORE_COLLISION_AVOIDANCE;
}


/**********************************************************************
 * Starts when the robot has the exact position of an trashBin.
 * Ends when the robot faces this trashBin.
 **********************************************************************/
BOOLEAN
approachCloseTrashBinState( Point rPos, float rRot)
{
  switch (approachCloseTrashBinInternalState) {
    
  case (Not_Active) :
    
    /*---------------------------------------------*
     * First rotate to the trashBin (still backwards).
     *---------------------------------------------*/
    
      return startApproachCloseTrashBinState( rPos, rRot);
    
  case (Wait_Until_Rotation_Finished) :
    
    if ( ! stillInRotation()) {
      
      /*---------------------------------------------*
       * Done. Now let's wait till the robot can start
       * to translate.
       *---------------------------------------------*/
      
      fprintf(stderr, "Rotation finished.\n");
      approachCloseTrashBinInternalState = Wait_Until_Translation_Finished;
    }
    
    return NO_MORE_COLLISION_AVOIDANCE;
    
  case (Wait_Until_Translation_Finished) :
      
      /*---------------------------------------------*
       * The robot has to translate to the object.
       * The function returns the status of the translation.
       *---------------------------------------------*/
      
      switch ( COLLI_translateToPoint( rPos,
				      rRot,
				      actualCloseTrashBin,
				      distForApproachCloseTrashBin,
				      translateToCloseTrashBinVelocity,
				      ROB_RADIUS,
				      TRUE)) {
	
      case (POINT_IS_NOT_YET_REACHED) :
	/* Nothing to do but wait. */
	return NO_MORE_COLLISION_AVOIDANCE;
      case (POINT_IS_REACHED) :
	/*---------------------------------------------*
	 * Now really face the trash bin.
	 *---------------------------------------------*/
	fprintf(stderr, "yeah.\n");
	COLLI_turnToPoint( rPos,
			  DEG_180 + rRot,
			  actualCloseTrashBin,
			  rotateToCloseTrashBinVelocity);
	
	approachCloseTrashBinInternalState = Wait_Until_Facing_Trash_Bin;
	return NO_MORE_COLLISION_AVOIDANCE;
	
      case (GAVE_UP) :
	return approachCloseTrashBinStateFailed();
      }
      
    case ( Wait_Until_Facing_Trash_Bin) :
      
      if ( ! stillInRotation()) {
	
	/*---------------------------------------------*
	 * Done. Now let's drop the object.
	 *---------------------------------------------*/
	return quitApproachCloseTrashBinState();
      }
      return NO_MORE_COLLISION_AVOIDANCE;
    }
  return NO_MORE_COLLISION_AVOIDANCE;
}


/**********************************************************************
 * Starts when the robot faces a trash bin.
 * Ends when the arm is inside the robot.
 **********************************************************************/
BOOLEAN
dropObjectState( Point rPos, float rRot)
{
    switch (dropObjectInternalState) {
	
    case (Not_Active) :

	/*---------------------------------------------*
	 * First open the gripper.
	 *---------------------------------------------*/
      SOUND_talk_text("Drop object.");
      SOUND_play_message(19);
      return startDropObjectState();

    case (Wait_Until_Object_Dropped) :

	if (armState == OBJECT_DROPPED) {
	    
	    /*---------------------------------------------*
	     * The object is dropped. The robot should translate
	     * some centimeters backwards.
	     *---------------------------------------------*/

	    BASE_TranslateVelocity ( translateFromCloseTrashBinVelocity);
	    BASE_TranslateCollisionVelocity ( translateFromCloseTrashBinVelocity);
	    BASE_RotateVelocity ( 0.0);
	    BASE_RotateCollisionVelocity ( 0.0);
	    BASE_Translate( - distForBackupFromCloseTrashBin);

	    dropObjectInternalState = Wait_Until_Translation_Finished;
	}
	return NO_MORE_COLLISION_AVOIDANCE;
	
    case (Wait_Until_Translation_Finished) :

	if ( ! stillInTranslation()) {
	    
	    /*---------------------------------------------*
	     * The robot has backed up from the trash bin.
	     * Now the arm can be move inside again. At the
	     * same time the robot turns away from the trash bin.
	     *---------------------------------------------*/
	    ARM_moveIn();
	    
	    BASE_TranslateVelocity ( 0.0);
	    BASE_TranslateCollisionVelocity ( 0.0);
	    BASE_RotateVelocity ( rotateFromCloseTrashBinVelocity);
	    BASE_RotateCollisionVelocity ( rotateFromCloseTrashBinVelocity);
	    BASE_Rotate( distForRotateFromCloseTrashBin);

	    /* Let the pantilt turn. */
#ifdef UNIBONN
	    tcxSendMsg(SUNVIS, "SUNVIS_look_forward", NULL);
#endif
	    dropObjectInternalState = Wait_Until_Arm_In;
	}
	return NO_MORE_COLLISION_AVOIDANCE;
	    
    case (Wait_Until_Arm_In) :

	if ( armState == INSIDE) {
	    
	    /*---------------------------------------------*
	     * The arm is inside we can start all over again.
	     *---------------------------------------------*/

	    return quitDropObjectState();
	}
       return NO_MORE_COLLISION_AVOIDANCE;
    }
    return NO_MORE_COLLISION_AVOIDANCE;
}




/***************************************************************************
 * Start / Quit / Failure of LOOK FOR TRASH BIN
 ***************************************************************************/

BOOLEAN
startLookForTrashBinState()
{
#ifdef VERBOSE
  fprintf( stderr, "************ START TO LOOK FOR TRASH BIN *************\n");
#endif
    
  stateTransitionSignal();
 
 /* Inform the collision avoidance to walk around randomly. */
  COLLI_SetMode( (double) ARM_OUT_RANDOM_MODE);
  COLLI_GoBackward();
  
  /* Let the robot start to move. The target point is not important. */
  COLLI_TranslateRelative( 10000.0, 0.0);
  
  sendRequestForFarObjects( EVERYTHING);
  
  lookForTrashBinInternalState = Wait_For_Coarse_Position;
  
  return CONTINUE_WITH_COLLISION_AVOIDANCE;
}

BOOLEAN
quitLookForTrashBinState()
{
    cleanUpState = APPROACH_FAR_TRASH_BIN;
    lookForTrashBinInternalState = Not_Active;

    return CONTINUE_WITH_COLLISION_AVOIDANCE;
}

BOOLEAN
lookForTrashBinStateFailed()
{
    failSignal();

    COLLI_StopCleaningUp();
    
    fprintf( stderr, "Failed to find a trash bin.\n");
    return CONTINUE_WITH_COLLISION_AVOIDANCE;
    
}

/***************************************************************************
 * Start / Quit / Failure of APPROACH FAR TRASH BIN
 ***************************************************************************/

BOOLEAN
startApproachFarTrashBinState()
{
#ifdef VERBOSE
  fprintf( stderr, "************ START TO APPROACH FAR TRASH BIN *************\n");
#endif
  
  stateTransitionSignal();
    
  /* We don't want to wait forever. */
  setTimer( MAX_WAITING_TIME_FOR_FAR_TRASH_BIN_REACHED);
  
  /* We take the first object. */
  COLLI_ApproachAbsolute( actualFarTrashBin.x,
			  actualFarTrashBin.y,
			  distForApproachFarTrashBin + ROB_RADIUS,
			  TRUE,   /* New target point. */
			  APPROACH_TRASH_BIN_MODE);
  
  approachFarTrashBinInternalState = Wait_Until_Trash_Bin_Reached;

  return CONTINUE_WITH_COLLISION_AVOIDANCE;
}

BOOLEAN
quitApproachFarTrashBinState()
{
    approachFarTrashBinInternalState = Not_Active;
    cleanUpState = GET_EXACT_TRASH_BIN_POSITION;

    return NO_MORE_COLLISION_AVOIDANCE;
}

BOOLEAN
approachFarTrashBinStateFailed()
{
  failSignal();
  
  SOUND_play_message(20);
  SOUND_talk_text("Failed");
  
  fprintf( stderr, "Failed to approach far trash bin.\n");

  /* Delete this trash bin. */
  removeFarTrashBin();

  /* Stop the robot and delete the target point. */
  BASE_TranslateHalt();
  BASE_RotateHalt();
  target_flag = FALSE;

  /* Reset the states. */
  approachFarTrashBinInternalState = Not_Active;
  cleanUpState = LOOK_FOR_TRASH_BIN;
  
  /* Look for another trash bin. */
  return startLookForTrashBinState();
}

/***************************************************************************
 * Start / Quit / Failure of GET EXACT TRASH BIN POS
 ***************************************************************************/

BOOLEAN
startGetExactTrashBinPositionState()
{
#ifdef VERBOSE
  fprintf( stderr, "************ START TO GET EXACT TRASH BIN POS *************\n");
#endif

  stateTransitionSignal();
    
  sendRequestForExactPosition( TRASH_BIN);

  /* We don't want to wait forever. */
  setTimer( MAX_WAITING_TIME_FOR_VISION);
  
  getExactTrashBinPositionInternalState = Wait_For_Exact_Position;
  
  return NO_MORE_COLLISION_AVOIDANCE;
}

BOOLEAN
quitGetExactTrashBinPositionState() 
{
    /* Got the information. Change state to drop
     * the object and copy the trash bin to the
     * next far trash bin. */
    
    numberOfCloseTrashBins = 0;
    numberOfFarTrashBins = 1;
    farTrashBins[0].x = actualCloseTrashBin.x;
    farTrashBins[0].y = actualCloseTrashBin.y;
    actualFarTrashBin = actualCloseTrashBin;
    
    getExactTrashBinPositionInternalState = Not_Active;
    cleanUpState = APPROACH_CLOSE_TRASH_BIN;

    return NO_MORE_COLLISION_AVOIDANCE;
}

BOOLEAN
getExactTrashBinPositionStateFailed()
{
    failSignal();

    removeFarTrashBin();
    
    fprintf( stderr, "Failed to get exact position. Start to look for");
    fprintf( stderr, " another trash bin.\n");

    getExactTrashBinPositionInternalState = Not_Active;
    cleanUpState =  LOOK_FOR_TRASH_BIN;

    return NO_MORE_COLLISION_AVOIDANCE;
}

/***************************************************************************
 * Start / Quit / Failure of APPROACH CLOSE TRASH BIN
 ***************************************************************************/

BOOLEAN
startApproachCloseTrashBinState( Point rPos, float rRot)
{
#ifdef VERBOSE
  fprintf( stderr, "************ START TO APPROACH CLOSE TRASH BIN *************\n");
#endif
  
  stateTransitionSignal();
    
  COLLI_turnToPoint( rPos, rRot,
		     actualCloseTrashBin,
		     rotateToCloseTrashBinVelocity);
  
  approachCloseTrashBinInternalState = Wait_Until_Rotation_Finished;
  
  return NO_MORE_COLLISION_AVOIDANCE;
}

BOOLEAN
quitApproachCloseTrashBinState()
{
    fprintf(stderr, "Facing the trash bin.\n");
    approachCloseTrashBinInternalState = Not_Active;
    cleanUpState = DROP_OBJECT;

    return NO_MORE_COLLISION_AVOIDANCE;}


BOOLEAN
approachCloseTrashBinStateFailed()
{
    failSignal();

    removeFarTrashBin();
    
    fprintf( stderr, "Something blocks the way. Start looking for");
    fprintf( stderr, " another trash bin.\n");


    approachCloseTrashBinInternalState = Not_Active;
    cleanUpState = LOOK_FOR_TRASH_BIN;

    return NO_MORE_COLLISION_AVOIDANCE;
}

/***************************************************************************
 * Start / Quit / Failure of DROP OBJECT
 ***************************************************************************/

BOOLEAN
startDropObjectState()
{
#ifdef VERBOSE
    fprintf( stderr, "************ START TO DROP THE OBJECT *************\n");
#endif
    stateTransitionSignal();
    
    ARM_dropObject();
    
    dropObjectInternalState = Wait_Until_Object_Dropped;

    return NO_MORE_COLLISION_AVOIDANCE;
}

BOOLEAN
quitDropObjectState()
{
    COLLI_GoForward();
    COLLI_StartCleaningUp();

    return CONTINUE_WITH_COLLISION_AVOIDANCE;
}

BOOLEAN
dropObjectStateFailed()
{
    failSignal();

    removeFarTrashBin();
    
    fprintf( stderr, "Failed to drop object. Move arm inside.\n");

    /* Move the arm inside and stop. */
    ARM_moveIn();

    COLLI_StopCleaningUp();
    
    return NO_MORE_COLLISION_AVOIDANCE;
}






#endif
