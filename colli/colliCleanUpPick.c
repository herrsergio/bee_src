
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

/* Different internal states of the main states. */
int randomWalkInternalState = Not_Active;
int approachFarObjectInternalState = Not_Active;
int getExactObjectPositionInternalState = Not_Active;
int approachCloseObjectInternalState = Not_Active;
int pickUpObjectInternalState = Not_Active;

Object   farObjects[MAX_NUMBER_OF_OBJECTS];
Object   closeObjects[MAX_NUMBER_OF_OBJECTS];

int numberOfFarObjects = 0;
int numberOfCloseObjects = 0;


/* Velocities */
const float rotateToCloseObjectVelocity = 30.0;
const float rotateToFarObjectVelocity = 30.0;
const float translateToCloseObjectVelocity = 35.0;
const float translateToFarObjectVelocity = 35.0;

/* How close does the robot have to approach? */
const float distForApproachFarObject = 150.0;
const float brakeDist = 50.0;
const float distForApproachCloseObject = 30.0;

/* Gripper at ground and open. Move some centimeters to grip the object. */
const float distForApproachLastCentimeters = 10.0;
const float velocityForApproachLastCentimeters = 20.0;

/* Gives the actual state of the clean up sequence. */
int cleanUpState = NOT_RUNNING;

Point actualFarObject;
Point actualCloseObject;

/**********************************************************************
 **********************************************************************
 *           Functions for the different states                       *
 **********************************************************************
 **********************************************************************/

/**********************************************************************
 * Starts the robot to move around randomly and informs the vision to
 * look for objects.
 * Ends when vision found an object.
 **********************************************************************/
BOOLEAN
randomWalkState( Point rPos, float rRot)
{
  switch (randomWalkInternalState) {
    
  case (Not_Active) :
    
    /*---------------------------------------------*
     * Start the random walk.                      *
     *---------------------------------------------*/
    
    return startRandomWalkState();
  
  case (Wait_For_Coarse_Position) :
    
    /*---------------------------------------------*
     * Just do nothing but wait for the vision to
     * find an object.
     *---------------------------------------------*/
    
    if ( numberOfFarObjects > 0) {
      
      /*---------------------------------------------*
       * The vision found an object. Quit this state
       * and change to the next one.
       *---------------------------------------------*/
      SOUND_play_message(14);
      SOUND_talk_text("Saw an object");
      return quitRandomWalkState( rPos, rRot);
    }
    else
      return CONTINUE_WITH_COLLISION_AVOIDANCE;
  }
  return CONTINUE_WITH_COLLISION_AVOIDANCE;
}


/**********************************************************************
 * Starts the robot to go to an object.
 * Ends when the robot faces the object.
 **********************************************************************/
BOOLEAN
approachFarObjectState( Point rPos, float rRot)
{ 
  switch (approachFarObjectInternalState) {
    
  case (Not_Active) :
    
    /*---------------------------------------------*
     * Start to approach the object.
     *---------------------------------------------*/
    
    return startApproachFarObjectState( rPos, rRot);
    
  case (Wait_Until_Object_Reached) :
    
    /*---------------------------------------------*
     * Just do nothing but wait for the robot to
     * get close enough to the object.
     *---------------------------------------------*/
    
    if ( rwi_base.collision_state.target_reached == TRUE) {
      
      /*---------------------------------------------*
       * The robot is close enough. Turn to the object.
       *---------------------------------------------*/

#ifdef VERBOSE
      fprintf( stderr, "Target reached. Turn to it.\n");
#endif
      COLLI_turnToPoint( rPos, rRot,
			actualFarObject,
			rotateToFarObjectVelocity);
      
      approachFarObjectInternalState = Wait_Until_Rotation_Finished;

      return NO_MORE_COLLISION_AVOIDANCE;
    }
    else {
       if ( stillInTimeInterval())
	  /* The robot still has to approach the target. */
	  return CONTINUE_WITH_COLLISION_AVOIDANCE;
       else {
	  /* That's too long. Forget the target. */
	  fprintf( stderr, "Can't reach the target. ");
	  fprintf( stderr, "Start to look for other objects.\n");
	  return approachFarObjectStateFailed();
       }
    }
    
 case (Wait_Until_Rotation_Finished) :
    
    /*---------------------------------------------*
     * Just do nothing but wait for the robot to
     * face the object.
     *---------------------------------------------*/
    
    if ( ! stillInRotation()) {
      
      /*---------------------------------------------*
       * Rotation finished. Get the right distance to it.
       *---------------------------------------------*/
      
      approachFarObjectInternalState = Wait_Until_Translation_Finished;
      
      return NO_MORE_COLLISION_AVOIDANCE;
    }
    
    return NO_MORE_COLLISION_AVOIDANCE;
    
  case (Wait_Until_Translation_Finished) :
    
    switch (COLLI_translateToPoint( rPos,
				   rRot,
				   actualFarObject,
				   distForApproachFarObject,
				   translateToFarObjectVelocity,
				   ROB_RADIUS,
				   TRUE)) {
      
    case (POINT_IS_NOT_YET_REACHED) :
      /* Nothing to do but wait. */
      return NO_MORE_COLLISION_AVOIDANCE;
    case (POINT_IS_REACHED) :
      /* Done. Start picking up. */
      return quitApproachFarObjectState();
    case (GAVE_UP) :
      {
	fprintf( stderr, "Something blocks the way. ");
	fprintf( stderr, "Start looking for other objects.\n");
	return approachFarObjectStateFailed();
      }
    }
    
  }
  return NO_MORE_COLLISION_AVOIDANCE;
}


/**********************************************************************
 * Starts when the robot faces an object. Sends the vision the information
 * to get the exact position of the object.
 * Ends when the position is known.
 **********************************************************************/
BOOLEAN
getExactObjectPositionState( Point rPos, float rRot)
{
   switch (getExactObjectPositionInternalState) {
		    
   case (Not_Active) :

      /*---------------------------------------------*
       * Send the vision the information to take
       * a closer look at the position of the object.
       *---------------------------------------------*/

     return startGetExactObjectPositionState();
      
   case (Wait_For_Exact_Position) :

      /*---------------------------------------------*
       * Just wait. If interval too long we start again.
       *---------------------------------------------*/
      
       if ( ! stillInTimeInterval()) {

	   fprintf(stderr, "Sorry, that's too long!\n");
	   SOUND_talk_text("That's too long. - Start again.");
	   return getExactObjectPositionStateFailed();
       }
	 
      if ( numberOfCloseObjects > 0) {
	 
	 /*---------------------------------------------*
	  * Got the information. Change state to pick up
	  * the object and delete the close object.
	  *---------------------------------------------*/

	  return quitGetExactObjectPositionState();
      }
      return NO_MORE_COLLISION_AVOIDANCE;
   }
   return NO_MORE_COLLISION_AVOIDANCE;
}


/**********************************************************************
 * Starts when the robot has the exact position of an object.
 * Ends when the robot faces this object.
 **********************************************************************/
BOOLEAN
approachCloseObjectState( Point rPos, float rRot)
{
  switch (approachCloseObjectInternalState) {
    
  case (Not_Active) :
    
    /*---------------------------------------------*
     * First rotate to the object.
     *---------------------------------------------*/

    return startApproachCloseObjectState( rPos, rRot);
    
  case (Wait_Until_Rotation_Finished) :
    
    if ( ! stillInRotation()) {
      
      /*---------------------------------------------*
       * Done. Now the robot can start to translate.
       *---------------------------------------------*/
      
      fprintf(stderr, "Rotation finished. Start to translate.\n");
      approachCloseObjectInternalState = Wait_Until_Translation_Finished;
    }
    
    return NO_MORE_COLLISION_AVOIDANCE;
    
  case (Wait_Until_Translation_Finished) :
	
	/*---------------------------------------------*
	 * The robot has to translate to the object.
	 * The function returns the current status of
	 * the translation.
	 *---------------------------------------------*/
	
	switch (COLLI_translateToPoint( rPos,
				       rRot,
				       actualCloseObject,
				       distForApproachCloseObject,
				       translateToCloseObjectVelocity,
				       ROB_RADIUS,
				       TRUE)) {
	case (POINT_IS_NOT_YET_REACHED) :
	  /* Nothing to do but wait. */
	  return NO_MORE_COLLISION_AVOIDANCE;
	case (POINT_IS_REACHED) :
	  /* Done. Start picking up. */
	  return quitApproachCloseObjectState();
	case (GAVE_UP) :
	    {
		fprintf( stderr, "Something blocks the way. ");
		fprintf( stderr, "Start looking for other objects.\n");
		return approachCloseObjectStateFailed();
	    }
	}
  }
  return NO_MORE_COLLISION_AVOIDANCE;
}

/**********************************************************************
 * Starts when the robot faces a close object.
 * Ends when the arm has lifted the object.
 **********************************************************************/
BOOLEAN
pickUpObjectState( Point rPos, float rRot)
{
   switch (pickUpObjectInternalState) {
      
   case (Not_Active) :

       /*---------------------------------------------*
	* First let the gripper move to the ground.
       *---------------------------------------------*/
     SOUND_play_message(16);
     SOUND_talk_text("Pick it up.");
       return startPickUpObjectState();

   case (Wait_Until_Arm_Down) :

       if ( armState == READY_TO_GRIP) {

	   /*---------------------------------------------*
	    * The gripper is down. The robot moves the last
	    * centimeters to the object.
	    *---------------------------------------------*/

	   fprintf( stderr, "Gripper down. Move last centimeters ...");

	   BASE_TranslateVelocity ( velocityForApproachLastCentimeters);
	   BASE_TranslateCollisionVelocity ( velocityForApproachLastCentimeters);
	   BASE_RotateVelocity ( 0.0);
	   BASE_RotateCollisionVelocity ( 0.0);
	   BASE_Translate( distForApproachLastCentimeters);
	   
	   pickUpObjectInternalState = Wait_Until_Translation_Finished;
       }
       return NO_MORE_COLLISION_AVOIDANCE;

   case (Wait_Until_Translation_Finished) :

       if ( ! stillInTranslation()) {

	 /*---------------------------------------------*
	  * Finished. Now let's grip the object. 
	  *---------------------------------------------*/
	 
	   fprintf( stderr, " and start to pick up.\n");
	   pickUpObjectInternalState = Wait_Until_Object_Lifted;

	   ARM_liftObject();
       }

       return NO_MORE_COLLISION_AVOIDANCE;

   case (Wait_Until_Object_Lifted) :       

       if  (armState == OBJECT_LIFTED) {
	   
	   /*---------------------------------------------*
	    * The object is picked up. Quit this state.
	    * The robot has to look for a trash bin.
	    *---------------------------------------------*/

	   return quitPickUpObjectState();
       }

       return NO_MORE_COLLISION_AVOIDANCE;
   }
   return NO_MORE_COLLISION_AVOIDANCE;
}




/***************************************************************************
 * Start / Quit / Failure of RANDOM WALK.
 ***************************************************************************/

BOOLEAN
startRandomWalkState()
{
#ifdef VERBOSE
  fprintf( stderr, "************ START RANDOM WALK *************\n");
#endif
  
  stateTransitionSignal();

  /* The robot should move around randomly. */
  COLLI_SetMode( (double) RANDOM_MODE);
  
  /* Let the robot start to move. The target point is not important. */
  COLLI_TranslateRelative( 10000.0, 0.0);
  
  /* Inform the vision to look for objects. */
  sendRequestForFarObjects( EVERYTHING);
      
  randomWalkInternalState = Wait_For_Coarse_Position;
  
  return CONTINUE_WITH_COLLISION_AVOIDANCE; 
}

BOOLEAN
quitRandomWalkState()
{
  cleanUpState = APPROACH_FAR_OBJECT;
  randomWalkInternalState = Not_Active;

  return CONTINUE_WITH_COLLISION_AVOIDANCE; 
}

BOOLEAN
randomWalkStateFailed()
{
  failSignal();
  
    /* This shouldn't happpen. */
    fprintf( stderr, "Failed to find an object.\n");
    
    COLLI_StopCleaningUp();
    return CONTINUE_WITH_COLLISION_AVOIDANCE; 
}

/***************************************************************************
 * Start / Quit / Failure of APPROACH FAR OBJECT.
 ***************************************************************************/

BOOLEAN
startApproachFarObjectState()
{
#ifdef VERBOSE
  fprintf( stderr, "************ START APPROACH FAR OBJECT *************\n");
#endif
  
  stateTransitionSignal();

  /* We don't want to wait forever. */
  setTimer( MAX_WAITING_TIME_FOR_FAR_OBJECT_REACHED);
  
  /* We take the first object. */
  COLLI_ApproachAbsolute( actualFarObject.x,
			  actualFarObject.y,
			  distForApproachFarObject + ROB_RADIUS + brakeDist,
			  TRUE,   /* New target point. */
			  APPROACH_OBJECT_MODE);
  
  approachFarObjectInternalState = Wait_Until_Object_Reached;

  return CONTINUE_WITH_COLLISION_AVOIDANCE;
}

BOOLEAN
quitApproachFarObjectState()
{
  approachFarObjectInternalState = Not_Active;
  cleanUpState = GET_EXACT_OBJECT_POSITION;
  
  return NO_MORE_COLLISION_AVOIDANCE;
}

BOOLEAN
approachFarObjectStateFailed()
{
  failSignal();
  
  SOUND_play_message(20);
  SOUND_talk_text("Failed");
  
  fprintf( stderr, "Failed to approach far object.\n");

  /* Delete this object. */
  removeFarObject();

  /* Stop the robot and delete the target point. */
  BASE_TranslateHalt();
  BASE_RotateHalt();
  target_flag = FALSE;
  
  COLLI_StartCleaningUp();
  
  return CONTINUE_WITH_COLLISION_AVOIDANCE;
}

/***************************************************************************
 * Start / Quit / Failure of GET EXACT POSITION.
 ***************************************************************************/

BOOLEAN
startGetExactObjectPositionState()
{
#ifdef VERBOSE
  fprintf( stderr, "************ START GET EXACT POS OBJECT *************\n");
#endif

  stateTransitionSignal();
    
  fprintf(stderr, "Send request to the vision module.\n");
  
  sendRequestForExactPosition( OBJECT);
  
  /* We don't want to wait forever. */
  setTimer( MAX_WAITING_TIME_FOR_VISION);
  
  getExactObjectPositionInternalState = Wait_For_Exact_Position;
  
  return NO_MORE_COLLISION_AVOIDANCE;
}

/*---------------------------------------------*
 * Got the information. Change state to pick up
 * the object and delete the close object.
 *---------------------------------------------*/
BOOLEAN
quitGetExactObjectPositionState()
{
  numberOfCloseObjects = 0;
  
  getExactObjectPositionInternalState = Not_Active;
  cleanUpState = APPROACH_CLOSE_OBJECT;

  return NO_MORE_COLLISION_AVOIDANCE;
}

BOOLEAN
getExactObjectPositionStateFailed()
{
  failSignal();

  removeFarObject();
  
  fprintf( stderr, "Failed to get exact position.\n");
  
  COLLI_StartCleaningUp();
  
  return CONTINUE_WITH_COLLISION_AVOIDANCE;
}

/***************************************************************************
 * Start / Quit / Failure of APPROACH CLOSE OBJECT
 ***************************************************************************/

BOOLEAN
startApproachCloseObjectState( Point rPos, float rRot)
{
#ifdef VERBOSE
  fprintf( stderr, "************ START APPROACH CLOSE OBJECT *************\n");
#endif
  
  stateTransitionSignal();
    
  COLLI_turnToPoint( rPos, rRot,
		    actualCloseObject,
		    rotateToCloseObjectVelocity);
  
  approachCloseObjectInternalState = Wait_Until_Rotation_Finished;
  
  return NO_MORE_COLLISION_AVOIDANCE;
}

BOOLEAN
quitApproachCloseObjectState()
{
    /* Done. Change to the next state. */
    approachCloseObjectInternalState = Not_Active;
    cleanUpState = PICKUP_OBJECT;
    return NO_MORE_COLLISION_AVOIDANCE;
}

BOOLEAN
approachCloseObjectStateFailed()
{
  failSignal();
  
  fprintf( stderr, "Failed to approach close.\n");

  removeFarObject();
  
  COLLI_StartCleaningUp();

  return NO_MORE_COLLISION_AVOIDANCE;
}

/***************************************************************************
 * Start / Quit / Failure of PICK UP
 ***************************************************************************/

BOOLEAN
startPickUpObjectState()
{
#ifdef VERBOSE
    fprintf( stderr, "************ START PICK UP *************\n");
#endif
    
    stateTransitionSignal();
    
    /* First let the gripper move to the ground. */
    ARM_moveToGround();
    
    pickUpObjectInternalState = Wait_Until_Arm_Down;
    
    return NO_MORE_COLLISION_AVOIDANCE;
}

BOOLEAN
quitPickUpObjectState()
{
  removeFarObject();
  
  pickUpObjectInternalState = Not_Active;
  cleanUpState = LOOK_FOR_TRASH_BIN;
  
  return NO_MORE_COLLISION_AVOIDANCE;
}

BOOLEAN
pickUpObjectStateFailed()
{
    failSignal();

    fprintf( stderr, "Failed to pick up an object.\n");

    /* Move the arm inside and start again. Same as if
     * the robot successfully dropped the object. */
    ARM_moveIn();

    removeFarObject();
    
    dropObjectInternalState = Wait_Until_Arm_In;
    cleanUpState = DROP_OBJECT;

    return NO_MORE_COLLISION_AVOIDANCE;
}

#endif
