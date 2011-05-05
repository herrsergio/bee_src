
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

/**********************************************************************/
/* The next functions communicate with the arm module. */
/**********************************************************************/
void ARM_moveToGround(void)
{
  if (use_arm) {
    tcxSendMsg(ARM, "ARM_pickup_at_ground_query", NULL);
    SOUND_talk_text("I pick it up."); 
    SOUND_play_message(16);
  }
  else {
    /* We ignore this command at the first time. The second
     * time we send the successful execution.
     * Second call should be performed by BASE.
     */
    static BOOLEAN firstTime = TRUE;
    if (firstTime) {
      /* SOUND_talk_text("I pick it up."); */
      firstTime = FALSE;
    }
    else {
      ARM_moveToGroundReady( TRUE);
      firstTime = TRUE;
    }
  }
}


/**********************************************************************/
void ARM_liftObject(void)
{
  if (use_arm) {
    tcxSendMsg(ARM, "ARM_lift_object_query", NULL);
    SOUND_talk_text("Got it."); 
    SOUND_play_message(9);
  }
  else {
    /* We ignore this command at the first time. The second
     * time we send the successful execution.
     * Second call should be performed by BASE.
     */
    static BOOLEAN firstTime = TRUE;
    if (firstTime) {
      /* SOUND_talk_text("Got it."); */
      firstTime = FALSE;
    }
    else {
      ARM_liftObjectReady( TRUE);
      firstTime = TRUE;
    }
  }
}

/**********************************************************************/
void ARM_dropObject(void)
{
  if (use_arm) {
    tcxSendMsg(ARM, "ARM_drop_object_query", NULL);
    SOUND_talk_text("Thank you."); 
    SOUND_play_message(19);
  }
  else {
    /* We ignore this command at the first time. The second
     * time we send the successful execution.
     * Second call should be performed by BASE.
     */
    static BOOLEAN firstTime = TRUE;
    if (firstTime) {
      /* SOUND_talk_text("Thank you."); */
      firstTime = FALSE;
    }
    else {
      ARM_dropObjectReady( TRUE);
      firstTime = TRUE;
    }
  }
}

/**********************************************************************/
void ARM_moveIn(void)
{
  if (use_arm) {
    tcxSendMsg(ARM, "ARM_move_in_query", NULL);
    SOUND_talk_text("That's it."); 
    SOUND_play_message(3);
  }
  else {
    /* We ignore this command at the first time. The second
     * time we send the successful execution.
     * Second call should be performed by BASE.
     */
    static BOOLEAN firstTime = TRUE;
    if (firstTime) {
      /* SOUND_talk_text("That's it."); */
      firstTime = FALSE;
    }
    else {
      ARM_moveInReady( TRUE);
      firstTime = TRUE;
    }
  }
}

/**********************************************************************
 *  The next functions react to successful execution of arm commands.
 **********************************************************************/

/* The arm is down and we translate the last centimeters. */
void ARM_moveToGroundReady( int success )
{
  fprintf( stderr,"Pickup ready (%d)!\n", success);

  armState = READY_TO_GRIP;
  COLLI_GoBackward();
}

/* The object is lifted. */
void ARM_liftObjectReady(int success)
{
  fprintf( stderr,"Lift ready (%d)!\n", success);

  armState = OBJECT_LIFTED;
}

/* The object is dropped. */
void ARM_dropObjectReady(int success)
{
  fprintf( stderr,"Drop ready (%d)!\n", success);

  armState = OBJECT_DROPPED;
}

/* The arm is inside agein. */
void ARM_moveInReady(int success)
{
  fprintf( stderr,"Move in ready (%d)!\n", success);

  armState = INSIDE;
  COLLI_GoForward();
}




#endif
