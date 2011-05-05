
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
#ifndef BUTTON_H
#define BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

#define BUTTON_LEFT_KILL    0
#define BUTTON_RED          1
#define BUTTON_YELLOW       2
#define BUTTON_GREEN        3
#define BUTTON_BLUE         4
#define BUTTON_RIGHT_KILL   5

/**** BUTTON data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/

/*
 * The stati and commands for the lights
 */

#define BUTTON_LIGHT_STATUS_OFF                     0 /* always off */
#define BUTTON_LIGHT_STATUS_ON                      1 /* always on */
#define BUTTON_LIGHT_STATUS_FLASHING                2 /* always flashing */
#define BUTTON_LIGHT_STATUS_FLASHING_TILL_PRESSED   3 /* flashing, until
						       * someone presses the 
						       * button, then off */
#define BUTTON_LIGHT_STATUS_ON_TILL_PRESSED         4 /* on, until
						       * someone presses the 
						       * button, then off */
#define BUTTON_LIGHT_STATUS_OFF_TILL_PRESSED        5 /* off, until
						       * someone presses the 
						       * button, then oon */
#define BUTTON_LIGHT_STATUS_TOGGLE_ON               6 /* toggles upon button
						       * press, currently on */
#define BUTTON_LIGHT_STATUS_TOGGLE_OFF              7 /* toggles upon button
						       * press, currently off*/
#define BUTTON_LIGHT_STATUS_DONT_CHANGE             8 /* only command param:
						       * TCX-message will not
						       * change the status */

#define BUTTON_UNPRESSED     0	/* button not pressed */
#define BUTTON_PRESSED       1	/* button pressed */

  /* ====================================================================== *
   *
   * PART 1:   Procedures concerned with registration/connection
   *
   * ====================================================================== */


void buttonRegister();

int buttonConnect(int wait_till_established); /* if parameter is 1, then
					       * this will wait until
					       * connection has been 
					       * established */

void tcxRegisterCloseHnd(void (*closeHnd)());


int buttonConnected;		/* 1, if there is a connection to *
				 * the button server, 0 if not    */


  /* ====================================================================== *
   *
   * PART 2:   Procedures and data structures 
   *           concerned with the transmission of button status
   *
   * ====================================================================== */


typedef struct buttonStatusType {
  int red_light_status;
  int yellow_light_status;
  int green_light_status;
  int blue_light_status;
  int left_kill_switch_light_status;
  int right_kill_switch_light_status;
  
  int red_button_pressed;
  int red_button_changed;

  int yellow_button_pressed;
  int yellow_button_changed;

  int green_button_pressed;
  int green_button_changed;

  int blue_button_pressed;
  int blue_button_changed;

} buttonStatusType;
  
typedef int (*buttonStatusCallbackType) (buttonStatusType *data);

void registerButtonStatusCallback( buttonStatusCallbackType fcn );

void buttonSetButtons( int red_light_status,
		       int yellow_light_status,
		       int green_light_status,
		       int blue_light_status,
		       int left_kill_switch_light_status,
		       int right_kill_switch_light_status );

void buttonSetButton( int button, int status );

void buttonStartCuteThing( );

void buttonRequestStatus( );

void buttonSubscribeStatus( int number );

#ifdef __cplusplus
}
#endif

#endif /* BUTTON_H */

/*
 * $Log: buttonClient.h,v $
 * Revision 1.3  1998/01/15 00:09:56  swa
 * buttonServer now handles restarts.
 *
 * Revision 1.2  1997/07/30 02:51:22  thrun
 * Oops. Something went wrong.
 * This version has more consistent status update, and a message
 * for simulating button presses
 *
 * Revision 1.1  1997/06/29 21:58:28  thrun
 * Added a ton of files for a client/server button thing.                (swa)
 *
 */
