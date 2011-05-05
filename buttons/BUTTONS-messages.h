
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




#ifndef BUTTONS_messages_defined
#define BUTTONS_messages_defined

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* 
   Notice: when including this file, you need to have the flag

   TCX_define_variables

   be defined exactly once. This will allocate memory for the
   module pointer and the message arrays.
*/

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** TCX module name - this is a unique identifier for TCX *******/

#define TCX_BUTTONS_MODULE_NAME "BUTTONS"

void tcxRegisterCloseHnd(void (*closeHnd)());

#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR BUTTONS;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR BUTTONS;	/* otherwise: reference */

#endif

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

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

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define BUTTONS_effect_format NULL

/*
 * set the lights
 */

typedef struct {
  int red_light_status;
  int yellow_light_status;
  int green_light_status;
  int blue_light_status;
  int left_kill_switch_light_status;
  int right_kill_switch_light_status;
} BUTTONS_set_lights_type, *BUTTONS_set_lights_ptr;
#define BUTTONS_set_lights_format "{int, int, int, int, int, int}"

/*
 * ask for a status report (not necessary if you subscribed to it)
 */

#define BUTTONS_status_query_format NULL

/*
 * reply of the status report
 */

typedef struct {
  int red_light_status;
  int yellow_light_status;
  int green_light_status;
  int blue_light_status;
  int left_kill_switch_light_status;
  int right_kill_switch_light_status;
  int red_button_changed;
  int red_button_pressed;
  int yellow_button_pressed;
  int yellow_button_changed;
  int green_button_pressed;
  int green_button_changed;
  int blue_button_pressed;
  int blue_button_changed;
} BUTTONS_status_reply_type, *BUTTONS_status_reply_ptr;
#define BUTTONS_status_reply_format "{int, int, int, int, int, int, int, int, int, int, int, int, int, int}"

typedef struct {
  int status;			/* 0=don't send, n>0 send every n-th frame */
} BUTTONS_register_auto_update_type, *BUTTONS_register_auto_update_ptr;
#define BUTTONS_register_auto_update_format "{int}"

typedef struct {
  int num;
  int status;
} BUTTONS_setButton_type, *BUTTONS_setButton_ptr;
#define BUTTONS_setButton_format "{int, int}"

#define BUTTONS_simulate_button_press_format "int"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** BUTTON commands - these are the commands/queries understood by BUTTON ****/


#ifdef TCX_define_variables		/* do this exactly once! */

#define BUTTONS_messages \
  {"BUTTONS_register_auto_update",   BUTTONS_register_auto_update_format},\
  {"BUTTONS_status_query",           BUTTONS_status_query_format},\
  {"BUTTONS_status_reply",           BUTTONS_status_reply_format},\
  {"BUTTONS_effect",                 BUTTONS_effect_format},\
  {"BUTTONS_setButton",              BUTTONS_setButton_format},\
  {"BUTTONS_set_lights",             BUTTONS_set_lights_format},\
  {"BUTTONS_simulate_button_press",  BUTTONS_simulate_button_press_format}
#endif

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_REPLY_HANDLERS

/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with BUTTON ******/

/******* (a) Procedure headers ******/

void BUTTONS_status_reply_handler(TCX_REF_PTR              ref,
				  BUTTONS_status_reply_ptr data);

/******* (b) Handler array ******/

TCX_REG_HND_TYPE BUTTONS_reply_handler_array[] = {
  {"BUTTONS_status_reply", "BUTTONS_status_reply_handler",
    (void (*)()) BUTTONS_status_reply_handler, TCX_RECV_ALL, NULL}
};


#endif

#endif
