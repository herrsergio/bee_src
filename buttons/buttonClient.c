
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
#ifndef lint
static char rcsid[] =
"$Id: buttonClient.c,v 1.3 1998/01/15 00:09:56 swa Exp $";
static void *const use_rcsid = ((void)&use_rcsid, (void)&rcsid, 0);
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <ctype.h>
#include <sys/time.h>

#include <tcx.h>
#include <tcxP.h>
#include <rai.h>

#define TCX_define_variables
#define DEFINE_REPLY_HANDLERS
#include <BUTTONS-messages.h>

#include <buttonClient.h>
#include <raiClient.h>

/*
 * This should include hostname and
 * PID or something like that
 */

#define BUTTONS_CLIENT_NAME "buttonclient"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


struct timeval BUTTONS_last_send_message_time = {0, 0};

static int status_subscription = 0;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

buttonStatusCallbackType userButtonFcn=NULL;

void registerButtonStatusCallback(buttonStatusCallbackType fcn)
{
  userButtonFcn=fcn;
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void BUTTONS_status_reply_handler( TCX_REF_PTR              ref,
				  BUTTONS_status_reply_ptr data )
{
  buttonStatusType info;

#ifdef TCX_DEBUG
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  info.red_light_status               = data->red_light_status;
  info.yellow_light_status            = data->yellow_light_status;
  info.green_light_status             = data->green_light_status;
  info.blue_light_status              = data->blue_light_status;
  info.left_kill_switch_light_status  = data->left_kill_switch_light_status;
  info.right_kill_switch_light_status = data->right_kill_switch_light_status;
  info.red_button_pressed             = data->red_button_pressed;
  info.red_button_changed             = data->red_button_changed;
  info.yellow_button_pressed          = data->yellow_button_pressed;
  info.yellow_button_changed          = data->yellow_button_changed;
  info.green_button_pressed           = data->green_button_pressed;
  info.green_button_changed           = data->green_button_changed;
  info.blue_button_pressed            = data->blue_button_pressed;
  info.blue_button_changed            = data->blue_button_changed;

  if ( userButtonFcn != NULL ) {
    userButtonFcn( &info );
  } else {
    fprintf( stderr,"%s: no callback registered for %s from %s\n",
	     __FILE__, __FUNCTION__, tcxModuleName(ref->module) );
  }

  tcxFree("BUTTONS_status_reply", data );
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void buttonSetButtons( int red_light_status,
		       int yellow_light_status,
		       int green_light_status,
		       int blue_light_status,
		       int left_kill_switch_light_status,
		       int right_kill_switch_light_status )
{
  BUTTONS_set_lights_type lights;

#ifdef TCX_DEBUG
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if ( buttonConnected  ) {
    
    lights.red_light_status              = red_light_status;
    lights.yellow_light_status           = yellow_light_status;
    lights.green_light_status            = green_light_status;
    lights.blue_light_status             = blue_light_status;
    lights.left_kill_switch_light_status = left_kill_switch_light_status;
    lights.right_kill_switch_light_status= right_kill_switch_light_status;
    tcxSendMsg(BUTTONS, "BUTTONS_set_lights", &lights );

  } else {
    fprintf(stderr,
	    "%s(%s): buttonServer is not connected. \n", __FILE__, __FUNCTION__);
    buttonConnect( 0 );
  }

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void buttonSetButton( int button, int status )
{
  BUTTONS_setButton_type data;

#ifdef TCX_DEBUG
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if (buttonConnected) {

    if ( button == BUTTON_LEFT_KILL ) {
    } else if ( button == BUTTON_RED ) {
    } else if ( button == BUTTON_YELLOW ) {
    } else if ( button == BUTTON_GREEN ) {
    } else if ( button == BUTTON_BLUE ) {
    } else if ( button == BUTTON_RIGHT_KILL ) {
    } else {
      fprintf( stderr,"%s: Oops, button %d does not exist.\n",
	       __FILE__, button );
      return;
    }
    
    data.num    = button;
    data.status = status;
    
    tcxSendMsg(BUTTONS, "BUTTONS_setButton", &data );

  } else {
    fprintf(stderr,
	    "%s(%s): buttonServer is not connected. \n", __FILE__, __FUNCTION__);
    buttonConnect( 0 );
  }

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void buttonStartCuteThing( ) {

#ifdef TCX_DEBUG
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if (buttonConnected) {
    tcxSendMsg(BUTTONS, "BUTTONS_effect", NULL );
  } else {
    fprintf(stderr,
	    "%s(%s): buttonServer is not connected. \n", __FILE__, __FUNCTION__);
    buttonConnect( 0 );
  }

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void buttonRequestStatus( )
{
 
#ifdef TCX_DEBUG
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if (buttonConnected){
    tcxSendMsg(BUTTONS, "BUTTONS_status_query", NULL );
  } else {
    fprintf(stderr,
	    "%s(%s): buttonServer is not connected. \n", __FILE__, __FUNCTION__);
    buttonConnect( 0 );
  }

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void button_subscribe()
{
  BUTTONS_register_auto_update_type subscribe;

#ifdef TCX_DEBUG
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if (buttonConnected){
    subscribe.status        = status_subscription;
    tcxSendMsg(BUTTONS, "BUTTONS_register_auto_update", &subscribe );
  } else {
    fprintf(stderr,
	    "%s(%s): buttonServer is not connected. \n", __FILE__, __FUNCTION__);
    buttonConnect( 0 );
  }

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void buttonSubscribeStatus(int number)
{
  status_subscription = number;

#ifdef TCX_DEBUG
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  button_subscribe();
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* ---------------------------------------------------------
 *
 * hook up to buttonServer. if buttonServer dies, try reconnecting 
 * every three seconds
 *
 * --------------------------------------------------------*/
int
buttonConnect( int wait_till_established )
{
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference;

#if ( defined(G_DEBUG_TCX) )
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);

  if (time_difference < 3.0)
    return -1;

  if ( wait_till_established ) { /* 1 */

    fprintf(stderr, "ButtonClient: Connecting to Button server...\n");
    BUTTONS = tcxConnectModule(TCX_BUTTONS_MODULE_NAME);
    buttonConnected = 1;
    fprintf(stderr, "ButtonClient: Connected.\n");

  } else {			/* 0 */

    if ( buttonConnected == 0 || !BUTTONS ) { /* doppelt haelt besser */
      fprintf(stderr, "ButtonClient: Connecting to Button server...\n");
      BUTTONS  = tcxConnectOptional(TCX_BUTTONS_MODULE_NAME);
      if( BUTTONS ) {
	buttonConnected = 1;
	button_subscribe();  
	fprintf(stderr, "ButtonClient: Connected.\n");
      } else {
	buttonConnected = 0;
      }
    }

  }

  last_time.tv_sec  = this_time.tv_sec;
  last_time.tv_usec = this_time.tv_usec;

  return 0;
}

/* old function -- not in use anymore */
int _buttonConnect(int wait_till_established)
{

#ifdef TCX_DEBUG
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if (wait_till_established){
    fprintf(stderr, "ButtonClient: Connecting to Button Server...\n");
    BUTTONS = tcxConnectModule(TCX_BUTTONS_MODULE_NAME);
  }
  else{
    BUTTONS = tcxConnectOptional(TCX_BUTTONS_MODULE_NAME);
  }

  if (BUTTONS){
    button_subscribe();  
    
    buttonConnected = 1;

    fprintf(stderr, "ButtonClient: Connected.\n");
    return 1;
  }
  else{
    buttonConnected = 0;
    return 0;
  }
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void buttonRegister()
{
  int numberOfMessages;
  int numberOfHandlers;


  TCX_REG_MSG_TYPE messages[] = { BUTTONS_messages }; 
  
  numberOfMessages = sizeof(messages)/sizeof(TCX_REG_MSG_TYPE); 
  numberOfHandlers = 
    sizeof(BUTTONS_reply_handler_array)/sizeof(TCX_REG_HND_TYPE); 

  registerInterface("", numberOfMessages, messages,
		    numberOfHandlers, BUTTONS_reply_handler_array);
}

/*
 * $Log: buttonClient.c,v $
 * Revision 1.3  1998/01/15 00:09:56  swa
 * buttonServer now handles restarts.
 *
 * Revision 1.2  1997/07/04 17:26:03  swa
 * Added lib support for buttons. Renamed the executable buttonServer to be
 * more consistent. No root permissions are necessary to run the buttonServer.
 * Renamed tons of things to be backward-compatible.
 *
 * Revision 1.1  1997/06/29 21:58:27  thrun
 * Added a ton of files for a client/server button thing.                (swa)
 *
 */
