
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
#include <stdlib.h>
#include <stdio.h>
#include <rai.h>

#include <bUtils.h>
#include "tcx.h"
#include "tcxP.h"

#define TCX_define_variables /* this makes sure variables are installed */
#include "BUTTONS-messages.h"

#include "buttonHandlers.h"
#include "button.h"
#include "buttonClient.h"
#include "beeSoftVersion.h"
#include "raiClient.h"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

TCX_REG_HND_TYPE BUTTONS_handler_array[] = {

  {"BUTTONS_register_auto_update", "BUTTONS_register_auto_update_handler",
     BUTTONS_register_auto_update_handler, TCX_RECV_ALL, NULL},

  {"BUTTONS_status_query", "BUTTONS_status_query_handler",
     BUTTONS_status_query_handler, TCX_RECV_ALL, NULL},

  {"BUTTONS_effect", "BUTTONS_effect_handler",
     BUTTONS_effect_handler, TCX_RECV_ALL, NULL},

  {"BUTTONS_setButton", "BUTTONS_setButton_handler",
     BUTTONS_setButton_handler, TCX_RECV_ALL, NULL},

  {"BUTTONS_set_lights", "BUTTONS_set_lights_handler",
     BUTTONS_set_lights_handler, TCX_RECV_ALL, NULL},

  {"BUTTONS_simulate_button_press", "BUTTONS_simulate_button_press_handler",
     BUTTONS_simulate_button_press_handler, TCX_RECV_ALL, NULL}

};

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/******* auto-reply stuff, data definitions ***************/

#define MAX_N_AUTO_UPDATE_MODULES 100

int n_auto_update_modules = 0; /* number of processes to whom  *
				* position should be forwarded *
				* automatically upon change    */

int n_auto_status_update_modules      = 0;

typedef struct{
  TCX_MODULE_PTR module;	/* pointer to TCX module */
  int            status;	/* >=1=subscribed to regular image updates */
  int            last_status;
} auto_update_type;

auto_update_type auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];

/************************************************************************
 *
 *   NAME:         count_auto_update_modules()
 *                 
 *   FUNCTION:     Counts, how many modules of the different types require
 *                 auto-updates
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/

void count_auto_update_modules()
{
  int i;

  n_auto_status_update_modules      = 0;

  for (i = 0; i < n_auto_status_update_modules; i++)
    if (auto_update_modules[i].status)
      n_auto_status_update_modules++;

}

/************************************************************************
 *
 *   NAME:         add_auto_update_module()
 *                 
 *   FUNCTION:     Adds a module to the list of modules that
 *                 get automatical status updates
 *                 
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *                 
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/
static int add_auto_update_module( TCX_MODULE_PTR                   module,
				   BUTTONS_register_auto_update_ptr data )
{
  int i;

  if (n_auto_update_modules >= MAX_N_AUTO_UPDATE_MODULES){
    fprintf(stderr, "ERROR: auto-module memory overflow. Request rejected.\n");
    return 0;
  }
  else
    for (i = 0; i < n_auto_update_modules; i++)
      if (auto_update_modules[i].module == module){
	fprintf(stderr, 
		"Module %s already known. Subscription modified: %d\n",
		tcxModuleName(module), data->status);
	auto_update_modules[i].status      = data->status; /* subsrc? */
	auto_update_modules[i].last_status = -1;
	count_auto_update_modules();
	return 1;
      }
  fprintf(stderr, "Add %s to auto-reply list: %d.\n",
	  tcxModuleName(module), data->status );
  auto_update_modules[n_auto_update_modules].module      = module; /* pointer*/
  auto_update_modules[n_auto_update_modules].status      = data->status; 
  auto_update_modules[n_auto_update_modules].last_status = -1;
  n_auto_update_modules++;
  count_auto_update_modules();
  return 1;
}

/************************************************************************
 *
 *   NAME:         remove_auto_update_module()
 *                 
 *   FUNCTION:     Attempts to remove a module from the list of 
 *                 modules that get automatical map updates 
 *                 
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *                 
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/
static int remove_auto_update_module(TCX_MODULE_PTR module)
{     
  int i, j, found = 0;;
  for (i = 0; i < n_auto_update_modules; i++) /* search for module */
    if (auto_update_modules[i].module == module){ /* if module found */
      fprintf(stderr, "Remove %s from auto-reply list.\n",
	      tcxModuleName(module));
      found++;
      n_auto_update_modules--;	/* remove that entry, one less now */
      for (j = i; j < n_auto_update_modules; j++){
	auto_update_modules[j].module = 
	  auto_update_modules[j+1].module; /* shift back */
	auto_update_modules[j].status = 
	  auto_update_modules[j+1].status; /* shift back */
	auto_update_modules[j].last_status = 
	  auto_update_modules[j+1].last_status; /* shift back */
      }
    }
  if (!found)
    fprintf(stderr, "(%s: no auto-replies removed)\n",
	    tcxModuleName(module));
  count_auto_update_modules();
  return found;
}

/************************************************************************
 *
 *   NAME:          BUTTONS_status_reply()
 *                 
 *   FUNCTION:      sends a button status report
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
void BUTTONS_send_status_to( TCX_MODULE_PTR module )
{

  tcxSendMsg( module, "BUTTONS_status_reply", &status);

}

/************************************************************************
 *
 *   NAME:         send_automatic_status_update
 *                 
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/
void send_automatic_status_update( )
{
  int i;

  for (i = 0; i < n_auto_update_modules; i++){
    if (auto_update_modules[i].status > 0){

      auto_update_modules[i].last_status++;
      if (auto_update_modules[i].last_status %
	  auto_update_modules[i].status == 0){
#ifdef TCX_DEBUG
	fprintf(stderr, "%s: Send status to %s.\n", __FILE__,
		tcxModuleName(auto_update_modules[i].module));
#endif
	auto_update_modules[i].last_status = 0;

	BUTTONS_send_status_to(auto_update_modules[i].module );
      }
    }
  }
}

/************************************************************************
 *
 *   NAME:         BUTTONS_register_auto_update_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
void BUTTONS_register_auto_update_handler(TCX_REF_PTR                      ref,
					  BUTTONS_register_auto_update_ptr data)
{

  add_auto_update_module(ref->module, data);

  if (data != NULL){
    tcxFree("BUTTONS_register_auto_update", data);
    data = NULL;
  }

}

/************************************************************************
 *
 *   NAME:         BUTTONS_set_lights_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
void BUTTONS_set_lights_handler(TCX_REF_PTR                ref,
				BUTTONS_set_lights_ptr     data)
{
  int i;

#ifdef TCX_DEBUG
  fprintf(stderr, "%s: Received a BUTTONS_set_lights message from %s.\n",
	  __FILE__,
	  tcxModuleName(ref->module));
#endif

  buttons_effect = 0;

  if (data->red_light_status >= 0 &&
      data->red_light_status  < BUTTON_LIGHT_STATUS_DONT_CHANGE)
    status.red_light_status   = data->red_light_status;

  if (data->yellow_light_status >= 0 &&
      data->yellow_light_status  < BUTTON_LIGHT_STATUS_DONT_CHANGE)
    status.yellow_light_status   = data->yellow_light_status;

  if (data->green_light_status >= 0 &&
      data->green_light_status  < BUTTON_LIGHT_STATUS_DONT_CHANGE)
    status.green_light_status   = data->green_light_status;

  if (data->blue_light_status >= 0 &&
      data->blue_light_status  < BUTTON_LIGHT_STATUS_DONT_CHANGE)
    status.blue_light_status   = data->blue_light_status;

  if (data->left_kill_switch_light_status >= 0 &&
      data->left_kill_switch_light_status  < BUTTON_LIGHT_STATUS_DONT_CHANGE)
    status.left_kill_switch_light_status = data->left_kill_switch_light_status;

  if (data->right_kill_switch_light_status >= 0 &&
      data->right_kill_switch_light_status  < BUTTON_LIGHT_STATUS_DONT_CHANGE)
    status.right_kill_switch_light_status = data->right_kill_switch_light_status;

  check_and_set_buttons();

  for (i = 0; i < 4; i++){
    setButton(i, 0);
    usleep(30000);
  }

  tcxFree("BUTTONS_set_lights_query", data);

#ifdef TCX_DEBUG
  fprintf(stderr, "%s: Send status to %s.\n", __FILE__, tcxModuleName(ref->module));
#endif

  send_automatic_status_update();
 
}

/************************************************************************
 *
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
void BUTTONS_setButton_handler( TCX_REF_PTR          ref,
			       BUTTONS_setButton_ptr data)
{

#ifdef TCX_DEBUG
  fprintf( stderr, "%s: Received a BUTTONB_setButton message from %s.\n",
	   __FILE__, tcxModuleName(ref->module));
  fprintf( stderr, "%s: data->button ....... : %d\n", __FILE__, data->num );
  fprintf( stderr, "%s: data->status ....... : %d\n", __FILE__, data->status );
#endif

  buttons_effect = 0;

  if ( data->num == BUTTON_RED ) {

    if (data->status >= 0 && data->status  < BUTTON_LIGHT_STATUS_DONT_CHANGE)
      status.red_light_status   = data->status;

    check_and_set_buttons();
    
    /*     setButton( data->num, status.red_light_status ); */

  } else if ( data->num == BUTTON_LEFT_KILL ) {

    if (data->status >= 0 && data->status  < BUTTON_LIGHT_STATUS_DONT_CHANGE)
      status.left_kill_switch_light_status = data->status;

    check_and_set_buttons();

    /*     setButton( data->num, status.left_kill_switch_light_status ); */
    
  } else if ( data->num == BUTTON_YELLOW ) {
    
    if (data->status >= 0 && data->status  < BUTTON_LIGHT_STATUS_DONT_CHANGE)
      status.yellow_light_status   = data->status;

    check_and_set_buttons();

    /*     setButton( data->num, status.yellow_light_status ); */
    
  } else if ( data->num == BUTTON_GREEN ) {

    if (data->status >= 0 && data->status  < BUTTON_LIGHT_STATUS_DONT_CHANGE)
      status.green_light_status   = data->status;

    check_and_set_buttons();

    /*     setButton( data->num, status.green_light_status ); */
    
  } else if ( data->num == BUTTON_BLUE ) {

    if (data->status >= 0 && data->status  < BUTTON_LIGHT_STATUS_DONT_CHANGE)
      status.blue_light_status   = data->status;
    
    check_and_set_buttons();
    
    /*     setButton( data->num, status.blue_light_status ); */
    
  } else if ( data->num == BUTTON_RIGHT_KILL ) {

    if (data->status >= 0 && data->status  < BUTTON_LIGHT_STATUS_DONT_CHANGE)
      status.right_kill_switch_light_status = data->status;

    check_and_set_buttons();
    
    /*     setButton( data->num, status.right_kill_switch_light_status ); */

  } else {
    /* ouch! */
  }  

  tcxFree("BUTTONS_setButton", data);

#ifdef TCX_DEBUG
  fprintf(stderr, "%s: Send status to %s.\n", __FILE__, tcxModuleName(ref->module));
#endif

  send_automatic_status_update();

}

/************************************************************************
 *
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
void 
BUTTONS_simulate_button_press_handler(TCX_REF_PTR          ref,
				      int *data)
{

  if (*data == BUTTON_RED){
    if (!status.red_button_pressed){
      status.red_button_pressed = 1;
      status.red_button_changed = 1;
      fprintf(stderr, "RED\n");
      send_automatic_status_update();
      usleep(500000);
      status.red_button_pressed = 0;
      status.red_button_changed = 1;
      send_automatic_status_update();
    }
  }

  else if (*data == BUTTON_YELLOW){
    if (!status.yellow_button_pressed){
      status.yellow_button_pressed = 1;
      status.yellow_button_changed = 1;
      fprintf(stderr, "YELLOW\n");
      send_automatic_status_update();
      usleep(500000);
      status.yellow_button_pressed = 0;
      status.yellow_button_changed = 1;
      send_automatic_status_update();
    }
  }

  else if (*data == BUTTON_GREEN){
    if (!status.green_button_pressed){
      status.green_button_pressed = 1;
      status.green_button_changed = 1;
      fprintf(stderr, "GREEN\n");
      send_automatic_status_update();
      usleep(500000);
      status.green_button_pressed = 0;
      status.green_button_changed = 1;
      send_automatic_status_update();
    }
  }

  else if (*data == BUTTON_BLUE){
    if (!status.blue_button_pressed){
      status.blue_button_pressed = 1;
      status.blue_button_changed = 1;
      fprintf(stderr, "BLUE\n");
      send_automatic_status_update();
      usleep(500000);
      status.blue_button_pressed = 0;
      status.blue_button_changed = 1;
      send_automatic_status_update();
    }
  }


  tcxFree("BUTTONS_simulate_button_press", data);

}

/************************************************************************
 *
 *   NAME:         BUTTONS_status_query_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
void BUTTONS_status_query_handler( TCX_REF_PTR  ref )
{

#ifdef TCX_DEBUG
  fprintf( stderr, "%s: Received a BUTTONS_status_query message from %s.\n",
	   __FILE__, tcxModuleName(ref->module));
#endif

#ifdef TCX_DEBUG
  fprintf( stderr, "%s: Send status to %s.\n", 
	   __FILE__, tcxModuleName(ref->module));
#endif

  BUTTONS_send_status_to( ref->module );

}

/************************************************************************
 *
 *   NAME:         
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
void BUTTONS_effect_handler(TCX_REF_PTR ref )
{

#ifdef TCX_DEBUG
  fprintf( stderr, "%s: Received a BUTTONS_effect message from %s.\n",
	   __FILE__,tcxModuleName(ref->module));
#endif

  buttons_effect = 1;

}

/************************************************************************
 *
 *   NAME:         BUTTON_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
void BUTTONS_close_handler(char *name, TCX_MODULE_PTR module)
{

#ifdef TCX_DEBUG
  fprintf(stderr, "BUTTON: closed connection detected: %s\n", name);
#endif

  remove_auto_update_module(module);

  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    exit(0);
  }

}

/************************************************************************
 *
 *   NAME:         BUTTONS_initialize_tcx
 *                 
 *   FUNCTION:     general initialization routine - must be called before
 *                 anything else. Returns error value (0=success, 1=error)
 *                 
 *   RETURN-VALUE: Returns error value (0=success, 1=error)
 *                 
 ************************************************************************/

int BUTTONS_initialize_tcx() {

  TCX_REG_MSG_TYPE TCX_message_array[] = {
    BUTTONS_messages
  };
  
  fprintf(stderr, "Connecting to TCX...");
  
  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize(TCX_BUTTONS_MODULE_NAME, tcxMachine);
  check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
		       BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
		       NULL, 0);
  
/*     check_version_number(libezx_major, libezx_minor, */
/* 			 libezx_robot_type, libezx_date, */
/* 			 "libezx", 0); */
/*     check_version_number(librobot_major, librobot_minor, */
/* 			 librobot_robot_type, librobot_date, */
/* 			 "librobot", 1); */
  
  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));
  
  tcxRegisterHandlers(BUTTONS_handler_array, 
		      sizeof(BUTTONS_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
  
  tcxRegisterCloseHnd(BUTTONS_close_handler);
  
  fprintf(stderr, "done.\n");
  
  return 1;
  
}

/*
 * $Log: buttonHandlers.c,v $
 * Revision 1.6  1997/11/06 17:53:04  swa
 * Fixed a bug with TCXHOST. If the environment variable TCXHOST is set,
 * take it. If not, take the TCXHOST from beeSoft.ini. If this doesn't
 * work, take localhost. If this doesn't work: bummer.
 *
 * Revision 1.5  1997/07/30 02:51:22  thrun
 * Oops. Something went wrong.
 * This version has more consistent status update, and a message
 * for simulating button presses
 *
 * Revision 1.4  1997/07/13 21:59:46  swa
 * Fixed a minor bug in add_auto_update_module().
 *
 * Revision 1.3  1997/07/04 17:26:03  swa
 * Added lib support for buttons. Renamed the executable buttonServer to be
 * more consistent. No root permissions are necessary to run the buttonServer.
 * Renamed tons of things to be backward-compatible.
 *
 * Revision 1.2  1997/06/29 22:13:26  thrun
 * Compiles nicely with "-Wall" (swa)
 *
 * Revision 1.1  1997/06/29 21:58:28  thrun
 * Added a ton of files for a client/server button thing.                (swa)
 *
 */
