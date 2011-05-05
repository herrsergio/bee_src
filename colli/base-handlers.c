
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




#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "tcx.h"
#include "tcxP.h"
#include "robot_specifications.h"
#include "rwibase_interface.h"
#include "sonar_interface.h"

#define TCX_define_variables /* this makes sure variables are installed */
#include "BASE-messages.h"
#include "SONAR-messages.h"
#include "LASER-messages.h"
#include "IR-messages.h"
#include "COLLI-messages.h"
#include "server.h"
#include "bUtils.h"

#include "collision.h"
#include "colli-handlers.h"

#define DEFINE_REPLY_HANDLERS

#ifdef UNIBONN
#include "SUNVIS-messages.h"
#include "SOUND-messages.h"
#endif
#ifdef CLEANUP
#include "ARM-messages.h"
#endif

#include "LASER_SERVER-messages.h"
#include "SIMULATOR-messages.h"



#include "base-handlers.h"

#include "beeSoftVersion.h"
#include "libbUtils.h"
#include "librobot.h"
#include "libezx.h"





/*---- 'BASE_debug' prints out messages upon receipt of a TCX message ----*/
/* #define BASE_debug */


BASE_update_status_reply_type base_tcx_status;
SONAR_sonar_reply_type        sonar_tcx_status;
LASER_laser_reply_type        laser_tcx_status;
IR_ir_reply_type              ir_tcx_status;

extern int  listen_for_tcx_events;	/* in devUtils.c */
extern int use_router;

struct timeval Last_ExternalObstaclePoint_Update;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/******* auto-reply stuff, data definitions ***************/


int n_auto_update_modules = 0; /* number of processes to whom
				* position should be forwarded
				* automatically upon change */

int n_auto_status_update_modules = 0;
int n_auto_sonar_update_modules  = 0;
int n_auto_laser_update_modules  = 0;
int n_auto_ir_update_modules     = 0;
int n_auto_colli_update_modules  = 0;


auto_update_type auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];
                               /* collection of TCX-module ptrs */



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

  n_auto_status_update_modules = 0;
  n_auto_sonar_update_modules  = 0;
  n_auto_laser_update_modules  = 0;
  n_auto_ir_update_modules     = 0;
  n_auto_colli_update_modules  = 0;

  for (i = 0; i < n_auto_update_modules; i++){
    if (auto_update_modules[i].status) n_auto_status_update_modules++;
    if (auto_update_modules[i].sonar)  n_auto_sonar_update_modules++;
    if (auto_update_modules[i].laser)  n_auto_laser_update_modules++;
    if (auto_update_modules[i].ir)     n_auto_ir_update_modules++;
    if (auto_update_modules[i].colli)  n_auto_colli_update_modules++;
  }
}


/************************************************************************
 *
 *   NAME:         add_auto_update_module()
 *                 
 *   FUNCTION:     Adds a module to the list of modules that
 *                 get automatical status updates
 *                 
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *                 int status                  1, if subscribe to status
 *                 int sonar                   1, if subscribe to sonar
 *                 int laser                   1, if subscribe to laser
 *                 int ir                      1, if subscribe to ir
 *                 int colli                   1, if subscribe to colli
 *                 
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/



static int add_auto_update_module(TCX_MODULE_PTR module, int status,
				  int sonar, int laser, int ir,
				  int colli)
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
		"Module %s already known. Subscription modified: %d %d %d %d\n",
		tcxModuleName(module), status, sonar, laser, colli);
	auto_update_modules[i].status = status; /* subsrc? */
	auto_update_modules[i].sonar  = sonar; /* subsrc? */
	auto_update_modules[i].laser  = laser; /* subsrc? */
	auto_update_modules[i].ir     = ir; /* subsrc? */
	auto_update_modules[i].colli  = colli; /* subsrc? */
	count_auto_update_modules();
	return 1;
      }
  fprintf(stderr, "Add %s to auto-reply list.\n",
	  tcxModuleName(module));
  auto_update_modules[n_auto_update_modules].module = module; /*new pointer*/
  auto_update_modules[n_auto_update_modules].status = status; /* subsrc? */
  auto_update_modules[n_auto_update_modules].sonar  = sonar; /* subsrc? */
  auto_update_modules[n_auto_update_modules].laser  = laser; /* subsrc? */
  auto_update_modules[n_auto_update_modules].ir     = ir;    /* subsrc? */
  auto_update_modules[n_auto_update_modules].colli  = colli; /* subsrc? */
  n_auto_update_modules++;
  count_auto_update_modules();
  return 1;
}



/************************************************************************
 *
 *   NAME:         remove_auto_update_module()
 *                 
 *   FUNCTION:     Attempts to remove a module from the list of 
 *                 modules that get automatical status updates 
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
	auto_update_modules[j].sonar = 
	  auto_update_modules[j+1].sonar; /* shift back */
	auto_update_modules[j].laser = 
	  auto_update_modules[j+1].laser; /* shift back */
	auto_update_modules[j].ir = 
	  auto_update_modules[j+1].ir; /* shift back */
	auto_update_modules[j].colli = 
	  auto_update_modules[j+1].colli; /* shift back */
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


static int n_send_automatic_status_update = 0;

void send_automatic_status_update(void)
{
  int i;

  for (i = 0; i < n_auto_update_modules; i++){
    if (auto_update_modules[i].status > 0 &&
	n_send_automatic_status_update % auto_update_modules[i].status == 0){
#ifdef BASE_debug
      fprintf(stderr, "Send status update to %s.\n",
	      tcxModuleName(auto_update_modules[i].module));
#endif
      tcxSendMsg(auto_update_modules[i].module, "BASE_update_status_reply",
		 &base_tcx_status); 
    }
  }
  n_send_automatic_status_update++;
}



/************************************************************************
 *
 *   NAME:         send_automatic_action_executed_message
 *
 *                 
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void send_automatic_action_executed_message(void)
{
  int i;
  BASE_action_executed_reply_type state; 

  state.x = (float) rwi_base.pos_x;
  state.y = (float) rwi_base.pos_y;
  state.orientation = (float) rwi_base.rot_position;

  for (i = 0; i < n_auto_update_modules; i++){
    if (auto_update_modules[i].status > 0){
#ifdef BASE_debug
      fprintf(stderr, "Send action executed update to %s.\n",
	      tcxModuleName(auto_update_modules[i].module));
#endif
      tcxSendMsg(auto_update_modules[i].module, 
		 "BASE_action_executed_reply", &state);
    }
  }
}



/************************************************************************
 *
 *   NAME:         send_automatic_sonar_update
 *                 
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

static int n_send_automatic_sonar_update = 0;

void send_automatic_sonar_update(void)
{
  int i;
  
  for (i = 0; i < n_auto_update_modules; i++){
    if (auto_update_modules[i].sonar > 0 &&
	n_send_automatic_sonar_update % auto_update_modules[i].sonar == 0){
#ifdef BASE_debug
      fprintf(stderr, "Send sonar update to %s.\n",
	      tcxModuleName(auto_update_modules[i].module));
#endif
      tcxSendMsg(auto_update_modules[i].module, "SONAR_sonar_reply",
		 &sonar_tcx_status); 
    }
  }
  n_send_automatic_sonar_update++;
}




/************************************************************************
 *
 *   NAME:         send_automatic_laser_update
 *                 
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

static int n_send_automatic_laser_update = 0;

void send_automatic_laser_update(void)
{
  int i;
  
  for (i = 0; i < n_auto_update_modules; i++){
    if ( auto_update_modules[i].laser > 0 &&
	 n_send_automatic_laser_update % auto_update_modules[i].laser == 0){
#ifdef BASE_debug
      fprintf(stderr, "Send laser update to %s.\n",
	      tcxModuleName(auto_update_modules[i].module));
#endif
      tcxSendMsg( auto_update_modules[i].module, "LASER_laser_reply",
		  &laser_tcx_status);
    }
  }
  n_send_automatic_laser_update++;
}





/************************************************************************
 *
 *   NAME:         send_automatic_ir_update
 *                 
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

static int n_send_automatic_ir_update = 0;

void send_automatic_ir_update(void)
{
  int i;
  
  for (i = 0; i < n_auto_update_modules; i++){
    if (auto_update_modules[i].ir > 0 &&
	n_send_automatic_ir_update % auto_update_modules[i].ir == 0){
#ifdef BASE_debug
      fprintf(stderr, "Send ir update to %s.\n",
	      tcxModuleName(auto_update_modules[i].module));
#endif
      tcxSendMsg(auto_update_modules[i].module, "IR_ir_reply",
		 &ir_tcx_status); 
    }
  }
  n_send_automatic_ir_update++;
}





/************************************************************************
 *
 *   NAME:         BASE_register_auto_update
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_register_auto_update_handler(TCX_REF_PTR                   ref,
				       BASE_register_auto_update_ptr data)
{
  int error_value = 1;		/* only relevant if not i386 machine! 
				 * but then there is no physical base/sonar
				 * anyhow. Thus, default = no error (=1) */

#ifdef BASE_debug
  fprintf(stderr, 
	  "Received a BASE_register_auto_update message [%d,%d %d %d] from %s.\n",
	  data->subscribe_status_report, data->subscribe_sonar_report,
	  data->subscribe_laser_report, data->subscribe_ir_report,
	  tcxModuleName(ref->module));
#endif

  add_auto_update_module(ref->module, 
			 data->subscribe_status_report,
			 data->subscribe_sonar_report,
			 data->subscribe_laser_report,
			 data->subscribe_ir_report,
			 data->subscribe_colli_report);

  tcxFree("BASE_register_auto_update", data);
}





/************************************************************************
 *
 *   NAME:         connect_to_SOUND
 *                 
 *   FUNCTION:     checks, and connects to SOUND, if possible
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


static struct timeval last_attempt_connect_SOUND = {0, 0};



void connect_to_SOUND(void)
{
#ifdef UNIBONN
   struct timeval current_time;
   
   if (!listen_for_tcx_events || base_device.dev.use_simulator)
     return;
   
   
   if (SOUND == NULL){
     
    gettimeofday(&current_time, NULL);
    if (current_time.tv_sec < last_attempt_connect_SOUND.tv_sec + 5
	|| (current_time.tv_sec == last_attempt_connect_SOUND.tv_sec + 5 &&
	    current_time.tv_usec < last_attempt_connect_SOUND.tv_usec))
      return;
    
    last_attempt_connect_SOUND.tv_sec  = current_time.tv_sec;
    last_attempt_connect_SOUND.tv_usec = current_time.tv_usec;
    
    
    SOUND = tcxConnectOptional(TCX_SOUND_MODULE_NAME); /* checks, but does 
							* not wait */
    
    if (SOUND != NULL){
/*        SOUND_play_text(MSG_gong);  */
      fprintf(stderr, "Successfully connected to SOUND module.\n");
    }
 }
#endif
}

void SOUND_stop()
{
#ifdef UNIBONN
  connect_to_SOUND();
  if (SOUND != NULL)
    tcxSendMsg(SOUND, "SOUND_pause", NULL);
#endif
}


void SOUND_play_reply_handler(TCX_REF_PTR ref, void *data)
{
#ifdef UNIBONN

#endif
}

#ifdef UNIBONN
void SOUND_playing_reply_handler(TCX_REF_PTR ref,
				 SOUND_playing_reply_type *data)
{
  tcxFree( "SOUND_playing_reply", data);
}
#endif

void SOUND_play_message( int messageNumber)
{
#ifdef UNIBONN
  connect_to_SOUND();
  if (SOUND != NULL)
    tcxSendMsg(SOUND, "SOUND_play", NULL);
#endif
}


void SOUND_talk_text(char *text)
{
#ifdef UNIBONN
  connect_to_SOUND();
  if (SOUND != NULL)
    tcxSendMsg(SOUND, "SOUND_play", NULL);
#endif
}




/************************************************************************
 *
 *   NAME:         BASE_robot_position_query_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_robot_position_query_handler(TCX_REF_PTR     ref,
				       void           *data)
{
  BASE_robot_position_reply_type position;

#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_robot_position_query message from %s.\n",
	  tcxModuleName(ref->module));
#endif
  position.x = rwi_base.pos_x;
  position.y = rwi_base.pos_y;
  position.orientation = rwi_base.rot_position;
  tcxReply(ref, "BASE_robot_position_reply", &position);
  fprintf(stderr, "Sent reply: %g %g %g\n", 
	  position.x, position.y, position.orientation);
}




/************************************************************************
 *
 *   NAME:         BASE_stop_robot_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_stop_robot_handler(TCX_REF_PTR     ref,
			     void           *data)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_stop_robot message from %s.\n",
	  tcxModuleName(ref->module));
#endif
  stop_robot();

}
/************************************************************************
 *
 *   NAME:         BASE_translate_halt_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_translate_halt_handler(TCX_REF_PTR     ref,
				 void           *data)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_translate_halt message from %s.\n",
	  tcxModuleName(ref->module));
#endif
  BASE_TranslateHalt();
 
}
/************************************************************************
 *
 *   NAME:         BASE_rotate_halt_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_rotate_halt_handler(TCX_REF_PTR     ref,
			     void           *data)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_rotate_halt message from %s.\n",
	  tcxModuleName(ref->module));
#endif
  BASE_RotateHalt();

}


/************************************************************************
 *
 *   NAME:         BASE_reset_joystick_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_reset_joystick_handler(TCX_REF_PTR     ref,
				 void           *data)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_reset_joystick message from %s.\n",
	  tcxModuleName(ref->module));
#endif
  BASE_ResetJoystick();
}



/************************************************************************
 *
 *   NAME:         BASE_notify_wall_orientation
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_notify_wall_orientation_handler(TCX_REF_PTR     ref,
					  float          *orientation)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_notify_wall_orientation message from %s.\n",
	  tcxModuleName(ref->module));
#endif


  tcxFree("BASE_notify_wall_orientation", orientation);
}




/************************************************************************
 *
 *   NAME:         BASE_update_status_query_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_update_status_query_handler(TCX_REF_PTR     ref,
				      void           *data)
{
  BASE_update_status_reply_type status;
  

#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_update_status_query message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  base_tcx_status.time                = (float) rwi_base.time;
  base_tcx_status.rot_acceleration    = (float) rwi_base.rot_acceleration;
  base_tcx_status.rot_current_speed   = (float) rwi_base.rot_current_speed;
  base_tcx_status.rot_set_speed       = (float) rwi_base.rot_set_speed;
  base_tcx_status.rot_position        = (float) rwi_base.rot_position;
  base_tcx_status.trans_acceleration  = (float) rwi_base.trans_acceleration;
  base_tcx_status.trans_current_speed = (float) rwi_base.trans_current_speed;
  base_tcx_status.trans_set_speed     = (float) rwi_base.trans_set_speed;
  base_tcx_status.trans_position      = (float) rwi_base.trans_position;
  base_tcx_status.pos_x               = (float) rwi_base.pos_x;
  base_tcx_status.pos_y               = (float) rwi_base.pos_y;
  base_tcx_status.orientation         = (float) rwi_base.rot_position;
  base_tcx_status.trans_direction     = 
    (unsigned char)   rwi_base.trans_direction;
  base_tcx_status.trans_set_direction = 
    (unsigned char)   rwi_base.trans_set_direction;
  base_tcx_status.rot_direction       = 
    (unsigned char)   rwi_base.rot_direction;
  base_tcx_status.rot_set_direction   = 
    (unsigned char)   rwi_base.rot_set_direction;
  base_tcx_status.rot_moving          = 
    (unsigned char)   rwi_base.rot_moving;
  base_tcx_status.trans_moving        = 
    (unsigned char)   rwi_base.trans_moving;
  base_tcx_status.bumpers             = 
    (unsigned char)   rwi_base.bumpers;
  base_tcx_status.bump                = 
    (unsigned char)   rwi_base.bump;
  base_tcx_status.emergency           = 
    (unsigned char)   rwi_base.emergency;
  base_tcx_status.emergencyProcedure  = 
    (unsigned char)   rwi_base.emergencyProcedure;
  base_tcx_status.collision_state.target_reached = 
    (unsigned char) rwi_base.collision_state.target_reached;
  
  tcxReply(ref, "BASE_update_status_reply", &base_tcx_status);
  fprintf(stderr, "Sent status reply %g.\n", base_tcx_status.time);
}



/************************************************************************
 *
 *   NAME:         BASE_sonar_query_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_sonar_query_handler(TCX_REF_PTR     ref,
			      void           *data)
{
  int i;

#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_sonar_query message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  for(i = 0; i < 24; i++)
    sonar_tcx_status.values[i] = (float) sonar_readings[i];

  tcxReply(ref, "BASE_sonar_reply", &sonar_tcx_status);
  fprintf(stderr, "Sent sonar reply.\n");
}




/************************************************************************
 *
 *   NAME:         BASE_set_velocity_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_set_velocity_handler(TCX_REF_PTR           ref,
			       BASE_set_velocity_ptr velocity)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_set_velocity message: %g %g from %s.\n",
	  velocity->rot_velocity, velocity->trans_velocity,
	  tcxModuleName(ref->module));
#endif
  if (velocity->rot_velocity < 0.0)
    velocity->rot_velocity = 0.0; /* non-negative velocities only */
  if (velocity->trans_velocity < 0.0)
    velocity->trans_velocity = 0.0; /* non-negative velocities only */

  BASE_RotateVelocity( (double) velocity->rot_velocity);
  BASE_TranslateVelocity( (double) velocity->trans_velocity);

  tcxFree("BASE_set_velocity", velocity);
}



/************************************************************************
 *
 *   NAME:         BASE_set_acceleration_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_set_acceleration_handler(TCX_REF_PTR               ref,
				   BASE_set_acceleration_ptr acceleration)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_set_acceletarion message: %g %g from %s.\n",
	  acceleration->rot_acceleration, acceleration->trans_acceleration,
	  tcxModuleName(ref->module));
#endif
  if (acceleration->rot_acceleration < 0.0)
    acceleration->rot_acceleration = 0.0; /* non-negative velocities only */
  if (acceleration->trans_acceleration < 0.0)
    acceleration->trans_acceleration = 0.0; /* non-negative velocities only */
  
  BASE_RotateAcceleration( (double) acceleration->rot_acceleration);
  BASE_TranslateAcceleration( (double) acceleration->trans_acceleration);
  
  tcxFree("BASE_set_acceleration", acceleration);
}




/************************************************************************
 *
 *   NAME:         BASE_rotate_clockwise_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void BASE_rotate_clockwise_handler(TCX_REF_PTR  ref,
				   void        *data)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_rotate_clockwise message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  BASE_RotateClockwise();
}




/************************************************************************
 *
 *   NAME:         BASE_rotate_anticlockwise_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void BASE_rotate_anticlockwise_handler(TCX_REF_PTR  ref,
				       void        *data)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_rotate_anticlockwise message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  BASE_RotateAnticlockwise();
}




/************************************************************************
 *
 *   NAME:         BASE_translate_forward_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void BASE_translate_forward_handler(TCX_REF_PTR  ref,
				    void        *data)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_translate_forward message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  BASE_TranslateForward();
}





/************************************************************************
 *
 *   NAME:         BASE_translate_backward_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void BASE_translate_backward_handler(TCX_REF_PTR  ref,
				     void        *data)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_translate_backward message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  BASE_TranslateBackward();
}



/************************************************************************
 *
 *   NAME:         BASE_translate_by
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void BASE_translate_by_handler(TCX_REF_PTR  ref,
			       float       *dist)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_translate_by message %f from %s.\n",
	  *dist, tcxModuleName(ref->module));
#endif

  BASE_Translate((double) *dist);
  tcxFree("BASE_translate_by", dist);

}



/************************************************************************
 *
 *   NAME:         BASE_rotate_by
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void BASE_rotate_by_handler(TCX_REF_PTR  ref,
			    float       *angle)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_rotate_by message %f from %s.\n",
	  *angle, tcxModuleName(ref->module));
#endif
  fprintf(stderr, "Received a BASE_rotate_by message %f from %s.\n",
	  *angle, tcxModuleName(ref->module));

  BASE_Rotate((double) *angle);
  tcxFree("BASE_rotate_by", angle);
}





/************************************************************************
 *
 *   NAME:         BASE_goto_relative_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_goto_relative_handler(TCX_REF_PTR            ref,
				BASE_goto_relative_ptr pos)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_goto_relative message: %g %g from %s.\n",
	  pos->rel_target_x, pos->rel_target_y,
	  tcxModuleName(ref->module));
#endif

   COLLI_GotoAbsolute(rwi_base.pos_x  
 		     - (pos->rel_target_x * 
 			cos(rwi_base.rot_position * M_PI / 180.0)) 
 		     + (pos->rel_target_y *  
 			sin(rwi_base.rot_position * M_PI / 180.0)), 
 		     rwi_base.pos_y 
 		     + (pos->rel_target_x * 
 			sin(rwi_base.rot_position * M_PI / 180.0)) 
 		     + (pos->rel_target_y *  
 			cos(rwi_base.rot_position * M_PI / 180.0)), 
 		     (BOOLEAN) 1); 


  tcxFree("BASE_goto_relative", pos);
}



/************************************************************************
 *
 *   NAME:         BASE_goto_absolute_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_goto_absolute_handler(TCX_REF_PTR            ref,
				BASE_goto_absolute_ptr pos)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_goto_absolute message: %g %g from %s.\n",
	  pos->abs_target_x, pos->abs_target_y,
	  tcxModuleName(ref->module));
#endif
  
  COLLI_GotoAbsolute(pos->abs_target_x, pos->abs_target_y, (BOOLEAN) pos->new_target);
  tcxFree("BASE_goto_absolute", pos);
}




/************************************************************************
 *
 *   NAME:         BASE_approach_absolute_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_approach_absolute_handler(TCX_REF_PTR            ref,
				BASE_approach_absolute_ptr pos)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_approach_absolute message: %g %g %g from %s.\n",
	  pos->approach_dist, pos->abs_target_x, pos->abs_target_y,
	  tcxModuleName(ref->module));
#endif

  COLLI_ApproachAbsolute( pos->abs_target_x, pos->abs_target_y,
			 pos->approach_dist, (BOOLEAN) pos->new_target, pos->mode );
  tcxFree("BASE_approach_absolute", pos);
}




/************************************************************************
 *
 *   NAME:         BASE_approach_absolute_two_points_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_approach_absolute_two_points_handler(TCX_REF_PTR            ref,
					       BASE_approach_absolute_two_points_ptr pos)
{
#ifdef BASE_debug 
  fprintf(stderr, "Received a BASE_approach_absolute_two_points message: ");
  fprintf(stderr, "p1 %g %g  p2 %g %g from %s.\n",
	  pos->abs_target_x1, pos->abs_target_y1,
	  pos->abs_target_x2, pos->abs_target_y2,
	  tcxModuleName(ref->module));
#endif

  COLLI_ApproachAbsolute_TwoPoints( pos->abs_target_x1, pos->abs_target_y1,
				    pos->abs_target_x2, pos->abs_target_y2,
				    pos->approach_dist, (BOOLEAN) pos->new_target, ref );
  tcxFree("BASE_approach_absolute_two_points_absolute", pos);
}



/************************************************************************
 *
 *   NAME:         BASE_setmode_handler
 *                 
 *   FUNCTION:     sets a mode for the collision avoidance.
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
void BASE_setmode_handler(TCX_REF_PTR ref, BASE_setmode_ptr modeInfo)
{
  static int firstTime = TRUE;
  static int considerLaser, considerSonar;
  
  if ( firstTime) {
    considerSonar = use_sonar;
    considerLaser = use_laser;
    firstTime     = FALSE;
  }
  
#define MODE_debug
#ifdef MODE_debug 
  fprintf(stderr, "Received a BASE_setmode message: %d %d %d from %s.\n",
	  modeInfo->modeNumber,
	  modeInfo->useSonar,
	  modeInfo->useLaser,
	  tcxModuleName(ref->module));
#endif

  if ( modeInfo->modeNumber != DONT_CHANGE)
    COLLI_SetMode( modeInfo->modeNumber);
  
  if ( modeInfo->useLaser == 0 || modeInfo->useLaser == 1) {
    if ( modeInfo->useLaser && ! considerLaser)
      fprintf( stderr, "Sorry. Laser not started.\n");
    else {
      if ( modeInfo->useLaser && ! use_laser)
	fprintf( stderr, "Switch on laser.\n");
      else if ( ! modeInfo->useLaser && use_laser)
	fprintf( stderr, "Switch off laser.\n");
      use_laser = modeInfo->useLaser;
    }
  }
  
  if ( modeInfo->useSonar == 0 || modeInfo->useSonar == 1) {
    if ( modeInfo->useSonar && ! considerSonar)
      fprintf( stderr, "Sorry. Sonar not started.\n");
    else {
      if ( modeInfo->useSonar && ! use_sonar)
	fprintf( stderr, "Switch on sonar.\n");
      else if ( ! modeInfo->useSonar && use_sonar)
	fprintf( stderr, "Switch off sonar.\n");
      use_sonar = modeInfo->useSonar;
    }
  }
  
  tcxFree("BASE_setmode", modeInfo);
}


/************************************************************************
 *
 *   NAME:         BASE_obstacle_points_handler
 *                 
 *   FUNCTION:     receives collision points from an external module
 *                 
 *   PARAMETERS:   struct no_of_points: number of collision points
                   array with the different points
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void BASE_obstacle_points_handler( TCX_REF_PTR               ref,
				   BASE_obstacle_points_ptr  obstacle_points)
{
  int i;

  if (External_Obstacle_Points.no_of_points > 0) 
    free (External_Obstacle_Points.points);
  
  External_Obstacle_Points.no_of_points = obstacle_points->no_of_points;

  if ( External_Obstacle_Points.no_of_points > 0) {
    
    External_Obstacle_Points.points = (Point *) 
      malloc( External_Obstacle_Points.no_of_points * sizeof(struct Point));
    
    for (i=0; i<External_Obstacle_Points.no_of_points; i++) {
#ifdef BASE_debug
      fprintf(stderr, " P: %d %d\n", 
	      obstacle_points->points[i].x,
	      obstacle_points->points[i].y);
#endif
      External_Obstacle_Points.points[i].x = (float) obstacle_points->points[i].x;
      External_Obstacle_Points.points[i].y = (float) obstacle_points->points[i].y;
    }
  }

  gettimeofday(&Last_ExternalObstaclePoint_Update, 0);
  COLLI_update_tcx_ExternalObstaclePoints(); 

  tcxFree("BASE_obstacle_points", obstacle_points);
}
 

/************************************************************************
 *
 *   NAME:         BASE_obstacle_lines_handler
 *                 
 *   FUNCTION:     receives collision lines from an external module
 *                 
 *   PARAMETERS:   struct no_of_lines: number of collision lines
                   array with the different lines
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/
void BASE_obstacle_lines_handler( TCX_REF_PTR               ref,
				  BASE_obstacle_lines_ptr  obstacle_lines)
{
  fprintf(stderr, "External obstalce lines not supported any longer (05/06/97).\n");
  tcxFree("BASE_obstacle_lines", obstacle_lines);
}

/************************************************************************
 *
 *   NAME:         BASE_reset_obstacle_line_field_handler
 *                 
 *   FUNCTION:     clears the current obstacle line field
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *
 *   COMMENT:  introduced by ds 01/14/98
 ************************************************************************/
 
void
BASE_reset_obstacle_line_field_handler(TCX_REF_PTR ref, void *data)
{
  COLLI_reset_obstacle_structs();
}

 
/************************************************************************
 *
 *   NAME:         BASE_disconnect_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void BASE_disconnect_handler(TCX_REF_PTR      ref,
			     void            *data)
{
#ifdef BASE_debug
  fprintf(stderr, "Received a BASE_disconnect message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  remove_auto_update_module(ref->module);
}




/************************************************************************
 *
 *   NAME:         BASE_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void BASE_close_handler(char *name, TCX_MODULE_PTR module)
{
#ifdef BASE_debug
  fprintf(stderr, "BASE: closed connection detected: %s\n", name);
#endif

  remove_auto_update_module(module);

  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    exit(0);
  }

  

#ifdef UNIBONN

  if (!strcmp(name, "SOUND")){ /* SOUND shut down */
    fprintf(stderr, "Diconnected from SOUND module.\n");
    SOUND = NULL;
  }

  if (!strcmp(name, "OBSTACLE_SERVER")){ /* OBSTACLE_SERVER shut down */
    if (External_Obstacle_Points.no_of_points > 0) 
      free (External_Obstacle_Points.points);    
    External_Obstacle_Points.no_of_points = 0;
    COLLI_update_tcx_ExternalObstaclePoints(); 
  }

#endif  
  if (use_simulator && !strcmp(name, "SIMULATOR")){ /* SIMULATOR shut down */
    fprintf(stderr, "Simulator died, and so will I. Bye-bye.\n");
    exit(0);
  }

  if (use_laser_server && !strcmp(name, "laserServer")){ /* laserServer shut down */
    fprintf(stderr, "LasrServer died, and so will I. Bye-bye.\n");
    exit(0);
  }

  if (use_rwi_server && !strcmp(name, TCX_SERVER_MODULE_NAME)){ /* SERVER shut down */
    fprintf(stderr, "RWI Server died, and so will I. Bye-bye.\n");
    exit(0);
  }
}




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



TCX_REG_HND_TYPE BASE_handler_array[] = {

  {"BASE_register_auto_update", "BASE_register_auto_update_handler",
     BASE_register_auto_update_handler, TCX_RECV_ALL, NULL},
  {"BASE_robot_position_query", "BASE_robot_position_query_handler",
     BASE_robot_position_query_handler, TCX_RECV_ALL, NULL},
  {"BASE_stop_robot", "BASE_stop_robot_handler",
     BASE_stop_robot_handler, TCX_RECV_ALL, NULL},

  {"BASE_translate_halt", "BASE_translate_halt_handler",
     BASE_translate_halt_handler, TCX_RECV_ALL, NULL},
  {"BASE_rotate_halt", "BASE_rotate_halt_handler",
     BASE_rotate_halt_handler, TCX_RECV_ALL, NULL},
  
  {"BASE_update_status_query", "BASE_update_status_query_handler",
     BASE_update_status_query_handler, TCX_RECV_ALL, NULL},
  {"BASE_set_velocity", "BASE_set_velocity_handler",
     BASE_set_velocity_handler, TCX_RECV_ALL, NULL},
  {"BASE_set_acceleration", "BASE_set_acceleration_handler",
     BASE_set_acceleration_handler, TCX_RECV_ALL, NULL},
  {"BASE_rotate_clockwise", "BASE_rotate_clockwise_handler",
     BASE_rotate_clockwise_handler, TCX_RECV_ALL, NULL},
  {"BASE_rotate_anticlockwise", "BASE_rotate_anticlockwise_handler",
     BASE_rotate_anticlockwise_handler, TCX_RECV_ALL, NULL},
  {"BASE_translate_forward", "BASE_translate_forward_handler",
     BASE_translate_forward_handler, TCX_RECV_ALL, NULL},
  {"BASE_translate_backward", "BASE_translate_backward_handler",
     BASE_translate_backward_handler, TCX_RECV_ALL, NULL},
  {"BASE_translate_by", "BASE_translate_by_handler",
     BASE_translate_by_handler, TCX_RECV_ALL, NULL},
  {"BASE_rotate_by", "BASE_rotate_by_handler",
     BASE_rotate_by_handler, TCX_RECV_ALL, NULL},
  {"BASE_goto_relative", "BASE_goto_relative_handler",
     BASE_goto_relative_handler, TCX_RECV_ALL, NULL},
  {"BASE_goto_absolute", "BASE_goto_absolute_handler",
     BASE_goto_absolute_handler, TCX_RECV_ALL, NULL},
  {"BASE_approach_absolute", "BASE_approach_absolute_handler",
     BASE_approach_absolute_handler, TCX_RECV_ALL, NULL},
  {"BASE_approach_absolute_two_points", "BASE_approach_absolute_two_points_handler",
     BASE_approach_absolute_two_points_handler, TCX_RECV_ALL, NULL},
  {"BASE_disconnect", "BASE_disconnect_handler",
     BASE_disconnect_handler, TCX_RECV_ALL, NULL},
  {"BASE_setmode", "BASE_setmode_handler",
     BASE_setmode_handler, TCX_RECV_ALL, NULL},
  {"BASE_obstacle_points", "BASE_obstacle_points_handler",
     BASE_obstacle_points_handler, TCX_RECV_ALL, NULL},
  {"BASE_obstacle_lines", "BASE_obstacle_lines_handler",
     BASE_obstacle_lines_handler, TCX_RECV_ALL, NULL},
  {"BASE_reset_obstacle_line_field", "BASE_reset_obstacle_line_field_handler",
     BASE_reset_obstacle_line_field_handler, TCX_RECV_ALL, NULL},
  {"BASE_reset_joystick", "BASE_reset_joystick_handler",
     BASE_reset_joystick_handler, TCX_RECV_ALL, NULL},
  {"BASE_notify_wall_orientation", "BASE_notify_wall_orientation_handler",
     BASE_notify_wall_orientation_handler, TCX_RECV_ALL, NULL},

#ifdef UNIBONN
  {"COLLI_vision_line", "COLLI_vision_line_handler",
     COLLI_vision_line_handler, TCX_RECV_ALL, NULL},
  {"COLLI_vision_point", "COLLI_vision_point_handler",
     COLLI_vision_point_handler, TCX_RECV_ALL, NULL},
  {"COLLI_parameter", "COLLI_parameter_handler",
     COLLI_parameter_handler, TCX_RECV_ALL, NULL}
#endif
};



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void init_tcx(void)
{
  const char *tcxMachine = NULL;
  


  TCX_REG_MSG_TYPE TCX_message_array[] = {
     BASE_messages,
     SONAR_messages,
     LASER_messages,
     LASER_SERVER_messages,
     IR_messages,
#ifdef UNIBONN 
     SUNVIS_messages,
     SOUND_messages, 
#endif
#ifdef CLEANUP
     ARM_messages,
#endif
     COLLI_messages,
     SIMULATOR_messages,
     SERVER_messages  
  };

  fprintf(stderr, "Connecting to TCX...");
  fflush(stderr);
      
  
  /* ====== INITIALIZING TCX ============ */
  tcxMachine = bParametersGetParam(bParamList, "", "TCXHOST");


  if (tcxMachine != NULL){
    
  
    printf("Initializing TCX...");
    fflush(stdout);
  
    if ( use_router == 0 ){
      tcxInitialize(TCX_BASE_MODULE_NAME, (char *)tcxMachine);
      check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
			   BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
			   NULL, 0);
      check_version_number(libezx_major, libezx_minor,
			   libezx_robot_type, libezx_date,
			   "libezx", 0);
      check_version_number(librobot_major, librobot_minor,
			   librobot_robot_type, librobot_date,
			   "librobot", 0);
      check_version_number(libbUtils_major, libbUtils_minor,
			   libbUtils_robot_type, libbUtils_date,
			   "libbUtils", 1);




    }
  

    else
      {
	fprintf (stderr,"\n\nSetting up BASE for ROUTER-Mode ...");
	tcxInitialize("BASE_ROUTED", (char *)tcxMachine);
	check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
			     BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
			     NULL, 0);
	check_version_number(libezx_major, libezx_minor,
			     libezx_robot_type, libezx_date,
			     "libezx", 0);
	check_version_number(librobot_major, librobot_minor,
			     librobot_robot_type, librobot_date,
			     "librobot", 0);
	check_version_number(libbUtils_major, libbUtils_minor,
			     libbUtils_robot_type, libbUtils_date,
			     "libbUtils", 1);
	
      }
  
    fprintf(stderr, "done.\n");
    fflush(stderr);
  
    tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
			/ sizeof(TCX_REG_MSG_TYPE));
  
    tcxRegisterHandlers( BASE_handler_array, 
			 sizeof(BASE_handler_array) / sizeof(TCX_REG_HND_TYPE));
#ifdef UNIBONN
    tcxRegisterHandlers( SOUND_reply_handler_array, 
			 sizeof(SOUND_reply_handler_array) / sizeof(TCX_REG_HND_TYPE));
#endif  
    tcxRegisterHandlers(SIMULATOR_reply_handler_array, 
			sizeof(SIMULATOR_reply_handler_array) / sizeof(TCX_REG_HND_TYPE));
    tcxRegisterHandlers(LASER_SERVER_reply_handler_array, 
			sizeof(LASER_SERVER_reply_handler_array) / sizeof(TCX_REG_HND_TYPE));
  
#ifdef UNIBONN
    tcxRegisterHandlers(SUNVIS_reply_handler_array, 
			sizeof(SUNVIS_reply_handler_array) / sizeof(TCX_REG_HND_TYPE));

#endif  /* UNIBONN */

#ifdef CLEANUP 
    tcxRegisterHandlers(ARM_reply_handler_array, 
			sizeof(ARM_reply_handler_array) / sizeof(TCX_REG_HND_TYPE));

#endif  /* CLEANUP */

    tcx_register_sonar();

    tcxRegisterCloseHnd(BASE_close_handler);
  }
  else{
    fprintf(stderr, "Error in TCX: TCXHOST not set appropriately\n");
  }
}
