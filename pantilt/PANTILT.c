
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
#include <sys/time.h>
#include <bUtils.h>
#include "robot_specifications.h"
#include "tcx.h"
#include "tcxP.h"
#include "global.h"

#include "panTilt_interface.h"

#define TCX_define_variables /* this makes sure variables are installed */
#include "PANTILT-messages.h"

#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */
#include "BASE-messages.h"

#include "beeSoftVersion.h"
#include "librobot.h"
#include "libezx.h"


struct timeval TCX_waiting_time = {1,0};


#define PANTILT_TIME_NEXT_TRACK 0.5



/*---- 'PANTILT_debug' prints out messages upon receipt of a TCX message ----*/
#undef PANTILT_debug

static int error_value = 0;	/* return value of the PANTILT
				 * device initialization. Will be
				 * returned to connecting module.
				 * I guess 0 is no error. */

extern float global_pan_correction;

PANTILT_status_update_type global_status = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0};

void connect_BASE();



float tracking_x = 0.0;
float tracking_y = 0.0;
float tracking_height = 0.0;
int tracking_on = 0;


PANTILT_limits_reply_type limits;



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/******* auto-reply stuff, data definitions ***************/

#define MAX_N_AUTO_UPDATE_MODULES 100

static int n_auto_update_modules = 0; /* number of processes to whom
				      * position should be forwarded
				      * automatically upon change */


static TCX_MODULE_PTR auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];
                       /* collection of TCX-module ptrs */



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



static int add_auto_update_module(TCX_MODULE_PTR module)
{
  int i;


  if (n_auto_update_modules >= MAX_N_AUTO_UPDATE_MODULES){
    fprintf(stderr, "ERROR: auto-module memory overflow. Request rejected.\n");
    return 0;
  }
  else
    for (i = 0; i < n_auto_update_modules; i++)
      if (auto_update_modules[i] == module){
	fprintf(stderr, "ERROR: module %s already known. Must be a bug.\n",
		tcxModuleName(module));
	return 0;
      }
  fprintf(stderr, "Add %s to auto-reply list.\n",
	  tcxModuleName(module));
  auto_update_modules[n_auto_update_modules++] = module; /* insert pointer */
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
    if (auto_update_modules[i] == module){ /* if module found */
      fprintf(stderr, "Remove %s from auto-reply list.\n",
	      tcxModuleName(module));
      found++;
      n_auto_update_modules--;	/* remove that entry, one less now */
      for (j = i; j < n_auto_update_modules; j++)  
	auto_update_modules[j] = auto_update_modules[j+1]; /* shift back */
    }
  if (!found)
    fprintf(stderr, "(%s: no auto-replies removed)\n",
	    tcxModuleName(module));
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


void send_automatic_status_update(void)
{
  int i;
  static float prev_pan_pos = -10000.0, prev_tilt_pos = -10000.0;

#ifdef PANTILT_debug
  struct timeval time;
  int time2;
#endif

  for (i = 0; i < n_auto_update_modules; i++){
#ifdef i386
    PANTILT_position(&(global_status.pan_pos), &(global_status.tilt_pos));
#endif
    global_status.error = 0;
    if (prev_pan_pos != global_status.pan_pos ||
	prev_tilt_pos != global_status.tilt_pos){
#ifdef PANTILT_debug
      gettimeofday(&time, NULL);
      time2 = (time.tv_sec % 3600) * 10 + time.tv_usec/100000;

      fprintf(stderr, "%s:%6d:%s() - [%6d]  %s.\n",
	      __FILE__, __LINE__, __FUNCTION__, time2,
	      tcxModuleName(auto_update_modules[i]));
#endif
      tcxSendMsg(auto_update_modules[i], "PANTILT_status_update",
		 &global_status);
      prev_pan_pos = global_status.pan_pos;
      prev_tilt_pos = global_status.tilt_pos;
    }
  }
}



/************************************************************************
 *
 *   NAME: PANTILT_init_query_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void PANTILT_init_query_handler(TCX_REF_PTR      ref,
			        int             *data)
{

#ifdef PANTILT_debug
  fprintf(stderr, "Received a PANTILT_init_query message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  if (*data) add_auto_update_module(ref->module);

  tcxReply(ref, "PANTILT_init_reply", &error_value);


  tcxSendMsg(ref->module, "PANTILT_status_update", &global_status);

  connect_BASE();
}




/************************************************************************
 *
 *   NAME:         PANTILT_terminate_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void PANTILT_terminate_handler(TCX_REF_PTR      ref,
			       void            *data)
{

#ifdef PANTILT_debug
  fprintf(stderr, "Received a PANTILT_terminate message from %s.\n",
	  tcxModuleName(ref->module));
#endif


#ifdef i386
  PANTILT_terminate();
#endif  

  connect_BASE();

}



/************************************************************************
 *
 *   NAME:         PANTILT_set_velocity_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void PANTILT_set_velocity_handler(TCX_REF_PTR      ref,
				  PANTILT_set_velocity_ptr data)
{

#ifdef PANTILT_debug
  fprintf(stderr, "Received a PANTILT_set_velocity message from %s.\n",
	  tcxModuleName(ref->module));
#endif


#ifdef i386
  if (global_status.pan_velocity != data->pan_velocity ||
      global_status.tilt_velocity != data->tilt_velocity)
    PANTILT_setVelocity(data->pan_velocity, data->tilt_velocity);
#endif
  global_status.pan_velocity = data->pan_velocity;
  global_status.tilt_velocity = data->tilt_velocity;

  send_automatic_status_update();
  tcxFree("PANTILT_set_velocity", data); 

  connect_BASE();

}




/************************************************************************
 *
 *   NAME:         PANTILT_set_acceleration_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void PANTILT_set_acceleration_handler(TCX_REF_PTR      ref,
				      PANTILT_set_acceleration_ptr data)
{

#ifdef PANTILT_debug
  fprintf(stderr, "Received a PANTILT_set_acceleration message from %s.\n",
	  tcxModuleName(ref->module));
#endif


#ifdef i386
  if (global_status.pan_acceleration != data->pan_acceleration ||
      global_status.tilt_acceleration != data->tilt_acceleration)
    PANTILT_setAcceleration(data->pan_acceleration, data->tilt_acceleration);
#endif

  global_status.pan_acceleration  = data->pan_acceleration;
  global_status.tilt_acceleration = data->tilt_acceleration;

  send_automatic_status_update();
  tcxFree("PANTILT_set_acceleration", data);

  connect_BASE();

}




/************************************************************************
 *
 *   NAME:         PANTILT_position_query_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void PANTILT_position_query_handler(TCX_REF_PTR      ref,
				    void            *data)
{
  PANTILT_position_reply_type pos;


#ifdef PANTILT_debug
  fprintf(stderr, "Received a PANTILT_position_query message from %s.\n",
	  tcxModuleName(ref->module));
#endif

#ifdef i386
  PANTILT_position(&(pos.pan_pos), &(pos.tilt_pos));
#endif

  tcxReply(ref, "PANTILT_position_reply", &pos);

  connect_BASE();

}




/************************************************************************
 *
 *   NAME:         PANTILT_pan_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void PANTILT_pan_handler(TCX_REF_PTR      ref,
			 float           *data)
{

#ifdef PANTILT_debug
  fprintf(stderr, "Received a PANTILT_pan message from %s.\n",
	  tcxModuleName(ref->module));
#endif


#ifdef i386
  PANTILT_pan(*data);
#else
  global_status.pan_pos = *data;
#endif

  send_automatic_status_update();
  tcxFree("PANTILT_pan", data);

  connect_BASE();

}




/************************************************************************
 *
 *   NAME:         PANTILT_pan_relative_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void PANTILT_pan_relative_handler(TCX_REF_PTR      ref,
				  float           *data)
{

#ifdef PANTILT_debug
  fprintf(stderr, "Received a PANTILT_pan_relative message from %s.\n",
	  tcxModuleName(ref->module));
#endif


#ifdef i386
  PANTILT_panRelative(*data);
#else
  global_status.pan_pos += *data;
#endif

  send_automatic_status_update();
  tcxFree("PANTILT_pan_relative", data);

  connect_BASE();

}




/************************************************************************
 *
 *   NAME:         PANTILT_tilt_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void PANTILT_tilt_handler(TCX_REF_PTR      ref,
			  float           *data)
{

#ifdef PANTILT_debug
  fprintf(stderr, "Received a PANTILT_tilt message from %s.\n",
	  tcxModuleName(ref->module));
#endif


#ifdef i386
  PANTILT_tilt(*data);
#else
  global_status.tilt_pos = *data;
#endif

  send_automatic_status_update();
  tcxFree("PANTILT_tilt", data);

  connect_BASE();

}




/************************************************************************
 *
 *   NAME:         PANTILT_tilt_relative_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void PANTILT_tilt_relative_handler(TCX_REF_PTR      ref,
				   float           *data)
{

#ifdef PANTILT_debug
  fprintf(stderr, "Received a PANTILT_tilt_relative message from %s.\n",
	  tcxModuleName(ref->module));
#endif

#ifdef i386
  PANTILT_tiltRelative(*data);
#else
  global_status.tilt_pos += *data;
#endif

  send_automatic_status_update();
  tcxFree("PANTILT_tilt_relative", data);

  connect_BASE();

}




/************************************************************************
 *
 *   NAME:         PANTILT_move_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void PANTILT_move_handler(TCX_REF_PTR      ref,
			  PANTILT_move_ptr data)
{

#ifdef PANTILT_debug
  fprintf(stderr, "Received a PANTILT_move message: %g %g from %s.\n",
	  data->pan_target, data->tilt_target, tcxModuleName(ref->module));
#endif

#ifdef i386
  PANTILT_move(data->pan_target, data->tilt_target);
#else
  global_status.pan_pos  = data->pan_target;
  global_status.tilt_pos = data->tilt_target;
#endif

  send_automatic_status_update();
  tcxFree("PANTILT_move", data);

  connect_BASE();

}




/************************************************************************
 *
 *   NAME:         PANTILT_moveRelative_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void PANTILT_moveRelative_handler(TCX_REF_PTR      ref,
				  PANTILT_moveRelative_ptr data)
{

#ifdef PANTILT_debug
  fprintf(stderr, "Received a PANTILT_moveRelative message from %s.\n",
	  tcxModuleName(ref->module));
#endif

#ifdef i386
  PANTILT_moveRelative(data->pan_delta_target, data->tilt_delta_target);
#else
  global_status.pan_pos  += data->pan_delta_target;
  global_status.tilt_pos += data->tilt_delta_target;
#endif

  send_automatic_status_update();
  tcxFree("PANTILT_moveRelative", data);  
  connect_BASE();

}




/************************************************************************
 *
 *   NAME:         PANTILT_reset_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void PANTILT_reset_handler(TCX_REF_PTR      ref,
			   void            *data)
{

#ifdef PANTILT_debug
  fprintf(stderr, "Received a PANTILT_reset message from %s.\n",
	  tcxModuleName(ref->module));
#endif

#ifdef i386
  PANTILT_reset();
#else
  global_status.pan_pos  = 0.0;
  global_status.tilt_pos = 0.0;
#endif
  send_automatic_status_update();

  connect_BASE();

}




/************************************************************************
 *
 *   NAME:         PANTILT_limits_query_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void PANTILT_limits_query_handler(TCX_REF_PTR      ref,
				  void            *data)
{


#ifdef PANTILT_debug
  fprintf(stderr, "Received a PANTILT_limits_query message from %s.\n",
	  tcxModuleName(ref->module));
#endif

#ifdef i386
  PANTILT_limits(&(limits.pan_min_angle), &(limits.pan_max_angle),
		 &(limits.tilt_min_angle), &(limits.tilt_max_angle), 
		 &(limits.pan_max_velocity), &(limits.tilt_max_velocity));
#endif
  tcxReply(ref, "PANTILT_limits_reply", &limits);  
  connect_BASE();

}



/************************************************************************
 *
 *   NAME:         PANTILT_track_point_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void PANTILT_track_point_handler(TCX_REF_PTR      ref,
				 PANTILT_track_point_ptr data)
{
  fprintf(stderr, "TRACKING: %6.4f %6.4f\n", data->x, data->y);

  tracking_x      = data->x;
  tracking_y      = data->y;
  tracking_height = data->height;
  tracking_on     = 1;
  tcxFree("PANTILT_track_point", data);
  connect_BASE();
}


/************************************************************************
 *
 *   NAME:         PANTILT_stop_tracking_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void PANTILT_stop_tracking_handler(TCX_REF_PTR      ref,
				   void            *data)
{
  tracking_on = 0;
  connect_BASE();
}




/************************************************************************
 *
 *   NAME:         PANTILT_disconnect_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void PANTILT_disconnect_handler(TCX_REF_PTR      ref,
				void            *data)
{

#ifdef PANTILT_debug
  fprintf(stderr, "Received a PANTILT_disconnect message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  remove_auto_update_module(ref->module);  
  connect_BASE();

}



  




/************************************************************************
 *
 *   NAME:         PANTILT_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void PANTILT_close_handler(char *name, TCX_MODULE_PTR module)
{

#ifdef PANTILT_debug
  fprintf(stderr, "PANTILT: closed connection detected: %s\n", name);
#endif
  remove_auto_update_module(module);

  if (!strcmp(name, "BASE")){ /* BASE shut down */
    BASE = NULL;
    fprintf(stderr, "PANTILT: BASE disconnected.\n");
  }
  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    exit(0);
  } 
}




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



TCX_REG_HND_TYPE PANTILT_handler_array[] = {
  {"PANTILT_init_query", "PANTILT_init_query_handler",
     PANTILT_init_query_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_terminate", "PANTILT_terminate_handler",
     PANTILT_terminate_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_set_velocity", "PANTILT_set_velocity_handler",
     PANTILT_set_velocity_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_set_acceleration", "PANTILT_set_acceleration_handler",
     PANTILT_set_acceleration_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_position_query", "PANTILT_position_query_handler",
     PANTILT_position_query_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_pan", "PANTILT_pan_handler",
     PANTILT_pan_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_pan_relative", "PANTILT_pan_relative_handler",
     PANTILT_pan_relative_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_tilt", "PANTILT_tilt_handler",
     PANTILT_tilt_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_tilt_relative", "PANTILT_tilt_relative_handler",
     PANTILT_tilt_relative_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_move", "PANTILT_move_handler",
     PANTILT_move_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_moveRelative", "PANTILT_moveRelative_handler",
     PANTILT_moveRelative_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_reset", "PANTILT_reset_handler",
     PANTILT_reset_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_limits_query", "PANTILT_limits_query_handler",
     PANTILT_limits_query_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_disconnect", "PANTILT_disconnect_handler",
     PANTILT_disconnect_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_track_point", "PANTILT_track_point_handler",
     PANTILT_track_point_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_stop_tracking", "PANTILT_stop_tracking_handler",
     PANTILT_stop_tracking_handler, TCX_RECV_ALL, NULL}

};



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void
connect_BASE()
{
  BASE_register_auto_update_type data;

  if (BASE == NULL){
    BASE = tcxConnectOptional(TCX_BASE_MODULE_NAME);
    
    if (BASE != NULL){
      fprintf(stderr, "PANTILT: BASE connected.\n");
      data.subscribe_status_report = 2;
      data.subscribe_sonar_report  = 0;
      data.subscribe_colli_report  = 0;
      data.subscribe_ir_report     = 0;
      data.subscribe_laser_report  = 0;
      tcxSendMsg(BASE, "BASE_register_auto_update", &data);
    }
  }
}


void BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				       BASE_robot_position_reply_ptr pos)
{
;
}

struct timeval com_time = {0, 0};

void BASE_update_status_reply_handler(TCX_REF_PTR                   ref,
				      BASE_update_status_reply_ptr status)
{
  float delta_x, delta_y, angle, distance, tilt;
  struct timeval this_time;
  float  time_difference;

  
  if (tracking_on){
    gettimeofday(&this_time, NULL);
    time_difference = ((float) (this_time.tv_sec - com_time.tv_sec))
      + (((float) (this_time.tv_usec - com_time.tv_usec)) / 1000000.0);
    if (time_difference > PANTILT_TIME_NEXT_TRACK){
      fprintf(stderr, "Robot at: %6.4f %6.4f %6.4f\n", 
	 status->pos_x, status->pos_y, status->orientation);
      fprintf(stderr, "Tracking: %6.4f %6.4f\n", tracking_x, tracking_y); 
      delta_x = tracking_x - status->pos_x;
      delta_y = tracking_y - status->pos_y;
      distance = sqrt((delta_x * delta_x) + (delta_y * delta_y));
      if (distance == 0.0){
	tilt = 0.0;
	angle = -90.0;
      }
      else{
	tilt = atan(fabs(tracking_height)/distance) * 180.0 / M_PI;
	if (tracking_height < 0.0)
	  tilt = 0.0 - tilt;
	angle = - ((atan2(delta_x, delta_y) ) * 180.0 / M_PI);
	/* fprintf(stderr, "Angle: %6.4f", angle); */ 
	angle = angle + status->orientation - 90.0;
	/* fprintf(stderr, " -> %6.4f", angle); */
      }
      for (; angle >= ((float) limits.pan_max_angle);) angle -= 360.0;
      for (; angle <  ((float) limits.pan_min_angle);) angle += 360.0;
      /* fprintf(stderr, " -> %6.4f\n\n", angle); */ 
      if (tilt < limits.tilt_min_angle) tilt = limits.tilt_min_angle + 0.1;
      if (tilt > limits.tilt_max_angle) tilt = limits.tilt_max_angle - 0.1;
      PANTILT_move(angle,tilt);
      send_automatic_status_update();
    }
    gettimeofday(&com_time, NULL);
  }
}
void BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
					BASE_action_executed_reply_ptr data)
{;}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/************************************************************************
 *
 *   NAME:         main()
 *                 
 *   FUNCTION:     Installs handlers, runs TCX loop
 *                 
 *   PARAMETERS:   none, yet
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


main(int argc, char **argv)
{
  struct bParamList * paramList = NULL;
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    PANTILT_messages,
    BASE_messages
  };
  fd_set readMask;

  BASE = NULL;

  /*
   * Set some parameters
   */

  /* add some defaults */
  paramList = bParametersAddEntry(paramList, "*", "pantilt.bps", "9600");
  paramList = bParametersAddEntry(paramList, "*", "pantilt.dev", "/dev/cur1");
  paramList = bParametersAddEntry(paramList, "", "TCXHOST", "localhost");
  paramList = bParametersAddEntry(paramList, "", "fork", "yes");

  /* add some parameter files */
  paramList = bParametersAddFile(paramList, "etc/beeSoft.ini");

#ifdef UNIBONN
  paramList = bParametersAddEntry(paramList, "", "fork", "no");
#endif

  /* add some enviroment variables */
  paramList = bParametersAddEnv(paramList, "", "TCXHOST");

  /* add command line arguements */
  paramList = bParametersAddArray(paramList, "", argc, argv);

  /* here is where we should add non "parameter format" options */


  /* Fill the the global bRobot struct */

  bParametersFillParams(paramList);

  if (!strcmp("none", bParametersGetParam(paramList, "robot", "pantilt"))) {
    fprintf(stderr,
	    "Pantilt name set to 'none'.  Check the robot.pantilt\n"
	    "value in beeSoft.ini.\n");
    exit(0);
  }
  
  /*
   *
   */

  if (argc >= 2)
    global_pan_correction = atoi(argv[1]);

  fprintf(stderr, "Global pan correction = %g.\n",
	  global_pan_correction);
  
  panTilt_device.dev.ttydev.ttyPort  = (char *)bRobot.pantilt_dev;
  panTilt_device.dev.ttydev.baudCode = bRobot.pantilt_bps;
#if 0
  panTilt_device.dev.debug = 1;
#endif

  fprintf(stderr, "Connecting to TCX...");

  tcxInitialize(TCX_PANTILT_MODULE_NAME, (char *)bRobot.TCXHOST);

  check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
		       BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
		       NULL, 0);
  check_version_number(libezx_major, libezx_minor,
		       libezx_robot_type, libezx_date,
		       "libezx", 0);
  check_version_number(librobot_major, librobot_minor,
		       librobot_robot_type, librobot_date,
		       "librobot", 1);


  fprintf(stderr, "done.\n");

  
#ifdef i386
  fprintf(stderr, "Connecting to PANTILT device...");
  error_value = PANTILT_init();	/* so far, we assume that a physical
				 * pan/tilt exists iff the machine
				 * is a i386. This allows us to run
				 * the code on a SPARC without any changes */
  fprintf(stderr, "done.\n");
#endif

  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));

  tcxRegisterHandlers(PANTILT_handler_array, 
		      sizeof(PANTILT_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(BASE_reply_handler_array,
		      sizeof(BASE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
  
  tcxRegisterCloseHnd(PANTILT_close_handler);

#ifdef i386
  fprintf(stderr, "-#1-");
  PANTILT_limits(&(limits.pan_min_angle), &(limits.pan_max_angle),
		 &(limits.tilt_min_angle), &(limits.tilt_max_angle), 
		 &(limits.pan_max_velocity), &(limits.tilt_max_velocity));
  fprintf(stderr, "-#2-");
  PANTILT_setAcceleration (8000.0, 8000.0);
  fprintf(stderr, "-#3-");
  PANTILT_setVelocity(PAN_HIGH_SPEED, TILT_HIGH_SPEED);
#ifdef UNIBONN
  fprintf(stderr, "\n");
#else
  fprintf(stderr, "-#4-");
  PANTILT_move (-90.0, 10.0);
  fprintf(stderr, "-#5-");
  PANTILT_move (0.0, 0.0);
#endif
#endif
  /*
  tracking_x = 1065.2010;
  tracking_y = -4.3956;
  tracking_on = 1;
*/
  connect_BASE();

  if (bRobot.fork) {
    bDaemonize("PANTILT.log");
  }

  for (;;){
    send_automatic_status_update();
    TCX_waiting_time.tv_sec    = 0;
    TCX_waiting_time.tv_usec   = 100000;
    tcxRecvLoop((void *) &TCX_waiting_time);
  }
}
