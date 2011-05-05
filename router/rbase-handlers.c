
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
#include "sonar_interface.h"

#define TCX_define_variables /* this makes sure variables are installed */
#include "BASE-messages.h"
#include "SONAR-messages.h"
#include "COLLI-messages.h"
#include "LASER-messages.h"
#include "IR-messages.h"

/* #include "collision.h" */
#include "colli-handlers.h"

#define DEFINE_REPLY_HANDLERS
/* #include "SIMULATOR-messages.h" */


#include "base-handlers.h"

#include "router.h"

#include "beeSoftVersion.h"
#include "libezx.h"
#include "librobot.h"
#include <bUtils.h>


/*---- 'BASE_debug' prints out messages upon receipt of a TCX message ----*/
/* #define BASE_debug */

/***************************************************************************
 * GLOBAL VARIABLES                                                        *
 ***************************************************************************/
BASE_update_status_reply_type base_tcx_status;
SONAR_sonar_reply_type        sonar_tcx_status;


extern int SONAR_AUTOREPLY_IS_ON;
extern int STATUS_AUTOREPLY_IS_ON;
extern int COLLI_AUTOREPLY_IS_ON;

extern int NumberOfStatusReplyModules;
extern int StatusAutoReplyList[];

extern RouterModuleReferenceType RMR[];
extern int SONAR_BUFFER[];
extern int verbose_on;

TCX_MODULE_PTR REAL_BASE;
unsigned int STATUS_REPLIES=0;

void tcx_register_colli ( void );
/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/******* auto-reply stuff, data definitions ***************/


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
  int SenderModule;
  int error_value = 1;		/* only relevant if not i386 machine! 
				 * but then there is no physical base/sonar
				 * anyhow. Thus, default = no error (=1) */

  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      if ( verbose_on == 1 )
	fprintf (stderr,"\nGot Autoreply request from Module %d with %d %d %d %d %d",
		 SenderModule,
		 data->subscribe_status_report,data->subscribe_sonar_report,
		 data->subscribe_laser_report,data->subscribe_ir_report,
		 data->subscribe_colli_report );
      
      SetupAutoReplyForModule ( SenderModule,
				data->subscribe_status_report,
				data->subscribe_sonar_report,
				data->subscribe_colli_report,
				data->subscribe_laser_report);
    }
  else
    fprintf (stderr,"\nROUTER Error : Subscribing module not known !");

  tcxFree("BASE_register_auto_update", data);
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
  int SenderModule,pass_thru=0;

  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      pass_thru = IncreaseQuery ( SenderModule , POSITION_QUERY );
      if ( pass_thru == 1 )
	{
	  tcxSendMsg ( REAL_BASE , "BASE_robot_position_query", NULL );
	  if ( verbose_on == 1 )
	    fprintf (stderr,"\nSEND A SINGLE POSITION QUERY ...!!!");
	}
    }
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE,"BASE_stop_robot",NULL );
    }
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "BASE_reset_joystick", NULL );
    }
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "BASE_notify_wall_orientation" , orientation );
    }
  
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
  int SenderModule,pass_thru=0;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      pass_thru = IncreaseQuery ( SenderModule , STATUS_QUERY );
      if ( pass_thru == 1 )
	{
	  tcxSendMsg ( REAL_BASE , "BASE_update_status_query", NULL );
	  if ( verbose_on == 1 )
	    fprintf (stderr,"\nSEND A SINGLE STATUS QUERY ...!!!");
	}
    } 
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

/* this query will be returned from a buffer */
void BASE_sonar_query_handler(TCX_REF_PTR     ref,
			      void           *data)
{  
  int SenderModule,i,pass_thru;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      if ( SONAR_AUTOREPLY_IS_ON == 1 ) /* use this if buffered from autor. */
	{
	  for ( i=0; i<24 ; i++)
	    sonar_tcx_status.values[i] = SONAR_BUFFER[i];
	  tcxReply ( ref, "BASE_sonar_reply",&sonar_tcx_status);
	  if ( verbose_on == 1 )
	    fprintf (stderr,"\nReplied BASE_SONAR directly !");
	}
      else
	{
	  pass_thru = IncreaseQuery ( SenderModule , SONAR_QUERY );
	  if ( pass_thru == 1 )
	    {
	      tcxSendMsg ( REAL_BASE , "BASE_sonar_query", NULL );
	      if ( verbose_on == 1 )
		fprintf (stderr,"\nSEND A SINGLE BASE SONAR QUERY ...!!!");
	    }
	}
    } 
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "BASE_set_velocity", velocity );
    } 
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "BASE_set_acceleration" , acceleration);
    } 
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "BASE_rotate_clockwise",NULL );
    }
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "BASE_rotate_anticlockwise",NULL );
    }
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "BASE_translate_forward", NULL );
    }
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "BASE_translate_backward", NULL );
    }
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "BASE_translate_by", dist );
    }
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "BASE_rotate_by",angle );
    }
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "BASE_goto_relative" , pos );
    }
  
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "BASE_goto_absolute", pos );
    }
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "BASE_approach_absolute", pos );
    }
  
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , 
		  "BASE_approach_absolute_two_points", pos );
    }
  tcxFree("BASE_approach_absolute_two_points", pos);
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


void BASE_setmode_handler(TCX_REF_PTR ref, BASE_setmode_ptr mode)
{
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "BASE_setmode" , mode );
    }
  tcxFree("BASE_setmode", mode);
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
  int SenderModule;
   
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      ChangeModuleStatus ( SenderModule , DISCONNECTED );
    }
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
  int what_module;
#ifdef BASE_debug
  fprintf(stderr, "BASE: closed connection detected: %s\n", name);
#endif

  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    fprintf (stderr,"\nTCX SERVER SHUTDOWN DETECTED !");
    fprintf (stderr,"\nEXITING NOW ...");
    exit(0);
  }
  if (!strcmp(name, "BASE_ROUTED"))
    {
      fprintf (stderr,"\n\nREAL BASE SHUTDOWN DETECTED");
      fprintf (stderr,"\nROUTER SHUTDOWN INITIATED ...");
      fprintf (stderr,"\nDONE !");
      exit(0);
    }
  what_module = SearchModuleByName ( name );
  fprintf (stderr,"\nCLOSE CONNECTION DETECTED FROM %s %d",name,what_module );
  ChangeModuleStatus ( what_module , DISCONNECTED );
}

/***************************************************************************
 * BASE_robot_position_reply_handler                                       *
 ***************************************************************************/
void BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				       BASE_robot_position_reply_ptr pos)
{
  int i,count=0;
  
  
  for ( i=0 ; i<MaxNumberOfModules ; i++ )
    {
      if ( GetQuery ( i , POSITION_QUERY ) > 0 )
	{
	  tcxSendMsg ( RMR[i].module,
		      "BASE_robot_position_reply", pos );
	  DecreaseQuery ( i , POSITION_QUERY );
	  count++;
	}
    }
  if ( verbose_on == 1 )
    fprintf (stderr,"P%d",count);
  tcxFree ("BASE_robot_position_reply",pos );
}

/***************************************************************************
 * BASE_update_status_reply_handler                                        *
 ***************************************************************************/
void BASE_update_status_reply_handler(TCX_REF_PTR                   ref,
				      BASE_update_status_reply_ptr status)
{
  int i,count=0;
  
  for ( i=0 ; i<NumberOfStatusReplyModules ; i++)
    {
      if (RMR[StatusAutoReplyList[i]].ModuleStatus == ACTIVE )
	{
	  if ( (STATUS_REPLIES % RMR[StatusAutoReplyList[i]].status) == 0)
	    {
	      tcxSendMsg ( RMR[StatusAutoReplyList[i]].module,
			  "BASE_update_status_reply", status );
	      count++;
	    }
	}
    }

   for ( i=0 ; i<MaxNumberOfModules ; i++ )
    {
      if ( RMR[i].status == 0 )   /* final security */
	{
	  if ( GetQuery ( i , STATUS_QUERY ) > 0 )
	    {
	      tcxSendMsg ( RMR[i].module,
			  "BASE_update_status_reply", status );
	      DecreaseQuery ( i , STATUS_QUERY );
	    }
	}
    }

  STATUS_REPLIES++;
  if (verbose_on == 1)
    fprintf (stderr,"B%d",count);
  tcxFree ("BASE_update_status_reply",status);
}

/***************************************************************************
 * BASE_action_executed_reply_handler                                      *
 ***************************************************************************/
void BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
					BASE_action_executed_reply_ptr data)
{
  int i,count=0;
  
  for ( i=0 ; i<NumberOfStatusReplyModules ; i++)
    {
      if (RMR[StatusAutoReplyList[i]].ModuleStatus == ACTIVE )
	{
	  if ( RMR[StatusAutoReplyList[i]].status > 0 )
	    {
	      tcxSendMsg ( RMR[StatusAutoReplyList[i]].module,
			  "BASE_action_executed_reply", data );
	      count++;
	    }
	}
    }
  if ( verbose_on == 1 )
    fprintf (stderr,"E%d",count);
  tcxFree ("BASE_action_executed_reply",data );
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
  {"BASE_reset_joystick", "BASE_reset_joystick_handler",
     BASE_reset_joystick_handler, TCX_RECV_ALL, NULL},
  {"BASE_notify_wall_orientation", "BASE_notify_wall_orientation_handler",
     BASE_notify_wall_orientation_handler, TCX_RECV_ALL, NULL},

  {"COLLI_vision_line", "COLLI_vision_line_handler",
     COLLI_vision_line_handler, TCX_RECV_ALL, NULL},
  {"COLLI_vision_point", "COLLI_vision_point_handler",
     COLLI_vision_point_handler, TCX_RECV_ALL, NULL},
  {"COLLI_parameter", "COLLI_parameter_handler",
     COLLI_parameter_handler, TCX_RECV_ALL, NULL}
};


/* copied from BASE-messages.h */ 
TCX_REG_HND_TYPE BASE_reply_handler_array[] = {

  {"BASE_robot_position_reply", "BASE_robot_position_reply_handler",
     BASE_robot_position_reply_handler, TCX_RECV_ALL, NULL},
  {"BASE_update_status_reply", "BASE_update_status_reply_handler",
     BASE_update_status_reply_handler, TCX_RECV_ALL, NULL},
  {"BASE_action_executed_reply", "BASE_action_executed_reply_handler",
     BASE_action_executed_reply_handler, TCX_RECV_ALL, NULL}
};

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_register_sonar(void);
void tcx_register_laser(void);

void init_tcx(void)
{
  char *tcxMachine = NULL;
  


  TCX_REG_MSG_TYPE TCX_message_array[] = {
    BASE_messages,
    SONAR_messages,
    COLLI_messages,
    LASER_messages
  };

  fprintf(stderr, "Connecting to TCX...");
  fflush(stderr);
      

  tcxMachine = (char *)bRobot.TCXHOST;

  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize(TCX_BASE_MODULE_NAME, tcxMachine);
  check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
		       BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
		       NULL, 0);
  check_version_number(librobot_major, librobot_minor,
		       librobot_robot_type, librobot_date,
		       "librobot", 0);
  check_version_number(libezx_major, libezx_minor,
		       libezx_robot_type, libezx_date,
		       "libezx", 1);


  fprintf(stderr, "done.\n");
  fflush(stderr);

  /* Router specific changes to the original BASE module */
  InitRouterModuleReferenceArray();
  /*
  InitRouterMessageReferenceArray();
  */
  Connect2RealBASE ();
  
  
  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));

  if ( verbose_on ==1 )
    fprintf (stderr,"\nRegistered Messages ...");
  
  SubscribeForAutoReply();

  tcxRegisterHandlers(BASE_handler_array, 
		      sizeof(BASE_handler_array) / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(BASE_reply_handler_array,
		      sizeof(BASE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  if ( verbose_on ==1 )
    fprintf (stderr,"\nRegistered BASE handlers ...");
  tcx_register_sonar();
  if ( verbose_on ==1 )
    fprintf (stderr,"\nRegistered SONAR handlers ...");
  tcx_register_laser();
  if ( verbose_on ==1 )
    fprintf (stderr,"\nRegistered LASER handlers ...");
  tcx_register_colli();
  if ( verbose_on ==1 )
    fprintf (stderr,"\nRegistered COLLI handlers ...");
  
  fprintf (stderr,"\n\nSTARTING ...");
  
  tcxRegisterCloseHnd(BASE_close_handler);
}


