
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
#include <signal.h>
#include <sys/time.h>

#include "tcx.h"
#include "tcxP.h"



#define TCX_define_variables 		/*** this makes sure variables are installed ***/
#include "SIMULATOR-messages.h"

#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */

#include "BASE-messages.h"
#include "robot.h"

#include "beeSoftVersion.h"
#include "libbUtils.h"
#include "librobot.h"



extern Boolean use_baseServer;
extern Boolean use_sonarServer;

extern char just_viewing;

void setRobotPosition(float,float,float);
void BeamRobot3(float,float,float);

static int tcx = 1;			/*** indicats if TCX shall be used, If not,
				 	 *** we will allow a simple simulator
				 	 *** test, nothing else. 
				 	 ***/

char	BaseCommand[81];
char	SonarCommand[81];



/*** TCX routines by S. Thrun ***/


/************************************************************************
 *
 *   NAME:         SIMULATOR_message_from_base_handler()
 *                 
 *   FUNCTION:     Receiving handler for BASE bessages.
 *                 
 *   PARAMETERS:   standard TCX parameters and a text string (message)
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SIMULATOR_message_from_base_handler(TCX_REF_PTR   ref,
					 char        **message)
{
  void	 base(char command[]);

#ifdef SIMULATOR_debug
  fprintf(stderr, "Received a SIMULATOR_message_from_base from %s: [%s]\n",
	  tcxModuleName(ref->module), *message);
#endif

  /*=================================================================
   *======= The first module that sends anything with this message
   *======= will be considered BASE. From now on, every outgoing BASE
   *======= message will be sent to this module.
   *======= Here we "register" this module in MODULE_BASE ===========*/

  if (MODULE_BASE == NULL){
    MODULE_BASE = ref->module;
    fprintf(stderr, "Registered module %s as MODULE_BASE.\n",
	    tcxModuleName(ref->module));
    use_baseServer = FALSE;
  }


  /***======= Reply to BASE - just for fun. =======***/

  /*
  if (MODULE_BASE != NULL){
    char *reply_msg;

    reply_msg = (char *) malloc(256);
    strcpy(reply_msg, "base connection established !");
    
    fprintf(stderr, "%10s:%5d:%14s(): \n",
    __FILE__, __LINE__, __FUNCTION__);
    
    tcxSendMsg(MODULE_BASE, "SIMULATOR_message_to_base", &reply_msg);

    free(reply_msg);
  }  
  */

  strcpy(BaseCommand, *message);
  base(BaseCommand);

#ifdef SIMULATOR_debug
	fprintf(stderr, "BaseCommand = %s\n", BaseCommand);
#endif

  /***======= free the message memory - do not remove! =======***/

  tcxFree("SIMULATOR_message_from_base", message);
}



/************************************************************************
 *
 *   NAME:         SIMULATOR_message_from_baseServer_handler()
 *                 
 *   FUNCTION:     Receiving handler for baseServer bessages.
 *                 
 *   PARAMETERS:   Binary character array - unsigned char[6]
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SIMULATOR_message_from_baseServer_handler(TCX_REF_PTR   ref,
					       char *message)
{
  void	 baseServer(unsigned char *cmd);

#ifdef SIMULATOR_debug
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);

  fprintf(stderr,
	  "Received a SIMULATOR_message_from_baseServer from %s:\n",
	  tcxModuleName(ref->module));
  fprintf(stderr, "\t\t\t%02X:%02X:%02X:%02X:%02X:%02X\n",
	  message[0],  message[1],  message[2],
	  message[3],  message[4],  message[5]);
#endif

  /*=================================================================
   *======= The first module that sends anything with this message
   *======= will be considered BASE. From now on, every outgoing BASE
   *======= message will be sent to this module.
   *======= Here we "register" this module in MODULE_BASE ===========*/

  if (MODULE_BASE == NULL){
    MODULE_BASE = ref->module;

    fprintf(stderr, "%10s:%5d:%14s(): \n",
	    __FILE__, __LINE__, __FUNCTION__);

    fprintf(stderr, "Registered module %s as MODULE_BASE.\n",
	    tcxModuleName(ref->module));
    use_baseServer = TRUE;
  }

  baseServer(message);

  /***======= free the message memory - do not remove! =======***/

  tcxFree("SIMULATOR_message_from_baseServer", message);

}



/************************************************************************
 *
 *   NAME:         SIMULATOR_message_from_sonar_handler()
 *                 
 *   FUNCTION:     Receiving handler for SONAR bessages.
 *                 
 *   PARAMETERS:   standard TCX parameters and a text string (message)
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SIMULATOR_message_from_sonar_handler(TCX_REF_PTR   ref,
					 char        **message)
{

  void	sonar(char command[]);

  

#ifdef SIMULATOR_debug
  fprintf(stderr, "Received a SIMULATOR_message_from_sonar from %s: [%s]\n",
	  tcxModuleName(ref->module), *message);
#endif

  /*==================================================================
   *======= The first module that sends anything with this message
   *======= will be considered SONAR. From now on, every outgoing SONAR
   *======= message will be sent to this module.
   *======= Here we "register" this module in MODULE_SONAR ===========*/

  if (MODULE_SONAR == NULL){
    MODULE_SONAR = ref->module;
    fprintf(stderr, "Registered module %s as MODULE_SONAR.\n",
	    tcxModuleName(ref->module));
    use_sonarServer = FALSE;
  }


  /***======= Reply to SONAR - just for fun. =======***/

  /*
  if (MODULE_SONAR != NULL){
    char *reply_msg;

    reply_msg = (char *) malloc(256);
    strcpy(reply_msg, "sonar connection established !");

    fprintf(stderr, "%10s:%5d:%14s(): tcxSendMsg\n",
    __FILE__, __LINE__, __FUNCTION__);

    tcxSendMsg(MODULE_SONAR, "SIMULATOR_message_to_sonar", &reply_msg);

    free(reply_msg);
  }  
  */

  strcpy(SonarCommand, *message);
  sonar(SonarCommand);

  /***======= free the message memory - do not remove! =======***/


  tcxFree("SIMULATOR_message_from_sonar", message);
}



/************************************************************************
 *
 *   NAME:         SIMULATOR_message_from_sonarServer_handler()
 *                 
 *   FUNCTION:     Receiving handler for SONAR bessages from sonarServer
 *                 
 *   PARAMETERS:
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
SIMULATOR_message_from_sonarServer_handler(TCX_REF_PTR   ref,
					   char        *message)
{

#ifdef SIMULATOR_debug
  fprintf(stderr,
	  "Received a SIMULATOR_message_from_sonarServer from %s: [%s]\n",
	  tcxModuleName(ref->module), "[binary message]");
#endif

  /*==================================================================
   *======= The first module that sends anything with this message
   *======= will be considered SONAR. From now on, every outgoing SONAR
   *======= message will be sent to this module.
   *======= Here we "register" this module in MODULE_SONAR ===========*/

  if (MODULE_SONAR == NULL){
    MODULE_SONAR = ref->module;
    fprintf(stderr, "Registered module %s as MODULE_SONAR.\n",
	    tcxModuleName(ref->module));
    use_sonarServer = TRUE;
  }

  sonarServer(message);

  /***======= free the message memory - do not remove! =======***/

  tcxFree("SIMULATOR_message_from_sonarServer", message);
}



/************************************************************************
 *
 *   NAME:         SIMULATOR_message_from_irServer_handler()
 *                 
 *   FUNCTION:     Receiving handler for IR bessages from irServer
 *                 
 *   PARAMETERS:
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
SIMULATOR_message_from_irServer_handler(TCX_REF_PTR   ref,
					char        *message)
{

#ifdef SIMULATOR_debug
  fprintf(stderr, "Received a SIMULATOR_message_from_irServer from %s: [%s]\n",
	  tcxModuleName(ref->module), "[binary message]");
#endif

  /*==================================================================
   *======= The first module that sends anything with this message
   *======= will be considered IR. From now on, every outgoing SONAR
   *======= message will be sent to this module.
   *======= Here we "register" this module in MODULE_IR ===========*/

  if (MODULE_IR == NULL){
    MODULE_IR = ref->module;
    fprintf(stderr, "Registered module %s as MODULE_IR.\n",
	    tcxModuleName(ref->module));
  }

  irServer(message);

  /***======= free the message memory - do not remove! =======***/

  tcxFree("SIMULATOR_message_from_irServer", message);
}



/************************************************************************
 *
 *   NAME:         SIMULATOR_message_from_tactileServer_handler()
 *                 
 *   FUNCTION:     Receiving handler for TACTILE bessages from tactileServer
 *                 
 *   PARAMETERS:
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
SIMULATOR_message_from_tactileServer_handler(TCX_REF_PTR   ref,
					char        *message)
{

#ifdef SIMULATOR_debug
  fprintf(stderr, "Received a SIMULATOR_message_from_tactileServer from %s: [%s]\n",
	  tcxModuleName(ref->module), "[binary message]");
#endif

  /*==================================================================
   *======= The first module that sends anything with this message
   *======= will be considered TACTILE. From now on, every outgoing SONAR
   *======= message will be sent to this module.
   *======= Here we "register" this module in MODULE_TACTILE ===========*/

  if (MODULE_TACTILE == NULL){
    MODULE_TACTILE = ref->module;
    fprintf(stderr, "Registered module %s as MODULE_TACTILE.\n",
	    tcxModuleName(ref->module));
  }

  tactileServer(message);

  /***======= free the message memory - do not remove! =======***/

  tcxFree("SIMULATOR_message_from_tactileServer", message);
}



/************************************************************************
 *
 *   NAME:         SIMULATOR_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SIMULATOR_close_handler(char *name, TCX_MODULE_PTR module)
{
#ifdef SIMULATOR_debug
  fprintf(stderr, "SIMULATOR: closed connection detected: %s\n", name);
#endif

  if (module == MODULE_BASE){
    fprintf(stderr, "Cancelled registration for MODULE_BASE.\n");
    MODULE_BASE = NULL;
    fprintf(stderr, "BASE died, and so will I. Bye-bye.\n");

    exit(0);
  }

  if (module == MODULE_SONAR){
    fprintf(stderr, "Cancelled registration for MODULE_SONAR.\n");
    MODULE_SONAR = NULL;
    exit(0);
  }

  if (module == MODULE_IR){
    fprintf(stderr, "Cancelled registration for MODULE_IR.\n");
    MODULE_SONAR = NULL;
    exit(0);
  }

  if (module == MODULE_TACTILE){
    fprintf(stderr, "Cancelled registration for MODULE_TACTILE.\n");
    MODULE_SONAR = NULL;
    exit(0);
  }

  if (!strcmp(name, "TCX Server")) { 		/*** TCX shut down ***/
    exit(0);
  }
}


void SIMULATOR_set_robot_position_handler(TCX_REF_PTR ref,
				      SIMULATOR_set_robot_position_ptr position)
{
#ifdef USER_debug
  fprintf(stderr, "TCX: Received a set_robot_position message.\n");
  fprintf(stderr, "robot: %g %g %g\n", 
	  position->x, position->y, position->rot);
#endif
  BeamRobot3(position->x,position->y,position->rot); 
  tcxFree("SIMULATOR_set_robot_position", position); /* don't remove this! */
}


/* -------------------------------------------------------------------- */


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         BASE_update_status_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_update_status_reply_handler(TCX_REF_PTR ref,
				      BASE_update_status_reply_ptr status)
{
#ifdef USER_debug
  fprintf(stderr, "TCX: Received a BASE_update_status_reply message.\n");
  fprintf(stderr, "robot: %g %g %g\n", 
	  status->pos_x, status->pos_y, status->orientation);
#endif
  setRobotPosition(status->pos_y,status->pos_x,status->orientation); 
  tcxFree("BASE_update_status_reply", status); /* don't remove this! */
}

void BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				       BASE_robot_position_reply_ptr pos){
  ;
}




void BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
					BASE_action_executed_reply_ptr data)
{
  fprintf(stderr,"Received a BASE_action_executed_reply\n");
}





/* -------------------------------------------------------------------- */



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/*** internal handlers for external messages ***/

TCX_REG_HND_TYPE SIMULATOR_handler_array[] = {
  {"SIMULATOR_message_from_base", "SIMULATOR_message_from_base_handler",
     SIMULATOR_message_from_base_handler, TCX_RECV_ALL, NULL},
  {"SIMULATOR_message_from_sonar", "SIMULATOR_message_from_sonar_handler",
     SIMULATOR_message_from_sonar_handler, TCX_RECV_ALL, NULL},
  {
    "SIMULATOR_message_from_baseServer",
    "SIMULATOR_message_from_baseServer_handler",
    SIMULATOR_message_from_baseServer_handler,
    TCX_RECV_ALL, NULL
  },
  {
    "SIMULATOR_message_from_sonarServer",
    "SIMULATOR_message_from_sonarServer_handler",
    SIMULATOR_message_from_sonarServer_handler,
    TCX_RECV_ALL, NULL
  },
  {
    "SIMULATOR_message_from_irServer",
    "SIMULATOR_message_from_irServer_handler",
    SIMULATOR_message_from_irServer_handler,
    TCX_RECV_ALL, NULL
  },
  {
    "SIMULATOR_message_from_tactileServer",
    "SIMULATOR_message_from_tactileServer_handler",
    SIMULATOR_message_from_tactileServer_handler,
    TCX_RECV_ALL, NULL
  },
  {
    "SIMULATOR_set_robot_position",
    "SIMULATOR_set_robot_position_handler",
    SIMULATOR_set_robot_position_handler,
    TCX_RECV_ALL, NULL
  }  
};

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/************************************************************************
 *
 *   NAME:         SIMULATOR_initialize_tcx()
 *                 
 *   FUNCTION:     Initializes and connects to TCX.
 *                 
 *   PARAMETERS:   name of the module
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



int
SIMULATOR_initialize_tcx(char *tcx_simulator_module_name,
			 const char *tcxMachine)
{
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    BASE_messages,
    SIMULATOR_messages
  };

  /*=========================================*
   *      initialize and connect to TCX
   *=========================================*/


  if (tcx_simulator_module_name != NULL){
    tcx = 1;
    
    fprintf(stderr, "Connecting to TCX...");
    tcxInitialize(tcx_simulator_module_name, (char *)tcxMachine);
    check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
			 BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
			 NULL, 0);
    check_version_number(librobot_major, librobot_minor,
			 librobot_robot_type, librobot_date,
			 "librobot", 0);
    check_version_number(libbUtils_major, libbUtils_minor,
			 libbUtils_robot_type, libbUtils_date,
			 "libbUtils", 1);



  

    
    if(just_viewing) {
      BASE_register_auto_update_type data;
      data.subscribe_status_report = 1;
      data.subscribe_sonar_report  = 0;
      data.subscribe_colli_report  = 0; 
      
      tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
			  / sizeof(TCX_REG_MSG_TYPE));  
      
      tcxRegisterHandlers(BASE_reply_handler_array,
			  sizeof(BASE_reply_handler_array)
			  / sizeof(TCX_REG_HND_TYPE));
/*  tcxRegisterHandlers(SONAR_reply_handler_array,
		      sizeof(SONAR_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE)); */

      fprintf(stderr, "Connecting to %s...", TCX_BASE_MODULE_NAME);
      BASE = tcxConnectModule(TCX_BASE_MODULE_NAME);
      fprintf(stderr, "done.\n");
      
      fprintf(stderr, "%10s:%5d:%14s(): tcxSendMsg\n",
	      __FILE__, __LINE__, __FUNCTION__);

      tcxSendMsg(BASE, "BASE_register_auto_update", &data);
    }
    else {
      tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
			  / sizeof(TCX_REG_MSG_TYPE));
      tcxRegisterHandlers(BASE_reply_handler_array,
			  sizeof(BASE_reply_handler_array)
			  / sizeof(TCX_REG_HND_TYPE));      
      tcxRegisterHandlers(SIMULATOR_handler_array, 
			  sizeof(SIMULATOR_handler_array)
			  / sizeof(TCX_REG_HND_TYPE));
    } 
    tcxRegisterCloseHnd(SIMULATOR_close_handler);
    
    fprintf(stderr, "done.\n");
  }
  else
    tcx = 0;
}
