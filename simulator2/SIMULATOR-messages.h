
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

#ifndef SIMULATOR_messages_defined
#define SIMULATOR_messages_defined





#include "tcx.h"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* 
 *  Notice: this file is part of BASE-messages.
 */


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** TCX module name - this is a unique identifier for TCX *******/

#define TCX_SIMULATOR_MODULE_NAME "SIMULATOR"



#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR SIMULATOR;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR SIMULATOR;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** SIMULATOR data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/




/*
 **** messages from the BASE module to the SIMULATOR. In plain text.
 * 
 * void SIMULATOR_message_from_base_handler(TCX_REF_PTR ref, char **message) 
 *
 */

#define SIMULATOR_message_from_base_format "string"
#define SIMULATOR_message_from_baseServer_format "[char:6]"





/*
 **** messages from the SIMULATOR back to the BASE module. In plain text.
 * 
 * void SIMULATOR_message_to_base_handler( TCX_REF_PTR ref, char **message) 
 *
 */

#define SIMULATOR_message_to_base_format "string"
#define SIMULATOR_message_to_baseServer_format "[char:128]"





/***************************************************************
 ***************************************************************
 ***************************************************************/

#define SIMULATOR_message_from_irServer_format "[char:132]"

typedef struct {
  int count;
  long *values;
} SIMULATOR_message_to_irServer_type;

#define SIMULATOR_message_to_irServer_format "{int, <int:1>}"


/***************************************************************
 ***************************************************************
 ***************************************************************/

#define SIMULATOR_message_from_tactileServer_format "[char:132]"

typedef struct {
  int count;
  long *values;
} SIMULATOR_message_to_tactileServer_type;

#define SIMULATOR_message_to_tactileServer_format "{int, <int:1>}"


/***************************************************************
 ***************************************************************
 ***************************************************************/

#define SIMULATOR_message_from_sonarServer_format "[char:132]"

#if 0 /* this marks a comment */

This struct definition is from sonarClient.h

typedef struct sonarType {
  int value;			/* the distance */
  int mostRecent;		/* was this one in most recent return? */
  struct timeval time;		/* time of capture.  not yet precise */
} sonarType;

#endif /* comment */

#define SIMULATOR_message_to_sonarServer_format "[{int, int, {long, long}}:24]"

/***************************************************************
 ***************************************************************
 ***************************************************************/

/*
 * messages from the SONAR module to the SIMULATOR. In plain text.
 * void SIMULATOR_message_from_sonar_handler(TCX_REF_PTR ref, char **message) 
 */

#define SIMULATOR_message_from_sonar_format "string"

/*
 * messages from the SIMULATOR back to the SONAR module. In plain text.
 * void SIMULATOR_message_to_sonar_handler(TCX_REF_PTR ref, char **message) 
 */

#define SIMULATOR_message_to_sonar_format "string"

/*
 **** messages from the SIMULATOR  to the LASER module in the given format
 * 
 * void SIMULATOR_message_to_sonar_handler(TCX_REF_PTR ref, char **message) 
 *
 */


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

typedef struct {
  int   f_numberOfReadings;
  int*  f_reading;
  float f_startAngle;
  float f_angleResolution;
  int   r_numberOfReadings;
  int*  r_reading;
  float r_startAngle;
  float r_angleResolution;
} SIMULATOR_message_to_laser_type, *SIMULATOR_message_to_laser_ptr;


#define SIMULATOR_message_to_laser_format "{int, <{int} : 1>, float, float, int, <{int} : 5>, float, float}"


typedef struct {
    float x;
    float y;
    float rot;
} SIMULATOR_set_robot_position_type, *SIMULATOR_set_robot_position_ptr;

#define SIMULATOR_set_robot_position_format "{float, float, float}"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** SIMULATOR commands - 
 **** these are the commands/queries understood by SIMULATOR ****/


#ifdef TCX_define_variables		/* do this exactly once! */

#define SIMULATOR_messages \
  {"SIMULATOR_message_from_base",  SIMULATOR_message_from_base_format},\
  {"SIMULATOR_message_to_base",    SIMULATOR_message_to_base_format},\
  {"SIMULATOR_message_from_sonar", SIMULATOR_message_from_sonar_format},\
  {"SIMULATOR_message_to_sonar",   SIMULATOR_message_to_sonar_format},\
  {"SIMULATOR_message_to_laser",   SIMULATOR_message_to_laser_format},\
  {"SIMULATOR_message_from_baseServer", \
     SIMULATOR_message_from_baseServer_format},\
  {"SIMULATOR_message_to_baseServer", \
     SIMULATOR_message_to_baseServer_format},\
  {"SIMULATOR_message_from_sonarServer", \
     SIMULATOR_message_from_sonarServer_format},\
  {"SIMULATOR_message_to_sonarServer", \
     SIMULATOR_message_to_sonarServer_format},\
  {"SIMULATOR_message_from_irServer", \
     SIMULATOR_message_from_irServer_format},\
  {"SIMULATOR_message_to_irServer", \
     SIMULATOR_message_to_irServer_format},\
  {"SIMULATOR_message_from_tactileServer", \
     SIMULATOR_message_from_tactileServer_format},\
  {"SIMULATOR_message_to_tactileServer", \
     SIMULATOR_message_to_tactileServer_format},\
  {"SIMULATOR_set_robot_position", \
     SIMULATOR_set_robot_position_format}
#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




#ifdef DEFINE_REPLY_HANDLERS


/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with SIMULATOR ******/




/******* (a) Procedure headers ******/



void SIMULATOR_message_to_base_handler(TCX_REF_PTR   ref,
				       char **message);

void SIMULATOR_message_to_baseServer_handler(TCX_REF_PTR   ref,
					     char *message);



void SIMULATOR_message_to_sonar_handler(TCX_REF_PTR   ref,
					char **message);

void SIMULATOR_message_to_laser_handler(TCX_REF_PTR   ref,
					SIMULATOR_message_to_laser_ptr	data);


/******* (b) Handler array ******/



TCX_REG_HND_TYPE SIMULATOR_reply_handler_array[] = {
  {"SIMULATOR_message_to_base", "SIMULATOR_message_to_base_handler",
     SIMULATOR_message_to_base_handler, TCX_RECV_ALL, NULL},
  {"SIMULATOR_message_to_sonar", "SIMULATOR_message_to_sonar_handler",
     SIMULATOR_message_to_sonar_handler, TCX_RECV_ALL, NULL},
  {"SIMULATOR_message_to_laser", "SIMULATOR_message_to_laser_handler",
     SIMULATOR_message_to_laser_handler, TCX_RECV_ALL, NULL}
};


#endif



/* P.W. */


extern 	TCX_MODULE_PTR MODULE_BASE; 		/* will be != NULL if BASE
					    * has already sent a message.
					    * Only then we will send any
					    * status-messages */
extern 	TCX_MODULE_PTR MODULE_SONAR;     	/* will be != NULL if SONAR
					    * has already sent a message.
					    * Only then we will send any
					    * status-messages */
extern TCX_MODULE_PTR MODULE_IR;
extern TCX_MODULE_PTR MODULE_TACTILE;

#endif
