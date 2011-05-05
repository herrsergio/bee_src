
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




#ifndef LASER_messages_defined
#define LASER_messages_defined





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

#define TCX_LASER_MODULE_NAME "LASER"


#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR LASER;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR LASER;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** LASER data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/

#define FRONT_LASER 0
#define BACK_LASER  1

typedef struct {
  int   f_numberOfReadings;
  int*  f_reading;
  float f_startAngle;
  float f_angleResolution;

  int   r_numberOfReadings;
  int*  r_reading;
  float r_startAngle;
  float r_angleResolution;
  float xPos;
  float yPos;
  float rotPos;
} LASER_laser_reply_type, *LASER_laser_reply_ptr;

#define LASER_laser_reply_format "{int, <{int} : 1>, float, float, int, <{int} : 5>, float, float, float, float, float}"


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** LASER commands - these are the commands/queries understood by LASER ****/


#ifdef TCX_define_variables		/* do this exactly once! */

#define LASER_messages \
  {"LASER_laser_reply",   LASER_laser_reply_format}


#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




#ifdef DEFINE_REPLY_HANDLERS


/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with LASER ******/




/******* (a) Procedure headers ******/




void LASER_laser_reply_handler(TCX_REF_PTR                ref,
			       LASER_laser_reply_ptr      data);




/******* (b) Handler array ******/



TCX_REG_HND_TYPE LASER_reply_handler_array[] = {
  {"LASER_laser_reply", "LASER_laser_reply_handler",
     LASER_laser_reply_handler, TCX_RECV_ALL, NULL}
};


#endif

#endif
