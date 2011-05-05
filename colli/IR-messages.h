
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




#ifndef IR_messages_defined
#define IR_messages_defined





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

#define TCX_IR_MODULE_NAME "INFRA"


#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR INFRA;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR INFRA;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** IR data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/



typedef struct {
  int dummy;
} IR_ir_reply_type, *IR_ir_reply_ptr;

#define IR_ir_reply_format "{int}"


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** IR commands - these are the commands/queries understood by IR ****/


#ifdef TCX_define_variables		/* do this exactly once! */

#define IR_messages \
  {"IR_ir_reply",   IR_ir_reply_format}


#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




#ifdef DEFINE_REPLY_HANDLERS


/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with IR ******/




/******* (a) Procedure headers ******/




void IR_ir_reply_handler(TCX_REF_PTR                ref,
			 IR_ir_reply_ptr      data);




/******* (b) Handler array ******/



TCX_REG_HND_TYPE IR_reply_handler_array[] = {
{"IR_ir_reply", "IR_ir_reply_handler",
   IR_ir_reply_handler, TCX_RECV_ALL, NULL}
};


#endif

#endif
