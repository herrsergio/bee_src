
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




#ifndef SONAR_messages_defined
#define SONAR_messages_defined





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

#define TCX_SONAR_MODULE_NAME "SONAR"



#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR SONAR;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR SONAR;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** SONAR data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/




/* void SONAR_switch_on(void) */

#define SONAR_switch_on_format NULL



/* void SONAR_switch_off(void) */

#define SONAR_switch_off_format NULL
  



/* void SONAR_activate_mask(int) */

#define SONAR_activate_mask_format "int"




#define SONAR_sonar_query_format NULL
#define SONAR_ir_query_format NULL



typedef struct {
  float values[24];
} SONAR_sonar_reply_type, *SONAR_sonar_reply_ptr;

#define SONAR_sonar_reply_format "{[float : 24]}"

typedef struct {
  int upperrow[24];
  int lowerrow[24];
  int drow[8];
} SONAR_ir_reply_type, *SONAR_ir_reply_ptr;
      
#define SONAR_ir_reply_format "{[int : 24],[int : 24],[int : 8]}"
      



/* void SONAR_define_mask(SONAR_define_mask_type) */

typedef struct {
  int number;			/* Reference number of the user-defined mask.
				 * must be in 2..9.
				 * Currently 0 and 1 are built-in default 
				 * masks. */
  int length;			/* Length of user-defined sonar mask */
  int *user_mask;		/* User-mask. */
} SONAR_define_mask_type, *SONAR_define_mask_ptr;

#define SONAR_define_mask_format "{int, int, <int : 2>}" 

    





/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** SONAR commands - these are the commands/queries understood by SONAR ****/


#ifdef TCX_define_variables		/* do this exactly once! */

#define SONAR_messages \
  {"SONAR_switch_on",     SONAR_switch_on_format},\
  {"SONAR_switch_off",    SONAR_switch_off_format},\
  {"SONAR_activate_mask", SONAR_activate_mask_format},\
  {"SONAR_sonar_query",   SONAR_sonar_query_format},\
  {"SONAR_sonar_reply",   SONAR_sonar_reply_format},\
  {"SONAR_ir_query",   SONAR_ir_query_format},\
  {"SONAR_ir_reply",   SONAR_ir_reply_format},\
  {"SONAR_define_mask",   SONAR_define_mask_format}



#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




#ifdef DEFINE_REPLY_HANDLERS


/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with SONAR ******/




/******* (a) Procedure headers ******/




void SONAR_sonar_reply_handler(TCX_REF_PTR                ref,
			       SONAR_sonar_reply_ptr      data);


void SONAR_ir_reply_handler(TCX_REF_PTR                ref,
                            SONAR_ir_reply_ptr      data);
                               

/******* (b) Handler array ******/



TCX_REG_HND_TYPE SONAR_reply_handler_array[] = {
  {"SONAR_sonar_reply", "SONAR_sonar_reply_handler",
     SONAR_sonar_reply_handler, TCX_RECV_ALL, NULL},
  {"SONAR_ir_reply", "SONAR_ir_reply_handler",
     SONAR_ir_reply_handler, TCX_RECV_ALL, NULL},
       
};



#endif

#endif
