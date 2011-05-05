
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
#include "libc.h"
#include "Common.h"
#include "tcx.h"
#include "robot_specifications.h"
#include "tcxP.h"
#include "sonar_interface.h"
#include <irClient.h>
#include "collision.h"

/* #define TCX_define_variables  this makes sure variables are installed */
#include "SONAR-messages.h"



/*---- 'SONAR_debug' prints out messages upon receipt of a TCX message ----*/
#define SONAR_debug





/*********************************************************************\
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
\*********************************************************************/





/************************************************************************
 *
 *   NAME: SONAR_switch_on_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void SONAR_switch_on_handler(TCX_REF_PTR      ref,
			     void            *data)
{

#ifdef SONAR_debug
  fprintf(stderr, "Received a  SONAR_switch_on_message from %s.\n",
	  tcxModuleName(ref->module));
#endif

if (use_sonar) SONAR_LoopStart(sonar_act_mask); 

}



/************************************************************************
 *
 *   NAME: SONAR_switch_off_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void SONAR_switch_off_handler(TCX_REF_PTR      ref,
			      void            *data)
{

#ifdef SONAR_debug
  fprintf(stderr, "Received a  SONAR_switch_off_message from %s.\n",
	  tcxModuleName(ref->module));
#endif
  
if (use_sonar)  SONAR_LoopEnd();

}



/************************************************************************
 *
 *   NAME: SONAR_activate_mask_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void SONAR_activate_mask_handler(TCX_REF_PTR      ref,
				 int             *mask)
{

#ifdef SONAR_debug
  fprintf(stderr, "Received a  SONAR_activate_mask_message from %s.\n",
	  tcxModuleName(ref->module));
#endif
  
  if (use_sonar) {
    if ((*mask < 0) || (*mask >= MAX_MASKS))
      fprintf(stderr, "\nCannot activate mask. Wrong mask number (%i).\n", *mask);
    else if (sonar_mask_array[*mask] == NULL) 
      fprintf(stderr, "\nCannot activate mask. Mask number (%i) not defined.\n", *mask);
    else
      SONAR_ChangeMask(sonar_mask_array[*mask]);
  }
  tcxFree("SONAR_activate_mask", mask);
}



/************************************************************************
 *
 *   NAME: SONAR_sonar_query_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void SONAR_sonar_query_handler(TCX_REF_PTR      ref,
			       void            *data)
{
  SONAR_sonar_reply_type sonar;
  int i;

#ifdef SONAR_debug
  fprintf(stderr, "Received a  SONAR_sonar_query_message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  for(i=0; i<24; i++)
    if (use_sonar) 
      sonar.values[i] = (float) sonar_readings[i];
    else
      sonar.values[i] = -1.0;
  
  tcxReply(ref, "SONAR_sonar_reply", &sonar);
}


/****************************************************************/
void SONAR_ir_query_handler(TCX_REF_PTR      ref,
                            void            *data)
  {  
    SONAR_ir_reply_type ir;
    int i;
    /*
      fprintf(stderr, "Received a  SONAR_ir_query_message from %s.\n",
      tcxModuleName(ref->module));
    */
    for(i=0; i<24; i++)
    {
      ir.upperrow[i] = irs[0][i].value;
      ir.lowerrow[i] = irs[1][i].value;
    };
    for(i=0; i<8; i++)
      ir.drow[i] = irs[2][i].value;
    tcxReply(ref, "SONAR_ir_reply", &ir);
  };
     
                   
                                                                                            



/************************************************************************
 *
 *   NAME: SONAR_define_mask_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SONAR_define_mask_handler(TCX_REF_PTR           ref,
			       SONAR_define_mask_ptr mask)
{

  int i, length, number;

  length = mask->length;
  number = mask->number;

#ifdef SONAR_debug
  fprintf(stderr, "Received a  SONAR_define_mask_message from %s.\n",
	  tcxModuleName(ref->module));
  fprintf(stderr, "Number: %d Length: %d\n", mask->number, length);
#endif

  if (use_sonar) {
    if (number < 0 || number >= MAX_MASKS) 
      fprintf(stderr, "Cannot define mask. Wrong number (%i).\n", number);
    
    else {
      if (sonar_mask_array[number] != NULL) 
	free(sonar_mask_array[number]); 
      sonar_mask_array[number] = (unsigned long *) malloc(sizeof(unsigned long) * length);
      
      sonar_mask_array[number][0] = (unsigned long) length;
      
      for (i = 1; i < length; i++)
	sonar_mask_array[number][i] = (unsigned long) (mask->user_mask)[i];
    }
  }
  
  tcxFree("SONAR_define_mask", mask);
}


/*********************************************************************\
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
\*********************************************************************/


TCX_REG_HND_TYPE SONAR_handler_array[] = {
  {"SONAR_switch_on", "SONAR_switch_on_handler",
     SONAR_switch_on_handler, TCX_RECV_ALL, NULL},
  {"SONAR_switch_off", "SONAR_switch_off_handler",
     SONAR_switch_off_handler, TCX_RECV_ALL, NULL},
  {"SONAR_activate_mask", "SONAR_activate_mask_handler",
     SONAR_activate_mask_handler, TCX_RECV_ALL, NULL},
  {"SONAR_sonar_query", "SONAR_sonar_query_handler",
     SONAR_sonar_query_handler, TCX_RECV_ALL, NULL},
  {"SONAR_define_mask", "SONAR_define_mask_handler",
     SONAR_define_mask_handler, TCX_RECV_ALL, NULL},
  {"SONAR_ir_query", "SONAR_ir_query_handler",
     SONAR_ir_query_handler, TCX_RECV_ALL, NULL}
       
};





void tcx_register_sonar(void)	/* make sure we are connected to tcx! */
{
  tcxRegisterHandlers(SONAR_handler_array, 
		      sizeof(SONAR_handler_array) / sizeof(TCX_REG_HND_TYPE));
}

