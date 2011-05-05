
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
#include <values.h>
#include "tcx.h"
#include "tcxP.h"
#include "sonar_interface.h" 

#include "SONAR-messages.h"

#define TCX_define_variables /* this makes sure variables are installed */
#include "BASE-messages.h"
#include "SONAR-messages.h"
#include "COLLI-messages.h"
#include "LASER-messages.h"
#include "IR-messages.h"


#include "collision.h"
#include "base-handlers.h"
#include "colli-handlers.h"

#include "router.h"

/*---- 'COLLI_debug' prints out messages upon receipt of a TCX message ----*/
/*#define COLLI_debug*/

/***************************************************************************
 * GLOBAL VARIABLES                                                        *
 ***************************************************************************/
COLLI_colli_reply_type        colli_tcx_status;

int COLLI_REPLIES = 0;
extern RouterModuleReferenceType RMR[];

extern int NumberOfColliReplyModules;
extern int ColliAutoReplyList[];
extern int verbose_on;
extern TCX_MODULE_PTR REAL_BASE;    


/************************************************************************
 *
 *   NAME:         COLLI_vision_line_handler
 *                 
 *   FUNCTION:     receives collision lines from the vision module
 *                 
 *   PARAMETERS:   struct no_of_lines: number of collision lines
                   array with the different lines
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void COLLI_vision_line_handler(TCX_REF_PTR                ref,
			       COLLI_vision_line_ptr      vision_lines)
{
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "COLLI_vision_line" , vision_lines );
    }
  tcxFree("COLLI_vision_line", vision_lines);
}



/************************************************************************
 *
 *   NAME:         COLLI_vision_point_handler
 *                 
 *   FUNCTION:     receives collision points from the vision module
 *                 
 *   PARAMETERS:   struct no_of_points: number of collision points
                   array with the different points
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void COLLI_vision_point_handler(TCX_REF_PTR                ref,
			       COLLI_vision_point_ptr      vision_points)
{
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "COLLI_vision_point" , vision_points );
    }
  tcxFree("COLLI_vision_point", vision_points);
}


/************************************************************************
 *
 *   NAME:         COLLI_parameter_handler
 *                 
 *   FUNCTION:     receives collision parameters.
 *                 
 *   PARAMETERS:   struct parameters:
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void COLLI_parameter_handler(TCX_REF_PTR                ref,
			       COLLI_parameter_ptr      parameters)
{
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "COLLI_parameter" , parameters );
    }
  tcxFree("COLLI_parameter", parameters);
}





/***************************************************************************
 * COLLI_colli_reply_handler                                               *
 ***************************************************************************/
void COLLI_colli_reply_handler(TCX_REF_PTR                ref,
			       COLLI_colli_reply_ptr      data)
{
  int i,count=0;
 
  for ( i=0 ; i<NumberOfColliReplyModules ; i++)
    {
      if ( RMR[ColliAutoReplyList[i]].ModuleStatus == ACTIVE )
	{
	  if ( ( COLLI_REPLIES % RMR[ColliAutoReplyList[i]].colli )==0)
	    {
	      if ( verbose_on == 1 )
		fprintf (stderr,">");
	      tcxSendMsg ( RMR[ColliAutoReplyList[i]].module,
			  "COLLI_colli_reply", data );
	      count++;
	    }
	}
    }
  
  for ( i=0 ; i<MaxNumberOfModules ; i++ )
    {
      if ( RMR[i].colli == 0 )   /* final security */
	{
	  if ( GetQuery ( i , COLLI_QUERY ) > 0 )
	    {
	      if ( verbose_on == 1 )
		fprintf (stderr,"\nSending COLLI REPLY to %d",i );
	      tcxSendMsg ( RMR[i].module,
			  "COLLI_colli_reply", data );
	      DecreaseQuery ( i , COLLI_QUERY );
	    }
	}
    }
  
  COLLI_REPLIES++;  
  if ( verbose_on == 1 )
    fprintf (stderr,"C%d",count);
  tcxFree ( "COLLI_colli_reply" , data );
}

/* copied from COLLI-messages.h */
/******* (b) Handler array ******/

TCX_REG_HND_TYPE COLLI_reply_handler_array[] = {
  {"COLLI_colli_reply", "COLLI_colli_reply_handler",
     COLLI_colli_reply_handler, TCX_RECV_ALL, NULL}
};



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void tcx_register_colli ( void )
{
  tcxRegisterHandlers(COLLI_reply_handler_array,
		      sizeof(COLLI_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
}

