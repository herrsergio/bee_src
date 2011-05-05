
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

#if 0
#define TCX_define_variables /* this makes sure variables are installed */
#endif
#include "SONAR-messages.h"

#include "router.h"

/*---- 'SONAR_debug' prints out messages upon receipt of a TCX message ----*/
#define SONAR_debug


/***************************************************************************
 * GLOBAL VARIABLES                                                        *
 ***************************************************************************/

extern RouterModuleReferenceType RMR[];

extern int NumberOfSonarReplyModules;
extern int SonarAutoReplyList[];
extern int verbose_on;

extern TCX_MODULE_PTR REAL_BASE;

float SONAR_BUFFER[24];
unsigned int SONAR_REPLIES = 0;

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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "SONAR_switch_on" , NULL );
    }
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "SONAR_switch_off" , NULL);
    }
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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "SONAR_activate_mask" , mask );
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
  int SenderModule,pass_thru=0;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      pass_thru = IncreaseQuery ( SenderModule , SONAR_QUERY );
      if ( pass_thru == 1 )
	{
	  tcxSendMsg ( REAL_BASE , "SONAR_sonar_query", NULL );
	  if ( verbose_on ==1 )
	    fprintf (stderr,"\nSEND A SINGLE SONAR QUERY ...!!!");
	}
    }
}

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
  int SenderModule;
  
  SenderModule = CheckIncomingModule ( ref );
  if ( SenderModule >= 0 && SenderModule < MaxNumberOfModules )
    {
      tcxSendMsg ( REAL_BASE , "SONAR_define_mask" , mask );
    }
 
  tcxFree("SONAR_define_mask", mask);
}

/**************************************************************************/
/**************************************************************************
 * SONAR_sonar_reply_handler                                              *
 **************************************************************************/
void SONAR_sonar_reply_handler(TCX_REF_PTR                  ref,
			       SONAR_sonar_reply_ptr        data)
{
  int i,count=0;

  /* buffer the new reading */
  for ( i=0 ; i<24 ; i++ )
    SONAR_BUFFER[i] = data->values[i];
  
  for ( i=0 ; i<NumberOfSonarReplyModules ; i++)
    {
      if ( RMR[SonarAutoReplyList[i]].ModuleStatus == ACTIVE )
	{
	  if ( ( SONAR_REPLIES % RMR[SonarAutoReplyList[i]].sonar )==0)
	    {
	      tcxSendMsg ( RMR[SonarAutoReplyList[i]].module,
			  "SONAR_sonar_reply", data );
	      count++;
	    }
	}
    }
  
  for ( i=0 ; i<MaxNumberOfModules ; i++ )
    {
      if ( RMR[i].sonar == 0 )   /* final security */
	{
	  if ( GetQuery ( i , SONAR_QUERY ) > 0 )
	    {
	      if ( verbose_on ==1 )
		fprintf (stderr,"\nSending SONAR REPLY to %d",i );
	      tcxSendMsg ( RMR[i].module,
			  "SONAR_sonar_reply", data );
	      DecreaseQuery ( i , SONAR_QUERY );
	    }
	}
    }
  
  SONAR_REPLIES++;  
  if ( verbose_on == 1 )
    fprintf (stderr,"S%d",count);
  tcxFree ("SONAR_sonar_reply",data);
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
     SONAR_define_mask_handler, TCX_RECV_ALL, NULL}
};

/* copied from SONAR-messages.h */
TCX_REG_HND_TYPE SONAR_reply_handler_array[] = {
  {"SONAR_sonar_reply", "SONAR_sonar_reply_handler",
     SONAR_sonar_reply_handler, TCX_RECV_ALL, NULL}
};



void tcx_register_sonar(void)	/* make sure we are connected to tcx! */
{
  tcxRegisterHandlers(SONAR_handler_array, 
		      sizeof(SONAR_handler_array) / sizeof(TCX_REG_HND_TYPE));
  tcxRegisterHandlers(SONAR_reply_handler_array,
		      sizeof(SONAR_reply_handler_array) /
		      sizeof(TCX_REG_HND_TYPE));

}







