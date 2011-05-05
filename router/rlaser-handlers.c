
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

#if 0
#define TCX_define_variables /* this makes sure variables are installed */
#endif
#include "LASER-messages.h"

#include "router.h"

/*---- 'SONAR_debug' prints out messages upon receipt of a TCX message ----*/
#define SONAR_debug


/***************************************************************************
 * GLOBAL VARIABLES                                                        *
 ***************************************************************************/

extern RouterModuleReferenceType RMR[];

extern int NumberOfLaserReplyModules;
extern int LaserAutoReplyList[];
extern int verbose_on;

extern TCX_MODULE_PTR REAL_BASE;

unsigned int LASER_REPLIES = 0;

/*********************************************************************\
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
|*********************************************************************|
\*********************************************************************/




/**************************************************************************/
/**************************************************************************
 * LASER_laser_reply_handler                                              *
 **************************************************************************/
void LASER_laser_reply_handler(TCX_REF_PTR                  ref,
			       LASER_laser_reply_ptr        data)
{
  int i,count=0;

  for ( i=0 ; i<NumberOfLaserReplyModules ; i++)
    {
      if ( RMR[LaserAutoReplyList[i]].ModuleStatus == ACTIVE )
	{
	  if ( ( LASER_REPLIES % RMR[LaserAutoReplyList[i]].laser )==0)
	    {
	      tcxSendMsg ( RMR[LaserAutoReplyList[i]].module,
			  "LASER_laser_reply", data );
	      count++;
	    }
	}
    }
  
  LASER_REPLIES++;  
  if ( verbose_on == 1 )
    fprintf (stderr,"S%d",count);
  tcxFree ("LASER_laser_reply",data);
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


TCX_REG_HND_TYPE LASER_handler_array[] = {
};

/* copied from LASER-messages.h */
TCX_REG_HND_TYPE LASER_reply_handler_array[] = {
  {"LASER_laser_reply", "LASER_laser_reply_handler",
     LASER_laser_reply_handler, TCX_RECV_ALL, NULL}
};



void tcx_register_laser(void)	/* make sure we are connected to tcx! */
{
  tcxRegisterHandlers(LASER_handler_array, 
		      sizeof(LASER_handler_array) / sizeof(TCX_REG_HND_TYPE));
  tcxRegisterHandlers(LASER_reply_handler_array,
		      sizeof(LASER_reply_handler_array) /
		      sizeof(TCX_REG_HND_TYPE));

}







