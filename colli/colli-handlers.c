
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
#include "rwibase_interface.h"

#include "SONAR-messages.h"
#include "LASER-messages.h"
#include "LASER_SERVER-messages.h"
#include "IR-messages.h"

#define TCX_define_variables /* this makes sure variables are installed */
#include "BASE-messages.h"
#include "COLLI-messages.h"

#ifdef UNIBONN
#include "SUNVIS-messages.h"
#endif

#include "collision.h"
#include "base-handlers.h"
#include "colli-handlers.h"

#ifdef CLEANUP
#include "ARM-messages.h" 
#include "colliCleanUp.h"
#endif

/*---- 'COLLI_debug' prints out messages upon receipt of a TCX message ----*/
/*#define COLLI_debug*/


COLLI_colli_reply_type  colli_tcx_status;



/************************************************************************
 *
 *   NAME:         dumpInformation
 *                 
 *   FUNCTION:     writes the tcx information in a dump file.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void
dumpInformation( COLLI_colli_reply_ptr colli)
{
    int i;
    static BOOLEAN firstTime = TRUE;
    static FILE* dumpFile;
    
    if ( firstTime) {
	char* dumpName = "TCX_dumpFile";
	
       firstTime = FALSE;
	if ((dumpFile = fopen( dumpName, "w")) == NULL) {
	    fprintf( stderr, "ERROR: cannot open dumpFile %s.\n", dumpName);
	    exit(1);
       }
    }
    
    fprintf( dumpFile, "\n************** NEXT UPDATE ***************\n\n");
    fprintf( dumpFile, "rrot %i  rposx  %i  rposy  %i  rvel  %i  tvel  %i\n",
	     colli->rrot, (colli->rpos).x, (colli->rpos).y, colli->rvel, colli->tvel);
    fprintf( dumpFile, "remember %i\n", colli->rememberInterval);

    fprintf( dumpFile, "\n------Sonarlines------\n");
    for ( i=0; i < 24; i++)
	fprintf( dumpFile, "p1x %i  p1y %i  p2x %i  p2y %i\n",
		 (colli->sonar_lines)[i].pt1.x, 
		 (colli->sonar_lines)[i].pt1.y, 
		 (colli->sonar_lines)[i].pt2.x, 
		 (colli->sonar_lines)[i].pt2.y);

    fprintf( dumpFile, "\n-------trajecotry---------\n");
    fprintf( dumpFile, "lline p1x %i  p1y %i  p2x %i  p2y %i\n",
	     (colli->leftLine).pt1.x, 
	     (colli->leftLine).pt1.y, 
	     (colli->leftLine).pt2.x, 
	     (colli->leftLine).pt2.y);
    
    fprintf( dumpFile, "rline p1x %i  p1y %i  p2x %i  p2y %i\n",
	     (colli->rightLine).pt1.x, 
	     (colli->rightLine).pt1.y, 
	     (colli->rightLine).pt2.x, 
	     (colli->rightLine).pt2.y);
    
    
    fprintf( dumpFile, "Circle  midx %i  midy %i  inner %i  outer %i\n",
	    (colli->innerCircle).M.x,
	    (colli->innerCircle).M.y,
	    (colli->innerCircle).rad,
	    (colli->outerCircle).rad);

}

static int n_COLLI_send_colli_update = 0;

void COLLI_send_colli_update()
{
  int i;

  if (n_auto_update_modules>0) {
    SONAR_look_for_sonar_device();

    for (i = 0; i < n_auto_update_modules; i++){
      
	 if (auto_update_modules[i].colli > 0 &&
	     n_COLLI_send_colli_update % auto_update_modules[i].colli == 0){
#ifdef COLLI_debug
	   dumpInformation( &colli_tcx_status);

	   fprintf(stderr, "Send colli update to %s.\n",
		   tcxModuleName(auto_update_modules[i].module));
#endif
	   tcxSendMsg(auto_update_modules[i].module, "COLLI_colli_reply",
		      &colli_tcx_status); 
	 }
       }
    
    colli_tcx_status.innerCircle.M.x = I_ERROR;
    colli_tcx_status.outerCircle.M.x = I_ERROR;
    colli_tcx_status.leftLine.pt1.x  = I_ERROR;
    
    colli_tcx_status.armCircle.M.x   = I_ERROR;
    colli_tcx_status.innerArmPoint.x = I_ERROR;
    colli_tcx_status.outerArmPoint.x = I_ERROR;
    
    n_COLLI_send_colli_update++;
  }
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
  COLLI_get_parameters(parameters);
  tcxFree("COLLI_parameter", parameters);
}


#ifdef UNIBONN

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
  BASE_obstacle_lines_handler( ref, vision_lines);
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
  BASE_obstacle_points_handler( ref, vision_points);
}


/************************************************************************
 *
 *   NAME:         SUNVIS_check_object_reply_handler
 *                 
 *   FUNCTION:     receives position of object to pick up
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void SUNVIS_check_object_reply_handler(TCX_REF_PTR                    ref,
				       SUNVIS_check_object_reply_ptr  dat )
{
#ifdef CLEANUP
  COLLI_ApproachCloseObject( dat);
#endif
  tcxFree("SUNVIS_check_object_reply", dat);
}


void SUNVIS_object_reply_handler(TCX_REF_PTR                ref,
				 SUNVIS_object_reply_ptr  object_msg)
     
{
#ifdef CLEANUP
  COLLI_ApproachFarObject( object_msg);
#endif
  tcxFree("SUNVIS_object_reply", object_msg);
}

#endif /* UNIBONN */

/************************************************************************
 *
 *   NAME:         ARM_Messages
 *                 
 *   FUNCTION:     communicates with the arm
 *                 
 *   PARAMETERS:   struct parameters:
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

#ifdef CLEANUP
void ARM_position_reply_handler(TCX_REF_PTR                   ref,
				ARM_position_reply_ptr pos)
{}

void ARM_update_status_reply_handler(TCX_REF_PTR                   ref,
				     ARM_update_status_reply_ptr status)
{}

void ARM_action_executed_reply_handler(TCX_REF_PTR                   ref,
				       ARM_action_executed_reply_ptr data)
{}

void ARM_pickup_at_ground_reply_handler(TCX_REF_PTR                   ref,
					ARM_pickup_at_ground_reply_ptr data)
{
  ARM_moveToGroundReady( data->success);
  tcxFree("ARM_pickup_at_ground_reply", data);
}

void ARM_lift_object_reply_handler(TCX_REF_PTR                   ref,
						ARM_lift_object_reply_ptr data)
{
  ARM_liftObjectReady( data->success);
  tcxFree("ARM_lift_object_reply", data);
}

void ARM_drop_object_reply_handler(TCX_REF_PTR                   ref,
						ARM_drop_object_reply_ptr data)
{
  ARM_dropObjectReady( data->success);
  tcxFree("ARM_drop_object_reply", data);
}

void ARM_move_in_reply_handler(TCX_REF_PTR                   ref,
						ARM_move_in_reply_ptr data)
{
  ARM_moveInReady( data->success);
  tcxFree("ARM_move_in_reply", data);
}
#endif /* CLEANUP */


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/












