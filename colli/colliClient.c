
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
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Welcome!
 *****
 ***** This file is part of the robot control software provided
 ***** by Real World Interface Inc.
 *****
 ***** All copyrights are by Real World Interface Inc., Carnegie
 ***** Mellon University, and the University of Bonn, Germany.
 ***** Removing the copyright information is illegal. Please read
 ***** and make sure you understand the disclaimer below.
 *****
 ***** Contact tyson@rwii.com if you have questions or would
 ***** like to obtain further information.
 *****
 ***** We hope you enjoy working with this software package.
 *****
 *****                      Tyson D. Sawyer and Sebastian Thrun
 *****
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED
 ***** BY APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING
 ***** THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM
 ***** "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR
 ***** IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 ***** OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE
 ***** ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM
 ***** IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME
 ***** THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO
 ***** LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES
 ***** SUSTAINED BY YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM
 ***** TO OPERATE WITH ANY OTHER PROGRAMS OR FAILURE TO CONTROL A
 ***** PHYSICAL DEVICE OF ANY TYPE), EVEN IF SUCH HOLDER OR OTHER
 ***** PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Source file:     $Source: /home/rhino/rhino/CVS/bee/src/colli/colliClient.c,v $
 *****
 ***** Created by:      $Author: swa $
 *****
 ***** Revision #:      $Revision: 1.11 $
 *****
 ***** Date of revision $Date: 1998/01/15 16:11:09 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: colliClient.c,v $
 * Revision 1.11  1998/01/15 16:11:09  swa
 * I also changed colliClient to allow reconnects. This involved
 * re-declaring a function to be int colliConnect(int) and no longer void
 * colliConnect(void). Replace any occurence of colliConnect() in your
 * programs with colliConnect(1) and your done.
 *
 * Revision 1.10  1997/09/05 21:53:57  thrun
 * Fixed a problem with colliApproachRelative():
 * Previously, this was translated into an absolute motion, now it uses
 * relative motion. For some reason, the absolute motion is broken.
 * We need to investigate this.
 *
 * Revision 1.9  1997/07/17 17:31:45  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.8  1997/04/07 14:51:28  thrun
 * Modified for only _one_ laser (using UNI_BONN)   (swa) (thrun)
 *
 * Revision 1.7  1997/03/26 09:09:44  fox
 * Fixed a bug in colliApproachRelative. The two parameters give directions
 * relative to the robot and NOT x and y coordinates.
 *
 * Revision 1.6  1997/03/24 20:02:38  thrun
 * Oops. Previous version (2 min ago) doesn't work properly, Try this one.
 *
 * Revision 1.5  1997/03/24 20:00:15  thrun
 * Fixed a typo in the declarations.
 *
 * Revision 1.4  1997/03/11 17:08:15  tyson
 * added IR simulation and other work
 *
 * Revision 1.3  1997/02/25 18:12:41  tyson
 * client lib for PANTILT and lots of little stuff
 *
 * Revision 1.2  1997/02/05 16:02:38  fox
 * Changed BASE_setmode message.
 *
 * Revision 1.1  1997/02/02 22:32:28  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#ifndef lint
static char rcsid[] =
"$Id: colliClient.c,v 1.11 1998/01/15 16:11:09 swa Exp $";
static void *const use_rcsid = ((void)&use_rcsid, (void)&rcsid, 0);
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <ctype.h>
#include <sys/time.h>

#include <tcx.h>
#include <rai.h>
#include <baseClient.h>

#define TCX_define_variables
#define DEFINE_REPLY_HANDLERS
#include <BASE-messages.h>

#include <colliClient.h>

#ifndef PI
#define PI M_PI
#endif


extern void tcxRegisterCloseHnd(void (*closeHnd)());

/*
 * This should include hostname and
 * PID or something like that
 */

#define COLLI_CLIENT_NAME "colliclient"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


struct timeval BASE_last_send_message_time = {0, 0};

int colliConnected = 0;

static int colliMode = 0;

static float colliSetRotVel     = 0.0;
static float colliSetRotAccel   = 0.0;
static float colliSetTransVel   = 0.0;
static float colliSetTransAccel = 0.0;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void
BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				  BASE_robot_position_reply_ptr pos)
{
#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif
  tcxFree("BASE_robot_position_reply", pos);
}


/*swa*/
void 
BASE_update_status_reply_handler(TCX_REF_PTR                   ref,
				 BASE_update_status_reply_ptr status)
{
#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif
  tcxFree("BASE_update_status_reply", status);
}


void
BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
				   BASE_action_executed_reply_ptr data)
{
#ifdef TCX_debug
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif
  tcxFree("BASE_action_executed_reply", data);
}

static void 
colliWait(struct timeval *last_time, long u_delay)
{ /* max: 1 sec*/
  struct timeval now;
  int ready = 1;
  
  do {
    if (!ready){
#ifdef TCX_debug
      fprintf(stderr, "+");
#endif
      usleep(u_delay/4);
    }
    gettimeofday(&now, NULL);
    ready = ( now.tv_sec >  last_time->tv_sec + 1 ||
	      (now.tv_sec == last_time->tv_sec + 1 &&
	       now.tv_usec + 1000000 > last_time->tv_usec + u_delay) ||
	      (now.tv_sec == last_time->tv_sec &&
	       now.tv_usec > last_time->tv_usec + u_delay)); 
  }
  while (!ready);
  
  last_time->tv_sec  = now.tv_sec;
  last_time->tv_usec = now.tv_usec;
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void
colliSetMode(int mode)
{
  int m = mode;

  BASE_setmode_type modeInfo;
  modeInfo.useSonar   = DONT_CHANGE;
  modeInfo.useLaser   = DONT_CHANGE;

#ifdef TCX_debug
  fprintf(stderr, "TCX: base_set_mode\n");
#endif

  if (colliConnected) {

    for (; m < 0;) m += NUMBER_OF_MODES;
    for (; m >= NUMBER_OF_MODES;) m -= NUMBER_OF_MODES;
    
    modeInfo.modeNumber = m;
    
    tcxSendMsg(BASE, "BASE_setmode", &modeInfo);

    colliMode = m;

  } else {
    fprintf(stderr,
	    "%s(%s): colliServer is not connected. \n", __FILE__, __FUNCTION__);
    colliConnect( 0 );
  }

}


void
colliStopRobot()
{
  int i;

#ifdef TCX_debug
  fprintf(stderr, "TCX: base_stop_robot\n");
#endif

  if (colliConnected) {
    tcxSendMsg(BASE, "BASE_stop_robot", NULL);
  } else {
    fprintf(stderr,
	    "%s(%s): colliServer is not connected. \n", __FILE__, __FUNCTION__);
    colliConnect( 0 );
  }
}


void
colliSetVelocity(float trans_speed, float rot_speed)
{
  BASE_set_velocity_type tcx_velocity;

#ifdef TCX_debug
  fprintf(stderr, "TCX: base_set_velocity: %g %g\n", trans_speed, rot_speed);
#endif

  if (colliConnected){

    if (trans_speed < 0.0) trans_speed = 0.0;
    if (rot_speed < 0.0)   rot_speed = 0.0;
    
    tcx_velocity.rot_velocity   = colliSetRotVel   = rot_speed;
    tcx_velocity.trans_velocity = colliSetTransVel = trans_speed;
    
#ifdef TCX_debug  
    fprintf(stderr, "%s:%s() | rot_speed=%f; trans_speed=%f;\n",
	    __FILE__, __FUNCTION__, rot_speed, trans_speed);
#endif
    
    colliWait(&BASE_last_send_message_time, 250000); /* at most 4
						      * messages per
						      * sec
						      */
    tcxSendMsg(BASE, "BASE_set_velocity", &tcx_velocity);
  } else {
    fprintf(stderr,
	    "%s(%s): colliServer is not connected. \n", __FILE__, __FUNCTION__);
    colliConnect( 0 );
  }

}


void
colliSetAcceleration(float trans_accel, float rot_accel)
{
  BASE_set_acceleration_type tcx_acceleration;

#ifdef TCX_debug
  fprintf(stderr, "TCX: base_set_acceleration: %g %g\n", 
	  trans_accel, rot_accel);
#endif

  if (colliConnected) {

    if (trans_accel < 0.0) trans_accel = 0.0;
    if (rot_accel < 0.0)   rot_accel = 0.0;
    
    tcx_acceleration.rot_acceleration   = colliSetRotAccel   = rot_accel;
    tcx_acceleration.trans_acceleration = colliSetTransAccel = trans_accel;
    
#ifdef TCX_debug  
    fprintf(stderr, "%s:%s() | rot_accel=%f; trans_accel=%f;\n",
	    __FILE__, __FUNCTION__, rot_accel, trans_accel);
#endif  
    
    tcxSendMsg(BASE, "BASE_set_acceleration", &tcx_acceleration);

  } else {
    fprintf(stderr,
	    "%s(%s): colliServer is not connected. \n", __FILE__, __FUNCTION__);
    colliConnect( 0 );
  }

}



void
colliApproachTwoPointsAbsolute(float x1, float y1, float x2, float y2,
			       float approachDist, int newTarget)
{
  BASE_approach_absolute_two_points_type tcx_coord;
  
#ifdef TCX_debug
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);
  fprintf(stderr, "\t[x1:y1]=[%f:%f]\n", x1, y1);
  fprintf(stderr, "\t[x2:y2]=[%f:%f]\n", x2, y2);
  fprintf(stderr, "\tapprochDist=%f\n", approachDist);
#endif

  if (colliConnected) {
    tcx_coord.new_target = newTarget;
    tcx_coord.approach_dist = approachDist;
    
    tcx_coord.abs_target_x1 = (bRobotToOdoX(x1, y1) -
			       (float)0x8000/bRobot.base_encPerCm);
    
    tcx_coord.abs_target_y1 = (bRobotToOdoY(x1, y1) -
			       (float)0x8000/bRobot.base_encPerCm);
    
    tcx_coord.abs_target_x2 = (bRobotToOdoX(x2, y2) -
			       (float)0x8000/bRobot.base_encPerCm);
    
    tcx_coord.abs_target_y2 = (bRobotToOdoY(x2, y2) -
			       (float)0x8000/bRobot.base_encPerCm);
    
#ifdef TCX_debug
    fprintf(stderr, "TCX: base_approach_absolute_2_points: "
	    "%g, %g: %g, %g), dist %g\n",
	    tcx_coord.abs_target_x1, tcx_coord.abs_target_y1,
	    tcx_coord.abs_target_x2, tcx_coord.abs_target_y2,
	    tcx_coord.approach_dist);
#endif
    
    tcxSendMsg(BASE, "BASE_approach_absolute_two_points", &tcx_coord);
  } else {
    fprintf(stderr,
	    "%s(%s): colliServer is not connected. \n", __FILE__, __FUNCTION__);
    colliConnect( 0 );
  }

}


void
colliApproachTwoPointsRelative(float x1, float y1, float x2, float y2,
			       float approachDist, int newTarget)
{
  float heading;
  float xa, ya, xb, yb;

  heading = bRobotHeading(0.0);

  xa = bRobotX(0.0) + x1*cos(heading) + y1*sin(heading);
  ya = bRobotY(0.0) + x1*sin(heading) + y1*cos(heading);

  xb = bRobotX(0.0) + x2*cos(heading) + y2*sin(heading);
  yb = bRobotY(0.0) + x2*sin(heading) + y2*cos(heading);

  colliApproachTwoPointsAbsolute(xa, ya, xb, yb, approachDist, newTarget);
}


void
colliApproachAbsolute(float x1, float y1, float approachDist, int newTarget)
{
  colliApproachTwoPointsAbsolute(x1, y1, x1, y1, approachDist, newTarget);
}


/* Direction to the target point is given by the sign of the parameters:
 * forward > 0  --> target point in front of the robot
 * sideward > 0 --> target point to the right of the robot */
void colliApproachRelative(float forward, float sideward,
			   float approachDist, int newTarget)
{
  float heading;
  float xa, ya;

  BASE_goto_relative_type data;

  data.rel_target_y = forward;
  data.rel_target_x = sideward;
  if (colliConnected) {
    tcxSendMsg(BASE, "BASE_goto_relative", &data);
  }
  return;


  heading = bRobotHeading(0.0);
  xa = bRobotX(0.0) + forward*cos(heading) + sideward*sin(heading);
  ya = bRobotY(0.0) + forward*sin(heading) - sideward*cos(heading);

  colliApproachTwoPointsAbsolute(xa, ya, xa, ya, approachDist, newTarget);
}

/* ---------------------------------------------------------
 *
 * hook up to colliServer. if colliServer dies, try reconnecting 
 * every three seconds
 *
 * --------------------------------------------------------*/
int
colliConnect( int wait_till_established )
{
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference;

#if ( defined(G_DEBUG_TCX) )
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);

  if (time_difference < 3.0)
    return -1;

  if ( wait_till_established ) { /* 1 */

    fprintf(stderr, "ColliClient: Connecting to Colli server...\n");
    BASE = tcxConnectModule(TCX_BASE_MODULE_NAME);
    colliConnected = 1;
    fprintf(stderr, "ColliClient: Connected.\n");

  } else {			/* 0 */

    if ( colliConnected == 0 || !BASE ) { /* doppelt haelt besser */
      fprintf(stderr, "ColliClient: Connecting to Colli server...\n");
      BASE  = tcxConnectOptional(TCX_BASE_MODULE_NAME);
      if( BASE ) {
	colliConnected = 1;
	/* 	colli_subscribe();   */ /* not auto_update_modules??? */
	fprintf(stderr, "ColliClient: Connected.\n");
      } else {
	colliConnected = 0;
      }
    }

  }

  last_time.tv_sec  = this_time.tv_sec;
  last_time.tv_usec = this_time.tv_usec;

  return 0;
}


void
_colliConnect()
{
  fprintf(stderr, "colliClient: Connecting to COLLI...\n");
  BASE = tcxConnectModule(TCX_BASE_MODULE_NAME);
  colliConnected = 1;
  fprintf(stderr, "Connected.\n");
}

void
colliRegister()
{
  int numberOfMessages;
  int numberOfHandlers;
  int i;

  TCX_REG_MSG_TYPE messages[] = { BASE_messages }; 
  
  numberOfMessages = sizeof(messages)/sizeof(TCX_REG_MSG_TYPE); 
  numberOfHandlers = 
    sizeof(BASE_reply_handler_array)/sizeof(TCX_REG_HND_TYPE); 

  registerInterface("", numberOfMessages, messages,
		    numberOfHandlers, BASE_reply_handler_array);
}
