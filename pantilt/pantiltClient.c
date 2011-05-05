
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

#ifndef lint
static char rcsid[] =
"$Id: pantiltClient.c,v 1.8 1998/01/14 19:38:35 swa Exp $";
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
#include <raiClient.h>
#include <bUtils.h>

#define TCX_define_variables
#define DEFINE_REPLY_HANDLERS
#define PANTILT_STATIC_HANDLERS
#include <PANTILT-messages.h>

extern void tcxRegisterCloseHnd(void (*closeHnd)());

/*
 * This should include hostname and
 * PID or something like that
 */

#define PANTILT_CLIENT_NAME "pantiltclient"

#define MIN_PAN_SPEED 0.2
#define MIN_TILT_SPEED 0.2

static float ptTargetPanPos    = 0.0;
static float ptTargetPanVel    = 128.0*M_PI/180.0;
static float ptTargetPanAccel  = 205.0*M_PI/180.0;

static float ptTargetTiltPos   = 0.0;
static float ptTargetTiltVel   = 128.0*M_PI/180.0;
static float ptTargetTiltAccel = 205.0*M_PI/180.0;

static float ptTargetVel       = 128.0*M_PI/180.0;

float ptStatusPanVel    = 0.0;
float ptStatusPanPos    = 0.0;
float ptStatusPanAccel  = 0.0;

float ptStatusTiltVel   = 0.0;
float ptStatusTiltPos   = 0.0;
float ptStatusTiltAccel = 0.0;

static int   ptStatusError     = 0;

static float ptLimitPanMax     =  154.0*M_PI/180.0 * .9;
static float ptLimitPanMin     = -154.0*M_PI/180.0 * .9;
static float ptLimitPanSpeed   =  0.0;

static float ptLimitTiltMax    =  30.0*M_PI/180.0 * .9;
static float ptLimitTiltMin    = -46.0*M_PI/180.0 * .9;
static float ptLimitTiltSpeed  =  0.0;

static int   ptTracking        = 0;
static float ptTrackX          = 0.0;
static float ptTrackY          = 0.0;
static float ptTrackZ          = 0.0;

static void (*userStatusCB)(void) = NULL;

#if 0 /* this is for raw pantilt units */

static float ptTiltToRad       = 0.0008975976;
static float ptPanToRad        = 0.0008975976;
static float ptRadToTilt       = 1114.0894946;
static float ptRadToPan        = 1114.0894946;

#else /* PANTILT server uses degrees */

static float ptTiltToRad       = M_PI/180.0;
static float ptPanToRad        = M_PI/180.0;
static float ptRadToTilt       = 180.0/M_PI;
static float ptRadToPan        = 180.0/M_PI;

#endif

/*
 * In the future this could be used to allow others to control 
 * the pantilt head and not allow this process to interfere.
 * Presently there is no locking method, but this might help
 * implement one.
 */

static int   ptHaveLock        = 1;
int ptConnected = 0;



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


static void
ptSetAccel2(float pan_accel,
	   float tilt_accel)
{
  PANTILT_set_acceleration_type data;

#ifdef TCX_debug
  fprintf(stderr, "%s:%s(): %g %g\n", __FILE__, __FUNCTION__,
	  pan_accel, tilt_accel);
#endif

  if (!PANTILT||ptConnected==0) {

    fprintf(stderr,
	    "%s(%s): ptServer is not connected. \n", __FILE__, __FUNCTION__);
    ptConnect( 0 );

  } else {

    ptTargetPanAccel  = pan_accel;
    ptTargetTiltAccel = tilt_accel;
    
    if (PANTILT && ptHaveLock) {
      data.pan_acceleration  = pan_accel * ptRadToPan;
      data.tilt_acceleration = tilt_accel * ptRadToTilt;
      tcxSendMsg(PANTILT, "PANTILT_set_acceleration", &data);
    }
  }
}

/*------------------------------------------------------------*/

void
ptSetAccel(float accel)
{

#define TCX_debug
#ifdef TCX_debug
  fprintf(stderr, "%s:%s(): %g\n", __FILE__, __FUNCTION__,
	  accel*180.0/M_PI);
#endif
#undef TCX_debug

  ptSetAccel2(accel, accel);
}

/*------------------------------------------------------------*/

static void
ptSetVel2(float pan_speed,
	  float tilt_speed)
{
  PANTILT_set_velocity_type data;

#ifdef TCX_debug
  fprintf(stderr, "%s:%6d:%s(): %g %g\n",
	  __FILE__, __LINE__, __FUNCTION__,
	  pan_speed*180.0/M_PI, tilt_speed*180.0/M_PI);
#endif

  if (!PANTILT||ptConnected==0) {

    fprintf(stderr,
	    "%s(%s): ptServer is not connected. \n", __FILE__, __FUNCTION__);
    ptConnect( 0 );

  } else {

    if (pan_speed<MIN_PAN_SPEED) {
      pan_speed = MIN_PAN_SPEED;
    }

    if (tilt_speed<MIN_TILT_SPEED) {
      tilt_speed = MIN_TILT_SPEED;
    }

    ptTargetPanVel  = pan_speed;
    ptTargetTiltVel = tilt_speed;

    if (PANTILT && ptHaveLock) {
      data.pan_velocity  = pan_speed * ptRadToPan;
      data.tilt_velocity = tilt_speed * ptRadToTilt;
      tcxSendMsg(PANTILT, "PANTILT_set_velocity", &data);
    }

  }

}

/*------------------------------------------------------------*/

void
ptSetVel(float speed)
{

#define TCX_debug
#ifdef TCX_debug
  fprintf(stderr, "%s:%s(): %g\n", __FILE__, __FUNCTION__,
	  speed*180.0/M_PI);
#endif
#undef TCX_debug

  if (!PANTILT||ptConnected==0) {
    fprintf(stderr,
	    "%s(%s): ptServer is not connected. \n", __FILE__, __FUNCTION__);
    ptConnect( 0 );
  } else {

    ptTargetVel  = speed;
    ptSetVel2(speed, speed);

  }
}

/*------------------------------------------------------------*/

void
ptMoveTo(float pan2,
	 float tilt2)
{
  PANTILT_move_type data;
  float panDist, tiltDist;
  float distance;

#ifdef TCX_debug
  fprintf(stderr, "%s:%6d:%s() - pan = %f tilt = %f\n",
	  __FILE__, __LINE__, __FUNCTION__, 
	  pan2*180.0/M_PI, tilt2*180.0/M_PI);
#endif

  if (!PANTILT||ptConnected==0) {
    fprintf(stderr,
	    "%s(%s): ptServer is not connected. \n", __FILE__, __FUNCTION__);
    ptConnect( 0 );
  } else {

    if (ptTracking) {
      ptStopTracking();
    }

    ptTargetPanPos  = pan2;
    ptTargetTiltPos = tilt2;

    if (ptTargetPanPos > ptLimitPanMax) {
      ptTargetPanPos = ptLimitPanMax;
    }

    if (ptTargetPanPos < ptLimitPanMin) {
      ptTargetPanPos = ptLimitPanMin;
    }

    if (ptTargetTiltPos > ptLimitTiltMax) {
      ptTargetTiltPos = ptLimitTiltMax;
    }

    if (ptTargetTiltPos < ptLimitTiltMin) {
      ptTargetTiltPos = ptLimitTiltMin;
    }
  
    panDist = ptStatusPanPos - ptTargetPanPos;
    tiltDist = ptStatusTiltPos - ptTargetTiltPos;
  
    distance = sqrt(panDist*panDist + tiltDist*tiltDist);

    if (distance) {

#ifdef TCX_debug
      fprintf(stderr, "%s:%s() - panSpeed = %f tiltSpeed = %f\n",
	      __FILE__, __FUNCTION__, panSpeed*180.0/M_PI, tiltSpeed*180.0/M_PI);
#endif
      if (PANTILT && ptHaveLock) {
	data.pan_target  = ptTargetPanPos * ptRadToPan;
	data.tilt_target = ptTargetTiltPos * ptRadToTilt;
	tcxSendMsg(PANTILT, "PANTILT_move", &data);
      }
    }

  }

}

/*------------------------------------------------------------*/

void
ptMoveBy(float pan2,
	 float tilt2)
{
  float pan, tilt;

#ifdef TCX_debug
  fprintf(stderr, "%s:%s(): %g %g\n", __FILE__, __FUNCTION__, pan2, tilt2);
#endif
  
  pan  = ptStatusPanPos  + pan2;
  tilt = ptStatusTiltPos + tilt2;
  
  ptMoveTo(pan, tilt);
}

/*------------------------------------------------------------*/

void
ptTrackPoint(float x,
	     float y,
	     float z)
{
  PANTILT_track_point_type point;

#ifndef TCX_debug
  fprintf(stderr, "%s:%s(): %g %g %g\n", __FILE__, __FUNCTION__, x, y, z);
#endif
  
  if (!PANTILT||ptConnected==0) {
    fprintf(stderr,
	    "%s(%s): ptServer is not connected. \n", __FILE__, __FUNCTION__);
    ptConnect( 0 );
  } else {

    if (PANTILT && ptHaveLock) {
      point.x      = x - ((float)0x8000/10.0) / bRobot.base_encPerCm;
      point.y      = y - ((float)0x8000/10.0) / bRobot.base_encPerCm;
      point.height = z - ((float)0x8000/10.0) / bRobot.base_encPerCm;

      tcxSendMsg(PANTILT, "PANTILT_track_point", &point);
    }

    ptTracking = 1;
    ptTrackX = x;
    ptTrackY = y;
    ptTrackZ = z;

  }

}

/*------------------------------------------------------------*/

void
ptStopTracking(void)
{
#ifdef TCX_debug
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);
#endif

  if (!PANTILT||ptConnected==0) {
    fprintf(stderr,
	    "%s(%s): ptServer is not connected. \n", __FILE__, __FUNCTION__);
    ptConnect( 0 );
  } else {

    if (PANTILT && ptHaveLock) {
      tcxSendMsg(PANTILT, "PANTILT_stop_tracking", NULL);
    }

    ptTracking = 0;

  }
}


/***************************************************************************
 ***************************************************************************
 ***************************************************************************/


static void
ptQueryLimits(void) {
#ifdef TCX_debug
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);
#endif

  if (!PANTILT||ptConnected==0) {
    fprintf(stderr,
	    "%s(%s): ptServer is not connected. \n", __FILE__, __FUNCTION__);
    ptConnect( 0 );
  } else {

    if (PANTILT && ptHaveLock) {
      tcxSendMsg(PANTILT, "PANTILT_limits_query", NULL);
    }

  }
}


/***************************************************************************
 ***************************************************************************
 ***************************************************************************/

void
ptRegister(void)
{
  int numberOfMessages;
  int numberOfHandlers;

  TCX_REG_MSG_TYPE messages[] = { PANTILT_messages }; 
  
  numberOfMessages = sizeof(messages)/sizeof(TCX_REG_MSG_TYPE); 
  numberOfHandlers = 
    sizeof(PANTILT_reply_handler_array)/sizeof(TCX_REG_HND_TYPE); 

  registerInterface("", numberOfMessages, messages,
		    numberOfHandlers, PANTILT_reply_handler_array);
}

/*------------------------------------------------------------*/

/* ---------------------------------------------------------
 *
 * hook up to ptServer. if ptServer dies, try reconnecting 
 * every three seconds
 *
 * --------------------------------------------------------*/
int
ptConnect( int wait_till_established )
{
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference;

  int auto_reply_position = 1;

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

    fprintf(stderr, "PanTiltClient: Connecting to Pt server...\n");
    PANTILT = tcxConnectModule(TCX_PANTILT_MODULE_NAME);
    ptConnected = 1;
    fprintf(stderr, "PanTiltClient: Connected.\n");

  } else {			/* 0 */

    if ( ptConnected == 0 || !PANTILT ) { /* doppelt haelt besser */
      fprintf(stderr, "PanTiltClient: Connecting to Pt server...\n");
      PANTILT  = tcxConnectOptional(TCX_PANTILT_MODULE_NAME);
      if (PANTILT && ptHaveLock) {
	ptConnected = 1;
	fprintf(stderr, "PanTiltClient: Connected.\n");
	tcxSendMsg(PANTILT, "PANTILT_init_query", &auto_reply_position);
	/*
	 * FIXME: There is a race condition between the limits query
	 * and the ptMoveTo() or any user ptMove command.
	 * I think some static state is needed to make it work right.
	 */
	ptQueryLimits();
	ptSetVel(ptTargetVel);
	ptSetAccel2(ptTargetPanAccel, ptTargetTiltAccel);
	if (ptTracking) {
	  ptTrackPoint(ptTrackX, ptTrackY, ptTrackZ);
	} else {
	  ptMoveTo(ptTargetPanPos, ptTargetTiltPos);
	}
      } else {
	ptConnected = 0;
      }
    }

  }
  
  last_time.tv_sec  = this_time.tv_sec;
  last_time.tv_usec = this_time.tv_usec;
  
  return 0;
}

/* old connect function */
int
_ptConnect( int wait_till_established )
{
  static time_t lastCheck = 0;
  static time_t thisCheck = 0;

  int auto_reply_position = 1;
  
#ifdef TCX_debug
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);
#endif

  thisCheck = time(NULL);

  if (thisCheck>lastCheck) {
    lastCheck = thisCheck;
    if (!PANTILT) {
      PANTILT = tcxConnectOptional(TCX_PANTILT_MODULE_NAME);
      
      if (PANTILT && ptHaveLock) {
	ptConnected = 1;

	tcxSendMsg(PANTILT, "PANTILT_init_query", &auto_reply_position);

	/*
	 * FIXME: There is a race condition between the limits query
	 * and the ptMoveTo() or any user ptMove command.
	 * I think some static state is needed to make it work right.
	 */

	ptQueryLimits();
	ptSetVel(ptTargetVel);
	ptSetAccel2(ptTargetPanAccel, ptTargetTiltAccel);
	if (ptTracking) {
	  ptTrackPoint(ptTrackX, ptTrackY, ptTrackZ);
	}
	else {
	  ptMoveTo(ptTargetPanPos, ptTargetTiltPos);
	}
      }
      else
	ptConnected = 0;
    }
  }
}

/*------------------------------------------------------------*/

void
ptTerminate(void)
{
#ifdef TCX_debug
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);
#endif

  if (!PANTILT||ptConnected==0) {
    fprintf(stderr,
	    "%s(%s): ptServer is not connected. \n", __FILE__, __FUNCTION__);
    ptConnect( 0 );
  } else {

    if (PANTILT) {
      tcxSendMsg(PANTILT, "PANTILT_terminate", NULL);
    }
    PANTILT = NULL;
    ptConnected = 0;
  }

}

/*------------------------------------------------------------*/

void
ptDisconnected(void)
{
#ifdef TCX_debug
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);
#endif

  /* need a ptConnected-check here??? */
  PANTILT = NULL;
  ptConnected = 0;

}

/*------------------------------------------------------------*/

void
ptStatusCB(void *cb)
{

  userStatusCB = cb;
}

/*------------------------------------------------------------*/

#if 0
/* don't call from user routines */
static void
ptDisconnect(void)
{
#ifdef TCX_debug
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);
#endif

  if (!PANTILT||ptConnected==0) {
    fprintf(stderr,
	    "%s(%s): ptServer is not connected. \n", __FILE__, __FUNCTION__);
    ptConnect( 0 );
  } else {

    if (PANTILT) {
      tcxSendMsg(PANTILT, "PANTILT_disconnect", NULL);
    }
  }
}
#endif

/***************************************************************************
 ***************************************************************************
 ***************************************************************************/


static void
PANTILT_position_reply_handler(TCX_REF_PTR                ref,
			       PANTILT_position_reply_ptr data)
{
  ptStatusPanPos  = data->pan_pos * ptPanToRad;
  ptStatusTiltPos = data->tilt_pos * ptTiltToRad;

#ifdef TCX_debug
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);
#endif
  tcxFree("PANTILT_position_reply", data);
}

static void
PANTILT_init_reply_handler(TCX_REF_PTR             ref,
			   int                    *data)
{
#ifdef TCX_debug
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);
#endif
  tcxFree("PANTILT_init_reply", data);
}

static void
PANTILT_limits_reply_handler(TCX_REF_PTR              ref,
			     PANTILT_limits_reply_ptr data)
{
  ptLimitPanMax     = data->pan_max_angle * ptPanToRad * .9;
  ptLimitPanMin     = data->pan_min_angle * ptPanToRad * .9;
  ptLimitPanSpeed   = data->pan_max_velocity * ptPanToRad;
  
  ptLimitTiltMax    = data->tilt_max_angle * ptTiltToRad * .9;
  ptLimitTiltMin    = data->tilt_min_angle * ptTiltToRad * .9;
  ptLimitTiltSpeed  = data->tilt_max_velocity * ptTiltToRad;
#ifdef TCX_debug
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);
#endif

  tcxFree("PANTILT_limits_reply", data);
}



static void
PANTILT_status_update_handler(TCX_REF_PTR              ref,
			      PANTILT_status_update_ptr data)
{
  ptStatusPanVel    = data->pan_velocity * ptPanToRad;
  ptStatusPanPos    = data->pan_pos * ptPanToRad;
  ptStatusPanAccel  = data->pan_acceleration * ptPanToRad;

  ptStatusTiltVel   = data->tilt_velocity * ptTiltToRad;
  ptStatusTiltPos   = data->tilt_pos * ptTiltToRad;
  ptStatusTiltAccel = data->tilt_acceleration * ptTiltToRad;

  ptStatusError     = data->error;
#if 0
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);

  fprintf(stderr, "PanVel=%f\n", ptStatusPanVel*180.0/M_PI);
  fprintf(stderr, "PanPos=%f\n", ptStatusPanPos*180.0/M_PI);
  fprintf(stderr, "PanAccel=%f\n", ptStatusPanAccel*180.0/M_PI);
  fprintf(stderr, "TiltVel=%f\n", ptStatusTiltVel*180.0/M_PI);
  fprintf(stderr, "TiltPos=%f\n", ptStatusTiltPos*180.0/M_PI);
  fprintf(stderr, "TiltAccel=%f\n", ptStatusTiltAccel*180.0/M_PI);
#endif
  tcxFree("PANTILT_status_update", data);

  if (userStatusCB) {
    userStatusCB();
  }
}


/***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#if 0
PanTiltHalt
PanTiltPosition
PanTiltOffset
PanTiltSetTargetSpeed
PanTiltSetOffsetSpeed
PanTiltSetAcceleration
PanTiltSetMovePowerMode
PanTiltSetHoldPowerMode
PanTiltSetBaseSpeed
PanTiltSetUpperSpeedLimit
PanTiltSetLowerSpeedLimit
PanTiltLimit
PanTiltReset
PanTiltDefaults
PanTiltImmediateCmdExec
PanTiltSlaveCmdExec
PanTiltAwaitMotorComplete
#endif
