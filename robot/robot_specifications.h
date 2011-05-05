
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










/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define dRESH		  640		/* H resolution */
#define dRESV		  480		/* V resolution */

#define X_SIZE dRESH
#define Y_SIZE (dRESV+12)

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/******** PAN/TILT CONSTANTS ******************/


#define MIN_TILT_ANGLE -46.95
#define MAX_TILT_ANGLE  31.26
#define MIN_PAN_ANGLE -158.86
#define MAX_PAN_ANGLE  158.91
#define INITIAL_TILT -10.0	/* in degree */
#define INITIAL_PAN    0.0	/* in degree */
#define PAN_LOW_SPEED    40.0
#define PAN_NORMAL_SPEED 80.0
#define PAN_HIGH_SPEED   130.0
#define TILT_LOW_SPEED    40.0
#define TILT_NORMAL_SPEED 80.0
#define TILT_HIGH_SPEED   130.0


#define PSEUDO_SUBSAMPLING_FACTOR    8
#define PSEUDO_PICTURE_SIZE_HORIZONTAL 320
#define PSEUDO_PICTURE_SIZE_VERTICAL   240
#define PSEUDO_PICTURE_OFFSET_HORIZONTAL 0 
#define PSEUDO_PICTURE_OFFSET_VERTICAL   2

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/******** BASE CONSTANTS ******************/


#define MAX_TRANS_VELOCITY     100.0
#define MAX_ROT_VELOCITY       100.0
#define MAX_TRANS_ACCELERATION 100.0
#define MAX_ROT_ACCELERATION   100.0


#define INITIAL_TRANS_VELOCITY      4.0
#define INITIAL_ROT_VELOCITY        20.0

#define INITIAL_TRANS_ACCELERATION  30.0
#define INITIAL_ROT_ACCELERATION    40.0

#define TELE_MAX_ROT_VELOCITY   40.0 /* for teleoperation */
#define TELE_MAX_TRANS_VELOCITY 50.0 /* for teleoperation */
#define TELE_MAX_TRANS_DIST 200.0 /* for teleoperation */


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


