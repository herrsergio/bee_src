
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







/*#define TCX_debug*/

#ifdef UNIBONN
#undef UNIBONN
#endif


#define TOURGUIDE_VERSION	/* activate, if you'd like to configure
				 * the commander as a tourguide control unit.
				 * This is NOT a standard BeeSoft feature */

/* #define CD_VERSION */        /* activate, if you have a CD player installed
				 * on the robot.
				 * This is NOT a standard BeeSoft feature */


#include "tcx.h"
#include "tcxP.h"
#include "robot_specifications.h"
#include "imagesize.h"
#include "BASE-messages.h"
#include "SONAR-messages.h"
#include "LASER-messages.h"
#include "IR-messages.h"
#include "COLLI-messages.h"
#include "PANTILT-messages.h"
#include "MAP-messages.h"
#include "PLAN-messages.h"
#include "CAMERA-messages.h"
#include "SIMULATOR-messages.h"
#ifdef UNIBONN
#include "ARM-messages.h"
#include "SUNVIS-messages.h"
#include "EZR-messages.h"
#include "EZR-defines.h"
#include "FLOW-messages.h"
#include "SPEECH-messages.h"
#endif /* UNIBONN */
#ifdef CD_VERSION
#include "struct.h"
#include "CD-tracks.h"
#include "CD-messages.h"
#endif /* CD_VERSION */

#include "BUTTONS-messages.h"


#include <bUtils.h>
#include "main.h"
#include "o-graphics.h"
#include "graphics.h"
#include "mouse.h"
#include "init.h"
#include "action.h"
#include "corr.h"
#ifdef UNIBONN
#include "task.h"
#endif /* UNIBONN */
#include "file.h"
#include "tour.h"



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern struct bParamList * bParamList;
extern const char *bRobotType;



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


#define MAX_RANDOM 1073741823.5
#define RAND() ((((float) random())/MAX_RANDOM)-1.0)
#define RAND_POS() ((((float) random())/MAX_RANDOM)/2.0)
#define pi  3.14159265358979323846
#define pi2 6.28318530717958647692
