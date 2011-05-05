
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
#include <unistd.h>	
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include <baseClient.h>
#include <pantiltClient.h>
#include <rai.h>

#include <bUtils.h>

#define TARGET_TYPE_NONE    0
#define TARGET_TYPE_VIRT    1
#define TARGET_TYPE_SONAR   2
#define TARGET_TYPE_IR      3
#define TARGET_TYPE_TACTILE 4

#define TARGET_LIFE_NONE    0
#define TARGET_LIFE_VIRT    999999
#define TARGET_LIFE_SONAR   2
#define TARGET_LIFE_IR      2
#define TARGET_LIFE_TACTILE 20


int odofn(int param);


struct targetStruct {
  int type;
  int row;
  int col;
  int val;
  float X;
  float Y;
  time_t tod;
  struct targetStruct *next;
};

struct targetStruct targetHead;

float safetyMargin;
float exploreRange;
float safeDistance;
float maxSpeed;
float transAccel;
float rotSpeed;
float rotAccel;

int gotOrigin = FALSE;
int statusCount = 0;
int indexed = 0;
int sonarsActive = 0;

int useVirt;
int useSonar;
int useIr;
int useTactile;
int limp;

/*------------------------------------------------------------*/

void
addTarget(int type, int row, int col, int val,
	  float X, float Y, time_t lifeSpan)
{
  struct targetStruct *new;

  new = malloc(sizeof(*new));

  if (!new) {
    return;
  }

  new->type = type;
  new->row  = row;
  new->col  = col;
  new->val  = val;
  new->X    = X;
  new->Y    = Y;
  new->tod  = time(NULL) + lifeSpan;
  new->next = targetHead.next;

  targetHead.next = new;

  return;
}

/*------------------------------------------------------------*/

void
ageTargets(void)
{
  struct targetStruct *last;
  struct targetStruct *next;
  time_t now;
  int countTargets = 0;
  int countVirtual = 0;
  int countSonar   = 0;
  int countIr      = 0;
  int countTactile = 0;
  int countNone    = 0;
  int countOther   = 0;

  static int count = 0;

  now = time(NULL);

  last = &targetHead;
  next = last->next;

  while (next) {
    countTargets++;

    if (count == 0) {
      switch (next->type) {
      case TARGET_TYPE_NONE:    countNone++;    break;
      case TARGET_TYPE_VIRT:    countVirtual++; break;
      case TARGET_TYPE_SONAR:   countSonar++;   break;
      case TARGET_TYPE_IR:      countIr++;      break;
      case TARGET_TYPE_TACTILE: countTactile++; break;
      defatult: countOther++;
      }
    }

    if (next->tod < now) {
      last->next = next->next;
      free(next);
    }
    else {
      last = next;
    }
    next = last->next;
  }

#if 0
  if (count == 0) {
    fprintf(stderr,
	    "%s:%6d:%s() - targets[%d] virt[%d] sonar[%d] "
	    "ir[%d] tactile[%d] other[%d]\n",
	    __FILE__, __LINE__, __FUNCTION__, 
	    countTargets, countVirtual, countSonar,
	    countIr, countTactile, countOther);
  }

  if (count++ > 4) {
    count = 0;
  }
#endif

  return;
}

/*------------------------------------------------------------*/

/*
 * Determine range and bearing to nearest front target
 */

struct targetStruct *
closestFront(float X, float Y, float heading,
	     float *bearing, float *range)
{
  struct targetStruct *next;
  struct targetStruct *target;

  float targetRange,   nextRange;
  float targetBearing, nextBearing;
  float Xdist;
  float Ydist;

  targetRange   = 1000.0;
  targetBearing = 0.0;

  target = NULL;
  next = targetHead.next;

  while (next) {
    Xdist = next->X - X;
    Ydist = next->Y - Y;
    nextRange = sqrt(Xdist*Xdist+Ydist*Ydist);
    nextBearing   = 0.0;

    if (nextRange>0.0) {
      nextBearing = bNormalizeAngle(atan2(Ydist, Xdist)-heading);
    }

    if ((nextBearing>-M_PI*.5) && (nextBearing<M_PI*.5)) {
      if ((target == NULL) || (nextRange < targetRange)) {
	targetRange = nextRange;
	targetBearing = nextBearing;
	target = next;
      }
    }

    next = next->next;
  }

  if (range) {
    *range = targetRange;
  }

  if (bearing) {
    *bearing   = targetBearing;
  }

  return(target);
}

/*------------------------------------------------------------*/

int
sonarCallback(sonarType *newSonar)
{
  float worldAngle;
  int ii;
  float X, Y;

  sonarsActive = 6;

  if (!gotOrigin) {
    return(0);
  }

  if (!useSonar) {
    return(0);
  }

  for (ii=0; ii < bRobot.sonar_cols[0]; ii++) {
    if (newSonar[ii].mostRecent && newSonar[ii].value/10 < 200) {
      worldAngle = bNormalizeAngle(bWorldAngle(bSonarAngle(0, ii), 0.0));
      X = ((float)newSonar[ii].value)/10.0 * cos(worldAngle);
      Y = ((float)newSonar[ii].value)/10.0 * sin(worldAngle);

      addTarget(TARGET_TYPE_SONAR, 0, ii, newSonar[ii].value,
		bRobotX(0.0) + X, bRobotY(0.0) + Y, TARGET_LIFE_SONAR);
    }
  }

  return(0);
}
  
/*------------------------------------------------------------*/

int
irCallback(irType ** newIr)
{
  float worldAngle;
  int ii, jj;
  float X, Y;
  float range;

  if (!gotOrigin) {
    return(0);
  }

  if (!useIr) {
    return(0);
  }

  /* skip the down looking row of the B21 */

  for (ii=0; (ii < bRobot.ir_rows) && (ii < 2); ii++) {
    for(jj=0; jj<bRobot.ir_cols[ii]; jj++) {
      if (newIr[ii][jj].mostRecent && newIr[ii][jj].value>40) {

	worldAngle = bNormalizeAngle(bWorldAngle(bIrAngle(ii, jj), 0.0));
	range = 30.0 - (float)newIr[ii][jj].value/3.5 + bRobot.base_radius;
	X = range * cos(worldAngle);
	Y = range * sin(worldAngle);

	addTarget(TARGET_TYPE_IR, ii, jj, newIr[ii][jj].value,
		  bRobotX(0.0) + X, bRobotY(0.0) + Y, TARGET_LIFE_IR);
      }
    }
  }

  return(0);
}
  
/*------------------------------------------------------------*/

int
tactileCallback(tactileType ** newTactile)
{
  float worldAngle;
  int ii, jj;
  float X, Y;

  for(ii=0; ii<bRobot.tactile_rows; ii++) {
    for(jj=0;jj<bRobot.tactile_cols[ii];jj++) {
      if (tactiles[ii][jj].value) {

	rotateLimp();
	translateLimp();

	if (gotOrigin && useTactile) {
	  worldAngle =
	    bNormalizeAngle(bWorldAngle(bTactileAngle(ii, jj), 0.0));
#if 0
	  fprintf(stderr,
		  "%s:%6d:%s() - robotAngle = %5.0f  worldAngle = %5.0f\n",
		  __FILE__, __LINE__, __FUNCTION__,
		  0.0, worldAngle*180.0/M_PI);
#endif
	  X = bRobot.base_radius * cos(worldAngle);
	  Y = bRobot.base_radius * sin(worldAngle);

	  addTarget(TARGET_TYPE_TACTILE, ii, jj, 1,
		    bRobotX(0.0) + X, bRobotY(0.0) + Y, TARGET_LIFE_TACTILE);
	}
      }
    }
  }
  
  return;
}

/*------------------------------------------------------------*/

void
statusCallback(statusReportType * newStatus)
{
  int ii;
  int jj;
  float X;
  float Y;
  float heading;
  float range, bearing;
  float transSpeed;
  float transVel;
  float rotVel;
  float newPan, newTilt;
  static int ptCount= 100;
  struct targetStruct *target;

  if (sonarsActive) {
    sonarsActive--;
  }
  else {
    rotateLimp();
    translateLimp();
    fprintf(stderr, "%s:%d:%s() - sonar timeout.\n",
	    __FILE__, __LINE__, __FUNCTION__);
    return;
  }


  watchdogTimer(0x100);             /* Units = seconds*256         */

  if (!indexed) {
    return;
  }
    
  statusCount++;

  if (statusCount<2) {
    return;
  }

  if (!gotOrigin) {
    gotOrigin=TRUE;
  }

  X = bRobotX(0.2);
  Y = bRobotY(0.2);
  heading = bNormalizeAngle(bRobotHeading(0.2));

  target = closestFront(X, Y, heading, &bearing, &range);

  /*
   * if on top of obstacle, stop. This is normal when just
   * starting and all obstacles are initialized to range 0
   */

  if (range < .01) {
    if (limp) {
      rotateLimp();
      translateLimp();
    }
    else {
      bSetVel(0.0, 0.0);
    }
    return;
  }

  if (range==safeDistance) {
    transSpeed = 0.0;
  }
  else if (range<safeDistance) {
    if (fabs(bearing) < 60.0*M_PI/180.0) {
      transSpeed = -0.5*(safeDistance-range);
    }
    else {
      transSpeed = 0.0;
    }
  }
  else {
    transSpeed =  sqrt(2.0*(transAccel/2.0)*(range-safeDistance));
  }

  if (transSpeed>maxSpeed) {
    transSpeed=maxSpeed;
  }

  transVel = transSpeed;
  rotVel = 0;

  if (range<10*safeDistance) {
    if (bearing==0) {
      bearing=0.01;
    }

    rotVel = -30.0/bearing/range;

    if (rotVel > M_PI_2) {
      rotVel = M_PI_2;
    }

    if (rotVel < -M_PI_2) {
      rotVel = -M_PI_2;
    }
    
    if (rotVel > rotSpeed) {
      rotVel = rotSpeed;
    }
    
    if (rotVel < -rotSpeed) {
      rotVel = -rotSpeed;
    }
    
  }

  if ((range<0.7*safeDistance) && (fabs(bearing)<60*M_PI/180.0)) {
    rotVel = -rotVel;
  }

#if 1
  {
    static int count= 100;
    char *targetType;
    int ii, jj;
    float robotAngle;
    int row, col, val;

    if (target) {
      switch (target->type) {
      case TARGET_TYPE_VIRT:    targetType = "V"; break;
      case TARGET_TYPE_SONAR:   targetType = "S"; break;
      case TARGET_TYPE_IR:      targetType = "I"; break;
      case TARGET_TYPE_TACTILE: targetType = "T"; break;
      default: targetType = "*?*";
      }
      row = target->row;
      col = target->col;
      val = target->val;
    }
    else {
      targetType = "none";
      row = 0;
      col = 0;
      val = 0;
    }

    if (count++>40) {
      fprintf(stderr,
	      "%s:%6d:%s() - %s[%3d,%3d:%5d] range: %5.1f "
	      "bear: %5.1f rot: %5.1f trans: %5.1f\n",
	      __FILE__, __LINE__, __FUNCTION__,
	      targetType, row, col, val,
	      range, bearing*180.0/M_PI,
	      rotVel*180/M_PI, transVel);
      count=0;
    }
  }
#endif
#if 0
  {
    static int count= 100;
    
    if (count++>10) {
      fprintf(stderr, "%s:%6d:%s() - X[%8.2f] Y[%8.2f] H[%8.2f]\n",
	      __FILE__, __LINE__, __FUNCTION__,
	      bRobotX(0.0),
	      bRobotY(0.0),
	      bRobotHeading(0.0)*180.0/M_PI);
      count=0;
    }
  }
#endif

  if (limp) {
    rotateLimp();
    translateLimp();
  }
  else {
    bSetVel(rotVel, transVel);
  }

  if (ptCount++>5) {

    newPan = rotVel/2.0;
    newTilt = atan2(-20.0, transVel);

    ptMoveTo(newPan, newTilt);

#if 0
    fprintf(stderr, "%s:%6d:%s() - pan[%8.2f] tilt[%8.2f]\n",
	    __FILE__, __LINE__, __FUNCTION__,
	    newPan*180.0/M_PI, newTilt*180.0/M_PI);
#endif

    ptCount=0;
  }

  return;
}
/*------------------------------------------------------------*/

void
baseCallback(unsigned long opcode, unsigned long value)
{

  switch (opcode) {

   /* error conditions */
  case BASE_translateError:
  case BASE_rotateError:
  case BASE_batteryHigh:
  case BASE_batteryLow:
    fprintf(stderr, "%s:%6d:%s() - error code = %d\n",
	    __FILE__, __LINE__, __FUNCTION__, opcode);
    break;

  case BASE_rotateHalt:
    fprintf(stderr, "%s:%6d:%s() - BASE_rotateHalt\n",
	    __FILE__, __LINE__, __FUNCTION__);
    break;

  case BASE_translateHalt:
    fprintf(stderr, "%s:%6d:%s() - BASE_translateHalt\n",
	    __FILE__, __LINE__, __FUNCTION__);
    break;

   /* commands that return values, message is used when returning value too */
  case BASE_batteryCurrent:
    fprintf(stderr, "%s:%6d:%s() - BASE_batteryCurrent = %5.1f\n",
	    __FILE__, __LINE__, __FUNCTION__, (float)value/10.0);
    break;

  case BASE_batteryVoltage:
    fprintf(stderr, "%s:%6d:%s() - BASE_batteryVoltage = %5.1f\n",
	    __FILE__, __LINE__, __FUNCTION__, (float)value/10.0);
    break;

  case BASE_rotateCurrent:
    fprintf(stderr, "%s:%6d:%s() - BASE_rotateCurrent = %d\n",
	    __FILE__, __LINE__, __FUNCTION__, value);
    break;

  case BASE_rotateWhere:
    fprintf(stderr, "%s:%6d:%s() - BASE_rotateWhere = %d\n",
	    __FILE__, __LINE__, __FUNCTION__, value);
    break;

  case BASE_translateCurrent:
    fprintf(stderr, "%s:%6d:%s() - BASE_translateCurrent = %d\n",
	    __FILE__, __LINE__, __FUNCTION__, value);
    break;

  case BASE_translateWhere:
    fprintf(stderr, "%s:%6d:%s() - BASE_translateWhere = %d\n",
	    __FILE__, __LINE__, __FUNCTION__, value);
    break;


  case BASE_indexReport:
    fprintf(stderr, "%s:%6d:%s() - BASE_indexReport = %d\n",
	    __FILE__, __LINE__, __FUNCTION__, value);

    bSetPosition(0.0, 0.0, 0.0);
    indexed = 1;
    break;

  default:
    fprintf(stderr, "%s:%6d:%s() - unexpected opcode = %d, value= %d\n",
	    __FILE__, __LINE__, __FUNCTION__, opcode, value);
  }

  return;
}

/*------------------------------------------------------------*/

void
demoPoll(RaiModule * demoModule)
{
  ageTargets();
}

/*------------------------------------------------------------*/

void
createDemoModule()
{
  RaiModule* demoModule;
  int ii;
  int virtTargets;
  float X, Y, angle;

  memset(&targetHead, 0, sizeof(targetHead));
  targetHead.type = TARGET_TYPE_NONE;
  targetHead.next = NULL;

  if (useVirt) {
    virtTargets = exploreRange * 2.0 * M_PI / 25.0; /* one target every 25cm */
    
    for (ii=0; ii<virtTargets; ii++) {
      angle = 2.0 * M_PI / virtTargets * (float)ii;
      X = exploreRange * cos(angle);
      Y = exploreRange * sin(angle);
      
      addTarget(TARGET_TYPE_VIRT, (int)exploreRange,
		(int)(angle * 180.0 / M_PI), 1, X, Y, TARGET_LIFE_VIRT);
    }
  }

  setTranslateAcceleration(transAccel*10);  /* units = mm/sec^2      */
  setRotateAcceleration(rotAccel*1023/M_PI);  /* units = 1024/2PI/sec  */
  setRotateVelocity(0x40);          /* units = radians*512/PI/sec  */
  loadPosition(0x80008000);         /* units = encoder counts/256  */
  statusReportPeriod(100*256/1000); /* Units = seconds*256         */

  bRegisterOdometryLockCallback(odofn);
  bRequestOdometryLock(2);

  registerSonarCallback(sonarCallback);
  registerIrCallback(irCallback);
  registerTactileCallback(tactileCallback);

  registerBaseCallback(baseCallback);
  registerStatusCallback(statusCallback);

  /* ask that your polling function be run every 500msec */
  demoModule=makeModule("Demo", NULL);
  addPolling(demoModule, demoPoll, 500);

  findRotIndex();
  assumeWatchdog();

  return;
}

/*------------------------------------------------------------*/

void
commShutdown(char *name, TCX_MODULE_PTR module)
{

  fprintf( stderr, "%s(%s): %s died. Closing. \n", 
	   __FILE__, __FUNCTION__, name );

  module = module;		/* prevents gcc from barking with `-Wall' */
  fflush(NULL);
  RaiShutdown();
  exit(0);
}

/*------------------------------------------------------------*/

int
main(int argc, char** argv )
{
  struct bParamList * params = NULL;

  /* set defaults */

  params = bParametersAddEntry(params, "", "safetyMargin", "20"); /* cm */
  params = bParametersAddEntry(params, "", "exploreRange", "200"); /* cm */
  params = bParametersAddEntry(params, "", "transSpeed", "40"); /* cm/s */
  params = bParametersAddEntry(params, "", "transAccel", "80"); /* cm/s^2 */
  params = bParametersAddEntry(params, "", "rotSpeed", "30"); /* deg/s */
  params = bParametersAddEntry(params, "", "rotAccel", "80"); /* deg/s^2 */

  params = bParametersAddEntry(params, "", "useVirt", "Y");
  params = bParametersAddEntry(params, "", "useSonar", "Y");
  params = bParametersAddEntry(params, "", "useIr", "Y");
  params = bParametersAddEntry(params, "", "useTactile", "Y");
  params = bParametersAddEntry(params, "", "limp", "n");

  /* add some parameter files */
  params = bParametersAddFile(params, "etc/beeSoft.ini");

  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(params);

  fprintf(stderr, "safetyMargin = [%s]\n", 
	  bParametersGetParam(params, "", "safetyMargin"));

  safetyMargin = atof(bParametersGetParam(params, "", "safetyMargin"));
  exploreRange = atof(bParametersGetParam(params, "", "exploreRange"));
  maxSpeed     = atof(bParametersGetParam(params, "", "transSpeed"));
  transAccel   = atof(bParametersGetParam(params, "", "transAccel"));

  rotSpeed = atof(bParametersGetParam(params, "", "rotSpeed"))*M_PI/180.0;
  rotAccel = atof(bParametersGetParam(params, "", "rotAccel"))*M_PI/180.0;

  safeDistance = safetyMargin + bRobot.base_radius;

  useVirt    = bStrToTruth(bParametersGetParam(params, "", "useVirt"));
  useSonar   = bStrToTruth(bParametersGetParam(params, "", "useSonar"));
  useIr      = bStrToTruth(bParametersGetParam(params, "", "useIr"));
  useTactile = bStrToTruth(bParametersGetParam(params, "", "useTactile"));
  
  limp = bStrToTruth(bParametersGetParam(params, "", "limp"));

  registerBaseClient();
  ptRegister();

  initClient("Wander", commShutdown); /* close function called if */
                                      /* the base server dies */

  baseConnect(1);                     /* hook up to running server*/
  ptConnect(0);

  ptSetAccel(135.0*M_PI/180.0);
  ptSetVel  ( 70.0*M_PI/180.0);

  RaiInit();                          /* init (but not start) scheduler*/
  catchInterrupts();

  initClientModules();                /* set up Rai modules to do      */
                                      /* communication for you */

  createDemoModule();                 /* set up your module to move robot*/

  fprintf(stderr, "\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "********  OPTIONS  **********\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "-safetyMargin=%f\n",    safetyMargin);
  fprintf(stderr, "-exploreRange=%f\n",    exploreRange);
  fprintf(stderr, "  -transSpeed=%f\n",    maxSpeed);
  fprintf(stderr, "  -transAccel=%f\n",    transAccel);
  fprintf(stderr, "    -rotSpeed=%f\n",    (float)(rotSpeed * 180.0/M_PI));
  fprintf(stderr, "    -rotAccel=%f\n",    (float)(rotAccel * 180.0/M_PI));
  fprintf(stderr, "\n");
  fprintf(stderr, "     -useVirt=%d\n",    useVirt);
  fprintf(stderr, "    -useSonar=%d\n",    useSonar);
  fprintf(stderr, "       -useIr=%d\n",    useIr);
  fprintf(stderr, "  -useTactile=%d\n",    useTactile);
  fprintf(stderr, "        -limp=%d\n",    limp);
  fprintf(stderr, "\n");
  fprintf(stderr, "*****************************\n");

  RaiStart(); /* This will not return */
  exit(0);
}



int odofn(int param) {
  if (param == B_ODOMETRY_I_HAVE_LOCK)
    fprintf(stderr, "I have the odometry lock.\n");
  else
    fprintf(stderr, "Someone else has the odometry lock.\n");
}
