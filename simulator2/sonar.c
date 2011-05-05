
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


#include <sonarClient.h>

#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "sundefines.h"
#include "playground.hh"
#include "robot.h"
#include "sonar.h"
#include "trigofkt.h"
#include "surface.hh"

#define ctoi(x)         ((x) - '0')


#define TCX_define_variables                            /*** this makes sure variables are installed ***/
#include "SIMULATOR-messages.h"

TCX_MODULE_PTR MODULE_SONAR = NULL;


float sonar_angle = SONAR_ANGLE;
float sonar_range = SONAR_RANGE;
float sonar_infinity = SONAR_INFINITY;
float sonar_offset = SONAR_OFFSET;
float sonar_zpos = 80.0;
float sonar_malfunc_rate = 0.02;
int rays_per_sonar = RAYS_PER_SONAR;

static float sonar_readings[B_MAX_SENSOR_COLS];
float sonar_dists[B_MAX_SENSOR_COLS];


static struct SonarVar {
  long    CP;             /*** Chain Power          ***/
  char    RT[5];          /*** Read Transducers     ***/

  /*
   * MSP type sonar variables
   */

  int sonarRunning;
  int lastTactileValues;
  sonarType sonars[B_MAX_SENSOR_COLS];

} SonarVariables;

static char report[NO_CHAINS][81], commandBack[81];

/*
#define SONARDEBUG
*/



/**********************************************************************************************************/
/* PROCEDURE :			sonar()      				***********************************/
/* Parameter :			command line				***********************************/
/* 									***********************************/
/* scans command line and performs sonar action				***********************************/
/* 									***********************************/
/**********************************************************************************************************/

void
sonar(char command[])
{
   int	i, j, k;
   long	ticks;
   float dist = 0, maxdist = 0.0;
   char	s[5], buf[81], buf2[81];
   int	sonar_no[NO_CHAINS];

   static int VsonarToRsonar[] = {
   	0,1,2,3,4,5,
   	6,7,8,9,10,11,
   	12,13,14,15,16,17,
   	18,19,20,21,22,23
   };
   
   static int RsonarToVsonar[] = {
   	0,1,2,3,4,5,
   	6,7,8,9,10,11,
   	12,13,14,15,16,17,
   	18,19,20,21,22,23
   };  

   struct timeval       before, after;
   int			time_lost;
   
   float GetDistance(int cell);

#ifdef SONARDEBUG
   fprintf(stderr, "SONAR got command : %s\n", command);
#endif
   
   if (strncmp(command, "RT", 2) == 0) {		/*** Read Transducers ***/
     
     if (SonarVariables.CP == 0) {			/*** Chain Power ? ***/
       fprintf(stderr, "Tried to Read Transducers while Chain Power is OFF !!!\n");
       return;
     }

     gettimeofday(&before, NULL);		/*** used to calculate amount of time lost when ***/
     						/*** caclulating sonar responases 		***/
     sscanf(command,"RT %s", buf);

     strcpy(commandBack, command);

     *buf2='\0';					/*** leading zeros !! 	    ***/
     while (strlen(buf)+strlen(buf2) < NO_CHAINS)	/*** important when not all ***/
     	strcat(buf2, "0");				/*** chains are on          ***/

     strcat(buf2, buf);

     for (i=0; i<NO_CHAINS; i++) {
	
       j = ctoi(buf2[i]);

       if (j != 0) {
	 dist = GetDistance(VsonarToRsonar[j + (NO_CHAINS-i-1)*6 - 1]);
	 if (dist > maxdist)
		maxdist = dist;
	 ticks = (long) (dist * 2 / CM_PER_SECOND / SECONDS_PER_CYCLE);
     
	 sprintf(report[i], "%4x", ticks);
	 for (k = 0; k < (int)strlen(report[i]); k++)
		if (isspace(report[i][k]))
			report[i][k] = '0';

	 strcat(report[i], " ");
	 if (strlen(report[i]) > 5) {
	   strcpy(report[i], "FFFF ");	   
	 }
       } else 
	   strcpy(report[i], "");
       
     } /* for */		
     gettimeofday(&after, NULL);		/*** used to calculate amount of time lost when ***/
     						/*** caclulating sonar responases 		***/
     
     time_lost = (after.tv_usec - before.tv_usec) / 1000;		/*** ms ***/
     if (time_lost < 0)
     	time_lost = 1000 + time_lost;
#ifdef DEBUG

#endif
     schedule_sonarReport(SONAR_ANSWER_TIME - time_lost);
   } else if (strncmp(command, "CP", 2) == 0) {		/*** Chain Power ***/
     /*** immer ein ***/
     SonarVariables.CP = 1;
   }
   
   else
     fprintf(stderr, "Unknown base command sent to sonar (simulator):  %s !\n", command);
   return;
   
} /* sonar() */

/**********************************************************************
 *
 * MSP type sonar support - tds
 *
 **********************************************************************/

mspSonarStart(int value) {
  extern void schedule_sonarServerReport(int delay);

#ifdef SONARDEBUG
  fprintf(stderr, "%10s:%5d:%14s(): %d\n",
	  __FILE__, __LINE__, __FUNCTION__, value);
#endif

  SonarVariables.sonarRunning = value;
  if (value) {
    schedule_sonarServerReport(100);
  }
}

void
sonarServer(char *message) {
  extern void mspIrStart(int);

  /*
   * XXX - This is a total gross hack right now. -tds
   */

  mspSonarStart(message[0]);
}

/*
 * Get sonar data and send sonar messages
 */

int
sonarServerReport (void) {
  static int setNum = 0;
  int sonarNum, sonar;
  unsigned long value;
  struct timeval time;
  float dist;

  float GetDistance(int cell);

  if (!SonarVariables.sonarRunning) {
    return(0);
  }

  gettimeofday(&time, NULL);    /* get time first */

  /* make previous data old */
  for (sonar = 0; sonar < bRobot.sonar_cols[0]; sonar++) {
    SonarVariables.sonars[sonar].mostRecent = FALSE;
  }
  
  /* iterate through new data */
  sonar = 0;

  for (sonar=0; sonar<6; sonar++) {
    sonarNum = setNum + sonar*4;
    memcpy(&(SonarVariables.sonars[sonarNum].time), &time,
	   sizeof(SonarVariables.sonars[sonarNum].time));
    SonarVariables.sonars[sonarNum].mostRecent = TRUE;

    dist = GetDistance((sonarNum+(((int) bRobot.sonar_cols[0])/2))%
		       bRobot.sonar_cols[0]);

    value = (unsigned long)(dist * 10);
    value = value + (10.0 * bRobot.enclosure_radius);
  
    if (value > NO_RETURN) {
      value = NO_RETURN;
    }

    SonarVariables.sonars[sonarNum].value = value;
  }

  setNum = (++setNum) & 0x03;

  /* call callback */
  /* if (moduleOps.sonarRep) return(moduleOps.sonarRep(sonars)); */

  /*
   * Do TCX message
   */

#ifdef SONARDEBUG
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif
  
  tcxSendMsg(MODULE_SONAR, "SIMULATOR_message_to_sonarServer",
	     SonarVariables.sonars);

  return(1);
}


/**********************************************************************************************************/
/* PROCEDURE :			InitSonar()    				***********************************/
/* Parameter :			none					***********************************/
/* 									***********************************/
/* obvious functionality						***********************************/
/* 									***********************************/
/**********************************************************************************************************/


void
InitSonar()
{
  int	i;


  for (i = 0; i < bRobot.sonar_cols[0]; i++)
	sonar_readings[i] = 0.0;

  return;

} /* InitSonar() */




/**********************************************************************************************************/
/* PROCEDURE :			GetDistance()      			***********************************/
/* Parameter :			No. of cell				***********************************/
/* 									***********************************/
/* Pass through list of obstacles and find the closest one inside range ***********************************/
/* Check angle and surface and decide if obstacle is recognized		***********************************/
/* 									***********************************/
/**********************************************************************************************************/

static int
round( float x){
  return (int) x + 0.5;
}

static int
randMax(int max)
{
  return (int) (((float) (max + 1) * rand()) / (RAND_MAX + 1.0));
}


float
GetDistance(int cell)
{
  float tdist,dist;
  float PosX, PosY,EndX,EndY;
  float rx,ry,rori;
  float f,deg,rad;
  float open_angle;
  float sonarAngleDistance = 360.0 / bRobot.sonar_cols[0];

  getRobotPosition(&rx,&ry,&rori);

  deg = ( rori - sonar_offset - cell * sonarAngleDistance);

  if (sonar_malfunc_rate > 0)
    if (randMax(round(1/sonar_malfunc_rate)) == 0){
      sonar_dists[cell] = sonar_infinity;
      return sonar_infinity;
    }
  tdist = dist = sonar_infinity;

/* we test only one beam at random */ 
  f = -sonar_angle/2;
  f += sonar_angle/rays_per_sonar * randMax(rays_per_sonar);

  rad = (deg + f) * M_PI /180;
  PosX = rx + robot.RobotRadius * cos(rad);
  PosY = ry + robot.RobotRadius * sin(rad);
  EndX = PosX + sonar_range * cos(rad);
  EndY = PosY + sonar_range * sin(rad);
    /*  open_angle determines the z-range which is hit depending on */ 
  /* the obstacles distance */
  open_angle = sqrt(sonar_angle*sonar_angle/4 - f*f) * M_PI/180.0;
  if(get_distance(SONAR_SENSOR,
		  PosX, PosY, sonar_zpos,
		  open_angle,
		  EndX, EndY, &dist)) {
    
    sonar_dists[cell] = dist;
  }
  else
    sonar_dists[cell] = sonar_infinity;
  return dist;
} /* GetDistance() */



/**********************************************************************************************************/
/* PROCEDURE :			sonarReport()      			***********************************/
/* Parameter :			none					***********************************/
/* 									***********************************/
/* Sends sonar report via TCX						***********************************/
/* 									***********************************/
/**********************************************************************************************************/


void
sonarReport()
{
    int i;

#ifdef SONARDEBUG
    fprintf(stderr, "SIMULATOR :   SENDING  SONAR Report :  %s\n", report);
#endif

    if (MODULE_SONAR != NULL){
    	char *reply_msg;

    	reply_msg = (char *) malloc(256);
    	strcpy(reply_msg, commandBack);

#ifdef SONARDEBUG
	fprintf(stderr, "%10s:%5d:%14s(): \n",
		__FILE__, __LINE__, __FUNCTION__);
#endif
    	tcxSendMsg(MODULE_SONAR, "SIMULATOR_message_to_sonar", &reply_msg);

	for (i = 0; i < NO_CHAINS; i++) {
    	   strcpy(reply_msg, report[i]);
	   if (strcmp(reply_msg, "") != 0) {
    	   	tcxSendMsg(MODULE_SONAR, "SIMULATOR_message_to_sonar", &reply_msg);
	   }
        }

    	free (reply_msg);
    }  

    return;

} /* sonarReport() */
