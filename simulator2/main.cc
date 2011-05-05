
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


#include <sys/time.h>
#include <bUtils.h>

#include "tcx.h"
#include "robot.h"
#include "playground.hh"
#include "simxw.h"
#include "sonar.h"
#include "ir.h"
#include "tactile.h"
#include "laser.h"
#include "surface.hh"
#include "schedule.hh"
#include "store.hh"

extern t_obstgrid *obstore;
extern char editing;

extern "C" {
    void laserReport();
    void robot_HandleMove();
}

#define DRAW_PRIO 5
#define LASER_PRIO 2
#define BASE_PRIO 0
#define STATUS_PRIO -1
#define SONAR_PRIO -1

#define SONAR_EVENT 1


/*************  timer intervals in ms for different tasks ****************/

int basevar_update_interval = 50;
int laser_update_interval = 200;
int robot_display_interval = 100;
int status_report_interval = 100;
int sonarServer_report_interval = 100;
int irServer_report_interval = 100;
int tactileServer_report_interval = 100;
int tcx_query_interval = 10;

t_scheduler *sim_scheduler;
struct timeval robot_display_duration = {0,1000};
struct timeval laser_update_duration = {0,1000};
struct timeval basevar_update_duration = {0,1000};
struct timeval tcx_query_duration = {0,1000};
struct timeval status_report_duration = {0,1000};
struct timeval sonar_report_duration = {0,1000};
struct timeval sonarServer_report_duration = {0,1000};
struct timeval irServer_report_duration = {0,1000};
struct timeval tactileServer_report_duration = {0,1000};

struct timeval robot_display_delta = {0, 0};
struct timeval laser_update_delta = {0, 0};
struct timeval basevar_update_delta = {0, 0};
struct timeval tcx_query_delta = {0, 0};
struct timeval status_report_delta = {0, 0};
struct timeval sonar_report_delta = {0, 0};
struct timeval sonarServer_report_delta = {0, 0};
struct timeval irServer_report_delta = {0, 0};
struct timeval tactileServer_report_delta = {0, 0};

char mapfile[80];
char inifile[80];
Boolean just_viewing = FALSE;
Boolean without_laser = FALSE;
Boolean want_help = FALSE;
Boolean no_tcx = FALSE;
Boolean mark_option = FALSE;

Boolean use_baseServer = FALSE;
Boolean use_sonarServer = FALSE;

typedef struct option {
    char *token;
    int type;
    union {
	Boolean *vbool;
	char *vstring;
    } value;
    char *descr;
};

#define TOGGLE 0
#define VALUE 1

char *message_help1 = "This message";
char *message_help2 = "dito";
char *message_help3 = "dito";
char *message_map   = "Load the mapfile <value>";
char *message_ini   = "Load the inifile <value>";
char *message_view  = 
  "Do not simulate the robot,\n"
  "\tsimply ask BASE for the robots position.\n"
  "\tUsing this option, the simulator is internally renamed to\n"
  "\tVIEWER, so you can connect BASE to another simulator\n";
char *message_mark =
   "Mark to front side of obstacles with a dot.\n"
   "Only cubes are handled yet. The front side is defined to be at\n"
   "the center point minus width/2.\n";
char *message_laser = 
"Disable laser simulation, this is usful, if BASE runs without\n"
"\t\tlaser support. It is not strictly neccessary, but it saves\n"
"\t\tsome CPU time";

char *message_notcx = "Do not connect, use simulator for map editing only";
char *message_baseServer   = "Use baseServer protocol instead "
                             "of BASE protocol\n";

#define NO_OPTIONS 9
option options[NO_OPTIONS] =
{
    {"-h", TOGGLE, &want_help, message_help1},
    {"-help", TOGGLE, &want_help, message_help2},
    {"--help", TOGGLE, &want_help, message_help3},
    {"-map", VALUE, mapfile, message_map},
    {"-ini", VALUE, inifile, message_ini},    
    {"-view", TOGGLE, &just_viewing, message_view},
    {"-laser", TOGGLE, &without_laser, message_laser},
    {"-tcx", TOGGLE, &no_tcx, message_notcx},
    {"+mark", TOGGLE, &mark_option, message_mark}
};

extern "C" {
void  InitBaseVar();  // from base.h which is incompatible with c++
void read_inifile(char *);
}

extern t_surflist *surfaces;

usage(char *exec)
{
    int i;
    printf("usage: %s [options]\n\nOptions:\n\n", exec);
    for( i = 0; i < NO_OPTIONS; i++) {
	switch(options[i].type) {
	case TOGGLE:
	    printf("%s\t\t%s\n",options[i].token,options[i].descr);
	    break;
	case VALUE:
	    printf("%s\t<value>\t%s\n",options[i].token,options[i].descr);
	}
    }
    /*    exit(0); */
}

read_options(int argc, char **argv)
{
    int i,j, found;
    for (i = 1; i < argc; i++) {
	found = 0;
	for(j = 0; j < NO_OPTIONS; j++) {
	    if(strcmp(options[j].token, argv[i]) == 0) {
		found = 1;
		break;
	    }
	}
	if(found) {
	    switch(options[j].type) {
	    case TOGGLE:
		*options[j].value.vbool = TRUE;
		break;
	    case VALUE:
		if(i < argc-1) {
		    strcpy(options[j].value.vstring,argv[i+1]);
		    i++;
		}
		else {
		    fprintf(stderr, "Missing value for option %s\n",argv[i]);
		    usage(argv[0]);
		}
	    }
	}
	else {
	    fprintf(stderr, "Unknown option %s\n", argv[i]);
	    usage(argv[0]);
	}
    }
}


void
MoveRobotJob()
{
    MoveRobot();
    if(!editing) obstore->Redraw();
    sim_scheduler->schedule(REGULAR, MoveRobotJob,
			    DRAW_PRIO, robot_display_interval*1000,
			    &robot_display_duration, &robot_display_delta);
}
void
LaserReportJob()
{
    laserReport();
    sim_scheduler->schedule(REGULAR, LaserReportJob,
			    LASER_PRIO, laser_update_interval*1000,
			    &laser_update_duration, &laser_update_delta);
}
void
BaseUpdateJob()
{
    robot_HandleMove();
    if(!editing) obstore->Update();    
    sim_scheduler->schedule(REGULAR, BaseUpdateJob,
			    BASE_PRIO, basevar_update_interval*1000,
			    &basevar_update_duration, &basevar_update_delta);    
}

extern "C" {
    void schedule_statusReport(int);
    void schedule_sonarReport(int);    
    void schedule_sonarServerReport(int);    
    void schedule_irServerReport(int);    
    void schedule_tactileServerReport(int);    
    void statusReport();
    void sonarReport();
    int sonarServerReport();
    int irServerReport();
    int tactileServerReport();
}

void the_statusReport()
{
    statusReport();
    sim_scheduler->schedule(REGULAR, the_statusReport,
			    STATUS_PRIO, status_report_interval*1000,
			    &status_report_duration, &status_report_delta);
}
void schedule_statusReport(int delay)
{
    status_report_interval = delay;
    sim_scheduler->schedule(REGULAR, the_statusReport,
			    STATUS_PRIO, delay*1000,
			    &status_report_duration,
			    &status_report_delta);
}
void schedule_sonarReport(int delay)
{
    sim_scheduler->schedule(SONAR_EVENT, (void (*)()) sonarReport,
			    SONAR_PRIO, delay*1000,
			    &sonar_report_duration, &sonar_report_delta);
}


void
the_irServerReport()
{
  if (irServerReport()) {
    sim_scheduler->schedule(REGULAR, the_irServerReport,
			    STATUS_PRIO, irServer_report_interval*1000,
			    &irServer_report_duration,
			    &irServer_report_delta);
  }
}

void
schedule_irServerReport(int delay)
{

  fprintf(stderr, "%10s:%5d:%14s()\n",
	  __FILE__, __LINE__, __FUNCTION__);

    irServer_report_interval = delay;
    sim_scheduler->schedule(REGULAR, the_irServerReport,
			    STATUS_PRIO, delay*1000,
			    &irServer_report_duration,
			    &irServer_report_delta);
}


void
the_tactileServerReport()
{
  if (tactileServerReport()) {
    sim_scheduler->schedule(REGULAR, the_tactileServerReport,
			    STATUS_PRIO, tactileServer_report_interval*1000,
			    &tactileServer_report_duration,
			    &tactileServer_report_delta);
  }
}

void
schedule_tactileServerReport(int delay)
{
  fprintf(stderr, "%10s:%5d:%14s()\n",
	  __FILE__, __LINE__, __FUNCTION__);

    tactileServer_report_interval = delay;
    sim_scheduler->schedule(REGULAR, the_tactileServerReport,
			    STATUS_PRIO, delay*1000,
			    &tactileServer_report_duration,
			    &tactileServer_report_delta);
#if 1
  fprintf(stderr, "%10s:%5d:%14s() - return\n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  return;
}


void
the_sonarServerReport()
{
  if (sonarServerReport()) {
    sim_scheduler->schedule(REGULAR, the_sonarServerReport,
			    STATUS_PRIO, sonarServer_report_interval*1000,
			    &sonarServer_report_duration,
			    &sonarServer_report_delta);
  }
}

void
schedule_sonarServerReport(int delay)
{
    sonarServer_report_interval = delay;
    sim_scheduler->schedule(REGULAR, the_sonarServerReport,
			    STATUS_PRIO, delay*1000,
			    &sonarServer_report_duration,
			    &sonarServer_report_delta);
#ifndef UNIBONN
    tactileInit();
#endif
}


main(int argc, char *argv[])
{
  struct bParamList * bParamList = NULL;
  int event_spec;
  struct timeval waiting_time;
  inifile[0] = 0;
  read_options(argc, argv);
  surfaces = new t_surflist();
  read_inifile(inifile);

  bParamList = bParametersAddEntry(bParamList, "robot", "name", "B21");
  bParamList = bParametersAddEntry(bParamList, "", "TCXHOST", "localhost");
  bParamList = bParametersAddEntry(bParamList, "", "fork", "yes");
  bParamList = bParametersAddFile (bParamList, "etc/beeSoft.ini");

#ifdef UNIBONN
  bParamList = bParametersAddEntry(bParamList, "", "fork", "no");
#endif

  /* add some enviroment variables */
  bParamList = bParametersAddEnv(bParamList, "", "TCXHOST");

  /* add command line arguements */
  bParamList = bParametersAddArray(bParamList, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(bParamList);

  InitWidgets(&argc, argv);
  map = new t_playground(mapfile, argc, argv);
  InitRobot();
  InitBaseVar();
  InitSonar();
  if(!no_tcx) {

      if (bRobot.fork) {
         bDaemonize("simulator2");
      }

      fprintf(stderr, "TCXHOST=[%s]\n", bRobot.TCXHOST);

      InitTCX(bRobot.TCXHOST);


      sim_scheduler = new t_scheduler();
      sim_scheduler->schedule(REGULAR, MoveRobotJob,
			      DRAW_PRIO, robot_display_interval*1000,
			      &robot_display_duration, &robot_display_delta);
      if(!without_laser) {
	  sim_scheduler->schedule(REGULAR, LaserReportJob,
				  LASER_PRIO, laser_update_interval*1000,
				  &laser_update_duration, &laser_update_delta);
      }
      if(!just_viewing) {
	  sim_scheduler->schedule(REGULAR, BaseUpdateJob,
				  BASE_PRIO, basevar_update_interval*1000,
				  &basevar_update_duration, &basevar_update_delta);
      }
      while(1) {
	  event_spec = sim_scheduler->triggerNextEvent();
	  switch(event_spec) {
	  case 0:
	  case 1:
	      query_x();
	      break;
	  case 2:
	      waiting_time.tv_sec = 0;
	      waiting_time.tv_usec = 0;		
	      tcxRecvLoop(&waiting_time);
	      break;
	  default:
	      break;
	  }
      }
  }
  else {
      mainloop();
  }
}
