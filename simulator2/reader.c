
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

#include "surface.hh"
#include "sonar.h"

#define INTEGER 0
#define FLOAT 1
#define STRING 2
#define FUNC 3

#define UNIQUE 0
#define MULTIPLE 1

FILE *inputfd;
char token[128];

typedef struct {
    char *token;
    int type;
    union {
	int *vint;
	int *vfloat;
	char *vstring;
	void (*fetch)();
    } value;
} equation;

typedef struct {
    char *token;
    int type;
    equation *equs;
    int nequs;
    int found;
    void (*init)(); 
} section;


extern int basevar_update_interval;
extern int robot_display_interval;
extern int tcx_query_interval;

extern float sonar_angle;
extern float sonar_offset;
extern float sonar_range;
extern float sonar_infinity;
extern float sonar_malfunc_rate;
extern float sonar_zpos;
extern int   rays_per_sonar; 

extern int   no_of_lasers;
extern int   no_of_readings;
extern float laser_resolution;
extern float laser_range;
extern float laser_offset;
extern float laser_zpos;
extern int laser_update_interval;

void init_surf();

#define NO_SONAR_EQUS 8
equation sonar_equs[NO_SONAR_EQUS] =
{
  {"sonars",       INTEGER, (void *) &(bRobot.sonar_cols[0])},
  {"malfunc_rate", FLOAT,   (void *) &sonar_malfunc_rate},
  {"angle",        FLOAT,   (void *) &sonar_angle},
  {"offset",       FLOAT,   (void *) &sonar_offset},
  {"range",        FLOAT,   (void *) &sonar_range},
  {"infinity",     FLOAT,   (void *) &sonar_infinity},
  {"height",       FLOAT,   (void *) &sonar_zpos},
  {"rays",         INTEGER, (void *) &rays_per_sonar}
};
#define NO_LASER_EQUS 7
equation laser_equs[NO_LASER_EQUS] =
{
  {"lasers",     INTEGER, (void *) &no_of_lasers},
  {"readings",   INTEGER, (void *) &no_of_readings},
  {"resolution", FLOAT,   (void *) &laser_resolution},
  {"range",      FLOAT,   (void *) &laser_range},
  {"offset",     FLOAT,   (void *) &laser_offset},
  {"height",     FLOAT,   (void *) &laser_zpos},  
  {"update_interval",     INTEGER, (void *) &laser_update_interval}
};

char surf_name[40];
char surf_color[40];
float surf_min_angle;
float surf_max_angle;
float surf_angle_range;

#define NO_SURFACE_EQUS 5
equation surface_equs[NO_SURFACE_EQUS] =
{
  {"name",        STRING, (void *) &surf_name},
  {"color",       STRING, (void *) &surf_color},
  {"min_angle",   FLOAT,  (void *) &surf_min_angle},
  {"max_angle",   FLOAT,  (void *) &surf_max_angle},
  {"angle_range", FLOAT,  (void *) &surf_angle_range}
};
#define NO_ROBOT_EQUS 3
equation robot_equs[NO_ROBOT_EQUS] =
{
  {"base_update_interval", INTEGER, (void *) &basevar_update_interval},
  {"robot_display_interval", INTEGER, (void *) &robot_display_interval},
  {"tcx_query_interval", INTEGER, (void *) &tcx_query_interval}    
};
#define NO_SECTIONS 4
section main_sections[NO_SECTIONS] =
{
  {"robot",   UNIQUE,   robot_equs,   NO_ROBOT_EQUS,   0, NULL},  
  {"sonar",   UNIQUE,   sonar_equs,   NO_SONAR_EQUS,   0, NULL},
  {"laser",   UNIQUE,   laser_equs,   NO_LASER_EQUS,   0, NULL},
  {"surface", MULTIPLE, surface_equs, NO_SURFACE_EQUS, 0, (void *) init_surf}
};

void
init_surf()
{
    new_surf_entry(surf_name, surf_color, surf_min_angle, surf_max_angle,
		   surf_angle_range);
}

void
skip_line()
{
  char c;
  do {
    fscanf(inputfd, "%c", &c);
  } while( c != '\n' && !feof(inputfd));
}

int
next_token()
{
 cont:
  fscanf(inputfd, "%s", token);
  if(feof(inputfd)) return 0;
  if(token[0] == '#') {
    skip_line();
    goto cont;
  }
  return 1;
}

int
section_ini(char *name, equation *equs, int n_equs)
{
  int i, found;
  next_token();
  while( token[0] != '}') {
    found = 0;
    for(i = 0; i < n_equs; i++) {
      if(strcmp(token, equs[i].token) == 0) {
	found = 1;
	break;
      }
    }
    if(found) {
      next_token();
      if(token[0] == '}') {
	fprintf(stderr, "missing value in section %s\n", name);
	return 0;
      }
      else {
	switch(equs[i].type) {
	case INTEGER:
	  *equs[i].value.vint = atoi(token);
	  break;
	case FLOAT:
	  sscanf(token, "%f", equs[i].value.vfloat);
	  break;
	case STRING:
	    strcpy(equs[i].value.vstring, token);
	    break;
	case FUNC:
	  equs[i].value.fetch();
	  break;
	}
      }
    }
    next_token();
  }
  return 1;
}

void
read_sections(section *sec, int no_sections)
{
  int i, found;
  while(1) {
    next_token();
    if(feof(inputfd)) break;
    if(strcmp(token, "section") == 0) {
      next_token();
      found = 0;
      for(i = 0; i < no_sections; i++) {
	if(strcmp(token, sec[i].token) == 0) {
	  found = 1;
	  break;
	}
      }
      if(found) {
	next_token();
	if(token[0] != '{') {
	  fprintf(stderr, "missing open brace in section %s\n", sec[i].token);
	  exit(0);
	}
	if(section_ini(sec[i].token, sec[i].equs, sec[i].nequs)) {
	    if(sec[i].init) sec[i].init();
	    sec[i].found++;
	}
      }
      else {
	fprintf(stderr, "Unknown section name %s\n", token);
	exit(0);
      }
    }
    else {
      fprintf(stderr, "Not a keyword %s, expected \"section\"\n",token);
      exit(0);
    }
  }
}

check_initialization(section *sec, int no_secs)
{
    int i;
    for (i = 0; i < no_secs; i++) {
	if(sec[i].found == 0) {
	    fprintf(stderr, "Section %s not defined in initfile, using default values\n", sec[i].token);
	}
	if(sec[i].type == UNIQUE && sec[i].found > 1) {
	    fprintf(stderr, "Unique section %s multiple defined in initfile, taking last definition\n",sec[i].token);		
	}
    }
}

void
read_inifile(char *fn)
{
  char fname1[128];
  char fname2[128];  
  fname1[0] = 0;
  fname2[0] = 0;

  if(!(inputfd = fopen(fn, "r"))) {
    if(! (inputfd = fopen("simulator2.ini", "r")) ) {
      sprintf(fname1,"%s/bee/etc/simulator2.ini",getenv("HOME"));
      inputfd = fopen(fname1, "r");
      if(!inputfd && getenv("RHINOHOME")) {
	sprintf(fname2,"%s/bee/etc/simulator2.ini",getenv("RHINOHOME"));
	inputfd = fopen(fname2, "r");
      }
      else {
	printf("reading inifile %s\n",fname1);
      }
    }
    else {
      printf("reading inifile ./simulator2.ini\n");
    }
  }
  else {
    printf("reading inifile %s\n",fn);
  }
  if(!inputfd) {
    fprintf(stderr,
	    "Could not read any inifile, I tried\n%s\n%s\n%s\nUsing defaults now\n",fn,fname1,fname2);
    return;
  }
  read_sections(main_sections,NO_SECTIONS);
  check_initialization(main_sections, NO_SECTIONS);
  fclose(inputfd);
}

