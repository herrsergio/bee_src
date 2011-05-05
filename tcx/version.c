
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



/*****************************************************************************
 * INCLUDES
 *****************************************************************************/

#include "stdio.h"
#include "ctype.h"
#include "signal.h"
#include "sys/time.h"
#include "sys/types.h"
#include "sys/socket.h"
#include "errno.h"

#include "tcx.h"
#include "tcxP.h"
#include "global.h"

#include "beeSoftVersion.h"


extern int beeSoftMaj;
extern int beeSoftMin;
extern int beeSoftRobotType;

static int version_major_error = 0;
static int version_warning = 0;
static int first_check = 1;

static int  local_major = -1;
static int  local_minor = -1;
static int  local_robot_type = -1;
static char local_date[80];

void
check_version_number(int major, int minor, int robot_type, char *date, 
		     char *reference_text, 
		     int final_check)
{
  static int external = 1;
  static int local_printed = 0;
  int header_printed = 0;

  /*
   * First check: verify the external version number
   */

#ifdef UNIBONN
  fprintf( stderr, "No version control wanted!\n");
  return;
#endif
  
  if (first_check){

    /*
     * First check: let's copy the parameters: local version
     */

    local_major = major;
    local_minor = minor;
    local_robot_type = robot_type;
    strcpy(local_date, date);

    /*
     * First check: now let's check the tcxServer version number
     */

    if (beeSoftMaj == -1 || beeSoftMin == -1 || beeSoftRobotType == -1){
      fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "WARNING: TCX not yet initialized. Unable to verify external version number.\n");
      external = 0;
      version_warning = 1;
    }
    
    if (external && beeSoftMaj != local_major){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "ERROR: Major version number mismatch (tcxServer).\n");
      version_major_error = 1;
    }
    
    if (external && beeSoftRobotType != local_robot_type){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "ERROR: Inconsistent robot type (tcxServer).\n");
      version_major_error = 1;
    }
    
    if (external && beeSoftMin != local_minor){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "WARNING: Minor version number mismatch (tcxServer).\n");
      version_warning = 1;
    }

    /*
     * First check: now let's check the libtcx.a version number
     */
    
    if (BEESOFT_VERSION_MAJOR != local_major){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "ERROR: Major version number mismatch (tcxServer).\n");
      version_major_error = 1;
    }
    
    if (BEESOFT_VERSION_ROBOT_TYPE != local_robot_type){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "ERROR: Inconsistent robot type (tcxServer).\n");
      version_major_error = 1;
    }
    
    if (BEESOFT_VERSION_MINOR != local_minor){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "WARNING: Minor version number mismatch (tcxServer).\n");
      version_warning = 1;
    }
    
    

  }
  /*
   * not the first check
   */

  else{
    /*
     * All other checks: now let's check the corresponding version number
     */
    
    if (major != local_major){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "ERROR: Major version number mismatch (%s).\n",
	      reference_text);
      version_major_error = 1;
    }
    
    if (robot_type != local_robot_type){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "ERROR: Inconsistent robot type (%s).\n",
	      reference_text);
      version_major_error = 1;
    }
    
    if (minor != local_minor){
      if ( !header_printed )
	fprintf(stderr, "\n*** BeeSoft Version Control Report ***\n");
      header_printed=1;
      fprintf(stderr, "WARNING: Minor version number mismatch (%s).\n",
	      reference_text);
      version_warning = 1;
    }
    
  }

  /*
   * print the version numbers, if necessary
   */

  if (version_major_error || version_warning){
    if (!local_printed){
      fprintf(stderr, "\tVersion number of this program: %d.%d.B%d %s\n",
	      local_major, local_minor, local_robot_type, local_date);
      fprintf(stderr, "\tVersion number of tcxlib.a:     %d.%d.B%d %s\n",
	      BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
	      BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE);
      if (external)
	fprintf(stderr, "\tVersion number of tcxServer:    %d.%d.B%d\n",
		beeSoftMaj, beeSoftMin, beeSoftRobotType);
      else
	fprintf(stderr, "\tVersion number of tcxServer:    (could not be verified)\n\n");
      
      local_printed = 1;
    }
    if (!first_check)
      fprintf(stderr, "\tVersion number of %s:     %d.%d.B%d %s\n",
	      reference_text, major, minor, robot_type, date);
  }


  /*
   * if final check: print last report / exit
   */
  
  if (final_check){
    if (!version_major_error && !version_warning)
      fprintf(stderr, "Version number %d.%d/B%d (%s) successfully verified.\n",
	      local_major, local_minor, local_robot_type, local_date);


    fprintf(stderr, "\n");
    if (version_major_error)
      exit (-1);
  }


  first_check = 0;

  return;
}
