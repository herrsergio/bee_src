
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
#include "bUtils.h"

int
main(int argc, char *argv[])
{
  int ii;
  struct bParamList *parameters = NULL;

  parameters=bParametersAddEntry(parameters, "",
				 "TCXHOST", "localhost");

  parameters=bParametersAddEntry(parameters, "",
				 "fork", "yes");

  parameters = bParametersAddFile(parameters, "etc/beeSoft.ini");
  parameters = bParametersAddArray(parameters, "", argc, argv);
  bParametersFillParams(parameters);

  /*
   * Print (not quite) everything (yet) for debugging purposes
   */

  fprintf(stderr, "TCXHOST = [%s]\n", bRobot.TCXHOST);
  fprintf(stderr, "fork = [%d]\n", bRobot.fork);

  fprintf(stderr, "base_type = [%s]\n", bRobot.base_type);
  fprintf(stderr, "base_host = [%s]\n", bRobot.base_host);
  fprintf(stderr, "base_dev  = [%s]\n", bRobot.base_dev);

  fprintf(stderr, "base_bps      = [%d]\n", bRobot.base_bps);
  fprintf(stderr, "base_radius   = [%f]\n", bRobot.base_radius);
  fprintf(stderr, "base_encPerCm = [%f]\n", bRobot.base_encPerCm);
  fprintf(stderr, "base_posPerCm = [%f]\n", bRobot.base_posPerCm);
  fprintf(stderr, "base_rotBackwards = [%d]\n", bRobot.base_rotBackwards);

  fprintf(stderr, "enclosure_type   = [%s]\n", bRobot.enclosure_type);
  fprintf(stderr, "enclosure_radius = [%f]\n", bRobot.enclosure_radius);

  fprintf(stderr, "sonar_type = [%s]\n", bRobot.sonar_type);
  fprintf(stderr, "sonar_dev  = [%s]\n", bRobot.sonar_dev);
  fprintf(stderr, "sonar_rows = [%d]\n", bRobot.sonar_rows);
  fprintf(stderr, "sonar_cols = { %d", bRobot.sonar_cols[0]);
  for (ii=1; ii<bRobot.sonar_rows; ii++) {
    fprintf(stderr, ", %d", bRobot.sonar_cols[ii]);
  }
  fprintf(stderr, " }\n");

  fprintf(stderr, "tactile_type = [%s]\n", bRobot.tactile_type);
  fprintf(stderr, "tactile_dev  = [%s]\n", bRobot.tactile_dev);
  fprintf(stderr, "tactile_rows = [%d]\n", bRobot.tactile_rows);
  fprintf(stderr, "tactile_cols = { %d", bRobot.tactile_cols[0]);
  for (ii=1; ii<bRobot.tactile_rows; ii++) {
    fprintf(stderr, ", %d", bRobot.tactile_cols[ii]);
  }
  fprintf(stderr, " }\n");

  fprintf(stderr, "ir_type = [%s]\n", bRobot.ir_type);
  fprintf(stderr, "ir_dev  = [%s]\n", bRobot.ir_dev);
  fprintf(stderr, "ir_rows = [%d]\n", bRobot.ir_rows);
  fprintf(stderr, "ir_cols = { %d", bRobot.ir_cols[0]);
  for (ii=1; ii<bRobot.ir_rows; ii++) {
    fprintf(stderr, ", %d", bRobot.ir_cols[ii]);
  }
  fprintf(stderr, " }\n");

  
  if (bRobot.fork) {
    bDaemonize("bFileTest.log");
  }

  exit(0);
}
