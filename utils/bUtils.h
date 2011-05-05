
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
#ifndef B_UTILS_H
#define B_UTILS_H

#include <stdio.h>
#include <termios.h>
#include <bNetwork.h>

/* These are used for statically allocated arrays */

#define B_MAX_SENSOR_ROWS 10
#define B_MAX_SENSOR_COLS 50

struct bParamList;

struct bRobotParams {
  const char *TCXHOST;
  int         fork;

  const char *base_type;
  const char *base_host;
  const char *base_dev;
  speed_t     base_bps;
  float       base_radius;
  float       base_encPerCm;
  float       base_posPerCm;
  int         base_rotBackwards;
  int         base_hasIndex;

  const char *enclosure_type;
  float       enclosure_radius;

  const char *sonar_type;
  const char *sonar_dev;
  int         sonar_rows;
  int         sonar_cols[10];

  const char *tactile_type;
  const char *tactile_dev;
  int         tactile_rows;
  int         tactile_cols[10];

  const char *ir_type;
  const char *ir_dev;
  int         ir_rows;
  int         ir_cols[10];

  const char *volt_type;
  float       volt_warn;
  float       volt_panic;

  const char *speech_type;
  const char *speech_host;
  const char *speech_dev;
  speed_t     speech_bps;

  const char *pantilt_type;
  const char *pantilt_host;
  const char *pantilt_dev;
  speed_t     pantilt_bps;

  const char *compass_type;
  const char *compass_host;
  const char *compass_dev;
  speed_t     compass_bps;

  const char *laser_type;
  const char *laser_host;
  const char *laser_dev;
  speed_t     laser_bps;

  const char *arm_type;
  const char *arm_mast_host;
  const char *arm_mast_dev;
  speed_t     arm_mast_bps;
  int         arm_mast_stow;
  int         arm_mast_boomTravel;
  const char *arm_grip_host;
  const char *arm_grip_dev;
  speed_t     arm_grip_bps;
};

extern struct bRobotParams bRobot;

/************************************************************
 * 
 * NOTE:  functions with names that end in 'M' return 
 *        pointers to malloc()'ed memory that must be managed
 *        and free()'d by the client function/program.
 *
 ************************************************************/


#ifdef __cplusplus
extern "C" {
#endif
  
  /************************************************************
   *
   * bFindFileM()
   *
   * will search in standard locations for a config file
   * and return char * to the file name.
   *
   * ! NOTE: The returned char * is malloc()'ed by bFindFileM() and 
   *          must be free()'ed by the calling function/program.
   *
   *
   * ./file
   * ./dir/file
   * ../dir/file
   * ../../dir/file
   * ../../../dir/file
   * ~bee/dir/file
   *
   ************************************************************/
  
  extern char *
    bFindFileM(const char *name);
  
  /************************************************************
   *
   * bFindDirM()
   *
   * will search in standard locations for a directory
   * and return char * to the directory name. 
   *
   * ! NOTE: The returned char * is malloc()'ed by bFindDir() and 
   *          must be free()'ed by the calling function/program.
   *
   * ./dir
   * ../dir
   * ../../dir
   * ../../../dir
   * ~bee/dir
   * ./
   *
   * Note: Policy and permissions checks need review
   *         and implementation - tds
   *
   ************************************************************/
  
  extern char *
    bFindDirM(const char *dirName);
  
  extern void
    bParametersFreeList(struct bParamList * list);
  
  extern struct bParamList *
    bParametersAddEntry(struct bParamList * list,
			const char *prefix,
			const char *name,
			const char *value);
  
  extern struct bParamList *
    bParametersAddEnv(struct bParamList *list,
		      const char *prefix,
		      const char *env);
  
  /************************************************************
   *
   * struct bParamList *
   * bParametersAddArray(struct bParamList *list, const char *prefix,
   *                     int argc, const char *argv[])
   *
   * This function is used to add command line args or any
   * other array of args to the parameter list.
   *
   * The array terminates at argc elements or when argv[i]==NULL,
   * which ever is first.
   *
   *  argv[i]                parameter file
   * ------------------------------------------------------
   * -parameter=value        "parameter" "value"
   *
   * The -parameter=value format is used instead of -parameter value
   * because the latter either doesn't allow a value without a parameter
   * or doesn't allow negative valued parameters.
   *
   * items that don't begin with "-" and contain an "=" are ignored.
   *
   ************************************************************/
  
  extern struct bParamList *
    bParametersAddArray(struct bParamList *list,
			const char *prefix,
			int argc, char *const * argv);
  
  /************************************************************
   *
   * bParametersAddFile(struct bParamList *list, FILE *fp)
   *
   * If list is NULL create list.
   *
   * If list is !NULL add to list.
   *
   * If fp==NULL or the file is empty or the file does not 
   *   begin with "[bParamFile]" or unable to malloc a block
   *   of memory return NULL.
   *
   * If returning NULL and list was NULL then free the created
   *   list before returning.  The only case of lost data that
   *   had been inserted in the list will be a malloc failure
   *   when starting with list==NULL.  All other failures will
   *   occur before creating the list.
   *
   * File syntax errors will be returned as "ERROR" entries
   *   in the list.
   *
   ************************************************************/

  extern struct bParamList *
    bParametersAddFile(struct bParamList * list,
		       const char *filename);

  extern const char *
    bParametersGetParam(const struct bParamList * list,
			const char *prefix,
			const char *name);

  extern int
    bDaemonize(const char *logfile);

  /**********************************************************************
   *
   * char * bGetArrayElementM(const char *arrayStr, int dimc, const int *dim)
   *
   * ! NOTE: The returned char * is malloc()'ed by bGetArrayElement() and 
   *          must be free()'ed by the calling function/program.
   *
   **********************************************************************/

  extern char *
    bGetArrayElementM(const char *arrayStr,
		      int dimc,
		      const int *userIndex);

  extern speed_t
    bStrToBaud(const char *baudStr);

  /************************************************************
   *
   * int bStrToTruth(const char *str)
   *
   * Interprets a string to be either true (1) or false(0)
   *
   * It looks for yes, y, true and t for true.
   * It looks for no, n, false and f for false.
   *   All of these tests are case insensitive.
   * 
   * If none of the first tests are found the string is converted
   * using strtol() and if the result is 0, 0 is returned
   * otherwise it returns 1.
   *
   ************************************************************/

  extern int bStrToTruth(const char *str);

  extern void bParametersFillParams(struct bParamList *list);

#ifdef __cplusplus
}
#endif

#endif /* B_UTILS_H */
