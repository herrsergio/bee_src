
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


#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <pwd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <errno.h>

#include "bUtils.h"

#define B_MAX_FILE_NAME_LEN 255
#define B_MAX_PARAM_NAME_LEN 32
#define B_MAX_PARAM_VALUE_LEN 256
#define B_MAX_PARAM_LINE_LEN B_MAX_PARAM_NAME_LEN+B_MAX_PARAM_VALUE_LEN+10

struct bParamList {
  char name[B_MAX_FILE_NAME_LEN];
  char value[B_MAX_PARAM_VALUE_LEN];
  struct bParamList * next;
};

struct bRobotParams bRobot;

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


char *
bFindFileM(const char *name)
{
  char *dirName;
  char *fileName;
  char * name2;
  char * fullName;
  struct passwd *passwdent;
  struct stat statBuf;

  fullName = malloc(B_MAX_FILE_NAME_LEN+1);

  if (!fullName) {
    return(NULL);
  }

  name2 = malloc(B_MAX_FILE_NAME_LEN+1);

  if (!name2) {
    free(fullName);
    return(NULL);
  }

  strncpy(name2, name, B_MAX_FILE_NAME_LEN);

  fileName = name2 + strlen(name2);

  while ((fileName >= name2) && (*fileName != '/')) {
    fileName--;
  }
  fileName++;

  if (fileName == name2) {
    dirName = ".";
  }
  else {
    dirName = name2;
    *(fileName - 1) = '\0';
  }

#ifdef VERBOSE
  fprintf(stderr, "dirName = [%s]  fileName = [%s]\n", dirName, fileName);
#endif

  /*
   * ./fileName
   */

  memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
  strncat(fullName, "./", B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, fileName, B_MAX_FILE_NAME_LEN-strlen(fullName));
#ifdef VERBOSE
  fprintf (stderr, "Trying to open %s ...\n", fullName);
#endif
  if (stat(fullName, &statBuf) == 0) {
    free( name2 );
    return(fullName);
  }

  /*
   * ./dirName/fileName
   */

  memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
  strncat(fullName, "./", B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, dirName, B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, fileName, B_MAX_FILE_NAME_LEN-strlen(fullName));
#ifdef VERBOSE
  fprintf (stderr, "Trying to open %s ...\n", fullName);
#endif
  if (stat(fullName, &statBuf) == 0) {
    free( name2 );
    return(fullName);
  }

  /*
   * ../dirName/fileName
   */

  memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
  strncat(fullName, "../", B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, dirName, B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, fileName, B_MAX_FILE_NAME_LEN-strlen(fullName));
#ifdef VERBOSE
  fprintf (stderr, "Trying to open %s ...\n", fullName);
#endif
  if (stat(fullName, &statBuf) == 0) {
    free( name2 );
    return(fullName);
  }

  /*
   * ../../dirName/fileName
   */

  memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
  strncat(fullName, "../../", B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, dirName, B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, fileName, B_MAX_FILE_NAME_LEN-strlen(fullName));
#ifdef VERBOSE
  fprintf (stderr, "Trying to open %s ...\n", fullName);
#endif
  if (stat(fullName, &statBuf) == 0) {
    free( name2 );
    return(fullName);
  }

  /*
   * ../../../dirName/fileName
   */

  memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
  strncat(fullName, "../../../", B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, dirName, B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, fileName, B_MAX_FILE_NAME_LEN-strlen(fullName));
#ifdef VERBOSE
  fprintf (stderr, "Trying to open %s ...\n", fullName);
#endif
  if (stat(fullName, &statBuf) == 0) {
    free( name2 );
    return(fullName);
  }

  
  /*
   * $HOME/bee/dirName/fileName
   */
  
  if (getenv("HOME")) {

    memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
    sprintf(fullName, "%s/bee/%s/%s",
	     getenv("HOME"), dirName, fileName);
#ifdef VERBOSE
    fprintf (stderr, "Trying to open %s ...\n", fullName);
#endif
    if (stat(fullName, &statBuf) == 0) {
      free( name2 );
      return(fullName);
    }
    
    
    /*
     * $HOME/dirName/fileName
     */
    
    memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
    sprintf(fullName, "%s/%s/%s",
	     getenv("HOME"), dirName, fileName);
#ifdef VERBOSE
    fprintf (stderr, "Trying to open %s ...\n", fullName);
#endif
    if (stat(fullName, &statBuf) == 0) {
      free( name2 );
      return(fullName);
    }
    
    /*
     * $HOME/fileName
     */
    
    memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
    sprintf(fullName, "%s/%s",
	     getenv("HOME"), fileName);
#ifdef VERBOSE
    fprintf (stderr, "Trying to open %s ...\n", fullName);
#endif
    if (stat(fullName, &statBuf) == 0) {
      free( name2 );
      return(fullName);
    }
    
  }
    
  /*
   * $BEEHOME/dirName/fileName
   */
  
  if (getenv("BEEHOME")) {
      
    memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
    sprintf(fullName, "%s/%s/%s",
	     getenv("BEEHOME"), dirName, fileName);
#ifdef VERBOSE
    fprintf (stderr, "Trying to open %s ...\n", fullName);
#endif
    if (stat(fullName, &statBuf) == 0) {
      free( name2 );
      return(fullName);
    }
      
  }

  /*
   * ~bee/dirName/fileName
   */
  
  memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
  passwdent = getpwnam("bee");
  
  if (passwdent) {
    strncat(fullName, passwdent->pw_dir,
	    B_MAX_FILE_NAME_LEN-strlen(fullName));
    strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
    strncat(fullName, dirName, B_MAX_FILE_NAME_LEN-strlen(fullName));
    strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
    strncat(fullName, fileName, B_MAX_FILE_NAME_LEN-strlen(fullName));
#ifdef VERBOSE
    fprintf(stderr, "Trying to open %s ...\n", fullName);
#endif
    if (stat(fullName, &statBuf) == 0) {
      free( name2 );
      return(fullName);
    }
  }
  
  /*
   * $RHINOHOME/bee/dirName/fileName
   */

  if (getenv("RHINOHOME")) {

    memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
    sprintf(fullName, "%s/bee/%s/%s",
	     getenv("RHINOHOME"), dirName, fileName);
#ifdef VERBOSE
    fprintf (stderr, "Trying to open %s ...\n", fullName);
#endif
    if (stat(fullName, &statBuf) == 0) {
      free( name2 );
      return(fullName);
    }

    /*
     * $RHINOHOME/dirName/fileName
     */

    memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
    sprintf(fullName, "%s/%s/%s",
	     getenv("RHINOHOME"), dirName, fileName);
#ifdef VERBOSE
    fprintf (stderr, "Trying to open %s ...\n", fullName);
#endif
    if (stat(fullName, &statBuf) == 0) {
      free( name2 );
      return(fullName);
    }

    /*
     * $RHINOHOME/fileName
     */

    memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
    sprintf(fullName, "%s/%s",
	     getenv("RHINOHOME"), fileName);
#ifdef VERBOSE
    fprintf (stderr, "Trying to open %s ...\n", fullName);
#endif
    if (stat(fullName, &statBuf) == 0) {
      free( name2 );
      return(fullName);
    }

  }

  fprintf(stderr, "File %s not found.\n", fileName);
  free( name2 );
  free( fullName );
  return(NULL);
}

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

char *
bFindDirM(const char *dirName)
{
  char * fullName;
  struct passwd *passwdent;
  struct stat statBuf;

  fullName = malloc(B_MAX_FILE_NAME_LEN+1);

  if (!fullName) {
    return(fullName);
  }

  /*
   * ./dirName/
   */

  memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
  strncat(fullName, "./", B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, dirName, B_MAX_FILE_NAME_LEN-strlen(fullName));
#ifdef VERBOSE
  fprintf (stderr, "Trying to open %s/ ...\n", fullName);
#endif
  if (stat(fullName, &statBuf) == 0) {
    if (S_ISDIR(statBuf.st_mode)) {
      strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
      return(fullName);
    }
  }

  /*
   * ../dirName/
   */

  memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
  strncat(fullName, "../", B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, dirName, B_MAX_FILE_NAME_LEN-strlen(fullName));
#ifdef VERBOSE
  fprintf (stderr, "Trying to open %s/ ...\n", fullName);
#endif
  if (stat(fullName, &statBuf) == 0) {
    if (S_ISDIR(statBuf.st_mode)) {
      strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
      return(fullName);
    }
  }

  /*
   * ../../dirName/
   */

  memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
  strncat(fullName, "../../", B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, dirName, B_MAX_FILE_NAME_LEN-strlen(fullName));
#ifdef VERBOSE
  fprintf (stderr, "Trying to open %s/ ...\n", fullName);
#endif
  if (stat(fullName, &statBuf) == 0) {
    if (S_ISDIR(statBuf.st_mode)) {
      strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
      return(fullName);
    }
  }

  /*
   * ../../../dirName/
   */

  memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
  strncat(fullName, "../../../", B_MAX_FILE_NAME_LEN-strlen(fullName));
  strncat(fullName, dirName, B_MAX_FILE_NAME_LEN-strlen(fullName));
#ifdef VERBOSE
  fprintf (stderr, "Trying to open %s/ ...\n", fullName);
#endif
  if (stat(fullName, &statBuf) == 0) {
    if (S_ISDIR(statBuf.st_mode)) {
      strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
      return(fullName);
    }
  }

  /*
   * $HOME/bee/dirName/
   */

  if (getenv("HOME")) {

    memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
    sprintf(fullName, "%s/bee/%s",
	     getenv("HOME"), dirName);
#ifdef VERBOSE
    fprintf (stderr, "Trying to open %s/ ...\n", fullName);
#endif
    if (stat(fullName, &statBuf) == 0) {
      if (S_ISDIR(statBuf.st_mode)) {
	strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
	return(fullName);
      }
    }

    /*
     * $HOME/dirName/
     */

    memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
    sprintf(fullName, "%s/%s",
	     getenv("HOME"), dirName);
#ifdef VERBOSE
    fprintf (stderr, "Trying to open %s/ ...\n", fullName);
#endif
    if (stat(fullName, &statBuf) == 0) {
      if (S_ISDIR(statBuf.st_mode)) {
	strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
	return(fullName);
      }
    }

  }

  /*
   * $BEEHOME/dirName/
   */

  if (getenv("BEEHOME")) {

    memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
    sprintf(fullName, "%s/%s",
	     getenv("BEEHOME"), dirName);
#ifdef VERBOSE
    fprintf (stderr, "Trying to open %s/ ...\n", fullName);
#endif
    if (stat(fullName, &statBuf) == 0) {
      if (S_ISDIR(statBuf.st_mode)) {
	strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
	return(fullName);
      }
    }

  }

  /*
   * ~bee/dirName/
   */

  memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
  passwdent = getpwnam("bee");
  
  if (passwdent) {
    strncat(fullName, passwdent->pw_dir,
	    B_MAX_FILE_NAME_LEN-strlen(fullName));
    strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
    strncat(fullName, dirName, B_MAX_FILE_NAME_LEN-strlen(fullName));
#ifdef VERBOSE
    fprintf(stderr, "Trying to open %s/ ...\n", fullName);
#endif
    if (stat(fullName, &statBuf) == 0) {
      if (S_ISDIR(statBuf.st_mode)) {
	strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
	return(fullName);
      }
    }
  }
  
  /*
   * $RHINOHOME/bee/dirName/
   */

  if (getenv("RHINOHOME")) {

    memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
    sprintf(fullName, "%s/bee/%s",
	     getenv("RHINOHOME"), dirName);
#ifdef VERBOSE
    fprintf (stderr, "Trying to open %s/ ...\n", fullName);
#endif
    if (stat(fullName, &statBuf) == 0) {
      if (S_ISDIR(statBuf.st_mode)) {
	strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
	return(fullName);
      }
    }

    /*
     * $RHINOHOME/dirName/
     */

    memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
    sprintf(fullName, "%s/%s",
	     getenv("RHINOHOME"), dirName);
#ifdef VERBOSE
    fprintf (stderr, "Trying to open %s/ ...\n", fullName);
#endif
    if (stat(fullName, &statBuf) == 0) {
      if (S_ISDIR(statBuf.st_mode)) {
	strncat(fullName, "/", B_MAX_FILE_NAME_LEN-strlen(fullName));
	return(fullName);
      }
    }

  }

  /*
   * Give up and return ./
   */

  memset(fullName, 0, B_MAX_FILE_NAME_LEN+1);
  strncat(fullName, "./", B_MAX_FILE_NAME_LEN-strlen(fullName));
  fprintf(stderr, "Directory %s not found.\n", dirName);
  return(fullName);
}

/************************************************************
 *
 * bParametersFreeList(struct bParamList *list)
 *
 ************************************************************/

void
bParametersFreeList(struct bParamList *list)
{
  struct bParamList *next;

#ifdef VERBOSE
  fprintf(stderr, "Entering %s\n", __FUNCTION__);
#endif

  while (list) {
    next = list->next;
    free(list);
    list = next;
  }

  return;
}

/************************************************************
 *
 * bParametersAddEntry(struct bParamList *list, const char *prefix
 *		       const char *name, const char *value)
 *
 ************************************************************/

struct bParamList *
bParametersAddEntry(struct bParamList *list, const char *prefix,
		    const char *name, const char *value)
{
  struct bParamList * prev=NULL;
  struct bParamList * next;

  next = malloc(sizeof(*next));
  
  if (next) {
    memset(next, 0, sizeof(*next));
  }
  else {
    fprintf(stderr, "Unable to allocate memory\n");
    return(NULL);
  }
    
  if (list) {
    prev = list;
    while (prev->next) {
      prev=prev->next;
    }
    prev->next = next;
  }
  else {
    list = next;
  }
  

  if (prefix && strlen(prefix)) {
    strncat(next->name, prefix, B_MAX_PARAM_NAME_LEN);
    strncat(next->name, ".", B_MAX_PARAM_NAME_LEN);
  }

  if (name) {
    strncat(next->name, name, B_MAX_PARAM_NAME_LEN);
  }
  else {
    strncat(next->name, "NULL", B_MAX_PARAM_NAME_LEN);
  }

  if (value) {
    strncpy(next->value, value, B_MAX_PARAM_VALUE_LEN);
  }
  else {
    strncpy(next->value, "NULL", B_MAX_PARAM_VALUE_LEN);
  }
#if 0
  fprintf(stderr, "%s(): name=[%s] value=[%s]\n",
	  __FUNCTION__, next->name, next->value);
#endif
  return(list);
}


/************************************************************
 *
 * struct bParamList *
 * bParametersAddEnv(struct bParamlist *list, const char *prefix,
 *                   const char *env)
 *
 ************************************************************/

struct bParamList *
bParametersAddEnv(struct bParamList *list, const char *prefix,
		  const char *env)
{
  const char *value;

  if (!env) {
    return(list);
  }

  value = getenv(env);

  if (!value) {
    return(list);
  }

  list = bParametersAddEntry(list, prefix, env, value);

  return(list);
}


/************************************************************
 *
 * struct bParamList *
 * bParametersAddArray(struct bParamList *list, const char *prefix,
 *                     int argc, char * const *argv)
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

struct bParamList *
bParametersAddArray(struct bParamList *list, const char *prefix,
		    int argc, char * const * argv)
{
  int ii;
  char *value;
  char parameter[B_MAX_PARAM_NAME_LEN];
  
  if (argc<1) {
    return(list);
  }

  ii = 1;
  argc--;

  while ((argc--) && (argv[ii])) {
    if (argv[ii][0] != '-') {
      ii++;
      continue;
    }

    strncpy (parameter, &argv[ii][1], B_MAX_PARAM_NAME_LEN);
    value = strchr(parameter, '=');

    if (value) {
      *value='\0';
    }

    value = strchr(argv[ii], '=');

    if (!value) {
      ii++;
      continue;
    }

    value++;
    list = bParametersAddEntry(list, prefix, parameter, value);
    
    ii++;
  }

  return(list);
}


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

struct bParamList *
bParametersAddFile(struct bParamList *list, const char *file)
{
  FILE * fp = NULL;
  char * fileName = NULL;
  struct bParamList *prev=NULL;
  struct bParamList *old;
  char line[B_MAX_PARAM_LINE_LEN];
  char prefix[B_MAX_PARAM_NAME_LEN];
  char *linePtr;
  char *name;
  char *value;
  int lineCount=0;
  
#ifdef VERBOSE
  fprintf(stderr, "Entering %s\n", __FUNCTION__);
#endif

  prefix[0] = '\0';

  fileName = bFindFileM(file);
  fprintf(stderr, "Found [%s]\n", fileName);
  fp = fopen(fileName, "r");
  free(fileName);

  old = list;

  if (!fp) {
    fprintf(stderr, "fp==NULL\n");
    fclose(fp);
    return(NULL);
  }
  
  if (list) {
    prev = list;
    while (prev->next) {
      prev=prev->next;
    }
  }
  
  if (!fgets(line, B_MAX_PARAM_LINE_LEN, fp)) {
    fprintf(stderr, "unable to read first line\n");
    fclose(fp);
    return(NULL);
  }
  
  lineCount++;

  if (strncmp("[bParamFile]", line, strlen("[bParamFile]"))) {
    fprintf(stderr, "First line does not appear to be '[bParamFile]'\n");
    fclose(fp);
    return(NULL);
  }
  
  while (fgets(line, B_MAX_PARAM_LINE_LEN, fp)) {
    lineCount++;
    linePtr = line;
    
    /*
     * skip leading white space
     */

    while (isspace(*linePtr)) {
      linePtr++;
    }
    
    /*
     * comment or blank line 
     */

    if ((*linePtr==';') || (*linePtr=='#') || (*linePtr=='\0')) {
      continue;
    }
    
    /*
     * group prefix
     */

    if (*linePtr=='[') {
      name = ++linePtr;

      while ((*linePtr != ';')  && (*linePtr != '#')  &&
	     (*linePtr != ' ')  && (*linePtr != '\t') &&
	     (*linePtr != '\n') && (*linePtr != '\r') &&
	     (*linePtr != ']')  && (*linePtr != '\0')) {
	linePtr++;
      }

      *linePtr = '\0';

      strncpy(prefix, name, B_MAX_PARAM_LINE_LEN-1);

      continue;
    }
      
    /*
     * get parameter name
     */

    name = linePtr;

    while ((*linePtr != ';')  && (*linePtr != '#')  &&
	   (*linePtr != ' ')  && (*linePtr != '\t') &&
	   (*linePtr != '\n') && (*linePtr != '\r') &&
	   (*linePtr != '\0')) {
      linePtr++;
    }

    /*
     * Imediate end of line or comment. No value given.
     */

    if (!isspace(*linePtr)) {
      *linePtr = '\0';
      value = linePtr;
      
      if (!(prev=bParametersAddEntry(prev, prefix, name, value))) {
	if (!old) {
	  bParametersFreeList(list);
	}
	fprintf(stderr, "unable to add parameter - 1\n");
	fclose(fp);
	return(NULL);
      }
      
      if (!list) {
	list = prev;
      }

      if (prev->next) {
	prev = prev->next;
      }
      
      continue;
    }


    /*
     * terminate name string and parse out value string
     */

    *linePtr++ = '\0';

    while (isspace(*linePtr)) {
      linePtr++;
    }
    
    value = linePtr;

    while ((*linePtr != ';')  && (*linePtr != '#')  &&
	   (*linePtr != '\n') && (*linePtr != '\r') &&
	   (*linePtr != '\0')) {
      linePtr++;
    }

    *linePtr-- = '\0';

    while(isspace(*linePtr)) {
      *linePtr-- = '\0';
    }

    if (!(prev=bParametersAddEntry(prev, prefix, name, value))) {
      if (!old) {
	bParametersFreeList(list);
      }
      fprintf(stderr, "unable to add parameter - 1\n");
      fclose(fp);
      return(NULL);
    }
      
    if (!list) {
      list = prev;
    }

    if (prev->next) {
      prev = prev->next;
    }

  }

  fclose(fp);
  return(list);
}

/************************************************************
 *
 * const char *
 * bParametersGetParam(const struct bParamList *list, const char *name)
 *
 ************************************************************/
const char *
bParametersGetParam(const struct bParamList *list, const char *prefix,
		    const char *name)
{
  char fullName[B_MAX_PARAM_VALUE_LEN+1];
  const struct bParamList * found = NULL;
  const char *paramName;

  if ((!list) || (!prefix) || (!name)) {
    return(NULL);
  }

  fullName[0] = '\0';

  if (prefix[0]) {
    strncat(fullName, prefix, B_MAX_PARAM_VALUE_LEN);
    strncat(fullName, ".", B_MAX_PARAM_VALUE_LEN);
  }

  strncat(fullName, name, B_MAX_PARAM_VALUE_LEN);

  while(list) {
    if (*(list->name)=='*') {
      paramName = fullName + strlen(fullName) - strlen(list->name) + 1;
      if (!strncasecmp(paramName, list->name + 1, B_MAX_PARAM_NAME_LEN)) {
	found = list;
      }
    }
    else {
      if (!strncasecmp(fullName, list->name, B_MAX_PARAM_NAME_LEN)) {
	found = list;
      }
    }
    list = list->next;
  }

  if (found) {
    return(found->value);
  }

  return(NULL);
}

/**********************************************************************
 *
 * int bDaemonize(const char *logfile)
 *
 **********************************************************************/

int
bDaemonize(const char *logfile)
{
  char * dirname;
  char filename[B_MAX_FILE_NAME_LEN];
  int retval;

  /* find log dir */

  dirname = bFindDirM("log");
  if (!dirname) {
    return(-1);
  }

  strncpy(filename, dirname, B_MAX_FILE_NAME_LEN);
  strncat(filename, logfile, B_MAX_FILE_NAME_LEN);

  /* fork */
  fprintf(stderr, "Forking and redirecting output to "
	  "%s.stdout and %s.stderr\n", filename, filename);

  switch(fork()) {
  case -1:
    perror("bfileUtils: ");
    free(dirname);
    return(-1);
    break;

  case 0:

    fclose(stdin);

    strncpy(filename, dirname, B_MAX_FILE_NAME_LEN);
    strncat(filename, logfile, B_MAX_FILE_NAME_LEN);
    strncat(filename, ".stdout", B_MAX_FILE_NAME_LEN);

    retval = unlink(filename);
    if (retval && (errno!=ENOENT)) {
      perror("bfileUtils: ");
      fprintf(stderr, "errno=%d ENOENT=%d\n", errno, ENOENT);
      free(dirname);
      return(-1);
      break;
    }

    fflush(stdout);
    freopen(filename, "w", stdout);
    fflush(stdout);
    setvbuf(stdout, NULL, _IOLBF, 512);

    strncpy(filename, dirname, B_MAX_FILE_NAME_LEN);
    strncat(filename, logfile, B_MAX_FILE_NAME_LEN);
    strncat(filename, ".stderr", B_MAX_FILE_NAME_LEN);
    fflush(stderr);

    retval = unlink(filename);
    if (retval && (errno!=ENOENT)) {
      perror("bfileUtils: ");
      fprintf(stderr, "errno=%d ENOENT=%d\n", errno, ENOENT);
      free(dirname);
      return(-1);
      break;
    }

    freopen(filename, "w", stderr);
    fflush(stderr);
    setvbuf(stdout, NULL, _IOLBF, 512);

    break;
    
  default:
    exit(0);
  }

  fprintf(stdout, "testing stdout\n");
  fprintf(stderr, "testing stderr\n");
  fflush(stdout);
  fflush(stderr);

  free(dirname);
  return(0);
}

/**********************************************************************
 *
 * char * bGetArrayElementM(const char *arrayStr, int dimc, const int *dim)
 *
 * ! NOTE: The returned char * is malloc()'ed by bGetArrayElement() and 
 *          must be free()'ed by the calling function/program.
 *
 **********************************************************************/

char *
bGetArrayElementM(const char *arrayStr, int dimc, const int *userIndex)
{
  int *index;
  int depth;
  int len;
  char lastChar;
  char *element;

  /*
   * dimc==0 could be considered a mutant case where the logical
   * extention of the rules is to interpret the entire string as
   * a single element that is what is being asked for.  In this case
   * we should return a copy of "the element".
   */

  if (arrayStr == NULL) {
    fprintf(stderr, "%s:%d:%s()\n", __FILE__, __LINE__, __FUNCTION__);
    return(NULL);
  }

  if (dimc == 0) {
    fprintf(stderr, "%s:%d:%s()\n", __FILE__, __LINE__, __FUNCTION__);
    return (strdup(arrayStr));
  }

  if (userIndex == NULL) {
    fprintf(stderr, "%s:%d:%s()\n", __FILE__, __LINE__, __FUNCTION__);
    return(NULL);
  }

  if (dimc < 0) {
    fprintf(stderr, "%s:%d:%s()\n", __FILE__, __LINE__, __FUNCTION__);
    return(NULL);
  }

  if (!(index = malloc(dimc * sizeof(*userIndex)))) {
    fprintf(stderr, "%s:%d:%s()\n", __FILE__, __LINE__, __FUNCTION__);
    return(NULL);
  }

  memcpy(index, userIndex, dimc * sizeof(*userIndex));
  memset(index, 0, dimc * sizeof(*userIndex));

  lastChar = '\0';
  depth = 0;

  while(1) {
    if (isspace(*arrayStr)) { /* skip over white space */
      arrayStr++;
      continue;
    }
      
    if (!*arrayStr) {  /* end of string */
      free(index);
      fprintf(stderr, "%s:%d:%s()\n", __FILE__, __LINE__, __FUNCTION__);
      return(NULL); /* no match was found */
    }

    if ((*arrayStr == '{') && !((lastChar == ',') || (lastChar == '\0'))) {
      free(index);    /* should always be a ',' preceding a '{' */
      fprintf(stderr, "%s:%d:%s()\n", __FILE__, __LINE__, __FUNCTION__);
      return(NULL);
    }

    lastChar = *arrayStr;

    if (*arrayStr == '{') {
      depth++;
    }
    else if (*arrayStr == '}') {
      if (depth <= dimc) {
	index[depth-1] = 0;
      }
      depth--;
      if (depth<1) {
	free(index);
	return(NULL);
      }
    }
    else if (*arrayStr == ',') {
      if (depth <= dimc) {
	index[depth-1]++;
      }
    }
    
    if (depth == 0) {
      arrayStr++;
      continue; /* match at depth zero is invalid */
    }

    if (!memcmp(index, userIndex, dimc * sizeof(*userIndex))) {
      arrayStr++;
      break; /* we have a match */
    }

    arrayStr++;
  }
  
  free(index);

  if (!*arrayStr) {
    fprintf(stderr, "%s:%d:%s()\n", __FILE__, __LINE__, __FUNCTION__);
    return(NULL);
  }

  /*
   * Now find the length of the element.  The element may be an
   * array which makes finding the end of it a little trickier.
   */

  /* skip over white space */

  while (isspace(*arrayStr)) {
    arrayStr++;
  }
      
  depth = 0;
  len = 0;
  lastChar = '\0';

  while (1) {
    if ((depth == 0) && (
			 (arrayStr[len] == '\0') ||
			 (arrayStr[len] == '}' ||
			  arrayStr[len] == ',')
			 )) {
      break; /* found end of element */
    }
    
    if (isspace(arrayStr[len])) { /* skip over white space */
      len++;
      continue;
    }
    
    if ((arrayStr[len] == '{') &&
	  !((lastChar == ',') || (lastChar == '\0'))) {
      fprintf(stderr, "%s:%d:%s()\n", __FILE__, __LINE__, __FUNCTION__);
      return(NULL);    /* should always be a ',' preceeding a '{' */
    }
    
    lastChar = arrayStr[len];

    if (arrayStr[len] == '{') {
      depth++;
    }
    else if (arrayStr[len] == '}') {
      depth--;
      if (depth<0) {
	fprintf(stderr, "%s:%d:%s()\n", __FILE__, __LINE__, __FUNCTION__);
	return(NULL);
      }
    }
    len++;
  }

  while (isspace(arrayStr[len-1])) {
    len--;
  }

  if (!(element = malloc(len+1))) {
    fprintf(stderr, "%s:%d:%s()\n", __FILE__, __LINE__, __FUNCTION__);
    return(NULL);
  }

  if (len<0) len=0;

  memcpy(element, arrayStr, len);
  element[len] = '\0';

  return(element);
}
/**********************************************************************
 *
 * speed_t bStrToBaud(const char *baudStr)
 *
 **********************************************************************/

speed_t
bStrToBaud(const char *baudStr)
{
  unsigned long baudRate;

  if (!baudStr) {
    return(0);
  }

  baudRate = atoi(baudStr);

  switch (baudRate) {
#ifdef B1200
  case 1200:   return(B1200);
#endif
#ifdef B2400
  case 2400:   return(B2400);
#endif
#ifdef B4800
  case 4800:   return(B4800);
#endif
#ifdef B9600
  case 9600:   return(B9600);
#endif
#ifdef B19200
  case 19200:  return(B19200);
#endif
#ifdef B38400
  case 38400:  return(B38400);
#endif
#ifdef B57600
  case 57600:  return(B57600);
#endif
#ifdef B115200
  case 115200: return(B115200);
#endif
#ifdef B230400
  case 230400: return(B230400);
#endif
#ifdef B460800
  case 460800: return(B460800);
#endif
  default: fprintf(stderr, "Invalid baud rate\n"); exit(0);
  }
}

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
 * Grumble: strol() isn't supported by SunOS.  The comparatively
 * broken atoi() is now used.
 *
 ************************************************************/

int
bStrToTruth(const char *str)
{

  if (!str) {
    return(0);
  }

  if (!strcasecmp("yes", str)) {
    return(1);
  }

  if (!strcasecmp("y", str)) {
    return(1);
  }

  if (!strcasecmp("no", str)) {
    return(0);
  }

  if (!strcasecmp("n", str)) {
    return(0);
  }

  if (!strcasecmp("true", str)) {
    return(1);
  }

  if (!strcasecmp("t", str)) {
    return(1);
  }

  if (!strcasecmp("false", str)) {
    return(0);
  }

  if (!strcasecmp("f", str)) {
    return(0);
  }

  if (!strcasecmp("+", str)) {
    return(1);
  }

  if (!strcasecmp("-", str)) {
    return(0);
  }

  if (atoi(str)) {
    return(1);
  }

  return(0);
}

/************************************************************
 *
 * void bParametersParse(struct bParamList *list)
 *
 ************************************************************/

void
bParametersFillParams(struct bParamList *list)
{
  const char *prefix;
  const char *string;
  char *element;

  int ii;
  int index[10];

  bRobot.TCXHOST = bParametersGetParam(list, "", "TCXHOST");
  bRobot.fork    = bStrToTruth(bParametersGetParam(list, "", "fork"));

  /*
   * basic robot system
   */

  prefix = bParametersGetParam(list, "robot", "name");

  /* mobile base */

  bRobot.base_type     = bParametersGetParam(list, prefix, "base.type");
  bRobot.base_host     = bParametersGetParam(list, prefix, "base.host");
  bRobot.base_dev      = bParametersGetParam(list, prefix, "base.dev");

  bRobot.base_bps      = 
    bStrToBaud(bParametersGetParam(list, prefix, "base.bps"));

  bRobot.base_radius   = 
    atof(bParametersGetParam(list, prefix, "base.radius"));

  bRobot.base_encPerCm =
    atof(bParametersGetParam(list, prefix, "base.encPerCm"));

  bRobot.base_posPerCm = 
    atof(bParametersGetParam(list, prefix, "base.posPerCm"));

  bRobot.base_rotBackwards = 
    bStrToTruth(bParametersGetParam(list, prefix, "base.rotBackwards"));

  bRobot.base_hasIndex = 
    bStrToTruth(bParametersGetParam(list, prefix, "base.hasIndex"));

  /* enclosure */

  bRobot.enclosure_type = bParametersGetParam(list, prefix, "enclosure.type");

  bRobot.enclosure_radius  = 
    atof(bParametersGetParam(list, prefix, "enclosure.radius"));

  /* sonar */

  bRobot.sonar_type     = bParametersGetParam(list, prefix, "sonar.type");
  bRobot.sonar_dev      = bParametersGetParam(list, prefix, "sonar.dev");

  string = bParametersGetParam(list, prefix, "sonar.geometry");
  bRobot.sonar_rows = 0;
  ii = 0;

  while (index[0] = ii++, (element = bGetArrayElementM(string, 1, index))) {
    bRobot.sonar_cols[bRobot.sonar_rows++] = atoi(element);
    free(element);
  }

  /* ir */

  bRobot.ir_type     = bParametersGetParam(list, prefix, "ir.type");
  bRobot.ir_dev      = bParametersGetParam(list, prefix, "ir.dev");

  string = bParametersGetParam(list, prefix, "ir.geometry");
  bRobot.ir_rows = 0;
  ii = 0;

  while (index[0] = ii++, (element = bGetArrayElementM(string, 1, index))) {
    bRobot.ir_cols[bRobot.ir_rows++] = atoi(element);
    free(element);
  }

  /* tactile */

  bRobot.tactile_type     = bParametersGetParam(list, prefix, "tactile.type");
  bRobot.tactile_dev      = bParametersGetParam(list, prefix, "tactile.dev");

  string = bParametersGetParam(list, prefix, "tactile.geometry");
  bRobot.tactile_rows = 0;
  ii = 0;

  while (index[0] = ii++, (element = bGetArrayElementM(string, 1, index))) {
    bRobot.tactile_cols[bRobot.tactile_rows++] = atoi(element);
    free(element);
  }

  /* power */

  bRobot.volt_type     = bParametersGetParam(list, prefix, "volt.type");

  string = bParametersGetParam(list, prefix, "volt.warn");

  bRobot.volt_warn  = 0.0;
  if (string) {
    bRobot.volt_warn  = atoi(string);
  }

  string = bParametersGetParam(list, prefix, "volt.panic");

  bRobot.volt_panic  = 0.0;
  if (string) {
    bRobot.volt_panic  = atoi(string);
  }

  /* 
   * Speech stuff
   */

  prefix = bParametersGetParam(list, "robot", "speech");

  bRobot.speech_type     = bParametersGetParam(list, prefix, "speech.type");
  bRobot.speech_host     = bParametersGetParam(list, prefix, "speech.host");
  bRobot.speech_dev      = bParametersGetParam(list, prefix, "speech.dev");

  bRobot.speech_bps      = 
    bStrToBaud(bParametersGetParam(list, prefix, "speech.bps"));


  /* 
   * pantilt stuff
   */

  prefix = bParametersGetParam(list, "robot", "pantilt");


  bRobot.pantilt_type     = bParametersGetParam(list, prefix, "pantilt.type");
  bRobot.pantilt_host     = bParametersGetParam(list, prefix, "pantilt.host");
  bRobot.pantilt_dev      = bParametersGetParam(list, prefix, "pantilt.dev");

  bRobot.pantilt_bps      = 
    bStrToBaud(bParametersGetParam(list, prefix, "pantilt.bps"));


  /* 
   * compass stuff
   */

  prefix = bParametersGetParam(list, "robot", "compass");

  bRobot.compass_type     = bParametersGetParam(list, prefix, "compass.type");
  bRobot.compass_host     = bParametersGetParam(list, prefix, "compass.host");
  bRobot.compass_dev      = bParametersGetParam(list, prefix, "compass.dev");

  bRobot.compass_bps      = 
    bStrToBaud(bParametersGetParam(list, prefix, "compass.bps"));

  /* 
   * laser stuff
   */

  prefix = bParametersGetParam(list, "robot", "laser");

  bRobot.laser_type     = bParametersGetParam(list, prefix, "laser.type");
  bRobot.laser_host     = bParametersGetParam(list, prefix, "laser.host");
  bRobot.laser_dev      = bParametersGetParam(list, prefix, "laser.dev");

  bRobot.laser_bps      = 
    bStrToBaud(bParametersGetParam(list, prefix, "laser.bps"));

  /* 
   * arm stuff
   */

  prefix = bParametersGetParam(list, "robot", "arm");

  bRobot.arm_type      = bParametersGetParam(list, prefix, "arm.type");
  bRobot.arm_mast_host = bParametersGetParam(list, prefix, "arm.mast.host");
  bRobot.arm_mast_dev  = bParametersGetParam(list, prefix, "arm.mast.dev");

  bRobot.arm_mast_bps  = 
    bStrToBaud(bParametersGetParam(list, prefix, "arm.mast.bps"));

  string = bParametersGetParam(list, prefix, "arm.mast.stow");

  bRobot.arm_mast_stow  = 0;
  if (string) bRobot.arm_mast_stow = atoi(string);

  string = bParametersGetParam(list, prefix, "arm.mast.boomTravel");

  bRobot.arm_mast_boomTravel  = 0;
  if (string) bRobot.arm_mast_boomTravel = atoi(string);

  bRobot.arm_grip_host = bParametersGetParam(list, prefix, "arm.grip.host");
  bRobot.arm_grip_dev  = bParametersGetParam(list, prefix, "arm.grip.dev");

  bRobot.arm_grip_bps  = 
    bStrToBaud(bParametersGetParam(list, prefix, "arm.grip.bps"));

  return;
}
