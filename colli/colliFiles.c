
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




#include "collisionIntern.h"
#include <bUtils.h>

BOOLEAN dumpInfo;
FILE* dumpFile = NULL;


/**********************************************************************
 * If this function is called relevant values will be written to
 * stderr. This is just for debugging.
 **********************************************************************/
void
COLLI_DumpInfo()
{
  dumpFile = fopen("dumpFile", "w");
  dumpInfo = TRUE;
}


/**********************************************************************
 * Sets a mark in the dumpfile.
 **********************************************************************/
void
COLLI_MarkInfo()
{
  fprintf( dumpFile, "*******************************************\n");
  fprintf( dumpFile, "************SET A MARK*********************\n");
  fprintf( dumpFile, "*******************************************\n");
}


/* Writes the status of some variables in the dumpfile. */
void
outputCollisionStatus()
{
  static int current_mode = -1;

  if (dumpInfo) {
    fprintf( dumpFile, "\nNext call to collision avoidance.\n");
    fprintf( dumpFile, "posx:          %f\n", rwi_base.pos_x);
    fprintf( dumpFile, "posy:          %f\n", rwi_base.pos_y);
    fprintf( dumpFile, "rot:           %f\n", rwi_base.rot_position);
    fprintf( dumpFile, "exc-rot-vel:   %f\n", ACTUAL_MODE->exception_rot_velocity);
    fprintf( dumpFile, "exc-rot-acc:   %f\n", ACTUAL_MODE->exception_rot_acceleration);
    fprintf( dumpFile, "rot-vel:       %f\n", rwi_base.rot_set_speed);
    fprintf( dumpFile, "rot-acc:       %f\n", rwi_base.rot_acceleration);
    fprintf( dumpFile, "des-rot-vel:   %f\n", desired_rot_velocity);
    fprintf( dumpFile, "curr-rot-vel:  %f\n", rwi_base.rot_current_speed);
    fprintf( dumpFile, "exc-trans-vel: %f\n", ACTUAL_MODE->exception_trans_velocity);
    fprintf( dumpFile, "exc-trans-acc: %f\n", ACTUAL_MODE->exception_trans_acceleration);
    fprintf( dumpFile, "trans-vel:     %f\n", rwi_base.trans_set_speed);
    fprintf( dumpFile, "trans-acc:     %f\n", rwi_base.trans_acceleration);
    fprintf( dumpFile, "des-trans-vel: %f\n", desired_trans_velocity);
    fprintf( dumpFile, "curr-tr-vel:   %f\n", rwi_base.trans_current_speed);
    fprintf( dumpFile, "target_flag:   %d\n", target_flag);

    fprintf( dumpFile, "halting:       %d\n", haltingFlag);
    fprintf( dumpFile, "achieve_dist:  %d\n", achieveDistanceFlag);
    fprintf( dumpFile, "rotate_away:   %d\n", rotatingAwayFlag);
    fprintf( dumpFile, "rot_wanted:    %d\n", rot_wanted);

    
    fprintf( stderr, "\nNext call to collision avoidance.\n");
    fprintf( stderr, "posx:          %f\n", rwi_base.pos_x);
    fprintf( stderr, "posy:          %f\n", rwi_base.pos_y);
    fprintf( stderr, "rot:           %f\n", rwi_base.rot_position);
    fprintf( stderr, "exc-rot-vel:   %f\n", ACTUAL_MODE->exception_rot_velocity);
    fprintf( stderr, "exc-rot-acc:   %f\n", ACTUAL_MODE->exception_rot_acceleration);
    fprintf( stderr, "rot-vel:       %f\n", rwi_base.rot_set_speed);
    fprintf( stderr, "rot-acc:       %f\n", rwi_base.rot_acceleration);
    fprintf( stderr, "des-rot-vel:   %f\n", desired_rot_velocity);
    fprintf( stderr, "curr-rot-vel:  %f\n", rwi_base.rot_current_speed);
    fprintf( stderr, "exc-trans-vel: %f\n", ACTUAL_MODE->exception_trans_velocity);
    fprintf( stderr, "exc-trans-acc: %f\n", ACTUAL_MODE->exception_trans_acceleration);
    fprintf( stderr, "trans-vel:     %f\n", rwi_base.trans_set_speed);
    fprintf( stderr, "trans-acc:     %f\n", rwi_base.trans_acceleration);
    fprintf( stderr, "des-trans-vel: %f\n", desired_trans_velocity);
    fprintf( stderr, "curr-tr-vel:   %f\n", rwi_base.trans_current_speed);
    fprintf( stderr, "target_flag:   %d\n", target_flag);

    fprintf( stderr, "halting:       %d\n", haltingFlag);
    fprintf( stderr, "achieve_dist:  %d\n", achieveDistanceFlag);
    fprintf( stderr, "rotate_away:   %d\n", rotatingAwayFlag);
    fprintf( stderr, "rot_wanted:    %d\n", rot_wanted);

  }
  
  if ( mode_number != current_mode) {
    current_mode = mode_number;
    switch (mode_number) {
    case APPROACH_OBJECT_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "APPROACH\n");
      fprintf(stderr, "APPROACH\n");
      break;
    case APPROACH_TRASH_BIN_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "APPROACH TRASH BIN\n");
      fprintf(stderr, "APPROACH TRASH BIN\n");
      break;
    case ARM_OUT_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "ARM OUT\n");
      fprintf(stderr, "ARM OUT\n");
      break;
    case RANDOM_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "RANDOM\n");
      fprintf(stderr, "RANDOM\n");
      break;
    case ARM_OUT_RANDOM_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "ARM OUT RANDOM\n");
      fprintf(stderr, "ARM OUT RANDOM\n");
      break;
    case FIND_DOOR_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "DOOR\n");
      fprintf(stderr, "DOOR\n");
      break;
    case FAST_TRAVEL_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "FAST\n");
      fprintf(stderr, "FAST\n");
      break;
    case SERVO_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "SERVO\n");
      fprintf(stderr, "SERVO\n");
      break;
    case DEFAULT_MODE: 
      if (dumpInfo)
	fprintf( dumpFile, "DEF\n");
      fprintf(stderr, "DEF\n");
      break;
    default:
      if (dumpInfo)
	fprintf( dumpFile, "UNKNOWN\n");
      fprintf(stderr, "UNKNOWN\n");
    }
  }
}



/**********************************************************************
 * Writes time spent sind begint in the dumpfile.
 **********************************************************************/
void
outputTimeInfo( struct timeval* begint)
{
  if (dumpInfo) {
    struct timeval endt;
    
    gettimeofday( &endt, 0);
    
    endt.tv_usec -= begint->tv_usec;
    endt.tv_sec -= begint->tv_sec;
    if (endt.tv_usec < 0) {
      endt.tv_usec += 1000000;
      endt.tv_sec -= 1;
    }
    
    if ( dumpFile != NULL)
      fprintf( dumpFile, "tv %.2f (%.4f)\n", actual_velocities.current_tvel,
	      (float)endt.tv_sec + (float)endt.tv_usec / 1000000.0);
  }
}


    
/* Checks wether the default mode is set already. If this is the first
 * time after having set the default mode we have to copy the values
 * into the other mode structures.
 */
static void
checkForDefaultMode( BOOLEAN* settingDefaultMode,
		     BOOLEAN* defaultModeAlreadySet)
{ 
    /* We have just read the default mode and can now copy the values to
     * the other modes. */
    if ( *settingDefaultMode) {
      copyDefaultMode();
      *settingDefaultMode = FALSE;
      *defaultModeAlreadySet = TRUE;
      return;
    }
    /* If the default mode is not set in the ini file we use the values
     * from the source code. */
    else if ( ! defaultModeAlreadySet) {
      putc( 7, stderr);
      fprintf( stderr, "Warning: the first mode in the parameter file should ");
      fprintf( stderr, "be the default mode.\n");
      return;
    }
    /* Everything is fine. */
}

/************************************************************************
 *
 *   NAME:         load_parameters
 *                 
 *   FUNCTION:     loads parameters for the different modes from a file.
 *                 The default values are replaced by these values.
 *                 
 ************************************************************************/

BOOLEAN
load_parameters(char *filename)
{
  FILE *iop;
  char line[256];
  char filename2[256];
  char *filename3;
  mode_structure *mode;
  BOOLEAN first_line = TRUE;
  BOOLEAN defaultModeAlreadySet = FALSE;
  BOOLEAN settingDefaultMode = FALSE;
  
  /* Allocate memory for all modes and set the values to default. */
  setAllModesToDefault();
  
  mode = mode_structure_array[DEFAULT_MODE];

  sprintf(filename2, "etc/%s", filename);

  filename3 = bFindFileM(filename2);
  
  if ((iop = fopen(filename3, "r")) == 0){
    fprintf(stderr, "Could not open input file %s. File not loaded.\n",
            filename3);
    fprintf(stderr, "WARNING: Failed to read file %s.\n", filename);
    fprintf(stderr, "All modes will have the same default values!\n");
    free(filename3);
    return 0;
  }

  while (!feof (iop)) {
    if ( fscanf (iop, "%s", line) == EOF)
       break;

    /****************************************************************************
     * Determine the mode.
     ****************************************************************************/
    if (strcmp(line, "DEFAULT_MODE") == 0) {
      /* The following values will be stored in the structure for exploration.*/
      settingDefaultMode = TRUE;
      first_line = FALSE;
      mode = mode_structure_array[DEFAULT_MODE];
    }
    else if (strcmp(line, "FIND_DOOR_MODE") == 0) {
      /* The following values will be stored in the structure for finding doors.*/
      checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
      first_line = FALSE;
      mode = mode_structure_array[FIND_DOOR_MODE];
    }
    else if (strcmp(line, "FAST_TRAVEL_MODE") == 0) {
      /* The following values will be stored in the structure for fast travel.*/
      checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
      mode = mode_structure_array[FAST_TRAVEL_MODE];
      first_line = FALSE;
    }
    else if (strcmp(line, "APPROACH_OBJECT_MODE") == 0) {
      /* The following values will be stored in the structure for approaching objects.*/
	checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
	mode = mode_structure_array[APPROACH_OBJECT_MODE];
	first_line = FALSE;
    }
    else if (strcmp(line, "APPROACH_TRASH_BIN_MODE") == 0) {
      /* The following values will be stored in the structure for approaching trash bins.*/
	checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
	mode = mode_structure_array[APPROACH_TRASH_BIN_MODE];
	first_line = FALSE;
    }
    else if (strcmp(line, "ARM_OUT_MODE") == 0) {
      /* ... will be stored in the structure for picking up objects.*/
	checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
	mode = mode_structure_array[ARM_OUT_MODE];
	first_line = FALSE;
    }
    else if (strcmp(line, "RANDOM_MODE") == 0) {
      /* ... will be stored in the structure for moving randomly.*/
	checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
	mode = mode_structure_array[RANDOM_MODE];
	first_line = FALSE;
    }
    else if (strcmp(line, "ARM_OUT_RANDOM_MODE") == 0) {
      /* ... will be stored in the structure for moving randomly.*/
	checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
	mode = mode_structure_array[ARM_OUT_RANDOM_MODE];
	first_line = FALSE;
    }
    else if (strcmp(line, "SERVO_MODE") == 0) {
      /* ...  will be stored in the structure for visual servoing.*/
	checkForDefaultMode( &settingDefaultMode, &defaultModeAlreadySet);
	mode = mode_structure_array[SERVO_MODE];
	first_line = FALSE;
    }
    else if (line[0] ==  '#') {
	fgets( line, sizeof(line),iop);
    }
    else if ( first_line) {
      fprintf(stderr,
	      "ERROR in mode ini file: No mode specified. First line must specify a mode!\n");
    }
    
    /****************************************************************************
     * Read the values for the mode.
     ****************************************************************************/

    /* These two values may only be set once are the same for all modes. */
    else if (strcmp(line, "REMEMBER_INTERVAL") == 0)
      fscanf (iop, "%i", &REMEMBER_INTERVAL);

    /* The interval between two updates of the collision avoidance. */
    else if (strcmp(line, "COLLISION_UPDATE_INTERVAL") == 0)  
	fscanf (iop, "%f", &COLLISION_UPDATE_INTERVAL);

    /* These Values are set local for each Mode */
    
    else if (strcmp(line, "SECURITY_ANGLE") == 0)
      fscanf (iop, "%f", &(mode->security_angle));
    else if (strcmp(line, "ROTATE_AWAY_PERSISTENCE") == 0)
       fscanf (iop, "%i", &(mode->rotate_away_persistence));
    

    
    /* The parameters of the evaluation function. */
    else if (strcmp(line, "VELOCITY_FACTOR") == 0) 
    fscanf (iop, "%f", &(mode->velocity_factor)); 
    else if (strcmp(line, "ANGLE_FACTOR") == 0)             
      fscanf (iop, "%f", &(mode->angle_factor));
    else if (strcmp(line, "DISTANCE_FACTOR") == 0)             
      fscanf (iop, "%f", &(mode->distance_factor));

    
    /* Number of velocities for the dynamic window. */
    else if (strcmp(line, "NUMBER_OF_RVELS") == 0)  
      fscanf (iop, "%i", &(mode->number_of_rvels));
    else if (strcmp(line, "NUMBER_OF_TVELS") == 0)  
      fscanf (iop, "%i", &(mode->number_of_tvels));

    /* Velocities and accelerations. */
    else if (strcmp(line, "TARGET_MAX_TRANS_SPEED") == 0)    
      fscanf (iop, "%f", &(mode->target_max_trans_speed));
    else if (strcmp(line, "TARGET_TRANS_ACCELERATION") == 0) 
      fscanf (iop, "%f", &(mode->target_trans_acceleration));
    else if (strcmp(line, "TARGET_MAX_ROT_SPEED") == 0)  {
      fscanf (iop, "%f", &(mode->target_max_rot_speed));
      mode->target_max_rot_speed = DEG_TO_RAD(mode->target_max_rot_speed);
    }
    else if (strcmp(line, "TARGET_ROT_ACCELERATION") == 0)   {
      fscanf (iop, "%f", &(mode->target_rot_acceleration));
      mode->target_rot_acceleration = DEG_TO_RAD(mode->target_rot_acceleration);
    }

    /* Velocities and accelerations if the robot is in an exception. */
    else if (strcmp(line, "EXCEPTION_TRANS_VELOCITY") == 0) 
      fscanf (iop, "%f", &(mode->exception_trans_velocity));
    else if (strcmp(line, "EXCEPTION_TRANS_ACCELERATION") == 0) 
      fscanf (iop, "%f", &(mode->exception_trans_acceleration));
    else if (strcmp(line, "EXCEPTION_ROT_VELOCITY") == 0)  {
      fscanf (iop, "%f", &(mode->exception_rot_velocity));
      mode->exception_rot_velocity = DEG_TO_RAD(mode->exception_rot_velocity);
    }
    else if (strcmp(line, "EXCEPTION_ROT_ACCELERATION") == 0)  {
      fscanf (iop, "%f", &(mode->exception_rot_acceleration));
      mode->exception_rot_acceleration = DEG_TO_RAD(mode->exception_rot_acceleration);
    }

    /* Values concerning trajectories. */
    else if (strcmp(line, "MIN_DIST") == 0)     
      fscanf (iop, "%f", &(mode->min_dist));
    else if (strcmp(line, "SMOOTH_WIDTH") == 0)     
      fscanf (iop, "%d", &(mode->smooth_width));
    else if (strcmp(line, "SECURITY_DIST") == 0)
      fscanf (iop, "%f", &(mode->security_dist));
    else if (strcmp(line, "MIN_DIST_FOR_TARGET_WAY_FREE") == 0)
      fscanf (iop, "%f", &(mode->min_dist_for_target_way_free));
    else if (strcmp(line, "MAX_COLLISION_LINE_LENGTH") == 0)  
      fscanf (iop, "%f", &(mode->max_collision_line_length));
    else if (strcmp(line, "MAX_RANGE") == 0)  
      fscanf (iop, "%f", &(mode->max_range));

    else if (strcmp(line, "EDGE_PORTION") == 0)
      fscanf (iop, "%f", &(mode->edge_portion));
    else if (strcmp(line, "MIN_SECURITY_SPEED") == 0)  
      fscanf (iop, "%f", &(mode->min_security_speed));
    else if (strcmp(line, "MAX_SECURITY_SPEED") == 0)  
      fscanf (iop, "%f", &(mode->max_security_speed));
    else if (strcmp(line, "MAX_SECURITY_DIST") == 0)  
      fscanf (iop, "%f", &(mode->max_security_dist));

    else {
      fprintf( stderr, "Wrong format in initialization file for the different modes:");
      fprintf( stderr, "don't know how to process %s\n", line);
      exit;
    }
  }

  fclose (iop);

  /* If only the default is set we have to copy the values. */
  if ( settingDefaultMode)
    copyDefaultMode();
    
  fprintf(stderr, "Successfully read mode structures from %s.\n",
	  filename2);

  return TRUE;
}


