
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




#ifdef VMS
#include "vms.h"
#include "vmstime.h"
#endif

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include "tcx.h"
#define TCX_define_variables /* this makes sure variables are installed */


#include "PLAN-messages.h"
#include "COLLI-messages.h"
#include "MAP-messages.h"
#include "BASE-messages.h"
#include "SONAR-messages.h"


#include "PLAN.h"
#include "o-graphics.h"

#include <bUtils.h>

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



/************************************************************************
 *
 *   NAME:         save_parameters()
 *                 
 *   FUNCTION:     saves a map into a file. Map-intrinsic format.
 *                 
 *   PARAMETERS:   filename
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


int save_parameters(char *filename,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int i, x, y, index;
  FILE *iop;
  char filename2[256];
  

  sprintf(filename2, "%s", filename);
  if ((iop = fopen(filename2, "w")) == 0){
    fprintf(stderr, "WARNING: Could not open output file %s.\n", filename2);
    sprintf(filename2, "../etc/%s", filename);
    if ((iop = fopen(filename2, "w")) == 0){
      fprintf(stderr, "WARNING: Could not open output file %s.\n", filename2);
      sprintf(filename2, "../../etc/%s", filename);
      if ((iop = fopen(filename2, "w")) == 0){
	fprintf(stderr, 
		"WARNING: Could not open output file %s. Not saved.\n",
		filename2);
	return 0;
      }
    }
  }



  fprintf(iop,"robot_specifications->global_mapsize_x             %g\n",
	  robot_specifications->global_mapsize_x);
  fprintf(iop,"robot_specifications->global_mapsize_y             %g\n",
	  robot_specifications->global_mapsize_y);
  fprintf(iop,"robot_specifications->resolution                   %g\n",
	  robot_specifications->resolution);
  fprintf(iop,"robot_specifications->robot_size                   %g\n",
	  robot_specifications->robot_size);
  fprintf(iop,"robot_specifications->max_security_dist            %g\n",
	  robot_specifications->max_security_dist);
  fprintf(iop,"robot_specifications->autoshift                    %d\n",
	  robot_specifications->autoshift);
  fprintf(iop,"robot_specifications->autoshift_distance           %g\n",
	  robot_specifications->autoshift_distance);
  fprintf(iop,"robot_specifications->autoshift_safety_margin      %g\n",
	  robot_specifications->autoshift_safety_margin);
  fprintf(iop,"robot_specifications->average_value                %g\n",
	  robot_specifications->average_value);
  fprintf(iop,"robot_specifications->collision_threshold          %g\n",
	  robot_specifications->collision_threshold);
  fprintf(iop,"robot_specifications->costs_exponent               %g\n",
	  robot_specifications->costs_exponent);
  fprintf(iop,"robot_specifications->min_base                     %g\n",
	  robot_specifications->min_base);
  fprintf(iop,"robot_specifications->max_base                     %g\n",
	  robot_specifications->max_base);
  fprintf(iop,"robot_specifications->max_adjust_angle             %g\n",
	  robot_specifications->max_adjust_angle);
  fprintf(iop,"robot_specifications->max_goal_distance            %g\n",
	  robot_specifications->max_goal_distance);
  fprintf(iop,"robot_specifications->max_approach_distance        %g\n",
	  robot_specifications->max_approach_distance);
  fprintf(iop,"robot_specifications->max_final_approach_distance  %g\n",
	  robot_specifications->max_final_approach_distance);
  fprintf(iop,"robot_specifications->map_update_frequency         %d\n",
	  robot_specifications->map_update_frequency);
  fprintf(iop,"robot_specifications->exploration_circle_size      %g\n",
	  robot_specifications->exploration_circle_size);
  fprintf(iop,"robot_specifications->min_mapvalue_along_path      %g\n",
	  robot_specifications->min_mapvalue_along_path);
  fprintf(iop,"robot_specifications->border_to_interior           %g\n",
	  robot_specifications->border_to_interior);
  fprintf(iop,"robot_specifications->exterior_factor              %g\n",
	  robot_specifications->exterior_factor);
  fprintf(iop,"robot_specifications->max_bounding_box_size        %g\n",
	  robot_specifications->max_bounding_box_size);
  fprintf(iop,"robot_specifications->fast_exploration             %d\n",
	  robot_specifications->fast_exploration);
  fprintf(iop,"robot_specifications->generate_actions_continuously %g\n",
	  robot_specifications->generate_actions_continuously);
  fprintf(iop, "robot_specifications->X_window_size                    %g\n",
	  robot_specifications->X_window_size);


  fclose(iop);
  fprintf(stderr, "File %s successfully written.\n", filename2);
  return 1;
}


/************************************************************************
 *
 *   NAME:         load_parameters()
 *                 
 *   FUNCTION:     loads a map from a file. Map-intrinsic format.
 *                 
 *   PARAMETERS:   filename
 *                 init        1, if MAP is initialized
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/

int load_parameters( char *filename, int init,
		     ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int i, x, y, index, int_value, int_value1, int_value2;
  float float_value;
  FILE *iop;
  int file_ended, error;
  char command[256];
  char filename2[256];
  
   char *filename3;

  sprintf(filename2, "etc/%s", filename);

  filename3 = bFindFileM(filename2);
  
  if ((iop = fopen(filename3, "r")) == 0){
    fprintf(stderr, "Could not open input file %s. File not loaded.\n",
            filename3);
    fprintf(stderr, "WARNING: Failed to read file %s.\n", filename);
    free(filename3);
    return 0;
  }

  free(filename3);

  file_ended = 0;
  error = 0;
  do{
    if (fscanf(iop, "%s", command) == EOF)
      file_ended = 1;
    
    else{
      
      if (!strcmp(command, "end"))
	file_ended = 2;
      
      else if (command[0] == '#'){ /* comment */
	if (!fgets(command,sizeof(command),iop))
	  file_ended = 2;	/* file may end with hash-sign /  comment */
	}






      else if (!strcmp(command, "robot_specifications->global_mapsize_x")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->global_mapsize_x = float_value;
	else if (float_value != robot_specifications->global_mapsize_x){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->global_mapsize_x");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      else if (!strcmp(command, "robot_specifications->global_mapsize_y")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->global_mapsize_y = float_value;
	else if (float_value != robot_specifications->global_mapsize_y){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->global_mapsize_y");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      



      else if (!strcmp(command, "robot_specifications->resolution")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->resolution = float_value;
	else if (float_value != robot_specifications->resolution){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->resolution");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      else if (!strcmp(command, "robot_specifications->robot_size")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->robot_size = float_value;
	else if (float_value != robot_specifications->robot_size){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->robot_size");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      else if (!strcmp(command, "robot_specifications->max_security_dist")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_security_dist = float_value;
	else if (float_value != robot_specifications->max_security_dist){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->max_security_dist");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      else if (!strcmp(command, "robot_specifications->autoshift")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->autoshift = int_value;
	else if (int_value != robot_specifications->autoshift){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->autoshift");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      else if (!strcmp(command, "robot_specifications->autoshift_distance")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->autoshift_distance = float_value;
	else if (float_value != robot_specifications->autoshift_distance){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->autoshift_distance");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      else if (!strcmp(command, "robot_specifications->autoshift_safety_margin")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->autoshift_safety_margin = float_value;
	else if (float_value != robot_specifications->autoshift_safety_margin){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->autoshift_safety_margin");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      else if (!strcmp(command, "robot_specifications->average_value")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->average_value = float_value;
	else if (float_value != robot_specifications->average_value){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->average_value");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      else if (!strcmp(command, "robot_specifications->collision_threshold")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->collision_threshold = float_value;
	else if (float_value != robot_specifications->collision_threshold){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->collision_threshold");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      
      else if (!strcmp(command, "robot_specifications->costs_exponent")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->costs_exponent = float_value;
	else if (float_value != robot_specifications->costs_exponent){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->costs_exponent");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      else if (!strcmp(command, "robot_specifications->min_base")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->min_base = float_value;
	else if (float_value != robot_specifications->min_base){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->min_base");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      else if (!strcmp(command, "robot_specifications->max_base")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_base = float_value;
	else if (float_value != robot_specifications->max_base){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->max_base");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      else if (!strcmp(command, "robot_specifications->max_adjust_angle")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_adjust_angle = float_value;
	else if (float_value != robot_specifications->max_adjust_angle){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->max_adjust_angle");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      else if (!strcmp(command, "robot_specifications->max_goal_distance")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_goal_distance = float_value;
	else if (float_value != robot_specifications->max_goal_distance){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->max_goal_distance");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, 
		       "robot_specifications->max_approach_distance")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_approach_distance = float_value;
	else if (float_value != robot_specifications->max_approach_distance){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->max_approach_distance");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
          
        
      else if (!strcmp(command,
		       "robot_specifications->max_final_approach_distance")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_final_approach_distance = float_value;
	else if (float_value !=
		 robot_specifications->max_final_approach_distance){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->max_final_approach_distance");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      
      else if (!strcmp(command, "robot_specifications->map_update_frequency")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->map_update_frequency = int_value;
	else if (int_value != robot_specifications->map_update_frequency){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->map_update_frequency");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        
      else if (!strcmp(command, 
		       "robot_specifications->exploration_circle_size")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->exploration_circle_size = float_value;
	else if (float_value != robot_specifications->exploration_circle_size){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->exploration_circle_size");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      else if (!strcmp(command, 
		       "robot_specifications->min_mapvalue_along_path")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->min_mapvalue_along_path = float_value;
	else if (float_value != robot_specifications->min_mapvalue_along_path){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->min_mapvalue_along_path");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      else if (!strcmp(command, 
		       "robot_specifications->border_to_interior")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->border_to_interior = float_value;
	else if (float_value != robot_specifications->border_to_interior){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->border_to_interior");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        


      else if (!strcmp(command, 
		       "robot_specifications->exterior_factor")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->exterior_factor = float_value;
	else if (float_value != robot_specifications->exterior_factor){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->exterior_factor");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        


      else if (!strcmp(command, 
		       "robot_specifications->max_bounding_box_size")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_bounding_box_size = float_value;
	else if (float_value != robot_specifications->max_bounding_box_size){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->max_bounding_box_size");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }

      else if (!strcmp(command, "robot_specifications->fast_exploration")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->fast_exploration = int_value;
	else if (int_value != robot_specifications->fast_exploration){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->fast_exploration");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      else if (!strcmp(command, "robot_specifications->X_window_size")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->X_window_size = float_value;
      }

      
      
      else if (!strcmp(command, 
		       "robot_specifications->generate_actions_continuously")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->generate_actions_continuously = float_value;
	else if (float_value != robot_specifications->generate_actions_continuously){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->generate_actions_continuously");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
  
      
      else{
	fprintf(stderr, "ERROR: Unknown keyword \"%s\" in %s. Must exit.\n ", 
		command, filename2);
	error = 1;
      }
 
      if (file_ended == 1)
	fprintf(iop, "Surprising end of file %s.\n ", filename2);
      
    }
  } while (!file_ended);
  fclose(iop);
  
  if (!error)
    fprintf(stderr, "File %s successfully read.\n", filename2);
  return (1 - error);
}

