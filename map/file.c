
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
#include "tcxP.h"
#include "MAP-messages.h"
#include "SONAR-messages.h"
#include "BASE-messages.h"
#include "MAP.h"
#include "EZX11.h"
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


int save_parameters(char *filename)
{
  int i, j, x, y, index;
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


  fprintf(iop, "robot_state->x                                         %g\n",
	  robot_state->x);
  fprintf(iop, "robot_state->y                                         %g\n",
	  robot_state->y);
  fprintf(iop, "robot_state->orientation                               %g\n",
	  robot_state->orientation);
  fprintf(iop, "robot_state->translational_speed                       %g\n",
	  robot_state->translational_speed);
  fprintf(iop, "robot_state->rotational_speed                          %g\n",
	  robot_state->rotational_speed);
  fprintf(iop, "robot_state->sensor_x                                  %g\n", 
	  robot_state->sensor_x);
  fprintf(iop, "robot_state->sensor_y                                  %g\n",
	  robot_state->sensor_y);
  fprintf(iop, "robot_state->sensor_orientation                        %g\n", 
	  robot_state->sensor_orientation);
  fprintf(iop, "robot_state->sensor_uncertainty                        %g\n",
	  robot_state->sensor_uncertainty);
  fprintf(iop, "robot_state->sensor_org_x                              %g\n", 
	  robot_state->sensor_org_x);
  fprintf(iop, "robot_state->sensor_org_y                              %g\n",
	  robot_state->sensor_org_y);
  fprintf(iop, "robot_state->sensor_org_orientation                    %g\n", 
	  robot_state->sensor_org_orientation);
  fprintf(iop, "robot_state->correction_parameter_x                    %g\n",
	  robot_state->correction_parameter_x);
  fprintf(iop, "robot_state->correction_parameter_y                    %g\n",
	  robot_state->correction_parameter_y);
  fprintf(iop, "robot_state->correction_parameter_angle                %g\n",
	  robot_state->correction_parameter_angle);
  fprintf(iop, "robot_state->correction_type                           %d\n",
	  robot_state->correction_type);
  fprintf(iop, "robot_state->map_orientation_defined                   %d\n",
	  robot_state->map_orientation_defined);
  fprintf(iop, "robot_state->map_orientation                           %g\n",
	  robot_state->map_orientation);



  fprintf(iop, "\n");


  fprintf(iop, "robot_specifications->global_mapsize_x                 %g\n",
	  robot_specifications->global_mapsize_x);
  fprintf(iop, "robot_specifications->global_mapsize_y                 %g\n",
	  robot_specifications->global_mapsize_y);
  fprintf(iop, "robot_specifications->resolution                       %g\n",
	  robot_specifications->resolution);
  fprintf(iop, "robot_specifications->robot_size                       %g\n",
	  robot_specifications->robot_size);
  fprintf(iop, "robot_specifications->drift                            %g\n",
	  robot_specifications->drift);
  fprintf(iop, "robot_specifications->smooth_radius                    %d\n",
	  robot_specifications->smooth_radius);
  fprintf(iop, "robot_specifications->autoshift                        %d\n",
	  robot_specifications->autoshift);
  fprintf(iop, "robot_specifications->autoshift_safety_margin          %g\n",
	  robot_specifications->autoshift_safety_margin);
  fprintf(iop, "robot_specifications->autoshift_distance               %g\n",
	  robot_specifications->autoshift_distance);
  fprintf(iop, "robot_specifications->autoshifted_x                    %g\n",
	  robot_specifications->autoshifted_x);
  fprintf(iop, "robot_specifications->autoshifted_y                    %g\n",
	  robot_specifications->autoshifted_y);
  fprintf(iop, "robot_specifications->autoshifted_int_x                %d\n",
	  robot_specifications->autoshifted_int_x);
  fprintf(iop, "robot_specifications->autoshifted_int_y                %d\n",
	  robot_specifications->autoshifted_int_y);
  fprintf(iop, "robot_specifications->do_position_correction           %d\n",
	  robot_specifications->do_position_correction);
  fprintf(iop, "robot_specifications->max_distance_in_match            %g\n",
	  robot_specifications->max_distance_in_match);
  fprintf(iop, "robot_specifications->map_fit_norm_L2                  %d\n",
	  robot_specifications->map_fit_norm_L2);
  fprintf(iop, "robot_specifications->prev_pos_norm_L2                 %d\n",
	  robot_specifications->prev_pos_norm_L2);
  fprintf(iop, "robot_specifications->max_niterations_in_search        %d\n",
	  robot_specifications->max_niterations_in_search);
  fprintf(iop, "robot_specifications->niterations_in_map_fitting       %d\n",
	  robot_specifications->niterations_in_map_fitting);
  fprintf(iop, "robot_specifications->max_translation_in_search        %g\n",
	  robot_specifications->max_translation_in_search);
  fprintf(iop, "robot_specifications->translation_weight_fit           %g\n",
	  robot_specifications->translation_weight_fit);
  fprintf(iop, 
	  "robot_specifications->translation_weight_fit_global_match %g\n",
	  robot_specifications->translation_weight_fit_global_match);
  fprintf(iop, "robot_specifications->translation_weight_prev_position %g\n",
	  robot_specifications->translation_weight_prev_position);
  fprintf(iop, "robot_specifications->translation_stepsize             %g\n",
	  robot_specifications->translation_stepsize);
  fprintf(iop, "robot_specifications->translation_momentum             %g\n",
	  robot_specifications->translation_momentum);
  fprintf(iop, "robot_specifications->max_rotation_in_search           %g\n",
	  robot_specifications->max_rotation_in_search);
  fprintf(iop, "robot_specifications->rotation_weight_fit              %g\n",
	  robot_specifications->rotation_weight_fit);
  fprintf(iop, "robot_specifications->rotation_weight_fit_global_match %g\n",
	  robot_specifications->rotation_weight_fit_global_match);
  fprintf(iop, "robot_specifications->rotation_weight_prev_position    %g\n",
	  robot_specifications->rotation_weight_prev_position);
  fprintf(iop, "robot_specifications->rotation_stepsize                %g\n",
	  robot_specifications->rotation_stepsize);
  fprintf(iop, "robot_specifications->rotation_momentum                %g\n",
	  robot_specifications->rotation_momentum);
  fprintf(iop, "robot_specifications->search_granularity               %d\n",
	  robot_specifications->search_granularity);
  fprintf(iop, "robot_specifications->do_path_fitting                  %d\n",
	  robot_specifications->do_path_fitting);
  fprintf(iop, "robot_specifications->weight_path_fit                  %g\n",
	  robot_specifications->weight_path_fit);
  fprintf(iop, "robot_specifications->n_path_points_in_fit             %d\n",
	  robot_specifications->n_path_points_in_fit);
  fprintf(iop, "robot_specifications->wall_error_threshold             %g\n",
	  robot_specifications->wall_error_threshold);
  fprintf(iop, "robot_specifications->number_subsequent_adjacent_walls %d\n",
	  robot_specifications->number_subsequent_adjacent_walls);
  fprintf(iop, "robot_specifications->wall_weight                      %g\n",
	  robot_specifications->wall_weight);
  fprintf(iop, "robot_specifications->min_advance_for_map_fitting      %g\n",
	  robot_specifications->min_advance_for_map_fitting);
  fprintf(iop, "robot_specifications->decay_old                        %g\n",
	  robot_specifications->decay_old);
  fprintf(iop, "robot_specifications->decay_new                        %g\n",
	  robot_specifications->decay_new);
  fprintf(iop, "robot_specifications->prior                            %g\n",
	  robot_specifications->prior);
  fprintf(iop, "robot_specifications->update_extreme_likelihoods       %d\n",
	  robot_specifications->update_extreme_likelihoods);
  fprintf(iop, "robot_specifications->lower_clipping_value             %g\n",
	  robot_specifications->lower_clipping_value);
  fprintf(iop, "robot_specifications->upper_clipping_value             %g\n",
	  robot_specifications->upper_clipping_value);
  fprintf(iop, "robot_specifications->reposition_robot_initially       %d\n",
	  robot_specifications->reposition_robot_initially);
  fprintf(iop, "robot_specifications->regular_gif_output_in_sec        %d\n",
	  robot_specifications->regular_gif_output_in_sec);
  fprintf(iop, "robot_specifications->X_window_size                    %g\n",
	  robot_specifications->X_window_size);
  fprintf(iop, "robot_specifications->data_logging                     %d\n",
	  robot_specifications->data_logging);
  fprintf(iop, "robot_specifications->map_erasing_period               %d\n",
	  robot_specifications->map_erasing_period);

  fprintf(iop, "\n");



  fprintf(iop, "path: %d\n", n_path_entries);
  for (j = 0; j < n_path_entries; j++)
    fprintf(iop, "%g %g %g\n", path[j][0], path[j][1],  path[j][2]);
  fprintf(iop, "\n");
  
  for (i = 0; i < NUM_GLOBAL_MAPS; i++){
    fprintf(iop, "global_map[%d]: %d %d\n", i,
	    robot_specifications->global_map_dim_x,
	    robot_specifications->global_map_dim_y);
    for (x = 0; x < robot_specifications->global_map_dim_x; x++){
      for (y = 0; y < robot_specifications->global_map_dim_y; y++){
	index = x * robot_specifications->global_map_dim_y + y;
	if (global_active_x[i][index])
	  fprintf(iop, " %g", global_map_x[i][index]);
	else
	  fprintf(iop, " -1");
      }
      fprintf(iop, "\n\n");
    }
  }

  fclose(iop);

  fprintf(stderr, "File %s successfully written.\n", filename2);

  return 1;
}


/************************************************************************
 *
 *   NAME:         save_gif()
 *                 
 *   FUNCTION:     saves a map into a file. Map-intrinsic format.
 *                 
 *   PARAMETERS:   filename
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

#include "gd.h"
#define NUMBER_OF_GREY_LEVELS 100
static struct timeval last_gif = {0, 0};

int save_gif(char *filename, int map_number, int force)
{
  int i, j, x, y, index;
  int index1;
  int min_i, max_i, min_j, max_j;
  FILE *iop;
  register int gifSizeX, gifSizeY;
  gdImagePtr GifPic;
  int color[NUMBER_OF_GREY_LEVELS];
  int unknownColor;
  int zoom = 1;
  struct timeval current_time;

  if (!force){
    gettimeofday(&current_time, NULL);
    if (robot_specifications->regular_gif_output_in_sec < 0 ||
	current_time.tv_sec < 
	last_gif.tv_sec + robot_specifications->regular_gif_output_in_sec
	|| (current_time.tv_sec == 
	    last_gif.tv_sec + robot_specifications->regular_gif_output_in_sec
	    && current_time.tv_usec < last_gif.tv_usec))
      return 0;
    /*else
      fprintf(stderr, "Time: %d\n",
	      (int) (current_time.tv_sec - last_gif.tv_sec));*/
  }    


  min_i = min_j = -1;
  max_i = max_j = -2;
  for (i = 0; i < robot_specifications->global_map_dim_x; i++)
    for (j = 0; j < robot_specifications->global_map_dim_y; j++){
      index1 = i * robot_specifications->global_map_dim_y + j;
      if (global_active_x[map_number][index1]){
	if (min_i < 0 || i < min_i)
	  min_i = i;
	if (min_j < 0 || j < min_j)
	  min_j = j;
	if (max_i < 0 || i > max_i)
	  max_i = i + 1;
	if (max_j < 0 || j > max_j)
	  max_j = j + 1;
      }
    }

  
  if (max_i > min_i && max_j > min_j){
    
    
    if ((iop = fopen(filename, "w")) == 0){
      fprintf(stderr, "WARNING: Could not open output file %s.\n", filename);
      return -1;
    }
    
    gifSizeX = (max_i - min_i) * zoom;
    gifSizeY = (max_j - min_j) * zoom;
    GifPic = gdImageCreate( gifSizeX, gifSizeY);
  
    for (i = 0; i < NUMBER_OF_GREY_LEVELS; i++){
      int greyValue = i * (255.0 / NUMBER_OF_GREY_LEVELS) + 0.5;
      color[i] = gdImageColorAllocate(GifPic, greyValue, greyValue, greyValue);
    }
    unknownColor = gdImageColorAllocate(GifPic, 0, 134, 138);


    for (i = min_i; i < max_i; i++)
      for (j = min_j; j < max_j; j++) {
	register int x, y;
	int tmpX, tmpY;
	int col;

	index = i * robot_specifications->global_map_dim_y + j;
	if (global_active_x[map_number][index])
	  col = color[ (int) (global_map_x[map_number][index] 
			      * (NUMBER_OF_GREY_LEVELS-1) + 0.5)];
	else
	  col = unknownColor;
	tmpX = (i-min_i) * zoom;
	tmpY = (j-min_j) * zoom;
	for (x = tmpX; x < tmpX + zoom; x++)
	  for (y = tmpY; y < tmpY + zoom; y++)
	    gdImageSetPixel( GifPic, x, gifSizeY - y - 1, col );
      }

  
    gdImageGif( GifPic, iop );
    fclose(iop);

    fprintf(stderr, "File %s successfully written.\n", filename);


    gettimeofday(&last_gif, NULL);

    return 1;
  }
  return 0;
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


int load_parameters(char *filename, int init)
{
  int i, x, y, index, int_value, int_value1, int_value2, int_value3;
  float float_value;
  FILE *iop;
  int file_ended, error;
  char command[256];
  char filename2[256];
  char *cmd_ptr;
 
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

      else if (!strcmp(command, "robot_state->x")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_state->x = float_value;
      }
      
      
      else if (!strcmp(command, "robot_state->y")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_state->y = float_value;
      }
      
      
      else if (!strcmp(command, "robot_state->orientation")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_state->orientation = float_value;
      }
      
      else if (!strcmp(command, "robot_state->translational_speed")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_state->translational_speed = float_value;
      }
      
      else if (!strcmp(command, "robot_state->rotational_speed")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_state->rotational_speed = float_value;
      }
      
      else if (!strcmp(command, "robot_state->sensor_x")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_state->sensor_x = float_value;
      }
      
      
      else if (!strcmp(command, "robot_state->sensor_y")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_state->sensor_y = float_value;
      }
      
      
      else if (!strcmp(command, "robot_state->sensor_orientation")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_state->sensor_orientation = float_value;
      }
      

      else if (!strcmp(command, "robot_state->sensor_uncertainty")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_state->sensor_uncertainty = float_value;
      }
      

      else if (!strcmp(command, "robot_state->sensor_org_x")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_state->sensor_org_x = float_value;
      }
      
      
      else if (!strcmp(command, "robot_state->sensor_org_y")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_state->sensor_org_y = float_value;
      }
      
      
      else if (!strcmp(command, "robot_state->sensor_org_orientation")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_state->sensor_org_orientation = float_value;
      }
      
      
      else if (!strcmp(command, "robot_state->correction_parameter_x")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_state->correction_parameter_x = float_value;
      }


      else if (!strcmp(command, "robot_state->correction_parameter_y")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	 robot_state->correction_parameter_y  = float_value;
      }


      else if (!strcmp(command,
		       "robot_state->correction_parameter_angle")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	 robot_state->correction_parameter_angle  = float_value;
      }


      else if (!strcmp(command, "robot_state->correction_type")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	 robot_state->correction_type  = int_value;
      }

      else if (!strcmp(command, "robot_state->map_orientation_defined")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	 robot_state->map_orientation_defined  = int_value;
      }


      else if (!strcmp(command,
		       "robot_state->map_orientation")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	 robot_state->map_orientation  = float_value;
      }


#ifdef OLD
      else if (!strncmp(command, "robot_state->map_orientation_defined",
			strlen("robot_state->map_orientation_defined"))){
	cmd_ptr = command + strlen("robot_state->map_orientation_defined");
	if (*cmd_ptr++ != '['){
	  fprintf(stderr, "Error: robot_state->map_orientation_defined ");
	  fprintf(stderr, "must be followed by a [number].\n");
	  error = 1;
	}
	else{
	  for (i = 0; i < (int) strlen(cmd_ptr); i++)
	    if (cmd_ptr[i] == ']')
	      cmd_ptr[i] = ' ';
	  int_value1 = atoi(cmd_ptr);
	  if (int_value1 < 0 || int_value1 >= NUM_GLOBAL_MAPS){
	    fprintf(stderr, "Error: index %d of ", int_value1);
	    fprintf(stderr, "robot_state->map_orientation ");
	    fprintf(stderr, "must be in [0,%d].\n", NUM_GLOBAL_MAPS);
	    error = 1;
	  }
	  else /* if (!init) */
	    if (fscanf(iop, "%d", &int_value2) == EOF)
	      file_ended = 1;
	    else
	      robot_state->map_orientation_defined[int_value1] = int_value2;
	}
	
      }

   
      else if (!strncmp(command, "robot_state->map_orientation",
			strlen("robot_state->map_orientation"))){
	cmd_ptr = command + strlen("robot_state->map_orientation");
	if (*cmd_ptr++ != '['){
	  fprintf(stderr, "Error: robot_state->map_orientation ");
	  fprintf(stderr, "must be followed by a [number].\n");
	  error = 1;
	}
	else{
	  for (i = 0; i < (int)strlen(cmd_ptr); i++)
	    if (cmd_ptr[i] == ']')
	      cmd_ptr[i] = ' ';
	  int_value1 = atoi(cmd_ptr);
	  if (int_value1 < 0 || int_value1 >= NUM_GLOBAL_MAPS){
	    fprintf(stderr, "Error: index %d of ", int_value1);
	    fprintf(stderr, "robot_state->map_orientation ");
	    fprintf(stderr, "must be in [0,%d].\n", NUM_GLOBAL_MAPS);
	    error = 1;
	  }
	  else /* if (!init) */
	    if (fscanf(iop, "%g", &float_value) == EOF)
	      file_ended = 1;
	    else
	      robot_state->map_orientation[int_value1] = float_value;
	}
	
      }
#endif
      

      else if (!strcmp(command, "robot_specifications->global_mapsize_x")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init){
	  robot_specifications->global_mapsize_x = float_value;
	  robot_specifications->global_map_dim_x = 
	    (int) (robot_specifications->global_mapsize_x
		   / robot_specifications->resolution);
	}
	else if (float_value != robot_specifications->global_mapsize_x){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->global_mapsize_x");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->global_mapsize_y")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init){
	  robot_specifications->global_mapsize_y = float_value;
	  robot_specifications->global_map_dim_y = 
	    (int) (robot_specifications->global_mapsize_y
		   / robot_specifications->resolution);
	}
	else if (float_value != robot_specifications->global_mapsize_y){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->global_mapsize_y");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      
      else if (!strcmp(command, "robot_specifications->resolution")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init){
	  robot_specifications->resolution = float_value;
	  robot_specifications->global_map_dim_x = 
	    (int) (robot_specifications->global_mapsize_x
		   / robot_specifications->resolution);
	  robot_specifications->global_map_dim_y = 
	    (int) (robot_specifications->global_mapsize_y
		   / robot_specifications->resolution);
	}
	else if (float_value != robot_specifications->resolution){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->resolution");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->robot_size")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->robot_size = float_value;
	else if (float_value != robot_specifications->robot_size){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->robot_size");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
    
      else if (!strcmp(command, "robot_specifications->drift")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->drift = float_value;
	else if (float_value != robot_specifications->drift){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->drift");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
    
      else if (!strcmp(command, "robot_specifications->smooth_radius")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->smooth_radius = int_value;
	else if (int_value != robot_specifications->smooth_radius){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->smooth_radius");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->autoshift")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->autoshift = int_value;
	else if (int_value != robot_specifications->autoshift){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->autoshift");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->autoshift_distance")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->autoshift_distance = float_value;
	else if (float_value != robot_specifications->autoshift_distance){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->autoshift_distance");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command,
		       "robot_specifications->autoshift_safety_margin")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->autoshift_distance = float_value;
	else if (float_value != robot_specifications->autoshift_safety_margin){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->autoshift_safety_margin");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->autoshifted_x")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */{
	  if (program_state->graphics_initialized){
	    G_shift_robot_local_coordinates(GLOBAL_ROBOT, 
					    float_value
					    - robot_specifications->
					    autoshifted_x, 0.0);
	    G_shift_robot_local_coordinates(BEST_FIT_POINT, 
					    float_value
					    - robot_specifications->
					    autoshifted_x, 0.0);
	    G_shift_markers_local_coordinates(PATH, 
					      float_value
					      - robot_specifications->
					      autoshifted_x, 0.0);
	    G_shift_markers_local_coordinates(TOPOLOGICAL_GRAPH,
					      float_value
					      - robot_specifications->
					      autoshifted_x, 0.0);
	  }
	  robot_specifications->autoshifted_x = float_value;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->autoshifted_y")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */{
	  if (program_state->graphics_initialized){
	    G_shift_robot_local_coordinates(GLOBAL_ROBOT, 0.0, 
					    float_value
					    - robot_specifications->
					    autoshifted_y);
	    G_shift_robot_local_coordinates(BEST_FIT_POINT, 0.0, 
					    float_value
					    - robot_specifications->
					    autoshifted_y);
	    G_shift_markers_local_coordinates(PATH, 0.0, 
					      float_value
					      - robot_specifications->
					      autoshifted_y);
	    G_shift_markers_local_coordinates(TOPOLOGICAL_GRAPH, 0.0, 
					      float_value
					      - robot_specifications->
					      autoshifted_y);
	  }
	  robot_specifications->autoshifted_y = float_value;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->autoshifted_int_x")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->autoshifted_int_x = int_value;
      }
      
      
      else if (!strcmp(command, "robot_specifications->autoshifted_int_y")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->autoshifted_int_y = int_value;
      }
      
      
      else if (!strcmp(command,
		       "robot_specifications->do_position_correction")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->do_position_correction = int_value;
      }


      else if (!strcmp(command,
		       "robot_specifications->max_distance_in_match")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->max_distance_in_match = float_value;
      }


      else if (!strcmp(command, "robot_specifications->map_fit_norm_L2")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->map_fit_norm_L2 = int_value;
      }


      else if (!strcmp(command, "robot_specifications->prev_pos_norm_L2")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->prev_pos_norm_L2 = int_value;
      }


      else if (!strcmp(command,
		       "robot_specifications->max_niterations_in_search")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->max_niterations_in_search = int_value;
      }


      else if (!strcmp(command,
		       "robot_specifications->niterations_in_map_fitting")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->niterations_in_map_fitting = int_value;
      }


      else if (!strcmp(command,
		       "robot_specifications->max_translation_in_search")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->max_translation_in_search = float_value;
      }


      else if (!strcmp(command, "robot_specifications->translation_weight_fit")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->translation_weight_fit = float_value;
      }


      else if (!strcmp(command, "robot_specifications->translation_weight_fit_global_match")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->translation_weight_fit_global_match = 
	    float_value;
      }


      else if (!strcmp(command, "robot_specifications->translation_weight_prev_position")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->translation_weight_prev_position = float_value;
      }


      else if (!strcmp(command, "robot_specifications->translation_stepsize")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->translation_stepsize = float_value;
      }


      else if (!strcmp(command, "robot_specifications->translation_momentum")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->translation_momentum = float_value;
      }


      else if (!strcmp(command, "robot_specifications->max_rotation_in_search")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->max_rotation_in_search = float_value;
      }


      else if (!strcmp(command, "robot_specifications->rotation_weight_fit")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->rotation_weight_fit = float_value;
      }


      else if (!strcmp(command, "robot_specifications->rotation_weight_fit_global_match")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->rotation_weight_fit_global_match = float_value;
      }


      else if (!strcmp(command, "robot_specifications->rotation_weight_prev_position")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->rotation_weight_prev_position = float_value;
      }


      else if (!strcmp(command, "robot_specifications->rotation_stepsize")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->rotation_stepsize = float_value;
      }


      else if (!strcmp(command, "robot_specifications->rotation_momentum")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->rotation_momentum = float_value;
      }


      else if (!strcmp(command, "robot_specifications->search_granularity")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->search_granularity = int_value;
      }

      else if (!strcmp(command, "robot_specifications->do_path_fitting")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->do_path_fitting = int_value;
      }

      
      else if (!strcmp(command, "robot_specifications->weight_path_fit")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->weight_path_fit = float_value;
      }

      
      else if (!strcmp(command, "robot_specifications->n_path_points_in_fit")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->n_path_points_in_fit = int_value;
      }

      
      else if (!strcmp(command, "robot_specifications->wall_error_threshold")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->wall_error_threshold = float_value;
      }

      else if (!strcmp(command, "robot_specifications->wall_weight")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->wall_weight = float_value;
      }

      else if (!strcmp(command, 
		       "robot_specifications->number_subsequent_adjacent_walls")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->number_subsequent_adjacent_walls = int_value;
      }

      
      else if (!strcmp(command, "robot_specifications->min_advance_for_map_fitting")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->min_advance_for_map_fitting = float_value;
      }

      else if (!strcmp(command, "robot_specifications->decay_old")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->decay_old = float_value;
      }

      
      else if (!strcmp(command, "robot_specifications->decay_new")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->decay_new = float_value;
      }

      else if (!strcmp(command, "robot_specifications->prior")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->prior = float_value;
      }

      else if (!strcmp(command, 
		       "robot_specifications->update_extreme_likelihoods")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->update_extreme_likelihoods = int_value;
      }

      else if (!strcmp(command, "robot_specifications->lower_clipping_value")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->lower_clipping_value = float_value;
      }

      else if (!strcmp(command, "robot_specifications->upper_clipping_value")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->upper_clipping_value = float_value;
      }

      else if (!strcmp(command, 
		       "robot_specifications->reposition_robot_initially")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->reposition_robot_initially = int_value;
      }

      
      else if (!strcmp(command, 
		       "robot_specifications->regular_gif_output_in_sec")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->regular_gif_output_in_sec = int_value;
      }

      else if (!strcmp(command, "robot_specifications->X_window_size")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->X_window_size = float_value;
      }

      else if (!strcmp(command, 
		       "robot_specifications->data_logging")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->data_logging = int_value;
      }

      else if (!strcmp(command, 
		       "robot_specifications->map_erasing_period")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->map_erasing_period = int_value;
      }

      

      else if (!strcmp(command, "path:")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else if (int_value > MAX_N_PATH_ENTRIES){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "path (param. n_path_entries[])");
	  fprintf(stderr, " exceeds internal specifications.\n\n");
	  error = 1;
	}
	else{
	  /* if (!init) */
	  n_path_entries = int_value;
	  if (program_state->graphics_initialized)
	    G_clear_markers(PATH);
	  for (i = 0; i < int_value; i++){
	    if (fscanf(iop, "%g", &float_value) == EOF)
	      file_ended = 1;
	    else{
	      /* if (!init) */
	      path[i][0] = float_value;
	      if (fscanf(iop, "%g", &float_value) == EOF)
		file_ended = 1;
	      else{
		/* if (!init) */
		path[i][1] = float_value;
		if (fscanf(iop, "%g", &float_value) == EOF)
		  file_ended = 1;
		else{ /* if (!init) */
		  path[i][2] = float_value;
		  if (program_state->graphics_initialized)
		    G_add_marker(PATH, path[i][0],
				 path[i][1], 0);
		}
	      }
	    }
	  }
	}
      }
     
      else if (!strncmp(command, "global_map", strlen("global_map"))){
	cmd_ptr = command + strlen("global_map");
	if (*cmd_ptr++ != '['){
	  fprintf(stderr, "Error: global_map ");
	  fprintf(stderr, "must be followed by a [number].\n");
	  error = 1;
	}
	else{
	  for (i = 0; i < (int)strlen(cmd_ptr); i++)
	    if (cmd_ptr[i] == ']' || cmd_ptr[i] == ':')
	      cmd_ptr[i] = ' ';
	  int_value3 = atoi(cmd_ptr);
	  if (int_value3 < 0 || int_value3 >= NUM_GLOBAL_MAPS){
	    fprintf(stderr, "Error: index %d of ", int_value3);
	    fprintf(stderr, "global_map ");
	    fprintf(stderr, "must be in [0,%d].\n", NUM_GLOBAL_MAPS);
	    error = 1;
	  }
	  else /* if (!init) */
	    if (fscanf(iop, "%d", &int_value1) == EOF)
	      file_ended = 1;
	    else if (!init &&
		     int_value1 != robot_specifications->global_map_dim_x){
	      fprintf(stderr, "Error when parsing %s: value for ", filename2);
	      fprintf(stderr, "map (param. ");
	      fprintf(stderr, "robot_specifications->global_map_dim_x)");
	      fprintf(stderr, " does not match internal specifications.\n\n");
	      error = 1;
	    }
	    else if (fscanf(iop, "%d", &int_value2) == EOF)
	      file_ended = 1;
	    else if (!init &&
		     int_value2 != robot_specifications->global_map_dim_y){
	      fprintf(stderr, "Error when parsing %s: value for ", filename2);
	      fprintf(stderr, "map (param. ");
	      fprintf(stderr, "robot_specifications->global_map_dim_y)");
	      fprintf(stderr, " does not match internal specifications.\n\n");
	      error = 1;
	    }
	    else{
	      /* init borders */
	      robot_specifications->min_display_index_x[int_value3]  = 
		robot_specifications->global_map_dim_x + 1;
	      robot_specifications->max_display_index_x[int_value3]  =  -1;
	      robot_specifications->min_display_index_y[int_value3]  = 
		robot_specifications->global_map_dim_x + 1;
	      robot_specifications->max_display_index_y[int_value3]  =  -1;

	      for (x = 0; x < int_value1; x++)
		for (y = 0; y < int_value2; y++){
		  if (fscanf(iop, "%g", &float_value) == EOF)
		    file_ended = 1;
		  else if (!init){
		    index = x * robot_specifications->global_map_dim_y + y;
		    global_label_x[int_value3][index]    = 0;
		    if (float_value == -1.0){
		      global_map_x[int_value3][index]    = 0.5;
		      global_active_x[int_value3][index] = 0;
		    }
		    else{
		      global_map_x[int_value3][index]    = float_value;
		      global_active_x[int_value3][index] = 1;
		      if (float_value == -1)
			global_label_x[int_value3][index] = 1;
		      if (robot_specifications->min_display_index_x[int_value3] > x)
			robot_specifications->min_display_index_x[int_value3] = x;
		      if (robot_specifications->min_display_index_y[int_value3] > y)
			robot_specifications->min_display_index_y[int_value3] = y;
		      if (robot_specifications->max_display_index_x[int_value3] < x)
			robot_specifications->max_display_index_x[int_value3] = x;
		      if (robot_specifications->max_display_index_y[int_value3] < y)
			robot_specifications->max_display_index_y[int_value3] = y;
		      
		    }
		  }
		}
	    }
	}
      }
      
      else{
	fprintf(stderr, "ERROR: Unknown keyword \"%s\" in %s. Must exit.\n ", 
		command, filename2);
	error = 1;
      }
 
      if (file_ended == 1)
	fprintf(stderr, "Surprising end of file %s.\n ", filename2);
      
    }
  } while (!file_ended);
  fclose(iop);
  
  if (!error && !init && program_state->graphics_initialized){
    set_map(program_state->actual_map);
    G_display_switch(DISPLAY_LOCAL_MAPVALUES_BUTTON, 2);
    G_display_switch(DISPLAY_GLOBAL_MAPVALUES_BUTTON, 2);
    
    G_activate(LOCAL_MAPVALUES);
    G_display_matrix(LOCAL_MAPVALUES);
    if (!program_state->regular_local_map_display)
      G_deactivate(LOCAL_MAPVALUES);
    
    G_activate(GLOBAL_MAPVALUES);
    G_display_matrix(GLOBAL_MAPVALUES);
    G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y,
		    robot_state->orientation, 0, NULL);
    if (!program_state->regular_global_map_display)
      G_deactivate(GLOBAL_MAPVALUES);
    
    G_display_switch(DISPLAY_LOCAL_MAPVALUES_BUTTON, 
		     program_state->regular_local_map_display);
    G_display_switch(DISPLAY_GLOBAL_MAPVALUES_BUTTON, 
		     program_state->regular_global_map_display);
  }

  if (!error){
    fprintf(stderr, "File %s successfully read.\n", filename2);
    program_state->first_base_report_received = 0;
  }

  return (1 - error);
}


