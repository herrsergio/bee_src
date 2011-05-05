
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










/************************************************************************
 *
 *   NAME:         compute_forward_correction
 *                 
 *   FUNCTION:     Computes a correction to robot position estimate
 *                 
 *   PARAMETERS:   robot_x, robot_y, robot_orientation  non-corrected
 *                                                      robot pos
 *                 corr_x, corr_y, corr_angle           corretion parameters
 *                 corr_type                            ditto
 *                 *corr_robot_x, *corr_robot_y, *corr_orientation
 *                                                      corrected position
 *
 *   RETURN-VALUE: nonw
 *                 
 ************************************************************************/


void compute_forward_correction(float robot_x, 
				float robot_y, 
				float robot_orientation,
				float corr_x, 
				float corr_y, 
				float corr_angle, /* in deg. */
				int   corr_type,
				float *corr_robot_x,
				float *corr_robot_y, 
				float *corr_orientation);



/************************************************************************
 *
 *   NAME:         compute_backward_correction
 *                 
 *   FUNCTION:     Computes a BASE robot position estimate
 *                 based on a corrected position
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: nonw
 *                 
 ************************************************************************/



void compute_backward_correction(float corr_robot_x, 
				 float corr_robot_y, 
				 float corr_robot_orientation,
				 float corr_x, 
				 float corr_y, 
				 float corr_angle, /* in deg. */
				 int   corr_type,
				 float *robot_x,
				 float *robot_y, 
				 float *orientation);


/************************************************************************
 *
 *   NAME:         compute_correction_parameters
 *                 
 *   FUNCTION:     Computes a set of correction parameters
 *                 from an uncorrected and a corrected position
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: nonw
 *                 
 ************************************************************************/



void compute_correction_parameters(float robot_x, 
				   float robot_y, 
				   float robot_orientation,
				   float corr_robot_x,
				   float corr_robot_y, 
				   float corr_orientation,
				   float *corr_x, 
				   float *corr_y, 
				   float *corr_angle,
				   int   *corr_type);



/************************************************************************
 *
 *   NAME:         update_correction_parameters
 *                 
 *   FUNCTION:     Updates a set of correction parameters
 *                 based on a previous set, and a local change
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: nonw
 *                 
 ************************************************************************/




void update_correction_parameters(float corr_robot_x, 
				  float corr_robot_y, 
				  float corr_robot_orientation,
				  float delta_x,
				  float delta_y,
				  float delta_orientation,
				  float *corr_x, 
				  float *corr_y, 
				  float *corr_angle,
				  int   *corr_type);

/************************************************************************
 *
 *   NAME:         test_correction()
 *                 
 *   FUNCTION:     Just an internal test to check the consistency
 *                 of above procedures. Don't use, until you have
 *                 serious doubts.
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: nonw
 *                 
 ************************************************************************/



void test_correction();
