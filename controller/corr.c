
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








#include <ctype.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include "all.h"





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
				float *corr_orientation)
{
  float delta_x, delta_y;
  float angle;

  if (corr_type){		/* 1=rotation about corr_x|corr_y */
    delta_x = robot_x - corr_x;	/* difference to center of rotation */
    delta_y = robot_y - corr_y;	/* difference to center of rotation */
    angle = corr_angle / 180.0 * M_PI;
    *corr_robot_x = corr_x + (delta_x * cos(angle)) - (delta_y * sin(angle));
    *corr_robot_y = corr_y + (delta_x * sin(angle)) + (delta_y * cos(angle));
    *corr_orientation = robot_orientation + corr_angle;
    for (;*corr_orientation >  180.0;) *corr_orientation -= 360.0;
    for (;*corr_orientation < -180.0;) *corr_orientation += 360.0;
  }

  else{				/* otherwise: just translation */
    *corr_robot_x = robot_x + corr_x;
    *corr_robot_y = robot_y + corr_y;
    *corr_orientation = robot_orientation + corr_angle;
  }
}


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
				 float *orientation)
{
  if (corr_type == 1)
    compute_forward_correction(corr_robot_x, corr_robot_y, 
			       corr_robot_orientation,
			       corr_x, corr_y, -corr_angle, corr_type,
			       robot_x, robot_y, orientation);
  else
    compute_forward_correction(corr_robot_x, corr_robot_y, 
			       corr_robot_orientation,
			       -corr_x, -corr_y, -corr_angle, corr_type,
			       robot_x, robot_y, orientation);
}



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
				   int   *corr_type)
{
  float distance;
  float delta_x, delta_y, center_x, center_y;
  float alpha, tan_alpha_half, length;

  delta_x = corr_robot_x - robot_x;	/* difference between both robots */
  delta_y = corr_robot_y - robot_y;	/* difference between both robots */
  distance = sqrt((delta_x * delta_x) + (delta_y * delta_y));
  center_x = 0.5 * (robot_x + corr_robot_x);
  center_y = 0.5 * (robot_y + corr_robot_y);
  alpha = corr_orientation - robot_orientation;
  for (;alpha >  180.0;) alpha -= 360.0;
  for (;alpha < -180.0;) alpha += 360.0;
  *corr_angle = alpha;		/* this is simple. Rotational error. */
    
  if (alpha >= 0.0){		/* rotation counterclockwise */
    tan_alpha_half = tan(alpha / 360.0 * M_PI);	/* tan of 
						 * angle between center of 
						 * circle and robot and
						 * center of circle and
						 * half-way between robots */
    if (tan_alpha_half < 0.0001 * distance)
      *corr_type = 0;		/* translation, see below */
    else{
      *corr_type = 1;		/* rotation */
      if (distance > 0.0){
	length = 0.5 * distance / tan_alpha_half;
	*corr_x = center_x - (length * delta_y / distance);
	*corr_y = center_y + (length * delta_x / distance);
      }
      else{
	*corr_x = center_x;
	*corr_y = center_y;
      }
    }

  }
  else{				/* alpha < 0, rotation clockwise */
    tan_alpha_half = tan((0.0 - alpha)
			 / 360.0 * M_PI); /* tan of 
					   * angle between center of 
					   * circle and robot and
					   * center of circle and
					   * half-way between robots */
    if (tan_alpha_half < 0.0001 * distance)
      *corr_type = 0;		/* translation, see below */
    else{
      *corr_type = 1;		/* rotation */
      if (distance > 0.0){
	length = 0.5 * distance / tan_alpha_half;
	*corr_x = center_x + (length * delta_y / distance);
	*corr_y = center_y - (length * delta_x / distance);
      }
      else{
	*corr_x = center_x;
	*corr_y = center_y;
      }
    }
  }
      
  if (*corr_type == 0){
    *corr_x = delta_x;
    *corr_y = delta_y;
  }
}    




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
				  int   *corr_type)
{
  float robot_x, robot_y, robot_orientation;
  float corr_robot_x2, corr_robot_y2, corr_robot_orientation2;
  

  compute_backward_correction(corr_robot_x, corr_robot_y, 
			      corr_robot_orientation,
			      *corr_x, *corr_y, *corr_angle, *corr_type,
			      &robot_x, &robot_y,  &robot_orientation);
  
  corr_robot_x2 = corr_robot_x + delta_x;
  corr_robot_y2 = corr_robot_y + delta_y;
  corr_robot_orientation2 = corr_robot_orientation + delta_orientation;

  
  compute_correction_parameters(robot_x, robot_y, robot_orientation,
				corr_robot_x2, corr_robot_y2, 
				corr_robot_orientation2,
				corr_x, corr_y, corr_angle, corr_type);
}




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



void test_correction()
{
  int i;

  float robot_x, robot_y, robot_orientation;
  float corr_robot_x, corr_robot_y, corr_robot_orientation;
  float test_robot_x, test_robot_y, test_robot_orientation;
  float corr_x, corr_y, corr_angle;
  int   corr_type;
  float change_x, change_y, change_orientation;
  float error, deviation;

  for (i = 0; i < 1000; i++){
    robot_x =                (float) ((int) (RAND() * 500.0));
    robot_y =                (float) ((int) (RAND() * 500.0));
    robot_orientation =      (float) ((int) (RAND() * 180.0));
    corr_robot_x =           (float) ((int) (RAND() * 500.0));
    corr_robot_y =           (float) ((int) (RAND() * 500.0));
    corr_robot_orientation = (float) ((int) (RAND() * 180.0));

    if (i == 0){
      robot_x =                10.0;
      robot_y =                10.0;
      robot_orientation =      30.0;
      corr_robot_x =           -10.0;
      corr_robot_y =           10.0;
      corr_robot_orientation = 90.0;
    }
    

    compute_correction_parameters(robot_x, robot_y, robot_orientation,
				  corr_robot_x, corr_robot_y, 
				  corr_robot_orientation,
				  &corr_x, &corr_y, &corr_angle, &corr_type);

    compute_forward_correction(robot_x, robot_y, robot_orientation,
			       corr_x, corr_y, corr_angle, corr_type,
			       &test_robot_x, &test_robot_y, 
			       &test_robot_orientation);


    deviation = fabs(test_robot_x - corr_robot_x) 
      + fabs(test_robot_y - corr_robot_y) 
	+ fabs(test_robot_orientation - corr_robot_orientation);

    if (deviation > 0.01)

      printf("(1)\trobot_x=%g\n\trobot_y=%g\n\trobot_orientation=%g\n\tcorr_robot_x=%g\n\tcorr_robot_y=%g\n\tcorr_robot_orientation=%g\n\ttest_robot_x=%g\n\ttest_robot_y=%g\n\ttest_robot_orientation=%g\n\tcorr_x=%g\n\tcorr_y=%g\n\tcorr_angle=%g\n\tcorr_type=%d\nError: %g\n\n", robot_x, robot_y, robot_orientation, corr_robot_x, corr_robot_y, corr_robot_orientation, test_robot_x, test_robot_y, test_robot_orientation, corr_x, corr_y, corr_angle, corr_type, deviation);


  }




  for (i = 0; i < 1000; i++){
    corr_robot_x =           (float) ((int) (RAND() * 500.0));
    corr_robot_y =           (float) ((int) (RAND() * 500.0));
    corr_robot_orientation = (float) ((int) (RAND() * 180.0));
    corr_x =                 (float) ((int) (RAND() * 500.0));
    corr_y =                 (float) ((int) (RAND() * 500.0));
    corr_angle =             (float) ((int) (RAND() * 180.0));
    corr_type =              (RAND() >= 0.0);

    compute_backward_correction(corr_robot_x, corr_robot_y, 
				corr_robot_orientation,
				corr_x, corr_y, corr_angle, corr_type,
				&robot_x, &robot_y,  &robot_orientation);
    
    compute_forward_correction(robot_x, robot_y, robot_orientation,
			       corr_x, corr_y, corr_angle, corr_type,
			       &test_robot_x, &test_robot_y, 
			       &test_robot_orientation);



    deviation = fabs(test_robot_x - corr_robot_x) 
      + fabs(test_robot_y - corr_robot_y) 
	+ fabs(test_robot_orientation - corr_robot_orientation);

    if (deviation > 0.01)

      printf("(2)\trobot_x=%g\n\trobot_y=%g\n\trobot_orientation=%g\n\tcorr_robot_x=%g\n\tcorr_robot_y=%g\n\tcorr_robot_orientation=%g\n\ttest_robot_x=%g\n\ttest_robot_y=%g\n\ttest_robot_orientation=%g\n\tcorr_x=%g\n\tcorr_y=%g\n\tcorr_angle=%g\n\tcorr_type=%d\nError: %g\n\n", robot_x, robot_y, robot_orientation, corr_robot_x, corr_robot_y, corr_robot_orientation, test_robot_x, test_robot_y, test_robot_orientation, corr_x, corr_y, corr_angle, corr_type, deviation);




  }





  for (i = 0; i < 2; i++){
    corr_robot_x =           (float) ((int) (RAND() * 500.0));
    corr_robot_y =           (float) ((int) (RAND() * 500.0));
    corr_robot_orientation = (float) ((int) (RAND() * 180.0));
    corr_x =                 (float) ((int) (RAND() * 500.0));
    corr_y =                 (float) ((int) (RAND() * 500.0));
    corr_angle =             (float) ((int) (RAND() * 180.0));
    corr_type =              (RAND() >= 0.0);
    change_x               = (float) ((int) (RAND() * 100.0));
    change_y               = (float) ((int) (RAND() * 100.0));
    change_orientation     = (float) ((int) (RAND() * 100.0));


    if (i < 1000){
      corr_robot_x           = 0.0;
      corr_robot_y           = 0.0;
      corr_robot_orientation = 0.0;
      corr_x                 = 10.0;
      corr_y                 = 0.0;
      corr_angle             = 0.0;
      corr_type              = i;
      change_x               = 0.0;
      change_y               = 0.0;
      change_orientation     = 0.0;
    }

    compute_backward_correction(corr_robot_x, corr_robot_y, 
				corr_robot_orientation,
				corr_x, corr_y, corr_angle, corr_type,
				&robot_x, &robot_y,  &robot_orientation);
    

    update_correction_parameters(robot_x, robot_y, robot_orientation,
				 change_x, change_y, change_orientation,
				 &corr_x, &corr_y, &corr_angle, &corr_type);


    compute_forward_correction(robot_x, robot_y, robot_orientation,
			       corr_x, corr_y, corr_angle, corr_type,
			       &test_robot_x, &test_robot_y, 
			       &test_robot_orientation);


    corr_robot_x += change_x;
    corr_robot_y += change_y;
    corr_robot_orientation += change_orientation;
    for (;corr_robot_orientation >  180.0;) corr_robot_orientation -= 360.0;
    for (;corr_robot_orientation < -180.0;) corr_robot_orientation += 360.0;



    deviation = fabs(test_robot_x - corr_robot_x) 
      + fabs(test_robot_y - corr_robot_y) 
	+ fabs(test_robot_orientation - corr_robot_orientation);

    if (deviation > 0.01)



      printf("(3)\tchange_x=%g\n\tchange_y=%g\n\tchange_orientation=%g\n\trobot_x=%g\n\trobot_y=%g\n\trobot_orientation=%g\n\tcorr_robot_x=%g\n\tcorr_robot_y=%g\n\tcorr_robot_orientation=%g\n\ttest_robot_x=%g\n\ttest_robot_y=%g\n\ttest_robot_orientation=%g\n\tcorr_x=%g\n\tcorr_y=%g\n\tcorr_angle=%g\n\tcorr_type=%d\nError: %g\n\n", change_x, change_y, change_orientation,  robot_x, robot_y, robot_orientation, corr_robot_x, corr_robot_y, corr_robot_orientation, test_robot_x, test_robot_y, test_robot_orientation, corr_x, corr_y, corr_angle, corr_type, deviation);


  }

}
