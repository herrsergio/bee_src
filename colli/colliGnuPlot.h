
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



/* This module is used to open and close gnuplot files. */

void
computeGnuPlotTrajectory(Point rpos, float rrot);

void
COLLI_DumpEvaluationFunction(  VelocityCombination** combinations,
			       float** Velocities, float** Distances,
			       float** Angles, 
			       float velFactor,
			       float distFactor, float angFactor,
			       int smooth,
			       int numberOfRvels, int numberOfTvels);

