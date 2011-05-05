
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
#include "collisionIntern.h"


static int dumpStep = 0;

FILE* angleFile;
FILE* velocityFile;
FILE* distanceFile;

#define angFile "angles"
#define velFile "velocities"
#define distFile "distances"

#define DUMP_ANGLES 1
#define DUMP_VELOCITIES 1
#define DUMP_DISTANCES 1


static void
openDumpFiles( int cnt)
{
   char fName[80];
   
   if (DUMP_ANGLES) {
      sprintf( fName, "%s.dump.%d", angFile, cnt);
      angleFile    = fopen( fName, "w"); 
   }
   
   if (DUMP_VELOCITIES) {
      sprintf( fName, "%s.dump.%d", velFile, cnt); 
      velocityFile    = fopen( fName, "w");
   }
   
   if (DUMP_DISTANCES) {
      sprintf( fName, "%s.dump.%d", distFile, cnt); 
      distanceFile    = fopen( fName, "w");
   }
   
}

    
static void
closeFiles()
{
    if (DUMP_ANGLES) 
       fclose( angleFile); 
    if (DUMP_VELOCITIES) 
       fclose( velocityFile); 
    if (DUMP_DISTANCES) 
       fclose( distanceFile); 
}


static void
dump( VelocityCombination** combinations,
      float** values, float factor, int smooth, FILE* file,
      int numberOfRvels, int numberOfTvels )
{
   int i,j;

   fprintf( file, "%d %d %f %d\n", numberOfRvels, numberOfTvels, factor, smooth);
   
   for ( i = 0; i < numberOfRvels; i++) {
      
      for ( j = 0; j < numberOfTvels; j++) {
	 
	 if ( values[i][j] == UNDEFINED) 
	    fprintf( file, "%f\t%f\t0.0\n", 
		    RAD_TO_DEG( combinations[i][j].rvel), 
		    combinations[i][j].tvel);
	 else
	    fprintf( file, "%f\t%f\t%f\n", 
		    RAD_TO_DEG( combinations[i][j].rvel), 
		    combinations[i][j].tvel, values[i][j]); 
      }
   } 
}

void COLLI_StartToDumpGnuPlot( double step)
{
   dumpStep = (int) step;
}



void
COLLI_DumpEvaluationFunction( VelocityCombination** combinations,
			      float** Velocities, float** Distances,
			      float** Angles,
			      float velFactor,
			      float distFactor, float angFactor,
			      int smooth,
			      int numberOfRvels, int numberOfTvels)
{
   if ( dumpStep > 0) {
      
      static int cnt = 0;
      
      if (++cnt % dumpStep == 0) {
	 
	 openDumpFiles( cnt);
	 
	 if (DUMP_ANGLES) 
	    dump( combinations, Angles, angFactor, smooth,
		  angleFile,
		  numberOfRvels, numberOfTvels);
	 if (DUMP_VELOCITIES) 
	     dump( combinations, Velocities, velFactor, smooth,
		   velocityFile,
		   numberOfRvels, numberOfTvels);
	 if (DUMP_DISTANCES) 
	     dump( combinations, Distances, distFactor, smooth,
		   distanceFile,
		   numberOfRvels, numberOfTvels);
	 closeFiles();
      }
   }

}

#ifdef GNUPLOT

#define NUMBER_OF_GNUPLOT_RVELS 50
#define NUMBER_OF_GNUPLOT_TVELS 30

#define MIN_TVEL 0.0
#define MAX_TVEL 90.0
#define MIN_RVEL -DEG_90
#define MAX_RVEL DEG_90

#define MAX_VEL 90.0
#define MAX_ANG DEG_180
#define MAX_DIST MAX_RANGE



static void
generateGnuPlotVelocities( VelocityCombination** combinations,
			   float minR, float maxR,
			   float minT, float maxT,
			   int numberOfRvels,
			   int numberOfTvels)
{

    int i, j;

    float tvelStep = (maxT - minT) / (numberOfTvels - 1.0);
    float rvelStep = (maxR - minR) / (numberOfRvels - 1.0);

    for ( i = 0; i < numberOfRvels; i++) {
	for ( j = 0; j < numberOfTvels; j++) {
	    combinations[i][j].tvel = minT + j * tvelStep;
	    combinations[i][j].rvel = minR + i * rvelStep;
	}
    }
}
		    
		
/**********************************************************************
 * Evaluates the different combinations of tvel and rvel.
 * Stores the values in the three matrices.
 **********************************************************************/
static void
evaluateGnuPlotCombinations( Point rpos, float rrot,
			     VelocityCombination** combinations,
			     float** velocities,
			     float** distances,
			     float** angles,
			     int iDim,
			     int jDim)
{
   int i, j;
   float collDist, targetDist;

   /* We don't want the robot to move very slow. Thats why we set some combinations
    * to undefined. To determine the slow combinations we set the fraction of
    * velocities to be deleted. */

   for ( i = 0; i < iDim; i++) {
      
      for ( j = 0; j < jDim; j++) {
	 
	 float tvel = combinations[i][j].tvel;
	 float rvel = combinations[i][j].rvel;
	 
	 if ( ! admissible( rpos,
			   rrot, 
			   tvel, 
			   rvel,
			   &collDist,
			   &targetDist)) {
	    velocities[i][j] = UNDEFINED;
	    angles[i][j]     = UNDEFINED;
	    distances[i][j]  = UNDEFINED;
	 }
	 else 
	 {
	    velocities[i][j] = velocityEvaluation( tvel) / MAX_VEL;

	    angles[i][j]     = angleEvaluation( rpos,
					       rrot,
					       tvel,
					       rvel) / MAX_ANG;
	    
	    distances[i][j]  = distanceEvaluation( tvel,
						  rvel,
						  collDist,
						  targetDist) / MAX_DIST;
	    
	 }
      }
   }
}

    
/**********************************************************************
 * Computes the best traectory to the target point. 
 **********************************************************************/
void
computeGnuPlotTrajectory(Point rpos, float rrot)
{
    VelocityCombination** combinations;
    float** angles;
    float** distances;
    float** velocities;
    float** values;
    float** smooth;

    int i;
    
    allocateEvaluations( &velocities, &distances, &angles, &values,
			 NUMBER_OF_GNUPLOT_RVELS, NUMBER_OF_GNUPLOT_TVELS);

    /* Let's create the smooth grid and the combinations by hand. */
    smooth = (float **) malloc(NUMBER_OF_GNUPLOT_RVELS * sizeof(float *));
    for (i = 0; i < NUMBER_OF_GNUPLOT_RVELS; i++) 
       smooth[i] = (float *) malloc( NUMBER_OF_GNUPLOT_TVELS * sizeof(float));
    
    combinations = (VelocityCombination **)
       malloc( NUMBER_OF_GNUPLOT_RVELS * sizeof(VelocityCombination *));
    for (i = 0; i < NUMBER_OF_GNUPLOT_RVELS; i++) 
       combinations[i] = (VelocityCombination *)
	  malloc( NUMBER_OF_GNUPLOT_TVELS * sizeof(VelocityCombination));
    
    
    generateGnuPlotVelocities( combinations,
			       MIN_RVEL, MAX_RVEL,
			       MIN_TVEL, MAX_TVEL,
			       NUMBER_OF_GNUPLOT_RVELS,
			       NUMBER_OF_GNUPLOT_TVELS);
    
    /* Uses the evaluation function for each of the combinations.
     * Also checks wether there is one combination admissible
     * which includes translation. */
    evaluateGnuPlotCombinations( rpos, rrot,
				 combinations,
				 velocities,
				 distances,
				 angles,
				 NUMBER_OF_GNUPLOT_RVELS,
				 NUMBER_OF_GNUPLOT_TVELS);

    dumpStep = 1;
    COLLI_DumpEvaluationFunction( combinations, 
				 velocities, distances,
				 angles,
				 ACTUAL_MODE->velocity_factor,
				 ACTUAL_MODE->distance_factor,
				 ACTUAL_MODE->angle_factor,
				 ACTUAL_MODE->smooth_width,
				 NUMBER_OF_GNUPLOT_RVELS,
				 NUMBER_OF_GNUPLOT_TVELS);

    COLLI_SetMode(1);
    
    /* Uses the evaluation function for each of the combinations.
     * Also checks wether there is one combination admissible
     * which includes translation. */
    evaluateGnuPlotCombinations( rpos, rrot,
				 combinations,
				 velocities,
				 distances,
				 angles,
				 NUMBER_OF_GNUPLOT_RVELS,
				 NUMBER_OF_GNUPLOT_TVELS);

    dumpStep = 1;
    COLLI_DumpEvaluationFunction( combinations, 
				 velocities, distances,
				 angles,
				 ACTUAL_MODE->velocity_factor,
				 ACTUAL_MODE->distance_factor,
				 ACTUAL_MODE->angle_factor,
				 ACTUAL_MODE->smooth_width,
				 NUMBER_OF_GNUPLOT_RVELS,
				 NUMBER_OF_GNUPLOT_TVELS);
    exit(1);
}

#endif




















