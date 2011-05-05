
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
#endif

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>

#include "tcx.h"
#include "tcxP.h"
#include "global.h"
#include "EZX11.h"
#include "o-graphics.h"
#include "MAP-messages.h"
#include "SONAR-messages.h"
#include "BASE-messages.h"
#include "MAP.h"
#include "bUtils.h"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


#define MAX_RANDOM 1073741823.5
#define RAND() ((((float) random())/MAX_RANDOM)-1.0)
#define RAND_POS() ((((float) random())/MAX_RANDOM)*0.5)

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define SIG(x) ((x >= 0.0) ? 1.0 : -1.0)

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

struct bParamList * bParamList = NULL;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


int block_wait(struct timeval *timeout, int tcx_initialized,
	       int X_initialized);

struct timeval block_waiting_time = {1, 0};

int read_map_dat = 0;
extern char init_file_name[256];

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

PROGRAM_STATE            program_state_data;
ROBOT_STATE              robot_state_data;
ROBOT_SPECIFICATIONS     robot_specifications_data;
PROGRAM_STATE_PTR        program_state        = &program_state_data;
ROBOT_STATE_PTR          robot_state          = &robot_state_data;
ROBOT_SPECIFICATIONS_PTR robot_specifications = &robot_specifications_data;
ALL_TYPE                 all;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

struct timeval noWaitTime = {0, 0};
int something_happened = 1;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


#ifndef MAX_N_PATH_ENTRIES
#define MAX_N_PATH_ENTRIES 10000
#endif

float path[MAX_N_PATH_ENTRIES][3];
int   n_path_entries;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* MAPS */



float *global_map_x[NUM_GLOBAL_MAPS];
int   *global_active_x[NUM_GLOBAL_MAPS];
unsigned char  *global_label_x[NUM_GLOBAL_MAPS];
float *global_map                        = NULL;
float *global_local_correlations         = NULL;
int   *global_active                     = NULL;
unsigned char  *global_label             = NULL;
float *local_map                         = NULL;
float *local_smooth_map                  = NULL;
int   *local_active                      = NULL;
float *global_voronoi                    = NULL;
int   *global_voronoi_active             = NULL;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




/* label:
   0 = may be updated, nohing is known       (frozen: 0)
   1 = robot path, may not be updated        (frozen: 1)
   2 = robot path, but may be updated        (frozen: 0)
   3 = door, may be updated in a special way (frozen: 0)
   */

int *frozen = NULL;		/* allocated and
				 * assigned in init.c */

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

struct timeval TCX_no_waiting_time = {0, 0};
struct timeval TCX_1_waiting_time  = {10, 0};


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/







/************************************************************************
 *
 *   NAME:         extract_value_from_local_map
 *                 
 *   FUNCTION:     Extracts a single value from a local map.
 *                 
 *   PARAMETERS:   robot_x, robot_y, robot_orientation  Pos of the robot
 *                 query_x, query_y                     Point to query
 *                 
 *   RETURN-PARAM: float *map_value                  MAP-value, if defined
 *                 foat *query_distance        distance query-point, robot
 *                 float *partial__value__wrt__robot_x            gradient
 *                 float *partial__value__wrt__robot_y            gradient
 *                 float *partial__value__wrt__robot_orientation  gradient
 *
 *   RETURN-VALUE: int   *map_active    1, if defined, 0 if not
 *                 
 ************************************************************************/




int extract_value_from_local_map(float robot_x, float robot_y, 
				 float robot_orientation,
				 float query_x, float query_y,
				 float *map_value,
				 float *partial__value__wrt__robot_x,
				 float *partial__value__wrt__robot_y,
				 float *partial__value__wrt__robot_orientation,
				 int   *found_extreme_likelihood,
				 ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  register int i, j, local_int_x, local_int_y, local_index, local_index2;
  register int count_active;
  float local_x,  local_y, angle, delta_x, delta_y;
  float truncation_error_x, truncation_error_y, query_distance;
  float denominator, factor, sin_angle, cos_angle;
  float sin_tot_angle, cos_tot_angle;
  float partial__local_x__wrt__robot_x;
  float partial__local_x__wrt__robot_y;
  float partial__local_x__wrt__robot_orientation;
  float partial__local_y__wrt__robot_x;
  float partial__local_y__wrt__robot_y;
  float partial__local_y__wrt__robot_orientation;
  float partial__factor__wrt__local_x;
  float partial__factor__wrt__local_y;
  float partial__value__wrt__local_x;
  float partial__value__wrt__local_y;
  float total_orientation;
  float extreme_likelihood = -1.0;
  int   n_extreme_likelihoods;

  *found_extreme_likelihood = 0;

  *partial__value__wrt__robot_x  
    = *partial__value__wrt__robot_y 
      = *partial__value__wrt__robot_orientation = 0.0;
  

  /*
   * delta_x/y measures distance from robot to query point in
   * global coordinate system
   */

  delta_x = query_x - robot_x;
  delta_y = query_y - robot_y;
  query_distance = sqrt((delta_x * delta_x) + (delta_y * delta_y));

  /* 
   * angle measures angle between a line connecting query point 
   * robot, and the x-axis in the  local coordinate system of the
   * local map
   */

  if (delta_x == 0.0 && delta_y == 0.0)
    angle =  0.0;
  else
    angle = (atan2(delta_y, delta_x) / M_PI * 180.0);
  total_orientation = 
    robot_orientation - robot_specifications->local_map_origin_orientation;
  angle -= total_orientation;
  sin_angle = query_distance * sin(angle * M_PI / 180.0);
  cos_angle = query_distance * cos(angle * M_PI / 180.0);
  sin_tot_angle = sin(total_orientation  * M_PI / 180.0);
  cos_tot_angle = cos(total_orientation  * M_PI / 180.0);

  /*
   * local_int_x and local_int_y are the corresponding points
   * for the query point in the coord system of the local map
   */

  local_x = (cos_angle + robot_specifications->local_map_origin_x)
    / robot_specifications->resolution; /* in grid points, not in cm */
  local_y = (sin_angle + robot_specifications->local_map_origin_y)
    / robot_specifications->resolution; /* in grid points, not in cm */
  
  local_int_x = (int) local_x; /* truncate! */
  local_int_y = (int) local_y;
  
  local_index = local_int_x 
    * robot_specifications->local_map_dim_y + local_int_y;
  local_index2 = (local_int_x+1)
    * robot_specifications->local_map_dim_y + (local_int_y+1);

  /* 
   * make sure that (a) the query point is inside the local map, and
   * (b) the local map is active at the query point.
   */

  if (local_int_x >= 0 && 
      local_int_x < robot_specifications->local_map_dim_x - 1 &&
      local_int_y >= 0 && 
      local_int_y < robot_specifications->local_map_dim_y - 1 &&

      local_active[local_index] && /* conservative approach: all 4 neighbors */
      local_active[local_index+1]  && /* must be defined (active) */
      local_active[local_index2-1] &&
      local_active[local_index2]){

    /*
     * check, if two ore more of the likelihood values are
     * extreme, meaning that we firmly believe these value
     */

    n_extreme_likelihoods = 0;

    if (local_active[local_index] == 99){
      extreme_likelihood = local_map[local_index]; /* this might not be
						    * quite correct, since
						    * to extreme values that
						    * are opposite might be
						    * next to each other,
						    * and we won't detect
						    * this */
      n_extreme_likelihoods++;
    }
    if (local_active[local_index+1] == 99){
      extreme_likelihood = local_map[local_index+1];
      n_extreme_likelihoods++;
    }
    if (local_active[local_index2-1] == 99){
      extreme_likelihood = local_map[local_index2-1];
      n_extreme_likelihoods++;
    }
    if (local_active[local_index2] == 99){
      extreme_likelihood = local_map[local_index2];
      n_extreme_likelihoods++;
    }
      
    if (n_extreme_likelihoods >= 2)
      *found_extreme_likelihood = 1;
      
    /*
     * compute the truncation error that ocurred when computing the
     * local map index (which is discrete)
     */
    
    truncation_error_x = local_x - ((float) local_int_x); /* rel. trunc.error*/
    truncation_error_y = local_y - ((float) local_int_y);
  
    /*
     *  Compute derivatives for gradient search (derivation see below) 
     */

    partial__local_x__wrt__robot_orientation = 
      M_PI / 180.0 * sin_angle / robot_specifications->resolution;
    partial__local_y__wrt__robot_orientation = 
      -M_PI / 180.0 * cos_angle / robot_specifications->resolution;
    partial__local_x__wrt__robot_x = 
      - cos_tot_angle / robot_specifications->resolution;
    partial__local_y__wrt__robot_x = 
      sin_tot_angle / robot_specifications->resolution;
    partial__local_x__wrt__robot_y = 
      - sin_tot_angle / robot_specifications->resolution;
    partial__local_y__wrt__robot_y = 
      - cos_tot_angle / robot_specifications->resolution;

    
    /*
     * Now interpolate between four nearest neighbors 
     */

    denominator = 0.0;	
    *map_value  = 0.0;
    count_active = 0;
    partial__value__wrt__local_x = 0.0;
    partial__value__wrt__local_y = 0.0;
    
    for (i = local_int_x; i < local_int_x + 2; i++)
      if (i >= 0 && i < robot_specifications->local_map_dim_x)
	for (j = local_int_y; j < local_int_y + 2; j++)
	  if (j >= 0 && j < robot_specifications->local_map_dim_y){
	    local_index = i * robot_specifications->local_map_dim_y + j;
	    if (local_active[local_index]){

	      /* compute interpolation weight, 4-nearest neighbor */
	      
	      if (i == local_int_x){
		if (j == local_int_y){
		  factor = (1.0 - truncation_error_x) 
		    * (1.0 - truncation_error_y);
		  partial__factor__wrt__local_x = - (1.0 - truncation_error_y);
		  partial__factor__wrt__local_y = - (1.0 - truncation_error_x);
		}
		else{
		  factor = (1.0 - truncation_error_x) * truncation_error_y;
		  partial__factor__wrt__local_x = - truncation_error_y;
		  partial__factor__wrt__local_y = (1.0 - truncation_error_x);
		}
	      }
	      else{
		if (j == local_int_y){
		  factor = truncation_error_x * (1.0 - truncation_error_y);
		  partial__factor__wrt__local_x = (1.0 - truncation_error_y);
		  partial__factor__wrt__local_y = - truncation_error_x;
		}
		else{
		  factor = truncation_error_x * truncation_error_y;
		  partial__factor__wrt__local_x = truncation_error_y;
		  partial__factor__wrt__local_y = truncation_error_x;
		}
	      }
	      
	      /* 
	       * if (i == local_int_x)
	       * factor = (1.0 - truncation_error_x);
	       * else
	       * factor = truncation_error_x;
	       * if (j == local_int_y)
	       * factor *= (1.0 - truncation_error_y);
	       * else
	       * factor *= truncation_error_y;
	       */
	      
	      *map_value += factor * local_smooth_map[local_index];
	      denominator += factor;
	      count_active++;
	      partial__value__wrt__local_x += 
		partial__factor__wrt__local_x * local_smooth_map[local_index];
	      partial__value__wrt__local_y +=
		partial__factor__wrt__local_y * local_smooth_map[local_index];
	      
	    }
	    else		/* We've been restrictive and generate
				 * only a guess when all 4 neighbors are
				 * active. */
	      fprintf(stderr, " ###### ERROR ######\n");
	  }
    
    if (*found_extreme_likelihood)
      *map_value = extreme_likelihood;

    else if (count_active == 0 || denominator == 0.0){
      *map_value = 0.5;		/* beyond the circle! */
      return 0;			/* could not generate a value here! */
    }
    
    else if (count_active < 4 && denominator < 1.0){
      *map_value /= denominator; /* necessary consequence of interpolation */
      partial__value__wrt__local_x /= denominator;
      partial__value__wrt__local_y /= denominator;
      fprintf(stderr, " ###### ERROR: %d %g ######\n",
	      count_active, denominator);
    }
    
    *partial__value__wrt__robot_x = 
      (partial__value__wrt__local_x * partial__local_x__wrt__robot_x)
	+ (partial__value__wrt__local_y * partial__local_y__wrt__robot_x);
    *partial__value__wrt__robot_y = 
      (partial__value__wrt__local_x * partial__local_x__wrt__robot_y)
	+ (partial__value__wrt__local_y * partial__local_y__wrt__robot_y);
    *partial__value__wrt__robot_orientation = 
      (partial__value__wrt__local_x * partial__local_x__wrt__robot_orientation)
	+ (partial__value__wrt__local_y 
	   * partial__local_y__wrt__robot_orientation);
    

    return 1;			/* could generate a value here! */
  }
  
  *map_value = 0.5;		/* beyond the circle! */
  return 0;
}
  
  
/***********************************************************************
 * 
 * Derivation of the Derivatives: Using Mathematica to compute:
 * 
 * ----------------------------------------------------------------------
 * 
 * angle[rx_,ry_,ro_,qx_,qy_,offs_,res_,orgx_,orgy_,orgo_]:=
 *   ArcTan[qx-rx,qy-ry] / Pi * 180.0 - (ro - orgo)
 * 
 * dist[rx_,ry_,ro_,qx_,qy_,offs_,res_,orgx_,orgy_,orgo_]:=
 *   Sqrt[((qx-rx)*(qx-rx)+(qy-ry)*(qy-ry))]
 * 
 * localx[rx_,ry_,ro_,qx_,qy_,offs_,res_,orgx_,orgy_,orgo_]:=
 *   (dist[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo] * \
 *    Cos[angle[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo]*Pi/180.0] + orgx) \
 *    / res
 * 
 * localy[rx_,ry_,ro_,qx_,qy_,offs_,res_,orgx_,orgy_,orgo_]:=
 *   (dist[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo] * \
 *   Sin[angle[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo]*Pi/180.0] + orgy) \
 *   / res
 * 
 * 
 * 
 * CForm[D[localx[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo],ro]]
 * 
 * CForm[D[localy[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo],ro]]
 * 
 * 
 * Yields the following partial derivatives:
 * 
 *    0.00555556*Pi*Sqrt(Power(qx - rx,2) + Power(qy - ry,2))*
 *      Sin(0.00555556*Pi*(orgo - ro + 180.*ArcTan(qx - rx,qy - ry)/Pi))/res
 * 
 *    -0.00555556*Pi*Sqrt(Power(qx - rx,2) + Power(qy - ry,2))*
 *      Cos(0.00555556*Pi*(orgo - ro + 180.*ArcTan(qx - rx,qy - ry)/Pi))/res
 * 
 * 
 * Which can be abbreviated as
 * 
 *    d local_x / d orientation =
 *      Pi/180*dist[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo]
 *        *Sin[Pi/180*(angle[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo])]/res
 * 
 *    d local_y / d orientation =
 *      -Pi/180*dist[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo]
 *        *Cos[Pi/180*(angle[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo])]/res
 * 
 * ----------------------------------------------------------------------
 * 
 * ...this is because gxo and gyo is always zero:
 * 
 * fxo[rx_,ry_,ro_,qx_,qy_,offs_,res_,orgx_,orgy_,orgo_]:= \
 * Evaluate[D[localx[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo],ro]]
 * 
 * gxo[rx_,ry_,ro_,qx_,qy_,offs_,res_,orgx_,orgy_,orgo_]:=\
 *  fxo[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo]-Pi/180*\
 *  dist[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo]*Sin[Pi/180*\
 *  (angle[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo])]/res
 * 
 * 
 * 
 * fyo[rx_,ry_,ro_,qx_,qy_,offs_,res_,orgx_,orgy_,orgo_]:= \
 * Evaluate[D[localy[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo],ro]]
 * 
 * gyo[rx_,ry_,ro_,qx_,qy_,offs_,res_,orgx_,orgy_,orgo_]:=\
 *  fyo[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo]+Pi/180*\
 *  dist[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo]*Cos[Pi/180*\
 *  (angle[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo])]/res
 * 
 * 
 * e.g.: N[gxo[21,-6,32,456,-95,65,53,-4,89,-304]]
 *       N[gyo[21,-6,32,456,-95,65,53,-4,89,-304]]
 * 
 * ----------------------------------------------------------------------
 * 
 * The following partial derivatives gxx, gxy, gyx, and gyy
 * 
 * fxx[rx_,ry_,ro_,qx_,qy_,offs_,res_,orgx_,orgy_,orgo_]:=\
 *    Evaluate[D[localx[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo],rx]]
 * gxx[rx_,ry_,ro_,qx_,qy_,offs_,res_,orgx_,orgy_,orgo_]:=\
 *    fxx[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo]+(Cos[(ro-orgo)/180*Pi]/res)
 * 
 * fyx[rx_,ry_,ro_,qx_,qy_,offs_,res_,orgx_,orgy_,orgo_]:=\
 *    Evaluate[D[localy[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo],rx]]
 * gyx[rx_,ry_,ro_,qx_,qy_,offs_,res_,orgx_,orgy_,orgo_]:=\
 *    fyx[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo]-(Sin[(ro-orgo)/180*Pi]/res)
 * 
 * fxy[rx_,ry_,ro_,qx_,qy_,offs_,res_,orgx_,orgy_,orgo_]:=\
 *    Evaluate[D[localx[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo],ry]]
 * gxy[rx_,ry_,ro_,qx_,qy_,offs_,res_,orgx_,orgy_,orgo_]:=\
 *    fxy[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo]+(Sin[(ro-orgo)/180*Pi]/res)
 * 
 * 
 * fyy[rx_,ry_,ro_,qx_,qy_,offs_,res_,orgx_,orgy_,orgo_]:=\
 *    Evaluate[D[localy[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo],ry]]
 * gyy[rx_,ry_,ro_,qx_,qy_,offs_,res_,orgx_,orgy_,orgo_]:=\
 *    fyy[rx,ry,ro,qx,qy,offs,res,orgx,orgy,orgo]+(Cos[(ro-orgo)/180*Pi]/res)
 * 
 * 
 * for example:
 * 
 * 
 * {N[gyy[21,-6,90,456,-95,5,1,-4,89,-304]],
 *  N[gyy[-21,-6,32,456,-95,5,53,-4,89,-304]],
 *  N[gyy[-21,76,32,456,-95,5,53,-4,89,-304]],
 *  N[gyy[-21,76,-42,456,-95,5,53,-4,89,-304]],
 *  N[gyy[-21,76,-42,56,-95,5,53,-4,89,-304]],
 *  N[gyy[-21,76,-42,56,25,5,53,-4,89,-304]],
 *  N[gyy[-21,76,-42,56,25,-2,53,-4,89,-304]],
 *  N[gyy[-21,76,-42,56,25,-2,-23,-4,89,-304]],
 *  N[gyy[-21,76,-42,56,25,-2,-23,992,89,-304]],
 *  N[gyy[-21,76,-42,56,25,-2,-23,992,-879,-304]],
 *  N[gyy[-21,76,-42,56,25,-2,-23,992,-879,-24]]}
 * 
 * 
 * 
 * are always zero, for any argument, which implies that
 * 
 * 
 * 
 * 
 * d local_x / d robot_x =
 *           - cos((orientation-org_orientation)/180*M_PI) / resolution
 * 
 * d local_y / d robot_x = 
 *           + sin((orientation-org_orientation)/180*M_PI) / resolution
 * 
 * d local_x / d robot_y = 
 *           - sin((orientation-org_orientation)/180*M_PI) / resolution
 * 
 * d local_y / d robot_y = 
 *           - cos((orientation-org_orientation)/180*M_PI) / resolution
 * 
 * 
 *************************************************************************/





/************************************************************************
 *
 *   NAME:         compute_bounds
 *                 
 *   FUNCTION:     Computes bounds for search and updates, based on 
 *                 the size and orientation of the local map
 *                 
 ************************************************************************/

void compute_bounds(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		    float robot_x, float robot_y, float robot_orientation,
		    int *from_x, int *to_x, int *from_y, int *to_y)
{
  float total_orientation, sin_tot_orient, cos_tot_orient, Px[4], Py[4];
  float min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
  float globalPx, globalPy;
  int i;

  /*
   * First: compute superset of points to make predictions at 
   *
   * Here we compute the angle of the local coordinate system wrt
   * to the global coordinate system
   */


  total_orientation = 
    robot_orientation - robot_specifications->local_map_origin_orientation;
  sin_tot_orient = sin(total_orientation * M_PI / 180.0);
  cos_tot_orient = cos(total_orientation * M_PI / 180.0);
  

  /* 
   * compute coordinates of the four corners of the local map
   * in a shifted local coordinate system (same orientation, but
   * robot is center of the shifted coordinate system)
   */

  Px[0] = Px[1] = - robot_specifications->local_map_origin_x;
  Px[2] = Px[3] = (((float) (robot_specifications->local_map_dim_x-1))
		   * robot_specifications->resolution)
    - robot_specifications->local_map_origin_x;
  Py[0] = Py[2] = - robot_specifications->local_map_origin_y;
  Py[1] = Py[3] = (((float) (robot_specifications->local_map_dim_y-1))
	     * robot_specifications->resolution)
    - robot_specifications->local_map_origin_y;

  /* 
   * Compute coordinates of the four corners of the local map
   * by standard rotation matrix multiplication, and find min/max.
   */

  for (i = 0; i < 4; i++){
    globalPx = robot_x + 
      (Px[i] * cos_tot_orient) - (Py[i] * sin_tot_orient);
    globalPy = robot_y + 
      (Px[i] * sin_tot_orient) + (Py[i] * cos_tot_orient);

    if (i == 0){
      min_x = max_x = globalPx;
      min_y = max_y = globalPy;
    }
    else{
      if (globalPx < min_x)
	min_x = globalPx;
      if (globalPx > max_x)
	max_x = globalPx;
      if (globalPy < min_y)
	min_y = globalPy;
      if (globalPy > max_y)
	max_y = globalPy;
    }
  }


  /* 
   * Compute coordinated in the global map (in integer)
   */


  *from_x = (int) (min_x / robot_specifications->resolution);
  *to_x   = (int) (max_x / robot_specifications->resolution);
  *from_y = (int) (min_y / robot_specifications->resolution);
  *to_y   = (int) (max_y / robot_specifications->resolution);
}




/************************************************************************
 *
 *   NAME:         update_internal_map
 *                 
 *   FUNCTION:     Updates internal maps from the last sonar reading.
 *                 
 ************************************************************************/


void update_internal_map(int new_map,
			 ROBOT_SPECIFICATIONS_PTR robot_specifications,
			 PROGRAM_STATE_PTR        program_state,
	 		 ROBOT_STATE_PTR          robot_state)
{
  int from_x, to_x, from_y, to_y, x, y, shifted_x, shifted_y, map_index;
  float query_x, query_y;
  float value_prediction;
  float likelihood, odds;
  float partial__value__wrt__robot_x;
  float partial__value__wrt__robot_y;
  float partial__value__wrt__robot_orientation;
  int  min_x, max_x, min_y, max_y;
  int found_extreme_likelihood;
  float *global_map2     = NULL;
  unsigned char *global_label2   = NULL;
  int   *global_active2  = NULL;
  int   i;
  float min_map_value;


  if (!program_state->map_update_on && !new_map){
    program_state->map_update_pending = 0;
    send_automatic_correction_update();
    return;
  }

  global_map2                 =
    global_map_x[robot_specifications->local_map_number];
  global_label2  =
    global_label_x[robot_specifications->local_map_number];
  global_active2              =
    global_active_x[robot_specifications->local_map_number];

  
  /*
   * Initialize interval bounds
   */

  min_x = robot_specifications->global_map_dim_x 
    + robot_specifications->autoshifted_int_x + 1000;
  max_x = robot_specifications->autoshifted_int_x - 1000;
  min_y = robot_specifications->global_map_dim_y 
    + robot_specifications->autoshifted_int_y + 1000;
  max_y = robot_specifications->autoshifted_int_y - 1000;

  

  compute_bounds(robot_specifications,
		 robot_state->sensor_best_x, 
		 robot_state->sensor_best_y,
		 robot_state->sensor_best_orientation,
		 &from_x, &to_x, &from_y, &to_y);

  /*
   * loop over all points around the robot 
   */

  for (x = from_x; x <= to_x; x++){
    shifted_x = x + robot_specifications->autoshifted_int_x;

    for (y = from_y; y <= to_y; y++){
      shifted_y = y + robot_specifications->autoshifted_int_y; 


      /* if point happens to be *in* the map */
      if (shifted_x >= 0 &&
	  shifted_x < robot_specifications->global_map_dim_x &&
	  shifted_y >= 0 &&
	  shifted_y < robot_specifications->global_map_dim_y){

	/* compute corresponding robot positions in cm */
	query_x = ((float) x) * robot_specifications->resolution;
	query_y = ((float) y) * robot_specifications->resolution;
	
	
	/* read value from the local map */
	if (extract_value_from_local_map(robot_state->sensor_best_x, 
					 robot_state->sensor_best_y,
					 robot_state->sensor_best_orientation,
					 query_x, query_y,
					 &value_prediction,
					 &partial__value__wrt__robot_x,
					 &partial__value__wrt__robot_y,
					 &partial__value__wrt__robot_orientation,
					 &found_extreme_likelihood,
					 robot_specifications)){ /* active?? */
	  
	  
#ifndef xxx
	  
	  /*
	   * update min-max bounds
	   */

	  if (shifted_x < min_x) min_x = shifted_x;
	  if (shifted_x > max_x) max_x = shifted_x;
	  if (shifted_y < min_y) min_y = shifted_y;
	  if (shifted_y > max_y) max_y = shifted_y;
	  
	  /*
	   * integrate predictions into global map 
	   */
	  
	  map_index = shifted_x 
	    * robot_specifications->global_map_dim_y + shifted_y;


	  /*
	   * If this is a new map, then integrate value staight into map
	   */
	  
	  if (new_map){
	    global_map2[map_index]    = value_prediction;
	    global_active2[map_index] = 1;
	    global_label2[map_index]  = (unsigned char) 0;
	  }

	  /*
	   * If we found an extreme value (like a 1 for a place
	   * where the robot is) write this value straight into the map
	   */
	   
	  else if (found_extreme_likelihood){
	    global_map2[map_index]    = value_prediction;
	    global_active2[map_index] = 1;
	    global_label2[map_index]  = (unsigned char) 1;
	  }

	  /*
	   * Otherwise, do propper Bayesian integration
	   */
	   
	  else{
	    
	    /*
	     * First, decay the likelihood by DECAY_NEW;
	     */
	    
	    value_prediction = (value_prediction - 0.5) 
	      * robot_specifications->decay_new + 0.5;
	    
	    
	    /*
	     * integrate local and global map
	     */
	    
	    if (!global_active2[map_index]){
	    
	      /* 
	       * Use your priors.
	       */
	      
	      global_map2[map_index] = robot_specifications->prior;
	      global_active2[map_index] = 1;
	    }
	    

	    if (!frozen[global_label2[map_index]]
		&& /* don't update if we labeled
		    * this point as non-updatable
		    */
		robot_specifications->decay_new != 0.0
		&&	/* don't update if
			 * the update won't
			 * change enything 
			 * anyhow */
		(robot_specifications->decay_old != 1.0
		 || /* extreme 
		     * likelihoods (0
		     * and 1) shall not
		     * be changed if
		     * decay_old=1
		     */
		 (global_map2[map_index] > 0.0 && 
		  global_map2[map_index] < 1.0))){
	      
	      
	      /* 
	       * Decay MAP likelihood value by DECAY_OLD
	       */
	  
	      likelihood = ((global_map2[map_index] - 0.5)
			    * robot_specifications->decay_old) + 0.5;
	      /*
	       * compute new odds using Bayes' rule 
	       */
	      
	      odds = likelihood / (1.0 - likelihood)
		* value_prediction / (1.0 - value_prediction);
	      
	      /*
	       * And reextract the posterior likelihood 
	       */
	      
	      global_map2[map_index] = odds / (odds + 1.0);
	    }
	  }

	  /*
	   * now update the 0-th map, which is the XOR-ed map
	   */
#ifdef old
	  min_map_value = 2.0;
	  for (i = 1; i < NUM_GLOBAL_MAPS; i++)
	    if (global_active_x[i][map_index] &&
		global_map_x[i][map_index] < min_map_value)
	      min_map_value = global_map_x[i][map_index];
	  if (min_map_value == 2.0){
	    global_active_x[0][map_index] = 0;
	    global_label_x[0][map_index]  = (unsigned char) 0;
	    global_map_x[0][map_index] = 0.0;
	  }
	  else{
	    global_active_x[0][map_index] = 1;
	    global_label_x[0][map_index]  = (unsigned char) 0;
	    global_map_x[0][map_index] = min_map_value;
	  }
#endif
	  
#else
	  {
	    static int xxx = 0;
	    if (!xxx){
	      fprintf(stderr, "############# test mode ############\n"); 
	      xxx=1;
	    }
	    map_index = shifted_x * 
	      robot_specifications->global_map_dim_y + shifted_y;
	    global_map2[map_index] = value_prediction;
	    global_active2[map_index] = 1;
	  }
#endif
	}  /* extract */
	
      } /* point in map */
      
    } /* for y */
  } /* for x */
  
  /*
   * Update the display box
   */

  if (robot_specifications->min_display_index_x[robot_specifications->local_map_number] >
      min_x)
    robot_specifications->min_display_index_x[robot_specifications->local_map_number] = 
      min_x;
  if (robot_specifications->min_display_index_y[robot_specifications->local_map_number] >
      min_y)
    robot_specifications->min_display_index_y[robot_specifications->local_map_number] = 
      min_y;

  if (robot_specifications->max_display_index_x[robot_specifications->local_map_number] < 
      max_x)
    robot_specifications->max_display_index_x[robot_specifications->local_map_number] = 
      max_x;
  if (robot_specifications->max_display_index_y[robot_specifications->local_map_number] < 
      max_y)
    robot_specifications->max_display_index_y[robot_specifications->local_map_number] = 
      max_y;
  

  if (robot_specifications->min_display_index_x[0] >
      min_x)
    robot_specifications->min_display_index_x[0] = 
      min_x;
  if (robot_specifications->min_display_index_y[0] >
      min_y)
    robot_specifications->min_display_index_y[0] = 
      min_y;

  if (robot_specifications->max_display_index_x[0] < 
      max_x)
    robot_specifications->max_display_index_x[0] = 
      max_x;
  if (robot_specifications->max_display_index_y[0] < 
      max_y)
    robot_specifications->max_display_index_y[0] = 
      max_y;
  
  /*
   * Yet another sanity check
   */
  
  if (min_x == robot_specifications->global_map_dim_x 
      + robot_specifications->autoshifted_int_x + 1000 ||
      max_x == robot_specifications->autoshifted_int_x - 1000 ||
      min_y == robot_specifications->global_map_dim_y 
      + robot_specifications->autoshifted_int_y + 1000 ||
      max_y == robot_specifications->autoshifted_int_y - 1000){
    fprintf(stderr, "\n\n\t## Potential Error1 in min/max bounds. ##");
  }
  
  /* 
   * inform all other processes out there of the grat new piece of map.
   */

  compute_map_0(robot_specifications, program_state, robot_state);

#ifdef old  
  if (max_x >= min_x && max_y >= min_y){
    update_maps(robot_specifications, program_state, 
		min_x - robot_specifications->autoshifted_int_x, 
		max_x-min_x + 1, 
		min_y - robot_specifications->autoshifted_int_y, 
		max_y-min_y + 1, NULL, 0);
    save_gif(MAP_GIF_NAME, 0, 0);
  }
#endif




  program_state->map_update_pending = 0;


#ifdef BASE_debug
  printf("Global Map updated.\n");
#endif

}


/************************************************************************
 *
 *   NAME:         computer_map_0
 *                 
 *   FUNCTION:     Computers the map that integrates all sensor modalities
 *                 
 ************************************************************************/

void
compute_map_0(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	      PROGRAM_STATE_PTR        program_state,
	      ROBOT_STATE_PTR          robot_state)
{
  int  x, y, z, i;
  int  min_x1, max_x1, min_y1, max_y1;
  int  min_x2, max_x2, min_y2, max_y2;
  int   active;
  float value;
  


  
  /*
   * Initialize interval bounds
   */

  min_x1 = max_x1 = min_y1 = max_y1 = -1;
  min_x2 = max_x2 = min_y2 = max_y2 = -1;
  

  /*
   * loop over all points in the map
   */

  for (x = 0, z = 0; x < robot_specifications->global_map_dim_x ; x++)
    for (y = 0; y < robot_specifications->global_map_dim_y ; y++, z++){
      active = 0;
      value  = 1.0;
      for (i = 1; i < NUM_GLOBAL_MAPS; i++){
	if (global_active_x[i][z]){
	  if (!active || global_map_x[i][z] < value)
	    value =  global_map_x[i][z];
	  active = 1;
	}
      }
      if (active){		/* display window */
	if (min_x2 == -1 || x < min_x2)
	  min_x2 = x;
	if (max_x2 == -1 || x > max_x2)
	  max_x2 = x;
	if (min_y2 == -1 || y < min_y2)
	  min_y2 = y;
	if (max_y2 == -1 || y > max_y2)
	  max_y2 = y;
      }
      if (active != global_active_x[0][z] ||
	  (active && value != global_map_x[0][z])){
	/* fprintf(stderr, "%d %d: %d %d %g %g\n",
	   x, y, active, global_active_x[0][z],
	   value, global_map_x[0][z]); */

	if (min_x1 == -1 || x < min_x1)	/* update window */
	  min_x1 = x;
	if (max_x1 == -1 || x > max_x1)
	  max_x1 = x;
	if (min_y1 == -1 || y < min_y1)
	  min_y1 = y;
	if (max_y1 == -1 || y > max_y1)
	  max_y1 = y;
	global_active_x[0][z] = active;
	global_map_x[0][z]    = value;
      }
    } /* for y, x */

  
  /*
   * Update the display box
   */

  if (min_x2 == -1){
    robot_specifications->min_display_index_x[0]  = 
      robot_specifications->global_map_dim_x + 1;
    robot_specifications->max_display_index_x[0]  =  -1;
    robot_specifications->min_display_index_y[0]  = 
      robot_specifications->global_map_dim_x + 1;
    robot_specifications->max_display_index_y[0]  =  -1;
  }
  else{
    robot_specifications->min_display_index_x[0] = min_x2;
    robot_specifications->max_display_index_x[0] = max_x2;
    robot_specifications->min_display_index_y[0] = min_y2;
    robot_specifications->max_display_index_y[0] = max_y2;
  }

#ifdef MAP_debug
  fprintf(stderr, "Map 0: [%d,%d/%d,%d] [%d,%d/%d,%d] (%d)\n",
	  min_x1, max_x1, min_y1, max_y1,
	  min_x2, max_x2, min_y2, max_y2,
	  program_state->force_map_update);
#endif
  
  /* 
   * inform all other processes out there of the grat new piece of map.
   */

  if (min_x1 != -1){
    update_maps(robot_specifications, program_state, 
		min_x1 - robot_specifications->autoshifted_int_x, 
		max_x1 - min_x1 + 1, 
		min_y1 - robot_specifications->autoshifted_int_y, 
		max_y1 - min_y1 + 1, NULL, 0);

    save_gif(MAP_GIF_NAME, 0, 0);
  }
}




/************************************************************************
 *
 *   NAME:         local_map_fit
 *                 
 *   FUNCTION:     computes the fit between a local and a global map
 *                 for a given robot position and sensor interpretation.
 *                 computes also the gradients for a better fit :-)
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



float compute_local_fit(float robot_x, 
			float robot_y,
			float robot_orientation,
			int granularity,
			float *partial__correlation__wrt__robot_x,
			float *partial__correlation__wrt__robot_y,
			float *partial__correlation__wrt__robot_orientation)
{

  register float path_weight;
  register int i;
  register int x, y, shifted_x, shifted_y, map_index;
  int from_x, to_x, from_y, to_y;
  float query_x, query_y;
  float value_prediction, value;
  float partial__value__wrt__robot_x;
  float partial__value__wrt__robot_y;
  float partial__value__wrt__robot_orientation;
  float sum_local_times_local;
  float sum_global_times_global;
  float sqroot_sum_local_times_local;
  float sqroot_sum_global_times_global;
  float sum_local_times_global;
  float sum_local__times__partial__value__wrt__robot_x;
  float sum_local__times__partial__value__wrt__robot_y;
  float sum_local__times__partial__value__wrt__robot_orientation;
  float sum_global__times__partial__value__wrt__robot_x;
  float sum_global__times__partial__value__wrt__robot_y;
  float sum_global__times__partial__value__wrt__robot_orientation;
  float correlation, fact1, fact2;
  float stepsize;
  float float_i, query_dist;
  int   found_extreme_likelihood;


  /*
   * initialize variables for computation of the correlation 
   * and the gradients
   */

  sum_local_times_global                                    = 0.0;
  sum_local_times_local                                     = 0.0;
  sum_global_times_global                                   = 0.0;
  sum_local__times__partial__value__wrt__robot_x            = 0.0;
  sum_local__times__partial__value__wrt__robot_y            = 0.0;
  sum_local__times__partial__value__wrt__robot_orientation  = 0.0;
  sum_global__times__partial__value__wrt__robot_x           = 0.0;
  sum_global__times__partial__value__wrt__robot_y           = 0.0;
  sum_global__times__partial__value__wrt__robot_orientation = 0.0;
  
  /*
   * superset of points to make predictions at 
   */

  compute_bounds(robot_specifications,
		 robot_x, robot_y, robot_orientation,
		 &from_x, &to_x, &from_y, &to_y);

  
  /*
   * loop over all points around the robot 
   */

  for (x = from_x; x <= to_x; x += granularity){
    shifted_x = x + robot_specifications->autoshifted_int_x;
    for (y = from_y; y <= to_y; y += granularity){
      shifted_y = y + robot_specifications->autoshifted_int_y;
      
      /*
       * if point happens to be *in* the map 
       */

      if (shifted_x >= 0 &&
	  shifted_x < robot_specifications->global_map_dim_x &&
	  shifted_y >= 0 &&
	  shifted_y < robot_specifications->global_map_dim_y){
	
	/*
	 * compute corresponding robot positions in cm 
	 */

	query_x = ((float) x) * robot_specifications->resolution;
	query_y = ((float) y) * robot_specifications->resolution;
	
	if (!program_state->global_map_matching_mode &&
	    robot_specifications->max_distance_in_match > 0.0)
	  query_dist = sqrt(((query_x-robot_x) * (query_x-robot_x)) +
			    ((query_y-robot_y) * (query_y-robot_y)));
	else			/* not ON! */
	  query_dist = robot_specifications->max_distance_in_match - 1.0;

	/*
	 * index in the global map 
	 */

	map_index = shifted_x 
	  * robot_specifications->global_map_dim_y + shifted_y;
	
	/*
	 * read value from the local map. Both maps must be defined. 
	 */

	if ((query_dist <= robot_specifications->max_distance_in_match &&
	     global_active[map_index]) || 
	    program_state->global_map_matching_mode){
	  if (extract_value_from_local_map(robot_x, robot_y, 
					   robot_orientation,
					   query_x, query_y,
					   &value_prediction,
					   &partial__value__wrt__robot_x,
					   &partial__value__wrt__robot_y,
					   &partial__value__wrt__robot_orientation,
					   &found_extreme_likelihood,
					   robot_specifications)){ /*active??*/
	    /*
	     * normalize value_prediction to lie in [-1,1] 
	     */
	    
	    value_prediction = (value_prediction * 2.0) - 1.0;
	    
	    /*
	     * get the corresponding value from the global map
	     */

	    if (global_active[map_index]){
	      /*!*/ /* if (program_state->global_map_matching_mode)
		 global_map[map_index] = 2.0;*//*!*/
	      value = global_map[map_index];
	    }
	    else{
	      /*!*/ /*fprintf(stderr, "[%d %d] ", shifted_x, shifted_y);
	      global_map[map_index] = -2.0;
	      global_active[map_index] = 1;*//*!*/
	      value = RAND() * 5.0 + 1.0; 
	    }
	    if (program_state->global_map_matching_mode){
	      if (value < 0.4) value = 0.0;
	      else if (value > 0.8) value = 1.0;
	      else value = 0.5;
	      if (value_prediction < 0.4) value_prediction = 0.0;
	      else if (value_prediction > 0.8) value_prediction = 1.0;
	      else value_prediction = 0.5;
	    }
	    /* 
	     * do the same with the global map. Notice that here, unlike in
	     * the local map, the symmetry point is already 0.5.
	     */
	    

	    value = (value * 2.0) - 1.0;
	    
	      
	    /*
	     * compute the vector correlation of the global vs. local map.
	     * this is done incrementally.
	     */
	    
	    sum_local_times_local   += value_prediction * value_prediction;
	    sum_global_times_global += value * value;
	    sum_local_times_global  += value_prediction * value;
	    
	    
	    /*
	     * further variables for computing the gradient
	     */
	    
	    sum_local__times__partial__value__wrt__robot_x +=
	      value_prediction * partial__value__wrt__robot_x;
	    sum_local__times__partial__value__wrt__robot_y +=
	      value_prediction * partial__value__wrt__robot_y;
	    sum_local__times__partial__value__wrt__robot_orientation  +=
	      value_prediction * partial__value__wrt__robot_orientation;
	    
	    sum_global__times__partial__value__wrt__robot_x +=
	      value * partial__value__wrt__robot_x;
	    sum_global__times__partial__value__wrt__robot_y +=
	      value * partial__value__wrt__robot_y;
	    sum_global__times__partial__value__wrt__robot_orientation  +=
	      value * partial__value__wrt__robot_orientation;
	    
	  }
	}
      }
    }
  }

  /*
   * loop over all point in the path - those shall be freespace!
   */

  
  if (robot_specifications->do_path_fitting){

    if (robot_specifications->n_path_points_in_fit < 0 ||
	robot_specifications->n_path_points_in_fit >= 
	n_path_entries)
      stepsize = 1.0;
    else
      stepsize = ((float) n_path_entries)
	/ ((float) robot_specifications->n_path_points_in_fit);
    
    path_weight = robot_specifications->weight_path_fit;


    for (float_i = 0.0; 
	 float_i < ((float) n_path_entries);
	 float_i += stepsize){
      
      /*
       * compute index i
       */

      i = (int) float_i;

      /*
       * compute corresponding robot positions in cm 
       */
      
      query_x = path[i][0];
      query_y = path[i][1];
      
      
      /*
       * index in the global map (except autoshift)
       */
      
      shifted_x = ((int) (query_x / robot_specifications->resolution));
      shifted_y = ((int) (query_y / robot_specifications->resolution));


      if (shifted_x >= from_x && shifted_x <= to_x &&
	  shifted_y >= from_y && shifted_y <= to_y){

	/*
	 * Compute global map coordinates - adjust for autoshift
	 */

	shifted_x += robot_specifications->autoshifted_int_x;
	shifted_y += robot_specifications->autoshifted_int_y;
	
	/*
	 * read value from the local map. 
	 */
	
	if (extract_value_from_local_map(robot_x, robot_y, 
					 robot_orientation,
					 query_x, query_y,
					 &value_prediction,
					 &partial__value__wrt__robot_x,
					 &partial__value__wrt__robot_y,
					 &partial__value__wrt__robot_orientation,
					 &found_extreme_likelihood,
					 robot_specifications)){ /*active??*/
	  
	  
	  
	  /*
	   * normalize value_prediction to lie in [-1,1] 
	   */
	  
	  value_prediction = (value_prediction * 2.0) - 1.0;
	  
	  /* 
	   * This point must be as close to 1 as possible, since the robot
	   * once went through this point.
	   */
	  
	  value = 1.0;
	  
	  
	  /*
	   * compute the vector correlation of the global vs. local map.
	   * this is done incrementally.
	   */
	  
	  sum_local_times_local   += path_weight 
	    * value_prediction * value_prediction;
	  sum_global_times_global += path_weight 
	    * value * value;
	  sum_local_times_global  += path_weight 
	    * value_prediction * value;
	  
	  
	  
	  /*
	   * further variables for computing the gradient
	   */
	  
	  sum_local__times__partial__value__wrt__robot_x +=
	    path_weight 
	      * value_prediction * partial__value__wrt__robot_x;
	  sum_local__times__partial__value__wrt__robot_y +=
	    path_weight 
	      * value_prediction * partial__value__wrt__robot_y;
	  sum_local__times__partial__value__wrt__robot_orientation  +=
	    path_weight 
	      * value_prediction * partial__value__wrt__robot_orientation;
	  
	  sum_global__times__partial__value__wrt__robot_x +=
	    path_weight 
	      * value * partial__value__wrt__robot_x;
	  sum_global__times__partial__value__wrt__robot_y +=
	    path_weight 
	      * value * partial__value__wrt__robot_y;
	  sum_global__times__partial__value__wrt__robot_orientation  +=
	    path_weight 
	      * value * partial__value__wrt__robot_orientation;

	}
      }
    }
  }    

  /*
   * Now calculate the correlation
   */
  
  if (sum_local_times_local > 0.0 && sum_global_times_global > 0.0){
    
    /*
     * compute square root - gives denominator
     */
    sqroot_sum_local_times_local   = sqrt(sum_local_times_local);
    sqroot_sum_global_times_global = sqrt(sum_global_times_global);
    fact1 = 1.0
      / (sqroot_sum_global_times_global * sqroot_sum_local_times_local);
    fact2 = sum_local_times_global * fact1 / sum_local_times_local;

    /*
     * compute correlation
     */

    correlation = sum_local_times_global * fact1;

    /*
     * compute gradients
     */

    *partial__correlation__wrt__robot_x =
      (fact1 * sum_global__times__partial__value__wrt__robot_x)
	+ (fact2 * sum_local__times__partial__value__wrt__robot_x);
      
    *partial__correlation__wrt__robot_y =
      (fact1 * sum_global__times__partial__value__wrt__robot_y)
	+ (fact2 * sum_local__times__partial__value__wrt__robot_y);
    
    *partial__correlation__wrt__robot_orientation =
      (fact1 * sum_global__times__partial__value__wrt__robot_orientation)
	+ (fact2 * sum_local__times__partial__value__wrt__robot_orientation);
    
  }

  else{			/*
			 * there are no two non-zero points in the sum 
			 */


    correlation = 0.0;		/*
				 * no correlation possible 
				 */


    *partial__correlation__wrt__robot_x = 0.0; /* ...and hence no gradients! */
    *partial__correlation__wrt__robot_y = 0.0;  
    *partial__correlation__wrt__robot_orientation = 0.0;


  }

  /*  printf(" %g ", correlation);*/


  return correlation;		/*
				 * and return the correlation here 
				 */


}
  



/************************************************************************
 *
 *   NAME:         free_extreme_values
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
/* 
 * void free_extreme_values()
 * {
 *   
 *   int x, y, index;
 *   float robot_x, robot_y;
 *   
 *   for (x = 0; x < robot_specifications->global_map_dim_x; x++){
 *     for (y = 0; y < robot_specifications->global_map_dim_y; y++){
 *       index = x * robot_specifications->global_map_dim_y + y;
 * 	if (global_active[index]){
 * 	  if (global_map[index] == 0.0)
 * 	    global_map[index] = 0.05;
 * 	  if (global_map[index] == 1.0)
 * 	    global_map[index] = 9.95;
 * 	}
 *     }
 *   }
 * }
 */	  



/************************************************************************
 *
 *   NAME:         compute_correlation_matrix
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void compute_correlation_matrix(float robot_orientation)
{
  int x, y, index;
  float robot_x, robot_y;
  float gradient_robot_x, gradient_robot_y, gradient_robot_orientation;

  for (x = 0; x < robot_specifications->global_map_dim_x; x++){
    for (y = 0; y < robot_specifications->global_map_dim_y; y++){
	index = x * robot_specifications->global_map_dim_y + y;
	if (global_active[index]){
	  robot_x = ((float) (x - robot_specifications->autoshifted_int_x))
	    * robot_specifications->resolution;
	  robot_y = ((float) (y - robot_specifications->autoshifted_int_y))
	    * robot_specifications->resolution;
	  global_local_correlations[index] =
	    compute_local_fit(robot_x, robot_y, robot_orientation, 2,
			      &gradient_robot_x, &gradient_robot_y,
			      &gradient_robot_orientation);
	  printf("{%5.3f}", global_local_correlations[index]);
	}
      }
    printf("."); fflush(stdout);
  }
}
	

/************************************************************************
 *
 *   NAME:         initiate_position_search()
 *                 
 *   FUNCTION:     resets all gradients to start a new search
 *                 
 *   PARAMETERS:   nothing worth mentioning
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

    
void initiate_position_search(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			      PROGRAM_STATE_PTR        program_state,
			      ROBOT_STATE_PTR          robot_state)
{
  robot_state->last_change_x           = 0.0;
  robot_state->last_change_y           = 0.0;
  robot_state->last_change_orientation = 0.0;
}



/************************************************************************
 *
 *   NAME:         do_search_step()
 *                 
 *   FUNCTION:     does a search step for the robot's position
 *                 
 *   PARAMETERS:   nothing worth mentioning
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

    
void do_search_step(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		    PROGRAM_STATE_PTR        program_state,
		    ROBOT_STATE_PTR          robot_state)
{
  float gradient_sensor_x, gradient_sensor_y, gradient_sensor_orientation;
  float weight_fit_trans, weight_fit_rot;
  float weight_prev_position_trans, weight_prev_position_rot;
  float uncorr_robot_x, uncorr_robot_y, uncorr_robot_orientation;
  float fit;
  float diff_orientation;

#ifdef MAP_debug
  fprintf(stderr, "MAP:do_search_step: starting\n");
#endif

  /*
   * check if we have to do something
   */

  if (!program_state->map_update_pending &&
      !program_state->global_map_matching_mode)
    return;			/* Why  shall we refine the robot position? 
				 *
				 * We are very restrictive here. Assumes
				 * that the map update is only applied when
				 * the search is over. If the update will ever
				 * be applied earlier, this needs changed.
				 *                           (P'burgh slang)
				 */


  if ((!robot_specifications->do_position_correction &&
       !program_state->global_map_matching_mode) ||
      program_state->tcx_localize_connected || /* localize has strict
						* priority */
      !program_state->maps_allocated) /* No comments! Speaks for itself! */

    return;


  if (!program_state->global_map_matching_mode && /* differnt rues aply
						   * in map-fitting */
      robot_specifications->max_niterations_in_search != -1 &&
      robot_specifications->niterations_in_search >=
      robot_specifications->max_niterations_in_search){	/* search over! */
    
    if (program_state->map_update_pending) /* There is still an update
						   * to be done. Then do it.
						   * Apparently, the search 
						   * is over
						   */
      update_internal_map(0, robot_specifications, program_state,
			  robot_state);

    return;
  }

  /*
   * compute fit (correlation) gradients
   */

  fit = compute_local_fit(robot_state->sensor_x, 
			  robot_state->sensor_y, 
			  robot_state->sensor_orientation, 
			  robot_specifications->search_granularity,
			  &gradient_sensor_x,
			  &gradient_sensor_y,
			  &gradient_sensor_orientation);
  
  /*
   * If L-2 norm, we have to take the complete first deriv. of the correlation
   */

  
  if (robot_specifications->map_fit_norm_L2){ /* L2 norm */
    gradient_sensor_x           *= (1.0 - fit);
    gradient_sensor_y           *= (1.0 - fit);
    gradient_sensor_orientation *= (1.0 - fit);

  }

  /*
   * compute the change in orientation. This must be more sophisticated
   * than just taking the difference, since the rotation is circular
   */

  diff_orientation = robot_state->sensor_org_orientation 
    - robot_state->sensor_orientation;
  for (;diff_orientation < -180.0;) diff_orientation += 360.0;
  for (;diff_orientation >  180.0;) diff_orientation -= 360.0;

  /*
   * add position-deviation gradients (this prevents the robot from
   * moving indefinitely by forcing the final position to be similar
   * to the initial position in the search)
   */

  
  if (!program_state->global_map_matching_mode){

    weight_prev_position_trans = 
      2.0 * robot_specifications->translation_weight_prev_position;
    weight_prev_position_rot = 
      2.0 * robot_specifications->rotation_weight_prev_position;

    weight_fit_trans = robot_state->sensor_uncertainty
      * robot_specifications->translation_weight_fit;
    weight_fit_rot = robot_state->sensor_uncertainty
      * robot_specifications->rotation_weight_fit;

  }
  else{

    weight_prev_position_trans = 0.0; /* global-map-matching special: */
    weight_prev_position_rot = 0.0; /* we don't care for prev. pos. */

    weight_fit_trans = 
      robot_specifications->translation_weight_fit_global_match;
    weight_fit_rot = 
      robot_specifications->rotation_weight_fit_global_match;


  }



  /*
   * weight_prev_position_trans = 0.0;
   * weight_prev_position_rot = 0.0;
   */

  if (robot_specifications->prev_pos_norm_L2){ /* L2 norm */
    gradient_sensor_x = (weight_fit_trans * gradient_sensor_x)
      + (weight_prev_position_trans 
	 * (robot_state->sensor_org_x - robot_state->sensor_x));
    gradient_sensor_y = (weight_fit_trans * gradient_sensor_y)
      + (weight_prev_position_trans 
	 * (robot_state->sensor_org_y - robot_state->sensor_y));
    gradient_sensor_orientation = 
      (weight_fit_rot * gradient_sensor_orientation)
	+ (weight_prev_position_rot * diff_orientation);
  }
  else{				/* L1 norm */
    gradient_sensor_x = (weight_fit_trans * gradient_sensor_x)
      + (weight_prev_position_trans 
	 * SIG(robot_state->sensor_org_x - robot_state->sensor_x));
    gradient_sensor_y = (weight_fit_trans * gradient_sensor_y)
      + (weight_prev_position_trans 
	 * SIG(robot_state->sensor_org_y - robot_state->sensor_y));
    gradient_sensor_orientation = 
      (weight_fit_rot * gradient_sensor_orientation)
	+ (weight_prev_position_rot * SIG(diff_orientation));
  }

  /*
   * add deviation into fit value (not done in global map-matching mode)
   */


  if (!program_state->global_map_matching_mode){
    fit *= weight_fit_trans;

    fit -= (robot_specifications->translation_weight_prev_position
	    * (((robot_state->sensor_org_x - robot_state->sensor_x)
		* (robot_state->sensor_org_x - robot_state->sensor_x))
	       + ((robot_state->sensor_org_y - robot_state->sensor_y)
		  * (robot_state->sensor_org_y - robot_state->sensor_y))))
      + (robot_specifications->rotation_weight_prev_position
	 * (diff_orientation * diff_orientation));
  }
  
#ifdef MAP_debug
/*  printf("fit: [%5.3f]", fit); */
#endif

  /*
   * check, if this is the best fit so far. If so, apply changes to state
   * and correction parameters
   */
  
  if (fit > robot_state->sensor_best_fit){

    /*
     * corrections will only be applied when we are matching bare sensor 
     * interpretations
     */
    
    if (!program_state->global_map_matching_mode){
      compute_backward_correction(robot_state->x,
				  robot_state->y,
				  robot_state->orientation,
				  robot_state->correction_parameter_x,
				  robot_state->correction_parameter_y,
				  robot_state->correction_parameter_angle,
				  robot_state->correction_type,
				  &uncorr_robot_x,
				  &uncorr_robot_y, 
				  &uncorr_robot_orientation);
      
      update_correction_parameters(robot_state->sensor_x,
				   robot_state->sensor_y,
				   robot_state->sensor_orientation,
				   robot_state->sensor_x 
				   - robot_state->sensor_best_x,
				   robot_state->sensor_y
				   - robot_state->sensor_best_y,
				   robot_state->sensor_orientation 
				   - robot_state->sensor_best_orientation,
				   &(robot_state->correction_parameter_x),
				   &(robot_state->correction_parameter_y),
				   &(robot_state->correction_parameter_angle),
				   &(robot_state->correction_type));
    }

    robot_state->sensor_best_x = robot_state->sensor_x;
    robot_state->sensor_best_y = robot_state->sensor_y;
    robot_state->sensor_best_orientation = robot_state->sensor_orientation;
    robot_state->sensor_best_fit = fit;
    
    
    if (!program_state->global_map_matching_mode){
      compute_forward_correction(uncorr_robot_x,
				 uncorr_robot_y, 
				 uncorr_robot_orientation,
				 robot_state->correction_parameter_x,
				 robot_state->correction_parameter_y,
				 robot_state->correction_parameter_angle,
				 robot_state->correction_type,
				 &robot_state->x,
				 &robot_state->y,
				 &robot_state->orientation);
    }
    
    


    fprintf(stderr, "#");
  }
  else if (fit == robot_state->sensor_best_fit)
    fprintf(stderr, "="); 
  else
    fprintf(stderr, "?"); 
  fflush(stderr);
  


  /*
   * apply the stepsize and the momentum
   */

  robot_state->last_change_x = 
    (robot_specifications->translation_stepsize * gradient_sensor_x) 
      + (robot_specifications->translation_momentum 
	 * robot_state->last_change_x);
  robot_state->last_change_y = 
    (robot_specifications->translation_stepsize * gradient_sensor_y)
      + (robot_specifications->translation_momentum 
	 * robot_state->last_change_y);
  robot_state->last_change_orientation = 
    (robot_specifications->rotation_stepsize * gradient_sensor_orientation)
      + (robot_specifications->rotation_momentum * 
	 robot_state->last_change_orientation);
  
  /*
   * clip changes to maximum legal values
   */

  if (robot_state->last_change_x >
      robot_specifications->max_translation_in_search)
    robot_state->last_change_x = 
      robot_specifications->max_translation_in_search;
  else if (robot_state->last_change_x <
	   -robot_specifications->max_translation_in_search)
    robot_state->last_change_x = 
      -robot_specifications->max_translation_in_search;
  
  if (robot_state->last_change_y > 
      robot_specifications->max_translation_in_search)
    robot_state->last_change_y = 
      robot_specifications->max_translation_in_search;
  else if (robot_state->last_change_y < 
	   -robot_specifications->max_translation_in_search)
    robot_state->last_change_y = 
      -robot_specifications->max_translation_in_search;
  
  
  if (robot_state->last_change_orientation > 
      robot_specifications->max_rotation_in_search)
    robot_state->last_change_orientation = 
      robot_specifications->max_rotation_in_search;
  else if (robot_state->last_change_orientation < 
	   -robot_specifications->max_rotation_in_search)
    robot_state->last_change_orientation = 
      -robot_specifications->max_rotation_in_search;


  /*
   * apply changes
   */

  robot_state->sensor_x               += robot_state->last_change_x;
  robot_state->sensor_y               += robot_state->last_change_y;
  robot_state->sensor_orientation     += robot_state->last_change_orientation;

 
  /*
   * and display the robot
   */
 

  if (program_state->graphics_initialized){

    if (program_state->global_map_matching_mode)

      G_display_robot(BEST_FIT_POINT, /* the green guy */
		      robot_state->sensor_x,
		      robot_state->sensor_y,
		      robot_state->sensor_orientation,
		      0, NULL);
    /*
    else

      G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y,
		      robot_state->orientation, 0, NULL);

		      */

  }
  
  
  /*
   * finally increment counter
   */
 
  robot_specifications->niterations_in_search++;
#ifdef MAP_debug
  fprintf(stderr, "MAP:do_search_step: exiting\n");
#endif
}




/************************************************************************
 *
 *   NAME:         set_map
 *                 
 *   FUNCTION:     Sets active global map
 *                 
 *   PARAMETERS:   int number    - nuber, must be in [0..NUM_GLOBAL_MAPS]
 *
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void set_map(int number)
{
  char angle_txt[128];
  int  i;

  for (; number < 0;) number += NUM_GLOBAL_MAPS;
  for (; number >= NUM_GLOBAL_MAPS; ) number -= NUM_GLOBAL_MAPS;

  /*
   * The usual sanity check
   */
  /*
   * if (number < 0 || number >= NUM_GLOBAL_MAPS){
   * fprintf(stderr, "WARNING: map number %d not available. Ignored.\n",
   * number);
   * return;
   * }
   */

  /*
   * Set map.
   */

  global_map    = global_map_x[number];
  global_active = global_active_x[number];
  global_label  = global_label_x[number];


  program_state->actual_map = number;

  /*
   * Broadcast that map to all clients - I took this out now.
   */
  /*  send_complete_map(robot_specifications, program_state, NULL); */


  if (program_state->graphics_initialized){
    G_change_matrix(GLOBAL_MAPVALUES,
		    global_map, global_active,
		    robot_specifications->global_map_dim_x,
		    robot_specifications->global_map_dim_y);
    G_display_switch(ACTUAL_MAP_BUTON, program_state->actual_map);
    if (robot_state->map_orientation_defined){
      sprintf(angle_txt, "(walls: %5.1f)", 
	      robot_state->map_orientation);
      G_set_new_text(LINE_ANGLE_BUTTON, angle_txt, 1);
      G_display_switch(LINE_ANGLE_BUTTON, 1);
    }
    else
      G_display_switch(LINE_ANGLE_BUTTON, 0);
  }

}	     



/************************************************************************
 *
 *   NAME:         smooth map
 *                 
 *   FUNCTION:     Smoothes a map
 *                 
 *   WARNING:      The caller has to allocate the memory
 *
 *   RETURN-VALUE: 1, if we found a match, 0 if error
 *                 
 ************************************************************************/

void smooth_map(float *original_map, 
		int   *original_active,
		float *smoothed_map,
		int    dim_x,
		int    dim_y,
		int    smooth_radius)
{
  register int   i;		/* No procedure without a variable "i"! */
  register int   j, ii, jj;
  int   index1, index2, index3;
  float count;			/* count can be float! */

  /* ================================================= *\
   */
  
  static int max_smooth_radius = -1; /* static! */
  static float *smooth = NULL;

  /*
   * ================================================= */


  /* 
   * compute smoothing table. this is done only a few times.
   */
  
  
  if (smooth != NULL && smooth_radius > max_smooth_radius)
    free(smooth);
  if (smooth == NULL){
    smooth = (float *) 
      malloc (sizeof(float) * smooth_radius * smooth_radius);
    max_smooth_radius = smooth_radius;
    for (i = 0; i < smooth_radius; i++)
      for (j = 0; j < smooth_radius; j++)
	smooth[i * smooth_radius + j] =
	  1.0 / ((float) (i+j+2));
  }
  
  /* 
   * compute smoothed map
   */
  
  
  if (smoothed_map == NULL){
    printf("Error. no map allocated,!\n");
    return; 
  }
  
  
  for (i = 0; i < dim_x * dim_y; i++)
    smoothed_map[i] = 0.0;
  
  for (i = 0; i < dim_x; i++)
    for (j = 0; j < dim_y; j++){
      count = 0.0;
      index1 = i * dim_y + j;
      for (ii = 0; ii < smooth_radius; ii++){
	if (i + ii < dim_x){
	  for (jj = 0; jj < smooth_radius; jj++)
	    if (j + jj < dim_y){
	      index2 = (i+ii) * dim_y + (j+jj);
	      index3 = ii * max_smooth_radius + jj;
	      if (original_active[index2]){
		smoothed_map[index1] += 
		  smooth[index3] * original_map[index2];
		count += smooth[index3];
	      }
	    }
	  for (jj = 0; jj < smooth_radius; jj++)
	    if (j - jj >= 0){
	      index2 = (i+ii) * dim_y + (j-jj);
	      index3 = ii * max_smooth_radius + jj;
	      if (original_active[index2]){
		smoothed_map[index1] += 
		  smooth[index3] * original_map[index2];
		count += smooth[index3];
	      }
	    }
	}
	if (i - ii >= 0){
	  for (jj = 0; jj < smooth_radius; jj++)
	    if (j + jj < dim_y){
	      index2 = (i-ii) * dim_y + (j+jj);
	      index3 = ii * max_smooth_radius + jj;
	      if (original_active[index2]){
		smoothed_map[index1] += 
		  smooth[index3] * original_map[index2];
		count += smooth[index3];
	      }
	    }
	  for (jj = 0; jj < smooth_radius; jj++)
	    if (j - jj >= 0){
	      index2 = (i-ii) * dim_y + (j-jj);
	      index3 = ii * max_smooth_radius + jj;
	      if (original_active[index2]){
		smoothed_map[index1] += 
		  smooth[index3] * original_map[index2];
		count += smooth[index3];
	      }
	    }
	}
      }
      if (count >= 0.0)
	smoothed_map[index1] /= count;
    }
}

/************************************************************************
 *
 *   NAME:         prepare_2nd_run
 *                 
 *   FUNCTION:     Prepares the map for the second run by
 *                    - clipping all mapvalues
 *                    - unfreezing the robot's path
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 1, if we found a match, 0 if error
 *                 
 ************************************************************************/


void prepare_2nd_run(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		     PROGRAM_STATE_PTR        program_state)
{
  register int x, y, index;

  for (x = 0, index = 0; x < robot_specifications->global_map_dim_x; x++){
    for (y = 0; y < robot_specifications->global_map_dim_y; y++, index++){
      if (global_active[index]){/* clip lieklihoods */
	if (global_map[index] > robot_specifications->upper_clipping_value) 
	  global_map[index] = robot_specifications->upper_clipping_value;
	if (global_map[index] < robot_specifications->lower_clipping_value) 
	  global_map[index] = robot_specifications->lower_clipping_value;
	if (global_label[index] == 1)
	  global_label[index] = 2; /* path, but may be overwritten */
      }
    }
  }
}


/************************************************************************
 *
 *   NAME:         combine_global_maps
 *                 
 *   FUNCTION:     Tries to match two global maps.
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 1, if we found a match, 0 if error
 *                 
 ************************************************************************/



int combine_global_maps(int base_map_number, /* serves as "global" map */
			int comp_map_number, /* serves as "local"  map */
			float global_map_start_x,
			float global_map_start_y,
			float global_map_start_orientation,
			int   combine, /* if 0, we do not want to 
					* change anything */
			int display,
			float *correlation,
			ROBOT_SPECIFICATIONS_PTR robot_specifications,
			PROGRAM_STATE_PTR        program_state,
			ROBOT_STATE_PTR          robot_state)
{
  int x, y, min_x, min_y, max_x, max_y, m_size, n, i;
  float *global_map_ptr;
  int   *global_active_ptr;
  float *local_map_ptr;
  int   *local_active_ptr;
  int   direction;
  float best_fit = -9999.0;	/* keep your dumb compiler happy! */
  float best_x = 0.0, best_y = 0.0, best_orientation = 0.0;
  int   prev_actual_map;
  float angle_diff;

  /*
   * Is the programmer sane?
   */

  if (base_map_number < 0 || base_map_number >= NUM_GLOBAL_MAPS){
    fprintf(stderr, "Error in comp_map_number: ");
    fprintf(stderr, "base_map_number must be in 0..%d\n",
	    NUM_GLOBAL_MAPS);
    return 0;
  }

  if (comp_map_number < 0 || comp_map_number >= NUM_GLOBAL_MAPS){
    fprintf(stderr, "Error in comp_map_number: ");
    fprintf(stderr, "comp_map_number must be in 0..%d\n",
	    NUM_GLOBAL_MAPS);
    return 0;
  }

  if (n_path_entries <= 0){
    fprintf(stderr, "Error: comp-map has no path.");
    return 0;
  }

  if (!robot_state->map_orientation_defined){
    fprintf(stderr, "Error in comp_map_number: ");
    fprintf(stderr, "the orientation of walls must be known for both maps.\n");
    return 0;
  }



  /*
   * Okay, sounds like fun.  Let's terminate any currently active matching 
   * process
   */

  if (program_state->map_update_pending)
    update_internal_map(0, robot_specifications, program_state,
			robot_state);
  

  /*
   * Figure out where the map #2 is defined.
   */

  min_x = -1;
  max_x = -1;
  min_y = -1;
  max_y = -1;
  global_map_ptr    = global_map_x[comp_map_number];
  global_active_ptr = global_active_x[comp_map_number];

  

  for (x = 0; x < robot_specifications->global_map_dim_x; x++)
    for (y = 0; y < robot_specifications->global_map_dim_y; y++)
      if (*global_active_ptr++){
	if (min_x == -1 || x < min_x)
	  min_x = x;
	if (max_x == -1 || x > max_x)
	  max_x = x;
	if (min_y == -1 || y < min_y)
	  min_y = y;
	if (max_y == -1 || y > max_y)
	  max_y = y;
      }
  

  if (min_x == -1 || max_x == -1 || min_y == -1 || max_y == -1){
    fprintf(stderr, "Error in comp_map_number: ");
    fprintf(stderr, "The comp-map is completely empty.\n");
    return 0;
  }
  
  max_x++;			/* Adjust! */
  max_y++;			/* Adjust! Makes counting easier */

  /*
   *          This is the map size: the size of the active map #2
   */
  
  robot_specifications->local_map_dim_x = max_x - min_x;
  robot_specifications->local_map_dim_y = max_y - min_y;
  
  m_size =  robot_specifications->local_map_dim_x *
    robot_specifications->local_map_dim_y;

  /*
   * Let's make the global map #2 a local map
   */

  if (local_map != NULL)
    free(local_map);
  if (local_smooth_map != NULL)
    free(local_smooth_map);
  if (local_active != NULL)
    free(local_active);


  /* 
   * Allocate memory
   */

  local_map         = (float *) (malloc(sizeof(float) * m_size));
  local_smooth_map  = (float *) (malloc(sizeof(float) * m_size));
  local_active      = (int *)   (malloc(sizeof(int) * m_size));


  global_map_ptr    = global_map_x[comp_map_number];
  global_active_ptr = global_active_x[comp_map_number];
  local_map_ptr     = local_map;
  local_active_ptr  = local_active;

  for (x = min_x; x < max_x; x++)
    for (y = min_y; y < max_y; y++){
      *local_map_ptr++ = 
	global_map_ptr[x * robot_specifications->global_map_dim_y + y];
      *local_active_ptr++ = 
	global_active_ptr[x * robot_specifications->global_map_dim_y + y];
    }
  
  
  
  /* 
   * Smooth the map to improve gradient descent search
   */
  

  smooth_map(local_map, local_active, local_smooth_map, 
	     robot_specifications->local_map_dim_x,
	     robot_specifications->local_map_dim_y, 5);


  /*
   * Set active map appropriately
   */

  prev_actual_map = program_state->actual_map;

  set_map(base_map_number);

  /* 
   * Display all maps
   */
  
  if (program_state->graphics_initialized && display){
      G_activate(LOCAL_MAPVALUES);
      G_change_matrix(LOCAL_MAPVALUES, local_smooth_map, local_active,
		      robot_specifications->local_map_dim_x,
		      robot_specifications->local_map_dim_y);
      G_display_matrix(LOCAL_MAPVALUES);
      if (!program_state->regular_local_map_display)
	G_deactivate(LOCAL_MAPVALUES);

      G_display_matrix(GLOBAL_MAPVALUES);
  }
  

  /*
   * Now start 4 searches, for the 4 different possible orientations
   */

  for (direction = 0; direction < 4; direction++){
#ifdef junk
    {
      int i, done = 0;
      for (i = 0; i < 100000 && !done; i++){/*!*/
	if (path[comp_map_number][i][0] != path[comp_map_number][i+1][0] ||
	    path[comp_map_number][i][1] != path[comp_map_number][i+1][1]){
	  done = 1;
	  angle_diff = atan2(path[comp_map_number][i+1][1] - path[comp_map_number][i][1]
	  
#endif	    
	    

    angle_diff =   
      (robot_state->map_orientation
       - robot_state->map_orientation 
       + (90.0 * ((float) direction)))
	- (global_map_start_orientation
	   - path[0][2]);
    for (; angle_diff < -180.0;) angle_diff += 360.0;
    for (; angle_diff >  180.0;) angle_diff -= 360.0;

    if (global_map_start_orientation < -900.0 ||
	fabs(angle_diff) <= 45.0){/*!*/
      
    fprintf(stderr, "%f %f %f -> %f   %f %f -> %f   %f\n",
	    robot_state->map_orientation,
	    robot_state->map_orientation,
	    (90.0 * ((float) direction)),
	    (robot_state->map_orientation
	     - robot_state->map_orientation 
	     + (90.0 * ((float) direction))),
	    global_map_start_orientation,
	    path[0][2],
	    global_map_start_orientation
	    - path[0][2],
	    angle_diff);

      /* ****************
       *
       * Now initialize the local matching variables
       */
      
      
      /*
       *          The origin, which serves as the rotation point, is in the
       *          center of the map 
       */
      

    robot_specifications->local_map_origin_x = 0.5 * ((float) (max_x - min_x))
      * robot_specifications->resolution;
    robot_specifications->local_map_origin_y =  0.5 * ((float) (max_y - min_y))
      * robot_specifications->resolution;
    robot_specifications->local_map_origin_orientation = 0.0;


/*
    robot_specifications->local_map_origin_x = 
      (path[comp_map_number][0][0] / robot_specifications->resolution) - 
	((float) (min_x + robot_specifications->autoshifted_int_x));
    
    robot_specifications->local_map_origin_y = 
      (path[comp_map_number][0][1] / robot_specifications->resolution) - 
	((float) (min_y + robot_specifications->autoshifted_int_y));
    
*/
      /*
       *          The local map is placed in the global map
       */
      
      robot_state->sensor_org_x 
	= robot_state->sensor_x
	  = robot_state->sensor_best_x
	    = global_map_start_x;
      
      robot_state->sensor_org_y
	= robot_state->sensor_y 
	  = robot_state->sensor_best_y 
	    = global_map_start_y;	/* orientation is set below */
      
      
      
      /*
       *          These are important, miscellaneous parameters
       */
      
      
      robot_state->sensor_uncertainty             = 1.0;
      program_state->map_update_pending           = 0;
      program_state->global_map_matching_mode     = 1;
      
      /*
       *          Setup the search direction
       */
      
      robot_state->sensor_org_orientation
	= robot_state->sensor_orientation 
	  = robot_state->sensor_best_orientation
	    = robot_state->map_orientation
	      - robot_state->map_orientation 
		+ (90.0 * ((float) direction));
      
      robot_state->last_change_x = 0.0;  
      robot_state->last_change_y = 0.0;  
      robot_state->last_change_orientation = 0.0;  
      robot_state->sensor_best_fit = -99999.0;
      robot_specifications->niterations_in_search = 0;
      
      /*
       * Set up the graphics
       */

      fprintf(stderr, "<<< %g >>>", robot_state->sensor_orientation);
      
      if (program_state->graphics_initialized && display){
	G_activate(BEST_FIT_POINT);
	G_display_robot(BEST_FIT_POINT, /* the green guy */
			robot_state->sensor_x,
			robot_state->sensor_y,
			robot_state->sensor_orientation,
			0, NULL);
      }
      
      
      /*
       * Print out comments
       */
      
      printf("\n----> Search %d: %g %g %g.\n", 
	     direction,
	     robot_state->sensor_best_x,
	     robot_state->sensor_best_y,
	     robot_state->sensor_best_orientation);
      
      /*
       *  ############  SEARCH LOOP #############
       */
      
      for (i = 0; i < robot_specifications->niterations_in_map_fitting; i++)
	do_search_step(robot_specifications, program_state, robot_state);      
      
      /*
       * Print out results
       */
      
      printf("----> Result: %g %g %g. Fit %g.\n", 
	     robot_state->sensor_best_x,
	     robot_state->sensor_best_y,
	     robot_state->sensor_best_orientation,
	     robot_state->sensor_best_fit);
      
      if (direction == 0 || best_fit < robot_state->sensor_best_fit){
	best_fit         = robot_state->sensor_best_fit;
	best_x           = robot_state->sensor_best_x;
	best_y           = robot_state->sensor_best_y;
	best_orientation = robot_state->sensor_best_orientation;
      }
    }
  }
  

  fprintf(stderr, "\n\tbest: %g %g %g %g\n\n",
	  best_fit, best_x, best_y, best_orientation);

  fprintf(stderr, "-1-");

  program_state->global_map_matching_mode = 0;

  if (program_state->graphics_initialized && display)
    G_deactivate(BEST_FIT_POINT);

  *correlation = best_fit;

  fprintf(stderr, "-2-");

    
  
  /************************************
   *
   * Finally, integrate the best fit! 
   */
  
  if (combine){
    fprintf(stderr, "\n\t+===========================================+\n");
    fprintf(stderr, "\t+===    integrate map %d into map %d      ====+\n",
	    base_map_number, comp_map_number);
    fprintf(stderr, "\t+===========================================+\n");
	    
    robot_state->sensor_best_x           = best_x;
    robot_state->sensor_best_y           = best_y;
    robot_state->sensor_best_orientation = best_orientation;
    fprintf(stderr, "-3-");
    

    /*
     * If we've been working on the map #2, let's now update the correction
     * parameters such that the robot is now back in map #1
     */


    G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y,
		    robot_state->orientation, 0, NULL);
    
    program_state->map_update_pending = 1;
    update_internal_map(0, robot_specifications, program_state,
			robot_state);

    fprintf(stderr, "-4-");
    
    if (1 || prev_actual_map == comp_map_number){ /*!*/
      float local_x, local_y;
      float delta_x, delta_y, delta_orientation;
      float uncorr_robot_x, uncorr_robot_y, uncorr_robot_orientation;
      int local_int_x, local_int_y;

      local_x = ((0.5 * ((float) (max_x + min_x)))
		 - ((float) robot_specifications->autoshifted_int_x))
	* robot_specifications->resolution;
      local_y = ((0.5 * ((float) (max_y + min_y)))
		 - ((float) robot_specifications->autoshifted_int_y))
	* robot_specifications->resolution;


      local_x = path[0][0];
      local_y = path[0][1];


      fprintf(stderr, ">>>>>>>>>> %g %g\n", path[0][0],
	      path[0][1]);
      
      fprintf(stderr, "Green point in comp-map has coordinates %g %g %g\n",
	      local_x, local_y, 0.0);

      fprintf(stderr, "Green point in base-map has coordinates %g %g %g\n",
	      best_x, best_y, best_orientation);

      fprintf(stderr, "-5-");

      delta_x = best_x - local_x;
      delta_y = best_y - local_y;
      delta_orientation = best_orientation - 0.0;
      for (;delta_orientation >  180.0;) delta_orientation -= 360.0;
      for (;delta_orientation < -180.0;) delta_orientation += 360.0;
      
      /*
       * G_add_marker(PATH[0], best_x, best_y, 0); 
       * G_add_marker(PATH[1], local_x, local_y, 0); 
       */

      compute_backward_correction(robot_state->x,
				  robot_state->y,
				  robot_state->orientation,
				  robot_state->correction_parameter_x,
				  robot_state->correction_parameter_y,
				  robot_state->correction_parameter_angle,
				  robot_state->correction_type,
				  &uncorr_robot_x,
				  &uncorr_robot_y, 
				  &uncorr_robot_orientation);
      
      update_correction_parameters(local_x, local_y, 0.0,
				   delta_x, delta_y, delta_orientation,
				   &(robot_state->correction_parameter_x),
				   &(robot_state->correction_parameter_y),
				   &(robot_state->correction_parameter_angle),
				   &(robot_state->correction_type)); 
				       
      compute_forward_correction(uncorr_robot_x,
				 uncorr_robot_y, 
				 uncorr_robot_orientation,
				 robot_state->correction_parameter_x,
				 robot_state->correction_parameter_y,
				 robot_state->correction_parameter_angle,
				 robot_state->correction_type,
				 &robot_state->x,
				 &robot_state->y,
				 &robot_state->orientation);
      fprintf(stderr, "-6-");

      printf("\n\t[CORR: %6.4f params: %6.4f %6.4f %6.4f %d]\n",/*!*/
	     0.0,
	     robot_state->correction_parameter_x,
	     robot_state->correction_parameter_y,
	     robot_state->correction_parameter_angle,
	     robot_state->correction_type);
      send_automatic_correction_update();
      fprintf(stderr, "-7-");

      G_activate(GLOBAL_MAPVALUES);/*!*/
      G_display_switch(GLOBAL_BACKGROUND, 0);
      G_display_matrix(GLOBAL_MAPVALUES);
      G_display_markers(PATH);
      G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y,
		      robot_state->orientation, 0, NULL);
      G_deactivate(GLOBAL_MAPVALUES);

      
    }

  }
  

  
  return 1;

}  


/************************************************************************
 *
 *   NAME:         check_and_combine_global_maps
 *                 
 *   FUNCTION:     Tries to match two global maps. Specialized to
 *                 maps 0 and 1 and robot_state->automatch_on
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 1, if we matched, 0 if not
 *                 
 ************************************************************************/




int check_and_combine_global_maps(ROBOT_SPECIFICATIONS_PTR 
				  robot_specifications,
				  PROGRAM_STATE_PTR        program_state,
				  ROBOT_STATE_PTR          robot_state)
{
  float fit;

  if (!robot_state->automatch_on)
    return 0;

  fprintf(stderr, "...made %f out of %f cm.\n",
	  robot_state->automatch_cumul_distance,
	  robot_specifications->min_advance_for_map_fitting);

  if (!robot_state->map_orientation_defined)
    return 0;

  if (robot_state->automatch_cumul_distance < 
      robot_specifications->min_advance_for_map_fitting)
    return 0;


  combine_global_maps(0, 1,
		      robot_state->automatch_pos_x,
		      robot_state->automatch_pos_y,
		      robot_state->automatch_pos_orientation,
		      1,	/* _do_ the integration */
		      1,	/* display */
		      &fit,
		      robot_specifications,
		      program_state,
		      robot_state);

  robot_state->automatch_on = 0; 


  G_display_switch(COMBINE_MAPS_BUTTON, 0);


  /*
   * Clear path
   */
  n_path_entries = 0;
  if (program_state->graphics_initialized)
    G_clear_markers(PATH);
  

  return 1;
}


/************************************************************************
 *
 *   NAME:         compute_voronoi_diagram
 *                 
 *   FUNCTION:     Computes voronoi diagram
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


#define MAX_RADIUS 30
#define CRITICAL_POINT_NEIGHBORHOOD 8
#define MAX_NUM_BASIS_POINTS 2
#define GRAPH_PLOT_CLEARANCE 4.0




static int c_dx[MAX_RADIUS][MAX_RADIUS*14]; /* here we store relative offsets*/
static int c_dy[MAX_RADIUS][MAX_RADIUS*14]; /* for circles of different sizes*/
static int c_size[MAX_RADIUS];
static int c_defined = 0;



void compute_voronoi_diagram(int                      plot_modus,
			     ROBOT_SPECIFICATIONS_PTR robot_specifications,
			     PROGRAM_STATE_PTR        program_state,
			     ROBOT_STATE_PTR          robot_state)
{
  int i, j, j1, max_j, x, y, xprime, yprime, k, l, found_wall, radius, xy, xy2;
  float cumul_x, cumul_y, cumul;
  int num_iterations, iteration;
  int   neighbor1, neighbor2, interesting;
  float alpha;
  float *global_map_ptr;
  int   *global_active_ptr;
  float *global_voronoi_ptr, *global_voronoi_ptr2;
  int   *global_voronoi_active_ptr;
  unsigned char *voronoi_type = NULL; /* 0 = occupied
				       * 1 = free space */
  unsigned char *voronoi_type_ptr;
  int *voronoi_clearance = NULL; 
  int *voronoi_clearance_ptr;
  int min_x, max_x, min_y, max_y;
  int min_x2, max_x2, min_y2, max_y2, x2, y2, xx, yy;
  int max_clearance = 0;
  int change, found;


  int voronoi_free_space_counter;
  int voronoi_occupied_space_marker;
  int voronoi_transition_counter, wall_found;
  int neighbor_is_voronoi;
  unsigned char circle_voronoi_type[MAX_RADIUS*14];
  unsigned char circle_voronoi_region_count[MAX_RADIUS*14];
  int better_neighbor_found;
  int walls_on_opposite_sides;
  
  int neighbor_dx[8] = {-1,  1,  0,  0,  1,  1, -1, -1};
  int neighbor_dy[8] = { 0,  0, -1,  1,  1, -1,  1, -1};
  int neighbor_costs[8] = {1.0, 1.0, 1.0, 1.0, 
			     1.41421, 1.41421, 1.41421, 1.41421};

  typedef struct{
    int x;			/* coordinates of the Voronoi Point */
    int y;			/* coordinates of the Voronoi Point */
    int basis_x[MAX_NUM_BASIS_POINTS]; /* coordinates of the basis points */
    int basis_y[MAX_NUM_BASIS_POINTS]; /* coordinates of the basis points */
    int num_basis_points;	/* number of basis points */
    int critical;		/* 0 = non-critical 
				 * 1 = non-critical, but there are walls
				 *     on opposite sides 
				 * 2 = critical 
				 * 3 = critical, and connected to a region with
				 *     at least 3 neighbors */
  } *voronoi_points_ptr, voronoi_points_type;
  voronoi_points_type *voronoi_points;
  int num_voronoi_points = 0;
  int num_critical_points = 0;
  int basis_i[MAX_NUM_BASIS_POINTS], new_basis_i;
  int num_basis_i;
  int active_wall_flag;
  float origin_x, delta_x, origin_y, delta_y, factor, dist1, dist2;

  int best_j, done;
  float best_costs;
  float *voronoi_DP1 = NULL; 
  float *voronoi_DP2 = NULL; 


  /*
   * topological regions
   */
  typedef struct{
    int   number;		/* number of the region */
    float float_x, float_y;	/* center of the region */
    float size;			/* size of the region */
    int   neighbors[MAX_NUM_EDGES]; /* links */
    int   num_neighbors;	/* number of links */
    int   color;		/* for color display - looks nice! */
    float costs;		/* for plannning shortest paths */
    float cumul_costs[MAX_NUM_NODES]; /* after Dynamic Prog. */
  } region_type, *region_ptr;
  region_type regions[MAX_NUM_NODES];
  int num_regions = 0;
  int *voronoi_region_name = NULL;
  int *voronoi_region_name_ptr;
  unsigned char *voronoi_graph = NULL;
  unsigned char *voronoi_graph_ptr;
  int path[MAX_NUM_NODES], path_length;
  int min_search_x, max_search_x, min_search_y, max_search_y;
  int min_search_x2, max_search_x2, min_search_y2, max_search_y2;

  /*
   * constants
   */
  static int X_COLORS[26] = 
    {C_YELLOW2, C_VIOLET, C_CYAN, C_LIMEGREEN,
       C_PINK, C_GOLD, C_CADETBLUE, C_MEDIUMVIOLETRED, C_MEDIUMPURPLE3,
       C_PALEGREEN4, C_STEELBLUE4, C_ORANGERED4, C_KHAKI4,
       C_DARKTURQUOISE, C_BLUE, C_FORESTGREEN, C_OLDLACE,
       C_PALETURQUOISE4, C_DEEPPINK, C_MAGENTA2, C_SIENNA4, C_PINK1,
       C_TURQUOISE4, C_ROYALBLUE, C_YELLOW, C_LIGHTVIOLET};
  
  static char *VORONOI_MODI[NUM_VORONOI_MODI] =
    {
      "Shaded voronoi diagram",	/* 0 */
      "Voronoi diagram with critical points/lines", /* 1 */
      "Topological regions (not pruned)", /* 2 */
      "Topological regions (not pruned, with numbers)", /* 3 */
      "Topological graph (not pruned)", /* 4 */
      "Topological regions (pruned)", /* 5 */
      "Topological regions (pruned, with numbers)",	/* 6 */
      "Topological graph (pruned)"	/* 7 */
    };
  
  /*
   * initial check
   */

  if (plot_modus < 0 || plot_modus >= NUM_VORONOI_MODI){
    fprintf(stderr, "Unknown plot_modus %d. Must exit.\n");
    return;
  }
  else
    fprintf(stderr, "COMPUTE_VORONOI_DIAGRAM: %s\n",
	    VORONOI_MODI[plot_modus]);





  /*
   * pre-calcluate the circles	(only done once)
   */

  if (!c_defined){
    for (i = 1; i < MAX_RADIUS; i++){
      max_j = i * 14;		/* we will construct the circle step-by-step */
      k = 0;			/* counter */
      for (j = 0; j < max_j; j++){
	alpha = 2.0 * M_PI * ((float) j) / ((float) max_j);
	x = (int) (sin(alpha) * (((float) i) + 0.5));
	y = (int) (cos(alpha) * (((float) i) + 0.5));
	if ((k == 0 || x != c_dx[i][k-1] || y != c_dy[i][k-1]) &&
	    (x != 0 || y != 0) &&
	    (k == 0 || x != c_dx[i][0] || y != c_dy[i][0])){
	  c_dx[i][k] = x;	/*  new point in the circle */
	  c_dy[i][k] = y;
	  k++;
	  /* printf("[%d %d] ", x, y); */
	}
      }
      c_dx[i][k] = 0;		/* special marker: end! */
      c_dy[i][k] = 0;
      c_size[i] = k;
      /* printf("Circle %d: %d (%d)\n", i, k, max_j); */
    }      
    c_defined = 1;
  }


  /*
   * alloc memory for the voronoi table
   */

  voronoi_type  = (unsigned char *)
    (malloc(sizeof(unsigned char) 
	    * robot_specifications->global_map_dim_x
	    * robot_specifications->global_map_dim_y));

  voronoi_graph  = (unsigned char *)
    (malloc(sizeof(unsigned char) 
	    * robot_specifications->global_map_dim_x
	    * robot_specifications->global_map_dim_y));

  voronoi_clearance  = (int *)
    (malloc(sizeof(int) 
	    * robot_specifications->global_map_dim_x
	    * robot_specifications->global_map_dim_y));

  voronoi_region_name  = (int *)
    (malloc(sizeof(int) 
	    * robot_specifications->global_map_dim_x
	    * robot_specifications->global_map_dim_y));

  voronoi_points  = (voronoi_points_type *)
    (malloc(sizeof(voronoi_points_type) 
	    * robot_specifications->global_map_dim_x
	    * robot_specifications->global_map_dim_y));

  voronoi_DP1  = (float *)
    (malloc(sizeof(float) 
	    * robot_specifications->global_map_dim_x
	    * robot_specifications->global_map_dim_y));

  voronoi_DP2  = (float *)
    (malloc(sizeof(float) 
	    * robot_specifications->global_map_dim_x
	    * robot_specifications->global_map_dim_y));


  /*
   * set some local pointes - makes things faster
   */

  global_map_ptr            = global_map;
  global_active_ptr         = global_active;
  global_voronoi_ptr        = global_voronoi;
  voronoi_type_ptr          = voronoi_type;
  voronoi_clearance_ptr     = voronoi_clearance;
  voronoi_region_name_ptr   = voronoi_region_name;
  global_voronoi_active_ptr = global_voronoi_active;
  voronoi_region_name_ptr   = voronoi_region_name;
  voronoi_graph_ptr         = voronoi_graph;

  /*
   * construct base-line table, initialization
   */

  min_x = max_x = min_y = max_y = -1; /* initialization */

  for (x = 0, i = 0; x < robot_specifications->global_map_dim_x; x++)
    for (y = 0; y < robot_specifications->global_map_dim_y; y++, i++){
      *global_voronoi_active_ptr++ = *global_active_ptr;
      if (*global_active_ptr++){
	if (*global_map_ptr++ > 0.97){
	  *voronoi_type_ptr++ = (unsigned char) 1; /* free space */
	  if (x < min_x || min_x == -1) min_x = x;
	  if (x > max_x || max_x == -1) max_x = x;
	  if (y < min_y || min_y == -1) min_y = y;
	  if (y > max_y || max_y == -1) max_y = y;
	}
	else
	  *voronoi_type_ptr++ = (unsigned char) 0; /* occupied */
      }
      else{
	global_map_ptr++;
	*voronoi_type_ptr++ = (unsigned char) 0; /* unexplored := occupied */
      }
      *voronoi_clearance_ptr++   = 0;	/* default */
      *voronoi_region_name_ptr++ = -1; /* default */
      *voronoi_graph_ptr++       = (unsigned char) 0;
    }

  if (min_x == -1 || min_y == -1){
    fprintf(stderr, "WARNING: No free-space found in the map - abort.\n");
    return;
  }

  min_x2 = min_x;
  max_x2 = max_x;
  min_y2 = min_y;
  max_y2 = max_y;
  if (min_x2 == 0) 
    min_x2 = 1;
  if (max_x2 == robot_specifications->global_map_dim_x - 1)
    max_x2 = robot_specifications->global_map_dim_x - 2;
  if (min_y2 == 0) 
    min_y2 = 1;
  if (max_y2 == robot_specifications->global_map_dim_y - 1)
    max_y2 = robot_specifications->global_map_dim_y - 2;


  /*
   * Identify Voronoi diagram
   */

  for (radius = 1; radius < MAX_RADIUS; radius++){
    fprintf(stderr, ".");
    for (x = min_x; x <= max_x; x++){
      voronoi_type_ptr = 
	&(voronoi_type[x * robot_specifications->global_map_dim_y + min_y]);
      voronoi_clearance_ptr = 
	&(voronoi_clearance[x * robot_specifications->global_map_dim_y 
			    + min_y]);
      for (y = min_y; y <= max_y; 
	   y++, voronoi_type_ptr++, voronoi_clearance_ptr++)
	if (*voronoi_type_ptr == 1 ||
	    *voronoi_type_ptr == 2 ||
	    *voronoi_type_ptr == 3){ /* free space with clearance >= radius */

	  /*
	   * extract the circle
	   */

	  wall_found = 0;

	  for (j = 0; j < c_size[radius]; j++){
	    xprime = x - c_dx[radius][j];
	    yprime = y - c_dy[radius][j];
	    if (xprime >= 0 && 
		xprime < robot_specifications->global_map_dim_x &&
		yprime >= 0 && 
		yprime < robot_specifications->global_map_dim_y){
	      if (voronoi_type[xprime *
			       robot_specifications->global_map_dim_y + 
			       yprime] == 0){
		circle_voronoi_type[j] = (unsigned char) 0;
		wall_found = 1;
	      }
	      else
		circle_voronoi_type[j] = (unsigned char) 1;
	    }
	    else{
	      wall_found = 1;
	      circle_voronoi_type[j] = (unsigned char) 0; /* unexplore 
							   * is like a  wall */
	    }
	  }

	  /*
	   * update clearance
	   */

	  if (!wall_found){
	    *voronoi_clearance_ptr = radius;
	    if (radius > max_clearance)
	      max_clearance = radius;
	  }

	  /*
	   * now let's compute the # changes, statistics
	   */

	  if (wall_found){
	    /**** (1) generate size of free-space regions */
	    voronoi_free_space_counter = 0; /* counts regions of free-space */
	    for (j = 0; j < 2 * c_size[radius]; j++){
	      j1 = j % c_size[radius];
	      if (circle_voronoi_type[j1] == 1){ /* free-space */
		voronoi_free_space_counter++; /* okay, found a free-space */
		circle_voronoi_region_count[j1] = 0;
	      }
	      else{		/* occupied region */
		circle_voronoi_region_count[j1] = voronoi_free_space_counter;
		voronoi_free_space_counter = 0;
	      }
	    }

	    /**** (2) check if neighbor already on Voronoi diagram,
	     ****     with lower clearance */
	    neighbor_is_voronoi = 0;
	    for (i = 0; i < 8; i++){
	      xprime = x + neighbor_dx[i];
	      yprime = y + neighbor_dy[i];
	      if (xprime >= 0 && 
		  xprime < robot_specifications->global_map_dim_x &&
		  yprime >= 0 && 
		  yprime < robot_specifications->global_map_dim_y)
		if (voronoi_type[xprime *
				 robot_specifications->global_map_dim_y + 
				 yprime] == 10 /* &&
		    voronoi_clearance[xprime *
				      robot_specifications->global_map_dim_y + 
				      yprime] < *voronoi_clearance_ptr */)
		  neighbor_is_voronoi = 1;
	    }

	    /**** (3) remove "uninteresting" transitions */
	    if (neighbor_is_voronoi){
	      for (j = 0; j < c_size[radius]; j++) /* now count! */
		if (circle_voronoi_region_count[j] <
		    (c_size[radius] / 40 + 2))
		  circle_voronoi_region_count[j] = 0;
	    }
	    else{
	      for (j = 0; j < c_size[radius]; j++) /* now count! */
		if (circle_voronoi_region_count[j] <
		    (c_size[radius] / 6 + 3))
		  circle_voronoi_region_count[j] = 0;
	    }


	    /**** (4) mark the walls of the "interesting" transitions */
	    active_wall_flag = 0;
	    for (j = 0; j < 2 * c_size[radius]; j++){ /* mark all walls by
						       * 2 that are used
						       * in the voronoi count*/
	      j1 = j % c_size[radius];
	      if (circle_voronoi_region_count[j1] > 0 &&
		  circle_voronoi_type[j1] == 1)
		fprintf(stderr, "ERROR ");
	      else if (circle_voronoi_region_count[j1] > 0){
		active_wall_flag = 1;
		circle_voronoi_type[j1] = 2; /* special wall! */
	      }
	      else if (active_wall_flag && circle_voronoi_type[j1] == 0)
		circle_voronoi_type[j1] = 2; /* special wall! */
	      else if (circle_voronoi_type[j1] == 1)
		active_wall_flag = 0;
	    }


	    /**** (5) check, if within the interesting transitions
	     ****     walls exist on opposite sides
	     ****     and mark these walls (=3) */
	    walls_on_opposite_sides = 0; /* flag, indicates if walls exist
					  * on exactly the opposite side */
	    for (j = 0; j < c_size[radius]; j++){
	      j1 = (j + (c_size[radius] / 2)) % c_size[radius];
	      if (circle_voronoi_type[j]  == 2 && /* check for opp. walls */
		  circle_voronoi_type[j1] == 2){
		walls_on_opposite_sides = 1;
		circle_voronoi_type[j]  = 3; /* special interesting wall */
		circle_voronoi_type[j1] = 3; /* special interesting wall */
	      }
	    }

	    /**** (6) count the # transitions */
	    voronoi_transition_counter = 0; /* counter for occ/free space
					     * transitions */
	    for (j = 0; j < c_size[radius]; j++) /* now count! */
	      if (circle_voronoi_region_count[j] > 0)
		voronoi_transition_counter++;


	    /**** (7) generate basis angles */
	    num_basis_i = 0;	/* find basis points */
	    voronoi_occupied_space_marker = -1; /* marks first occupied i */
	    for (j = 0; j < 2 * c_size[radius]; j++){
	      j1 = j % c_size[radius];
	      
	      if (circle_voronoi_type[j1] >= 3 &&
		  voronoi_occupied_space_marker == -1) /* transition! */
		voronoi_occupied_space_marker = j; /* memorize that number */
	      else if (circle_voronoi_type[j1] < 3){ /* non-interesting reg.*/
		if (voronoi_occupied_space_marker != -1){ /* we memorized j */
		  /* insert the basis point into list */
		  found = 0;
		  new_basis_i = ((j + voronoi_occupied_space_marker) / 2)
		    % c_size[radius]; /* memorize center of occ. reg. */
		  for (l = 0; l < num_basis_i && !found; l++)
		    if (basis_i[l] == new_basis_i)
		      found = 1;
		  if (!found && num_basis_i < MAX_NUM_BASIS_POINTS)
		    basis_i[num_basis_i++] = new_basis_i;
		  /* also insert the basis point on the opposite side
		   * will only work if we keep looking for pairs of 
		   * basis points on opposite sides */
		  found = 0;
		  new_basis_i = (((j + voronoi_occupied_space_marker) / 2) +
				 (c_size[radius] / 2))
		    % c_size[radius]; /* memorize opp. center of occ. reg. */
		  for (l = 0; l < num_basis_i && !found; l++)
		    if (basis_i[l] == new_basis_i)
		      found = 1;
		  if (!found && num_basis_i < MAX_NUM_BASIS_POINTS)
		    basis_i[num_basis_i++] = new_basis_i;
		  voronoi_occupied_space_marker = -1;
		}
	      }
	    }
	    	      
	    /**** (8) check if Voronoi point */

	    if (voronoi_transition_counter >= 2){
	      *voronoi_type_ptr = 10; /* found a Voronoi point */
	      voronoi_points[num_voronoi_points].x = x;
	      voronoi_points[num_voronoi_points].y = y;
	      for (i = 0; i < num_basis_i; i++){
		voronoi_points[num_voronoi_points].basis_x[i] = 
		  x - c_dx[radius][basis_i[i]]; 
		voronoi_points[num_voronoi_points].basis_y[i] = 
		  y - c_dy[radius][basis_i[i]]; 
	      }		
	      voronoi_points[num_voronoi_points].num_basis_points = 
		num_basis_i;
	      if (!walls_on_opposite_sides)
		voronoi_points[num_voronoi_points].critical = 0;
	      else
		voronoi_points[num_voronoi_points].critical = 1;
	      num_voronoi_points++;	      

	    }
	    else if (*voronoi_type_ptr <= 1)
	      *voronoi_type_ptr += 1; /* okay, last chance */
	    else
	      *voronoi_type_ptr = 20; /* not a Voronoi point */
	  }
	}
    }
  }

  /*
   * find critical points
   */

  fprintf(stderr, "\n%d voronoi points\n", num_voronoi_points);

  for (i = 0; i < num_voronoi_points; i++)
    if (voronoi_points[i].critical){ /* points on the opposite side - 
				      * precondition for critical point */
      xy = voronoi_points[i].x * robot_specifications->global_map_dim_y +
	voronoi_points[i].y;
      voronoi_type_ptr      = &(voronoi_type[xy]);
      voronoi_clearance_ptr = &(voronoi_clearance[xy]);
      better_neighbor_found = 0;
      for (radius = 1; radius < CRITICAL_POINT_NEIGHBORHOOD; radius++)
	for (j = 0; j < c_size[radius]; j++){
	  xprime = voronoi_points[i].x - c_dx[radius][j];
	  yprime = voronoi_points[i].y - c_dy[radius][j];
	  if (xprime >= 0 && 
	      xprime < robot_specifications->global_map_dim_x &&
	      yprime >= 0 && 
	      yprime < robot_specifications->global_map_dim_y)
	    if (voronoi_type[xprime *
			     robot_specifications->global_map_dim_y + 
			     yprime] == 11 ||	/* already critical point */
		(voronoi_type[xprime *
			      robot_specifications->global_map_dim_y + 
			      yprime] == 10 &&	/* regular Voronoi point */
		 voronoi_clearance[xprime * /* but smaller clearance */
				   robot_specifications->global_map_dim_y + 
				   yprime] < *voronoi_clearance_ptr))
	      better_neighbor_found = 1;
	}
      if (!better_neighbor_found){
	*voronoi_type_ptr = 11;	/* found critical point */
	voronoi_points[i].critical = 2;
	num_critical_points++;
      }
    }

  fprintf(stderr, "\n%d critical points\n", num_critical_points);


  /*
   * draw critical lines
   */
  if (plot_modus != 0){
    for (i = 0; i < num_voronoi_points; i++)
      if (voronoi_points[i].critical == 2){
	for (j = 0; j < voronoi_points[i].num_basis_points; j++)
	  for (k = 0; k <= MAX_RADIUS * 2; k++){
	    xprime = (int) (0.0 + ((float) voronoi_points[i].x) +
			    (((float) (k)) / ((float) (MAX_RADIUS * 2))) *
			    ((float) (voronoi_points[i].basis_x[j] -
				      voronoi_points[i].x)));
	    yprime = (int) (0.0 + ((float) voronoi_points[i].y) +
			    (((float) (k)) / ((float) (MAX_RADIUS * 2))) *
			    ((float) (voronoi_points[i].basis_y[j] -
				      voronoi_points[i].y)));
	    xy = xprime * robot_specifications->global_map_dim_y + yprime;
	    voronoi_type[xy] = 11;
	  }
      }
  }


  num_regions = 0;
  if (plot_modus == 5 || plot_modus == 6 || plot_modus == 7)
    num_iterations = 2;		/* twice, if regions are fused */
  else if (plot_modus == 0 || plot_modus == 1)	
    num_iterations = 0;		/* only Voronoi diagra */
  else
    num_iterations = 1;		/* normal operation */
  for (iteration = 0; iteration < num_iterations; iteration++){
    
    /*
     * compute regions
     */
    
    num_regions = 0;
    for (i = 0, x = min_x; x <= max_x; x++){ /* initialization */
      voronoi_region_name_ptr = 
	&(voronoi_region_name[x * robot_specifications->global_map_dim_y
			      + min_y]);
      voronoi_type_ptr = 
	&(voronoi_type[x * robot_specifications->global_map_dim_y + min_y]);
      for (y = min_y; y <= max_y; 
	   y++, voronoi_region_name_ptr++, voronoi_type_ptr++){
	if (*voronoi_type_ptr == 0  || 
	    (iteration == 0 && *voronoi_type_ptr == 11) || 
	    *voronoi_type_ptr == 12)
	  *voronoi_region_name_ptr = -1;
	else
	  *voronoi_region_name_ptr = i++;
      }
    }
	  
    
    do{				/* fusion of regions */
      change = 0;
      fprintf(stderr, "+");
      for (x = min_x; x <= max_x; x++){
	voronoi_region_name_ptr = 
	  &(voronoi_region_name[x * robot_specifications->global_map_dim_y
				+ min_y]);
	for (y = min_y; y <= max_y; y++, voronoi_region_name_ptr++){
	  if (*voronoi_region_name_ptr != -1){
	    for (i = 0; i < 4; i++){
	      xprime = x + neighbor_dx[i];
	      yprime = y + neighbor_dy[i];
	      if (xprime >= 0 && 
		  xprime < robot_specifications->global_map_dim_x &&
		  yprime >= 0 && 
		  yprime < robot_specifications->global_map_dim_y){
		xy = xprime * robot_specifications->global_map_dim_y + yprime;
		if (voronoi_region_name[xy] != -1){
		  if (*voronoi_region_name_ptr < voronoi_region_name[xy]){
		    voronoi_region_name[xy] = *voronoi_region_name_ptr;
		    change = 1;
		  }
		  else if (*voronoi_region_name_ptr > voronoi_region_name[xy]){
		    *voronoi_region_name_ptr = voronoi_region_name[xy];
		    change = 1;
		  }
		}
	      }
	    }
	  }
	}
      }
    } while (change);
    


    for (x = min_x; x <= max_x; x++){ /* identify regions, and re-color them */
      voronoi_region_name_ptr = 
	&(voronoi_region_name[x * robot_specifications->global_map_dim_y
			      + min_y]);
      for (y = min_y; y <= max_y; y++, voronoi_region_name_ptr++){
	if (*voronoi_region_name_ptr != -1){
	  found = 0;
	  for (i = 0; i < num_regions && !found; i++)
	    if (regions[i].number == *voronoi_region_name_ptr)
	      found = 1;
	  if (!found){ /* new region */
	    if (num_regions < MAX_NUM_NODES){
	      regions[num_regions].number        = *voronoi_region_name_ptr;
	      regions[num_regions].size          = 0;
	      regions[num_regions].num_neighbors = 0;
	      regions[num_regions].color         = -1;
	      regions[num_regions].costs         = 1.0;
	      for (l = 0; l < MAX_NUM_NODES; l++)
		regions[num_regions].cumul_costs[l] = -1.0; /* special! */
	      regions[num_regions].cumul_costs[num_regions] = 0.0;
	      *voronoi_region_name_ptr           = num_regions; /* rename! */
	      num_regions                        += 1;
	    }
	    else{
	      static int this_printed = 0;
	      if (!this_printed)
		fprintf(stderr, 
			"ERROR: Too many regions. Increase MAX_NUM_NODES.\n");
	      this_printed = 1;
	    }
	  }
	  else			/* region known */
	    *voronoi_region_name_ptr = i-1; /* rename! */
	}
      }
    }

    fprintf(stderr, "\n%d topological regions\n", num_regions);
    
    
    /* the adjacency matrix is computed by reconstructing critical lines. */
    for (i = 0; i < num_voronoi_points; i++) /* compute adjacency matrix */
      if ((iteration == 0 && voronoi_points[i].critical == 2) ||
	  (iteration == 1 && voronoi_points[i].critical == 3)){ /* critical */
	for (j = 0; j < voronoi_points[i].num_basis_points; j++){
	  neighbor1 = neighbor2 = -1; /* no neighbor found yet */
	  for (k = 0; k <= MAX_RADIUS * 2 && neighbor2 == -1; 
	       k++){ /* re-build critical line */
	    xprime = (int) (0.0 + ((float) voronoi_points[i].x) +
			    (((float) (k)) / ((float) (MAX_RADIUS * 2))) *
			    ((float) (voronoi_points[i].basis_x[j] -
				      voronoi_points[i].x)));
	    yprime = (int) (0.0 + ((float) voronoi_points[i].y) +
			    (((float) (k)) / ((float) (MAX_RADIUS * 2))) *
			    ((float) (voronoi_points[i].basis_y[j] -
				      voronoi_points[i].y)));
	    for (l = 1; l < 8; l++){
	      x = xprime + neighbor_dx[l];
	      y = yprime + neighbor_dy[l];
	      if (x >= 0 && 
		  x < robot_specifications->global_map_dim_x &&
		  y >= 0 && 
		  y < robot_specifications->global_map_dim_y){
		xy = x * robot_specifications->global_map_dim_y + y;
		if (voronoi_region_name[xy] >= 0){
		  if (neighbor1 == -1)
		    neighbor1 = voronoi_region_name[xy];
		  else if (neighbor1 != voronoi_region_name[xy])
		    neighbor2 = voronoi_region_name[xy]; /* done! */
		}
	      }
	    }
	  }
	  if (neighbor2 == -1)
	    fprintf(stderr, 
		    "STRANGE: critical line does not connect 2 regs.\n");
	  /* else
	     fprintf(stderr, "Neighbor: %d %d\n", neighbor1, neighbor2); */
	  
	  for (k = 0, 
	       found = 0; 
	       k < regions[neighbor1].num_neighbors && !found; 
	     k++)
	    if (regions[neighbor1].neighbors[k] == neighbor2)
	      found = 1;
	  if (!found){		/* new link - insert into the adjacency table*/
	    if (regions[neighbor1].num_neighbors < MAX_NUM_EDGES){
	      regions[neighbor1].neighbors[regions[neighbor1].num_neighbors]
		= neighbor2;
	      regions[neighbor1].num_neighbors += 1;
	    }
	    else
	      fprintf(stderr, "WARNING: Too may edges for region %d.\n",
		      neighbor1);
	    if (regions[neighbor2].num_neighbors < MAX_NUM_EDGES){
	      regions[neighbor2].neighbors[regions[neighbor2].num_neighbors]
		= neighbor1;
	      regions[neighbor2].num_neighbors += 1;
	    }
	    else
	      fprintf(stderr, "WARNING: Too may edges for region %d.\n",
		      neighbor2);
	  }
	}
      }
    
    

    
    /*
     * fuse regions again: insignificant regions (with <=2 neighnors)
     */
    
    if (iteration == 0 && 
	(plot_modus == 5 || plot_modus == 6 || plot_modus == 7)){
      
      
      /* compute the interesting critical lines (>=3 neighbors...) */
      for (i = 0; i < num_voronoi_points; i++) /* compute adjacency matrix */
	if (voronoi_points[i].critical == 2){ /* critical point */
	  interesting = 0;
	  for (j = 0;
	       j < voronoi_points[i].num_basis_points && !interesting; j++){
	    neighbor1 = neighbor2 = -1; /* no neighbor found yet */
	    for (k = 0; k <= MAX_RADIUS * 2 && neighbor2 == -1; 
		 k++){ /* re-build critical line */
	      xprime = (int) (0.0 + ((float) voronoi_points[i].x) +
			      (((float) (k)) / ((float) (MAX_RADIUS * 2))) *
			      ((float) (voronoi_points[i].basis_x[j] -
					voronoi_points[i].x)));
	      yprime = (int) (0.0 + ((float) voronoi_points[i].y) +
			      (((float) (k)) / ((float) (MAX_RADIUS * 2))) *
			      ((float) (voronoi_points[i].basis_y[j] -
					voronoi_points[i].y)));
	      for (l = 1; l < 8; l++){
		x = xprime + neighbor_dx[l];
		y = yprime + neighbor_dy[l];
		if (x >= 0 && 
		    x < robot_specifications->global_map_dim_x &&
		    y >= 0 && 
		    y < robot_specifications->global_map_dim_y){
		  xy = x * robot_specifications->global_map_dim_y + y;
		  if (voronoi_region_name[xy] >= 0){
		    if (neighbor1 == -1)
		      neighbor1 = voronoi_region_name[xy];
		    else if (neighbor1 != voronoi_region_name[xy])
		      neighbor2 = voronoi_region_name[xy]; /* done! */
		  }
		}
	      }
	    }
	    if (!(regions[neighbor1].num_neighbors <= 1) &&
		!(regions[neighbor2].num_neighbors <= 1) &&
		!(regions[neighbor1].num_neighbors == 2 &&
		  regions[neighbor2].num_neighbors == 2))
	      interesting = 1;
	  }
	  if (interesting)
	    voronoi_points[i].critical = 3;
	}
    
      
      /*
       * color interesting critical lines differently
       */
      
      for (i = 0; i < num_voronoi_points; i++)
	if (voronoi_points[i].critical == 3){
	  for (j = 0; j < voronoi_points[i].num_basis_points; j++)
	    for (k = 0; k <= MAX_RADIUS * 2; k++){
	      xprime = (int) (0.0 + ((float) voronoi_points[i].x) +
			      (((float) (k)) / ((float) (MAX_RADIUS * 2))) *
			      ((float) (voronoi_points[i].basis_x[j] -
					voronoi_points[i].x)));
	      yprime = (int) (0.0 + ((float) voronoi_points[i].y) +
			      (((float) (k)) / ((float) (MAX_RADIUS * 2))) *
			      ((float) (voronoi_points[i].basis_y[j] -
					voronoi_points[i].y)));
	      xy = xprime * robot_specifications->global_map_dim_y + yprime;
	      voronoi_type[xy] = 12;
	    }
	}
    }
  }  


  /*
   * graphical stuff: compute labels and color
   */

  
  G_clear_markers(TOPOLOGICAL_GRAPH); /* graphical stuff */

  for (i = 0; i < num_regions; i++){
    j = 0;
    do{
      found = 0;
      for (k = 0; k < regions[i].num_neighbors && !found; k++)
	if (regions[regions[i].neighbors[k]].color == j)
	  found = 1;
      if (found) j++;
    } while (found);
    regions[i].color = j;

    if (regions[i].num_neighbors > 0){
      cumul_x = cumul_y = cumul = 0.0;
      for (x = min_x; x <= max_x; x++){
	voronoi_region_name_ptr = 
	  &(voronoi_region_name[x * robot_specifications->global_map_dim_y
				+ min_y]);
	for (y = min_y; y <= max_y; y++, voronoi_region_name_ptr++){
	  if (*voronoi_region_name_ptr == i){
	    cumul   += 1.0;
	    cumul_x += ((float) x);
	    cumul_y += ((float) y);
	  }
	}
      }
      if (cumul == 0.0){
	fprintf(stderr, "WARNING: Error in code.\n");
	cumul = 1.0;
      }
      regions[i].size    = cumul * robot_specifications->resolution 
	* robot_specifications->resolution;
      regions[i].float_x = cumul_x / cumul * robot_specifications->resolution 
	- robot_specifications->autoshifted_x;
      regions[i].float_y = cumul_y / cumul * robot_specifications->resolution 
	- robot_specifications->autoshifted_y;
      
      if (plot_modus == 3 || plot_modus == 6 ||
	  plot_modus == 4 || plot_modus == 7)
	G_add_marker(TOPOLOGICAL_GRAPH, 
		     regions[i].float_x, regions[i].float_y, i);
      
      fprintf(stderr, "Region %d: size=%g x=%g y=%g neighbors=", 
	      i, regions[i].size, regions[i].float_x, regions[i].float_y);
      for (j = 0; j < regions[i].num_neighbors; j++)
	if (j == regions[i].num_neighbors-1)
	  fprintf(stderr, "%d\n", regions[i].neighbors[j]);
	else
	  fprintf(stderr, "%d,", regions[i].neighbors[j]);
    }
  }
  

  /*
   * now, finally: if we only like to see a graph, we have to 
   * compute a new table
   */

  if (plot_modus == 4 || plot_modus == 7){
    for (i = 0; i < num_regions; i++)
      for (j = 0; j < regions[i].num_neighbors; j++)
	for (k = 0; k <= robot_specifications->global_map_dim_y * 2; k++){
	  origin_x = (regions[i].float_x + 
		      robot_specifications->autoshifted_x) / 
			robot_specifications->resolution;
	  delta_x  = (regions[regions[i].neighbors[j]].float_x 
		       - regions[i].float_x) / 
		      robot_specifications->resolution;

	  origin_y = (regions[i].float_y + 
		      robot_specifications->autoshifted_y) / 
			robot_specifications->resolution;
	  delta_y  = (regions[regions[i].neighbors[j]].float_y 
		      - regions[i].float_y) / 
		      robot_specifications->resolution;
	  factor   = ((float) (k) / 
		      ((float) (robot_specifications->global_map_dim_y * 2)));
	  dist1    = factor * sqrt((delta_x * delta_x) + (delta_y * delta_y));
	  dist2    = (1.0 - factor)
	    * sqrt((delta_x * delta_x) + (delta_y * delta_y));


	  if (dist1 >= GRAPH_PLOT_CLEARANCE && dist2 >= GRAPH_PLOT_CLEARANCE){
	    xprime = (int) (origin_x + ((factor * delta_x)));
	    yprime = (int) (origin_y + ((factor * delta_y)));
	    
	    if (xprime >= 0 && 
		xprime < robot_specifications->global_map_dim_x &&
		yprime >= 0 && 
		yprime < robot_specifications->global_map_dim_y){
	      xy = xprime * robot_specifications->global_map_dim_y + yprime;
	      voronoi_graph[xy] = (unsigned char) 1;
	    }
	  }
	}
  }

#ifdef PLANNING_STATISTICS_FOR_THE_PAPER

  /*
   * do some dynamic programming/Dijkstra(?) to plan shortest paths!
   */

  if (num_regions > 0){
    do{
      change = 0;
      fprintf(stderr, ":");
      for (i = 0; i < num_regions; i++)
	for (j = 0; j < regions[i].num_neighbors; j++)
	  for (k = 0; k < num_regions; k++)
	    if (regions[regions[i].neighbors[j]].cumul_costs[k] != -1.0)
	      if (regions[i].cumul_costs[k] == -1.0 ||
		  regions[i].cumul_costs[k] > 
		  regions[regions[i].neighbors[j]].cumul_costs[k] +
		  regions[i].costs){
		regions[i].cumul_costs[k] = 
		  regions[regions[i].neighbors[j]].cumul_costs[k] +
		    regions[i].costs;
		change = 1;
	      }
    } while (change);
    fprintf(stderr, "\n");
    
    
    /*
       for (i = 0; i < num_regions; i++){
       fprintf(stderr, "REGION %d:", i);
       for (k = 0; k < num_regions; k++)
       fprintf(stderr, " %g(%d)", regions[i].cumul_costs[k], k);
       fprintf(stderr, "\n");
       }
       */   
    
    
    /*
     * Do off-line region planning
     */
    
    
    done = 0;
    for (i = 0; i < num_regions && !done; i++)
      for (k = 0; k < num_regions && !done; k++)
	if (i != k){
	  if (regions[i].cumul_costs[k] != -1.0){
	    /* fprintf(stderr, "Shortest path from %d to %d (costs=%g): [%d", 
	       i, k, regions[i].cumul_costs[k], i); */
	    j = i;
	    do{
	      best_j = -1;
	      best_costs = 1.0 + regions[j].cumul_costs[k];
	      for (l = 0; l < regions[j].num_neighbors; l++)
		if (regions[regions[j].neighbors[l]].cumul_costs[k] 
		    < best_costs){
		  best_costs = regions[regions[j].neighbors[l]].cumul_costs[k];
		  best_j = regions[j].neighbors[l];
		}
	      if (best_j == -1 || best_j == j){
		fprintf(stderr, "STRANGE: error in code - find me!\n");
		done = 1;
	      }
	      /* fprintf(stderr, " %d", best_j); */
	      j = best_j;
	    } while (j != k && !done);
	    /* fprintf(stderr, "]\n"); */
	  }
	  else{
	    /* fprintf(stderr, "Shortest path from %d to %d: DOES NOT EXIST\n", 
	       i, k);  */
	  }
	}

    
    /*
     * Compare the -optimal- plan vs. the region-plan
     * (for the paper: tells us how much we suffer through the abstration)
     */
    
    for (x = min_x2; x <= max_x2; x += 3) /* goal point */
      for (y = min_y2; y <= max_y2; y += 3) /* goal point */
	if (global_map[x * robot_specifications->global_map_dim_y + y] 
	    > 0.75 &&
	    voronoi_region_name[x * robot_specifications->global_map_dim_y 
				+ y] >= 0 /*! >= 0 */){ /* part of a region */
	  /*
	   * clear DP table
	   */
	  for (xx = min_x2-1; xx <= max_x2+1; xx++)
	    for (yy = min_y2-1; yy <= max_y2+1; yy++){
	      xy = (xx * robot_specifications->global_map_dim_y) + yy;
	      if (global_map[xy] > 0.75)
		voronoi_DP1[xy] = -1.0; /* infinity */
	      else
		voronoi_DP1[xy] = -2.0; /* occupied */
	    }
	  
	  voronoi_DP1[x * robot_specifications->global_map_dim_y + y] = 0.0;
	  /*
	   * do DP
	   */
	  
	  
	  min_search_x2 = x - 1; /* initial search window */
	  max_search_x2 = x + 1;
	  min_search_y2 = y - 1;
	  max_search_y2 = y + 1;
	  do{
	    change = 0;
	    min_search_x = min_search_x2;
	    max_search_x = max_search_x2;
	    min_search_y = min_search_y2;
	    max_search_y = max_search_y2;
	    if (min_search_x < min_x2) min_search_x = min_x2;
	    if (max_search_x > max_x2) max_search_x = max_x2;
	    if (min_search_y < min_y2) min_search_y = min_y2;
	    if (max_search_y > max_y2) max_search_y = max_y2;
	    max_search_x2 = min_search_x;
	    min_search_x2 = max_search_x;
	    max_search_y2 = min_search_y;
	    min_search_y2 = max_search_y;
	    for (xx = min_search_x; xx <= max_search_x; xx++)
	      for (yy = min_search_y; yy <= max_search_y; yy++){
		xy = (xx * robot_specifications->global_map_dim_y) + yy;
		if (voronoi_DP1[xy] >= -1.0){
		  for (l = 0; l < 8; l++){
		    xy2 = ((xx + neighbor_dx[l])
			   * robot_specifications->global_map_dim_y)
		      + (yy + neighbor_dy[l]);
		    if (voronoi_DP1[xy2] >= 0.0){
		      if (voronoi_DP1[xy] == -1.0 ||
			  voronoi_DP1[xy] > (voronoi_DP1[xy2]
					     + neighbor_costs[l])){
			voronoi_DP1[xy] = 
			  voronoi_DP1[xy2] + neighbor_costs[l];
			change = 1;
			if (xx < min_search_x2) min_search_x2 = xx;
			if (xx > max_search_x2) max_search_x2 = xx;
			if (yy < min_search_y2) min_search_y2 = yy;
			if (yy > max_search_y2) max_search_y2 = yy;
		      }
		    }
		  }
		}
	      }
	    min_search_x2--;
	    max_search_x2++;
	    min_search_y2--;
	    max_search_y2++;
	  } while (change);
	  
	  
	  /*
	   * generate a starting region
	   */
	  
	  for (i = 0; i < num_regions; i++){
	    
	    /*
	     * determine the topological path.
	     */ 
	    
	    path_length = 0;
	    done = 0;
	    k = voronoi_region_name[x * robot_specifications->global_map_dim_y 
				    + y];
	    
	    if (i != k){	/* both points shall be in different regions */
	      if (regions[i].cumul_costs[k] != -1.0){
		j = i;
		path[path_length++] = i;
		fprintf(stderr, "# %d %d [%d", i, k, i);
		do{
		  best_j = -1;
		  best_costs = 1.0 + regions[j].cumul_costs[k];
		  for (l = 0; l < regions[j].num_neighbors; l++)
		    if (regions[regions[j].neighbors[l]].cumul_costs[k] 
			< best_costs){
		      best_costs = 
			regions[regions[j].neighbors[l]].cumul_costs[k];
		      best_j = regions[j].neighbors[l];
		    }
		  if (best_j == -1 || best_j == j){
		    fprintf(stderr, "STRANGE: error in code - find me!\n");
		    done = 1;
		  }
		  j = best_j;
		  path[path_length++] = j;	
		  fprintf(stderr, ",%d", j);
		} while (j != k && !done);
		fprintf(stderr, "]\n");
	      }
	      else{
		fprintf(stderr, 
			"Shortest path from %d to %d: DOES NOT EXIST\n", 
			i, k);
	      }
	      
	      /*
	       * determine the constrained optimal path: clear DP table
	       */
	      
	      for (xx = min_x2-1; xx <= max_x2+1; xx++)
		for (yy = min_y2-1; yy <= max_y2+1; yy++){
		  xy = (xx * robot_specifications->global_map_dim_y) + yy;
		  if (global_map[xy] > 0.75){
		    if (voronoi_region_name[xy] >= 0)
		      found = 0;
		    else
		      found = 1;
		    for (l = 0; l < path_length && !found; l++)
		      if (voronoi_region_name[xy] == path[l]) /* must be on
							       * the topol.
							       * path */
			found = 1;
		    if (found)
		      voronoi_DP2[xy] = -1.0; /* infinity */
		    else
		      voronoi_DP2[xy] = -2.0; /* occupied */
		  }
		  else
		    voronoi_DP2[xy] = -2.0; /* occupied */
		}
	      
	      voronoi_DP2[x * robot_specifications->global_map_dim_y + y] 
		= 0.0;

	      /*
	       * do DP
	       */
	    
	      
	      min_search_x2 = x - 1; /* initial search window */
	      max_search_x2 = x + 1;
	      min_search_y2 = y - 1;
	      max_search_y2 = y + 1;
	      do{
		change = 0;
		min_search_x = min_search_x2;
		max_search_x = max_search_x2;
		min_search_y = min_search_y2;
		max_search_y = max_search_y2;
		if (min_search_x < min_x2) min_search_x = min_x2;
		if (max_search_x > max_x2) max_search_x = max_x2;
		if (min_search_y < min_y2) min_search_y = min_y2;
		if (max_search_y > max_y2) max_search_y = max_y2;
		max_search_x2 = min_search_x;
		min_search_x2 = max_search_x;
		max_search_y2 = min_search_y;
		min_search_y2 = max_search_y;
		for (xx = min_search_x; xx <= max_search_x; xx++)
		  for (yy = min_search_y; yy <= max_search_y; yy++){
		    xy = (xx * robot_specifications->global_map_dim_y) + yy;
		    if (voronoi_DP2[xy] >= -1.0){
		      for (l = 0; l < 8; l++){
			xy2 = ((xx + neighbor_dx[l])
			       * robot_specifications->global_map_dim_y)
			  + (yy + neighbor_dy[l]);
			if (voronoi_DP2[xy2] >= 0.0){
			  if (voronoi_DP2[xy] == -1.0 ||
			      voronoi_DP2[xy] > (voronoi_DP2[xy2]
						 + neighbor_costs[l])){
			    voronoi_DP2[xy] = 
			      voronoi_DP2[xy2] + neighbor_costs[l];
			    change = 1;
			    if (xx < min_search_x2) min_search_x2 = xx;
			    if (xx > max_search_x2) max_search_x2 = xx;
			    if (yy < min_search_y2) min_search_y2 = yy;
			    if (yy > max_search_y2) max_search_y2 = yy;
			  }
			}
		      }
		    }
		  }
		min_search_x2--;
		max_search_x2++;
		min_search_y2--;
		max_search_y2++;
	      } while (change);
	    }
	    
	    
	    
	    /*
	     * generate starting point within region i
	     */ 
	    
	    
	    for (x2 = min_x2; x2 <= max_x2; x2++) /* start point */
	      for (y2 = min_y2; y2 <= max_y2; y2++) /* start point */
		if (global_map[x2 * robot_specifications->global_map_dim_y 
			       + y2] > 0.75 &&
		    voronoi_region_name[x2 * 
					robot_specifications->global_map_dim_y 
					+ y2] == i && 
		    (1 || x2 > x || (x2 == x && y2 > y))){
		  
		  /*
		   * compute path lengths
		   */
		  xy = (x2 * robot_specifications->global_map_dim_y) + y2;
		  
		  if (i == k){
#ifdef long
		    fprintf(stderr, 
			    "%d %d (%d) to %d %d (%d):",
			    x2, y2, i, x, y, k);
		    fprintf(stderr, " %g %g  \t0\n", 
			    voronoi_DP1[xy], voronoi_DP1[xy]);
#else
		    fprintf(stderr, "%g %g\n", 
			    voronoi_DP1[xy], voronoi_DP1[xy]);
#endif
		  }
		  else{
#ifdef long
		    fprintf(stderr, 
			    "%d %d (%d) to %d %d (%d):",
			    x2, y2, i, x, y, k);
		    fprintf(stderr, " %g %g  \t%g\n", 
			    voronoi_DP1[xy], voronoi_DP2[xy],
			    voronoi_DP2[xy] - voronoi_DP1[xy]);
		    
#else
		    fprintf(stderr, "%g %g\n", 
			    voronoi_DP1[xy], voronoi_DP2[xy]);
#endif
		  }
		}
	  }
	}
    
  }
#endif


  
  /*
   * display
   */

  global_active_ptr         = global_active;
  global_voronoi_ptr        = global_voronoi;
  voronoi_type_ptr          = voronoi_type;
  voronoi_clearance_ptr     = voronoi_clearance;
  global_map_ptr            = global_map;
  global_voronoi_active_ptr = global_voronoi_active;
  voronoi_region_name_ptr   = voronoi_region_name;
  voronoi_graph_ptr         = voronoi_graph;
  
  for (x = 0; x < robot_specifications->global_map_dim_x; x++)
    for (y = 0; y < robot_specifications->global_map_dim_y; y++){
      if (*global_active_ptr){

	if (plot_modus == 0){	/* regular Voronoi diagram, walls black,
				 * shaded: clearance */
	  if (*global_map_ptr < 0.75)
	    *global_voronoi_ptr = 0.0;
	  else if (*voronoi_type_ptr == 10) /* regular Voronoi point */
	    *global_voronoi_ptr = 2.0;
	  else if (*voronoi_type_ptr == 11) /* critical Voronoi point */
	    *global_voronoi_ptr = 2.0;
	  else if (*voronoi_type_ptr == 12) /* int. critical Voronoi point */
	    *global_voronoi_ptr = -1.0;
	  else if (*voronoi_type_ptr == 0)
	    *global_voronoi_ptr = 0.0;
	  else if (max_clearance > 0)
	    *global_voronoi_ptr = 1.0 - (((float) *voronoi_clearance_ptr) 
					 / ((float) max_clearance)); 
	  else
	    *global_voronoi_ptr = 1.0;
	}

	else if (plot_modus == 1){ /* regular Voronoi diagram, critical
				    * points, walls black */
	  if (*global_map_ptr < 0.75)
	    *global_voronoi_ptr = 0.0;
	  else if (*voronoi_type_ptr == 10) /* regular Voronoi point */
	    *global_voronoi_ptr = 2.0;
	  else if (*voronoi_type_ptr == 11) /* critical Voronoi point */
	    *global_voronoi_ptr = -1.0;
	  else if (*voronoi_type_ptr == 12) /* int. critical Voronoi point */
	    *global_voronoi_ptr = -1.0;
	  else if (*voronoi_type_ptr == 0)
	    *global_voronoi_ptr = 0.0;
	  else
	    *global_voronoi_ptr = 1.0;
	}

	else if (plot_modus == 2 ||
		 plot_modus == 3){ /* regions */
	  if (*global_map_ptr < 0.75)
	    *global_voronoi_ptr = 0.0;
	  else if (*voronoi_type_ptr == 11) /* critical Voronoi point */
	    *global_voronoi_ptr = 1.0; /* grey */
	  else if (*voronoi_type_ptr == 12) /* int. critical Voronoi point */
	    *global_voronoi_ptr = 1.0; /* grey */
	  else if (*voronoi_region_name_ptr != -1){
	    *global_voronoi_ptr = 2.0;
	    *global_voronoi_active_ptr = 
	      - X_COLORS[regions[*voronoi_region_name_ptr].color % 26];
	  }
	  else if (*voronoi_type_ptr == 0)
	    *global_voronoi_ptr = 0.0;
	  else
	    *global_voronoi_ptr = 1.0;
	}

	else if (plot_modus == 5 ||
		 plot_modus == 6){ /* pruned regions */
	  if (*global_map_ptr < 0.75)
	    *global_voronoi_ptr = 0.0;
	  else if (*voronoi_type_ptr == 12) /* inter. critical Voronoi point */
	    *global_voronoi_ptr = 1.0; /* grey */
	  else if (*voronoi_region_name_ptr != -1){
	    *global_voronoi_ptr = 2.0;
	    *global_voronoi_active_ptr = 
	      - X_COLORS[regions[*voronoi_region_name_ptr].color % 26];
	  }
	  else if (*voronoi_type_ptr == 0)
	    *global_voronoi_ptr = 0.0;
	  else
	    *global_voronoi_ptr = 1.0;
	}

	else if (plot_modus == 4 || plot_modus == 7){
	  if (*voronoi_graph_ptr == (unsigned char) 1)
	    *global_voronoi_ptr = 0.0;
	  else
	    *global_voronoi_ptr = 1.0;
	}

	else{
	  static int this_printed = 0;
	  if (!this_printed){
	    fprintf(stderr, "Sorry - not implemented.\n");
	    this_printed = 1;
	  }
	  *global_voronoi_ptr = 0.0;
	}
      }
      global_active_ptr++;
      global_voronoi_ptr++;
      voronoi_type_ptr++;
      voronoi_clearance_ptr++;
      global_map_ptr++;
      global_voronoi_active_ptr++;
      voronoi_region_name_ptr++;
      voronoi_graph_ptr++;
    }

  if (plot_modus == 0)
    G_deactivate(TOPOLOGICAL_GRAPH);
  else
    G_activate(TOPOLOGICAL_GRAPH);


  /*
   * free memory
   */
  free(voronoi_type);
  voronoi_type = NULL;

  free(voronoi_graph);
  voronoi_graph = NULL;

  free(voronoi_points);
  voronoi_points = NULL;

  free(voronoi_region_name);
  voronoi_region_name = NULL;

  free(voronoi_clearance);
  voronoi_clearance = NULL;

  free(voronoi_DP1);
  voronoi_DP1 = NULL;

  free(voronoi_DP2);
  voronoi_DP2 = NULL;

}


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
 *   RETURN-VALUE: none
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
 *   RETURN-VALUE: none
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
 *   RETURN-VALUE: none
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
    if (tan_alpha_half < 0.00001 * distance)
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
    if (tan_alpha_half < 0.00001 * distance)
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
 *   RETURN-VALUE: none
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
 *   RETURN-VALUE: none
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




  exit(1);
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




/************************************************************************
 *
 *   NAME:         main 
 *                 
 *   FUNCTION:     main loop - checks for tcx events
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


main(int argc, char **argv)
{
  struct timeval read_begin_time;
  struct timeval read_end_time;
  long int sleep_duration;

  /* add some parameter files */
  bParamList = bParametersAddFile(bParamList, "etc/beeSoft.ini");
  
  /* add some enviroment variables */
  bParamList = bParametersAddEnv(bParamList, "", "TCXHOST");
  
  /* add command line arguements */
  bParamList = bParametersAddArray(bParamList, "", argc, argv);
  
  /* Fill the global parameter structure */
  bParametersFillParams(bParamList);
  
  check_commandline_parameters(argc, argv, program_state);
  init_program(program_state, robot_state, 
	       robot_specifications, &all);
  init_tcx(program_state);
  read_init_file(robot_state, program_state, robot_specifications);
  allocate_everything(robot_state, program_state, robot_specifications);
  init_graphics(robot_state, program_state, robot_specifications);
  initiate_position_search(robot_specifications, program_state, robot_state);



  if (read_map_dat){
    G_display_switch(LOAD_BUTTON, 1);
    load_parameters(init_file_name, 0);
    G_display_switch(LOAD_BUTTON, 0);/*!*/
  }
  do{

    if (!something_happened &&
	!program_state->map_update_pending){
#ifdef MAP_debug
      gettimeofday(&read_begin_time, NULL);
#endif
      block_waiting_time.tv_sec  = 0;
      block_waiting_time.tv_usec = 100000;
      /*fprintf(stderr, "(");*/
      (void) block_wait(&block_waiting_time, program_state->tcx_initialized,
			program_state->graphics_initialized);   
      /*fprintf(stderr, ")");*/
#ifdef MAP_debug
      gettimeofday(&read_end_time, NULL);
      sleep_duration = read_end_time.tv_usec - read_begin_time.tv_usec;
      sleep_duration += 1000000 
	* (read_end_time.tv_sec - read_begin_time.tv_sec);
      fprintf(stderr, "MAP:main:skipping = %f sec\n",
	      (float)sleep_duration/1000000.0);
#endif
    }
    

    /*
       save_gif(MAP_GIF_NAME, 0, 0);
       fprintf(stderr, ".");
       */

    if (!program_state->graphics_initialized &&
	program_state->tcx_initialized &&
	!something_happened &&
	!program_state->map_update_pending)
    {
      fd_set readMask;
      readMask = (Global->tcxConnectionListGlobal);
#ifdef MAP_debug
      gettimeofday(&read_begin_time, NULL);
#endif
      block_waiting_time.tv_sec  = 1;
      block_waiting_time.tv_usec = 0;
      /*fprintf(stderr, "[");*/
      select(FD_SETSIZE, &readMask, NULL, NULL, &block_waiting_time);
      /*fprintf(stderr, "]");*/
#ifdef MAP_debug
      gettimeofday(&read_end_time, NULL);
      sleep_duration = read_end_time.tv_usec - read_begin_time.tv_usec;
      sleep_duration += 1000000 
	* (read_end_time.tv_sec - read_begin_time.tv_sec);
      fprintf(stderr, "MAP:main:skipping = %f sec\n",
	      (float)sleep_duration/1000000.0);
#endif
    }

    
    something_happened = 0;
    
    if (program_state->tcx_initialized){
      TCX_no_waiting_time.tv_sec = 0;
      TCX_no_waiting_time.tv_usec = 0;
      /*fprintf(stderr, "<");*/
      tcxRecvLoop((void *) &TCX_no_waiting_time);
      /*fprintf(stderr, ">");*/
    }
    
    
    if (program_state->graphics_initialized)
      if (mouse_test_loop(robot_state, program_state, robot_specifications))
	something_happened = 1;
	
    if (!program_state->tcx_base_connected)
      connect_to_BASE(program_state); /* tries - every 5 sec - to connect
				       * to base */
    
    if (!program_state->tcx_localize_connected)
      connect_to_LOCALIZE(program_state); /* tries - every 5 sec - to connect
					   * to localize */
    
    do_search_step(robot_specifications, program_state, robot_state);

  } while (program_state->quit == 0);
  
  if (log_iop) fclose(log_iop);  
}


