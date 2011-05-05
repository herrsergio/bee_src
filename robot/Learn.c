
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






/*

note: there is an inconsistency in output_bp and output_xi,
which should be fixed!

*/


#define MAX_RANDOM 1073741823.5
#define RAND() ((((float) random())/MAX_RANDOM)-1.0)
#define RAND_POS() ((((float) random())/MAX_RANDOM)*0.5)



/************************************************************************
 *
 *   FILENAME:         Learn.c
 *                 
 *   FUNCTION:         general backpropagation tool, interfacable to LISP
 *                 
 *   HOW TO COMPILE:   cc -c Learn.c -O4
 *
 *   CREATOR:          Sebastian Thrun
 *   
 *   DATE:             May 1st, 1992 - Nov 16th, 1993
 *   
 *   GENERAL COMMENTS: Have Fun.
 *
 ************************************************************************/

#ifdef VMS
#include "vms.h"
#include "vmstime.h"
#endif

#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include "Application.h"
#include "Net.h"

#define LOG_FACTOR 10.0



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

static int print_flag;
static int n_printout_E;
static int n_printout_search_result;
static int n_printout_patterns;
static int n_printout_params;
static int random_generator = default_random_generator;
    
/* above parameters are moved from Learn.c (originally nn2.c) */






/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/*-------------------------------------------*/

static int train_with_bp;
static int train_with_xi;
static float table_logistic[32000];
static int fast_logistic_initialized = 0;
static float table_sin[32000];
static float table_cos[32000];
static int sin_cos_initialized = 0;

void init_sin_cos_table(){
  int i;
  float x;

  if (!sin_cos_initialized){
    for (i = 0; i < 32000; i++){
      x = ((float) i) * 0.001;
      table_sin[i] = sin(LOG_FACTOR * log(1.0+x));
      table_cos[i] = cos(LOG_FACTOR * log(1.0+x));
    }
    sin_cos_initialized = 1;
  }
}



/************************************************************************
 *
 *   NAME:         create_network
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:           --- Topology of the network ---
 *                 ninputs  
 *                 nhidden1          (might be 0)
 *                 nhidden2          (might be 0)
 *                 noutputs
 *                 max_npatterns_train    (just for *training*, otherwise 0)
 *                 max_npatterns_test     (just for *testing*, otherwise 0)
 *                 target_xi_defined    1, if you intend to use slope targets
 *
 *   NOTE:         If you plan to *search* in input space, the input and
 *                 output layer must both be partitioned such that non-
 *                 search units come first:
 *                 a) input: first come the units which are not changeable
 *                    by the search, then those which are to be changed
 *                 b) output: first come these units whoe values is irrelevent
 *                    for the search, then those whose value is to be 
 *                    maximized.
 *                 
 *   RETURN-VALUE: pointer to network, if succeeded, NULL otherwise
 *                 
 ************************************************************************/



net_ptr create_network(int l_ninputs, 
		       int l_nhidden1, 
		       int l_nhidden2, 
		       int l_noutputs, 
		       int l_max_npatterns_train, 
		       int l_max_npatterns_test,
		       int target_xi_defined)
{
  int i,j;
  int from, num, from2, num2;
  int malloc_size[2];
  net_ptr NPTR;
  int l_max_npatterns[N_PATTERN_SETS];
  static welcome = 0;


  if (!welcome){
    printf("********  EBNN Backprop-Tangentprop     ********\n");
    printf("***** by S.Thrun, R.Masuoka and T.Mitchell *****\n");
#ifdef LISP
    printf("***************   LISP version *****************\n\n");
#else
    printf("*****************   C version  *****************\n\n");
#endif
    welcome = 1;
  }

  if (target_xi_defined != 0 && target_xi_defined != 1){
    printf("ABORT: You might want to check your create_network() params!!\n");
    exit(99);			/* documents that those changed recently */
  }


  /* PART 1: Initialize topology of the network */
  if (l_ninputs > max_ninputs || l_noutputs > max_noutputs ||
      l_ninputs + l_nhidden1 + l_nhidden2 + l_noutputs > max_nunits){
    printf("ABORT: Network too big - change default settings in c-file\n");
    fflush(stdout);
    exit(1);
    return NULL;		/* this makes the compiler happy */
  }

  else{
    malloc_size[0] = sizeof(net_type);
    NPTR = (net_ptr) (malloc(malloc_size[0]));
    if (NPTR == NULL){
      printf("ABORT: out of memory in nn2!\n");
      exit(1);
    } else {
      printf("### New net: %d bytes.\n", malloc_size[0]);
    }

    l_max_npatterns[TRAINING_SET] = l_max_npatterns_train;
    l_max_npatterns[TESTING_SET]  = l_max_npatterns_test;

    for (i = 0; i < N_PATTERN_SETS; i++){
      if (l_max_npatterns[i] > 0){
	malloc_size[i] = sizeof(nn_pattern_type) * l_max_npatterns[i];
	NPTR->patterns[i] = (nn_pattern_ptr) (malloc(malloc_size[i]));
	NPTR->patterns_xi[i] = NULL;
	NPTR->patterns_xi1[i] = NULL;
	if (target_xi_defined){
	  if (l_noutputs == 1){
	    NPTR->patterns_xi1[i] = (nn_pattern_xi1_ptr) 
	    (malloc(sizeof(nn_pattern_xi1_type) * l_max_npatterns[i]));
	    malloc_size[i] += sizeof(nn_pattern_xi1_type) 
	      * l_max_npatterns[i];
	  }
	  else{
	    NPTR->patterns_xi[i] = (nn_pattern_xi_ptr) 
	      (malloc(sizeof(nn_pattern_xi_type) * l_max_npatterns[i]));
	    malloc_size[i] += sizeof(nn_pattern_xi_type)
	      * l_max_npatterns[i];
	  }
	}
	if (NPTR->patterns[i] == NULL || 
	    (target_xi_defined && 
	     NPTR->patterns_xi[i] == NULL 
	     && NPTR->patterns_xi1[i] == NULL)){
	  printf("ABORT: out of memory in nn2!\n");
	  exit(1);
	}
	if (!target_xi_defined)
	  printf("### New pattern buffer: %d bytes, no slope targets.\n", 
		 malloc_size[i]);
	else if (l_noutputs == 1)
	  printf("### New pattern buffer: %d bytes, incl. 1-slope targets.\n",
		 malloc_size[i]);
	else
	  printf("### New pattern buffer: %d bytes, incl. slope targets.\n",
		 malloc_size[i]);
      }
      else
	NPTR->patterns[i] = NULL;
    }
    
    NPTR->target_xi_defined = target_xi_defined; /* never change this value! */
    NPTR->ninputs = l_ninputs;
    NPTR->nhidden1 = l_nhidden1;
    NPTR->nhidden2 = l_nhidden2;
    NPTR->noutputs = l_noutputs;
    i = 0;
    from = -1;
    num = -1;
    from2 = -1;
    num2 = -1;
    
    for (j = 0; j < NPTR->ninputs; j++){
      NPTR->first_unit_to[i] = from;
      NPTR->num_units_to[i] = num;
      from2 = 0;
      num2 = NPTR->ninputs;
      i++;
    }
    from = from2;
    num = num2;
    
    NPTR->first_hidden1 = i;
    for (j = 0; j < NPTR->nhidden1; j++){
      NPTR->first_unit_to[i] = from;
      NPTR->num_units_to[i] = num;
      from2 = NPTR->first_hidden1;
      num2 = NPTR->nhidden1;
      i++;
    }
    from = from2;
    num = num2;
    
    NPTR->first_hidden2 = i;
    for (j = 0; j < NPTR->nhidden2; j++){
      NPTR->first_unit_to[i] = from;
      NPTR->num_units_to[i] = num;
      from2 = NPTR->first_hidden2;
      num2 = NPTR->nhidden2;
      i++;
    }
    from = from2;
    num = num2;
    
    NPTR->first_output = i;
    for (j = 0; j < NPTR->noutputs; j++){
      NPTR->first_unit_to[i] = from;
      NPTR->num_units_to[i] =  num;
      i++;
    }
    
    NPTR->nunits = i;
    NPTR->first_search_input = 0;
    NPTR->first_search_output = 0;
    NPTR->css = 0.0;
    NPTR->gcor = 0.0;

    /* compile list of successors - assumes layered network! */
    for (i = 0; i < NPTR->nunits; i++){
      NPTR->first_unit_from[i] = -1;
      NPTR->num_units_from[i]  = -1;
      for (j = NPTR->nunits-1; j >= 0; j--)
	if (i >= NPTR->first_unit_to[j] && i < NPTR->first_unit_to[j]
	    + NPTR->num_units_to[j])
	  NPTR->first_unit_from[i] = j;
      for (j = 0; j < NPTR->nunits; j++)
	if (i >= NPTR->first_unit_to[j] && i < NPTR->first_unit_to[j] 
	    + NPTR->num_units_to[j])
	  NPTR->num_units_from[i] = j - NPTR->first_unit_from[i] + 1;
    }
	
   
    /* PART 1: Initialize weight, biases etc. */
    for (i = 0; i < max_ninputs; i++)
      NPTR->individual_lrate_input_bp[i] = 1.0;
    for (i = 0; i < max_noutputs; i++)
      NPTR->individual_lrate_target_bp[i] = 1.0;
    for (i = 0; i < NPTR->nunits; i++){
      NPTR->unit_type[i] = LOGISTIC;
      NPTR->bias[i] = 0.0;
      NPTR->bias_lower_bound[i] = -9999999999.9;
      NPTR->bias_upper_bound[i] =  9999999999.9;
      NPTR->x[i] = 0.0;
      NPTR->net[i] = 0.0;
      NPTR->dbias_bp[i] = 0.0;
      NPTR->dbias_xi[i] = 0.0;
      NPTR->ddbias[i] = 0.0;
      NPTR->delta[i] = 0.0;
      NPTR->prev_dbias[i] = 0.0;
      for (j = 0; j < NPTR->nunits; j++){
	NPTR->w[i][j] = 0.0;
	NPTR->prev_dw[i][j] = 0.0;
	NPTR->w_lower_bound[i][j] = -9999999999.9;
	NPTR->w_upper_bound[i][j] =  9999999999.9;
	NPTR->xi[i][j] = 0.0;
	NPTR->epsil[i][j] = 0.0;
	NPTR->output_xi[i][j] = 0.0;
	NPTR->dw_bp[i][j] = 0.0;
	NPTR->dw_xi[i][j] = 0.0;
	NPTR->ddw[i][j] = 0.0;
	/*	if (NPTR->target_xi_defined)
		for (k = 0; k < NPTR->nunits; k++)
		NPTR->alpha[i][j][k] = 0.0; */
      }
    }
    
    
    for (i = 0; i < NPTR->nunits; i++){
      NPTR->bias1[i] = 0.0;  /* backup storage */
      NPTR->bias2[i] = 0.0;  /* backup storage */
      for (j = 0; j < NPTR->nunits; j++){
	NPTR->w1[i][j] = 0.0;
	NPTR->w2[i][j] = 0.0;
      }
    }
    
        
    for (i = 0; i < NPTR->ninputs; i++){
      NPTR->lower_bound[i] = 0.0;
      NPTR->upper_bound[i] = 1.0;
    }
    
    /* PART 3: Set  parameters  */
    for (i = 0; i < N_PATTERN_SETS; i++){
      NPTR->nn_pattern_num[i] = 0;
      NPTR->npatterns[i] = 0;
      NPTR->actual_pattern[i] = 0;
    }
    NPTR->E_bp = 0.0;
    NPTR->E_xi = 0.0;



    
    /* PART 3: Set default parameters  */
    NPTR->lrate                       = default_lrate;
    NPTR->fact_bp_to_xi               = default_fact_bp_to_xi;
    NPTR->decay                       = default_decay;
    NPTR->momentum                    = default_momentum;
    NPTR->max_steps                   = default_max_steps;
    NPTR->max_E_bp                    = default_max_E_bp;
    NPTR->max_E_xi                    = default_max_E_xi;
    NPTR->batch_training              = default_batch_training;
    NPTR->variable_momentum           = default_variable_momentum;
    NPTR->max_npatterns[TRAINING_SET] = l_max_npatterns_train;
    NPTR->max_npatterns[TESTING_SET]  = l_max_npatterns_train;
    NPTR->search_grid_size            = default_search_grid_size;
    NPTR->search_max_nsteps           = default_search_max_nsteps;
    NPTR->search_min_change           = default_search_min_change; 
    NPTR->search_lrate                = default_search_lrate; 
    NPTR->augment_factor              = default_augment_factor;
    NPTR->weights_init_range          = default_weights_init_range;
    NPTR->mathematica_gridsize        = default_mathematica_gridsize;
    NPTR->active_pattern_set          = default_active_pattern_set;
    NPTR->do_cross_validation         = default_do_cross_validation;
    NPTR->cross_validation_frequency  = default_cross_validation_frequency;
    NPTR->cross_validation_training_error_weight =
      default_cross_validation_training_error_weight;

    init_weights_random(NPTR);
    
    /* PART 4: Init tables */
    init_fast_logistic();
    init_sin_cos_table();

    initialize_xi_epsilon(NPTR);

    /* PART 5: Return net pointer */
    return NPTR;
  }
}



/************************************************************************
 *
 *   NAME:         delete_network
 *                 
 *   FUNCTION:     removes network, wts, and training examples, freeing space
 *                 (it remains to be tested that this function really
 *                 works, especially in combination with LISP)
 *                 
 *   PARAMETERS:   net_ptr NPTR           network
 *                 
 *   RETURN-VALUE: 1 if success, 0 if fail
 *                 
 ************************************************************************/

int delete_network(net_ptr NPTR)
{
  int i;
  
  if (NPTR == NULL){
    printf("Warning: Network not defined, delete_network failed.\n");
    return 0;
  }
  
  
  else{
    for (i = 0; i < N_PATTERN_SETS; i++){
      if (NPTR->patterns[i] != NULL)
	free(NPTR->patterns[i]);
      if (NPTR->patterns_xi[i] != NULL)
	free(NPTR->patterns_xi[i]);
      if (NPTR->patterns_xi1[i] != NULL)
	free(NPTR->patterns_xi1[i]);
    }
    free(NPTR);		/* free network itself */
    NPTR = NULL;
    return 1;
  }
}




/************************************************************************
 *
 *   NAME:         init_weights_random
 *                 
 *   FUNCTION:     Initializes the weights of a network with small random
 *                 values. 
 *                 -------> THIS IS ALREADY DONE IN CREATE_NETWORK !!! <-----
 *                 
 *   PARAMETERS:   net_ptr NPTR           network
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void init_weights_random(net_ptr NPTR)
{
  int i,j;
  
  /* Reset 2nd derivatives and stuff */

  clear_ddw_ddbias(NPTR);
  clear_gcor_stuff(NPTR);

  /* Set the weights and the biases between [-0.01, 0.01] */

  for (i = NPTR->first_hidden1; i < NPTR->nunits; i++){
    if (NPTR->unit_type[i] != GAUSSIAN){
      NPTR->bias[i] = (uniform_distribution() * 2.0 - 1.0) 
	* NPTR->weights_init_range; 
      for (j = NPTR->first_unit_to[i]; j < NPTR->first_unit_to[i]+
	   NPTR->num_units_to[i]; j++)
	NPTR->w[i][j] = (uniform_distribution() * 2.0 - 1.0) 
	  * NPTR->weights_init_range;
    }
    else{				/* GAUSSIAN only! */
      NPTR->bias[i] = uniform_distribution()* NPTR->weights_init_range;
      for (j = NPTR->first_unit_to[i]; j < NPTR->first_unit_to[i]+
	   NPTR->num_units_to[i]; j++)
	NPTR->w[i][j] = uniform_distribution();
      /* I took out the factor  'NPTR->weights_init_range' from the
	 center of the Gaussians. This will change all results 
	 based on Gaussians!!! (st 93-7-20) */ 
    }
  }
}



/************************************************************************
 *
 *   NAME:         modify_unit_type
 *                 
 *   FUNCTION:     the default unit type is LOGISTIC
 *                 this function changes the unit type of a group of units
 *                 
 *   NOTE:         Activations etc. are not updated
 *                  
 *   PARAMETERS:   net_ptr NPTR           network
 *                 int from, num          first unit to be changed, number..
 *                 int new_type           new unit_type
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void modify_unit_type(net_ptr NPTR, int from, int num, int new_type)
{
  int i;

  if (new_type < 0 || new_type > LAST_KNOWN_TYPE)
    printf("ERROR: Cannot set unittype to %d.\n", new_type);
  
  else{

    if (from < 0 || from + num > NPTR->nunits)
      printf("WARNING: Invalid unit-type range truncated: from=%d num=%d.\n",
	     from, num);

    for (i = from; i < from + num; i++)	/* change unit type */
      if (i >= 0 && i < NPTR->nunits)
	NPTR->unit_type[i] = new_type;
  }
}


/************************************************************************
 *
 *   NAME:         push_network
 *                 
 *   NOTE:         This function is defined if and only if the
 *                 flag "push_pop" is defined (see beginning of this code)
 *                 
 *   FUNCTION:     saves weights and biases of a network into a
 *                 backup memory, can be retrieved by pop_network
 *                 
 *   PARAMETERS:   net_ptr NPTR           network
 *                 int n                  backup-memory, valid values are
 *                                        1 or 2
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void push_network(net_ptr NPTR, int n)
{
  int i,j;
  
  if (n == 1){
    for (i = NPTR->first_hidden1; i < NPTR->nunits; i++){
      NPTR->bias1[i] = NPTR->bias[i];
      for (j = NPTR->first_unit_to[i]; j < NPTR->first_unit_to[i]+
	   NPTR->num_units_to[i]; j++)
	NPTR->w1[i][j] =NPTR->w[i][j];
    }
  }
  else if (n == 2){
    for (i = NPTR->first_hidden1; i < NPTR->nunits; i++){
      NPTR->bias2[i] = NPTR->bias[i];
      for (j = NPTR->first_unit_to[i]; j < NPTR->first_unit_to[i]+
	   NPTR->num_units_to[i]; j++)
	NPTR->w2[i][j] =NPTR->w[i][j];
    }
  }
  else
    printf("\nWarning: push-network %d not defined.\n",n);
}  




/************************************************************************
 *
 *   NAME:         pop_network
 *                 
 *   NOTE:         This function is defined if and only if the
 *                 flag "push_pop" is defined (see beginning of this code)
 *                 
 *   FUNCTION:     restores weights and biases from a backup by push_network.
 *                 
 *   PARAMETERS:   net_ptr NPTR           network
 *                 int n                  backup-memory, valid values are
 *                                        1 or 2
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void pop_network(net_ptr NPTR, int n)
{
  int i,j;

  if (n == 1){
    for (i = NPTR->first_hidden1; i < NPTR->nunits; i++){
      NPTR->bias[i] = NPTR->bias1[i];
      for (j = NPTR->first_unit_to[i]; j < NPTR->first_unit_to[i]+
	   NPTR->num_units_to[i]; j++)
	NPTR->w[i][j] =NPTR->w1[i][j];
    }
  }
  else if (n == 2){
    for (i = NPTR->first_hidden1; i < NPTR->nunits; i++){
      NPTR->bias[i] = NPTR->bias2[i];
      for (j = NPTR->first_unit_to[i]; j < NPTR->first_unit_to[i]+
	   NPTR->num_units_to[i]; j++)
	NPTR->w[i][j] =NPTR->w2[i][j];
    }
  }
  else
    printf("\nWarning: pop-network %d not defined.\n",n);
}  




/************************************************************************
 *
 *   NAME:         read_weights
 *                 
 *   FUNCTION:     reads weights from a file into a network
 *                 
 *   PARAMETERS:   net_ptr NPTR           network
 *                 char *filename         filename, might include path
 *                 
 *   RETURN-VALUE: 1 if successful, 0 if not.
 *                 
 ************************************************************************/



int read_weights(net_ptr NPTR, char *filename)
{
  int i,j;
  FILE *iop;
  float d;
  int done = 0;
  float sum = 0.0;
  

  clear_ddw_ddbias(NPTR);
  clear_gcor_stuff(NPTR);

  if ((iop = fopen(filename, "r")) != 0){
    for (i = NPTR->first_hidden1; i < NPTR->nunits && done == 0; i++)
      for (j = NPTR->first_unit_to[i]; j < NPTR->first_unit_to[i] +
	   NPTR->num_units_to[i] && done == 0; j++){
	if (fscanf(iop, "%f", &d) == EOF)
	  done = 1;
	else{
	  NPTR->w[i][j] = d;
	  sum += fabs(d);
	}
      }
    for (i = 0; i < NPTR->nunits && done == 0; i++){
      if (fscanf(iop, "%f", &d) == EOF)
	done = 1;
      else{
	NPTR->bias[i] = d;
	sum += fabs(d);
      }
    }
    if (done == 1){
      printf("\nWarning: Error while reading %s.\n", filename);
      return 0;
    }
    else
      printf("last bias: %g (%g)\n",d,sum);
    fclose(iop);
    return 1;
  }
  else{
    printf("\nWarning: File %s not found.\n", filename);
    return 0;
  }
}





/************************************************************************
 *
 *   NAME:         save_weights
 *                 
 *   FUNCTION:     save weights of a network to a file
 *                 
 *   PARAMETERS:   net_ptr NPTR           network
 *                 char *filename         filename, might include path
 *                 
 *   RETURN-VALUE: none yet
 *                 
 ************************************************************************/


void save_weights(net_ptr NPTR, char *filename)
{
  int i,j;
  FILE *iop;
  
  iop = fopen(filename, "w");
  for (i = NPTR->first_hidden1; i < NPTR->nunits; i++){
    for (j = NPTR->first_unit_to[i]; j < NPTR->first_unit_to[i] + 
	 NPTR->num_units_to[i]; j++)
      fprintf(iop, "%g ",NPTR->w[i][j]);
    fprintf(iop, "\n");
  }

  for (i = 0; i < NPTR->nunits; i++)
    fprintf(iop, "%g\n",NPTR->bias[i]);
  /*  printf("Weights saved in %s.\n", filename);*/
  fclose(iop);
}




/************************************************************************
 *
 *   NAME:         read_patterns
 *                 
 *   FUNCTION:     reads training patterns from a file
 *                 Note that the patterns are stored in a ring buffer:
 *                 It will not be detected if previously read patterns
 *                 get overwritten.
 *                 
 *   PARAMETERS:   net_ptr NPTR           network
 *                 char *filename         filename, might include path
 *                 int append             1=append, 0=overwrite (clear)
 *                 int xi_defined         1=pattern file has xi-values, 0=not
 *                 int pattern_set        Valid choices are:
 *                                        TRAINING_SET and TESTING_SET
 *                 
 *   RETURN-VALUE: 0, if error, 1, if successful
 *                 
 ************************************************************************/


int read_patterns(net_ptr NPTR, 
		  char *filename, 
		  int append, 
		  int xi_defined,
		  int pattern_set)
{

  int i,j,k;
  int done;
  FILE *iop;
  float d;
  char pat_name[80];

  if (xi_defined && !NPTR->target_xi_defined){
    printf("ERROR in read_patterns(): not a slope version!\n");
    return 0;
  }

  if (pattern_set < 0 || pattern_set > N_PATTERN_SETS){
    printf("ERROR in read_patterns(): pattern set %d unknown.\n",
	   pattern_set);
    return 0;
  }

  if (append == 0){
    NPTR->nn_pattern_num[pattern_set] = 0;
    NPTR->npatterns[pattern_set] = 0;
  }
  done = 0;
  j = 0;


  if (NPTR->patterns[pattern_set] != NULL){	/* pattern buffer defined */
    if ((iop = fopen(filename, "r")) != 0){
      do{
	if (NPTR->nn_pattern_num[pattern_set] >= 
	    NPTR->max_npatterns[pattern_set])
	  NPTR->nn_pattern_num[pattern_set] = 0;
	
	if (fscanf(iop, "%s", pat_name) == EOF)
	  done = 1;

	/* default: lrate=1.0, shoud maybe be changed here and in save_p.. */
	NPTR->patterns[pattern_set][NPTR->nn_pattern_num[pattern_set]
				    ].lrate_target_values = 1.0;
	NPTR->patterns[pattern_set][NPTR->nn_pattern_num[pattern_set]
				    ].lrate_target_derivatives = 1.0;

	
	for (i = 0; i < NPTR->ninputs && done == 0; i++){
	  if (fscanf(iop, "%f", &d) == EOF)
	    done = 1;
	  else
	    NPTR->patterns[pattern_set][NPTR->nn_pattern_num[pattern_set]
					].input[i] = d;
	}
	
	for (k = 0; k < NPTR->noutputs; k++){
	  if (fscanf(iop, "%f", &d) == EOF)
	    done = 1;
	  else
	    NPTR->patterns[pattern_set][NPTR->nn_pattern_num[pattern_set]
					].target_bp[k] = d;
	}      
	

	if (xi_defined == 1)
	  for (i = 0; i < NPTR->ninputs && done == 0; i++)
	    for (k = 0; k < NPTR->noutputs && done == 0; k++){
	      if (fscanf(iop, "%f", &d) == EOF)
		done = 1;
	      else{
		if (NPTR->noutputs == 1)
		  NPTR->patterns_xi1[pattern_set]
		    [NPTR->nn_pattern_num[pattern_set]].target_xi[i] = d;
		else
		  NPTR->patterns_xi[pattern_set]
		    [NPTR->nn_pattern_num[pattern_set]].target_xi[k][i] = d;
	      }
	    }

	if (done == 0){
	  NPTR->nn_pattern_num[pattern_set]++; 
	  j++;
	  if (NPTR->nn_pattern_num[pattern_set] > NPTR->npatterns[pattern_set])
	    NPTR->npatterns[pattern_set] = NPTR->nn_pattern_num[pattern_set];
	}
      }
      while (done == 0);
      if (NPTR->npatterns[pattern_set] == 1)
	printf("%d pattern read in %s (%d pattern is now memorized).\n",j,
	       filename, NPTR->npatterns[pattern_set]);
      else
	printf("%d patterns read in %s (%d patterns are now memorized).\n",j,
	       filename, NPTR->npatterns[pattern_set]);
      fclose(iop);
      return 1;
    }
    else{
      printf("\nWarning: File %s not found.\n", filename);
      return 0;
    }
  }
  else{
    printf("Warning: no pattern set allocated to this net.\n");
    return 0;
  }
}



/************************************************************************
 *
 *   NAME:         save_patterns
 *                 
 *   FUNCTION:     saves training patterns into a file
 *                 
 *   PARAMETERS:   net_ptr NPTR           network
 *                 char *filename         filename, might include path
 *                 int xi_defined         1=pattern file has xi-values, 0=not
 *                 int pattern_set        Valid choices are:
 *                                        TRAINING_SET and TESTING_SET
 *                 
 *   RETURN-VALUE: none yet
 *                 
 ************************************************************************/


void save_patterns(net_ptr NPTR, 
		   char *filename, 
		   int xi_defined,
		   int pattern_set)
{
  int i,k,n;
  FILE *iop;

  if (xi_defined && !NPTR->target_xi_defined){
    printf("ERROR in save_patterns(): not a slope version!\n");
    return;
  }

  if (pattern_set < 0 || pattern_set > N_PATTERN_SETS){
    printf("ERROR in save_patterns(): pattern set %d unknown.\n",
	   pattern_set);
    return;
  }

  if (NPTR->patterns[pattern_set] != NULL){
    iop = fopen(filename, "w");
    for (n = 0; n < NPTR->npatterns[pattern_set]; n++){
      
      
      fprintf(iop, "#%d", n);
      for (i = 0; i < NPTR->ninputs; i++)
	fprintf(iop, " %g",NPTR->patterns[pattern_set][n].input[i]);
      fprintf(iop, " ");
      
      for (k = 0; k < NPTR->noutputs; k++)
	fprintf(iop, " %g",NPTR->patterns[pattern_set][n].target_bp[k]);
      fprintf(iop, " ");
      
      if (xi_defined == 1)
	for (i = 0; i < NPTR->ninputs; i++)
	  for (k = 0; k < NPTR->noutputs; k++){
	    if (NPTR->noutputs == 1)
	      fprintf(iop, " %g",NPTR->patterns_xi1[pattern_set]
		      [n].target_xi[i]);
	    else
	      fprintf(iop, " %g",NPTR->patterns_xi[pattern_set]
		      [n].target_xi[k][i]);
	  }
      fprintf(iop, "\n");
    }
    fclose(iop);
  }
  else
    printf("Warning: no pattern set allocated to this net.\n");
}



/*********************************************************************\
|*****************                         ***************************|
|*****************   INTERNAL PROCEDURES   ***************************|
|*****************                         ***************************|
\*********************************************************************/




float  logistic(float x)
{
    double  exp ();
      if (x > 16.0)
	 return(.99999988);
      else
	 if (x < -16.0)
	     return(.00000012);
    else
	return(1.0 / (1.0 + (float) exp( (double) ((-1.0) * x))));
}




void init_fast_logistic()
{
  int i;
  float x;
  
  if (!fast_logistic_initialized){
    for (i = 0; i < 32000; i++){
      x = ((float) (i-16000)) * 0.001;
      table_logistic[i] = logistic(x);
    }
    fast_logistic_initialized = 1;
  }
}



float fast_logistic(float x)
{
  int index;

  if (x >= 16.0)
    return(.99999988);
  else
    if (x < -16.0)
      return(.00000012);
    else{
      index = (int) ((x + 16.0) * 1000.0);
      return table_logistic[index];
    }
}



float gaussian(float x)
{
  return(exp(0.0 - x));
}




  


  
float cos_fade(float x, int derivative)
{
  float  y, interpol_factor, local_sin, local_cos;
  int index;  

  y = fabs(x);

  index = (int) (y * 1000.0);

  if (index > 10 && index < 32000-1){		/* look-up table version */
    interpol_factor = (y * 1000.0) - ((float) index);
    local_sin = ((1.0 - interpol_factor) * table_sin[index])
      + (interpol_factor * table_sin[index+1]);
    local_cos = ((1.0 - interpol_factor) * table_cos[index])
      + (interpol_factor * table_cos[index+1]);	/* =linear interpolation */


    if (derivative == 0)
      return (0.5 * local_cos + 0.5);
    else if (derivative == 1){
      if (x >= 0.0)
	return (-0.5 * LOG_FACTOR * local_sin / (1.0+y));
      else
	return (0.5 * LOG_FACTOR * local_sin / (1.0+y));
    }
    else
      return (0.5 * LOG_FACTOR * (local_sin - (LOG_FACTOR * local_cos))
	      / (1.0+y) / (1.0+y));
  }
  else{				/* standard version */
    if (derivative == 0)
      return (0.5 * cos(LOG_FACTOR*log(1.0+y)) + 0.5);
    else if (derivative == 1){
      if (x >= 0.0)
	return (-0.5 * LOG_FACTOR * sin(LOG_FACTOR*log(1.0+y))
		/ (1.0+y));
      else
	return (0.5 * LOG_FACTOR * sin(LOG_FACTOR*log(1.0+y))
		/ (1.0+y));
    }
    else
      return (0.5 * LOG_FACTOR * 
	      (sin(LOG_FACTOR*log(1.0+y)) 
	       - (LOG_FACTOR * cos(LOG_FACTOR*log(1.0+y))))
	      / (1.0+y) / (1.0+y));
  }
}










float slow_logistic_integral(float x, int derivative)
{
  if (derivative == 0)
    return (x+log(1+exp(-x)));
  else if (derivative == 1)
    return (1/(1+exp(-x)));
  else
    return 1/(exp(x) * pow(1+exp(-x), 2));
}





/* ===================================================================== */

/* Definition for TEST_SQUASHING */
#define ALPHA_S 1.0
#define BETA_S -0.5


float sigma(net_ptr NPTR, int i)
{
  if (NPTR->unit_type[i] == LOGISTIC)
    return fast_logistic(NPTR->net[i]);

  else if (NPTR->unit_type[i] == COSINE)
    return ((0.5 * cos(NPTR->net[i])) + 0.5);

  else if (NPTR->unit_type[i] == COS_FADE)
    return cos_fade(NPTR->net[i], 0); 

  else if (NPTR->unit_type[i] == LOGISTIC_INT)
    return slow_logistic_integral(NPTR->net[i], 0);

  else if (NPTR->unit_type[i] == GAUSSIAN)
    return gaussian(NPTR->net[i]);

  else if (NPTR->unit_type[i] == LINEAR)
    return (NPTR->net[i]);

  else{
    printf("WARNING: unit_type %d unknown in sigma().\n", NPTR->unit_type[i]);
    return 0.0;
  }
}



float sigma_prime(net_ptr NPTR, int i)
{
  if (NPTR->unit_type[i] == LOGISTIC)
    return (NPTR->x[i] * (1.0-NPTR->x[i]));

  else if (NPTR->unit_type[i] == COSINE)
    return (-0.5 * sin(NPTR->net[i]));

  else if (NPTR->unit_type[i] == COS_FADE)
    return cos_fade(NPTR->net[i], 1); 

  else if (NPTR->unit_type[i] == LOGISTIC_INT)
    return slow_logistic_integral(NPTR->net[i], 1);

  else if (NPTR->unit_type[i] == GAUSSIAN)
    return (0.0 - NPTR->x[i]);

  else if (NPTR->unit_type[i] == LINEAR)
    return (1.0);

  
  else{
    printf("WARNING: unit_type %d unknown in sigma_prime().\n", 
	   NPTR->unit_type[i]);
    return 0.0;
  }
}





float sigma_prime_prime(net_ptr NPTR, int i)
{
  if (NPTR->unit_type[i] == LOGISTIC)
    return ((2.0*NPTR->x[i]*NPTR->x[i]*NPTR->x[i]) - 
	    (3.0*NPTR->x[i]*NPTR->x[i]) + NPTR->x[i]);

  else if (NPTR->unit_type[i] == COSINE)
    return (-0.5 * cos(NPTR->net[i]));

  else if (NPTR->unit_type[i] == COS_FADE)
    return cos_fade(NPTR->net[i], 2); 

  else if (NPTR->unit_type[i] == LOGISTIC_INT)
    return slow_logistic_integral(NPTR->net[i], 2);

  else if (NPTR->unit_type[i] == LINEAR)
    return (0.0);

  else if (NPTR->unit_type[i] == GAUSSIAN)
    return NPTR->x[i];

  else{
    printf("WARNING: unit_type %d unknown in sigma_prime_prime().\n", 
	   NPTR->unit_type[i]);
    return 0.0;
  }
}



/*-------------------------------*/


void clamp_pattern_input(net_ptr NPTR)
{
  int i;
  int p_help;

  p_help = (print_flag == 1 && n_printout_patterns > 0
	    && NPTR->steps % n_printout_patterns == 0);
  if (p_help) printf("\n");
  for (i = 0; i < NPTR->ninputs; i++){
    NPTR->x[i] = NPTR->patterns[NPTR->active_pattern_set]
      [NPTR->actual_pattern[NPTR->active_pattern_set]].input[i];
    if (p_help) printf(" [%d]=%g", i, NPTR->x[i]);
  }
}



void clamp_input(net_ptr NPTR, float *input)
{
  int i;

  for (i = 0; i < NPTR->ninputs; i++){
    if (input[i] < 0.0 || input[i] > 1.0)
      printf("Warning: invalid input value %d in clamp_input(): %g\n",
	     i, input[i]);
    NPTR->x[i] = input[i];
  }
}




void compute_net_x(net_ptr NPTR)
{
  int i,j;
  register float help;
  
  for (i = NPTR->first_hidden1; i < NPTR->nunits; i++){
    if (NPTR->unit_type[i] != GAUSSIAN){
      help = NPTR->bias[i];
      for (j = NPTR->first_unit_to[i]; j < NPTR->first_unit_to[i]+
	   NPTR->num_units_to[i]; j++){
	help += NPTR->w[i][j] * NPTR->x[j];
      }
    }
    else{			/* special: only Gaussians! */
      help = 0.0;
      for (j = NPTR->first_unit_to[i]; j < NPTR->first_unit_to[i]+
	   NPTR->num_units_to[i]; j++){
	help += (NPTR->x[j] - NPTR->w[i][j]) * (NPTR->x[j] - NPTR->w[i][j]);
      }
      help *= fabs(NPTR->bias[i]);
    }
    NPTR->net[i] = help;
    NPTR->x[i] = sigma(NPTR, i);
  }
}

  
/* compute_xi(NPTR) has been moved to epsilon.c */


void compute_output(net_ptr NPTR)
{
  int i,j, p_help;

  p_help = (print_flag == 1 && n_printout_patterns > 0
	    && NPTR->steps % n_printout_patterns == 0);
  if (p_help) printf(" ->");
  for (i = NPTR->first_output; i < NPTR->first_output+NPTR->noutputs; i++){
    NPTR->output_bp[i] = NPTR->x[i];	
    if (p_help)
      printf(" [%d]=%f (%f)",i,NPTR->output_bp[i], 
	     NPTR->patterns[NPTR->active_pattern_set]
	     [NPTR->actual_pattern[NPTR->active_pattern_set]
			    ].target_bp[i-NPTR->first_output]);
    if (NPTR->target_xi_defined){	/* otherwise ignore! */
      for (j = 0; j < NPTR->ninputs; j++){
	NPTR->output_xi[i][j] = NPTR->xi[i][j];
	if (p_help){
	  if (NPTR->noutputs == 1)
	    printf(" [%d][%d]=%f (%f)",i,j,
		   NPTR->output_xi[i][j],
		   NPTR->patterns_xi1[NPTR->active_pattern_set]
		   [NPTR->actual_pattern[NPTR->active_pattern_set]
		    ].target_xi[j]);
	  else
	    printf(" [%d][%d]=%f (%f)",i,j,
		   NPTR->output_xi[i][j],
		   NPTR->patterns_xi[NPTR->active_pattern_set]
		   [NPTR->actual_pattern[NPTR->active_pattern_set]
		    ].target_xi[i-NPTR->first_output][j]);
	}
      }
    }
  }
}




void compute_delta_epsil(net_ptr NPTR)
{
  int i,k;
  

  if (train_with_bp == 1)
    for (k = NPTR->first_output; k < NPTR->first_output+NPTR->noutputs; k++){
      NPTR->delta[k] = 
	NPTR->patterns[NPTR->active_pattern_set]
	  [NPTR->actual_pattern[NPTR->active_pattern_set]
	   ].target_bp[k-NPTR->first_output]
	- NPTR->output_bp[k];
      NPTR->delta[k] *=
	NPTR->patterns[NPTR->active_pattern_set]
	  [NPTR->actual_pattern[NPTR->active_pattern_set]
	   ].lrate_target_values *
	  NPTR->individual_lrate_target_bp[k-NPTR->first_output];
      NPTR->E_bp += NPTR->delta[k] * NPTR->delta[k];
    }
  else
    for (k = NPTR->first_output; k < NPTR->first_output+NPTR->noutputs; k++)
      NPTR->delta[k] = 0.0;
  

  if (train_with_xi == 1 && NPTR->target_xi_defined)
    for (k = NPTR->first_output; k < NPTR->first_output+NPTR->noutputs; k++)
      for (i = 0; i < NPTR->ninputs; i++){
	if (NPTR->noutputs == 1)
	  NPTR->epsil[k][i] = 
	    NPTR->patterns_xi1[NPTR->active_pattern_set]
	      [NPTR->actual_pattern[NPTR->active_pattern_set]].target_xi[i] 
	      - NPTR->output_xi[k][i];
	else
	  NPTR->epsil[k][i] = 
	    NPTR->patterns_xi[NPTR->active_pattern_set]
	      [NPTR->actual_pattern[NPTR->active_pattern_set]
	       ].target_xi[k-NPTR->first_output][i] 
	      - NPTR->output_xi[k][i];
	NPTR->epsil[k][i] *=
	  NPTR->patterns[NPTR->active_pattern_set]
	    [NPTR->actual_pattern[NPTR->active_pattern_set]
	     ].lrate_target_derivatives;
	NPTR->E_xi += NPTR->epsil[k][i] * NPTR->epsil[k][i];
      }
  else
    for (k = NPTR->first_output; k < NPTR->first_output+NPTR->noutputs; k++)
      for (i = 0; i < NPTR->ninputs; i++)
	NPTR->epsil[k][i] = 0.0;
}



void clear_errors(net_ptr NPTR)
{
  NPTR->E_bp = 0.0;
  NPTR->E_xi = 0.0;
}

void clear_dw_dbias(net_ptr NPTR)
{
  int j,l;

  for (j = NPTR->first_hidden1; j < NPTR->nunits; j++){
    NPTR->dbias_bp[j] = 0.0;
    NPTR->dbias_xi[j] = 0.0;
    for (l = NPTR->first_unit_to[j]; l < NPTR->first_unit_to[j] +
	 NPTR->num_units_to[j]; l++){
      NPTR->dw_bp[j][l] = 0.0;
      NPTR->dw_xi[j][l] = 0.0;
    }
  }
}

void clear_gcor_stuff(net_ptr NPTR)
{
  int j, l;

  NPTR->css  = 0.0;
  NPTR->gcor = 0.0;

  for (j = NPTR->first_hidden1; j < NPTR->nunits; j++){
    NPTR->prev_dbias[j] = 0.0;
    for (l = NPTR->first_unit_to[j]; l < NPTR->first_unit_to[j] +
	 NPTR->num_units_to[j]; l++)
      NPTR->prev_dw[j][l] = 0.0;
  } 
}

void clear_ddw_ddbias(net_ptr NPTR)
{
  int j, l;

  for (j = NPTR->first_hidden1; j < NPTR->nunits; j++){
    NPTR->ddbias[j] = 0.0;
    for (l = NPTR->first_unit_to[j]; l < NPTR->first_unit_to[j] +
	 NPTR->num_units_to[j]; l++)
      NPTR->ddw[j][l] = 0.0;
  } 
}

/* ************************************************************ */
/* ************************************************************ */

void change_w_bias(net_ptr NPTR)
{
  int k,l;
  float s;
  float final_lrate_bp_bias, final_lrate_bp_w;
  float final_lrate_xi_bias, final_lrate_xi_w;
  float individual_lrate_factor;
  float dp, den, prev_css;	/* names taken from Rumelhart's 1986 code,
				 * just compute gradient corrlation of
				 * two subsequent updates */

  final_lrate_bp_bias = NPTR->lrate_bp_bias * NPTR->lrate_fact_bp;
  final_lrate_xi_bias = NPTR->lrate_xi_bias * NPTR->lrate_fact_xi;
  final_lrate_bp_w = NPTR->lrate_bp_w * NPTR->lrate_fact_bp;
  final_lrate_xi_w = NPTR->lrate_xi_w * NPTR->lrate_fact_xi;
  prev_css = NPTR->css;
  NPTR->css = 0.0;
  dp = den = 0.0;

  for (k = NPTR->first_hidden1; k < NPTR->nunits; k++){
    s = (final_lrate_bp_bias * NPTR->dbias_bp[k])
      + (final_lrate_xi_bias * NPTR->dbias_xi[k])
      - (NPTR->decay * NPTR->bias[k]);
    NPTR->ddbias[k] = s + NPTR->momentum * NPTR->ddbias[k];
    NPTR->bias[k] += NPTR->ddbias[k];
    if (NPTR->bias[k] < NPTR->bias_lower_bound[k])
      NPTR->bias[k] = NPTR->bias_lower_bound[k];
    if (NPTR->bias[k] > NPTR->bias_upper_bound[k])
      NPTR->bias[k] = NPTR->bias_upper_bound[k];
    /* for g-correlation only */
    NPTR->css += (s * s);
    dp  += (s * NPTR->prev_dbias[k]);
    NPTR->prev_dbias[k] = s;

    for (l = NPTR->first_unit_to[k]; l < NPTR->first_unit_to[k]+
	 NPTR->num_units_to[k]; l++){
      if (l < NPTR->ninputs)	/* the inputs can be weighted according 
				 * to their importance! (-> NeuroChess) */
	individual_lrate_factor = NPTR->individual_lrate_input_bp[l];
      else			/* not an inpuit weight */
	individual_lrate_factor = 1.0;
      s = (final_lrate_bp_w * NPTR->dw_bp[k][l] * individual_lrate_factor)
	+ (final_lrate_xi_w * NPTR->dw_xi[k][l])
	  - (NPTR->decay * NPTR->w[k][l]);
      NPTR->ddw[k][l] = s + NPTR->momentum * NPTR->ddw[k][l];
      NPTR->w[k][l] += NPTR->ddw[k][l];   
      if (NPTR->w[k][l] < NPTR->w_lower_bound[k][l])
	NPTR->w[k][l] = NPTR->w_lower_bound[k][l];
      if (NPTR->w[k][l] > NPTR->w_upper_bound[k][l])
	NPTR->w[k][l] = NPTR->w_upper_bound[k][l];
      /* for g-correlation only */
      NPTR->css += (s * s);
      dp  += (s * NPTR->prev_dw[k][l]);
      NPTR->prev_dw[k][l] = s;
    }
  }

  /* for g-correlation only */
  den = prev_css * NPTR->css;	/* denominator (squared) */
  if (den > 0.0 && NPTR->batch_training){ /* only defined for batch training */
    den = (float) sqrt((double) den);
    NPTR->gcor = dp / den; /* dot product times denominator */
  }
  else
    NPTR->gcor = 0.0;	    /* den must be > 0, otherwise something's wrong */
  
  if (NPTR->batch_training && NPTR->variable_momentum){
    if (NPTR->gcor > 0.9 && NPTR->gcor <= 1.0)
      NPTR->momentum = NPTR->gcor * NPTR->gcor * NPTR->gcor;
    else
      NPTR->momentum = 0.0;	/* black magic! */
  }
}



/* Changed by Masuoka */
void printout_all(net_ptr NPTR)
{
  int i,j;
  int layer;
  int i_number_in_layer;
  int j_number_in_layer;

  float final_lrate_bp_bias, final_lrate_bp_w;
  float final_lrate_xi_bias, final_lrate_xi_w;

  final_lrate_bp_bias = NPTR->lrate_bp_bias * NPTR->lrate_fact_bp;
  final_lrate_xi_bias = NPTR->lrate_xi_bias * NPTR->lrate_fact_xi;
  final_lrate_bp_w = NPTR->lrate_bp_w * NPTR->lrate_fact_bp;
  final_lrate_xi_w = NPTR->lrate_xi_w * NPTR->lrate_fact_xi;


  printf("\n\nbp_w -> %g, xi_w -> %g, bp_b -> %g, xi_b -> %g\n", 
	 final_lrate_bp_w, final_lrate_xi_w, 
	 final_lrate_bp_bias, final_lrate_xi_bias);
  printf("decay -> %g, momentum -> %g\n", NPTR->decay, NPTR->momentum);
  printf("NPTR->num_units_from[0] = %d\n", NPTR->num_units_from[0]);


  printf("\n\n===WEIGHT and BIAS:\n{");
  for (i = NPTR->first_hidden1; i < NPTR->nunits; i++) {
    if (i >= NPTR->first_output)
      {layer = 3; 
       i_number_in_layer = i - NPTR->first_output + 1;
       j_number_in_layer = NPTR->first_hidden1 - 1;}
    else 
      {layer = 2;
       i_number_in_layer = i - NPTR->first_hidden1 + 1;
       j_number_in_layer = - 1;}
    for (j = NPTR->first_unit_to[i]; j < NPTR->first_unit_to[i] +
	 NPTR->num_units_to[i]; j++){

      printf("w%d%d%d -> %g (* %g + %g *), \n",layer, i_number_in_layer, 
	     j - j_number_in_layer,NPTR->w[i][j], 
	     NPTR->dw_bp[i][j], NPTR->dw_xi[i][j]); 
    }
  }
  for (i = NPTR->first_hidden1; i < NPTR->nunits; i++) {
    if (i >= NPTR->first_output)
      {layer = 3; 
       i_number_in_layer = i - NPTR->first_output + 1; }
    else if (i >= NPTR->first_hidden1)
      {layer = 2;
       i_number_in_layer = i - NPTR->first_hidden1 + 1;}
    else 
      {layer = 1;
       i_number_in_layer = i + 1;}

    printf("b%d%d -> %g (* %g + %g *)",layer, i_number_in_layer, 
	   NPTR->bias[i], NPTR->dbias_bp[i], NPTR->dbias_xi[i]);
    if (i != NPTR->nunits - 1) printf(", \n");
  }
  printf("}\n");
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


void initialize_xi_epsilon(net_ptr NPTR)
{
  int i,j,k;
  for (i = 0; i < NPTR->ninputs; i++){
    for (j = 0; j < NPTR->ninputs; j++){
      NPTR->xi[i][j] = 0.0;
    }
  }
  for (i = 0; i < NPTR->ninputs; i++){
    NPTR->xi[i][i] = 1.0;
  }
  for (k = NPTR->first_output; 
       k < NPTR->first_output+NPTR->noutputs; k++){
    NPTR->epsilon_value[k] = 0.0;
  }
}

void compute_xi(net_ptr NPTR)
{
  register float help_xi;
  register int i,j,l;

  for (l = NPTR->first_hidden1; 
       l < NPTR->first_hidden1 + NPTR->nhidden1; l++){
    if (NPTR->unit_type[l] != GAUSSIAN)
      for (i = 0; i < NPTR->ninputs; i++){
	NPTR->xi[l][i] = (NPTR->xi_tmp[l][i] = NPTR->w[l][i]) 
	  * (NPTR->sigma_single_prime[l] = sigma_prime(NPTR, l));
      }
    else			/* only Gaussians! note yet used! */
      for (i = 0; i < NPTR->ninputs; i++){
	NPTR->xi[l][i] = fabs(NPTR->bias[l])
	  * (NPTR->xi_tmp[l][i] = (2.0 * (NPTR->x[i] - NPTR->w[l][i]))) 
	    * (NPTR->sigma_single_prime[l] = sigma_prime(NPTR, l));
      }
  }
  for (l = NPTR->first_hidden1 + NPTR->nhidden1; 
       l < NPTR->nunits; l++){
    if (NPTR->unit_type[l] != GAUSSIAN){
      for (i = 0; i < NPTR->ninputs; i++){
	help_xi = 0.0;
	for (j = NPTR->first_unit_to[l]; j < NPTR->first_unit_to[l]+
	     NPTR->num_units_to[l]; j++){
	  help_xi += NPTR->w[l][j] * NPTR->xi[j][i];
	}
	NPTR->xi[l][i] = (NPTR->xi_tmp[l][i] = help_xi) 
	  * (NPTR->sigma_single_prime[l] = sigma_prime(NPTR, l));
      }
    }
    else{			/* only GAUSSIANS! note yet used! */
      for (i = 0; i < NPTR->ninputs; i++){
	NPTR->xi[l][i] = fabs(NPTR->bias[l])
	  * (NPTR->xi_tmp[l][i] = (2.0 * (NPTR->x[i] - NPTR->w[l][i]))) 
	    * (NPTR->sigma_single_prime[l] = sigma_prime(NPTR, l));
      }
    }
  }
}



void compute_dw_dbias_bp(net_ptr NPTR)
{
  register float help_delta;
  register int j,k,l,m;

  for (k = NPTR->first_output; 
       k < NPTR->first_output+NPTR->noutputs; k++){
    NPTR->delta_bp[k] = NPTR->delta[k] * sigma_prime(NPTR, k);
  }

  for (l = NPTR->first_output - 1 ; l >= NPTR->first_hidden1; l--){
    help_delta = 0.0;
    for(m = NPTR->first_unit_from[l]; 
	m < NPTR->first_unit_from[l] + NPTR->num_units_from[l]; m++){
      if (NPTR->unit_type[m] != GAUSSIAN)
	help_delta += NPTR->delta_bp[m] * NPTR->w[m][l];
      else
	help_delta += NPTR->delta_bp[m] * NPTR->bias[m] * NPTR->bias[m] *
	  2.0 * (NPTR->x[l] - NPTR->w[m][l]);
    }
    NPTR->delta_bp[l] = help_delta * sigma_prime(NPTR, l);
  }

  for (l = NPTR->first_hidden1; l < NPTR->nunits; l++){
    if (NPTR->unit_type[l] != GAUSSIAN){
      NPTR->dbias_bp[l] += NPTR->delta_bp[l];
      for (j = NPTR->first_unit_to[l]; j < NPTR->first_unit_to[l]
	   +NPTR->num_units_to[l]; j++){
	NPTR->dw_bp[l][j] += NPTR->delta_bp[l] * NPTR->x[j];
      }
    }
    else{			/* GAUSSIAN only! */
      if (NPTR->bias[l] != 0.0)	/* otherwise dE/dbias = 0 (and net=0) */
	NPTR->dbias_bp[l] += NPTR->delta_bp[l] * NPTR->net[l] /
	  NPTR->bias[l];
      for (j = NPTR->first_unit_to[l]; j < NPTR->first_unit_to[l]
	   +NPTR->num_units_to[l]; j++){
	NPTR->dw_bp[l][j] += 2.0 * NPTR->delta_bp[l] * fabs(NPTR->bias[l])
	  * (NPTR->w[l][j] - NPTR->x[j]);
      }
    }
  }
}
  
void compute_dw_dbias_xi(net_ptr NPTR)
{
  register float help_epsilon_bar;
  register float help_epsilon_bar_x_tilde;
  register float help_epsilon_value_tilde;
  register int i,j,k,l,m;

  /* *********************** */
  /* Here comes Tangent Prop */
  /* *********************** */
  /* *********************** */
  /* Now this is done in compute_xi().
     for (l = NPTR->first_hidden1; l < NPTR->nunits; l++){
     NPTR->sigma_single_prime[l] = sigma_prime(NPTR, l);
     }
     */
  /* *********************** */
    
  for (l = NPTR->first_hidden1; l < NPTR->nunits; l++){
    NPTR->sigma_double_prime[l] = sigma_prime_prime(NPTR, l);
    NPTR->dbias_xi_tmp[l] = 0.0;
  }
    
  for (i = 0; i < NPTR->ninputs; i++){
    for (k = NPTR->first_output; 
	 k < NPTR->first_output+NPTR->noutputs; k++){
      NPTR->epsilon_bar_x_tilde[k] = 
	(NPTR->epsilon_bar[k] = NPTR->epsil[k][i]) * 
	  NPTR->sigma_single_prime[k];
      NPTR->epsilon_value_tilde[k] = NPTR->epsilon_bar[k] *
	NPTR->sigma_double_prime[k] * NPTR->xi_tmp[k][i];
    }
    for (l = NPTR->first_output - 1 ; l >= NPTR->first_hidden1; l--){
      help_epsilon_bar = 0.0;
      help_epsilon_value_tilde = 0.0;
      for(m = NPTR->first_unit_from[l]; 
	  m < NPTR->first_unit_from[l] + NPTR->num_units_from[l]; m++){
	help_epsilon_bar += NPTR->epsilon_bar_x_tilde[m] * NPTR->w[m][l];
	help_epsilon_value_tilde += 
	  NPTR->epsilon_value_tilde[m] * NPTR->w[m][l];
      }
      NPTR->epsilon_bar_x_tilde[l] = 
	(NPTR->epsilon_bar[l] = help_epsilon_bar)
	  * NPTR->sigma_single_prime[l];
      NPTR->epsilon_value_tilde[l] = 
	NPTR->sigma_single_prime[l] * help_epsilon_value_tilde 
	  + NPTR->epsilon_bar[l] * NPTR->sigma_double_prime[l] * 
	    NPTR->xi_tmp[l][i];
    }
    /* weight and bias update */
    for (l = NPTR->first_hidden1; 
	 l < NPTR->first_hidden1 + NPTR->nhidden1; l++){
      NPTR->dbias_xi_tmp[l] += NPTR->epsilon_value_tilde[l];
      NPTR->dw_xi[l][i] += NPTR->epsilon_bar_x_tilde[l];
      }
    for (l = NPTR->first_hidden1 + NPTR->nhidden1; 
	 l < NPTR->nunits; l++){
      NPTR->dbias_xi_tmp[l] += NPTR->epsilon_value_tilde[l];
      help_epsilon_bar_x_tilde = NPTR->epsilon_bar_x_tilde[l];
      for (j = NPTR->first_unit_to[l]; j < NPTR->first_unit_to[l]
	   +NPTR->num_units_to[l]; j++){
	NPTR->dw_xi[l][j] += help_epsilon_bar_x_tilde * NPTR->xi[j][i];
      }
    }
  }
  for (l = NPTR->first_hidden1; l < NPTR->nunits; l++){
    NPTR->dbias_xi[l] += NPTR->dbias_xi_tmp[l];
    for (j = NPTR->first_unit_to[l]; j < NPTR->first_unit_to[l]
	 +NPTR->num_units_to[l]; j++){
      NPTR->dw_xi[l][j] += NPTR->dbias_xi_tmp[l] * NPTR->x[j];
    }
  }
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



/************************************************************************
 *
 *   NAME:         clear_pattern_set
 *                 
 *   FUNCTION:     As the name suggests: clears the training set
 *                 
 *   PARAMETERS:   net_ptr NPTR           network
 *                 int pattern_set        Valid choices are:
 *                                        TRAINING_SET and TESTING_SET
 *                                        and ALL_SETS (which clears all)
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void clear_pattern_set(net_ptr NPTR, int pattern_set)
{
  int i;

  if ((pattern_set < 0 || pattern_set > N_PATTERN_SETS)
      && pattern_set != ALL_SETS)
    printf("ERROR in clear_pattern_set(): pattern set %d unknown.\n",
	   pattern_set);
  else{
    if (pattern_set == ALL_SETS)
      for (i = 0; i < N_PATTERN_SETS; i++)
	NPTR->npatterns[i]              = 0;
    else{
      NPTR->npatterns[pattern_set]      = 0;
      NPTR->nn_pattern_num[pattern_set] = 0;
    }
  }
}




/************************************************************************
 *
 *   NAME:         add_new_pattern
 *                 
 *   FUNCTION:     adds a single pattern to the training set
 *                 
 *   PARAMETERS:   net_ptr NPTR           network
 *                 int mode               if 1, include target derivatives
 *                                        if 0, ignore
 *                 float *inputs          vector of input values
 *                 float *target_bp       vector of target values
 *                 float *target_xi       vector of target derivatives
 *                                        (is NULL if not defined)
 *                 float lrate_target_values       relative weight values
 *                 float lrate_target_derivatives  relative weight derivatives
 *                 int pattern_set        Valid choices are:
 *                                        TRAINING_SET and TESTING_SET
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void add_new_pattern(net_ptr NPTR, int mode, 
		     float *input, float *target_bp, float *target_xi,
		     float lrate_target_values, float lrate_target_derivatives,
		     int pattern_set)
{
  int i, k, index;


  /* COPYING TRAINING PATTERN */


  if (mode && !NPTR->target_xi_defined){
    printf("ERROR in add_new_pattern(): not a slope version!\n");
    fflush(stdout);
    return;
  }

  if (pattern_set < 0 || pattern_set >= N_PATTERN_SETS){
    printf("ERROR in add_new_pattern(): pattern set %d unknown.\n",
	   pattern_set);
    return;
  }

  if (NPTR->patterns[pattern_set] != NULL){
    for (i = 0; i < NPTR->ninputs; i++)
      NPTR->patterns[pattern_set][NPTR->nn_pattern_num[pattern_set]
				  ].input[i] = input[i];
    for (k = 0; k < NPTR->noutputs; k++)
      NPTR->patterns[pattern_set][NPTR->nn_pattern_num[pattern_set]
				  ].target_bp[k] = target_bp[k];
    if (mode == 1){
      index = 0;
      for (k = 0; k < NPTR->noutputs; k++)
	for (i = 0; i < NPTR->ninputs; i++){
	  if (NPTR->noutputs == 1)
	    NPTR->patterns_xi1[pattern_set][NPTR->nn_pattern_num[pattern_set]
					    ].target_xi[i] 
	      = target_xi[index++];
	  else
	    NPTR->patterns_xi[pattern_set][NPTR->nn_pattern_num[pattern_set]
					   ].target_xi[k][i] 
	      = target_xi[index++];
	}
    }
    NPTR->patterns[pattern_set][NPTR->nn_pattern_num[pattern_set]
				].lrate_target_values 
      = lrate_target_values;
    NPTR->patterns[pattern_set][NPTR->nn_pattern_num[pattern_set]
				].lrate_target_derivatives 
      = lrate_target_derivatives;

    /* RING BUFFER INCEREMENT */
    
    NPTR->nn_pattern_num[pattern_set]++;
    if (NPTR->nn_pattern_num[pattern_set] > NPTR->npatterns[pattern_set]) 
      NPTR->npatterns[pattern_set] = NPTR->nn_pattern_num[pattern_set];
    if (NPTR->nn_pattern_num[pattern_set] >= 
	NPTR->max_npatterns[pattern_set]) 
      NPTR->nn_pattern_num[pattern_set] = 0;
  }
  else
    printf("Warning: no pattern set allocated to this net.\n");
}     



/************************************************************************
 *
 *   NAME:         generate_random_testing_set
 *                 
 *   FUNCTION:     moves i% of the patterns in a set into the
 *                 other set (for cross validation purpose)
 *                 
 *   PARAMETERS:   net_ptr NPTR             network
 *                 int     int pattern_set  target set (destination)
 *                 float   ratio            relative number of patterns
 *                                          (between 0.0 and 1.0)
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void generate_random_testing_set(net_ptr NPTR, int pattern_set, float ratio)
{
  int number, i, s, p, q, help;
  int source, destination;

  if (ratio < 0.0 || ratio > 1.0){
    printf("ERROR: illegal ratio %g in generate_random_testing_set().\n",
	   ratio);
    return;
  }
  

  if (pattern_set < 0 || pattern_set > N_PATTERN_SETS){
    printf("ERROR in add_new_pattern(): pattern set %d unknown.\n",
	   pattern_set);
    return;
  }



  else{
    destination = pattern_set;
    source = 1 - pattern_set;
    number = (int) (((float) NPTR->npatterns[source]) * ratio);
    for (i = 0, p = 0; i < number; i++){
      s = (int) (RAND_POS() * ((float) NPTR->npatterns[source]));
      p = (p + s) % NPTR->npatterns[source]; /* determine one pattern @random*/
      /*********** move pattern to destination *************/
      printf("Move pattern %d (%f) from source to destination\n", 
	     p, NPTR->patterns[source][p].input[0]);
      if (NPTR->noutputs == 1)
	add_new_pattern(NPTR, NPTR->target_xi_defined, 
			NPTR->patterns[source][p].input,
			NPTR->patterns[source][p].target_bp,
			NPTR->patterns_xi1[source][p].target_xi, /* <== */
			NPTR->patterns[source][p].lrate_target_values,
			NPTR->patterns[source][p].lrate_target_derivatives,
			destination);
      else
	add_new_pattern(NPTR, NPTR->target_xi_defined, 
			NPTR->patterns[source][p].input,
			NPTR->patterns[source][p].target_bp,
			&(NPTR->patterns_xi[source][p].target_xi[0][0]),/*<<*/
			NPTR->patterns[source][p].lrate_target_values,
			NPTR->patterns[source][p].lrate_target_derivatives,
			destination);
      /*********** delete pattern *************/
      help = NPTR->npatterns[source];
      if (p < NPTR->npatterns[source] - 1){ /* was not the last pattern! */
	NPTR->nn_pattern_num[source] = p; /* move into p-th position */
	q = NPTR->npatterns[source] - 1; /* ...the last pattern */
	printf("Shift pattern %d (%f) to slot %d\n", q, 
	       NPTR->patterns[source][q].input[0], p);
	if (NPTR->noutputs == 1)
	  add_new_pattern(NPTR, NPTR->target_xi_defined, 
			  NPTR->patterns[source][q].input,
			  NPTR->patterns[source][q].target_bp,
			  NPTR->patterns_xi1[source][q].target_xi, /* <== */
			  NPTR->patterns[source][q].lrate_target_values,
			  NPTR->patterns[source][q].lrate_target_derivatives,
			  source);
	else
	  add_new_pattern(NPTR, NPTR->target_xi_defined, 
			  NPTR->patterns[source][q].input,
			  NPTR->patterns[source][q].target_bp,
			  &(NPTR->patterns_xi[source][q].target_xi[0][0]),/*<*/
			  NPTR->patterns[source][q].lrate_target_values,
			  NPTR->patterns[source][q].lrate_target_derivatives,
			  source);
      }
      NPTR->npatterns[source] = help - 1; /* and subtract last pattern */
    }
    NPTR->nn_pattern_num[source] = NPTR->npatterns[source];
  }
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




void one_step_training(net_ptr NPTR)
{
  if (train_with_xi && !NPTR->target_xi_defined){
    printf("ERROR in one_step_training(): not a slope version!\n");
    return;
  }

  NPTR->active_pattern_set = TRAINING_SET; /* training only using this set */
  clear_errors(NPTR);
  clear_dw_dbias(NPTR);
  for (NPTR->actual_pattern[TRAINING_SET] = 0; 
       NPTR->actual_pattern[TRAINING_SET] 
       < NPTR->npatterns[TRAINING_SET]; 
       NPTR->actual_pattern[TRAINING_SET]++){
    clamp_pattern_input(NPTR);
    compute_net_x(NPTR);

    if (train_with_xi == 1){
      compute_xi(NPTR);
    }
    compute_output(NPTR);
    compute_delta_epsil(NPTR); 

    if (train_with_bp == 1){
      compute_dw_dbias_bp(NPTR);
    }
    if (train_with_xi == 1){
      compute_dw_dbias_xi(NPTR);
    }

    if (NPTR->batch_training == 0){
      change_w_bias(NPTR);
      clear_dw_dbias(NPTR);
    }
  }
  if (print_flag == 1 &&  n_printout_params > 0 && 
      NPTR->steps % n_printout_params == 0) 
    printout_all(NPTR);
  if (print_flag == 1) fflush(stdout);
  NPTR->E_bp *= NPTR->E_fact_bp;
  NPTR->E_xi *= NPTR->E_fact_xi;
  if (NPTR->batch_training == 1)
      change_w_bias(NPTR);
  if (print_flag == 1 && n_printout_E > 0 && 
      NPTR->steps % n_printout_E == 0){
    printf(" %d E=%g (%g+%g) gcor=%g", NPTR->steps,
	   NPTR->E_bp*NPTR->fact_bp_to_xi+NPTR->E_xi, 
	   NPTR->E_bp, NPTR->E_xi, NPTR->gcor); 
    if (NPTR->variable_momentum)
      printf(" mom=%g", NPTR->momentum);
    printf("\n");
    fflush(stdout);
  } 
}




/************************************************************************
 *
 *   NAME:         compute_set_error
 *                 
 *   FUNCTION:     calculates the error over a training set
 *                 value- and slope-error are combined
 *                 
 *   PARAMETERS:   net_ptr NPTR           network
 *                 float fact_bp_to_xi    combining factor (weight)
 *                 int pattern_set        Valid choices are:
 *                                        TRAINING_SET and TESTING_SET
 *                 
 *   RETURN-VALUE: error
 *                 
 ************************************************************************/


float compute_set_error(net_ptr NPTR, float *E_bp, float *E_xi, 
			int pattern_set)
{
  float E_bp_old, E_xi_old;
  int i, count;

  if (train_with_xi && !NPTR->target_xi_defined){
    printf("ERROR in compute_set_error(): not a slope version!\n");
    return -1.0;
  }

  if ((pattern_set < 0 || pattern_set > N_PATTERN_SETS)
      && pattern_set != ALL_SETS){
    printf("ERROR in compute_set_error(): pattern set %d unknown.\n",
	   pattern_set);
    return -1.0;
  }
  

  E_bp_old = NPTR->E_bp;
  E_xi_old = NPTR->E_xi;
  count = 0;
  clear_errors(NPTR);

  for (i = 0; i < N_PATTERN_SETS; i++){
    if (i == pattern_set || pattern_set == ALL_SETS){
      NPTR->active_pattern_set = i; /* ...to get this straigh */
      for (NPTR->actual_pattern[i] = 0; NPTR->actual_pattern[i]
	   < NPTR->npatterns[i]; NPTR->actual_pattern[i]++){
	clamp_pattern_input(NPTR);
	compute_net_x(NPTR);
	if (train_with_xi == 1) compute_xi(NPTR);
	compute_output(NPTR);
	compute_delta_epsil(NPTR); 
	count++;
      }
    }
  }

  *E_bp = NPTR->E_bp / ((float) count) / ((float) NPTR->noutputs);
  *E_xi = NPTR->E_xi / ((float) count) / ((float) NPTR->noutputs)
    / ((float) NPTR->ninputs);

  NPTR->E_bp = E_bp_old;
  NPTR->E_xi = E_xi_old;

  return ((*E_bp * NPTR->fact_bp_to_xi) + *E_xi);
}



/************************************************************************
 *
 *   NAME:         training
 *                 
 *   FUNCTION:     training of a network, for either a pregiven number
 *                 of steps, or until some convergence criterium
 *                 is reached (whatever happens sooner)
 *                 
 *   PARAMETERS:   net_ptr NPTR   network
 *                 int mode_bp    training on values?
 *                 int mode_xi    training on derivatives?
 *                 int clear2nd   shall the 2nd derivatives be cleared
 *                                1=clear, 0=don't clear
 *                                (only relevant if momentum != 0.0)
 *                 
 *   RETURN-VALUE: convergence criterium fulfilled
 *                 
 ************************************************************************/



int training(net_ptr NPTR, int mode_bp, int mode_xi, int clear2nd)
{
  int i, gaussian_defined, fine = 0;
  float current_error, min_error, minE_xi = 0.0, minE_bp = 0.0, E_bp, E_xi;
  int error_going_up, cv_step_number;

  if (mode_xi && !NPTR->target_xi_defined){
    printf("ERROR in training(): not a slope version!\n");
    return -1;
  }

  NPTR->steps = 0;
  train_with_bp = mode_bp;
  train_with_xi = mode_xi;
  NPTR->active_pattern_set = TRAINING_SET; /* training only using this set */

  if (mode_xi){
    gaussian_defined = 0;
    for (i = 0; i < NPTR->nunits; i++)
      if (NPTR->unit_type[i] == GAUSSIAN)
	gaussian_defined = 1;
    if (gaussian_defined)
      printf("ERROR: slope learning not yet implemented for GAUSSIAN units\n");
  }
  
  if (NPTR->npatterns[TRAINING_SET] > 0){

    /* SETTING LERNING RATES 
       if trained on both values and derivatives, the learnrate is
       split with the ratio NPTR->fact_bp_to_xi */
    
    if (train_with_bp && train_with_xi){
      NPTR->lrate_bp_w    = NPTR->lrate * NPTR->fact_bp_to_xi / 
	(1.0 + NPTR->fact_bp_to_xi);
      NPTR->lrate_bp_bias = NPTR->lrate * NPTR->fact_bp_to_xi / 
	(1.0 + NPTR->fact_bp_to_xi);
      NPTR->lrate_xi_w    = NPTR->lrate / (1.0 + NPTR->fact_bp_to_xi);
      NPTR->lrate_xi_bias = NPTR->lrate / (1.0 + NPTR->fact_bp_to_xi);
    }
    else if (train_with_bp){
      NPTR->lrate_bp_w    = NPTR->lrate;
      NPTR->lrate_bp_bias = NPTR->lrate;
      NPTR->lrate_xi_w    = 0.0;
      NPTR->lrate_xi_bias = 0.0;
    }
    else{
      NPTR->lrate_bp_w    = 0.0;
      NPTR->lrate_bp_bias = 0.0;
      NPTR->lrate_xi_w    = NPTR->lrate;
      NPTR->lrate_xi_bias = NPTR->lrate;
    }
    if (clear2nd != 0){
      clear_ddw_ddbias(NPTR);
      clear_gcor_stuff(NPTR);
    }
    if (NPTR->batch_training == 0){
      NPTR->lrate_fact_bp = 1.0 / ((float) NPTR->noutputs);
      NPTR->lrate_fact_xi = 1.0 / ((float) NPTR->noutputs) / 
	((float) NPTR->ninputs);
    }
    else{
      NPTR->lrate_fact_bp = 1.0 / /*((float) NPTR->npatterns[TRAINING_SET]) / */
	((float) NPTR->noutputs);
      NPTR->lrate_fact_xi = 1.0 / /*((float) NPTR->npatterns[TRAINING_SET]) / */
	((float) NPTR->noutputs) 
	  / ((float) NPTR->ninputs);
    }
    
    /* FACTORS FOR THE ERROR FUNCTION - DOESN'T INFLUENCE WEIGHT UPDATE */
    NPTR->E_fact_bp = 1.0 / ((float) NPTR->npatterns[TRAINING_SET]) /
      ((float) NPTR->noutputs);
    NPTR->E_fact_xi = 1.0 / ((float) NPTR->npatterns[TRAINING_SET]) /
      ((float) NPTR->noutputs)  / ((float) NPTR->ninputs);
    min_error = -1.0;
    error_going_up = 0;
    cv_step_number = -1;


    /* MAIN TRAINING LOOP */
    do{ 
      NPTR->steps++;

      
      if (NPTR->do_cross_validation &&
	  (NPTR->steps-1) % NPTR->cross_validation_frequency == 0 && 
	  NPTR->npatterns[TESTING_SET] > 0){ /* cross validation on test set */
	current_error = 0.0;
	if (NPTR->cross_validation_training_error_weight != 0.0)
	  current_error += NPTR->cross_validation_training_error_weight 
	    * compute_set_error(NPTR, &E_bp, &E_xi, TRAINING_SET);
	if (NPTR->cross_validation_training_error_weight != 1.0)
	  current_error += (1.0 - NPTR->cross_validation_training_error_weight)
	    * compute_set_error(NPTR, &E_bp, &E_xi, TESTING_SET);
/*	if (NPTR->cross_validation_training_error_weight > 0.0) ????*/
	if (current_error < min_error || min_error < 0.0){
	  min_error = current_error; 
	  error_going_up = 0;
	  minE_bp = NPTR->E_bp;
	  minE_xi = NPTR->E_xi;
	  push_network(NPTR, 1);
	  cv_step_number = NPTR->steps;
	}
	else if (current_error > min_error)
	  error_going_up = 1;
	
	if (NPTR->do_cross_validation && print_flag == 1 &&
	    n_printout_E > 0 && (NPTR->steps-1) % n_printout_E == 0
	    && min_error >= 0.0)
	  printf("CV-error: %g (%g %d)\n", min_error, current_error,
		 error_going_up); 
      }
      one_step_training(NPTR);

      fine = (NPTR->E_bp <= NPTR->max_E_bp && 
	      NPTR->E_xi <= NPTR->max_E_xi);



    }
    while (NPTR->steps < NPTR->max_steps && !fine);
    
    if (NPTR->do_cross_validation && min_error >= 0.0){
      NPTR->E_bp = minE_bp;
      NPTR->E_xi = minE_xi;
      pop_network(NPTR, 1);
      if (cv_step_number == 1 && NPTR->max_steps > 1)
	fine = 1;
    }
    current_error = compute_set_error(NPTR, &E_bp, &E_xi, TRAINING_SET);
    NPTR->E_bp = E_bp; NPTR->E_xi = E_xi;
    if (print_flag)
      printf("NN2:Result training(NPTR): %g %g in %d/%d (out of %d) steps\n",
	     NPTR->E_bp, NPTR->E_xi, NPTR->steps, cv_step_number,
	     NPTR->max_steps);

    return fine;
  }
  else
    return 1;
  
}





/************************************************************************
 *
 *   NAME:         run_network
 *                 
 *   FUNCTION:     one forward propagation in a network
 *                 
 *   PARAMETERS:   net_ptr NPTR           network
 *                 int mode               0=values only, 1=values+derivatives
 *                 float *input           input values
 *                 float *output_bp       output values
 *                 float *output_xi       output derivatives (NULL if mode==0)
 *                 
 *   NOTE: this procedure can be run with derivatives even if slopes
 *         are *not* defined!
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void run_network(net_ptr NPTR, int mode,
		 float *input, float *output_bp, float *output_xi)
{
  int i, k, xi_i;

  /* CLAMP INPUT */
  for (i = 0; i < NPTR->ninputs; i++)
    NPTR->x[i] = input[i];

  /* PROPAGATE */
  compute_net_x(NPTR);
  if (mode != 0)
    compute_xi(NPTR);
  

  /* RETURN RESULTS */
  xi_i = 0;
  for (k = 0; k < NPTR->noutputs; k++){
    output_bp[k] = NPTR->x[k+NPTR->first_output];
    if (mode != 0)
      for (i = 0; i < NPTR->ninputs; i++){
	output_xi[xi_i++] = NPTR->xi[k+NPTR->first_output][i];
      }
  }
}



/************************************************************************
 *
 *   NAME:         augment_pattern_set
 *                 
 *   FUNCTION:     Augments a pattern set with derivatives by generating
 *                 synthetic points based on these derivatives
 *                 works only, if enough storage available.
 *                 
 *   PARAMETERS:   net_ptr NPTR           network
 *                 int pattern_set        Valid choices are:
 *                                        TRAINING_SET and TESTING_SET
 *                 
 *   RETURN-VALUE: 0, if failure, 1 if success
 *                 
 ************************************************************************/

int augment_pattern_set(net_ptr NPTR, int pattern_set)
{
  int i, k, n, m, d, npatterns;
  float min_dist, dist;
  float syn_input[max_ninputs];
  float syn_target_bp[max_noutputs];
  float syn_target_xi[max_noutputs][max_ninputs];

  if (!NPTR->target_xi_defined){
    printf("ERROR in augment_pattern_set(): not a slope version!\n");
    return 0;
  }

  if (pattern_set < 0 || pattern_set > N_PATTERN_SETS){
    printf("ERROR in (): pattern set %d unknown.\n",
	   pattern_set);
    return 0;
  }

  if (NPTR->npatterns[pattern_set] * (1 + (2 * NPTR->noutputs)) 
      >= NPTR->max_npatterns[pattern_set]){
    printf("NN2: Out of pattern space in augment_pattern_set()\n");
    return 0;
  }

  npatterns = NPTR->npatterns[pattern_set];
  for (n = 0; n < npatterns; n++){ /* over all patterns in the memory */

    /* ****************** PART 1: compute the distance to next pattern */
    min_dist = sqrt((float) NPTR->ninputs) * 0.5; /* canonical distance */
    for (m = 0; m < npatterns; m++){
      dist = 0.0;
      for (i = 0; i < NPTR->ninputs; i++)
	dist += (NPTR->patterns[pattern_set][m].input[i] 
		 - NPTR->patterns[pattern_set][n].input[i])
	  * (NPTR->patterns[pattern_set][m].input[i] 
	     - NPTR->patterns[pattern_set][n].input[i]);
      dist = sqrt(dist);
      if (m != n && dist > 0.0 && dist < min_dist)
	min_dist = dist;
    }

    /* ****************** PART 2: generate sythesized patterns */
    for (d = 0; d < NPTR->ninputs; d++){ /* d is dimension to extrapolate */
      /* ====== PART2a: positive extrapolation ======= */
      for (i = 0; i < NPTR->ninputs; i++) /* first copy pattern */
	syn_input[i] = NPTR->patterns[pattern_set][n].input[i];
      for (k = 0; k < NPTR->noutputs; k++){
	syn_target_bp[k] = NPTR->patterns[pattern_set][n].target_bp[k];
	for (i = 0; i < NPTR->ninputs; i++){
	  if (NPTR->noutputs == 1)
	    syn_target_xi[k][i] 
	      = NPTR->patterns_xi1[pattern_set][n].target_xi[i]; /* end */
	  else
	    syn_target_xi[k][i] 
	      = NPTR->patterns_xi[pattern_set][n].target_xi[k][i]; /* end */
	}
      }	

      syn_input[d] += min_dist * NPTR->augment_factor; /* extrapolate */
      if (NPTR->noutputs == 1)
	for (k = 0; k < NPTR->noutputs; k++) /* loop can be removed! */
	  syn_target_bp[k] += min_dist * NPTR->augment_factor
	    * NPTR->patterns_xi1[pattern_set][n].target_xi[d];
      else
	for (k = 0; k < NPTR->noutputs; k++)
	  syn_target_bp[k] += min_dist * NPTR->augment_factor
	    * NPTR->patterns_xi[pattern_set][n].target_xi[k][d];


      /*!!!! I wonder if this is correct! */
      add_new_pattern(NPTR, 1, syn_input, syn_target_bp, 
		      &(syn_target_xi[0][0]),
		      NPTR->patterns[pattern_set][n].lrate_target_values,
		      NPTR->patterns[pattern_set][n].lrate_target_derivatives,
		      pattern_set);
      
      /* ====== PART2b: negative extrapolation ======= */
      for (i = 0; i < NPTR->ninputs; i++) /* first copy pattern */
	syn_input[i] = NPTR->patterns[pattern_set][n].input[i];
      for (k = 0; k < NPTR->noutputs; k++){
	syn_target_bp[k] = NPTR->patterns[pattern_set][n].target_bp[k];
	if (NPTR->noutputs == 1)
	  for (i = 0; i < NPTR->ninputs; i++)
	    syn_target_xi[k][i] 
	      = NPTR->patterns_xi1[pattern_set][n].target_xi[i]; /* end */
	else
	  for (i = 0; i < NPTR->ninputs; i++)
	    syn_target_xi[k][i] 
	      = NPTR->patterns_xi[pattern_set][n].target_xi[k][i]; /* end */
      }	

      syn_input[d] -= min_dist * NPTR->augment_factor; /* extrapolate */
      if (NPTR->noutputs == 1)
	for (k = 0; k < NPTR->noutputs; k++) /* loop can be removed! */
	  syn_target_bp[k] -= min_dist * NPTR->augment_factor
	    * NPTR->patterns_xi1[pattern_set][n].target_xi[d];
      else
	for (k = 0; k < NPTR->noutputs; k++)
	  syn_target_bp[k] -= min_dist * NPTR->augment_factor
	    * NPTR->patterns_xi[pattern_set][n].target_xi[k][d];
      
      /*!!!! I wonder if this is correct! */
      add_new_pattern(NPTR, 1, syn_input, syn_target_bp, 
		      &(syn_target_xi[0][0]),
		      NPTR->patterns[pattern_set][n].lrate_target_values /*! /
		      ((float) NPTR->ninputs) * 0.5 */,
		      NPTR->patterns[pattern_set][n].lrate_target_derivatives /
		      ((float) NPTR->ninputs),
		      pattern_set);
    }
  }
  return 1;
}




/************************************************************************
 *
 *   NAME:         n_of_outputs
 *                 
 *   FUNCTION:     returns nuber of output units (for the LISP interface)
 *                 
 *   PARAMETERS:   net_ptr NPTR           network
 *                 
 *   RETURN-VALUE: int noutputs
 *                 
 ************************************************************************/


int n_of_outputs(net_ptr NPTR)
{
  return NPTR->noutputs;
}




/************************************************************************
 *
 *   NAME:         n_of_inputs
 *                 
 *   FUNCTION:     returns nuber of output units
 *                 
 *   PARAMETERS:   net_ptr NPTR           network
 *                 
 *   RETURN-VALUE: int inputs
 *                 
 ************************************************************************/


int n_of_inputs(net_ptr NPTR)
{
  return NPTR->ninputs;
}




/************************************************************************
 *
 *   NAME:         init_search
 *                 
 *   FUNCTION:     specifies which part of the network should be searched
 *                 search_in_action_space() seeks to optimize only part of
 *                 the output units, by changing part of the input units.
 *                 
 *   NOTE:         This procedure is optional. If not executed before a 
 *                 search, search_in_action_space() will assume that
 *                 all input and output units are relevant.
 *                 
 *   PARAMETERS:   net_ptr NPTR            network
 *                 int first_search_input  number of first search unit in the
 *                                         input layer
 *                 int first_search_output corresponding unit number in the
 *                                         output layer
 *                 
 *   RETURN-VALUE: none
 *
 ************************************************************************/


void init_search(net_ptr NPTR, int first_search_input, int first_search_output)
{
  if (first_search_input >= 0 && first_search_input <= NPTR->ninputs)
    NPTR->first_search_input = first_search_input;
  else
    printf("Warning: invalid value #1 in init_search-parameter not changed\n");
  if (first_search_output >= 0 && first_search_output <= NPTR->noutputs)
    NPTR->first_search_output = first_search_output;
  else
    printf("Warning: invalid value #2 in init_search-parameter not changed\n");
}



/************************************************************************
 *
 *   NAME:         search_in_action_space
 *                 
 *   NOTE:         This procedure searches the input spa in two steps:
 *                 a) grid search, on NPTR->search_grid_size^dim-input-space
 *                    points
 *                 b) starting with the winner from grid search, gradient
 *                    descent search, stepsize NPTR->search_lrate. Search
 *                    is stopps either after NPTR->search_max_nsteps steps,
 *                    or if none of the changed values is changed by
 *                    more than NPTR->search_min_change.
 *                    Values are bounded in the interval
 *                    [NPTR->lower_bound[i], NPTR->upper_bound[i]]
 *                 
 *   OPTIMIZATION: The search seeks to maximize some or all output
 *                 values, starting with the index NPTR->first_search_output
 *                 (typically set to first_output-nstates, and thus referres
 *                 to reward-prediction units). The search input starts
 *                 with the index NPTR->first_search_input. Thus, for search, 
 *                 the input must be partitioned by first-states-
 *                 then-actions, and the output space must be partitioned 
 *                 by first-states-then-rewards.
 *                 
 *   LIMITATIONS:  In the current version, the bounding interval cannot
 *                 be changed, and is set to [0,1]. 
 *                 
 *   FUNCTION:     Optimizes reward.
 *                 
 *   PARAMETERS:   net_ptr NPTR            network
 *                 float *inputs           initial search value (if not
 *                                         grid search)
 *                                         returns: optimized input
 *                 int norm                optimization norm: 1=L1, 2=L2
 *                 
 *   RETURN-VALUE: float reward            predicted reward
 *                 
 ************************************************************************/



float evaluate_output_deviation(net_ptr NPTR, int norm) 
     /* requires that input is clamped on */
{     
  float dev;
  int i;

  /* clamp_input(NPTR, input); */
  compute_net_x(NPTR);
  dev = 0.0;
  for (i = NPTR->first_output+NPTR->first_search_output; i < NPTR->nunits; i++)
    if (norm == 1)
      dev += fabs(NPTR->x[i]);
    else
      dev += NPTR->x[i] * NPTR->x[i];
  return dev;
}  




int change_input(net_ptr NPTR, int norm)
{
  int i,k;
  float old[max_ninputs];
  int change;

  change = 0;
  for (i = NPTR->first_search_input; i < NPTR->ninputs; i++){
    old[i] = NPTR->x[i];
    if (norm == 1)
      for (k = NPTR->first_output + NPTR->first_search_output;
	   k < NPTR->nunits; k++)
	NPTR->x[i] += NPTR->search_lrate * NPTR->xi[k][i];
    else /* norm == 2 */
      for (k = NPTR->first_output + NPTR->first_search_output;
	   k < NPTR->nunits; k++)
	NPTR->x[i] += NPTR->search_lrate * (1.0 - NPTR->x[k]) * NPTR->xi[k][i];
    
    if (NPTR->x[i] < NPTR->lower_bound[i]) 
      NPTR->x[i] = NPTR->lower_bound[i];
    if (NPTR->x[i] > NPTR->upper_bound[i])
      NPTR->x[i] = NPTR->upper_bound[i];
    if (fabs(old[i] - NPTR->x[i]) > NPTR->search_min_change)
      change = 1;
  }
  return change;
}




float search_in_action_space(net_ptr NPTR, float *input, int norm)
{   
  int i,m, carry, valid;
  int change = 1;
  float best_reward, worst_reward, reward;
  int test_value[max_ninputs];
  float best_value[max_ninputs];

  if (norm != 1 && norm != 2)
    printf("WARNING: norm L%d not implemented - take L2 instead.\n", norm);
   

  /******* NO EXPLICIT SEARCH REQUIRED - RETURN OUTPUT VALUES *******/

  if (NPTR->first_search_input >= NPTR->ninputs ||
      NPTR->first_search_output >= NPTR->noutputs){
    clamp_input(NPTR, input);
    reward = evaluate_output_deviation(NPTR, norm);
    if (print_flag == 1 && n_printout_search_result > 0 && 
	NPTR->steps % n_printout_search_result == 0)
      printf("search_in_action_space() - no search -> %g\n", reward);
  }    



  /******* SEARCH REQUIRED - FIRST GRID-SEARCH, THEN GRAD-DESC SEARCH *******/

  else{
    /* PART 1 - CLAMP ON STATE INPUTS, IF ANY */
    clamp_input(NPTR, input);
    
    /* PART 2 - GRID SEARCH */
    
    best_reward = -1.0;
    worst_reward = 999999999999999.0;
    for (i = 0; i < max_ninputs; i++)
      test_value[i] = 0; /* memorizes current gridindex */
    
    do{
      valid = 1;
      for (i = NPTR->first_search_input; i < NPTR->ninputs; i++){
	NPTR->x[i] = ((float) test_value[i]) / 
	  ((float) NPTR->search_grid_size);
	if (NPTR->x[i] < NPTR->lower_bound[i] ||
	    NPTR->x[i] > NPTR->upper_bound[i])
	  valid = 0;	    /* WARNING: THIS MAY FAIL IN FINDING ALL VALUES */
      }
      if (valid == 1){
	reward =  (evaluate_output_deviation(NPTR, norm));
	if (reward > best_reward){
	  best_reward = reward;
	  for (i = NPTR->first_search_input; i < NPTR->ninputs; i++)
	    best_value[i] = NPTR->x[i];
	}
	if (reward < worst_reward)
	  worst_reward = reward;
      }
      
      carry = 1; /* grid index increment */
      for (i = NPTR->first_search_input; i < NPTR->ninputs; i++){
	test_value[i] += carry;
	if (test_value[i] > NPTR->search_grid_size) 
	  test_value[i] = 0;
	else
	  carry = 0;
      }
    }
    while (carry == 0);
    if (best_reward < 0.0)
      printf("\nERROR: no action found in search_in_action_space().\n");
    else{
      for (i = NPTR->first_search_input; i < NPTR->ninputs; i++)
	NPTR->x[i] = best_value[i];
    }
    
    
    /* PART 3 - GRADIENT DESCENT SEARCH */
    
    change = 1;
    for (m = 0; m < NPTR->search_max_nsteps && change == 1; m++){
      compute_net_x(NPTR);
      compute_xi(NPTR);
      change = change_input(NPTR, norm);
    }
    
    
    /* PART 4 - COMPUTATION OF THE RESULTS  */
    
    for (i = 0; i < NPTR->ninputs; i++)
      input[i] = NPTR->x[i];
    
    reward = evaluate_output_deviation(NPTR, norm);
    if (print_flag == 1 && n_printout_search_result > 0 && 
	NPTR->steps % n_printout_search_result == 0)
      printf("search_in_action_space() improvement: %g %g -> %g\n",
	     worst_reward, best_reward, reward);
    
  }

  return reward;
}




/************************************************************************
 *
 *   NAME:         two_dim_mathematica_plot 
 *                 
 *   FUNCTION:     Plots an output function performed by a network
 *                 along two (user-specified) dimensions.
 *                 
 *   PARAMETERS:   net_ptr NPTR           network
 *                 float *input           input vector
 *                 int  input_1, input2   2 input variables to be plotted
 *                 char *filename         Name of the mathematica file
 *                 char *title_string     Name of the chart
 *                 
 *   NOTE:         If the code is compiled with the -DLISP flag, the
 *                 variables input_1 and input_2 are 1-based!
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void two_dim_mathematica_plot(net_ptr NPTR, 
			      float *input,
			      int input_1, 
			      int input_2,
			      char *filename,
			      char *title_string)
{
  int i, j, k;
  char math_filename[80];
  FILE *iop;
  float output[max_noutputs];
  char today[80], title[80], listname[80];
  struct timeval tp;

#ifdef LISP
  input_1--;
  input_2--;
#endif

  if (NPTR == NULL)
    printf("\nWARNING: Netowrk not defined. No output generated.\n");
  else if (input_1 >= NPTR->ninputs  || input_1 < 0 ||
	   input_2 >= NPTR->ninputs  || input_2 < 0)
    printf("\nWARNING: Illegal input parameters. No output generated.\n");

  else{
    gettimeofday(&tp, NULL);	
    for (k = 0; k < NPTR->noutputs; k++){
      if (NPTR->noutputs == 1){
	sprintf(math_filename, "%s", filename);
	sprintf(listname, "l");
      }
      else{
	sprintf(math_filename, "%s.%.3d", filename, k);
	sprintf(listname, "l%.3d", k);
      }
      if ((iop = fopen(math_filename, "w")) == 0)
	printf("Error when attempting to open %s.\n", math_filename);
      else{
	printf("Generating Mathematic output file %s...", math_filename);
	fflush(stdout);
	fprintf(iop, "(* How to display me:\n");
	fprintf(iop, "\t- use a machine that runs Mathematica (e.g. mm)\n");
	fprintf(iop, "\t- setenv DISPLAY to your favorite machine\n");
	fprintf(iop, "\t- start Mathematica by typing 'math'\n");
	fprintf(iop, "\t- type '<<\"%s\"' and wait.\n*)\n\n", math_filename);
	if (NPTR->ninputs > 2){
	  fprintf(iop, "(* Values of the other input units:");
	  for (i = 0; i < NPTR->ninputs; i++)
	    if (i != input_1 && i != input_2)
	      fprintf(iop, "  unit_%d = %g", i, input[i]);
	  fprintf(iop, " *)\n\n");
	}
	fprintf(iop, "%s={", listname);
	for (i = 0; i <= NPTR->mathematica_gridsize; i++){
	  if (i != 0)
	    fprintf(iop, ",\n");
	  fprintf(iop, "{");
	  for (j = 0; j <= NPTR->mathematica_gridsize; j++){
	    input[input_2] 
	      = ((float) i) / ((float) NPTR->mathematica_gridsize);
	    input[input_1] 
	      = ((float) j) / ((float) NPTR->mathematica_gridsize);
	    run_network(NPTR, 0, input, output, NULL);
	    if (j != 0)
	      fprintf(iop, ",");
	    fprintf(iop, "%g", output[k]);
	  }
	  fprintf(iop, "}");
	}
	fprintf(iop, "}\n\n");
	fprintf(iop, "ListPlot3D[%s,PlotRange->{{1,%d},{1,%d},{0,1}},\n",
		listname, NPTR->mathematica_gridsize+1, 
		NPTR->mathematica_gridsize+1);
#ifdef LISP
	fprintf(iop, "AxesLabel->{\"unit %d\",\"unit %d\",\"output %d\"},\n",
		input_1+1, input_2+1, k);
#else
	fprintf(iop, "AxesLabel->{\"unit %d\",\"unit %d\",\"output %d\"},\n",
		input_1, input_2, k);
#endif
	fprintf(iop, "Ticks->{{{1,0},{%g,0.5},{%d,1}},",
		((float) (NPTR->mathematica_gridsize+1)) * 0.5,
		NPTR->mathematica_gridsize+1);
	fprintf(iop, "{{1,0},{%g,0.5},{%d,1}},Automatic},\n",
		((float) (NPTR->mathematica_gridsize+1)) * 0.5,
		NPTR->mathematica_gridsize+1);
#ifdef LISP	
	sprintf(title, "%s", title_string);
#else	
	strftime(today, 99, "%m-%d-%y %H:%M", localtime((time_t *) &(tp.tv_sec)));
	sprintf(title, "%s (%s)", title_string, today);
#endif
	fprintf(iop, "PlotLabel->FontForm[\"%s\",{\"Helvetica-Bold\",14}]]\n",
		title);
	

	fclose(iop);
	printf("done.\n");
	fflush(stdout);
      }
    }
  }
}



/************************************************************************
 *
 *   NAME:         set_float
 *                 
 *   FUNCTION:     sets a floating-point parameter, either a global
 *                 variable, or for a specific network
 *                 
 *   PARAMETERS:   net_ptr NPTR           network (will be ignored if
 *                                        global variable (global==1)
 *                 int global             1=global, 0=network-specific
 *                 char *name             name of the variable, see procedure
 *                                        below
 *                 float value            value
 *                 
 *   RETURN-VALUE: int error              1=error occurred, 0=success
 *                 
 ************************************************************************/


int set_float(net_ptr NPTR, int global, char *name, float value)
{
  int ret = 0;

  if (global == 1){		/* global variable */
    ret = 1; /* no global float values yet! */
    printf("\nWARNING: variable name %s (float global) unknown!\n", name);
  }

  else if (global == 0){	/* network-specific variable */
    if (!strcmp(name,"lrate") || !strcmp(name,"LRATE"))
      NPTR->lrate = value;
    else if (!strcmp(name,"fact_bp_to_xi") || !strcmp(name,"FACT_BP_TO_XI"))
      NPTR->fact_bp_to_xi = value;
    else if (!strcmp(name,"decay") || !strcmp(name,"DECAY"))
      NPTR->decay = value;
    else if (!strcmp(name,"momentum") || !strcmp(name,"MOMENTUM"))
      NPTR->momentum = value;
    else if (!strcmp(name,"max_E_bp") || !strcmp(name,"MAX_E_BP"))
      NPTR->max_E_bp = value;
    else if (!strcmp(name,"max_E_xi") || !strcmp(name,"MAX_E_XI"))
      NPTR->max_E_xi = value;
    else if (!strcmp(name,"search_lrate") || !strcmp(name,"SEARCH_LRATE"))
      NPTR->search_lrate = value;
    else if (!strcmp(name,"weights_init_range") 
	     || !strcmp(name,"WEIGHTS_INIT_RANGE"))
      NPTR->weights_init_range = value;
    else if (!strcmp(name,"weights_init_range") || 
	     !strcmp(name,"WEIGHTS_INIT_RANGE"))
      NPTR->weights_init_range = value;
    else if (!strcmp(name,"search_min_change") 
	     || !strcmp(name,"SEARCH_MIN_CHANGE"))
      NPTR->search_min_change = value;
    else if (!strcmp(name,"cross_validation_training_error_weight")
	     || !strcmp(name,"CROSS_VALIDATION_TRAINING_ERROR_WEIGHT"))
      NPTR->cross_validation_training_error_weight = value;
    else if (!strcmp(name,"augment_factor") 
	     || !strcmp(name, "augment_factor"))
      NPTR->augment_factor = value;
    else {
      ret = 1;
      printf("\nWARNING: variable name %s (float local) unknown!\n", name);
    }
  }

  return ret;
}


/************************************************************************
 *
 *   NAME:         set_int
 *                 
 *   FUNCTION:     sets a integer parameter, either a global
 *                 variable, or for a specific network
 *                 
 *   PARAMETERS:   net_ptr NPTR           network (will be ignored if
 *                                        global variable (global==1)
 *                 int global             1=global, 0=network-specific
 *                 char *name             name of the variable, see procedure
 *                                        below
 *                 int value              value
 *                 
 *   RETURN-VALUE: int error              1=error occurred, 0=success
 *                 
 ************************************************************************/




int set_int(net_ptr NPTR, int global, char *name, int value)
{
  int ret = 0;
  
  if (global == 1){		/* global variable */
    if (!strcmp(name,"print_flag") || !strcmp(name,"PRINT_FLAG"))
      print_flag = value;
    else if (!strcmp(name,"n_printout_E") || !strcmp(name,"N_PRINTOUT_E"))
      n_printout_E = value;
    else if (!strcmp(name,"n_printout_search_result")
	     || !strcmp(name,"N_PRINTOUT_SEARCH_RESULT"))
      n_printout_search_result = value;
    else if (!strcmp(name,"n_printout_patterns") 
	     || !strcmp(name,"N_PRINTOUT_PATTERNS"))
      n_printout_patterns = value;
    else if (!strcmp(name,"n_printout_params") 
	     || !strcmp(name,"N_PRINTOUT_PARAMS"))
      n_printout_params = value;
    else if (!strcmp(name,"random_generator") 
	     || !strcmp(name,"RANDOM_GENERATOR"))
      random_generator = value;
    else {
      ret = 1;
      printf("\nWARNING: variable name %s (int global) unknown!\n", name);
    }
  }

  else if (global == 0){	/* network-specific variable */
    if (!strcmp(name,"batch_training") || !strcmp(name,"BATCH_TRAINING"))
      NPTR->batch_training = value;
    else if (!strcmp(name,"max_npatterns_train")
	     || !strcmp(name,"MAX_NPATTERNS_TRAIN"))
      NPTR->max_npatterns[TRAINING_SET] = value;
    else if (!strcmp(name,"max_npatterns_test")
	     || !strcmp(name,"MAX_NPATTERNS_TEST"))
      NPTR->max_npatterns[TESTING_SET] = value;
    else if (!strcmp(name,"max_steps") || !strcmp(name,"MAX_STEPS"))
      NPTR->max_steps = value;
    else if (!strcmp(name,"search_grid_size")
	     || !strcmp(name,"SEARCH_GRID_SIZE"))
      NPTR->search_grid_size = value;
    else if (!strcmp(name,"search_max_nsteps")
	     || !strcmp(name,"SEARCH_MAX_NSTEPS"))
      NPTR->search_max_nsteps = value;
    else if (!strcmp(name,"mathematica_gridsize")
	     || !strcmp(name,"MATHEMATICA_GRIDSIZE"))
      NPTR->mathematica_gridsize = value;
    else if (!strcmp(name,"do_cross_validation")
	     || !strcmp(name,"DO_CROSS_VALIDATION"))
      NPTR->do_cross_validation = value;
    else if (!strcmp(name,"cross_validation_frequency")
	     || !strcmp(name,"CROSS_VALIDATION_FREQUENCY"))
      NPTR->cross_validation_frequency = value;
    else if (!strcmp(name,"variable_momentum")
	     || !strcmp(name,"VARIABLE_MOMENTUM"))
      NPTR->variable_momentum = value;
    else if (!strcmp(name,"active_pattern_set")
	     || !strcmp(name,"ACTIVE_PATTERN_SET")){
      if (value >= 0 && value < N_PATTERN_SETS)
	NPTR->active_pattern_set = value;
      else
	printf("ERROR: active_pattern_set=%d exceeds range.\n", value);
    }
    else {
      ret = 1;
      printf("\nWARNING: variable name %s (int local) unknown!\n", name);
    }
  }
  
  return ret;
}


/*               DUMMY FOR FURTHER VARIABLES:

    else if (!strcmp(name,"") || !strcmp(name,""))
       = value;

    else if (!strcmp(name,"") || !strcmp(name,""))
      NPTR-> = value;
*/



/************************************************************************
 *
 *   NAME:         get_float
 *                 
 *   FUNCTION:     gets value of a floating-point parameter, either a global
 *                 variable, or for a specific network
 *                 
 *   PARAMETERS:   net_ptr NPTR           network (will be ignored if
 *                                        global variable (global==1)
 *                 int global             1=global, 0=network-specific
 *                 char *name             name of the variable, see procedure
 *                                        below
 *
 *   COMMENTS:  WHY ON EARTH DOESN'T THIS WORK!!!
 *
 *   RETURN-VALUE: the value of the floating-point parameter
 *                 
 ************************************************************************/


float get_float(net_ptr NPTR, int global, char *name)
{
  float ret = 0.0;

  if (global == 1){             /* global variable */
    /* no global float values yet! */
    printf("\nWARNING: variable name %s (float global) unknown!\n", name);
  }

  else if (global == 0){        /* network-specific variable */
    if (!strcmp(name,"lrate") || !strcmp(name,"LRATE"))
      ret = NPTR->lrate;
    else if (!strcmp(name,"fact_bp_to_xi") || !strcmp(name,"FACT_BP_TO_XI"))
      ret = NPTR->fact_bp_to_xi;
    else if (!strcmp(name,"decay") || !strcmp(name,"DECAY"))
      ret = NPTR->decay;
    else if (!strcmp(name,"momentum") || !strcmp(name,"MOMENTUM"))
      ret = NPTR->momentum;
    else if (!strcmp(name,"max_E_bp") || !strcmp(name,"MAX_E_BP"))
      ret = NPTR->max_E_bp;
    else if (!strcmp(name,"max_E_xi") || !strcmp(name,"MAX_E_XI"))
      ret = NPTR->max_E_xi;
    else if (!strcmp(name,"search_lrate") || !strcmp(name,"SEARCH_LRATE"))
      ret = NPTR->search_lrate;
    else if (!strcmp(name,"search_min_change") 
             || !strcmp(name,"SEARCH_MIN_CHANGE"))
      ret = NPTR->search_min_change;
    else {
      printf("\nWARNING: variable name %s (float local) unknown!\n", name);
    }
  }
  return ret;
}



/************************************************************************
 *
 *   NAME:         get_int
 *                 
 *   FUNCTION:     gets an integer parameter, either a global
 *                 variable, or for a specific network
 *                 
 *   PARAMETERS:   net_ptr NPTR           network (will be ignored if
 *                                        global variable (global==1)
 *                 int global             1=global, 0=network-specific
 *                 char *name             name of the variable, see procedure
 *                                        below
 *   RETURN-VALUE: the integer value of the parameter
 *                 
 ************************************************************************/

int get_int(net_ptr NPTR, int global, char *name)
{
  int ret = 0;
  
  if (global == 1){             /* global variable */
    if (!strcmp(name,"print_flag") || !strcmp(name,"PRINT_FLAG"))
      ret = print_flag;
    else if (!strcmp(name,"n_printout_E") || !strcmp(name,"N_PRINTOUT_E"))
      ret = n_printout_E;
    else if (!strcmp(name,"n_printout_search_result")
             || !strcmp(name,"N_PRINTOUT_SEARCH_RESULT"))
      ret = n_printout_search_result;
    else if (!strcmp(name,"n_printout_patterns") 
             || !strcmp(name,"N_PRINTOUT_PATTERNS"))
      ret = n_printout_patterns;
    else if (!strcmp(name,"n_printout_params") 
             || !strcmp(name,"N_PRINTOUT_PARAMS"))
      ret = n_printout_params;
    else if (!strcmp(name,"random_generator") || 
             !strcmp(name,"RANDOM_GENERATOR"))
      ret = random_generator;
    else {
      printf("\nWARNING: variable name %s (int global) unknown!\n", name);
    }
  }

  else if (global == 0){        /* network-specific variable */
    if (!strcmp(name,"batch_training") || !strcmp(name,"BATCH_TRAINING"))
      ret = NPTR->batch_training;
    else if (!strcmp(name,"max_npatterns_train") ||
	     !strcmp(name,"MAX_NPATTERNS_TRAIN"))
      ret = NPTR->max_npatterns[TRAINING_SET];
    else if (!strcmp(name,"max_npatterns_test") ||
	     !strcmp(name,"MAX_NPATTERNS_TEST"))
      ret = NPTR->max_npatterns[TESTING_SET];
    else if (!strcmp(name,"max_steps") || !strcmp(name,"MAX_STEPS"))
      ret = NPTR->max_steps;
    else if (!strcmp(name,"search_grid_size")
             || !strcmp(name,"SEARCH_GRID_SIZE"))
      ret = NPTR->search_grid_size;
    else if (!strcmp(name,"search_max_nsteps")
             || !strcmp(name,"SEARCH_MAX_NSTEPS"))
      ret = NPTR->search_max_nsteps;
    else if (!strcmp(name,"mathematica_gridsize")
             || !strcmp(name,"MATHEMATICA_GRIDSIZE"))
      ret = NPTR->mathematica_gridsize;
    else if (!strcmp(name,"active_pattern_set")
             || !strcmp(name,"ACTIVE_PATTERN_SET"))
      ret = NPTR->active_pattern_set;
    else if (!strcmp(name,"do_cross_validation")
             || !strcmp(name,"DO_CROSS_VALIDATION"))
      ret = NPTR->do_cross_validation;
    else if (!strcmp(name,"cross_validation_frequency")
             || !strcmp(name,"CROSS_VALIDATION_FREQUENCY"))
      ret = NPTR->cross_validation_frequency;
    else if (!strcmp(name,"variable_momentum")
             || !strcmp(name,"VARIABLE_MOMENTUM"))
      ret = NPTR->variable_momentum;
    else {
      printf("\nWARNING: variable name %s (int local) unknown!\n", name);
    }
  }
  
  return ret;
}


/************************************************************************
 *
 *   NAME:         normal_distribution
 *                 
 *   FUNCTION:     to return a random number drawn from the uniform 
 *                 distribution over the interval [0.0, 1.0]
 *                 
 *   RETURN-VALUE: float
 *                 
 ************************************************************************/



#ifndef RAND_MAX
#define RAND_MAX ((float)(2147483647))     /* (2**31 - 1) */
#endif

#ifndef RANDOM_MAX
#define RANDOM_MAX ((float)(2147483647))   /* (2**31 - 1) */
#endif

#ifndef LRAND48_MAX 
#define LRAND48_MAX ((float)(2.147483648e9)) /* (2**31) */
#endif

static LISP_warning = 0;



float uniform_distribution()
{
  switch (random_generator) {
  case 1:
    return((float)(random())/RANDOM_MAX);
    break;
  case 2:
#ifdef LISP	
    if (!LISP_warning)
      printf("WARNING: lrand48() not defined in LISP. Chose a different random number generator.\n");
    LISP_warning = 1;
    return((float)(random())/RANDOM_MAX);
#else
    return((float)(lrand48())/LRAND48_MAX);
#endif
    break;
  case 3:
    return((float)(rand())/RAND_MAX);
    break;
  default:
    printf("# ERROR : out of range for random_generator (= %d)\n",
	   random_generator);
    exit(1);
    return -9999.9;		/* this makes the compiler happy */
    break;
  }
}



/************************************************************************
 *
 *   NAME:         test_code
 *                 
 *   FUNCTION:     just a little test procedure
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


/* three functions: 1=linear, 2=nonlinear, 3=step function */


float f(int i, float x, float y, float z, float w)
{
  float s;

  if (i == 1)
    return(0.5*(x+y));

  /* this is the g fn - note it is indep of z,w */
  else if (i == 2){  
    s = ((x-0.2)*(x-0.2)) + ((y-0.5)*(y-0.5));
    s = exp ( -10.0 * s);
    s = (0.5 * s) + (sin(8.0*x*y)*0.25) + 0.25;
    return s;
  }

  else if (i == 3){
    if (y<0.5-0.5*x || y>2.0-2.0*x)
      return 0.0;
    else
      return 1.0;
  }
  else return -9999.9;
}


float df_dx(int i, float x, float y, float z, float w)
{
  float s;

  if (i == 1)
    return 0.5;
  else if (i == 2){  /* partial g by partial x */
    s = ((x-0.2)*(x-0.2)) + ((y-0.5)*(y-0.5));
    s = exp ( -10.0 * s);
    s = (-10.0 * (x-0.2) * s) + (2.0 * y * cos(8.0*x*y));
    return s;
  }
  else if (i == 3)
    return 0.0;
  else return -9999.9;
}



float df_dy(int i, float x, float y, float z, float w)
{
  float s;
  
  if (i == 1)
    return 0.5;
  else if (i == 2){  
    s = ((x-0.2)*(x-0.2)) + ((y-0.5)*(y-0.5));
    s = exp ( -10.0 * s);
    s = (-10.0 * (y-0.5) * s) + (2.0 * x * cos(8.0*x*y));
    return s;
  }
  else if (i == 3)
    return 0.0;
  else return -9999.9;
}



float df_dz(int i, float x, float y, float z, float w)
{
  return 0.0;
}



float df_dw(int i, float x, float y, float z, float w)
{
  return 0.0;
}

 
  /******************************************************************
   ******************************************************************
   ******************************************************************
   ******************************************************************/

void test_code2(int test_function) /* this is the original routine by Seb. */
{  
  int n, i, j, k;
  float testE;
  net_ptr mynet;
  float input[2];
  float target_bp[1];
  float output_bp[1];
  float target_xi[2];
  int number_training_pat;
  static int test_grid_size = 200;

  printf("Welcome to test_code().\n");
  printf("Parameter: %d\n", test_function);
  mynet = create_network(2,6,0,1,1000,100,1);		/* 4,6,0,1,1000 */
  modify_unit_type(mynet, 0, 2+6+0+1, LOGISTIC); /* or: LOGISTIC, COS_FADE */


  /* training on values is <fact_bp_to_xi> as strong as training on slopes */

  set_float(mynet, 0, "lrate", 0.01); 		/* 0.4 */
  set_float(mynet, 0, "fact_bp_to_xi", 8.0); 	/* 16.0 */
  set_float(mynet, 0, "decay", 0.0); 		/* 0.000001 */
  set_float(mynet, 0, "momentum", 0.9);		/* 0.7 */
  set_int  (mynet, 0, "max_steps", 10000); 	/* 10000 */
  set_float(mynet, 0, "max_E_bp", 0.0);		/* 0.001 */
  set_float(mynet, 0, "max_E_xi", 0.0); 	/* 0.01 */
  set_float(mynet, 0, "augment_factor", 0.2);

  /* control output on screen during training */
  set_int  (NULL,  1, "print_flag", 1);
  set_int  (NULL,  1, "n_printout_params", 9999);
  set_int  (NULL,  1, "n_printout_E", 100); 	
  set_int  (NULL,  1, "n_printout_patterns", 9999);
  
  /******************************************************************
   ******************************************************************
   ******************************************************************
   ******************************************************************/

  clear_pattern_set(mynet, ALL_SETS);    
  srandom(10);
  input[0]        = 0.5;
  input[1]        = 0.5;
  for (n = 0; n < 30; n++){
      input[0] = ((float) n) / 9.0;
      input[1] = ((float) (9-n)) / 9.0;
      target_bp[0]    = f(test_function, input[0], input[1], 0.0, 0.0);
      target_xi[0]    = df_dx(test_function, input[0], input[1], 0.0, 0.0);
      target_xi[1]    = df_dy(test_function, input[0], input[1], 0.0, 0.0);
      add_new_pattern(mynet, 1, input, target_bp, target_xi, 1.0, 1.0,
		      TRAINING_SET);
    }
  save_patterns(mynet, "train1.pat", 1, TRAINING_SET);
  save_patterns(mynet, "test1.pat",  1, TESTING_SET);

  generate_random_testing_set(mynet, TESTING_SET, 0.3);
  
  save_patterns(mynet, "train2.pat", 1, TRAINING_SET);
  save_patterns(mynet, "test2.pat",  1, TESTING_SET);
  
  set_int  (mynet, 0, "cross_validation_frequency", 100);
  set_float(mynet, 0, "cross_validation_training_error_weight", 0.5);
  set_int  (mynet, 0, "variable_momentum", 1);

  training(mynet, 1, 0, 1); /* (net, values?, slopes? clear2ndDer?) */
  exit(1);
  
  /******************************************************************
   ******************************************************************
   ******************************************************************
   ******************************************************************/

  for (number_training_pat = 1; number_training_pat < 10;
       number_training_pat++){
    
    clear_pattern_set(mynet, ALL_SETS);    
    /* bp == values, xi == slopes */
    /* initialize with some random patterns */
    srandom(10);
    input[0]        = 0.5;
    input[1]        = 0.5;
    for (n = 0; n < number_training_pat * 3; n++){
      target_bp[0]    = f(test_function, input[0], input[1], 0.0, 0.0);
      target_xi[0]    = df_dx(test_function, input[0], input[1], 0.0, 0.0);
      target_xi[1]    = df_dy(test_function, input[0], input[1], 0.0, 0.0);
      if (n == number_training_pat) printf("-----------------------\n");
      if (n < number_training_pat)
	add_new_pattern(mynet, 1, input, target_bp, target_xi, 1.0, 1.0,
			TRAINING_SET);
      else
	add_new_pattern(mynet, 1, input, target_bp, target_xi, 1.0, 1.0,
			TESTING_SET);
      printf("%g %g -> %g %g %g\n",input[0],input[1], 
	     target_bp[0],target_xi[0] ,target_xi[1]);
      input[0] = uniform_distribution();
      input[1] = uniform_distribution();
    }
    
    
    srandom(10000);
    init_weights_random(mynet);
    set_int(mynet, 0, "do_cross_validation", 1);
    (void) training(mynet, 1, 0, 1); /* (net, values?, slopes? clear2ndDer?) */

    two_dim_mathematica_plot(mynet, input, 0, 1, "x.math", "title");
  
    /* ********** GENERALIZATION ERROR **************************** */
    testE = 0.0;
    for (i = 0; i < test_grid_size+1; i++){
      input[0] = ((float) i) / ((float) test_grid_size);
      for (j = 0; j < test_grid_size+1; j++){
	input[1] = ((float) j) / ((float) test_grid_size);
	target_bp[0]    = f(test_function, input[0], input[1], 0.0, 0.0);
	run_network(mynet, 0, input, output_bp, NULL);
	testE += (output_bp[0] - target_bp[0]) 
	  * (output_bp[0] - target_bp[0]);
      }      
    }
    testE /= (float) (test_grid_size+1);
    testE /= (float) (test_grid_size+1);
    printf("-----> generalization error (val, cross): %g  (1 %d)\n", testE, 
	   number_training_pat);
    /* ************************************************************ */
    

    srandom(10000);
    init_weights_random(mynet);
    set_int(mynet, 0, "do_cross_validation", 0);
    (void) training(mynet, 1, 0, 1); /* (net, values?, slopes? clear2ndDer?) */

    two_dim_mathematica_plot(mynet, input, 0, 1, "x.math", "title");

    
    /* ********** GENERALIZATION ERROR **************************** */
    testE = 0.0;
    for (i = 0; i < test_grid_size+1; i++){
      input[0] = ((float) i) / ((float) test_grid_size);
      for (j = 0; j < test_grid_size+1; j++){
	input[1] = ((float) j) / ((float) test_grid_size);
	target_bp[0]    = f(test_function, input[0], input[1], 0.0, 0.0);
	run_network(mynet, 0, input, output_bp, NULL);
	testE += (output_bp[0] - target_bp[0]) 
	  * (output_bp[0] - target_bp[0]);
      }      
    }
    testE /= (float) (test_grid_size+1);
    testE /= (float) (test_grid_size+1);
    printf("-----> generalization error (val, no cross): %g  (1 %d)\n", testE, 
	   number_training_pat);
    /* ************************************************************ */
    


    srandom(10000);
    init_weights_random(mynet);
    set_int(mynet, 0, "do_cross_validation", 1);
    (void) training(mynet, 1, 1, 1); /* (net, values?, slopes? clear2ndDer?) */
    
    
    /* ********** GENERALIZATION ERROR **************************** */
    testE = 0.0;
    for (i = 0; i < test_grid_size+1; i++){
      input[0] = ((float) i) / ((float) test_grid_size);
      for (j = 0; j < test_grid_size+1; j++){
	input[1] = ((float) j) / ((float) test_grid_size);
	target_bp[0]    = f(test_function, input[0], input[1], 0.0, 0.0);
	run_network(mynet, 0, input, output_bp, NULL);
	testE += (output_bp[0] - target_bp[0]) 
	  * (output_bp[0] - target_bp[0]);
      }      
    }
    testE /= (float) (test_grid_size+1);
    testE /= (float) (test_grid_size+1);
    printf("-----> generalization error (slopes, cross): %g  (1 %d)\n", testE, 
	   number_training_pat);
    /* ************************************************************ */


    srandom(10000);
    init_weights_random(mynet);
    set_int(mynet, 0, "do_cross_validation", 0);
    (void) training(mynet, 1, 1, 1); /* (net, values?, slopes? clear2ndDer?) */
    
    
    /* ********** GENERALIZATION ERROR **************************** */
    testE = 0.0;
    for (i = 0; i < test_grid_size+1; i++){
      input[0] = ((float) i) / ((float) test_grid_size);
      for (j = 0; j < test_grid_size+1; j++){
	input[1] = ((float) j) / ((float) test_grid_size);
	target_bp[0]    = f(test_function, input[0], input[1], 0.0, 0.0);
	run_network(mynet, 0, input, output_bp, NULL);
	testE += (output_bp[0] - target_bp[0]) 
	  * (output_bp[0] - target_bp[0]);
      }      
    }
    testE /= (float) (test_grid_size+1);
    testE /= (float) (test_grid_size+1);
    printf("-----> generalization error (slopes, no cross): %g  (1 %d)\n",
	   testE, number_training_pat);
    /* ************************************************************ */


#ifdef jjjjjjjjjjjjjjjjjjjjjjjjjjj   
    srandom(10000);
    init_weights_random(mynet);
    augment_pattern_set(mynet, TRAINING_SET);
    augment_pattern_set(mynet, TESTING_SET);
    set_int(mynet, 0, "do_cross_validation", 1);
    (void) training(mynet, 1, 0, 1); /* (net, values?, slopes? clear2ndDer?) */
    
    

    /* ********** GENERALIZATION ERROR **************************** */
    testE = 0.0;
    for (i = 0; i < test_grid_size+1; i++){
      input[0] = ((float) i) / ((float) test_grid_size);
      for (j = 0; j < test_grid_size+1; j++){
	input[1] = ((float) j) / ((float) test_grid_size);
	target_bp[0]    = f(test_function, input[0], input[1], 0.0, 0.0);
	run_network(mynet, 0, input, output_bp, NULL);
	testE += (output_bp[0] - target_bp[0]) 
	  * (output_bp[0] - target_bp[0]);
      }      
    }
    testE /= (float) (test_grid_size+1);
    testE /= (float) (test_grid_size+1);
    printf("-----> generalization error (augment, cross): %g  (1 %d)\n", 
	   testE, number_training_pat);
    /* ************************************************************ */


    srandom(10000);
    init_weights_random(mynet);
    set_int(mynet, 0, "do_cross_validation", 0);
    (void) training(mynet, 1, 0, 1); /* (net, values?, slopes? clear2ndDer?) */
    
    

    /* ********** GENERALIZATION ERROR **************************** */
    testE = 0.0;
    for (i = 0; i < test_grid_size+1; i++){
      input[0] = ((float) i) / ((float) test_grid_size);
      for (j = 0; j < test_grid_size+1; j++){
	input[1] = ((float) j) / ((float) test_grid_size);
	target_bp[0]    = f(test_function, input[0], input[1], 0.0, 0.0);
	run_network(mynet, 0, input, output_bp, NULL);
	testE += (output_bp[0] - target_bp[0]) 
	  * (output_bp[0] - target_bp[0]);
      }      
    }
    testE /= (float) (test_grid_size+1);
    testE /= (float) (test_grid_size+1);
    printf("-----> generalization error (augment, no cross): %g  (1 %d)\n", 
	   testE, number_training_pat);
    /* ************************************************************ */


#endif

  }
  
}



#ifdef MAIN_DEFINED
main(argc, argv)
     int	argc;
     char	**argv;
{
  if (argc == 2 && atoi(argv[1]) > 0 && atoi(argv[1]) <= 3)
    test_code2(atoi(argv[1]));
  else
    printf("Usage: %s <function_nr>\nwith <function_nr> = 1,2, or 3\n",
	   argv[0]);
}
#endif



/*********************************************************************\
|*******************  --> LISP VERSION <-- ***************************|
\*********************************************************************/

#ifdef LISP

/************************************************************************
 *
 *   FILENAME:         ...partially taken from "Extras.c"
 *                 
 *   FUNCTION:         Utility functions for Learn.c, for Theo-EBNN
 *                 
 *   CREATOR:          Tom Mitchell
 *   
 *   DATE:             July 14, 1993
 *   
 ************************************************************************/


/************************************************************************
 *
 *   Following are interface functions to allow Lisp to call C functions
 *   that return floats.  For some reason, the C-Lisp interface cannot
 *   pass floats, but it can pass an array of floats.  So each of the 
 *   following functions accepts an array as an input parameter, and
 *   stores the float result in the first element of the array.
 *                 
 *   Naming convention: if original C fn is called x, then fn is call cl_x
 ************************************************************************/


void cl_add_new_pattern(net_ptr NPTR, int mode, /* For LISP users only */
			float *lrate_target_values, 
			float *lrate_target_derivatives,
			float *input, float *target_bp, float *target_xi,
			int pattern_set)
{
  add_new_pattern(NPTR, mode, input, target_bp, target_xi,
		  lrate_target_values[0], lrate_target_derivatives[0],
		  pattern_set);
}






int cl_set_float(net_ptr NPTR, int global, /* For LISP users only */
		 char *name, float *value)
{
  set_float(NPTR, global, name, value[0]);
}


void cl_get_float (net_ptr NPTR, char *name, int global, float *rslt)
{
  rslt[0] = get_float(NPTR, global, name);
}



void cl_search_in_action_space(net_ptr NPTR, float *input,
			       int norm, float *rslt)
{
  rslt[0] = search_in_action_space(NPTR, input, norm);
}




void cl_f(int i, float x, float y, float z, float w, float *rslt)
{
  rslt[0] = f(i,x,y,z,w);
}

void cl_df_dx(int i, float x, float y, float z, float w, float *rslt)
{
  rslt[0] = df_dx(i,x,y,z,w);
}

void cl_df_dy(int i, float x, float y, float z, float w, float *rslt)
{
  rslt[0] = df_dy(i,x,y,z,w);
}

void cl_df_dz(int i, float x, float y, float z, float w, float *rslt)
{
  rslt[0] = df_dz(i,x,y,z,w);
}

void cl_df_dw(int i, float x, float y, float z, float w, float *rslt)
{
  rslt[0] = df_dw(i,x,y,z,w);
}



/************************************************************************
 *
 *   NAME:         test_code
 *                 
 *   FUNCTION:     Tom's simple test
 *                 
 ************************************************************************/


net_ptr test_code(void)
{  
  int n, i, j, k;
  net_ptr mynet;
  float input[2];
  float target_bp[1];
  float output_bp[1];
  float target_xi[2];
  int number_training_pat;

  printf("Begin test function\n");
  mynet = create_network(2,6,0,1,100,0,1);		

  /* control output on screen during training */
  set_int  (NULL,  1, "print_flag", 1);
  set_int  (NULL,  1, "n_printout_params", 9999);
  set_int  (NULL,  1, "n_printout_E", 100); 	
  set_int  (NULL,  1, "n_printout_patterns", 9999);

  clear_pattern_set(mynet, ALL_SETS);    
  /* bp == values, xi == slopes */
  /* initialize with some random patterns */
  srandom(10);
  input[0]        = 0.5;
  input[1]        = 0.5;
  number_training_pat = 3;
  for (n = 0; n < number_training_pat; n++){
      target_bp[0]    = f(1, input[0], input[1], 0.0, 0.0);
      target_xi[0]    = df_dx(1, input[0], input[1], 0.0, 0.0);
      target_xi[1]    = df_dy(1, input[0], input[1], 0.0, 0.0);
      add_new_pattern(mynet, 1, input, target_bp, target_xi, 1.0, 1.0,
		      TRAINING_SET);
      printf("%g %g -> Value: %g, Slopes:  %g %g\n",input[0],input[1], 
	     target_bp[0],target_xi[0] ,target_xi[1]);
      input[0] = RAND_POS();
      input[1] = RAND_POS();  
    }
  printf("Begin training:\n");
  (void) training(mynet, 1, 1, 1); /* (net, values?, slopes? clear2ndDer?) */
  
  printout_all(mynet);
  return mynet;
      
}

net_ptr tc1(void)
{  
  int n, i, j, k;
  net_ptr mynet;
  float input[2];
  float target_bp[1];
  float output_bp[1];
  float target_xi[2];
  int number_training_pat;

  printf("Begin test function\n");
  mynet = create_network(2,6,0,1,100,0,1);		

  /* control output on screen during training */
  set_int  (NULL,  1, "print_flag", 1);
  set_int  (NULL,  1, "n_printout_params", 9999);
  set_int  (NULL,  1, "n_printout_E", 100); 	
  set_int  (NULL,  1, "n_printout_patterns", 9999);

  clear_pattern_set(mynet, ALL_SETS);    
  /* bp == values, xi == slopes */
  /* initialize with some random patterns */
  srandom(10);
  input[0]        = 0.5;
  input[1]        = 0.5;
  number_training_pat = 3;
  for (n = 0; n < number_training_pat; n++){
      target_bp[0]    = f(1, input[0], input[1], 0.0, 0.0);
      target_xi[0]    = df_dx(1, input[0], input[1], 0.0, 0.0);
      target_xi[1]    = df_dy(1, input[0], input[1], 0.0, 0.0);
      add_new_pattern(mynet, 1, input, target_bp, target_xi, 1.0, 1.0,
		      TRAINING_SET);
      printf("%g %g -> Value: %g, Slopes:  %g %g\n",input[0],input[1], 
	     target_bp[0],target_xi[0] ,target_xi[1]);
      input[0] = RAND_POS();
      input[1] = RAND_POS();  
    }
/*  printf("Begin training:\n");
  (void) training(mynet, 1, 1, 1);
 */  
  printout_all(mynet);
  return mynet;
      
}
net_ptr tc2(void)
{  
  int n, i, j, k;
  net_ptr mynet;
  float input[2];
  float target_bp[1];
  float output_bp[1];
  float target_xi[2];
  int number_training_pat;

  printf("Begin test function\n");
  mynet = create_network(2,6,0,1,100,0,1);		

  /* control output on screen during training */
  set_int  (NULL,  1, "print_flag", 1);
  set_int  (NULL,  1, "n_printout_params", 9999);
  set_int  (NULL,  1, "n_printout_E", 100); 	
  set_int  (NULL,  1, "n_printout_patterns", 9999);

  clear_pattern_set(mynet, ALL_SETS);    
  /* bp == values, xi == slopes */
  /* initialize with some random patterns */
  srandom(10);
  input[0]        = 0.5;
  input[1]        = 0.5;
  number_training_pat = 3;
/*
  for (n = 0; n < number_training_pat; n++){
      target_bp[0]    = f(1, input[0], input[1], 0.0, 0.0);
      target_xi[0]    = df_dx(1, input[0], input[1], 0.0, 0.0);
      target_xi[1]    = df_dy(1, input[0], input[1], 0.0, 0.0);
      add_new_pattern(mynet, 1, input, target_bp, target_xi, 1.0, 1.0,
		      TRAINING_SET);
      printf("%g %g -> Value: %g, Slopes:  %g %g\n",input[0],input[1], 
	     target_bp[0],target_xi[0] ,target_xi[1]);
      input[0] = RAND_POS();
      input[1] = RAND_POS();  
    }
  printf("Begin training:\n");
  (void) training(mynet, 1, 1, 1);
*/  
  printout_all(mynet);
  return mynet;
      
}
net_ptr tc3(void)
{  
  int n, i, j, k;
  net_ptr mynet;
  float input[2];
  float target_bp[1];
  float output_bp[1];
  float target_xi[2];
  int number_training_pat;

  printf("Begin test function\n");
  mynet = create_network(2,6,0,1,100,0,1);		

  /* control output on screen during training 
  set_int  (NULL,  1, "print_flag", 1);
  set_int  (NULL,  1, "n_printout_params", 9999);
  set_int  (NULL,  1, "n_printout_E", 100); 	
  set_int  (NULL,  1, "n_printout_patterns", 9999);
*/
  clear_pattern_set(mynet, ALL_SETS);    
  input[0]        = 0.5;
  input[1]        = 0.5;
  target_bp[0] = 0.5;
  target_xi[0] = 0.5;
  target_xi[1] = 0.5;
  add_new_pattern(mynet, 1, input, target_bp, target_xi, 1.0, 1.0,
		  TRAINING_SET);
  
  input[0]        = 0.25;
  input[1]        = 0.25;
  target_bp[0] = 0.25;
  add_new_pattern(mynet, 1, input, target_bp, target_xi, 1.0, 1.0,
		  TRAINING_SET);
  
  input[0]        = 0.8;
  input[1]        = 0.5;
  target_bp[0] = 0.65;
  add_new_pattern(mynet, 1, input, target_bp, target_xi, 1.0, 1.0,
		  TRAINING_SET);

  (void) training(mynet, 1, 1, 1); 
  
  printout_all(mynet);
  return mynet;
      
}








#endif

