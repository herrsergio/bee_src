
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
 *   FILENAME:         Net.h
 *                 
 *   CREATOR:          Sebastian Thrun
 *   
 *   DATE:             May 11th, 1992 - Oct 23th, 1993
 *   
 *   GENERAL COMMENTS: Have Fun
 *
 ************************************************************************/


#define pi2 6.28318530717958647692


#define TRAINING_SET   0
#define TESTING_SET    1
#define ALL_SETS       2
#define N_PATTERN_SETS 2

#define default_lrate                      0.1
#define default_fact_bp_to_xi              1.0
#define default_decay                      0.0
#define default_momentum                   0.9
#define default_max_steps                  100
#define default_max_E_bp                   0.0
#define default_max_E_xi                   0.0
#define default_batch_training             1
#define default_variable_momentum          0
#define default_search_grid_size           8
#define default_search_max_nsteps          40
#define default_search_min_change          0.0
#define default_search_lrate               0.5
#define default_augment_factor             0.2
#define default_weights_init_range         0.01
#define default_random_generator           1
#define default_mathematica_gridsize       14
#define default_active_pattern_set         TRAINING_SET
#define default_do_cross_validation        1
#define default_cross_validation_frequency 10
#define default_cross_validation_training_error_weight 0.0

/*-------------------------------------------*/

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define LOGISTIC        0
#define COSINE          1
#define COS_FADE        2
#define LOGISTIC_INT    3
#define GAUSSIAN        4
#define LINEAR          5
#define LAST_KNOWN_TYPE LINEAR

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

typedef struct{
  float lrate_target_values;
  float lrate_target_derivatives;
  float input[max_ninputs];
  float target_bp[max_noutputs];
} nn_pattern_type, *nn_pattern_ptr;

/*-------------------------------------------*/

typedef struct{			/* separation necessary, since we sometimes */
  float target_xi[max_noutputs][max_ninputs]; /* won't use it! */
} nn_pattern_xi_type, *nn_pattern_xi_ptr;

/*-------------------------------------------*/

typedef struct{			/* separation necessary, since we sometimes */
  float target_xi[max_ninputs]; /* won't use it! */
} nn_pattern_xi1_type, *nn_pattern_xi1_ptr;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

typedef struct{

  /*...............this.part.defines.individual.network...........*/
  
  int ninputs;
  int nhidden1;
  int nhidden2;
  int noutputs;

  float lrate;
  float fact_bp_to_xi;
  float decay;
  float momentum;
  float weights_init_range;

  int max_steps;
  float max_E_bp, max_E_xi;
  int batch_training;
  int variable_momentum;

  int search_grid_size;
  int search_max_nsteps;
  float search_min_change;
  float search_lrate;

  float augment_factor;
  int target_xi_defined;      /* without target_xi, there is no TangentProp! */

  /*...............end.network.definition...........*/
  
  int first_hidden1, first_hidden2, first_output, nunits;
  int first_search_input, first_search_output;
  int unit_type[max_nunits];
  float gcor, css;
  float xi[max_nunits][max_nunits];
  float x[max_nunits], net[max_nunits];
  float w[max_nunits][max_nunits], bias[max_nunits];
  float w_lower_bound[max_nunits][max_nunits], bias_lower_bound[max_nunits];
  float w_upper_bound[max_nunits][max_nunits], bias_upper_bound[max_nunits];
  float epsil[max_nunits][max_nunits];
  float delta[max_nunits];
  float prev_dw[max_nunits][max_nunits], prev_dbias[max_nunits];
  float dw_bp[max_nunits][max_nunits], dbias_bp[max_nunits];
  float ddw[max_nunits][max_nunits], ddbias[max_nunits];
  float output_bp[max_nunits];
  float dw_xi[max_nunits][max_nunits], dbias_xi[max_nunits];
  float output_xi[max_nunits][max_nunits];
  float upper_bound[max_ninputs], lower_bound[max_ninputs];
  int first_unit_to[max_nunits], num_units_to[max_nunits];
  int first_unit_from[max_nunits], num_units_from[max_nunits];
  float E_fact_bp, E_fact_xi, lrate_fact_bp, lrate_fact_xi;
  float lrate_bp_w, lrate_bp_bias, lrate_xi_w, lrate_xi_bias;
  float E_xi, E_bp;
  nn_pattern_ptr patterns[2];  
  nn_pattern_xi_ptr patterns_xi[2];
  nn_pattern_xi1_ptr patterns_xi1[2]; /* special case: only one output unit */
  int npatterns[2], nn_pattern_num[2], max_npatterns[2];
  int actual_pattern[2];
  int active_pattern_set;
  int steps, mathematica_gridsize;
  int do_cross_validation;
  int cross_validation_frequency;
  float cross_validation_training_error_weight;

  float w1[max_nunits][max_nunits], bias1[max_nunits];
  float w2[max_nunits][max_nunits], bias2[max_nunits];

  /* Added by Masuoka */
  float sigma_single_prime[max_nunits]; /* originally NPTR->x_tilde[] */
  float sigma_double_prime[max_nunits];
  float delta_bp[max_nunits];
  float epsilon_bar_x_tilde[max_nunits];
  float epsilon_value_tilde[max_nunits];
  float epsilon_value[max_nunits];
  float epsilon_tilde[max_nunits];
  float epsilon_bar[max_nunits];
  float xi_tmp[max_nunits][max_nunits];
  float dbias_xi_tmp[max_nunits];
  float individual_lrate_target_bp[max_noutputs];
  float individual_lrate_input_bp[max_ninputs];

} net_type, *net_ptr;

/************************************************************************
 *
 *   Additional Parameters Added by Masuoka
 *        (and later removed by Thrun)
 *                 
 ************************************************************************/

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

/*********************************************************************\
|*****************                         ***************************|
|*****************   INTERNAL PROCEDURES   ***************************|
|*****************                         ***************************|
\*********************************************************************/



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
		       int target_xi_defined);



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

int delete_network(net_ptr NPTR);



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

void init_weights_random(net_ptr NPTR);



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

void modify_unit_type(net_ptr NPTR, int from, int num, int new_type);



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

void push_network(net_ptr NPTR, int n);
  



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

void pop_network(net_ptr NPTR, int n);
  




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

int read_weights(net_ptr NPTR, char *filename);



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

void save_weights(net_ptr NPTR, char *filename);



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
		  int pattern_set);



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
		   int pattern_set);



/*********************************************************************\
|*****************                         ***************************|
|*****************   INTERNAL PROCEDURES   ***************************|
|*****************                         ***************************|
\*********************************************************************/

float  logistic(float x);

void init_fast_logistic();

float fast_logistic(float x);

float gaussian(float x);
  
float cos_fade(float x, int derivative);

float slow_logistic_integral(float x, int derivative);

float sigma(net_ptr NPTR, int i);

float sigma_prime(net_ptr NPTR, int i);

float sigma_prime_prime(net_ptr NPTR, int i);

void clamp_pattern_input(net_ptr NPTR);

void clamp_input(net_ptr NPTR, float *input);

void compute_net_x(net_ptr NPTR);

void compute_output(net_ptr NPTR);

void compute_delta_epsil(net_ptr NPTR);

void clear_errors(net_ptr NPTR);

void clear_gcor_stuff(net_ptr NPTR);

void clear_dw_dbias(net_ptr NPTR);

void clear_ddw_ddbias(net_ptr NPTR);

void change_w_bias(net_ptr NPTR);

void printout_all(net_ptr NPTR);

void initialize_xi_epsilon(net_ptr NPTR);

void compute_xi(net_ptr NPTR);

void compute_dw_dbias_bp(net_ptr NPTR);

  
void compute_dw_dbias_xi(net_ptr NPTR);

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
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void clear_pattern_set(net_ptr NPTR, int pattern_set);



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
		     int pattern_set);
     

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

void generate_random_testing_set(net_ptr NPTR, int pattern_set, float ratio);


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void one_step_training(net_ptr NPTR);



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

int training(net_ptr NPTR, int mode_bp, int mode_xi, int clear2nd);



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
		 float *input, float *output_bp, float *output_xi);



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

int augment_pattern_set(net_ptr NPTR, int pattern_set);



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

int n_of_outputs(net_ptr NPTR);



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

int n_of_inputs(net_ptr NPTR);



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

void init_search(net_ptr NPTR, int first_search_input, int first_search_output);



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

float evaluate_output_deviation(net_ptr NPTR, int norm);
  

int change_input(net_ptr NPTR, int norm);

float search_in_action_space(net_ptr NPTR, float *input, int norm);



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
 *                 char* filename         Name of the mathematica file
 *                 char *title_string     Name of the chart
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void two_dim_mathematica_plot(net_ptr NPTR, 
			      float *input,
			      int input_1, 
			      int input_2,
			      char* filename,
			      char *title_string);


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

int set_float(net_ptr NPTR, int global, char *name, float value);



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

int set_int(net_ptr NPTR, int global, char *name, int value);



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

float get_float(net_ptr NPTR, int global, char *name);



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

int get_int(net_ptr NPTR, int global, char *name);




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

float uniform_distribution();



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

float f(int i, float x, float y, float z, float w);

float df_dx(int i, float x, float y, float z, float w);

float df_dy(int i, float x, float y, float z, float w);

float df_dz(int i, float x, float y, float z, float w);

float df_dw(int i, float x, float y, float z, float w);

void test_code2(int test_function);


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
			int pattern_set);



int cl_set_float(net_ptr NPTR, int global, /* For LISP users only */
		 char *name, float *value);



void cl_get_float (net_ptr NPTR, char *name, int global, float *rslt);




void cl_search_in_action_space(net_ptr NPTR, float *input,
			       int norm, float *rslt);





void cl_f(int i, float x, float y, float z, float w, float *rslt);


void cl_df_dx(int i, float x, float y, float z, float w, float *rslt);


void cl_df_dy(int i, float x, float y, float z, float w, float *rslt);


void cl_df_dz(int i, float x, float y, float z, float w, float *rslt);


void cl_df_dw(int i, float x, float y, float z, float w, float *rslt);




/************************************************************************
 *
 *   NAME:         test_code
 *                 
 *   FUNCTION:     Tom's simple test
 *                 
 ************************************************************************/


net_ptr test_code(void);


net_ptr tc1(void);

net_ptr tc2(void);

net_ptr tc3(void);









#endif

