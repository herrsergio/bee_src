
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








extern float *dummy_sensors;
extern float *null_sensors;
extern char *ModuleNameGlobal;



/************************************************************************
 *
 *   NAME:         check_commandline_parameters
 *                 
 *   FUNCTION:     browses the command-line parameters for useful info.
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void check_commandline_parameters(int argc, char **argv, ALL_PARAMS);


/************************************************************************
 *
 *   NAME:         init_program
 *                 
 *   FUNCTION:     Initializes the three structures "robot_state",
 *                 "action_ptr", and "program_state"
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 NOTE: "NULL" disables initialization
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void init_program(ALL_PARAMS);





/************************************************************************
 *
 *   NAME:         allocate_memory
 *                 
 *   FUNCTION:     Allocales all memory
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void allocate_memory(ALL_PARAMS);



/************************************************************************
 *
 *   NAME:         free_memory
 *                 
 *   FUNCTION:     Counterpart to "init_program()"
 *                 frees memory
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 NOTE: "NULL" disables free-ing
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void free_memory(ALL_PARAMS);





/************************************************************************
 *
 *   NAME:         clear_maps
 *                 
 *   FUNCTION:     Clears the maps
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void clear_maps(ALL_PARAMS);



/************************************************************************
 *
 *   NAME:         connect_to_tca
 *                 
 *   FUNCTION:     connects and registers messages
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void connect_to_tca(ALL_PARAMS);




/************************************************************************
 *
 *   NAME:         interrupt_handler()
 *                 
 *   FUNCTION:     some ^C signal or kill command arrived
 *                 
 *   PARAMETERS:   int sig           signal
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void interrupt_handler(int sig);
