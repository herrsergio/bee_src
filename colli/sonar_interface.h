
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








/*****************************************************************************
 * PROJECT: rhino
 *****************************************************************************/

#ifndef SONAR_INTERFACE_LOADED
#define SONAR_INTERFACE_LOADED

#include "devUtils.h"
#include "handlers.h"


#define SONAR_POLLSECONDS 1
#define CM_PER_SECOND 33000.0
#define SECONDS_PER_CYCLE 3.2552e-6



/************************************************************************
 *  Sonar device type.
 ************************************************************************/

typedef struct {
  DEV_TYPE dev;
  /* put in here whatever is needed for starting up the real device.*/
}  SONAR_TYPE, *SONAR_PTR;


/************************************************************************
 *  Simple sonar commands
 *  NOTE : SONAR_init must be called prior to any other SONAR command.
 ************************************************************************/

int SONAR_init();
void SONAR_Debug(BOOLEAN flag, char *filename);
void SONAR_outputHnd(int fd, long chars_available);
void SONAR_terminate(void);
BOOLEAN start_sonar();
void SONAR_missed(Handler handler, Pointer client_data);
void stop_sonar(void);
void SONAR_LoopStart(unsigned long *mask);
void SONAR_ChangeMask(unsigned long *mask);
void SONAR_LoopEnd(void);
void SONAR_SetLoopIntervall(double i);
void SONAR_Read(int rt_no);
/* Checks wether the sonar returned some information */
void SONAR_look_for_sonar_device(void);



/************************************************************************
 *  vars for sonar loop
 ************************************************************************/

#define MAX_MASKS 10
#define LOOP_RECOVER_TIME 2
#define LOOP_RECOVER_CNT 3


extern float sonar_start_rot;
extern float sonar_start_pos_x;
extern float sonar_start_pos_y;


extern float sonar_readings[];

extern float ir_readings[3][24];

extern unsigned long *sonar_act_mask;
extern unsigned long *sonar_mask_array[];
extern HandlerList sonar_handlers;
/*****************************************************************************
 * EVENTS
 *
 *   To install an event handler, call the function SONAR_InstallHandler
 *
 *   void HandlerEvent1(<type> callback_data, Pointer client_data)
 *   {
 *   ...
 *   }
 *
 *   SONAR_InstallHandler(HandlerEvent1, EVENT1, client_data)
 *
 *****************************************************************************/

void SONAR_InstallHandler(Handler handler, int event, Pointer client_data);

/*
 *      EVENT                       Type of callback data 
 *      *****                       *********************                   
 */

#define SONAR_REPORT       0        /* sonar_readings of one sonar loop complete */
#define SONAR_RT_COMPLETE  1        /* sonar_readings of one RT command complete */
#define SONAR_MISSED       2        /* void *data */

#define SONAR_NUMBER_EVENTS       2

#ifdef DECLARE_SONAR_VARS

SONAR_TYPE   sonar_device =
{
  { FALSE, 
      { "", DEFAULT_PORT},
      { "/dev/ttyS1", 14},
      SONAR_DEV_NAME,
      -1,
      TRUE,
      FALSE,
      (FILE *) NULL,
      (fd_set *) NULL,
      (DEVICE_OUTPUT_HND) SONAR_outputHnd,
      (void (*)(void)) NULL,  
      (DEVICE_SET_TIMEOUT)  setTimeout,
      (DEVICE_CANCEL_TIMEOUT) cancelTimeout,
      (void (*)(void)) NULL,  
      {0, 0},
      {LONG_MAX, 0},
      {LONG_MAX, 0},
      (void (*)(void)) NULL,
      FALSE
      }
};

#else

extern SONAR_TYPE   sonar_device;

#endif

#endif



void SIMULATOR_message_to_sonar_handler(TCX_REF_PTR   ref,
				       char        **message);

