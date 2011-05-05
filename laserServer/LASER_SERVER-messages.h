
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
#ifndef LASER_SERVER_messages_defined
#define LASER_SERVER_messages_defined



#define TCX_LASER_SERVER_MODULE_NAME "laserServer"
#ifdef TCX_define_variables   /* do this exactly once! */
TCX_MODULE_PTR LASER_SERVER;	      /* needs to be allocate in a user program */
#else
extern TCX_MODULE_PTR LASER_SERVER;  /* otherwise: reference */
#endif

/* -------------------------------------------------- */
/* Messages                                           */
/* -------------------------------------------------- */

typedef struct {
  int sweep0;	/* laser0 - 0=don't send, n>0 send every n-th frame */
  int sweep1;   /* laser1 - 0=don't send, n>0 send every n-th frame */
} LASER_SERVER_register_auto_update_type, *LASER_SERVER_register_auto_update_ptr;

#define LASER_SERVER_register_auto_update_format "{ int, int }"

/* ----------------------------------------------------------------------- */

typedef struct {
  int numLaser;      /* query which laser, num = 0 | 1 */
} LASER_SERVER_sweep_query_type, *LASER_SERVER_sweep_query_ptr;

#define LASER_SERVER_sweep_query_format "{ int }"

typedef struct {
  int numberLasers; /* will be set to NUMBER_LASERS in laserHandlers.c */
  int numLaser;     /* which laser */
  float *value;
} LASER_SERVER_sweep_reply_type, *LASER_SERVER_sweep_reply_ptr;

#define LASER_SERVER_sweep_reply_format "{ int, int, <float : 1> }"

/* ----------------------------------------------------------------------- */

#ifdef TCX_define_variables		/* do this exactly once! */
#define LASER_SERVER_messages \
{"LASER_SERVER_register_auto_update", LASER_SERVER_register_auto_update_format},\
{"LASER_SERVER_sweep_reply",          LASER_SERVER_sweep_reply_format},\
{"LASER_SERVER_sweep_query",          LASER_SERVER_sweep_query_format}
#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_REPLY_HANDLERS

/******* (a) Procedure headers ******/

void LASER_SERVER_sweep_reply_handler( TCX_REF_PTR            ref,
				 LASER_SERVER_sweep_reply_ptr sweep);

/******* (b) Handler array ******/

TCX_REG_HND_TYPE LASER_SERVER_reply_handler_array[] = {

  {"LASER_SERVER_sweep_reply", "LASER_SERVER_sweep_reply_handler",
     LASER_SERVER_sweep_reply_handler, TCX_RECV_ALL, NULL}

};

#endif /* define reply handlers */

#endif /* messages defined */








