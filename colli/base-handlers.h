
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






#ifndef BASE_HANDLERS_INCLUDED
#define BASE_HANDLERS_INCLUDED


void init_tcx(void);

void send_automatic_status_update(void);
void send_automatic_action_executed_message(void);
void send_automatic_sonar_update(void);
void send_automatic_laser_update(void);
void send_automatic_ir_update(void);

extern BASE_update_status_reply_type base_tcx_status;
extern SONAR_sonar_reply_type        sonar_tcx_status;
extern LASER_laser_reply_type        laser_tcx_status;
extern IR_ir_reply_type              ir_tcx_status;
extern COLLI_colli_reply_type        colli_tcx_status;

extern BOOLEAN use_simulator;
extern BOOLEAN use_rwi_server;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern struct bParamList * bParamList;
extern const char *bRobotType;



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/******* auto-reply stuff, data definitions ***************/


#define MAX_N_AUTO_UPDATE_MODULES 100



extern int n_auto_update_modules; /* number of processes to whom
				   * position should be forwarded
				   * automatically upon change */

extern int n_auto_status_update_modules;
extern int n_auto_sonar_update_modules;
extern int n_auto_laser_update_modules;
extern int n_auto_ir_update_modules;
extern int n_auto_colli_update_modules;


typedef struct{
  TCX_MODULE_PTR module;	/* pointer to TCX module */
  int            status;	/* 1=subscribed to regular status updates */
  int            sonar;		/* 1=subscribed to regular sonar updates */
  int            laser;		/* 1=subscribed to regular laser updates */
  int            ir;		/* 1=subscribed to regular ir  updates */
  int            colli;		/* 1=subscribed to regular colli updates */
} auto_update_type;



extern auto_update_type auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];
                                /* collection of TCX-module ptrs */

extern TCX_REF_PTR TCX_sender_of_action;


void connect_to_SOUND(void);

void SOUND_talk_text(char *text);
void SOUND_play_message( int messageNumber);

#endif
