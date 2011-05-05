
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
 * Copyright 1994, Brown University, Providence, RI
 *
 * Permission to use and modify this software and its documentation for
 * any purpose other than its incorporation into a commercial product is
 * hereby granted without fee.  Permission to copy and distribute this
 * software and its documentation only for non-commercial use is also
 * granted without fee, provided, however, that the above copyright notice
 * appear in all copies, that both that copyright notice and this permission
 * notice appear in supporting documentation, that the name of Brown
 * University not be used in advertising or publicity pertaining to
 * distribution of the software without specific, written prior permission,
 * and that the person doing the distribution notify Brown University of
 * such distributions outside of his or her organization. Brown University
 * makes no representations about the suitability of this software for
 * any purpose.  It is provided "as is" without express or implied warranty.
 * Brown University requests notification of any modifications to this
 * software or its documentation.
 *
 * Send the following redistribution information:
 *
 *	Name:
 *	Organization:
 *	Address (postal and/or electronic):
 *
 * To:
 *	Software Librarian
 *	Computer Science Department, Box 1910
 *	Brown University
 *	Providence, RI 02912
 *
 *		or
 *
 *	brusd@cs.brown.edu
 *
 * We will acknowledge all electronic notifications.
 */

#ifndef lint
static char rcsid[] =
"$Id: speechServer.c,v 1.10 1997/03/26 23:49:05 thrun Exp $";
static void *const use_rcsid = ((void)&use_rcsid, (void)&rcsid, 0);
#endif

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <speech.h>
#include "speechServer.h"
#include <lockNames.h>
#include <utils.h>
#include <bUtils.h>

#include "beeSoftVersion.h"
#include "libbUtils.h"
#include "libutils.h"
#include "libspeech.h"


extern void tcxRegisterCloseHnd(void (*closeHnd)());

int BUFFER_TYPE = SpeechBufferIgnore;
int VERBOSE = FALSE;
static int ttsType;

#define SPEECH_HANDLERS  {   \
  {SPEECH_FIXED_MESSAGE,"speechFxd",handleSpeechFixed, TCX_RECV_ALL,NULL}, \
  {SPEECH_VARIABLE_MESSAGE,"speechVar",handleSpeechVariable, TCX_RECV_ALL,NULL}}


/*------------------------------------------------------------*/

void voiceClose(char *moduleName, TCX_MODULE_PTR module)
{
  SPEECH_clear();

  if (!strcmp(moduleName, "TCX Server")){ /* TCX shut down */
    fprintf(stderr, "TCX Server died, and so will I.\n");
    exit(0);
  }
}

/*------------------------------------------------------------*/

void
speak(char *words)
{
  /* This function may not be used */

  switch (BUFFER_TYPE)
    {
    case SpeechBufferIgnore	:	/* this won't work with LT yet */
      if (! SPEECH_is_speaking())
	SPEECH_talk_text(words);
      break;

    case SpeechBufferClobber  :
      /*      if (SPEECH_is_speaking()) */
      SPEECH_clear();
      SPEECH_talk_text(words);
      break;
      
    case SpeechBufferSave:
    default :		
      SPEECH_talk_text(words);
    }
}

/*------------------------------------------------------------*/

void
dispatchFixedMessage(int operation, unsigned long parameter)
{
  
  switch (operation)
    {
    case SPEECH_TCX_setBufferType:  
      BUFFER_TYPE = parameter;
      break;

    case SPEECH_TCX_clear	:
      SPEECH_clear();
      break;

    case SPEECH_TCX_setSpeed	:
      SPEECH_set_speed(parameter);
      break;

    case SPEECH_TCX_setFrequency	:
      SPEECH_set_frequency(parameter);
      break;
    }
}

/*------------------------------------------------------------*/

void
handleSpeechFixed(TCX_REF_PTR message,RAI_FixedMsgType *msg_ptr)
{
  int operation;
  unsigned long param;

  operation = msg_ptr->operation;
  param = msg_ptr->parameter;
  if (VERBOSE)
    fprintf(stderr,"speechServer fixed msg %d %lu\n",operation,param); 

  if (ttsType) {
    dispatchFixedMessage(operation,param);
  }

  tcxFreeByRef(message,msg_ptr);
}

/*------------------------------------------------------------*/

void
dispatchVariableMessage(int operation, int bufsize, unsigned char* buffer)
{
  switch (operation)
    {
    case SPEECH_TCX_say:
      if (VERBOSE)
	fprintf(stderr,"About to say %s\n",buffer);
      SPEECH_talk_text(buffer);
      break;
      
    default:  
      break;
    }
}

/*------------------------------------------------------------*/

void
handleSpeechVariable(TCX_REF_PTR message,RAI_VariableMsgType *msg_ptr)
{

  int operation;
  int bufsize;
  unsigned char* buffer;
  operation = msg_ptr->operation;
  bufsize = msg_ptr->bufsize;
  buffer = msg_ptr->buffer;
  if (VERBOSE)
    fprintf(stderr,"speechServer received var msg %d %d\n",operation,bufsize); 

  if (ttsType) {
    dispatchVariableMessage(operation,bufsize,buffer);
  }

  tcxFreeByRef(message,msg_ptr);
}

/*------------------------------------------------------------*/

void initVoice()
{

  TCX_REG_MSG_TYPE messageArray[] = SPEECH_MESSAGES;
  TCX_REG_HND_TYPE handlerArray[] = SPEECH_HANDLERS;
  char* tcxMachine; 
  int numberOfHandlers;
  int numberOfMessages;  

  numberOfHandlers = sizeof(handlerArray)/sizeof(TCX_REG_HND_TYPE); 
  numberOfMessages = sizeof(messageArray)/sizeof(TCX_REG_MSG_TYPE); 
  tcxMachine = getenv("TCXHOST");   
  
  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize(SPEECH_SERVER_NAME,tcxMachine);
  check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
		       BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
		       NULL, 0);
  check_version_number(libspeech_major, libspeech_minor,
		       libspeech_robot_type, libspeech_date,
		       "libspeech", 0);
  check_version_number(libutils_major, libutils_minor,
		       libutils_robot_type, libutils_date,
		       "libutils", 0);
  check_version_number(libbUtils_major, libbUtils_minor,
		       libbUtils_robot_type, libbUtils_date,
		       "libbUtils", 1);


  tcxRegisterMessages(messageArray,numberOfMessages);
  tcxRegisterHandlers(handlerArray,numberOfHandlers);
  fprintf(stderr,"Speech Registered\n");
  tcxRegisterCloseHnd(voiceClose);

}

/*------------------------------------------------------------*/

void interrupt_handler(int sig)
{
  if (ttsType) {
    SPEECH_clear();  /* make sure we do not leave bogus words in buffer */
    SPEECH_terminate();
  }

  exit(0);
}

/*------------------------------------------------------------*/

int main(int argc,char ** argv) 
{
  struct bParamList * paramList = NULL;

  /*
   * Set some parameters
   */

  /* add some defaults */
  paramList = bParametersAddEntry(paramList, "", "TCXHOST", "localhost");
  paramList = bParametersAddEntry(paramList, "", "fork", "yes");

  /* add some parameter files */
  paramList = bParametersAddFile(paramList, "etc/beeSoft.ini");

  /* add some enviroment variables */
  paramList = bParametersAddEnv(paramList, "", "TCXHOST");

  /* add command line arguements */
  paramList = bParametersAddArray(paramList, "", argc, argv);

  /* here is where we should add non "parameter format" options */


  bParametersFillParams(paramList);

  if (!strcmp("none", bParametersGetParam(paramList, "robot", "speech"))) {
    fprintf(stderr,
	    "Speech name set to 'none'.  Check the robot.speech\n"
	    "value in beeSoft.ini.\n");
    exit(0);
  }
  
  if (!strcmp(bRobot.speech_type, "doubleTalkPC")) {
    ttsType = 1;
  }
  else if (!strcmp(bRobot.speech_type, "doubleTalkLT")) {
    ttsType = 2;
  }
  else {
    ttsType = 0;
  }
 
  if (ttsType) {
    if (makeLock(SPEECH_SERVER_LOCK)<0) {
      fprintf(stderr,"%s:  Already running speech server\n",argv[0]);
      exit(-1);
    }
  }
  else {
    fprintf(stderr,"Starting speech server in printing mode\n");
  }

#if 0
  if (argc > 1) {
    VERBOSE = TRUE;
    fprintf(stderr,"Starting speech server in verbose mode\n");
  }
#endif
  initVoice();

  if (bRobot.fork) {
    bDaemonize("speechServer.log");
  }

  if (ttsType) {
    SPEECH_init(ttsType, bRobot.speech_dev, bRobot.speech_bps);
    SPEECH_clear();
    SPEECH_set_pitch(33);
    SPEECH_set_speed(1);
    SPEECH_set_frequency(70);
    SPEECH_set_timeout_delay(10);
    SPEECH_set_text_delay(3);
    SPEECH_set_volume(9);  /* should be set to 11 :) */ 
    SPEECH_set_tone(TREBLE);
  }

  tcxRecvLoop(NULL);     /* loop forever, calling callbacks when we get msgs */
  exit(0);
}
