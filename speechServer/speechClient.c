
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
 * FILE:  speechClient.c
 * DESC:  client side of the speech server API
 *
 * $Id: speechClient.c,v 1.4 1997/06/26 23:08:17 tyson Exp $
 *
 * Modification history: $Log: speechClient.c,v $
 * Modification history: Revision 1.4  1997/06/26 23:08:17  tyson
 * Modification history: I dont remember what
 * Modification history:
 * Modification history: Revision 1.3  1997/03/25 21:44:49  tyson
 * Modification history: Many bug fixes.
 * Modification history:
 * Modification history: Revision 1.2  1997/03/11 17:16:46  tyson
 * Modification history: added IR simulation and other work
 * Modification history:
 * Modification history: Revision 1.1.1.1  1996/09/22 16:46:22  rhino
 * Modification history: General reorganization of the directories/repository, fusion with the
 * Modification history: RWI software.
 * Modification history:
 * Modification history: Revision 1.1.1.1  1996/03/19 16:33:46  tyson
 * Modification history: Inital 1.2 repository
 * Modification history:
 * Revision 1.5  1995/10/02  20:48:03  jmk
 * *** empty log message ***
 *
 * Revision 1.4  1995/10/02  20:47:33  jmk
 * Fixed bug in text mode (ie no real speech server present).  In order to
 * have this transparent (ie same client program runs with or without
 * speechServer) the client needs to register its TCX messages even though
 * they will never be used.  Otherwise calls to TCX will bag.
 *
 * Revision 1.3  1995/08/18  18:20:24  jal
 * Changed debugging output to go to stderr.
 *
 * Revision 1.2  1995/07/31  19:09:32  jmk
 * Fiddled with speed and interword delay parameters to get speech much
 * clearer.
 *
 *
 *
 * Revision 1.0  1994/11/  23:35:59  jmk
 * Created file
 *
 */
 
#ifndef lint
static char rcsid[] =
"$Id: speechClient.c,v 1.4 1997/06/26 23:08:17 tyson Exp $";
static void *const use_rcsid = ((void)&use_rcsid, (void)&rcsid, 0);
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <bUtils.h>
#include <tcx.h>

#include <speechServer.h>
#include <speechClient.h>

extern void tcxRegisterCloseHnd(void (*closeHnd)());

TCX_MODULE_PTR  speechServer = NULL;

static int SILENT = 0;
static int speechHaveLock = 1;

/*------------------------------------------------------------*/

void
speechRegister()
{
  /* OK to register these even if we do not use them */
  int numberOfMessages;
  TCX_REG_MSG_TYPE messages[] = SPEECH_MESSAGES; 
  
  if (bRobot.speech_type &&
      (!strcmp(bRobot.speech_type, "doubleTalkPC") ||
       !strcmp(bRobot.speech_type, "doubleTalkLT"))) {
    SILENT = FALSE;

    numberOfMessages = sizeof(messages)/sizeof(TCX_REG_MSG_TYPE); 
    registerInterface(SPEECH_SERVER_NAME, numberOfMessages,messages, 0, NULL);
  }
  else {
    SILENT = TRUE;
  }
}

/*------------------------------------------------------------*/

void
speechConnect()
{
  static time_t lastCheck = 0;
  static time_t thisCheck = 0;

#ifdef TCX_debug
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);
#endif

  thisCheck = time(NULL);

  if (thisCheck>lastCheck) {
    lastCheck = thisCheck;

    if (!speechServer) {
      speechServer = tcxConnectOptional(SPEECH_SERVER_NAME);
    }
  }
  return;
}

/*------------------------------------------------------------*/

void
speechDisconnected(void)
{
#ifdef TCX_debug
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);
#endif

  speechServer = NULL;
}

/********************************************************************
 *
 *  VOICE CLIENT SUPPORT 
 *
 ********************************************************************/

void
speechOn()
{
  SILENT = 0;
}

/*------------------------------------------------------------*/

void
speechOff()
{
  SILENT = 1;
}

/*------------------------------------------------------------*/

void
speechSend(char * newtext)
{
  RAI_VariableMsgType command;

  if (!speechServer) {
    speechConnect();
  }

  if (!SILENT && speechServer && speechHaveLock) {
    command.operation = SPEECH_TCX_say;
    command.bufsize = strlen(newtext)+1; /* dont forget the null at the end*/
    command.buffer = newtext;
    tcxSendMsg(speechServer, SPEECH_VARIABLE_MESSAGE, &command);
  }
  else {
    printf("speechServer says: %s\n",newtext);
  }
  return;
}

/*------------------------------------------------------------*/

void
speechBuffer(int type)
{
  RAI_FixedMsgType command;
  
  if (SILENT) {
    return;
  }
  
  if (!speechServer) {
    speechConnect();
  }

  if (speechServer && speechHaveLock) {
    command.operation = SPEECH_TCX_setBufferType;
    command.parameter = type;
    tcxSendMsg(speechServer, SPEECH_FIXED_MESSAGE, &command);
  }

  return;
}

/*------------------------------------------------------------*/

void
speechClear()
{
  RAI_FixedMsgType command;

  if (SILENT) {
    return;
  }
  
  if (!speechServer) {
    speechConnect();
  }

  if (speechServer && speechHaveLock) {
    command.operation = SPEECH_TCX_clear;
    command.parameter = 0;
    tcxSendMsg(speechServer, SPEECH_FIXED_MESSAGE, &command);
  }

  return;
}

/*------------------------------------------------------------*/

void
speechSpeed(int speed)
{
  RAI_FixedMsgType command;

  if (SILENT) {
    return;
  }
  
  if (!speechServer) {
    speechConnect();
  }

  if (speechServer && speechHaveLock) {
    command.operation = SPEECH_TCX_setFrequency;
    command.parameter = speed;
    tcxSendMsg(speechServer, SPEECH_FIXED_MESSAGE, &command);
  }

  return;
}

/*------------------------------------------------------------*/

void
speechFrequency(int freq)
{
  RAI_FixedMsgType command;

  if (SILENT) {
    return;
  }
  
  if (!speechServer) {
    speechConnect();
  }

  if (speechServer && speechHaveLock) {
    command.operation = SPEECH_TCX_setFrequency;
    command.parameter = freq;
    tcxSendMsg(speechServer, SPEECH_FIXED_MESSAGE, &command);
  }

  return;
}

/*------------------------------------------------------------*/

void
speechIntonation(int toggle)
{
  RAI_FixedMsgType command;

  if (SILENT) {
    return;
  }
  
  if (!speechServer) {
    speechConnect();
  }

  if (speechServer && speechHaveLock) {
    command.operation = SPEECH_TCX_setIntonation;
    command.parameter = toggle;
    tcxSendMsg(speechServer, SPEECH_FIXED_MESSAGE, &command);
  }

  return;
}

/*------------------------------------------------------------*/

void silenceSpeechServer()              { speechOff();              }
void findSpeechServer()                 { speechConnect();          }
void registerSpeechClient()             { speechRegister();         }

void sendSpeech(char * newtext)         { speechSend(newtext);      }
void setSpeechBuffer(int type)          { speechBuffer(type);       }
void sendSpeechClear()                  { speechClear();            }
void setSpeechSpeed(int speed)          { speechSpeed(speed);       }
void setSpeechFrequency(int freq)       { speechFrequency(freq);    }
void setSpeechUseIntonation(int toggle) { speechIntonation(toggle); }
