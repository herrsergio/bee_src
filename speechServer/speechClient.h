
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



#ifndef _SPEECH_CLIENT_H
#define _SPEECH_CLIENT_H

#include <raiClient.h>

#ifdef __cplusplus
extern "C" {
#endif

  void speechRegister(void);
  void speechConnect(void);
  void speechDisconnected(void);
  void speechOn(void);
  void speechOff(void);
  void speechSend(char *text);
  void speechBuffer(int type);
  void speechClear(void);
  void speechSpeed(int speed);
  void speechFrequency(int freq);
  void speechIntonation(int toggle);

  /* --------- old function names -------- */


  void registerSpeechClient();
  void findSpeechServer();
  void silenceSpeechServer();
  void sendSpeech(char * newtext);
  void sendSpeechClear();
  void setSpeechBuffer(int type);
  void setSpeechSpeed(int speed);
  void setSpeechFrequency(int freq);
  void setSpeechUseIntonation(int toggle);

#ifdef __cplusplus
}
#endif


#endif
