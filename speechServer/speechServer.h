
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



#include <stdlib.h>
#include <raiServer.h>
#include <bUtils.h>


#define SPEECH_SERVER_NAME "speechTCXServer"
#define SPEECH_FIXED_MESSAGE "speechFxd"
#define SPEECH_VARIABLE_MESSAGE "speechVar"


/***** TCX messages and handlers for the base server ****/


#define SPEECH_MESSAGES  { {SPEECH_FIXED_MESSAGE, raiFixedMsg},     \
			 {SPEECH_VARIABLE_MESSAGE, raiVariableMsg}  }


/***** commands for the speech server ****/

typedef enum {
   SPEECH_TCX_setBufferType,
   SPEECH_TCX_clear,
   SPEECH_TCX_setSpeed,
   SPEECH_TCX_setFrequency,
   SPEECH_TCX_setIntonation,
   SPEECH_TCX_say
}  SpeechTCXMessages;

/***************************************************************************/
/* These determine what the server does when it gets a new request to speak */
/* and it is not done saying the previous request yet. Ignore is the default.*/
/* They are used by setSpeechBuffer */

#define SpeechBufferIgnore 0	   /* ignore the new request */
#define SpeechBufferClobber 1	   /* stop in the middle of old, start new */
#define SpeechBufferSave 2	   /* allow new requests to pile up in buffer */

