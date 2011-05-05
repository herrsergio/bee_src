
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


#define DEV_PORT

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>

#define DECLARE_SPEECH_VAR

#include <speech.h>
#include <utils.h>   /* for localto information */

#define _SPEECH_TEXT_MODE    0
#define _SPEECH_CHAR_MODE    1
#define _SPEECH_PHONEME_MODE 2
#define _SPEECH_TONE_MODE    3
#define _SPEECH_PCM_MODE     4


/*****************************************************************************
 * Global variables 
 *****************************************************************************/

int ttsFd;   /* fd for /dev/port or serial port */
int ttsType; /* PC on /dev/port, 2 = LT */

/************************************************************************/
/* Global internal variables hiding the details of hardware from user   */
/************************************************************************/
static int   _speech_current_mode          = _SPEECH_TEXT_MODE;
static int   _speech_current_text_delay    = DEFAULT_TEXT_DELAY;
static int   _speech_current_char_delay    = DEFAULT_CHAR_DELAY;
static int   _speech_current_timeout_delay = DEFAULT_TIMEOUT_DELAY;
static int   _speech_current_frequency     = DEFAULT_FREQUENCY;
static int   _speech_current_speed         = DEFAULT_SPEED;
static int   _speech_current_pitch         = DEFAULT_PITCH;
static int   _speech_current_volume        = DEFAULT_VOLUME;
static int   _speech_current_tone          = DEFAULT_TONE;
static int   _speech_current_punc_level    = DEFAULT_PUNCTATION;

/*****************************************************************************
 * prototyping of _very_ local hardware functions
 *****************************************************************************/
static void _speechOutput(char *buff);
static void _speechOutputBuf(char *buffer, int length);
static void _send_to_tts(unsigned char command);
static void _send_string_to_tts(char *string);
static void _send_buffer_to_tts(char *buffer, int length);
int  _setup_tts(void);


/***********************************************************************/
/* Low level commands to directly access the hardware                  */
/***********************************************************************/
#define TALK_DTR_ADDRESS		0x29e
#define TALK_DTR_LPC_PORT		0x00
#define TALK_DTR_TTS_PORT		0x01
#define TTS_BASE                       (TALK_DTR_TTS_PORT+TALK_DTR_ADDRESS)
/* #define TTS_BASE 0x29f */

/*****************************************************************************
 *
 * FUNCTION: int SPEECH_init(void);
 *
 * DESCRIPTION: Open the speech device and initialize it.
 *
 * INPUTS:
 * SPEECH_PTR init - structure describing speech device.
 *
 * OUTPUTS: Returns true if device opened successfully, false otherwise.
 *
 * HISTORY:
 *
 *****************************************************************************/

int SPEECH_init(int _ttsType, const char *devname, speed_t baud)
{
  if (localTo(SPEECH_CONFIG_VARIABLE) != TRUE) {
    fprintf(stderr,"Could not verify that this host has speech synthesizer\n");
    exit(-1);
  }

#ifdef i386
#ifdef DEV_PORT

  ttsType = _ttsType;
  ttsFd = open(devname, O_RDWR);

  if (ttsType == 2) {
    setBaudRate(ttsFd, baud);
  }

#else
  /* Important: This command allows us to access the I/O ports.
   * If not set, io interfacing will result in a segm. fault. Usually,
   * only S-users are allowed to set ioperms. We have changed the kernel
   * (/usr/src/linux/kernel/ioport.c so that everyone may change 
   * permissins. This is somehow dangerous, but a simple fix for our 
   * problems. S.Thrun and B.Trouvain, 94-1-22 */

  ioperm((unsigned long) TTS_BASE, (unsigned long) 1, 1); 
#endif

#else
  fprintf(stderr, "\n\tWarning: speech interface works only under Linux\n");
  exit(1);
#endif

  _setup_tts();

  return TRUE;
}


/*****************************************************************************
 *
 * FUNCTION: int SPEECH_terminate(void);
 *
 * DESCRIPTION: Close the speech device.
 *
 * INPUTS:
 *
 * OUTPUTS: Returns true if device opened successfully, false otherwise.
 *
 * HISTORY:
 *
 *****************************************************************************/

void SPEECH_terminate(void)
{
  ;
}

/************************************************************************/
/*  Simplest speech commands                                            */
/************************************************************************/


int SPEECH_talk_text(char *plain_text)
{
  int count=1;
  char buff[20];
  
  if (plain_text==NULL)
    return -1;
  
  /* Convert to text mode */
  if (_speech_current_mode!=_SPEECH_TEXT_MODE)
    {
      _speech_current_mode=_SPEECH_TEXT_MODE;
      sprintf(buff,"\01%iT", _speech_current_text_delay);
      _speechOutput( buff);
    }
  _speechOutput( plain_text);
  
  return count;
}



int SPEECH_talk_character(char *spelled_text)
{
  int count=1;
  char buff[20];
  
  if (spelled_text==NULL)
    return -1;
  
  /* Convert to spelled mode */
  if (_speech_current_mode!=_SPEECH_CHAR_MODE)
    {
      _speech_current_mode=_SPEECH_CHAR_MODE;
      sprintf(buff,"\01%iC", _speech_current_char_delay);
      _speechOutput( buff);
    }
  _speechOutput( spelled_text);
  
  return count;
}



int SPEECH_talk_phoneme(char *phoneme_text)
{
  int count=1;
  
  if (phoneme_text==NULL)
    return -1;
  
  /* Convert to phoneme mode */
  if (_speech_current_mode!=_SPEECH_PHONEME_MODE)
    {
      _speech_current_mode=_SPEECH_PHONEME_MODE;
      _speechOutput("\01D");
    }
  
  _speechOutput( phoneme_text);
  
  return count;
}



int SPEECH_talk_sample(int *PCM_sample, int length, int rate)
{
  int pcm_rate;
  char buff[20];
  
  /* rate is in kHz, convert to the integers used in the command */
  
  pcm_rate = 179-800/rate;
  
  if ((pcm_rate<0) || (pcm_rate>99)) {
    fprintf(stderr, "Invalid sample rate for sampled speech %d \n", rate);
    return FALSE;
  }
  
  if (PCM_sample==NULL)
    return FALSE;
  
  /* Convert to PCM mode, then back to Text mode */
  _speech_current_mode =_SPEECH_PCM_MODE;
  sprintf(buff,"\01%02d#", pcm_rate);
  _speechOutput(buff);
  _speechOutputBuf( (char *) PCM_sample, length);
  _speechOutput("\128");
  _speech_current_mode =_SPEECH_TEXT_MODE;
  
  return TRUE;
  
}

/************************************************************************/
/*  Parameter tuning                                                    */
/************************************************************************/

void SPEECH_enable_intonation(void)
{
  _speechOutput("\01E");
}

void SPEECH_disable_intonation(void)
{
  _speechOutput("\01M");
}


int SPEECH_set_frequency(int frequency)
{
  char buff[20];
  
  if (_speech_current_frequency==frequency)
    return frequency;
  
  if ((frequency<0)||(frequency>9))
    return (-1);
  _speech_current_frequency=frequency;
  
  sprintf(buff,"\01%iF", _speech_current_frequency);
  _speechOutput( buff);
  
  return frequency;
}


int SPEECH_set_speed(int speed)
{
  char buff[20];
  
  if (_speech_current_speed==speed)
    return speed;
  
  if ((speed<0)||(speed>9))
    return (-1);
  
  _speech_current_speed=speed;
  
  sprintf(buff,"\01%iS", _speech_current_speed);
  _speechOutput( buff);
  
  return speed;
}


int SPEECH_set_pitch(int pitch)
{
  char buff[20];
  
  if (_speech_current_pitch==pitch)
    return pitch;
  
  if ((pitch<0)||(pitch>99))
    return (-1);
  
  _speech_current_pitch=pitch;
  
  sprintf(buff,"\01%iP", _speech_current_pitch);
  _speechOutput( buff);
  
  return pitch;
}


int SPEECH_set_volume(int volume)
{
  char buff[20];
  
  if (_speech_current_volume==volume)
    return volume;
  
  if ((volume<0)||(volume>0))
    return (-1);
  
  _speech_current_volume=volume;
  
  sprintf(buff,"\01%iV", _speech_current_volume);
  _speechOutput( buff);
  
  return volume;
}

int SPEECH_set_tone(int tone)
{
  char buff[20];
  
  if (_speech_current_tone==tone)
    return tone;
  
  if ((tone<0)||(tone>1))
    return (-1);
  
  _speech_current_tone=tone;
  
  sprintf(buff,"\01%iX", _speech_current_tone);
  _speechOutput( buff);
  
  
  return tone;
}

int SPEECH_set_punctuation_level(int punc_level)
{
  char buff[20];
  
  if (_speech_current_punc_level==punc_level)
    return punc_level;
  
  if ((punc_level<0)||(punc_level>7))
    return (-1);
  
  _speech_current_punc_level=punc_level;
  
  sprintf(buff,"\01%iB", _speech_current_punc_level);
  _speechOutput( buff);
  
  return punc_level;
}

int SPEECH_set_timeout_delay(int delay)
{
  char buff[20];
  
  if (_speech_current_timeout_delay==delay)
    return delay;
  
  if ((delay<0)||(delay>15))
    return (-1);
  
  _speech_current_timeout_delay=delay;
  
  sprintf(buff,"\01%iY",  _speech_current_timeout_delay);
  _speechOutput( buff);
  
  return delay;
}

int SPEECH_set_text_delay(int delay_between_words)
{
  char buff[20];
  
  if (_speech_current_text_delay==delay_between_words)
    return _speech_current_text_delay;
  
  if ((delay_between_words<0)||(delay_between_words>15))
    return (-1);
  
  _speech_current_text_delay=delay_between_words;
  
  sprintf(buff,"\01%iT", _speech_current_text_delay);
  _speechOutput( buff);
  
  return _speech_current_text_delay;
}


int SPEECH_set_character_delay(int delay_between_letters)
{
  char buff[20];
  
  if (_speech_current_char_delay==delay_between_letters)
    return _speech_current_char_delay;
  
  if ((delay_between_letters<0)||(delay_between_letters>31))
    return (-1);
  
  _speech_current_char_delay=delay_between_letters;
  
  sprintf(buff,"\01%iC", _speech_current_char_delay);
  _speechOutput( buff);
  
  return _speech_current_char_delay;
}


/************************************************************************/
/*  Exception dictionary handling                                       */
/************************************************************************/
int SPEECH_load_exceptions(char *exception_rules)
{
  return -1;
}


void SPEECH_enable_exceptions(void)
{
}

void SPEECH_disable_exceptions(void)
{
}


/************************************************************************/
/*  Tune interface                                                      */
/************************************************************************/


void SPEECH_tone_initialize(int voice_amplitude, int tempo)
{
  char settings[4];
  
  /* the pattern is 0, Amplitude, tempo (lsb), tempo (msb) */
  settings[0] = 0;
  settings[1] = (char) voice_amplitude;
  settings[2] = (char) tempo;
  settings[3] = 0; /* (char) tempo; */
  
  /* Convert to tone mode */
  if (_speech_current_mode!=_SPEECH_TONE_MODE) {
    _speechOutputBuf("\01J",2);
    _speech_current_mode =_SPEECH_TONE_MODE;
  }
  
  _speechOutputBuf(settings,4);
}



void SPEECH_tone_store(int length, VOICE_FRAME *song)
{
  if (_speech_current_mode!=_SPEECH_TONE_MODE) {
    SPEECH_tone_initialize(DEFAULT_AMPLITUDE, DEFAULT_TEMPO);
  }
  _speechOutputBuf((char *) song, length * sizeof(VOICE_FRAME));
}




void SPEECH_tone_replay(void)
{
  static char playCmd[] = {0, 0, 1, 1};
  static char quitCmd[] = {0, 0, 0, 0};
  
  if (_speech_current_mode!=_SPEECH_TONE_MODE) {
    SPEECH_tone_initialize(DEFAULT_AMPLITUDE, DEFAULT_TEMPO);
  }
  _speechOutputBuf(playCmd, 4);
  _speechOutputBuf(quitCmd, 4);
}



void SPEECH_tone_play(int length, VOICE_FRAME *song)
{
  if (_speech_current_mode!=_SPEECH_TONE_MODE) {
    SPEECH_tone_initialize(DEFAULT_AMPLITUDE, DEFAULT_TEMPO);
  }
  
  SPEECH_tone_store(length,song);
  SPEECH_tone_replay();
}



/************************************************************************/
/*  Hardware management                                                 */
/************************************************************************/


void SPEECH_clear(void)
{
  _speechOutput("\01R");
}




void SPEECH_reinitialize(void)
{
  _speechOutput("\01@");
}




void SPEECH_zap_command(void)
{
  _speechOutput("\01Z");
}



#ifdef i386

#ifdef DEV_PORT

static void
outb (char val, short port)
{
  if (ttsType == 1) {
    lseek(ttsFd, port, SEEK_SET);
  }
  write(ttsFd, &val, 1);
}

static unsigned char
inb (short port)
{
  unsigned int ret;

  if (ttsType == 1) {
    lseek(ttsFd, port, SEEK_SET);
    read(ttsFd, &ret, 1);
  }
  else {
    ret = 0x10;
  }

  return ret;
}

#else

/* 
 * These assemply macros are from Robert Baron
 */


#define inb(y) \
({ int _tmp__; \
     asm volatile("inb %%dx, %0" : "=a" (_tmp__) : "d" (y)); \
     _tmp__; })


#define outb(y, x) \
({ int _tmp__; \
     asm volatile("outb %1, %%dx" : "=r" (_tmp__) : "a" (y) , "d" (x)); \
     y; })


#endif

#else
/* This performs a toggling so that if the speech unit is called from a non486 
   machine, it doesn't lock forever, but continuosly toggles */
int _speechNotSimulatorHack = 0xFF;
#define inb(y) (_speechNotSimulatorHack=(!_speechNotSimulatorHack))
#define outb(y, x) 
#endif

/***********************************************************************
 * FUNCTION: SPEECH_is_speaking(void)
 *
 * DESCRIPTION: Queries if the speech device is producing output
 *
 ***********************************************************************/



int SPEECH_is_speaking(void)
{
  unsigned char rdy=0;
  
  rdy=inb(TTS_BASE);
  return ((int) (rdy & 0x20));
}


/***********************************************************************
 * FUNCTION: _is_speaking(void)
 *
 * DESCRIPTION: Waits until the voice is not actually speaking
 *              (Note that this is different to actually proceesing data!)
 *
 ***********************************************************************/



void _is_speaking(void)
{
  unsigned char rdy=0;
  
  do 
    {
      rdy=inb(TTS_BASE);
      usleep(1);
    }
  while ((rdy & 0x20));       /* While still speaking */
}



/***********************************************************************
 * FUNCTION: _send_to_tts(command)
 *               byte command;
 * 
 * DESCRIPTION: Sends a low level command to the TTS port (ie direct speech or
 *              phonemes. Checks to make sure that device is ready to receive
 *              commands. usleeps() seem to be required as MACH appears to be 
 *              'trapping' the bus. The introduction of a system call clears up
 *              this problem. Should be corrected if realtime LPC production is
 *              required.
 ***********************************************************************/


void _send_to_tts (unsigned char command)
{
  unsigned char rdy=0;



  do
    {
      rdy=inb(TTS_BASE);
      usleep(1);
    }
  while (!(rdy & 0x10));      /* While not ready to receive */
  outb(command, TTS_BASE);
  do {
    rdy=inb(TTS_BASE);
    usleep(1);
  }
  while (!(rdy & 0x10));      /* While not finished receiving */
}

/***********************************************************************
 * FUNCTION: setup_tts(void)
 *
 * DESCRIPTION: Sends reset sequence to the speech unit.
 ***********************************************************************/

int _setup_tts(void)
{
  
  _send_to_tts(0x1e);   /* Interrupt     */
  /* _send_to_tts(1);*/      /*   sequence    */
  /* _send_to_tts('@'); */    /* Reset */
  /* _send_to_tts(0); */     /* Flush these to tts */
  SPEECH_reinitialize();
  SPEECH_clear();
  return TRUE;
}

/***********************************************************************
 * FUNCTION: send_string_to_tts(string)
 *
 * DESCRIPTION: Bundles up the sending of a string to the tts. 
 ***********************************************************************/

void _send_string_to_tts(char *string)
{
  unsigned int i;
  
  for (i=0; i<strlen(string); i++)
    _send_to_tts(string[i]);
  _send_to_tts(0);
}

/***********************************************************************
 * FUNCTION: send_buffer_to_tts(char *buffer, int length)
 *
 * DESCRIPTION: Bundles up the sending of a string to the tts. 
 ***********************************************************************/

static void _send_buffer_to_tts(char *buffer, int length)
{
  int i;
  
  for (i=0; i<length; i++)
    _send_to_tts(buffer[i]);
}

/***********************************************************************
 * FUNCTION: _speechOutput(string)
 *
 * DESCRIPTION: Bundles up the sending of a string to the tts. 
 ***********************************************************************/

static void _speechOutput(char *string)
{
  _send_string_to_tts(string);
}

/***********************************************************************
 * FUNCTION: _speechOutputBuf(string, length)
 *
 * DESCRIPTION: Bundles up the sending of a string to the tts. 
 ***********************************************************************/

static void _speechOutputBuf(char *buffer, int length)
{
  _send_buffer_to_tts(buffer,length);
}
