
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





/* added for RWI - determines an envariable we will check to make sure */
/* we are on the right host before continuing  */
#define SPEECH_CONFIG_VARIABLE "SPEECHHOST"

#ifdef i386
#include <sys/types.h>
#include <termios.h>

/*
 * Linux glibc-2/libc-6 cleanely doesn't 
 * bother supporting this old waste of space.
 */

#ifndef TIOCGETP 
#define TIOCGETP        0x5481
#define TIOCSETP        0x5482
#define RAW             1
#define CBREAK          64

struct sgttyb
{
    unsigned short sg_flags;
    char sg_ispeed;
    char sg_ospeed;
    char sg_erase;
    char sg_kill;
    struct termios t;
    int check;
};
#endif /* TIOCGETP */
#endif

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
/* #include "Common.h"  */
#include <unistd.h>

#ifndef SPEECH_INTERFACE_LOADED
#define SPEECH_INTERFACE_LOADED
 
#define TRUE 1
#define FALSE !TRUE

#define DEFAULT_TEXT_DELAY    1
#define DEFAULT_CHAR_DELAY    1
#define DEFAULT_TIMEOUT_DELAY 0
#define DEFAULT_FREQUENCY     5
#define DEFAULT_SPEED         2
#define DEFAULT_PITCH         40
#define DEFAULT_VOLUME        5
#define DEFAULT_TONE          1
#define DEFAULT_PUNCTATION    6
#define DEFAULT_AMPLITUDE     200
#define DEFAULT_TEMPO         86

/* Tones */
#define BASS       0
#define TREBLE     1

/* Puctuation Levels */
#define ALL_DIGITS   0
#define MOST         1
#define SOME_DIGITS  2
#define NONE_DIGITS  3
#define ALL_NUMBERS  4
#define MOST_NUMBERS 5
#define SOME_NUMBERS 6
#define NONE_NUMBERS 7

/* Musical Notes */
#define C4Nat   255
#define C4sharp 241
#define D4Nat   228
#define D3sharp 215
#define E3Nat   203
#define F3Nat   192
#define F3sharp 181
#define G3Nat   171
#define G3sharp 161
#define A3Nat   152
#define A3sharp 144
#define B3Nat   136
#define C3Nat   128
#define C3sharp 121
#define D3Nat   114
#define D2sharp 107
#define E2Nat   101
#define F2Nat   96
#define F2sharp 90
#define G2Nat   85
#define G2sharp 81
#define A2Nat   76
#define A2sharp 72
#define B2Nat   68
#define C2Nat   64
#define C2sharp 60
#define D2Nat   57
#define D1sharp 54
#define E1Nat   51
#define F1Nat   48
#define F1sharp 45
#define G1Nat   43
#define G1sharp 40
#define A1Nat   38
#define A1sharp 36
#define B1Nat   34
#define C1Nat   32
#define C1sharp 30
#define D1Nat   28
#define D0sharp 27
#define E0Nat   25
#define F0Nat   24
#define F0sharp 23
#define G0Nat   21
#define G0sharp 20
#define A0Nat   19
#define A0sharp 18
#define B0Nat   17
#define C0Nat   16
#define C0sharp 15
#define D0Nat   14

/* Musical durations */
#define WHOLE_NOTE         192
#define HALF_NOTE          96
#define QUARTER_NOTE       48
#define SIXTEENTH_NOTE     24
#define THIRTY_SECOND_NOTE 12
#define SIXTY_FOURTH_NOTE  6

typedef struct{
  char duration;
  char voice_pitch[3];
} VOICE_FRAME;


/************************************************************************/
/*  Simplest speech commands                                            */
/************************************************************************/
/* SPEECH_init must be called prior to any of the SPEECH commands being 
   utilised. It determines where in the memory space DoubleTalk is located, 
   and then resets the card, yielding an internal pointer used in the SPEECH
   routines. If the synthesis board cannot be found, -1 will be returned. 0 is
   returned on sucess. For clean management the card should be terminated
   cleanly. Otherwise, it may not be found when the code is rerun!
   */


/*
 * ttsTypes:
 *
 * 1 = DoubleTalkPC
 * 2 = DoubleTalkLT
 */

int  SPEECH_init(int ttsType, const char *ttsDev, speed_t ttsBaud);
void SPEECH_terminate(void);

/************************************************************************/
/*  Simplest speech commands                                            */
/************************************************************************/

/* All text sent to DoubleTalk is spoken as complete sentences. Punctuation 
   is also taken into consideration by the intonation generation algorithms.
   DoubleTalk will not begin translating text until it receives a CR (0d)
   -this ensures that sentence boundaries receive the proper inflection.
   plain_text: string to be said.
   returns: number of characters spoken, -1 on error.
   */
int SPEECH_talk_text(char *plain_text);

/* This mode causes DoubleTalk to translate input text on a
   character-by-character basis; i.e., text will be spelled instead of spoken 
   as words. DoubleTalk does not wait for a CR in this mode.
   spelled_text: string to be spelled.
   returns: number of characters spoken, -1 on error.
   */
int SPEECH_talk_character(char *spelled_text);

/* This mode disables DoubleTalk's text-to-speech translator, allowing
   DoubleTalk's phonemes to be directly accessed. Phonemes in the input buffer 
   will not be spoken until a CR is received.
   phoneme_text: string to be pronounced.
   returns: number of characters spoken, -1 on error.
   */
int SPEECH_talk_phoneme(char *phoneme_text);

/* In PCM mode, all data sent to DoubleTalk is written directly to
   DoubleTalk's digital-to-analog converter (DAC) buffer. This results in
   a very high data rate, but provides the capability of producing the
   highest quality speech, as well as sound effects that are not possible
   using the other modes. DoubleTalk also supports ADPCM, which reduces
   the effective data rate by a factor of one-half to one-third that of
   standard PCM.
   
   The PCM_sample is directed to the DAC buffer. Since the PCM data is
   buffered within DoubleTalk, the output (sampling) data rate is
   independent of the data rate into DoubleTalk, as long as the input
   data rate is greater than or equal to the sampling rate. The sampling
   rate can be programmed to virtually any rate between 4 kHz and 11.1
   kHz (32,000-88,800 bps) with the PCM Mode command. The relationship
   between the command parameter n and the sampling rate fs is
   
   n = 155.25 - 625/fs
   fs = 625/(155.25 - n)
   
   where fs is measured in kHz. The range of n is 0-99, hence fs can range
   from 4 kHz to 11.1 kHz. Only absolute parameters can be used with this
   command.
   
   PCM_sample: digitilized sound.
   length: number of bytes in sample.
   rate: sampling rate.
   returns: number of characters spoken, -1 on error.
           NOTE - actually returns 1 on sucess, not no characters!
   */


int SPEECH_talk_sample(int *PCM_sample,
			   int length,
			   int rate);

/************************************************************************/
/*  Parameter tuning                                                    */
/************************************************************************/
/*

   Intonation is the variation of pitch throughout a sentance or
   phrase. When intonation is enabled, the synthesizer attempts to
   approximate the pitch patterns of humman speech as closely as
   possible. For example, when a sentence ends in a period, the pitch
   drops at the end of a sentence. If a sentence ends in a question mark
   and the sentance does not begin with "wh" (who, what, where, etc.),
   the pitch rises - otherwise it falls, like a period.
   
   When intonation is enabled, the pitch will also vary in the Character
   and Phoneme modes. The Pitch follows the same rules for punctuation as
   it does for Text mode.
   
   Be default intonation is enabled. If automatic intonation is not
   desired, it may be disabled, causing the synthesizer to speak in a
   monotone voice.  Intonation should be disabled whenever amnual
   intonation is applied using the set_pitch command
   */

void SPEECH_enable_intonation(void);
void SPEECH_disable_intonation(void);

/* This command adjusts the synthesizer's overall frequency response (vocal 
   tract formant frequencies), over the range 0 through 9. By varying the 
   frequency, speech quality can be fine-tuned or voice type changed. The 
   default frequency is 5. Note that this command takes effect immediately - 
   any text sent prior to the command will be spoken at the new formant 
   frequency.     
   */ 
int SPEECH_set_frequency(int frequency);

/* The synthesizer's overall rate (speed) of speech can be adjusted with 
   this command, from 0 (slowest) through 9 (fastest). The default speed 
   is 5.  
   */
int SPEECH_set_speed(int speed);

/* This command varies the synthesizer's pitch over a wide range, which can be 
   used to change the average pitch during speech, produce manual intonation, 
   or create sound effects. Pitch values can range from 0 through 99; the 
   default is 50.   
   */
int SPEECH_set_pitch(int pitch);

/* This command controls the synthesizer's volume level. Volume commands can 
   range from 0 through 9; the default is 5. The command 0 yields the 
   lowest possible volume; maximum volume is attained at 9. The command can 
   be used to set a new listening level or create emphasis in speech. 
   */
int SPEECH_set_volume(int volume);

/* The synthesizer supports two tone settings, BASS and TREBLE, which work much
   like the bass and treble controls on a stereo. The best setting to use 
   depends on the speaker being used and personal preference. Treble is the 
   default setting.
   */
int SPEECH_set_tone(int tone);

/* Depending on the application, it may be desirable to limit the speaking of
   certain punctuation. For example, if the synthesizer is used to proofread
   documents, the application may call for only unusual punctuation to be read.
   On the other hand, an application which orally echoes keyboard entries on a
   computer for a blind user may require that all punctuation be spoken.
   
   Eight levels of punctuation are used.  
   ALL_DIGITS      All/Digits
   MOST_DIGITS     Most (all punctuation except CR, LF, Space)/Digits
   SOME_DIGITS     Some ($%&#@=+*^<>|/\)/Digits
   NONE_DIGITS     None/Digits
   ALL_NUMBERS     All/Numbers
   MOST_NUMBERS    Most/Numbers
   SOME_NUMBERS    Some/Numbers
   NONE_NUMBERS    None/Numbers
   
   Besides determining which punctuation characters will be spoken and
   which will not, the Punctuation Level command also determines how
   number strings will be pronounced: as numbers (e.g., "one hundred
   twenty three") or digits ("one two three"). Levels SOME_NUMBERS and
   NONE_NUMBERS also cause currency strings to be read as they are
   normally spoken-for example, $11.95 is read as "eleven dollars and
   ninety five cents." The default punctuation level is SOME_NUMBERS. 
   */

int SPEECH_set_punctuation_level(int punc_level);

/* 

   The Text and Phoneme modes of the synthesizer defer the translation
   and synthesis of the contents of the input buffer until a CR or Null
   is received.  This ensures that text is spoken smoothly from word to
   word, and that the proper intonation is given to the beginning and
   ending of sentences. If text is sent to the synthesizer without a CR
   or Null, it will remain untranslated in the input buffer indefinitely.
   If it is expected that this condition may occur, use the Timeout
   command.

   DoubleTalk contains a software timer which forces the synthesizer to
   translate the buffer contents when the timer times out. The timer is
   enabled only if the Timeout parameter n is non-zero, the synthesizer
   is not active (not talking), and the input buffer contains no CR or
   Null characters. Any characters sent to DoubleTalk before timeout will
   automatically restart the timer.
   
   0    Indefinite (wait for CR/Null)
   1    200 milliseconds
   2    400 milliseconds
   3    600 milliseconds
   .
   .
   15   3000 milliseconds (3 sec.)
   
   The Timeout parameter n specifies the number of 200 millisecond (.2
   sec) periods in the delay time, which can range from 200 milliseconds
   to 3 seconds.  The default value is zero, which disables the timer. 
   */


int SPEECH_set_timeout_delay(int delay);

/* 
   The delay parameter n can be used to create a variable pause between
   words.  The shortest, and default delay of 0, is used for normal
   speech. For users not accustomed to synthesized speech, the
   synthesizer's intelligibility may be improved by using a longer delay.
   The longest delay that can be specified is 15.
   */

int SPEECH_set_text_delay(int delay_between_words);

/*
   This command puts DoubleTalk in the Character translation mode. The
   optional delay parameter n specifies how long the synthesizer will
   pause between characters. Values between 0 (the default) and 15
   provide pauses from shortest to longest, respectively. Values between
   16 and 31 provide the same range of pauses, but control characters
   will not be spoken. If the delay parameter is omitted, the current
   value will be used.
   */

int SPEECH_set_character_delay(int delay_between_letters);

/************************************************************************/
/*  Exception dictionary handling                                       */
/************************************************************************/

/* 
   This command purges DoubleTalk's exception dictionary of any
   exceptions and stores subsequent output from the host in the exception
   dictionary RAM. Since the memory used by the exception dictionary is
   the same physical RAM used by the input buffer, the space available
   for the input buffer is decreased proportionally by the size of the
   dictionary.
   
   The dictionary can be purged from DoubleTalk with the Reinitialize
   command, or by loading a "null" dictionary file into the dictionary
   RAM. Both methods reallocate the memory space occupied by the
   dictionary to the input buffer.
   
   Although exceptions are composed of standard ASCII characters, they
   must be compiled into the internal format used by DoubleTalk. This is
   done internally by the function, which means that its hardly intended
   for realtime usage.
   
   The topic of writing exception dictionaries is somewhat complex for
   the average (sane) user, and may be found on DoubleTalk's Dev. Tools
   disk.
   
   Once the exceptions are load, they may be enable and disable
   arbitarily as required, only having effect in Text or Character
   modes. 
   */

int SPEECH_load_exceptions(char *exception_rules);
void SPEECH_enable_exceptions(void);
void SPEECH_disable_exceptions(void);

/************************************************************************/
/*  Tune interface                                                      */
/************************************************************************/

/* 
   The Initialize command sets up the tone generators' amplitude and
   tempo (speed). The Initialize command may, however, be issued at any
   time afterward to change the volume or tempo.
   
   The overall range of the tempo is 1-65,535; the larger the value, the
   slower the overall speed of play. The amplitude and tempo affect all
   three tone generators, and stay in effect until another Initialize
   command is issued.  The command can be issued between Voice frames to
   change the volume or tempo on the fly (only Voice frames following the
   command will be affected). 
   */

void SPEECH_tone_initialize(int voice_amplitude, int tempo);

/* 
   Voice frames contain the duration and frequency (pitch) information
   for the tone generators. All Voice frames are stored in a 4K buffer
   within DoubleTalk, but are not played until the Play command is
   issued. If the number of voice frames exceeds 4K bytes in length,
   DoubleTalk will automatically begin playing the data.

   Voice frames (Figure 2) consist of three frequency time constants
   (K1-K3) and duration byte (KD), which specifies how long the three
   voices are to be played.  The relationship between the time constants
   Kn and the output frequency fn is:
   
   fn = 16,768/voice{n}

   where fn is in Hertz and voice{n} = 4-255. Setting voice{n} to zero
   will silence voice n during the frame.

   The duration parameter may be programmed to any value between 1 and
   255. The larger duration is made, the longer the voices will play
   during the frame.

   SPEECH_tone_play plays a tone immediately, tone_store appends without
   playing, an array(length) of voices to the buffer which can be heard
   using the replay command. 
   */

void SPEECH_tone_play(int length, VOICE_FRAME *voice);
void SPEECH_tone_store(int length, VOICE_FRAME *song);
void SPEECH_tone_replay(void);


/************************************************************************/
/*  Hardware management                                                 */
/************************************************************************/

/* 
   The Clear command stops the synthesizer and clears the input buffer of
   all text and commands. None of the synthesizer settings are affected,
   but any untranslated commands will be ignored. DoubleTalk will react
   immediately, even if the buffer is full. 
   */

void SPEECH_clear(void);

/* This command clears the input buffer (see Clear command) and
   restores all of the speech parameters to their default settings. The
   exception dictionary memory is also cleared and reallocated to the
   input buffer. 
   */

void SPEECH_reinitialize(void);

/* Beware the following - extreme effects */
/* This command prevents DoubleTalk from honoring subsequent commands,
   enabling it to read commands as they are issued (this can be useful
   for debugging some types of programs). Any pending commands in the
   input buffer will still be honored. The only way to restore command
   recognition after the Zap command has been issued is to perform a
   hardware reset, which is extremely unfriendly - Xavier won't be happy.
   */
void SPEECH_zap_command(void);


/* This command queries if the speech board is busy speaking or not.
   Has not yet been tested. S. Thrun 94-1-29
   */
int SPEECH_is_speaking(void);

#endif

