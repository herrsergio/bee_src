
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
 *

 * MODULE: sonar
 * FILE: sonar_interface.c
 *
 * ABSTRACT:
 * Wrapper package to allow easy access to rhino's sonar unit.
 *
 *****************************************************************************/

#ifdef VMS
#include "vms.h"
#include "vmstime.h"
#endif

#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include <math.h>
#include <sys/time.h>

#include "tcx.h"
#include "tcxP.h"
#include "SIMULATOR-messages.h"

#include "Common.h"
#include "libc.h"
#define DECLARE_SONAR_VARS
#include "sonar_interface.h"
#include "rwibase_interface.h"

/*****************************************************************************
 * Global Constants
 *****************************************************************************/

#define COMMAND_TIMEOUT  10

/*****************************************************************************
 * Global variables 
 *****************************************************************************/

float sonar_start_rot;
float sonar_start_pos_x;
float sonar_start_pos_y;

float sonar_readings[24];
unsigned long *sonar_act_mask;
unsigned long *sonar_mask_array[MAX_MASKS];

static FILE *debug_file_in = NULL;
HandlerList sonar_handlers;
static BOOLEAN LoopFlag = FALSE;
static int LoopIntervall = 1;

/* For recovery from errors */
static int recover_cnt=0;
static struct timeval Last_RT_Time;


static unsigned long *NewMask;
static int NewLoopIntervall;
static BOOLEAN ChangeMaskFlag = FALSE;
static BOOLEAN ChangeLoopIntervallFlag = FALSE;
static BOOLEAN SentAllFlag = FALSE;

static int VsonarToRsonar[] = {
  0,1,2,3,4,5,
  6,7,8,9,10,11,
  12,13,14,15,16,17,
  18,19,20,21,22,23
};

static int RsonarToVsonar[] = {
  0,1,2,3,4,5,
  6,7,8,9,10,11,
  12,13,14,15,16,17,
  18,19,20,21,22,23
};  

unsigned long all_mask[] = {
  6,
  0x1111,
  0x4444,
  0x6666,
  0x3333,
  0x5555,
  0x2222
};

unsigned long test_mask[] = {
  3,
  0x1111,
  0x4444,
  0x6666
};



/*****************************************************************************
 * Forward procedure declarations
 *****************************************************************************/

static BOOLEAN InitDevice(void);
static int WriteCommand(char *command, int n_bits, unsigned long l);
static void ProcessLine(char *line);
static void SonarParseReturn(char *start);
static void SONAR_report_check(void);
static void InitMasks(void);
static void InitSonar(void);
static void ResetSonarReadings(void);
static void LoopRecover(void);
static unsigned long Vhex_to_Rhex(unsigned long);

/*****************************************************************************
 *
 * FUNCTION: SONAR_Read(int rt_no)
 *
 * DESCRIPTION: Sends LoopIntervall RT commands to the sonar.
 *              The sequence is specified through sonar_act_mask[]
 *
 * INPUT: rt_no: Which position to start in sonar_act_mask.
 *               If rt_no<0 then RT_no is used (important for loop)
 *****************************************************************************/

void SONAR_Read(int rt_no)
{
  
  int cnt;
  unsigned long tmp;
  static int RT_no = 1;


  if (ChangeLoopIntervallFlag) {
    LoopIntervall = NewLoopIntervall;
    ChangeLoopIntervallFlag = FALSE;
  }
  RT_no++;
  if (rt_no >= 0)
    RT_no = rt_no;
  for (cnt=0; cnt < LoopIntervall; cnt++) {
    RT_no = RT_no % sonar_act_mask[0];
    if (RT_no == 0) {
      RT_no = sonar_act_mask[0];
      SentAllFlag = TRUE;
    }
    tmp = Vhex_to_Rhex( sonar_act_mask[RT_no]);  
    sonar_start_rot = (float) rwi_base.rot_position;
    sonar_start_pos_x = (float) rwi_base.pos_x;
    sonar_start_pos_y = (float) rwi_base.pos_y;

    WriteCommand("RT", 16, tmp);
  }
}


/*****************************************************************************
 *
 * FUNCTION: SONAR_LoopStart(unsigned long *mask);
 *
 * DESCRIPTION: Starts loop to read sonar. 
 * INPUTS:      mask[]  - pointer on array with hexnumbers describing
 *              read sequence.  mask[0] contains number of commands in mask[].
 *
 *****************************************************************************/

void SONAR_LoopStart(unsigned long *mask)
{
  gettimeofday( &Last_RT_Time, 0);
  
  LoopFlag = TRUE;
  sonar_act_mask = mask;
  ResetSonarReadings();
  SONAR_Read(0);
}


/*****************************************************************************
 *
 * FUNCTION: SONAR_LoopEnd(void);
 *
 * DESCRIPTION: Stops loop to read sonar. 
 *
 *****************************************************************************/
void SONAR_LoopEnd(void)
{
  LoopFlag = FALSE;
}


/*****************************************************************************
 *
 * FUNCTION: BOOLEAN SONAR_SetLoopIntervall(double param);
 *
 * DESCRIPTION: sets the number of RT commands to be sent at once
 *              (see SONAR_Read)
 *
 *****************************************************************************/

void SONAR_SetLoopIntervall(double param)
{
  ChangeLoopIntervallFlag = TRUE;
  NewLoopIntervall = (int) param;
}



/*****************************************************************************
 *
 * FUNCTION: BOOLEAN SONAR_ChangeMask(unsigned long *mask);
 *
 * DESCRIPTION: sets the number of RT commands to be sent at once
 *              (see SONAR_Read)
 *
 *****************************************************************************/

void SONAR_ChangeMask(unsigned long *mask)
{
  ChangeMaskFlag = TRUE;
  NewMask = mask;
}


/*****************************************************************************
 *
 * FUNCTION: unsigned long Vhex_to_Rhex(unsigned long Vhex)
 *
 * DESCRIPTION: computes the real RT parameter from the virtual
 *              hex number used in mask
 * INPUT: hex number of the virtual RT parameter (see mask)
 * OUTPUT: real hex number for the RT command
 *
 *****************************************************************************/

unsigned long Vhex_to_Rhex(unsigned long Vhex)
{
  int vsonar_no, rsonar_no, vsonar_no_in_chain, rsonar_no_in_chain;
  int rchain_no, i;
  unsigned long Rhex = 0, tmp, test;

  /* sonar_no[] is the transducer number in the linear array of all
   * transducers in sonar_readings[24].
   * How to convert the sonar_no is given by the conversion array 
   * VsonarToRsonar[24].
   */


  /* fprintf(stderr, "%x  -->  ", Vhex);   */
  for (i=0; i<4; i++) {
    if ((vsonar_no_in_chain = Vhex % 16) != 0) {
      if (vsonar_no_in_chain > 6) {
	printf("Wrong transducer number: %x\n", vsonar_no_in_chain);
	return(-1);
      }
      else {
	vsonar_no = vsonar_no_in_chain + 6*(3-i) - 1;
	/* Now we have the virtual sonar_no and convert it to the real sonar_no */
	rsonar_no = VsonarToRsonar[vsonar_no];
	/* The real sonar_no must be converted in a hex number */
	rchain_no = rsonar_no / 6;
	rsonar_no_in_chain = (rsonar_no % 6) + 1;
	tmp = rsonar_no_in_chain;
	tmp = tmp << (rchain_no * 4);
	/* Test, if these bits (i.e. this chain) are used already */
	test = 0xF << (rchain_no *4);
	if (test & Rhex) {
	  printf("More than 1 transducer on chain number %i!", rchain_no);
	  return(-1);
      }
	Rhex += tmp;
      }
    }
    Vhex = Vhex >> 4;
  }
  /* fprintf(stderr, "%x\n", Rhex); */
  return(Rhex);
}  


/*****************************************************************************
 *
 * FUNCTION: BOOLEAN SONAR_init(void);
 *
 * DESCRIPTION: Open the sonar device and initialize it.
 *
 * INPUTS:
 *
 * OUTPUTS: Returns true if device opened successfully, false otherwise.
 *
 *****************************************************************************/

BOOLEAN SONAR_init(void)
{

  struct timeval poll_interval;

  if (!sonar_device.dev.use_simulator && !sonar_device.dev.use_rwi_server){
    devInit();
    if (!InitDevice())
      return FALSE;
  }
    
  sonar_handlers = CreateHandlerList(SONAR_NUMBER_EVENTS);
  
  poll_interval.tv_usec = 0;
  poll_interval.tv_sec = SONAR_POLLSECONDS;

  if (!sonar_device.dev.use_simulator && !sonar_device.dev.use_rwi_server)
    devStartPolling(&sonar_device.dev, &poll_interval, 
		    LoopRecover);

  InitSonar();
  InitMasks();
  return TRUE;
}


/*****************************************************************************
 *
 * FUNCTION: BOOLEAN SONAR_outputHnd(int fd, long chars_available);
 *
 * DESCRIPTION: Handles character output from the sonar device.
 *
 *****************************************************************************/

#define SONAR_BUFFER_SIZE DEFAULT_LINE_LENGTH

void SONAR_outputHnd(int fd, long chars_available)
{

  static char buffer[SONAR_BUFFER_SIZE+1];
  static char *startPos = buffer; /* position to start parsing from */
  static char *endPos = buffer; /* position to add more characters */
  char *lineEnd;
  int numRead = 0;


  while (chars_available > 0) {
    if (startPos == endPos)
      { 
	startPos = endPos = buffer;
	bzero(buffer, SONAR_BUFFER_SIZE+1);
      }
    
    /* read in the output. */
    numRead = readN(&sonar_device.dev, endPos, 
		    MIN(chars_available,(SONAR_BUFFER_SIZE - 
					 (endPos - startPos))));
    endPos += numRead;
    if (numRead == 0)
      { /* handle error here. The port is already closed. */
      }
    else {
      /* see if we have a \n  or null character */
      lineEnd = (char *) strpbrk(startPos,"\n\r");
      while (lineEnd != NULL)
	{/* found a string, pass it to the parsing routines. */
	  *lineEnd = '\0';
	  SonarParseReturn(startPos);
	  startPos = lineEnd+1;
	  lineEnd = (char *) strpbrk(startPos,"\n\r");
	}
      /* Fix up the buffer. Throw out any consumed lines.*/
      if (startPos >= endPos) 
	{ /* parsed the whole thing, just clean it all up */
	  bzero(buffer, SONAR_BUFFER_SIZE+1);
	  startPos = endPos = buffer;
	}
      else if (startPos != buffer)
	{ /* slide it back and wait for more characters */
	  bcopy(startPos, buffer, (endPos - startPos));
	  endPos = buffer + (endPos - startPos);
	  startPos = buffer;
	}
    }
    chars_available = numChars(fd);
  }
}



/*****************************************************************************
 *	Function Name: LoopRecover()
 *	Arguments:
 *	Description:   Called when there didn't come any data from sonar.
 *                     Important for the loop to start again
 *	Returns: 
 ******************************************************************************/

void LoopRecover(void)
{
  struct timeval now;

  if (LoopFlag) {

    /* compute time since last RT received from sonar */
    gettimeofday( &now, 0);
    
    now.tv_usec -= Last_RT_Time.tv_usec;
    now.tv_sec -= Last_RT_Time.tv_sec;
    if (now.tv_usec < 0) {
      now.tv_usec += 1000000;
      now.tv_sec -= 1;
    }

    if ((int) now.tv_sec > LOOP_RECOVER_TIME) {
      if (recover_cnt < LOOP_RECOVER_CNT) {
	fprintf(stderr, "Didn't receive any sonar data for %i", (int) now.tv_sec);
	fprintf(stderr, " seconds! Try to start sonar loop again.\n");
	recover_cnt ++;
	SONAR_LoopStart(sonar_act_mask);
      }
      else {
	/* nearly same as SONAR_init() */
	struct timeval poll_interval;
	
	fprintf(stderr, "Didn't receive any sonar data for %i", (int) now.tv_sec);
	fprintf(stderr, " seconds! Try to initialize sonar again.\n");
	
	InitSonar();
	recover_cnt = 0;
	SONAR_LoopStart(sonar_act_mask);
      }
    }
  }
}


/*****************************************************************************
 *	Function Name: SONAR_timeoutHnd
 *	Arguments:
 *	Description:   Called when there is a timeout
 *	Returns: 
 ******************************************************************************/

void SONAR_timeoutHnd(void)
{
  /* stub : will be called when there is a timeout.
   */
  fprintf(stderr, "Sonar timed out\n");
}


/*****************************************************************************
 *
 * FUNCTION: BOOLEAN SONAR_terminate(void);
 *
 * DESCRIPTION: Close the sonar device.
 *
 * OUTPUTS: Returns true if device closed successfully, false otherwise.
 *
 *****************************************************************************/

void SONAR_terminate(void)
{
  LoopFlag = FALSE;
  close(sonar_device.dev.fd);
  sonar_device.dev.fd = -1;
}


/*****************************************************************************
 *	Function Name: SONAR_Debug
 *	Arguments:     debug_flag -- boolean
 *	Description:   This function set or reset the sonar debug, which
 *                     will printout the characters send and received by
 *                     the sonar interface.
 *	Returns: 
 ******************************************************************************/

void SONAR_Debug (BOOLEAN debug_flag, char *file_name)
{
  char file[30];
  
  sonar_device.dev.debug = debug_flag;
  if (debug_flag &&
      file_name != NULL) {
    strcpy (file, file_name);
    strcat (file, ".out");
    sonar_device.dev.debug_file = fopen(file, "w");
    strcpy (file, file_name);
    strcat (file, ".in");
    debug_file_in = fopen(file, "w");
  }
}


/* see devUtils.c for explanation */

void  SONAR_InstallHandler(Handler handler, int event, Pointer client_data)
{
  InstallHandler(sonar_handlers, handler, event, client_data);
}


void  SONAR_RemoveHandler(Handler handler, int event)
{
  RemoveHandler(sonar_handlers, handler, event);
}


void  SONAR_RemoveAllHandlers(int event)
{
  RemoveAllHandlers(sonar_handlers, event);
}


/* Checks wether the sonar returned some information */
void SONAR_look_for_sonar_device(void)
{
  if (!sonar_device.dev.use_simulator && !sonar_device.dev.use_rwi_server)
    ProcessSingleDevice(&sonar_device.dev);
}



/* starts sonar */
static void InitSonar()
{
  unsigned long power;
  char buffer[DEFAULT_LINE_LENGTH];

  if (sonar_device.dev.use_rwi_server)
    return;

  sprintf(buffer, "\n\n\n\r");
  flushChars(&(sonar_device.dev));
  writeN(&(sonar_device.dev), buffer,strlen(buffer));
  
  power = 0x0F;
  WriteCommand("CP", 8, power);
  sleep(1);
  power = 0xFF;
  WriteCommand("CP", 8, power);
  sleep(1);
}



/*****************************************************************************
 *
 * FUNCTION: void InitMasks(void);
 *
 * DESCRIPTION: initializes the loop masks
 *            
 *****************************************************************************/

static void InitMasks(void)
{
  int i;

  for (i=0; i<MAX_MASKS; i++)
    sonar_mask_array[i] = NULL;

  sonar_mask_array[0] = all_mask;
  sonar_mask_array[1] = test_mask;
  sonar_mask_array[2] = test_mask;
  sonar_act_mask      = sonar_mask_array[0];
}



static BOOLEAN InitDevice()
{
  /* code to open the real device here. */
  connectTotty(&sonar_device.dev);
  if( sonar_device.dev.fd == -1)
    return FALSE;
  else {
    connectDev(&sonar_device.dev);
    return TRUE;
  }
}



/*	Function Name: 
 *	Arguments:
 *	Description:
 *	Returns: 
 */



static int WriteCommand (char *command, int n_bits, unsigned long l)
{
  unsigned long mask;
  char buffer[DEFAULT_LINE_LENGTH];
  
  if (!sonar_device.dev.use_simulator && !sonar_device.dev.use_rwi_server &&
      sonar_device.dev.fd == -1) {
    fprintf (stderr, "Error in sonar_interface::WriteCommand>> device is not initialized\n");
    return -1;
  }
  
  /* check that l doesn't overflow nbits */
  
  switch (n_bits) {
  case 0:
    l = 0;
    mask = 0x0;
  case 8:
    mask = 0xFF;
    break;
  case 16:
    mask = 0xFFFF;
    break;
  case 32:
    mask = 0xFFFFFFFF;
    break;
  default:
    fprintf(stderr, "Error in sonar_interface::WriteCommand>> bad nbits %d\n", n_bits);
    return -1;
  }
  if (~mask & l) {
    fprintf(stderr, "Error in sonar_interface::WriteCommand>> overflow on argument\n");
    fprintf(stderr, "debug: mask = %x l = %x\n", mask, l);
    fprintf(stderr, "debug: ~mask = %x ~mask | l = %x\n", ~mask, (~mask | l));
    return -1;
  }
  
  if (!sonar_device.dev.use_simulator && !sonar_device.dev.use_rwi_server){
    if (n_bits != 0)
      sprintf (buffer, "%s %x\r", command, l);
    else
      sprintf (buffer, "%s\r", command);
    
    /* set the timeout, if one is available */
    if (sonar_device.dev.setTimeout != NULL)
      (* sonar_device.dev.setTimeout)(&sonar_device.dev,COMMAND_TIMEOUT);
    
    flushChars(&(sonar_device.dev));
    
    return ((int) writeN(&(sonar_device.dev), buffer,strlen(buffer)));
  }
  else if (sonar_device.dev.use_simulator){ /* **** simulator: Use TCX ***** */
    char *message;
    int length;

    message = (char *) malloc (sizeof(char) * DEFAULT_LINE_LENGTH);
    if (n_bits != 0)
      sprintf (message, "%s %x\r", command, l);
    else
      sprintf (message, "%s\r", command);
    tcxSendMsg(SIMULATOR, "SIMULATOR_message_from_sonar", &message);
    length = strlen(message);
    free(message);
    return length; 
  }
  else {			/* **** RWI server: Use TCX ******** */
    ;				/* sonar data is sent automatically! */
  }
  return(-1);
}


/*****************************************************************************
 *	Function Name: ProcessLine
 *	Arguments:     line -- output line from the sonar
 *	Description:   process the sonar output, updating the global
 *                     variable rwi_sonar and firing the installed handlers
 *	Returns: 
 ******************************************************************************/

static void ProcessLine(char *line) 
{
  double data_double,dist;
  float data_float; 
  int    data_int, len, i, tmp;
  unsigned long data_unsigned;
  char  a;

  static int no_of_chains_missing = 0;
  static int next_sonar = 0;
  static int cnt = 0;
  static int sonar_no[4];
  static BOOLEAN report_corrupted = FALSE;

  if (sonar_device.dev.debug)
    if (debug_file_in)
      fprintf(debug_file_in,"SonarInterface receiving: [%s]\n", line);
    else
      fprintf(stderr,"SonarInterface receiving: [%s]\n", line);
    

  /* all the streaming data come without any *, and
   * the first line with one star marks the end of the
   * transmission
   */
  
  a = *line++;
  
  if (a != '*')
    line--;
    
  /* RT specifies which sonar data are sent in the next lines.
   * The sonar numbers of the chains are saved in sonar_no_in_chain
   */
  if (strncmp (line, "CP", 2) == 0) {
    return;
  }

  else if (strncmp (line, "RT", 2) == 0) {

    gettimeofday( &Last_RT_Time, 0);
    recover_cnt = 0;

    if (cnt < LoopIntervall) 
      cnt++;
    else            /* there must have been an error in the loop */
      cnt = 1;      /* or some RT command from outside the loop */

    report_corrupted = FALSE;

    if (no_of_chains_missing != 0) {
      fprintf(stderr, "error: missing %i sonar returns\n", no_of_chains_missing);
      fprintf(stderr, "starting to get new sonar data\n");
      no_of_chains_missing = 0;
    }

    sscanf(line,"RT %x", &data_int);
    if (data_int > 0xFFFF) {
      fprintf(stderr, "error: overflow of sonar reading (%i)\n", data_int);
      return;
    }
    
    /* put the virtual sonar numbers in sonar_no for the next no_of_chains_missing
     * sonar data to come 
     */
    for (i=0; i<4; i++) {
      tmp = data_int % 16;
      if (tmp != 0) {
	no_of_chains_missing++;
	sonar_no[3-i] = RsonarToVsonar[tmp + i*6 - 1];
      }
      else
	sonar_no[3-i] = -1;
      data_int = data_int >> 4;
    }
    next_sonar = 0;
  }

  /* read the data of the sonars specified in RT */
  else  if ((no_of_chains_missing > 0) && !report_corrupted) {
    
    /* See if the string that comes is the correct one by 
       testing the length of the line (4 numbers + 1 blank) */
    
    len = strlen(line);
    
    if (len != 5){
      report_corrupted = TRUE;
      no_of_chains_missing = 0;
      fprintf(stderr, "Bad sonar_data: %s\n", line);
      return;
    }
    
    while (sonar_no[next_sonar] < 0) next_sonar++;
    if (next_sonar == 4) {
      fprintf(stderr, "wrong number of sonar data\n");
      return;
    }
    
    no_of_chains_missing--;

    sscanf(line,"%x", &data_int);
    dist = ((float) data_int) * CM_PER_SECOND * SECONDS_PER_CYCLE / 2.0;
    sonar_readings[sonar_no[next_sonar++]] = dist; 
    
    if (no_of_chains_missing == 0) {  /* all data of last RT command collected */

      if (LoopFlag) {
	FireHandler(sonar_handlers, SONAR_RT_COMPLETE, (Pointer) sonar_no);
	
	if (cnt == LoopIntervall) {
	  cnt = 0;
	  if (SentAllFlag) {
	    FireHandler(sonar_handlers, SONAR_REPORT, (Pointer) NULL);
	    SentAllFlag = FALSE;
	  }
	  if (ChangeMaskFlag) {
	    sonar_act_mask = NewMask;
	    ResetSonarReadings();
	    fprintf( stderr, "Oops. This part of code shouldn't be reached. Contact fox@cs.uni-bonn.de.\n");
	    /* COLLI_init_Lines(); */
	    ChangeMaskFlag = FALSE;
	    SONAR_Read(0);
	  }
	  else	 
	    SONAR_Read(-1); /* send next LoopIntervall RT commands */
	}
      }
    } 
  }
  
  else if (report_corrupted)
    fprintf(stderr, "sonar report corrupted, wait for next RT reply\n");
  
  else 
   fprintf(stderr, "SONAR: unknown data: %s\n", line);   
}

/*****************************************************************************
 *	Function Name: SIMULATOR_message_to_sonar_handler
 *	Arguments:
 *	Description:   Pseudo input via the simulator
 *	Returns: 
 ******************************************************************************/

void SIMULATOR_message_to_sonar_handler(TCX_REF_PTR   ref,
				       char        **message)
{
  /* fprintf(stderr, "Received a sonar-message <%s> from the simulator\n", 
   *message); */
  ProcessLine(*message);
  tcxFree("SIMULATOR_message_to_sonar", message);
}

/*****************************************************************************
 *	Function Name: SonarParseReturn
 *	Arguments:
 *	Description:   Used to parse the return from the sonar, a line at a time
 *	Returns: 
 ******************************************************************************/

static void SonarParseReturn(char *start)
{
  /* cancel the timeout. */
  if (sonar_device.dev.cancelTimeout != NULL)
    (* sonar_device.dev.cancelTimeout)(&(sonar_device.dev));
  
  /* parse the characters in the buffer and update state */
  
  if (strcmp(start, "") != 0)
    ProcessLine (start);
  
}

/*****************************************************************************
 *
 * FUNCTION: void SONAR_missed(Handler handler, 
 *                             Pointer client_data)
 *
 * DESCRIPTION: register a function to handle cases where a sonar sweep 
 * never arrives before the next sweep is requested.
 *
 *****************************************************************************/

void SONAR_missed(Handler handler, Pointer client_data)
{
  SONAR_InstallHandler(handler, SONAR_MISSED, client_data);
}


static void SONAR_report_check(void)
{

  /* only dummy (see BASE_report_check in rwibase_interface.c) */

}



/*****************************************************************************
 *
 * FUNCTION: ResetSonarReadings
 *
 *****************************************************************************/

void ResetSonarReadings(void)
{
  int i;

  for (i=0; i<24; i++)
    sonar_readings[i] = -1.0;

}


/*****************************************************************************
 *
 * FUNCTION: void signal_sonar(void);
 *
 * DESCRIPTION:
 *
 * Interrupt handler for the sonar.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

void signal_sonar(void)
{
  fprintf(stderr,"shutting down the sonar\n");
}


/*****************************************************************************
 *
 * FUNCTION: BOOLEAN start_sonar()
 *
 * DESCRIPTION:
 *
 * This routine starts sonar device on the robot.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/


BOOLEAN start_sonar(void)
{
  BOOLEAN result=TRUE;

  sonar_device.dev.sigHnd = signal_sonar;  


  
  /* Start up sonar device */
  result = SONAR_init();

  return result;
}


void stop_sonar(void)
{
  unsigned long power;

  power = 0x00;
  WriteCommand("CP", 8, power);
}









