
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
 * MODULE: laser
 * FILE: laser_interface.c
 *
 * ABSTRACT:
 * Wrapper package to allow easy access to rhino's laser unit.
 *
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include <math.h>
#include <sys/time.h>

#include "tcx.h"
#include "tcxP.h"
#include "SIMULATOR-messages.h"
#include "LASER_SERVER-messages.h"

#include "Common.h"
#include "libc.h"
#include "collision.h"

#define DECLARE_LASER_VARS
#include "laser_interface.h"
#include "collisionIntern.h"


/*****************************************************************************
 * Global Constants
 *****************************************************************************/

#define LASER_DEVICE_TIMEOUT 10

#ifdef BONN_LASER

#define NUMBER_OF_LASERS         2
#define NUMBER_OF_LASER_READINGS 180
#define FRONT_LASER_START_ANGLE   DEG_270
#define REAR_LASER_START_ANGLE   DEG_90

#else

#define NUMBER_OF_LASERS         1
#define NUMBER_OF_LASER_READINGS 180
#define FRONT_LASER_START_ANGLE   DEG_270
#define REAR_LASER_START_ANGLE   DEG_90

#endif

/*****************************************************************************
 * Defines for the format of the protocol
 *****************************************************************************/

#define LASER_BUFFER_SIZE 8196

#define LASER_MESSAGE_LENGTH ((NUMBER_OF_LASER_READINGS) * 2 + 10)

#define INITIAL_ACKNOWLEDGE -1
#define ACKNOWLEDGED     0
#define NOT_ACKNOWLEDGED 1
#define TIME_OUT         2

/*****************************************************************************
 * Global variables 
 *****************************************************************************/

static int communication_verbose = FALSE;
static int sensors_verbose = FALSE;

/* Is necessary to delete the static defined buffer after having sent a
 * command to the laser. The buffer is deleted in Laser_outputHnd. */
static BOOLEAN commandwritten[NUMBER_OF_LASERS];

LASER_reading frontLaserReading =
{0, 0, NULL, NULL, NULL, NULL, 0.0, 0.0, {0.0, 0.0}, 0.0};
LASER_reading rearLaserReading  =
{0, 0, NULL, NULL, NULL, NULL, 0.0, 0.0, {0.0, 0.0}, 0.0};

HandlerList laser_handlers;

static Point positionAtLastRequest[NUMBER_OF_LASERS];
static float rotationAtLastRequest[NUMBER_OF_LASERS];

/*****************************************************************************
 * Forward procedure declarations
 *****************************************************************************/

static void initLaserReadingStructures( void);

/* Same functions for both lasers. */
static unsigned short computeCRC( unsigned char* CommData, unsigned short uLen);
static void ProcessLaserStatus(   unsigned char *infos, int length);

/* Same functions for both lasers. Need LASER_TYPE as parameter. */
static void ProcessLine( LASER_reading* scan, unsigned char *line, int length) ;
static void LASER_outputHnd( int fd, long chars_available,LASER_TYPE* laserDevice);
static BOOLEAN initAndStartLaserDevice( LASER_TYPE* laserDevice);
static int WriteLaserCommand( LASER_TYPE* laserDevice,
			      unsigned char command, char *argument, int arg_length);
static int waitforACK( LASER_TYPE* laserDevice);
static void requestLaserStatus( LASER_TYPE* laserDevice);
static int laserRequestReading( LASER_TYPE* laserDevice, int num);
static void adaptBaudRate( LASER_TYPE* laserDevice);
static void laser_9600_baud( LASER_TYPE* laserDevice);
static void laser_19200_baud( LASER_TYPE* laserDevice);
static void laser_38400_baud( LASER_TYPE* laserDevice);
static void closeLaserDevice( LASER_TYPE* laserDevice);
static void signalLaser(void);


/*****************************************************************************
 *****************************************************************************
 * GLOBAL functions. 
 *****************************************************************************
 *****************************************************************************/

/*****************************************************************************
 *
 * FUNCTION: BOOLEAN start_laser()
 *
 * DESCRIPTION:
 *
 * This routine starts laser device on the robot.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/
BOOLEAN
start_laser(void)
{
  static BOOLEAN firstTime = TRUE;
  BOOLEAN success = TRUE;
  
  if ( firstTime) {
    
    /* Create the list for the handlers. */
    laser_handlers = CreateHandlerList(LASER_NUMBER_EVENTS);
    
    initLaserReadingStructures();

    /* Do the device stuff (only if the simulator is not used). */
    if ( USE_FRONT_LASER && ! frontLaserDevice.dev.use_simulator &&
	 ! use_laser_server) {
      if ( ! initAndStartLaserDevice( &frontLaserDevice)) {
	fprintf( stderr, "Cannot initialize front laser.\n");
	success = FALSE;
      }
    }
    if ( USE_REAR_LASER && ! rearLaserDevice.dev.use_simulator &&
	 ! use_laser_server) {
      if ( ! initAndStartLaserDevice( &rearLaserDevice)) {
	fprintf( stderr, "Cannot initialize rear laser.\n");
	success = FALSE;
      }
    }
  }

  firstTime = FALSE;
  return success;
}


void stop_laser(void)
{
  if ( USE_FRONT_LASER)
    closeLaserDevice( &frontLaserDevice);
  if ( USE_REAR_LASER)
    closeLaserDevice( &rearLaserDevice);
}


/* Checks wether the laser returned some information */
void LASER_look_for_laser_device(void)
{
  if ( USE_FRONT_LASER && ! frontLaserDevice.dev.use_simulator &&
       ! use_laser_server)
    ProcessSingleDevice(&frontLaserDevice.dev);
  if ( USE_REAR_LASER && ! rearLaserDevice.dev.use_simulator &&
       ! use_laser_server)
    ProcessSingleDevice(&rearLaserDevice.dev);
}



/*****************************************************************************
 *
 * FUNCTION: Functions to deal with the handlers of certain laser events. 
 * see devUtils.c for explanation 
 *****************************************************************************/
void
LASER_InstallHandler( Handler handler, int event, Pointer client_data)
{
  InstallHandler( laser_handlers, handler, event, client_data);
}


void
LASER_RemoveHandler(Handler handler, int event)
{
  RemoveHandler(laser_handlers, handler, event);
}


void
LASER_RemoveAllHandlers(int event)
{
  RemoveAllHandlers(laser_handlers, event);
}


/*****************************************************************************
 *****************************************************************************
 * LOCAL functions. 
 *****************************************************************************
 *****************************************************************************/

/*****************************************************************************
 *
 * FUNCTION: BOOLEAN initAndStartLaserDevice();
 *
 * DESCRIPTION: Open the laser device and initialize it.
 *
 * INPUTS: see devUtils.h for explanation
 *
 * OUTPUTS: Returns true if device opened successfully, false otherwise.
 *
 *****************************************************************************/
static BOOLEAN
initAndStartLaserDevice( LASER_TYPE* laserDevice)
{

  static void sendInitToLaser( LASER_TYPE* laserDevice);
  
  /* Initialize the structure in devUtils.c */
  devInit();
  
  /* Open real device */
  connectTotty(&(laserDevice->dev));
  if( laserDevice->dev.fd == -1)
    return FALSE;

  connectDev(&(laserDevice->dev));
  laserDevice->dev.sigHnd = signalLaser;

  sendInitToLaser( laserDevice);
  
  return TRUE;
}
  
/*****************************************************************************
 *	Function Name: SIMULATOR_message_to_sonar_handler
 *	Arguments:
 *	Description:   Pseudo input via the simulator
 *	Returns: 
 ******************************************************************************/
void SIMULATOR_message_to_laser_handler( TCX_REF_PTR   ref,
					 SIMULATOR_message_to_laser_ptr laser)
{
  int i;
  
  if ( use_laser) {
    
    /* Get the actual position of the robot. */
    updateActualPosition( &(frontLaserReading.rPos),
			  &(frontLaserReading.rRot),
			  DONT_CONSIDER_DIRECTION);
    updateActualPosition( &(rearLaserReading.rPos),
			  &(rearLaserReading.rRot),
			  DONT_CONSIDER_DIRECTION);
    
    /*------------------------------------------------------------------
     * Copy the values into the struct for the laser readings from the devices.
     *------------------------------------------------------------------*/
    if ( USE_FRONT_LASER) {

      frontLaserReading.new                = TRUE;
      gettimeofday( &frontLaserReading.time,  0);
      frontLaserReading.startAngle         = FRONT_LASER_START_ANGLE;
      if ( laser->f_numberOfReadings > 1)
	frontLaserReading.angleResolution    = DEG_180 / (laser->f_numberOfReadings - 1);
      else 
	frontLaserReading.angleResolution    = DEG_360;
      
      /* If the number of readings has changed we allocate new memory. */
      if ( frontLaserReading.numberOfReadings != laser->f_numberOfReadings) {
	
	if ( frontLaserReading.numberOfReadings > 0)
	  free ( frontLaserReading.reading);
	
	frontLaserReading.numberOfReadings   = laser->f_numberOfReadings;
	
	if ( frontLaserReading.numberOfReadings > 0) 
	  frontLaserReading.reading = (float*) 
	  malloc( frontLaserReading.numberOfReadings * sizeof( float));
	else
	  frontLaserReading.reading = (float *) NULL; 
      }
      
      for ( i = 0; i < frontLaserReading.numberOfReadings; i++) 
	frontLaserReading.reading[i] = MAX( ROB_RADIUS - FRONT_LASER_OFFSET + 0.001,
					    (float) laser->f_reading[i]);
    }
    else
      frontLaserReading.new = FALSE;
    
    if ( USE_REAR_LASER) {
      
      rearLaserReading.new                = TRUE;
      gettimeofday( &rearLaserReading.time,  0);
      rearLaserReading.startAngle         = REAR_LASER_START_ANGLE;
      if ( laser->r_numberOfReadings > 1)
	rearLaserReading.angleResolution    = DEG_180 / (laser->r_numberOfReadings - 1);
      else 
	rearLaserReading.angleResolution    = DEG_360;
      
      /* If the number of readings has changed we allocate new memory. */
      if ( rearLaserReading.numberOfReadings != laser->r_numberOfReadings) {
	
	if ( rearLaserReading.numberOfReadings > 0)
	  free ( rearLaserReading.reading);
	
	rearLaserReading.numberOfReadings   = laser->r_numberOfReadings;
	
	if ( rearLaserReading.numberOfReadings > 0) 
	  rearLaserReading.reading = (float*) 
	  malloc( rearLaserReading.numberOfReadings * sizeof( float));
	else
	  rearLaserReading.reading = (float *) NULL; 
      }
      for ( i = 0; i < rearLaserReading.numberOfReadings; i++) {
	rearLaserReading.reading[i] = MAX( ROB_RADIUS - REAR_LASER_OFFSET + 0.001,
					    (float) laser->r_reading[i]);
      }
    }
    else
      rearLaserReading.new = FALSE;
    
    /* Fire the handlers to handle the new readings. */
    FireHandler( laser_handlers, SINGLE_LASER_REPORT, (Pointer) NULL);
    FireHandler( laser_handlers, COMPLETE_LASER_REPORT, (Pointer) NULL);
  }
  
  tcxFree("SIMULATOR_message_to_laser", laser);
}


/*****************************************************************************
 *
 * FUNCTION: BOOLEAN closeLaserDevice(void);
 *
 * DESCRIPTION: Close the laser device.
 *
 * OUTPUTS: 
 *
 *****************************************************************************/
static void
closeLaserDevice( LASER_TYPE* laserDevice)
{
  close(laserDevice->dev.fd);
  laserDevice->dev.fd = -1;
}


#define CRC16_GEN_POL 0x8005
#define CRC16_GEN_POL0 0x80
#define CRC16_GEN_POL1 0x05

static
unsigned short
computeCRC(unsigned char* CommData, unsigned short uLen)
{
  unsigned char abData[2];
  unsigned char uCrc16[2];

  
  abData[1] = 0;
  abData[0] = 0;
  uCrc16[0] = 0;
  uCrc16[1] = 0;

  while(uLen--) {
    abData[0] = abData[1];
    abData[1] = *CommData++;

    if(uCrc16[0] & 0x80) {
      uCrc16[0] <<= 1;
      if (uCrc16[1] & 0x80)
	uCrc16[0] |= 0x01;
      uCrc16[1] <<= 1;

      uCrc16[0] ^= CRC16_GEN_POL0;
      uCrc16[1] ^= CRC16_GEN_POL1;
    }
    else{
      uCrc16[0] <<= 1;
      if (uCrc16[1] & 0x80)
	uCrc16[0] |= 0x01;
      uCrc16[1] <<= 1;
    }
    uCrc16[0] ^= abData[0];
    uCrc16[1] ^= abData[1];
  }
  
  return (((unsigned short) uCrc16[0]) * 256
	  + ((unsigned short)  uCrc16[1]));
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void
LASER_timeoutHnd(void)
{
  ;
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define MAX_LASER_MESSAGE_LENGTH 400

static void
LASER_outputHnd( int fd, long chars_available, LASER_TYPE* laserDevice)
{
  static unsigned char buffer[NUMBER_OF_LASERS][LASER_BUFFER_SIZE+1];
  static unsigned char *startPos[NUMBER_OF_LASERS];
  static unsigned char *endPos[NUMBER_OF_LASERS];
  static unsigned char *startPos2;
  int numRead = 0;
  int length;
  unsigned int check1, check2;
  int length_error, header_error, num_chars_removed;
  int local_length_error, local_header_error;
  static int error_free_message[NUMBER_OF_LASERS];
  static int firstPosition[NUMBER_OF_LASERS];
  int laserNumber = laserDevice->laserNumber;
  LASER_reading* scan = laserDevice->scan;
  
  /* We have to store the positions of the scans to interpolate the
   * next position. */
  Point actualPosition;
  float actualRot;

  static BOOLEAN firstTime = TRUE;
  if ( firstTime) {
    int i;
    
    setStartTime( FRONT_LASER);
    setStartTime( REAR_LASER);
    
    for ( i = 0; i < NUMBER_OF_LASERS; i++) {
      startPos[i]  = &(buffer[i][0]);
      endPos[i]  = &(buffer[i][0]);
      error_free_message[i] = FALSE;
      firstPosition[i] = TRUE;
      commandwritten[i] = FALSE;
    }
    firstTime = FALSE;
  }

  while (chars_available > 0) {
    
    if ((startPos[laserNumber] == endPos[laserNumber]) || commandwritten[laserNumber])
      { 
	if (communication_verbose)
	  fprintf(stderr,"Buffer cleared \n");
	startPos[laserNumber] = endPos[laserNumber] = buffer[laserNumber];
	bzero(buffer[laserNumber], LASER_BUFFER_SIZE+1); /* 8196 +1*/
	commandwritten[laserNumber] = FALSE;
      }
    
    
    /* read in the output. */
    if (communication_verbose)
      fprintf(stderr, "readN: %d (%d %d %d)\n",
	      MIN(chars_available,(LASER_BUFFER_SIZE - 
				   (endPos[laserNumber] - startPos[laserNumber]))),
	      chars_available, LASER_BUFFER_SIZE, endPos[laserNumber] - startPos[laserNumber]);

    
    numRead = readN( &(laserDevice->dev), endPos[laserNumber], 
		    MIN(chars_available,
			(LASER_BUFFER_SIZE - 
			 (endPos[laserNumber] - startPos[laserNumber]))));

    if(communication_verbose)
      fprintf(stderr, "Read: %d\n",numRead);


    endPos[laserNumber] += numRead;

    if (numRead == 0)
      fprintf(stderr, "\n\t??? empty message ???\n");


    /*
     * Show buffer
     */
    
    length = 0;
    if (communication_verbose)
      fprintf(stderr, "\nBuffer: [ %d",endPos[laserNumber]-startPos[laserNumber]);
    for (startPos2 = startPos[laserNumber]; startPos2 < endPos[laserNumber]; startPos2++){
      if (*startPos2 == 0x02){
	if (communication_verbose)
	  fprintf(stderr, " (%d)\n", length);
	length = 0;
      }
      if (communication_verbose)
	if (length < 10)
	  fprintf(stderr, " %.2x", *startPos2);
      length++;
    }
    if (communication_verbose)
      fprintf(stderr, "]\n"); 


    /*
     * Remove irrelevant initial bytes
     */

    
    
    while ((*startPos[laserNumber] != 0x02 && startPos[laserNumber] + 1 < endPos[laserNumber])
	   || (endPos[laserNumber] - startPos[laserNumber] >= 6 &&
	       (((int) *(startPos[laserNumber]+3)) * 256) + ((int) *(startPos[laserNumber]+2)) + 6 >
	       MAX_LASER_MESSAGE_LENGTH)){
      if (communication_verbose)
	fprintf(stderr, " <<%.2x>>", *startPos[laserNumber]);
      startPos[laserNumber]++;
    }

    /*
     * Calculate length
     */
    
    if (endPos[laserNumber] - startPos[laserNumber] < 6)
      length = 0;
    else
      length = (((int) *(startPos[laserNumber]+3)) * 256) + ((int) *(startPos[laserNumber]+2)) + 6;


    /*
     * Check if we got a complete message here
     */
    
    while (endPos[laserNumber] - startPos[laserNumber] >= 6 && /* minimum length */
	   endPos[laserNumber] - startPos[laserNumber] >= length){ /* complete message */
      
      /*
       * Okay, we got a complete message, let's parse it
       */
      
      if (communication_verbose){
	fprintf(stderr, "\nmessage length= %d (%.2x)\n", length, length); 
	fprintf(stderr, "Parsing: [");
	for (startPos2 = startPos[laserNumber]; startPos2 < endPos[laserNumber]; startPos2++)
	  fprintf(stderr, " %.2x", *startPos2);
	fprintf(stderr, "]\n"); 
      }
      

      /*
       * Print the checksum
       */

      check1 = (*(startPos[laserNumber]+length-1) * 256) + *(startPos[laserNumber]+length-2);
      check2 = computeCRC((startPos[laserNumber]), (unsigned short) length-2);

      if (check1 != check2)
	fprintf(stderr, "Laser checksum error: %d %d (length=%d)\n",
		check1, check2, length);
      
      else {

	/* We interpolate between the position when the last reading came
	 * and the actual position. */
	updateActualPosition( &actualPosition, &actualRot,
			      DONT_CONSIDER_DIRECTION);
	
	/* Not too much time expired since the last reading. */
	if ( timeExpired( laserNumber) < 0.5 && ! firstPosition[laserNumber]) {

#define LAST_POS_WEIGHT 0.0
#define CURRENT_POS_WEIGHT 1.0
	    /* Do the interpolation. */
	  scan->rPos.x = CURRENT_POS_WEIGHT * actualPosition.x
	    + LAST_POS_WEIGHT * positionAtLastRequest[laserNumber].x;
	  scan->rPos.y = CURRENT_POS_WEIGHT * actualPosition.y
	    + LAST_POS_WEIGHT * positionAtLastRequest[laserNumber].y;
	  if ( fabs( actualRot - rotationAtLastRequest[laserNumber]) < DEG_180)
	    scan->rRot = (CURRENT_POS_WEIGHT * actualRot
			  + LAST_POS_WEIGHT * rotationAtLastRequest[laserNumber]);
	  else {
	    if ( actualRot < DEG_180) 
	      scan->rRot =
		normed_angle( CURRENT_POS_WEIGHT * (actualRot + DEG_360) +
			      LAST_POS_WEIGHT * rotationAtLastRequest[laserNumber]);
	    else
	      scan->rRot =
		normed_angle( CURRENT_POS_WEIGHT * actualRot +
			      LAST_POS_WEIGHT * (rotationAtLastRequest[laserNumber] + DEG_360));
	  }
	}
	else {
	  /* Position too old. Use the actual position. */
	  updateActualPosition( &(scan->rPos),
				&(scan->rRot),
				DONT_CONSIDER_DIRECTION);
	  if ( sensors_verbose)
	    fprintf( stderr, "too old : %f      ---> ",
		     timeExpired( laserNumber));

	  firstPosition[laserNumber] = FALSE;
	}
	  
	/* Set timer for the next reading. */
	setStartTime( laserNumber);
	
	ProcessLine( scan, startPos[laserNumber]+4, length - 6);
      }


      /*
       * Ask for next sensor value
       */
      laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS);
	
      
      /*
       * Okay, message parsed, remove it
       */

      startPos[laserNumber] += length;

      
      /*
       * Show the remaining buffer
       */
      if (communication_verbose){
	fprintf(stderr, "\nRemaining buffer Parsing: [");
	for (startPos2 = startPos[laserNumber]; startPos2 < endPos[laserNumber]; startPos2++)
	  fprintf(stderr, " %.2x", *startPos2);
	fprintf(stderr, "]\n"); 
      }
      
    }
    
    /* 
     * Fix up the buffer. Throw out any consumed lines.
     */
    if (startPos[laserNumber] >= endPos[laserNumber]){	/* parsed the whole thing, 
				 * just clean it all up */
      bzero(buffer[laserNumber], LASER_BUFFER_SIZE+1);
      startPos[laserNumber] = endPos[laserNumber] = buffer[laserNumber];
    }
    else if (startPos[laserNumber] != buffer[laserNumber]){/* slide it back and wait for 
				  * more characters */
      bcopy(startPos[laserNumber], buffer[laserNumber], (endPos[laserNumber] - startPos[laserNumber]));
      endPos[laserNumber] = buffer[laserNumber] + (endPos[laserNumber] - startPos[laserNumber]);
      startPos[laserNumber] = buffer[laserNumber];
    }
    
    /*
     * Show the remaining buffer
     */
    if ( communication_verbose){
      fprintf(stderr, "\nRRemaining buffer: [");
      for (startPos2 = startPos[laserNumber]; startPos2 < endPos[laserNumber]; startPos2++)
	fprintf(stderr, " %.2x", *startPos2);
      fprintf(stderr, "]\n"); 
    }
    
    /*
     * Check if new characters arrived in the meantime
     */
    
    chars_available = numChars(fd);
    
  }
}


void
requestNextLaserScan( int laserNumber)
{
  if ( laserNumber == FRONT_LASER)
    laserRequestReading( &frontLaserDevice, NUMBER_OF_LASER_READINGS);
  else
    laserRequestReading( &rearLaserDevice, NUMBER_OF_LASER_READINGS);
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void
FRONT_LASER_outputHnd( int fd, long chars_available)
{
  LASER_outputHnd( fd, chars_available, &frontLaserDevice);
}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

void
REAR_LASER_outputHnd( int fd, long chars_available)
{
  LASER_outputHnd( fd, chars_available, &rearLaserDevice);
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/*	Function Name: ProcessLine
 *	Arguments:     line -- output line from the laser
 *	Description:   process the laser output
 *	Returns: 
 */


static void 
ProcessLine( LASER_reading* scan, unsigned char *line, int length) 
{
  unsigned char type;
  unsigned char *ptr = line;
  int body_length, i;
  int num_measurements;
  struct timeval actual_time;
  float time_diff;

  /* To check wether the scans of both finders are complete. */
  BOOLEAN bothComplete     = FALSE;
  static BOOLEAN frontRead = ! USE_FRONT_LASER;
  static BOOLEAN rearRead  = ! USE_REAR_LASER;

  type        = *ptr++;
  body_length = length -1;

  if (type == 0xb0){
    num_measurements = *(ptr+1) * 256 + *ptr;
    if (sensors_verbose){
      fprintf(stderr, "\n----------- SENSOR MEASUREMENT --------\n");
      fprintf(stderr, "  num_measurements      = %d (%.2x%.2x)\n",
	      num_measurements, *ptr, *(ptr+1));
    }
    
    ptr += 2;

    if (num_measurements != NUMBER_OF_LASER_READINGS)
      fprintf(stderr, "WARNING: Unexpected number of sensor values: %d\n", 
	      num_measurements);
    else{
      scan->new = TRUE;
      for (i = 0; i < num_measurements; i++){
	scan->reading[i]    = ((*(ptr+1)) & 0x1f) * 256 + *ptr;
	scan->blendung[i] = ((int) ((*(ptr+1)) & 0x20)) / 32;
	scan->wfv[i]      = ((int) ((*(ptr+1)) & 0x40)) / 64;
	scan->sfv[i]      = ((int) ((*(ptr+1)) & 0x80)) / 128;

	if (sensors_verbose)
	  fprintf(stderr, "%d: %6.3f (blend=%d wfv=%d sfv=%d) (%.2x%.2x)\n",
		  i, scan->reading[i], scan->blendung[i], 
		  scan->wfv[i], scan->sfv[i], *ptr, *(ptr+1));
	if (scan->reading[i] > LASER_MAX_RANGE)
	  scan->reading[i] = LASER_MAX_RANGE;
	ptr += 2;
      }

      /*
       * Check timing
       */
      
      gettimeofday(&actual_time, NULL);
      if (scan->time.tv_sec != 0){
	time_diff = ((float) (actual_time.tv_sec - scan->time.tv_sec))
	  + (((float) (actual_time.tv_usec - scan->time.tv_usec)) 
	     / 1000000.0);
	if (sensors_verbose)
	  fprintf(stderr, "Time: %6.3f sec\n", time_diff);
      }
      scan->time.tv_sec  = actual_time.tv_sec;
      scan->time.tv_usec = actual_time.tv_usec;
      
      /* Check wether both range finders have been read. */
      if ( USE_FRONT_LASER && frontLaserReading.new)
	frontRead = TRUE;
      if ( USE_REAR_LASER && rearLaserReading.new)
	rearRead  = TRUE;
      
      if ( frontRead && rearRead) {
	bothComplete = TRUE;
	frontRead    = ! USE_FRONT_LASER;
	rearRead     = ! USE_REAR_LASER;
      }	
      
      /* Done. */ 
      FireHandler(laser_handlers, SINGLE_LASER_REPORT, (Pointer) NULL);
      if ( bothComplete) {
	FireHandler(laser_handlers, COMPLETE_LASER_REPORT, (Pointer) NULL);
      }
    }
  }

  else
    if(type == 0xb1)
      ProcessLaserStatus(ptr++,length);  /* STATUS-Meldung */
    else
      fprintf(stderr, "WARNING: Unknown message type: %.2x\n", type);
}





/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

static int 
WriteLaserCommand( LASER_TYPE* laserDevice,
		   unsigned char command, char *argument, int arg_length)
{
  unsigned char buffer[DEFAULT_LINE_LENGTH];
  int      pos = 0;
  int      i, n, answer;
  unsigned short check;
  unsigned short length = 0;

  /*
   * SYNC CHARS
   */

  buffer[pos++] = 0x02;

  /*
   * ADDRESS
   */

  buffer[pos++] = 0x00;		/* broadcast */

  /*
   * MESSAGE LENGTH
   */

  length = 1 + arg_length;


  buffer[pos++] = length & 0x00ff;
  buffer[pos++] = length / 256;	/* I wonder if that works */

  /*
   * COMMAND
   */
  
  
  buffer[pos++] = command;

  
  /*
   * ARGUMENT
   */
  
  if (arg_length > 0)
    for (i = 0; i < arg_length; i++)
      buffer[pos++] = argument[i];
      
  
  /*
   * CHECKSUM
   */


  check = computeCRC(buffer, length + 4);
  buffer[pos++] = check & 0x00ff;
  buffer[pos++] = check / 256;
 
  if (communication_verbose){
    fprintf(stderr, "\nWriteLaserCommand (%d) [", arg_length);
    for (i = 0; i < pos; i++)
      fprintf(stderr, " %.2x", buffer[i]);  
    fprintf(stderr, "]\n");
  }
  
  /* set the timeout, if one is available */
  if (laserDevice->dev.setTimeout != NULL)
    (* laserDevice->dev.setTimeout)(&(laserDevice->dev), LASER_DEVICE_TIMEOUT);
  
  flushChars(&(laserDevice->dev));
  
  /*
   * WRITE TO PORT
   */
 
  n = ((int) writeN(&(laserDevice->dev), buffer, pos));

  answer = waitforACK( laserDevice);  /* Software-Handshake */

  if (sensors_verbose) {
    if (answer == NOT_ACKNOWLEDGED)
      fprintf(stderr, "NAK-Fehler beim Senden von: %.2x (%.2x)\n", command,*(buffer+5));
    else if (answer == TIME_OUT)
      fprintf(stderr, "Timeout-Fehler beim Senden von: %.2x (%.2x)\n", command,*(buffer+5));
  }
  
  commandwritten[laserDevice->laserNumber] = TRUE; /* internal buffer */

  return answer;
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

static void
sendInitToLaser( LASER_TYPE* laserDevice) 
{
  char arg[128];

  adaptBaudRate( laserDevice);

  laser_38400_baud( laserDevice); 
}


static void
adaptBaudRate( LASER_TYPE* laserDevice)
{
  m_setparms(laserDevice->dev.fd, "9600", "EVEN", "8", 0, 0);
  if ( laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS) != TIME_OUT)
    return;
  
  m_setparms(laserDevice->dev.fd, "19200", "EVEN", "8", 0, 0);
  if ( laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS) != TIME_OUT)
    return;

  m_setparms(laserDevice->dev.fd, "38400", "EVEN", "8", 0, 0);
  if ( laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS) != TIME_OUT)
    return;
}


void
laser_38400_baud( LASER_TYPE* laserDevice)
{
  char arg[9];

  arg[0] = 0x00;
  arg[1] = 'S';
  arg[2] = 'I';
  arg[3] = 'C';
  arg[4] = 'K';
  arg[5] = '_';
  arg[6] = 'P';
  arg[7] = 'L';
  arg[8] = 'S';
  WriteLaserCommand( laserDevice, 0x20, arg, 9);
  sleep(3);

  arg[0] = 0x40;
  WriteLaserCommand( laserDevice, 0x20, arg, 1);
  sleep(3);

  arg[0] = 0x25;
  WriteLaserCommand( laserDevice, 0x20, arg, 1);
  sleep(3);

  m_setparms(laserDevice->dev.fd, "38400", "EVEN", "8", 0, 0);
  sleep(1);

  laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS);
}


void
laser_19200_baud( LASER_TYPE* laserDevice)
{
  char arg[9];

  arg[0] = 0x00;
  arg[1] = 'S';
  arg[2] = 'I';
  arg[3] = 'C';
  arg[4] = 'K';
  arg[5] = '_';
  arg[6] = 'P';
  arg[7] = 'L';
  arg[8] = 'S';
  WriteLaserCommand( laserDevice, 0x20, arg, 9);
  sleep(3);


  arg[0] = 0x41;
  WriteLaserCommand( laserDevice, 0x20, arg, 1);
  sleep(3);

 
  arg[0] = 0x25;
  WriteLaserCommand( laserDevice, 0x20, arg, 1);
  sleep(3);

  m_setparms(laserDevice->dev.fd, "19200", "EVEN", "8", 0, 0);
  sleep(1);

  laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS);
}

void
laser_9600_baud( LASER_TYPE* laserDevice)
{
  char arg[9];

  arg[0] = 0x00;
  arg[1] = 'S';
  arg[2] = 'I';
  arg[3] = 'C';
  arg[4] = 'K';
  arg[5] = '_';
  arg[6] = 'P';
  arg[7] = 'L';
  arg[8] = 'S';
  WriteLaserCommand( laserDevice, 0x20, arg, 9);
  sleep(3);

  arg[0] = 0x42;

  WriteLaserCommand( laserDevice, 0x20, arg, 1);
  sleep(3);

  arg[0] = 0x25;
  WriteLaserCommand( laserDevice, 0x20, arg, 1);
  sleep(3);

  m_setparms(laserDevice->dev.fd, "9600", "EVEN", "8", 0, 0);
  sleep(2);

  laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS);
}


static int
laserRequestReading( LASER_TYPE* laserDevice, int num)
{
  unsigned char sens_text[128];

  /* Save the corresponding position of the robot. */
  updateActualPosition( &(positionAtLastRequest[laserDevice->laserNumber]),
			&(rotationAtLastRequest[laserDevice->laserNumber]),
			DONT_CONSIDER_DIRECTION);
  
  sens_text[0] = 0x05;
  sens_text[1] = (unsigned char) num;
  return WriteLaserCommand( laserDevice, 0x30, sens_text, 2);
}


/* Fragt den Laser-Status ab und wertet ihn via LASER_outputHnd aus */
 
static void
requestLaserStatus(LASER_TYPE* laserDevice)
{
  long int answer;
  char arg[1];
  
  arg[0] = 0x10;

  WriteLaserCommand( laserDevice, 0x20, arg ,1);
  sleep(2);
  WriteLaserCommand( laserDevice, 0x31, NULL ,0);
  sleep(1);
  LASER_outputHnd( laserDevice->dev.fd,130, laserDevice);
  arg[0] = 0x25;
  WriteLaserCommand( laserDevice, 0x20, arg ,1);
  sleep(2);
  laserRequestReading( laserDevice, NUMBER_OF_LASER_READINGS);
}

/* Wertet die Antwort des Lasers auf request_laser_status aus */

static void
ProcessLaserStatus(unsigned char *infos, int length)
{
  unsigned short baud;
  unsigned char text[7];
  
  text[0]=*infos++;
  text[1]=*infos++;
  text[2]=*infos++;
  text[3]=*infos++;
  text[4]=*infos++;
  text[5]=*infos++;
  text[6]=*infos++;
  fprintf(stdout,"Version: %s\n",text);
  

  baud = *(infos+108)+(*(infos+109))*256;

  switch(baud)
    {
    case 0x8019 : fprintf(stdout,"38400 Baud\n"); break; 
    case 0x8033 : fprintf(stdout,"19200 Baud\n"); break;
    case 0x8067 : fprintf(stdout,"9600 Baud\n"); break;
    default : fprintf(stdout,"Baudrate nicht korrekt ermittelbar: %.2x\n",baud); 
    }
}

/* 
Waits for acknowledge from the laser.
Laser answer if correct: ACK (006H) else NAK (015H).
Returns 0=ACK, or 1=NAK, or 2=Timeout (>1 sec).
*/

static int
waitforACK( LASER_TYPE* laserDevice)
{
  unsigned char answer; 
  short r_wert = INITIAL_ACKNOWLEDGE;

  setStartTime( EVERYBODIES_TIMER);

  do {

    if (numChars(laserDevice->dev.fd) > 0) {
      readN(&(laserDevice->dev), &answer, 1);
      if (answer == 0x06)
	r_wert = ACKNOWLEDGED;
      else
	if (answer == 0x15)
	  r_wert = NOT_ACKNOWLEDGED;
    }
    else if ( timeExpired( EVERYBODIES_TIMER) > 0.1) 
      r_wert = TIME_OUT;
  } while (r_wert < 0);

  return(r_wert);
}




/*****************************************************************************
 *
 * FUNCTION: initLaserReadingStructures
 *
 *****************************************************************************/
static void
initLaserReadingStructures(void)
{
  int i;
  struct timeval now;

  gettimeofday( &now, 0);

  /* Initialize the global structs for the laser readings. */

  /* FRONT LASER */
  frontLaserReading.new = FALSE;

  if ( USE_FRONT_LASER && ! frontLaserDevice.dev.use_simulator &&
       ! use_laser_server) {
    frontLaserReading.numberOfReadings = NUMBER_OF_LASER_READINGS;
    frontLaserReading.startAngle       = FRONT_LASER_START_ANGLE;
    frontLaserReading.angleResolution  =
      DEG_180 / (float) (NUMBER_OF_LASER_READINGS - 1);
    frontLaserReading.time             = now;

    /* Allocate memroy. */
    if ( frontLaserReading.reading == NULL)
      frontLaserReading.reading          = (float*)
      malloc( NUMBER_OF_LASER_READINGS * (sizeof (float)));
    if ( frontLaserReading.blendung == NULL)
      frontLaserReading.blendung          = (int*)
      malloc( NUMBER_OF_LASER_READINGS * (sizeof (int)));
    if ( frontLaserReading.wfv == NULL)
      frontLaserReading.wfv          = (int*)
      malloc( NUMBER_OF_LASER_READINGS * (sizeof (int)));
    if ( frontLaserReading.sfv == NULL)
      frontLaserReading.sfv          = (int*)
      malloc( NUMBER_OF_LASER_READINGS * (sizeof (int)));
  }
  
  /* REAR LASER */
  rearLaserReading.new = FALSE;
  
  if ( USE_REAR_LASER && ! rearLaserDevice.dev.use_simulator &&
       ! use_laser_server) {
    rearLaserReading.numberOfReadings = NUMBER_OF_LASER_READINGS;
    rearLaserReading.startAngle       = REAR_LASER_START_ANGLE;
    rearLaserReading.angleResolution  = DEG_180
      / (float) (NUMBER_OF_LASER_READINGS - 1);
    rearLaserReading.time             = now;  
    
    /* Allocate memory. */
    if ( rearLaserReading.reading == NULL)
      rearLaserReading.reading          = (float*)
      malloc( NUMBER_OF_LASER_READINGS * (sizeof (float)));
    if ( rearLaserReading.blendung == NULL)
      rearLaserReading.blendung          = (int*)
      malloc( NUMBER_OF_LASER_READINGS * (sizeof (int)));
    if ( rearLaserReading.wfv == NULL)
      rearLaserReading.wfv          = (int*)
      malloc( NUMBER_OF_LASER_READINGS * (sizeof (int)));
    if ( rearLaserReading.sfv == NULL)
      rearLaserReading.sfv          = (int*)
      malloc( NUMBER_OF_LASER_READINGS * (sizeof (int)));

    /* Now set the values to undefined. */
    for (i=0; i<frontLaserReading.numberOfReadings; i++)
      frontLaserReading.reading[i] = -1.0;

    for (i=0; i<rearLaserReading.numberOfReadings; i++)
      rearLaserReading.reading[i] = -1.0;
  }
}

/*****************************************************************************
 *
 * FUNCTION: void signalLaser(void);
 *
 * DESCRIPTION:
 *
 * Interrupt handler for the laser.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

static void
signalLaser(void)
{
  fprintf(stderr,"shutting down the laser\n");
}




/*****************************************************************************
 *
 * FUNCTION: LASER_SERVER_sweep_reply_handler
 *
 * DESCRIPTION: for laser server
 *
 * 
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

void 
LASER_SERVER_sweep_reply_handler( TCX_REF_PTR            ref,
				  LASER_SERVER_sweep_reply_ptr sweep)
{
  int i;

/*   fprintf(stderr, " [%d %d] ", sweep->numLaser, sweep->numberLasers); */

  if ( use_laser) {
    
        /* Get the actual position of the robot. */
    updateActualPosition( &(frontLaserReading.rPos),
			  &(frontLaserReading.rRot),
			  DONT_CONSIDER_DIRECTION);
    updateActualPosition( &(rearLaserReading.rPos),
			  &(rearLaserReading.rRot),
			  DONT_CONSIDER_DIRECTION);
    

    /*------------------------------------------------------------------
     * Copy the values into the struct for the laser readings from the devices.
     *------------------------------------------------------------------*/

    if ( USE_FRONT_LASER && sweep->numLaser == FRONT_LASER) {

      frontLaserReading.new                = TRUE;
      gettimeofday( &frontLaserReading.time,  0);
      frontLaserReading.startAngle         = FRONT_LASER_START_ANGLE;
      if ( sweep->numberLasers > 1)
	frontLaserReading.angleResolution    = DEG_180 / (sweep->numberLasers - 1);
      else 
	frontLaserReading.angleResolution    = DEG_360;
      
      /* If the number of readings has changed we allocate new memory. */
      if ( frontLaserReading.numberOfReadings != sweep->numberLasers) {
	
	if ( frontLaserReading.numberOfReadings > 0)
	  free ( frontLaserReading.reading);
	
	frontLaserReading.numberOfReadings   = sweep->numberLasers;
	
	if ( frontLaserReading.numberOfReadings > 0) 
	  frontLaserReading.reading = (float*) 
	  malloc( frontLaserReading.numberOfReadings * sizeof( float));
	else
	  frontLaserReading.reading = (float *) NULL; 
      }
      
      for ( i = 0; i < frontLaserReading.numberOfReadings; i++) 
	frontLaserReading.reading[i] = MAX( ROB_RADIUS - FRONT_LASER_OFFSET + 0.001,
					    (float) sweep->value[i]);
    }
    else
      frontLaserReading.new = FALSE;
    
    if ( USE_REAR_LASER && sweep->numLaser == REAR_LASER) {
      
      rearLaserReading.new                = TRUE;
      gettimeofday( &rearLaserReading.time,  0);
      rearLaserReading.startAngle         = REAR_LASER_START_ANGLE;
      if ( sweep->numberLasers > 1)
	rearLaserReading.angleResolution    = DEG_180 / (sweep->numberLasers - 1);
      else 
	rearLaserReading.angleResolution    = DEG_360;
      
      /* If the number of readings has changed we allocate new memory. */
      if ( rearLaserReading.numberOfReadings != sweep->numberLasers) {
	
	if ( rearLaserReading.numberOfReadings > 0)
	  free ( rearLaserReading.reading);
	
	rearLaserReading.numberOfReadings   = sweep->numberLasers;
	
	if ( rearLaserReading.numberOfReadings > 0) 
	  rearLaserReading.reading = (float*) 
	  malloc( rearLaserReading.numberOfReadings * sizeof( float));
	else
	  rearLaserReading.reading = (float *) NULL; 
      }
      for ( i = 0; i < rearLaserReading.numberOfReadings; i++) {
	rearLaserReading.reading[i] = MAX( ROB_RADIUS - REAR_LASER_OFFSET + 0.001,
					    (float) sweep->value[i]);
      }
    }
    else
      rearLaserReading.new = FALSE;
    
    /* Fire the handlers to handle the new readings. */
    FireHandler( laser_handlers, SINGLE_LASER_REPORT, (Pointer) NULL);
    FireHandler( laser_handlers, COMPLETE_LASER_REPORT, (Pointer) NULL);
  }
  
  tcxFree("LASER_SERVER_sweep_reply", sweep);
}
