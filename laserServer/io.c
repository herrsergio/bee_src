
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include "tcx.h"
#include "tcxP.h"
#include "global.h"
#include "devUtils.h"
#include "io.h"
#include "mainlaser.h"

int communication_verbose = 0;

int sensors_verbose       = 0;

LaserSensorValueType LaserSensors;


LaserDeviceType LaserDevice = { 
  { FALSE,
    { "", DEFAULT_PORT},
    /* 14 is for a baud rate of 19200 */
    { "/dev/cur6", 13}, /* gets now set in mainlaser from .ini (swa) */
    LASER_DEV_NAME,
    -1,
    TRUE,
    FALSE,
    (FILE *) NULL,
    (fd_set *) NULL,
    (DEVICE_OUTPUT_HND) LaserHandleOutput,
    LaserHandleTimeout,  
    (DEVICE_SET_TIMEOUT)  setTimeout,  
    (DEVICE_CANCEL_TIMEOUT) cancelTimeout,
    (void (*)(void)) NULL,  
    {0, 0},
    {LONG_MAX, 0},
    {0x2FFFFFFF, 0},
    (void (*)(void)) NULL,
    FALSE
  }
};

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
static unsigned short
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

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
static unsigned short
computeCRCoriginal(unsigned char* CommData, unsigned short uLen)
{
  unsigned short uCrc16;
  unsigned char abData[2];
  unsigned char *p_uCrc16[2];

  p_uCrc16[0] = (unsigned char *) &uCrc16;
  p_uCrc16[1] = p_uCrc16[0] + 1;
  
  uCrc16 = 0;
  abData[1] = 0;
  abData[0] = 0;

  while(uLen--) {
    abData[0] = abData[1];
    abData[1] = *CommData++;

    if(uCrc16 & 0x8000) {
      uCrc16 <<= 1;
      uCrc16 ^= CRC16_GEN_POL;
    }
    else{
      uCrc16 <<= 1;
    }
    uCrc16 ^= *(unsigned short *) &abData;
  }
  
  return (uCrc16);
}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void LaserHandleOutput(int fd, long chars_available)
{
  static unsigned char buffer[LASER_BUFFER_SIZE+1];
  static unsigned char *startPos = buffer; /* position to start parsing from */
  static unsigned char *startPos2 = buffer; 
  static unsigned char *endPos = buffer; /* position to add more characters */
  int numRead = 0;
  int length;
  unsigned int check1, check2;

  while (chars_available > 0) {


    if (startPos == endPos)
      { 
	startPos = endPos = buffer;
	bzero(buffer, LASER_BUFFER_SIZE+1);
      }
    
    
    /* read in the output. */

    if (communication_verbose)
      fprintf( stderr, 
	       "readN: %ld (%ld %d %d)\n",
	       MIN( chars_available,(LASER_BUFFER_SIZE-(endPos - startPos))),
	       chars_available, 
	       LASER_BUFFER_SIZE, 
	       endPos-startPos );

    
    numRead = readN(&LaserDevice.dev, endPos, 
		    MIN(chars_available,(LASER_BUFFER_SIZE - 
					 (endPos - startPos))));
    
    endPos += numRead;
    
    if (numRead == 0)
      fprintf(stderr, "\n\t??? empty message ???\n");


    /*
     * Show buffer
     */
    
    length = 0;
    if (communication_verbose)
      fprintf(stderr, "\nBuffer: [");
    for (startPos2 = startPos; startPos2 < endPos; startPos2++){
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

    while ((*startPos != 0x02 && startPos + 1 < endPos)
	   || (endPos - startPos >= 6 &&
	       (((int) *(startPos+3)) * 256) + ((int) *(startPos+2)) + 6 >
	       MAX_LASER_MESSAGE_LENGTH)){
      if (communication_verbose)
	fprintf(stderr, " <%.2x>", *startPos);
      startPos++;
    }

    /*
     * Calculate length
     */
    
    if (endPos - startPos < 6)
      length = 0;
    else
      length = (((int) *(startPos+3)) * 256) + ((int) *(startPos+2)) + 6;


    /*
     * Check if we got a complete message here
     */
    
    while (endPos - startPos >= 6 && /* minimum length */
	   endPos - startPos >= length){ /* complete message */
      
      /*
       * Okay, we got a complete message, let's parse it
       */
      
      if (communication_verbose){
	fprintf(stderr, "\nmessage length= %d (%.2x)\n", length, length); 
	fprintf(stderr, "Parsing: [");
	for (startPos2 = startPos; startPos2 < endPos; startPos2++)
	  fprintf(stderr, " %.2x", *startPos2);
	fprintf(stderr, "]\n"); 
      }
      

      /*
       * Print the checksum
       */

      check1 = (*(startPos+length-1) * 256) + *(startPos+length-2);
      check2 = computeCRC((startPos), (unsigned short) length-2);

      if (check1 != check2)
	fprintf(stderr, "Laser checksum error: %d %d (length=%d)\n",
		check1, check2, length);
      
      else{
	LaserProcessOutputLine(startPos+4, length - 6);
      }


      /*
       * Ask for next sensor value
       */

      LasterRequestReading(NUMBER_LASERS);

      /*
       * Okay, message parsed, remove it
       */

      startPos += length;
 

      
      /*
       * Show the remaining buffer
       */
      if (communication_verbose){
	fprintf(stderr, "\nRemaining buffer: [");
	for (startPos2 = startPos; startPos2 < endPos; startPos2++)
	  fprintf(stderr, " %.2x", *startPos2);
	fprintf(stderr, "]\n"); 
      }
      
    }
    
    /* 
     * Fix up the buffer. Throw out any consumed lines.
     */
    if (startPos >= endPos){	/* parsed the whole thing, 
				 * just clean it all up */
      bzero(buffer, LASER_BUFFER_SIZE+1);
      startPos = endPos = buffer;
    }
    else if (startPos != buffer){/* slide it back and wait for 
				  * more characters */
      bcopy(startPos, buffer, (endPos - startPos));
      endPos = buffer + (endPos - startPos);
      startPos = buffer;
    }
    
    /*
     * Show the remaining buffer
     */
    if (0 && communication_verbose){
      fprintf(stderr, "\nRemaining buffer: [");
      for (startPos2 = startPos; startPos2 < endPos; startPos2++)
	fprintf(stderr, " %.2x", *startPos2);
      fprintf(stderr, "]\n"); 
    }
    
    /*
     * Check if new characters arrived in the meantime
     */
    chars_available = numChars(fd);
  }
}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void LaserHandleTimeout() {

/*   fprintf( stderr, "%s: received timeout\n", __FILE__ ); */

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void LaserShutdownLaser() {

  fprintf( stderr, "%s: shutting down the laser\n", __FILE__ );

}

/* ---------------------------------------------------------
 * Arguments:     line -- output line from the laser
 * Description:   process the laser output
 * --------------------------------------------------------*/
void LaserProcessOutputLine(unsigned char *line, int length) 
{
  unsigned char type;
  unsigned char *ptr = line;
  int body_length, i;
  int num_measurements;
  struct timeval actual_time;
  float time_diff;

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

    if (num_measurements != NUMBER_LASERS)
      fprintf(stderr, "WARNING: Unexpected number of sensor values: %d\n", 
	      num_measurements);
    else{
      LaserSensors.defined = 1;
      for (i = 0; i < num_measurements; i++){
	LaserSensors.values[i]    = ((*(ptr+1)) & 0x1f) * 256 + *ptr;
	LaserSensors.blendung[i] = ((int) ((*(ptr+1)) & 0x20)) / 32;
	LaserSensors.wfv[i]      = ((int) ((*(ptr+1)) & 0x40)) / 64;
	LaserSensors.sfv[i]      = ((int) ((*(ptr+1)) & 0x80)) / 128;
	if (sensors_verbose)
	  fprintf(stderr, "%d: %6.3f (blend=%d wfv=%d sfv=%d) (%.2x%.2x)\n",
		  i, LaserSensors.values[i], LaserSensors.blendung[i], 
		  LaserSensors.wfv[i], LaserSensors.sfv[i], *ptr, *(ptr+1));
	if ( LaserSensors.values[i] > MAX_LASER_RANGE )
	  LaserSensors.values[i] = MAX_LASER_RANGE;
	ptr += 2;
      }

      /*
       * Check timing
       */
      
      gettimeofday(&actual_time, NULL);
      if ( LaserSensors.time.tv_sec != 0){
	time_diff = ((float) (actual_time.tv_sec - LaserSensors.time.tv_sec))
	  + (((float) (actual_time.tv_usec - LaserSensors.time.tv_usec)) 
	     / 1000000.0);
	if (sensors_verbose)
	  fprintf(stderr, "Time: %6.3f sec\n", time_diff);
      }
      LaserSensors.time.tv_sec  = actual_time.tv_sec;
      LaserSensors.time.tv_usec = actual_time.tv_usec;
    }
  }

  else
    fprintf(stderr, "WARNING: Unknown message type: %.2x\n", type);
}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
int LaserWriteCommand(unsigned char command, char *argument, int arg_length)
{
  static unsigned char buffer[DEFAULT_LINE_LENGTH];
  int      pos = 0;
  int      i, n;
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
  
  flushChars(&(LaserDevice.dev));

  if (communication_verbose){
    fprintf(stderr, "\nLaserWriteCommand (%d) [", arg_length);
    for (i = 0; i < pos; i++)
      fprintf(stderr, " %.2x", buffer[i]);  
    fprintf(stderr, "]\n");
  }



  /*
   * WRITE TO PORT
   */

  n = ((int) writeN(&(LaserDevice.dev), buffer, pos));


  return n;


}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void LaserInit() {

  char arg[128];

  LaserWriteCommand(0x10, NULL, 0);
  sleep(3);
  arg[0] = 0x24;
  LaserWriteCommand(0x20, arg, 1);
  LaserSensors.defined = 0;
  LaserSensors.time.tv_sec  = 0;
  LaserSensors.time.tv_usec = 0;

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void LaserSetBaud19200() {

  char arg[128];

  arg[0] = 0x00;
  arg[1] = 'S';
  arg[2] = 'I';
  arg[3] = 'C';
  arg[4] = 'K';
  arg[5] = '_';
  arg[6] = 'P';
  arg[7] = 'L';
  arg[8] = 'S';

  fprintf( stderr, "%s: %s\n", __FILE__, __FUNCTION__ );
  fprintf( stderr, "DON'T USE THIS FUNCTION!!!\n" );
  return;

  LaserWriteCommand(0x20, arg, 9);
/*   fprintf(stderr, "-a-"); */
  sleep(3);


  arg[0] = 0x41;
  LaserWriteCommand(0x20, arg, 1);
  m_setparms(LaserDevice.dev.fd, "19200", "EVEN", "8", 0, 0);
/*   fprintf(stderr, "-b-"); */
  sleep(3);

 
  arg[0] = 0x24;
  LaserWriteCommand(0x20, arg, 1);
/*   fprintf(stderr, "-c-"); */
  sleep(3);

  LasterRequestReading(NUMBER_LASERS);
/*   fprintf(stderr, "-d-"); */

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void LaserSetBaud9600() {

  char arg[1];

  arg[0] = 0x42;

  LaserWriteCommand(0x20, arg, 1);
  m_setparms(LaserDevice.dev.fd, "9600", "EVEN", "8", 0, 0);
  sleep(1);
  LasterRequestReading(NUMBER_LASERS);
}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void LasterRequestReading(int num) {

  unsigned char sens_text[128];

  sens_text[0] = 0x05;
  sens_text[1] = (unsigned char) num;
  LaserWriteCommand(0x30, sens_text, 2);
}

/*
 * $Log: io.c,v $
 * Revision 1.4  1997/09/19 00:59:22  swa
 * The laserServer now honours the entries in beeSoft.ini, that is device and
 * baudrate. Internally however, it only works with 9600 baud. It does some
 * moderate error checking on the entries.
 *
 * Revision 1.3  1997/08/07 03:50:20  swa
 * Fixed a bunch of bugs. Still not working.
 *
 * Revision 1.2  1997/08/07 02:45:50  swa
 * First version of a laserServer. Compiles nicely, yet produces nothing useful. ;)
 * Just to copy things to a safe place ...
 *
 * Revision 1.1  1997/08/06 15:12:32  swa
 * Very first and incomplete version of a laserServer. No TCX comm yet.
 *
 *
 */
