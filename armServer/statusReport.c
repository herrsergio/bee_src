
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

/****************************************************************/
/*                                                              */
/*  Originally written mjc.  Rewritten by jmk.                  */
/*  FILE: BinComm.C                                             */
/*  DATE: Tue Jun 14 15:10:59 1994                              */
/*                                                              */
/****************************************************************/

#include <rai.h>  /* raigettime, raisettime */ 
#include <mcpStatus.h>  /* raigettime, raisettime */ 
#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <bUtils.h>
#include "arm.h"

#define MAX_CALLBACKS 20
typedef void (*func_ptr_type)(statusReportType*);
typedef void (*wd_func_ptr_type)(void);

void* callbacks[MAX_CALLBACKS];

#undef DEBUG


statusReportType* initStatusStructure()
{
  int i;
  statusReportType * report;

  report  = (statusReportType*) calloc(1,sizeof(statusReportType));
  setRaiTime();
  for (i = 0; i < MAX_CALLBACKS; i++)
    callbacks[i] = NULL;
  return report;
}



/* copies 2 bytes in net order into a long in host order */
/* and returns the pointer to the next unused byte */

char* copy2Bytes(unsigned long * dest, char* src)
{
      *dest = 0;
      memcpy((char*)(dest)+2,src,2);
      *dest = htonl(*dest);
      return src+2;
}



void parseStatusReport(statusReportType* report,unsigned char* status_values)
{
  unsigned long time;
  unsigned long itemsReported;
  unsigned char* string_position = (status_values + 1);

  time = getRaiTime();

#ifdef DEBUG
  fprintf(stderr,"entering parseReport\n");
#endif

  /* first long is bit flag for rest of items */
  memcpy(&(itemsReported),string_position,4);
  string_position += 4;

  itemsReported = htonl(itemsReported);
  report->Request = itemsReported;
  #ifdef DEBUG
  printf("Got status bits %lu\n",itemsReported);
  #endif

  /**** these must remain in numerical order !! *******/
  /**** as the base will pass them back that way ******/

  if (itemsReported & REPORT_BASE_CLOCK)
    {
      memcpy(&(report->Clock),string_position,4);
      report->Clock = htonl(report->Clock);
      string_position += 4;
    }

  if (itemsReported & REPORT_GENERAL_STATUS)
    string_position  = copy2Bytes(&report->GeneralStatus,string_position);

  if (itemsReported & REPORT_X)
    {
      string_position  = copy2Bytes(&report->Xpos,string_position);
      report->Xpos = 0;
    }

  if (itemsReported & REPORT_Y)
    {
      string_position  = copy2Bytes(&report->Ypos,string_position);
      report->Ypos = 0;
#ifdef DEBUG
      fprintf(stderr,"Status Yposition=%d",report->Ypos); 
#endif
    }

  if (itemsReported & REPORT_HEADING)
    {
      string_position  = copy2Bytes(&report->Heading,string_position);
    }

  if (itemsReported & REPORT_BASE_RELATIVE_HEADING)
    {
      string_position  = copy2Bytes(&report->BaseRelativeHeading,
				    string_position);
    }

  if (itemsReported & REPORT_TRANSLATE_ERROR)
    {
      string_position  = copy2Bytes(&report->TranslateError,string_position);
      report->TranslateError = 0;
    }

  if (itemsReported & REPORT_TRANSLATE_VELOCITY)
    {
      string_position=copy2Bytes(&report->TranslateVelocity,string_position);
      report->TranslateVelocity /= GRIP_ENCODERS_PER_MM;
    }

  if (itemsReported & REPORT_TRANSLATE_STATUS)
    string_position=copy2Bytes(&report->TranslateStatus,string_position);

  if (itemsReported & REPORT_ROTATE_ERROR)
      string_position  = copy2Bytes(&report->RotateError,string_position);

  if (itemsReported & REPORT_ROTATE_VELOCITY)
      string_position  = copy2Bytes(&report->RotateVelocity,string_position);

  if (itemsReported & REPORT_ROTATE_STATUS)
    string_position=copy2Bytes(&report->RotateStatus,string_position);


/* this should be done outside of parse function, which is can then */
/* be common to all apps */



#ifdef DEBUG
  fprintf(stderr,"exiting parse report \n");
#endif
}



/**************************************************************************
void regStatusCallback(void* function_ptr)
{
  int i = 0;

  while ((callbacks[i] != 0)  && (i<MAX_CALLBACKS))
    i++;

  if (i < MAX_CALLBACKS)
    callbacks[i] = function_ptr;
}


void handleStatusCallbacks(StatusReportType* report)
{
  void (*callback_function)(statusReportType*);
  int i;
  for (i = 0; i < MAX_CALLBACKS; i++)
    {
      if ((callbacks[i]) != 0)
	{
	  callback_function = (func_ptr_type)(callbacks[i]);
	  (*callback_function)(report);
	}
    }
}
*/
