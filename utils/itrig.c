
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

/*****************************************************
*
*  This isn't a module, but is rather just a library
*  for doing integer triginometry.
*
*  James Kurien
*
******************************************************/

#include <limits.h>
#include <itrig.h>

#ifndef M_PI
#define M_PI PI
#endif

#define TRUE 1
#define FALSE !TRUE


/* we multiply our values by this when putting them in the table */
/* then divide out when we give an answer, letting us store      */
/* non-integral values. This only makes sense because usually we */
/* are multiplying the table value by a large scalar when we     */
/* retreive it */

#define ITRIG_INT_SCALE   256

int isin_table[ITRIG_UNITS_PER_CIRCLE];
int icos_table[ITRIG_UNITS_PER_CIRCLE];
int itan_table[ITRIG_UNITS_PER_CIRCLE];

/* just so we do not waste time initializing twice */ 
int itrig_initialized = FALSE;

/* This will not work for tan because it is undefine some */
/* places.  We do tan by itself below */

void init_table(int* table, int size,double(*function)(double))
{
  int i,int_val;
  double func_val,arg;

  /* This is pretty verbose just to make sure all the implicit*/
  /* type conversion gets done */

  for(i=0;i<size;i++)
    {
      arg = (2*M_PI*i)/size;
      func_val = function(arg); /* almost by definition :-) */
      int_val  = func_val * ITRIG_INT_SCALE;
      table[i] = int_val;
    }
}


void init_tan(int* table, int size)
{
  int i,int_val;
  double func_val,arg;

  /* This is pretty verbose just to make sure all the implicit*/
  /* type conversion gets done */

  for(i=0;i<size;i++)
    {
      arg = (2*M_PI*i)/size;
      if ((i == size/4) || i == (3*size/4))
	func_val = INT_MAX / size;
      else
	func_val = tan(arg); 
      int_val  = func_val * ITRIG_INT_SCALE;
      table[i] = int_val;
    }
}


void ITrigInit(void)
{
  if (!itrig_initialized)
    {
      init_table(isin_table,ITRIG_UNITS_PER_CIRCLE,sin);
      init_table(icos_table,ITRIG_UNITS_PER_CIRCLE,cos);
      init_tan(itan_table,ITRIG_UNITS_PER_CIRCLE);
      itrig_initialized = TRUE;
    }
}


/* NOTE:  % is bogus if angle is negative and more than ITRIG_UNITS_PER_CIRCLE */

inline int iCos(int angle,int scalar)
{
  return 
    (scalar * icos_table[(angle) % ITRIG_UNITS_PER_CIRCLE]) / ITRIG_INT_SCALE;
}


inline int iSin(int angle,int scalar)
{
  return 
    (scalar * isin_table[(angle) % ITRIG_UNITS_PER_CIRCLE]) / ITRIG_INT_SCALE;
}

inline int iTan(int angle,int scalar)
{
  return
    (scalar * itan_table[(angle) % ITRIG_UNITS_PER_CIRCLE]) / ITRIG_INT_SCALE;
}



