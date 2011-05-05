
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

#include <math.h>

#define ITRIG_UNITS_PER_CIRCLE 4096

#ifdef __cplusplus
extern "C" {
#endif 

void ITrigInit(void);

/* returns your scalar times the cos|sin|tan of the angle */
int iCos(int angle, int scalar);
int iSin(int angle, int scalar);


/* Note:  Where Tan is not defined, iTan is defined to be */
/*  (INT_MAX/ITRIG_UNITS_PER_CIRCLE).   It would just be  */
/*  INT_MAX, but then the first time you multiplied it,   */
/*  you would overflow */

int iTan(int angle, int scalar);

void sonarToXY(int sonar_num,int distance,int* x, int* y);

#ifdef __cplusplus
}
#endif 
