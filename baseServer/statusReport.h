
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
/*    ACCT: mjc                                                 */
/*    FILE: statusReport.h                                      */
/*    DATE: Mon Jun 20 17:23:36 1994                            */
/*                                                              */
/*    Modification history:                                     */
/*                                                              */
/*    7/20/94 - jmk                                             */
/*                                                              */
/*    Made handleStatusCallbacks public so a polling function   */
/*    can force the status report to service its callbacks      */
/*                                                              */
/****************************************************************/


#ifndef STATUSSTORAGE_HEADER
#define STATUSSTORAGE_HEADER
#include <sys/types.h>
#include <unistd.h>
#include <rai.h>
#include <baseMessages.h>

extern statusReportType activeStatusReport;

#ifdef __cplusplus
extern "C" {
#endif

void handleStatusCallbacks();
void initStatusStructure();
statusReportType* parseReport(unsigned char*);
void regStatusCallback(void* function_ptr);

#ifdef __cplusplus
}
#endif


#endif
