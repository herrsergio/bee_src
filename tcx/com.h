
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








/**********************************************************************
 *
 * PROJECT: TCX
 *
 * PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
 * (c) Copyright 1993 Christopher Fedor. All rights reserved.
 * 
 * MODULE: Communications 
 *
 * FILE: com.h
 *
 * ABSTRACT: Communication Include File 
 *
 * EXPORTS:
 *
 * HISTORY:
 *  $Log: com.h,v $
 *  Revision 1.1.1.1  1996/09/22 16:46:02  rhino
 *  General reorganization of the directories/repository, fusion with the
 *  RWI software.
 *
 *  Revision 1.3  1994/09/30 13:08:22  thrun
 *  Changed header declaration.
 *
 * Revision 1.2  1994/05/31  21:00:45  rhino
 * General reorganization. New header file. New makefiles.
 *
 * Revision 1.1.1.1  1994/03/31  08:35:03  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:25  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:27  rhino
 * test
 *
 * Revision 1.3  1993/03/12  20:58:12  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.2  1992/07/10  16:04:46  fedor
 * Changes to compile for VxWorks. Also some changes to avoid compiler warnings.
 *
 * Revision 1.1.1.1  1992/06/16  17:22:03  fedor
 * Import TCX 2.0 16-Jun-92
 *
 * Revision 1.3  1992/04/20  10:37:56  fedor
 * *** empty log message ***
 *
 * Revision 1.2  1992/04/20  10:30:26  fedor
 * test file
 *
 * Revision 1.2  1992/04/20  10:30:26  fedor
 * test file
 *
 * Revision 1.1  1992/04/20  07:05:53  fedor
 * Initial revision
 *
 *  
 *  $Revision: 1.1.1.1 $
 *  $Date: 1996/09/22 16:46:02 $
 *  $Author: rhino $
 *
 *********************************************************************/

extern int readAll(int sd, char *buf, int nbytes);
extern int writeAll(int sd, char *buf, int nbytes);
extern int connectAtPort(char *machine, int port, int *sd);
extern int listenAtPort(int *port, int *sd);

extern int connectToServer(char *serverHost);
extern int singlePassActiveModules(void *timeout);
