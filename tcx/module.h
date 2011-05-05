
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
 *  PROJECT:  TCX
 *
 * PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
 * (c) Copyright 1993 Christopher Fedor. All rights reserved.
 * 
 *  MODULE: Module
 *
 *  FILE: module.h
 *
 *  ABSTRACT: Module Include File.
 *
 *  EXPORTS:
 *
 *  HISTORY:
 *  $Log: module.h,v $
 *  Revision 1.1.1.1  1996/09/22 16:46:02  rhino
 *  General reorganization of the directories/repository, fusion with the
 *  RWI software.
 *
 *  Revision 1.2  1994/05/31 21:00:53  rhino
 *  General reorganization. New header file. New makefiles.
 *
 * Revision 1.1.1.1  1994/03/31  08:35:03  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:26  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:27  rhino
 * test
 *
 * Revision 1.7  1993/03/12  20:58:54  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.6  1992/11/19  03:02:06  fedor
 * Remove module from a pending sets if it crashes before a confirming connection.
 *
 * Revision 1.5  1992/11/16  15:51:06  fedor
 * Rewrote initializing connections and auto reconnect.
 *
 * Revision 1.4  1992/11/13  14:47:17  fedor
 * Update archive with a working version and partially working server route
 *
 * Revision 1.3  1992/10/23  20:15:28  fedor
 * Redid method of connecting to modules. Added the notion of a connection.
 * Added some auto-reconnect for those modules that are probably listening.
 * See detail notes.
 *
 * Revision 1.2  1992/07/03  10:13:22  fedor
 * New module definition and routines
 *
 * Revision 1.1.1.1  1992/06/16  17:22:06  fedor
 * Import TCX 2.0 16-Jun-92
 *
 * Revision 1.2  1992/04/20  10:32:36  fedor
 * *** empty log message ***
 *
 * Revision 1.1  1992/04/20  09:19:35  fedor
 * Initial revision
 *
 * Revision 1.1  1992/04/20  09:19:35  fedor
 * Initial revision
 *
 *  
 *  $Revision: 1.1.1.1 $
 *  $Date: 1996/09/22 16:46:02 $
 *  $Author: rhino $
 *
 *********************************************************************/

#ifndef INCmodule
#define INCmodule

#define MODULE_IS_ACTIVE(module) (module->status & STATUS_ACTIVE)

extern MODULE_PTR moduleCreate(int sd, int port, char *name, char *host,
			       int status);

extern MODULE_PTR moduleCreate2(int sd, MODULE_INFO_PTR moduleInfo,
			       int status);

extern void moduleOpen(int sd);
extern void moduleClose(MODULE_PTR module);
extern void reconnectModule2(MODULE_PTR module);

#endif INCmodule
