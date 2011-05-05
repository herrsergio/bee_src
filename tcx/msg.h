
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
 *  PROJECT:  Task Control
 *
 * PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
 * (c) Copyright 1993 Christopher Fedor. All rights reserved.
 *
 *  MODULE: Message
 *
 *  FILE: msg.h
 *
 *  ABSTRACT: Message Definition Header File.
 *
 *  EXPORTS:
 *
 *  HISTORY:
 *  $Log: msg.h,v $
 *  Revision 1.1.1.1  1996/09/22 16:46:02  rhino
 *  General reorganization of the directories/repository, fusion with the
 *  RWI software.
 *
 *  Revision 1.2  1994/05/31 21:00:54  rhino
 *  General reorganization. New header file. New makefiles.
 *
 * Revision 1.1.1.1  1994/03/31  08:35:03  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:27  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:27  rhino
 * test
 *
 * Revision 1.4  1993/03/12  20:59:00  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.3  1992/07/27  04:02:38  fedor
 * Added registration of handlers to server. Started message forwarding.
 *
 * Revision 1.2  1992/07/08  15:20:29  fedor
 * Changed message registration to use server generated id values.
 * Added a message handler registration - name may change.
 * Now there are two kinds of handlers one for messages of the form (ref,
 * data, cdata) and the many parameter one (mod, id, ref, data, cdata)
 *
 * Revision 1.1.1.1  1992/06/16  17:22:06  fedor
 * Import TCX 2.0 16-Jun-92
 *
 * Revision 1.4  1992/04/20  10:37:56  fedor
 * *** empty log message ***
 *
 * Revision 1.3  1992/04/20  10:30:26  fedor
 * test file
 *
 * Revision 1.3  1992/04/20  10:30:26  fedor
 * test file
 *
 * Revision 1.2  1992/03/30  07:39:30  fedor
 * new data.c and msg.c
 *
 * Revision 1.1  1992/03/26  06:44:20  fedor
 * Initial revision
 *
 *  
 *  $Revision: 1.1.1.1 $
 *  $Date: 1996/09/22 16:46:02 $
 *  $Author: rhino $
 *
 *********************************************************************/

#ifndef INCmsg
#define INCmsg

typedef struct {
  int id;
  char *name;
  char *msgForm;
  LIST_PTR hndList;
  TCX_FMT_PTR msgFormat;
} MSG_DEF_TYPE, *MSG_DEF_PTR;

typedef struct {
  MSG_DEF_PTR msg;
  int id, ref, len;
  TCX_MODULE_PTR module;
  void *encodedData;
  void *decodedData;
} MSG_INS_TYPE, *MSG_INS_PTR;

extern void messageInitialize();

extern void messageDefinition(int id, char *msgName,  char *msgForm);

extern MSG_INS_PTR messageCreateInstance(TCX_MODULE_PTR module, int id, 
					 int len, void *encodedData);

extern void messageInsertQueue(MSG_INS_PTR msgInstance);

extern void messageAddNewInstance(TCX_MODULE_PTR module, int id, int len, 
			   void *encodedData);

extern MSG_DEF_PTR messageFindById(int id);
extern MSG_DEF_PTR messageFindByName(char *name);

extern TCX_REF_PTR messageRefCreate(TCX_MODULE_PTR module, int id, int ref);
extern void messageRefFree(TCX_REF_PTR ref);

#endif
