
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








/**************************************************************************
* 
* PROJECT: Carnegie Mellon Planetary Rover Project
*
* PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
* (c) Copyright 1993 Christopher Fedor. All rights reserved.
*
* MODULE: list
*
* FILE: list.h
*
* ABSTRACT: The list module provides basic list creation and manipulation
* routines and serves as the base abstract data type for the tca.
* The include file list.h provides the top level routines for other modules.
*
* EXPORTS:
*
* REVISION HISTORY:
*
* See list.c for history.
*
**************************************************************************/

#ifndef INClist
#define INClist

typedef struct _LIST_ELEM {
  char *item;
  struct _LIST_ELEM *next, *previous;
} LIST_ELEM_TYPE, *LIST_ELEM_PTR;

typedef struct _LIST {
  int length;
  struct _LIST *freeList;
  LIST_ELEM_PTR first, last, next;
} LIST_TYPE, *LIST_PTR;

extern LIST_PTR listCreate();
extern void listFree();
extern void listInsertItem();
extern void listDeleteItem();
extern void listFreeAllItems();
extern void listTestDeleteItem();
extern void listInsertItemAfter();
extern void listInsertUnique();

extern int listEqual();
extern int listLength();
extern int listIterate();
extern int listMemberItem();

extern LIST_PTR listCopy();

extern LIST_PTR listMake1();
extern LIST_PTR listMake2();

extern char *listFirst();
extern char *listLast();
extern char *listNext();

#endif INClist
