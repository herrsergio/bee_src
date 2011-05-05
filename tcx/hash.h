
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








/******************************************************************************
*
* PROJECT: Carnegie Mellon Planetary Rover Project
*          Task Control Architecture 
* 
* (c) Copyright 1991 Christopher Fedor and Reid Simmons.  All rights reserved.
* 
* MODULE: hash
*
* FILE: hash.h
*
* ABSTRACT:
* 
* Generic hash table abstract data type.
*
* REVISION HISTORY
*
*  6-Apr-90 Christopher Fedor, School of Computer Science, CMU
* Revised to Software Standards.
*
* 10-Feb-89 Christopher Fedor, School of Computer Science, CMU
* Created.
*
******************************************************************************/

#ifndef INChash
#define INChash

typedef struct _HASH_ELEM {
  char *key;
  char *data;
  struct _HASH_ELEM *next;
} HASH_ELEM_TYPE, *HASH_ELEM_PTR;

typedef struct {
  int size;
  int (*hashFunc)();
  int (*eqFunc)();
  HASH_ELEM_PTR *table;
} HASH_TABLE_TYPE, *HASH_TABLE_PTR;

extern HASH_TABLE_PTR hashTableCreate();
extern char *hashTableFind();
extern char *hashTableInsert();
extern char *hashTableRemove();
extern void hashTableIterate();
extern void hashTableStats();

#endif INChash
