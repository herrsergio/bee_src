
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
* MODULE: memory
*
* FILE: tcaMem.c
*
* ABSTRACT:
* 
* Interface to memory management routines.
*
* REVISION HISTORY
*
*  4-Jul-91 Christopher Fedor, School of Computer Science, CMU
* Added tcaRegisterMallocHnd to complete the routines so that modules
* can handle all memory allocation.
*
*  4-Jun-91 Christopher Fedor, School of Computer Science, CMU
* Added tcaRegisterFreeMemHnd so that modules can be called to free
* memory to satisfy a malloc request.
*
* 25-Oct-90 Christopher Fedor, School of Computer Science, CMU
* Created.
*
******************************************************************************/

#ifdef VMS
#include "vms.h"
#endif

#ifdef VXWORKS
#include "/usr/vxworks/vx5.0.2b/h/vxWorks.h"
#include "/usr/vxworks/vx5.0.2b/h/stdioLib.h"
#else
#include "stdio.h" 
#endif

#include "global.h"

extern char *malloc();

#if 0
extern void dataMsgDisplayStats();
#endif


/******************************************************************************
*
* FUNCTION: void tcaRegisterFreeMemHnd(func, retry)
*
* DESCRIPTION: 
* Sets a function for freeing memory when malloc returns NULL.
* The function is passed an unsigned int amount of the memory requested.
* The number of times this routine is called is set by retry.
* The default is 1.
*
* INPUTS: 
* void (*func)();
* int retry;
*
* OUTPUTS: void.
*
******************************************************************************/

void tcaRegisterFreeMemHnd(func, retry)
void (*func)();
int retry;
{
  Global->freeMemRetryAmount = retry;

  if (Global->freeMemRetryAmount < 1)
    Global->freeMemRetryAmount = 1;

  Global->tcaFreeMemoryHnd = func;
}


/******************************************************************************
*
* FUNCTION: void tcaRegisterMallocHnd(func, retry)
*
* DESCRIPTION: 
* Registers a function to call in place of malloc.
* The routine will be passed an unsigned int of the amount of storage needed.
* The routine will be called a max of retry times if NULL is returned.
* The default retry amount is 1.
*
* INPUTS: 
* char *(*func)();
* int retry;
*
* OUTPUTS:
*
******************************************************************************/

void tcaRegisterMallocHnd(func, retry)
char *(*func)();
int retry;
{
  Global->mallocMemRetryAmount = retry;

  if (Global->mallocMemRetryAmount < 1)
    Global->mallocMemRetryAmount = 1;

  if (func)
    Global->tcaMallocMemHnd = func;
}


/******************************************************************************
*
* FUNCTION: void tcaFree(item)
*
* DESCRIPTION: 
* An interface to free - should use tcaFreeData or tcaFreeReply in most cases.
*
* INPUTS: char *item;
*
* OUTPUTS: void.
*
******************************************************************************/

void tcaFree(item)
char *item;
{
  /* printf("tcaFree:\n"); */
  free(item);
}


/******************************************************************************
*
* FUNCTION: char *tcaMalloc(amount)
*
* DESCRIPTION: Interface to malloc requests from tca.
*
* INPUTS: unsigned int amount;
*
* OUTPUTS: char * - generic pointr to the memory
*
* NOTES: 
* Stops everything if we have run out of memory. 
* Note there may not be enough memory to print that we have run out.
*
******************************************************************************/

void *tcaMalloc(amount)
unsigned int amount;
{
  int i, j;
  char *mem;
/*printf("=[%d]=",amount);fflush(stdout);(S.Thrun 93-5-1) */
  mem = NULL;

  if (Global->tcaMallocMemHnd) 
    for(i=0;!mem && i < Global->mallocMemRetryAmount;i++) {
      mem = (*(Global->tcaMallocMemHnd))(amount);
    }

  if (mem) {
    Global->totalMemRequest += amount;
    return(mem);
  }

  if (Global->tcaFreeMemoryHnd) 
    for(j=0;!mem && j < Global->freeMemRetryAmount;j++) {
      (*(Global->tcaFreeMemoryHnd))(amount);

      if (Global->tcaMallocMemHnd) 
	for(i=0;!mem && i < Global->mallocMemRetryAmount;i++) {
	  mem = (*(Global->tcaMallocMemHnd))(amount);
	}
    }

  if (mem) {
    Global->totalMemRequest += amount;
    return(mem);
  }
  
  fprintf(stderr, 
	  "tcaMalloc: NULL returned from malloc for request: %d\n", amount);
#ifdef LISP
  fflush(stderr);
#endif
  tcaModError(NULL);
  return NULL;
}


/******************************************************************************
*
* FUNCTION: void tcaStats()
*
* DESCRIPTION: Quick hack to display some memory stats.
*
* INPUTS: none.
*
* OUTPUTS: void.
*
******************************************************************************/

void tcaStats()
{
  fprintf(stderr, "Total Memory Requests Filled: %d\n", 
	  Global->totalMemRequest);
  fprintf(stderr, "\n");
#if 0
  dataMsgDisplayStats();
#endif
  fprintf(stderr, "\n");
#ifdef LISP
  fflush(stderr);
#endif
}



void tcxLock()
{
;
}


void tcxUnlock()
{
;
}
