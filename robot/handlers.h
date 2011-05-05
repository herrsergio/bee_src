
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








/*****************************************************************************
 * PROJECT: Rhino
 *
 * FILE: handlers.h
 *
 * ABSTRACT:
 *
 *****************************************************************************/

#ifndef HANDLERS_H
#define HANDLERS_H


/***********************************************************************
 *
 *  A handler is a function that expect two parameters: callback_data
 *  and client_data. The callback_data is the new data from the device,
 *  the client_data is a data given by the client when it installed the
 *  the handler (may be NULL).
 *  The type of the callback_data can be a generic pointer (which can be
 *  used to store a char, int, long or whatever pointer, but NOT a double
 *  neither a float) or a double.
 *  In case the device have to pass more than one parameter, we'll have
 *  to define an structure in which store those paramaters, and the 
 *  callback_data will be a pointer to that structure.
 *  The type of the client_data is always a generic pointer.
 *
 *  Examples of correct definitions of handlers:
 *
 *  void NewLaserStream(Pointer callback_data, Pointer client_data)
 *  void ChangeInVelocity(double new_velocity, Pointer client_data)
 *
 *
 *  
 *  See the test at the end of handlers.c
 ***********************************************************************/


#include <stdio.h>
#include "Common.h"

#define MAX_HANDLERS_PER_EVENT 10

typedef void (*Handler)(Pointer, Pointer);

typedef struct _HandlerData {
  Handler handler[MAX_HANDLERS_PER_EVENT];
  Pointer client_data[MAX_HANDLERS_PER_EVENT];
} _HandlerData;

typedef _HandlerData *HandlerList;



HandlerList CreateHandlerList(int number_handlers);
void        InstallHandler(HandlerList handler_list, Handler handler, 
			   int position, void *client_data);
BOOLEAN     IsHandlerInstalled(HandlerList handler_list, Handler handler, int position);
void        RemoveHandler(HandlerList handler_list, Handler handler, int position);
void        RemoveAllHandlers(HandlerList handler_list, int position);
void        FireHandler(HandlerList handler_list, int position, void *callback_data);

#endif
