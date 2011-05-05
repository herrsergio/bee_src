
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
 * FILE: handlers.c
 *
 * ABSTRACT:
 * 
 * A couple of utility functions to manage handlers.
 * Possible improvements: allow more than one event handler
 *
 *****************************************************************************/

#ifdef VMS
#include "vms.h"
#include "vmstime.h"
#endif

#include <stdio.h>
#include <stdlib.h>

#include "Common.h"
#include "libc.h"
#include "handlers.h"



/*	Function Name: CreateHandlerList
 *	Arguments:     number_handlers
 *	Description:   create a handler list to manage the number
 *                     of handlers given as paramenter
 *	Returns:       nothing
 */

HandlerList CreateHandlerList(int number_handlers)
{
  HandlerList handler_list;
  int i, j;
  
  handler_list = (_HandlerData *) calloc((size_t)sizeof(_HandlerData),
					 (size_t)number_handlers);
  for (i=0; i<number_handlers; i++)
      for (j=0; j<MAX_HANDLERS_PER_EVENT; j++)
	  handler_list[i].handler[j] = NULL;
  return handler_list;
}



/*	Function Name: InstallHandler
 *	Arguments:     list     -- handler list
 *                     position -- event position where store the handler
 *                     client_data -- client data to store with the handler
 *	Description:   add a handler in the list, in the given position.
 *                     The module support a maximum of MAX_HANDLERS_PER_EVENT (10)
 *                     per event (position)
 *	returns:       nothing
 */

void InstallHandler(HandlerList list, Handler handler, 
		    int position, void *client_data)
{
    int i;

    for( i = 0; i < MAX_HANDLERS_PER_EVENT; i++ ) {
	if( list[position].handler[i] == handler )
	    return;
	if (list[position].handler[i] == NULL)
	    break;
    }
    if (i == MAX_HANDLERS_PER_EVENT)
	return;
    list[position].handler[i] = handler;
    list[position].client_data[i] = client_data;
}


/*	Function Name: IsHandlerInstalled
 *	Arguments:     list     -- handler list
 *                     handler  -- handler to ask about
 *                     position -- event position
 *	Description:   check if <handler> is in the given position
 *	Returns:       boolean
 */

BOOLEAN IsHandlerInstalled(HandlerList list, Handler handler, int position)
{
    int i;

    for( i = 0; i < MAX_HANDLERS_PER_EVENT; i++ ) 
	if (list[position].handler[i] == handler)
	    return TRUE;
	else if (list[position].handler[i] == NULL)
	    return FALSE;
    return FALSE;
}



/*	Function Name: RemoveHandler
 *	Arguments:     list     -- handler list
 *                     handler  -- handler
 *                     position -- event position
 *	Description:   remove a handler from the handler list
 *	Returns:       nothing
 */

void RemoveHandler(HandlerList list, Handler handler, int position)
{
    int i;
    
    for( i = 0; i < MAX_HANDLERS_PER_EVENT; i++ )
	if (list[position].handler[i] == handler)
	    break;
    if ( i == MAX_HANDLERS_PER_EVENT ) /* not found */
	return;
    for (i++; i < MAX_HANDLERS_PER_EVENT; i++){
      /* Changed the order of the sequence because otherwise the last handler
       * will not be removed
       */
	list[position].handler[i-1] = list[position].handler[i];
	list[position].client_data[i-1] = list[position].client_data[i];
	if (list[position].handler[i] == NULL)
	    return;
    }
}

void RemoveAllHandlers(HandlerList list, int position)
{
    int i;
    
    for( i = 0; (i < MAX_HANDLERS_PER_EVENT) && (list[position].handler[i] != NULL); i++ )
	list[position].handler[i] = NULL;
}



/*	Function Name: FireHandler
 *	Arguments:     list          -- handler list
 *                     position      -- event position
 *                     callback_data -- pointer that will be read
 *                                      by the handler
 *	Description:   the handler is executed with the given data
 *                     and the client_data previously installed. The
 *                     callback_data can be a pointer to a structure,
 *                     a char or an integer (the later two casted to 
 *                     void *)
 *	Returns:       nothing
 */

void FireHandler(HandlerList list, int position, Pointer callback_data)
{
  int i;
  
  for( i = 0; 
      (i < MAX_HANDLERS_PER_EVENT) && (list[position].handler[i] != NULL);
      i++ )
    (*list[position].handler[i])(callback_data, list[position].client_data[i]);
}

/* Old - removed by RTG. */
/*	Function Name: FireHandlerDouble
 *	Arguments:     list          -- handler list
 *                     position      -- event position
 *                     callback_data -- double that will be read
 *                                      by the handler
 *	Description:   the handler is executed with the given data
 *                     and the client_data previously installed. This
 *                     function is used when the callback_data is a 
 *                     double, since a double cannot be casted to a
 *                     pointer.
 *	Returns:       nothing
 */

/* 
 * void FireHandlerDouble(HandlerList list, int position, double callback_data)
 * {
 *   if (IsHandlerInstalled(list, position))
 *     (*list[position].handler)(callback_data, list[position].client_data);
 *   return;
 * }
 */


/************/
/*   TEST   */
/************/


/*  
  
  #define EVENT1 0
  #define EVENT2 1
  #define EVENT3 2
  #define NUM_EVENTS 3
  
  typedef struct test {
  float data1, data2, data3;
  } _Test, *Test;
  
  
  static void hand1()
  {
  printf ("\nHello world, event1\n");
  }
  
  static void hand2(double *callback_data, void *client_data)
  {
  printf ("\nHello world, event2\n");
  printf("Callback data: %f \nClient data: %d\n", callback_data, (int) client_data);
  }
  
  static void hand3(void *callback_data, void *client_data)
  {
  Test test;
  
  test = (Test) callback_data;
  
  printf ("\nHello world, event3\n");
  printf("Callback data: %f %f %f\nClient data: %d\n", test->data1, 
  test->data2, test->data3, (int) client_data);
  }
  
  void main(int argc, char *argv[])
  {
  HandlerList list;
  double      pp;
  _Test       test;
  
  list = CreateHandlerList(NUM_EVENTS);
  
  InstallHandler(list, hand1, EVENT1, NULL);
  InstallHandler(list, hand2, EVENT2, (Pointer) 2000);
  InstallHandler(list, hand3, EVENT3, (Pointer) 2000);
  
  FireHandler(list, EVENT1, NULL);
  
  test.data1 = 100.0;
  test.data2 = 200.0;
  test.data3 = 300.0;
  
  FireHandler(list, EVENT3, &test);
  FireHandler(list, EVENT2, NULL);
  
  RemoveHandler(list, hand3, EVENT3);
  FireHandler(list, EVENT3, NULL);
  }
  
*/  
