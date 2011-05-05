
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
* (c) Copyright 1991 Christopher Fedor and Reid Simmons.  All rights reserved.
* 
* MODULE: list
*
* FILE: list.c
*
* ABSTRACT:
* The list module provides basic list creation and manipulation
* routines for doubly linked lists.
*
* The include file list.h provides the top level routines for other modules.
*
* EXPORTS:
*
* LIST_PTR listCreate()
* Creates an empty list.
*
* void listFree(list)
* LIST_PTR list;
* Frees storage associated with a list.
*
* void listInsertItemFirst(item, list)
* char *item;
* LIST_PTR list;
* Adds item as the first item in the list.
*
* void listInsertItemLast(item, list)
* char *item;
* LIST_PTR list;
* Adds item as the last item in the list.
*
* void listDeleteItem(item, list)
* char *item;
* LIST_PTR list;
* Removes item from list.
*
* void listDeleteItemAll(item, list)
* char *item;
* LIST_PTR list;
* Removes all elements containing item from list.
*
* void listTestDeleteItem(func, param, list)
* int (*func)();
* char *param;
* LIST_PTR list;
* Removes the first item in the list found such that func(param, item)
* returns 1 (TRUE).
*
* void listTestDeleteItemAll(func, param, list)
* int (*func)();
* char *param;
* LIST_PTR list;
* Removes all items in the list found such that func(param, item)
* returns 1 (TRUE).
*
* int listMemberItem(item, list)
* char *item;
* LIST_PTR list;
* Returns 1 (TRUE) if item is in the list, otherwise 0 (FALSE) is returned.
*
* char *listMemReturnItem(func, param, list)
* int (*func)();
* char *param;
* LIST_PTR list;
* listMemReturnItem is a more general form of listMemberItem.
* listMemReturnItem will return the item (or one of the items) in list
* for which func(param, item) is non-zero, i.e. is TRUE.
* The function takes two arguments, the first is the param and the second is 
* an item of the list and returns an integer value. int func(param, item).
* If the functions does not satisfy any of the items in the list NULL
* is returned.
*
* int listIterateFromFirst(func, param, list)
* int (*func)();
* char *param;
* LIST_PRT list;
*
* int listIterateFromLast(func, param, list)
* int (*func)();
* char *param;
* LIST_PRT list;
*
* listIterateFromFirst and listIterateFromLast will call the function
* func with param on all of its elements stoping when the list is
* finished or when func returns 0 (ie FALSE). The function func will
* take two arguments the first is param the second is an item of the
* set. func(param, item).  listIterate returns 0 (FALSE) if the function
* func returns 0 (FALSE).  Otherwise listIterate returns 1 (TRUE).
* listIterate will return 0 (FASLE) if the list is NULL.
*
* listIterateFromFirst starts iteration from the first item in the list going
* forward through the list.
*
* listIterateFromLast starts iteration from the last item in the list going
* backwards through the list.
*
* char *listPopItem(list)
* LIST_PTR list
* Removes and returns first item from list.
*
* REVISION HISTORY:
*
* 8-Dec-91  Christopher Fedor at School of Computer Science, CMU
* Added nextTmp so that listDelete can be used from within listMemReturnItem.
*
* 21-Aug-91  Christopher Fedor at School of Computer Science, CMU
* Added the routine listInsertItemAfter.
*
* 22-Dec-90  Christopher Fedor at School of Computer Science, CMU
* Added tests so that nill can not be inserted in a list.
* This avoids possible confusion in list iterate and delete functions.
*
* 11-Dec-90  Christopher Fedor at School of Computer Science, CMU
* Added item tests in list iterate so that holes created by listDelete 
* do no kill the func call. - this may still need work.
*
*  3-Oct-90  Christopher Fedor at School of Computer Science, CMU
* Added nextTmp and previousTmp to list iterate routines so that
* they terminate gracefully if there test function calls listDelete.
* This was needed for the removeConnection routine which is called from
* within a listIterateFromFirst. Just goes to show that there is
* still much trouble with interactions among generic data structure 
* routines.
*
*  5-Apr-90  Christopher Fedor at School of Computer Science, CMU
* Added listFreeListGlobal as a list of free top level list elements.
* Also added warning code for those routines who insist on freeing
* already freed lists.
*
* 13-Nov-89  Christopher Fedor at School of Computer Science, CMU
* Added listCellFreeListGlobal as a list element free list.
*
* 16-Oct-89  Christopher Fedor at School of Computer Science, CMU
* Added listPushItem, in list.h,  and listPopItem.
*
* 13-Oct-89  Christopher Fedor at School of Computer Science, CMU
* Added listTestDeleteItem and listTestDeleteItemAll routines.
*
* 10-Oct-89  Christopher Fedor at School of Computer Science, CMU
* Added membership and iteration routines so this list abstract 
* data type can be used as a simple set abstract data type.
*
*  9-Oct-89  Christopher Fedor at School of Computer Science, CMU
* Rewrote to form basis for sets and queues. Updated to Software Standards.
* Based routine names on functionality, got rid of lispish names.
* Removed Long-Ji's test for empty list because it was a redundant test
* - sounds like the real problem was soleved by a correction elsewhere.
*
* 29-Aug-89  Long-Ji Lin at School of Computer Science, CMU
* Modified "DoList" and "DoList1" to make "body" go 
* after "list_var = Cdr(list_var)"
*
* 24-Aug-89  Long-Ji Lin at School of Computer Science, CMU
* Added a check to return from listDeleteItem (was SetDelete) if the list
* exists but is empty.
*
* Ancient    Reid Simmons at School of Computer Science, CMU
* created.
*
**************************************************************************/
                  
#ifdef VMS              
#include "vms.h"
#endif

#ifdef VXWORKS
#include "/usr/vxworks/vx5.0.2b/h/vxWorks.h"
#include "/usr/vxworks/vx5.0.2b/h/stdioLib.h"
#else
#include "stdio.h" 
#endif

#include "list.h"

#include "global.h"

#define LIST_INC_AMOUNT      10
#define LIST_CELL_INC_AMOUNT 10


/**************************************************************************
*
* FUNCTION: listIncFreeList()
*
* DESCRIPTION: Increment list top level free list.
*
* INPUTS: none.
*
* OUTPUTS: none.
*
* EXCEPTIONS: If malloc returns NULL, call tcaError.
*
* DESIGN: Malloc LIST_INC_AMOUNT number of list elments.
*
* NOTES: listFreeListGlobal points to the newly created elements.
*
**************************************************************************/

void listIncFreeList()
{
  int i;
  LIST_PTR newList;

  for(i=1;i < LIST_INC_AMOUNT;i++) {
    newList = (LIST_TYPE *)malloc(sizeof(LIST_TYPE));

    if (!newList) 
      tcaModError("list: Can not increment list top level free list.");

    newList->length = 0;
    newList->first = NULL;
    newList->last = NULL;
    newList->next = NULL;

    newList->freeList = Global->listFreeListGlobal;
    Global->listFreeListGlobal = newList;
  }
}


/**************************************************************************
*
* FUNCTION: listIncCellFreeList()
*
* DESCRIPTION: Increment list element free list.
*
* INPUTS: none.
*
* OUTPUTS: none.
*
* EXCEPTIONS: If malloc returns NULL, call tcaError.
*
* DESIGN: Malloc LIST_CELL_INC_AMOUNT number of list elments.
*
* NOTES: listCellFreeListGlobal points to the newly created elements.
*
**************************************************************************/

void listIncCellFreeList()
{
  int i;
  LIST_ELEM_PTR newCell;

  extern void **testPtr;
  
  for(i=1;i<LIST_CELL_INC_AMOUNT;i++) {
#if 0
    if (testPtr) {
      fprintf(stderr, "%s:%6d:%s() - *testPtr = 0x%0X "
	      "testPtr = 0x%X &newCell = 0x%X\n",
	      __FILE__, __LINE__, __FUNCTION__,
	      *testPtr, testPtr, &newCell);
    }
#endif
    newCell = (LIST_ELEM_TYPE *)malloc(sizeof(LIST_ELEM_TYPE));
#if 0
    if (testPtr) {
      fprintf(stderr, "%s:%6d:%s() - *testPtr = 0x%0X "
	      "newCell = 0x%X sizeof(*newCell) = 0x%X\n",
	      __FILE__, __LINE__, __FUNCTION__,
	      *testPtr, newCell, sizeof(LIST_ELEM_TYPE));
    }
#endif
    if (!newCell) 
      tcaModError("list: Can not increment list element free list.");

    newCell->item = NULL;
    newCell->previous = NULL;

    newCell->next = Global->listCellFreeListGlobal;
    Global->listCellFreeListGlobal = newCell;
  }
}


/**************************************************************************
*
* FUNCTION: void listFreeTop(list)
*
* DESCRIPTION: Initializes a list top level and returns it to the free list.
*
* INPUTS: LIST_PTR list.
*
* OUTPUTS: none.
*
* EXCEPTIONS: none.
*
* DESIGN: Initializes the cell and inserts it as the first item to 
* listFreeListGlobal.
*
* NOTES:
*
**************************************************************************/

void listFreeTop(list)
LIST_PTR list;
{
  list->length = 0;
  list->first = NULL;
  list->last = NULL;
  list->next = NULL;
  
  list->freeList = Global->listFreeListGlobal;
  Global->listFreeListGlobal = list;
}


/**************************************************************************
*
* FUNCTION: void listFreeCell(listCell)
*
* DESCRIPTION: Initializes a list element and returns it to the free list.
*
* INPUTS: LIST_ELEM_PTR listCell.
*
* OUTPUTS: none.
*
* EXCEPTIONS: none.
*
* DESIGN: Initializes the cell and inserts it as the first item to 
* listCellFreeListGlobal.
*
* NOTES:
*
**************************************************************************/

void listFreeCell(listCell)
LIST_ELEM_PTR listCell;
{
  listCell->item = NULL;
  listCell->previous = NULL;
  
  listCell->next = Global->listCellFreeListGlobal;
  Global->listCellFreeListGlobal = listCell;
}


/**************************************************************************
*
* FUNCTION: void listFree(list)
*
* DESCRIPTION: Frees a list.
*
* INPUTS: LIST_PTR list.
*
* OUTPUTS: none.
*
* EXCEPTIONS: none.
*
* DESIGN: Call listFreeCell on each list element.
*
* NOTES: 
*
**************************************************************************/

void listFree(list)
LIST_PTR list;
{
  LIST_ELEM_PTR tmpA, tmpB;

  if (!list)
    return;

  if (list->freeList) {
    fprintf(stderr, 
	    "listFree: OP IGNORED WARNING: list already on free list.\n");
    return;
  }

  tmpA = list->first;

  while (tmpA) {
    tmpB = tmpA;
    tmpA = tmpA->next;

    listFreeCell(tmpB);
  }
  
  listFreeTop(list);
}    


/**************************************************************************
*
* FUNCTION: LIST_PTR listCreate()
*
* DESCRIPTION: Creates an empty list.
*
* INPUTS: none.
*
* OUTPUTS: A pointer to the newly created empty list of type LIST_PTR.
* If there is an error NULL is returned.
*
* EXCEPTIONS: none.
*
* DESIGN: malloc up storage for the list, checking to see if space was
* successfully allocated, and initialize the list structure.
*
* NOTES:
*
**************************************************************************/

LIST_PTR listCreate()
{
  LIST_PTR list;

  if (!Global->listFreeListGlobal)
    listIncFreeList();
  
  list = Global->listFreeListGlobal;
  Global->listFreeListGlobal = Global->listFreeListGlobal->freeList;

  list->freeList = NULL;

  return list;
}


/**************************************************************************
*
* FUNCTION: void listInsertItemFirst(item, list)
*
* DESCRIPTION: Adds item as the first item in the list.
*
* INPUTS: 
* char *item - a pointer to an item of data.
* LIST_PTR list - a pointer to a list.
*
* OUTPUTS: none.
*
* EXCEPTIONS:  
* If the list is NULL, return.
* If the item is NULL, return.
*
* DESIGN: 
* Create a list element to store item and insert the element as the first item
* in the list.
*
* NOTES: If malloc returns NULL simply return <- this is a major problem.
*
**************************************************************************/

void listInsertItemFirst(item, list)
char *item;
LIST_PTR list;
{
   LIST_ELEM_PTR element;

   if (!item || !list)
     return;
   
   if (!Global->listCellFreeListGlobal)
     listIncCellFreeList();

   element = Global->listCellFreeListGlobal;
   Global->listCellFreeListGlobal = Global->listCellFreeListGlobal->next;

   element->item = item;
   element->next = list->first;
   element->previous = NULL;

   if (!list->first) {
     list->first = element;
   }
   else {
     list->first->previous = element;
   }

   if (!list->last) {
     list->last = element;
   }

   list->length++;
   list->first = element;
}


/**************************************************************************
*
* FUNCTION: void listInsertItemLast(item, list)
*
* DESCRIPTION: Adds item as the last item in the list.
*
* INPUTS: 
* char *item - a pointer to an item of data.
* LIST_PTR list - a pointer to a list.
*
* OUTPUTS: none.
*
* EXCEPTIONS:  
* If the list is NULL, return.
* If the item is NULL, return.
*
* DESIGN: 
* Create a list element to store item and insert the element as the first item
* in the list.
*
* NOTES: If malloc returns NULL simply return <- this is a major problem.
*
**************************************************************************/

void listInsertItemLast(item, list)
char *item;
LIST_PTR list;
{
   LIST_ELEM_PTR element;

   if (!item || !list)
     return;

   if (!Global->listCellFreeListGlobal)
     listIncCellFreeList();

   element = Global->listCellFreeListGlobal;
   Global->listCellFreeListGlobal = Global->listCellFreeListGlobal->next;

   element->item = item;
   element->next = NULL;
   element->previous = list->last;

   if (!list->first) {
     list->first = element;
   }

   if (!list->last) {
     list->last = element;
   }
   else {
     list->last->next = element;
   }

   list->length++;
   list->last = element;
}



/**************************************************************************
*
* FUNCTION: void listInsertItemAfter(item, after, list)
*
* DESCRIPTION: Splices item into the list after, after.
*
* INPUTS: 
* char *item - a pointer to an item of data.
* char *after - the item to be inserted after.
* LIST_PTR list - a pointer to a list.
*
* OUTPUTS: none.
*
* EXCEPTIONS:  
* If the list is NULL, return.
* If the item is NULL, return.
* If after is NULL call listInsertItemFirst.
* If after is not found call listInsertItemLast.
*
* NOTES: If malloc returns NULL simply return <- this is a major problem.
*
**************************************************************************/

void listInsertItemAfter(item, after, list)
char *item, *after;
LIST_PTR list;
{
   LIST_ELEM_PTR element, tmp;

   if (!item || !list)
     return;

   if (!after) {
     listInsertItemFirst(item, list);
     return;
   }

   tmp = list->first;

   while (tmp && tmp->item != after) 
     tmp = tmp->next;

   if (!tmp) {
     listInsertItemLast(item, list);
     return;
   }

   if (!Global->listCellFreeListGlobal)
     listIncCellFreeList();

   element = Global->listCellFreeListGlobal;
   Global->listCellFreeListGlobal = Global->listCellFreeListGlobal->next;

   element->item = item;
   element->next = tmp->next;
   element->previous = tmp;

   tmp->next = element;

   list->length++;
}


/**************************************************************************
*
* FUNCTION: char *listPopItem(list)
*
* DESCRIPTION: Removes and returns first item from list.
*
* INPUTS: 
* LIST_PTR list - a pointer to a list.
*
* OUTPUTS:
* The first item or NULL
*
* EXCEPTIONS:  If the list is NULL or the list is empty, return NULL.
*
* DESIGN: 
* Remove the first list element, return the item and free the element.
*
* NOTES:
* might want to start a free list of list elements.
*
**************************************************************************/

char *listPopItem(list)
LIST_PTR list;
{
  char *item;
  LIST_ELEM_PTR oldElement;

  item = NULL;
  
  if (list && list->first) {
    item = list->first->item;
    oldElement = list->first;
    list->first = list->first->next;
    if (list->first) {
      list->first->previous = NULL;
    }
    if (list->last == oldElement) {
      list->last = NULL;
    }
    list->length--;
    listFreeCell(oldElement);
  }

  return item;
}



void listTestDeleteItem(func, param, list)
int (*func)();
char *param;
LIST_PTR list;
{
  LIST_ELEM_PTR current, previous;

  if (!list || !list->first)
    return;

  current = previous = list->first;

  if ((*func)(param, current->item)) {
    /* item is the first element of the list */
    if (list->last == current) {
      list->last = NULL;
    }
    list->first = current->next;
    if (current->next) {
      current->next->previous = NULL;
    }
    list->length--;
    listFreeCell(current);
    return;
  }

  current = current->next;

  while (current) {
    if ((*func)(param, current->item)) {
      if (list->last == current) {
	list->last = previous;
      }
      current->previous = previous;
      previous->next = current->next;

      if (current->next) {
	current->next->previous = previous;
      }

      list->length--;
      listFreeCell(current);
      return;
    }
    previous = current;
    current = current->next;
  }
}


/**************************************************************************
*
* FUNCTION: void listTestDeleteItemAll(func, param, list)
*
* DESCRIPTION: 
* Removes all items in the list found such that func(param, item)
* returns 1 (TRUE).
*
* INPUTS: 
* int (*func)() - pointer to a test function of the form func(param, item).
* char *param - a pointer to a parameter for func.
* LIST_PTR list - a pointer to a list.
*
* OUTPUTS: none.
*
* EXCEPTIONS:  If the list is NULL or the list is empty, return;
*
* DESIGN: 
* Linearly search the list for a list element containing item,
* such that func(param, item) returns 1 (TRUE). If found
* the list element is removed and freed. All the items of the list
* are tested. Reset element's previous pointer and list->last if needed.
*
* NOTES: item is not freed. Use listDeleteItem to delete a single occurance
* of item. Modified from listDeleteItem.
*
**************************************************************************/

void listTestDeleteItemAll(func, param, list)
int (*func)();
char *param;
LIST_PTR list;
{
  LIST_ELEM_PTR current, previous;

  if (!list || !list->first)
    return;

  while (list->first && (*func)(param, list->first->item)) {
    list->length--;
    current = list->first;
    list->first = current->next;

    if (list->first) {
      list->first->previous = NULL;
    }
    if (list->last == current) {
      list->last = NULL;
    }
    listFreeCell(current);
  }

  if (!list->first)
    return;

  previous = list->first;
  current  = list->first->next;

  while (current) {
    if ((*func)(param, current->item)) {
      if (list->last == current) {
	list->last = previous;
      }

      previous->next = current->next;

      if (current->next) {
	current->next->previous = previous;
      }

      list->length--;
      listFreeCell(current);
     
      current = previous->next;
    }
    else {
      previous = current;
      current = current->next;
    }
  }
}


/**************************************************************************
*
* FUNCTION: void listFreeAllItems(func, list)
*
* DESCRIPTION: 
* Applies the func to each item in the list while removing it.
*
* INPUTS: 
* int (*func)() - pointer to a function of the form func(item).
* LIST_PTR list - a pointer to a list.
*
* OUTPUTS: none.
*
* EXCEPTIONS:  If the list is NULL or the list is empty, return;
*
* DESIGN: 
* Based on calling listTestDeleteItemAll.
*
* NOTES: 
* The list itself is not freed.
*
**************************************************************************/

int listApplyFunc(func, item)
void (*func)();
char *item;
{
  (*func)(item);

  return 1;
}

void listFreeAllItems(func, list)
void (*func)();
LIST_PTR list;
{
  listTestDeleteItemAll(listApplyFunc, func, list);
}


/**************************************************************************
*
* FUNCTION: int listItemEq(a, b)
*
* DESCRIPTION: Simple Equal Test for listDeleteItem.
*
* INPUTS:
* char *a, *b;
*
* OUTPUTS: Returns 1 TRUE or 0 FALSE.
*
* DESIGN: return(a == b);
*
* NOTES:
*
**************************************************************************/

int listItemEq(a, b)
char *a, *b;
{
  return(a == b);
}


/**************************************************************************
*
* FUNCTION: listDeleteItem(item, list)
*
* DESCRIPTION: Removes an item from list.
*
* INPUTS:
* char *item; 
* LIST_PTR *list;
*
* OUTPUTS: none.
*
* DESIGN: call listTestDeleteItem with listItemEq test.
*
* NOTES: list is modified.
*
**************************************************************************/

void listDeleteItem(item, list)
char *item;
LIST_PTR list;
{
  listTestDeleteItem(listItemEq, item, list);
}


/**************************************************************************
*
* FUNCTION: listDeleteItemAll(item, list)
*
* DESCRIPTION: Removes an all such item from list.
*
* INPUTS:
* char *item; 
* LIST_PTR *list;
*
* OUTPUTS: none.
*
* DESIGN: call listTestDeleteItemAll with listItemEq test.
*
* NOTES: list is modified.
*
**************************************************************************/

void listDeleteItemAll(item, list)
char *item;
LIST_PTR list;
{
  listTestDeleteItemAll(listItemEq, item, list);
}


/**************************************************************************
*
* FUNCTION: int listMemberItem(item, list)
*
* DESCRIPTION: Tests if item is an element of list.
*
* INPUTS: 
* char *item - a pointer to an item of data.
* LIST_PTR list - a pointer to a list.
*
* OUTPUTS: 
* 0 - FALSE 
* 1 - TRUE
*
* EXCEPTIONS:  If the list is NULL or the list is empty, return;
*
* DESIGN: 
* Linearly search the list for a list element containing item. 
* If found the value 1 is returned, else the value 0 is returned.
*
* NOTES:
*
**************************************************************************/

int listMemberItem(item, list)
char *item;
LIST_PTR list;
{
  LIST_ELEM_PTR tmp;

  if (!list)
    return 0; /* False */

  tmp = list->first;

  while (tmp) {
    if (tmp->item == item)
      return 1; /* TRUE */
    tmp = tmp->next;
  }

  return 0; /* False */
}


/**************************************************************************
*
* FUNCTION: char *listMemReturnItem(func, param, list)
*
* DESCRIPTION:
* listMemReturnItem is a more general form of listMemberItem.
* listMemReturnItem will return the item (or one of the items) in list
* for which func(param, item) is non-zero, i.e. is TRUE.
* The function takes two arguments, the first is the param and the second is 
* an item of the list and returns an integer value. int func(param, item).
* If the functions does not satisfy any of the items in the list NULL
* is returned.
*
* INPUTS: 
* int (*func)();
* char *param - a pointer to a parameter that is passed to func.
* LIST_PTR list - a pointer to a list.
*
* OUTPUTS: 
* A pointer to an item in the list that satisfies func(param, item) 
* or NULL if no such item exists. 
*
* EXCEPTIONS:  If the list is NULL or the list is empty, NULL is returned.
*
* DESIGN: 
* Linearly search the list for a list element containing item, such that
* func(param, item) is non-zero. Then return the item.
*
* NOTES:
*
**************************************************************************/

char *listMemReturnItem(func, param, list)
int (*func)();
char *param;
LIST_PTR list;
{
  LIST_ELEM_PTR tmp, nextTmp;

  if (!list)
    return NULL;
  else {
    tmp = list->first;
    while (tmp) {
      nextTmp = tmp->next;
      if ((*func)(param, tmp->item))
	return tmp->item;
      else
	tmp = nextTmp;
    }
    return NULL;
  }
}


/**************************************************************************
*
* FUNCTION: int listIterateFromFirst(func, param, list)
*
* DESCRIPTION:
* listIterateFromFirst will call the function func with param on all of its
* elements stoping when the list is finished or when func returns 0 (ie
* FALSE). The function func will take two arguments the first is 
* param the second is an item of the set. func(param, item).
*
* listIterateFromFirst starts from the first item in the list and iterates
* forward through the items in the list.
*
* INPUTS: 
* int (*func)();
* char *param - a pointer to a parameter that is passed to func.
* LIST_PTR list - a pointer to a list.
*
* OUTPUTS: 
* listIterate returns 0 (FALSE) if the function func returns 0 (FALSE).
* Otherwise listIterate returns 1 (TRUE).
*
* EXCEPTIONS: 
*
* listIterate will return 0 (FASLE) if the list is NULL.
*
* DESIGN: 
* iterate through the list of elements calling func on each item.
* return when the list is finished or func has returned 0 (FALSE).
*
* NOTES:
*
* 11-Dec-90: fedor: 
* Added item test so holes generated by listDelete do not kill func call
*
*  3-Oct-90: fedor: 
* nextTmp allows the iteration to terminate gracefully if the list element
* is removed if func calls listDelete.
*
**************************************************************************/

int listIterateFromFirst(func, param, list)
int (*func)();
char *param;
LIST_PTR list;
{
  char *item;
  LIST_ELEM_PTR tmp, nextTmp;
  /* printf("-1-");fflush(stdout);(S.Thrun 93-5-1) */

  if (!list){
    /* printf("-2-");fflush(stdout);(S.Thrun 93-5-1) */
    return 0;
  }
  /* printf("-3-");fflush(stdout);(S.Thrun 93-5-1) */

  tmp = list->first;
  /* printf("-4-");fflush(stdout);(S.Thrun 93-5-1) */

  while (tmp) {
    /* printf("-5-");fflush(stdout);(S.Thrun 93-5-1) */
    item = tmp->item;
    nextTmp = tmp->next;
    /* printf("-6-");fflush(stdout);(S.Thrun 93-5-1) */
    if (item && !(*func)(param, item))
      return 0;
    /* printf("-7-");fflush(stdout);(S.Thrun 93-5-1) */
    tmp = nextTmp;
  }

  return 1;
}


/**************************************************************************
*
* FUNCTION: int listIterateFromLast(func, param, list)
*
* DESCRIPTION:
* listIterateFromLast will call the function func with param on all of its
* elements stoping when the list is finished or when func returns 0 (ie
* FALSE). The function func will take two arguments the first is 
* param the second is an item of the set. func(param, item).
*
* listIterateFromLast starts with thelast item in the list and iterates
* backwards through the list.
*
* INPUTS: 
* int (*func)();
* char *param - a pointer to a parameter that is passed to func.
* LIST_PTR list - a pointer to a list.
*
* OUTPUTS: 
* listIterateFromLast returns 0 (FALSE) if the function func returns 0 (FALSE).
* Otherwise listIterateFromLast returns 1 (TRUE).
*
* EXCEPTIONS: 
*
* listIterate will return 0 (FASLE) if the list is NULL.
*
* DESIGN: 
* iterate through the list of elements calling func on each item.
* return when the list is finished or func has returned 0 (FALSE).
*
* NOTES:
*
* 11-Dec-90: fedor: 
* Added item test so holes generated by listDelete do not kill func call
*
*  3-Oct-90: fedor: 
* previousTmp allows the iteration to terminate gracefully if the list element
* is removed if func calls listDelete.
*
**************************************************************************/

int listIterateFromLast(func, param, list)
int (*func)();
char *param;
LIST_PTR list;
{
  char *item;
  LIST_ELEM_PTR tmp, previousTmp;

  if (!list)
    return 0;

  tmp = list->last;

  while (tmp) {
    item = tmp->item;
    previousTmp = tmp->previous;
    if (item && !(*func)(param, item))
      return 0;
    tmp = previousTmp;
  }

  return 1;
}


/**************************************************************************
*
* FUNCTION: LIST_PTR listCopy(list)
*
* DESCRIPTION: Copies the given list.
*
* INPUTS: LIST_PTR list
*
* OUTPUTS: A pointer to the newly created list of type LIST_PTR.
* If there is an error NULL is returned.
*
* EXCEPTIONS: none.
*
* DESIGN: Iterate through the original list inserting the items into the
* new list.
*
* NOTES:
*
**************************************************************************/

int listCopyInsert(list, item)
LIST_PTR list;
char *item;
{
  listInsertItemFirst(item , list);

  return 1;
}

LIST_PTR listCopy(list)
LIST_PTR list;
{
  LIST_PTR newList;

  newList = listCreate();

  listIterateFromLast(listCopyInsert, newList, list);

  return newList;
}


/**************************************************************************/

/* List interface routines */

void listInsertItem(item, list)
char *item;
LIST_PTR list;
{
  listInsertItemFirst(item, list);
}

int listIterate(func, param, list)
int (*func)();
char *param;
LIST_PTR list;
{
  return(listIterateFromFirst(func, param, list));
}

int listLength(list)
LIST_PTR list;
{
  if (!list)
    return 0;

  return(list->length);
}


int listEqual(list1, list2)
LIST_PTR list1, list2;
{
  int good;
  LIST_ELEM_PTR a, b;

  if (list1 == list2)
    return 1;

  /* this is the same style test used in tms.c but it is not general ! */

  a = list1->first;
  b = list2->first;

  good = 1;
  while (good && a && b) {
    good = (a->item == b->item);
    a = a->next;
    b = b->next;
  }

  return(good && (a == NULL) && (b == NULL));
}

  
LIST_PTR listMake1(item)
char *item;
{
  LIST_PTR list;

  list = listCreate();

  listInsertItem(item, list);

  return list;
}

  
LIST_PTR listMake2(item1, item2)
char *item1, *item2;
{
  LIST_PTR list;

  list = listCreate();
  listInsertItem(item2, list);
  listInsertItem(item1, list);

  return list;
}


char *listFirst(list)
LIST_PTR list;
{
  if (list && list->first) {
    list->next = list->first;
    return(list->first->item);
  }
  else
    return NULL;
}

char *listLast(list)
LIST_PTR list;
{
  if (list && list->last) {
    return(list->last->item);
  }
  else
    return NULL;
}

char *listNext(list)
LIST_PTR list;
{
  if (list && list->next) {
    list->next = list->next->next;

    if (list->next)
      return(list->next->item);
  }

  return NULL;
}


void listInsertUnique(item, list)
char *item;
LIST_PTR list;
{
  if (!listMemberItem(item, list)) {
    listInsertItem(item, list);
  }
}
