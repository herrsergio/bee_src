
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



#include "collisionIntern.h"




int actualStackSize = 0;

struct node *head;
struct node *bottom;


/* This procedure initializes the whole stack-structure */
void initStack()
{
    head = (struct node *) malloc(sizeof(struct node));
    if (!head)
      { 
	printf("initStack : Nicht genug Speicher vorhanden");
	exit(3);
      }

    bottom = (struct node *) malloc(sizeof(struct node));
    if (!bottom)
      { 
	printf("initStack : Nicht genug Speicher vorhanden");
	exit(3);
      }

    head->next = bottom;
    head->prev = head;
    bottom->next = bottom;
    bottom->prev = head;
}

/* The following function inserts a value of type itemType at the top of
   the stack and increases the actualStackSize. If actualStackSize exceeds
   ACTUAL_MODE->stack_size then we delete the value at the bottom of
   the stack                                                               */
void push(struct itemType v)
{
    struct node *t = (struct node *) malloc(sizeof(struct node));
    if (!t)
      { 
	printf("push : Nicht mehr genug Speicher vorhanden");
	exit(3);
      }

    t->key.pos.x = v.pos.x;
    t->key.pos.y = v.pos.y;
    t->key.rot   = v.rot;
    t->next = head->next;
    head->next = t;
    t->next->prev = t;
    t->prev = head;
    actualStackSize += 1;
    
    if(actualStackSize > ACTUAL_MODE->stack_size)
      {
	deleteBottom();
	actualStackSize -= 1;
      }
	
}


/* This function pops the value at the top of the stack */
struct itemType pop()
{
    if(!isempty())
      {
	struct itemType tmp;
	struct node *t = head->next;
	
	head->next = t->next;
	t->next->prev = head;
	tmp.pos.x = t->key.pos.x;
	tmp.pos.y = t->key.pos.y;
	tmp.rot   = t->key.rot;
	free((void *)t);
	
	actualStackSize -= 1;
	return tmp;
      }
    else
      {
	printf("pop: Der Stack ist bereits leer \n");
	exit(1);
      }
} 


/* Service-function */
/* Deletes the value at the bottom of the stack */
void deleteBottom()
{
    struct node *t = bottom->prev;
    
    bottom->prev = t->prev;
    t->prev->next = bottom;
    free((void *)t);
}
    

/* Tests if the stack is empty */
int isempty()
{
    return (head->next == bottom);
}


void printStack()
{   
    struct node *tmpNode = head->next;
    
    printf("\n Stackgroesse: %i \n",actualStackSize);
    while(tmpNode != bottom)
	{
	    printf("(%f , %f, %f) \n",tmpNode->key.pos.x,tmpNode->key.pos.y,
		   tmpNode->key.rot);
	    tmpNode = tmpNode->next;
	}
    printf("\n");
}














