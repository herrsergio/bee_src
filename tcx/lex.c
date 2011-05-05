
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
* MODULE: lex
*
* FILE: lex.c
*
* ABSTRACT:
* 
* primitve lexer
*
* REVISION HISTORY
*
*  $Log: lex.c,v $
*  Revision 1.1.1.1  1996/09/22 16:46:01  rhino
*  General reorganization of the directories/repository, fusion with the
*  RWI software.
*
*  Revision 1.2  1994/10/22 18:47:33  tyson
*  VMS version. Fixed structure indexing (computation of the offsets
*  in a struct). Added signal handlers to a1, b1 tcxServer.
*
 * Revision 1.1.1.1  1994/03/31  08:35:04  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:30  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:28  rhino
 * test
 *
 * Revision 1.7  1993/03/12  20:58:42  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.6  1992/07/20  00:31:45  fedor
 * Added receive message style calls. Added data freeing calls.
 *
 * Revision 1.5  1992/07/10  16:04:56  fedor
 * Changes to compile for VxWorks. Also some changes to avoid compiler warnings.
 *
 * Revision 1.4  1992/07/05  13:06:48  fedor
 * Changed printf to fprintf(stderr,
 * Removed old list. Some file reorganization.
 *
* Revision 1.3  1992/06/29  17:34:25  darby
* 29-June-92  Chad Darby, Field Robotics Center, CMU
* Added logging mechanism for comment header
*
* 26-May-92  Chad Darby, Field Robotics Center, CMU
*    Removed the usage of linked lists
*    nextToken() tokenizes next character in string
*    freeToken(token) frees a token
*
* 13-Feb-89 Christopher Fedor, School of Computer Science, CMU
* Changed to lex the whole string at once.
*
* 10-Feb-89 Christopher Fedor, School of Computer Science, CMU
* Created.
*
* NOTES: 
* 17-Jun-91: fedor:
* Needs to be rewritten to take advantage of abstract data type routines
* and to start looking at memory use. Blah this is a mess!
*
******************************************************************************/

#ifdef VMS
#include "vms.h"
#endif                    

#ifdef VXWORKS
#include "/usr/vxworks/vx5.0.2b/h/vxWorks.h"
#include "/usr/vxworks/vx5.0.2b/h/stdioLib.h"
#include "/usr/vxworks/vx5.0.2b/h/string.h"
#include "/usr/vxworks/vx5.0.2b/h/ctype.h"
#else
#include "stdio.h" 
#include "string.h"
#include "ctype.h"
#endif

#include "lex.h"

#include "global.h"

#define NEW_TOKEN(token, typ) token = ((TOKEN_PTR)malloc(sizeof(TOKEN_TYPE)));\
				  token->type = typ;\
                                  token->loc = Global->currentLocationGlobal


/*  ungetToken() - backs up one position in the string */
void ungetToken(token)
TOKEN_PTR token;
{
  Global->ungetTokenGlobal = token;
}


/******************************************************************************
*
*  FUNCTION:  TOKEN_PTR readString(s)
*
*  DESCRIPTION:  reads a string into the current token's value
*
*  INPUTS: char string
*
*  OUTPUTS: returns the token w/ the string in token->value.str
*
*  EXCEPTIONS: none
*
*  DESIGN: read string of characters not include delimiters i.e. {}[]<>\0
*
*  NOTES: none
*
******************************************************************************/
TOKEN_PTR readString(s)
char *s;
{
  char *t2;
  TOKEN_PTR tmp;
  int i, amount;

  for (i=Global->currentLocationGlobal;
       (s[i] != ',' && s[i] != '}' && s[i] != '{' && s[i] != '[' 
       && s[i] != ']' && s[i] != ':' && s[i] != ' ' && s[i] != '>'
       && s[i] != '<' && s[i] != '\0');
       i++);

  amount = i - Global->currentLocationGlobal;
  NEW_TOKEN(tmp, STR_TOK);
  tmp->value.str = (char *)malloc(sizeof(char)*(amount+1));
  t2 = s + Global->currentLocationGlobal;
  strncpy(tmp->value.str, t2, amount);
  tmp->value.str[amount] = '\0';

  Global->currentLocationGlobal += amount;

  return tmp;
}


/******************************************************************************
*
*  FUNCTION:  TOKEN_PTR readInt(s)
*
*  DESCRIPTION:  reads the integers from a string s
*
*  INPUTS: char string s
*
*  OUTPUTS: returns the token w/ the integer in token->value.num
*
*  EXCEPTIONS: none
*
*  DESIGN: read string of characters not include delimiters i.s. {}[]<>\0
*
*  NOTES: none
*
******************************************************************************/
TOKEN_PTR readInt(s)
char *s;
{
  char *t, *t2;
  TOKEN_PTR tmp;
  int i, num, amount;

  for (i = Global->currentLocationGlobal;
       (isdigit(s[i]) && s[i] != ',' && s[i] != '}' && s[i] != '{' 
	&& s[i] != '[' && s[i] != ']' && s[i] != ':' && s[i] != ' ' 
	&& s[i] != '>' && s[i] != '<' && s[i] != '\0');
       i++);

  amount = i - Global->currentLocationGlobal;
  NEW_TOKEN(tmp, INT_TOK);
  t = (char *)malloc(sizeof(char)*(amount+1));
  t2 = s + Global->currentLocationGlobal;
  strncpy(t, t2, amount);
  t[amount] = '\0';

#ifndef VXWORKS
  tmp->value.num = atoi(t);
#else
  sscanf(t, "%d", &num);
  tmp->value.num = num;
#endif

  free(t);

  Global->currentLocationGlobal += amount;

  return tmp;
}    


/******************************************************************************
*
*  FUNCTION: void freeToken (tok)
*
*  DESCRIPTION: Takes a token and frees the memory allocated for it.
*
*  INPUTS: token of type TOKEN_PTR
*
*  OUTPUTS: none
* 
*  EXCEPTIONS: does nothing if input in NULL
*
*  DESIGN: Basically free(token) unless it has a string for value tok->value.str
*
*  NOTES: none
*
******************************************************************************/
void freeToken (tok)
TOKEN_PTR tok;
{
  if (tok) {
    if (tok->type == STR_TOK)
      free(tok->value.str);
    free(tok);
  }
}


/******************************************************************************
*
*  FUNCTION: TOKEN_PTR nextToken()
*
*  DESCRIPTION: reads a character and returns the tokenized result
*
*  INPUTS: none
*
*  OUTPUTS: TOKEN_PTR token w/ the token->type updated
*
*  EXCEPTIONS: none
*
*  DESIGN: checks each character against token table.  Then creates token
*          by using NEW_TOKEN macro.
*
*  NOTES: Assumes that currentString is defined.
*
******************************************************************************/
TOKEN_PTR nextToken()
{
  TOKEN_PTR token;

  if (Global->ungetTokenGlobal) {
    token = Global->ungetTokenGlobal;
    Global->ungetTokenGlobal = NULL;
    return token;
  }

  if (!Global->currentStringGlobal) {
    NEW_TOKEN(token, EOS_TOK);
    return token;
  }

 Start:
 switch(Global->currentStringGlobal[Global->currentLocationGlobal]) {
  case ' ':
  case '\t':
  case '\f':
    Global->currentLocationGlobal++;
    goto Start;
  case '{':
    NEW_TOKEN(token, LBRACE_TOK);
    Global->currentLocationGlobal++;
    break;
  case '}':
    NEW_TOKEN(token, RBRACE_TOK);
    Global->currentLocationGlobal++;
    break;
  case '[':
    NEW_TOKEN(token, LBRACK_TOK);
    Global->currentLocationGlobal++;
    break;
  case ']':
    NEW_TOKEN(token, RBRACK_TOK);
    Global->currentLocationGlobal++;
    break;
  case '*':
    NEW_TOKEN(token, STAR_TOK);
    Global->currentLocationGlobal++;
    break;
  case '!':
    NEW_TOKEN(token, BANG_TOK);
    Global->currentLocationGlobal++;
    break;
  case ',':
    NEW_TOKEN(token, COMMA_TOK);
    Global->currentLocationGlobal++;
    break;
  case '<':
    NEW_TOKEN(token, LT_TOK);
    Global->currentLocationGlobal++;
    break;
  case '>':
    NEW_TOKEN(token, GT_TOK);
    Global->currentLocationGlobal++;
    break;
  case ':':
    NEW_TOKEN(token, COLON_TOK);
    Global->currentLocationGlobal++;
    break;
  case '\0':
    NEW_TOKEN(token, EOS_TOK);
    Global->currentLocationGlobal++;
    break;
  default:
    if (isdigit(Global->currentStringGlobal[Global->currentLocationGlobal]))
      token = readInt(Global->currentStringGlobal);
    else 
      token = readString(Global->currentStringGlobal);
  }

  return token;
}


/******************************************************************************
*
*  FUNCTION: void printToken(token)
*
*  DESCRIPTION: takes a token and displays its type to stdout
*
*  INPUTS: TOKEN_PTR token
*
*  OUTPUTS: displays token->type to stdout
*
*  EXCEPTIONS: none
*
*  DESIGN:  checks token->type against token type and prints the type
*
*  NOTES: none
*
******************************************************************************/
void printToken(token)
TOKEN_PTR token;
{
  fprintf(stderr, "loc: %d: type: ", token->loc);

  switch(token->type) {
  case LBRACE_TOK:
    fprintf(stderr, "LBRACE_TOK\n");
    break;
  case RBRACE_TOK:
    fprintf(stderr, "RBRACE_TOK\n");
    break;
  case LBRACK_TOK:
    fprintf(stderr, "LBRACK_TOK\n");
    break;
  case RBRACK_TOK:
    fprintf(stderr, "RBRACK_TOK\n");
    break;
  case STAR_TOK:
    fprintf(stderr, "STAR_TOK\n");
    break;
  case BANG_TOK:
    fprintf(stderr, "BANG_TOK\n");
    break;
  case COMMA_TOK:
    fprintf(stderr, "COMMA_TOK\n");
    break;
  case LT_TOK:
    fprintf(stderr, "LT_TOK\n");
    break;
  case GT_TOK:
    fprintf(stderr, "GT_TOK\n");
    break;
  case COLON_TOK:
    fprintf(stderr, "COLON_TOK\n");
    break;
  case EOS_TOK:
    fprintf(stderr, "EOS_TOK\n");
    break;
  case INT_TOK:
    fprintf(stderr, "INT_TOK: %d\n", token->value.num);
    break;
  case STR_TOK:
    fprintf(stderr, "STR_TOK: %s\n", token->value.str);
    break;
  case PRIMITIVE_TOK:
    fprintf(stderr, "PRIMITIVE_TOK: %s\n", token->value.str);
    break;
  default:
    fprintf(stderr, "Token of Unknown type: %d\n", token->type);
    break;
  }
}


/******************************************************************************
*
*  FUNCTION: void initLex(s)
*
*  DESCRIPTION: Initializes the lexer the read first element in char string
*
*  INPUTS: char string s
*
*  OUTPUTS: none
*
*  EXCEPTIONS: none
*
*  DESIGN: Sets the currentLocation to 0. That is the index for the string.
*          Sets the currentString to s.
*
*  NOTES: This should be called before you lex a string
*
******************************************************************************/
void initLex(s)
char *s;
{
  Global->currentStringGlobal = s;
  Global->currentLocationGlobal = 0;
  Global->ungetTokenGlobal = NULL;
}

/*
main()
{
  TOKEN_PTR token;

  initLex("[abc, def]");

  token = nextToken();

  while (token->type != EOS_TOK) {
    printToken (token);
    freeToken(token);
    token = nextToken();
  }
  printToken (token);
}
*/


