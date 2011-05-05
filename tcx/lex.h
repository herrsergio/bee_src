
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
* FILE: lex.h
*
* ABSTRACT:
* Lexer include
*
* REVISION HISTORY
* 
* 04-Jun-92 Chad Darby, Field Robotics Center, CMU
*    Modified data structure such that it isn't a linked list
*    Removed FormatFunc from _TOK data structure
*    Modifed code to current FRC software standards
* 
* 10-Feb-89 Christopher Fedor, School of Computer Science, CMU
* Created.
*
******************************************************************************/

#ifndef INClex
#define INClex

typedef enum {LBRACE_TOK, RBRACE_TOK, COMMA_TOK, LT_TOK, GT_TOK,
		STAR_TOK, BANG_TOK, COLON_TOK, LBRACK_TOK, RBRACK_TOK,
		INT_TOK, STR_TOK, PRIMITIVE_TOK, EOS_TOK} TOKEN_ID_TYPE;

typedef struct _TOK{
  int loc;
  TOKEN_ID_TYPE type;
  union {
    char *str;
    int num;
  } value;
} TOKEN_TYPE, *TOKEN_PTR;

#endif INClex
