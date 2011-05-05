
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
* MODULE: Formatters
*
* FILE: parseFmttrs.c
*
* ABSTRACT:
* Parser and Printer for Formatter Data Structures
*
* REVISION HISTORY
*
*  $Log: parseFmttrs.c,v $
*  Revision 1.2  1997/04/01 16:51:58  thrun
*  nothing major
*
*  Revision 1.1.1.1  1996/09/22 16:46:01  rhino
*  General reorganization of the directories/repository, fusion with the
*  RWI software.
*
*  Revision 1.2  1994/10/22 18:47:39  tyson
*  VMS version. Fixed structure indexing (computation of the offsets
*  in a struct). Added signal handlers to a1, b1 tcxServer.
*
 * Revision 1.1.1.1  1994/03/31  08:35:05  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:30  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:29  rhino
 * test
 *
 * Revision 1.11  1993/03/12  20:59:03  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.10  1992/07/20  00:31:56  fedor
 * Added receive message style calls. Added data freeing calls.
 *
 * Revision 1.9  1992/07/10  16:05:04  fedor
 * Changes to compile for VxWorks. Also some changes to avoid compiler warnings.
 *
 * Revision 1.8  1992/07/05  13:21:40  fedor
 * Removed defns.h and changed list4 to list - old list removed.
 *
 * Revision 1.7  1992/07/05  13:07:01  fedor
 * Changed printf to fprintf(stderr,
 * Removed old list. Some file reorganization.
 *
 * Revision 1.6  1992/06/29  17:34:28  darby
 * 29-June-92  Chad Darby, Field Robotics Center, CMU
 * Added logging mechanism for comment header
 *
* 16-May-92 Chad Darby, Field Robotics Center, CMU
* modified parser such that it would parse these:
*    {int, *int}  ; pointer example
*    {int, *!}    ; self pointer
*
* 26-May-92 Chad Darby, Field Robotics Center, CMU
* Made changes such interface w/ new version of lexer.
* Actual changes includes:
*        o removed link list
*
* 10-Feb-89 Reid Simmons, School of Computer Science, CMU
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

#include "formatters.h"
#include "hash.h"
#include "lex.h"

#include "list.h"

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#include "global.h"

extern int printToken();
extern TOKEN_PTR nextToken();
extern void ungetToken();
extern void initLex();

extern FORMAT_PTR createIntegerFormat();

/* forward ref */
static FORMAT_PTR parse(); 


/******************************************************************************
*
*  FUNCTION: int findSameFormat (key, format}
*
*  DESCRIPTION:  returns hash table key for the specified format
*
*  INPUTS: char *key, FORMAT_PTR format
*
*  OUTPUTS: integer TRUE(1) or FALSE(0)
*
*  EXCEPTIONS: none
*
*  DESIGN: check if format type are same and integer
*
*  NOTES: none
*
******************************************************************************/
int findSameFormat (key, format)
char *key; FORMAT_PTR format;
{ 
  if ((format->type == Global->searchFormat->type) &&
      (format->formatter.i == Global->searchFormat->formatter.i)) {
    Global->foundKey = key;
    return FALSE;
  }
  else return TRUE;
}


/******************************************************************************
*
*  FUNCTION: searchHashTableForFormat
*
*  DESCRIPTION:  does table lookup for the specified format
*
*  INPUTS: FORMAT_PTR format
*
*  OUTPUTS: char *Global->foundKey
*
*  EXCEPTIONS: none
*
*  DESIGN: find the hash position for format
*
*  NOTES: none
*
******************************************************************************/

char *searchHashTableForFormat(format)
FORMAT_PTR format;
{ 
  Global->foundKey = NULL;
  Global->searchFormat = format;
  hashTableIterate(findSameFormat, Global->formatNamesTable);
  return Global->foundKey;
}

/******************************************************************************
*
*  FUNCTION: void parserError
*
*  DESCRIPTION:  displays the format parsing error and exits program
*
*  INPUTS: TOKEN_PTR badToken, char *expecting
*
*  OUTPUTS: error message
*
*  EXCEPTIONS: none
*
*  DESIGN: prints the bad token and the string it was expecting
*
*  NOTES: none
*
******************************************************************************/
void parserError(badToken, expecting)
TOKEN_PTR badToken;
char *expecting;
{ 
  int i;

  fprintf(stderr, "Format Parsing Error: Expected %s.\n%s\n", 
	  expecting, Global->parseString);
  for (i=0; i<badToken->loc; i++) 
    fprintf(stderr, " ");
  fprintf(stderr, "\n");

  exit(1);
}


/******************************************************************************
*
*  FUNCTION: FORMAT_PTR newFformatter(type, formatter)
*
*  DESCRIPTION:  creates a new formatter of given type
*
*  INPUTS: type, formatter
*
*  OUTPUTS: returns a new format
*
*  EXCEPTIONS: none 
*
*  DESIGN: create new formatter, assignments
*
*  NOTES: none
*
******************************************************************************/
FORMAT_PTR newFformatter(type, formatter)
FORMAT_CLASS_TYPE type;
FORMAT_PTR formatter;
{ 
  FORMAT_PTR newFormatter;

  newFormatter = NEW_FORMATTER();
  newFormatter->type = type;
  newFormatter->formatter.f = formatter;
  return newFormatter;
}


/******************************************************************************
*
*  FUNCTION: FORMAT_PTR newAformatter(type, array_size) 
*
*  DESCRIPTION:  creates a new array formatt
*
*  INPUTS: format type and array_size
*
*  OUTPUTS: returns the new format
*
*  EXCEPTIONS: none
*
*  DESIGN: create new format and then put array_size in first cell
*
*  NOTES:  First element in a FORMAT_ARRAY_PTR is the size of the array.
*
******************************************************************************/

FORMAT_PTR newAformatter(type, arraySize)
FORMAT_CLASS_TYPE type; 
int arraySize;
{ 
  FORMAT_PTR newFormatter;

  newFormatter = NEW_FORMATTER();
  newFormatter->type = type;
  newFormatter->formatter.a = NEW_FORMAT_ARRAY(arraySize);
  newFormatter->formatter.a[0].i = arraySize;
  return newFormatter;
}


/******************************************************************************
*
*  FUNCTION: FORMAT_PTR structFormat()
*
*  DESCRIPTION:  creates a list of items with the structure
*
*  INPUTS: none
*
*  OUTPUTS: completed list of formats
*
*  EXCEPTIONS: none
*
*  DESIGN: append each token to format list, except for commas
*
*  NOTES: none
*
******************************************************************************/
FORMAT_PTR structFormat()
{ 
  FORMAT_PTR form;
  LIST_PTR formatList;
  int  numFormats, i;
  TOKEN_PTR token;

  formatList = listCreate();

  listInsertItemFirst(parse(TRUE),formatList);
  numFormats = 1;

  token = nextToken();
  while (token->type == COMMA_TOK) {
    freeToken(token);
    listInsertItemFirst(parse(TRUE),formatList);
    token = nextToken();
    numFormats++;
  }

   ungetToken(token);

  form = newAformatter(structFMT, numFormats+1);

  /* Index from high to low since "formatList" 
     has formatters in reverse order */

  for(i=numFormats;i>0;i--) {
    form->formatter.a[i].f = ((FORMAT_PTR)listPopItem(formatList)); 
  }

  return form;
}


/******************************************************************************
*
*  FUNCTION: FORMAT_PTR fixedArrayFormat
*
*  DESCRIPTION:  creates a format list for items in fixed array
*
*  INPUTS: none
*
*  OUTPUTS: format list
*
*  EXCEPTIONS: none
*
*  DESIGN: for each token, append to format list...except commas
*
*  NOTES: none
*
******************************************************************************/
FORMAT_PTR fixedArrayFormat()
{ 
  FORMAT_PTR form, nextFormat;
  TOKEN_PTR token, tmp;
  LIST_PTR argList;
  int numberOfIndexes, goAhead, arrayIndex;

  nextFormat = parse(FALSE); /* Formatter */

  token = nextToken();
  if (token->type != COLON_TOK)
    parserError(token, "':'");
  else freeToken(token);

  argList = listCreate();
  tmp = nextToken();
  numberOfIndexes = 0;
  goAhead = TRUE;
  do {
    if (tmp->type != INT_TOK) 
      parserError(tmp, "an integer value");
    else {
      numberOfIndexes++;
      listInsertItemLast(tmp, argList);

      tmp = nextToken();

      if (tmp->type == COMMA_TOK) {
	freeToken(tmp);
	tmp = nextToken();
	goAhead = TRUE;
      } else if (tmp->type == RBRACK_TOK) {
	freeToken(tmp);
	goAhead = FALSE;
      }
      else 
	parserError(tmp, "a ',' or ']'");
    }
  } while (goAhead);

  form = newAformatter(fixedArrayFMT, numberOfIndexes+2);
  form->formatter.a[1].f = nextFormat;

  /* this time munch tokens */
  numberOfIndexes += 2;
  token = (TOKEN_PTR)listFirst(argList);

  for (arrayIndex=2; arrayIndex < numberOfIndexes; arrayIndex++) {
    form->formatter.a[arrayIndex].i = token->value.num;
    freeToken(token);
    token = (TOKEN_PTR)listNext(argList);
  }
  listFree(argList);
  return form;
}


/******************************************************************************
*
*  FUNCTION: FORMAT_PTR varArrayFormat()
*
*  DESCRIPTION:  makes list of formats for items in variable length array
*
*  INPUTS: none
*
*  OUTPUTS: format list
*
*  EXCEPTIONS: none
*
*  DESIGN: for each token in array, put format in format list, except commas
*
*  NOTES: none
*
******************************************************************************/
FORMAT_PTR varArrayFormat()
{ 
  FORMAT_PTR form, nextFormat;
  TOKEN_PTR token, tmp;
  LIST_PTR argList;
  int numberOfIndexes, goAhead, arrayIndex;

  nextFormat = parse(FALSE); /* Formatter */

  token = nextToken();
  if (token->type != COLON_TOK)
    parserError(token, "':'");
  else freeToken(token);

  argList = listCreate();
  tmp = nextToken();
  numberOfIndexes = 0;
  goAhead = TRUE;
  do {
    if (tmp->type != INT_TOK) 
      parserError(tmp, "an integer value");
    else {
      numberOfIndexes++;
      listInsertItemLast(tmp, argList);

      tmp = nextToken();

      if (tmp->type == COMMA_TOK) {
	freeToken(tmp);
	tmp = nextToken();
	goAhead = TRUE;
      } else if (tmp->type == GT_TOK) {
	freeToken(tmp);
	goAhead = FALSE;
      }
      else 
	parserError(tmp, "a ',' or '>'");
    }
  } while (goAhead);

  form = newAformatter(varArrayFMT, numberOfIndexes+2);
  form->formatter.a[1].f = nextFormat;

  /* this time munch tokens */
  numberOfIndexes += 2;
  token = (TOKEN_PTR)listFirst(argList);

  for (arrayIndex=2; arrayIndex < numberOfIndexes; arrayIndex++) {
    form->formatter.a[arrayIndex].i = token->value.num;
    freeToken(token);
    token = (TOKEN_PTR)listNext(argList);
  }
  listFree(argList);
  return form;
}


/******************************************************************************
*
*  FUNCTION: FORMAT_PTR selfPtrFormat()
*
*  DESCRIPTION:  returns a format pointer that signifies it is a self pointer
*
*  INPUTS: none
*
*  OUTPUTS: new format
*
*  EXCEPTIONS: none
*
*  DESIGN:
*
*  NOTES:  none
*
******************************************************************************/
FORMAT_PTR selfPtrFormat()
{ 
  return newFformatter(pointerFMT, NULL);
}


/******************************************************************************
*
*  FUNCTION: FORMAT_PTR prtFormat()
*
*  DESCRIPTION:  returns a format pointer that signifies it is a pointer
*                within a structure
*
*  INPUTS: none
*
*  OUTPUTS: new format
*
*  EXCEPTIONS: none
*
*  DESIGN:
*
*  NOTES: none
*
******************************************************************************/
FORMAT_PTR ptrFormat()
{ 
  return newFformatter(pointerFMT, parse(FALSE));
}


/******************************************************************************
*
*  FUNCTION: lengthFormat(token)
*
*  DESCRIPTION:  returns an integer format for a tokens value
*
*  INPUTS: TOKEN_PTR token
*
*  OUTPUTS: new integer format
*
*  EXCEPTIONS: none
*
*  DESIGN:
*
*  NOTES: none
*
******************************************************************************/
FORMAT_PTR lengthFormat(token)
TOKEN_PTR token;
{ 
  return createIntegerFormat(lengthFMT, token->value.num);
}


/******************************************************************************
*
*  FUNCTION: namedFormat(token)
*
*  DESCRIPTION:  return the format for a token
*
*  INPUTS: token
*
*  OUTPUTS: format
*
*  EXCEPTIONS: exits if format is not registered
*
*  DESIGN: do a hash lookup on token name, if not there then generate error
*
*  NOTES: none
*
******************************************************************************/
FORMAT_PTR namedFormat(token)
TOKEN_PTR token;
{ 
  FORMAT_PTR Form;

  Form = (FORMAT_PTR)hashTableFind(token->value.str, 
				   Global->formatNamesTable);
  if (!Form) {
    fprintf(stderr, "Format %s is not registered\n%s\n", 
	    token->value.str, Global->parseString);
    exit(1);
  }

  return Form;
}


/******************************************************************************
*
*  FUNCTION: FORMAT_PTR primitiveFormat(token)
*
*  DESCRIPTION:  returns the format of a primitive (i.e. int, char...)
*
*  INPUTS: TOKEN_PTR token
*
*  OUTPUTS: format
*
*  EXCEPTIONS: none
*
*  DESIGN: makes a call to namedFormat
*
*  NOTES: none
*
******************************************************************************/
FORMAT_PTR primitiveFormat(token)
TOKEN_PTR token;
{ 
  return namedFormat(token);
}



/******************************************************************************
*
*  FUNCTION: parseFormatString(formString)
*
*  DESCRIPTION:  parse an input string and returns the format of string
*
*  INPUTS: char *formString
*
*  OUTPUTS: returns formatterp
*
*  EXCEPTIONS: none
*
*  DESIGN: lex the tokens and parse each one on the fly
*
*  NOTES: none
*
******************************************************************************/
FORMAT_PTR parseFormatString(formString) 
char *formString;
{ 
  FORMAT_PTR formatter;

  Global->parseString = formString;
  initLex(formString);
  formatter = parse(FALSE);
  return formatter;
}


/******************************************************************************
*
*  FUNCTION: FORMAT_PTR parse
*
*  DESCRIPTION:  Parse the current token 
*
*  INPUTS: int withinStructFlag (see below)
*
*  OUTPUTS: return the format
*
*  EXCEPTIONS: none
*
*  DESIGN: switch(token->type) and get new formats for string recursively
*
*  NOTES:  "withinStructFlag" is TRUE if the enclosing format is "{...}" 
*
******************************************************************************/
FORMAT_PTR parse(withinStructFlag) 
int withinStructFlag;
{ 
  TOKEN_PTR token, tmp;
  FORMAT_PTR returnForm = NULL;

  token = nextToken();

  switch(token->type) {
  case LBRACE_TOK:
    freeToken(token);
    returnForm = structFormat();
    token = nextToken();
    if (token->type != RBRACE_TOK) parserError(token, "'}'");
    else freeToken(token);
    break;
  case LBRACK_TOK:
    returnForm = fixedArrayFormat();
    break;
  case LT_TOK:
    if (!withinStructFlag)
      parserError(token,
	  "var array format '<..>' embedded within a structure '{..}'");
    else
      returnForm = varArrayFormat();
    break;
  case STAR_TOK:
    tmp = nextToken();
    if (tmp->type == BANG_TOK) {
      freeToken(tmp);
      if (!withinStructFlag) 
	parserError(token, 
		    "self pointer '*!' embedded within a structure '{..}'");
      else {
	returnForm = selfPtrFormat();
      }
    }
    else {
      ungetToken(tmp);
      returnForm = ptrFormat();
    }
    break;
  case INT_TOK:
    returnForm = lengthFormat(token);
    break;
  case STR_TOK:
    returnForm = namedFormat(token);
    break;
  case PRIMITIVE_TOK:
    returnForm = primitiveFormat(token);
    break;
  case EOS_TOK:
    parserError(token, 
		"additional tokens; premature end of string encountered");
    break;
  default:
    parserError(token, "a different token type");
  }

  return returnForm;
}




