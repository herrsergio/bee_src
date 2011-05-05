
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
* PROJECT: Carnegie Mellon Planetary Rover Project
*          Task Control Architecture
*
* (c) Copyright 1991 Christopher Fedor and Reid Simmons.  All rights reserved.
* 
* MODULE: formatters
*
* FILE: primFmttrs.c
*
* ABSTRACT:
*
* Primitive Data Formatters
*
* Formatter functions take five arguments -- an "op," a pointer to the user's
*"DataStruct" (the top-level structure being encoded or decoded), the "DStart" 
* (where in "DataStruct" to start encoding/decoding), a pointer to a "Buffer" 
* (the data that will be sent/was received over the communications channel), 
* and the "BStart" (where in "Buffer" to start encoding/decoding).
*
* The functions all return an integer, whose interpretation 
* depends on the "op."
*
* The "op"s are:
*
* Encode:  Linearize the user's "DataStruct" (starting at "DStart") and place 
*          the bytes in the "Buffer" (starting at "BStart").  
*          Returns the number of bytes linearized.
*
* ELength: Returns the number of bytes that would be linearized by "Encode."
*
* Decode:  Using the "Buffer" (starting at "BStart"), fill in the user's
*          "DataStruct" structure (starting at "DStart").  
*          Encode and Decode are inverse functions.
*          Returns the number of bytes used up from the "Buffer".
*
* ALength: Returns the Advance length, that is, the number of bytes taken 
*          up by the top-level structure.  Equal to the "sizeof" the 
*          data type, which is not necessarily equal to the ELength.  
*          For example, STRING_FMT has an ALength
*          of 4 (i.e., sizeof(char *)), but its ELength depend on the number of
*          characters in the string.
*
* RLength: Restritive Length. Similar to ALength used by the sun4 for
*          the mostRestiveElement call. Matrices most restrictive element 
*          is actually 4 but ALength returned the sizeof the struct which
*          is 24. This is mainly because matrix code uses special transfer
*          functions.
*
* SimpleType: Returns TRUE (1) if the format is a fixed length type (i.e., the
*          ELength always equals the ALength).
*
* DPrint:  Print out the data, using the "Print_" functions defined in file
*          print_data.  For DPrint, the "buffer" argument is the stream to be
*          printed on, and the "bstart" argument is the "keep_with_next" number
*          of characters needed to complete a line (see file "print_data").
*
* DFree:   Free the data structure if it was "malloc"ed while doing a
*          "Decode" operation. 
*
* REVISION HISTORY:
*
* 28-Feb-91 Christopher Fedor at School of Computer Science, CMU
* Removed NULL matrix as a silly idea since a matrix is never really NULL.
* A matrix with all bounds equal 0 is still a matrix of one element.
* Made sure all printfs were follwed by a fflush(stdout) for lisp code.
*
* Added "sub-image" primitive formatters:
* siucmat, sicmat, sismat, siimat, silmat, sifmat, sidmat.
*
* 27-Feb-91 Christopher Fedor at School of Computer Science, CMU
* Added code to all matrix formatters so that a NULL matrix could be
* sent on user defined error conditions. A NULL matrix is one with
* all bounds equal 0 but mat storage equal NULL.
*
* 28-Oct-90 Christopher Fedor at School of Computer Science, CMU
* Removed logging and print data routines to minimize module size.
*
* 19-Oct-90 Christopher Fedor at School of Computer Science, CMU
* Added RLength to deal with sun4 transfer problem of "{int, dmat}".
*
* 15-Oct-90 Christopher Fedor at School of Computer Science, CMU
* Defined byteFormat, charFormat, shortFormat, intFormat, longFormat,
* floatFormat and doubleFormat for mapPrint to avoid calling
* parseFormatString in this file. 
*
* Added code to deal with NULL or empty strings in STR_Trans.
* If a NULL or empty string is encoded it is decoded as a NULL.
* "string" decodes to a pointer to NULL
* {string} decodes as a struct whose element's value is a NULL pointer.
*
* 18-Jul-90 Reid Simmons at School of Computer Science, CMU
* Added code for freeing data structures.
*
*  3-Apr-90 Christopher Fedor at School of Computer Science, CMU
* Added expanded matrix support for perception.
*
*  9-Mar-89 Reid Simmons at School of Computer Science, CMU
* Added SimpleType and DPrint ops
*
* 28-Feb-89 Christopher Fedor at School of Computer Science, CMU
* Changed newfmat to newfmat2 to avoid GIL conflict
*
*  9-Feb-89 Reid Simmons at School of Computer Science, CMU
* Split into 2 files -- "PrimFmttrs" and "Formatters"
*
*  6-Feb-89 Reid Simmons at School of Computer Science, CMU
* Added structured formatters.
*
*  3-Feb-89 Reid Simmons at School of Computer Science, CMU
* Standardized C and LISP interface to formatter functions.
*
*  2-Jan-89 Reid Simmons at School of Computer Science, CMU
* Added INT and FLOAT formats for LISP interface.
*
*    Dec-88 Christopher Fedor at School of Computer Science, CMU
* created.
*
*****************************************************************************/

#ifdef VMS
#include "vms.h"                                 
#endif

#ifdef VXWORKS
#include "/usr/vxworks/vx5.0.2b/h/vxWorks.h"
#include "/usr/vxworks/vx5.0.2b/h/stdioLib.h"
#else
#include "stdio.h" 
#endif

/* #include "tcaInternal.h"*/

#include "tcx.h"
#include "formatters.h"
#include "printData.h"

#include "tcaMatrix.h"

#if 0
extern TCA_REF_PTR tcaRefCreate();
#endif

FORMAT_PTR byteFormat;
FORMAT_PTR charFormat;
FORMAT_PTR shortFormat;
FORMAT_PTR intFormat;
FORMAT_PTR longFormat;
FORMAT_PTR floatFormat;
FORMAT_PTR doubleFormat;

void (*dPrintSTR_FN)();
void (*dPrintFORMAT_FN)();
void (*dPrintMAP_FN)();
void (*dPrintTCA_FN)();
void (*dPrintCHAR_FN)();
void (*dPrintSHORT_FN)();
void (*dPrintLONG_FN)();
void (*dPrintINT_FN)();
void (*dPrintFLOAT_FN)();
void (*dPrintDOUBLE_FN)();
void (*dPrintBYTE_FN)();
void (*dPrintTWOBYTE_FN)();

 /* 7-Nov-89: fedor: kept for REF macro - both should go away */
typedef char *StringType;


/******************************************************************************
*
* FUNCTION: int tcaStrLen(s)
*
* DESCRIPTION: Returns strlen of string or 0 if string, s is NULL.
*
* INPUTS: char *s;
*
* OUTPUTS: int
*
******************************************************************************/

int tcaStrLen(s)
char *s;
{
  if (s)
    return(strlen(s));
  else
    return 0;
}



/******************************************************************************
*
* FUNCTION: int STR_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
* "StringType" is a null terminated array of characters.
* Decodes to:  length of string, Character-List
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* GENERIC_DATA_PTR datastruct; 
* int dstart; 
* char *buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int STR_Trans(TRANS_OP_TYPE op, DATA_PTR datastruct, int dstart,
	      char *buffer, int bstart)
{ 
  char *string, tmp;
  int current_byte, length, intVal;

  switch(op) {
  case Encode:
    current_byte = bstart;
    string = REF(char *, datastruct, dstart);
    length = tcaStrLen(string)*sizeof(char);
    intVal = htonInt(length);
    TO_BUFFER_AND_ADVANCE(&intVal, buffer, current_byte, sizeof(int));

    if (length) {
      TO_BUFFER_AND_ADVANCE(string, buffer, current_byte, length);
    }
    else {
      TO_BUFFER_AND_ADVANCE("Z", buffer, current_byte, sizeof(char));
    }

    return current_byte - bstart;

  case Decode:
    current_byte = bstart;
    FROM_BUFFER_AND_ADVANCE(&intVal, buffer, current_byte, sizeof(int));
    length = ntohInt(intVal);
    
    if (length) {
      string = (char *)tcaMalloc((unsigned)(length+1));
      bcopy((char *)&string, datastruct+dstart, sizeof(char *));
      FROM_BUFFER_AND_ADVANCE(string, buffer, current_byte, length);
      string[length/sizeof(char)] = '\0';
    }
    else {
      string = NULL;
      bcopy((char *)&string, datastruct+dstart, sizeof(char *));
      FROM_BUFFER_AND_ADVANCE(&tmp, buffer, current_byte, 1);
    }

    return current_byte - bstart;

  case ELength: 
    length = tcaStrLen(REF(char *, datastruct, dstart)) * sizeof(char);

    if (length)
      return(length+sizeof(int));
    else
      return(sizeof(char)+sizeof(int));

  case RLength:
  case ALength: 
    return sizeof(char *);

  case SimpleType: 
    return FALSE;

  case DPrint: 
    dPrintSTR_FN((FILE *)buffer, REF(StringType, datastruct, dstart), bstart);
    return 1; 

  case DFree:
    /* Free the string */
    string = REF(char *, datastruct, dstart);
    if (string)
      tcaFree(string);
    return TRUE;

  default: 
    fprintf(stderr, "STR_Trans: undefined op: %d\n", op); 
  }
  return FALSE; /* should not get here */
}


/******************************************************************************
*
* FUNCTION: int FORMAT_Trans_Encode(format, buffer, bstart)
*
* DESCRIPTION: 
* A format is a (recursive) structure of {type, union{int, array, format}}
*
* INPUTS: 
* FORMAT_PTR format;
* char *buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

static int FORMAT_Trans_Encode(FORMAT_PTR format, char *buffer, int bstart)
{ 
  int i, current_byte, intVal;
  char PtrVal;
  FORMAT_ARRAY_PTR format_array;

  current_byte = bstart;

  intVal = htonInt(format->type);
  TO_BUFFER_AND_ADVANCE(&intVal, buffer, current_byte, sizeof(int));

  switch (format->type) {
  case lengthFMT: 
  case primitiveFMT:
    intVal = htonInt(format->formatter.i);
    TO_BUFFER_AND_ADVANCE(&intVal, buffer, current_byte, sizeof(int));
    break;
  case pointerFMT: 
    PtrVal = (format->formatter.f) ? 'Z' : '\0'; 
    TO_BUFFER_AND_ADVANCE(&PtrVal, buffer, current_byte, sizeof(char));
    if (format->formatter.f)
      current_byte += FORMAT_Trans_Encode(format->formatter.f, 
					  buffer, current_byte);
    break;
  case structFMT: 
    format_array = format->formatter.a;
    intVal = htonInt(format_array[0].i);
    TO_BUFFER_AND_ADVANCE(&intVal, buffer, current_byte, sizeof(int));
    for(i=1;i<format_array[0].i;i++)
      current_byte += FORMAT_Trans_Encode(format_array[i].f, buffer, 
					  current_byte);
    break;
  case varArrayFMT:
  case fixedArrayFMT:
    format_array = format->formatter.a;
    intVal = htonInt(format_array[0].i);
    TO_BUFFER_AND_ADVANCE(&intVal, buffer, current_byte, sizeof(int));
    current_byte += FORMAT_Trans_Encode(format_array[1].f, buffer, 
					current_byte);
    for(i=2;i<format_array[0].i;i++) {
      intVal = htonInt(format_array[i].i);
      TO_BUFFER_AND_ADVANCE(&intVal, buffer, current_byte, sizeof(int));
    }
    break;
  }
  return current_byte - bstart;
}


/******************************************************************************
*
* FUNCTION: int FORMAT_Trans_Decode(format, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* FORMAT_PTR format;
* char *buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

static int FORMAT_Trans_Decode(FORMAT_PTR format, char *buffer, int bstart)
{ 
  int i, current_byte, array_size, intVal;
  char PtrVal;
  FORMAT_ARRAY_PTR format_array;

  current_byte = bstart;

  FROM_BUFFER_AND_ADVANCE(&intVal, buffer, current_byte, sizeof(int));
  format->type = ntohInt(intVal);

  switch (format->type) {
  case lengthFMT: 
  case primitiveFMT:
    FROM_BUFFER_AND_ADVANCE(&intVal, buffer, current_byte, sizeof(int));
    format->formatter.i = ntohInt(intVal);
    break;

  case pointerFMT: 
    FROM_BUFFER_AND_ADVANCE(&PtrVal, buffer, current_byte, sizeof(char));
    if (PtrVal == '\0') format->formatter.f = NULL;
    else {
      format->formatter.f = NEW_FORMATTER();
      current_byte += FORMAT_Trans_Decode(format->formatter.f,
					  buffer, current_byte);
    }
    break;

  case structFMT: 
    FROM_BUFFER_AND_ADVANCE(&intVal, buffer, current_byte, sizeof(int));
    array_size = ntohInt(intVal);
    format_array = format->formatter.a = NEW_FORMAT_ARRAY(array_size);
    format_array[0].i = array_size;
    for(i=1;i<array_size;i++) {
      format_array[i].f = NEW_FORMATTER();
      current_byte += FORMAT_Trans_Decode(format_array[i].f, buffer,
					  current_byte);
    }
    break;

  case varArrayFMT:
  case fixedArrayFMT:
    FROM_BUFFER_AND_ADVANCE(&intVal, buffer, current_byte, sizeof(int));
    array_size = ntohInt(intVal);
    format_array = format->formatter.a = NEW_FORMAT_ARRAY(array_size);
    format_array[0].i = array_size;
    format_array[1].f = NEW_FORMATTER();
    current_byte += FORMAT_Trans_Decode(format_array[1].f, buffer, 
					current_byte);
    for(i=2;i<format_array[0].i;i++) {
      FROM_BUFFER_AND_ADVANCE(&intVal, buffer, current_byte, sizeof(int));
      format_array[i].i = ntohInt(intVal);
    }
    break;
  }
  return current_byte - bstart;
}


/******************************************************************************
*
* FUNCTION: int FORMAT_Trans_ELength(format) 
*
* DESCRIPTION:
*
* INPUTS: FORMAT_PTR format;
*
* OUTPUTS: int
*
******************************************************************************/

static int FORMAT_Trans_ELength(FORMAT_PTR format)
{ 
  int i, size;
  FORMAT_ARRAY_PTR format_array;

  size = sizeof(int);

  switch (format->type) {
  case lengthFMT: 
  case primitiveFMT: 
    size += sizeof(int); 
    break;
  case pointerFMT: 
    size += sizeof(char);
    if (format->formatter.f) size += FORMAT_Trans_ELength(format->formatter.f);
    break;
  case structFMT: 
    format_array = format->formatter.a;
    size += sizeof(int);
    for(i=1;i<format_array[0].i;i++)
      size += FORMAT_Trans_ELength(format_array[i].f);
    break;
  case varArrayFMT:
  case fixedArrayFMT:
    format_array = format->formatter.a;
    size += (sizeof(int) + FORMAT_Trans_ELength(format_array[1].f)
	     + (format_array[0].i-2)*sizeof(int));
    break;
  }

  return size;
}


/******************************************************************************
*
* FUNCTION: int FORMAT_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* char *buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int FORMAT_Trans(TRANS_OP_TYPE op, DATA_PTR datastruct, int dstart,
		 char *buffer, int bstart)
{ 
  FORMAT_PTR format;
  int size, null_flag = -1;

  switch(op) {
  case Encode: 
    format = REF(FORMAT_PTR, datastruct, dstart);
    if (format == NULL) {
      bcopy((char *)&null_flag, buffer+bstart, sizeof(int));
      return sizeof(int);
    }
    else 
      return FORMAT_Trans_Encode(format, buffer, bstart);
  case Decode: 
    bcopy(buffer+bstart, (char *)&null_flag, sizeof(int));
    if (null_flag == -1) {
      format = NULL;
      size = sizeof(int);
    }
    else {
      format = NEW_FORMATTER();
      size = FORMAT_Trans_Decode(format, buffer, bstart);
    }
    bcopy((char *)&format, datastruct+dstart, sizeof(FORMAT_PTR));
    return size;
  case ELength: 
    format = REF(FORMAT_PTR, datastruct, dstart);
    if (format) 
      return FORMAT_Trans_ELength(format);
    else 
      return sizeof(int);

  case RLength:
  case ALength: 
    return sizeof(FORMAT_PTR);

  case SimpleType: 
    return FALSE;

  case DPrint: 
    format = REF(FORMAT_PTR, datastruct, dstart);
    dPrintFORMAT_FN((FILE *)buffer, format, bstart);

    return 1; 

  case DFree:
    (void)fprintf(stderr,"\nWARNING: Format Data Structure Not Freed.\n");
#ifdef LISP
    (void)fflush(stderr);
#endif
    return FALSE; /* nothing to free */

  default: 
    fprintf(stderr, "FORMAT_Trans: undefined op: %d\n", op); 
  }
  return FALSE; /* should not get here */
}


/******************************************************************************
*
* FUNCTION:
* int ucmatLength(lb1, ub1, lb2, ub2)
* int cmatLength(lb1, ub1, lb2, ub2)
* int smatLength(lb1, ub1, lb2, ub2)
* int imatLength(lb1, ub1, lb2, ub2)
* int lmatLength(lb1, ub1, lb2, ub2)
* int fmatLength(lb1, ub1, lb2, ub2)
* int dmatLength(lb1, ub1, lb2, ub2)
*
* DESCRIPTION: Calculate matrix encode/decode size.
*
* INPUTS: int lb1, ub1, lb2, ub2;
*
* OUTPUTS: int
*
******************************************************************************/

static int ucmatLength(int lb1, int ub1, int lb2, int ub2)
{ 
  return sizeof(unsigned char) * (ub1 - lb1 + 1) * (ub2 - lb2 + 1);
}

static int cmatLength(int lb1, int ub1, int lb2, int ub2)
{ 
  return sizeof(char) * (ub1 - lb1 + 1) * (ub2 - lb2 + 1);
}

static int smatLength(int lb1, int ub1, int lb2, int ub2)
{ 
  return sizeof(short) * (ub1 - lb1 + 1) * (ub2 - lb2 + 1);
}

static int imatLength(int lb1, int ub1, int lb2, int ub2)
{ 
  return sizeof(int) * (ub1 - lb1 + 1) * (ub2 - lb2 + 1);
}

static int lmatLength(int lb1, int ub1, int lb2, int ub2)
{ 
  return sizeof(long) * (ub1 - lb1 + 1) * (ub2 - lb2 + 1);
}

static int fmatLength(int lb1, int ub1, int lb2, int ub2)
{ 
  return sizeof(float) * (ub1 - lb1 + 1) * (ub2 - lb2 + 1);
}

static int dmatLength(int lb1, int ub1, int lb2, int ub2)
{ 
  return sizeof(double) * (ub1 - lb1 + 1) * (ub2 - lb2 + 1);
}



static void transferMapBounds (int lb1, int ub1, int lb2, int ub2, 
			       char *buffer, int *currentBytePtr)
{
  int intVal;

  intVal = htonInt(lb1);
  TO_BUFFER_AND_ADVANCE(&intVal, buffer, *currentBytePtr, sizeof(int));
  intVal = htonInt(ub1);
  TO_BUFFER_AND_ADVANCE(&intVal, buffer, *currentBytePtr, sizeof(int));
  intVal = htonInt(lb2);
  TO_BUFFER_AND_ADVANCE(&intVal, buffer, *currentBytePtr, sizeof(int));
  intVal = htonInt(ub2);
  TO_BUFFER_AND_ADVANCE(&intVal, buffer, *currentBytePtr, sizeof(int));
}

static void retrieveMapBounds (int *lb1, int *ub1, int *lb2, int *ub2,
			       char *buffer, int *currentBytePtr)
{
  int intVal;

  FROM_BUFFER_AND_ADVANCE(&intVal, buffer, *currentBytePtr, sizeof(int));
  *lb1 = ntohInt(intVal);
  FROM_BUFFER_AND_ADVANCE(&intVal, buffer, *currentBytePtr, sizeof(int));
  *ub1 = ntohInt(intVal);
  FROM_BUFFER_AND_ADVANCE(&intVal, buffer, *currentBytePtr, sizeof(int));
  *lb2 = ntohInt(intVal);
  FROM_BUFFER_AND_ADVANCE(&intVal, buffer, *currentBytePtr, sizeof(int));
  *ub2 = ntohInt(intVal);
}

/******************************************************************************
*
* FUNCTION: int UCMAT_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* char *buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int UCMAT_Trans(TRANS_OP_TYPE op, DATA_PTR datastruct, int dstart,
		char *buffer, int bstart)
{ 
  ucmat *Map;
  int length, current_byte;
  int lb1, ub1, lb2, ub2, error;
  
  switch(op) {
  case Encode:
    current_byte = bstart;
    Map = (ucmat *)(datastruct+dstart);
    length = ucmatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2);
    transferMapBounds(Map->lb1, Map->ub1, Map->lb2, Map->ub2, 
		      buffer, &current_byte);
    TO_BUFFER_AND_ADVANCE(&(Map->el[Map->lb1][Map->lb2]), 
			   buffer, current_byte, length);
    return current_byte - bstart;

  case Decode:
    current_byte = bstart;
    retrieveMapBounds(&lb1, &ub1, &lb2, &ub2, buffer, &current_byte);
    Map = (ucmat *)(datastruct+dstart);
    *Map = newucmat2(lb1, ub1, lb2, ub2, &error);
    if (error)
      tcaModError("newucmat2 error");
    if (!Map) { 
      tcaModError("UCMAT_Trans: Decode. Map cannot be allocated");
    }
    else { 
      FROM_BUFFER_AND_ADVANCE(&(Map->el[lb1][lb2]), buffer, current_byte, 
			      ucmatLength(lb1, ub1, lb2, ub2));
      return current_byte - bstart;
    }

  case ELength:
    Map = (ucmat *)(datastruct+dstart);
    return ucmatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2) + 4*sizeof(int);

  case RLength:
    return sizeof(int *);

  case ALength: 
    return sizeof(ucmat);

  case SimpleType: 
    return FALSE;

  case DPrint: 
    Map = (ucmat *)(datastruct+dstart);
    dPrintMAP_FN((FILE *)buffer, Map, byteFormat, bstart,
		 &(Map->el[Map->lb1][Map->lb2]));
    return 1; /* dummy */

  case DFree:
    Map = (ucmat *)(datastruct+dstart);
    FREEMAT(*Map);
    return TRUE;

  default: 
    fprintf(stderr, "UCMAT_Trans: undefined op: %d\n", op); 

  }
  return FALSE; /* should not get here */
}


/******************************************************************************
*
* FUNCTION: int CMAT_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* char *buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int CMAT_Trans(TRANS_OP_TYPE op, DATA_PTR datastruct, int dstart,
	       char *buffer, int bstart)
{ 
  cmat *Map;
  int length, current_byte;
  int lb1, ub1, lb2, ub2, error;
  
  switch(op) {
  case Encode:
    current_byte = bstart;
    Map = (cmat *)(datastruct+dstart);
    length = cmatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2);
    transferMapBounds(Map->lb1, Map->ub1, Map->lb2, Map->ub2, 
		      buffer, &current_byte);
    TO_BUFFER_AND_ADVANCE(&(Map->el[Map->lb1][Map->lb2]), 
			   buffer, current_byte, length);
    return current_byte - bstart;

  case Decode:
    current_byte = bstart;
    retrieveMapBounds(&lb1, &ub1, &lb2, &ub2, buffer, &current_byte);
    Map = (cmat *)(datastruct+dstart);
    *Map = newcmat2(lb1, ub1, lb2, ub2, &error);
    if (error)
      tcaModError("newcmat2 error");
    if (!Map) { 
      tcaModError("CMAT_Trans: Decode. Map cannot be allocated");
    }
    else { 
      FROM_BUFFER_AND_ADVANCE(&(Map->el[lb1][lb2]), buffer, current_byte, 
				    cmatLength(lb1, ub1, lb2, ub2));
      return current_byte - bstart;
    }

  case ELength:
    Map = (cmat *)(datastruct+dstart);
    return cmatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2) + 4*sizeof(int);

  case RLength:
    return sizeof(int *);

  case ALength: 
    return sizeof(cmat);

  case SimpleType: 
    return FALSE;

  case DPrint: 
    Map = (cmat *)(datastruct+dstart);
    dPrintMAP_FN((FILE *)buffer, Map, charFormat, bstart,
		 &(Map->el[Map->lb1][Map->lb2]));
    return 1;

  case DFree:
    Map = (cmat *)(datastruct+dstart);
    FREEMAT(*Map);
    return TRUE;

  default: 
    fprintf(stderr, "CMAT_Trans: undefined op: %d\n", op); 
  }
  return FALSE; /* should not get here */
}


/******************************************************************************
*
* FUNCTION: int SMAT_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* DATA_PTR buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int SMAT_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  smat *Map;
  int length, current_byte;
  int lb1, ub1, lb2, ub2, error;
  
  /* printf("+1+");fflush(stdout);(S.Thrun 93-5-1) */
  switch(op) {
  case Encode:
    /* printf("+2+");fflush(stdout);(S.Thrun 93-5-1) */
    current_byte = bstart;
    Map = (smat *)(datastruct+dstart);
    length = smatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2);
    transferMapBounds(Map->lb1, Map->ub1, Map->lb2, Map->ub2, 
		      buffer, &current_byte);
#if !(defined(i386) || defined(pmax) || defined(VMS))
    TO_BUFFER_AND_ADVANCE(&(Map->el[Map->lb1][Map->lb2]), 
			  buffer, current_byte, length);
#else
    { int i, j;
      short shortVal;
      for (i=Map->lb1; i<=Map->ub1; i++)
	for (j=Map->lb2; j<=Map->ub2; j++) {
	  shortVal = htonShort(Map->el[i][j]);
	  TO_BUFFER_AND_ADVANCE(&shortVal, buffer, current_byte, sizeof(short));
	}
    }
#endif
    /* printf("+3+");fflush(stdout);(S.Thrun 93-5-1) */
    return current_byte - bstart;


  case Decode:
    current_byte = bstart;
    retrieveMapBounds(&lb1, &ub1, &lb2, &ub2, buffer, &current_byte);
    Map = (smat *)(datastruct+dstart);
    *Map = newsmat2(lb1, ub1, lb2, ub2, &error);
    if (error)
      tcaModError("newsmat2 error");
    if (!Map) { 
      tcaModError("SMAT_Trans: Decode. Map cannot be allocated");
    }
    else { 
#if !(defined(i386) || defined(pmax) || defined(VMS))
      FROM_BUFFER_AND_ADVANCE(&(Map->el[lb1][lb2]), buffer, current_byte, 
			      smatLength(lb1, ub1, lb2, ub2));
#else
      int i, j;
      short shortVal;
      for (i=Map->lb1; i<=Map->ub1; i++)
	for (j=Map->lb2; j<=Map->ub2; j++) {
	  FROM_BUFFER_AND_ADVANCE(&shortVal, buffer, current_byte, 
				  sizeof(short));
	  Map->el[i][j] = ntohShort(shortVal);
	}
#endif
      return current_byte - bstart;
    }
#ifdef eruisfhuierlhfluiwehf
  case Decode:
    /* printf("+4+");fflush(stdout);(S.Thrun 93-5-1) */
    current_byte = bstart;
    FROM_BUFFER_AND_ADVANCE(&lb1, buffer, current_byte, sizeof(int));
    FROM_BUFFER_AND_ADVANCE(&ub1, buffer, current_byte, sizeof(int));
    FROM_BUFFER_AND_ADVANCE(&lb2, buffer, current_byte, sizeof(int));
    FROM_BUFFER_AND_ADVANCE(&ub2, buffer, current_byte, sizeof(int));
    Map = (smat *)(datastruct+dstart);
    *Map = newsmat2(lb1, ub1, lb2, ub2, &error);
    /* printf("+5+");fflush(stdout);(S.Thrun 93-5-1) */
    if (error)
      tcaModError("newsmat2 error");
    if (!Map) { 
      tcaModError("SMAT_Trans: Decode. Map cannot be allocated");
    }
    else { 
      FROM_BUFFER_AND_ADVANCE(&(Map->el[lb1][lb2]), buffer, current_byte, 
				    smatLength(lb1, ub1, lb2, ub2));
      return current_byte - bstart;
    }
#endif
  case ELength:
    Map = (smat *)(datastruct+dstart);
    return smatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2) + 4*sizeof(int);

  case RLength:
    return sizeof(int *);

  case ALength: 
    return sizeof(smat);

  case SimpleType: 
    return FALSE;

  case DPrint: 
    Map = (smat *)(datastruct+dstart);
    dPrintMAP_FN((FILE *)buffer, Map, shortFormat, bstart,
	     &(Map->el[Map->lb1][Map->lb2]));
    return 1; /* dummy */

  case DFree:
    Map = (smat *)(datastruct+dstart);
    FREEMAT(*Map);
    return TRUE;

  default: 
    fprintf(stderr, "SMAT_Trans: undefined op: %d\n", op); 
#ifdef LISP
    fflush(stderr);
#endif
    tcaModError(NULL);
    return FALSE;
  }
}


/******************************************************************************
*
* FUNCTION: int IMAT_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* DATA_PTR buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int IMAT_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  imat *Map;
  int length, current_byte;
  int lb1, ub1, lb2, ub2, error;
  
  switch(op) {
  case Encode:
    current_byte = bstart;
    Map = (imat *)(datastruct+dstart);
    length = imatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2);
    transferMapBounds(Map->lb1, Map->ub1, Map->lb2, Map->ub2, 
		      buffer, &current_byte);
#if !(defined(i386) || defined(pmax) || defined(VMS))
    TO_BUFFER_AND_ADVANCE(&(Map->el[Map->lb1][Map->lb2]), 
			   buffer, current_byte, length);
#else
    { int i, j, intVal;
      for (i=Map->lb1; i<=Map->ub1; i++)
	for (j=Map->lb2; j<=Map->ub2; j++) {
	  intVal = htonInt(Map->el[i][j]);
	  TO_BUFFER_AND_ADVANCE(&intVal, buffer, current_byte, sizeof(int));
	}
    }
#endif
    return current_byte - bstart;

  case Decode:
    current_byte = bstart;
    retrieveMapBounds(&lb1, &ub1, &lb2, &ub2, buffer, &current_byte);
    Map = (imat *)(datastruct+dstart);
    *Map = newimat2(lb1, ub1, lb2, ub2, &error);
    if (error)
      tcaModError("newimat2 error");
    if (!Map) { 
      tcaModError("IMAT_Trans: Decode. Map cannot be allocated");
    }
    else { 
#if !(defined(i386) || defined(pmax) || defined(VMS))
      FROM_BUFFER_AND_ADVANCE(&(Map->el[lb1][lb2]), buffer, current_byte, 
			      imatLength(lb1, ub1, lb2, ub2));
#else
      int i, j, intVal;
      for (i=Map->lb1; i<=Map->ub1; i++)
	for (j=Map->lb2; j<=Map->ub2; j++) {
	  FROM_BUFFER_AND_ADVANCE(&intVal, buffer, current_byte, sizeof(int));
	  Map->el[i][j] = ntohInt(intVal);
	}
#endif
      return current_byte - bstart;
    }

  case ELength:
    Map = (imat *)(datastruct+dstart);
    return imatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2) + 4*sizeof(int);

  case RLength:
    return sizeof(int *);

  case ALength: 
    return sizeof(imat);

  case SimpleType: 
    return FALSE;

  case DPrint: 
    Map = (imat *)(datastruct+dstart);
    dPrintMAP_FN((FILE *)buffer, Map, intFormat, bstart,
	     &(Map->el[Map->lb1][Map->lb2]));
    return 1; /* dummy */

  case DFree:
    Map = (imat *)(datastruct+dstart);
    FREEMAT(*Map);
    return TRUE;

  default: 
    fprintf(stderr, "IMAT_Trans: undefined op: %d\n", op); 
#ifdef LISP
    fflush(stderr);
#endif
    tcaModError(NULL);
    return FALSE;

  }
}


/******************************************************************************
*
* FUNCTION: int LMAT_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* DATA_PTR buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int LMAT_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  lmat *Map;
  int length, current_byte;
  int lb1, ub1, lb2, ub2, error;
  
  switch(op) {
  case Encode:
    current_byte = bstart;
    Map = (lmat *)(datastruct+dstart);
    length = lmatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2);
    transferMapBounds(Map->lb1, Map->ub1, Map->lb2, Map->ub2, 
		      buffer, &current_byte);
#if !(defined(i386) || defined(pmax) || defined(VMS))
    TO_BUFFER_AND_ADVANCE(&(Map->el[Map->lb1][Map->lb2]), 
			   buffer, current_byte, length);
#else
    { int i, j;
      long longVal;
      for (i=Map->lb1; i<=Map->ub1; i++)
	for (j=Map->lb2; j<=Map->ub2; j++) {
	  longVal = htonLong(Map->el[i][j]);
	  TO_BUFFER_AND_ADVANCE(&longVal, buffer, current_byte, sizeof(long));
	}
    }
#endif
    return current_byte - bstart;

  case Decode:
    current_byte = bstart;
    retrieveMapBounds(&lb1, &ub1, &lb2, &ub2, buffer, &current_byte);
    Map = (lmat *)(datastruct+dstart);
    *Map = newlmat2(lb1, ub1, lb2, ub2, &error);
    if (error)
      tcaModError("newlmat2 error");
    if (!Map) { 
      tcaModError("LMAT_Trans: Decode. Map cannot be allocated");
    }
    else { 
#if !(defined(i386) || defined(pmax) || defined(VMS))
      FROM_BUFFER_AND_ADVANCE(&(Map->el[lb1][lb2]), buffer, current_byte, 
			      lmatLength(lb1, ub1, lb2, ub2));
#else
      int i, j;
      long longVal;
      for (i=Map->lb1; i<=Map->ub1; i++)
	for (j=Map->lb2; j<=Map->ub2; j++) {
	  FROM_BUFFER_AND_ADVANCE(&longVal, buffer, current_byte, sizeof(long));
	  Map->el[i][j] = ntohLong(longVal);
	}
#endif
      return current_byte - bstart;
    }

  case ELength:
    Map = (lmat *)(datastruct+dstart);
    return lmatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2) + 4*sizeof(int);

  case RLength:
    return sizeof(int *);

  case ALength: 
    return sizeof(lmat);

  case SimpleType: 
    return FALSE;

  case DPrint: 
    Map = (lmat *)(datastruct+dstart);
    dPrintMAP_FN((FILE *)buffer, Map, longFormat, bstart,
	     &(Map->el[Map->lb1][Map->lb2]));
    return 1; /* dummy */

  case DFree:
    Map = (lmat *)(datastruct+dstart);
    FREEMAT(*Map);
    return TRUE;

  default: 
    fprintf(stderr, "LMAT_Trans: undefined op: %d\n", op); 
#ifdef LISP
    fflush(stderr);
#endif
    tcaModError(NULL);
    return FALSE;
  }
}


/******************************************************************************
*
* FUNCTION: int FMAT_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* DATA_PTR buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int FMAT_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  fmat *Map;
  int length, current_byte;
  int lb1, ub1, lb2, ub2, error;
  
  switch(op) {
  case Encode:
    current_byte = bstart;
    Map = (fmat *)(datastruct+dstart);
    length = fmatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2);
    transferMapBounds(Map->lb1, Map->ub1, Map->lb2, Map->ub2, 
		      buffer, &current_byte);
#if !(defined(i386) || defined(pmax) || defined(VMS))
    TO_BUFFER_AND_ADVANCE(&(Map->el[Map->lb1][Map->lb2]), 
			   buffer, current_byte, length);
#else
    { int i, j;
      float floatVal;
      for (i=Map->lb1; i<=Map->ub1; i++)
	for (j=Map->lb2; j<=Map->ub2; j++) {
	  floatVal = htonFloat(Map->el[i][j]);
	  TO_BUFFER_AND_ADVANCE(&floatVal, buffer, current_byte, sizeof(float));
	}
    }
#endif
    return current_byte - bstart;

  case Decode:
    current_byte = bstart;
    retrieveMapBounds(&lb1, &ub1, &lb2, &ub2, buffer, &current_byte);
    Map = (fmat *)(datastruct+dstart);
    *Map = newfmat2(lb1, ub1, lb2, ub2, &error);
    if (error)
      tcaModError("newfmat2 error");
    if (!Map) { 
      tcaModError("FMAT_Trans: Decode. Map cannot be allocated");
    }
    else { 
#if !(defined(i386) || defined(pmax) || defined(VMS))
      FROM_BUFFER_AND_ADVANCE(&(Map->el[lb1][lb2]), buffer, current_byte, 
			      fmatLength(lb1, ub1, lb2, ub2));
#else
      int i, j;
      float floatVal;
      for (i=Map->lb1; i<=Map->ub1; i++)
	for (j=Map->lb2; j<=Map->ub2; j++) {
	  FROM_BUFFER_AND_ADVANCE(&floatVal, buffer, current_byte,
				  sizeof(float));
	  Map->el[i][j] = ntohFloat(floatVal);
	}
#endif
      return current_byte - bstart;
    }

  case ELength:
    Map = (fmat *)(datastruct+dstart);
    return fmatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2) + 4*sizeof(int);

  case RLength:
    return sizeof(int *);

  case ALength: 
    return sizeof(fmat);

  case SimpleType: 
    return FALSE;

  case DPrint: 
    Map = (fmat *)(datastruct+dstart);
    dPrintMAP_FN((FILE *)buffer, Map, floatFormat, bstart,
	     &(Map->el[Map->lb1][Map->lb2]));
    return 1; /* dummy */

  case DFree:
    Map = (fmat *)(datastruct+dstart);
    FREEMAT(*Map);
    return TRUE;

  default: 
    fprintf(stderr, "FMAT_Trans: undefined op: %d\n", op); 
#ifdef LISP
    fflush(stderr);
#endif
    tcaModError(NULL);
    return FALSE;
  }
}


/******************************************************************************
*
* FUNCTION: int DMAT_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* DATA_PTR buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int DMAT_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  dmat *Map;
  int length, current_byte;
  int lb1, ub1, lb2, ub2, error;
  
  switch(op) {
  case Encode:
    current_byte = bstart;
    Map = (dmat *)(datastruct+dstart);
    length = dmatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2);
    transferMapBounds(Map->lb1, Map->ub1, Map->lb2, Map->ub2, 
		      buffer, &current_byte);
#if !(defined(i386) || defined(pmax) || defined(VMS))
    TO_BUFFER_AND_ADVANCE(&(Map->el[Map->lb1][Map->lb2]), 
			   buffer, current_byte, length);
#else
    { int i, j;
      double doubleVal;
      for (i=Map->lb1; i<=Map->ub1; i++)
	for (j=Map->lb2; j<=Map->ub2; j++) {
	  doubleVal = htonDouble(Map->el[i][j]);
	  TO_BUFFER_AND_ADVANCE(&doubleVal, buffer, current_byte, 
				sizeof(double));
	}
    }
#endif
    return current_byte - bstart;

  case Decode:
    current_byte = bstart;
    retrieveMapBounds(&lb1, &ub1, &lb2, &ub2, buffer, &current_byte);
    Map = (dmat *)(datastruct+dstart);
    *Map = newdmat2(lb1, ub1, lb2, ub2, &error);
    if (error)
      tcaModError("newdmat2 error");
    if (!Map) { 
      tcaModError("DMAT_Trans: Decode. Map cannot be allocated");
    }
    else { 
#if !(defined(i386) || defined(pmax) || defined(VMS))
      FROM_BUFFER_AND_ADVANCE(&(Map->el[lb1][lb2]), buffer, current_byte, 
			      dmatLength(lb1, ub1, lb2, ub2));
#else
      int i, j;
      double doubleVal;
      for (i=Map->lb1; i<=Map->ub1; i++)
	for (j=Map->lb2; j<=Map->ub2; j++) {
	  FROM_BUFFER_AND_ADVANCE(&doubleVal, buffer, current_byte,
				  sizeof(double));
	  Map->el[i][j] = ntohDouble(doubleVal);
	}
#endif
      return current_byte - bstart;
    }

  case ELength:
    Map = (dmat *)(datastruct+dstart);
    return dmatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2) + 4*sizeof(int);

  case RLength:
    return sizeof(int *);

  case ALength: 
    return sizeof(dmat);

  case SimpleType: 
    return FALSE;

  case DPrint: 
    Map = (dmat *)(datastruct+dstart);
    dPrintMAP_FN((FILE *)buffer, Map, doubleFormat, bstart,
	     &(Map->el[Map->lb1][Map->lb2]));
    return 1; /* dummy */

  case DFree:
    Map = (dmat *)(datastruct+dstart);
    FREEMAT(*Map);
    return TRUE;

  default: 
    fprintf(stderr, "DMAT_Trans: undefined op: %d\n", op); 
#ifdef LISP
    fflush(stderr);
#endif
    tcaModError(NULL);
    return FALSE;
  }
}


/******************************************************************************
*
* FUNCTION: int SIUCMAT_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* DATA_PTR buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int SIUCMAT_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  ucmat *Map;
  int i, length, current_byte;
  int lb1, ub1, lb2, ub2, error;
  
  switch(op) {
  case Encode:
    current_byte = bstart;
    Map = (ucmat *)(datastruct+dstart);

    TO_BUFFER_AND_ADVANCE(&Map->lb1, buffer, current_byte, sizeof(int));
    TO_BUFFER_AND_ADVANCE(&Map->ub1, buffer, current_byte, sizeof(int));
    TO_BUFFER_AND_ADVANCE(&Map->lb2, buffer, current_byte, sizeof(int));
    TO_BUFFER_AND_ADVANCE(&Map->ub2, buffer, current_byte, sizeof(int));

    length = sizeof(unsigned char)*(Map->ub2 - Map->lb2 + 1);

    for (i=Map->lb1;i <= Map->ub1;i++) {
	TO_BUFFER_AND_ADVANCE(&(Map->el[i][Map->lb2]), 
			      buffer, current_byte, length);
      }

    return current_byte - bstart;

  case Decode:
    current_byte = bstart;
    FROM_BUFFER_AND_ADVANCE(&lb1, buffer, current_byte, sizeof(int));
    FROM_BUFFER_AND_ADVANCE(&ub1, buffer, current_byte, sizeof(int));
    FROM_BUFFER_AND_ADVANCE(&lb2, buffer, current_byte, sizeof(int));
    FROM_BUFFER_AND_ADVANCE(&ub2, buffer, current_byte, sizeof(int));
    Map = (ucmat *)(datastruct+dstart);
    *Map = newucmat2(lb1, ub1, lb2, ub2, &error);
    if (error)
      tcaModError("newucmat2 error");
    if (!Map) { 
      tcaModError("SIUCMAT_Trans: Decode. Map cannot be allocated");
    }
    else { 
      FROM_BUFFER_AND_ADVANCE(&(Map->el[lb1][lb2]), buffer, current_byte, 
				    ucmatLength(lb1, ub1, lb2, ub2));
      return current_byte - bstart;
    }

  case ELength:
    Map = (ucmat *)(datastruct+dstart);
    return ucmatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2) + 4*sizeof(int);

  case RLength:
    return sizeof(int *);

  case ALength: 
    return sizeof(ucmat);

  case SimpleType: 
    return FALSE;

  case DPrint: 
    Map = (ucmat *)(datastruct+dstart);
    dPrintMAP_FN((FILE *)buffer, Map, byteFormat, bstart,
		 &(Map->el[Map->lb1][Map->lb2]));
    return 1; /* dummy */

  case DFree:
    Map = (ucmat *)(datastruct+dstart);
    FREEMAT(*Map);
    return TRUE;

  default: 
    fprintf(stderr, "SIUCMAT_Trans: undefined op: %d\n", op); 
#ifdef LISP
    fflush(stderr);
#endif
    tcaModError(NULL);
    return FALSE;
  }
}


/******************************************************************************
*
* FUNCTION: int SICMAT_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* DATA_PTR buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int SICMAT_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  cmat *Map;
  int i, length, current_byte;
  int lb1, ub1, lb2, ub2, error;

  switch(op) {
  case Encode:
    current_byte = bstart;
    Map = (cmat *)(datastruct+dstart);

    TO_BUFFER_AND_ADVANCE(&Map->lb1, buffer, current_byte, sizeof(int));
    TO_BUFFER_AND_ADVANCE(&Map->ub1, buffer, current_byte, sizeof(int));
    TO_BUFFER_AND_ADVANCE(&Map->lb2, buffer, current_byte, sizeof(int));
    TO_BUFFER_AND_ADVANCE(&Map->ub2, buffer, current_byte, sizeof(int));

    length = sizeof(char)*(Map->ub2 - Map->lb2 + 1);

    for (i=Map->lb1;i <= Map->ub1;i++) {
	TO_BUFFER_AND_ADVANCE(&(Map->el[i][Map->lb2]), 
			      buffer, current_byte, length);
      }

    return current_byte - bstart;

  case Decode:
    current_byte = bstart;
    FROM_BUFFER_AND_ADVANCE(&lb1, buffer, current_byte, sizeof(int));
    FROM_BUFFER_AND_ADVANCE(&ub1, buffer, current_byte, sizeof(int));
    FROM_BUFFER_AND_ADVANCE(&lb2, buffer, current_byte, sizeof(int));
    FROM_BUFFER_AND_ADVANCE(&ub2, buffer, current_byte, sizeof(int));
    Map = (cmat *)(datastruct+dstart);
    *Map = newcmat2(lb1, ub1, lb2, ub2, &error);
    if (error)
      tcaModError("newcmat2 error");
    if (!Map) { 
      tcaModError("SICMAT_Trans: Decode. Map cannot be allocated");
    }
    else { 
      FROM_BUFFER_AND_ADVANCE(&(Map->el[lb1][lb2]), buffer, current_byte, 
				    cmatLength(lb1, ub1, lb2, ub2));
      return current_byte - bstart;
    }

  case ELength:
    Map = (cmat *)(datastruct+dstart);
    return cmatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2) + 4*sizeof(int);

  case RLength:
    return sizeof(int *);

  case ALength: 
    return sizeof(cmat);

  case SimpleType: 
    return FALSE;

  case DPrint: 
    Map = (cmat *)(datastruct+dstart);
    dPrintMAP_FN((FILE *)buffer, Map, charFormat, bstart,
		 &(Map->el[Map->lb1][Map->lb2]));
    return 1;

  case DFree:
    Map = (cmat *)(datastruct+dstart);
    FREEMAT(*Map);
    return TRUE;

  default: 
    fprintf(stderr, "SICMAT_Trans: undefined op: %d\n", op); 
#ifdef LISP
    fflush(stderr);
#endif
    tcaModError(NULL);
    return FALSE;
  }
}


/******************************************************************************
*
* FUNCTION: int SISMAT_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* DATA_PTR buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int SISMAT_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  smat *Map;
  int i, length, current_byte;
  int lb1, ub1, lb2, ub2, error;
  
  switch(op) {
  case Encode:
    current_byte = bstart;
    Map = (smat *)(datastruct+dstart);
    transferMapBounds(Map->lb1, Map->ub1, Map->lb2, Map->ub2, 
		      buffer, &current_byte);

    length = sizeof(short)*(Map->ub2 - Map->lb2 + 1);

    for (i=Map->lb1;i <= Map->ub1;i++) {
#if !(defined(i386) || defined(pmax) || defined(VMS))
      TO_BUFFER_AND_ADVANCE(&(Map->el[i][Map->lb2]), 
			    buffer, current_byte, length);
#else
      int j;
      short shortVal;
      for (j=Map->lb2; j<=Map->ub2; j++) {
	shortVal = htonShort(Map->el[i][j]);
	TO_BUFFER_AND_ADVANCE(&shortVal, buffer, current_byte, sizeof(short));
      }
#endif
    }
    return current_byte - bstart;

  case Decode:
    current_byte = bstart;
    retrieveMapBounds(&lb1, &ub1, &lb2, &ub2, buffer, &current_byte);
    Map = (smat *)(datastruct+dstart);
    *Map = newsmat2(lb1, ub1, lb2, ub2, &error);
    if (error)
      tcaModError("newsmat2 error");
    if (!Map) { 
      tcaModError("SISMAT_Trans: Decode. Map cannot be allocated");
    }
    else { 
#if !(defined(i386) || defined(pmax) || defined(VMS))
      FROM_BUFFER_AND_ADVANCE(&(Map->el[lb1][lb2]), buffer, current_byte, 
			      smatLength(lb1, ub1, lb2, ub2));
#else
      int i, j;
      short shortVal;
      for (i=Map->lb1; i<=Map->ub1; i++)
	for (j=Map->lb2; j<=Map->ub2; j++) {
	  FROM_BUFFER_AND_ADVANCE(&shortVal, buffer, current_byte,
				  sizeof(short));
	  Map->el[i][j] = ntohShort(shortVal);
	}
#endif
      return current_byte - bstart;
    }

  case ELength:
    Map = (smat *)(datastruct+dstart);
    return smatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2) + 4*sizeof(int);

  case RLength:
    return sizeof(int *);

  case ALength: 
    return sizeof(smat);

  case SimpleType: 
    return FALSE;

  case DPrint: 
    Map = (smat *)(datastruct+dstart);
    dPrintMAP_FN((FILE *)buffer, Map, shortFormat, bstart,
	     &(Map->el[Map->lb1][Map->lb2]));
    return 1; /* dummy */

  case DFree:
    Map = (smat *)(datastruct+dstart);
    FREEMAT(*Map);
    return TRUE;

  default: 
    fprintf(stderr, "SISMAT_Trans: undefined op: %d\n", op); 
#ifdef LISP
    fflush(stderr);
#endif
    tcaModError(NULL);
    return FALSE;
  }
}


/******************************************************************************
*
* FUNCTION: int SIIMAT_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* DATA_PTR buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int SIIMAT_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  imat *Map;
  int i, length, current_byte;
  int lb1, ub1, lb2, ub2, error;
  
  switch(op) {
  case Encode:
    current_byte = bstart;
    Map = (imat *)(datastruct+dstart);
    transferMapBounds(Map->lb1, Map->ub1, Map->lb2, Map->ub2, 
		      buffer, &current_byte);

    length = sizeof(int)*(Map->ub2 - Map->lb2 + 1);

    for (i=Map->lb1;i <= Map->ub1;i++) {
#if !(defined(i386) || defined(pmax) || defined(VMS))
      TO_BUFFER_AND_ADVANCE(&(Map->el[i][Map->lb2]), 
			    buffer, current_byte, length);
#else
      int j, intVal;
      for (j=Map->lb2; j<=Map->ub2; j++) {
	intVal = htonInt(Map->el[i][j]);
	TO_BUFFER_AND_ADVANCE(&intVal, buffer, current_byte, sizeof(int));
      }
#endif
    }
    return current_byte - bstart;

  case Decode:
    current_byte = bstart;
    retrieveMapBounds(&lb1, &ub1, &lb2, &ub2, buffer, &current_byte);
    Map = (imat *)(datastruct+dstart);
    *Map = newimat2(lb1, ub1, lb2, ub2, &error);
    if (error)
      tcaModError("newimat2 error");
    if (!Map) { 
      tcaModError("SIIMAT_Trans: Decode. Map cannot be allocated");
    }
    else { 
#if !(defined(i386) || defined(pmax) || defined(VMS))
      FROM_BUFFER_AND_ADVANCE(&(Map->el[lb1][lb2]), buffer, current_byte, 
			      imatLength(lb1, ub1, lb2, ub2));
#else
      int i, j, intVal;
      for (i=Map->lb1; i<=Map->ub1; i++)
	for (j=Map->lb2; j<=Map->ub2; j++) {
	  FROM_BUFFER_AND_ADVANCE(&intVal, buffer, current_byte, sizeof(int));
	  Map->el[i][j] = ntohInt(intVal);
	}
#endif
      return current_byte - bstart;
    }

  case ELength:
    Map = (imat *)(datastruct+dstart);
    return imatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2) + 4*sizeof(int);

  case RLength:
    return sizeof(int *);

  case ALength: 
    return sizeof(imat);

  case SimpleType: 
    return FALSE;

  case DPrint: 
    Map = (imat *)(datastruct+dstart);
    dPrintMAP_FN((FILE *)buffer, Map, intFormat, bstart,
	     &(Map->el[Map->lb1][Map->lb2]));
    return 1; /* dummy */

  case DFree:
    Map = (imat *)(datastruct+dstart);
    FREEMAT(*Map);
    return TRUE;

  default: 
    fprintf(stderr, "SIIMAT_Trans: undefined op: %d\n", op); 
#ifdef LISP
    fflush(stderr);
#endif
    tcaModError(NULL);
    return FALSE;
  }
}


/******************************************************************************
*
* FUNCTION: int SILMAT_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* DATA_PTR buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int SILMAT_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  lmat *Map;
  int i, length, current_byte;
  int lb1, ub1, lb2, ub2, error;
  
  switch(op) {
  case Encode:
    current_byte = bstart;
    Map = (lmat *)(datastruct+dstart);
    transferMapBounds(Map->lb1, Map->ub1, Map->lb2, Map->ub2, 
		      buffer, &current_byte);

    length = sizeof(long)*(Map->ub2 - Map->lb2 + 1);

    for (i=Map->lb1;i <= Map->ub1;i++) {
#if !(defined(i386) || defined(pmax) || defined(VMS))
      TO_BUFFER_AND_ADVANCE(&(Map->el[i][Map->lb2]), 
			    buffer, current_byte, length);
#else
      int j;
      long longVal;
      for (j=Map->lb2; j<=Map->ub2; j++) {
	longVal = htonLong(Map->el[i][j]);
	TO_BUFFER_AND_ADVANCE(&longVal, buffer, current_byte, sizeof(long));
      }
#endif
    }
    return current_byte - bstart;

  case Decode:
    current_byte = bstart;
    retrieveMapBounds(&lb1, &ub1, &lb2, &ub2, buffer, &current_byte);
    Map = (lmat *)(datastruct+dstart);
    *Map = newlmat2(lb1, ub1, lb2, ub2, &error);
    if (error)
      tcaModError("newlmat2 error");
    if (!Map) { 
      tcaModError("SILMAT_Trans: Decode. Map cannot be allocated");
    }
    else { 
#if !(defined(i386) || defined(pmax) || defined(VMS))
      FROM_BUFFER_AND_ADVANCE(&(Map->el[lb1][lb2]), buffer, current_byte, 
			      lmatLength(lb1, ub1, lb2, ub2));
#else
      int i, j;
      long longVal;
      for (i=Map->lb1; i<=Map->ub1; i++)
	for (j=Map->lb2; j<=Map->ub2; j++) {
	  FROM_BUFFER_AND_ADVANCE(&longVal, buffer, current_byte, sizeof(long));
	  Map->el[i][j] = ntohLong(longVal);
	}
#endif
      return current_byte - bstart;
    }

  case ELength:
    Map = (lmat *)(datastruct+dstart);
    return lmatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2) + 4*sizeof(int);

  case RLength:
    return sizeof(int *);

  case ALength: 
    return sizeof(lmat);

  case SimpleType: 
    return FALSE;

  case DPrint: 
    Map = (lmat *)(datastruct+dstart);
    dPrintMAP_FN((FILE *)buffer, Map, longFormat, bstart,
	     &(Map->el[Map->lb1][Map->lb2]));
    return 1; /* dummy */

  case DFree:
    Map = (lmat *)(datastruct+dstart);
    FREEMAT(*Map);
    return TRUE;

  default: 
    fprintf(stderr, "SILMAT_Trans: undefined op: %d\n", op); 
#ifdef LISP
    fflush(stderr);
#endif
    tcaModError(NULL);
    return FALSE;
  }
}


/******************************************************************************
*
* FUNCTION: int SIFMAT_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* DATA_PTR buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int SIFMAT_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  fmat *Map;
  int i, length, current_byte;
  int lb1, ub1, lb2, ub2, error;
  
  switch(op) {
  case Encode:
    current_byte = bstart;
    Map = (fmat *)(datastruct+dstart);
    transferMapBounds(Map->lb1, Map->ub1, Map->lb2, Map->ub2, 
		      buffer, &current_byte);

    length = sizeof(float)*(Map->ub2 - Map->lb2 + 1);

    for (i=Map->lb1;i <= Map->ub1;i++) {
#if !(defined(i386) || defined(pmax) || defined(VMS))
      TO_BUFFER_AND_ADVANCE(&(Map->el[i][Map->lb2]), 
			    buffer, current_byte, length);
#else
      int j;
      float floatVal;
      for (j=Map->lb2; j<=Map->ub2; j++) {
	floatVal = htonFloat(Map->el[i][j]);
	TO_BUFFER_AND_ADVANCE(&floatVal, buffer, current_byte, sizeof(float));
      }
#endif
    }
    return current_byte - bstart;

  case Decode:
    current_byte = bstart;
    retrieveMapBounds(&lb1, &ub1, &lb2, &ub2, buffer, &current_byte);
    Map = (fmat *)(datastruct+dstart);
    *Map = newfmat2(lb1, ub1, lb2, ub2, &error);
    if (error)
      tcaModError("newfmat2 error");
    if (!Map) { 
      tcaModError("SIFMAT_Trans: Decode. Map cannot be allocated");
    }
    else { 
#if !(defined(i386) || defined(pmax) || defined(VMS))
      FROM_BUFFER_AND_ADVANCE(&(Map->el[lb1][lb2]), buffer, current_byte, 
			      fmatLength(lb1, ub1, lb2, ub2));
#else
      int i, j;
      float floatVal;
      for (i=Map->lb1; i<=Map->ub1; i++)
	for (j=Map->lb2; j<=Map->ub2; j++) {
	  FROM_BUFFER_AND_ADVANCE(&floatVal, buffer, current_byte,
				  sizeof(float));
	  Map->el[i][j] = ntohFloat(floatVal);
	}
#endif
      return current_byte - bstart;
    }

  case ELength:
    Map = (fmat *)(datastruct+dstart);
    return fmatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2) + 4*sizeof(int);

  case RLength:
    return sizeof(int *);

  case ALength: 
    return sizeof(fmat);

  case SimpleType: 
    return FALSE;

  case DPrint: 
    Map = (fmat *)(datastruct+dstart);
    dPrintMAP_FN((FILE *)buffer, Map, floatFormat, bstart,
	     &(Map->el[Map->lb1][Map->lb2]));
    return 1; /* dummy */

  case DFree:
    Map = (fmat *)(datastruct+dstart);
    FREEMAT(*Map);
    return TRUE;

  default: 
    fprintf(stderr, "SIFMAT_Trans: undefined op: %d\n", op); 
#ifdef LISP
    fflush(stderr);
#endif
    tcaModError(NULL);
    return FALSE;
  }
}


/******************************************************************************
*
* FUNCTION: int SIDMAT_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* DATA_PTR buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int SIDMAT_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  dmat *Map;
  int i, length, current_byte;
  int lb1, ub1, lb2, ub2, error;
  
  switch(op) {
  case Encode:
    current_byte = bstart;
    Map = (dmat *)(datastruct+dstart);
    transferMapBounds(Map->lb1, Map->ub1, Map->lb2, Map->ub2, 
		      buffer, &current_byte);

    length = sizeof(double)*(Map->ub2 - Map->lb2 + 1);

    for (i=Map->lb1;i <= Map->ub1;i++) {
#if !(defined(i386) || defined(pmax) || defined(VMS))
      TO_BUFFER_AND_ADVANCE(&(Map->el[i][Map->lb2]), 
			    buffer, current_byte, length);
#else
      int j;
      double doubleVal;
      for (j=Map->lb2; j<=Map->ub2; j++) {
	doubleVal = htonDouble(Map->el[i][j]);
	TO_BUFFER_AND_ADVANCE(&doubleVal, buffer, current_byte, sizeof(double));
      }
#endif
    }

    return current_byte - bstart;

  case Decode:
    current_byte = bstart;
    retrieveMapBounds(&lb1, &ub1, &lb2, &ub2, buffer, &current_byte);
    Map = (dmat *)(datastruct+dstart);
    *Map = newdmat2(lb1, ub1, lb2, ub2, &error);
    if (error)
      tcaModError("newdmat2 error");
    if (!Map) { 
      tcaModError("SIDMAT_Trans: Decode. Map cannot be allocated");
    }
    else { 
#if !(defined(i386) || defined(pmax) || defined(VMS))
      FROM_BUFFER_AND_ADVANCE(&(Map->el[lb1][lb2]), buffer, current_byte, 
			      dmatLength(lb1, ub1, lb2, ub2));
#else
      int i, j;
      double doubleVal;
      for (i=Map->lb1; i<=Map->ub1; i++)
	for (j=Map->lb2; j<=Map->ub2; j++) {
	  FROM_BUFFER_AND_ADVANCE(&doubleVal, buffer, current_byte,
				  sizeof(double));
	  Map->el[i][j] = ntohDouble(doubleVal);
	}
#endif
      return current_byte - bstart;
    }

  case ELength:
    Map = (dmat *)(datastruct+dstart);
    return dmatLength(Map->lb1, Map->ub1, Map->lb2, Map->ub2) + 4*sizeof(int);

  case RLength:
    return sizeof(int *);

  case ALength: 
    return sizeof(dmat);

  case SimpleType: 
    return FALSE;

  case DPrint: 
    Map = (dmat *)(datastruct+dstart);
    dPrintMAP_FN((FILE *)buffer, Map, doubleFormat, bstart,
	     &(Map->el[Map->lb1][Map->lb2]));
    return 1; /* dummy */

  case DFree:
    Map = (dmat *)(datastruct+dstart);
    FREEMAT(*Map);
    return TRUE;

  default: 
    fprintf(stderr, "SIDMAT_Trans: undefined op: %d\n", op); 
#ifdef LISP
    fflush(stderr);
#endif
    tcaModError(NULL);
    return FALSE;
  }
}


/******************************************************************************
*
* FUNCTION: int TCA_REF_PTR_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* DATA_PTR buffer; 
* int bstart;
*
* OUTPUTS: int
*
******************************************************************************/

int TCA_REF_PTR_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 

#if 0
  char tmp;
  TCA_REF_PTR *ref2, ref;
  int length, current_byte;
  
  switch(op) {
  case Encode:
    current_byte = bstart;

    ref2 = (TCA_REF_PTR *)(datastruct+dstart);
    ref = *ref2;

    intVal = htonInt(ref->refId);
    TO_BUFFER_AND_ADVANCE(&(ref->refId), buffer, current_byte, sizeof(int));

    length = tcaStrLen(ref->name)*sizeof(char);
    intVal = htonInt(length);
    TO_BUFFER_AND_ADVANCE(&length, buffer, current_byte, sizeof(int));

    if (length) {
      TO_BUFFER_AND_ADVANCE(ref->name, buffer, current_byte, length);
    }
    else {
      TO_BUFFER_AND_ADVANCE("Z", buffer, current_byte, sizeof(char));
    }

    return current_byte - bstart;

  case Decode:
    current_byte = bstart;

    ref = tcaRefCreate(NULL, NULL, 0);

    FROM_BUFFER_AND_ADVANCE(&(ref->refId), buffer, current_byte, sizeof(int));

    FROM_BUFFER_AND_ADVANCE(&length, buffer, current_byte, sizeof(int));
    
    if (length) {
      ref->name = (char *)tcaMalloc(length+1);
      FROM_BUFFER_AND_ADVANCE(ref->name, buffer, current_byte, length);
      ref->name[length/sizeof(char)] = '\0';
    }
    else {
      FROM_BUFFER_AND_ADVANCE(&tmp, buffer, current_byte, 1);
    }

    bcopy(&ref, datastruct+dstart, sizeof(TCA_REF_PTR));

    return current_byte - bstart;

  case ELength:
    ref2 = (TCA_REF_PTR *)(datastruct+dstart);
    ref = *ref2;

    length = tcaStrLen(ref->name)*sizeof(char);

    if (length)
      return(sizeof(int)+length+sizeof(int));
    else
      return(sizeof(int)+sizeof(char)+sizeof(int));

  case ALength: 
  case RLength:
    return sizeof(TCA_REF_PTR);

  case SimpleType: 
    return FALSE;

  case DPrint:
    ref2 = (TCA_REF_PTR *)(datastruct+dstart);
    ref = *ref2;

    dPrintTCA_FN((FILE *)buffer, ref, 0);

    return 1; 

  case DFree:
    ref2 = (TCA_REF_PTR *)(datastruct+dstart);
    ref = *ref2;

    tcaRefFree(ref);
    
    return TRUE;

  default: 
    fprintf(stderr, "TCA_REF_PTR_Trans: undefined op: %d\n", op); 
#ifdef LISP
    fflush(stderr);
#endif
    tcaModError(NULL);
    return FALSE;
  }
#endif
    return 0;
}




/******************************************************************************
*
* FUNCTION: int nonStructuredFormatTrans(op, datastruct, dstart, buffer, 
*                                        bstart, size, printFN)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* char *buffer; 
* int bstart;
* int size;
* VOID_FN printFN;
*
* OUTPUTS: int
*
* NOTES:
*
* 15-Feb-89: reids: Modified to take advantage of similarities in primitive,
* non-structured formatters
*
*  2-Jan-89: reids: Added to facilitate LISP interface.
*
******************************************************************************/

int nonStructuredFormatTrans(op, datastruct, dstart, buffer, bstart, size,
			     printFN)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
int size;
void (*printFN)();
{ 
#if defined(i386) || defined(pmax) || defined(VMS)
  short shortVal;
  long longVal;
  double doubleVal;
#endif

  switch(op) {
  case Encode:
#if !(defined(i386) || defined(pmax) || defined(VMS))
    bcopy(datastruct+dstart, buffer+bstart, size);
#else
    if (size == sizeof(short)) {
      bcopy(datastruct+dstart, (char *)&shortVal, size);
      shortVal = htonShort(shortVal);
      bcopy((char *)&shortVal, buffer+bstart, size);
    } else if (size == sizeof(long)) {
      bcopy(datastruct+dstart, (char *)&longVal, size);
      longVal = htonLong(longVal);
      bcopy((char *)&longVal, buffer+bstart, size);
    } else if (size == sizeof(double)) {
      bcopy(datastruct+dstart, (char *)&doubleVal, size);
      doubleVal = htonDouble(doubleVal);
      bcopy((char *)&doubleVal, buffer+bstart, size);
    } else {
      bcopy(datastruct+dstart, buffer+bstart, size);
    }
#endif
    return size;

  case Decode:
#if !(defined(i386) || defined(pmax) || defined(VMS))
    bcopy(buffer+bstart, datastruct+dstart, size);
#else
    if (size == sizeof(short)) {
      bcopy(buffer+bstart, (char *)&shortVal, size);
      shortVal = ntohShort(shortVal);
      bcopy((char *)&shortVal, datastruct+dstart, size);
    } else if (size == sizeof(long)) {
      bcopy(buffer+bstart, (char *)&longVal, size);
      longVal = ntohLong(longVal);
      bcopy((char *)&longVal, datastruct+dstart, size);
    } else if (size == sizeof(double)) {
      bcopy(buffer+bstart, (char *)&doubleVal, size);
      doubleVal = ntohDouble(doubleVal);
      bcopy((char *)&doubleVal, datastruct+dstart, size);
    } else {
      bcopy(buffer+bstart, datastruct+dstart, size);
    }
#endif
    return size;

  case ELength: 
  case RLength:
  case ALength: 
    return size;

  case SimpleType: 
    return TRUE;

  case DPrint: 
    printFN((FILE *)buffer, (datastruct+dstart), bstart);
#ifdef LISP
    (void)fflush(stdout);
#endif
    return 1;

  case DFree:
    return FALSE; /* nothing to free */

  default: 
    tcaModError(NULL);
  }
  return FALSE; /* should not get here */
}



/******************************************************************************
*
* FUNCTION:
* int CHAR_Trans(op, datastruct, dstart, buffer, bstart)
* int SHORT_Trans(op, datastruct, dstart, buffer, bstart)
* int LONG_Trans(op, datastruct, dstart, buffer, bstart)
* int INT_Trans(op, datastruct, dstart, buffer, bstart)
* int FLOAT_Trans(op, datastruct, dstart, buffer, bstart)
* int DOUBLE_Trans(op, datastruct, dstart, buffer, bstart)
* int BOOLEAN_Trans(op, datastruct, dstart, buffer, bstart)
* int BYTE_Trans(op, datastruct, dstart, buffer, bstart)
* int TWOBYTE_Trans(op, datastruct, dstart, buffer, bstart)
*
* DESCRIPTION:
*
* INPUTS: 
* TRANS_OP_TYPE op; 
* DATA_PTR datastruct; 
* int dstart; 
* DATA_PTR buffer;
* int bstart;
*
* OUTPUTS: int
*
* NOTES:
*
* 15-Feb-89: reids: Added BYTE_Trans and TWOBYTE_Trans
* to round out list of primitive formatters.
*
******************************************************************************/

int CHAR_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer;
int bstart;
{ 
  return nonStructuredFormatTrans(op, datastruct, dstart, buffer, 
				  bstart, sizeof(char), dPrintCHAR_FN);
}

/*************************************/

int SHORT_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer;
int bstart;
{ 
  return nonStructuredFormatTrans(op, datastruct, dstart, buffer, 
				  bstart, sizeof(short), dPrintSHORT_FN);
}

/*************************************/

int LONG_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer;
int bstart;
{ 
  return nonStructuredFormatTrans(op, datastruct, dstart, buffer, 
				  bstart, sizeof(long), dPrintLONG_FN);
}

/*************************************/

int INT_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer;
int bstart;
{ 
  return nonStructuredFormatTrans(op, datastruct, dstart, buffer, 
				  bstart, sizeof(int), dPrintINT_FN);
}

/*************************************/

int FLOAT_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  return nonStructuredFormatTrans(op, datastruct, dstart, buffer, 
				  bstart, sizeof(float), dPrintFLOAT_FN);
}

/*************************************/

int DOUBLE_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  return nonStructuredFormatTrans(op, datastruct, dstart, buffer, 
				  bstart, sizeof(double), dPrintDOUBLE_FN);
}

/*************************************/

int BOOLEAN_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  return nonStructuredFormatTrans(op, datastruct, dstart, buffer,
				  bstart, sizeof(int), dPrintINT_FN);
}

/*************************************/

int BYTE_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  return nonStructuredFormatTrans(op, datastruct, dstart, buffer, 
				  bstart, sizeof(char), dPrintBYTE_FN);
}

/*************************************/

int TWOBYTE_Trans(op, datastruct, dstart, buffer, bstart)
TRANS_OP_TYPE op; 
DATA_PTR datastruct; 
int dstart; 
DATA_PTR buffer; 
int bstart;
{ 
  return nonStructuredFormatTrans(op, datastruct, dstart, buffer, 
				  bstart, 2*sizeof(char), dPrintTWOBYTE_FN);
}


/*  Functions needed by the LISP version */

int DecodeInteger(DataArray, Start)
DATA_PTR DataArray; 
int Start;
{ 
  int Integer;

  DataArray += Start;
  bcopy(DataArray, &Integer, sizeof(int));
  return Integer;
}

void EncodeInteger(Integer, DataArray, Start)
int Integer; 
DATA_PTR DataArray; 
int Start;
{ 
  DataArray += Start;
  bcopy(&Integer, DataArray, sizeof(int));
}

/* LISP floating point numbers are equivalent to C "double" format */
double DecodeFloat(DataArray, Start)
DATA_PTR DataArray; 
int Start;
{ 
  float Float;

  DataArray += Start;
  bcopy(DataArray, &Float, sizeof(float));
  return (double)Float;
}

/* LISP floating point numbers are equivalent to C "double" format */
void EncodeFloat(Float, DataArray, Start)
double Float; 
DATA_PTR DataArray; 
int Start;
{ 
  float RealFloat;

  DataArray += Start;
  RealFloat = (float)Float;
  bcopy(&RealFloat, DataArray, sizeof(float));
}

float DecodeDouble(DataArray, Start)
DATA_PTR DataArray; 
int Start;
{ 
  double Double;

  DataArray += Start;
  bcopy(DataArray, &Double, sizeof(double));
  return Double;
}

void EncodeDouble(Double, DataArray, Start)
double Double; 
DATA_PTR DataArray; 
int Start;
{ 
  DataArray += Start;
  bcopy(&Double, DataArray, sizeof(double));
}

void Bcopy_Bytes(Array1, Start1, Array2, Start2, Num_Bytes)
char Array1[], Array2[]; 
int Start1, Start2, Num_Bytes;
{ 
  bcopy(Array1+Start1, Array2+Start2, Num_Bytes);
}


#if defined(i386) || defined(pmax) || defined(VMS)
float htonFloat (float f)
{
  u_long l;

  bcopy((char *)&f, (char *)&l, sizeof(float));
  l = htonl(l);
  bcopy((char *)&l, (char *)&f, sizeof(float));
  return f;
}

double htonDouble (double d)
{
  u_long l1, l2;

  bcopy((char *)(u_long *)(&d), (char *)&l1, sizeof(u_long));
  l1 = htonl(l1);
  bcopy((char *)(((u_long *)&d)+1), (char *)&l2, sizeof(u_long));
  l2 = htonl(l2);
  bcopy((char *)&l2, (char *)&d, sizeof(u_long));
  bcopy((char *)&l1, (char *)(((u_long *)&d)+1), sizeof(u_long));

  return d;
}

float ntohFloat (float f)
{
  u_long l;

  bcopy((char *)&f, (char *)&l, sizeof(float));
  l = ntohl(l);
  bcopy((char *)&l, (char *)&f, sizeof(float));
  return f;
}

double ntohDouble (double d)
{
  u_long l1, l2;

  bcopy((char *)&d, (char *)&l1, sizeof(u_long));
  l1 = ntohl(l1);
  bcopy((char *)(((u_long *)&d)+1), (char *)&l2, sizeof(u_long));
  l2 = ntohl(l2);
  bcopy((char *)&l2, (char *)&d, sizeof(u_long));
  bcopy((char *)&l1, (char *)(((u_long *)&d)+1), sizeof(u_long));

  return d;
}

#endif
