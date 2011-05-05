
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
* FILE: formatters.c
*
* ABSTRACT:
*
* Data Format Routines.
*
* REVISION HISTORY:
*
* $Log: formatters.c,v $
* Revision 1.2  1997/03/11 17:16:47  tyson
* added IR simulation and other work
*
* Revision 1.1.1.1  1996/09/22 16:46:01  rhino
* General reorganization of the directories/repository, fusion with the
* RWI software.
*
* Revision 1.4  1994/10/23 08:51:10  tyson
* fixed (I hope) i386 structure indexing.
*
 * Revision 1.3  1994/10/22  18:47:28  tyson
 * VMS version. Fixed structure indexing (computation of the offsets
 * in a struct). Added signal handlers to a1, b1 tcxServer.
 *
 * Revision 1.1.1.1  1994/03/31  08:35:04  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:29  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:30  rhino
 * test
 *
 * Revision 1.8  1993/03/12  20:58:24  fedor
 * Updated test cases and hid global variables for vxworks
 *
*
*  9-Jul-91 Christopher Fedor, School of Computer Science, CMU
* Removed isIntFormat test routine because we now have string formats to test
* and this allows testing of formats without first parsing it.
*
*  2-Jul-91 Reid Simmons at School of Computer Science, CMU
* Added code for freeing class data.
*
* 30-Jan-91 Christopher Fedor, School of Computer Science, CMU
* Added fflush(stdout) to printf for module code calls from lisp
*
* 23-Oct-90 Christopher Fedor at School of Computer Science, CMU
* Moved isIntFormat from mon.c to here - it logically belongs here 
* even though it increases module code a little.
* format needs a complete rewrite! flush SIZES_TYPE, flush the union
* - saves little space and makes debugging very difficult ... additional 
* problems.
*
* 15-Oct-90 Christopher Fedor at School of Computer Science, CMU
* Moved tcaRegisterLengthFormatter and tcaRegisterNamedFormatter
* to module routines.
*
* 18-Jul-90 Reid Simmons at School of Computer Science, CMU
* Added code for freeing data structures.
*
* 10-Apr-90 Christopher Fedor at School of Computer Science, CMU
* Changed ifdef SUN3 to ifndef SUN4.
*
*  9-Apr-90 Reid Simmons at School of Computer Science, CMU
* Modified "alignField" to work for both Sun3 and Sun4 (Sparc) versions.
*
*  2-Apr-90 Christopher Fedor at School of Computer Science, CMU
* Revised to software standards.
*
* 11-Mar-89 Christopher Fedor at School of Computer Science, CMU
* Added tcaRegisterLengthFormatter
*
*  9-Feb-89 Reid Simmons at School of Computer Science, CMU
* Split into 2 files -- "PrimFmttrs" and "Formatters"; changed how 
* structured formatters are represented.  Added parser to take
* formatter string and create TC_FORMAT_PTR structures.
* INCOMPATIBLE CHANGE WITH MAJOR VERSION 1.x
*
*  6-Feb-89 Reid Simmons at School of Computer Science, CMU
* Added structured formatters.
*
*  3-Feb-89 Reid Simmons at School of Computer Science, CMU
* Standardized C and LISP interface to formatter functions.
*
*  2-Jan-89 Reid Simmons at School of Computer Science, CMU
* (1.1) Added INT and FLOAT formats for LISP interface.
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

#include "hash.h"
#include "formatters.h"

#include "global.h"

#ifdef LISP
#include "lisp.h"
#endif

typedef struct{
  int buffer, data;
} SIZES_TYPE;

extern int STR_Trans();
extern int FORMAT_Trans();

extern int UCMAT_Trans();
extern int CMAT_Trans();
extern int SMAT_Trans();
extern int IMAT_Trans();
extern int LMAT_Trans();
extern int FMAT_Trans();
extern int DMAT_Trans();

extern int SIUCMAT_Trans();
extern int SICMAT_Trans();
extern int SISMAT_Trans();
extern int SIIMAT_Trans();
extern int SILMAT_Trans();
extern int SIFMAT_Trans();
extern int SIDMAT_Trans();

extern int INT_Trans();
extern int CHAR_Trans();
extern int SHORT_Trans();
extern int LONG_Trans();

extern int FLOAT_Trans();
extern int BOOLEAN_Trans();
extern int DOUBLE_Trans();
extern int BYTE_Trans();
extern int TWOBYTE_Trans();

extern int TCA_REF_PTR_Trans();

#if 0
extern MSG_PTR msgFind();
#endif

int (*TransTable[MAXFORMATTERS+1])();

/* If there is no formatter, it means that the ptr is recursive (self-ptr) */
#define CHOOSE_PTR_FORMAT(format, parentFormat) \
  (((format)->formatter.f) ? ((format)->formatter.f) : parentFormat)

#define ODDP(x) ((x) & 1)

#ifdef LISP

void blockCopyToArray(buffer, array, amount)
BUFFER_PTR buffer;
char *array;
int amount;
{
  bcopy(buffer->buffer+buffer->bstart, array, amount);
  buffer->bstart += amount;
}

void blockCopyFromArray(buffer, array, amount)
BUFFER_PTR buffer;
char *array;
int amount;
{
  bcopy(array, buffer->buffer+buffer->bstart, amount);
  buffer->bstart += amount;
}

int tcaRefNameLength(ref)
TCA_REF_PTR ref;
{
  if (ref && ref->name)
    return strlen(ref->name);
  else
    return 0;
}

char *tcaRefNameLisp(ref, length)
TCA_REF_PTR ref;
int length;
{
  bcopy(ref->name, (char *)Vecdata(SymbolValue(lisp_value(0))), length);

  return (char *)SymbolValue(lisp_value(0));
}

char *returnLispFlagGlobal()
{
  return &LispFlagGlobal;
}

void setIntValue(item, value)
int *item, value;
{
  *item = value;
}

int formatPrimitiveProc(format)
FORMAT_PTR format;
{
  return(format->formatter.i);
}

int formatType(format)
FORMAT_PTR format;
{
  return(format->type);
}

FORMAT_PTR formatChoosePtrFormat(format, parentFormat)
FORMAT_PTR format, parentFormat;
{
  return(CHOOSE_PTR_FORMAT(format, parentFormat));
}

/* 14-Feb-91: fedor: format array sillyness from structFMT loops */

FORMAT_ARRAY_PTR formatFormatArray(format)
FORMAT_PTR format;
{
  return(format->formatter.a);
}

int formatFormatArrayMax(formatArray)
FORMAT_ARRAY_PTR formatArray;
{
  return(formatArray[0].i);
}

FORMAT_PTR formatFormatArrayItem(formatArray, i)
FORMAT_ARRAY_PTR formatArray;
int i;
{
  return(formatArray[i].f);
}

int formatGetInt(buffer)
BUFFER_PTR buffer;
{
  int i;

  bcopy(buffer->buffer+buffer->bstart, &i, sizeof(int));
  buffer->bstart += sizeof(int);

  return i;
}

char formatGetChar(buffer)
BUFFER_PTR buffer;
{
  char c;

  bcopy(buffer->buffer+buffer->bstart, &c, sizeof(char));
  buffer->bstart += sizeof(char);

  return c;
}

double formatGetDouble(buffer)
BUFFER_PTR buffer;
{
  double d;

  bcopy(buffer->buffer+buffer->bstart, &d, sizeof(double));
  buffer->bstart += sizeof(double);

  return d;
}

void formatPutInt(i, buffer)
int i;
BUFFER_PTR buffer;
{
  bcopy(&i, buffer->buffer+buffer->bstart, sizeof(int));
  buffer->bstart += sizeof(int);
}

void formatPutChar(c, buffer)
char c;
BUFFER_PTR buffer;
{
  bcopy(&c, buffer->buffer+buffer->bstart, sizeof(char));
  buffer->bstart += sizeof(char);
}

float formatGetFloat(buffer)
BUFFER_PTR buffer;
{
  float f;

  buffer->bstart += 
    FLOAT_Trans(Decode, &f, 0, buffer->buffer, buffer->bstart);

  return f;
}

void formatPutFloat(f, buffer)
float f;
BUFFER_PTR buffer;
{
  /* 14-Mar-91: fedor: this sillyness forces a pointer to a float
     without changing that float into a double. This is needed
     because FLOAT_Trans will transfer only 4 bytes but the conversion
     to a double will only transfer the first 4 bytes of the correct value. */

  float f2;

  f2 = f;
  buffer->bstart += 
    FLOAT_Trans(Encode, &f2, 0, buffer->buffer, buffer->bstart);
}


void formatPutDouble(d, buffer)
double d;
BUFFER_PTR buffer;
{
  bcopy(&d, buffer->buffer+buffer->bstart, sizeof(double));
  buffer->bstart += sizeof(double);
}

#endif

FORMAT_PTR createIntegerFormat(type, integer)
FORMAT_CLASS_TYPE type; 
int integer;
{ 
  FORMAT_PTR format;

  format = NEW_FORMATTER();
  format->type = type;
  format->formatter.i = integer;
  return format;
}

FORMAT_PTR createPrimitiveFormat(format_number) 
int format_number;
{ 
  return createIntegerFormat(primitiveFMT, format_number);
}

int formatterHashFN(key)
char *key;
{ 
  int i, sum;

  for(i=0, sum=0; key[i] != '\0'; i++) 
    sum += key[i];
  return sum;
}

int formatterEqFN(key1, key2) 
char *key1, *key2;
{ 
  return(!strcmp(key1, key2));
}

/* KeyLength is calculated assuming "name" is a null-terminated string */
void addFormatToTable(name, format)
char *name; 
FORMAT_PTR format;
{ 
  hashTableInsert(name, 1+strlen(name), format, Global->formatNamesTable);
}


/*****************************************************************************
*
* FUNCTION: void registerPrimitiveFormat(formatter, formatNumber, 
*                                           translationFunction)
*
* DESCRIPTION:
*
* INPUTS:
* char *formatter;
* int formatNumber, (*translationFunction)();
*
* OUTPUTS: void.
*
*****************************************************************************/

void registerPrimitiveFormat(formatter, formatNumber, translationFunction)
char *formatter;
int formatNumber, (*translationFunction)();
{ 
  if ((formatNumber < 1) || (formatNumber > MAXFORMATTERS)) {
    fprintf(stderr, 
	    "RegisterPrimitiveFormatter: 0 < formatter number: %d < %d\n",
	    formatNumber, MAXFORMATTERS+1);
#ifdef LISP
    fflush(stderr);
#endif
    tcaModError(NULL);
  }
  else if (TransTable[formatNumber]) {
    fprintf(stderr, 
	    "RegisterFormatter: Formatter number %d already in use", 
	    formatNumber);
#ifdef LISP
    fflush(stderr);
#endif
    tcaModError(NULL);
  }
  else {
    TransTable[formatNumber] = translationFunction;
    addFormatToTable(formatter, createPrimitiveFormat(formatNumber));
  }
}


/*****************************************************************************
*
* FUNCTION: void formatInitialize()
*
* DESCRIPTION:
*
* INPUTS: none.
*
* OUTPUTS: void.
*
*****************************************************************************/

void formatInitialize()
{ 
  int i;
  
  Global->formatNamesTable = hashTableCreate(50, formatterHashFN, 
					    formatterEqFN);
  
  for(i=0; i< MAXFORMATTERS; i++) 
    TransTable[i] = NULL;
    
  registerPrimitiveFormat("string", STR_FMT, STR_Trans);
  registerPrimitiveFormat("char", CHAR_FMT, CHAR_Trans);
  registerPrimitiveFormat("short", SHORT_FMT, SHORT_Trans);
  registerPrimitiveFormat("long", LONG_FMT, LONG_Trans);
  registerPrimitiveFormat("int", INT_FMT, INT_Trans);
  registerPrimitiveFormat("float", FLOAT_FMT, FLOAT_Trans);
  registerPrimitiveFormat("boolean", BOOLEAN_FMT, BOOLEAN_Trans);
  registerPrimitiveFormat("double", DOUBLE_FMT, DOUBLE_Trans);
  registerPrimitiveFormat("byte", BYTE_FMT, BYTE_Trans);
  registerPrimitiveFormat("twobyte", TWOBYTE_FMT, TWOBYTE_Trans);

  registerPrimitiveFormat("format", FORMAT_FMT, FORMAT_Trans);

  registerPrimitiveFormat("ucmat", UCMAT_FMT, UCMAT_Trans);
  registerPrimitiveFormat("cmat", CMAT_FMT, CMAT_Trans);
  registerPrimitiveFormat("smat", SMAT_FMT, SMAT_Trans);
  registerPrimitiveFormat("imat", IMAT_FMT, IMAT_Trans);
  registerPrimitiveFormat("lmat", LMAT_FMT, LMAT_Trans);
  registerPrimitiveFormat("fmat", FMAT_FMT, FMAT_Trans);
  registerPrimitiveFormat("dmat", DMAT_FMT, DMAT_Trans);

  registerPrimitiveFormat("siucmat", SIUCMAT_FMT, SIUCMAT_Trans);
  registerPrimitiveFormat("sicmat", SICMAT_FMT, SICMAT_Trans);
  registerPrimitiveFormat("sismat", SISMAT_FMT, SISMAT_Trans);
  registerPrimitiveFormat("siimat", SIIMAT_FMT, SIIMAT_Trans);
  registerPrimitiveFormat("silmat", SILMAT_FMT, SILMAT_Trans);
  registerPrimitiveFormat("sifmat", SIFMAT_FMT, SIFMAT_Trans);
  registerPrimitiveFormat("sidmat", SIDMAT_FMT, SIDMAT_Trans);

  registerPrimitiveFormat("TCA_REF_PTR", TCA_REF_PTR_FMT, TCA_REF_PTR_Trans);

  /* 
    Added 18-Aug-89, used in HERO LISP system. "ubyte" (unsigned byte) is
    treated the same as a regular C byte.
    */
  registerPrimitiveFormat("ubyte", UBYTE_FMT, BYTE_Trans);
}


/*****************************************************************************
*
* FUNCTION: int fixedArraySize(formatArray) 
*
* DESCRIPTION:
* First element in a FORMAT_ARRAY_PTR is the size of the array.
* For a fixedArrayFMT (and varArrayFMT) the second element is the 
* formatter, the rest of the elements are the dimensions.
*
* INPUTS: FORMAT_ARRAY_PTR formatArray;
*
* OUTPUTS: int
*
*****************************************************************************/

int fixedArraySize(formatArray) 
FORMAT_ARRAY_PTR formatArray;
{
  int i, arraySize;

  for(i=2, arraySize=1; i<formatArray[0].i; i++)
    arraySize *= formatArray[i].i;
  return arraySize;
}


/*****************************************************************************
*
* FUNCTION: int varArraySize(formatArray, parentFormat, dataStruct, dStart)
*
* DESCRIPTION:
*
* INPUTS:
* FORMAT_ARRAY_PTR formatArray; 
* FORMAT_PTR parentFormat;
* DATA_PTR dataStruct; 
* int dStart;
*
* OUTPUTS: int
*
*****************************************************************************/

int varArraySize(formatArray, parentFormat, dataStruct, dStart)
FORMAT_ARRAY_PTR formatArray; 
FORMAT_PTR parentFormat;
DATA_PTR dataStruct; 
int dStart;
{ 
  int i, j, arraySize, size;
  FORMAT_ARRAY_PTR parentStructArray;
  int currentPlace, sizePlace, foundPlace, offset;

  parentStructArray = parentFormat->formatter.a;
  foundPlace = 0;
  for(currentPlace=1;!foundPlace;currentPlace++)
    foundPlace = ((parentStructArray[currentPlace].f->type == varArrayFMT) &&
		  (parentStructArray[currentPlace].f->formatter.a
		   == formatArray));
  currentPlace--;
  
  for(i=2, arraySize=1; i < formatArray[0].i; i++) {
    sizePlace = formatArray[i].i;
    offset = 0;
    if (currentPlace < sizePlace)
      for (j=currentPlace; j < sizePlace; j++)
	offset += dataStructureSize(parentStructArray[j].f);
    else 
      for (j=sizePlace; j < currentPlace; j++)
	offset -= dataStructureSize(parentStructArray[j].f);
    bcopy(dataStruct+dStart+offset, &size, sizeof(int));
    arraySize *= size;
  }

  return arraySize;
}


/*****************************************************************************
*
* FUNCTION: int elementSize(format) 
*
* DESCRIPTION:
* Returns the size (ALength) of the format's element.
* If the format is a structured type, returns 0 unless all the elements
* have the same length.
*
* INPUTS: FORMAT_PTR format;
*
* OUTPUTS: int
*
*****************************************************************************/

int elementSize(format)
FORMAT_PTR format;
{
  int firstSize, i;

  switch (format->type) {
  case lengthFMT: 
    return format->formatter.i;
  case primitiveFMT:
    return (TransTable[format->formatter.i])(ALength, NULL, 0, NULL, 0);
  case pointerFMT:
  case varArrayFMT: 
    return sizeof(DATA_PTR);
  case fixedArrayFMT: 
    return elementSize(format->formatter.a[1].f);
  case structFMT:
    firstSize = elementSize(format->formatter.a[1].f);
    if (firstSize != 0) {
      for (i=2; i<format->formatter.a[0].i; i++) {
	if (firstSize != elementSize(format->formatter.a[i].f))
	  return 0;
      }
    }
    return firstSize;
  default:
    fprintf(stderr, "ERROR: elementSize: UNKNOWN CASE\n");
    return 0;
  }
}


/*****************************************************************************
*
* FUNCTION: int fixedLengthFormat(format) 
*
* DESCRIPTION:
* Returns TRUE (1) if the Format contains no pointers.
* (i.e., it is composed only of simple types).
*
* INPUTS: FORMAT_PTR format;
*
* OUTPUTS: int
*
*****************************************************************************/

int fixedLengthFormat(format) 
FORMAT_PTR format;
{ 
  int i;
  int subelementSize = 0;

  switch(format->type) {
  case lengthFMT: 
    return TRUE;
  case primitiveFMT:
    return (TransTable[format->formatter.i])(SimpleType, NULL, 0, NULL, 0);
  case pointerFMT:
  case varArrayFMT: 
    return FALSE;
  case fixedArrayFMT: 
    return fixedLengthFormat(format->formatter.a[1].f);
  case structFMT:
    /* For compatibility between Sun3's and Sun4's, in addition to all
     *  being simple types, the subelements of a structure must be the
     *  same size in order for the structure to be considered "fixed length" 
     */
    /* return(elementSize(format->formatter.a[1].f));*/
    for (i=1; i<format->formatter.a[0].i; i++) {
      if (!fixedLengthFormat(format->formatter.a[i].f)) {
	return FALSE;
      }
      else if (i==1) { /* First element */
	subelementSize = elementSize(format->formatter.a[1].f);
	if (subelementSize == 0) return FALSE;
      }
      else if (subelementSize != elementSize(format->formatter.a[i].f))
	return FALSE;
    }
    return TRUE;
  default:
    fprintf(stderr, "ERROR: fixedLengthFormat: UNKNOWN CASE\n");
    return FALSE;
  }
}


/*****************************************************************************
*
* FUNCTION: int mostRestrictiveElement(format) 
*
* DESCRIPTION:
* Returns the longest element (ALength) of the format.
*
* INPUTS: FORMAT_PTR format;
*
* OUTPUTS: int
*
*****************************************************************************/

#if (defined(sun) && !defined(SUN3)) || defined(SUN4) || defined(pmax) \
    || defined(VMS) || defined (i386)
int mostRestrictiveElement(format)
FORMAT_PTR format;      
{
  int maxSize, nextSize, i;

  switch (format->type) {
  case lengthFMT: return format->formatter.i;

  case primitiveFMT: 
    return (TransTable[format->formatter.i])(RLength, NULL, 0, NULL, 0);

  case pointerFMT:
  case varArrayFMT: 
    return sizeof(DATA_PTR);

  case fixedArrayFMT: 
    return mostRestrictiveElement(format->formatter.a[1].f);

  case structFMT:
    maxSize = mostRestrictiveElement(format->formatter.a[1].f);
    for (i=2; i<format->formatter.a[0].i; i++) {
      nextSize = mostRestrictiveElement(format->formatter.a[i].f);
      if (nextSize > maxSize) maxSize = nextSize;
    }
    return maxSize;
  }
  fprintf(stderr, "%s:%6d:%s() - ***** unhandled case! [%d] ***\n",
	  __FILE__, __LINE__, __FUNCTION__, format->type);
  exit(0);
}
#endif


/*****************************************************************************
*
* FUNCTION: int alignField(format, currentField, currentDataSize)
*
* DESCRIPTION:
* Returns either "currentDataSize" or 1 + "currentDataSize", to reflect
* how C would align fields in a structure.
*
* Sun3 version:
* This function works on the (empirical) model that the C compiler used on the
* Sun3s aligns fields in structures on word boundaries (unless the next 
* field in the structure is an odd number of bytes).
*
* Sun4 version:
* Fields must be aligned on the "appropriate" boundaries - e.g., ints on 4 byte
* boundaries, doubles on 8 byte boundaries.  Structures are padded at the end 
* to align with the most restrictive (longest) field in the structure.
*
* i386 version:
* It was observed that the i386 aligns as follows:
*      chars on byte boundries
*      shorts on word (2 byte) boundries
*      ints, floats, doubles on lword (4 byte) boundries
*           ... and lord only knows why it is so difficult to index
*           into a structure.
*
*  This would fit the SUN4 model if we were to add the restriction
*  that we use appropriateSize=min(4, mostRestrictiveElement());
*  so, I will give that a try.
*                                                     - Tyson Sawyer
*
* INPUTS:
* FORMAT_PTR format; 
* int currentField, currentDataSize;
*
* OUTPUTS: 
*
*****************************************************************************/

int alignField(format, currentField, currentDataSize)
FORMAT_PTR format; 
int currentField, currentDataSize;
{ 
#if (defined(SUN4) || defined(sun) || defined(VMS) || defined(i386))

  int nextField, appropriateSize, rem;
  FORMAT_ARRAY_PTR formatArray;   

  formatArray = format->formatter.a;
  nextField = 1+currentField;
  if (nextField == formatArray[0].i) {
    /* end of structure; pad to appropriate boundary of longest subelement */
    appropriateSize = mostRestrictiveElement(format);
  }
  else {
    /* on Sparc, element must start on boundary compatible with size of largest
       element within the sub-structure */
    appropriateSize = mostRestrictiveElement(formatArray[nextField].f);
  }
  /* Round up to the next even multiple of "appropriateSize" */
#if defined(i386)
  if (appropriateSize>4) appropriateSize=4;
#endif

  rem = currentDataSize % appropriateSize;
  if (rem != 0)    
    currentDataSize += appropriateSize - rem;

  return currentDataSize;

#else

  int nextField;
  FORMAT_PTR nextFormat;
  FORMAT_ARRAY_PTR formatArray; 
                  
  if (!ODDP(currentDataSize)) 
    return currentDataSize;
  else {
    formatArray = format->formatter.a;
    nextField = 1 + currentField;
    if (nextField == formatArray[0].i)
      /* end of structure */
      return 1+currentDataSize;
    else {
      nextFormat = formatArray[nextField].f;
      if (((nextFormat->type == lengthFMT) && 
	   ODDP(nextFormat->formatter.i)) ||
	  ((nextFormat->type == primitiveFMT) &&
   ODDP((TransTable[nextFormat->formatter.i])(ALength, NULL, 0, NULL, 0))))
	return currentDataSize;
      else return 1+currentDataSize;
    }
  }

#endif
}


/*****************************************************************************
*
* FUNCTION: FORMAT_PTR optimizeFormatter(format) 
*
* DESCRIPTION:
* If the formatter (or any sub-formatter of it) is a fixed length format, 
* replace the definition with the appropriate length format.
* For example, "{{int, float}, string}" gets optimized to "{4, 4, string}"
* and "{int, float, [int:3]}" gets optimized to "20".
*
* Returns the new formatter, or the original one if no change occurs.
*
* INPUTS: FORMAT_PTR format;
*
* OUTPUTS: FORMAT_PTR
*
*****************************************************************************/

FORMAT_PTR optimizeFormatter(format) 
FORMAT_PTR format;
{ 
  int i;
  FORMAT_ARRAY_PTR formatArray;

  if (fixedLengthFormat(format))
    return createIntegerFormat(lengthFMT, dataStructureSize(format));
  else {
    switch(format->type) {
    case structFMT: 
      formatArray = format->formatter.a;
      for(i=1;i < formatArray[0].i;i++) 
	formatArray[i].f = optimizeFormatter(formatArray[i].f);
      break;
    case pointerFMT:
      if (format->formatter.f)
	format->formatter.f = optimizeFormatter(format->formatter.f);
      break;
    case varArrayFMT:
    case fixedArrayFMT: 
      format->formatter.a[1].f = optimizeFormatter(format->formatter.a[1].f);

    default:
      fprintf(stderr, "%s:%6d:%s() - ***** unhandled case! [%d] ***\n",
	      __FILE__, __LINE__, __FUNCTION__, format->type);
      exit(0);
      
    }
    return format;
    
  }
}


/*****************************************************************************
*
* FUNCTION: SIZES_TYPE bufferSize1(format, dataStruct, dStart, parentFormat)
*
* DESCRIPTION:
* "ParentFormat" is needed by SelfPtr ("*!") and VarArray ("<..>") formatters.
*  Both these formatters can only be embedded in a Struct format ("{...}".
*
* INPUTS:
* FORMAT_PTR format, parentFormat;
* DATA_PTR dataStruct; 
* int dStart; 
*
* OUTPUTS: SIZES_TYPE
*
*****************************************************************************/

SIZES_TYPE bufferSize1(format, dataStruct, dStart, parentFormat)
FORMAT_PTR format, parentFormat;
DATA_PTR dataStruct; 
int dStart; 
{ 
  SIZES_TYPE sizes;
  int (*formatProc)();
  DATA_PTR structPtr;
  FORMAT_PTR nextFormat;
  FORMAT_ARRAY_PTR formatArray;
  int i, bufferSize, currentData, arraySize, structStart, elements;

  bufferSize = 0;
  currentData = dStart;

  switch (format->type) {
  case lengthFMT:
    bufferSize += format->formatter.i;
    currentData += format->formatter.i;
    break;
  case primitiveFMT: 
    formatProc = TransTable[format->formatter.i];
    bufferSize += formatProc(ELength, dataStruct, currentData, NULL, 0);
    currentData += formatProc(ALength, NULL, 0, NULL, 0);
    break;
  case pointerFMT:
    structPtr = REF(DATA_PTR, dataStruct, currentData);
    bufferSize += sizeof(char);
    if (structPtr) { 
      nextFormat = CHOOSE_PTR_FORMAT(format, parentFormat);
      sizes = bufferSize1(nextFormat, structPtr, 0, NULL);
      bufferSize += sizes.buffer;
    }
    currentData += sizeof(DATA_PTR);
    break;
  case structFMT:
    formatArray = format->formatter.a;
    for(i=1;i < formatArray[0].i;i++) {
      sizes = bufferSize1(formatArray[i].f, dataStruct, currentData, format);
      bufferSize += sizes.buffer;
      currentData = alignField(format, i, currentData+sizes.data);
    }
    break;
  case fixedArrayFMT:
    formatArray = format->formatter.a;
    arraySize = fixedArraySize(formatArray);
    nextFormat = formatArray[1].f;
    if (fixedLengthFormat(nextFormat)) {
      elements = arraySize * dataStructureSize(nextFormat);
      bufferSize += elements;
      currentData += elements;
    }
    else {
      for(i=0;i < arraySize;i++) {
	sizes = bufferSize1(nextFormat, dataStruct, currentData, NULL);
	bufferSize += sizes.buffer;
	currentData += sizes.data;
      }
    }
    break;
  case varArrayFMT:
    formatArray = format->formatter.a;
    nextFormat = formatArray[1].f;
    arraySize = varArraySize(formatArray, parentFormat, 
				dataStruct, currentData);
    bufferSize += sizeof(int); /* for the size of the array */
    if (fixedLengthFormat(nextFormat))
      bufferSize += arraySize * dataStructureSize(nextFormat);
    else {
      structPtr = REF(DATA_PTR, dataStruct, currentData);
      structStart = 0;
      for(i=0;i < arraySize;i++) {
	sizes = bufferSize1(nextFormat, structPtr, structStart, NULL);
	bufferSize += sizes.buffer;
	structStart += sizes.data;
      }
    }
    currentData += sizeof(int); /* skip over the pointer to the array */
  }

  sizes.buffer = bufferSize;
  sizes.data = currentData - dStart;
  return sizes;
}


/*****************************************************************************
*
* FUNCTION: SIZES_TYPE transferToBuffer(format, dataStruct, dStart, buffer, 
*                                      bStart, parentFormat)
*
* DESCRIPTION:
*
* INPUTS:
* FORMAT_PTR format, parentFormat;
* DATA_PTR dataStruct; 
* int dStart, bStart; 
* DATA_PTR buffer;
*
* OUTPUTS: SIZES_TYPE
*
*****************************************************************************/

SIZES_TYPE transferToBuffer(format, dataStruct, dStart, buffer, 
			   bStart, parentFormat)
FORMAT_PTR format, parentFormat;
DATA_PTR dataStruct; 
int dStart, bStart; 
DATA_PTR buffer;
{ 
  SIZES_TYPE sizes;
  int (*formatProc)();
  DATA_PTR structPtr;
  FORMAT_PTR nextFormat;
  FORMAT_ARRAY_PTR formatArray;
  int i, currentByte, currentData, structStart, arraySize, intVal;
  int elements;

  currentByte = bStart;
  currentData = dStart;

  switch(format->type) {
  case lengthFMT:
    TO_BUFFER_AND_ADVANCE(dataStruct+currentData, buffer, currentByte, 
			   format->formatter.i);
    currentData += format->formatter.i;
    break;
  case primitiveFMT:
    formatProc = TransTable[format->formatter.i];
    currentByte += formatProc(Encode, dataStruct, currentData,
				buffer, currentByte);
    currentData += formatProc(ALength, NULL, 0, NULL, 0);
    break;
  case pointerFMT:
    structPtr = REF(DATA_PTR, dataStruct, currentData);
    /* Z means data, 0 means NULL*/
    buffer[currentByte] = (structPtr) ? 'Z' : '\0'; 
    
    currentByte += sizeof(char);
    currentData += sizeof(DATA_PTR);
    if (structPtr) { 
      nextFormat = CHOOSE_PTR_FORMAT(format, parentFormat);
      sizes = transferToBuffer(nextFormat, structPtr, 0, 
			       buffer, currentByte, NULL);
      currentByte += sizes.buffer;
    }
    break;
  case structFMT:
    formatArray = format->formatter.a;
    for(i=1;i < formatArray[0].i;i++) {
      sizes = transferToBuffer(formatArray[i].f, dataStruct,
			       currentData, buffer, currentByte, format);
      currentByte += sizes.buffer;
      currentData = alignField(format, i, currentData+sizes.data);
    }
    break;
  case fixedArrayFMT:
    formatArray = format->formatter.a;
    arraySize = fixedArraySize(formatArray);
    nextFormat = formatArray[1].f;
#if !(defined(i386) || defined(pmax) || defined(VMS))
    if (fixedLengthFormat(nextFormat)) {
      elements = arraySize * dataStructureSize(nextFormat);
      bcopy(dataStruct+currentData, buffer+currentByte, elements);
      currentByte += elements;
      currentData += elements;
    }
    else 
#endif
      {
      for(i=0;i < arraySize;i++) {
	sizes = transferToBuffer(nextFormat, dataStruct, currentData,
				 buffer, currentByte, NULL);
	currentByte += sizes.buffer;
	currentData += sizes.data;
      }
    }
    break;
  case varArrayFMT:
    formatArray = format->formatter.a;
    arraySize = varArraySize(formatArray, parentFormat,
				dataStruct, currentData);
    intVal = htonInt(arraySize);
    nextFormat = formatArray[1].f;
    TO_BUFFER_AND_ADVANCE(&intVal, buffer, currentByte, sizeof(int));
    structPtr = REF(DATA_PTR, dataStruct, currentData);
#if !(defined(i386) || defined(pmax) || defined(VMS))
    if (fixedLengthFormat(nextFormat)) {
      elements = arraySize * dataStructureSize(nextFormat);
      bcopy(structPtr, buffer+currentByte, elements);
      currentByte += elements;
    } else 
#endif
      {
      structStart = 0;
      for(i=0;i < arraySize;i++) {
	sizes = transferToBuffer(nextFormat, structPtr, structStart,
				 buffer, currentByte, NULL);
	currentByte += sizes.buffer;
	structStart += sizes.data;
      }
    }
    currentData += sizeof(DATA_PTR);
  }

  sizes.buffer = currentByte - bStart;
  sizes.data = currentData - dStart;
  return sizes;  
}


/*****************************************************************************
*
* FUNCTION: int dataStructureSize(format) 
*
* DESCRIPTION:
*
* INPUTS: FORMAT_PTR format;
*
* OUTPUTS: int
*
*****************************************************************************/

int dataStructureSize(format) 
FORMAT_PTR format;
{ 
  int i, size;
  FORMAT_ARRAY_PTR formatArray;

  size = 0;

  switch(format->type) {
  case lengthFMT: 
    size += format->formatter.i; 
    break;
  case primitiveFMT:
    size += TransTable[format->formatter.i](ALength, NULL, 0, NULL, 0);
    break;
  case pointerFMT: 
  case varArrayFMT: 
    size += sizeof(DATA_PTR); 
    break;
  case structFMT: 
    formatArray = format->formatter.a;
    for(i=1;i < formatArray[0].i;i++) 
      size = alignField(format, i, size+dataStructureSize(formatArray[i].f));
    break;
  case fixedArrayFMT:
    formatArray = format->formatter.a;
    size += fixedArraySize(formatArray) * 
      dataStructureSize(formatArray[1].f);
    break;
  }

  return size;
}


/*****************************************************************************
*
* FUNCTION: 
*
* DESCRIPTION:
*
* INPUTS:
*
* OUTPUTS: 
*
*****************************************************************************/

SIZES_TYPE transferToDataStructure(format, dataStruct, dStart, buffer, bStart, 
				  parentFormat)
FORMAT_PTR format, parentFormat;
DATA_PTR dataStruct; 
int dStart, bStart;
DATA_PTR buffer;
{ 
  SIZES_TYPE sizes;
  int (*formatProc)();
  char ptrVal;
  DATA_PTR newStruct;
  FORMAT_PTR nextFormat;
  FORMAT_ARRAY_PTR formatArray;
  int i, currentByte, currentData, structStart, arraySize, intVal;
  int elements;

  currentByte = bStart;
  currentData = dStart;

  switch(format->type) {
  case lengthFMT:
    FROM_BUFFER_AND_ADVANCE(dataStruct+currentData, buffer, currentByte, 
			     format->formatter.i);
    currentData += format->formatter.i;
    break;
  case primitiveFMT:
    formatProc = TransTable[format->formatter.i];
    currentByte += formatProc(Decode, dataStruct, currentData,
				buffer, currentByte);
    currentData += formatProc(ALength, NULL, 0, NULL, 0);
    break;
  case pointerFMT:
    FROM_BUFFER_AND_ADVANCE(&ptrVal, buffer, currentByte, sizeof(char));
    if (ptrVal == '\0') 
      newStruct = NULL;
    else {
      nextFormat = CHOOSE_PTR_FORMAT(format, parentFormat);    
      newStruct = (DATA_PTR)tcaMalloc(dataStructureSize(nextFormat));
      sizes = transferToDataStructure(nextFormat, newStruct, 0,
				      buffer, currentByte, NULL);
      currentByte += sizes.buffer;
    }
    TO_BUFFER_AND_ADVANCE(&newStruct, dataStruct, 
			  currentData, sizeof(DATA_PTR));
    break;
  case structFMT:
    formatArray = format->formatter.a;
    for(i=1;i < formatArray[0].i;i++) {
      sizes = transferToDataStructure(formatArray[i].f, dataStruct, 
				      currentData, buffer, currentByte, 
				      format);
      currentByte += sizes.buffer;
      currentData = alignField(format, i, currentData+sizes.data);
    }
    break;
  case fixedArrayFMT:
    formatArray = format->formatter.a;
    arraySize = fixedArraySize(formatArray);
    nextFormat = formatArray[1].f;
#if !(defined(i386) || defined(pmax) || defined(VMS))
    if (fixedLengthFormat(nextFormat)) {
      elements = arraySize * dataStructureSize(nextFormat);
      bcopy(buffer+currentByte, dataStruct+currentData, elements);
      currentByte += elements;
      currentData += elements;
    }
    else 
#endif
      {
	for(i=0;i < arraySize;i++) {
	  sizes = transferToDataStructure(nextFormat, dataStruct, currentData,
					  buffer, currentByte, NULL);
	  currentByte += sizes.buffer;
	  currentData += sizes.data;
	}
      }
    break;
  case varArrayFMT:
    formatArray = format->formatter.a;
    FROM_BUFFER_AND_ADVANCE(&intVal, buffer, currentByte, sizeof(int));
    arraySize = ntohInt(intVal);
    nextFormat = formatArray[1].f;
    newStruct = ((arraySize == 0) ? NULL :
		 (DATA_PTR)tcaMalloc(arraySize*dataStructureSize(nextFormat)));
    TO_BUFFER_AND_ADVANCE(&newStruct, dataStruct, currentData, 
			  sizeof(DATA_PTR));
    if (newStruct) {
#if !(defined(i386) || defined(pmax) || defined(VMS))
      if (fixedLengthFormat(nextFormat)) {
	elements = arraySize * dataStructureSize(nextFormat);
	bcopy(buffer+currentByte, newStruct, elements);
	currentByte += elements;
      }
      else 
#endif
	{
	structStart = 0;
	for(i=0;i < arraySize;i++) {
	  sizes = transferToDataStructure(nextFormat, newStruct, 
					  structStart, buffer, 
					  currentByte, NULL);
	  currentByte += sizes.buffer;
	  structStart += sizes.data;
	}
      }
    }
  }

  sizes.buffer = currentByte - bStart;
  sizes.data = currentData - dStart;
  return sizes;  
}


/*************************************************************

  THESE FUNCTIONS FORM THE INTERFACE TO THE REST OF THE SYSTEM

*************************************************************/

int bufferSize(Format, DataStruct)
FORMAT_PTR Format; DATA_PTR DataStruct; 
{ 
  SIZES_TYPE sizes;

  sizes = bufferSize1(Format, DataStruct, 0, NULL);
  return sizes.buffer;
}

/*************************************************************
		 
int dataStructureSize(Format) FORMAT_PTR Format;
{
  .
  . Function is defined (and used) above
  .
}

*************************************************************/

void encodeData(Format, DataStruct, Buffer, BStart)
FORMAT_PTR Format;
DATA_PTR DataStruct; 
DATA_PTR Buffer; 
int BStart;
{
  transferToBuffer(Format, DataStruct, 0, Buffer, BStart, NULL); 
}

/*************************************************************/

char *decodeData(Format, Buffer, BStart)
FORMAT_PTR Format; 
DATA_PTR Buffer; 
int BStart;
{
  char *DataStruct;

  DataStruct = (char *)tcaMalloc(dataStructureSize(Format));

  transferToDataStructure(Format, DataStruct, 0, Buffer, BStart, NULL);

  return DataStruct;
}


/*****************************************************************************
*
* FUNCTION: freeDataElements
*
* DESCRIPTION: Frees the data elements "malloc"ed by the equivalent call
*              to "TransferToDataStructure"
*
* INPUTS: format of data structure
*         pointer data structure itself
*         start of relevant part of data structure
*         format of parent data structure (or NULL)
*
* OUTPUTS: number of bytes processed in the top-level dataStruct.
*
*****************************************************************************/

int freeDataElements(format, dataStruct, dStart, parentFormat)
FORMAT_PTR format, parentFormat;
DATA_PTR dataStruct; 
int dStart;
{ 
  int (*formatProc)();
  DATA_PTR structPtr;
  FORMAT_PTR nextFormat;
  FORMAT_ARRAY_PTR formatArray;
  int size, i, currentData, structStart, arraySize;

  currentData = dStart;

  switch(format->type) {
  case lengthFMT:
    currentData += format->formatter.i;
    break;

  case primitiveFMT:
    formatProc = TransTable[format->formatter.i];
    (void)formatProc(DFree, dataStruct, currentData, NULL, 0);
    currentData += formatProc(ALength, NULL, 0, NULL, 0);
    break;

  case pointerFMT:
    structPtr = REF(DATA_PTR, dataStruct, dStart);
    currentData += sizeof(DATA_PTR);
    if (structPtr) {
      nextFormat = CHOOSE_PTR_FORMAT(format, parentFormat);    
      size = freeDataElements(nextFormat, structPtr, 0, NULL);
      tcaFree(structPtr);
    }
    break;

  case structFMT:
    formatArray = format->formatter.a;
    for(i=1;i < formatArray[0].i;i++) {
    size = freeDataElements(formatArray[i].f, dataStruct, currentData, format);
      currentData = alignField(format, i, currentData+size);
    }
    break;

  case fixedArrayFMT:
    formatArray = format->formatter.a;
    arraySize = fixedArraySize(formatArray);
    nextFormat = formatArray[1].f;
    if (fixedLengthFormat(nextFormat)) {
      currentData += arraySize * dataStructureSize(nextFormat);
    }
    else {
      for(i=0;i < arraySize;i++) {
	size = freeDataElements(nextFormat, dataStruct, currentData, NULL);
	currentData += size;
      }
    }
    break;

  case varArrayFMT:
    structPtr = REF(DATA_PTR, dataStruct, currentData);
    if (structPtr) {
      formatArray = format->formatter.a;
      arraySize = varArraySize(formatArray, parentFormat,
				 dataStruct, currentData);
      nextFormat = formatArray[1].f;
      if (!fixedLengthFormat(nextFormat)) {
	structStart = 0;
	for(i=0;i < arraySize;i++) {
	  size = freeDataElements(nextFormat, structPtr, structStart, NULL);
	  structStart += size;
	}
      }
      tcaFree(structPtr);
    }
    currentData += sizeof(DATA_PTR);
  }

  return currentData - dStart;
}

/*************************************************************/

void freeDataStructure(format, dataStruct)
FORMAT_PTR format; 
char *dataStruct;
{
  freeDataElements(format, dataStruct, 0, NULL);

  tcaFree(dataStruct);
}

#if 0


/*****************************************************************************
*
* FUNCTION: void tcaFreeData(msgName, dataStruct)
*
* DESCRIPTION: 
* Using the queryForm format of the message, free the data structure
* and all of its allocated subelements
*
* INPUTS: 
* char *msgName; 
* char *dataStruct;
*
* OUTPUTS: void.
*
*****************************************************************************/

void tcaFreeData(msgName, dataStruct)
char *msgName; 
char *dataStruct;
{
  MSG_PTR msg;

  msg = msgFind(msgName, NULL);

  freeDataStructure(msg->msgData->msgForm, dataStruct);
}


/*****************************************************************************
*
* FUNCTION: void tcaFreeReply(msgName, replyData)
*
* DESCRIPTION: 
* Using the replyForm format of the message, free the all of the allocated
* subelements of the data structure, but do not free the data structure itself.
*
* INPUTS:
* char *msgName; 
* char *replyData;
*
* OUTPUTS: none
*
*****************************************************************************/

void tcaFreeReply(msgName, replyData)
char *msgName; 
char *replyData;
{
  MSG_PTR msg;

  msg = msgFind(msgName, NULL);

  freeDataElements(msg->msgData->resForm, replyData, 0, NULL);
}


/*****************************************************************************
*
* FUNCTION: void classDataFree(class, classData)
*
* DESCRIPTION: 
* Finds the class format associated with the class, and frees the data.
*
* INPUTS:
* TCA_MSG_CLASS_TYPE class;
* char *classData;
*
* OUTPUTS: none
*
*****************************************************************************/

void classDataFree(class, classData)
TCA_MSG_CLASS_TYPE class;
char *classData;
{
  CLASS_FORM_PTR classForm;

  if (classData) {
    classForm = (CLASS_FORM_PTR)hashTableFind(&class, 
					      Global->classFormatTable);
    if (!classForm) {
      tcaModError("ERROR: freeClassData: missing class format.");
    } else {
      freeDataStructure(classForm->format, classData);
    }
  }
}

#endif

