
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
* FILE: formatters.h
*
* ABSTRACT:
*
* Data Format Routines. Include File.
*
*****************************************************************************/

#ifndef INCformatters
#define INCformatters

typedef enum {Encode, Decode, ELength,
		ALength, RLength, SimpleType, DPrint, DFree} TRANS_OP_TYPE;

/* note: fedor: 31-Aug-89 should this go away??? */
typedef char *DATA_PTR;

typedef enum {primitiveFMT, lengthFMT, structFMT, pointerFMT, fixedArrayFMT,
	      varArrayFMT} FORMAT_CLASS_TYPE;

typedef union { 
  int i;
  struct _FORMAT_TYPE *f;
} FORMAT_ARRAY_TYPE, *FORMAT_ARRAY_PTR;

typedef union {
int i;
struct _FORMAT_TYPE *f;
FORMAT_ARRAY_PTR     a;
} FMT_ELEMENT_TYPE;

typedef struct _FORMAT_TYPE {
  FORMAT_CLASS_TYPE type;
  FMT_ELEMENT_TYPE formatter;
} FORMAT_TYPE, *FORMAT_PTR;

typedef struct {
  int bstart;
  char *buffer;
} BUFFER_TYPE, *BUFFER_PTR;

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#define MAXFORMATTERS 30

#define INT_FMT     1
#define BOOLEAN_FMT 2
#define FLOAT_FMT   3
#define DOUBLE_FMT  4
#define BYTE_FMT    5
#define TWOBYTE_FMT 6
#define STR_FMT     7
#define FORMAT_FMT  8
#define UBYTE_FMT   9
#define CMAT_FMT    10
#define SMAT_FMT    11
#define IMAT_FMT    12
#define LMAT_FMT    13
#define FMAT_FMT    14
#define DMAT_FMT    15
#define CHAR_FMT    16
#define SHORT_FMT   17
#define LONG_FMT    18
#define UCMAT_FMT   19
#define TCA_REF_PTR_FMT 20

#define SIUCMAT_FMT 21
#define SICMAT_FMT  22
#define SISMAT_FMT  23
#define SIIMAT_FMT  24
#define SILMAT_FMT  25
#define SIFMAT_FMT  26
#define SIDMAT_FMT  27

#define REF(type, datastruct, dstart) *(type *)(datastruct+dstart)

#define TO_BUFFER_AND_ADVANCE(data, buffer, bstart, length) \
  {bcopy(data, (buffer)+bstart, length); bstart += length;}

#define FROM_BUFFER_AND_ADVANCE(data, buffer, bstart, length) \
  {bcopy((buffer)+bstart, data, length); bstart += length;}

#define NEW_FORMATTER() (FORMAT_PTR)tcaMalloc(sizeof(FORMAT_TYPE))

#define NEW_FORMAT_ARRAY(size) \
  (FORMAT_ARRAY_PTR)tcaMalloc(size * sizeof(FORMAT_ARRAY_TYPE))

#endif INCformatters
