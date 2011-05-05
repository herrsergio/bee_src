
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
* PROJECT: TCX
* 
* PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
* (c) Copyright 1993 Christopher Fedor. All rights reserved.
*
* MODULE: communications
*
* FILE: tcx.h
*
* ABSTRACT:
* 
* New Communications
*
* REVISION HISTORY
*
* 23-Nov-91 Christopher Fedor, School of Computer Science, CMU
* created.
*
******************************************************************************/

#ifdef i386
#include <malloc.h>
#endif

#ifndef INCtcxP
#define INCtcxP

/* Integer Version Values for Handshaking */
/* Do not change these values manually */
/* They will get updated automatically upon a 'cvs commit' */
#define TCX_VERSION_MAJOR   9
#define TCX_VERSION_MINOR	0
#define VERSION_DATE		"21-Feb-97"
#define TCX_VERSION "TCX-VERSION 9.0 21-Feb-97"

/* tca services ports 1621 is debug. Correct reserved number is 1381 */
#define SERVER_PORT 1387 /* the vapor service */
#define SERVER_NAME "TCX Server"

#define MODULE_TEST  0x0001
#define ID_TEST      0x0002
#define REF_TEST     0x0004
#define MOD_RETURN   0x0008
#define ID_RETURN    0x0010
#define REF_RETURN   0x0020

#define STATUS_UNKNOWN      0
#define STATUS_PENDING      0x0001
#define STATUS_INITIATED    0x0002
#define STATUS_CONNECTED    0x0004
#define STATUS_CLOSED       0x0008

#define STATUS_SERVER_FORW  0x0020
#define STATUS_SINGLE_CONN  0x0010

#define STATUS_SERVER_ROUTE (STATUS_SINGLE_CONN | STATUS_SERVER_FORW)

#define STATUS_ACTIVE       (STATUS_INITIATED | STATUS_CONNECTED)
#define STATUS_INACTIVE     (STATUS_PENDING   | STATUS_CLOSED)

typedef struct {
  int vMaj;
  int vMin;
  int beeSoftMaj;
  int beeSoftMin;
  int beeSoftRobotType;
} VERSION_TYPE, *VERSION_PTR;

#define MSG_REG_FORM         "{int, <{string, string}:1>}"
#define MSG_REG_REPLY_FORM   "{int, <int: 1>}"
#define HND_REG_FORM         "{int, <{string, string, long, int, long}: 1>}"

#define MSG_REG2_FORM        "{int, <{string, string, int}:1>}"
#define MSG_REG2_REPLY_FORM  "{int, <int: 1>}"

#define MSG_FORW_FORM        NULL

#define VERSION_FORM         "{int, int, int, int, int}"

#define NEW_CON_FORM         "{int, string, string}"

typedef struct {
  int port;
  char *name;
  char *host;
} NEW_CON_TYPE, *NEW_CON_PTR;

TCX_FMT_PTR newConFMT;

/************************************************/

typedef struct {
  int num;
  TCX_REG_MSG_TYPE *msgs;
} MSG_REG_TYPE, *MSG_REG_PTR;

typedef struct {
  int num;
  int *ids;
} MSG_REG_REPLY_TYPE, *MSG_REG_REPLY_PTR;

typedef struct {
  int num;
  TCX_REG2_MSG_TYPE *msgs;
} MSG_REG2_TYPE, *MSG_REG2_PTR;

typedef struct {
  int num;
  int *ids;
} MSG_REG2_REPLY_TYPE, *MSG_REG2_REPLY_PTR;

typedef struct {
  int num;
  TCX_REG_HND_TYPE *hnds;
} HND_REG_TYPE, *HND_REG_PTR;

typedef struct {
  int msg1;
  int ref1;
  int size1;
  int msg2;
  int ref2;
  int size2;
} HDR_TYPE, *HDR_PTR;

typedef struct {
  int port;
  int status;
  char *name, *host;
  int vmajor, vminor;
} MODULE_INFO_TYPE, *MODULE_INFO_PTR;

#define MODULE_INFO_FORMAT "{int, int, string, string, int, int}"

typedef struct _module {
  int sd;
  int recv;
  int status;
  MODULE_INFO_PTR moduleInfo;
  struct _LIST *itemQ;
  struct _LIST *pending;
  struct _LIST *connections;
  struct _LIST *oldConnections;
} MODULE_TYPE, *MODULE_PTR;

typedef struct {
  int hnd;
  int id;
  int ref;
  int mask;
  MODULE_PTR module;
} ITEM_TEST_TYPE, *ITEM_TEST_PTR;

typedef struct {
  char *name;
  MODULE_PTR module;
  struct _LIST *taps;
} PEND_CONNECT_TYPE, *PEND_CONNECT_PTR;

typedef struct {
  int len;
  char *buf;
} DATA_BUF_TYPE, *DATA_BUF_PTR;

typedef enum {DataHnd, MsgHnd} HND_TYPE_ENUM;

typedef struct {
  void (*proc)();
  int control;
  int status;
  int id;
  int mask;
  TCX_FMT_PTR fmt;
  void *callData;
  HND_TYPE_ENUM type;
  MODULE_PTR module;
} HND_TYPE, *HND_PTR;

typedef struct _TCX_REF_TYPE {
  MODULE_PTR module;
  int id, ref, free;
} TCX_REF_TYPE;

/* 13-Jan-93: fedor: this should go away with rewrite */
typedef struct {
  char *name;
  void (*openHnd)();
  void (*closeHnd)();
} HND_CONNECT_TYPE, *HND_CONNECT_PTR;

#define HND_EXEC    1
#define HND_NO_EXEC 2

#endif /* INCtcxP */
