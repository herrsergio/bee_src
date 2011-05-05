
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


#ifndef __RAI_SERVER__
#define __RAI_SERVER__

#include <tcx.h>
#include <tcxP.h>

/* how often, in msec, a server should poll TCX for messages from clients */
#define TCX_SERVER_POLLING_INTERVAL 100
#define raiVariableMsg "{int,int,<char: 2>}"

typedef struct 
   {int operation; int bufsize; unsigned char * buffer;} RAI_VariableMsgType;
  

#define raiFixedMsg "{int, long}"
typedef struct {int operation; unsigned long parameter;} RAI_FixedMsgType;

#endif
