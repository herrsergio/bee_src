
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



#ifndef _INIT_CLIENT_H
#define _INIT_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <tcx.h>

typedef void (*clientCloseFcn)(char *name, TCX_MODULE_PTR module);
typedef void (*clientCallback)(int,unsigned long); 

/* how often the client should poll for replies from servers, in msec */
#define TCX_CLIENT_POLLING_INTERVAL 100

void initClient(char* name, clientCloseFcn f);
void shutdownClient();
void registerInterface(char* name, int numMsgs, TCX_REG_MSG_TYPE messages[],
		       int numHnds, TCX_REG_HND_TYPE handlers[]);
void closeClient(void);

void initClientModules(void);

#ifdef __cplusplus
}
#endif

#endif
