
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
* communications
*
* REVISION HISTORY
*
* 23-Nov-91 Christopher Fedor, School of Computer Science, CMU
* created.
*
******************************************************************************/

#ifndef INCtcx
#define INCtcx


/******** begin changes Sebastian Thrun 93-1-5 ******************/
/******** updated to support VMS - Tyson Sawyer 13 Oct 1994 *****/
#if defined(i386) || defined(VMS)

#ifndef htons
#include <sys/types.h>
#include <netinet/in.h>
#endif

#define htonChar(c) (c)
#define htonShort(s) (short)htons((u_short)s)
#define htonInt(i) (int)htonl((u_long)i)
#define htonLong(l) (long)htonl((u_long)l)
extern float htonFloat(float f);
extern double htonDouble(double d);

#define ntohChar(c) (c)
#define ntohShort(s) (short)ntohs((u_short)s)
#define ntohInt(i) (int)ntohl((u_long)i)
#define ntohLong(l) (long)ntohl((u_long)l)
extern float ntohFloat(float f);
extern double ntohDouble(double d);

#else

#define htonChar(c) (c)
#define htonShort(s) (s)
#define htonInt(i) (i)
#define htonLong(l) (l)
#define htonFloat(f) (f)
#define htonDouble(d) (d)

#define ntohChar(c) (c)
#define ntohShort(s) (s)
#define ntohInt(i) (i)
#define ntohLong(l) (l)
#define ntohFloat(f) (f)
#define ntohDouble(d) (d)

#endif
/******** end changes Sebastian Thrun 93-1-5 ******************/






#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define TCX_RECV_ALL  1
#define TCX_RECV_NONE 2
#define TCX_RECV_ONCE 3

#define NO_HND       0x0000
#define TCX_HND      0x0001
#define USER_HND     0x0002
#define ALL_HND      (USER_HND | TCX_HND)

typedef struct _module *TCX_MODULE_PTR;

typedef struct _TCX_REF_TYPE *TCX_REF_PTR;

typedef struct _FORMAT_TYPE *TCX_FMT_PTR;

typedef struct {
  char *msgName;
  char *msgFormat;
} TCX_REG_MSG_TYPE;

typedef struct {
  char *name;
  char *format;
  int connections;
} TCX_REG2_MSG_TYPE;
                    
typedef struct {
  char *msgName;
  char *hndName;
  void (*hndProc)();
  int hndControl;
  unsigned char  *hndData;
} TCX_REG_HND_TYPE;

extern TCX_FMT_PTR tcxCreateFMT;

#ifdef __cplusplus
extern "C" {
#endif
extern void tcxInitialize(char *module, char *server);
extern void tcxInitializeServerRoute(char *module, char *server);

extern TCX_MODULE_PTR tcxConnectModule(char *module);
extern TCX_MODULE_PTR tcxConnectOptional(char *module);

extern char *tcxModuleName(TCX_MODULE_PTR module);

extern void tcxSendData(TCX_MODULE_PTR module, int id, int ins, void *data, 
			int len, TCX_FMT_PTR fmt);

extern int tcxRecvData(TCX_MODULE_PTR *module, int *id, int *ins, void *data, 
		       TCX_FMT_PTR fmt, int allowHnd, void *timeout);

extern TCX_FMT_PTR tcxParseFormat(char *format);

extern void tcxFree(char *msgName, void *data);
extern void tcxFreeReply(char *msgName, void *data);
extern void tcxFreeByRef(TCX_REF_PTR ref, void *data);
extern void tcxFreeData(TCX_FMT_PTR fmt, void *data);

extern void tcxDecodeData(TCX_FMT_PTR fmt, void *buf, void *data);

extern void tcxRegisterExitHnd(void (*proc)());

extern void tcxRegisterConnectHnd(char *name, void (*openHnd)(), 
				  void (*closeHnd)());

extern void tcxRegisterMessages(TCX_REG_MSG_TYPE *msgArray, int size);
extern void tcxRegisterHandlers(TCX_REG_HND_TYPE *hndArray, int size);

extern void tcxRegisterMHnd(char *msgName, void (*proc)(), 
			    int control, int mask, void *data);

extern void tcxRegisterDHnd(int id, TCX_FMT_PTR fmt, 
			    void (*proc)(), int control, int mask, void *data);

extern void tcxRecvFlush(TCX_MODULE_PTR module, int id);

extern int tcxSendMsg(TCX_MODULE_PTR module, char *msgName, void *data);

extern int tcxSendDoubleMsg(TCX_MODULE_PTR module, char *name1, void *data1,
		      char *name2, void *data2);

extern int tcxRecvLoop(void *timeout);

extern void tcxQuery(TCX_MODULE_PTR module, char *msgName, 
		     void *data, char *replyMsgName, void *reply);

extern void tcxReply(TCX_REF_PTR ref, char *replyMsgName, void *reply);

extern void tcxReplyData(TCX_MODULE_PTR module, int id, int ins, void *reply);

extern char *tcxMessageName(int id);
extern int tcxMessageId(char *name);

extern TCX_MODULE_PTR tcxRefModule(TCX_REF_PTR ref);
extern int tcxRefId(TCX_REF_PTR ref);
extern int tcxRefIns(TCX_REF_PTR ref);

extern int tcxRecvMsg(char *msgName, int *ins, void *data, void *timeout);
extern int tcxRecvMsgNoHnd(char *msgName, int *ins, void *data, void *timeout);

extern int tcxRecvMsgE(TCX_MODULE_PTR *module, char *msgName, int *ins, 
		       void *data, int allowHnd, void *timeout);

extern int tcxRecvResp(char *msgName, int *ins, void *data, void *timeout);
extern int tcxRecvRespE(TCX_MODULE_PTR *module, char *msgName, int *ins, 
		       void *data, int allowHnd, void *timeout);

/* 3-nov-93: fedor ***** new stuff for pbk - will need general clean up ****/

extern void tcxRegisterCloseHnd(void (*closeHnd)());

extern int tcxNRecvMsgE(TCX_MODULE_PTR *module, char **msgName, void *data, 
			void *timeout, int *inst, int hndMask);

extern char *tcxCurrentModuleName(void);

/*****************/

extern TCX_REF_PTR tcxRefCopy(TCX_REF_PTR ref);
extern void tcxRefFree(TCX_REF_PTR ref);

extern void tcxSetAutoReconnect();
extern int tcxTestActiveModule(TCX_MODULE_PTR module);

extern void tcxCloseAll();

extern void tcxLock();		/* not implemented */

extern void tcxUnlock();		/* not implemented */

/*****************/

void
check_version_number(int major, int minor, int robot_type, char *date, 
		     char *reference_text, 
		     int final_check);



#ifdef __cplusplus
}
#endif
#endif /* INCtcx */





