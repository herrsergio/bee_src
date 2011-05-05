
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

#if 0
#include <syslog.h>
#endif

#define _pgmTrace() \
  { \
    fprintf(stderr, DBG_PRE_FMT"\n", DBG_PRE_ARGS); \
  }

#define _return_d(x) \
  { \
    fprintf(stderr, DBG_PRE_FMT "return(0x%08X, %d);\n", \
           DBG_PRE_ARGS, (unsigned)x, (int)x); \
    return(x); \
  }

#define _return_v \
  { \
    fprintf(stderr, DBG_PRE_FMT "return(void);\n", \
           DBG_PRE_ARGS); \
    return; \
  }

#ifdef DEBUG_PGM_TRACE

#define pgmTrace() \
  { \
    fprintf(stderr, DBG_PRE_FMT "\n", DBG_PRE_ARGS); \
  }

#define return_d(x) \
  { \
    fprintf(stderr, DBG_PRE_FMT "return(0x%08X, %d);\n", \
           DBG_PRE_ARGS, (unsigned)x, (int)x); \
    return(x); \
  }

#define return_v \
  { \
    fprintf(stderr, DBG_PRE_FMT "return(void);\n", \
           DBG_PRE_ARGS); \
    return; \
  }

#else /* DEBUG_PGM_TRACE */

#define pgmTrace() {}
#define return_d(x) return(x)
#define return_v return

#endif /* DEBUG_PGM_TRACE */

#ifdef DEBUG_INFORM
#define dbgInform(format, args...) fprintf(stderr, \
                                        DBG_PRE_FMT format, \
                                        DBG_PRE_ARGS , ## args)
#else
#define dbgInform(A...) {}
#endif /* DEBUG_WARN */

#ifdef DEBUG_ALERT
#define dbgAlert(format, args...) fprintf(stderr, \
                                        DBG_PRE_FMT "ALERT: " format, \
                                        DBG_PRE_ARGS , ## args)
#else
#define dbgAlert(A...) {}
#endif /* DEBUG_ALERT */

#ifdef DEBUG_WARN
#define dbgWarn(format, args...) fprintf(stderr, \
                                        DBG_PRE_FMT "Warn: " format, \
                                        DBG_PRE_ARGS , ## args)
#else
#define dbgWarn(A...) {}
#endif /* DEBUG_WARN */

#ifdef DEBUG_CMD
#define dbgCmd(format, args...)  fprintf(stderr, \
                                        DBG_PRE_FMT "Cmd: " format, \
                                        DBG_PRE_ARGS , ## args)
#else
#define dbgCmd(A...) {}
#endif /* DEBUG_CMD */

#ifdef DEBUG_MSG_1
#define dbgMsg1(format, args...) fprintf(stderr, \
                                        DBG_PRE_FMT "dbgMsg1: " format, \
                                        DBG_PRE_ARGS , ## args)
#else
#define dbgMsg1(A...) {}
#endif /* DEBUG_MSG_1 */

#ifdef DEBUG_MSG_2
#define dbgMsg2(format, args...) fprintf(stderr, \
                                        DBG_PRE_FMT "dbgMsg2: " format, \
                                        DBG_PRE_ARGS , ## args)
#else
#define dbgMsg2(A...) {}
#endif /* DEBUG_MSG_2 */

#ifdef DEBUG_MSG_3
#define dbgMsg3(format, args...) fprintf(stderr, \
                                        DBG_PRE_FMT "dbgMsg3: " format, \
                                        DBG_PRE_ARGS , ## args)
#else
#define dbgMsg3(A...) {}
#endif /* DEBUG_MSG_1 */

/*****************************************************************
 *
 * End of debug utilities
 *
 *****************************************************************/
