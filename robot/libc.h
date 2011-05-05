
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








/***********************************************************************
 * HISTORY
 * $Log: libc.h,v $
 * Revision 1.5  1998/02/07 23:40:15  swa
 * now works with redhat5.0
 *
 * Revision 1.4  1997/10/02 11:27:08  rhino
 * Adapted to new solaris version.
 *
 * Revision 1.3  1997/06/27 01:48:33  thrun
 * commented out sone lines that caused the simulator under SUN OS (only)
 * some trouble
 *
 * Revision 1.2  1997/02/22 05:16:49  thrun
 * Version numbers are now also checked for libraries.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:12  rhino
 * General reorganization of the directories/repository, fusion with the
 * RWI software.
 *
 * Revision 1.8  1996/05/16 20:14:05  tyson
 * Added support for RAI watchdog - TDS
 *
 * Revision 1.7  1996/01/09  11:08:45  rhino
 * First atempt to change to SOLARIS
 *
 * Revision 1.6  1994/12/14  16:02:39  tyson
 * added VMS support
 *
 * Revision 1.5  1994/12/12  17:19:14  rhino
 * removed a c++ comment.
 *
 * Revision 1.4  1994/11/30  12:14:48  ft
 * avoid C++ problems
 *
 * Revision 1.3  1994/05/31  20:59:11  rhino
 * General reorganization. New header file. New makefiles.
 *
 * Revision 1.2  1994/05/31  16:18:11  rhino
 * .
 *
 * Revision 1.1.1.1  1994/03/31  08:34:59  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:16  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:31  rhino
 * test
 *
 * Revision 1.9  1993/11/24  22:22:52  rich
 * New version of tca.
 *
 * Revision 1.8  1993/10/27  16:11:27  rich
 * Saving other's changes that looked good.
 *
 * Revision 1.7  1993/08/27  19:59:09  rich
 * Got rid of VOID_FN and INT_FN and replaced them with real definitions.
 * Put all the Xlib includes in ezx.h
 *
 * Revision 1.6  1993/07/28  19:05:20  rich
 * Post DC cleanup
 *
 * Revision 1.5  1993/06/28  06:13:42  rich
 * Added some more missing routines to libc.h
 *
 * Revision 1.4  1993/06/03  22:43:35  rich
 * Updated some headers.
 *
 * Revision 1.3  1993/04/24  16:41:07  rich
 * RTG - added automatic logging to source files
 *
 * Revision 1.2  1993/02/19  21:41:04  rich
 * RTG - fixed libc.h for pmax
 *
 * Revision 1.1  1993/02/19  20:16:26  rich
 * RTG - forgot libc.h
 *
 * Revision 1.7  89/04/03  11:10:45  vanryzin
 * 	Changed definition of qsort for c++ to indicate the procedure
 * 	passed to qsort has parameters.  Since we were unsure if ANSI C
 * 	could handle the syntax I placed the new definition within #if
 * 	defined(c_plusplus) conditionals.  This may not be necessary
 * 	and perhaps should be fixed at a later time.
 * 	[89/04/03            vanryzin]
 * 
 * Revision 1.6  89/02/05  15:55:57  gm0w
 * 	Added extern char *errmsg().
 * 	[89/02/04            gm0w]
 * 
 * Revision 1.5  89/01/20  15:34:40  gm0w
 * 	Moved all of the STDC changes to other existing include files
 * 	back into this one.  Added non-STDC extern declarations for
 * 	all functions without int return values to match those defined
 * 	by STDC.  Added include of sysent.h.  Removed obsolete cdate
 * 	extern declaration.
 * 	[88/12/17            gm0w]
 * 
 * Revision 1.4  88/12/22  16:58:56  mja
 * 	Correct __STDC__ parameter type for getenv().
 * 	[88/12/20            dld]
 * 
 * Revision 1.3  88/12/14  23:31:42  mja
 * 	Made file reentrant.  Added declarations for __STDC__.
 * 	[88/01/06            jjk]
 * 
 * 30-Apr-88  Glenn Marcy (gm0w) at Carnegie-Mellon University
 *	Added pathof() extern.
 *
 * 01-Dec-85  Glenn Marcy (gm0w) at Carnegie-Mellon University
 *	Added getname() extern.
 *
 * 29-Nov-85  Glenn Marcy (gm0w) at Carnegie-Mellon University
 *	Added lseek() extern.
 *
 * 02-Nov-85  Glenn Marcy (gm0w) at Carnegie-Mellon University
 *	Added salloc() extern.
 *
 * 14-Aug-81  Mike Accetta (mja) at Carnegie-Mellon University
 *	Created.
 *
 **********************************************************************
 */


/***************************************************************************
 ***************************************************************************
 ***************************************************************************/


extern int  librobot_major;
extern int  librobot_minor;
extern int  librobot_robot_type;
extern char librobot_date[80];


/***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#ifndef VMS
#if !(defined(_LIBC_H_) || defined(NEXT))
#define	_LIBC_H_ 1

#ifndef _TYPES_
#include <sys/types.h>
#endif	/* _TYPES_ */

#ifndef	_SYSENT_H_
/* #include <sysent.h> */
#endif	/* _SYSENT_H_ */

#ifndef	FILE
#include <stdio.h>
#endif	/* FILE */

#ifndef	_STRINGS_H_
#ifdef VMS
#include <string.h>
#else
#include <strings.h>
#endif
#endif	/* _STRINGS_H_ */

#ifndef	_SYS_TIME_H_
#include <sys/time.h>
#endif	/* _SYS_TIME_H_ */

/*  CMU stdio additions */
#if defined(__STDC__)
extern FILE *fopenp(const char*, const char*, char*, char*);
extern FILE *fwantread(const char*, const char*, const char*, const char*);
extern FILE *fwantwrite(const char*, const char*, const char*, const char*,
			int);
#else	/* __STDC__ */
extern FILE *fopenp();
extern FILE *fwantread();
extern FILE *fwantwrite();
#endif	/* __STDC__ */

/* CMU string routines */
#if defined(__STDC__)
#ifndef i386
/*extern int strncasecmp(const char *c, const char *wait_str, int wait_str_len);*/
#endif
extern char* foldup(char*, const char*);
extern char* folddown(char*, const char*);
extern char* sindex(const char*, const char*);
extern char* skipto(const char*, const char*);
extern char* skipover(const char*, const char*);
extern char* nxtarg(char**, const char*);
extern char _argbreak;
#ifndef getstr
extern char* getstr(const char*, char*, char*);
#endif
extern int getstab(const char*, const char**, const char*);
extern int getsearch(const char*, const char**, const char*);
extern char* strarg(const char**, const char*, const char*, char*, char*);
extern int stabarg(const char**, const char*, const char*, const char**,
		   const char*);
extern int searcharg(const char**, const char*, const char*, const char**,
		     const char*);
extern int getint(const char*, int, int, int);
extern int intarg(const char**, const char*, const char*, int, int, int);
extern long getlong(const char*, long, long, long);
extern long longarg(const char**, const char*, const char*, long, long, long);
extern short getshort(const char*, short, short, short);
extern short shortarg(const char**, const char*, const char*,
		      short, short, short);
extern float getfloat(const char*, float, float, float);
extern float floatarg(const char**, const char*, const char*,
		      float, float, float);
extern double getdouble(const char*, double, double, double);
extern double doublearg(const char**, const char*, const char*,
			double, double, double);
extern unsigned int getoct(const char*, unsigned int, unsigned int,
			   unsigned int);
extern unsigned int octarg(const char**, const char*, const char*,
			   unsigned int, unsigned int, unsigned int);
extern unsigned int gethex(const char*, unsigned int, unsigned int,
			   unsigned int);
extern unsigned int hexarg(const char**, const char*, const char*,
			   unsigned int, unsigned int, unsigned int);
extern unsigned int atoo(const char*);
extern unsigned int atoh(const char*);
extern char *salloc(const char*);
extern char *concat(const char*, int, ...);
#else	/* __STDC__ */
extern int strncasecmp();
extern char *foldup(), *folddown();
extern char *sindex(), *skipto(), *skipover(), *nxtarg();
extern char *getstr(), *strarg();
extern long getlong(), longarg();
extern short getshort(), shortarg();
extern float getfloat(), floatarg();
extern double getdouble(), doublearg();
extern unsigned int getoct(), octarg(), gethex(), hexarg();
extern unsigned int atoo(), atoh();
extern char *salloc();
extern char *concat();
#endif	/* __STDC__ */

/* CMU library routines */
#if defined(__STDC__)
extern char *getname(int);
extern char *pathof(char *);
extern char *errmsg(int);
#else	/* __STDC__ */
extern char *getname();
extern char *pathof();
extern char *errmsg();
#endif	/* __STDC__ */

/*  CMU time additions */
#if defined(__STDC__)
extern long gtime(const struct tm*);
extern long atot(const char*);
#else	/* __STDC__ */
extern long gtime();
extern long atot();
#endif	/* __STDC__ */

/* 4.3 BSD standard library routines; taken from man(3) */
#if defined(__STDC__)
typedef int (*PFI)(void);
#if defined(c_plusplus)
typedef int (*PFI2)(...);
#endif /* c_plusplus */
/****************************************/
#ifndef  __cplusplus
extern int abs(int);
#else
extern "C" int abs(int);
#endif
/****************************************/
#ifndef i386
extern double atof(const char *);
extern int atoi(const char *);
extern long atol(const char *);
#endif
#ifndef i386
/*extern void bcopy(const void *, void *, int);*/
/*extern int bcmp(const void *, const void *, int);*/
/*extern void bzero(void *, int);*/
#endif
extern int ffs(int);
extern char *crypt(const char *, const char *);
/*extern void setkey(char *);*/
extern void encrypt(char *, int);
#ifndef i386
extern char *ecvt(double, int, int *, int *);
extern char *fcvt(double, int, int *, int *);
extern char *gcvt(double, int, char *);
#endif
#ifdef i386
extern int execl(const char *, ...);
extern int execv(const char *, const char **);
extern int execle(const char *, ...);
extern int exect(const char *, const char **, const char **);
#endif
extern char *getenv(const char *);
extern char *getlogin(void);
#ifndef i386
/****************************************/
#ifndef  __cplusplus
/*extern int getopt(int, const char **, const char *);*/
#else
extern "C" int getopt(int, const char **, const char *);
#endif
/****************************************/
#endif
extern char *getpass(const char *);
extern char *getusershell(void);
extern void setusershell(void);
extern void endusershell(void);
extern char *getwd(char *);
#ifndef i386
extern char *cfree(void *);
/*extern void *malloc(unsigned);*/
/*extern void *realloc(void *, unsigned);*/
/*extern void *calloc(unsigned,unsigned);*/
#endif

extern int initgroups(const char *, gid_t);
extern char *mktemp(char *);
extern int mkstemp(char *);
extern void monitor(PFI, PFI, short *, int, int);
extern void monstartup(PFI, PFI);
extern void moncontrol(int);
extern int pause(void);
#ifndef i386
extern long random(void);
#if (OS_VER == 5) && (OS_MINOR_VER != 6) 
extern int srandom(int);
#endif
#endif
#ifndef i386
#if (OS_VER == 5) && (OS_MINOR_VER != 6) 
extern void *initstate(unsigned, void *, int);
#endif
#endif
#ifndef i386
#if (OS_VER == 5) && (OS_MINOR_VER != 6) 
extern void *setstate(void *);
#endif
extern int rcmd(char **, int, const char *, const char *, const char *, int);
#endif
extern int rresvport(int *);
#ifndef i386
extern int ruserok(char *, int, const char *, const char *);
#endif
extern char *re_comp(char *);
extern int re_exec(char *);
extern int rexec(char **, int, const char *, const char *, const char *,
		 int *);
extern int setuid(uid_t);
extern int seteuid(uid_t);
extern int setruid(uid_t);
extern int setgid(gid_t);
extern int setegid(gid_t);
extern int setrgid(gid_t);
#ifdef i386
extern void sleep(unsigned);
#endif
#ifndef i386
/*extern void swab(void *, void *, int);*/
#endif
extern int system(const char *);
extern char *ttyname(int);
extern int isatty(int);
extern int ttyslot(void);
extern unsigned ualarm(unsigned, unsigned);
/****************************************/
#ifndef  __cplusplus
extern void usleep(unsigned);
#else
extern "C" void usleep(unsigned);
#endif
/****************************************/

/* define some things that are in the stdio.h file for __STDC__ */
extern int _filbuf(FILE*);
extern int printf(const char*,...);
extern int scanf(const char*,...);
extern int fscanf(FILE*, const char*,...);
#ifndef i386
extern int sscanf(const char*, const char*,...);
#endif
extern int fflush(FILE*);
extern int fprintf(FILE*, const char*,...);
extern int fseek(FILE*, long, int);

/* defines that should be in <sys/socket.h> */
#include <sys/socket.h>
#ifndef i386
extern int accept(int, struct sockaddr *, int *);
#if (OS_VER == 5) && (OS_MINOR_VER != 6) 
extern int bind(int, struct sockaddr *, int);
#endif
#endif
#ifndef i386
extern int connect(int, struct sockaddr *, int);
#endif
/*extern int getsockopt(int, int, int, void *optval, int *);*/
#ifndef i386
/*extern int setsockopt(int, int, int, void *optval, int);*/
extern int listen(int, int);
#endif
extern int socket(int, int, int);
extern int gethostname(const char *name, int namelen);
extern shutdown(int s, int how);

#ifndef timeb
#include <sys/timeb.h>
#endif
/* defines that should be in <sys/time.h> */
extern long time(long*);
extern int ftime(struct timeb *tp);
/*extern int gettimeofday(struct timeval *, struct timezone *);*/

/* defines that should be in <sys/errno.h> */
extern int errno;
extern void perror(const char *);
#ifndef i386
extern char *sys_errlist[];
#endif
extern int sys_nerr;

/* defines that should be in <strings.h> */
extern char *strstr(const char*, const char *);
extern char *strdup(const char *);

/* other declarations */
#ifndef i386
extern int ioctl(int, unsigned long, void *);
#endif
/****************************************/
#ifndef  __cplusplus
extern int select(int, fd_set *, fd_set *, fd_set *, struct timeval *);
#else
extern "C" int select(int, fd_set *, fd_set *, fd_set *, struct timeval *);
#endif
/****************************************/
#ifdef i386
extern int write(int, void *, int);
extern int read(int, void *, int);
#endif
extern int close(int);
extern int fclose(FILE*);
/*#ifndef i386*/
/*extern int open(const char *, int, int);*/
/*#endif*/
#else	/* __STDC__ */
/****************************************/
#ifndef  __cplusplus
extern int abs();
#else
extern "C" int abs();
#endif
/****************************************/
extern double atof();
extern int atoi();
extern long atol();
extern void bcopy();
extern int bcmp();
extern void bzero();
extern int ffs();
extern char *crypt();
extern void setkey();
extern void encrypt();
extern char *ecvt();
extern char *fcvt();
extern char *gcvt();
extern int execl();
extern int execv();
extern int execle();
extern int exect();
extern char *getenv();
extern char *getlogin();
/****************************************/
#ifndef  __cplusplus
extern int getopt();
#else
extern "C" int getopt();
#endif
/****************************************/
extern char *getpass();
extern char *getusershell();
extern void setusershell();
extern void endusershell();
extern int initgroups();
extern char *getwd();
#ifndef i386
extern char *cfree();
#endif
extern char *malloc();
extern char *realloc();
extern char *calloc();
extern char *alloca();
extern char *mktemp();
extern int mkstemp();
extern void monitor();
extern void monstartup();
extern void moncontrol();
extern int pause();
#ifndef i386
extern long random();
#endif
extern char *initstate();
extern char *setstate();
extern int rcmd();
extern int rresvport();
extern int ruserok();
extern char *re_comp();
extern int re_exec();
extern int rexec();
extern int setuid();
extern int seteuid();
extern int setruid();
extern int setgid();
extern int setegid();
extern int setrgid();
extern void sleep();
extern void swab();
extern int system();
extern char *ttyname();
extern int isatty();
extern int ttyslot();
extern unsigned ualarm();
/****************************************/
#ifndef  __cplusplus
extern void usleep();
#else
extern "C" void usleep();
#endif
/****************************************/

/* define some things that are in the stdio.h file for __STDC__ */
extern int _filbuf();
extern int printf();
extern int fprintf();
extern int scanf();
#ifndef i386
extern int sscanf();
#endif
extern int fscanf();
extern int fflush();
extern int fclose();
#ifndef i386
extern char *sprintf();
#endif
extern int fseek();

/* defines that should be in <sys/socket.h> */
extern int accept();
extern int bind();
extern int connect();
extern int getsockopt();
extern int setsockopt();
extern int listen();
extern int socket();

/* defines that should be in <sys/time.h> */
extern long time();
extern int gettimeofday();

/* defines that should be in <sys/errno.h> */
extern void perror();

/* defines that should be in <strings.h> */
extern char *strstr();
extern char *strdup();

/* other declarations */
extern int ioctl();
/****************************************/
#ifndef  __cplusplus
extern int select();
#else
extern "C" int select();
#endif
/****************************************/
extern int write();
extern int read();
extern int close();
extern int open();

#endif	/* __STDC__ */

#endif	/* not _LIBC_H_ */
#endif /* !VMS */

