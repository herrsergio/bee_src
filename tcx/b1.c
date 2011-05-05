
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
* PROJECT: Carnegie Mellon Erebus Project
*          Task Control Architecture (TCX)
*
* PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
* (c) Copyright 1993 Christopher Fedor. All rights reserved.
* 
* MODULE: b1
*
* FILE: b1.c
*
* ABSTRACT:
* 
* This module is a test case for TCX.  It merely recieves data
* structures from one module to the next.
*
* REVISION HISTORY:
*
* $Log: b1.c,v $
* Revision 1.2  1996/12/03 05:38:34  thrun
* If TCXHOST is not se, the software will now assume "localhost" and
* won't terminate.
*
* Revision 1.1.1.1  1996/09/22 16:46:00  rhino
* General reorganization of the directories/repository, fusion with the
* RWI software.
*
* Revision 1.3  1994/10/23 08:51:00  tyson
* fixed (I hope) i386 structure indexing.
*
 * Revision 1.2  1994/10/22  18:47:21  tyson
 * VMS version. Fixed structure indexing (computation of the offsets
 * in a struct). Added signal handlers to a1, b1 tcxServer.
 *
 * Revision 1.1.1.1  1994/03/31  08:35:04  rhino
 *
 * Revision 1.1.1.1  1994/03/17  14:22:28  rhino
 *
 * Revision 1.1.1.1  1994/03/08  15:29:29  rhino
 * test
 *
 * Revision 1.4  1993/03/12  21:48:11  fedor
 * Completed first port to vxworks.
 * Still problems with cleaning up sockets after exit and restarting.
 *
 * Revision 1.3  1993/03/12  20:57:57  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.2  1993/03/03  20:18:34  fedor
 * Added tcxNRecvMsgE start of new message interface.
 * Added tcxRegisterCloseHnd - generic all module close.
 * Added tcxCurrentModuleName - give module name - change to pass NULL to
 * tcxModuleName?
 *
 * Revision 1.1  1993/03/02  20:43:15  fedor
 * Adding test cases at top level - remove old from Attic
 *
 * Revision 1.7  1992/11/13  14:48:28  fedor
 * Update test directory with current a & b examples
 *
 * Revision 1.6  1992/07/30  18:38:51  fedor
 * Ignore multiple calls to tcxInitialize.
 * Test cases for passing pointer to data to send and recv calls within procedures
 *
 * Revision 1.2  1992/07/03  11:16:04  fedor
 * New module definition and routines
 *
 * Revision 1.1  1992/06/29  19:05:50  darby
 * 29-Jun-92, Chad Darby, Field Robotics Center, CMU
 * Moved these files to separate test directory
 *
* Revision 1.2  1992/06/29  17:34:35  darby
*
* 29-Jun-92  Chad Darby, Field Robotics Center, CMU
* Created
*
* NOTES:
*  none
*
******************************************************************************/

#ifdef VMS
#include "vms.h" 

struct timeval {
        long    tv_sec;         /* seconds */
        long    tv_usec;        /* and microseconds */
};

#endif

#ifdef VXWORKS
#include "/usr/vxworks/vx5.0.2b/h/vxWorks.h"
#include "/usr/vxworks/vx5.0.2b/h/stdioLib.h"
#else
#include "stdio.h" 
#include <sys/types.h>
#include <sys/time.h>
#endif

#include <signal.h>
#include "tcx.h"

#include "sampleTest.h"

#ifndef VMS        
#ifndef VXWORKS
extern char *getenv(char *);
#endif
#endif

TCX_MODULE_PTR moduleD1G;


void SelfPtrHnd(module, id, ref, data)
TCX_MODULE_PTR module;
int id, ref;
SelfPtr data;
{
  /*
  printf("Start: SelfpointerStuff\n");
  printf ("px: %d\n", &x);
  printf ("x: %d\n", x);
  printf("End: SelfpointerStuff\n");
  */
}

void SelfQueryHnd(module, id, ref, x)
TCX_MODULE_PTR module;
int id, ref;
int *x;
{
  printf("SelfQueryHnd: Start.\n");

  printf("x = %d\n", *x);

  printf("SelfQueryHnd: Done.\n");
}

void SampleHnd(module, id, ref, Sample)
TCX_MODULE_PTR module;
int id, ref;
SamplePtr Sample;
{

  printf("SampleHnd: Start.\n");

  printf("continue with SampleHnd ...\n");

  if (Sample) {
    printf("Sample: %s\n", Sample->s);

    free(Sample->s);
    free(Sample);
  }
  else {
    printf("Sample: NULL\n");
    
  }
  
  printf("SampleHnd: Done.\n");
}

void GoalHnd(module, id, ref, Sample)
TCX_MODULE_PTR module;
int id, ref;
SamplePtr Sample;
{
  int a, b;

  printf("GoalHnd: Start\n");

  printf("Message: %s\n", tcxMessageName(id));

  printf("press return\n");
  getchar();
/*
  a = 42;
  tcaQuery("SelfQueryMsg", &a, &b);

  printf("b = %d\n", b);
*/
  printf("GoalHnd: Done.\n");
}

recvTestData(p)
PolyType **p;
{
  tcxRecvMsg("PolyMsg", NULL, p, NULL);
}


void FloatHnd(ref, f_value)
TCX_REF_PTR ref;
float *f_value;
{
  printf("FloatHnd: Start.\n");
  printf("ins: %d\n", tcxRefIns(ref));
  printf("f_value: %g\n", *f_value);
  printf("FloatHnd: Done.\n\n");
}
 

void DoubleHnd(ref, d_value)
TCX_REF_PTR ref;
double *d_value;
{
  printf("DoubleHnd: Start.\n");
  printf("ins: %d\n", tcxRefIns(ref));
  printf("d_value: %g\n", *d_value);
  printf("DoubleHnd: Done.\n\n");
}

void PolyHnd(ref, Poly)
TCX_REF_PTR ref;
PolyPtr Poly;
{
  int i;

  PolyType *p;

  char sampleChar;

  printf("PolyHnd: Start.\n");

  printf("ins: %d\n", tcxRefIns(ref));

  printf("Poly->silly: %d\n", Poly->silly);
  printf("Poly->n: %d\n", Poly->n);
  for(i=0;i < 2;i++) {
    printf("Poly->x[%d]: %g\n", i, Poly->x[i]);
    printf("Poly->y[%d]: %g\n", i, Poly->y[i]);
  }

#if 0
  recvTestData(&p);

  printf("\n");
  printf("recvTestData\n");
  printf("\n");

  printf("Poly->silly: %d\n", Poly->silly);
  printf("Poly->n: %d\n", Poly->n);
  for(i=0;i < 2;i++) {
    printf("Poly->x[%d]: %g\n", i, Poly->x[i]);
    printf("Poly->y[%d]: %g\n", i, Poly->y[i]);
  }
#endif

  printf("PolyHnd: Done.\n\n");
}

/*************/

void charMsgHnd(ref, sampleChar)
TCX_REF_PTR ref;
char *sampleChar;
{
  char *tcxMachine = NULL;

  printf("sampleCharHnd: Start.\n");

  printf("ins: %d\n", tcxRefIns(ref));

  printf("sampleChar: %c\n", *sampleChar);
  
#if 0
  /***************************************/
  printf("close all - press return\n");
  getchar();

  tcxCloseAll();
  Global->tcxInitFlagG = 1;
  tcxMachine = getenv("TCXHOST");

  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize("b1", tcxMachine);

  printf("blah yum yum yum\n");
  getchar();

  moduleD1G = tcxConnectModule("d1");

  printf("charMsg -> d1: press return\n");
  getchar();

  if (tcxTestActiveModule(moduleD1G)) {
    tcxSendMsg(moduleD1G, "charMsg", sampleChar);
  } else {
    printf("trying to connect to d1\n");
    moduleD1G = tcxConnectModule("d1");
    tcxSendMsg(moduleD1G, "charMsg", sampleChar);
  }

  printf("press return.\n");
  getchar();
  /***************************************/
#endif

  printf("sampleCharHnd: End.\n\n");
}

/*************/

void ucmatMapHnd(ref, ucmatMap)
TCX_REF_PTR ref;
ucmat *ucmatMap;
{
  printf("ucmatMap: Start.\n");

  printf("%x\n", ucmatMap->el[1][5]);
  printf("%x\n", ucmatMap->el[1][6]);
  printf("%x\n", ucmatMap->el[1][7]);
  printf("%x\n", ucmatMap->el[2][5]);
  printf("%x\n", ucmatMap->el[2][6]);
  printf("%x\n", ucmatMap->el[2][7]);

  printf("ucmatMap: End.\n\n");
}

/**********/

void cmatMapHnd(ref, cmatMap)
TCX_REF_PTR ref;
cmat *cmatMap;
{

  printf("cmatMap: Start.\n");

  printf("%c\n", cmatMap->el[1][5]);
  printf("%c\n", cmatMap->el[1][6]);
  printf("%c\n", cmatMap->el[1][7]);
  printf("%c\n", cmatMap->el[2][5]);
  printf("%c\n", cmatMap->el[2][6]);
  printf("%c\n", cmatMap->el[2][7]);

  printf("cmatMap: End.\n\n");
}

/***********/

void smatMapHnd(TCX_REF_PTR ref, smat *smatMap)
{
  int i;

  printf("smatMap: Start.\n");

  printf("%d\n", smatMap->el[1][5]);
  printf("%d\n", smatMap->el[1][6]);
  printf("%d\n", smatMap->el[1][7]);
  printf("%d\n", smatMap->el[2][5]);
  printf("%d\n", smatMap->el[2][6]);
  printf("%d\n", smatMap->el[2][7]);

  printf("smatMap: End.\n\n");
}

void imatMapHnd(TCX_REF_PTR ref, imat *imatMap)
{
  printf("imatMap: Start.\n");

  printf("%d\n", imatMap->el[1][5]);
  printf("%d\n", imatMap->el[1][6]);
  printf("%d\n", imatMap->el[1][7]);
  printf("%d\n", imatMap->el[2][5]);
  printf("%d\n", imatMap->el[2][6]);
  printf("%d\n", imatMap->el[2][7]);

  printf("imatMap: End.\n\n");
}

void lmatMapHnd(TCX_REF_PTR ref, lmat *lmatMap)
{
  printf("lmatMap: Start.\n");

  printf("%ld\n", lmatMap->el[1][5]);
  printf("%ld\n", lmatMap->el[1][6]);
  printf("%ld\n", lmatMap->el[1][7]);
  printf("%ld\n", lmatMap->el[2][5]);
  printf("%ld\n", lmatMap->el[2][6]);
  printf("%ld\n", lmatMap->el[2][7]);

  printf("lmatMap: End.\n\n");
}

void fmatMapHnd(TCX_REF_PTR ref, fmat *fmatMap)
{
  printf("fmatMap: Start.\n");

  printf("%f\n", fmatMap->el[1][5]);
  printf("%f\n", fmatMap->el[1][6]);
  printf("%f\n", fmatMap->el[1][7]);
  printf("%f\n", fmatMap->el[2][5]);
  printf("%f\n", fmatMap->el[2][6]);
  printf("%f\n", fmatMap->el[2][7]);

  printf("fmatMap: End.\n\n");
}

void dmatMapHnd(TCX_REF_PTR ref, dmat *dmatMap)
{

  printf("dmatMap: Start.\n");

  printf("%f\n", dmatMap->el[1][5]);
  printf("%f\n", dmatMap->el[1][6]);
  printf("%f\n", dmatMap->el[1][7]);
  printf("%f\n", dmatMap->el[2][5]);
  printf("%f\n", dmatMap->el[2][6]);
  printf("%f\n", dmatMap->el[2][7]);

  printf("dmatMap: End.\n\n");
}

/*************/

void LinkHnd(ref, s2)
TCX_REF_PTR ref;
s2Ptr s2;
{
  s2Ptr tmp;
  printf("LinkHnd: Start.\n");

  tmp = s2;
  while(tmp) {
    printf("s2->a: %d\n", tmp->a);
    printf("s2->b: %d\n", tmp->b);
    printf("s2->s: %s\n", tmp->s);
    printf("\n");
    /* should free this list when we are done with it. */
    tmp = tmp->next;
  }
  printf("LinkHnd: End.\n\n");

}

/***********/

void FixedArrayHnd(TCX_REF_PTR ref, two_d_array fixed_array)
{
  int       i, j;

  printf("FixedArrayHnd: Start.\n");

  for(i=0; i<FIXED_ARRAY_DIM1; i++) {
    for(j=0; j<FIXED_ARRAY_DIM2; j++)
      printf("%d ", fixed_array[i][j]);
    printf("\n");
  }

   printf("FixedArrayHnd: End.\n");

}

void VarArrayHnd(TCX_REF_PTR ref, var_two_d_array *var_array)
{
  ReplyType ReplyData;
  int       i, j, accessor;

  printf("VarArrayHnd: Start.\n");

  for(i=0; i<var_array->dim1; i++){
    for(j=0; j<var_array->dim2; j++) {
      /* Can't do multiple subscripts on variable length arrays */
      accessor = i + j*var_array->dim1;
      printf("%s ", var_array->elements[accessor]);
    }
    printf("\n");
  }

  printf("VarArrayHnd: End.\n");
}

#define set_point(point, x1, y1, z1) \
{ (point).x = x1; (point).y = y1; (point).z = z1;}

#define set_two_points(two_points, Leg_No, Name, x1, y1, z1, x2, y2, z2) \
{ (two_points).leg_no = Leg_No; (two_points).name = Name; \
  set_point((two_points).pt1, x1, y1, z1); \
  set_point((two_points).pt2, x2, y2, z2);}

void print_point (apoint) point *apoint;
{ if (apoint) printf("Point: %f, %f, %f\n", apoint->x, apoint->y, apoint->z);
  else printf("No Point\n");}

void print_two_points (twopoints) two_points *twopoints;
{ 
  if (twopoints) {
    printf("Two Points: %d, %s\n", twopoints->leg_no, twopoints->name);
    printf("     "); print_point(&(twopoints->pt1));
    printf("     "); print_point(&(twopoints->pt2));
  }
  else printf("No Points\n");
}

void print_complex_points (complexpoints) complex_points *complexpoints;
{ int i;

  printf("Complex Point:\n");
  for(i=0; i<6; i++) {
    printf("%d. ", i); 
    print_point(&(complexpoints->points[i]));
  }
  if (complexpoints->length)
    for(i=0; i<complexpoints->length; i++) {
      printf("%d. ", i); 
      print_two_points(&(complexpoints->more_points[i]));
    }
  else printf("No Points\n");
  print_point(complexpoints->pt_ptr);
}

PointsHnd(module, id, ref, Points)
TCX_MODULE_PTR module;
int id, ref;
complex_points *Points;
{
  two_points reply_point;

  print_complex_points(Points);

  if (Points->more_points) reply_point = Points->more_points[1];
  else set_two_points(reply_point, -1, "Null Reply", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  printf("tcxReplyData - missing\n");
/*  tcxReplyData(module, id, ref, &reply_point);*/

}

void structTest1Hnd(ref, t)
TCX_REF_PTR ref;
STRUCT_TEST1_PTR t;
{
  int r;

  r = 1;

  printf("structTest1Hnd: start\n");

  printf("test1->x: %d\n", t->x);
  printf("test1->a: %c\n", t->a);
  printf("test1->y: %f\n", t->y);

  printf("tcxReplyData - missing\n");
/*  tcxReplyData(module, id, ref, &r);*/
  printf("structTest1Hnd: end\n");
}

void structTest2Hnd(ref, t)
TCX_REF_PTR ref;
STRUCT_TEST2_PTR t;
{
  int r;

  r = 2;

  printf("structTest2Hnd: start\n");

  printf ("test2->A: %c\n", t->A);
  printf ("test2->x: %d\n", t->x);
  printf ("test2->B: %c\n", t->B);
  printf ("test2->n: %d\n", t->n);
  printf ("test2->C: %c\n", t->C);
  printf ("test2->y: %f\n", t->y);
  printf ("test2->a: %c\n", t->a);
  printf ("test2->b: %c\n", t->b);

  printf("tcxReplyData - missing\n");
/*  tcxReplyData(module, id, ref, &r);*/
  printf("structTest2Hnd: end\n");
}

void structTest3Hnd(ref, t)
TCX_REF_PTR ref;
STRUCT_TEST3_PTR t;
{
  int r;

  r = 3;

  printf("structTest3Hnd: start\n");

  printf("test3->x: %d\n", t->x);
  printf("test3->a: %c\n", t->a);

  printf("tcxReplyData - missing\n");
/*  tcxReplyData(module, id, ref, &r);*/
  printf("structTest3Hnd: end\n");
}

void structTest4Hnd(ref, t)
TCX_REF_PTR ref;
STRUCT_TEST4_PTR t;
{
  int r;

  r = 4;

  printf("structTest4Hnd: start\n");

  printf("test4->w: %d\n", t->w);
  printf("test4->t.x: %d\n", t->t.x);
  printf("test4->t.a: %c\n", t->t.a);
  printf("test4->z: %f\n", t->z);

  printf("tcxReplyData - missing\n");
/*  tcxReplyData(module, id, ref, &r);*/
  printf("structTest4Hnd: end\n");
}


void regInit()
{
  /* Register the messages */
  printf ("I'm Initing Messages\n\n");

  tcxRegisterMessages(messageArray,
		      sizeof(messageArray)/sizeof(TCX_REG_MSG_TYPE));
}

void queryHnd(ref, c)
TCX_REF_PTR ref;
char *c;
{
  int i;

  printf("queryHnd\n");
  printf("c: %c\n", *c);
  
  i = 17;
  tcxReply(ref, "replyInt", &i);
}
	 
/**************/

void stringHnd(ref, s)
TCX_REF_PTR ref;
char **s;
{
  printf("s: %s\n", *s);
}


/**************/

void specialExit()
{
  printf("weee special exit.\n");
  exit(1);
}


TCX_REG_HND_TYPE handlersArray[] = {
  {"FloatMsg", "FloatHnd", FloatHnd, TCX_RECV_ALL, NULL},
  {"DoubleMsg", "DoubleHnd", DoubleHnd, TCX_RECV_ALL, NULL},
  {"PolyMsg", "PolyHnd", PolyHnd, TCX_RECV_NONE, NULL},
  {"charMsg", "charMsgHnd", charMsgHnd, TCX_RECV_ALL, NULL},
  {"ucmatMsg", "ucmatMapHnd", ucmatMapHnd, TCX_RECV_ALL, NULL},
  {"cmatMsg", "cmatMapHnd", cmatMapHnd, TCX_RECV_ALL, NULL},
  {"queryChar", "queryHnd", queryHnd, TCX_RECV_ALL, NULL},
  {"smatMsg", "smatMapHnd", smatMapHnd, TCX_RECV_ALL, NULL},
  {"imatMsg", "imatMapHnd", imatMapHnd, TCX_RECV_ALL, NULL},
  {"lmatMsg", "lmatMapHnd", lmatMapHnd, TCX_RECV_ALL, NULL},
  {"fmatMsg", "fmatMapHnd", fmatMapHnd, TCX_RECV_ALL, NULL},
  {"dmatMsg", "dmatMapHnd", dmatMapHnd, TCX_RECV_ALL, NULL},
  {"LinkTestMsg", "LinkHnd", LinkHnd, TCX_RECV_ALL, NULL},
  {"FixedArrayMsg", "FixedArrayHnd", FixedArrayHnd, TCX_RECV_ALL, NULL},
  {"VarArrayMsg", "VarArrayHnd", VarArrayHnd, TCX_RECV_ALL, NULL},
  {"StructTest1Msg", "structTest1Hnd", structTest1Hnd, TCX_RECV_ALL, NULL},
  {"StructTest2Msg", "structTest2Hnd", structTest2Hnd, TCX_RECV_ALL, NULL},
  {"StructTest3Msg", "structTest3Hnd", structTest3Hnd, TCX_RECV_ALL, NULL},
  {"StructTest4Msg", "structTest4Hnd", structTest4Hnd, TCX_RECV_ALL, NULL},
  {"stringMsg", "stringHnd", stringHnd, TCX_RECV_ALL, NULL}
};


/***

***/

void openHnd(char *name)
{
  printf("open: %s\n", name);
}

void closeHnd(char *name)
{
  printf("close: %s\n", name);
}

void closeHnd2(char *name)
{
  printf("close2: %s\n", name);
}

void Quit(void) {
    printf ("Exiting on SIGINT ...\n");
    exit(0);
}

#ifdef VXWORKS
b1main(host)
char *host;
#else
main()
#endif
{
  PolyType poly;
  TCX_FMT_PTR fmt;

  TCX_MODULE_PTR module;

  int ret, inst;
  char *msgName;
  void *data;
  char *tcxMachine = NULL;

#ifndef VXWORKS
  struct timeval noTime;
#endif

  signal(SIGINT, (void *)Quit);

  printf("Connect ...\n");

  /* register open/close handlers */
  /* tcxRegisterConnectHnd("a1", openHnd, closeHnd); */
  tcxRegisterConnectHnd("d1", openHnd, closeHnd);

  tcxRegisterCloseHnd(closeHnd2);
  
#ifdef VXWORKS
  tcxInitialize("b1", host);
#else
  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }
  tcxInitialize("b1", tcxMachine);
#endif

  printf("my name: %s\n", tcxCurrentModuleName());

  printf("looking for 'd1' ...");
  moduleD1G = tcxConnectOptional("d1");

  if (moduleD1G) {
    printf("found\n");
  } else {
    printf("not found\n");
  }

  /***
  printf("looking for 'a1' ... \n");
  tcxConnectModule("a1");
  printf("done.\n");

  printf("press return\n");
  getchar();
  ****/  

  tcxRegisterMessages(messageArray, 
		      sizeof(messageArray)/sizeof(TCX_REG_MSG_TYPE));

  tcxRegisterHandlers(handlersArray,
		      sizeof(handlersArray)/sizeof(TCX_REG_HND_TYPE));

/*
  tcxRecvMsg("PolyMsg", NULL, &poly, NULL);

  PolyHnd(NULL, &poly);

  exit(1);
*/

  tcxRecvLoop(NULL); 

/*********************
  while (1) {
    module = NULL;
    msgName = NULL;
    data = NULL;
    inst = 0;
    ret = tcxNRecvMsgE(&module, &msgName, &data, NULL, &inst, ALL_HND);
    printf("module: %s\n", tcxModuleName(module));
    printf("msg: %s: inst: %d: ret: %d\n", msgName, inst, ret);
    printf("\n");
    if (msgName) {
      tcxFree(msgName, data);
    }
  }
*******************/
}









