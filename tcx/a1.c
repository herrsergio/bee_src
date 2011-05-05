
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
* MODULE: a1
*
* FILE: a1.c
*
* ABSTRACT:
* 
* This module is a test case for TCX.  It merely sends data
* structures from one module to the next.
*
* REVISION HISTORY:
*
* $Log: a1.c,v $
* Revision 1.2  1996/12/03 05:38:33  thrun
* If TCXHOST is not se, the software will now assume "localhost" and
* won't terminate.
*
* Revision 1.1.1.1  1996/09/22 16:46:00  rhino
* General reorganization of the directories/repository, fusion with the
* RWI software.
*
* Revision 1.4  1994/10/23 08:50:53  tyson
* fixed (I hope) i386 structure indexing.
*
 * Revision 1.3  1994/10/22  18:47:19  tyson
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
 * Revision 1.4  1993/03/12  21:48:08  fedor
 * Completed first port to vxworks.
 * Still problems with cleaning up sockets after exit and restarting.
 *
 * Revision 1.3  1993/03/12  20:57:51  fedor
 * Updated test cases and hid global variables for vxworks
 *
 * Revision 1.2  1993/03/03  20:18:30  fedor
 * Added tcxNRecvMsgE start of new message interface.
 * Added tcxRegisterCloseHnd - generic all module close.
 * Added tcxCurrentModuleName - give module name - change to pass NULL to
 * tcxModuleName?
 *
 * Revision 1.1  1993/03/02  20:43:11  fedor
 * Adding test cases at top level - remove old from Attic
 *
 * Revision 1.7  1992/11/13  14:48:26  fedor
 * Update test directory with current a & b examples
 *
 * Revision 1.6  1992/07/30  18:38:48  fedor
 * Ignore multiple calls to tcxInitialize.
 * Test cases for passing pointer to data to send and recv calls within procedures
 *
 * Revision 1.2  1992/07/03  11:15:58  fedor
 * New module definition and routines
 *
 * Revision 1.1  1992/06/29  19:05:43  darby
 * 29-Jun-92, Chad Darby, Field Robotics Center, CMU
 * Moved these files to separate test directory
 *
* Revision 1.2  1992/06/29  17:34:35  darby
*
* 29-Jun-92  Chad Darby, Field Robotics Center, CMU
* Created
*
* NOTES:
*   none
*
******************************************************************************/

#ifdef VMS
#include "vms.h"
#endif

#ifdef VXWORKS
#include "/usr/vxworks/vx5.0.2b/h/vxWorks.h"
#include "/usr/vxworks/vx5.0.2b/h/stdioLib.h"
#else
#include "stdio.h" 
#endif

#include <signal.h>
#include "tcx.h"

#include "sampleTest.h"

#ifndef VXWORKS
#ifndef VMS
extern char *getenv(char *);
#endif
#endif

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


sendTestData(a)
PolyType *a;
{
 /* tcxSendMsg(NULL, "PolyMsg", a);*/

/*  tcxSendMsg(NULL, "PolyMsg", a);*/
}

void openHnd(char *name)
{
  printf("open: %s\n", name);
}

void closeHnd(char *name)
{
  printf("close: %s\n", name);
}

void Quit(void) {
    printf ("Exiting on SIGINT ...\n");
    exit(0);
}

#ifdef VXWORKS
a1main(host)
char *host;
#else
main()
#endif
{ 
  char *tcxMachine = NULL;
  int             i, j, accessor, ref;
  ReplyType       ReplyData;
  SampleType      Sample;
  PolyType        Poly;
  s2Ptr           s2;
  two_d_array     fixed_array;
  var_two_d_array var_array;
  complex_points the_points;
  two_points     the_answer;

  ucmat ucmatMap;
  cmat cmatMap;
  smat smatMap;
  imat imatMap;
  lmat lmatMap;
  fmat fmatMap;
  dmat dmatMap;

  char sampleChar;
  TCX_MODULE_PTR module;
  int error;

  int a;
  int *pa;

  float f_value = 3.14159265358979323846e-14;
  double d_value = 3.14159265358979323846e-14;

  STRUCT_TEST1_TYPE test1;
  STRUCT_TEST2_TYPE test2;
  STRUCT_TEST3_TYPE test3;
  STRUCT_TEST4_TYPE test4;

  char *string = "abcde";

  signal (SIGINT, (void *)Quit);

  printf("Connect ...\n");

  /* register open/close handlers */
  tcxRegisterConnectHnd("b1", openHnd, closeHnd);

#ifdef VXWORKS
  tcxInitialize("a1", host);
#else
  tcxMachine = getenv("TCXHOST");   

  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }
  tcxInitialize("a1", tcxMachine);
#endif

  /* init the register names */

  tcxRegisterMessages(messageArray, 
		      sizeof(messageArray)/sizeof(TCX_REG_MSG_TYPE));

  /* tcxRegMsgSend(mArray, sizeof(mArray)/sizeof(TCX_REG2_MSG_TYPE));*/

  /* tcxRegMsgRecv */

  printf("press return to connect to 'b1'\n");
  getchar();

  printf("Connecting to 'b1' ...\n");
  module = tcxConnectModule("b1");

  printf ("Module is: %s\n\n", tcxModuleName(module));

  printf("connected.\n");

  printf("send float: %g\n", f_value);
  printf("send double: %g\n", d_value);
  ref = tcxSendMsg(module, "FloatMsg", &f_value);
  ref = tcxSendMsg(module, "DoubleMsg", &d_value);
  printf("done.\n");
  printf("press return\n");
  getchar();

  i = 42;

  Sample.x = 17;
  Sample.s = (StringType) malloc(sizeof(char)*4);
  Sample.s[0] = 'a';
  Sample.s[1] = 'b';
  Sample.s[2] = 'c';
  Sample.s[3] = '\0';

  /* printf("send string: %s\n", Sample.s);*/
  printf("send string: %s\n", string);

  /* ref = tcxSendMsg(module, "stringMsg", &(Sample.s));  */
  ref = tcxSendMsg(module, "stringMsg", &string); 

  printf("done.\n");
  printf("press return\n");
  getchar();

  Poly.silly = 17;

  Poly.x  = (double *)malloc(sizeof(double)*2);
  Poly.y  = (double *)malloc(sizeof(double)*2);

  Poly.x[0] = 1.0;
  Poly.x[1] = 2.0;

  Poly.y[0] = 3.0;
  Poly.y[1] = 4.0;

  Poly.n = 2;

  printf ("PolyHnd: Start.\n");

  printf("Poly->silly: %d\n", Poly.silly);
  printf("Poly->n: %d\n", Poly.n);
  for(i=0;i < 2;i++) {
    printf("Poly->x[%d]: %g\n", i, Poly.x[i]);
    printf("Poly->y[%d]: %g\n", i, Poly.y[i]);
  }
  printf("PolyHnd: Done.\n\n");

  ref = tcxSendMsg(module, "PolyMsg", &Poly);
  /* sendTestData(&Poly);*/

  printf("ref: %d\n", ref);

  printf("press return.\n");
  getchar();

  sampleChar = 'X';
  printf("sampleCharHnd: Start.\n");
  printf("sampleChar: %c\n", sampleChar);
  printf("sampleCharHnd: End.\n\n");

  ref = tcxSendMsg(module, "charMsg", &sampleChar);

/*  ref = tcxSendDoubleMsg(module, "PolyMsg", &Poly, "charMsg", &sampleChar);*/

  printf("ref: %d\n", ref);
  printf("press return.\n");
  getchar();

  /*****
  tcxQuery(module, "queryChar", &sampleChar, "replyInt", &i);
  printf("query: i: %d\n", i);


  printf("press return.\n");
  getchar();
  *****/

  ucmatMap = newucmat2(1, 2, 5, 7, &error);
  ucmatMap.el[1][5] = 506;
  ucmatMap.el[1][6] = 507;
  ucmatMap.el[1][7] = 508;
  ucmatMap.el[2][5] = 509;
  ucmatMap.el[2][6] = 510;
  ucmatMap.el[2][7] = 511;

  printf ("ucmatMap: Start.\n");

  printf("%x\n", ucmatMap.el[1][5]);
  printf("%x\n", ucmatMap.el[1][6]);
  printf("%x\n", ucmatMap.el[1][7]);
  printf("%x\n", ucmatMap.el[2][5]);
  printf("%x\n", ucmatMap.el[2][6]);
  printf("%x\n", ucmatMap.el[2][7]);

  printf ("ucmatMap: End\n\n");

  ref = tcxSendMsg(module, "ucmatMsg", &ucmatMap);

  printf("ref: %d\n", ref);
  printf("press return.\n");
  getchar();

  cmatMap = newcmat2(1, 2, 5, 7, &error);
  cmatMap.el[1][5] = 'a';
  cmatMap.el[1][6] = 'b';
  cmatMap.el[1][7] = 'c';
  cmatMap.el[2][5] = 'd';
  cmatMap.el[2][6] = 'e';
  cmatMap.el[2][7] = 'f';

  printf("cmatMap: Start.\n");

  printf("%c\n", cmatMap.el[1][5]);
  printf("%c\n", cmatMap.el[1][6]);
  printf("%c\n", cmatMap.el[1][7]);
  printf("%c\n", cmatMap.el[2][5]);
  printf("%c\n", cmatMap.el[2][6]);
  printf("%c\n", cmatMap.el[2][7]);

  printf("cmatMap: End.\n\n");

  ref = tcxSendMsg(module, "cmatMsg", &cmatMap);

  printf("ref: %d\n", ref);
  printf("press return.\n");
  getchar();
  
  smatMap = newsmat2(1, 2, 5, 7, &error);
  smatMap.el[1][5] = 1;
  smatMap.el[1][6] = 2;
  smatMap.el[1][7] = 3;
  smatMap.el[2][5] = 4;
  smatMap.el[2][6] = 5;
  smatMap.el[2][7] = 6;

  printf("smatMap: Start.\n");

  printf("%d\n", smatMap.el[1][5]);
  printf("%d\n", smatMap.el[1][6]);
  printf("%d\n", smatMap.el[1][7]);
  printf("%d\n", smatMap.el[2][5]);
  printf("%d\n", smatMap.el[2][6]);
  printf("%d\n", smatMap.el[2][7]);

  printf("smatMap: End.\n\n");

  tcxSendMsg(module, "smatMsg", &smatMap);
/*
  tcxQuery(module, "smatMsg", &smatMap, "replyInt", &i);

  printf("i: %d\n", i);
*/

  printf("press return.\n");
  getchar();

  imatMap = newimat2(1, 2, 5, 7, &error);
  imatMap.el[1][5] = 1;
  imatMap.el[1][6] = 2;
  imatMap.el[1][7] = 3;
  imatMap.el[2][5] = 4;
  imatMap.el[2][6] = 5;
  imatMap.el[2][7] = 6;

  printf("imatMap: Start.\n");

  printf("%d\n", imatMap.el[1][5]);
  printf("%d\n", imatMap.el[1][6]);
  printf("%d\n", imatMap.el[1][7]);
  printf("%d\n", imatMap.el[2][5]);
  printf("%d\n", imatMap.el[2][6]);
  printf("%d\n", imatMap.el[2][7]);

  printf("imatMap: End.\n\n");

  ref = tcxSendMsg(module, "imatMsg", &imatMap);
  printf("ref: %d\n", ref);

  printf("press return.\n");
  getchar();

  lmatMap = newlmat2(1, 2, 5, 7, &error);
  lmatMap.el[1][5] = 2147483640;
  lmatMap.el[1][6] = 2147483641;
  lmatMap.el[1][7] = 2147483642;
  lmatMap.el[2][5] = 2147483643;
  lmatMap.el[2][6] = 2147483644;
  lmatMap.el[2][7] = 2147483645;

  printf("lmatMap: Start.\n");

  printf("%ld\n", lmatMap.el[1][5]);
  printf("%ld\n", lmatMap.el[1][6]);
  printf("%ld\n", lmatMap.el[1][7]);
  printf("%ld\n", lmatMap.el[2][5]);
  printf("%ld\n", lmatMap.el[2][6]);
  printf("%ld\n", lmatMap.el[2][7]);

  printf("lmatMap: End.\n\n");

  ref = tcxSendMsg(module, "lmatMsg", &lmatMap);
  printf("ref: %d\n", ref);

  printf("press return.\n");
  getchar();

  fmatMap = newfmat2(1, 2, 5, 7, &error);
  fmatMap.el[1][5] = 1.1;
  fmatMap.el[1][6] = 2.2;
  fmatMap.el[1][7] = 3.3;
  fmatMap.el[2][5] = 4.4;
  fmatMap.el[2][6] = 5.5;
  fmatMap.el[2][7] = 6.6;

  printf("fmatMap: Start.\n");

  printf("%f\n", fmatMap.el[1][5]);
  printf("%f\n", fmatMap.el[1][6]);
  printf("%f\n", fmatMap.el[1][7]);
  printf("%f\n", fmatMap.el[2][5]);
  printf("%f\n", fmatMap.el[2][6]);
  printf("%f\n", fmatMap.el[2][7]);

  printf("fmatMap: End.\n\n");
  ref = tcxSendMsg(module, "fmatMsg", &fmatMap);
  printf("ref: %d\n", ref);

/****
  printf("press return.\n");
  getchar();
****/

  dmatMap = newdmat2(1, 2, 5, 7, &error);
  dmatMap.el[1][5] = 1.1;
  dmatMap.el[1][6] = 2.2;
  dmatMap.el[1][7] = 3.3;
  dmatMap.el[2][5] = 4.4;
  dmatMap.el[2][6] = 5.5;
  dmatMap.el[2][7] = 6.6;

  printf("dmatMap: Start.\n");

  printf("%f\n", dmatMap.el[1][5]);
  printf("%f\n", dmatMap.el[1][6]);
  printf("%f\n", dmatMap.el[1][7]);
  printf("%f\n", dmatMap.el[2][5]);
  printf("%f\n", dmatMap.el[2][6]);
  printf("%f\n", dmatMap.el[2][7]);

  printf("dmatMap: End.\n\n");

  ref = tcxSendMsg(module, "dmatMsg", &dmatMap);
  printf("ref: %d\n", ref);

/****
  printf("press return.\n");
  getchar();
****/

  s2 = (s2Type *)malloc(sizeof(s2Type));
  s2->a = 1;
  s2->b = 2;
  s2->s = (StringType) malloc(sizeof(char)*4);
  s2->s[0] = 'a';
  s2->s[1] = 'b';
  s2->s[2] = 'c';
  s2->s[3] = '\0';
  s2->next = (s2Type *)malloc(sizeof(s2Type));
  s2->next->a = 3;
  s2->next->b = 4;
  s2->next->s = (StringType) malloc(sizeof(char)*4);
  s2->next->s[0] = 'e';
  s2->next->s[1] = 'f';
  s2->next->s[2] = 'g';
  s2->next->s[3] = '\0';
  s2->next->next = NULL;

  printf("LinkHnd: Start.\n");

  printf("s2->a: %d\n", s2->a);
  printf("s2->b: %d\n", s2->b);
  printf("s2->s: %s\n", s2->s);
  printf("\n");
  printf("s2->a: %d\n", s2->next->a);
  printf("s2->b: %d\n", s2->next->b);
  printf("s2->s: %s\n", s2->next->s);
  printf("\n");

  printf("LinkHnd: End.\n\n");

  ref = tcxSendMsg(module, "LinkTestMsg", s2);
  printf("ref: %d\n", ref);
  printf("End: LinkTestMsg\n");

#if 0
  printf("Start: SelfpointerStuff\n");
  a = 1969;
  pa = &a;
  printf ("pa: %d\n", pa);
  printf ("a: %d\n", a);
  printf("End: SelfpointerStuff\n");
  tcxSendMsg(module, "SelfPtrMsg", s2);
#endif

  printf("Query: FixedArrayMsg %d\n", sizeof(fixed_array));

  for(i=0; i<FIXED_ARRAY_DIM1; i++) {
    for(j=0; j<FIXED_ARRAY_DIM2; j++) {
      fixed_array[i][j] = i+j;
      printf("%d ", fixed_array[i][j]);
    }
    printf("\n");
  }

  ref = tcxSendMsg(module, "FixedArrayMsg", fixed_array);
/***
  printf("ref: %d\n", ref);
  printf("press return.\n");
  getchar();
***/

  printf("Query: VarArrayMsg\n");

  var_array.dim1 = VAR_ARRAY_DIM1;
  var_array.dim2 = VAR_ARRAY_DIM2;
  var_array.elements = (StringType *)malloc(sizeof(StringType) * 
					    var_array.dim1 * var_array.dim2);
  for(i=0; i<var_array.dim1; i++){
    for(j=0; j<var_array.dim2; j++) {
      accessor = i + j*var_array.dim1;
      var_array.elements[accessor] = (StringType)malloc(sizeof(char)*10);
      sprintf(var_array.elements[accessor], "%d-%d\0", i, j);
      printf("%s ", var_array.elements[accessor]);
    }
    printf("\n");
  }


  ref = tcxSendMsg(module, "VarArrayMsg", &var_array);
/***
  printf("ref: %d\n", ref);
  printf("press return.\n");
  getchar();
***/

  /* darby: Must register points format */
#if 0
  printf("Query: Structured Formatters Msg\n");

  for(i=0;i<6;i++) set_point(the_points.points[i], i+0.1, i+0.5, i+0.9);

  the_points.length = 3;
  the_points.more_points = (two_points *)malloc(the_points.length *
						sizeof(two_points));
  for(i=0;i<the_points.length;i++) 
    set_two_points(the_points.more_points[i], i, "", 
		   i+0.1, i+0.2, i+0.3, i+1.1, i+1.2, i+1.3);
  the_points.more_points[0].name = "The First";
  the_points.more_points[1].name = "The Second";
  the_points.more_points[2].name = "The Third";

  the_points.pt_ptr = NULL;
#endif

#if 0
  the_points.length = 0;
  the_points.more_points = NULL;

  the_points.pt_ptr = (point *)malloc(sizeof(point));
  set_point(*the_points.pt_ptr, 1.23, 4.56, 7.89);
  print_complex_points(&the_points);

  tcxQuery(module, "StructuredFormatters", &the_points, &the_answer);
  print_two_points(&the_answer);
#endif

  test1.x = 1234567890;
  test1.a = 'a';
  test1.y = 12345.6789;
  
  printf ("test1.x: %d\n", test1.x);
  printf ("test1.a: %c\n", test1.a);
  printf ("test1.y: %f\n", test1.y);
  tcxSendMsg(module, "StructTest1Msg", &test1);
/**
  printf("press return.\n");
  getchar();
**/

  test2.A = 'A';
  test2.x = 1245678901;
  test2.B = 'B';
  test2.n = 13901;
  test2.C = 'C';
  test2.y = 1245678.901;
  test2.a = 'b';
  test2.b = 'w';
  printf ("test2.A: %c\n", test2.A);
  printf ("test2.x: %d\n", test2.x);
  printf ("test2.B: %c\n", test2.B);
  printf ("test2.n: %d\n", test2.n);
  printf ("test2.C: %c\n", test2.C);
  printf ("test2.y: %f\n", test2.y);
  printf ("test2.a: %c\n", test2.a);
  printf ("test2.b: %c\n", test2.b);
  tcxSendMsg(module, "StructTest2Msg", &test2);
/**
  printf("press return.\n");
  getchar();
**/

  test3.x = 1356789012;
  test3.a = 'c';
  printf ("test3.x: %d\n", test3.x);
  printf ("test3.a: %c\n", test3.a);
  tcxSendMsg(module, "StructTest3Msg", &test3);
/**
  printf("press return.\n");
  getchar();
**/

  test4.w = 1467890123;
  test4.t.x = 1578901234;
  test4.t.a = 'd';
  test4.z = 14678.90123;
  printf ("test4.w: %d\n", test4.w);
  printf ("test4.t.x: %d\n", test4.t.x);
  printf ("test4.t.a: %c\n", test4.t.a);
  printf ("test4.z: %f\n", test4.z);
  tcxSendMsg(module, "StructTest4Msg", &test4);
/**
  printf("press return.\n");
  getchar();
**/
}
