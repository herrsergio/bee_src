
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




#ifndef LIB_MATH_LOADED
#define LIB_MATH_LOADED

typedef struct {
  float x,y;
} FPOINT_TYPE, *FPOINT_PTR;

typedef struct {
  int x,y;
} IPOINT_TYPE, *IPOINT_PTR;

#define FPointForm "{float,float}"
#define IPointForm "{int,int}"


/*
  ; The line passing through pt1 and pt2 is defined as 
  ;     " a * x + b * y + c = 0 "
  */

typedef struct {
  FPOINT_TYPE  pt1, pt2;
  double  len;				/* length */
  double  a,b,c;
} LINE_TYPE, *LINE_PTR;


double _0_to_2pi(double deg);
double  _pi_to_pi(double deg);
double _0_to_360(double deg);
double _180_to_180(double deg);
double floatMod(double num1, double num2);

#endif

