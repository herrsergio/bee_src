
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




#ifndef COMMON_LOADED
#define COMMON_LOADED

#ifdef __GNUC__
#define INLINE __inline__
#else
#define INLINE inline
#endif

#define EXTERN extern
#define PRIVATE static
#define PUBLIC

#ifdef  NULL
#undef  NULL
#endif
#define NULL  (void*) 0

#define BOOLEAN int
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#define CBOOLEAN char
#define CBOOL	 char
#define CTRUE (char)1
#define CFALSE (char)0
  
#define UCHAR unsigned char
#define USINT unsigned short int
#define SINT short int
  
#define NILPTR (char *)0

typedef void  *Pointer;

typedef float DEGREES;
typedef float RADIANS;
typedef float METERS;
typedef float CMS;
typedef float FEET;

typedef void (*VOID_FN1)(double);
typedef void (*VOID_FN2)(double,double);
typedef void (*VOID_FN3)(double,double,double);
typedef void (*VOID_FN4)(double,double,double,double);

#define DEGREES_FORMAT "float" /* for TCA usage */
#define RADIANS_FORMAT "float" /* for TCA usage */
#define FEET_FORMAT    "float" /* for TCA usage */
#define METERS_FORMAT  "float" /* for TCA usage */
#define CMS_FORMAT     "float" /* for TCA usage */

#define Abs(x)	  ((x) >= 0 ? (x) : -(x))
#define Max(x,y)  ((x) > (y) ? (x) : (y))
#define Min(x,y)  ((x) > (y) ? (y) : (x))
#define Sqr(x)    ((x) * (x))

#define ABS(x)	  ((x) >= 0 ? (x) : -(x))
#define MAX(x,y)  ((x) > (y) ? (x) : (y))
#define MIN(x,y)  ((x) > (y) ? (y) : (x))
#define SQR(x)    ((x) * (x))

#define NEAR(x1,x2,eps) (ABS((x1)-(x2))<=(eps))


#define IRINT(x)  ((int) rint(x))

#ifndef PI
#define PI 3.1415926535897932384626433
#endif
#define SQRT2 1.4142135
#define TWO_PI (2 * PI)
#define Rad_to_Deg(r)   ((r) * 180.0 / PI)
#define Deg_to_Rad(d)   ((d) * PI / 180.0)

#define RAD_TO_DEG(r)   ((r) * 57.29578)
#define DEG_TO_RAD(d)   ((d) * 0.017453293)

#define FT2M (12.0/39.37)
#define FEET_TO_METERS(ft) ((ft)*FT2M)
#define METERS_TO_FEET(ms) ((ms)/FT2M)

#define INCHES_TO_CMS(in) (100.0*FEET_TO_METERS((in)/12.0))
#define CMS_TO_INCHES(cm) (12.0*METERS_TO_FEET((cm)/100.0))

#define DEFAULT_LINE_LENGTH 80

#endif






