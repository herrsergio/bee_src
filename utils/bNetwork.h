
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
#ifndef NETWORK_H
#define NETWORK_H


#ifdef __cplusplus
extern "C" {
#endif


/* This currently assumes that a double is twice the length of a long */
/* and that everything is running on an Intel processor compiled with */
/* gcc.  This will have to be changed for other platorms with ugly    */
/* #ifdef statements at a later date.                                 */

#include <sys/types.h>
#include <netinet/in.h>

#define B_LONGS_PER_DOUBLE 2

typedef union {
  double number;
  unsigned long array[B_LONGS_PER_DOUBLE];
} bDoubleUnion;

typedef union {
  float number;
  unsigned long long_number;
} bFloatUnion;


/* Maybe we should be using one global static union here, since it would */
/* remove the overhead of allocating it each time.                       */

#define htond(X, Y) {bDoubleUnion bNetworkTmp; bNetworkTmp.number = X; \
    Y[0] = bNetworkTmp.array[0]; Y[1] = bNetworkTmp.array[1];}

#define ntohd(X, Y) {bDoubleUnion bNetworkTmp; bNetworkTmp.array[0] = X[0]; \
  bNetworkTmp.array[1] = X[1]; Y = bNetworkTmp.number;}

#define htonf(X, Y) {bFloatUnion bNetworkTmp; bNetworkTmp.number = X; \
    Y = bNetworkTmp.long_number;}

#define ntohf(X, Y) {bFloatUnion bNetworkTmp; bNetworkTmp.long_number = X; \
    Y = bNetworkTmp.number;}


#ifdef __cplusplus
}
#endif


#endif
