
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




#ifndef VECTOR_H
#define VECTOR_H

#include <math.h>
#include "Common.h"
#include "LIB-math.h"

typedef struct _vector {
    double x;
    double y;
} _vector, *Vector;


void   VectorCopy(Vector vector1, Vector vector2);
void   CoordinatesToVector(Vector vector_result, double x, double y);
void   VectorGetCoordinates(Vector vector, double *x, double *y);
void   PolarToVector(Vector vector_result, double dist, double degrees);
void   VectorAdd(Vector vector1, Vector vector2);
void   VectorSub(Vector vector1, Vector vector2);
void   VectorTimes(Vector vector, double times);
void   VectorTurn(Vector vector, double degrees);
double VectorModule(Vector vector);
double VectorDirection(Vector vector);
double VectorAngle(Vector vector1, Vector vector2);
double RadToMap(double rad);
double MapToRad(double deg);
double NormalizeDegrees(double deg);

#endif
