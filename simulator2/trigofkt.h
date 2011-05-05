
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

#ifdef __cplusplus
extern "C" {
#endif
extern float _mySIN(float), _myCOS(float),_myTAN(float);
void initTrigFkt();
#ifdef __cplusplus
}
#endif

#define mySIN(x) sin(x)
#define myCOS(x) cos(x)
#define myTAN(x) tan(x)
