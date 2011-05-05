
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
void
  installSimMap(FILE *mapfd);
int
  getDistance(float fromX, float fromY, float fromZ,
		float toX, float toY,
		float *distance);
#ifdef __cplusplus
}
#endif
