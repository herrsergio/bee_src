
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


#ifndef PANTILT_CLIENT_H
#define PANTILT_CLIENT_H

/**** TCX module name - this is a unique identifier for TCX *******/
#define TCX_PANTILT_MODULE_NAME "PANTILT"

extern float ptStatusPanVel;
extern float ptStatusPanPos;
extern float ptStatusPanAccel;

extern float ptStatusTiltVel;
extern float ptStatusTiltPos;
extern float ptStatusTiltAccel;
extern int   ptConnected;

#ifdef __cplusplus
extern "C" {
#endif

int ptConnected;		/* 1, if there is a connection to
				 * the server, 0 if not */

void ptMoveTo(float pan2, float tilt2);
void ptMoveBy(float pan2, float tilt2);
void ptSetVel(float speed);
void ptSetAccel(float accel);
void ptTrackPoint(float x, float y, float z);
void ptStopTracking(void);
void ptRegister();

int ptConnect(int wait_till_established); /* if parameter is 1, then
					   * this will wait until
					   * connection has been 
					   * established */

void ptTerminate(void);
void ptDisconnected(void);
void ptStatusCB(void *);

#ifdef __cplusplus
}
#endif

#endif /* PANTILT_CLIENT_H */
