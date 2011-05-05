
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

#ifndef LASERCLIENT_H
#define LASERCLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

void laserRegister( );

int laserConnect(int wait_till_established); /* if parameter is 1, then
					      * this will wait until
					      * connection has been 
					      * established */

void tcxRegisterCloseHnd(void (*closeHnd)());

int laserConnected;		/* 1, if there is a connection to
				 * the laser server, 0 if not    */

typedef struct laserSweepType {
  int numberLasers;		/* how many beams per laser */
  int numLaser;			/* which laser 0|1 */
  float *value;			/* readings */
} laserSweepType;

typedef int (*laserSweepCallbackType) (laserSweepType *data);

void registerLaserSweepCallback( laserSweepCallbackType fcn );

void laserRequestSweep( int numLaser );

void laserSubscribeSweep( int numLaser, int number );

#ifdef __cplusplus
}
#endif

#endif /* LASERCLIENT_H */

/*
 * $Log: laserClient.h,v $
 * Revision 1.3  1997/09/20 22:56:22  swa
 * Added paramter and beesoft section to the docu.
 *
 * Revision 1.2  1997/08/07 15:42:53  swa
 * First working version of the laserServer. Seems to run fine. Has support for
 * a second laser clientwise, yet the device stuff for the second one is not
 * implemented.
 *
 * Revision 1.1  1997/08/07 02:45:51  swa
 * First version of a laserServer. Compiles nicely, yet produces nothing useful. ;)
 * Just to copy things to a safe place ...
 *
 *
 */
