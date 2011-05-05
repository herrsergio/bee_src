
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


void colliSetMode(int mode);
void colliStopRobot(void);
void colliSetVelocity(float trans_speed, float rot_speed);
void colliSetAcceleration(float trans_accel, float rot_accel);
void colliApproachAbsolute(float x, float y,
			   float approachDist, int newTarget);

/* Direction to the target point is given by the sign of the parameters:
 * forward > 0  --> target point in front of the robot
 * sideward > 0 --> target point to the right of the robot */
void colliApproachRelative(float forward, float sideward,
			   float approachDist, int newTarget);

void colliRegister(void);

int colliConnect(int wait_till_established); /* if parameter is 1, then
					      * this will wait until
					      * connection has been 
					      * established */

int colliConnected;		/* 1, if there is a connection to
				 * the server, 0 if not    */

/*
 * The two point functions in COLLI seem to not work at this time.
 */

void colliApproachTwoPointsAbsolute(float x1, float y1, float x2, float y2,
				    float approachDist, int newTarget);
void colliApproachTwoPointsRelative(float x1, float y1, float x2, float y2,
				    float approachDist, int newTarget);
