
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




#ifndef _ARM_CLIENT_H
#define _ARM_CLIENT_H


#ifdef  __cplusplus
extern "C" {
#endif

#include <raiClient.h>
#include <armMessages.h>

void registerArmClient();	/* this function is obsolete. DO NOT USE ANYMORE */
void findArmServer();		/* this function is obsolete. DO NOT USE ANYMORE */

void armRegister();

int armConnect(int wait_till_established); /* if parameter is 1, then
					    * this will wait until
					    * connection has been 
					    * established */

void subscribeToArm(clientCallback);

int armConnected;		/* 1, if there is a connection to
				 * the arm server, 0 if not */
 
  void ArmInit(void);
  void ArmShutdown(void);
  void deployArm();
  void stowArm();
  void armLimp();

  /* Mast */
  void mastLimp();  /* in case of emergency or CTRL-C */
  void mastHalt();  
  void mastRelativeUp(unsigned long amount);
  void mastRelativeDown(unsigned long amount);
  void mastToPos(unsigned long pos);
  void mastVelocityDown();
  void mastVelocityUp();
  void mastWhere(void);

  /* gripper */
  void gripLimp();
  void gripHalt();
  void gripToPos(unsigned long amount);
  void gripRelativeOpen(unsigned long amount);
  void gripRelativeClose(unsigned long amount);
  void gripWhere(void);

  /* wrist */
  void wristLimp(void);
  void wristHalt(void);
  void wristToPos(unsigned long pos);
  void wristRelativePos(unsigned long amount);
  void wristRelativeNeg(unsigned long amount);

  void wristWhere(void);


#ifdef  __cplusplus
}
#endif



#endif

