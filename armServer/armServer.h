
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



#include <raiServer.h>
#include <stdlib.h>

#include <armMessages.h>

#define ARM_SERVER_NAME "armTCXServer"
#define ARM_FIXED_MESSAGE "armFixed"
#define ARM_MESSAGES  { {ARM_FIXED_MESSAGE, raiFixedMsg}}

