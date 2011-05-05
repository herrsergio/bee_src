
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




#include <stdlib.h>
#include <raiServer.h>
#include <sensorMessages.h>

/***** commands for the base server ****/
#include <baseMessages.h>
#include <odoClient.h>


#define BASE_SERVER_NAME "baseTCXServer"
#define BASE_FIXED_MESSAGE "baseFixed"
#define BASE_VARIABLE_MESSAGE "baseVar"

/***** TCX messages and handlers for the base server ****/

#define BASE_MESSAGES \
{BASE_FIXED_MESSAGE, raiFixedMsg}, \
{BASE_VARIABLE_MESSAGE, raiVariableMsg}


void bSetXPos(unsigned long number);
void bSetYPos(unsigned long number);
void bSetHeading(unsigned long number);



