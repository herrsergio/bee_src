
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

#include "beeSoftVersion.h"

int  libcolliClient_major      = BEESOFT_VERSION_MAJOR;
int  libcolliClient_minor      = BEESOFT_VERSION_MINOR;
int  libcolliClient_robot_type = BEESOFT_VERSION_ROBOT_TYPE;
char libcolliClient_date[80]   = BEESOFT_VERSION_DATE;

/*
 * $Log: libcameraClient.c,v $
 * Revision 1.2  1997/10/04 00:13:09  swa
 * First working version of the cameraServer that supports two (count'em
 * two!) framegrabbers.  Although the server seems to work just fine, it
 * has not yet been fully tested.
 *
 * Revision 1.1  1997/06/19 03:54:17  thrun
 * client library created
 *
 */
