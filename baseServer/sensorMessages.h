
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



#ifndef _SENSOR_MESSAGES_H
#define _SENSOR_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
   TACTILE_newValues,
   SONAR_newValues,
   IR_newValues
}  SensorMessages;


#ifdef __cplusplus
}
#endif

#endif /* _SENSOR_MESSAGES_H */
