
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


/*
 * NOTE:
 *
 * This file is provided for backwards compatibility with RAI only.
 * It should not be used for new code development.
 *
 */


/* Defines specific to the particular robot instantiation */
/* These are for the generic B21 with standard size and   */
/* gearing */

#ifndef ROBOT_H
#define ROBOT_H

#ifdef B14
#define CONFIG_B14
#undef  CONFIG_B14_PUSHER
#undef  CONFIG_B21
#endif

#ifdef B14_PUSHER
#define CONFIG_B14
#define CONFIG_B14_PUSHER
#undef  CONFIG_B21
#endif

#ifdef B21
#define CONFIG_B21
#undef  CONFIG_B14
#undef  CONFIG_B14_PUSHER
#endif

/**********************************************************************/
/**********************************************************************/

#ifdef CONFIG_B21

/*
 * Definitions for the base
 */

#define CONFIG_B21_Base
 
#define ROBOT_RADIUS 267        /* maximum radius of the robot */

#define MM_TO_BASE_ENCODERS 27.3
                     /* multiplier to convert base shaft encoder */
                     /* to mm, so that  translateRelativePos()   */
                     /* can take millimeters on any robot        */  

#define BASE_POSITION_TO_MM 9.37 
                     /* multiplier to convert status report x and y to */
                     /* millimeters. Each position unit = 256 encoders */
                     /* so this is 256/MM_TO_BASE_ENCODERS. Note if you*/
                     /* define this as (256/MM_TO_BASE_ENCODERS)       */
                     /* be truncated to an int, which is not accurate  */

#undef BASE_ROTATES_BACKWARDS
                     /* Some bases, specifically Ramona, rotate backwards*/
                     /* which totally screws up the base library unless  */
                     /* we fix it.  If your robot rotates backwards,     */
                     /* change the above undef to a define */


#define WARNING_VOLTAGE 44
#define PANIC_VOLTAGE   39 



/*
 * Definitions for the enclosure
 */


#define CONFIG_B21_Enclosure
#define ENCLOSURE_RADIUS 248
 

/*
 * Definitions for the IR
 */


#define CONFIG_B21_Ir

#define IR_RADIUS ENCLOSURE_RADIUS
#define NUM_IR_ROWS      3
#define IR_ROWS { 24, 24, 8 } /* Used to fill in irsInRow in Ir.h */
#define MAX_IRS_PER_ROW 24


/*
 * Definitions for the sonar
 */


#define CONFIG_B21_Sonar
#define SONAR_RADIUS ENCLOSURE_RADIUS
#define NUM_SONARS 24

#define CONFIG_Msp
#undef CONFIG_SonarG96

/*
 * Definitions for the tactile
 */


#define CONFIG_B21_Tactile
#define NUM_TACTILE_ROWS      4
#define TACTILE_ROWS { 12, 12, 16, 16 }
#define MAX_TACTILES_PER_ROW 16

#endif /* CONFIG_B21 */

/**********************************************************************/
/**********************************************************************/

#ifdef CONFIG_B14

/*
 * Definitions for the base
 */

#define CONFIG_B14_Base
 
#define ROBOT_RADIUS 168        /* maximum radius of the robot */

#define MM_TO_BASE_ENCODERS 21.1
                     /* multiplier to convert base shaft encoder */
                     /* to mm, so that  translateRelativePos()   */
                     /* can take millimeters on any robot        */  

#define BASE_POSITION_TO_MM 12.1 
                     /* multiplier to convert status report x and y to */
                     /* millimeters. Each position unit = 256 encoders */
                     /* so this is 256/MM_TO_BASE_ENCODERS. Note if you*/
                     /* define this as (256/MM_TO_BASE_ENCODERS)       */
                     /* be truncated to an int, which is not accurate  */

#undef BASE_ROTATES_BACKWARDS
                     /* Some bases, specifically Ramona, rotate backwards*/
                     /* which totally screws up the base library unless  */
                     /* we fix it.  If your robot rotates backwards,     */
                     /* change the above undef to a define */


#define WARNING_VOLTAGE 10
#define PANIC_VOLTAGE   9  



/*
 * Definitions for the enclosure
 */


#define CONFIG_B14_Enclosure
#define ENCLOSURE_RADIUS 153
 

/*
 * Definitions for the IR
 */


#define CONFIG_B14_Ir

#define IR_RADIUS ENCLOSURE_RADIUS
#define NUM_IR_ROWS      1
#define IR_ROWS { 16  } /* Used to fill in irsInRow in Ir.h */
#define MAX_IRS_PER_ROW 16

 

/*
 * Definitions for the sonar
 */


#define CONFIG_B14_Sonar
#define SONAR_RADIUS ENCLOSURE_RADIUS
#define NUM_SONARS 16

#define CONFIG_Msp
#undef CONFIG_SonarG96

/*
 * Definitions for the tactile
 */


#define CONFIG_B14_Tactile
#define NUM_TACTILE_ROWS      4
#define TACTILE_ROWS { 8, 8, 6, 12 }
#define MAX_TACTILES_PER_ROW 12

#endif /* CONFIG_B14 */

/**********************************************************************/
/**********************************************************************/

/*
 * Definitions for the speech support
 */

#ifdef DOUBLE_TALKER
#define SPEECH_TYPE doubleTalker
#endif

#ifdef NO_SPEECH
#undef SPEECH_TYPE
#endif

/**********************************************************************/
/**********************************************************************/

/*
 * Definitions for the B21 arm
 */

#define ARM_TYPE 4AXIS

/**********************************************************************/
/**********************************************************************/

#ifdef CONFIG_SonarG96

/*  The Robot.h file must define the variable NUM_SONARS to be
    the number of sonars controlled by the SonarG96 controller.  */

/*  The Robot.h file must define the variable NUM_PINGS to be
    the number of pings necessary to complete one sweep of all
    of the sonars.  This number must be consistent with SONAR_PINGMAP. */

/*  The Robot.h file must define the variable SONARS_PER_PING to be
    the number of sonars pinged for each iteration of SONAR_PINGMAP.

    An example of SONARS_PER_PING and NUM_PINGS (for NUM_SONARS == 16) is:

#define NUM_PINGS 4
#define SONARS_PER_PING 4

    Note that (NUM_PINGS * SONARS_PER_PING) == NUM_SONARS.
    This setting of NUM_PINGS and SONARS_PER_PING is consistent with
    the following example of the use of SONAR_PINGMAP.  */

/*  The Robot.h file must define the variable SONAR_PINGMAP.
    This variable defines the order of sonar pings.  This variable
    must be consistent with NUM_PINGS.

    An example of a SONAR_PINGMAP (for NUM_SONARS == 16) is:

#define SONAR_PINGMAP { "RT 1111\r", "RT 2222\r", "RT 3333\r", "RT 4444\r" }

*/

#endif /* CONFIG_SonarG96 */

#endif  /* ROBOT_H */
