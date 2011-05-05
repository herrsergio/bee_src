
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



/*   WorkBone CD Rom Player Software

     Copyright (c) 1994  Thomas McWilliams 
   
     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2, or (at your option)
     any later version.
     
     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.
     
     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

*/

#define CDNULL 0
#define CDPLAY 1
#define CDPAUZ 3
#define CDSTOP 4
#define CDEJECT 5

#ifdef WBLAZY
#define WBS_DELAY 2
#define WBU_DELAY 0
#endif

#ifndef WBS_DELAY
#define WBS_DELAY 1
#endif

#ifndef WBU_DELAY
#define WBU_DELAY 0
#endif

#define UPCUR "\033[1A"
#define REVON "\033[7m"
#define REVOF "\033[27m"

/* kernel 1.0 uses different escape sequence for pc character set */
#define OGON "\033(U"
#define OGOF "\033(B"
#define GON "\033[11m"
#define GOF "\033[10m"

#define MTAB "        "
#define MTAB3 MTAB MTAB MTAB 
#define ROW0 MTAB3 "\263                      \263\n"
#define ROW1 MTAB3 "\263    \376\376    ||    |\020    \263\n"
#define ROW2 MTAB3 "\263    |\021    \036\036    \020|    \263\n"
#define ROW3 MTAB3 "\263    \021\021    ..    \020\020    \263\n"
#define ROW4 MTAB3 "\263   " REVON "   quit   " REVOF "   " REVON " ? " REVOF "   \263\n"
#define HLIN1 "\304\304\304\304\304\304\304\304\304\304\304"
#define HLIN2 "\304\304\304\304"
#define HDR "\264 number pad \303"
#define ROWT MTAB3 "\332" HLIN2 HDR HLIN2 "\277\n" 
#define ROWB MTAB3 "\300" HLIN1 HLIN1 "\331\n" "%s\n" UPCUR "\r"

#define AROW0 MTAB3 "|                      |\n"
#define AROW1 MTAB3 "|    []    ||    =>    |\n"
#define AROW2 MTAB3 "|    <     ^^     >    |\n"
#define AROW3 MTAB3 "|    <<    ..    >>    |\n"
#define AROW4 MTAB3 "|    quit         ?    |\n"
#define AHLIN1 "-----------"
#define AHLIN2 "----"
#define AHDR "| number pad |"
#define AROWT MTAB3 "+" AHLIN2 AHDR AHLIN2 "+\n" 
#define AROWB MTAB3 "+" AHLIN1 AHLIN1 "+\n" "\n" UPCUR "\r"

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

void play_cd( int start, int pos, int end) ;
void strmcpy (char **t, const char *s);
int eject_cd( void );
int cd_status( void );
void pause_cd( void );
void stop_cd( void );
void show_terms( const char **p );

