
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








/******************************************************************************
*
* PROJECT: Carnegie Mellon Planetary Rover Project
*          Task Control Architecture 
* 
* (c) Copyright 1991 Christopher Fedor and Reid Simmons.  All rights reserved.
* 
* MODULE: print data
*
* FILE: printData.h
*
* ABSTRACT:
* 
* Print Data Interface Routines - central server only.
*
* REVISION HISTORY
*
* 28-Oct-90 Christopher Fedor, School of Computer Science, CMU
* (slightly) revised to standards.
*
* 24-Jul-89 Reid Simmons, School of Computer Science, CMU
* created.
*
******************************************************************************/

#ifndef INCprintData
#define INCprintData

/* extern int indentGlobal; */

extern void printString();
extern void printInt();
extern void printChar();
extern void printShort();
extern void printLong();
extern void printFloat();
extern void printDouble();
extern void printByte();
extern void printTwoByte();
extern void printSpace();
extern void Print_Formatted_Data();
extern void Print_Data();
extern void Print_Formatter();

extern void mapPrint();
extern void dPrintSTR();
extern void dPrintFORMAT();
extern void dPrintTCA();

#endif INCprintData
