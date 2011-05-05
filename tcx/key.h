
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
* MODULE: key
*
* FILE: key.h
*
* ABSTRACT:
* 
* Collection of hash and key functions for table lookups.
*
* REVISION HISTORY
*
* 16-Aug-90 Christopher Fedor, School of Computer Science, CMU
* created.
*
******************************************************************************/

#ifndef INCkey
#define INCkey

extern int intHashFunc();
extern int intKeyEqFunc();
extern int strHashFunc();
extern int strKeyEqFunc();
extern int hndHashFunc();
extern int hndKeyEqFunc();
extern int classHashFunc();
extern int classEqFunc();

#endif INCkey
