
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



/*****************************************************************************
* PROJECT: Carnegie Mellon Planetary Rover Project
*          Task Control Architecture
*
* (c) Copyright 1991 Christopher Fedor and Reid Simmons.  All rights reserved.
* 
* MODULE: matrix
*
* FILE: tcaMatrix.c
*
* ABSTRACT:
*
* matrix creation routines for matrix package used in GIL
* Modified from existing matrix package.
*
* REVISION HISTORY:
*
* 17-Apr-90 Christopher Fedor at School of Computer Science, CMU
* Added newucmat2.
*
*  3-Apr-90 Christopher Fedor at School of Computer Science, CMU
* Created from matrix.c to avoid perception link conflicts.
*
*****************************************************************************/

/* matrix.c -- library routines for constructing dynamic matrices
 * with arbitrary bounds using Iliffe vectors
 ****************************************************************
 * HISTORY
 * 25-Nov-80  David Smith (drs) at Carnegie-Mellon University
 * Changed virtual base address name to "el" for all data
 * types (Previously vali, vald, ...)  This was possible due to the
 * compiler enhancement which keeps different structure declarations
 * separate.
 *
 * 30-Oct-80  David Smith (drs) at Carnegie-Mellon University
 *	Rewritten for record-style matrices
 *
 * 28-Oct-80  David Smith (drs) at Carnegie-Mellon University
 *	Written.
 *
 */

#ifdef VMS
#include "vms.h"                      
#endif                

#include "tcaMatrix.h"
extern char *tcaMalloc();

ucmat newucmat2(rs, re, cs, ce, error)
	int rs, re, cs, ce, *error;
	{
	register unsigned char *p, **b;
	register int r, rows, cols;
	ucmat matrix;

	*error=0;
	rows = re-rs+1;
	cols = ce-cs+1;

	if (rows<=0 || cols<=0) {*error=1; return matrix;}

	matrix.lb1 = rs;
	matrix.ub1 = re;
	matrix.lb2 = cs;
	matrix.ub2 = ce;
	b = (unsigned char **)
	    tcaMalloc(rows*sizeof(unsigned char *) +
	    rows*cols*sizeof(unsigned char));
	if (b==0) {*error=1; return(matrix);}
	matrix.mat_sto = (char *) b;
	p = ((unsigned char *) &b[rows]) - cs;
	matrix.el = b -= rs;
	for (r=rs; r<=re; r++)
	    {
	    b[r] = p;
	    p += cols;
	    }
	return matrix;
      }

cmat newcmat2(rs, re, cs, ce, error)
	int rs, re, cs, ce, *error;
	{
	register char *p, **b;
	register int r, rows, cols;
	cmat matrix;

	rows = re-rs+1;
	cols = ce-cs+1;
	if (rows<=0 || cols<=0) {*error=1; return matrix;}
	matrix.lb1 = rs;
	matrix.ub1 = re;
	matrix.lb2 = cs;
	matrix.ub2 = ce;
	b = (char **)
	    tcaMalloc(rows*sizeof(char *) + rows*cols*sizeof(char));
	if (b==0) {*error=1; return(matrix);}
	matrix.mat_sto = (char *) b;
	*error=0;
	p = ((char *) &b[rows]) - cs;
	matrix.el = b -= rs;
	for (r=rs; r<=re; r++)
	    {
	    b[r] = p;
	    p += cols;
	    }
	return matrix;
	}

smat newsmat2(rs, re, cs, ce, error)
	int rs, re, cs, ce, *error;
	{
	register short *p, **b;
	register int r, rows, cols;
	smat matrix;

	rows = re-rs+1;
	cols = ce-cs+1;
	if (rows<=0 || cols<=0) {*error=1; return matrix;}
	matrix.lb1 = rs;
	matrix.ub1 = re;
	matrix.lb2 = cs;
	matrix.ub2 = ce;
	b = (short **)
	    tcaMalloc(rows*sizeof(short *) + rows*cols*sizeof(short));
	if (b==0) {*error=1; return(matrix);}
	matrix.mat_sto = (char *) b;
	*error=0;
	p = ((short *) &b[rows]) - cs;
	matrix.el = b -= rs;
	for (r=rs; r<=re; r++)
	    {
	    b[r] = p;
	    p += cols;
	    }
	return matrix;
	}

imat newimat2(rs, re, cs, ce, error)
	int rs, re, cs, ce, *error;
	{
	register int *p, **b;
	register int r, rows, cols;
	imat matrix;

	rows = re-rs+1;
	cols = ce-cs+1;
	if (rows<=0 || cols<=0) {*error=1; return matrix;}
	matrix.lb1 = rs;
	matrix.ub1 = re;
	matrix.lb2 = cs;
	matrix.ub2 = ce;
	b = (int **)
	    tcaMalloc(rows*sizeof(int *) + rows*cols*sizeof(int));
	if (b==0) {*error=1; return(matrix);}
	matrix.mat_sto = (char *) b;
	*error=0;
	p = ((int *) &b[rows]) - cs;
	matrix.el = b -= rs;
	for (r=rs; r<=re; r++)
	    {
	    b[r] = p;
	    p += cols;
	    }
	return matrix;
	}

lmat newlmat2(rs, re, cs, ce, error)
	int rs, re, cs, ce, *error;
	{
	register long *p, **b;
	register int r, rows, cols;
	lmat matrix;

	rows = re-rs+1;
	cols = ce-cs+1;
	if (rows<=0 || cols<=0) {*error=1; return matrix;}
	matrix.lb1 = rs;
	matrix.ub1 = re;
	matrix.lb2 = cs;
	matrix.ub2 = ce;
	b = (long **)
	    tcaMalloc(rows*sizeof(long *) + rows*cols*sizeof(long));
	if (b==0) {*error=1; return(matrix);}
	matrix.mat_sto = (char *) b;
	*error=0;
	p = ((long *) &b[rows]) - cs;
	matrix.el = b -= rs;
	for (r=rs; r<=re; r++)
	    {
	    b[r] = p;
	    p += cols;
	    }
	return matrix;
	}

fmat newfmat2(rs, re, cs, ce, error)
	int rs, re, cs, ce, *error;
	{
	register float *p, **b;
	register int r, rows, cols;
	fmat matrix;

	rows = re-rs+1;
	cols = ce-cs+1;
	if (rows<=0 || cols<=0) {*error=1; return matrix;}
	matrix.lb1 = rs;
	matrix.ub1 = re;
	matrix.lb2 = cs;
	matrix.ub2 = ce;
	b = (float **)
	    tcaMalloc(rows*sizeof(float *) + rows*cols*sizeof(float));
	if (b==0) {*error=1; return(matrix);}
	matrix.mat_sto = (char *) b;
	*error=0;
	p = ((float *) &b[rows]) - cs;
	matrix.el = b -= rs;
	for (r=rs; r<=re; r++)
	    {
	    b[r] = p;
	    p += cols;
	    }
	return matrix;
	}

dmat newdmat2(rs, re, cs, ce, error)
	int rs, re, cs, ce, *error;
	{
	register double *p, **b;
	register int r, rows, cols;
	dmat matrix;

	rows = re-rs+1;
	cols = ce-cs+1;
	if (rows<=0 || cols<=0) {*error=1; return matrix;}
	matrix.lb1 = rs;
	matrix.ub1 = re;
	matrix.lb2 = cs;
	matrix.ub2 = ce;
	b = (double **)
	    tcaMalloc(rows*sizeof(double *) + rows*cols*sizeof(double));
	if (b==0) {*error=1; return(matrix);}
	matrix.mat_sto = (char *) b;
	*error=0;
	p = ((double *) &b[rows]) - cs;
	matrix.el = b -= rs;
	for (r=rs; r<=re; r++)
	    {
	    b[r] = p;
	    p += cols;
	    }
	return matrix;
	}

