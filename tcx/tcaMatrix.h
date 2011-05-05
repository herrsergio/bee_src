
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
* PRELIMINARY RELEASE. COMMENTS AND QUESTIONS CAN BE SENT TO fedor@cs.cmu.edu 
* (c) Copyright 1993 Christopher Fedor. All rights reserved.
*
* MODULE: matrix
*
* FILE: tcaMatrix.h
*
* ABSTRACT:
*
* typedefs for matrix package used in GIL
*
* REVISION HISTORY:
*
* 17-Apr-90 Christopher Fedor at School of Computer Science, CMU
* Added ucmat. Seems that the gil vision version of matrix.h is different.
*
*  3-Apr-90 Christopher Fedor at School of Computer Science, CMU
* Created from matrix.h to avoid perception link conflicts.
*
*****************************************************************************/

#ifndef INCtcaMatrix
#define INCtcaMatrix

/* matrix.h -- define types for matrices using Iliffe vectors
 *************************************************************
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
 */

typedef struct {
	int	lb1, ub1, lb2, ub2;
	char	*mat_sto;
	unsigned char	**el;
	}
	ucmat;

typedef struct {
	int	lb1, ub1, lb2, ub2;
	char	*mat_sto;
	char	**el;
	}
	cmat;

typedef struct {
	int	lb1, ub1, lb2, ub2;
	char	*mat_sto;
	short	**el;
	}
	smat;

typedef struct {
	int	lb1, ub1, lb2, ub2;
	char	*mat_sto;
	int	**el;
	}
	imat;

typedef struct {
	int	lb1, ub1, lb2, ub2;
	char	*mat_sto;
	long	**el;
	}
	lmat;

typedef struct {
	int	lb1, ub1, lb2, ub2;
	char	*mat_sto;
	float	**el;
	}
	fmat;

typedef struct {
	int	lb1, ub1, lb2, ub2;
	char	*mat_sto;
	double	**el;
	}
	dmat;

extern ucmat newucmat2();
extern cmat newcmat2();
extern smat newsmat2();
extern imat newimat2();
extern lmat newlmat2();
extern fmat newfmat2();
extern dmat newdmat2();

#define FREEMAT(m) free((m).mat_sto)

#endif /* INCtcaMatrix */
