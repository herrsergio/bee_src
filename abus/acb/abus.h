
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
 * libabdriver.h   library for abus device drivers
 *
 * Tyson D. Sawyer
 * tyson@rwii.com
 * Real World Interface, Inc.
 *
 * The creation and continued develoment of this software
 * is sponsored and directed by RWI in the interest of
 * providing the mobile robotics and AI research communities
 * with a well designed and robust Robot Applications
 * Interface (RAI) for the complete line of RWI mobile robots. 
 */

/*
 * Copyright 1995, Real World Interface, Inc.
 *
 * Permission to use and modify this software is herby granted by
 * Real World Interface, Inc. (RWI) for non-commercial uses without
 * fee, provided, however, that the above copyright notice appear in
 * all copies, that both the copyright notice and this permission
 * notice appear in supporting documentation.  This permission may be
 * extended to commercial use of this software with specific written
 * permision from RWI.  RWI makes no representations about the
 * suitability of this software for any purpose.  It is provided
 * "as is" without express or implied warranty.  RWI requests
 * notification of any modifications to this software or its
 * documentation.
 *
 *   ==Contact  support@rwii.com  for further information==
 */

#ifndef _AB_DRIVER_H_
#define _AB_DRIVER_H_

#include <acb/global.h>

#define ABD_MAX_DEVS  (20)

#define ABD_DEV_STATE_NOT 0	/* dev struct not in use */
#define ABD_DEV_STATE_NEW 1	/* dev offered */
#define ABD_DEV_STATE_ACK 2	/* dev accepted, waiting for ack */
#define ABD_DEV_STATE_RDY 3	/* dev ready for use */

typedef struct {
  int state;			/* is this struct in use, etc. */
  int busId;			/* bus address of this dev */
  int fd;			/* fd of this dev */

  DevId     devId;		/* prot, rev, vendor, module, devNum */
  DevProt   devProt;		/* prot, type, model */
  DevStatus devStatus;		/* status */
} abdDevType;

extern abdDevType abdDev[];

/* These send the respective messages with arguments */

int D2M_reset       (void);
int D2M_linkRequest (const char *prot, const char *type, const char *model);
int D2M_linkApprove (int devId, int approve);
int D2M_getLongId   (int devId);
int D2M_getId       (int devId);
int D2M_getType     (int devId);
int D2M_getStatus   (int devId);
int D2M_enable      (int devId, int enable);
int D2M_msgToDevice (ABMSG *msg);
int D2M_disconnect  (int devId);

/* These are normally called by abDriverLibSelect   */
/* They will update the assocated abdDev struct and */
/* call msgToDriverCB().                            */

int M2D_linkReply       (ABMSG *msg);
int M2D_longId          (ABMSG *msg);
int M2D_id              (ABMSG *msg);
int M2D_type            (ABMSG *msg);
int M2D_status          (ABMSG *msg);
int M2D_disconnect      (ABMSG *msg);
int M2D_linkApproveAck  (ABMSG *msg);

/* This is normally called by abDriverLibSelect */
/* It will only call msgToDriverCB()            */

int M2D_msgToDriver     (ABMSG *msg);

/* wrapper for M2D functions */
int abDriverLibSelect(int fd);

int abDriverLibInit(int fd, int (*msgToDriverCB)(ABMSG *msg));

#endif /* _AB_DRIVER_H_ */
