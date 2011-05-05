
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
#ifndef BUTTON_H_LOADED
#define BUTTON_H_LOADED

BUTTONS_status_reply_type status;

char *tcxMachine;

int buttons_effect;

int resetButtons();

void setButton(int button, unsigned char position);

void interrupt_handler();

int check_buttons();

void check_and_set_buttons();

#endif
/*
 * $Log: button.h,v $
 * Revision 1.3  1997/11/06 17:53:04  swa
 * Fixed a bug with TCXHOST. If the environment variable TCXHOST is set,
 * take it. If not, take the TCXHOST from beeSoft.ini. If this doesn't
 * work, take localhost. If this doesn't work: bummer.
 *
 * Revision 1.2  1997/07/04 16:15:19  swa
 * Added lib support for buttons. No more TCX.
 *
 * Revision 1.1  1997/06/29 21:58:27  thrun
 * Added a ton of files for a client/server button thing.                (swa)
 *
 */
