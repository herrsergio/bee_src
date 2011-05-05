
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


#include <stdio.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>

#include <bUtils.h>

#include "robot_specifications.h"
#include "tcx.h"
#include "tcxP.h"

#include <raiClient.h>

#include "BUTTONS-messages.h"
#include "buttonClient.h"
#include "buttonHandlers.h"
#include "beeSoftVersion.h"
#include "librobot.h"
#include "libezx.h"
#include "button.h"

static unsigned char button_bits = 0x00;

int test;

int simulator;

/* ---------------------------------------------------------
 *
 * ouput byte from port
 *
 * --------------------------------------------------------*/
#ifdef i386
void outb( int addr, u_char b) {
  static int fd=-1;
  if ( simulator )
    fprintf( stderr, "Button action outb()\n");
  else {
    if ( fd==-1 && (fd=open("/dev/port", O_WRONLY))==-1 ) {
      fprintf(stderr,"%s: cannot open '/dev/port'. Wrong permissions?\n",
	      __FILE__ );
      exit(1);
    }
    lseek(fd, addr, SEEK_SET);
    write(fd, &b, 1);
  }
}
#else
void outb( int addr, u_char b) {
}
#endif

/* ---------------------------------------------------------
 *
 * input byte from port
 *
 * --------------------------------------------------------*/
#ifdef i386
u_char inb( int addr ) {
  static int fd=-1;
  u_char b;
  if ( simulator ) {
    fprintf( stderr, "Button action inbb()\n");
    return 0;
  } else {
    if ( fd==-1 && (fd=open("/dev/port", O_RDONLY))==-1 ) {
      fprintf(stderr,"%s: cannot open '/dev/port'. Wrong permissions?\n",
	      __FILE__ );
      exit(1);
    }
    lseek(fd, addr, SEEK_SET);
    read(fd, &b, 1);
    return(b);
  }
}
#else
u_char inb( int addr) {
}
#endif

/************************************************************************
 *
 *   NAME:          resetButtons()
 *                 
 *   FUNCTION:      switches all buttons off, initialization
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
int resetButtons()
{
  unsigned char inbits;

  inbits = (inb(0x378) & 0x80);
  button_bits = 0x30;
  outb(0x378, (inbits | button_bits));

  status.red_light_status                  = BUTTON_LIGHT_STATUS_TOGGLE_OFF;
  status.yellow_light_status               = BUTTON_LIGHT_STATUS_TOGGLE_OFF;
  status.green_light_status                = BUTTON_LIGHT_STATUS_TOGGLE_OFF;
  status.blue_light_status                 = BUTTON_LIGHT_STATUS_TOGGLE_OFF;
  status.left_kill_switch_light_status     = BUTTON_LIGHT_STATUS_ON;
  status.right_kill_switch_light_status    = BUTTON_LIGHT_STATUS_ON;
  status.red_button_pressed                = BUTTON_UNPRESSED;
  status.red_button_changed                = 0;
  status.yellow_button_pressed             = BUTTON_UNPRESSED;
  status.yellow_button_changed             = 0;
  status.green_button_pressed              = BUTTON_UNPRESSED;
  status.green_button_changed              = 0;
  status.blue_button_pressed               = BUTTON_UNPRESSED;
  status.blue_button_changed               = 0;

  return 0;
}

/************************************************************************
 *
 *   NAME:          setButton   ()
 *                 
 *   FUNCTION:      switches a particular button
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
void setButton(int button, unsigned char position)
{
  unsigned char inbits;

  if (button < 6){
    inbits = (inb(0x378) & 0x80);
    if (position)
	button_bits |= (0x01 << button);
    else
      button_bits &= (0xFF ^ (0x01 << button));
    outb(0x378, (inbits | button_bits));

  }
}

/************************************************************************
 *
 *   FUNCTION:      ^C
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
void commShutdown()
{
  unsigned char inbits;

  fprintf( stderr, "%s: Somebody died. Exiting.\n", __FILE__ );

  button_bits = 0x00;

  inbits = (inb(0x378) & 0x80);

  outb(0x378, (inbits | button_bits));

  exit(0);
}

/************************************************************************
 *
 *   NAME:          check_buttons()
 *                 
 *   FUNCTION:      checks all buttons
 *                 
 *   RETURN-VALUE: 1, if a button changed
 *                 
 ************************************************************************/
int check_buttons()
{
  unsigned char a = 0;
  int changed = 0;
  int red, yellow, green, blue;

  status.red_button_changed                = 0;
  status.yellow_button_changed             = 0;
  status.green_button_changed              = 0;
  status.blue_button_changed               = 0;

  a = inb(0x379);

  red = (int) (a & 0x10);
  if (red != status.red_button_pressed){
    status.red_button_pressed = red;
    changed = 1;
    status.red_button_changed = 1;
    if (red)
      fprintf(stderr, "RED\n");
  }

  yellow = (int) (a & 0x20);
  if (yellow != status.yellow_button_pressed){
    status.yellow_button_pressed = yellow;
    changed = 1;
    status.yellow_button_changed = 1;
    if (yellow)
      fprintf(stderr, "YELLOW\n");
  }

  green = (int) (a & 0x40);
  if (green != status.green_button_pressed){
    status.green_button_pressed = green;
    changed = 1;
    status.green_button_changed = 1;
    if (green)
      fprintf(stderr, "GREEN\n");
  }

  blue = (int) ((a ^ 0xFF) & 0x80);
  if (blue != status.blue_button_pressed){
    status.blue_button_pressed = blue;
    changed = 1;
    status.blue_button_changed = 1;
    if (blue)
      fprintf(stderr, "BLUE\n");
  }

  return changed;
}

/************************************************************************
 *
 *   NAME:         check_and_set_buttons()
 *                 
 *   FUNCTION:     kind of the main routine
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
void check_and_set_buttons()
{
  static int counter = 0;
  int changed;

  counter = ((counter + 1) % 15);

  changed = check_buttons();

  if (status.red_button_pressed && status.red_button_changed){
    if (status.red_light_status == 
	BUTTON_LIGHT_STATUS_FLASHING_TILL_PRESSED ||
	status.red_light_status == BUTTON_LIGHT_STATUS_ON_TILL_PRESSED)
      status.red_light_status = BUTTON_LIGHT_STATUS_OFF;
    if (status.red_light_status == BUTTON_LIGHT_STATUS_OFF_TILL_PRESSED)
      status.red_light_status = BUTTON_LIGHT_STATUS_ON;
    if (status.red_light_status == BUTTON_LIGHT_STATUS_TOGGLE_ON)
      status.red_light_status = BUTTON_LIGHT_STATUS_TOGGLE_OFF;
    else if (status.red_light_status == BUTTON_LIGHT_STATUS_TOGGLE_OFF)
      status.red_light_status = BUTTON_LIGHT_STATUS_TOGGLE_ON;
  }
  if (status.red_light_status == BUTTON_LIGHT_STATUS_ON ||
      status.red_light_status == BUTTON_LIGHT_STATUS_ON_TILL_PRESSED ||
      status.red_light_status == BUTTON_LIGHT_STATUS_TOGGLE_ON ||
      ((status.red_light_status == BUTTON_LIGHT_STATUS_FLASHING ||
	status.red_light_status == BUTTON_LIGHT_STATUS_FLASHING_TILL_PRESSED)
       && counter > 5))
    setButton(0, 1);
  else
    setButton(0, 0);


  if (status.yellow_button_pressed && status.yellow_button_changed){
    if (status.yellow_light_status == 
	BUTTON_LIGHT_STATUS_FLASHING_TILL_PRESSED ||
	status.yellow_light_status == BUTTON_LIGHT_STATUS_ON_TILL_PRESSED)
      status.yellow_light_status = BUTTON_LIGHT_STATUS_OFF;
    if (status.yellow_light_status == BUTTON_LIGHT_STATUS_OFF_TILL_PRESSED)
      status.yellow_light_status = BUTTON_LIGHT_STATUS_ON;
    if (status.yellow_light_status == BUTTON_LIGHT_STATUS_TOGGLE_ON)
      status.yellow_light_status = BUTTON_LIGHT_STATUS_TOGGLE_OFF;
    else if (status.yellow_light_status == BUTTON_LIGHT_STATUS_TOGGLE_OFF)
      status.yellow_light_status = BUTTON_LIGHT_STATUS_TOGGLE_ON;
  }
  if (status.yellow_light_status == BUTTON_LIGHT_STATUS_ON ||
      status.yellow_light_status == BUTTON_LIGHT_STATUS_ON_TILL_PRESSED ||
      status.yellow_light_status == BUTTON_LIGHT_STATUS_TOGGLE_ON ||
      ((status.yellow_light_status == BUTTON_LIGHT_STATUS_FLASHING ||
	status.yellow_light_status ==
	BUTTON_LIGHT_STATUS_FLASHING_TILL_PRESSED)
       && counter > 5))
    setButton(1, 1);
  else
    setButton(1, 0);



  if (status.green_button_pressed && status.green_button_changed){
    if (status.green_light_status == 
	BUTTON_LIGHT_STATUS_FLASHING_TILL_PRESSED ||
	status.green_light_status == BUTTON_LIGHT_STATUS_ON_TILL_PRESSED)
      status.green_light_status = BUTTON_LIGHT_STATUS_OFF;
    if (status.green_light_status == BUTTON_LIGHT_STATUS_OFF_TILL_PRESSED)
      status.green_light_status = BUTTON_LIGHT_STATUS_ON;
    if (status.green_light_status == BUTTON_LIGHT_STATUS_TOGGLE_ON)
      status.green_light_status = BUTTON_LIGHT_STATUS_TOGGLE_OFF;
    else if (status.green_light_status == BUTTON_LIGHT_STATUS_TOGGLE_OFF)
      status.green_light_status = BUTTON_LIGHT_STATUS_TOGGLE_ON;
  }
  if (status.green_light_status == BUTTON_LIGHT_STATUS_ON ||
      status.green_light_status == BUTTON_LIGHT_STATUS_ON_TILL_PRESSED ||
      status.green_light_status == BUTTON_LIGHT_STATUS_TOGGLE_ON ||
      ((status.green_light_status == BUTTON_LIGHT_STATUS_FLASHING ||
	status.green_light_status == BUTTON_LIGHT_STATUS_FLASHING_TILL_PRESSED)
       && counter > 5))
    setButton(2, 1);
  else
    setButton(2, 0);


  if (status.blue_button_pressed && status.blue_button_changed){
    if (status.blue_light_status == 
	BUTTON_LIGHT_STATUS_FLASHING_TILL_PRESSED ||
	status.blue_light_status == BUTTON_LIGHT_STATUS_ON_TILL_PRESSED)
      status.blue_light_status = BUTTON_LIGHT_STATUS_OFF;
    if (status.blue_light_status == BUTTON_LIGHT_STATUS_OFF_TILL_PRESSED)
      status.blue_light_status = BUTTON_LIGHT_STATUS_ON;
    if (status.blue_light_status == BUTTON_LIGHT_STATUS_TOGGLE_ON)
      status.blue_light_status = BUTTON_LIGHT_STATUS_TOGGLE_OFF;
    else if (status.blue_light_status == BUTTON_LIGHT_STATUS_TOGGLE_OFF)
      status.blue_light_status = BUTTON_LIGHT_STATUS_TOGGLE_ON;
  }
  if (status.blue_light_status == BUTTON_LIGHT_STATUS_ON ||
      status.blue_light_status == BUTTON_LIGHT_STATUS_ON_TILL_PRESSED ||
      status.blue_light_status == BUTTON_LIGHT_STATUS_TOGGLE_ON ||
      ((status.blue_light_status == BUTTON_LIGHT_STATUS_FLASHING ||
	status.blue_light_status == BUTTON_LIGHT_STATUS_FLASHING_TILL_PRESSED)
       && counter > 5))
    setButton(3, 1);
  else
    setButton(3, 0);


  if (status.left_kill_switch_light_status == BUTTON_LIGHT_STATUS_ON ||
      status.left_kill_switch_light_status == 
      BUTTON_LIGHT_STATUS_ON_TILL_PRESSED ||
      status.left_kill_switch_light_status == BUTTON_LIGHT_STATUS_TOGGLE_ON ||
      ((status.left_kill_switch_light_status == BUTTON_LIGHT_STATUS_FLASHING ||
	status.left_kill_switch_light_status == 
	BUTTON_LIGHT_STATUS_FLASHING_TILL_PRESSED)
       && counter > 5))
    setButton(4, 1);
  else
    setButton(4, 0);

  if (status.right_kill_switch_light_status == BUTTON_LIGHT_STATUS_ON ||
      status.right_kill_switch_light_status == 
      BUTTON_LIGHT_STATUS_ON_TILL_PRESSED ||
      status.right_kill_switch_light_status == BUTTON_LIGHT_STATUS_TOGGLE_ON ||
      ((status.right_kill_switch_light_status ==
	BUTTON_LIGHT_STATUS_FLASHING ||
	status.right_kill_switch_light_status == 
	BUTTON_LIGHT_STATUS_FLASHING_TILL_PRESSED)
       && counter > 5))
    setButton(5, 1);
  else
    setButton(5, 0);

  if ( changed )
    send_automatic_status_update( );

}

/* ---------------------------------------------------------
 *
 *
 *
 * --------------------------------------------------------*/
int displaybParameters() {

  fprintf(stderr, "********  OPTIONS  **********\n");
  if ( test )
    fprintf(stderr, "test      = on\n");
  else
    fprintf(stderr, "test      = off\n");
  fprintf(stderr, "simulator = %d\n", simulator );
  fprintf(stderr, "*****************************\n");

  return 0;
}

/* ---------------------------------------------------------
 *
 *
 *
 * --------------------------------------------------------*/
int DoDaTesting() {

  unsigned char a, b;
  int i, c;

  b = 0; a = 0; /* so the compiler stops barfing when using -Wall */
  b = inb(0x379);
  c = 0;
  do{
    for (i = 0; i < 6; i++){
      setButton(i, 1);
      usleep(10000);
    }
    for (i = 0; i < 6; i++){
      setButton(i, 0);
      usleep(10000);
    }
    a = inb(0x379);
  } while (a == b && ( 0 || c++ < 3));

  fprintf(stderr, "Buttons ready.\n");

  return 0;
}

/************************************************************************
 *
 *   NAME:         main()
 *                 
 *   FUNCTION:     main program
 *                 
 ************************************************************************/
int main( int argc, char **argv ) {

  struct bParamList * params = NULL;
  struct timeval TCX_waiting_time = {0, 0};
  int i;

  /* set defaults */
  params = bParametersAddEntry(params, "", "test", "0");

  /* set defaults */
  params = bParametersAddEntry(params, "", "simulator", "0");

  /* add some parameter files */
  params = bParametersAddFile(params, "etc/beeSoft.ini");

  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams( params );

#if (!defined(sun))
  test    = atoi(bParametersGetParam(params, "", "test"));
#else
  test    = 0;
#endif

  simulator = atoi(bParametersGetParam(params, "", "simulator"));

  tcxMachine = (char*)bParametersGetParam(params, "", "TCXHOST");

  displaybParameters();

  if ( resetButtons() != 0 ) {
    return -1;
  }

  signal(SIGTERM, &commShutdown); /* kill interupt handler */
  signal(SIGINT,  &commShutdown); /* control-C interupt handler */

  BUTTONS_initialize_tcx();

  if ( test ) {
    DoDaTesting();
    commShutdown();
  }

  resetButtons();

  buttons_effect = 0;

  for (;;) {
    
    if ( simulator )
      TCX_waiting_time.tv_sec  = 1;
    else 
      TCX_waiting_time.tv_sec  = 0;

    TCX_waiting_time.tv_usec = 0;

#ifdef i386
    check_and_set_buttons();
#endif

    if ( buttons_effect ) {
      for (i = 0; i < 4; i++){
	setButton(i, 1);
	usleep(30000);
      }
      for (i = 0; i < 4; i++){
	setButton(i, 0);
	usleep(30000);
      }
    } else {      
      usleep(20000);
    }

#if ( defined(TCX_DEBUG) )
/*     fprintf(stderr, "("); */
#endif
    tcxRecvLoop((void *) &TCX_waiting_time);
#if ( defined(TCX_DEBUG) )
/*     fprintf(stderr, ")"); */
#endif
    
  }

  return 0;

}
