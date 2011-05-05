
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
#include <rai.h>

#include <bUtils.h>

#include "tcx.h"
#include "robot_specifications.h"
#include "tcxP.h"
#include "global.h"

#define TCX_define_variables /* this makes sure variables are installed */
#include "CAMERA-messages.h"   /* messages for VISION   */

#define DEFINE_REPLY_HANDLERS
#ifdef SEBASTIAN
#include "BASE-messages.h"     /* messages for BASE     */
#include "PANTILT-messages.h"  /* messages for PANTILT  */
#endif

#include "libezx.h"

#include <signal.h>
#include <sys/mman.h>

#if (!defined(sun))
#include <sys/fcntl.h> 
#include <ioctl_meteor.h> 
#endif

#include "grab.h"
#include "imagesize.h"

#include <Common.h>
#include <EZX11.h>
#include "display.h"

#include "handlers.h"

#include "misc.h"
#include "cameraClient.h"
#include "beeSoftVersion.h"
#include "raiClient.h"


TCX_REG_HND_TYPE CAMERA_handler_array[] = {

  {"CAMERA_image_query", "CAMERA_image_query_handler",
   CAMERA_image_query_handler, TCX_RECV_ALL, NULL},

  {"CAMERA_save", "CAMERA_save",
   CAMERA_save_handler, TCX_RECV_ALL, NULL},

  {"CAMERA_load", "CAMERA_load",
   CAMERA_load_handler, TCX_RECV_ALL, NULL},

  {"CAMERA_startstop", "CAMERA_startstop",
   CAMERA_startstop_handler, TCX_RECV_ALL, NULL},

  {"CAMERA_shmid_query", "CAMERA_shmid_query_handler",
   CAMERA_shmid_query_handler, TCX_RECV_ALL, NULL},
  
  {"CAMERA_register_auto_update", "CAMERA_register_auto_update_handler",
   CAMERA_register_auto_update_handler, TCX_RECV_ALL, NULL}

};


#ifdef SEBASTIAN
static int base_connected = 0;
static int pantilt_connected = 0;
#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/******* auto-reply stuff, data definitions ***************/


#define MAX_N_AUTO_UPDATE_MODULES 100


int n_auto_update_modules = 0; /* number of processes to whom
				* position should be forwarded
				* automatically upon change */

int n_auto_image_update_modules      = 0;



typedef struct{
  TCX_MODULE_PTR module;	/* pointer to TCX module */
  int            numGrabber;	/* zero for the first grabber */
  int            image;		/* >=1=subscribed to regular image updates */
  int            last_image;
  int            image_xsize;
  int            image_ysize;
} auto_update_type;


auto_update_type auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];
                                /* collection of TCX-module ptrs */

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/************************************************************************
 *
 *   NAME:         count_auto_update_modules()
 *                 
 *   FUNCTION:     Counts, how many modules of the different types require
 *                 auto-updates
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/

void count_auto_update_modules()
{
  int i;

  n_auto_image_update_modules      = 0;

  for (i = 0; i < n_auto_update_modules; i++){
    if (auto_update_modules[i].image)      n_auto_image_update_modules++;
  }
}


/************************************************************************
 *
 *   NAME:         add_auto_update_module()
 *                 
 *   FUNCTION:     Adds a module to the list of modules that
 *                 get automatical map updates
 *                 
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *                 data                        subscription information
 *                 
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/



static int add_auto_update_module(TCX_MODULE_PTR                  module,
				  CAMERA_register_auto_update_ptr data)
{
  int i;

  if (n_auto_update_modules >= MAX_N_AUTO_UPDATE_MODULES){
    fprintf(stderr, "ERROR: auto-module memory overflow. Request rejected.\n");
    return 0;
  }
  else
    for (i = 0; i < n_auto_update_modules; i++)
      if (auto_update_modules[i].module == module){
	fprintf( stderr, 
		 "Module %s already known. Subscription modified: %d (%dx%d,%d)\n",
		 tcxModuleName(module), 
		 data->image,
		 data->image_xsize, 
		 data->image_ysize,
		 data->numGrabber );
	auto_update_modules[i].image       = data->image; /* subsrc? */
	auto_update_modules[i].numGrabber  = data->numGrabber;
	auto_update_modules[i].image_xsize = data->image_xsize;
	auto_update_modules[i].image_ysize = data->image_ysize;
	auto_update_modules[i].last_image = -1;
	count_auto_update_modules();
	return 1;
      }
  fprintf(stderr, "Add %s to auto-reply list: %d (%dx%d).\n",
	  tcxModuleName(module), data->image, 
	  data->image_xsize, data->image_ysize);
  auto_update_modules[n_auto_update_modules].module     = module; /* pointer*/
  auto_update_modules[n_auto_update_modules].image      = data->image; 
  auto_update_modules[n_auto_update_modules].numGrabber = data->numGrabber;
  auto_update_modules[n_auto_update_modules].image_xsize = data->image_xsize;
  auto_update_modules[n_auto_update_modules].image_ysize = data->image_ysize;
  auto_update_modules[n_auto_update_modules].last_image = -1;
  n_auto_update_modules++;
  count_auto_update_modules();
  return 1;
}



/************************************************************************
 *
 *   NAME:         remove_auto_update_module()
 *                 
 *   FUNCTION:     Attempts to remove a module from the list of 
 *                 modules that get automatical map updates 
 *                 
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *                 
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/


static int remove_auto_update_module(TCX_MODULE_PTR module)
{     
  int i, j, found = 0;;
  for (i = 0; i < n_auto_update_modules; i++) /* search for module */
    if (auto_update_modules[i].module == module){ /* if module found */
      fprintf(stderr, "Remove %s from auto-reply list.\n",
	      tcxModuleName(module));
      found++;
      n_auto_update_modules--;	/* remove that entry, one less now */
      for (j = i; j < n_auto_update_modules; j++){
	auto_update_modules[j].module = auto_update_modules[j+1].module; 
	auto_update_modules[j].image = auto_update_modules[j+1].image;
	auto_update_modules[j].numGrabber = auto_update_modules[j+1].numGrabber;
	auto_update_modules[j].image_xsize = auto_update_modules[j+1].image_xsize;
	auto_update_modules[j].image_ysize = auto_update_modules[j+1].image_ysize;
	auto_update_modules[j].last_image = auto_update_modules[j+1].last_image;
      }
    }
  if (!found)
    fprintf(stderr, "(%s: no auto-replies removed)\n", tcxModuleName(module));
  count_auto_update_modules();
  return found;
}
  


/************************************************************************
 *
 *   NAME:         send_automatic_image_update
 *                 
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/




void send_automatic_image_update( int numGrabber, char *pCameraImage )
{
  int i;

  for (i = 0; i < n_auto_update_modules; i++){
    if (auto_update_modules[i].image > 0){

      auto_update_modules[i].last_image++;
      if (auto_update_modules[i].last_image %
	  auto_update_modules[i].image == 0){
#ifdef CAMERA_debug	
	fprintf(stderr, "Send image from grabber %d of size %dx%d to %s.\n",
		numGrabber,
		auto_update_modules[i].image_xsize,
		auto_update_modules[i].image_ysize,
		tcxModuleName(auto_update_modules[i].module));
#endif
	auto_update_modules[i].last_image = 0;

	CAMERA_send_camera_image_to( auto_update_modules[i].module,
				     numGrabber,
				     pCameraImage,
				     auto_update_modules[i].image_xsize,
				     auto_update_modules[i].image_ysize );
      }
    }
  }
}


/************************************************************************
 *
 *   NAME:         check_commandline_parameters
 *                 
 *   FUNCTION:     checks the command line options!
 *                 
 *   PARAMETERS:   
 *                 
 *                 NOTE: "NULL" disables initialization
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void check_commandline_parameters(int argc, char **argv)
{
  int i, j, e, bug = 0;

  for (i = 1; i < argc && !bug; i++){
    for (j = 0, e = 0; j < (int) strlen(argv[i])-1; j++)
      if (argv[i][j] == '=')
	e = 1;
    if (!strcmp(argv[i], "-nodisplay") || !strcmp(argv[i], "-nd"))
      display = 0;
    else if (!strcmp(argv[i], "-notcx") || !strcmp(argv[i], "-nt"))
      tcx = 0;
    else if (!strcmp(argv[i], "-color")  || !strcmp(argv[i], "-co"))
      color = 1;
    else if (argv[i][0] != '-' || !e){
      fprintf(stderr, "Cannot parse argument: %s\n", argv[i]);
      bug = 1;
    }
  }

  if (bug){
    fprintf(stderr, "Usage: '%s [-nodisplay] [-notcx] [-color] [-fileonly]\n", 
	    argv[0]);
    exit(1);
  }

}



/************************************************************************
 *
 *   NAME: CAMERA_register_auto_update_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/




void CAMERA_register_auto_update_handler(TCX_REF_PTR  ref,
					 CAMERA_register_auto_update_ptr data)
{
  add_auto_update_module(ref->module, data);

  if (data != NULL){
    tcxFree("CAMERA_register_auto_update", data);
    data = NULL;
  }
}


/************************************************************************
 *
 *   NAME:         CAMERA_initialize_board_and_tcx
 *                 
 *   FUNCTION:     general initialization routine - must be called before
 *                 anything else. Returns error value (0=success, 1=error)
 *                 
 *   RETURN-VALUE: Returns error value (0=success, 1=error)
 *                 
 ************************************************************************/

int CAMERA_initialize_tcx()
{
  
  /* messages used by CAMERA */
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    CAMERA_messages,
#ifdef SEBASTIAN
    BASE_messages,
    PANTILT_messages
#endif
  };
  
  /* TCX */
  
  fprintf(stderr, "Connecting to TCX...");

  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize(TCX_CAMERA_MODULE_NAME, tcxMachine);
  check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
		       BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
		       NULL, 0);
  check_version_number(libezx_major, libezx_minor,
		       libezx_robot_type, libezx_date,
		       "libezx", 0);



  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));
  
  /* Handlers */
  
  tcxRegisterHandlers(CAMERA_handler_array, 
		      sizeof(CAMERA_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
#ifdef SEBASTIAN
  tcxRegisterHandlers(BASE_reply_handler_array,
		      sizeof(BASE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
    
  tcxRegisterHandlers(PANTILT_reply_handler_array,
		      sizeof(PANTILT_reply_handler_array)
			/ sizeof(TCX_REG_HND_TYPE));
#endif
  
  tcxRegisterCloseHnd(CAMERA_close_handler);
  
  /* Connections */

#ifdef SEBASTIAN
  connect_to_Base();
  connect_to_Pantilt();
#endif
  
  /* Return */
  return (1) ; 
  
}


/************************************************************************
 *
 *   NAME:         connect_to_Base
 *                 
 *   FUNCTION:     checks, and connects to BASE, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

#ifdef SEBASTIAN			/* no clue what this is for ... */

void connect_to_Base(void)
{
  struct timeval current_time;
  static struct timeval last_attempt_connect_BASE = {0, 0};


  if(!base_connected){

    gettimeofday(&current_time, NULL);
    if (current_time.tv_sec < last_attempt_connect_BASE.tv_sec + 5
	|| (current_time.tv_sec == last_attempt_connect_BASE.tv_sec + 5 &&
	    current_time.tv_usec < last_attempt_connect_BASE.tv_usec))
      return;
    
    last_attempt_connect_BASE.tv_sec  = current_time.tv_sec;
    last_attempt_connect_BASE.tv_usec = current_time.tv_usec;
    

    BASE = tcxConnectOptional(TCX_BASE_MODULE_NAME); /* checks, but does 
						      * not wait */

    if (BASE != NULL){
      base_connected = 1;
      fprintf(stderr, "TCX: connected to BASE.\n");
      tcx_base_subscribe();
    }
  }
}



/************************************************************************
 *
 *   NAME:         connect_to_Pantilt
 *                 
 *   FUNCTION:     checks, and connects to PANTILT, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void connect_to_Pantilt(void)
{
  struct timeval current_time;
  static struct timeval last_attempt_connect_PANTILT = {0, 0};


  if(!pantilt_connected){

    gettimeofday(&current_time, NULL);
    if (current_time.tv_sec < last_attempt_connect_PANTILT.tv_sec + 5
	|| (current_time.tv_sec == last_attempt_connect_PANTILT.tv_sec + 5 &&
	    current_time.tv_usec < last_attempt_connect_PANTILT.tv_usec))
      return;
    
    last_attempt_connect_PANTILT.tv_sec  = current_time.tv_sec;
    last_attempt_connect_PANTILT.tv_usec = current_time.tv_usec;
    

    PANTILT = tcxConnectOptional(TCX_PANTILT_MODULE_NAME); /* checks, but does 
						      * not wait */

    if (PANTILT != NULL){
      pantilt_connected = 1;
      fprintf(stderr, "TCX: connected to PANTILT.\n");
    }
  }
}

#endif

/************************************************************************
 *
 *   NAME:         CAMERA_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void CAMERA_close_handler(char *name, TCX_MODULE_PTR module)
{
#ifdef CAMERA_debug
  fprintf(stderr, "CAMERA: closed connection detected: %s\n", name);
#endif

  remove_auto_update_module(module);

#ifdef SEBASTIAN
  if (!strcmp(name, "BASE")){ /* BASE shut down */
    base_connected = 0;
    BASE = NULL;
    fprintf(stderr, "TCX: disconnected from BASE.\n");
  }

  if (!strcmp(name, "PANTILT")){ /* PANTILT shut down */
    pantilt_connected = 0;
    PANTILT = NULL;
    fprintf(stderr, "TCX: disconnected from PANTILT.\n");
  }
#endif

  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    commShutdown(NULL);
  }
}




#ifdef SEBASTIAN			/* no clue what this is for ... */
void PANTILT_position_reply_handler(TCX_REF_PTR                ref,
				    PANTILT_position_reply_ptr data)
{
  tcxFree("PANTILT_position_reply", data);
}

void PANTILT_init_reply_handler(TCX_REF_PTR             ref,
				int                    *data)
{
  tcxFree("PANTILT_init_reply", data);

}


void PANTILT_limits_reply_handler(TCX_REF_PTR              ref,
				  PANTILT_limits_reply_ptr data)
{
  tcxFree("PANTILT_limits_reply", data);
}



void PANTILT_status_update_handler(TCX_REF_PTR              ref,
				  PANTILT_status_update_ptr data)
{
  tcxFree("PANTILT_status_update", data);
}



void tcx_pantilt_init()
{
  int auto_reply_position = 1;

  if (pantilt_connected)
    tcxSendMsg(PANTILT, "PANTILT_init_query", &auto_reply_position);
}

/************************************************************************
 *
 *   NAME:         tcx_base_sub<scribe
 *                 
 *   FUNCTION:     subscribe for BASE messages 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void tcx_base_subscribe()
{
  BASE_register_auto_update_type data;

  data.subscribe_status_report = 1;
  data.subscribe_sonar_report  = 0;
  data.subscribe_colli_report  = 0;
  data.subscribe_ir_report     = 0;
  data.subscribe_laser_report  = 0;

  if (base_connected)
    tcxSendMsg(BASE, "BASE_register_auto_update", &data);
}




void BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
					BASE_action_executed_reply_ptr data){
;}

/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         BASE_robot_position_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				       BASE_robot_position_reply_ptr pos)
{
}



/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         BASE_update_status_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_update_status_reply_handler(TCX_REF_PTR                   ref,
				      BASE_update_status_reply_ptr  pos)
{

#ifdef CAMERA_debug
  fprintf(stderr, "TCX: Received a BASE_update_status_reply message.\n");
  fprintf(stderr, "time stamp: %g robot: %g %g %g\n", pos->time,
	  pos->pos_x, pos->pos_y, pos->orientation);
#endif
  
  tcxFree("BASE_robot_position_reply", pos);

}


#endif /* of SEBASTIAN */




/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         CAMERA_send_camera_image_to
 *                 
 *   FUNCTION:     sends a camera image to a module
 *                 
 *   PARAMETERS:   
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void CAMERA_send_camera_image_to( TCX_MODULE_PTR module,
				  int   numGrabber,
				  char *pImage,
				  int   image_xsize,
				  int   image_ysize )
{
  CAMERA_image_reply_type image;
  int i, x, y, xx, yy;
  float *R, *G, *B;
  float *c;
  float factor;

  if (image_xsize <= 0 || image_xsize > COLS ||
      image_ysize <= 0 || image_ysize > ROWS){
    fprintf(stderr, "WARNING: Invalid image size: %d %d. Request ignored.\n",
	    image_xsize, image_ysize);
  }
  else{

    image.numGrabber = numGrabber;
    image.size  = image_xsize * image_ysize;
    image.xsize = image_xsize;
    image.ysize = image_ysize;
    image.red   = (unsigned char *) malloc(sizeof(unsigned char) * image.size);
    image.green = (unsigned char *) malloc(sizeof(unsigned char) * image.size);
    image.blue  = (unsigned char *) malloc(sizeof(unsigned char) * image.size);

    R = (float *) malloc(sizeof(float) * image.size);
    G = (float *) malloc(sizeof(float) * image.size);
    B = (float *) malloc(sizeof(float) * image.size);
    c = (float *) malloc(sizeof(float) * image.size);
    
    factor = (((float) COLS) / ((float) image_xsize)) *
      (((float) ROWS) / ((float) image_ysize));

    i = 0;
    for (y = 0; y < image.ysize; y++)
      for (x = 0; x < image.xsize; x++){
	c[i] = 0.0;
	R[i] = G[i] = B[i] = 0.0;
	i++;
      }


    if (factor > 20.0){
      for (yy = 0; yy < ROWS; yy++)
	for (xx = 0; xx < COLS; xx++){
	  x = ((int) (((float) xx) * ((float) image.xsize) / ((float) COLS)));
	  y = ((int) (((float) yy) * ((float) image.ysize) / ((float) ROWS)));
	  i = y * image.xsize + x;
	  R[i] = R[i] + ((float) pImage[4*(yy*COLS+xx)+0]);
	  G[i] = G[i] + ((float) pImage[4*(yy*COLS+xx)+1]);
	  B[i] = B[i] + ((float) pImage[4*(yy*COLS+xx)+2]);
	  c[i] = c[i] + 1.0;
	  /*printf(" %d", i);*/
	}
      
      
      i = 0;
      for (y = 0; y < image.ysize; y++)
	for (x = 0; x < image.xsize; x++){
	  R[i] = R[i] / c[i];
	  G[i] = G[i] / c[i];
	  B[i] = B[i] / c[i];
	  i++;
	}
    }
    else{
      for (yy = 0; yy < ROWS; yy++)
	for (xx = 0; xx < COLS; xx++){
	  x = ((int) (((float) xx) * ((float) image.xsize) / ((float) COLS)));
	  y = ((int) (((float) yy) * ((float) image.ysize) / ((float) ROWS)));
	  i = y * image.xsize + x;
	  R[i] = ((float) pImage[4*(yy*COLS+xx)+0]);
	  G[i] = ((float) pImage[4*(yy*COLS+xx)+1]);
	  B[i] = ((float) pImage[4*(yy*COLS+xx)+2]);
	  c[i] = 1.0;
	  /*printf(" %d", i);*/
	}
   
    }
      
    i = 0;
    for (y = 0; y < image.ysize; y++)
      for (x = 0; x < image.xsize; x++){
	if (c[i] > 0.0){
	  c[i] = 1.0 / c[i];
	  image.red[i]   = ((int) (R[i] + 0.5));
	  image.green[i] = ((int) (G[i] + 0.5));
	  image.blue[i]  = ((int) (B[i] + 0.5));
	}
	i++;
      }




#ifdef old
    i = 0;
    for (y = 0; y < image.ysize; y++)
      for (x = 0; x < image.xsize; x++){
	image.red[i]   = pImage[4*(y*COLS+x)+0];
	image.green[i] = pImage[4*(y*COLS+x)+1];
	image.blue[i]  = pImage[4*(y*COLS+x)+2];
	i++;
      }
#endif    
    
#ifdef TCX_debug
    fprintf( stderr, "TCX: Sending CAMERA_image_reply from grabber %d.\n",
	    numGrabber);
#endif

    tcxSendMsg(module, "CAMERA_image_reply", &image); 
    
    free(c);
    free(B);
    free(G);
    free(R);
    free(image.blue);
    free(image.green);
    free(image.red);
  }

}

/************************************************************************
 *
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 ************************************************************************/

void CAMERA_save_handler( TCX_REF_PTR      ref,
			  CAMERA_save_ptr  data )
{

#ifdef TCX_debug
  fprintf(stderr, "Received a CAMERA_save_message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  ref = ref;			/* -Wall */

  if ( data->frames == 0 ) {

    saving[data->numGrabber]       = 0;
    savefileopen[data->numGrabber] = 0;
    saveframes[data->numGrabber]   = 0;
    sprintf( filename[data->numGrabber], "%s", data->filename );
    fclose( videofp[data->numGrabber] );

  } else if ( data->frames == -1 ) {

    saving[data->numGrabber]       = 1;
    savefileopen[data->numGrabber] = 0;
    saveframes[data->numGrabber]   = 0;
    sprintf( filename[data->numGrabber], "%s", data->filename );

  } else {

    saveframes[data->numGrabber]   = data->frames;
    saving[data->numGrabber]       = 1;
    savefileopen[data->numGrabber] = 0;
    sprintf( filename[data->numGrabber], "%s", data->filename );

  }

  tcxFree("CAMERA_save", data);

}
				 
/************************************************************************
 *
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 ************************************************************************/

void CAMERA_load_handler( TCX_REF_PTR      ref,
			  CAMERA_load_ptr  data )
{

  CAMERA_load_reply_type info;

#ifdef TCX_debug
  fprintf(stderr, "Received a CAMERA_load_message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  openAndReadFile( data->numGrabber, data->filename );

  /* finished -> send reply to module and start the grabber again */
  info.dummy = 1;
#ifdef TCX_debug
  fprintf( stderr, "TCX: Sending CAMERA_load_reply.\n");
#endif
  tcxSendMsg( ref->module, "CAMERA_load_reply", &info );

  startMeteor( data->numGrabber, 1 );

  tcxFree("CAMERA_file", data);

}

/************************************************************************
 *
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 ************************************************************************/

void CAMERA_startstop_handler( TCX_REF_PTR           ref,
			       CAMERA_startstop_ptr  data )
{

#ifdef TCX_debug
  fprintf(stderr, "Received a CAMERA_startstop_message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  ref = ref;			/* -Wall */

  if ( data->action == 1 ) {
    startMeteor( data->numGrabber, 1 );

  } else if ( data->action == 0 ) {
    stopMeteor( data->numGrabber);

  } else {
    /* bummer */
  }

  tcxFree("CAMERA_file", data);

}

/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         CAMERA_image_query_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void CAMERA_image_query_handler(TCX_REF_PTR             ref,
				CAMERA_image_query_ptr  data)
{

#ifdef TCX_debug
  fprintf(stderr, "Received a CAMERA_image_query_message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  /* this does not handle the case in which we load frames from file
     and request one of them using this function. Too bad. */

  if ( !((data->numGrabber+1)&useGrabber) && useGrabber!=0 ) {
    fprintf( stderr,"%s: numGrabber %d is NOT in use.\n",
	     __FUNCTION__, data->numGrabber );
      return;
  }

  CAMERA_send_camera_image_to( ref->module, 
			       data->numGrabber,
			       pCameraImage[data->numGrabber],
			       data->xsize,
			       data->ysize );

  tcxFree("CAMERA_image_query", data);
}


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void CAMERA_shmid_query_handler( TCX_REF_PTR             ref,
				 CAMERA_shmid_query_ptr  data)

{
  CAMERA_shmid_reply_type info;

#ifdef TCX_debug
  fprintf(stderr, "Received a CAMERA_shmid_query_message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  info.numGrabber = data->numGrabber;
  info.shmid      = shmid[data->numGrabber]; /* smid defined in grab.h */

#ifdef TCX_debug
  fprintf(stderr, "TCX: Sending CAMERA_shmid_reply.\n");
#endif

  tcxFree("CAMERA_shmid_query", data);

  tcxSendMsg( ref->module, "CAMERA_shmid_reply", &info ); 

}

/*
 * $Log: handlers.c,v $
 * Revision 1.24  1998/02/08 00:13:14  swa
 * now works with redhat5.0
 *
 * Revision 1.23  1998/01/13 00:35:14  swa
 * Added two new functions -- start and stop. They can start and stop either
 * of the two frame grabbers.
 *
 * Revision 1.22  1997/11/06 17:54:38  swa
 * Fixed a bug with TCXHOST. If the environment variable TCXHOST is set,
 * take it. If not, take the TCXHOST from beeSoft.ini. If this doesn't
 * work, take localhost. If this doesn't work: bummer.
 *
 * Revision 1.21  1997/10/04 18:01:06  swa
 * Fixed a bug in CAMERA-messages so that sending images over TCX works again.
 *
 * Revision 1.20  1997/10/04 01:06:46  swa
 * Fixed some bugs and inconsistencies wrt to both frame grabbers.
 *
 * Revision 1.19  1997/10/04 00:13:07  swa
 * First working version of the cameraServer that supports two (count'em
 * two!) framegrabbers.  Although the server seems to work just fine, it
 * has not yet been fully tested.
 *
 * Revision 1.18  1997/09/19 23:03:18  swa
 * Image-over-TCX works now correctly when using file as the source. A bunch
 * of function calls were missing in the load-file function.
 *
 * Revision 1.17  1997/08/02 18:50:54  swa
 * Support for file-only mode (SUN and Linux) added.
 *
 * Revision 1.16  1997/07/24 18:48:01  swa
 * Renamed two internal functions. Tested it again with the Tcl program. Works
 * fine. :)
 *
 * Revision 1.15  1997/07/24 00:54:25  swa
 * Fixed some minor bugs, extended the README file and tested the version. The
 * cameraAttachExample does not yet die when the cameraServer dies. Will be
 * fixed in future versions.
 *
 * Revision 1.14  1997/07/23 22:43:33  swa
 * This is the first version of camControl, a Tcl program that remotely controls
 * the cameraServer. The user can save and read frames. recorder was renamed
 * to camControl, since the program will not only record but also read frames.
 *
 * Revision 1.13  1997/07/22 22:47:16  swa
 * Added file handling (reading/saving). Renamed two (three?) functions to
 * be consistent.
 *
 * Revision 1.12  1997/07/04 17:28:31  swa
 * Left and right mouseclicks have been added to the example-program. Left
 * mouse saves the current cameraimage to a 4 byte .ppm format, right mouse
 * shows the current coordinates.
 *
 * Revision 1.11  1997/06/25 23:52:44  thrun
 * Various changes. Makde display in o-graphics much faster. changed
 * some of the parameter files to more sensible values. improves the
 * display in "learn". the commander now displays high-resolution
 * images.
 *
 * Revision 1.10  1997/06/25 18:46:50  thrun
 * Added support for tcx messages. It is for mainatt no longer necessary to
 * use argv[1]. It now sends out messages. (swa)
 *
 * Revision 1.9  1997/06/24 22:48:13  thrun
 * checking of the command line arguments.
 *
 * Revision 1.8  1997/06/24 17:05:43  thrun
 * Fixed some debug flags (swa)
 *
 * Revision 1.7  1997/06/23 23:48:43  thrun
 * shared memory and other stuff (swa)
 *
 * Revision 1.6  1997/06/23 02:36:00  thrun
 * Stefan's new camera server with shared memory
 *
 * Revision 1.5  1997/06/21 22:36:22  thrun
 * Improved interface, cleaner
 *
 * Revision 1.4  1997/06/20 01:09:29  thrun
 * Renamed function names to be more consistent. Currently the cameraServer
 * can track various object based on its color information. The color info
 * is supplied in a seperate textfile and is loaded into the server upon the
 * first request. (swa)
 *
 * Revision 1.3  1997/06/19 21:06:57  thrun
 * Added various stuff for faceInfo (callbacks, etc)
 *
 * Revision 1.2  1997/06/18 15:58:14  thrun
 * now with auto-update
 *
 * Revision 1.1.1.1  1997/06/16 22:32:20  thrun
 * New Camera Server, much faster, more reliable, works already with
 * commander.
 *
 */
