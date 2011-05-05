
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





/*#define main_defined */



/************************************************************************
 *
 *   FILENAME:         o-graphics.c
 *                 
 *   FUNCTION:         Extension of Easy-X to mobile robot control
 *                 
 *   HOW TO COMPILE:   make o-graphics
 *
 *   CREATOR:          Sebastian Thrun
 *   
 *   DATE:             
 *   
 *   GENERAL COMMENTS: Have Fun
 *
 ************************************************************************/

#ifdef VMS
#include "vms.h"
struct timeval {
        long    tv_sec;         /* seconds */
        long    tv_usec;        /* and microseconds */
};
#endif

#include <ctype.h>
#include <math.h>
#include <stdio.h>
/*#include <malloc.h>*/
#include "o-graphics.h"
#define MAX_RANDOM 1073741823.5
#define RAND() ((((float) random())/MAX_RANDOM)-1.0)
#define RAND_POS() ((((float) random())/MAX_RANDOM)/2.0)

#define pi  3.14159265358979323846
#define pi2 6.28318530717958647692


/*-------------------------------------------------------*/
/* ======== Switch object ======== */

#define max_n_texts 510


typedef struct{
  int   activated;
  float min_x, max_x, min_y, max_y, size_x, size_y;
  int   int_min_x, int_max_x, int_min_y, int_max_y;  
  char  texts[max_n_texts][64];
  int   num_texts;
  int   actual_text;
  int   background_color[max_n_texts];
  int   frame_color[max_n_texts];
  int   text_color[max_n_texts];
  int   fonts[max_n_texts];
} G_switch_type, *G_switch_ptr;



/*-------------------------------------------------------*/
/* ======== Value object ======== */

#define NO_VALUE -100000000000.0

typedef struct{
  int   activated;
  float min_x, max_x, min_y, max_y, size_x, size_y;
  int   int_min_x, int_max_x, int_min_y, int_max_y;  
  int   direction;
  float value, min_value, max_value, range_value;
  char  text[128];
  int   background_color;
  int   foreground_color;
  int   frame_color;
  int   text_color;
  int   font;
} G_value_type, *G_value_ptr;




/*-------------------------------------------------------*/
/* ======== Matrix object ======== */


typedef struct{
  int   activated;
  float min_x, max_x, min_y, max_y, size_x, size_y;
  int   int_min_x, int_max_x, int_min_y, int_max_y;  
  float *values; 
  int   *active; 
  int   *display_pos_x;
  int   *display_pos_y;
  int   display_length_x, display_length_y;
  int   size_matrix_x, size_matrix_y;
  float min_value, max_value, range_value;
  char  text[128];
  int   background_color;
  int   frame_color;
  int   text_color;
  int   font;
} G_matrix_type, *G_matrix_ptr;


/*-------------------------------------------------------*/
/* ======== Robot object ======== */


typedef struct{
  int   activated;
  float min_x, max_x, min_y, max_y, size_x, size_y;
  int   int_min_x, int_max_x, int_min_y, int_max_y;  
  float min_value_x, max_value_x, min_value_y, max_value_y; /* local coord */
  float range_value_x, range_value_y;
  float robot_x;		/* local x-coord of the robot */
  float robot_y;		/* local y-coord of the robot */
  float robot_orientation;	/* local orientation, in degrees */
  float robot_size;		/* size of the robot, in local coord. */
  int   num_sensors;		/* number sof sensors in vector */
  float max_sensors_range;	/* maximum sensor range, local coordin. */
  float *sensors;		/* <num_sensors> sensor values [0,1] */
  float *sensors_angles;	/* corresponding angles */
  char  text[128];		/* name of the robot */

  int   background_color;	/* color background rectangle */
  int   frame_color;		/* color frame rectangle */
  int   robot_color;		/* color robot interior */
  int   robot_frame_color;	/* color robot boundary and line */
  int   sensors_color;		/* color sensor beams color */
  int   sensors_max_range_color; /* color sensors max-circle color */
  int   sensors_circle_color;	/* color background circle mac-sensor-range */
  int   text_color;		/* color text color */
  int   font;
} G_robot_type, *G_robot_ptr;



/*-------------------------------------------------------*/
/* ======== Markers object ======== */

#define max_n_markers 10000

typedef struct{
  int   activated;
  float min_x, max_x, min_y, max_y, size_x, size_y;
  int   int_min_x, int_max_x, int_min_y, int_max_y;  
  float min_value_x, max_value_x, min_value_y, max_value_y; /* local coord */
  float range_value_x, range_value_y;
  float x[max_n_markers];
  float y[max_n_markers];
  int   type[max_n_markers];
  int   connected;
  float marker_size;
  int   num_markers;
  char  texts[max_n_texts][128];
  int   num_texts;
  int   background_color;
  int   frame_color;
  int   foreground_color[max_n_texts];
  int   text_color[max_n_texts];
  int   fonts[max_n_texts];
} G_markers_type, *G_markers_ptr;




/*-------------------------------------------------------*/
/*-------------------------------------------------------*/
/* ======== general window variables ======== */


int display = 1;		/* 1 if display activated, 0 if not */
int flush = 1;			/* 1 = EZXFlush, 0 not */
int window_initialized = 0;	/* 1 if window initialized (displayed) */

EZXW_p w_ptr;			/* Pointer to simulator window */

char G_name[128] = "NoName";
float G_unit_length = 200.0;	/* size of unit interval (in pixels) */
float G_margin = 20.0;          /* size of G_margin (in pixels) */
int G_background_color = C_WHITE;     


/*-------------------------------------------------------*/
/* ======== general graphics variables ======== */





#define max_num_graphic_objects 1000
     
     
typedef struct{			/* XXX add new object types here */
  G_switch_ptr  switch_ptr;
  G_value_ptr   value_ptr;
  G_matrix_ptr  matrix_ptr; 
  G_robot_ptr   robot_ptr; 
  G_markers_ptr markers_ptr; 
  int           type;
} G_all_objects_type;


G_all_objects_type  G_objects[max_num_graphic_objects];
int G_num_objects = 0;

float G_min_x     = 0.0;
float G_max_x     = 0.0;
float G_min_y     = 0.0;
float G_max_y     = 0.0;
float G_size_x    = 0.0;
float G_size_y    = 0.0;
int G_int_size_x  = 0;
int G_int_size_y  = 0;

int G_markers_display_style = 1; /* 1=fill them up */

#define max_num_fonts 30

char font_list[max_num_fonts*128] = "8x13";
int  num_fonts_defined = 1;



/*-------------------------------------------------------*/
/* ======== Mouse variables ======== */

float last_mouse_x, last_mouse_y;
int last_int_mouse_x, last_int_mouse_y;
int last_button;
int last_mouse_event_defined = 0;
int last_mouse_event_evaluated = 0;


/* typedef struct{
 *   float x, y;
 *   int button;
 *   int name;	
 *   int type;	
 *   float rel_x, rel_y;
 *   int actual_text;
 *   float value;
 *   float value_x, value_y;
 *   int active;
 *   int index_x, index_y;
 *   float robot_delta_x, robot_delta_y;
 *   float robot_delta_orientation;
 *   int marker_name;
 *   int marker_type;
 *   float marker_x, marker_y;
 * } G_mouse_type, *G_mouse_ptr;
 */


G_mouse_ptr G_mouse;
int G_mouse_allocated = 0;



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* These procedures extend EASY-Z to floating values - scaled in [0,1] */

void FLOAT_FillRectangle(EZXW_p w, float x, float y, float width, float height)
{
  EZX_FillRectangle(w, 
		    (int) ((x-G_min_x)*G_unit_length - 0.5), 
		    (int) ((G_max_y-y-height)*G_unit_length - 0.5),
		    (int) (width*G_unit_length), 
		    (int) (height*G_unit_length));
}


void FLOAT_DrawRectangle(EZXW_p w, float x, float y, float width, float height)
{
  EZX_DrawRectangle(w,
		    (int) ((x-G_min_x)*G_unit_length), 
		    (int) ((G_max_y-y-height)*G_unit_length),
		    (int) (width*G_unit_length), 
		    (int) (height*G_unit_length)); 
}

void FLOAT_Rectangle(EZXW_p w, float x, float y, float width, float height)
{
  int a, b, c, d;
  a = (int) ((x-G_min_x)*G_unit_length);
  b = (int) ((G_max_y-y-height)*G_unit_length);
  c = (int) (width*G_unit_length);
  d = (int) (height*G_unit_length);
  EZX_DrawRectangle(w, a, b, c, d);
  EZX_FillRectangle(w, a, b, c, d);
}



void FLOAT_DrawCircle(EZXW_p w, float x, float y, float r)
{
  EZX_DrawCircle(w, 
		 (int) ((x-G_min_x)*G_unit_length), 
		 (int) ((G_max_y-y)*G_unit_length),
		 (int) (r*G_unit_length));
}


void FLOAT_FillCircle(EZXW_p w, float x, float y, float r)
{
    EZX_FillCircle(w, 
		 (int) ((x-G_min_x)*G_unit_length), 
		 (int) ((G_max_y-y)*G_unit_length),
		 (int) (r*G_unit_length));
}

void FLOAT_DrawLine(EZXW_p w, float x1, float y1, float x2, float y2)
{
    EZX_DrawLine(w, 
		    (int) ((x1-G_min_x)*G_unit_length), 
		    (int) ((G_max_y-y1)*G_unit_length),
		    (int) ((x2-G_min_x)*G_unit_length), 
		    (int) ((G_max_y-y2)*G_unit_length));
}


void FLOAT_FillPolygon(EZXW_p w, int npoints, float *points_x, float *points_y)
{
  XPoint *points = NULL;
  int i;

  if (npoints <= 0 || npoints > 1000) 
    return;
  points = (XPoint *) malloc(sizeof(XPoint) * npoints);
  for (i = 0; i < npoints; i++){
    points[i].x = (short) ((points_x[i]-G_min_x)*G_unit_length);
    points[i].y = (short) ((G_max_y-points_y[i])*G_unit_length);
  }
  EZX_FillPolygon(w, npoints, points);
  free (points);
  points = NULL;
}


void FLOAT_DrawString(EZXW_p w, float x, float y, char *string)
{
  EZX_DrawString(w, 
		 (int) ((x-G_min_x)*G_unit_length), 
		 (int) ((G_max_y-y)*G_unit_length),
		 string);
}



void FLOAT_DrawStringCentered(EZXW_p w, float x, float y, char *string)
{
  EZX_DrawString(w, 
		 (int) (((x-G_min_x)*G_unit_length) 
			- (((float) EZX_GetTextWidth(string)) / 2.0)),
		 (int) (((G_max_y-y)*G_unit_length)
			+ (((float) EZX_GetFontHeight()) / 2.0)),
		 string);
}
		 





/* These procedures extend EASY-Z to integer values - with *correct* coord. */



void INT_FillRectangle(EZXW_p w, int x, int y, int width, int height)
{
  EZX_FillRectangle(w, x, G_int_size_y - y - height,
		    width, height);
}


void INT_DrawRectangle(EZXW_p w, int x, int y, int width, int height)
{
  EZX_DrawRectangle(w, x, G_int_size_y - y - height, 
		    width, height);
}

void INT_Rectangle(EZXW_p w, int x, int y, int width, int height)
{
  EZX_DrawRectangle(w, x, G_int_size_y - y - height, 
		    width, height);
  EZX_FillRectangle(w, x, G_int_size_y - y - height,
		    width, height);
}


void INT_DrawCircle(EZXW_p w, int x, int y, int r)
{
  EZX_DrawCircle(w, x, G_int_size_y - y, r);
}

void INT_FillCircle(EZXW_p w, int x, int y, int r)
{
  EZX_FillCircle(w, x, G_int_size_y - y, r);
}


void INT_DrawLine(EZXW_p w, int x1, int y1, int x2, int y2)
{
  EZX_DrawLine(w, x1, G_int_size_y - y1, x2, G_int_size_y - y2);
}


void INT_DrawString(EZXW_p w, int x, int y, char *string)
{
  EZX_DrawString(w, x, G_int_size_y - y, string);
}


void INT_DrawStringCenterd(EZXW_p w, int x, int y, char *string)
{
  EZX_DrawString(w, x - (EZX_GetTextWidth(string) / 2),
		 G_int_size_y - y + (EZX_GetFontHeight() / 2), string);
}



/************************************************************************
 *
 *   NAME:         G_initialize_graphics
 *                 
 *   FUNCTION:     initializes this program - not mandatory
 *                 
 *   PARAMETERS:   char *name            name of the window
 *                 float unit_length     number of pixels for unit interval
 *                 float margin          number of pixels G_margin
 *                 int background_color  general background color for window
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void G_initialize_graphics(char *name, 
			   float unit_length, 
			   float margin, 
			   int background_color)
{
  if (window_initialized == 1)
    printf("WARNING: graphics initialization only BEFORE first display.\n");
  else{
    if (strlen(name) < 128)
      strcpy(G_name, name);
    if (unit_length > 0.0)
      G_unit_length = unit_length;
    if (margin >= 0.0)
      G_margin = margin;
    if (background_color >= 0)
      G_background_color = background_color;
  }
}
  


/************************************************************************
 *
 *   NAME:         G_initialize_fonts
 *                 
 *   FUNCTION:     initializes fonts - up to <max_num_fonts>
 *                 does *not* check if fontnames correct
 *                 
 *   PARAMETERS:   int num_fonts      number of fonts
 *                 char *fonts[]      list of fontnames
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_initialize_fonts(int num_fonts, char *fonts[])
{
  int i;


  if (num_fonts < 1 || num_fonts > max_num_fonts)
    printf("WARNING: to few or too many fonts. Command ignored.\n");
  
  else{
    num_fonts_defined = num_fonts;
    for (i = 0; i < num_fonts; i++)
      strcpy(&(font_list[i*128]), fonts[i]); 
  }
}





/************************************************************************
 *
 *   NAME:         G_initialize_window
 *                 
 *   FUNCTION:     initializes and displays window, internal use only
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_initialize_window(void){
  int i, j, x, y;
  G_switch_ptr  SWITCH;
  G_value_ptr   VALUE;
  G_matrix_ptr  MATRIX;
  G_robot_ptr   ROBOT;
  G_markers_ptr MARKERS;

  if (display == 1 && window_initialized == 0){

    EZX_InitX(NULL, "EZX");

    /* Part 1 - compute size of the global window variables */
    
    G_min_x -= G_margin / G_unit_length;
    G_max_x += G_margin / G_unit_length;
    G_min_y -= G_margin / G_unit_length;
    G_max_y += G_margin / G_unit_length;
    G_size_x = G_max_x - G_min_x;
    /*    if (G_size_x == 0.0) G_size_x = 1.0;*/
    G_size_y = G_max_y - G_min_y;
    /*    if (G_size_y == 0.0) G_size_y = 1.0;*/
    
    G_int_size_x = (int) (G_size_x * G_unit_length);
    G_int_size_y = (int) (G_size_y * G_unit_length);
    
    /* Part 2 - compute size of the local window variables */
    
    for (i = 0; i < G_num_objects; i++){
      if (G_objects[i].type == G_SWITCH){
	SWITCH = G_objects[i].switch_ptr;
	SWITCH->int_min_x = (int) ((SWITCH->min_x - G_min_x) * G_unit_length);
	SWITCH->int_max_x = (int) ((SWITCH->max_x - G_min_x) * G_unit_length);
	SWITCH->int_min_y = (int) ((SWITCH->min_y - G_min_y) * G_unit_length);
	SWITCH->int_max_y = (int) ((SWITCH->max_y - G_min_y) * G_unit_length);
      }
      else if (G_objects[i].type == G_VALUE){
	VALUE = G_objects[i].value_ptr;
	VALUE->int_min_x = (int) ((VALUE->min_x - G_min_x) * G_unit_length);
	VALUE->int_max_x = (int) ((VALUE->max_x - G_min_x) * G_unit_length);
	VALUE->int_min_y = (int) ((VALUE->min_y - G_min_y) * G_unit_length);
	VALUE->int_max_y = (int) ((VALUE->max_y - G_min_y) * G_unit_length);
      }
      else if (G_objects[i].type == G_MATRIX){
	MATRIX = G_objects[i].matrix_ptr;
	MATRIX->int_min_x = (int) ((MATRIX->min_x - G_min_x) * G_unit_length);
	MATRIX->int_max_x = (int) ((MATRIX->max_x - G_min_x) * G_unit_length);
	MATRIX->int_min_y = (int) ((MATRIX->min_y - G_min_y) * G_unit_length);
	MATRIX->int_max_y = (int) ((MATRIX->max_y - G_min_y) * G_unit_length);
	for (x = 0; x < MATRIX->size_matrix_x; x++)
	  for (y = 0; y < MATRIX->size_matrix_y; y++){
	    MATRIX->display_pos_x[x*MATRIX->size_matrix_y+y] = 
	      (int) ((MATRIX->min_x + 
		      ((((float) x) / 
			((float) MATRIX->size_matrix_x)) 
		       * (MATRIX->max_x - MATRIX->min_x))
		      - G_min_x) * G_unit_length);
	    MATRIX->display_pos_y[x*MATRIX->size_matrix_y+y] 
	      = (int) ((MATRIX->min_y + 
			((((float) y) / 
			  ((float) MATRIX->size_matrix_y)) 
			 * (MATRIX->max_y - MATRIX->min_y))
			- G_min_y) * G_unit_length);
	  }
	MATRIX->display_length_x = (int) ((MATRIX->max_x - MATRIX->min_x) 
					  / ((float) MATRIX->size_matrix_x) 
					  * G_unit_length);
	if (MATRIX->display_length_x == 0)
	  MATRIX->display_length_x = 1;
	MATRIX->display_length_y = (int) ((MATRIX->max_y - MATRIX->min_y) 
					  / ((float) MATRIX->size_matrix_y) 
					  * G_unit_length);
	if (MATRIX->display_length_y == 0)
	  MATRIX->display_length_y = 1;
      }
      
      else if (G_objects[i].type == G_ROBOT){
	ROBOT = G_objects[i].robot_ptr;
	ROBOT->int_min_x = (int) ((ROBOT->min_x - G_min_x) * G_unit_length);
	ROBOT->int_max_x = (int) ((ROBOT->max_x - G_min_x) * G_unit_length);
	ROBOT->int_min_y = (int) ((ROBOT->min_y - G_min_y) * G_unit_length);
	ROBOT->int_max_y = (int) ((ROBOT->max_y - G_min_y) * G_unit_length);
      }
      else if (G_objects[i].type == G_MARKERS){
	MARKERS = G_objects[i].markers_ptr;
	MARKERS->int_min_x = (int) ((MARKERS->min_x - G_min_x) *G_unit_length);
	MARKERS->int_max_x = (int) ((MARKERS->max_x - G_min_x) *G_unit_length);
	MARKERS->int_min_y = (int) ((MARKERS->min_y - G_min_y) *G_unit_length);
	MARKERS->int_max_y = (int) ((MARKERS->max_y - G_min_y) *G_unit_length);
      }

      
      /* XXX add new object types here */
      else
	printf("STRANGE1: Object #%d type %d unknown!\n", 
	       i, G_objects[i].type);
    }
    
    /* Part 3 - display window */
    
    w_ptr = EZX_MakeWindow(G_name, G_int_size_x, G_int_size_y, NULL);
    /*
    if (!EZX_LoadBestColorMap(w_ptr)){
      fprintf(stderr, "O-Graphics: Error when loading colormap. Must exit.\n");
      exit(2);
    }
    theColormap   = XDefaultColormap( theDisplay, theScreen );
    EZX_InitDefaultColors();
    */
    EZX_Flush();
    EZX_UseFont(theGC, &(font_list[0*128])); /* (st) 94-6-10. Looks superfluous */
    if (G_background_color != NO_COLOR){
      EZX_SetColor(G_background_color);
      INT_Rectangle(w_ptr, 0, 0, G_int_size_x, G_int_size_y); 
    }
    EZX_Flush();
    window_initialized = 1;
  }
}







/************************************************************************
 *
 *   NAME:         G_create_switch_object
 *                 
 *   FUNCTION:     Creates a new switch object
 *                 
 *   PARAMETERS:   float position[4]       position of the object: 
 *                                                 min_x, max_x, min_y, max_y
 *                 int num_texts           number of strings displayed
 *                                                 in this switch field
 *                 char *texts[]           strings (texts)
 *                 int *background_color   vector of background colors
 *                 int *frame_color        vector of frame colors
 *                 int *text_color         vector of text colors
 *                 int *fonts              vector of font-numbers
 *                 
 *   RETURN-VALUE: int switch              pointer to this graphics object
 *                 
 ************************************************************************/


int G_create_switch_object(float position[4], 
			   int num_texts, 
			   char *texts[],
			   int *background_color, 
			   int *frame_color, 
			   int *text_color, 
			   int *fonts)
{     

  int i;
  G_switch_ptr G;



  if (G_num_objects == max_num_graphic_objects)
    printf("ERROR: graphics object memory overflow!\n");
  else{

  /* ------ ALLOCATE NEW MEMORY --------- */

    G = (G_switch_ptr) (malloc(sizeof(G_switch_type)));
    /*XXX printf(" -M80- "); */
    if (G == NULL){
      printf("ABORT: out of memory!\n");
      exit(1);
    }
    
    /* ------ COPY SPECIFICATIONS --------- */

   
    G->min_x = position[0];
    G->max_x = position[1];
    G->min_y = position[2];
    G->max_y = position[3];
    G->num_texts = num_texts;
    if (G->num_texts > max_n_texts){
      G->num_texts = max_n_texts;
      printf("WARNING: num_texts=%d exceeds limit %d.\n", 
	     num_texts, max_n_texts);
    }
    for (i = 0; i < G->num_texts; i++){
      strcpy(G->texts[i], texts[i]); 
      G->background_color[i] = background_color[i];
      G->frame_color[i] = frame_color[i];
      G->text_color[i] = text_color[i];
      G->fonts[i] = fonts[i];
      if (fonts[i] < 0 || fonts[i] >= num_fonts_defined){
	printf("WARNING: font type %d not defined!\n", fonts[i]);
	G->fonts[i] = 0;
      }
    }
    G->actual_text = 0;


    /* ------ CHANGE LOCAL VARIABLES --------- */

    G->size_x = G->max_x - G->min_x;
    if (G->size_x <= 0.0) G->size_x = 1.0;
    G->size_y = G->max_y - G->min_y;
    if (G->size_y <= 0.0) G->size_y = 1.0;
    
    G->activated = 1;

    /* ------ CHANGE GLOBAL VARIABLES --------- */
    
    G_objects[G_num_objects].switch_ptr = G;
    G_objects[G_num_objects].type       = G_SWITCH;
    
    if (G->min_x < G_min_x || G_num_objects == 0)
      G_min_x = G->min_x;
    if (G->max_x > G_max_x || G_num_objects == 0)
      G_max_x = G->max_x;
    if (G->min_y < G_min_y || G_num_objects == 0)
      G_min_y = G->min_y;
    if (G->max_y > G_max_y || G_num_objects == 0)
      G_max_y = G->max_y;
    
    G_num_objects++;
    
    return (G_num_objects-1);
  }
  return -1;
}



/************************************************************************
 *
 *   NAME:         G_create_value_object
 *                 
 *   FUNCTION:     Creates a new value object
 *                 
 *   PARAMETERS:   float position[4]       position of the object: 
 *                                                 min_x, max_x, min_y, max_y
 *                 
 *                 char *text              title text of the graphics object
 *                 int direction           direction of the value-variable
 *                                         display: 1=along x-axis, 2=along
 *                                         y-axis
 *                 float value             initial value itself
 *                 float min_value, max_value   bounding interval for value
 *                 int colors[4]           vector of colors:
 *                                            - background
 *                                            - foreground
 *                                            - frame
 *                                            - text
 *                 int font                number of font
 *                 
 *   RETURN-VALUE: int value               pointer to this graphics object
 *                 
 ************************************************************************/


int G_create_value_object(float position[4],
			  char *text,
			  int direction,
			  float value, 
			  float min_value, 
			  float max_value,
			  int colors[4], 
			  int font)
{
  G_value_ptr G;

  if (G_num_objects == max_num_graphic_objects)
    printf("ERROR: graphics object memory overflow!\n");

  else{

  /* ------ ALLOCATE NEW MEMORY --------- */

    G = (G_value_ptr) (malloc(sizeof(G_value_type)));
    /*XXX printf(" -M81- "); */
    if (G == NULL){
      printf("ABORT: out of memory!\n");
      exit(1);
    }
    
    /* ------ COPY SPECIFICATIONS --------- */

    G->min_x = position[0];
    G->max_x = position[1];
    G->min_y = position[2];
    G->max_y = position[3];
    strcpy(G->text, text); 
    G->direction = direction;
    G->value = value;
    G->min_value = min_value;
    G->max_value =  max_value;
    G->background_color = colors[0];
    G->foreground_color = colors[1];
    G->frame_color      = colors[2];
    G->text_color       = colors[3];
    G->font = font;
    if (font < 0 || font >= num_fonts_defined){
      printf("WARNING: font type %d not defined!\n", font);
      G->font = 0;
    }

    /* ------ CHANGE LOCAL VARIABLES --------- */
    
    G->size_x = G->max_x - G->min_x;
    if (G->size_x <= 0.0) G->size_x = 1.0;
    G->size_y = G->max_y - G->min_y;
    if (G->size_y <= 0.0) G->size_y = 1.0;

    G->range_value = G->max_value - G->min_value;
    if (G->range_value == 0.0) G->range_value = 1.0; /* serves as divisor */

    if (G->direction != 1 && G->direction != 2){
      printf("WARNING: wrong direction encountered.\n");
      G->direction = 1;
    }

    G->activated = 1;

    /* ------ CHANGE GLOBAL VARIABLES --------- */
    
    G_objects[G_num_objects].value_ptr = G;
    G_objects[G_num_objects].type       = G_VALUE;
    
    if (G->min_x < G_min_x || G_num_objects == 0)
      G_min_x = G->min_x;
    if (G->max_x > G_max_x || G_num_objects == 0)
      G_max_x = G->max_x;
    if (G->min_y < G_min_y || G_num_objects == 0)
      G_min_y = G->min_y;
    if (G->max_y > G_max_y || G_num_objects == 0)
      G_max_y = G->max_y;
    
    G_num_objects++;
    
    return (G_num_objects-1);
  }
  return -1;
}




/************************************************************************
 *
 *   NAME:         G_create_matrix_object
 *                 
 *   FUNCTION:     Creates a new matrix object
 *                 
 *   PARAMETERS:   float position[4]       position of the object: 
 *                                                 min_x, max_x, min_y, max_y
 *                 
 *                 char *text              title text of the graphics object
 *                 float *values           pointer to matrix array. For 
 *                                         efficiency reasons, matrix must have
 *                                         a given size.
 *                 int *active             pointer to array of
 *                                         integer values: 0 = matrix element
 *                                         inactive, 1 = active
 *                 int   size_matrix_x, size_matrix_y;  virtual dimension of
 *                                         the matrix
 *                 float min_value, max_value  scaling factors for the
 *                                         values in the matrix
 *                 int colors[3]           vector of colors:
 *                                            - background
 *                                            - frame
 *                                            - text
 *                 int font                number of font
 *                 
 *   RETURN-VALUE: int matrix              pointer to this graphics object
 *                 
 ************************************************************************/





int G_create_matrix_object(float  position[4],
 			   char   *text,
			   float  *values,
			   int    *active,
			   int    size_matrix_x, 
			   int    size_matrix_y,
			   float  min_value, 
			   float  max_value,
			   int    colors[3],
			   int    font)
{
  G_matrix_ptr G;

  if (G_num_objects == max_num_graphic_objects)
    printf("ERROR: graphics object memory overflow!\n");

  else if (size_matrix_x < 1 || size_matrix_y < 1)
    printf("ERROR: matrix too small!\n");

  else{

  /* ------ ALLOCATE NEW MEMORY --------- */

    G = (G_matrix_ptr) (malloc(sizeof(G_matrix_type)));
    /*XXX printf(" -M82- "); */
    if (G == NULL){
      printf("ABORT: out of memory!\n");
      exit(1);
    }
    

    G->display_pos_x = (int *) (malloc(sizeof(int) * size_matrix_x *
				       size_matrix_y));
    /*XXX printf(" -M83- "); */
    G->display_pos_y = (int *) (malloc(sizeof(int) * size_matrix_x *
				       size_matrix_y));
    /*XXX printf(" -M84- "); */
    if (G->display_pos_x == NULL || G->display_pos_y == NULL){
      printf("ABORT: out of memory!\n");
      exit(1);
    }

    /* ------ COPY SPECIFICATIONS --------- */

    G->min_x = position[0];
    G->max_x = position[1];
    G->min_y = position[2];
    G->max_y = position[3];
    strcpy(G->text, text); 
    G->values = values; 
    G->active = active; 
    G->size_matrix_x = size_matrix_x;
    G->size_matrix_y = size_matrix_y;
    G->min_value = min_value;
    G->max_value = max_value;
    G->background_color = colors[0];
    G->frame_color      = colors[1];
    G->text_color       = colors[2];
    G->font = font;
    if (font < 0 || font >= num_fonts_defined){
      printf("WARNING: font type %d not defined!\n", font);
      G->font = 0;
    }

    /* ------ CHANGE LOCAL VARIABLES --------- */
    
    G->size_x = G->max_x - G->min_x;
    if (G->size_x <= 0.0) G->size_x = 1.0;
    G->size_y = G->max_y - G->min_y;
    if (G->size_y <= 0.0) G->size_y = 1.0;

    G->range_value = G->max_value - G->min_value;
    if (G->range_value == 0.0) G->range_value = 1.0; /* serves as divisor */

    G->activated = 1;


    /* ------ CHANGE GLOBAL VARIABLES --------- */
    
    G_objects[G_num_objects].matrix_ptr = G;
    G_objects[G_num_objects].type       = G_MATRIX;
    
    if (G->min_x < G_min_x || G_num_objects == 0)
      G_min_x = G->min_x;
    if (G->max_x > G_max_x || G_num_objects == 0)
      G_max_x = G->max_x;
    if (G->min_y < G_min_y || G_num_objects == 0)
      G_min_y = G->min_y;
    if (G->max_y > G_max_y || G_num_objects == 0)
      G_max_y = G->max_y;
    
    G_num_objects++;
    
    return (G_num_objects-1);
  }
  return -1;
}





/************************************************************************
 *
 *   NAME:         G_change_matrix
 *                 
 *   FUNCTION:     Changes matrix pointers and size.
 *                 
 *   PARAMETERS:   int name                name of the matrix
 *                 float *values           pointer to matrix array. For 
 *                                         efficiency reasons, matrix must have
 *                                         a given size.
 *                 int *active             pointer to array of
 *                                         integer values: 0 = matrix element
 *                                         inactive, 1 = active
 *                 int size_matrix_x, size_matrix_y; new dimension.
 *                                         set to "-1" of unchanged
 *                 
 *   RETURN-VALUE: int matrix              pointer to this graphics object
 *                 
 ************************************************************************/





void G_change_matrix(int    name, 
		     float *values, 
		     int   *active,
		     int    size_matrix_x, 
		     int    size_matrix_y)
{
  G_matrix_ptr G;
  int prev_size_matrix_x, prev_size_matrix_y, x, y;

  if (name < 0 || name >= G_num_objects)
    printf("ERROR: object %d not defined in G_change_matrix.\n", name);
  
  else if(G_objects[name].type != G_MATRIX)
    printf("ERROR: object %d is not a matrix object.\n", name);

  else{

    /*
     * Update matrix pointers
     */

    G = G_objects[name].matrix_ptr;
    G->values = values; 
    G->active = active; 


    /*
     * Update size information
     */

    prev_size_matrix_x = G->size_matrix_x;
    prev_size_matrix_y = G->size_matrix_y;
    if (size_matrix_x >= 0)
      G->size_matrix_x = size_matrix_x;
    if (size_matrix_y >= 0)
      G->size_matrix_y = size_matrix_y;

    /*
     * Now compute new display coordinates, if the matrix size changed. 
     */

    if (prev_size_matrix_x != G->size_matrix_x ||
	prev_size_matrix_y != G->size_matrix_y){
      if (prev_size_matrix_x * prev_size_matrix_y <
	  G->size_matrix_x * G->size_matrix_y){
	free(G->display_pos_x);
	free(G->display_pos_y);
	G->display_pos_x = (int *) (malloc(sizeof(int) * size_matrix_x *
					   size_matrix_y));
	G->display_pos_y = (int *) (malloc(sizeof(int) * size_matrix_x *
					 size_matrix_y));
      }

      for (x = 0; x < G->size_matrix_x; x++)
	for (y = 0; y < G->size_matrix_y; y++){
	  G->display_pos_x[x*G->size_matrix_y+y] = 
	    (int) ((G->min_x + 
		    ((((float) x) / 
		      ((float) G->size_matrix_x)) 
		     * (G->max_x - G->min_x))
		    - G_min_x) * G_unit_length);
	  G->display_pos_y[x*G->size_matrix_y+y] 
	    = (int) ((G->min_y + 
		      ((((float) y) / 
			((float) G->size_matrix_y)) 
		       * (G->max_y - G->min_y))
			- G_min_y) * G_unit_length);
	}
      G->display_length_x = (int) ((G->max_x - G->min_x) 
				   / ((float) G->size_matrix_x) 
				   * G_unit_length);
      if (G->display_length_x == 0)
	G->display_length_x = 1;

      G->display_length_y = (int) ((G->max_y - G->min_y) 
				   / ((float) G->size_matrix_y) 
				   * G_unit_length);
      if (G->display_length_y == 0)
	G->display_length_y = 1;
    }
  }
}




/************************************************************************
 *
 *   Name:         G_create_robot_object
 *                 
 *   FUNCTION:     Creates a new robot object
 *                 
 *   PARAMETERS:   float position[4]       position of the object: 
 *                 char *text              title text of the graphics object
 *                 float min_value_x, max_value_x,
 *                       min_value_y, max_value_y  defines local coordinate
 *                                         system for valid robot/sensor
 *                                         positions/values
 *                 float robot_x, robot_y  initial robot position (local coord)
 *                 float robot_orientation initial orientation (local coord)
 *                 float robot_size        robot's size (local coord)
 *                 int num_sensors         length of the sensor vector (number)
 *                 float max_sensors_range Max. sensor range (local coord)
 *                 float *sensors          Initial values of sensors
 *                 float *sensors_angles   Angles of the sensors
 *                 int colors[8]           vector of colors:
 *                                            - background
 *                                            - frame
 *                                            - robot
 *                                            - robot_frame
 *                                            - sensors
 *                                            - sensors_max_range_circle
 *                                            - sensors_circle
 *                                            - text
 *                 int font                number of font
 *                 
 *   RETURN-VALUE: int robot              pointer to this graphics object
 *                 
 ************************************************************************/


int G_create_robot_object(float position[4],
			  char *text,
			  float min_value_x, 
			  float max_value_x,  
			  float min_value_y,  
			  float max_value_y,
			  float robot_x,  
			  float robot_y,  
			  float robot_orientation,  
			  float robot_size,
			  int num_sensors,
			  float max_sensors_range,
			  float *sensors,
			  float *sensors_angles,
			  int colors[8],
			  int font)
{
  int i;
  G_robot_ptr G;

  if (G_num_objects == max_num_graphic_objects)
    printf("ERROR: graphics object memory overflow!\n");
  else{

  /* ------ ALLOCATE NEW MEMORY --------- */

    G = (G_robot_ptr) (malloc(sizeof(G_robot_type)));
    /*XXX printf(" -M85- "); */
    if (G == NULL){
      printf("ABORT: out of memory!\n");
      exit(1);
    }
    
    /* ------ COPY SPECIFICATIONS --------- */

   
    G->min_x = position[0];
    G->max_x = position[1];
    G->min_y = position[2];
    G->max_y = position[3];
    G->min_value_x = min_value_x;
    G->max_value_x = max_value_x;
    G->min_value_y = min_value_y;
    G->max_value_y = max_value_y;
    G->robot_x = robot_x;
    G->robot_y = robot_y;
    G->robot_orientation = robot_orientation;
    G->robot_size = robot_size;
    G->num_sensors = num_sensors;
    G->max_sensors_range = max_sensors_range;
    strcpy(G->text, text); 
    G->background_color        = colors[0];
    G->frame_color             = colors[1];
    G->robot_color             = colors[2];
    G->robot_frame_color       = colors[3];
    G->sensors_color           = colors[4];
    G->sensors_max_range_color = colors[5];
    G->sensors_circle_color    = colors[6];
    G->text_color              = colors[7];
    G->font = font;
    if (font < 0 || font >= num_fonts_defined){
      printf("WARNING: font type %d not defined!\n", font);
      G->font = 0;
    }

    G->sensors        = (float *) (malloc(sizeof(float) * num_sensors));
    /*XXX printf(" -M86- "); */
    G->sensors_angles = (float *) (malloc(sizeof(float) * num_sensors));
    /*XXX printf(" -M87- "); */
    if (G->sensors == NULL || G->sensors_angles == NULL){
      printf("ABORT: out of memory!\n");
      exit(1);
    }
    for (i = 0; i < num_sensors; i++){
      G->sensors[i] = sensors[i];
      G->sensors_angles[i] = sensors_angles[i];
    }



    /* ------ CHANGE LOCAL VARIABLES --------- */

    G->size_x = G->max_x - G->min_x;
    if (G->size_x <= 0.0) G->size_x = 1.0;
    G->size_y = G->max_y - G->min_y;
    if (G->size_y <= 0.0) G->size_y = 1.0;

    G->range_value_x = G->max_value_x - G->min_value_x;
    if (G->range_value_x == 0.0) G->range_value_x = 1.0; /* serves as divisor*/
    G->range_value_y = G->max_value_y - G->min_value_y;
    if (G->range_value_y == 0.0) G->range_value_y = 1.0; /* serves as divisor*/

    G->activated = 1;


    /* ------ CHANGE GLOBAL VARIABLES --------- */
    
    G_objects[G_num_objects].robot_ptr = G;
    G_objects[G_num_objects].type       = G_ROBOT;
    
    if (G->min_x < G_min_x || G_num_objects == 0)
      G_min_x = G->min_x;
    if (G->max_x > G_max_x || G_num_objects == 0)
      G_max_x = G->max_x;
    if (G->min_y < G_min_y || G_num_objects == 0)
      G_min_y = G->min_y;
    if (G->max_y > G_max_y || G_num_objects == 0)
      G_max_y = G->max_y;
    
    G_num_objects++;
    
    return (G_num_objects-1);
  }
  return -1;
}




/************************************************************************
 *
 *   NAME:         G_create_markers_object
 *                 
 *   FUNCTION:     Creates a new markers object
 *                 
 *   PARAMETERS:   float position[4]       position of the object: 
 *                                                 min_x, max_x, min_y, max_y
 *                 int connected          1=markers be connected by line, 0else
 *                 float marker_size      size of the marker
 *                 float min_value_x, max_value_x,
 *                       min_value_y, max_value_y  defines local coordinate
 *                                         system for valid marker values
 *                 int num_texts           number of marker names
 *                 char *texts[]           strings (texts)
 *                 int background_color    background color
 *                 int frame_color         frame color
 *                 int *foreground_color   vector of foreground colors
 *                 int *text_color         vector of text colors
 *                 int *fonts              vector of font-numbers
 *                 
 *   RETURN-VALUE: int markers              pointer to this graphics object
 *                 
 ************************************************************************/




int G_create_markers_object(float position[4],
			    int   connected,
			    float marker_size,
			    float min_value_x, 
			    float max_value_x, 
			    float min_value_y, 
			    float max_value_y,
			    int   num_texts,
			    char *texts[],
			    int   background_color,
			    int   frame_color,
			    int  *foreground_color, 
			    int  *text_color,
			    int  *fonts)
{     

  int i;
  G_markers_ptr G;


  if (G_num_objects == max_num_graphic_objects)
    printf("ERROR: graphics object memory overflow!\n");
  else{

  /* ------ ALLOCATE NEW MEMORY --------- */

    G = (G_markers_ptr) (malloc(sizeof(G_markers_type)));
    /*XXX printf(" -M88- "); */
    if (G == NULL){
      printf("ABORT: out of memory!\n");
      exit(1);
    }
    
    /* ------ COPY SPECIFICATIONS --------- */

   
    G->min_x            = position[0];
    G->max_x            = position[1];
    G->min_y            = position[2];
    G->max_y            = position[3];
    G->connected        = connected;
    G->marker_size      = marker_size;
    G->min_value_x      = min_value_x;
    G->max_value_x      = max_value_x;
    G->min_value_y      = min_value_y;
    G->max_value_y      = max_value_y;
    G->background_color = background_color;
    G->frame_color      = frame_color;
    G->num_texts        = num_texts;
    if (G->num_texts > max_n_texts){
      G->num_texts = max_n_texts;
      printf("WARNING: num_texts=%d exceeds limit %d.\n", 
	     num_texts, max_n_texts);
    }
    for (i = 0; i < num_texts; i++){
      strcpy(G->texts[i], texts[i]); 
      G->foreground_color[i] = foreground_color[i];
      G->text_color[i] = text_color[i];
      G->fonts[i] = fonts[i];
      if (fonts[i] < 0 || fonts[i] >= num_fonts_defined){
	printf("WARNING: font type %d not defined!\n", fonts[i]);
	G->fonts[i] = 0;
      }
    }


    /* ------ CHANGE LOCAL VARIABLES --------- */

    G->num_markers      = 0;

    G->size_x = G->max_x - G->min_x;
    if (G->size_x <= 0.0) G->size_x = 1.0;
    G->size_y = G->max_y - G->min_y;
    if (G->size_y <= 0.0) G->size_y = 1.0;
    
    G->range_value_x = G->max_value_x - G->min_value_x;
    if (G->range_value_x == 0.0) G->range_value_x = 1.0; /* serves as divisor*/
    G->range_value_y = G->max_value_y - G->min_value_y;
    if (G->range_value_y == 0.0) G->range_value_y = 1.0; /* serves as divisor*/

    G->activated = 1;

    /* ------ CHANGE GLOBAL VARIABLES --------- */
    
    G_objects[G_num_objects].markers_ptr = G;
    G_objects[G_num_objects].type       = G_MARKERS;
    
    if (G->min_x < G_min_x || G_num_objects == 0)
      G_min_x = G->min_x;
    if (G->max_x > G_max_x || G_num_objects == 0)
      G_max_x = G->max_x;
    if (G->min_y < G_min_y || G_num_objects == 0)
      G_min_y = G->min_y;
    if (G->max_y > G_max_y || G_num_objects == 0)
      G_max_y = G->max_y;
    
    G_num_objects++;
    
    return (G_num_objects-1);
  }
  return -1;
}



/************************************************************************
 *
 *   NAME:         G_shift_robot_local_coordinates
 *                 
 *   FUNCTION:     special function: changes local coordinate frame of robot
 *                 
 *   PARAMETERS:   int name          name of robot object
 *                 float shift_x     change in x direction
 *                 float shift_y     change in y direction
 *                 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void G_shift_robot_local_coordinates(int name, float shift_x, float shift_y)
{
  G_robot_ptr G;

  if (name < 0 || name >= G_num_objects)
    printf("ERROR: object %d not defined in G_shift_robot_local_coordinates.\n", name);
  
  else if(G_objects[name].type != G_ROBOT)
    printf("ERROR: object %d is not a robot object.\n", name);

  else{
    G = G_objects[name].robot_ptr;
    
    G->min_value_x -= shift_x;
    G->max_value_x -= shift_x;
    G->min_value_y -= shift_y;
    G->max_value_y -= shift_y;
  }
}




/************************************************************************
 *
 *   NAME:         G_shift_markers_local_coordinates
 *                 
 *   FUNCTION:     special function: changes local coordinate frame of markers
 *                 
 *   PARAMETERS:   int name          name of markers object
 *                 float shift_x     change in x direction
 *                 float shift_y     change in y direction
 *                 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void G_shift_markers_local_coordinates(int name, float shift_x, float shift_y)
{
  G_markers_ptr G;

  if (name < 0 || name >= G_num_objects)
    printf("ERROR: object %d not defined in G_shift_markers_local_coordinates.\n", name);
  
  else if(G_objects[name].type != G_MARKERS)
    printf("ERROR: object %d is not a markers object.\n", name);

  else{
    G = G_objects[name].markers_ptr;
    
    G->min_value_x -= shift_x;
    G->max_value_x -= shift_x;
    G->min_value_y -= shift_y;
    G->max_value_y -= shift_y;
  }
}



/************************************************************************
 *
 *   NAME:         G_clear_markers
 *                 
 *   FUNCTION:     resets markers object
 *                 
 *   PARAMETERS:   int name          name of markers object
 *                 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void G_clear_markers(int name)
{
  G_markers_ptr G;

  if (name < 0 || name >= G_num_objects)
    printf("ERROR: object %d not defined in G_clear_markers.\n", name);
  
  else if(G_objects[name].type != G_MARKERS)
    printf("ERROR: object %d is not a markers object.\n", name);

  else{
    G = G_objects[name].markers_ptr;
    
    G->num_markers = 0;
  }
}




/************************************************************************
 *
 *   NAME:         G_object_type
 *                 
 *   FUNCTION:     returns type of an object
 *                 
 *   PARAMETERS:   int name 
 *                 
 *   RETURN-VALUE: int type     G_NO_OBJECT, if undefined
 *                 
 ************************************************************************/


int G_object_type(int name)
{

  if (name < 0 || name >= G_num_objects) /* object not defined */
    return G_NO_OBJECT;
  else
    return G_objects[name].type;
}



/************************************************************************
 *
 *   NAME:         G_display_switch
 *                 
 *   FUNCTION:     Displays a switch window and changes actual_text, if
 *                 selected
 *                 
 *   PARAMETERS:   int name         pointer to the object to display 
 *                 int actual_text  new actual text, -1 = no change
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_display_switch(int name, int actual_text)
{
  G_switch_ptr G;

  if (name < 0 || name >= G_num_objects)
    printf("ERROR: object %d in G_display_switch not defined.\n", name);
  
  else if(G_objects[name].type != G_SWITCH)
    printf("ERROR: object %d in G_display_switch no switch object.\n", name);
    
  else if (display == 1){
    if (window_initialized == 0)
      G_initialize_window();
    
    G = G_objects[name].switch_ptr;
    if (actual_text >= 0)
      G->actual_text = actual_text % G->num_texts;
    

    if (G->activated == 1){
      
      if (G->background_color[G->actual_text] != NO_COLOR){
	EZX_SetColor(G->background_color[G->actual_text]);
	INT_Rectangle(w_ptr, 
		      G->int_min_x, G->int_min_y, 
		      G->int_max_x - G->int_min_x,
		      G->int_max_y - G->int_min_y);
      }
      if (G->frame_color[G->actual_text] != NO_COLOR){
	EZX_SetColor(G->frame_color[G->actual_text]);
	EZX_SetLineWidth(2);
	INT_DrawLine(w_ptr,
		     G->int_max_x,
		     G->int_max_y,
		     G->int_max_x,
		     G->int_min_y);
	INT_DrawLine(w_ptr,
		     G->int_max_x,
		     G->int_min_y,
		     G->int_min_x,
		     G->int_min_y);
	if (G->frame_color[G->actual_text] >= C_GREY5 &&
	     G->frame_color[G->actual_text] <= C_GREY100)
	  EZX_SetColor(C_GREY100 + C_GREY5 - G->frame_color[G->actual_text]);
	if (G->frame_color[G->actual_text] >= C_WHITE &&
	     G->frame_color[G->actual_text] <= C_BLACK)
	  EZX_SetColor(C_BLACK + C_WHITE - G->frame_color[G->actual_text]);
	INT_DrawLine(w_ptr,
		     G->int_min_x,
		     G->int_min_y,
		     G->int_min_x,
		     G->int_max_y);
	INT_DrawLine(w_ptr,
		     G->int_min_x,
		     G->int_max_y,
		     G->int_max_x,
		     G->int_max_y);
	EZX_SetLineWidth(1);
	/* INT_DrawRectangle(w_ptr, 
	   G->int_min_x, G->int_min_y, 
	   G->int_max_x - G->int_min_x,
	   G->int_max_y - G->int_min_y); */
      }
      if (G->text_color[G->actual_text] != NO_COLOR){
	EZX_SetColor(G->text_color[G->actual_text]);
	EZX_UseFont(theGC, &(font_list[G->fonts[G->actual_text]*128]));
	INT_DrawString(w_ptr, 
		       (G->int_max_x + G->int_min_x 
			- EZX_GetTextWidth(G->texts[G->actual_text])) / 2,
		       (G->int_max_y + G->int_min_y 
			- EZX_GetFontHeight()) / 2,
		       G->texts[G->actual_text]); 
      }
      if (flush) EZX_Flush();
    }
  }
}  







/************************************************************************
 *
 *   NAME:         G_display_value
 *                 
 *   FUNCTION:     Displays a value window and changes
 *                 
 *   PARAMETERS:   int name         pointer to the object to display 
 *                 float value      new value  (only updated, if in
 *                                     [min_value, max_value]
 *                 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_display_value(int name, float value)
{
  G_value_ptr G;


  if (name < 0 || name >= G_num_objects)
    printf("ERROR: object %d in G_display_value not defined.\n", name);
  
  else if(G_objects[name].type != G_VALUE)
    printf("ERROR: object %d in G_display_value no value object.\n", name);
    
  else if (display == 1){
    if (window_initialized == 0)
      G_initialize_window();
    
    G = G_objects[name].value_ptr;
    if (value >= G->min_value && value <= G->max_value)
      G->value = value;

    if (G->activated == 1){
      
      if (G->background_color != NO_COLOR){
	EZX_SetColor(G->background_color);
	INT_Rectangle(w_ptr, 
		      G->int_min_x, G->int_min_y, 
		      G->int_max_x - G->int_min_x,
		      G->int_max_y - G->int_min_y);
      }
      
      if (G->foreground_color != NO_COLOR){ /* draw value itself */
	EZX_SetColor(G->foreground_color);
	if (G->value >= G->min_value && G->value <= G->max_value){
	  if (G->direction == 1)
	    INT_FillRectangle(w_ptr, G->int_min_x, G->int_min_y, 
			      (int) ((G->value - G->min_value) 
				     * G_unit_length
				     / G->range_value * G->size_x), 
				G->int_max_y - G->int_min_y);
	  else
	    INT_FillRectangle(w_ptr, G->int_min_x, G->int_min_y, 
				G->int_max_x - G->int_min_x,
				(int) ((G->value - G->min_value) 
				       * G_unit_length
				/ G->range_value * G->size_y));
/*	  if (G->direction == 1)
	    FLOAT_FillRectangle(w_ptr, G->min_x, G->min_y, 
				(G->value - G->min_value) 
				/ G->range_value * G->size_x, 
				G->max_y - G->min_y);
	  else
	    FLOAT_FillRectangle(w_ptr, G->min_x, G->min_y, 
				G->max_x - G->min_x ,
				(G->value - G->min_value) 
				/ G->range_value * G->size_y);*/
	}
	else
	  printf("WARNING: value is outside its range - no display.\n");
      }
      
      if (G->frame_color != NO_COLOR){
	EZX_SetColor(G->frame_color);
	EZX_SetLineWidth(2);
	INT_DrawLine(w_ptr,
		     G->int_max_x,
		     G->int_max_y,
		     G->int_max_x,
		     G->int_min_y);
	INT_DrawLine(w_ptr,
		     G->int_max_x,
		     G->int_min_y,
		     G->int_min_x,
		     G->int_min_y);
	if (G->frame_color >= C_GREY5 &&
	     G->frame_color <= C_GREY100)
	  EZX_SetColor(C_GREY100 + C_GREY5 - G->frame_color);
	if (G->frame_color >= C_WHITE &&
	     G->frame_color <= C_BLACK)
	  EZX_SetColor(C_BLACK + C_WHITE - G->frame_color);
	INT_DrawLine(w_ptr,
		     G->int_min_x,
		     G->int_min_y,
		     G->int_min_x,
		     G->int_max_y);
	INT_DrawLine(w_ptr,
		     G->int_min_x,
		     G->int_max_y,
		     G->int_max_x,
		     G->int_max_y);
	EZX_SetLineWidth(1);

	/* INT_DrawRectangle(w_ptr, 
			  G->int_min_x, G->int_min_y, 
			  G->int_max_x - G->int_min_x,
			  G->int_max_y - G->int_min_y);*/
      }
      
      if (G->text_color != NO_COLOR){
	EZX_SetColor(G->text_color);
	EZX_UseFont(theGC, &(font_list[G->font*128]));
	INT_DrawString(w_ptr, 
		       (G->int_max_x + G->int_min_x 
			- EZX_GetTextWidth(G->text)) / 2,
		       (G->int_max_y + G->int_min_y 
			- EZX_GetFontHeight()) / 2,
		       G->text); 
      }
      if (flush) EZX_Flush();
    }
  }
}  






/************************************************************************
 *
 *   NAME:         G_display_partial_matrix
 *                 
 *   FUNCTION:     Displays a sub-matrix window
 *                 
 *   PARAMETERS:   int name         pointer to the object to display 
 *                 int from_x       first element to display in x-direction
 *                 int num_x        number of elemtns in x direction
 *                 int from_y       first element to display in y-direction
 *                 int num_y        number of elemtns in y direction
 *                 
 *   NOTE:         If from_x or from_y is negative, the whole matrix will
 *                 be displayed.
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

int G_matrix_grey_scale = 0;	/* Shall real-gray output be produced? 
				 * If not, the output will be black and
				 * white boxes, and the size indicates the
				 * magnitude of the value. These diagrams
				 * are sometimes referred to as Hinton-
				 * diagrams.*/

int G_invert_display = 0;	/* inverts the grey-scale display */



void G_display_partial_matrix(int name, 
			      int from_x, int num_x, 
			      int from_y, int num_y)
{
  G_matrix_ptr G;
  int     x,y,xy;
  int     display_all;
  int     color;
  float   normalized_value, normalized_value2; 

  int     slookup, tlookup, x2, y2;
  int image_size_x, image_size_y;
  XImage *ximage = NULL;
  char   *ximage_buf = NULL;

  
  if (name < 0 || name >= G_num_objects)
    printf("ERROR: object %d in G_display_matrix not defined.\n", name);
  
  else if(G_objects[name].type != G_MATRIX)
    printf("ERROR: object %d in G_display_matrix no matrix object.\n", name);
  
  else if (display == 1){
    G = G_objects[name].matrix_ptr;
    
    if (window_initialized == 0)
      G_initialize_window();
    

    display_all = (from_x < 0 || from_y < 0); /* ...full size */
    if (display_all){
      from_x = from_y = 0;
      num_x = G->size_matrix_x;
      num_y = G->size_matrix_y;
    }
    
    if (G->activated == 1){
      
      if (display_all)
	if (G->background_color != NO_COLOR){
	  EZX_SetColor(G->background_color);
	  INT_Rectangle(w_ptr, 
			G->int_min_x, G->int_min_y, 
			G->int_max_x - G->int_min_x,
			G->int_max_y - G->int_min_y);
	}
      




      if (G_matrix_grey_scale){	/***** normal grey-scale display */
	
	image_size_x = (int) (((float) num_x) / ((float) G->size_matrix_x)
			      * ((float) (G->int_max_x - G->int_min_x)));
	image_size_y = (int) (((float) num_y) / ((float) G->size_matrix_y)
			      * ((float) (G->int_max_y - G->int_min_y)));
	
	if (image_size_x > 0 && image_size_y > 0){

	  ximage_buf = (char *) malloc(sizeof(char) * 
				       image_size_x * image_size_y);
	  if (ximage_buf == NULL) {
	    fprintf(stderr, 
		    "ERROR: Out of memory in G_display_partial_matrix()\n");
	    exit(1);
	  }
	  XSetForeground(theDisplay, theGC,theBlackPixel);
	  XSetBackground(theDisplay, theGC,theWhitePixel);

	  for (y = 0; y < image_size_y; y++) {
	    y2 = from_y + (int) (((float) (image_size_y - y - 1))
				 * ((float) num_y) / ((float) image_size_y));
	    slookup = y2 * num_x;
	    tlookup = y  * image_size_x;

	    for (x = 0; x < image_size_x; x++) {
	      x2 = from_x + (int) (((float) x) * ((float) num_x) 
				   / ((float) image_size_x));
	      xy = x2 * G->size_matrix_y + y2;
	  
	      if (G->active == NULL || G->active[xy] != 0){
		color = (int) ((G->values[xy] - G->min_value)
			       / G->range_value * 20.0); /* 20 grays */
		if (G_invert_display)
		  color = 20 - color;
	      
		if ((color < 0 || color > 20) && G->active != NULL
		    && G->active[xy] < 0)
		  color = -G->active[xy]; /* active can be used to
					   * set colors (negative values) */
		else if (color < 0) 
		  color = C_LAWNGREEN;
		else if (color > 20)
		  color = C_RED;
		else
		  color = C_GREY0 + color;
      
	      }
	      else if (G->background_color != NO_COLOR)
		color = G->background_color;
	      else
		color = C_WHITE;

	      if (theDepth==1 && color != C_BLACK) 
		color = C_WHITE;
	    
	      ximage_buf[tlookup+x] = thePixels[color];
	    }
	  }

	  ximage = XCreateImage(theDisplay, theVisual,
				theDepth, ZPixmap,
				0, 
				ximage_buf,
				image_size_x,
				image_size_y,
				8, 
				image_size_x);

	  if (ximage == NULL){
	    fprintf(stderr, 
		    "ERROR: Out of memory in G_display_partial_matrix()\n");
	    exit(1);
	  }

	  ximage->byte_order = XImageByteOrder(theDisplay);

	  xy = from_x * G->size_matrix_y + (from_x + num_y); 

	  XPutImage(theDisplay, w_ptr->w, theGC, ximage, 
		    0, 0, 
		    /*G->display_pos_x[xy],*/
		    G->int_min_x +
		    ((int) (((float) from_x)
			    / ((float) G->size_matrix_x)
			    * ((float) (G->int_max_x - G->int_min_x)))),
		    G_int_size_y - G->int_min_y - 
		    ((int) (((float) from_y + num_y)
			    / ((float) G->size_matrix_y)
			    * ((float) (G->int_max_y - G->int_min_y)))),
		    image_size_x,
		    image_size_y);

	  free( ximage_buf );

	  ximage->data = NULL;

	  XDestroyImage( ximage );
	}
      }

      else{		/********** draw a Hinton-type box */

	
	for (x = from_x; x < from_x + num_x; x++)
	  for (y = from_y; y < from_y + num_y; y++)
	    if (x >= 0 && x < G->size_matrix_x &&
		y >= 0 && y < G->size_matrix_y){
	      xy = x * G->size_matrix_y + y; 
	      if (G->active == NULL || G->active[xy] != 0){
		color = (int) ((G->values[xy] - G->min_value)
			       / G->range_value * 20.0); /* 20 grays */
		

		normalized_value = (G->values[xy] - G->min_value)
		  / G->range_value - 0.5; /* should scale from -0.5 to 0.5 */
		normalized_value2 = 0.5 - (0.95 *
					  fabs(normalized_value)); /* size */


		EZX_SetColor(C_GREY70);	/* draw background */
		INT_Rectangle(w_ptr,
			      G->display_pos_x[xy],
			      G->display_pos_y[xy],
			      G->display_length_x,
			      G->display_length_y);
		
		/*** draw interior ***/

		if (normalized_value > 0.5){ /* too big! */
		  EZX_SetColor(C_LAWNGREEN);
		  normalized_value = 0.5;
		}
		
		else if (normalized_value >= 0.0)
		  EZX_SetColor(C_WHITE); /* positive values: black */
		
		else if (normalized_value >= -0.5)
		  EZX_SetColor(C_BLACK); /* negative values: white */
		
		else{		/* too small! */
		  EZX_SetColor(C_RED);
		  normalized_value = -0.5;
		}
		
		INT_Rectangle(w_ptr, /* Hinton-Box: variable size */
			      G->display_pos_x[xy] +
			      ((int) (normalized_value2 
				      * ((float) G->display_length_x))),
			      G->display_pos_y[xy] +
			      ((int) (normalized_value2 
				      * ((float) G->display_length_y))),
			      (int) ((1.0 - (normalized_value2 * 2.0))
				     * ((float) G->display_length_x)),
			      (int) ((1.0 - (normalized_value2 * 2.0))
				     * ((float) G->display_length_y)));
		
		/*** draw a frame, inverse color ***/
		
		if (normalized_value > 0.5){ /* too big! */
		  EZX_SetColor(C_RED);
		  normalized_value = 0.5;
		}
		
		else if (normalized_value >= 0.0)
		  EZX_SetColor(C_BLACK); /* positive values: black */
		
		else if (normalized_value >= -0.5)
		  EZX_SetColor(C_WHITE); /* negative values: white */
		
		else{		/* too small! */
		  EZX_SetColor(C_LAWNGREEN);
		  normalized_value = -0.5;
		}
		
		INT_DrawRectangle(w_ptr, /* Hinton-Box: variable size */
				  G->display_pos_x[xy] +
				  ((int) (normalized_value2 
					  * ((float) G->display_length_x))),
				  G->display_pos_y[xy] +
				  ((int) (normalized_value2 
					  * ((float) G->display_length_y))),
				  (int) ((1.0 - (normalized_value2 * 2.0))
					 * ((float) G->display_length_x)),
				  (int) ((1.0 - (normalized_value2 * 2.0))
					 * ((float) G->display_length_y)));
		
		
	       
	      }
	      else if (G->background_color != NO_COLOR){
		EZX_SetColor(G->background_color);
		INT_Rectangle(w_ptr,
			      G->display_pos_x[xy],
			      G->display_pos_y[xy],
			      G->display_length_x,
			      G->display_length_y);
	      }
	    }
      }

      
      
      if (display_all)
	if (G->frame_color != NO_COLOR){
	  EZX_SetColor(G->frame_color);
	  EZX_SetLineWidth(2);
	  INT_DrawLine(w_ptr,
		       G->int_max_x,
		       G->int_max_y,
		       G->int_max_x,
		       G->int_min_y);
	  INT_DrawLine(w_ptr,
		       G->int_max_x,
		       G->int_min_y,
		       G->int_min_x,
		       G->int_min_y);
	  if (G->frame_color >= C_GREY5 &&
	      G->frame_color <= C_GREY100)
	    EZX_SetColor(C_GREY100 + C_GREY5 - G->frame_color);
	  if (G->frame_color >= C_WHITE &&
	      G->frame_color <= C_BLACK)
	    EZX_SetColor(C_BLACK + C_WHITE - G->frame_color);
	  INT_DrawLine(w_ptr,
		       G->int_min_x,
		       G->int_min_y,
		       G->int_min_x,
		       G->int_max_y);
	  INT_DrawLine(w_ptr,
		       G->int_min_x,
		       G->int_max_y,
		       G->int_max_x,
		       G->int_max_y);
	  EZX_SetLineWidth(1);
	  /* INT_DrawRectangle(w_ptr, 
			    G->int_min_x, G->int_min_y, 
			    G->int_max_x - G->int_min_x,
			    G->int_max_y - G->int_min_y);*/
	}

      if (G->text_color != NO_COLOR){
	EZX_SetColor(G->text_color);
	EZX_UseFont(theGC, &(font_list[G->font*128]));
	INT_DrawString(w_ptr, 
		       (G->int_max_x + G->int_min_x 
			- EZX_GetTextWidth(G->text)) / 2,
		       (G->int_max_y + G->int_min_y 
			- EZX_GetFontHeight()) / 2,
		       G->text); 
      }
      if (flush) EZX_Flush();
    }
  }
}  





/************************************************************************
 *
 *   Name:         G_display_matrix
 *                 
 *   FUNCTION:     Displays a matrix window
 *                 
 *   PARAMETERS:   int name         pointer to the object to display 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_display_matrix(int name)
{
  G_display_partial_matrix(name, -1, -1, -1, -1); /* it's that simple */
}  



/************************************************************************
 *
 *   NAME:         G_matrix_set_display_range
 *                 
 *   FUNCTION:     sets range of a matrix object
 *                 
 *   PARAMETERS:   int name                      name of the object
 *                 float min_value, max_value    new range
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void G_matrix_set_display_range(int name, float min_value, float max_value)
{
  G_matrix_ptr G;

  if (name < 0 || name >= G_num_objects)
    printf("ERROR: object %d in G_matrix_set_display_range not defined.\n",
	   name);
  
  else if(G_objects[name].type != G_MATRIX)
    printf("ERROR: object %d in G_matrix_set_display_range no matrix obj.\n", 
	   name);
  
  else{
    G = G_objects[name].matrix_ptr;
    G->min_value = min_value;
    G->max_value = max_value;
    G->range_value = G->max_value - G->min_value;
    if (G->range_value == 0.0) G->range_value = 1.0; /* serves as divisor */
  }
}



/************************************************************************
 *
 *   Name:         G_set_matrix_display_style
 *                 
 *   FUNCTION:     defines the style with which to display a matrix
 *                 
 *   PARAMETERS:   int style    1 = grey scale (color screen recom.)
 *                              0 = Hinton-type
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void G_set_matrix_display_style(int style)
{
  G_matrix_grey_scale = style;
}




/************************************************************************
 *
 *   NAME:         G_display_robot
 *                 
 *   FUNCTION:     Displays a robot window and changes actual_text, if
 *                 selected
 *                 
 *   PARAMETERS:   int name             pointer to the object to display 
 *                 float robot_x, robot_y, robot_orientation
 *                                      new robot position
 *                 int num_sensors      number of sensor values passed
 *                                         (0, if none)
 *                 float *sensors       vector of new sensor values
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_display_robot(int name, 
		     float robot_x, 
		     float robot_y, 
		     float robot_orientation, 
		     int num_sensors, 
		     float *sensors)
{
  G_robot_ptr G;
  int i;
  float help_robot_x, help_robot_y, help_robot_size;
  float points_x[3], points_y[3];
  float sensor_beam_angle;


  if (name < 0 || name >= G_num_objects)
    printf("ERROR: object %d in G_display_robot not defined.\n", name);
  
  else if(G_objects[name].type != G_ROBOT)
    printf("ERROR: object %d in G_display_robot no robot object.\n", name);
    
  else if (display == 1){
    if (window_initialized == 0)
      G_initialize_window();
    
    G = G_objects[name].robot_ptr;

    if (robot_x >= G->min_value_x && robot_x <= G->max_value_x &&
	robot_y >= G->min_value_y && robot_y <= G->max_value_y &&
	robot_orientation >= -360.0 && robot_orientation <= 360.0){
      G->robot_x = robot_x;	/* update robot position */
      G->robot_y = robot_y;
      G->robot_orientation = robot_orientation;
    }

    if (num_sensors > 0 && num_sensors <= G->num_sensors)
      for (i = 0; i < num_sensors; i++)
	G->sensors[i] = sensors[i]; /* update sensor values */


    if (G->activated == 1){
      
      help_robot_x = G->min_x + ((G->robot_x - G->min_value_x)
				 / G->range_value_x * G->size_x);
      help_robot_y = G->min_y + ((G->robot_y - G->min_value_y)
				 / G->range_value_y * G->size_y);
      help_robot_size = G->robot_size / G->range_value_x * G->size_x;
      /* NOTE: in this expression, "G->size_x" fails, if x-unit unequal y-unit,
	 because then the roubot should be displayed as an ellipse */
      
      
      if (G->background_color != NO_COLOR){ /* BACKGROUND */
	EZX_SetColor(G->background_color);
	INT_Rectangle(w_ptr, 
		      G->int_min_x, G->int_min_y, 
		      G->int_max_x - G->int_min_x,
		      G->int_max_y - G->int_min_y);
      }
      
      if (G->frame_color != NO_COLOR){ /* FRAME */
	EZX_SetColor(G->frame_color);
	EZX_SetLineWidth(2);
	INT_DrawLine(w_ptr,
		     G->int_max_x,
		     G->int_max_y,
		     G->int_max_x,
		     G->int_min_y);
	INT_DrawLine(w_ptr,
		     G->int_max_x,
		     G->int_min_y,
		     G->int_min_x,
		     G->int_min_y);
	if (G->frame_color >= C_GREY5 &&
	     G->frame_color <= C_GREY100)
	  EZX_SetColor(C_GREY100 + C_GREY5 - G->frame_color);
	if (G->frame_color >= C_WHITE &&
	     G->frame_color <= C_BLACK)
	  EZX_SetColor(C_BLACK + C_WHITE - G->frame_color);
	INT_DrawLine(w_ptr,
		     G->int_min_x,
		     G->int_min_y,
		     G->int_min_x,
		     G->int_max_y);
	INT_DrawLine(w_ptr,
		     G->int_min_x,
		     G->int_max_y,
		     G->int_max_x,
		     G->int_max_y);
	EZX_SetLineWidth(1);
	/* INT_DrawRectangle(w_ptr, 
			  G->int_min_x, G->int_min_y, 
			  G->int_max_x - G->int_min_x,
			  G->int_max_y - G->int_min_y);*/
      }
      
      if (G->sensors_circle_color != NO_COLOR){ /* SENSORS_CIRCLE */
	EZX_SetColor(G->sensors_circle_color);
	FLOAT_FillCircle(w_ptr, help_robot_x, help_robot_y, 
			 G->max_sensors_range / G->range_value_x * G->size_x); 
	/* NOTE: in this expression, "G->size_x" fails, if x-unit != y-unit,
	   because then the roubot should be displayed as an ellipse */
      }
      
      if (G->sensors_max_range_color != NO_COLOR){ /* SENSORS_MAX_RANGE */
	EZX_SetColor(G->sensors_max_range_color);
	FLOAT_DrawCircle(w_ptr, help_robot_x, help_robot_y, 
			 G->max_sensors_range / G->range_value_x * G->size_x); 
	/* NOTE: in this expression, "G->size_x" fails, if x-unit != y-unit,
	   because then the roubot should be displayed as an ellipse */
      }
      
      if (G->sensors_color != NO_COLOR){	/* SENSORS */
	EZX_SetColor(G->sensors_color);
	for (i = 0; i < G->num_sensors; i++){
	  sensor_beam_angle = 10.0;
	  if (G->num_sensors > 1){
	    if (i != 0 && 
		sensor_beam_angle > 
		0.4 * fabs(G->sensors_angles[i-1] - G->sensors_angles[i]))
	      sensor_beam_angle = 
		0.4 * fabs(G->sensors_angles[i-1] - G->sensors_angles[i]);
	    if (i != G->num_sensors - 1 &&
		sensor_beam_angle > 
		0.4 * fabs(G->sensors_angles[i+1] - G->sensors_angles[i]))
	      sensor_beam_angle = 
		0.4 * fabs(G->sensors_angles[i+1] - G->sensors_angles[i]);
	  }
	  points_x[0] = help_robot_x;
	  points_y[0] = help_robot_y;
	  points_x[1] = help_robot_x +
	    ((cos((G->robot_orientation + G->sensors_angles[i] 
		   + sensor_beam_angle)
		  / 180.0 * pi) * G->sensors[i])
	     / G->range_value_x * G->size_x);
	  points_y[1] = help_robot_y +
	    ((sin((G->robot_orientation + G->sensors_angles[i] 
		   + sensor_beam_angle)
		  / 180.0 * pi) * G->sensors[i])
	     / G->range_value_y * G->size_y);
	  points_x[2] = help_robot_x +
	    ((cos((G->robot_orientation + G->sensors_angles[i] 
		   - sensor_beam_angle)
		  / 180.0 * pi) * G->sensors[i])
	     / G->range_value_x * G->size_x);
	  points_y[2] = help_robot_y +
	    ((sin((G->robot_orientation + G->sensors_angles[i] 
		   - sensor_beam_angle)
		  / 180.0 * pi) * G->sensors[i])
	     / G->range_value_y * G->size_y);

	  /* FLOAT_FillPolygon(w_ptr, 3, points_x, points_y); */


	  FLOAT_DrawLine(w_ptr, points_x[0], points_y[0], 
			 points_x[1], points_y[1]);
	  FLOAT_DrawLine(w_ptr, points_x[1], points_y[1], 
			 points_x[2], points_y[2]);
	  FLOAT_DrawLine(w_ptr, points_x[2], points_y[2], 
			 points_x[0], points_y[0]);


	}
      }
    
      
      if (G->robot_color != NO_COLOR &&	/* ROBOT */
	  G->robot_x + G->robot_size >= G->min_value_x &&
	  G->robot_x + G->robot_size <  G->max_value_x &&
	  G->robot_y + G->robot_size >= G->min_value_y &&
	  G->robot_y + G->robot_size <  G->max_value_y){
	EZX_SetColor(G->robot_color);
	FLOAT_FillCircle(w_ptr, help_robot_x, help_robot_y, help_robot_size);
      }
      
      if (G->robot_frame_color != NO_COLOR){	/* ROBOT-MARGIN */
	EZX_SetColor(G->robot_frame_color);
	FLOAT_DrawCircle(w_ptr, help_robot_x, help_robot_y, help_robot_size);
	FLOAT_DrawLine(w_ptr, help_robot_x, help_robot_y,
		       (cos(G->robot_orientation / 180.0 * pi) 
			* help_robot_size)
		       + help_robot_x,
		       (sin(G->robot_orientation / 180.0 * pi) 
			* help_robot_size)
		       + help_robot_y);
      }		      
      
      if (G->text_color != NO_COLOR){ /* TEXT */
	EZX_SetColor(G->text_color);
	EZX_UseFont(theGC, &(font_list[G->font*128]));
	INT_DrawString(w_ptr, (G->int_max_x + G->int_min_x 
			       - EZX_GetTextWidth(G->text)) / 2,
		       (G->int_max_y + G->int_min_y - EZX_GetFontHeight()) / 2,
		       G->text); 
      }
      if (flush) EZX_Flush();
    }
  }
}  




/* ********************************************************************** *\
 * ********************************************************************** *
\* ********************************************************************** */


int 
G_check_intersection(float p_x1, float p_y1, float p_x2, float p_y2,
		     float q_x1, float q_y1, float q_x2, float q_y2,
		     float *x, float *y)
{
  float mu, lambda;

  float p_dx;
  float p_dy;
  float q_dx;
  float q_dy;
  float n;
  float d;

  p_dx = p_x2 - p_x1;
  p_dy = p_y2 - p_y1;
  q_dx = q_x2 - q_x1;
  q_dy = q_y2 - q_y1;

  n = (q_x1 * q_dy) - (q_y1 * q_dx) - (p_x1 * q_dy) + (p_y1 * q_dx);
  d = (p_dx * q_dy) - (p_dy * q_dx);

  if (d == 0.0)			/* collinear */
    return 0;

  lambda = n/d;
  
  n = (p_x1 * p_dy) - (p_y1 * p_dx) - (q_x1 * p_dy) + (q_y1 * p_dx);
  d = (q_dx * p_dy) - (q_dy * p_dx);
  
  if (d == 0.0)			/* ...shouldn't happen here! */
    return 0;

  mu = n/d;


  /*
  fprintf(stderr, "### P=(%6.4f %6.4f ->%6.4f %6.4f) Q=(%6.4f %6.4f ->%6.4f %6.4f)   %6.4f %6.4f, lambda=%6.4f mu=%6.4f -> %d\n",
	  p_x1,  p_y1,  p_x2,  p_y2, q_x1,  q_y1,  q_x2,  q_y2,
	  p_x1 + (lambda * p_dx) - q_x1 - (mu * q_dx),
	  p_y1 + (lambda * p_dy) - q_y1 - (mu * q_dy), 
	  lambda, mu, 
	  (lambda >= 0.0 && lambda <= 1.0 && mu >= 0.0 && mu <= 1.0));
  */

  if (lambda >= 0.0 && lambda <= 1.0 && mu >= 0.0 && mu <= 1.0){
    *x = p_x1 + (lambda * p_dx);
    *y = p_y1 + (lambda * p_dy);
    return 1;
  }
  else
    return 0;



}



/************************************************************************
 *
 *   NAME:         G_display_markers
 *                 
 *   FUNCTION:     Displays a markers window and changes actual_text, if
 *                 selected
 *                 
 *   PARAMETERS:   int name             pointer to the object to display 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_display_markers(int name)
{
  G_markers_ptr G;
  int i;
  float help_markers_x, help_markers_y, help_marker_size;
  float x1, y1, x2, y2, x3, y3;
  float last_help_markers_x = 0.0, last_help_markers_y = 0.0;
  int first_out, second_out;

  if (name < 0 || name >= G_num_objects)
    printf("ERROR: object %d in G_display_markers not defined.\n", name);
  
  else if(G_objects[name].type != G_MARKERS)
    printf("ERROR: object %d in G_display_markers no markers object.\n", name);
    
  else if (display == 1){
    if (window_initialized == 0)
      G_initialize_window();
    
    G = G_objects[name].markers_ptr;

    if (G->activated == 1){
      
      
      if (G->background_color != NO_COLOR){ /* BACKGROUND */
	EZX_SetColor(G->background_color);
	INT_Rectangle(w_ptr, 
		      G->int_min_x, G->int_min_y, 
		      G->int_max_x - G->int_min_x,
		      G->int_max_y - G->int_min_y);
      }
      
      if (G->frame_color != NO_COLOR){ /* FRAME */
	EZX_SetColor(G->frame_color);
	EZX_SetLineWidth(2);
	INT_DrawLine(w_ptr,
		     G->int_max_x,
		     G->int_max_y,
		     G->int_max_x,
		     G->int_min_y);
	INT_DrawLine(w_ptr,
		     G->int_max_x,
		     G->int_min_y,
		     G->int_min_x,
		     G->int_min_y);
	if (G->frame_color >= C_GREY5 &&
	     G->frame_color <= C_GREY100)
	  EZX_SetColor(C_GREY100 + C_GREY5 - G->frame_color);
	if (G->frame_color >= C_WHITE &&
	     G->frame_color <= C_BLACK)
	  EZX_SetColor(C_BLACK + C_WHITE - G->frame_color);
	INT_DrawLine(w_ptr,
		     G->int_min_x,
		     G->int_min_y,
		     G->int_min_x,
		     G->int_max_y);
	INT_DrawLine(w_ptr,
		     G->int_min_x,
		     G->int_max_y,
		     G->int_max_x,
		     G->int_max_y);
	EZX_SetLineWidth(1);
	/* INT_DrawRectangle(w_ptr, 
			  G->int_min_x, G->int_min_y, 
			  G->int_max_x - G->int_min_x,
			  G->int_max_y - G->int_min_y);*/
      }
      

      help_marker_size = G->marker_size / G->range_value_x * G->size_x;
      if (help_marker_size < 1.1 / G_unit_length)
	help_marker_size = 1.1 / G_unit_length;

      for (i = 0; i < G->num_markers; i++){
	if (G->foreground_color[G->type[i]] != NO_COLOR){
	  EZX_SetColor(G->foreground_color[G->type[i]]);
	  help_markers_x = G->min_x + ((G->x[i] - G->min_value_x)
				       / G->range_value_x * G->size_x);
	  help_markers_y = G->min_y + ((G->y[i] - G->min_value_y)
				       / G->range_value_y * G->size_y);
	  
	  /*
	  if (G->x[i] >= G->min_value_x && G->x[i] < G->max_value_x &&
	      G->y[i] >= G->min_value_y && G->y[i] < G->max_value_y)
	    if (G->marker_size > 0.0){
	      if (G_markers_display_style)
		FLOAT_FillCircle(w_ptr, help_markers_x, help_markers_y, 
				 help_marker_size);
	      else
		FLOAT_DrawCircle(w_ptr, help_markers_x, help_markers_y, 
				 help_marker_size);
	    }
	    */
	  if (G->connected && i > 0){
	    x1 = G->x[i];
	    y1 = G->y[i];
	    x2 = last_help_markers_x;
	    y2 = last_help_markers_y;
	    first_out = (x1 < G->min_value_x || x1 > G->max_value_x ||
			 y1 < G->min_value_y || y1 > G->max_value_y);
	    second_out = (x2 < G->min_value_x || x2 > G->max_value_x ||
			  y2 < G->min_value_y || y2 > G->max_value_y);
	    if (!first_out || !second_out){
	      if (first_out){
		if (G_check_intersection(x1, y1, x2, y2,
					 G->min_value_x, G->min_value_y,
					 G->min_value_x, G->max_value_y, 
					 &x3, &y3)){
		  x1 = x3;
		  y1 = y3;
		}
		else if (G_check_intersection(x1, y1, x2, y2,
					      G->min_value_x, G->max_value_y,
					      G->max_value_x, G->max_value_y, 
					      &x3, &y3)){
		  x1 = x3;
		  y1 = y3;
		}
		else if (G_check_intersection(x1, y1, x2, y2,
					      G->max_value_x, G->max_value_y,
					      G->max_value_x, G->min_value_y, 
					      &x3, &y3)){
		  x1 = x3;
		  y1 = y3;
		}
		else if (G_check_intersection(x1, y1, x2, y2,
					      G->max_value_x, G->min_value_y,
					      G->min_value_x, G->min_value_y, 
					      &x3, &y3)){
		  x1 = x3;
		  y1 = y3;
		}
	      }
	      else if (second_out){
		if (G_check_intersection(x1, y1, x2, y2,
					 G->min_value_x, G->min_value_y,
					 G->min_value_x, G->max_value_y, 
					 &x3, &y3)){
		  x2 = x3;
		  y2 = y3;
		}
		else if (G_check_intersection(x1, y1, x2, y2,
					      G->min_value_x, G->max_value_y,
					      G->max_value_x, G->max_value_y, 
					      &x3, &y3)){
		  x2 = x3;
		  y2 = y3;
		}
		else if (G_check_intersection(x1, y1, x2, y2,
					      G->max_value_x, G->max_value_y,
					      G->max_value_x, G->min_value_y, 
					      &x3, &y3)){
		  x2 = x3;
		  y2 = y3;
		}
		else if (G_check_intersection(x1, y1, x2, y2,
					      G->max_value_x, G->min_value_y,
					      G->min_value_x, G->min_value_y, 
					      &x3, &y3)){
		  x2 = x3;
		  y2 = y3;
		}
	      }
	      x1 = G->min_x + ((x1 - G->min_value_x)
			       / G->range_value_x * G->size_x);
	      y1 = G->min_y + ((y1 - G->min_value_y)
			       / G->range_value_y * G->size_y);
	      x2 = G->min_x + ((x2 - G->min_value_x)
			       / G->range_value_x * G->size_x);
	      y2 = G->min_y + ((y2 - G->min_value_y)
			       / G->range_value_y * G->size_y);
	      FLOAT_DrawLine(w_ptr, x1, y1, x2, y2);
	    }
	  }
	  if (G->x[i] >= G->min_value_x && G->x[i] < G->max_value_x &&
	      G->y[i] >= G->min_value_y && G->y[i] < G->max_value_y){

	    if (G->marker_size > 0.0){
	      if (G_markers_display_style)
		FLOAT_FillCircle(w_ptr, help_markers_x, help_markers_y, 
				 help_marker_size);
	      else
		FLOAT_DrawCircle(w_ptr, help_markers_x, help_markers_y, 
				 help_marker_size);
	    }

	    if (G->text_color[G->type[i]] != NO_COLOR){
	      EZX_SetColor(G->text_color[G->type[i]]);
	      EZX_UseFont(theGC, &(font_list[G->fonts[G->type[i]]*128]));
	      FLOAT_DrawStringCentered(w_ptr, help_markers_x, help_markers_y,
				       G->texts[G->type[i]]); 
	    }
	  }
	}
	last_help_markers_x = G->x[i];
	last_help_markers_y = G->y[i];
      }
      if (flush) EZX_Flush();
    }
  }
}  





/************************************************************************
 *
 *   NAME:         G_undisplay_markers
 *                 
 *   FUNCTION:     undisplays a or several markers 
 *                 
 *   PARAMETERS:   int name             pointer to the object to display 
 *                 int number           number of marker, or -1 = all
 *                 int COLOR            color to fill markers with
 *                 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_undisplay_markers(int name, int number, int COLOR)
{
  G_markers_ptr G;
  int i;
  float help_markers_x, help_markers_y, help_marker_size;
  float last_help_markers_x = 0.0, last_help_markers_y = 0.0;
  int out_of_range;

  if (name < 0 || name >= G_num_objects)
    printf("ERROR: object %d in G_display_markers not defined.\n", name);
  
  else if(G_objects[name].type != G_MARKERS)
    printf("ERROR: object %d in G_display_markers no markers object.\n", name);
    
  else if (display == 1){
    if (window_initialized == 0)
      G_initialize_window();

    G = G_objects[name].markers_ptr;
    out_of_range = 0;

    if (G->activated == 1){
      
      
      if (G->background_color != NO_COLOR && number == -1){ /* BACKGROUND */
	EZX_SetColor(COLOR);
	INT_Rectangle(w_ptr, 
		      G->int_min_x, G->int_min_y, 
		      G->int_max_x - G->int_min_x,
		      G->int_max_y - G->int_min_y);
      }
      
      if (G->frame_color != NO_COLOR && number == -1){ /* FRAME */
	EZX_SetColor(COLOR);
	EZX_SetLineWidth(2);
	INT_DrawLine(w_ptr,
		     G->int_max_x,
		     G->int_max_y,
		     G->int_max_x,
		     G->int_min_y);
	INT_DrawLine(w_ptr,
		     G->int_max_x,
		     G->int_min_y,
		     G->int_min_x,
		     G->int_min_y);
	if (COLOR >= C_GREY5 &&
	     COLOR <= C_GREY100)
	  EZX_SetColor(C_GREY100 + C_GREY5 - COLOR);
	if (COLOR >= C_WHITE &&
	     COLOR <= C_BLACK)
	  EZX_SetColor(C_BLACK + C_WHITE - COLOR);
	INT_DrawLine(w_ptr,
		     G->int_min_x,
		     G->int_min_y,
		     G->int_min_x,
		     G->int_max_y);
	INT_DrawLine(w_ptr,
		     G->int_min_x,
		     G->int_max_y,
		     G->int_max_x,
		     G->int_max_y);
	EZX_SetLineWidth(1);
	/*
	INT_DrawRectangle(w_ptr, 
			  G->int_min_x, G->int_min_y, 
			  G->int_max_x - G->int_min_x,
			  G->int_max_y - G->int_min_y);
			  */
      }
      

      help_marker_size = G->marker_size / G->range_value_x * G->size_x;

      for (i = 0; i < G->num_markers; i++){
	if (G->x[i] >= G->min_value_x && G->x[i] < G->max_value_x &&
	    G->y[i] >= G->min_value_y && G->y[i] < G->max_value_y){
	  help_markers_x = G->min_x + ((G->x[i] - G->min_value_x)
				       / G->range_value_x * G->size_x);
	  help_markers_y = G->min_y + ((G->y[i] - G->min_value_y)
				       / G->range_value_y * G->size_y);
	  if (G->foreground_color[G->type[i]] != NO_COLOR && 
	      (number == -1 || number == i)){
	    EZX_SetColor(COLOR);
	    if (G->connected && i > 0 && !out_of_range)
	      FLOAT_DrawLine(w_ptr, last_help_markers_x, last_help_markers_y, 
			     help_markers_x, help_markers_y);

	    if (G_markers_display_style)
	      FLOAT_FillCircle(w_ptr, help_markers_x, help_markers_y, 
			       help_marker_size);
	    else
	      FLOAT_DrawCircle(w_ptr, help_markers_x, help_markers_y, 
			       help_marker_size);
	  }
	  if (G->text_color[G->type[i]] != NO_COLOR && 
	      (number == -1 || number == i)){
	    EZX_SetColor(COLOR);
	    EZX_UseFont(theGC, &(font_list[G->fonts[G->type[i]]*128]));
	    FLOAT_DrawStringCentered(w_ptr, help_markers_x, help_markers_y,
				     G->texts[G->type[i]]); 
	  }
	  last_help_markers_x = help_markers_x;
	  last_help_markers_y = help_markers_y;
	  out_of_range = 0;
	}
	else
	  out_of_range = 1;
      }
      if (flush) EZX_Flush();
    }
  }
}  





/************************************************************************
 *
 *   NAME:         G_add_marker
 *                 
 *   FUNCTION:     adds a new marker in a markers object
 *                 
 *   PARAMETERS:   int name      name of object
 *                 float x,y     coordinates of the new marker
 *                 int type      type of the marker (color, text etc)
 *                 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_add_marker(int name, float x, float y, int type)
{
  G_markers_ptr G;
  int i;

  if (name < 0 || name >= G_num_objects)
    printf("ERROR: object %d in G_add_marker not defined.\n", name);
  
  else if(G_objects[name].type != G_MARKERS)
    printf("ERROR: object %d in G_add_marker no markers object.\n", name);
    
  else{
    G = G_objects[name].markers_ptr;
    if (G->num_markers >= max_n_markers)
      printf("WARNING: Overflow in G_add_marker().\n");

#ifdef OLD_STUFF
    else if (x < G->min_value_x || x >= G->max_value_x ||
	     y < G->min_value_y || y >= G->max_value_y)
      printf("WARNING: wrong (x,y)-value in G_add_marker().\n");
#endif    

    else if (type < 0 || type >= G->num_texts)
      printf("WARNING: wrong type in G_add_marker().\n");

    else{
      G->x[G->num_markers]    = x;
      G->y[G->num_markers]    = y;
      G->type[G->num_markers] = type;
      G->num_markers++;
    }
  }
}      
    


/************************************************************************
 *
 *   NAME:         G_delete_marker
 *                 
 *   FUNCTION:     deletes one (or all) marker(s) which intersect with a
 *                 specific x-y-position
 *                 
 *   PARAMETERS:   int name        name of the markers object
 *                 float x,y       coordinates of the marker
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_delete_marker(int name, float x, float y)
{
  G_markers_ptr G;
  int i, j;
  float help;
  int n_delete;
  
  if (name < 0 || name >= G_num_objects)
    printf("ERROR: object %d in G_delete_marker not defined.\n", name);
  
  else if(G_objects[name].type != G_MARKERS)
    printf("ERROR: object %d in G_delete_marker no markers object.\n", name);
  
  else{
    G = G_objects[name].markers_ptr;
    n_delete = 0;
    for (i = 0; i < G->num_markers; i++){	/* browsing and marking */
      help = sqrt(((x - G->x[i]) * (x - G->x[i])) +
		  ((y - G->y[i]) * (y - G->y[i])));
      if (help <= G->marker_size){
	G->type[i] = -1;
	n_delete++;
      }
    }

    if (n_delete > 0){
      G->num_markers -= n_delete;	/* then delete */
      for (i = 0, j = 0; i < G->num_markers; i++, j++){	
	for (; G->type[j] == -1; ) j++;
	G->x[i]    = G->x[j];
	G->y[i]    = G->y[j];
	G->type[i] = G->type[j];
      }
    }
  }
}




/************************************************************************
 *
 *   NAME:         G_return_num_markers
 *                 
 *   FUNCTION:     returns number of markers in a markers object
 *                 
 *   PARAMETERS:   int name      name of object
 *                 int mode      1 = count *all* markers
 *                               0 count only visible markers
 *                 
 *   RETURN-VALUE: int number   number of markers, -1 if error
 *                 
 ************************************************************************/


int G_return_num_markers(int name, int mode)
{
  G_markers_ptr G;
  int i, sum;

  if (name < 0 || name >= G_num_objects)
    printf("ERROR: object %d in G_return_num_markers not defined.\n", name);

  else if(G_objects[name].type != G_MARKERS)
    printf("ERROR: object %d in G_return_num_markers no markers object.\n", 
	   name);

  else if (mode)		/* return number of all markers */
    return (G_objects[name].markers_ptr->num_markers);

  else{				/* return number of visible markers only */
    G = G_objects[name].markers_ptr;
    sum = 0;
    for (i = 0; i < G->num_markers; i++)
      if (G->x[i] >= G->min_value_x && G->x[i] < G->max_value_x &&
	  G->y[i] >= G->min_value_y && G->y[i] < G->max_value_y)
	sum++;
    return sum;
  }

  return -1;
}      
    



/************************************************************************
 *
 *   NAME:         G_return_marker_coordinates
 *                 
 *   FUNCTION:     returns a marker coordinate
 *                 
 *   PARAMETERS:   int name      name of object
 *                 int number    number of the marker
 *                 float *x,*y   coordinates of the new marker
 *                 int *type     type of the marker (color, text etc)
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_return_marker_coordinates(int name, int number, 
				 float *x, float *y, int *type)
{
  G_markers_ptr G;

  *x = 0.0;
  *y = 0.0;
  *type = -1;

  if (name < 0 || name >= G_num_objects)
    printf("ERROR: object %d in G_return_marker_coordinates not defined.\n", 
	   name);
  
  else if(G_objects[name].type != G_MARKERS)
    printf("ERROR: object %d in G_return_marker_coordinates no markers ob.\n", 
	   name);

  else{

    G = G_objects[name].markers_ptr;
    if (number < 0 || number >= G->num_markers)
      printf("ERROR: number %d in G_return_marker_coordinates illegal.\n",
	     number);
    
    else{
      *x = G->x[number];
      *y = G->y[number];
      *type = G->type[number];
    }
  }
}      


    

/************************************************************************
 *
 *   NAME:         G_display_all()
 *                 
 *   FUNCTION:     displays the whole graphics window
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_display_all(){
  int i;

  if (display == 1){
    if (window_initialized == 0)
      G_initialize_window();
    flush = 0;
    if (G_background_color != NO_COLOR){
      EZX_SetColor(G_background_color);
      INT_Rectangle(w_ptr, 0, 0, G_int_size_x, G_int_size_y); 
    }
    for (i = 0; i < G_num_objects; i++){
      if (G_objects[i].type == G_SWITCH)
	G_display_switch(i, -1);
      else if (G_objects[i].type == G_VALUE)
	G_display_value(i, NO_VALUE);
      else if (G_objects[i].type == G_MATRIX)
	G_display_matrix(i);
      else if (G_objects[i].type == G_ROBOT)
	G_display_robot(i, NO_VALUE, NO_VALUE, NO_VALUE, -1, NULL);
      else if (G_objects[i].type == G_MARKERS)
	G_display_markers(i);
      
      /* XXX add new object types here */
      else
	printf("STRANGE2: Object type unknown!\n");
    }
    flush = 1;
    EZX_Flush();
  }
}




/************************************************************************
 *
 *   NAME:         G_set_display
 *                 
 *   FUNCTION:     Activates (default) or deactivates display
 *                 
 *   PARAMETERS:   int d         0=no display, 1=display
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_set_display(int d)
{
  display = d;
}



/************************************************************************
 *
 *   NAME:         G_activate
 *                 
 *   FUNCTION:     activates graphics object (mouse and display)
 *                 
 *   PARAMETERS:   int name    name of the object
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_activate(int name)
{
  
  if (name < 0 || name >= G_num_objects)
    printf("WARNING: attempt to activate unknown object: no change.\n");
  
  else{
    
    if (G_objects[name].type == G_SWITCH)
      (G_objects[name].switch_ptr)->activated = 1; 
    else if (G_objects[name].type == G_VALUE)
      (G_objects[name].value_ptr)->activated = 1; 
    else if (G_objects[name].type == G_MATRIX)
      (G_objects[name].matrix_ptr)->activated = 1; 
    else  if (G_objects[name].type == G_ROBOT)
      (G_objects[name].robot_ptr)->activated = 1; 
    else  if (G_objects[name].type == G_MARKERS)
      (G_objects[name].markers_ptr)->activated = 1; 
    
    /* XXX add new object types here */
    
    else
      printf("STRANGE5: Object type unknown!\n");
  }
}



/************************************************************************
 *
 *   NAME:         G_deactivate
 *                 
 *   FUNCTION:     deactivates graphics object (mouse and display)
 *                 
 *   PARAMETERS:   int name    name of the object
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_deactivate(int name)
{
  
  if (name < 0 || name >= G_num_objects)
    printf("WARNING: attempt to activate unknown object: no change.\n");
  
  else{
    
    if (G_objects[name].type == G_SWITCH)
      (G_objects[name].switch_ptr)->activated = 0; 
    else if (G_objects[name].type == G_VALUE)
      (G_objects[name].value_ptr)->activated = 0; 
    else if (G_objects[name].type == G_MATRIX)
      (G_objects[name].matrix_ptr)->activated = 0; 
    else  if (G_objects[name].type == G_ROBOT)
      (G_objects[name].robot_ptr)->activated = 0; 
    else  if (G_objects[name].type == G_MARKERS)
      (G_objects[name].markers_ptr)->activated = 0; 
    
    /* XXX add new object types here */
    
    else
      printf("STRANGE5: Object type unknown!\n");
  }
}

  

/************************************************************************
 *
 *   NAME:         G_set_new_text
 *                 
 *   FUNCTION:     sets a new text
 *                 
 *   PARAMETERS:   int name         name of the object
 *                 char *new_text   the new text
 *                 int  text_nr     just for switches: the number of the
 *                                  text to be changed
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_set_new_text(int name, char *new_text, int text_nr)
{
  if (name < 0 || name >= G_num_objects)
    printf("WARNING: attempt to change text in unknown object: no change.\n");
  else if (new_text == NULL)
    printf("WARNING: NULL is not a legitimate text.\n");
  
  else{
    /*    +++++++     */
    if (G_objects[name].type == G_SWITCH){
      if (text_nr < 0 || text_nr > (G_objects[name].switch_ptr)->num_texts)
	printf("ERROR: wrong value text_nr=%d in set_new_text.\n",
	       text_nr);
      strcpy((G_objects[name].switch_ptr)->texts[text_nr], new_text);
    }
    else if (G_objects[name].type == G_VALUE)
      strcpy((G_objects[name].value_ptr)->text, new_text);
    else if (G_objects[name].type == G_MATRIX)
      strcpy((G_objects[name].matrix_ptr)->text, new_text);
    else  if (G_objects[name].type == G_ROBOT)
      strcpy((G_objects[name].robot_ptr)->text, new_text);
    else  if (G_objects[name].type == G_MARKERS){
      if (text_nr < 0 || text_nr > (G_objects[name].markers_ptr)->num_texts)
	printf("ERROR: wrong value text_nr=%d in set_new_text.\n",
	       text_nr);
      strcpy((G_objects[name].markers_ptr)->texts[text_nr], new_text);
    }
    
    /* XXX add new object types here */
    
    else
      printf("STRANGE5: Object type unknown!\n");
  }
}



/************************************************************************
 *
 *   NAME:         G_test_mouse
 *                 
 *   FUNCTION:     Tests mouse event, returns whether mouse button detected
 *                 
 *   PARAMETERS:   int wait      1=wait for mouse event, (careful, may cause
 *                                                        trouble)
 *                               0=return immediately, if no button pressed
 *                 
 *   RETURN-VALUE: int button    1=button detected
 *                               2=motion event detected
 *                               0=nothing
 *                 
 ************************************************************************/




int G_test_mouse(int wait)
{
  int c_x, c_y, button;
  
  if (window_initialized == 1){
    if (wait == 1)		/* has been found to cause some trouble */
      button = EZX_GetCursor(&c_x, &c_y);
    else
      button = EZX_TestGetCursor(NULL, &c_x,  &c_y);
    
    if (c_x >= 0 && c_y >= 0){	/* button-press or motion */
      c_y = G_int_size_y - c_y;
      last_button      = button;
      last_int_mouse_x = c_x;
      last_int_mouse_y = c_y;
      last_mouse_x     = ((float) c_x) / ((float) G_unit_length) + G_min_x;
      last_mouse_y     = ((float) c_y) / ((float) G_unit_length) + G_min_y;
      last_mouse_event_defined = 1;
      last_mouse_event_evaluated = 0;
      
      if (button >= 0)		/* button-press event occurred? */
	return 1;		/* button event */
      else
	return 2;		/* motion event */
    }
  }
  return 0;
}




/************************************************************************
 *
 *   NAME:         G_evaluate_mouse()
 *                 
 *   FUNCTION:     evaluates last mouse event, if defined
 *                 returns type and position of last mouse event
 *                 returns also a list of mouse events relativ to the
 *                 internal graphics objects, and evaluates mouse event
 *                 with respect to those
 *                 
 *   PARAMETERS:   float *x, *y        absolute ccordinate values
 *                 int *button         button number
 *                 int *num_events     number of "special" events, -1 if no 
 *                                     last mouse click defined
 *                 
 *   RETURN-VALUE: G_mouse_ptr mouse_events   pointer to structure memorizing
 *                                            "special" events
 *                 
 ************************************************************************/



G_mouse_ptr G_evaluate_mouse(float *x, float *y, int *button, int *num_events)
{
  int i, j;
  G_switch_ptr   SWITCH;
  G_value_ptr    VALUE;
  G_matrix_ptr   MATRIX;
  G_robot_ptr    ROBOT;
  G_markers_ptr  MARKERS;
  float help;


  if (G_mouse_allocated == 0){ /* allocate memory, if not yet done */
    G_mouse = (G_mouse_ptr)
      (malloc(sizeof(G_mouse_type) * (max_num_graphic_objects+1)));
    /*XXX printf(" -M89- "); */
    G_mouse_allocated = 1;
  }
  G_mouse[0].name = G_NO_OBJECT;
  G_mouse[0].type = G_NO_OBJECT;

  

  if (last_mouse_event_defined == 1){

    *x = last_mouse_x;
    *y = last_mouse_y;
    *button = last_button;
    *num_events = 0;
    last_mouse_event_evaluated = 1;
   
    for (i = 0; i < G_num_objects; i++){
      

      if (G_objects[i].type == G_SWITCH){ /* SWITCH object */
	SWITCH = G_objects[i].switch_ptr;
	if (last_int_mouse_x >= SWITCH->int_min_x && 
	    last_int_mouse_x <= SWITCH->int_max_x &&
	    last_int_mouse_y >= SWITCH->int_min_y && 
	    last_int_mouse_y <= SWITCH->int_max_y &&
	    SWITCH->activated == 1){ 		/* object found */
	  G_mouse[*num_events].type  = G_SWITCH;
	  G_mouse[*num_events].name  = i;
	  G_mouse[*num_events].button = last_button;
	  G_mouse[*num_events].rel_x = ((last_mouse_x - SWITCH->min_x) /
				       SWITCH->size_x);
	  G_mouse[*num_events].rel_y = ((last_mouse_y - SWITCH->min_y) /
				       SWITCH->size_y);
	  G_mouse[*num_events].actual_text = SWITCH->actual_text; 
	  (*num_events)++;
	}
      }


      else if (G_objects[i].type == G_VALUE){ /* VALUE object */
	VALUE = G_objects[i].value_ptr;
	if (last_int_mouse_x >= VALUE->int_min_x && 
	    last_int_mouse_x <= VALUE->int_max_x &&
	    last_int_mouse_y >= VALUE->int_min_y && 
	    last_int_mouse_y <= VALUE->int_max_y &&
	    VALUE->activated == 1){ 		/* object found */
	  G_mouse[*num_events].type  = G_VALUE;
	  G_mouse[*num_events].name  = i;
	  G_mouse[*num_events].button = last_button;
	  G_mouse[*num_events].rel_x = ((last_mouse_x - VALUE->min_x) /
				       VALUE->size_x);
	  G_mouse[*num_events].rel_y = ((last_mouse_y - VALUE->min_y) /
					VALUE->size_y);
	  if (VALUE->direction == 1) /* copy x-coord into value */
	    G_mouse[*num_events].value = (G_mouse[*num_events].rel_x
					  * (VALUE->max_value 
					     - VALUE->min_value))
	      + VALUE->min_value;
	  else			/* copy y-coord into value */
	    G_mouse[*num_events].value = (G_mouse[*num_events].rel_y
					  * (VALUE->max_value 
					     - VALUE->min_value))
	      + VALUE->min_value;
	  (*num_events)++;
	}
      }

      else if (G_objects[i].type == G_MATRIX){ /* MATRIX object */
	MATRIX = G_objects[i].matrix_ptr;
	if (last_int_mouse_x >= MATRIX->int_min_x && 
	    last_int_mouse_x <= MATRIX->int_max_x &&
	    last_int_mouse_y >= MATRIX->int_min_y && 
	    last_int_mouse_y <= MATRIX->int_max_y &&
	    MATRIX->activated == 1){ 		/* object found */
	  G_mouse[*num_events].type    = G_MATRIX;
	  G_mouse[*num_events].name    = i;
	  G_mouse[*num_events].button = last_button;
	  G_mouse[*num_events].rel_x   = ((last_mouse_x - MATRIX->min_x) /
					  MATRIX->size_x);
	  G_mouse[*num_events].rel_y   = ((last_mouse_y - MATRIX->min_y) /
					  MATRIX->size_y);
	  G_mouse[*num_events].index_x = (int) (G_mouse[*num_events].rel_x *
						MATRIX->size_matrix_x);
	  G_mouse[*num_events].index_y = (int) (G_mouse[*num_events].rel_y *
						MATRIX->size_matrix_y);
	  G_mouse[*num_events].value = MATRIX->values
	    [G_mouse[*num_events].index_x 
	     *  MATRIX->size_matrix_y +
	     G_mouse[*num_events].index_y];
	  if (MATRIX->active)
	    G_mouse[*num_events].active = MATRIX->active
	      [G_mouse[*num_events].index_x 
	      * MATRIX->size_matrix_y +
	      G_mouse[*num_events].index_y];
	  else
	    G_mouse[*num_events].active = 1;
	  
	  (*num_events)++;
	}
      }
      
	
      else if (G_objects[i].type == G_ROBOT){ /* ROBOT object */
	ROBOT = G_objects[i].robot_ptr;
	if (last_int_mouse_x >= ROBOT->int_min_x && 
	    last_int_mouse_x <= ROBOT->int_max_x &&
	    last_int_mouse_y >= ROBOT->int_min_y && 
	    last_int_mouse_y <= ROBOT->int_max_y &&
	    ROBOT->activated == 1){ 		/* object found */
	  G_mouse[*num_events].type    = G_ROBOT;
	  G_mouse[*num_events].name    = i;
	  G_mouse[*num_events].button = last_button;
	  G_mouse[*num_events].rel_x   = ((last_mouse_x - ROBOT->min_x) /
					  ROBOT->size_x);
	  G_mouse[*num_events].rel_y   = ((last_mouse_y - ROBOT->min_y) /
					  ROBOT->size_y);

	  G_mouse[*num_events].value_x = 
	    (G_mouse[*num_events].rel_x * ROBOT->range_value_x)
	      + ROBOT->min_value_x;
	  G_mouse[*num_events].value_y = 
	    (G_mouse[*num_events].rel_y * ROBOT->range_value_y)
	      + ROBOT->min_value_y;
	  G_mouse[*num_events].robot_delta_x = 
	    (G_mouse[*num_events].rel_x * ROBOT->range_value_x)
	      + ROBOT->min_value_x - ROBOT->robot_x;
	  G_mouse[*num_events].robot_delta_y = 
	    (G_mouse[*num_events].rel_y * ROBOT->range_value_y)
	      + ROBOT->min_value_y - ROBOT->robot_y;
	  G_mouse[*num_events].robot_delta_orientation =
	    (atan2(G_mouse[*num_events].robot_delta_y,
		   G_mouse[*num_events].robot_delta_x)
	     / pi * 180.0) - ROBOT->robot_orientation;
	  for (; G_mouse[*num_events].robot_delta_orientation
	       + ROBOT->robot_orientation >= 360.0; )
	    G_mouse[*num_events].robot_delta_orientation -= 360.0;
	  for (; G_mouse[*num_events].robot_delta_orientation
	       + ROBOT->robot_orientation < 0.0; )
	    G_mouse[*num_events].robot_delta_orientation += 360.0;
	  (*num_events)++;
	}
      }
      
      else if (G_objects[i].type == G_MARKERS){ /* MARKERS object */
	MARKERS = G_objects[i].markers_ptr;
	if (last_int_mouse_x >= MARKERS->int_min_x && 
	    last_int_mouse_x <= MARKERS->int_max_x &&
	    last_int_mouse_y >= MARKERS->int_min_y && 
	    last_int_mouse_y <= MARKERS->int_max_y &&
	    MARKERS->activated == 1){ 		/* object found */
	  G_mouse[*num_events].type    = G_MARKERS;
	  G_mouse[*num_events].name    = i;
	  G_mouse[*num_events].button = last_button;
	  G_mouse[*num_events].rel_x   = ((last_mouse_x - MARKERS->min_x) /
					  MARKERS->size_x);
	  G_mouse[*num_events].rel_y   = ((last_mouse_y - MARKERS->min_y) /
					  MARKERS->size_y);

	  G_mouse[*num_events].value_x = 
	    (G_mouse[*num_events].rel_x * MARKERS->range_value_x)
	      + MARKERS->min_value_x;
	  G_mouse[*num_events].value_y = 
	    (G_mouse[*num_events].rel_y * MARKERS->range_value_y)
	      + MARKERS->min_value_y;
	  G_mouse[*num_events].marker_name = -1;
	  G_mouse[*num_events].marker_type = -1;
	  G_mouse[*num_events].marker_x    = -999999.9;
	  G_mouse[*num_events].marker_y    = -999999.9;
	  for (j = 0; j < MARKERS->num_markers; j++){
	    help = sqrt(((G_mouse[*num_events].value_x-MARKERS->x[j]) *
			 (G_mouse[*num_events].value_x-MARKERS->x[j])) +
			((G_mouse[*num_events].value_y-MARKERS->y[j]) *
			 (G_mouse[*num_events].value_y-MARKERS->y[j])));
	    if (help <= MARKERS->marker_size){
	      G_mouse[*num_events].marker_name = j;
	      G_mouse[*num_events].marker_type = MARKERS->type[j];
	      G_mouse[*num_events].marker_x    = MARKERS->x[j];
	      G_mouse[*num_events].marker_y    = MARKERS->y[j];
	    }
	  }
	  (*num_events)++;

	}
      }
      
	
      /* XXX add new object types here */
      
      else
	printf("STRANGE3: Object type unknown!\n");

      G_mouse[*num_events].name = G_NO_OBJECT; /* mark end of sequence */
      G_mouse[*num_events].type = G_NO_OBJECT;
      
    }
  }
  else
    *num_events = -1;
  
  
  return G_mouse;
}




/************************************************************************
 *
 *   NAME:         G_mouse_event_at
 *                 
 *   FUNCTION:     tests if button was clicked in a particular object
 *                 
 *   PARAMETERS:   int name                  name of the object
 *                 G_mouse_ptr mouse_events  pointer to mouse evaluation
 *                 int *number               number in the mouse-event struct
 *                                           -1 if none.
 *                 
 *   RETURN-VALUE: 1, if event ocurred in object 'name', 0 if not
 *                 
 *                 
 ************************************************************************/



int G_mouse_event_at(int name, G_mouse_ptr mouse_events, int *number)
{
  int i;
  int found;

  found = 0;

  if (last_mouse_event_evaluated == 1)
    for (i = 0; mouse_events[i].type != G_NO_OBJECT && 
	 i <= max_num_graphic_objects; i++)
      if (mouse_events[i].name == name){
	*number = i;
	found = 1;
      }


  return found;
}




/************************************************************************
 *
 *   NAME:         G_test_graphics
 *                 
 *   FUNCTION:     test procedure
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void G_test_graphics(){
  int i,j;

  G_mouse_ptr mouse_events;
  int num_mouse_events, button;
  float mouse_x, mouse_y;


  static char *myfonts[] = {"5x8", "6x10", "8x13", 
			      "9x15", "10x20", "12x24"};


  int switch1, switch_1_num             = 2;
  static float switch_1_pos[]           = {0.2, 0.8, 0.1, 0.3};
  static char *switch_1_texts[]         = {"switch-1-0", "switch-1-1"};
  static int switch_1_background_color[]= {C_BLUE, C_WHITE};
  static int switch_1_frame_color[]     = {C_WHITE, C_BLUE};
  static int switch_1_text_color[]      = {C_WHITE, C_RED};
  static int switch_1_fonts[]           = {2, 4};


  int switch2, switch_2_num             = 3;
  static float switch_2_pos[]           = {0.5, 1.5, 0.0, 0.2};
  static char *switch_2_texts[]         = {"s-2-0", "s-2-1", "s-2-2"};
  static int switch_2_background_color[]= {C_WHITE, C_WHITE, C_MEDIUMPURPLE3};
  static int switch_2_frame_color[]     = {C_MEDIUMPURPLE3, C_MEDIUMPURPLE3,
					     C_WHITE};
  static int switch_2_text_color[]      = {C_BLACK, C_BLACK, C_WHITE};
  static int switch_2_fonts[]           = {0, 1, 2};


  int value1;
  static float value_1_pos[]            = {0.3, 0.4, 0.0, 1.0};
  static char *value_1_text             = "value 1";
  int direction_value_1                 = 2;
  static float value_1_x                = 0.5;
  float min_value1_x                    = 0.0;
  float max_value1_x                    = 1.0;
  static int value_1_colors[]           = {C_WHITE, C_BLACK,C_YELLOW,C_BLACK}; 
  static int value_1_font               = 3;

  int value2;
  static float value_2_pos[]            = {0.2, 0.5, 0.8, 0.85};
  static char *value_2_text             = "value 2";
  int direction_value_2                 = 1;
  static float value_2_x                = 0.5;
  float min_value2_x                    = 0.0;
  float max_value2_x                    = 1.0;
  static int value_2_colors[]           = {C_WHITE, C_BLACK,C_CYAN,C_BLACK}; 
  static int value_2_font               = 4;
  

  int matrix1;
  static float matrix_1_pos[]            = {0.4, 0.9, 0.5, 1.2};
  char *matrix_1_text                    = "matrix 1";
  static float matrix_1_values[10][5];
  static int   matrix_1_active[10][5];
  int size_matrix1_x                     = 10;
  int size_matrix1_y                     = 5;
  float min_value_matrix1                = -1.0;
  float max_value_matrix1                = 2.0;
  static int matrix_1_colors[]           = {C_YELLOW,C_BLACK,C_BLACK,C_BLACK}; 
  static int matrix_1_font               = 0;

  int robot1;
  static float pos_r1[]                 = {0.5, 1.5, 0.3, 1.3};
  static char *text_r1                  = "ROBBY";
  float min_value_x_r1                  = -200.0;
  float max_value_x_r1                  =  300.0;
  float min_value_y_r1                  = -400.0;
  float max_value_y_r1                  =  100.0;
  float robot_x_r1                      =  0.0;
  float robot_y_r1                      =  0.0;
  float robot_orientation_r1            =  45.0;
  float robot_size_r1                   =  10.0;
  int num_sensors_r1                    =  24;
  float max_sensors_range_r1            =  100.0;
  static float sensors_r1[24];
  static float sensors_angles_r1[24];
  static int colors_r1[]                = {NO_COLOR, C_KHAKI4, C_RED, C_BLUE,
					     C_LAWNGREEN, NO_COLOR, NO_COLOR,
					     C_KHAKI4};
  static int robot_1_font               = 5;



  int markers1;
  int mar1_num                            = 3;
  static float pos_mar1[]                 = {1.4, 2.4, 0.5, 1.5};
  static char *text_mar1[]                = {"LEFT", "MIDDLE", "RIGHT"};
  float min_value_x_mar1                  = -400.0;
  float max_value_x_mar1                  =  100.0;
  float min_value_y_mar1                  = -100.0;
  float max_value_y_mar1                  =  400.0;
  float markers_size_mar1                 =  7.0;
  int connected_mar1                      =  1;
  static int mar1_frame_color             = C_RED;
  static int mar1_background_color        = C_BLUE;
  static int mar1_foreground_color[]      = {C_YELLOW, C_LAWNGREEN, C_KHAKI4};
  static int mar1_text_color[]            = {C_BLACK, C_GREY50, C_GREY80};
  static int mar1_fonts[]                 = {1, 2, 3};



  for (i = 0; i < 24; i++){
    sensors_r1[i] = max_sensors_range_r1 * (sin(((float) i)*0.6)*0.5+0.5);
    sensors_angles_r1[i] = ((float) i) / 24.0 * 360.0;
  }



  for (i = 0; i < size_matrix1_x; i++)
    for (j = 0; j < size_matrix1_y; j++){
      matrix_1_values[i][j] = ((float) (i * size_matrix1_y + j))
	/ ((float) ((size_matrix1_x*size_matrix1_y)-1)) *
	  (max_value_matrix1-min_value_matrix1) + min_value_matrix1;
      matrix_1_active[i][j] = 1;
      if (i == 1)
	matrix_1_active[i][j] = 0;
    }



  G_initialize_graphics("ODYSSEUS", 400.0, 10.0, C_CYAN);
  G_initialize_fonts(6, myfonts);
    
  switch1 = G_create_switch_object(switch_1_pos, 
				   switch_1_num, 
				   switch_1_texts,
				   switch_1_background_color,
				   switch_1_frame_color, 
				   switch_1_text_color,
				   switch_1_fonts);

  switch2 = G_create_switch_object(switch_2_pos, 
				   switch_2_num, 
				   switch_2_texts,
				   switch_2_background_color,
				   switch_2_frame_color, 
				   switch_2_text_color,
				   switch_2_fonts);

  
  value1 = G_create_value_object(value_1_pos, value_1_text, direction_value_1,
				 value_1_x, min_value1_x, max_value1_x, 
				 value_1_colors, value_1_font);

  value2 = G_create_value_object(value_2_pos, value_2_text, direction_value_2,
				 value_2_x, min_value2_x, max_value2_x, 
				 value_2_colors, value_2_font);



  robot1 = G_create_robot_object(pos_r1, text_r1, 
				 min_value_x_r1, max_value_x_r1, 
				 min_value_y_r1,max_value_y_r1, 
				 robot_x_r1, robot_y_r1, 
				 robot_orientation_r1, robot_size_r1,
				 num_sensors_r1, max_sensors_range_r1,
				 sensors_r1, sensors_angles_r1, colors_r1,
				 robot_1_font);



  matrix1 = G_create_matrix_object(matrix_1_pos, matrix_1_text, 
				   &(matrix_1_values[0][0]), 
				   &(matrix_1_active[0][0]), 
				   size_matrix1_x, size_matrix1_y, 
				   min_value_matrix1, max_value_matrix1, 
				   matrix_1_colors, matrix_1_font);
     

  markers1 =
    G_create_markers_object(pos_mar1, connected_mar1, markers_size_mar1,
			    min_value_x_mar1, max_value_x_mar1,  
			    min_value_y_mar1, max_value_y_mar1 ,
			    mar1_num, text_mar1, mar1_background_color,  
			    mar1_frame_color, mar1_foreground_color, 
			    mar1_text_color, mar1_fonts);

  G_display_all();




  do{
    if (G_test_mouse(1)){
      mouse_events = G_evaluate_mouse(&mouse_x, &mouse_y, 
				      &button, &num_mouse_events);
      
      if (num_mouse_events == 0 && button == RIGHT_BUTTON)
	G_display_all();

      else
	for (i = 0; i < num_mouse_events; i++)

	  if (mouse_events[i].type == G_SWITCH) /* SWITCH EVENT */
	    G_display_switch(mouse_events[i].name,
			     mouse_events[i].actual_text+1);

	  else if (mouse_events[i].type == G_VALUE) /* VALUE EVENT */
	    G_display_value(mouse_events[i].name,  
			    mouse_events[i].value);

	  else if (mouse_events[i].type == G_MATRIX){ /* MATRIX EVENT */
	    if (mouse_events[i].name == matrix1)
	      matrix_1_active[mouse_events[i].index_x][mouse_events[i].index_y]
		= 1 - mouse_events[i].active; /* refers to the same value! */
	    else
	      printf("Oops - unknown matrix name found.\n");
	    G_display_matrix(mouse_events[i].name);
	  }
      
	  else if (mouse_events[i].type == G_ROBOT){ /* ROBOT EVENT */
	    if (mouse_events[i].name == robot1){
	      robot_x_r1 += mouse_events[i].robot_delta_x;
	      robot_y_r1 += mouse_events[i].robot_delta_y;
	      robot_orientation_r1 += mouse_events[i].robot_delta_orientation;

	      G_display_robot(mouse_events[i].name,
			      robot_x_r1, robot_y_r1, robot_orientation_r1,
			      -1, NULL);
	    }
	    else
	      printf("Oops - unknown robot name found.\n");
	  }
      
	  else if (mouse_events[i].type == G_MARKERS){ /* MARKERS EVENT */
	    if (mouse_events[i].marker_name >= 0)
	      G_delete_marker(mouse_events[i].name,mouse_events[i].value_x,
			      mouse_events[i].value_y);
	    else 
	      G_add_marker(mouse_events[i].name, mouse_events[i].value_x,
			   mouse_events[i].value_y, button);
	    G_display_markers(mouse_events[i].name);
	  }
      
      /* XXX add new object types here */
	  else
	    printf("STRANGE4: Object #%d type %d unknown!\n", 
		   i, mouse_events[i].type);
    }
  }
  while (button != MIDDLE_BUTTON || num_mouse_events > 0);
}


#ifdef main_defined

main(){
  G_test_graphics();
}

#endif


/* Ugly glue to make screen refreshes work */
void check_redraw_screen() {
  XEvent ev;

  if (XCheckMaskEvent(theDisplay, ExposureMask, &ev)) {
    /* Eat multiple events */
    while (XCheckMaskEvent(theDisplay, ExposureMask, &ev));

    G_display_all();
  }
}


/* -------- FILE ENDS HERE ----------------- */



/************************************************************************
 *
 *   NAME:         G_
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

