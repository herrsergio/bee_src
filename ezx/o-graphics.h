
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








/************************************************************************
 *
 *   FILENAME:         o-graphics.h
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


#include "EZX11.h"

extern EZXW_p w_ptr;			/* Pointer to simulator window */


/*-------------------------------------------------------*/



#define G_NO_OBJECT 0
#define G_SWITCH    1
#define G_VALUE     2
#define G_MATRIX    3
#define G_ROBOT     4
#define G_MARKERS   5


#define NO_COLOR -1



/*-------------------------------------------------------*/
/* ======== Mouse variables ======== */



typedef struct{
  float x, y;			/* absolute x, y */
  int button;			/* button no */
  int name;			/* name of the graphics object */
  int type;			/* type of the object */
  float rel_x, rel_y;		/* relative coordinates */
  int actual_text;              /* only defined for switch objects */
  float value;	                /* defined only for value/matrix objetcs */
  float value_x, value_y;	/* defined only for robot */
  int active;	                /* defined only for matrix objetcs */
  int index_x, index_y;		/* defined only for matrix objects */
  float robot_delta_x, robot_delta_y;  /* only defined for robot window */
  float robot_delta_orientation; /* only defined for robot window */
  int marker_name;		/* defined only for markers */
  int marker_type;		/* defined only for markers */
  float marker_x, marker_y;	/* defined only for markers */
} G_mouse_type, *G_mouse_ptr;


extern int  G_markers_display_style; /* 1=fill them up */
extern int  G_invert_display;


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
			   int background_color);
  


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


void G_initialize_fonts(int num_fonts, char *fonts[]);





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


void G_initialize_window(void);







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
			   int *fonts);



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
			  int font);




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
 *                 int   size_matrix_x, size_matrix_y;  virtual domension of
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
			   int    font);





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
		     int    size_matrix_y);




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
 *                 float max_sensors_range  Max. sensor range (local coord)
 *                 float *sensors          Initial values of sensors
 *                 float *sensor_angles    Angles of the sensors
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
			  int font);




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
			    int  *fonts);



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

void G_shift_robot_local_coordinates(int name, float shift_x, float shift_y);




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

void G_shift_markers_local_coordinates(int name, float shift_x, float shift_y);



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

void G_clear_markers(int name);




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


int G_object_type(int name);



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


void G_display_switch(int name, int actual_text);







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


void G_display_value(int name, float value);






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


void G_display_partial_matrix(int name, 
			      int from_x, int num_x, 
			      int from_y, int num_y);




/************************************************************************
 *
 *   NAME:         G_display_matrix
 *                 
 *   FUNCTION:     Displays a matrix window
 *                 
 *   PARAMETERS:   int name         pointer to the object to display 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void G_display_matrix(int name);



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



void G_matrix_set_display_range(int name, float min_value, float max_value);


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

void G_set_matrix_display_style(int style);



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
		     float *sensors);





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


void G_display_markers(int name);





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


void G_undisplay_markers(int name, int number, int COLOR);





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


void G_add_marker(int name, float x, float y, int type);
    


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


void G_delete_marker(int name, float x, float y);




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


int G_return_num_markers(int name, int mode);
    



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
				 float *x, float *y, int *type);


    

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


void G_display_all();




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


void G_set_display(int d);



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


void G_activate(int name);



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


void G_deactivate(int name);

  

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


void G_set_new_text(int name, char *new_text, int text_nr);





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




int G_test_mouse(int wait);




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



G_mouse_ptr G_evaluate_mouse(float *x, float *y, int *button, int *num_events);




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



int G_mouse_event_at(int name, G_mouse_ptr mouse_events, int *number);




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

void G_test_graphics();



	     
