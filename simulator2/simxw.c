
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


#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/Shell.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/Command.h>
#include <X11/Xaw/Toggle.h>
#include <X11/Xaw/Label.h>
#include <X11/Xaw/Dialog.h>
#include <X11/Xaw/Viewport.h>
#include <X11/Xaw/Text.h>
#include <X11/Xaw/AsciiText.h>
#include <X11/Xaw/MenuButton.h>
#include <X11/Xaw/SimpleMenu.h>
#include <X11/Xaw/SmeBSB.h>

#include "Drawing.h"
#include "base.h"
#include "trigofkt.h"
#include "robot.h"
#include "playground.hh"
#include "simxw.h"
#include "SIMULATOR-messages.h"
#include "sonar.h"


#define PLACE_ROBOT 0
#define DELETE_OBSTACLE 1
#define SWITCH_OBSTACLE 2
#define CONTROL_OBSTACLE 3
#define INSERT_OBSTACLE 4

#define DW_FigurePos (DW_FigureX|DW_FigureY)
#define DW_FigureDim (DW_FigureWidth|DW_FigureHeight)

/*****************************************************************************/

Display *theDisplay;

Figure RobotFig, robot_circle, robot_line;
FigureAttributes rc_attrs, rl_attrs;

Figure robot_elements[2];
Figure beam[B_MAX_SENSOR_COLS];
FigureAttributes beam_attrs[B_MAX_SENSOR_COLS];
Figure laserbeam[360];
FigureAttributes laserbeam_attrs[360];

Figure traceLines[1001];         /* for visualising the trace */ 
int NtraceLines = 0;
Boolean traceOn = FALSE;
Boolean sonarOn = FALSE;
Boolean laserOn = FALSE;


#define RECTANGLE 0
#define CIRCLE 1
#define DOOR 2
#define HUMAN 3

#define OBSTACLE_TYPES 4
char *obstacle_type[OBSTACLE_TYPES] =
  {"Rectangle", "Circle", "Door", "Human"};  

Boolean editing = FALSE;
int editing_mode = PLACE_ROBOT;
int edit_obstacle_type = RECTANGLE;
extern Boolean just_viewing;
extern Boolean without_laser;

static float
       canvas_minx,canvas_miny,
       canvas_maxx,canvas_maxy,
       canvas_ysize,canvas_xsize;

String default_resources[] = {
  
"*background:		LightBlue",
"*simQuit.background:			red",
"*simSave.background:			RoyalBlue2",
"*simLoad.background:			RoyalBlue1",
"*simFileDialog*translations; #override \\\n\
<Key>Return:                  no-op(RingBell)\n",
"*simGridToggle.background:			grey70",
"*simTraceToggle.background:			gold",
"*simHelpToggle.background:			orange",
"*simSonarToggle.background:			PaleVioletRed",
"*simLaserToggle.background:			blue",
"*simZoomIn.background:			LightSlateBlue",
"*simZoomOut.background:		DarkSlateBlue",
"*simView.background:			LightBlue",
"*simCanvas.background:			grey80",
"*simCanvasCoords.background:		grey80",
"*simObstacleType.background:		grey80",
"*simDeleteObstacle.background:		grey80",
"*simSwitchObstacle.background:	        grey80",
"*simMoveObstacle.background:		grey80",
"*simInsertObstacle.background:		grey80",
"*simObstacleMenuButton.background:	grey80",
"*simRobotCenter.background:		gold",
"*simRobotCoords.background:		grey80",
"*simCanvas.translations:	#override \\\n\
!!! Shift<Btn1Down>:		InsertRectangle(start)\\n\\\n\
!!! Shift<Btn1Motion>:		InsertRectangle(drag)\\n\\\n\
!!! Shift<Btn1Up>:		InsertRectangle(stop)\\n\\\n\
!!! Shift<Btn2Down>:		InsertCircle(start)\\n\\\n\
!!! Shift<Btn2Motion>:		InsertCircle(drag)\\n\\\n\
!!! Shift<Btn2Up>:		InsertCircle(stop)\\n\\\n\
!!! Shift<Btn3Down>:	DeleteObstacle()\\n\\\n\
!!! <Btn3Down>:	        ToggleObstacle()\\n\\\n\
!!! Ctrl<Btn1Down>:		ObstacleAction(start)\\n\\\n\
!!! Ctrl<Btn1Motion>:	ObstacleAction(drag)\\n\\\n\
<Btn2Down>:		SetTarget()\\n\\\n\
<Btn1Down>:		SelectedAction(start)\\n\\\n\
<Btn1Up>:		SelectedAction(stop)\\n\\\n\
<Btn1Motion>:		SelectedAction(drag)\n"};


XtAppContext sim_context;
Widget topLevel,
       simMainForm,
       simQuit,
       simSave,
       simLoad,
       simFileButtonDesc,
       simView,
       simCanvas,
       simCanvasCoords,
       simCanvasCoLabel,
       simRobotCoLabel,
       simRobotCoords,
       simRobotCenter,
       simVertButtonForm,
       simMapButtonDesc,
       simZoomIn,
       simZoomOut,
       simSonarToggle,
       simLaserToggle,
       simGridToggle,
       simHelpToggle,
       simObstacleButtonDesc,
       simObstacleType,
       simObstacleMenuButton,
       simObstacleMenu,
       simDeleteObstacle,
       simSwitchObstacle,
       simMoveObstacle,
       simInsertObstacle,
       simHorizButtonForm,
       simRobotButtonDesc,
       simTraceToggle,
       help_popup,
       help_text;


/* -------------------------- some helper functions ----------------- */

void
SetValue(Widget w, String resource, XtArgVal val)
{
  Arg arg = {resource, val};
  XtSetValues(w, &arg, 1); 
}

void
GetValue(Widget w, String resource, XtArgVal val)
{
  Arg arg = {resource, val};
  XtGetValues(w, &arg, 1); 
}

void
GetCoords (widget, event, x, y)
    Widget  widget;
    XEvent *event;
    float  *x;
    float  *y;
{
    int event_x;
    int event_y;


    switch (event -> type) {
    case KeyPress:
    case KeyRelease:
	event_x = event -> xkey.x;
	event_y = event -> xkey.y;
	break;


    case ButtonPress:
    case ButtonRelease:
	event_x = event -> xbutton.x;
	event_y = event -> xbutton.y;
	break;


    case MotionNotify:
	event_x = event -> xmotion.x;
	event_y = event -> xmotion.y;
	break;


    default:
	event_x = 0;
	event_y = 0;
	break;
    }


    DW_TranslateCoords (widget, event_x, event_y, x, y);
}

/* ------------------ Toggle button Callbacks ----------------------- */

void
CenterRobotCallback(XtPointer, XtIntervalId*);

void
QuitCallback(Widget w, XtPointer client_data, XtPointer call_data)
{
  exit(0); 
}

void
toggleGridCallback(Widget w, XtPointer client_data, XtPointer call_data)
{
  Boolean state;
  GetValue(w, XtNstate, (XtArgVal) &state);
  SetValue(simCanvas, XtNgrid, state);
}

void
DisplayHelp();

void
toggleHelpCallback(Widget w, XtPointer client_data, XtPointer call_data)
{
  Boolean state;
  GetValue(w, XtNstate, (XtArgVal) &state);
  if(state) DisplayHelp();
  else   XtDestroyWidget(help_popup);
}

void
zoomInCallback(Widget w, XtPointer client_data, XtPointer call_data)
{
  Arg   args [2];
  float x_scale;
  float y_scale;
  Dimension width,height;
  XtSetArg (args [0], XtNxScale, &x_scale);
  XtSetArg (args [1], XtNyScale, &y_scale);
  XtGetValues (simCanvas, args, 2);
  XtSetArg(args [0], XtNwidth, &width);
  XtSetArg(args [1], XtNheight, &height);  
  XtGetValues (simCanvas, args, 2);
  if(32767 < (int)width*2) return;
  if(32767 < (int)height*2) return;  
  
  x_scale *= 2;
  y_scale *= 2;
  XtSetArg (args [0], XtNxScale, Float2Arg (x_scale));
  XtSetArg (args [1], XtNyScale, Float2Arg (y_scale));
  XtSetValues (simCanvas, args, 2);
  CenterRobotCallback(NULL,NULL);
}

void
zoomOutCallback(Widget w, XtPointer client_data, XtPointer call_data)
{
  Arg   args [2];
  float x_scale;
  float y_scale;
  Dimension v_width,v_height;
  Dimension c_width,c_height;  
  XtSetArg (args [0], XtNwidth, &v_width);
  XtSetArg (args [1], XtNheight, &v_height);    
  XtGetValues(simView, args, 2);  
  XtSetArg (args [0], XtNwidth, &c_width);
  XtSetArg (args [1], XtNheight, &c_height);    
  XtGetValues(simCanvas, args, 2);  
  if((int)v_width >= (int)c_width &&
     (int)v_height >= (int)c_height) return;
  
  XtSetArg (args [0], XtNxScale, &x_scale);
  XtSetArg (args [1], XtNyScale, &y_scale);
  XtGetValues (simCanvas, args, 2);

  x_scale /= 2;
  y_scale /= 2;
  XtSetArg (args [0], XtNxScale, Float2Arg (x_scale));
  XtSetArg (args [1], XtNyScale, Float2Arg (y_scale));
  XtSetValues (simCanvas, args, 2);
  CenterRobotCallback(NULL,NULL);
}

void
obstacleModeCallback(Widget w, XtPointer client_data, XtPointer call_data)
{
  Boolean state;
  editing = FALSE;
  GetValue(w, XtNstate, (XtArgVal) &state);
  if(!state) {
    editing_mode = PLACE_ROBOT;
    return;
  } else if(w == simDeleteObstacle) {
      editing = TRUE;
      editing_mode = DELETE_OBSTACLE;
      return;
  } else if(w == simSwitchObstacle) {
    editing_mode = SWITCH_OBSTACLE;
    return;
  } else if(w == simMoveObstacle) {
    editing_mode = CONTROL_OBSTACLE;
    return;
  } else if(w == simInsertObstacle) {
    editing_mode = INSERT_OBSTACLE;
    return;
  }
}

void
toggleSonarCallback(Widget w, XtPointer client_data, XtPointer call_data)
{
  Boolean state;
  Figure Beams;
  GetValue(w, XtNstate, (XtArgVal) &state);
  if(!state) {
    Beams = DW_Group(simCanvas, beam, bRobot.sonar_cols[0]);
    DW_RemoveFigure(simCanvas, Beams);
    DW_Redraw(simCanvas);
  }
  else {
    int i;
    DW_SetForeground(simCanvas,"purple");
    for(i = 0; i < bRobot.sonar_cols[0]; i++) {
      beam[i] = DW_DrawLine(simCanvas, 0, 0, 0, 0);
      DW_GetAttributes(simCanvas, beam[i],&beam_attrs[i]);
    }
    DW_SetForeground(simCanvas,WALLCOLOR);
  }
  sonarOn = state;
}

void
FileRequest(char*);

void
fileCallback(Widget w, XtPointer client_data, XtPointer call_data)
{
  FileRequest((char *) client_data);  
}

void
toggleLaserCallback(Widget w, XtPointer client_data, XtPointer call_data)
{
  Boolean state;
  Figure Beams;
  GetValue(w, XtNstate, (XtArgVal) &state);
  if(!state) {
    Beams = DW_Group(simCanvas, laserbeam, 360);
    DW_RemoveFigure(simCanvas, Beams);
    DW_Redraw(simCanvas);
  }
  else {
    int i;
    DW_SetForeground(simCanvas,"blue");    
    for(i = 0; i < 360; i++) {
      laserbeam[i] = DW_DrawLine(simCanvas, 0, 0, 0, 0);
      DW_GetAttributes(simCanvas, laserbeam[i],&laserbeam_attrs[i]);
    }    
    DW_SetForeground(simCanvas,WALLCOLOR);
  }
  laserOn = state;
}

void
toggleTraceCallback(Widget w, XtPointer client_data, XtPointer call_data)
{
  Boolean state;
  Figure Trace;
  GetValue(w, XtNstate, (XtArgVal) &state);
  if(!state) {
    Trace = DW_Group(simCanvas, traceLines, NtraceLines);
    DW_RemoveFigure(simCanvas, Trace);
    DW_Redraw(simCanvas);
    NtraceLines = 0;
  }
  traceOn = state;
}



void
selectObstacleType(Widget w, XtPointer type, XtPointer call_data)
{
  SetValue(simObstacleType, XtNlabel, (XtArgVal) obstacle_type[(int) type]);
  edit_obstacle_type = (int) type;
}

/* ----------------------------- Canvas operation callbacks -------------- */

void
reportCoordCallback(Widget w, XtPointer clientdata, DrawingReport* report)
{
    float x,y;
    Arg arg[1];
    char labelstring[80];
    GetCoords(w, report->event, &x,&y);
    sprintf(labelstring,"% 6.1f,% 6.1f",x,y);    
    XtSetArg(arg[0], XtNlabel, (XtArgVal) labelstring);
    XtSetValues(simCanvasCoords, arg, 1);
}
/* ------------- routines for obstacle insertions ------------------- */


		       /* keep one corner point or the center point pixed */
		       /* used for rubber banding */
float fixpoint_x;
float fixpoint_y;


/* resizing a rectangle using rubber banding */

void
dragRectangle(Widget w, XEvent *event, Figure nrect) {
  float x1, y1, x2, y2;
  FigureAttributes r_attrs;
  if(!editing) return;
  GetCoords(w, event, &x2, &y2);
  DW_GetAttributes(w, nrect, &r_attrs);
		       /* rectangles have to be drawn from the lower */
                       /* left corner */
  if(x2 <= fixpoint_x) {
    x1 = x2;
    x2 = fixpoint_x;
  } else x1 = fixpoint_x;
  if(y2 <= fixpoint_y) {
    y1 = y2;
    y2 = fixpoint_y;
  } else y1 = fixpoint_y;
  
  r_attrs.x = x1;
  r_attrs.y = y1;
  r_attrs.width = x2 - x1;
  r_attrs.height = y2 - y1;
  DW_SetAttributes(w, nrect, (DW_FigurePos|DW_FigureDim), &r_attrs);
}

/* rectangle insert action the action is assumed to have one argument with 
   one of the values 
   start: start drawing a new rectangle 
   drag:  resize the rectangle currently to be drawn 
   stop:  turn of rubber banding, draw rectangle
*/
static Figure edit_fig;

void
insertRectangle(Widget w, XEvent *event,
		     String *params, Cardinal *num_params)
{
  float x1, y1;
  if(num_params == 0) return;
  if(strcmp(params[0], "start") == 0) {
    editing = TRUE;
    DW_SetInteractive(w, TRUE);
    GetCoords(w, event, &x1, &y1);
    DW_SetForeground(w, WALLCOLOR);
    edit_fig = DW_FillRectangle(w, TRUE, x1, y1, 0, 0);
    DW_SetForeground(w, WALLCOLOR);
    fixpoint_x = x1;
    fixpoint_y = y1;    
    return;
  }
  else if(strcmp(params[0], "stop") == 0) {
    FigureAttributes r_attrs;
    if(!editing) return;
    DW_SetInteractive(w, FALSE);
    dragRectangle(w, event, edit_fig);
    DW_GetAttributes(w, edit_fig, &r_attrs);
    DW_RemoveFigure(w, edit_fig);
    new_rectangle(r_attrs.x,r_attrs.y,
	       r_attrs.width+r_attrs.x,r_attrs.height+r_attrs.y);
    editing = FALSE;
    return;
  }
  dragRectangle(w, event, edit_fig);
}

void
insertDoor(Widget w, XEvent *event,
		     String *params, Cardinal *num_params)
{
  float x1, y1;
  if(num_params == 0) return;
  if(strcmp(params[0], "start") == 0) {
    editing = TRUE;
    DW_SetInteractive(w, TRUE);
    GetCoords(w, event, &x1, &y1);
    DW_SetForeground(w, WALLCOLOR);
    edit_fig = DW_FillRectangle(w, TRUE, x1, y1, 0, 0);
    DW_SetForeground(w, WALLCOLOR);
    fixpoint_x = x1;
    fixpoint_y = y1;    
    return;
  }
  else if(strcmp(params[0], "stop") == 0) {
    FigureAttributes r_attrs;
    if(!editing) return;
    DW_SetInteractive(w, FALSE);
    dragRectangle(w, event, edit_fig);
    DW_GetAttributes(w, edit_fig, &r_attrs);
    DW_RemoveFigure(w, edit_fig);
    if(fixpoint_x == r_attrs.x && fixpoint_y == r_attrs.y)
      new_door(fixpoint_x,fixpoint_y,
	       r_attrs.width,r_attrs.height);
    else if( fixpoint_x == r_attrs.x)
      new_door(fixpoint_x,fixpoint_y,
	       r_attrs.width, -r_attrs.height);
    else if( fixpoint_y == r_attrs.y)
      new_door(fixpoint_x,fixpoint_y,
	       -r_attrs.width,r_attrs.height);
    else
      new_door(fixpoint_x,fixpoint_y,
	       -r_attrs.width,-r_attrs.height);
    editing = FALSE;
    return;
  }
  dragRectangle(w, event, edit_fig);
}

/* ---------------------------------------------------------------- */

/* resize the circle currently to be drawn, keep the center point fixed */

void
dragCircle(Widget w, XEvent *event, Figure ncircle)
{
  float x1, y1, x2, y2, dx, dy;
  FigureAttributes c_attrs;
  if(!editing) return;
  GetCoords(w, event, &x2, &y2);
  DW_GetAttributes(w, edit_fig, &c_attrs);
  if(x2 <= fixpoint_x) {
    x1 = x2;
    x2 = fixpoint_x;
  } else x1 = fixpoint_x;
  if(y2 <= fixpoint_y) {
    y1 = y2;
    y2 = fixpoint_y;
  } else y1 = fixpoint_y;
  dx = x2 - x1;
  dy = y2 - y1;
  c_attrs.width = (float) sqrt(dx*dx + dy*dy);
  c_attrs.height = c_attrs.width;
  DW_SetAttributes(w, edit_fig, (DW_FigurePos|DW_FigureDim), &c_attrs);
}

/* same as insertRectangle but for circles (see above) */
void
insertCircle(Widget w, XEvent *event,
		     String *params, Cardinal *num_params)
{
  float x1, y1;
  if(num_params == 0) return;
  if(strcmp(params[0], "start") == 0) {
    editing = TRUE;
    DW_SetInteractive(w, TRUE);
    GetCoords(w, event, &x1, &y1);
    DW_SetForeground(w, WALLCOLOR);
    edit_fig = DW_FillArc(w, TRUE, x1, y1, 0.0, 0.0, 0.0, 360.0);
    DW_SetForeground(w, WALLCOLOR);
    fixpoint_x = x1;
    fixpoint_y = y1;    
    return;
  }
  else if(strcmp(params[0], "stop") == 0) {
    FigureAttributes c_attrs;
    if(!editing) return;
    DW_SetInteractive(w, FALSE);
    dragCircle(w, event, edit_fig);
    DW_GetAttributes(w, edit_fig, &c_attrs);
    DW_RemoveFigure(w, edit_fig);
    new_circle(c_attrs.x,c_attrs.y,
	       c_attrs.width/2);
    editing = FALSE;
    return;
  }
  dragCircle(w, event, edit_fig);
}

void
dragLine(Widget w, XEvent *event, Figure nline)
{
  float x2, y2, dx, dy;
  FigureAttributes l_attrs;
  if(!editing) return;
  GetCoords(w, event, &x2, &y2);
  DW_GetAttributes(w, edit_fig, &l_attrs);
  l_attrs.points[0].x = fixpoint_x;
  l_attrs.points[0].y = fixpoint_y;  
  l_attrs.points[1].x = x2;
  l_attrs.points[1].y = y2;  
  DW_SetAttributes(w, edit_fig, (DW_FigurePoints), &l_attrs);
}


void
insertHuman(Widget w, XEvent *event,
		     String *params, Cardinal *num_params)
{
  float x1, y1,x2,y2,dx,dy,speed,angle;
  if(num_params == 0) return;
  if(strcmp(params[0], "start") == 0) {
    editing = TRUE;
    DW_SetInteractive(w, TRUE);
    GetCoords(w, event, &x1, &y1);
    DW_SetForeground(w, WALLCOLOR);
    edit_fig = DW_DrawLine(w, x1, y1, x1, y1);
    DW_SetForeground(w, WALLCOLOR);
    fixpoint_x = x1;
    fixpoint_y = y1;    
    return;
  }
  else if(strcmp(params[0], "stop") == 0) {
    FigureAttributes r_attrs;
    if(!editing) return;
    DW_SetInteractive(w, FALSE);
    dragLine(w, event, edit_fig);
    DW_GetAttributes(w, edit_fig, &r_attrs);
    DW_RemoveFigure(w, edit_fig);
    x1 = r_attrs.points[0].x;
    y1 = r_attrs.points[0].y;
    x2 = r_attrs.points[1].x;
    y2 = r_attrs.points[1].y;
    dx = x2-x1;
    dy = y2-y1;    
    speed = sqrt(dx*dx+dy*dy);
    if(speed > 0.001) {
	if(dx == 0) {
	    if(dy > 0) angle = M_PI;
	    else angle = -M_PI;
	}
	else angle = atan(dy/dx);
	if(dx < 0) angle += M_PI;
	new_human(fixpoint_x, fixpoint_y, speed, angle);
    }
    editing = FALSE;
    return;
  }
  dragLine(w, event, edit_fig);
}


void
abortEditing(Widget w, XEvent *event,
	     String *params, Cardinal *num_params)
{
  if(!editing) return;
  DW_RemoveFigure(w, edit_fig);
  DW_SetInteractive(w, FALSE);  
  editing = FALSE;
}

void
RemoveFigure(Figure fig)
{
    DW_RemoveFigure(simCanvas, fig);
}

void
deleteObstacle(Widget w, XEvent *event,
	     String *params, Cardinal *num_params)
{
  float x1, y1;
  GetCoords(w, event, &x1, &y1);  
  RemoveObstacle(x1,y1);
}

void
toggleObstacle(Widget w, XEvent *event,
	     String *params, Cardinal *num_params)
{
  float x1, y1;
  GetCoords(w, event, &x1, &y1);  
  ToggleObstacle(x1,y1);
}

/* ------------------------------------------------------------------------- */
/* routines for manipulating the playground robot, obstacles and target */

void
placeRobot(Widget w, XEvent *event,
		     String *params, Cardinal *num_params)
{
  float x1, y1;
  GetCoords(w, event, &x1, &y1);
  if(x1 > canvas_maxx - robot.RobotRadius || x1 < canvas_minx+robot.RobotRadius)
    return;
  if(y1 > canvas_maxy - robot.RobotRadius || y1 < canvas_miny+robot.RobotRadius)
    return;  
  if(InsideObstacle(x1,y1)) return;
  BeamRobot(x1, y1);
}


/* stubs for obstacle manipulation */

void
obstAction(Widget w, XEvent *event,
		     String *params, Cardinal *num_params)
{
    float x1, y1;
    if(num_params == 0) return;
    GetCoords(w, event, &x1, &y1);
    if(strcmp(params[0], "start") == 0)
      ObstaclePushAction(x1,y1);
    else if(strcmp(params[0], "drag") == 0)
      ObstacleDragAction(x1,y1);    
}

/* action for setting the target point at the current mouse position */  

static Boolean target_set = 0;
void
setTarget(Widget w, XEvent *event,
		     String *params, Cardinal *num_params)
{
  static float x1, y1;
  static Figure target;
  FigureAttributes t_attrs;
  GetCoords(w, event, &x1, &y1);
  if(x1 > canvas_maxx || x1 < canvas_minx) return;
  if(y1 > canvas_maxy || y1 < canvas_miny) return;    
  if(InsideObstacle(x1,y1)) return;
  if(!target_set) {
    target = DW_FillArc(w, TRUE, x1, y1,
			20, 20, 0, 360);
    DW_GetAttributes(w, target, &t_attrs);
    t_attrs.color = "red";
    DW_SetAttributes(w, target, DW_FigureColor, &t_attrs);
    target_set = TRUE;
  }
  else {
    DW_GetAttributes(w, target, &t_attrs);
    t_attrs.x = x1;
    t_attrs.y = y1;    
    DW_SetAttributes(w, target, DW_FigurePos, &t_attrs);    
  }
  robot_setTarget(x1,y1);
}

void
insertObstacle(Widget w, XEvent *event,
		     String *params, Cardinal *num_params)
{
  if(num_params == 0) return;
  switch(edit_obstacle_type) {
  case RECTANGLE:
    insertRectangle(w, event, params, num_params);
    break;
  case CIRCLE:
    insertCircle(w, event, params, num_params);
    break;
  case DOOR:
    insertDoor(w, event, params, num_params);
    break;
  case HUMAN:
    insertHuman(w, event, params, num_params);
    break;
  }
}

void
selectedAction(Widget w, XEvent *event,
		     String *params, Cardinal *num_params)
{
  if(num_params == 0) return;
  switch(editing_mode) {
  case PLACE_ROBOT:
    if(strcmp(params[0],"start") == 0)        
      placeRobot(w, event, params, num_params);
    break;
  case DELETE_OBSTACLE:
    if(strcmp(params[0],"start") == 0) 
      deleteObstacle(w, event, params, num_params);
    break;
  case SWITCH_OBSTACLE:
    if(strcmp(params[0],"start") == 0)
      toggleObstacle(w, event, params, num_params);
    break;
  case INSERT_OBSTACLE:
    insertObstacle(w, event, params, num_params);
    break;
  case CONTROL_OBSTACLE:
    obstAction(w, event, params, num_params);
    break;
  }
}


/* ------------------------- exposures ---------------------------------- */

/* these functions handle the exposure of obstacles read from a mapfile */
/* and after manual mannipulation */

/* debugging hack! */
void
place_dot(float x, float y, float height, float angle)
{
  Figure dot;
  float x1 = x - height/2 * sin(angle);
  float y1 = y + height/2 * cos(angle);
  DW_SetForeground(simCanvas, "violet");
  dot = DW_FillArc(simCanvas, TRUE, x1-5,y1-5, 10, 10, 0, 360); 
  DW_RaiseFigure(simCanvas,dot);
  DW_SetForeground(simCanvas, WALLCOLOR);
}

Figure
expose_rectangle(float x1, float y1, float x2, float y2, char* color)
{
    Figure fig;
    float width = x2 - x1;
    float height = y2 - y1;
    DW_SetForeground(simCanvas, color);
    fig = DW_FillRectangle(simCanvas, TRUE, x1, y1,
		     x2-x1, y2-y1 );
    DW_SetForeground(simCanvas,WALLCOLOR);
    return fig;
}

Figure
expose_disabled_rectangle(float x1, float y1, float x2, float y2, char* color)
{
    Figure fig;    
    DW_SetForeground(simCanvas,color);
    fig = DW_DrawRectangle(simCanvas, TRUE, x1, y1,
		     x2-x1, y2-y1 );
    DW_SetForeground(simCanvas,WALLCOLOR);
    return fig;
}

Figure
redraw_rectangle(Figure rect, float x1, float y1, float x2, float y2)
{
    FigureAttributes rectattr;
    DW_GetAttributes(simCanvas, rect,&rectattr);
    rectattr.x = x1;
    rectattr.y = y1;
    rectattr.width = x2-x1;
    rectattr.height = y2-y1;
    DW_SetAttributes(simCanvas, rect, DW_FigureDim|DW_FigurePos, &rectattr);
    return rect;
}

Figure
redraw_circle(Figure circ, float x1, float y1, float r)
{
    FigureAttributes circattr;
    DW_GetAttributes(simCanvas, circ,&circattr);
    circattr.x = x1;
    circattr.y = y1;
    circattr.width = 2*r;
    circattr.height = 2*r;    
    DW_SetAttributes(simCanvas, circ, DW_FigureDim|DW_FigurePos, &circattr);
    return circ;
}


#define is_rectangle(x, y) \
  ( (x[0] == x[1]) && \
    (x[2] == x[3]) && \
    (y[1] == y[2]) && \
    (y[0] == y[3]))

Figure
expose_polygon(int n, float *x, float *y, char* color)
{
    int i;
    Figure fig;
    Point points[n+1];
    if(is_rectangle(x,y)) {
	fig = expose_rectangle(x[0],y[0],x[2],y[2],color);
    }
    else {
	for(i = 0; i <= n; i++) {
	    points[i].x = x[i%n];
	    points[i].y = y[i%n];
	}
	DW_SetForeground(simCanvas,color);
	fig = DW_FillPolygon(simCanvas, TRUE, points, n+1);
	DW_SetForeground(simCanvas,WALLCOLOR);
    }
    return fig;
}

Figure
expose_disabled_polygon(int n, float *x, float *y, char* color)
{
    int i;
    Figure fig;
    Point points[n+1];
    if(is_rectangle(x,y)) {
	fig = expose_disabled_rectangle(x[0],y[0],x[2],y[2],color);
    }
    else {
	for(i = 0; i <= n; i++) {
	    points[i].x = x[i%n];
	    points[i].y = y[i%n];
	}
	DW_SetForeground(simCanvas,color);
	fig = DW_DrawPolygon(simCanvas, TRUE, points, n+1);
	DW_SetForeground(simCanvas,WALLCOLOR);
    }
    return fig;
}



/* we use rectangles whenever possible, because polygons
   slow down Canvas refresh */

Figure
redraw_polygon(Figure poly, int n, float *x, float *y)
{
    FigureAttributes polyattr;
    DW_GetAttributes(simCanvas, poly, &polyattr);
    if(polyattr.type == RectangleFigure) {
	if(is_rectangle(x,y)) {
	    return redraw_rectangle(poly, x[0],y[0],x[2],y[2]);
	}
	else {
	    DW_RemoveFigure(simCanvas, poly);
	    return expose_polygon(n, x, y,polyattr.color);
	}
    }
    else {
	int i;
	for(i = 0; i <= n; i++) {
	    polyattr.points[i].x = x[i%n];
	    polyattr.points[i].y = y[i%n];
	}
	DW_SetAttributes(simCanvas, poly, DW_FigurePoints, &polyattr);
	return poly;
    }
}

Figure
redraw_disabled_polygon(Figure poly, int n, float *x, float *y)
{
    FigureAttributes polyattr;
    DW_GetAttributes(simCanvas, poly, &polyattr);
    if(polyattr.type == RectangleFigure) {
	if(is_rectangle(x,y)) {
	    return redraw_rectangle(poly, x[0],y[0],x[2],y[2]);
	}
	else {
	    DW_RemoveFigure(simCanvas, poly);
	    return expose_disabled_polygon(n, x, y,polyattr.color);
	}
    }
    else {
	int i;
	for(i = 0; i <= n; i++) {
	    polyattr.points[i].x = x[i%n];
	    polyattr.points[i].y = y[i%n];
	}
	DW_SetAttributes(simCanvas, poly, DW_FigurePoints, &polyattr);
	return poly;
    }
}

Figure
expose_circle(float x1, float y1, float r, char *color)
{
  Figure fig;
  DW_SetForeground(simCanvas,color);  
  fig = DW_FillArc(simCanvas, TRUE, x1, y1,
		   2*r, 2*r, 0.0, 360.0);
  DW_SetForeground(simCanvas,WALLCOLOR);
  return fig;
}

Figure
expose_disabled_circle(float x1, float y1, float r, char *color)
{
  Figure fig;
  DW_SetForeground(simCanvas,color);  
  fig = DW_DrawArc(simCanvas, TRUE, x1, y1,
		   2*r, 2*r, 0.0, 360.0);
  DW_SetForeground(simCanvas,WALLCOLOR);
  return fig;
}

/* -------------------  draw the robot ---------------------------- */

void
drawTraceLine(float xo,float yo,float xn,float yn)
{
  if(editing) return;
  DW_SetForeground(simCanvas,"grey128");
  if (NtraceLines == 0){
    xo = xn;
    yo = yn;
  }
  traceLines[NtraceLines++] =
    DW_DrawLine(simCanvas, xo, yo, xn, yn);
		       /* if line buffer full -> group lines together */
  if(NtraceLines == 1000) {
    Figure traceGroup;
    traceGroup = DW_Group(simCanvas, traceLines, 1000);
    traceLines[0] = traceGroup;
    NtraceLines = 1;
  }
}


char  ro_coords[80];
void
set_robot_coords(float x, float y)
{
  Arg   args [1];
  sprintf(ro_coords, "% 6.1f,% 6.1f", x,y);  
  XtSetArg(args[0], XtNlabel, &ro_coords);
  XtSetValues(simRobotCoords, args,1);
}

static void
move_center_to_xy(float x, float y)
{
  float thumbV,thumbH,xpos,ypos;
  Widget horizBar = XtNameToWidget(simView, "horizontal");
  Widget vertBar = XtNameToWidget(simView, "vertical");  
  GetValue(horizBar, XtNshown, (XtArgVal) &thumbH);
  GetValue(vertBar, XtNshown, (XtArgVal) &thumbV);  
  xpos = (x-canvas_minx)/canvas_xsize - thumbH/2;
  ypos = 1.0 - (y-canvas_miny)/canvas_ysize - thumbV/2;
  if(xpos > 1.0) xpos = 1.0;
  if(ypos > 1.0) ypos = 1.0;  
  XawViewportSetLocation(simView,xpos,ypos);
}

void
CenterRobotCallback(XtPointer client_data, XtIntervalId *id)
{
  float x = rl_attrs.points[0].x;
  float y = rl_attrs.points[0].y;
  move_center_to_xy(x,y);
}

void
DrawRobotFigure(Boolean alarm,float x,float y,float r,float deg)
{
  if(editing) {
    return ;
  }
  rc_attrs.x = x;
  rc_attrs.y = y;
  if(alarm) rc_attrs.color = "orange";
  else rc_attrs.color = "red";
  rl_attrs.points[0].x = x;
  rl_attrs.points[0].y = y;
  set_robot_coords(x, y);
  rl_attrs.points[1].x = x + r * myCOS(-M_PI * deg / 180.0 );
  rl_attrs.points[1].y = y + r * mySIN(-M_PI * deg / 180.0);
  DW_SetAttributes(simCanvas, robot_circle, DW_FigurePos|DW_FigureColor, &rc_attrs);
  DW_SetAttributes(simCanvas, robot_line, DW_FigurePoints, &rl_attrs);
}

void
expose_beam(int i, float x1, float y1, float x2, float y2) {
  if(editing) return;
  beam_attrs[i].points[0].x = x1;
  beam_attrs[i].points[0].y = y1;
  beam_attrs[i].points[1].x = x2;
  beam_attrs[i].points[1].y = y2;
  DW_SetAttributes(simCanvas, beam[i], DW_FigurePoints, &beam_attrs[i]);
}

void
expose_laser_beam(int k, float x1, float y1, float x2, float y2) {
    if(editing) return;
    laserbeam_attrs[k].points[0].x = x1;
    laserbeam_attrs[k].points[0].y = y1;
    laserbeam_attrs[k].points[1].x = x2;
    laserbeam_attrs[k].points[1].y = y2;
    DW_SetAttributes(simCanvas, laserbeam[k], DW_FigurePoints, &laserbeam_attrs[k]);
}

void
change_color(Figure fig, char *color)
{
    FigureAttributes figattrs;
    DW_GetAttributes(simCanvas, fig, &figattrs);
    figattrs.color = color;
    DW_SetAttributes(simCanvas, fig, DW_FigureColor, &figattrs);
}

void automatic_redraw(Boolean Val)
{
  DW_SetAutoRedraw(simCanvas, Val);
}

/* ----------------------- Graphics initialisation -------------------- */


/* registered actions */

XtActionsRec
actions [ ] = {
    {"InsertRectangle", insertRectangle},
    {"InsertCircle", insertCircle},
    {"SetTarget", setTarget},
    {"PlaceRobot", placeRobot},
    {"ObstacleAction", obstAction},
    {"Abort",abortEditing},
    {"DeleteObstacle", deleteObstacle},
    {"ToggleObstacle", toggleObstacle},
    {"SelectedAction", selectedAction}    
};


void
InitRobotFigure(float x, float y, float r, float deg)
{
  float x2,y2;
  robot_circle = DW_FillArc(simCanvas, TRUE, x, y, 2*r, 2*r, 0, 360.0);
  DW_GetAttributes(simCanvas, robot_circle, &rc_attrs);
  rc_attrs.color = "red";
  DW_SetAttributes(simCanvas, robot_circle, DW_FigureColor, 
		   &rc_attrs);
  x2 = x+r * (float)cos(M_PI * deg / 180.0);
  y2 = y+r * (float)sin(M_PI * deg / 180.0);    
  robot_line = DW_DrawLine(simCanvas, x, y, x2, y2);
  DW_GetAttributes(simCanvas, robot_line, &rl_attrs);
  rl_attrs.color = "black";
  DW_SetAttributes(simCanvas, robot_line, DW_FigureColor, 
		   &rl_attrs);
  robot_elements[0] = robot_circle;
  robot_elements[1] = robot_line;
  RobotFig = DW_Group(simCanvas, robot_elements, 2);
  DW_SetForeground(simCanvas,WALLCOLOR);
  
  return;
  
}

/* ------------------------------------------------------------------------- */
/*                   file handling saving and loading                        */
/* ------------------------------------------------------------------------- */

Widget file_popup;
Widget file_dialog;
Widget file_ok;
Widget file_cancel;
#define FILENAME_LENGTH 256
char file_filename[FILENAME_LENGTH];

void
fileOkCallback(Widget w, XtPointer mode, XtPointer call_data)
{
    strncpy(file_filename, XawDialogGetValueString(file_dialog), 79);
    file_filename[FILENAME_LENGTH-1] = 0;
    if(strcmp(mode, "Save") == 0) {
      if (strlen(file_filename) > 0)
	SavePlayground(file_filename);
      else
	fprintf(stderr, "No file name given, nothing saved.");
    }
    else
      if ( strcmp(mode, "Load") == 0) {
	if (strlen(file_filename) > 0)
	  LoadPlayground(file_filename);
	else
	  fprintf(stderr, "No file name given, nothing loaded.");
      }
      else printf("Oops %s unknown mode!\n", mode);
    XtDestroyWidget(file_popup);
}

void
fileCancelCallback(Widget w, XtPointer client_data, XtPointer call_data)
{
    XtDestroyWidget(file_popup);    
}


char help_message[] = \
"---> press the HELP button again to remove this window <---\n\
\n\
Controlling the playground\n\
\n\
Find Robot: Change the visible area of the playground such that the
            robot is in the center or at least visible.\n
Zoom In:    Scale the visible area by 1/2 in each direction.\n\
Zoom Out:   Scale the visible area by 2 in each direction.\n\
            This button has no effect, if the hole playground is already\n\
            visible.\n\
Grid:       Toggle the display of a grid on the playground. The size\n\
            of a grid cell is 1 m*m.\n\
\n\
Handling obstacles\n\
\n\
There are four different tasks which can be performed on obstacles:\n\
\n\
-- delete an obstacle,\n\
-- activate/deactivate an obstacle,\n\
-- change configuration of an obstacle (e.g., open/close door, move chair)\n\
-- insert a new obstacle.\n\
\n\
One of these four actions can be activated by enabling the corresponding\n\
radio button. The selected action will take place after clicking\n\
the left mouse button on an obstacle in the playground.\n\
For the insert action, the currently selected obstacle type is displayed in\n\
the text field directly underneath the \"insert:\" button. You can select\n\
a different obstacle type using the pull down menu, which occurs after\n\
clicking the question mark. 
\n\
Controlling the robot\n\
\n\
Place robot:  Make sure obstacle handling is disabled, e.g. none of the\n\
              buttons delete, toggle, control and insert is pressed.\n\
              Clicking the left mouse button on the destination places\n\
              the robot.\n\
Set Target:   Click middle mouse button on the destination.\n\
\n\
Trace:        After enabling this toggle button a trace of the robots\n\
              movements will be drawn.\n\
Sonar:        Visualize the sonar beams.\n\
Laser:        Visualize the laser beams.\n\
\n\
File\n\
\n\
Load:         Load a map. Opens a filename requester first.\n\
              (not yet implemented)\n\
Save:         Save current map. Opens a filename requester first.\n\
\n";


void
DisplayHelp()
{
  help_popup = XtVaCreatePopupShell("simHelpPopup",
				    topLevelShellWidgetClass,
				    topLevel,
				    XtNlabel,"Help",
				    XtNx, 300,
				    XtNy, 300,
				    NULL);
  help_text = XtVaCreateManagedWidget(
				      "simHelpText",
				      asciiTextWidgetClass,
				      help_popup,
				      XtNeditType, XawtextRead,
				      XtNstring, help_message,
				      XtNwidth, 600,
				      XtNheight, 660,
					NULL);
  XtPopup(help_popup,XtGrabNone);
}

void
FileRequest(char *mode)
{
  file_popup = XtVaCreatePopupShell("simFilePopup",
				    transientShellWidgetClass,
				    topLevel,
				    XtNlabel,mode,
				    XtNx, 300,
				    XtNy, 300,
				    XtNwidth, 600,
				    NULL);
  file_dialog = XtVaCreateManagedWidget(
					"simFileDialog",
					dialogWidgetClass,
					file_popup,
					XtNlabel, "File name to use:",
					XtNvalue, file_filename,
					NULL);
  file_ok = XtVaCreateManagedWidget(
				    "simFileOkButton",
				    commandWidgetClass,
				    file_dialog,
				    XtNlabel, "Ok",
				    NULL);
  XtAddCallback(file_ok, XtNcallback, fileOkCallback, mode);
  file_cancel = XtVaCreateManagedWidget(
				    "simFileCancelButton",
				    commandWidgetClass,
				    file_dialog,
				    XtNlabel, "Cancel",
				    NULL);
  XtAddCallback(file_cancel, XtNcallback, fileCancelCallback, mode);    
  XtPopup(file_popup,XtGrabExclusive);
}

/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


void
InitPlaygroundGraphics(char *filename, float minx, float miny, float maxx, float maxy)
{
  Arg args[8];
  Dimension width;
  Dimension height;
  strcpy(file_filename, filename);      /* filename for file dialogbox */
  canvas_ysize = maxy - miny;
  canvas_xsize = maxx - minx;
  canvas_minx = minx;
  canvas_miny = miny;  
  canvas_maxx = maxx;
  canvas_maxy = maxy;  
  GetValue(simView, XtNwidth, (XtArgVal) &width);
  GetValue(simView, XtNheight, (XtArgVal) &height);  
  XtSetArg(args [0], XtNxScale, Float2Arg( 0.2) ); 
  XtSetArg(args [1], XtNyScale, Float2Arg( 0.2) );
  XtSetArg(args [2], XtNgrid, (FALSE));
  XtSetArg(args [3], XtNgridSize, Float2Arg(100.0));
  XtSetArg(args [4], XtNxMin, Float2Arg(minx));
  XtSetArg(args [5], XtNyMin, Float2Arg(miny));
  XtSetArg(args [6], XtNxMax, Float2Arg(maxx));
  XtSetArg(args [7], XtNyMax, Float2Arg(maxy));    
  XtSetValues (simCanvas, args, 8);
  move_center_to_xy((float)canvas_minx+canvas_xsize/2,
		    (float)canvas_miny+canvas_ysize/2);
}


void
InitWidgets(int *argc, char **argv)
{
  int i;
  topLevel = XtAppInitialize(
			     &sim_context,
			     "Simulator",
			     NULL, 0,
			     argc, argv,
			     default_resources,
			     NULL,
			     0);
  theDisplay = XtDisplay(topLevel);
  simMainForm = XtVaCreateManagedWidget(
					"simMainForm",
					formWidgetClass,
					topLevel,
					XtNheight, 570,
					XtNwidth, 780,
					NULL);
  simView = XtVaCreateManagedWidget(
				    "simView",
				    viewportWidgetClass,
				    simMainForm,
				    XtNwidth, 650,
				    XtNheight, 500,
				    XtNallowHoriz, TRUE,
				    XtNallowVert, TRUE,
				    XtNforceBars, TRUE,
				    XtNuseBottom, TRUE,
				    XtNleft, XtChainLeft,
				    XtNtop, XtChainTop,
				    XtNright, XtChainRight,
				    XtNbottom, XtChainBottom, 
				    NULL);
  
  simCanvas = XtVaCreateManagedWidget(
				      "simCanvas",
				      drawingWidgetClass,
				      simView,
				      XtNleft, XtChainLeft,
				      XtNtop, XtChainTop,
				      XtNright, XtChainRight,
				      XtNbottom, XtChainBottom,
				      NULL);				      
  XtAddCallback(simCanvas, XtNmotionCallback, (XtCallbackProc)reportCoordCallback, NULL);  
  simCanvasCoLabel = XtVaCreateManagedWidget(
					     "simCanvasCoLabel",
					     labelWidgetClass,
					     simMainForm,
					     XtNfromVert, simView,
					     XtNlabel, "CursorPos:",
					     XtNleft, XtChainLeft,
					     XtNright,XtChainLeft,
					     XtNtop,XtChainBottom,
					     XtNbottom, XtChainBottom,
					     XtNborderWidth, 0,
					     NULL);
  simCanvasCoords = XtVaCreateManagedWidget(
					    "simCanvasCoords",
					    labelWidgetClass,
					    simMainForm,
					    XtNfromVert, simView,
					    XtNfromHoriz, simCanvasCoLabel,
					    XtNlabel, "00000.0,00000.0",
					    XtNleft, XtChainLeft,
					    XtNright,XtChainLeft,
					    XtNtop,XtChainBottom,
					    XtNbottom, XtChainBottom,
					    XtNborderWidth, 1,
					    NULL);
  simRobotCoLabel = XtVaCreateManagedWidget(
					     "simRobotCoLabel",
					     labelWidgetClass,
					     simMainForm,
					     XtNfromVert, simView,
					     XtNhorizDistance, 205,
					     XtNlabel, "RobotPos:",
					     XtNleft, XtChainLeft,
					     XtNright,XtChainLeft,
					     XtNtop,XtChainBottom,
					     XtNbottom, XtChainBottom,
					     XtNborderWidth, 0,
					     NULL);
  simRobotCoords = XtVaCreateManagedWidget(
					     "simRobotCoords",
					     labelWidgetClass,
					     simMainForm,
					     XtNfromVert, simView,
					     XtNfromHoriz, simRobotCoLabel,
					     XtNlabel, "00000.0,00000.0",
					     XtNleft, XtChainLeft,
					     XtNright,XtChainLeft,
					     XtNtop,XtChainBottom,
					     XtNbottom, XtChainBottom,
					     XtNborderWidth, 1,
					     NULL);
  simVertButtonForm =
    XtVaCreateManagedWidget(
			    "simVertButtonForm",
			    formWidgetClass,
			    simMainForm,
			    XtNtop, XtChainTop,
			    XtNbottom, XtChainTop,
			    XtNright, XtChainRight,
			    XtNleft, XtChainRight,
			    XtNfromHoriz, simView,
			    XtNhorizDistance, 30,
			    XtNborderWidth, 0,
			    NULL);
  simHelpToggle = XtVaCreateManagedWidget(
					  "simHelpToggle",
					  toggleWidgetClass,
					  simVertButtonForm,
					  XtNlabel, "HELP",
					  XtNwidth, 80,
					  XtNheight, 20,
					  XtNvertDistance, 10,
					  NULL);
  XtAddCallback(simHelpToggle, XtNcallback, toggleHelpCallback, NULL);    
  simMapButtonDesc = XtVaCreateManagedWidget(
					   "simMapButtonDesc",
					   labelWidgetClass,
					   simVertButtonForm,
					   XtNfromVert, simHelpToggle,
					   XtNvertDistance, 20,
					   XtNjustify, XtJustifyLeft,
					   XtNlabel, "Playground",
					   XtNwidth, 80,
					   XtNheight, 14,
					   XtNborderWidth, 0,
					   NULL);
  simRobotCenter = XtVaCreateManagedWidget(
					   "simRobotCenter",
					   commandWidgetClass,
					   simVertButtonForm,
					   XtNfromVert, simMapButtonDesc,
					   XtNvertDistance, 5,
					   XtNlabel, "Find Robot",
					   XtNwidth, 80,
					   XtNheight, 15,
					   NULL);
  XtAddCallback(simRobotCenter, XtNcallback, (XtCallbackProc) CenterRobotCallback, NULL);    
  simZoomIn = XtVaCreateManagedWidget(
				      "simZoomIn",
				      commandWidgetClass,
				      simVertButtonForm,
				      XtNfromVert, simRobotCenter,
				      XtNvertDistance, 5,
				      XtNlabel, "Zoom In",
				      XtNwidth, 80,
				      XtNheight, 15,
				      NULL);
  XtAddCallback(simZoomIn, XtNcallback, zoomInCallback, NULL);  
  simZoomOut = XtVaCreateManagedWidget(
				      "simZoomOut",
				       commandWidgetClass,
				       simVertButtonForm,
				       XtNfromVert, simZoomIn,
				       XtNvertDistance, 5,
				       XtNlabel, "Zoom Out",
				       XtNwidth, 80,
				       XtNheight, 15,
				       NULL);
  XtAddCallback(simZoomOut, XtNcallback, zoomOutCallback, NULL);    
  simGridToggle = XtVaCreateManagedWidget(
					  "simGridToggle",
					  toggleWidgetClass,
					  simVertButtonForm,
					  XtNfromVert, simZoomOut,
					  XtNvertDistance, 5,
					  XtNlabel, "Grid",
					  XtNwidth, 80,
					  XtNheight, 15,
					  NULL);
  XtAddCallback(simGridToggle, XtNcallback, toggleGridCallback, NULL);
  simObstacleButtonDesc = XtVaCreateManagedWidget(
						  "simObstacleButtonDesc",
						  labelWidgetClass,
						  simVertButtonForm,
						  XtNfromVert, simGridToggle,
						  XtNvertDistance, 15,
						  XtNjustify, XtJustifyLeft,
						  XtNlabel, "Obstacle",
						  XtNwidth, 80,
						  XtNheight, 14,
						  XtNborderWidth, 0,
						  NULL);
  simDeleteObstacle = XtVaCreateManagedWidget(
					   "simDeleteObstacle",
					   toggleWidgetClass,
					   simVertButtonForm,
					   XtNfromVert, simObstacleButtonDesc,
					   XtNvertDistance, 5,
					   XtNlabel, "Delete",
					   XtNwidth, 80,
					   XtNheight, 15,
					   NULL);
  XtAddCallback(simDeleteObstacle, XtNcallback, obstacleModeCallback, NULL);
  simSwitchObstacle = XtVaCreateManagedWidget(
					   "simMoveObstacle",
					   toggleWidgetClass,
					   simVertButtonForm,
					   XtNfromVert, simDeleteObstacle,
					   XtNradioGroup, simDeleteObstacle,
					   XtNvertDistance, 5,
					   XtNlabel, "(De)activate",
					   XtNwidth, 80,
					   XtNheight, 15,
					   NULL);
  XtAddCallback(simSwitchObstacle, XtNcallback, obstacleModeCallback, NULL);  
  simMoveObstacle = XtVaCreateManagedWidget(
					   "simMoveObstacle",
					   toggleWidgetClass,
					   simVertButtonForm,
					   XtNfromVert, simSwitchObstacle,
					   XtNradioGroup, simDeleteObstacle,
					   XtNvertDistance, 5,
					   XtNlabel, "Control",
					   XtNwidth, 80,
					   XtNheight, 15,
					   NULL);
  XtAddCallback(simMoveObstacle, XtNcallback, obstacleModeCallback, NULL);
  simInsertObstacle = XtVaCreateManagedWidget(
					   "simInsertObstacle",
					   toggleWidgetClass,
					   simVertButtonForm,
					   XtNfromVert, simMoveObstacle,
					   XtNradioGroup, simDeleteObstacle,
					   XtNvertDistance, 10,
					   XtNlabel, "Insert:",
					   XtNwidth, 80,
					   XtNheight, 15,
					   NULL);
  XtAddCallback(simInsertObstacle, XtNcallback, obstacleModeCallback, NULL);  
  simObstacleType = XtVaCreateManagedWidget(
					   "simObstacleType",
					   labelWidgetClass,
					   simVertButtonForm,
					   XtNfromVert, simInsertObstacle,
					   XtNvertDistance, 5,
					   XtNlabel, "Rectangle",
					   XtNwidth, 63,
					   XtNheight, 15,
					   NULL);
  simObstacleMenuButton = XtVaCreateManagedWidget(
					   "simObstacleMenuButton",
					   menuButtonWidgetClass,
					   simVertButtonForm,
					   XtNfromVert, simInsertObstacle,
					   XtNvertDistance, 5,
					   XtNfromHoriz, simObstacleType,
					   XtNmenuName, "simObstacleMenu",
					   XtNhorizDistance, 1,
					   XtNlabel, "?",
					   XtNwidth, 15,
					   XtNheight, 15,
					   NULL);
  simObstacleMenu = XtVaCreatePopupShell(
					    "simObstacleMenu",
					    simpleMenuWidgetClass,
					    simObstacleMenuButton,
					    NULL);
  for(i = 0; i < OBSTACLE_TYPES; i++) {
    Widget entry = XtVaCreateManagedWidget(
				    obstacle_type[i],
				    smeBSBObjectClass,
				    simObstacleMenu,
				    NULL);
    XtAddCallback(entry, XtNcallback, selectObstacleType, (XtPointer) i);
  }
  simRobotButtonDesc = XtVaCreateManagedWidget(
					   "simRobotButtonDesc",
					   labelWidgetClass,
					   simVertButtonForm,
					   XtNfromVert, simObstacleType,
					   XtNjustify, XtJustifyLeft,
					   XtNvertDistance, 15,
					   XtNlabel, "Robot",
					   XtNwidth, 80,
					   XtNheight, 14,
					   XtNborderWidth, 0,
					   NULL);
  simTraceToggle = XtVaCreateManagedWidget(
					   "simTraceToggle",
					   toggleWidgetClass,
					   simVertButtonForm,
					   XtNfromVert, simRobotButtonDesc,
					   XtNvertDistance, 5,
					   XtNlabel, "Trace On",
					   XtNwidth, 80,
					   XtNheight, 15,
					   NULL);
  XtAddCallback(simTraceToggle, XtNcallback, toggleTraceCallback, NULL);
  simSonarToggle = XtVaCreateManagedWidget(
					   "simSonarToggle",
					   toggleWidgetClass,
					   simVertButtonForm,
					   XtNfromVert, simTraceToggle,
					   XtNvertDistance, 10,
					   XtNlabel, "Sonar",
					   XtNwidth, 80,
					   XtNheight, 15,
					   NULL);
  XtAddCallback(simSonarToggle, XtNcallback, toggleSonarCallback, NULL);  
  simLaserToggle = XtVaCreateManagedWidget(
					   "simLaserToggle",
					   toggleWidgetClass,
					   simVertButtonForm,
					   XtNfromVert, simSonarToggle,
					   XtNvertDistance, 10,
					   XtNlabel, "Laser",
					   XtNwidth, 80,
					   XtNheight, 15,
					   NULL);
  XtAddCallback(simLaserToggle, XtNcallback, toggleLaserCallback, NULL);  
  simFileButtonDesc = XtVaCreateManagedWidget(
					   "simFileButtonDesc",
					   labelWidgetClass,
					   simVertButtonForm,
					   XtNfromVert, simLaserToggle,
					   XtNvertDistance, 20,
					   XtNjustify, XtJustifyLeft,
					   XtNlabel, "File",
					   XtNwidth, 80,
					   XtNheight, 14,
					   XtNborderWidth, 0,
					   NULL);
  simLoad = XtVaCreateManagedWidget(
				    "simLoad",
				    commandWidgetClass,
				    simVertButtonForm,
				    XtNfromVert, simFileButtonDesc,
				    XtNvertDistance, 5,
				    XtNlabel, "Load",
				    XtNwidth, 80,
				    XtNheight, 15,
				    NULL);
  XtAddCallback(simLoad, XtNcallback, fileCallback, (XtPointer) "Load");      
  simSave = XtVaCreateManagedWidget(
				    "simSave",
				    commandWidgetClass,
				    simVertButtonForm,
				    XtNfromVert, simLoad,
				    XtNvertDistance, 10,
				    XtNlabel, "Save",
				    XtNwidth, 80,
				    XtNheight, 15,
				    NULL);
  XtAddCallback(simSave, XtNcallback, fileCallback, (XtPointer) "Save");    
  simQuit = XtVaCreateManagedWidget(
				    "simQuit",
				    commandWidgetClass,
				    simVertButtonForm,
				    XtNfromVert, simSave,
				    XtNvertDistance, 10,
				    XtNlabel, "Quit",
				    XtNwidth, 80,
				    XtNheight, 20,
				    NULL);
  XtAddCallback(simQuit, XtNcallback, QuitCallback, NULL);    
  XtAppAddActions (sim_context, actions, XtNumber (actions)); 
  XtRealizeWidget(topLevel);
}


void
InitTCX(const char *tcxMachine)
{
  if(just_viewing) {
    fprintf(stderr, "Starting SIMULATOR as VIEWER\n");
    SIMULATOR_initialize_tcx("VIEWER", tcxMachine);
  }
  else {
    SIMULATOR_initialize_tcx(TCX_SIMULATOR_MODULE_NAME, tcxMachine);
  }
  printf("Waiting for baseServer or Colli to come up...\n");
  while(!MODULE_BASE && !just_viewing) {
      struct timeval TCX_waiting_time = {0, 0};
      tcxRecvLoop((void *) &TCX_waiting_time);
      usleep(50000);
  }
  printf("done.\n");
}

void query_x()
{
    XEvent event;
    while(XtAppPending(sim_context)) {
	XtAppNextEvent(sim_context, &event);
	XtDispatchEvent(&event);
    }
}

void
mainloop()
{
    XtAppMainLoop(sim_context);
}
