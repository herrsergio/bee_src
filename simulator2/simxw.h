
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
#include <X11/Intrinsic.h>
#include "Drawing.h"

#define mySGN(x)  ((x) >= 0 ? (1) : (-1))
#define myABS(x)  ((x) < 0 ? -(x) : (x))
#define mySQR(x)  ((x) * (x))
#define myMAX(x, y)     ((x) > (y) ? (x) : (y))
#define myMIN(x, y)     ((x) < (y) ? (x) : (y))
#define ctoi(x)         ((x) - '0')

/*** tables with trigonometric values -> used for speed up ***/
#define WALLCOLOR "grey50"

#ifdef __cplusplus
extern "C" {
#endif

void AddTimeOut(int, void(*)());
void automatic_redraw(Boolean Val);
void place_dot(float, float, float, float);
Figure expose_disabled_polygon(int, float*,float*,char*);    
Figure expose_disabled_rectangle(float,float,float,float,char*);
Figure expose_disabled_circle(float,float,float,char*);
Figure expose_polygon(int, float*,float*,char*);    
Figure expose_rectangle(float,float,float,float,char*);
Figure expose_circle(float,float,float, char*);
Figure redraw_polygon(Figure, int, float*, float*);
Figure redraw_disabled_polygon(Figure, int, float*, float*);
Figure redraw_rectangle(Figure,float,float,float,float);
Figure redraw_circle(Figure,float,float,float);
void DrawRobotFigure(Boolean, float,float,float,float);
void expose_beam(int, float, float, float, float);
void expose_laser_beam(int, float, float, float, float);
void change_color(Figure, char*);
void drawTraceLine(float,float,float,float);
void InitRobotFigure(float, float, float, float);
void InitPlaygroundGraphics(char*,float,float,float,float);
void InitWidgets(int*, char**);
void RemoveFigure(Figure);
void InitTimerCallbacks();
void StartMainloop();
void query_x();
void mainloop();
void InitTCX();
extern Boolean traceOn;
extern Boolean sonarOn;
extern Boolean laserOn;
extern Boolean just_viewing;
extern Display *theDisplay;
#ifdef __cplusplus
}
#endif

