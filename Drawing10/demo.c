/*
    This file is part of the FElt finite element analysis package.
    Copyright (C) 1993,1994 Jason I. Gobat and Darren C. Atkinson

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

/************************************************************************
 * File:	demo.c							*
 *									*
 * Description:	This file contains the demo functions for the Drawing	*
 *		widget.							*
 ************************************************************************/

# include <X11/Intrinsic.h>
# include <X11/StringDefs.h>
# include "Drawing.h"


/************************************************************************
   Function:	GetCoords
   Description:	Retrieves the real coordinates from an event.
 ************************************************************************/

void GetCoords (widget, event, x, y)
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


/************************************************************************
   Function:	Quit
   Description:	Quits the application.
 ************************************************************************/

void Quit (widget, event, params, num_params)
    Widget    widget;
    XEvent   *event;
    String   *params;
    Cardinal *num_params;
{
    XtDestroyApplicationContext (XtWidgetToApplicationContext (widget));
    exit (0);
}


/************************************************************************
   Function:	Zoom
   Description:	Zooms in or out:
			params [0] = {in,out}
 ************************************************************************/

void Zoom (widget, event, params, num_params)
    Widget    widget;
    XEvent   *event;
    String   *params;
    Cardinal *num_params;
{
    Arg   args [2];
    float x_scale;
    float y_scale;


    if (*num_params >= 1) {
	XtSetArg (args [0], XtNxScale, &x_scale);
	XtSetArg (args [1], XtNyScale, &y_scale);
	XtGetValues (widget, args, 2);

	if (!strcmp (params [0], "in")) {
	    x_scale *= 2;
	    y_scale *= 2;
	} else {
	    x_scale /= 2;
	    y_scale /= 2;
	}

	XtSetArg (args [0], XtNxScale, Float2Arg (x_scale));
	XtSetArg (args [1], XtNyScale, Float2Arg (y_scale));
	XtSetValues (widget, args, 2);
    }
}


/************************************************************************
   Function:	MoveFigure
   Description:	Moves a figure:
			params [0] = {begin,move,end}
 ************************************************************************/

void MoveFigure (widget, event, params, num_params)
    Widget    widget;
    XEvent   *event;
    String   *params;
    Cardinal *num_params;
{
    float		    x;
    float		    y;
    static Figure	    outline;
    static Figure	    figure;
    static FigureAttributes attributes;


    GetCoords (widget, event, &x, &y);


    if (*num_params >= 1) {
	if (!strcmp (params [0], "begin")) {
	    if ((figure = DW_FindFigure (widget, x, y))) {
		DW_SetInteractive (widget, True);
		outline = DW_DrawText (widget, False, x, y, "move");
		DW_GetAttributes (widget, outline, &attributes);
	    }

	} else if (!strcmp (params [0], "end") && figure) {
	    DW_RemoveFigure (widget, outline);
	    DW_SetInteractive (widget, False);
	    DW_SetAttributes (widget, figure, DW_FigureLocation, &attributes);
	    figure = NULL;

	} else if (figure) {
	    attributes.x = x;
	    attributes.y = y;
	    DW_SetAttributes (widget, outline, DW_FigureLocation, &attributes);
	}
    }
}


/************************************************************************
   Function:	DrawFigure
   Description:	Draws a figure on the drawing widget:
			params [0] = {rectangle,circle}
			params [1] = {small,medium,large}
			params [2] = {scaled,unscaled}
			params [3] = color
 ************************************************************************/

void DrawFigure (widget, event, params, num_params)
    Widget    widget;
    XEvent   *event;
    String   *params;
    Cardinal *num_params;
{
    float   x;
    float   y;
    float   width;
    float   height;
    Boolean scaled;


    GetCoords (widget, event, &x, &y);

    if (*num_params >= 4)
	DW_SetForeground (widget, params [3]);


    scaled = False;

    if (*num_params >= 3)
	if (!strcmp (params [2], "scaled"))
	    scaled = True;


    height = width = scaled ? 1.0 : 20;

    if (*num_params >= 2)
	if (!strcmp (params [1], "small"))
	    height = width = scaled ? 0.5 : 10;
	else if (!strcmp (params [1], "large"))
	    height = width = scaled ? 2.0 : 40;


    if (*num_params >= 1) {
	if (!strcmp (params [0], "circle"))
	    DW_DrawArc (widget, scaled, x, y, width, height, 0.0, 360.0);
	else
	    DW_DrawRectangle (widget, scaled, x, y, width, height);
    }
}


/************************************************************************
   Function:	FillFigure
   Description:	Draws a filled figure on the drawing widget:
			params [0] = {rectangle,circle}
			params [1] = {small,medium,large}
			params [2] = {scaled,unscaled}
			params [3] = color
 ************************************************************************/

void FillFigure (widget, event, params, num_params)
    Widget    widget;
    XEvent   *event;
    String   *params;
    Cardinal *num_params;
{
    float   x;
    float   y;
    float   width;
    float   height;
    Boolean scaled;


    GetCoords (widget, event, &x, &y);

    if (*num_params >= 4)
	DW_SetForeground (widget, params [3]);


    scaled = False;

    if (*num_params >= 3)
	if (!strcmp (params [2], "scaled"))
	    scaled = True;


    height = width = scaled ? 1.0 : 20;

    if (*num_params >= 2)
	if (!strcmp (params [1], "small"))
	    height = width = scaled ? 0.5 : 10;
	else if (!strcmp (params [1], "large"))
	    height = width = scaled ? 2.0 : 40;


    if (*num_params >= 1) {
	if (!strcmp (params [0], "circle"))
	    DW_FillArc (widget, scaled, x, y, width, height, 0.0, 360.0);
	else
	    DW_FillRectangle (widget, scaled, x, y, width, height);
    }
}


/************************************************************************
   Function:	ChangeFigure
   Description:	Changes a figure under the pointer.
			params [0] = {raise,lower,remove}
 ************************************************************************/

void ChangeFigure (widget, event, params, num_params)
    Widget    widget;
    XEvent   *event;
    String   *params;
    Cardinal *num_params;
{
    float  x;
    float  y;
    Figure figure;


    if (*num_params >= 1) {
	GetCoords (widget, event, &x, &y);

	if ((figure = DW_FindFigure (widget, x, y)))
	    if (!strcmp (params [0], "lower"))
		DW_LowerFigure (widget, figure);
	    else if (!strcmp (params [0], "remove"))
		DW_RemoveFigure (widget, figure);
	    else
		DW_RaiseFigure (widget, figure);
    }
}


/************************************************************************
   Function:	ToggleResource
   Description:	Toggles a resource of the drawing widget:
			params [0] = {snap,grid}
 ************************************************************************/

void ToggleResource (widget, event, params, num_params)
    Widget    widget;
    XEvent   *event;
    String   *params;
    Cardinal *num_params;
{
    Arg     arg;
    String  resource;
    Boolean value;


    if (*num_params >= 1) {
	if (!strcmp (params [0], "grid"))
	    resource = XtNgrid;
	else if (!strcmp (params [0], "snap"))
	    resource = XtNsnap;
	else
	    return;

	XtSetArg (arg, resource, &value);
	XtGetValues (widget, &arg, 1);

	value = !value;
	XtSetArg (arg, resource, value);
	XtSetValues (widget, &arg, 1);
    }
}


XtActionsRec actions [ ] = {
    {"Quit",	       Quit},
    {"Zoom",	       Zoom},
    {"MoveFigure",     MoveFigure},
    {"DrawFigure",     DrawFigure},
    {"FillFigure",     FillFigure},
    {"ChangeFigure",   ChangeFigure},
    {"ToggleResource", ToggleResource},
};


/************************************************************************
   Function:	main
   Description:	Driver function for the demo application.
 ************************************************************************/

int main (argc, argv)
    int   argc;
    char *argv [ ];
{
    Widget	 drawing;
    Widget	 toplevel;
    XtAppContext app_context;


    toplevel = XtAppInitialize (&app_context, "Demo", NULL, 0,
			&argc, argv, NULL, NULL, 0);

    drawing = XtCreateManagedWidget ("drawing", drawingWidgetClass, toplevel,
			NULL, 0);

    XtAppAddActions (app_context, actions, XtNumber (actions));

    XtRealizeWidget (toplevel);
    XtAppMainLoop (app_context);
}
