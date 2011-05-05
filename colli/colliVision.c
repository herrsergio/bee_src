
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






#include "collisionIntern.h"


/**********************************************************************
* Receives collision lines from the vision module and uses them 
* like normal CollLines. 
**********************************************************************/
void COLLI_get_CollLines_from_vision(COLLI_vision_line_ptr vision_lines)
{
  int i;
  float x, y;

  if (vision_lines->no_of_lines > 0) {
    
    if (dumpInfo)
      fprintf( dumpFile, "Received vision line(s) at\n");
    fprintf(stderr, "Received vision line(s) at\n");
    if (External_Obstacle_Lines.no_of_lines > 0) 
      free (External_Obstacle_Lines.lines);
    
    External_Obstacle_Lines.no_of_lines = vision_lines->no_of_lines;
    External_Obstacle_Lines.lines = (LineSeg *) 
      malloc( External_Obstacle_Lines.no_of_lines * sizeof(struct LineSeg));
    
    for (i=0; i<External_Obstacle_Lines.no_of_lines; i++) {
      fprintf(stderr, " P1: %d %d  P2: %d %d\n", 
	      vision_lines->lines[i].pt1.x,
	      vision_lines->lines[i].pt1.y, 
	      vision_lines->lines[i].pt2.x,
	      vision_lines->lines[i].pt2.y);

      External_Obstacle_Lines.lines[i].pt1.x = (float) vision_lines->lines[i].pt1.x;
      External_Obstacle_Lines.lines[i].pt1.y = (float) vision_lines->lines[i].pt1.y;
      External_Obstacle_Lines.lines[i].pt2.x = (float) vision_lines->lines[i].pt2.x;
      External_Obstacle_Lines.lines[i].pt2.y = (float) vision_lines->lines[i].pt2.y;
    }
    
    gettimeofday(&Last_VisionLine_Update, 0);
    COLLI_update_tcx_External_Obstacle_Lines();

    x = 0.5 * (External_Obstacle_Lines.lines[0].pt1.x + External_Obstacle_Lines.lines[0].pt2.x);
    y = 0.5 * (External_Obstacle_Lines.lines[0].pt1.y + External_Obstacle_Lines.lines[0].pt2.y);
    
 }
  else
    fprintf( stderr, "Received 0 lines from vision.\n");
}

/**********************************************************************/
void COLLI_get_CollPoints_from_vision(COLLI_vision_point_ptr vision_points)
{
  int i;

  if (vision_points->no_of_points > 0) {
    
    fprintf(stderr, "Received %d vision point(s).\n",vision_points->no_of_points );
    if (External_Obstacle_Points.no_of_points > 0) 
      free (External_Obstacle_Points.points);
    
    External_Obstacle_Points.no_of_points = vision_points->no_of_points;
    External_Obstacle_Points.points = (Point *) 
      malloc( External_Obstacle_Points.no_of_points * sizeof(struct Point));
    
    for (i=0; i<External_Obstacle_Points.no_of_points; i++) {
            fprintf(stderr, " P: %d %d\n", 
	      vision_points->points[i].x,
	      vision_points->points[i].y);
	
      External_Obstacle_Points.points[i].x = (float) vision_points->points[i].x;
      External_Obstacle_Points.points[i].y = (float) vision_points->points[i].y;
    }

    gettimeofday(&Last_VisionPoint_Update, 0);
    COLLI_update_tcx_External_Obstacle_Points();
  }
}

