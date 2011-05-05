
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



#include <bUtils.h>
#include "collisionIntern.h"

#define SONAR_RANGE_IN_ARM_DIRECTION 100.0

float SONAR_UPDATE_TIME = 1.1;

float *SonarAngle;


/**********************************************************************/
void
init_SonarAngle(void)
{
  float tmp;
  int i;
  
  /* The angle of the first sonar sensor has changed. 
     SonarAngle[0] = DEG_TO_RAD(360.0 - 1.0); */
  
  if (bRobot.sonar_cols[0] == 24)
    SonarAngle[0] = DEG_TO_RAD(360.0 - 7.5);
  else if (bRobot.sonar_cols[0] == 16)
    SonarAngle[0] = DEG_TO_RAD(360.0 - 11.25);
  else{
    fprintf(stderr, "COLLI is not configured for a robot with %d sonars\n",
	    bRobot.sonar_cols[0]);
    exit(1);
  }

  tmp = DEG_TO_RAD(360.0 / ((float) bRobot.sonar_cols[0]));

  
  for (i=1; i<NO_OF_SONARS; i++)
    SonarAngle[i] = (SonarAngle[i-1] - tmp);
}



/**********************************************************************
 * Given the position of the robot and the distance of a particular 
 * sonar the collision line representing this reading is given back.
**********************************************************************/
LineSeg sonar_to_lineseg( Point rpos, float rrot, 
			  int sonar_no, float dist)
{
  Point sonar_pos,  sonar_left_edge, sonar_right_edge;
  LineSeg line;
  float sonar_rot, sonar_plain, open_angle, tmp, start_angle;

  open_angle = SONAR_OPEN_ANGLE;

  /* If the line is too far away we don't consider it. */ 
  if (dist > 0.0 && dist <= MAX_RANGE) {
    
    if ( armState == INSIDE) {

      /* If the robot is very fast we don't consider the lines behind it. 
       * If the current tvel is 100.0 we only use lines starting at 90.0 
       * degrees. This is only allowed if the arm is inside.
       */
      tmp   = (float) rwi_base.trans_current_speed * 0.01;
      
      start_angle = DEG_180 - tmp * DEG_90; 
      
      if ( SonarAngle[sonar_no] > start_angle && 
	  SonarAngle[sonar_no] < DEG_360 - start_angle) {
	line.pt1.x = F_ERROR;
	return(line);      
      }
    }

    /* We want to limit the length of the lineseg to MAX_COLLISION_LINE_LENGTH */
    if (dist > EPSILON && ACTUAL_MODE->max_collision_line_length > 0.0)
      if ((tmp = 0.5 * atan(ACTUAL_MODE->max_collision_line_length / dist))
	  < open_angle)
	open_angle = tmp;
    
    sonar_rot = rrot + SonarAngle[sonar_no];
    norm_angle(&sonar_rot);
    
    dist /= fcos(open_angle);
    
    sonar_plain = sonar_rot + DEG_90;
    
    sonar_pos.x = rpos.x + (fcos( sonar_rot) * ROB_RADIUS);
    sonar_pos.y = rpos.y + (fsin( sonar_rot) * ROB_RADIUS);

    sonar_left_edge.x = sonar_pos.x + (fcos( sonar_plain) * COLLI_SONAR_RADIUS);
    sonar_left_edge.y = sonar_pos.y + (fsin( sonar_plain) * COLLI_SONAR_RADIUS);
    sonar_right_edge.x = sonar_pos.x - (fcos( sonar_plain) * COLLI_SONAR_RADIUS);
    sonar_right_edge.y = sonar_pos.y - (fsin( sonar_plain) * COLLI_SONAR_RADIUS);
    
    line.pt1.x = sonar_left_edge.x + fcos( sonar_rot + open_angle) * dist;
    line.pt1.y = sonar_left_edge.y + fsin( sonar_rot + open_angle) * dist;
    line.pt2.x = sonar_right_edge.x + fcos( sonar_rot - open_angle) * dist;
    line.pt2.y = sonar_right_edge.y + fsin( sonar_rot - open_angle) * dist;
    
    /* We want the point with the smaller x value (if possible) to be pt1 */
    if (smaller( line.pt2, line.pt1))
      swap(&line);
    
    return(line);
  }
  else line.pt1.x = F_ERROR;
  
  
  return(line);
}




/**********************************************************************
 * In the case that the robot has to move backward we just swap the
 * sonar information such that the rear sonars seem to be at the front
 * of the robot (the base then only has to change the sign of the
 * velocities.
 **********************************************************************/
void update_CollLines(Pointer callback_data, Pointer client_data)
{
  int i, tmp;
  int *sonar_no;
  float rrot;
  Point rpos;

  if ( use_sonar) {
    
#ifdef B21
    /* If the arm is outside we can't use these sonars. */
    if (armState != INSIDE) {
      sonar_readings[0] = sonar_readings[23] = SONAR_RANGE_IN_ARM_DIRECTION;
      sonar_readings[1] = sonar_readings[22] = SONAR_RANGE_IN_ARM_DIRECTION;
    }
#endif

    rpos.x = ((float) rwi_base.pos_x + sonar_start_pos_x) * 0.5;
    rpos.y = ((float) rwi_base.pos_y + sonar_start_pos_y) * 0.5;
    if ((sonar_start_rot < 90.0 && rwi_base.rot_position > 270.0) ||
	(sonar_start_rot > 270.0 && rwi_base.rot_position < 90.0))
      rrot = (rwi_base.rot_position + 360.0 + sonar_start_rot) * 0.5;
    else
      rrot = (rwi_base.rot_position + sonar_start_rot) * 0.5;
    
    rrot   = normed_angle( (float) DEG_TO_RAD(90.0 -  rrot));
    
    sonar_no = (int *) callback_data;
    
    for (i=0; i<4; i++) {
      if(( tmp = sonar_no[i]) >= 0) {
	CollLines[tmp][next_CollLine_reading] = 
	  sonar_to_lineseg(rpos, rrot, tmp, sonar_readings[tmp]);
      }
      else {
	fprintf(stderr, "sonar reading not ok.\n");
      }
    }
    
    gettimeofday(&Last_CollLine_Update, 0);
  }

  COLLI_update_tcx_CollLines(); 
}






