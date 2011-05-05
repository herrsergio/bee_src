
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



#ifdef CLEANUP


#include "collisionIntern.h"
#include "colliCleanUp.h"

/* Just for testing without vision. */

static void
COLLI_SimulateClose( double forward, double side, int type);
static void
COLLI_SimulateFar( double forward, double side, int type);

/***************************************************************************
 * Transforms the relative position of an object into absolute position and
 * stores it in the object.
 ***************************************************************************/
static void
setObjectPosition( float forward,
		   float side,
		   int type,
		   Point robot,
		   float rRot,
		   Object* object)
{
    object->x = (float) ((robot.x + cos( rRot) * forward + sin( rRot) * side));
    object->y = (float) ((robot.y + sin( rRot) * forward - cos( rRot) * side));
    object->type = type;
    
    fprintf( stderr, "Object position: x %f  y %f\n", object->x, object->y);
}
    
/***************************************************************************
 * Transforms the relative position of a trash bin into absolute position and
 * stores it in the trash bin.
 ***************************************************************************/
static void
setTrashBinPosition( float forward,
		     float side,
		     Point robot,
		     float rRot,
		     TrashBin* bin)
{
    bin->x = (float) ((robot.x + cos( rRot) * forward + sin( rRot) * side));
    bin->y = (float) ((robot.y + sin( rRot) * forward - cos( rRot) * side));
    
    fprintf( stderr, "Trash bin position: x %f  y %f\n", bin->x, bin->y);
}
    


/***************************************************************************
 * Sends SUNVIS a request for far objects of type <type>.
 ***************************************************************************/
void
sendRequestForFarObjects( int type)
{
#ifdef UNIBONN
  if ( use_vision) {
    SUNVIS_object_query_type msg ;
    msg.obj_x = 0.0;
    msg.obj_y = 0.0;
    msg.seconds = 0.0;
    msg.type = type;
    fprintf(stderr, "send request\n");
    tcxSendMsg(SUNVIS, "SUNVIS_object_query", &msg);
  }
#endif
}


/***************************************************************************
 * Sends SUNVIS a request for exact position of an object of type <type>.
 ***************************************************************************/
void
sendRequestForExactPosition( int type)
{
#ifdef UNIBONN
  if ( use_vision) {
    SUNVIS_check_object_query_type msg ;
    msg.obj_x = 0.0;
    msg.obj_y = 0.0;
    msg.seconds = 0.0;
    msg.type = type;
    
    tcxSendMsg(SUNVIS, "SUNVIS_check_object_query", &msg);
  }
#endif
}
	 



/***************************************************************************
 * Gets coarse position of an object by hand. Let the robot
 * approach this object. *
 ***************************************************************************/
void
COLLI_SimulateFarObject( double forward, double side)
{
  COLLI_SimulateFar( forward, side, OBJECT);
}

void
COLLI_SimulateFarTrashBin( double forward, double side)
{
  COLLI_SimulateFar( forward, side, TRASH_BIN);
}

static void
COLLI_SimulateFar( double forward, double side, int type)
{
  Point robot;
  float rRot;
  
  if ( dontAcceptFarThings()) {
    fprintf( stderr, "Sorry. don't accept far things now.\n");
    return;
  }

  robot.x = (float) rwi_base.pos_x;
  robot.y = (float) rwi_base.pos_y;
  rRot    = (float) DEG_TO_RAD(90.0 - rwi_base.rot_position);

  if ( type == TRASH_BIN) {
    
      numberOfFarTrashBins = 1;
      
      setTrashBinPosition( forward,
			  side,
			  robot,
			  rRot,
			  &( farTrashBins[0]));
      
    }
  else {
      numberOfFarObjects = 1;
      
      setObjectPosition( forward,
			side,
			type,
			robot,
			rRot,
			&( farObjects[0]));
      
    }

    actualFarTrashBin.x = farTrashBins[0].x;
    actualFarTrashBin.y = farTrashBins[0].y;
    actualFarObject.x = farObjects[0].x;
    actualFarObject.y = farObjects[0].y;
}

/***************************************************************************
 * Gets coarse position of an object from SUNVIS. Let's the robot
 * approach this object. *
 ***************************************************************************/

#ifndef UNIBONN

typedef struct{

  int num_objects;		/* number of objects in this messsage */

  /*************
   *
   *  Here comes the list of objects
   */

  int *checked;			/* 1, if we *tried* to recognize
				 * an object */
  int *type;			/* see the defines above */

  float *obj_x;			/* relative to the robot position */
  float *obj_y;

  float *confidence;		/* 1=certain, 0=nonsense */


  /*************
   *
   *  And finally we send the robot position when the snapshot was taken
   */

  float robot_x;
  float robot_y;
  float robot_orientation;	/* location of the robot when the picture
				 * was taken */
  float pan_angle;
  float tilt_angle;
} SUNVIS_object_reply_type, *SUNVIS_object_reply_ptr;

typedef struct{

  int   type ;

  float obj_x;			/* object position relativ to camera */
  float obj_y;                

  char  valid ;                /* zero if nothing found             */

} SUNVIS_check_object_reply_type, *SUNVIS_check_object_reply_ptr;
#endif


#ifdef CLEANUP
void
COLLI_ApproachFarObject( SUNVIS_object_reply_ptr object)
{
    double rRot;
    Point robot;
    int i;
    BOOLEAN firstTrashBin = TRUE;
    BOOLEAN firstObject   = TRUE;
    
    if ( dontAcceptFarThings()) {
	fprintf( stderr, "Sorry. don't accept far things now.\n");
	return;
    }

    robot.x = object->robot_x;
    robot.y = object->robot_y;
    rRot  = DEG_TO_RAD( object->robot_orientation);
    
    putc(7, stderr);
    fprintf(stderr, "robot position: %f %f\n: ",rwi_base.pos_x, rwi_base.pos_y);
    fprintf(stderr, "SUNVIS far objects: rposX %f   rposY %f  rRot %f\n",
	    robot.x, robot.y, rRot);

    for ( i = 0; i < object->num_objects; i++) {
	
	fprintf(stderr, "side %f   forward %f  type %d\n",
		object->obj_x[i], object->obj_y[i], object->type[i]);


	if ( object->type[i] == TRASH_BIN) {

	    /* We only want to store the most recent positions. */
	    if ( firstTrashBin) {
		numberOfFarTrashBins = 0;
		firstTrashBin = FALSE;
	    }
	    if ( numberOfFarTrashBins == MAX_NUMBER_OF_TRASH_BINS)
		fprintf( stderr, "Sorry too many trash bins. Skip it.\n");
	    
	    setTrashBinPosition( object->obj_y[i],
				 object->obj_x[i],
				 robot,
				 rRot,
				 &( farTrashBins[numberOfFarTrashBins++]));
	    
	}
	else if ( object->type[i] == OBJECT) {

	    /* We only want to store the most recent positions. */
	    if ( firstObject) {
		numberOfFarObjects = 0;
		firstObject = FALSE;
	    }
	    if ( numberOfFarObjects == MAX_NUMBER_OF_OBJECTS)
		fprintf( stderr, "Sorry too many objects. Skip it.\n");
	    else
		setObjectPosition( object->obj_y[i],
				   object->obj_x[i],
				   object->type[i],
				   robot,
				   rRot,
				   &( farObjects[numberOfFarObjects++]));
	}
    }

    /* Now let's check wether there was an object needed in the current state. */

    if ( (cleanUpState == RANDOM_WALK && numberOfFarObjects == 0) ||
	(cleanUpState == LOOK_FOR_TRASH_BIN && numberOfFarTrashBins == 0))
      /* Let's try it again. */
      sendRequestForFarObjects( EVERYTHING);
      
    actualFarTrashBin.x = farTrashBins[0].x;
    actualFarTrashBin.y = farTrashBins[0].y;
    actualFarObject.x = farObjects[0].x;
    actualFarObject.y = farObjects[0].y;
}
#endif
    
    
/***************************************************************************
 * Gets coarse position of an object from SUNVIS. Let's the robot
 * approach this object. *
 ***************************************************************************/
void
COLLI_SimulateCloseObject( double forward, double side)
{
  COLLI_SimulateClose( forward, side, OBJECT);
}

void
COLLI_SimulateCloseTrashBin( double forward, double side)
{
  COLLI_SimulateClose( forward, side, TRASH_BIN);
}

static void
COLLI_SimulateClose( double forward, double side, int type)
{
  Point robot;
  float rRot;
  
  if ( dontAcceptCloseThings()) {
    fprintf( stderr, "Sorry. don't accept close things now.\n");
    return;
  }
  
  robot.x = (float) rwi_base.pos_x;
  robot.y = (float) rwi_base.pos_y;
  rRot    = (float) DEG_TO_RAD(90.0 - rwi_base.rot_position);

  if ( type == TRASH_BIN) {
    
      numberOfCloseTrashBins = 1;
      
      setTrashBinPosition( forward,
			  side,
			  robot,
			  rRot,
			  &( closeTrashBins[0]));
      
    }
  else {

      numberOfCloseObjects = 1;
      
      setObjectPosition( forward,
			side,
			type,
			robot,
			rRot,
			&( closeObjects[0]));
      
    }
  
  actualCloseTrashBin.x = closeTrashBins[0].x;
  actualCloseTrashBin.y = closeTrashBins[0].y;
  actualCloseObject.x = closeObjects[0].x;
  actualCloseObject.y = closeObjects[0].y;
}


/**********************************************************************
 * This is the exact position of the object. The robot starts to pick
 * up the object.
 **********************************************************************/
#ifdef CLEANUP
void
COLLI_ApproachCloseObject( SUNVIS_check_object_reply_ptr object)
{
    double rRot;
    Point robot;
    BOOLEAN firstTrashBin = TRUE;
    BOOLEAN firstObject   = TRUE;
    
    if ( dontAcceptCloseThings()) {
	fprintf( stderr, "Sorry. don't accept far things now.\n");
	return;
    }

    robot.x = (float) rwi_base.pos_x;
    robot.y = (float) rwi_base.pos_y;
    rRot   = (float) DEG_TO_RAD(90.0 - rwi_base.rot_position);
    
    putc(7, stderr);
    fprintf(stderr, "SUNVIS close object: rposX %f   rposY %f  rRot %f\n",
	    robot.x, robot.y, rRot);

    if ( object->obj_x == 0.0 && object->obj_y == 0.0) {
       fprintf( stderr, "Both coordinates are zero.\n");
       return;
    }
    
    fprintf(stderr, "side %f   forward %f  type %d\n",
	    object->obj_x, object->obj_y, object->type);
    
    if ( object->type == TRASH_BIN) {
	
	/* We only want to store the most recent positions. */
	if ( firstTrashBin) {
	    numberOfCloseTrashBins = 0;
	    firstTrashBin = FALSE;
	}
	if ( numberOfCloseTrashBins == MAX_NUMBER_OF_TRASH_BINS)
	    fprintf( stderr, "Sorry too many trash bins. Skip it.\n");
	
	setTrashBinPosition( object->obj_y,
			     object->obj_x,
			     robot,
			     rRot,
			     &( closeTrashBins[numberOfCloseTrashBins++]));
	
    }
    else if ( object->type == OBJECT) {
	
	/* We only want to store the most recent positions. */
	if ( firstObject) {
	    numberOfCloseObjects = 0;
	    firstObject = FALSE;
	}
	if ( numberOfCloseObjects == MAX_NUMBER_OF_OBJECTS)
	    fprintf( stderr, "Sorry too many objects. Skip it.\n");
	else
	    setObjectPosition( object->obj_y,
			       object->obj_x,
			       object->type,
			       robot,
			       rRot,
			       &( closeObjects[numberOfCloseObjects++]));
    }

    actualCloseTrashBin.x = closeTrashBins[0].x;
    actualCloseTrashBin.y = closeTrashBins[0].y;
    actualCloseObject.x = closeObjects[0].x;
    actualCloseObject.y = closeObjects[0].y;
}
#endif



#endif
