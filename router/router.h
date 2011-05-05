
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






/* needs tcx includes */

#define MaxNumberOfModules 64


#define BASE_REGISTER_AUTO_UPDATE           1
#define BASE_ROBOT_POSITION_QUERY           2
#define BASE_ROBOT_POSITION_REPLY           3
#define BASE_STOP_ROBOT                     4
#define BASE_UPDATE_STATUS_QUERY            5
#define BASE_UPDATE_STATUS_REPLY            6
#define BASE_SET_VELOCITY                   7
#define BASE_SET_ACCELERATION               8
#define BASE_ROTATE_CLOCKWISE               9
#define BASE_ROTATE_ANTICLOCKWISE          10
#define BASE_TRANSLATE_FORWARD             11
#define BASE_TRANSLATE_BACKWARD            12

#define BASE_GOTO_RELATIVE                 14
#define BASE_TRANSLATE_BY                  15

#define BASE_ROTATE_BY                     17
#define BASE_GOTO_ABSOLUTE                 18
#define BASE_APPROACH_ABSOLUTE             19
#define BASE_APPROACH_ABSOLUTE_TWO_POINTS  20
#define BASE_DISCONNECT                    21
#define BASE_SETMODE                       22
#define BASE_ACTION_EXECUTED_REPLY         23
#define BASE_RESET_JOYSTICK                24
#define BASE_NOTIFY_WALL_ORIENTATION       25

#define SONAR_SWITCH_ON                    26
#define SONAR_SWITCH_OFF                   27
#define SONAR_ACTIVATE_MASK                28
#define SONAR_SONAR_QUERY                  29
#define SONAR_SONAR_REPLY                  30
#define SONAR_DEFINE_MASK                  31

#define COLLI_COLLI_REPLY                  32
#define COLLI_VISION_LINE                  33
#define COLLI_VISION_POINT                 34
#define COLLI_PARAMETER                    35
#define COLLI_DISCONNECT                   36

#define LASER_LASER_REPLY                  37

#define MaxNumberOfMessages  37  /* starting at 1  !!! */

/* ------------------------------------------------------------------------ */

#define ACTIVE       1
#define INACTIVE     2
#define DISCONNECTED 3


#define MAX_QUERIES      4
#define STATUS_QUERY     0
#define SONAR_QUERY      1
#define COLLI_QUERY      2
#define POSITION_QUERY   3


typedef struct
{
  char ModuleName[100];
  int ModuleID;
  int ModuleStatus;
  int NumOfMessagesSend;
  int NumOfMessagesReceived;
  int Autoreply;
  TCX_MODULE_PTR module;	/* pointer to TCX module */
  int            status;	/* 1=subscribed to regular status updates */
  int            sonar;		/* 1=subscribed to regular sonar updates */
  int            colli;		/* 1=subscribed to regular sonar updates */
  int            laser;		/* 1=subscribed to regular laser updates */
  int            QUERYS[MAX_QUERIES];
} RouterModuleReferenceType,*RouterModuleReferencePtr;


#define PASS_THRU     1
#define INTERCEPT     2
#define NOTIFY        3
#define REPLY_IT      4

typedef struct
{
  int MessageType;       /* PASS_THRU , INTERCEPT , NOTIFY */
  int ReplyExpected;     /* 1 means yes so that incoming queries are being
                            hold until this reply comes in to avoid 
                            fast queries when replys take more too long.
			    0 is when no reply is expected and the query
                            can be passed through. This function can be
                            disabled by the flag -force */
  int ReplyModuleList[MaxNumberOfModules];
  int Amount_Incoming;   /* Just to count that nothing gets lost */
  int Amount_Transferred; /* Just to count that nothing gets lost on the way */

} RouterMessageReferenceType,*RouterMessageReferencePtr;

							   



/****************************************************************************
 * InitRouterModuleReferenceArray                                           *
 *                                                                          *
 * Parameters : NONE                                                        *
 * Returns    : 0 if ok , -1 if failure                                     *
 * Does       : Initializes the RMR by blanking.                            *
 * Called     : At the very beginning, when the router is initialized       *
 ****************************************************************************/
int InitRouterModuleReferenceArray ( void );


/****************************************************************************
 * InitRouterMessageReferenceArray                                          *
 *                                                                          *
 * Parameters : NONE                                                        *
 * Returns    : 0 if ok , -1 if failure                                     *
 * Does       : Initializes the RmR by blanking.                            *
 *              Defines the type of each message.                           *
 * Called     : At the very beginning, when the router is initialized       *
 ****************************************************************************/
int InitRouterMessageReferenceArray ( void );


/****************************************************************************
 * Connect2RealBASE                                                         *
 *                                                                          *
 * Parameters : NONE                                                        *
 * Returns    : 0 if ok , -1 if failure                                     *
 *              Blocks until connection can be established                  *
 * Does       : Creates TCX Module Reference                                *
 * Called     : At the very beginning, when the router is initialized       *
 ****************************************************************************/
int Connect2RealBASE ( void );


/****************************************************************************
 * CheckIncomingModule                                                      *
 *                                                                          *
 * Parameters : TCX_REF_PTR                                                 *
 * Returns    : Module index if known,or -1 if failure                      *
 * Does       : Searches the Module list for the Module index or            * 
 *              creates a new module entry.                                 *
 * Called     : At the very beginning, when the router is initialized       *
 ****************************************************************************/
int CheckIncomingModule ( TCX_REF_PTR REF );


/****************************************************************************
 * SubscribeForAutoReply                                                    *
 *                                                                          *
 * Parameters : None                                                        *
 * Returns    : Nothing                                                     *
 * Does       : Sends the Subscribe message to the REAL_BASE                * 
 * Called     : At the very beginning, when the router is initialized       *
 ****************************************************************************/
void SubscribeForAutoReply ( void );


/****************************************************************************
 * SetupAutoReplyForModule                                                  *
 *                                                                          *
 * Parameters : ModuleIndex, status 0/1, sonar 0/1 , colli 0/1              *
 *              0 = OFF , 1 = ON                                            *
 * Returns    : 0 if ok ,or -1 if failure                                   *
 * Does       : Sets the module flags for receiving autoreplys              *
 * Called     : Always when a module requests subscriptions to autoreply    *
 ****************************************************************************/
int SetupAutoReplyForModule ( int Module , int status , int sonar, int colli,
			      int laser);


/****************************************************************************
 * UpdateAutoReplyList                                                      *
 *                                                                          *
 * Parameters : None                                                        *
 * Returns    : Nothing                                                     *
 * Does       : Creates global lists from which other routines can find out *
 *              which modules have to get a sonar or status report.         *
 * Called     : Always when there is a change in the RMR                    *
 ****************************************************************************/
void UpdateAutoReplyList ( void );


/****************************************************************************
 * SearchModuleByName                                                       *
 *                                                                          *
 * Parameters : A string describing a modules name                          *
 * Returns    : The index of the RMR is module exists or -1 if not          *
 * Does       : Scans the RMR list for the module name                      *
 * Called     : When the RMR index is needed and only the module name avail *
 ****************************************************************************/
int SearchModuleByName ( char *ModuleName );


/****************************************************************************
 * ChangeModuleStatus                                                       *
 *                                                                          *
 * Parameters : The RMR Module index and the new status                     *
 * Returns    : Nothing                                                     *
 * Does       : Changes the module connection status                        *
 * Called     : When the status of a particular module has to be changed    *
 ****************************************************************************/
void ChangeModuleStatus ( int ModuleIndex , int newStatus );


/*****************************************************************************
 * IncreaseQuery                                                             *
 *                                                                           *
 * Parameters : Module and type of query                                     *
 * Returns    : 0 if query is stopped or 1 if query has to be passed         *
 * Does       : Checks whether the query value has to be increased or not.   *
 * Called     : Whenever there is a query                                    *
 *****************************************************************************/
int IncreaseQuery ( int ModuleIndex , int query_type );


/*****************************************************************************
 * GetQuery                                                                  *
 *                                                                           *
 * Parameters : Module and type of query                                     *
 * Returns    : the number of queries left to be processed                   *
 * Does       : Nothing                                                      *
 * Called     : When the information is needed                               *
 *****************************************************************************/
int GetQuery ( int ModuleIndex , int query_type );


/*****************************************************************************
 * DecreaseQuery                                                             *
 *                                                                           *
 * Parameters : Module and type of query                                     *
 * Returns    : Nothing                                                      *
 * Does       : Decreases the number of queries                              *
 * Called     : After the reply has been sent back                           *
 *****************************************************************************/
void DecreaseQuery ( int ModuleIndex , int query_type );


/*****************************************************************************
 * MainControlLoop                                                           *
 *                                                                           *
 * Parameters : None                                                         *
 * Returns    : Nothing, stopped by SIGKILL                                  *
 * Does       : Looking for incoming messages with tcxRecvLoop               *
 * Called     : After initialization, will loop forever                      *
 *****************************************************************************/
void MainControlLoop ( void );









