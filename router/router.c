
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






/* -------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <sys/time.h>
#include <string.h>
/* -------------------------------------------------------------------------*/
#include "tcx.h"
#include "tcxP.h"
/* -------------------------------------------------------------------------*/
#include "BASE-messages.h"
/* -------------------------------------------------------------------------*/
#include "router.h"
/* -------------------------------------------------------------------------*/


/* Global init of RouterModuleReference Array  RMR */
int CurrentNumberOfModules = 0;

RouterModuleReferenceType RMR[MaxNumberOfModules];
static int RMR_Initialized=0;

RouterMessageReferenceType RmR[MaxNumberOfMessages];
static int RmR_Initialized=0;

int SONAR_AUTOREPLY_IS_ON   = 0;
int STATUS_AUTOREPLY_IS_ON  = 0;
int COLLI_AUTOREPLY_IS_ON   = 0;
int LASER_AUTOREPLY_IS_ON   = 0;

int NumberOfSonarReplyModules  = 0;    /* these are for the lengths of  */
int NumberOfStatusReplyModules = 0;    /* the autoreply lists           */
int NumberOfColliReplyModules  = 0;
int NumberOfLaserReplyModules  = 0;

int SonarAutoReplyList[MaxNumberOfModules];
int StatusAutoReplyList[MaxNumberOfModules];
int ColliAutoReplyList[MaxNumberOfModules];
int LaserAutoReplyList[MaxNumberOfModules];

TCX_MODULE_PTR REAL_BASE = NULL;

struct timeval block_waiting_time = {1,0};
struct timeval TCX_waiting_time = {0,0};

/* DEFAULT FOR AUTOREPLY */
int sonar_on    = 1;
int status_on   = 1;
int colli_on    = 0;
int laser_on    = 0;
int verbose_on  = 0;

/*------- LOCAL SUPPORT ROUTINES ----------------------------------------*/
int GetFreeModuleEntry ( void );



/*------- THE CODE ------------------------------------------------------*/
/****************************************************************************
 * InitRouterModuleReferenceArray                                           *
 *                                                                          *
 * Parameters : NONE                                                        *
 * Returns    : 0 if ok , -1 if failure                                     *
 * Does       : Initializes the RMR by blanking.                            *
 * Called     : At the very beginning, when the router is initialized       *
 ****************************************************************************/
int InitRouterModuleReferenceArray ( void )
{
  int i=0;
  
  if ( RMR_Initialized == 0 )
    {
      for ( i=0 ; i < MaxNumberOfModules ; i++ )
	{
	  RMR[i].ModuleName[0] = 0;
	  RMR[i].NumOfMessagesSend = 0;
	  RMR[i].NumOfMessagesReceived = 0;
	  RMR[i].Autoreply = 0;
	  RMR[i].ModuleStatus = DISCONNECTED;  /* initial configuration */
	  /* initialize queries */
	  RMR[i].QUERYS[STATUS_QUERY] = 0;
	  RMR[i].QUERYS[SONAR_QUERY] = 0;
	  RMR[i].QUERYS[COLLI_QUERY] = 0;
	  RMR[i].QUERYS[POSITION_QUERY] = 0;
	}
      RMR_Initialized++;
    }
  else
    {
      fprintf (stderr,"\nROUTER Warning : RMR array can not be initialized more than once !");
      return -1;
    }
  return 0;
}

/****************************************************************************
 * InitRouterMessageReferenceArray                                          *
 *                                                                          *
 * Parameters : NONE                                                        *
 * Returns    : 0 if ok , -1 if failure                                     *
 * Does       : Initializes the RmR by blanking.                            *
 *              Defines the type of each message.                           *
 * Called     : At the very beginning, when the router is initialized       *
 ****************************************************************************/
int InitRouterMessageReferenceArray ( void )
{
  int i=0,j=0;

  if ( RmR_Initialized == 0 )
    {
      for ( i=0 ; i < MaxNumberOfMessages ; i++ )
	{
	  RmR[i].ReplyExpected = 0;
	  RmR[i].Amount_Incoming = 0;
	  RmR[i].Amount_Transferred = 0;
	  
	  /* initialize the module reply list */
	  
	  for ( j=0 ; j < MaxNumberOfModules ; j++ )
	    RmR[i].ReplyModuleList[j]=0;
	    
	}

      /* initialize message type by hand */
      
      RmR[BASE_REGISTER_AUTO_UPDATE].MessageType            = INTERCEPT;
      RmR[BASE_ROBOT_POSITION_QUERY].MessageType            = NOTIFY;
      RmR[BASE_ROBOT_POSITION_REPLY].MessageType            = REPLY_IT;
      RmR[BASE_STOP_ROBOT].MessageType                      = PASS_THRU;
      RmR[BASE_UPDATE_STATUS_QUERY].MessageType             = NOTIFY; 
      RmR[BASE_UPDATE_STATUS_REPLY].MessageType             = REPLY_IT;
      RmR[BASE_SET_VELOCITY].MessageType                    = PASS_THRU;
      RmR[BASE_SET_ACCELERATION].MessageType                = PASS_THRU;
      RmR[BASE_ROTATE_CLOCKWISE].MessageType                = PASS_THRU;
      RmR[BASE_ROTATE_ANTICLOCKWISE].MessageType            = PASS_THRU;
      RmR[BASE_TRANSLATE_FORWARD].MessageType               = PASS_THRU;
      RmR[BASE_TRANSLATE_BACKWARD].MessageType              = PASS_THRU;
      RmR[BASE_GOTO_RELATIVE].MessageType                   = PASS_THRU;
      RmR[BASE_TRANSLATE_BY].MessageType                    = PASS_THRU;
      RmR[BASE_ROTATE_BY].MessageType                       = PASS_THRU;
      RmR[BASE_GOTO_ABSOLUTE].MessageType                   = PASS_THRU;
      RmR[BASE_APPROACH_ABSOLUTE].MessageType               = PASS_THRU;
      RmR[BASE_APPROACH_ABSOLUTE_TWO_POINTS].MessageType    = PASS_THRU;
      RmR[BASE_DISCONNECT].MessageType                      = INTERCEPT;
      RmR[BASE_SETMODE].MessageType                         = PASS_THRU;
      RmR[BASE_ACTION_EXECUTED_REPLY].MessageType           = REPLY_IT;
      RmR[BASE_RESET_JOYSTICK].MessageType                  = PASS_THRU;
      RmR[BASE_NOTIFY_WALL_ORIENTATION].MessageType         = PASS_THRU;
      
      RmR[SONAR_SWITCH_ON].MessageType                      = PASS_THRU;
      RmR[SONAR_SWITCH_OFF].MessageType                     = PASS_THRU;
      RmR[SONAR_ACTIVATE_MASK].MessageType                  = PASS_THRU;
      RmR[SONAR_SONAR_QUERY].MessageType                    = NOTIFY;
      RmR[SONAR_SONAR_REPLY].MessageType                    = REPLY_IT;
      RmR[SONAR_DEFINE_MASK].MessageType                    = PASS_THRU;
     
      RmR[COLLI_COLLI_REPLY].MessageType                    = PASS_THRU;
      RmR[COLLI_VISION_LINE].MessageType                    = PASS_THRU;
      RmR[COLLI_VISION_POINT].MessageType                   = PASS_THRU;
      RmR[COLLI_PARAMETER].MessageType                      = PASS_THRU;
      RmR[COLLI_DISCONNECT].MessageType                     = PASS_THRU;

      RmR[LASER_LASER_REPLY].MessageType                    = PASS_THRU;

      RmR_Initialized++;
    }
  else
    {
      fprintf (stderr,"\nROUTER Warning : RmR array can not be initialized more than once !");
      return -1;
    }
  return 0;
}

/****************************************************************************
 * Connect2RealBASE                                                         *
 *                                                                          *
 * Parameters : NONE                                                        *
 * Returns    : 0 if ok , -1 if failure                                     *
 *              Blocks until connection can be established                  *
 * Does       : Creates TCX Module Reference                                *
 * Called     : At the very beginning, when the router is initialized       *
 ****************************************************************************/
int Connect2RealBASE ( void )
{ 
  fprintf (stderr,"\nConnecting to real BASE process ...");
  
  /* BASE_ROUTED is the name of the real BASE module when it is being started
     with the option -router */

  REAL_BASE = tcxConnectModule ( "BASE_ROUTED" );
  
  fprintf (stderr," done !\n");
 
  if ( REAL_BASE == NULL )
    {
      fprintf (stderr,"\nROUTER Critical Error : TCX returned NULL pointer when connecting to real BASE !");
      exit(-1);
    }
  
  return 0;
}

/****************************************************************************
 * SubscribeForAutoReply                                                    *
 *                                                                          *
 * Parameters : None                                                        *
 * Returns    : Nothing                                                     *
 * Does       : Sends the Subscribe message to the REAL_BASE                * 
 * Called     : At the very beginning, when the router is initialized       *
 ****************************************************************************/
void SubscribeForAutoReply ( void )
{
  BASE_register_auto_update_type BASE_register_auto_update;
  
  /* DEFAULT SETUP */
  BASE_register_auto_update.subscribe_status_report = 1;
  BASE_register_auto_update.subscribe_sonar_report  = 1;
  BASE_register_auto_update.subscribe_colli_report  = 0;
  BASE_register_auto_update.subscribe_laser_report  = 0;

  if ( verbose_on == 1 )
    fprintf (stderr,"\nSubscribing for auto reply ...");

  if ( status_on != 0 )
    {
      BASE_register_auto_update.subscribe_status_report = 1;
      STATUS_AUTOREPLY_IS_ON = 1;
    }

  if ( sonar_on != 0 )
    {
      BASE_register_auto_update.subscribe_sonar_report  = 1;
      SONAR_AUTOREPLY_IS_ON = 1;
    }
  
  if ( colli_on != 0 )
    {
      BASE_register_auto_update.subscribe_colli_report  = 3;
      COLLI_AUTOREPLY_IS_ON = 1;
    }
  
  if ( laser_on != 0 )
    {
      BASE_register_auto_update.subscribe_laser_report  = 1;
      LASER_AUTOREPLY_IS_ON = 1;
    }
  
  tcxSendMsg (REAL_BASE,"BASE_register_auto_update",
	      &BASE_register_auto_update );
  if ( verbose_on == 1)
    fprintf (stderr," done !\n");
}

/****************************************************************************
 * CheckIncomingModule                                                      *
 *                                                                          *
 * Parameters : TCX_REF_PTR or the module Name                              *
 * Returns    : Module index if known,or -1 if failure                      *
 * Does       : Searches the Module list for the Module index or            * 
 *              creates a new module entry.                                 *
 * Called     : At the very beginning, when the router is initialized       *
 ****************************************************************************/
int CheckIncomingModule ( TCX_REF_PTR REF )
{
  int IncomingModule=-2,newEntry=-1;
  
  /* Check if TCX_REF_PTR is ok */
  if ( REF->module->moduleInfo->name != NULL )
    IncomingModule = SearchModuleByName ( REF->module->moduleInfo->name );

  if ( IncomingModule == -1 ) /* not known */
    {
      newEntry = GetFreeModuleEntry ();

      if ( verbose_on == 1 )
	fprintf (stderr,"\nNew Entry at %d for module %s",newEntry,
		 REF->module->moduleInfo->name );
      if ( newEntry != -1 )
	{
	  /* enter data about module into the RMR */
	  strcpy ( RMR[newEntry].ModuleName , REF->module->moduleInfo->name ); 
	  RMR[newEntry].module = REF->module; /* the TCX_MODULE_PTR */
	  RMR[newEntry].ModuleStatus = ACTIVE;
	  CurrentNumberOfModules++;
	}
      else
	fprintf (stderr,"\nROUTER Critical Error : Can't get new entry for module <<%s>>",REF->module->moduleInfo->name );
      return newEntry;
    }
  
  if ( IncomingModule != -2 )
    return IncomingModule;
  
  return -1;
}

/****************************************************************************
 * GetFreeModuleEntry                                                       *
 *                                                                          *
 * Parameters : None                                                        *
 * Returns    : The index to a free module in the RMR or -1 if failure      *
 * Does       : Scans the RMR list for a free entry                         *
 * Called     : When a new Module is entered into the RMR                   *
 ****************************************************************************/
int GetFreeModuleEntry ( void )
{
  int ModuleIndex=0,free_entry=-1;

  while ( ModuleIndex < MaxNumberOfModules && free_entry == -1 )
    {
      if ( RMR[ModuleIndex].ModuleStatus == DISCONNECTED )
	free_entry = ModuleIndex;
      ModuleIndex++;
    }
  /* Just for security */
  if ( free_entry >= MaxNumberOfModules )
    free_entry =-1;

  return free_entry;
}

/****************************************************************************
 * SearchModuleByName                                                       *
 *                                                                          *
 * Parameters : A string describing a modules name                          *
 * Returns    : The index of the RMR is module exists or -1 if not          *
 * Does       : Scans the RMR list for the module name                      *
 * Called     : When the RMR index is needed and only the module name avail *
 ****************************************************************************/
int SearchModuleByName ( char *ModuleName )
{
  int foundName = -1,listindex=0;
  
  if ( ModuleName != NULL )
    {
      while ( foundName == -1 && listindex < CurrentNumberOfModules )
	{
	  if ( strlen ( ModuleName ) ==
	      strlen ( RMR[listindex].ModuleName ) )
	    {
	      if ( strcmp ( ModuleName,RMR[listindex].ModuleName ) == 0 )
		foundName=listindex;
	    }
	  listindex++;
	}
      if ( foundName == -1 && verbose_on == 1 )
	fprintf ( stderr,"\nModule %s is not known", ModuleName );
    }
  return foundName;
}

/****************************************************************************
 * ChangeModuleStatus                                                       *
 *                                                                          *
 * Parameters : The RMR Module index and the new status                     *
 * Returns    : Nothing                                                     *
 * Does       : Changes the module connection status                        *
 * Called     : When the status of a particular module has to be changed    *
 ****************************************************************************/
void ChangeModuleStatus ( int ModuleIndex , int newStatus )
{
  if ( ModuleIndex >= 0 && ModuleIndex < MaxNumberOfModules )
    RMR[ModuleIndex].ModuleStatus = newStatus;
}



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
			      int laser)
{
  if ( verbose_on == 1 )
    fprintf(stderr,"\nSetting up autoreply for module %s",RMR[Module].ModuleName);
  if ( Module >= 0 && Module < CurrentNumberOfModules )
    {
      RMR[Module].status = status;
      RMR[Module].sonar  = sonar;
      RMR[Module].colli  = colli;
      RMR[Module].laser  = laser;
      RMR[Module].ModuleStatus = ACTIVE;
      UpdateAutoReplyList();
    }
  return 0;
}

/****************************************************************************
 * UpdateAutoReplyList                                                      *
 *                                                                          *
 * Parameters : None                                                        *
 * Returns    : Nothing                                                     *
 * Does       : Creates global lists from which other routines can find out *
 *              which modules have to get a sonar or status report.         *
 * Called     : Always when there is a change in the RMR                    *
 ****************************************************************************/
void UpdateAutoReplyList ( void )
{
  int i;
  
  NumberOfSonarReplyModules  = 0;
  NumberOfStatusReplyModules = 0;
  NumberOfColliReplyModules  = 0;
  NumberOfLaserReplyModules  = 0;

  for ( i=0 ; i<CurrentNumberOfModules ; i++ )
    {
      if ( RMR[i].status > 0 && RMR[i].ModuleStatus == ACTIVE )
	{
	  StatusAutoReplyList[NumberOfStatusReplyModules] = i;
	  NumberOfStatusReplyModules++;
	}
      if ( RMR[i].sonar > 0 && RMR[i].ModuleStatus == ACTIVE )
	{
	  SonarAutoReplyList[NumberOfSonarReplyModules] = i;
	  NumberOfSonarReplyModules++;
	}
      if ( RMR[i].colli > 0 && RMR[i].ModuleStatus == ACTIVE )
	{
	  ColliAutoReplyList[NumberOfColliReplyModules] = i;
	  NumberOfColliReplyModules++;
	}
      if ( RMR[i].laser > 0 && RMR[i].ModuleStatus == ACTIVE )
	{
	  LaserAutoReplyList[NumberOfLaserReplyModules] = i;
	  NumberOfLaserReplyModules++;
	}
    }
  if ( verbose_on == 1 )
    fprintf(stderr,"\nUpdating AutoReplyList : STATUS(%d) SONAR(%d) COLLI(%d) LASER(%d)",
	    NumberOfStatusReplyModules,
	    NumberOfSonarReplyModules,
	    NumberOfColliReplyModules,
	    NumberOfLaserReplyModules);
  
}

/*****************************************************************************
 * IncreaseQuery                                                             *
 *                                                                           *
 * Parameters : Module and type of query                                     *
 * Returns    : 0 if query is stopped or 1 if query has to be passed         *
 * Does       : Checks whether the query value has to be increased or not.   *
 * Called     : Whenever there is a query                                    *
 *****************************************************************************/
int IncreaseQuery ( int ModuleIndex , int query_type )
{
  int rtn=1;

  switch ( query_type )
    {
    case STATUS_QUERY:
      /* check if there is already autoreply */
      if ( RMR[ModuleIndex].status == 1 )
	rtn = 0;
      else
	RMR[ModuleIndex].QUERYS[STATUS_QUERY]++;
      if ( STATUS_AUTOREPLY_IS_ON == 1 )
	rtn=0; /* block if autoreply is on */
      break;
    case SONAR_QUERY:
      /* check if there is already autoreply */
      if ( RMR[ModuleIndex].sonar == 1 )
	{
	  rtn=0;
	  if ( verbose_on == 1)
	    fprintf (stderr,"\nSONAR QUERY BLOCKED !");
	}
      else
	{
	  RMR[ModuleIndex].QUERYS[SONAR_QUERY]++;
	  if ( verbose_on == 1)
	    fprintf (stderr,"\nSONAR QUERY STORED !");
	}
      if ( SONAR_AUTOREPLY_IS_ON == 1 )
	{
	  rtn=0; /* block if autoreply is on */
	  if ( verbose_on == 1)
	    fprintf (stderr,"\nSONAR QUERY FINALLY BLOCKED !");
	}
      break;
    case POSITION_QUERY:
      RMR[ModuleIndex].QUERYS[POSITION_QUERY]++;
      break;
    default: 
      fprintf (stderr,"\nROUTER Error : IncreaseQuery reports unknown query type (%d) !",query_type );
      break;
    }
  return rtn;
}

/*****************************************************************************
 * GetQuery                                                                  *
 *                                                                           *
 * Parameters : Module and type of query                                     *
 * Returns    : the number of queries left to be processed                   *
 * Does       : Nothing                                                      *
 * Called     : When the information is needed                               *
 *****************************************************************************/
int GetQuery ( int ModuleIndex , int query_type )
{
  return RMR[ModuleIndex].QUERYS[query_type];
}

/*****************************************************************************
 * DecreaseQuery                                                             *
 *                                                                           *
 * Parameters : Module and type of query                                     *
 * Returns    : Nothing                                                      *
 * Does       : Decreases the number of queries                              *
 * Called     : After the reply has been sent back                           *
 *****************************************************************************/
void DecreaseQuery ( int ModuleIndex , int query_type )
{
  if ( RMR[ModuleIndex].QUERYS[query_type] > 0 )
    RMR[ModuleIndex].QUERYS[query_type]--;
  else
    {
      fprintf (stderr,"\nROUTER Error : QUERY Amount is already <= 0 !!" );
      RMR[ModuleIndex].QUERYS[query_type]=0;
    }
}

/*****************************************************************************
 * MainControlLoop                                                           *
 *                                                                           *
 * Parameters : None                                                         *
 * Returns    : Nothing, stopped by SIGKILL                                  *
 * Does       : Looking for incoming messages with tcxRecvLoop               *
 * Called     : After initialization, will loop forever                      *
 *****************************************************************************/
void MainControlLoop ( void )
{
  if ( verbose_on == 1 )
    fprintf (stderr,"\nEntering Router Main Control !!!");
  while ( 1 != 0 )
    {
      block_waiting_time.tv_sec = 1;
      block_waiting_time.tv_usec = 0;
      block_wait(&block_waiting_time, 1,0 );
      tcxRecvLoop ((void *) &TCX_waiting_time);
    }
}






