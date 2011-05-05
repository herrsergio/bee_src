
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



/*
 * Copyright 1994, Brown University, Providence, RI
 *
 * Permission to use and modify this software and its documentation for
 * any purpose other than its incorporation into a commercial product is
 * hereby granted without fee.  Permission to copy and distribute this
 * software and its documentation only for non-commercial use is also
 * granted without fee, provided, however, that the above copyright notice
 * appear in all copies, that both that copyright notice and this permission
 * notice appear in supporting documentation, that the name of Brown
 * University not be used in advertising or publicity pertaining to
 * distribution of the software without specific, written prior permission,
 * and that the person doing the distribution notify Brown University of
 * such distributions outside of his or her organization. Brown University
 * makes no representations about the suitability of this software for
 * any purpose.  It is provided "as is" without express or implied warranty.
 * Brown University requests notification of any modifications to this
 * software or its documentation.
 *
 * Send the following redistribution information:
 *
 *	Name:
 *	Organization:
 *	Address (postal and/or electronic):
 *
 * To:
 *	Software Librarian
 *	Computer Science Department, Box 1910
 *	Brown University
 *	Providence, RI 02912
 *
 *		or
 *
 *	brusd@cs.brown.edu
 *
 * We will acknowledge all electronic notifications.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h> 
#include <signal.h>     /* We want to make sure ^C turns off scheduler */
#include <rai.h>


/*************************************************

 GLOBALS IN THIS FILE                          

  RaiModule* modules[]                         
  int module_count                             			
                                               
 These implement a table of registered modules 
 When the user calls makeModule() a new module 
 is created and placed in the modules[] by the 
 register_module macro.  We then later look up 
 the modules in this array when scheduling.    
                                               
 int rai_initialized                           			
                                               
  Set the first time RaiInit() is called, so   
  that subsequent calls to RaiInit() do nothing 
                                               
 int shutdown_called                           			
                                               
  When this flag is set, the scheduling loop,  
  in handle_modules quits.  It is set when a   
  module calls the RaiShutdown() function.     
                                               
                                               
 RAI_TIME_START				 	
 The timeval which the getRaiTime() clock should
 consider the beginning of time.  See           
 setRaiTime and getRaiTime()                   

  time_check_on, profile_on
  Allow the user to turn output from these
  facilities on and off at run time, 
  assuming the system has been compile to 
  use them.

 FILE * RaiError;
  File pointer where error output will go.
  This defaults to stderr, but can be set
  via setErrorStream(); 

*************************************************/


RaiModule * (modules[MAX_MODULES]);
int module_count=0;
int shutdown_called=FALSE;
int rai_initialized=FALSE;
timeval RAI_TIME_START; 

int time_check_on = FALSE;
int profile_on = FALSE;
FILE * RaiError = stderr; 

int report_interval = DEFAULT_REPORT_INTERVAL;

#define register_module(mod) {	\
	  if (module_count >=MAX_MODULES) \
	    {fprintf(RaiError,"Exceeded MAX_MODULES in module.h\n"); \
	     exit(-1);} \
	else				\
	  modules[module_count++]=mod;}	


#define check_init(mod_name) { \
	  if (!rai_initialized) \
	    {fprintf(RaiError,"You must call RaiInit() before %s\n",mod_name); \
	     fprintf(RaiError,"Exiting RAI\n"); \
	     exit(-1);} }

/* If TRACE was defined at compile time and the user has  */
/* turned tracing on at runtime, report some event          */

/* Note: this will choke big if you screw up and pass something */
/* that is neither NULL or a string */

#ifdef TRACE 				      
#define mod_report(mod,event,arg) { \
	  if (mod->trace_on) \
	    {  fprintf(RaiError,"TRACE %s: ",(char*) mod->name);   \
	       fprintf(RaiError,"%s", (char*) event);     \
	       if (arg) fprintf(RaiError,"%d",(int) arg); \
	       fprintf(RaiError,"\n");}}

#else
#define mod_report(mod,event,arg) {}
#endif



/**********************************************************/
/*                                                        */
/* RaiInit                                                */
/*                                                        */
/*  Note you can call the more than once before RaiStart  */     
/**********************************************************/

void RaiInit(void)
{
  if (!rai_initialized) 
    {

  module_count=0;
  shutdown_called = FALSE;
  rai_initialized = TRUE;
}

}

/**********************************************************/
/*                                                        */
/*                                                        */
/*                                                        */
/*                                                        */
/**********************************************************/

RaiModule* makeModule(char* name, void (*shutdown) (RaiModule *))
{

  RaiModule * newmod;
  int i;

  check_init("makeModule");

  for(i=0;i<module_count;i++)
    if (!strcmp(name,modules[i]->name))
      fprintf(RaiError,"makeModule: Two modules have name %s\n",name);

  newmod = (RaiModule *) malloc(sizeof(RaiModule));
  newmod->name = strdup(name);
  newmod->shutdown = shutdown;
  newmod->trace_on = FALSE;

  /* assume time is unlimited unless set in set_time_limit */
  newmod->poll_time_limit.tv_sec = END_OF_TIME;
  newmod->poll_time_limit.tv_usec = 0;

  newmod->timeout_time_limit.tv_sec = END_OF_TIME;
  newmod->timeout_time_limit.tv_usec = 0;

  newmod->select_time_limit.tv_sec = END_OF_TIME;
  newmod->select_time_limit.tv_usec = 0;


  /* turn off polling and timeouts */
  disableTimeout(newmod);
  disablePolling(newmod);
  disableSelect(newmod);

  /* set fd to something obviously empty */
  /* This is important to setting up the fd->module table later*/
  newmod->fd = ERROR;

  /* register module with module structure */
  register_module(newmod);
  return newmod;
}


void addPolling(RaiModule * mod, void (*poll) (RaiModule *), unsigned long msecs)
{

  timeval now;

  mod_report(mod,"addPolling of ",msecs);

  mod->poll = poll;
  msecsToUsecs(msecs,mod->poll_interval.tv_sec,mod->poll_interval.tv_usec);
  enablePolling(mod);

  /* If RaiStart is called after this, it will reset the next poll    */
  /* time so that it is relative to the start of the schedulere .     */
  /* However, we set it here just in case we call this after RaiStart */
  /* has been called */ 

  gettimeofday(&now,NULL);
  addTime(now,mod->poll_interval,mod->next_poll);
  
#ifdef TIME_CHECK
  setPollingErrorLimit(mod,msecs);
#endif


}

void addTimeout(RaiModule *mod, void (*timeout)(RaiModule *),unsigned long msecs)
{

  timeval now;


  mod_report(mod,"add timout of ",msecs);

  mod->timeout = timeout;
  msecsToUsecs(msecs,mod->timeout_limit.tv_sec,mod->timeout_limit.tv_usec);
  enableTimeout(mod);

  /* If RaiStart is called after this, it will reset the timeout   */
  /* time so that it is relative to the start of the schedulere.   */
  /* However, we set it here just in case we call this after RaiStart */
  /* has been called */ 

  gettimeofday(&now,NULL);
  addTime(now,mod->timeout_limit,mod->next_timeout);

}

void resetTimeout(RaiModule *mod)
{
  timeval now;

  mod_report(mod,"reset timeout",NULL);

  gettimeofday(&now,NULL);
  addTime(now,mod->timeout_limit,mod->next_timeout);
  enableTimeout(mod);
}

void setTimeLimit(RaiModule * mod, timeval * limit, unsigned long msecs)
{
#ifndef TIME_CHECK
  fprintf(RaiError,"Warning: set time limit for %s but ",mod->name);
  fprintf(RaiError,"TIME_CHECK is not enabled\n");
#endif

  msecsToUsecs(msecs,limit->tv_sec,limit->tv_usec);
}


void setPollingErrorLimit(RaiModule * mod, unsigned long msecs)
{
#ifndef TIME_CHECK
  fprintf(RaiError,"Warning: set polling error limit for %s but ",mod->name);
  fprintf(RaiError,"TIME_CHECK is not enabled\n");
#endif

  msecsToUsecs(msecs,mod->poll_error_limit.tv_sec,
	       mod->poll_error_limit.tv_usec);
}


void addSelect(RaiModule *mod, void (*select)(RaiModule *),int fd)
{


  mod_report(mod,"init select fd=",fd);

  if (fd == ERROR)
    {
      fprintf(RaiError,"init_select bad fd passed \n");
      exit(-1);
    }
  mod->fd = fd;
  mod->select = select;
  enableSelect(mod);
}



void raiTrace(RaiModule* mod)
{
#ifndef TRACE
  fprintf(RaiError,"Warning: Attempting to turn trace on but\n");
  fprintf(RaiError,"the system was compiled with TRACE disabled\n");
#endif
 mod->trace_on = TRUE;
}   

void raiUntrace(RaiModule* mod)
{
  mod->trace_on = FALSE;
}   




/************************************************************************
 *
 *   NAME:         interrupt_handler()
 *                 
 *   FUNCTION:     some ^C signal or kill command arrived, turn off scheduler
 *                 
 *   PARAMETERS:   int sig           signal
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void rai_interrupt_handler(int sig)
{
  RaiShutdown();
  exit(0);
}


void catchInterrupts(void)
{
  signal(SIGTERM, &rai_interrupt_handler); /* kill interupt handler */
  signal(SIGINT,  &rai_interrupt_handler); /* control-C interupt handler */
  signal(SIGABRT, &rai_interrupt_handler); /* failed assert handler */
}


void setErrorStream(FILE * new_error)
{
  RaiError = new_error;
}


/**********************************************************

   Profiling and debugging stuff

**********************************************************/

void timeCheckOn(void)
{

#ifndef TIME_CHECK
  fprintf(RaiError,"Warning: Attempting to turn time checking on but\n");
  fprintf(RaiError,"the system was compiled with TIME_CHECK disabled\n");
#endif
 time_check_on = TRUE;
}   

void timeCheckOff(void)
{
 time_check_on = FALSE;
}   

void profileOn(void)
{
#ifndef PROFILE
  fprintf(RaiError,"Warning: Attempting to turn profiling on but\n");
  fprintf(RaiError,"the system was compiled with PROFILE disabled\n");
#endif
 profile_on = TRUE;
}   

void profileOff(void)
{
  profile_on = FALSE;
}   

void setReportInterval(int interval)
{
  if (interval)
    report_interval = interval;
}



void profile_modules(void)
{
  int i;
  RaiModule* curr_mod;

  for (i=0;i<module_count;i++)
    {
      curr_mod = modules[i];
      if (curr_mod != NULL)
	{
	  fprintf(RaiError,"PROFILE: %s\n",curr_mod->name);	  
	  
	  fprintf(RaiError,"\ttimeout\t%d\t",curr_mod->timeout_occurrences);
	  curr_mod->timeout_occurrences = 0;
	  if (!nullTime(curr_mod->timeout_time))
	    {
	      printTimeAsMsecs(RaiError,curr_mod->timeout_time);
	      resetTime(curr_mod->timeout_time);
	    }
	  else
	    fprintf(RaiError,"\n");

	  fprintf(RaiError,"\tselect\t%d\t",curr_mod->timeout_occurrences);
	  curr_mod->select_occurrences = 0;
	  if (!nullTime(curr_mod->select_time))	  
	    {
	      printTimeAsMsecs(RaiError,curr_mod->select_time);
	      resetTime(curr_mod->select_time);
	    }
	  else
	    fprintf(RaiError,"\n");

	  fprintf(RaiError,"\tpoll\t%d\t",curr_mod->poll_occurrences);
	  curr_mod->poll_occurrences = 0;
	  if (!nullTime(curr_mod->poll_time))	  
	    {
	      printTimeAsMsecs(RaiError,curr_mod->poll_time);
	      resetTime(curr_mod->poll_time);
	    }
	  else
	    fprintf(RaiError,"\n");
	}
    }
}

void  check_time(char* mod_name,timeval* limit, char* callback_name,
		 timeval* before,timeval* after)
{
   timeval interval;

   if (time_check_on)
     {
       subtractTime((*after),(*before),interval);
       if (timeGreater(interval,(*limit)))
	 {
	   fprintf(RaiError,"TIME_CHECK: %s exceeds time in callback %s\n",mod_name,
		   callback_name);
	   fprintf(RaiError,"Time allowed: ");
	   printTimeAsMsecs(RaiError,(*limit));
	   fprintf(RaiError,"Time used: ");
	   printTimeAsMsecs(RaiError,interval);
	   fprintf(RaiError,"\n");
	 }
     }
}



/**********************************************************/
/*                                                        */
/*                                                        */
/*                                                        */
/*                                                        */
/**********************************************************/

void handle_modules(int * select_fds,int total_fd_count, RaiModule ** fd_module_map)
{
  fd_set select_set;
  fd_set dummy_set;

  int fd, selected;

  int i = 0;
  RaiModule * mod;
  timeval end_of_time,now,event;
  timeval * low_time_ptr; 


#if ((defined TRACE) || (defined PROFILE) || (defined TIME_CHECK))
  timeval before;
#endif

#if ((defined PROFILE) || (defined TIME_CHECK))
  timeval after;
#endif

#if (defined TIME_CHECK)
  timeval poll_error;
#endif

#ifdef PROFILE
  int loops =0;
  timeval select_time;
  select_time.tv_sec = 0;
  select_time.tv_usec = 0;
#endif;

  end_of_time.tv_sec = END_OF_TIME;
  end_of_time.tv_usec = END_OF_TIME;

  for(;;)
    {
      /* first, figure out what the next event we need to worry about is */
      low_time_ptr = &end_of_time;
      for (i=0;i<module_count;i++)
	{
	  mod = modules[i];
	  if (mod->timeout_on&&(timeGreater((*low_time_ptr),mod->next_timeout)))
	    low_time_ptr = &(mod->next_timeout);
	  if (mod->poll_on && (timeGreater((*low_time_ptr),mod->next_poll)))
	    low_time_ptr = &(mod->next_poll);
	}

      /* reset the FD sets, as they get modified in place */
      FD_ZERO(&dummy_set);
      FD_ZERO(&select_set);
      for(i=0;i<total_fd_count;i++)
	FD_SET(select_fds[i],&select_set);

      gettimeofday(&now,NULL);
      subtractTime((*low_time_ptr),now,event);

      #ifdef PROFILE
      /* we add the event time, then subtract the time left over */
      /* returned in &even by select to get the total time used */
      /* in select */
      addTime(select_time,event,select_time);
      #endif

      selected=select(FD_SETSIZE,&select_set,&dummy_set,&dummy_set,&event);
      if (selected)
	{
          #ifdef PROFILE
	  subtractTime(select_time,event,select_time);
          #endif

	  for(i=0;i<total_fd_count;i++)
	    {
	      fd =select_fds[i];
	      if (FD_ISSET(fd,&select_set))
		{
		  mod = fd_module_map[fd];
		  if (mod->select_on)
		    {
#if ((defined PROFILE)||(defined TIME_CHECK)||(defined TRACE))
		      gettimeofday(&before,NULL);
#endif
		      mod_report(mod,"select at ",timevalToMsecs(before));
		      mod->select(mod);
#if ((defined PROFILE) || (defined TIME_CHECK))
		      gettimeofday(&after,NULL);
#endif
#ifdef TIME_CHECK
		      check_time(mod->name,&(mod->select_time_limit),
				 "select",&before,&after);
#endif
		      
#ifdef PROFILE
		      addInterval(before,after,mod->select_time);
		      mod->select_occurrences++;
#endif	      
		    }
		  selected--;
		  if (!selected)
		    break;
		}
	    }
	}
      
      /* we have selected or timed out, see if anyone has expired */
      /* could also check to see if selected==0, ie we timed out*/
      for (i=0;i<module_count;i++)
	{
	  mod = modules[i];

	  /*******************************************************
	            Do polling
	  *******************************************************/

	  if (mod->poll_on && (timeGreater(now,mod->next_poll)))
	    {
#if ((defined PROFILE) || (defined TIME_CHECK) || (defined TRACE))
	      gettimeofday(&before,NULL);
#endif 

	      mod_report(mod,"poll at ",timevalToMsecs(before));
	      mod->poll(mod);   

#if ((defined PROFILE) || (defined TIME_CHECK))
	      gettimeofday(&after,NULL);
#endif

#ifdef TIME_CHECK
	      check_time(mod->name,&(mod->poll_time_limit),"poll",&before,&after);
	      
	      subtractTime(now,mod->next_poll,poll_error);
	      if(timeGreater(poll_error,mod->poll_error_limit))
		{
		  fprintf(RaiError,"TIME_CHECK: %s poll interval missed by ",mod->name);
		  printTimeAsMsecs(RaiError,poll_error);
		}
#endif

#ifdef PROFILE
	      addInterval(before,after,mod->poll_time);
	      mod->poll_occurrences++;
#endif	      
	      addTime(now,mod->poll_interval,mod->next_poll);
	    }

      /*******************************************************
	Do timeout
	*******************************************************/

      if (mod->timeout_on && (timeGreater(now,mod->next_timeout)))
	{
#if ((defined PROFILE) || (defined TIME_CHECK) || (defined TRACE))
	      gettimeofday(&before,NULL);
#endif

	      mod_report(mod,"timeout at ",timevalToMsecs(before));
	      /* we are about to service timeout, so it is no longer pending*/
	      /* if the user wants another, they should use resetTimeout*/
	      disableTimeout(mod);	     
	      mod->timeout(mod);


#if ((defined PROFILE) || (defined TIME_CHECK))
	      gettimeofday(&after,NULL);
#endif

#ifdef TIME_CHECK
	      check_time(mod->name,&(mod->timeout_time_limit),"timeout",
			 &before,&after);
#endif

#ifdef PROFILE
	      addInterval(before,after,mod->timeout_time);
	      mod->timeout_occurrences++;
#endif	      
	    }
	}

#ifdef PROFILE

      loops++;
      if (loops == report_interval)
	{
	  /* allow user to turn profile reporting on and off */
	  /* at run time */
	  if (profile_on)	
	    {
	      fprintf(RaiError,"Time spent in select ");
	      printTimeAsMsecs(RaiError,select_time);
	      profile_modules();
	    }
	  loops = 0;
	  select_time.tv_sec =0;
	  select_time.tv_usec =0;
	}
#endif	      

      /*  If one of the above callbacks called shutdown, it will */
      /*  call the shutdown function for each module, then turn  */
      /*  all the polling_on and like bits off, as well as set   */
      /*  the shutdown_called flag.  With the xx_on bits off, we */
      /*  immediately fall through to here and stop scheduling   */

      if (shutdown_called)
	return;
    }
}

/**********************************************************/
/*                                                        */
/*                                                        */
/*                                                        */
/*                                                        */
/**********************************************************/


void RaiStart(void)
{

  int i;
  timeval now;
  RaiModule * curr_mod;

  int * select_fds; 
  RaiModule ** fd_module_map = NULL;

  int total_fd_count =0;
  int curr_mod_fd = 0;
  int curr_fd_count = 0;
  int max_fd = -1;

  check_init("RaiStart");

  /* first, figure out how many selects we have, and what the max fd is*/
  /* This will be used to pass the list of fds to select, and map the */
  /* fds returned by select back into modules */

  for (i=0;i<module_count;i++)
    {
      curr_mod_fd = (modules[i])->fd;
      if (curr_mod_fd != ERROR)
	{
	  total_fd_count++;
	  if (curr_mod_fd > max_fd)
	    max_fd = curr_mod_fd;
	}
    }

  if (max_fd >= 0)
    fd_module_map = (RaiModule**) calloc((max_fd+1), sizeof(RaiModule*));
  select_fds = (int*) calloc( (total_fd_count+1), sizeof(int));

  for (i=0;i<module_count;i++)
    {
      curr_mod = modules[i];
      curr_mod_fd = curr_mod->fd;
      if (curr_mod_fd != ERROR)
	{
	  fd_module_map[curr_mod_fd] = curr_mod;
	  select_fds[curr_fd_count++] = curr_mod_fd;
	}
    }     

  gettimeofday(&now,NULL);
  for (i=0;i<module_count;i++)
    {
      curr_mod = modules[i];
      if (curr_mod->poll_on)
	addTime(now,curr_mod->poll_interval,curr_mod->next_poll);
      if (curr_mod->timeout_on)
	addTime(now,curr_mod->timeout_limit,curr_mod->next_timeout);
    }

  handle_modules(select_fds,total_fd_count,fd_module_map);


/*  Why do we free everything here rather than in shutdown()? */
/*  Even though the system is shutting down, it is possible   */
/*  that the memory for a module will be referenced, for      */
/*  by the last call to check_time or profile_modules.  Thus, */
/*  we do not want to free the memory until handle_modules is */
/*  completely dead. */

  for (i=0;i<module_count;i++)
    {
      free(modules[i]->name);
      free(modules[i]);
    }
  if (fd_module_map != NULL)
    free(fd_module_map);
  free(select_fds);
}



/**********************************************************/
/*                                                        */
/*                                                        */
/*                                                        */
/*                                                        */
/**********************************************************/

void RaiShutdown(void)
{
  int i;
  RaiModule* curr_mod;

  if (shutdown_called)
    return;
  else
    shutdown_called = TRUE;

  for (i=0;i<module_count;i++)
    {
      curr_mod = modules[i];
      mod_report(curr_mod," shutdown",NULL);
      if(curr_mod->shutdown != NULL)
	curr_mod->shutdown(curr_mod);
    }

/*  Why do we turn everything off here rather than as we shutdown */
/*  each module above?  It is possible that shutdown for module i+1*/
/*  could turn polling for module i back on.  It isn't likely, but*/
/*  this prevents it.                                             */

  for (i=0;i<module_count;i++)
    {
      disablePolling(modules[i]);
      disableTimeout(modules[i]);
      disableSelect(modules[i]);
    }
}



/**********************************************************/
/*                                                        */
/* help parse input line                                  */
/*                                                        */
/*                                                        */
/*                                                        */
/**********************************************************/

int turnedOn(int argc, char** argv, char* name)
{
  int polarity; 
  int arg_no;

  /* if no extra args are passed, everything is on */
  if (argc==1)
    return TRUE;
  
  if (argc==2)
    return ERROR;

  if (!strncmp(argv[1],"+",1))
    polarity = TRUE;
  else
    if (!strncmp(argv[1],"-",1))
      polarity = FALSE;
	else
	  return ERROR;
  
  /* if we find the name in the args, return   	*/
  /* TRUE if we are turning on with +	 	*/
  /* FALSE if we are turning off with - 	*/

  for(arg_no=2;arg_no<argc;arg_no++)
      if (!strcasecmp(name,argv[arg_no]))
	return polarity;

  /* if we do not find the name, the module is on */
  /* if we were turning off, and off if we were   */
  /* turning on					  */	

  return !polarity;

}

/**********************************************************/
/*                                                        */
/* 			                                  */
/*                                                        */
/*                                                        */
/*                                                        */
/**********************************************************/

unsigned long getRaiTime(void)
{
  timeval now;

  gettimeofday(&now,NULL);
  subtractTime(now,RAI_TIME_START,now);
  return timevalToMsecs(now);
}


void setRaiTime(void)
{
  gettimeofday(&RAI_TIME_START,NULL);
}


