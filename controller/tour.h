
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


#ifdef TOURGUIDE_VERSION

#define MAX_NUMBER_TOUR_GOALS 20


void 
start_learning_tour(ALL_PARAMS);

void 
add_tour_goal(ALL_PARAMS);

void 
finish_learning_tour(ALL_PARAMS);

void 
start_giving_tour(ALL_PARAMS);

void 
move_to_next_tour_goal(ALL_PARAMS);

void 
reached_a_tour_goal(ALL_PARAMS);

void
next_text(ALL_PARAMS);

void 
stop_giving_tour(ALL_PARAMS, int restart);

void 
save_tour(ALL_PARAMS);

void 
load_tour(ALL_PARAMS);

void 
handle_button_in_tour(ALL_PARAMS);

void 
handle_flow_reply_in_tour(ALL_PARAMS);


void
tourguide_check_for_timeout(ALL_PARAMS);
#endif /* TOURGUIDE_VERSION */

