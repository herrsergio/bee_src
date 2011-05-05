
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







void
COLLI_send_colli_update();

extern COLLI_colli_reply_type        colli_tcx_status;
extern int n_auto_colli_update_modules;

void COLLI_vision_line_handler(TCX_REF_PTR                ref,
			       COLLI_vision_line_ptr      vision_lines);

void COLLI_vision_point_handler(TCX_REF_PTR                ref,
			       COLLI_vision_point_ptr      vision_points);

void COLLI_parameter_handler(TCX_REF_PTR                ref,
			     COLLI_parameter_ptr      parameters);





