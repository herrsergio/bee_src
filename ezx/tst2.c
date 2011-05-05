
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




#ifdef VMS
#include "vms.h"
struct timeval {
        long    tv_sec;         /* seconds */
        long    tv_usec;        /* and microseconds */
};
#endif

#include "EZX11.h"

main()
{
   EZXW_p w1, w2, w3, w4;
   int i,x,y,x1,y1,x2,y2,x3=0,y3=0,x4,y4,x5=0,y5=0,flag=0;

   char **fontList;
   int fontListNum;

   /* EZX_InitX(); */
   w1 = EZX_MakeWindow("demo",     300, 300, "+600+140");  /* "+200+200" */
   w2 = EZX_MakeWindow("action",   300, 300, "+200+80");  /* "+200+200" */
   w3 = EZX_MakeWindow("graphics", 500, 400, "+500+480");  /* "+200+200" */
   w4 = EZX_MakeWindow("graphics", 400, 300, "+50+480");  /* "+200+200" */

   EZX_SetColor(C_RED);

   for(i=10; i<90; i++)
     EZX_DrawRectangle(w2,i,i/2,i,i);

   EZX_Flush();

   EZX_SetColor(C_FORESTGREEN);

   EZX_DrawLine(w1,10,10,100,100);
   EZX_SetLineStyle(LineOnOffDash);
   EZX_DrawLine(w1,20,10,110,100);
   EZX_SetDashes(2,6);
   EZX_SetLineStyle(LineDoubleDash);
   EZX_DrawLine(w1,30,10,120,100);
   EZX_SetLineStyle(LineSolid);

   EZX_SetLineWidth(5);
   EZX_DrawLine(w1,110,110,200,200);

   EZX_SetLineStyle(LineDoubleDash);
   EZX_DrawLine(w1,110,10,200,100);


   EZX_SetLineWidth(15);
   EZX_SetLineStyle(LineSolid);
   EZX_SetColor(C_BLUE);
   EZX_DrawLine(w1,110,30,200,10);

   EZX_SetMode(GXxor);
   EZX_DrawLine(w1,110,50,200,30);
   EZX_SetMode(GXor);
   EZX_DrawLine(w1,110,70,200,50);
   EZX_SetColor(C_YELLOW);
   EZX_DrawLine(w1,110,90,200,70);

   EZX_UseFont(theGC, "lucidasans-18");
   EZX_SetColor(C_RED);
   EZX_DrawTextAt(w1, 150, 280-EZX_GetFontHeight(), "Press any button",'C'); 
   EZX_DrawTextAt(w1, 150, 280, "in top left window to quit",'C');

   EZX_UseFont(theGC, "8x16");
   EZX_SetColor(C_FORESTGREEN);
   EZX_DrawTextAt(w3, 250, 380-2*EZX_GetFontHeight(), 
    "Try pressing",'C'); 
   EZX_DrawTextAt(w3, 250, 380-EZX_GetFontHeight(), 
    "left, middle and right button",'C');
   EZX_DrawTextAt(w3, 250, 380, 
    "in here",'C');

   EZX_UseFont(theGC, "7x13");
   EZX_SetColor(C_BLUE);
   EZX_DrawTextAt(w4, 200, 280-2*EZX_GetFontHeight(), 
    "Try pressing",'C'); 
   EZX_DrawTextAt(w4, 200, 280-EZX_GetFontHeight(), 
    "left, middle and right button",'C');
   EZX_DrawTextAt(w4, 200, 280, 
    "in here",'C');

   EZX_SetLineWidth(5);
   EZX_SetLineStyle(LineSolid);
   EZX_SetMode(GXcopy);

   x=random()%300;
   y=random()%300;
   while (! EZX_TestCursor(w2))
     {
     x1=(x-15+(random()%31)); 
     if (x1>299) x1=299-(x1-299);
     if (x1<0) x1=-x1;
     y1=(y-15+(random()%31));
     if (y1>299) y1=299-(y1-299);
     if (y1<0) y1=-y1;
     switch (random()%5)
       {
       case 0: EZX_SetColor(C_RED); break;
       case 1: EZX_SetColor(C_BLUE); break;
       case 2: EZX_SetColor(C_FORESTGREEN); break;
       case 3: EZX_SetColor(C_YELLOW); break;
       case 4: EZX_SetColor(C_PINK); break;
       }
     EZX_DrawLine(w2,x,y,x1,y1); x=x1; y=y1;

     if (EZX_TestCursor(w3))
       {
       switch (EZX_TestGetCursor(w3,&x2,&y2))
	  {
	  case LEFT_BUTTON:   x3=x2; y3=y2; 
			      break;
          case MIDDLE_BUTTON: {
         		      int save = EZX_SetColor(C_BLACK);

 		              EZX_DrawLine(w3,x3,y3,x2,y2);
  
  		              EZX_SetColor(save);

		              flag=0;
         		      }
			      break;
	  case RIGHT_BUTTON:  EZX_ClearWindow(w3);
			      break;
	  default:	      break;
          }
       }

     if (EZX_TestCursor(w4))
       {
       switch (EZX_TestGetCursor(w4,&x4,&y4))
	  {
	  case LEFT_BUTTON:   x5=x4; y5=y4; 
			      break;
          case MIDDLE_BUTTON: {
         		      int save = EZX_SetColor(C_BLACK);

 		              EZX_DrawLine(w4,x5,y5,x4,y4);
  
  		              EZX_SetColor(save);

		              flag=0;
         		      }
			      break;
	  case RIGHT_BUTTON:  EZX_ClearWindow(w4);
			      break;
	  default:	      break;
          }
       }

     if (EZX_TestCursor(w1))
       {
       EZX_TestGetCursor(w1,&x2,&y2);
       EZX_bell();
       fprintf(stderr,"don't press button in that window!\n");
       }

     EZX_Flush();	
     }

  EZX_EndWindow(w1);
  EZX_EndWindow(w2);
  EZX_EndWindow(w3);

}

