/* ***************************************************************************

  T_DISTEX.C

  Distance sensor task source code.
  For use with TINYAMR.C

  11 1  2002 - Created
  11 20 2011 - modified for tinyamr.c

*************************************************************************** */

#include  "smte.h"
#include  "names.h"
#include  "messages.h"
#include  "t_dist.h"

#include  "HC08drv.h"
#include  "servo.h"
#include  "gp2d02.h"

/* -----------------------------------------
   global definitions
----------------------------------------- */

/* -----------------------------------------
   functiona prototypes
----------------------------------------- */

void stow(void);
void point(void);

/* -----------------------------------------
   types
----------------------------------------- */

typedef enum {
              STATE_STOW = 0,  // stop scan and stow sensor
              STATE_POINT      // point sensor for distance measurment
             } State_Type;

/* -----------------------------------------
   task globals
----------------------------------------- */

static int        nNavTask      = 0;
static State_Type distanceState = STATE_STOW;

int               nPointDir     = CENTER_FP;


/* -----------------------------------------
   state functions
----------------------------------------- */

/* -----------------------------------------
   stow()

   distance sensor stow state.
----------------------------------------- */
void
stow(void)
{
 int         nMsg;
 WORD        wPayload;
 DWORD       dwPayload;

 // stow sensor to point to forward position
 servoSetAngle(CENTER_FP);

 // wait for message
 nMsg = waitMsg(__ANY__, 0, &wPayload, &dwPayload);

 // parse message
 switch ( nMsg )
    {
     // change scan mode
     case SN_DIST_MODE:
          switch ( wPayload )
             {
              case SN_STOW: /* do nothing */
                   break;

              case SN_POINT:
                   nPointDir = (int) dwPayload;
                   distanceState = STATE_POINT;
                   break;

              case SN_GETHC08STAT:
                   wPayload = hc08GetStatus();
                   putMsg(nNavTask, SN_HC08STAT, wPayload, DW_DONT_CARE);
                   break;

              default:
                   putDebugMsg(DB_TRACE, DB_BAD_MSG, __LINE__); // @@ bad distance mode command in stow()
             } // switch on wPayload
          break;

     // default handler
     default:
          putDebugMsg(DB_TRACE, DB_BAD_MSG, __LINE__); // @@ bad message in stow() 
    } // switch on bMsg
}

/* -----------------------------------------
   point()

   point and measure state.
----------------------------------------- */
void
point(void)
{
 BYTE        bMsg;
 WORD        wPayload;
 DWORD       dwPayload;

 int         nDistance;
 int         nIsTriggered;

 // position servo head
 servoSetAngle(nPointDir);

 // wait for message
 bMsg = waitMsg(__ANY__, 0, &wPayload, &dwPayload);

 // parse message
 switch ( bMsg )
    {
     // change scan mode, redirect sensor or measure distance
     case SN_DIST_MODE:
          switch ( wPayload )
             {
              case SN_STOW:
                   distanceState = STATE_STOW;
                   break;

              case SN_POINT:
                   nPointDir = (int) dwPayload;
                   break;

              case SN_MEASURE:
                   nIsTriggered = gp2d02Trigger();

                   if ( nIsTriggered )
                   {
                      suspend(45);
                      nDistance = gp2d02GetDistance();
                      nDistance = gp2d02DataToDistance(nDistance);

                      // SN_MEASURE will never come from t_ctrlex only from t_nav
                      putMsg(nNavTask, SN_DISTANCE, (WORD) nDistance, (DWORD) nPointDir);
                   }
                   else
                   {
                      putMsg(nNavTask, SN_DISTANCE, (WORD) 0, (DWORD) nPointDir);
                   }
                   break;

              default:
                   putDebugMsg(DB_TRACE, DB_BAD_MSG, __LINE__); // @@ bad distance mode command in point()
             } // switch on wPayload
          break;

     // default handler
     default:
          putDebugMsg(DB_TRACE, DB_BAD_MSG, __LINE__); // @@ bad message in point()
    } // switch on bMsg
}

/* -----------------------------------------
   task function code
----------------------------------------- */

void
t_dist(void)
{
 static State_Type prevState;

 /* attach to nav task */
 nNavTask = getTidByName(TASK_NAME_NAV);
 if ( nNavTask == 0 )
    {
     putDebugMsg(DB_TRACE, DB_BAD_TNAME, __LINE__); // @@ bad task name for TASK_NAME_NAV
     return;
    }

 /* initialize state machine */
 distanceState = STATE_STOW;
 prevState    = ~STATE_STOW;

 /* start control */
 while (1)
    {
     //print("dist\r\n");
     
     if ( prevState != distanceState )
        {
         putDebugMsg(DB_TRACE, DB_INFO, (DWORD) distanceState); // @@ state change info
         prevState = distanceState;
        }

     switch ( distanceState )
     {
      case STATE_STOW:
           stow();
           break;

      case STATE_POINT:
           point();
           break;

      default:
           distanceState = STATE_STOW;
           putDebugMsg(DB_TRACE, DB_BAD_PARAM, __LINE__); // @@ variable 'distanceState' contains unrecognized state
     }
    } /* endless loop */
}
