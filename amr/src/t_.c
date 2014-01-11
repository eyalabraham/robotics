/* ***************************************************************************

  T_<task>.C

  Task source code.

  xx xx 20xx - Created

*************************************************************************** */

#include  "smte.h"
#include  "t_<task>.h"

/* -----------------------------------------
   task function code
----------------------------------------- */

void
t_taskname(void)
{
 int    nMsg;
 WORD   wPayload;
 DWORD  dwPayload;

 // start control
 while (1) // <-- can remove so that task function 'falls through'
    {
     // wait for message
     nMsg = waitMsg(__ANY__, 0, &wPayload, &dwPayload);

     // parse message
     switch ( nMsg )
        {
         case <msg type>:
              break;
         default:
              putDebugMsg(DB_ERROR, DB_BAD_MSG, 0L);
        } // switch on bMsg
    } // endless loop
}