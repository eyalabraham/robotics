/* ***************************************************************************

  T_WDOG.C

  Watch dog task source code.

  August 6 1999 - Created

*************************************************************************** */

#include  <conio.h>

#include  "smte.h"
#include  "ctrlio.h"
#include  "t_wdog.h"

/* -----------------------------------------
   globals
----------------------------------------- */

extern BYTE bUtil;  /* defined in amr.c */

/* -----------------------------------------
   t_wdog()

   This task is executed periodically,
   its function is to trigger the watchdog one-shot timer.
----------------------------------------- */

void
t_wdog(void)
{
 /* pulse watchdog trigger line */

 //print("w-dog\r\n");

 CRIT_SEC_START;

 bUtil |= MASK_WDOG;
 outp(UTIL, bUtil);
 bUtil &= (~MASK_WDOG);
 outp(UTIL, bUtil);

 CRIT_SEC_END;
}


