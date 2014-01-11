/* ***************************************************************************

  T_DSPLY.C

  Seven segment display control task source code.

  August 9 2001 - Created

*************************************************************************** */

#include  <conio.h>

#include  "smte.h"
#include  "ctrlio.h"
#include  "t_dsply.h"

/* -----------------------------------------
   globals
----------------------------------------- */

/* -----------------------------------------
   external variables
----------------------------------------- */

extern BYTE bUtil;              /* defined in amr.c */

/* -----------------------------------------
   task function code
   7-segment display is set acording to the
   following display command message format:

   display data and attribute
   --------------------------
   d0 - d3: display code
   d4     : reserved ( =1 )
   d5     : reserved ( =1 )
   d6     : 1 = high blink rate
            0 = low blink rete
   d7     : 1 = blink enable
            0 = no blink

----------------------------------------- */

void
t_dsply(void)
{
 BYTE     bMsg;
 WORD     wPayload;
 DWORD    dwPayload;

 WORD     wBlinkDelay = 0; /* initialize as no blink processing */
 BYTE     bDisplay    = 0xff;

 while (1)
    {
     //print("display\r\n");

     /* wait for message */
     bMsg = (BYTE) waitMsg(__ANY__, wBlinkDelay, &wPayload, &dwPayload);

     /* parse message */
     switch ( bMsg )
        {
         case Q_EMPTY:
              CRIT_SEC_START;

              bUtil ^= MASK_DISP_BLK;
              outp(UTIL, bUtil);

              CRIT_SEC_END;
              break;
         default:
              if ( bDisplay != bMsg )
                 {
                  bDisplay = bMsg;
                  if ( bDisplay & BLINK_ENA )
                     { /* blinking character     */
                      if ( bDisplay & BLINK_HIGH_ENA )
                         wBlinkDelay = BLINK_HIGH_RATE;
                      else
                         wBlinkDelay = BLINK_LOW_RATE;
                     }
                  else /* non-blinking character */
                     {
                      wBlinkDelay = 0;
                     }

                  CRIT_SEC_START;

                  bUtil &= 0xf0;               /* clear old character */
                  bUtil |= (bDisplay & 0x0f);  /* set new character   */
                  bUtil |= MASK_DISP_BLK;      /* enable charachter   */
                  outp(UTIL, bUtil);
                  
                  CRIT_SEC_END;
                 }
        } /*switch on bMsg */
    } /* endless loop */
}


