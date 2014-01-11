/* ***************************************************************************

  STARTUP.C

  Startup code to initialize hardware.
  Executable should be included after boot in AUTOEXEC.BAT

  March 5 2000 - Created

*************************************************************************** */

#include <stdlib.h>
#include <stdio.h>
#include <conio.h>
#include <dos.h>

#include "v25.h"
#include "ctrlio.h"

#define  CON_COMM_CTRL         2     // base rate Fclk/512
#define  CON_COMM_BAUD_19200   65    //

/* -----------------------------------------
   main module initialization code
----------------------------------------- */

void
main(void)
{
 int              nRunFlag;
 int              nBaudRateFlag;
 struct SFR _far* pSfr = MK_FP(0xf000, 0xff00);

 // get baud rate selection from SRR register
 nBaudRateFlag = inp(SRR) & ENA_19200_BAUD;
 if ( nBaudRateFlag )
    {
     // change BAUD rate to 19200
     pSfr->scc0  = CON_COMM_CTRL;
     pSfr->brg0  = CON_COMM_BAUD_19200;
     printf("\n\n@@@@\n\n");
     printf("INFO: BAUD Rate = 19200 BAUD.\n");
    }
 else
    {
     printf("INFO: BAUD Rate = 9600 BAUD.\n");
    }

 // build date
 printf("BUILD: startup.exe %s %s\n", __DATE__, __TIME__);

 // force motors to off state and
 // blank 7-deg display
 outp(UTIL, 0x00);
 outp(MOTOR_CTRL, 0x77);
 printf("INFO: Motor controls = OFF.\n");

 // port-1 setup
 pSfr->portmc1  = P1_MODE_CTRL;                // IO port-1 setup
 pSfr->portm1   = P1_MODE;
 pSfr->port1    = P1_INIT;

 // port-2 setup
 pSfr->portmc2  = P2_MODE_CTRL;                // IO port-2 setup for wheel encoder
 pSfr->portm2   = P2_MODE;
 pSfr->port2    = P2_INIT;

 // display run flag bit state (SRR register)
 nRunFlag = inp(SRR) & AUTO_RUN;
 if ( nRunFlag )
    {
     printf("INFO: Run flag = ON.\n");
    }
 else
    {
     printf("INFO: Run flag = OFF.\n");
    }

 // return run flag state
 exit(nRunFlag ? 1 : 0);
}
