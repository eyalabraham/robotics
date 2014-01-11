/* ***************************************************************************

  HC08drv.H

  HC08 comm driver

  Dec. 31 2011 - created

*************************************************************************** */

#ifndef __HC08DRV_H__
#define __HC08DRV_H__

#include "smte.h"

/* ---------------------------------------------------------
   definitions
--------------------------------------------------------- */

#define VERSION       0x0013    // HC08 firmware version, see 'srvgp2d.asm'

#define HC08_COMMFAIL 0

/* ---------------------------------------------------------
   hc08Init()

   This function initialized port2 bits 5 and 4 for HC08
   comm interfacing.
   returns '0' on failure to test with assert()
--------------------------------------------------------- */
WORD hc08Init(void);

/* ---------------------------------------------------------
   hc08GetStatus()

   get status from HC08, return in WORD format to test with assert():
    Hi byte is version number
    Lo byte is last Reset status on HC08
--------------------------------------------------------- */
WORD hc08GetStatus(void);

/* ---------------------------------------------------------
   hc08Echo()

   sends an echo byte to HC08 special command and expects
   it to beechoed
   returns 'true' on success or 'false' on failure
--------------------------------------------------------- */
int hc08Echo(void);

#endif /* __HC08DRV_H__ */

