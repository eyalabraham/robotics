/* ***************************************************************************

  gp2d02.h

  GP2D02 driver header file.

  March 26 2002 - Created
  Jan   2  2012 - modified to use HC08 and driver

*************************************************************************** */

#ifndef __GP2D02_H__
#define __GP2D02_H__

#include "smte.h"

/* -----------------------------------------
   function prototypes
----------------------------------------- */

/* ---------------------------------------------------------
   gp2d02Trigger()

   This function will trigger the GP2D sensor through HC08.

   returns      0 = on error, trigger failed
             0xff = trigger ok
--------------------------------------------------------- */
int
gp2d02Trigger(void);

/* ---------------------------------------------------------
   This function reads the GP2D02 distance data from
   device serial interface.
   This function blocks for the duration of the GP2D read
   time of 45mSec or more.

   returns      0 = on error
            non-0 = distance data
--------------------------------------------------------- */
int
gp2d02GetDistance(void);

/* ---------------------------------------------------------
   This function converts the gp2d02 data byte to
   distance in [cm].
   Distance results in fixed-point 12_4 format.
--------------------------------------------------------- */
int
gp2d02DataToDistance(int);

#endif  /* __GP2D02_H__ */
