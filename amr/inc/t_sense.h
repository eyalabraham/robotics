/* ***************************************************************************

  T_SENSE.H

  Bumper switch and shaft encoder control task header

  December 10 1999 - Created

*************************************************************************** */

#ifndef __T_SENSE__
#define __T_SENSE__

#include  <dos.h>

int
auxCommInit(void);

void
t_sense(void);     /* task prototype                   */

#endif
