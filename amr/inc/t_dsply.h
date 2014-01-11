/* ***************************************************************************

  T_DSPLY.H

  Seven segment display control task header file.

  August 9 2001 - Created

*************************************************************************** */

#ifndef  __T_DSPLY__
#define  __T_DSPLY__

/* -----------------------------------------
   definiitons
----------------------------------------- */

#define   BLINK_HIGH_RATE  200   /* 4 Hz rate               */
#define   BLINK_LOW_RATE   1000  /* 1 Hz rate             */

#define   BLINK_ENA        0x80  /* blink rate control bits */
#define   BLINK_HIGH_ENA   0x40

/* -----------------------------------------
   display code definitions
----------------------------------------- */

#define SIGNAL(bCharacter) putMsg(bDisplayTask, (bCharacter | 0x30), 0, 0L)

#define SIG_LINEAR             0
#define SIG_TURN               1
#define SIG_ABORT              2

#define SIG_ENCODER_ERROR      0 + BLINK_ENA + BLINK_HIGH_ENA
#define SIG_LEFT_DRIVE_ERROR   1 + BLINK_ENA + BLINK_HIGH_ENA
#define SIG_RIGHT_DRIVE_ERROR  2 + BLINK_ENA + BLINK_HIGH_ENA
#define SIG_BAD_CMD            3 + BLINK_ENA + BLINK_HIGH_ENA

#define SIG_BLANK             15

/* -----------------------------------------
   task prototype
----------------------------------------- */

void
t_dsply(void);

#endif