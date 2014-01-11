/* ***************************************************************************

  servo.h

  Servo driver header file.

  March 16 2002 - Created

*************************************************************************** */

#ifndef __SERVO_H__
#define __SERVO_H__

#include "smte.h"
#include "fxp.h"

#define  LIM_CCW         125    // max. CCW position count [+90 deg]
#define  LIM_CW          35     // max. CW position count  [-90 deg]
#define  MID_POS         80     // center position count   [  0 deg]

#define  LIM_LEFT_FP     0xffce // max. CCW   -1.5625  [rad] in Q10.5
#define  LIM_RIGHT_FP    0x0032 // max. CW    +1.5625  [rad] in Q10.5
#define  CENTER_FP       0x0000 // center position 0   [rad]

/* -----------------------------------------
   function prototypes
----------------------------------------- */

/* ---------------------------------------------------------
   Set pulse width count to 'bCount',
   count information in integer notation 1-LSB = 0.6[uSec]

   Returns FALSE = input param out of range
           TRUE  = ok
--------------------------------------------------------- */
int
servoSetCount(BYTE);

/* ---------------------------------------------------------
   convert angle [rad] to pulse width count
   angle in fixed point notation +/- [rad] from
   forward pointing sensor (forward point = 0 [rad])

   Returns FALSE = input param out of range
           TRUE  = ok
--------------------------------------------------------- */
int
servoSetAngle(QmnFP_t);

#endif  /* __SERVO_H__ */
