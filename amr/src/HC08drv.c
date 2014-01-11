/* ***************************************************************************

  HC08DRV.C

  HC08 comm driver.

  Dec. 31 2011 - Created

*************************************************************************** */

#include <stdio.h>    // required for printf() debug

#include "ctrlio.h"
#include "v25.h"
#include "fxp.h"

#include "HC08drv.h"
#include "servo.h"
#include "gp2d02.h"
#include "sonar.h"

/* -----------------------------------------
   definitions
----------------------------------------- */


#define HC08_GETREV   1         // read revision and reset status
                                // two 8-bit values will follow to V25:
                                // revision code and last reset status/reason
#define HC08_SETSRV   2         // set servo position
                                // 8-bit servo position will follow from V25
#define HC08_GP2DTRG  3         // trigger GP2D sensor
                                // 0xff ok, 0x00 fail to V25 will follow
#define HC08_GP2DRD   4         // read distance from GP2D
                                // 8-bit GP2D distance data to V25 will follow
#define HC08_GETVOLT  5         // read voltage
                                // 8-bit battary voltage to V25 will follow
#define HC08_ECHO     0xaa      // special 'echo' command

#define mHC08_DATA    0x10      // HC08 data port pin mask
#define mHC08_CLK     0x20      // HC08 clock port pin mask
#define mSONAR_ON     0x40      // set SONAR ON mask

#define WAIT_CYCLES   6         // wait cycles in 500uSec increments for HC08 ready for Tx/Rx signal

#define CONST_IN2CM   0x004e    // Qmn constant to conver [in] to [cm] x 2.45 (0x004e = 2.4375)

/* -----------------------------------------
   HC08 communicaiton function prototypes
----------------------------------------- */

int hc08GetByte(BYTE*, WORD);
int hc08SendByte(BYTE);
int hc08SendCommand(BYTE);

/* -- these prototypes are defined in HC08drv.h
WORD hc08Init(void);
WORD hc08GetStatus(void);
int hc08Echo(void);
*/

/* -----------------------------------------
   globals
----------------------------------------- */

#define DATA_POINTS    33
#define DATA           1
#define DIST           0

static BYTE  bDistance[DATA_POINTS][2] = {{10, 229}, // conversion table for gp2d02 data to distance
                                          {12, 204}, // data pairs in table are: {<distance>; <data>}
                                          {14, 186},
                                          {16, 172},
                                          {18, 160},
                                          {20, 151},
                                          {22, 144},
                                          {24, 138},
                                          {26, 133},
                                          {28, 128},
                                          {30, 124},
                                          {32, 120},
                                          {34, 117},
                                          {36, 114},
                                          {38, 112},
                                          {40, 109},
                                          {42, 106},
                                          {44, 104},
                                          {46, 102},
                                          {48, 100},
                                          {50,  98},
                                          {52,  96},
                                          {54,  94},
                                          {56,  92},
                                          {58,  90},
                                          {60,  89},
                                          {62,  86},
                                          {64,  85},
                                          {66,  84},
                                          {70,  80},
                                          {75,  78},
                                          {80,  75},
                                          {120, 55}};

/* =========================================================
   SONAR module/detector
========================================================= */

/* ---------------------------------------------------------
   SonarOn()

   turn sonar Tx bit on
--------------------------------------------------------- */
void
SonarOn(void)
{
 struct SFR _far*  pSfr = MK_FP(0xf000, 0xff00);

 // set sonar Tx pin
 pSfr->port1 |= mSONAR_ON;
}

/* ---------------------------------------------------------
   SonatOff()

   turn sonat Tx bit off
--------------------------------------------------------- */
void
SonarOff(void)
{
 struct SFR _far*  pSfr = MK_FP(0xf000, 0xff00);

 // clear sonar Tx pin
 pSfr->port1 &= ~(mSONAR_ON);
}

/* ---------------------------------------------------------
   SonarGetRange()

   read SONAR range

   return:   '0'    - on failure
           non-zero - SONAR range in [cm] on success
--------------------------------------------------------- */
int
SonarGetRange(int nLeaveIRon)
{
 BYTE     bReturn;
 QmnFP_t  qRange = 0;
 
 // make sure SONAR is on
 SonarOn();

 bReturn = (BYTE) hc08SendCommand(HC08_GETVOLT);

 if ( bReturn )
 {
    hc08GetByte(&bReturn, 1);
 }

 if ( !nLeaveIRon )
 {
    SonarOff();
 }

 qRange = INT2Qmn((int) bReturn);
 qRange = MUL_Qmn(qRange, CONST_IN2CM);

 return (int) Qmn2INT(qRange);
}

/* =========================================================
   GP2D02 function implementation
========================================================= */

/* ---------------------------------------------------------
   gp2d02Trigger()

   This function will trigger the GP2D sensor through HC08.

   returns      0 = on error, trigger failed
             0xff = trigger ok
--------------------------------------------------------- */
int
gp2d02Trigger(void)

{
 BYTE     bReturn;

 bReturn = (BYTE) hc08SendCommand(HC08_GP2DTRG);

 if ( bReturn )
 {
  hc08GetByte(&bReturn, 1);
 }

 return (int) bReturn;
}

/* ---------------------------------------------------------
   gp2d02GetDistance()

   This function reads the GP2D02 distance data GP2D
   through HC08.
   This function blocks for the duration of the GP2D read
   time of 70mSec or more.

   returns      0 = on error
            non-0 = distance data
--------------------------------------------------------- */
int
gp2d02GetDistance(void)

{
 BYTE     bDistance;

 bDistance = (BYTE) hc08SendCommand(HC08_GP2DRD);

 if ( bDistance )
 {
     hc08GetByte(&bDistance, 100);
 }

 return (int) bDistance;
}

/* ---------------------------------------------------------
   gp2d02DataToDistance()

   This function converts the gp2d02 data byte to
   distance in [cm].
   Results in fixed-point Q10.5 format.
--------------------------------------------------------- */
int
gp2d02DataToDistance(int nData)
{
 int i;
 int nAlpha;
 int nDistDelta;
 int nDataDelta;

 if ( nData >= 229 )
    return INT2Qmn(10);

 for ( i = 1 ; i < DATA_POINTS; i++ )
    {
     // find closest match for sensor data in the table
     if ( bDistance[i][DATA] <= nData )
        {
         // calculate graph gradiant
         nDistDelta = INT2Qmn(bDistance[i][DIST] - bDistance[i - 1][DIST]);
         nDataDelta = (bDistance[i - 1][DATA] - bDistance[i][DATA]);
         nAlpha = nDistDelta / nDataDelta;

         // return interpolated distance point
         return ( INT2Qmn(bDistance[i][DIST]) - (nAlpha * (nData - bDistance[i][DATA])) );
        }
    }

 // data point not in table range
 return 0;
}

/* =========================================================
   Servo function implementation
========================================================= */

/* ---------------------------------------------------------
   servoSetCount()

   This function sets the pulse width count.
   count information in integer notation 1[LSB] = 0.6[uSec]

   Returns TRUE  = input param out of range
           FALSE = ok
--------------------------------------------------------- */
int
servoSetCount(BYTE bCount)
{
 if ( bCount < LIM_CW )
    bCount = LIM_CW;

 if ( bCount > LIM_CCW )
    bCount = LIM_CCW;

 if ( hc08SendCommand(HC08_SETSRV) && hc08SendByte(bCount))
    return TRUE;
 else
    return FALSE;
}

/* ---------------------------------------------------------
   servoSetAngle()

   This function converts angle [rad] to pulse width count.
   Angle information in 'nAngle' in fixed point 10_5 notation.

   Returns TREU  = input param out of range
           FALSE = ok
--------------------------------------------------------- */
int
servoSetAngle(QmnFP_t qAngle)
{
 WORD wCount;

 // scale range by 29.0 -> S000|0011|101.0|0000 -> 0x03a0
 // to get from [rad] to servo position number
 wCount = MID_POS - Qmn2INT(MUL_Qmn(0x03a0, qAngle));
 return servoSetCount((BYTE) wCount);
}

/* =========================================================
   HC08 communication function implementation
========================================================= */

/* ---------------------------------------------------------
   hc08SendCommand()

   sends command byte to HC08
   returns 'true' on success or 'false' on failure
--------------------------------------------------------- */
int
hc08SendCommand(BYTE bCommand)
{
 return hc08SendByte(bCommand);
}

/* ---------------------------------------------------------
   hc08Init()

   This function initialized port2 bits 5 and 4 for HC08
   comm interfacing.
   returns '0' on failure to test with assert()
--------------------------------------------------------- */
WORD
hc08Init()
{
 struct SFR _far*  pSfr = MK_FP(0xf000, 0xff00);

 // initialize I/O pins and variables
 pSfr->portmc1 = P1_MODE_CTRL;
 pSfr->port1   = P1_INIT; // port pin defaults take effect at in/out MODE set!
 pSfr->portm1  = P1_MODE;

 // probe HC08 for version and reset status, return '0' if not successful
 return hc08GetStatus();
}

/* ---------------------------------------------------------
   hc08GetByte()

   reads byte from HC08 comm interface
   returns 'true' on success or 'false' on dailure
--------------------------------------------------------- */
int
hc08GetByte(BYTE* pbData,
            WORD  wTOV)
{
 struct SFR _far*  pSfr      = MK_FP(0xf000, 0xff00);
 WORD              wWait     = 0;
 BYTE              bBitVal   = 1;
 int               nBitCount = 0;
 int               nReturn   = FALSE;

 *pbData = 0;

 if ( wTOV == 0 )
    wTOV = 1;

 wTOV *= 20;          // convert for a 0.5mSec loop-count for delay loop
 wTOV += WAIT_CYCLES; // wait at least WAIT_CYCLES delay cycles

 asm { cli }

 // test HC08 data line for '1' state, fail if not '1'
 if ( pSfr->port1 & mHC08_DATA )
 {
    pSfr->port1 &= ~(mHC08_CLK);

    // wait for HC08 line to go '0' signaling ready
    while ( (pSfr->port1 & mHC08_DATA) && (wWait < wTOV) )
    {
       DELAY_50uSEC;
       wWait++;
    }

    // if HC08 did not time-out then read byte from HC08
    if ( wWait < wTOV )
    {
       // clock and read a byte out of HC08
       // toggle V25 clock line to '1' so HC08 will set bit.0
       pSfr->port1 |= mHC08_CLK;

       // loop through 8 bit data Rx while toggling V25 clock pin:
       for (nBitCount = 0; nBitCount < 8; nBitCount++, bBitVal *= 2)
       {
          DELAY_25uSEC;                    // 1) delay

          if ( pSfr->port1 & mHC08_DATA )  // 2) read data bit in V25 data pin
          {
             *pbData += bBitVal;
          }
          pSfr->port1 ^= mHC08_CLK;        // 3) toggle clock pin
       }                                   // 4) loop to #1 for next bit
       nReturn = TRUE;
    }
 }

 // V25 clock and data lines to default '1'
 // will make sure that data bit output latch will be '1' when comming
 // out of input mode
 pSfr->port1 |= mHC08_CLK;
 pSfr->port1 |= mHC08_DATA;

 asm { sti }

 return nReturn;
}

/* ---------------------------------------------------------
   hc08SendByte()

   sends byte to HC08 interface.
   returns 'true' on success or 'false' on dailure
--------------------------------------------------------- */
int
hc08SendByte(BYTE bData)
{
 struct SFR _far*  pSfr      = MK_FP(0xf000, 0xff00);
 WORD              wWait     = 0;
 BYTE              bBitVal   = 1;
 int               nBitCount = 0;
 int               nReturn   = FALSE;

 asm { cli }

 // test HC08 data line for '1' state, fail if not '1'
 if ( pSfr->port1 & mHC08_DATA )
 {
    // signal Tx start by pulling V25 clock line to '0'
    pSfr->port1 &= ~(mHC08_CLK);

    // wait for HC08 line to go '0' signaling ready
    while ( (pSfr->port1 & mHC08_DATA) && (wWait < WAIT_CYCLES) )
    {
       DELAY_50uSEC;
       wWait++;
    }

    // if HC08 did not time-out then read byte from HC08
    if ( wWait < WAIT_CYCLES )
    {
       // toggle V25 clock line to '1' to signal swap of data pin directions
       pSfr->port1 |= mHC08_CLK;

       // change V25 data direction to 'out'
       //pSfr->portm1 &= ~(mHC08_DATA);
       pSfr->portm1 = P1_MODE & ~(mHC08_DATA);

       // delay while HC08 changes data direction to 'in'
       DELAY_25uSEC;

       // loop through 8 bit data Tx while toggling V25 clock pin:
       for (nBitCount = 0; nBitCount < 8; nBitCount++, bBitVal *= 2)
       {
           if ( bBitVal & bData )            // 1) setup data bit in V25 data pin
           {
               pSfr->port1 |= mHC08_DATA;
           }
           else
           {
               pSfr->port1 &= ~(mHC08_DATA);
           }
           pSfr->port1 ^= mHC08_CLK;         // 2) toggle clock pin
           DELAY_25uSEC;                     // 3) delay 50uSec min.
       }                                     // 4) loop to #1 for next bit
       nReturn = TRUE;
    }
 }

 // V25 clock line to default '1'
 pSfr->port1 |= mHC08_CLK;

 // change V25 data direction back to 'in', HC08 will change back to 'out'
 // make sure that data bit output latch will be '1' next time it
 // goes to output mode
 pSfr->port1 |= mHC08_DATA;
 //pSfr->portm1 |= mHC08_DATA;
 pSfr->portm1 = P1_MODE | mHC08_DATA;

 asm { sti }

 return nReturn;
}

/* ---------------------------------------------------------
   hc08GetStatus()

   get status from HC08, return in WORD format to test with assert():
    Hi byte is version number
    Lo byte is last Reset status on HC08

    will return 0x0000 if failed
--------------------------------------------------------- */
WORD
hc08GetStatus()
{
 WORD wReturnValue = 0;
 BYTE bVersion     = 0;
 BYTE bRstStatus   = 0;

 if ( hc08SendCommand(HC08_GETREV) )  // send command
 {
  hc08GetByte(&bVersion, 1);          // get version byte
  hc08GetByte(&bRstStatus, 1);        // get HC08 reset status byte

  if ( bVersion == VERSION )
  {
   wReturnValue  = bVersion * 256;   // move version to Hi byte
   wReturnValue += bRstStatus;
  }
 }
 
 return wReturnValue;
}

/* ---------------------------------------------------------
   hc08Echo()

   sends an echo byte to HC08 special command and expects
   it to beechoed
   returns 'true' on success or 'false' on failure
--------------------------------------------------------- */
int
hc08Echo()
{
 BYTE bReturn;

 hc08SendByte(HC08_ECHO);
 hc08GetByte(&bReturn, 1);

 if ( bReturn == HC08_ECHO )
    return TRUE;
 else
    return FALSE;
}

