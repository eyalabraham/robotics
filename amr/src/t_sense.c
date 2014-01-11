/* ***************************************************************************

  T_SENSE.C

  Bumper switch and shaft encoder control task.

  April    6  2012 - updated switch and wheel handling
  December 10 1999 - Created

*************************************************************************** */

#include <stdlib.h>
#include <mem.h>
#include <dos.h>

#include "v25.h"
#include "smte.h"
#include "ctrlio.h"
#include "messages.h"
#include "names.h"
#include "t_sense.h"

#pragma intrinsic memcpy        /* compile memcpy() as inline               */

/* -----------------------------------------
   module definitions
----------------------------------------- */

#define  AUX_COMM_VEC     17    // serial recieve interrupt
#define  AUX_COMM_MODE    0x69  // 1-stop, 8-data, P-odd, Rx-ena, Tx-dis
#define  AUX_COMM_CTRL    0x03  // base rate Fclk/16
#define  AUX_COMM_BAUD    50    // 12500 baud
#define  AUX_COMM_RX_INT  0x20  // enable and set to macro service
#define  AUX_COMM_RX_MAC  0x10  // Rx macro ctrl 8-bit, SFR to memory, ch 0
#define  AUX_COMM_DIS     0xbf  // Disable Rx, and Tx on aux. comm port
#define  AUX_COMM_ENA     0x40  // Enable Rx, disable Tx on aux. comm port

#define  SFR_REGISTER     0x70  // Rx register

#define  MAX_SENSE_BLOCKS 50    // see calculations below

#define  PS2_ACK_TOV      0xffff

#define  SW_RIGHT         0x01
#define  SW_LEFT          0x02
#define  SW_MID           0x04  // never pressed

/* -----------------------------------------
   private type definitions
----------------------------------------- */

typedef struct senseBlock_tag
        {
         BYTE          bSwitches;
         signed char   bLeftWheel;
         signed char   bRightWheel;
         DWORD         dwTicks;
        } senseBlock;

/* -----------------------------------------
   task globals
----------------------------------------- */

/* NOTE: index 0 in the buffer is for temp use,
         indexes should circulate between 1 and MAX_SENSE_BLOCKS */

volatile struct senseBlock_tag senseBlockBuffer[MAX_SENSE_BLOCKS]; /* circular buffer */
volatile int                   nBuffIn   = 1;                      /* input index     */
volatile int                   nBuffOut  = 1;                      /* output index    */
volatile int                   nCount    = 0;                      /* sense blocks    */

int                            nCtrlTask = 0;

/* -----------------------------------------
   external variables
----------------------------------------- */

extern int nSenseSampleInterval;

/* -----------------------------------------
   hardware & system initialization function
----------------------------------------- */

/* -----------------------------------------
   ps2_write()

   Write byte to PS2 device.
----------------------------------------- */
extern int
_near ps2_write(unsigned char bPS2data);

/* -----------------------------------------
   auxCommIsr()

   Aux. communication port ISR
----------------------------------------- */
void interrupt
auxCommIsr(void)
{
 /* copy sensor data, switches and wheel encoders */
 memcpy((void*) &senseBlockBuffer[nBuffIn],
        (void*) &senseBlockBuffer[0],
        (sizeof(struct senseBlock_tag) - sizeof(DWORD)));

 /* add current clock tick value */
 senseBlockBuffer[nBuffIn].dwTicks = getGlobalTicks();

 /* circulate buffer */
 nBuffIn++;
 if ( nBuffIn == MAX_SENSE_BLOCKS )
    nBuffIn = 1;
 nCount++;

 /* re-initialize macro registers */
 ((struct macroChannel_tag _far*) MK_FP(0xf000, 0xfe00))->bMSC  = (sizeof(struct senseBlock_tag) - sizeof(DWORD));
 ((struct macroChannel_tag _far*) MK_FP(0xf000, 0xfe00))->wMSP  = FP_OFF(&senseBlockBuffer[0]);

 /* reselect macro mode */
 ((struct SFR _far*) MK_FP(0xf000, 0xff00))->sric1 |= AUX_COMM_RX_INT;

 /* ISR epilog */
 asm { db  0x0f   /* the function will only be hooked if __V25__ is defined */
       db  0x92   /* so this is ok.                                         */
     }
}

/* -----------------------------------------
   auxCommInit()

   Aux. communication port setup:
   12500 baud, 8 bit, 1 stop, ODD parity.

   Interuupt vector set to point to ISR, and
   operate in macro service mode.
   Macro service executes to get 3 Rx bytes
   and ISR transfers 3 byte block to circuar
   buffer common to 't_sense' task.

   returns '-1' if init Ok, '0' if failed.
----------------------------------------- */
#pragma argsused

int
auxCommInit(void)
{
 #ifdef __V25__

 BYTE                          bPS2response;
 WORD _far*                    wpVector;
 struct SFR _far*              pSfr;
 struct macroChannel_tag _far* pMacroChannel;

 pSfr = MK_FP(0xf000, 0xff00);

 // IO port-2 setup for wheel encoder
 pSfr->portmc2  = P2_MODE_CTRL;
 pSfr->portm2   = P2_MODE;
 pSfr->port2    = P2_INIT;

 /* aux comm port setup                                           */
 pSfr->scm1  = AUX_COMM_MODE;
 pSfr->scc1  = AUX_COMM_CTRL;
 pSfr->brg1  = AUX_COMM_BAUD;

 /* setup ps/2 device (wheel encoder & bumper switches)           */
 pSfr->scm1    &= AUX_COMM_DIS; /* disable serial port 1 Rx       */
 ps2_write(0xea);               /* set PS2 device to stream mode  */
 pSfr->scm1    |= AUX_COMM_ENA; /* enable serial port 1 Rx        */

 while ( (pSfr->scs1 & 0x10) == 0 ) {};

 bPS2response = pSfr->rxb1;
 if ( bPS2response != 0xfa )
    {
     return 0;
    }

 pSfr->scm1    &= AUX_COMM_DIS; /* disable serial port 1 Rx       */
 ps2_write(0xf4);               /* enable PS2 device              */
 pSfr->scm1    |= AUX_COMM_ENA; /* enable serial port 1 Rx        */

 while ( (pSfr->scs1 & 0x10) == 0 ) {};

 bPS2response = pSfr->rxb1;
 if ( bPS2response != 0xfa )
    {
     return 0;
    }

 /* setup aux comm. interrupt vector */

 asm { cli }

 wpVector      = MK_FP(0, (AUX_COMM_VEC * 4));
 *wpVector++   = FP_OFF(auxCommIsr);
 *wpVector     = FP_SEG(auxCommIsr);

 /* aux comm port macro service setup */

 pSfr->srms1 = AUX_COMM_RX_MAC;
 pSfr->sric1 = (pSfr->sric1 & 0x8f) | AUX_COMM_RX_INT;

 /* macro channel setup */

 pMacroChannel = MK_FP(0xf000, 0xfe00);

 pMacroChannel->bSFRP = SFR_REGISTER;
 pMacroChannel->bMSC  = sizeof(struct senseBlock_tag) - sizeof(DWORD);
 pMacroChannel->bSCHR = 0;
 pMacroChannel->wMSP  = FP_OFF(&senseBlockBuffer[0]);
 pMacroChannel->wMSS  = _DS;

 asm { sti }
 
 #endif /* __V25__ */

 return -1;
}

/* -----------------------------------------
   timing calculations

   at 12500 BAUD, 11 bits per byte and 3 bytes per
   data packet we get a max of 2.64 [mSec] per data
   packet from the mouse.

   at a transmission interval of 100mSec
   about 40 data packets can be sent by the mouse.
   MAX_SENSE_BLOCKS is set to 50.

----------------------------------------- */

/* -----------------------------------------
   task function code
----------------------------------------- */

void
t_sense(void)
{
 register BYTE bSwStateChange;
 register BYTE bCurrSwitchState;
 static   BYTE bPrevSwitchState = 0x00;
 
 static int    nCanRun          = 1;
 static int    nDataToSend      = 0;
 static int    nRightClicks     = 0;
 static int    nLeftClicks      = 0;
 static WORD   wDataTxRate      = 0;
 static DWORD  dwTo             = 0;
 static WORD   wMeasureInterval;
 static DWORD  dwTxEventTime;
 static DWORD  dwTx;

 DWORD         dwTemp;

 //print("sense\r\n");

 // on first entry into the task assign t_ctrl task ID and
 // initialize other variables
 if ( (nCtrlTask == 0) && (nCanRun == 1) )
 {
     nCtrlTask = getTidByName(TASK_NAME_CONTROL);
     if ( nCtrlTask == 0 )
     {
        putDebugMsg(DB_TRACE, DB_BAD_TNAME, __LINE__); // @@ bad task name for TASK_NAME_CONTROL
        nCanRun = 0;
     }

     wDataTxRate   = MSEC2TICKS(nSenseSampleInterval); // desired Tx interval
     dwTo          = getGlobalTicks();                 // current time
     dwTxEventTime = dwTo + wDataTxRate;               // future Tx time
 }

 if ( nCanRun == 1 )
 {
    // process sensor datd packets
    while ( (nCount > 0) || nDataToSend ) // <-- consider/test performance 'while' vs. 'if'
    {
        // test for buffer overrun
       if ( nCount >= MAX_SENSE_BLOCKS )
       {
          putMsg(nCtrlTask, SN_ERROR, SN_ENCODER_OVR, (DWORD) nCount);
          nBuffIn     = 1;
          nBuffOut    = 1;
          nCount      = 0;
          nDataToSend = 0;
       }
    
       // process bumper switches and whell clicks
       if ( nCount > 0 )
       {
          /* process wheel encoder data starting with the micro switches
             check for change from previouse packet and send message as applicable

             0 sw released
             1 sw pressed

            <prev> xor <current>  <ischange>
               0          0         0
               0          1         1
               1          0         1
               1          1         0

            (a) <current> & 0x07; to mask out bits other than switch bits
            (b) <prev> xor <current> = <ischange>
            (c) if <ischange> then process switch bits
                (i)  if ( <current> and <sw_mask> ) switch pressed 'ON'
                (ii) else switch released 'OFF'
            (d) save current states for next packet
          */

          bCurrSwitchState  = senseBlockBuffer[nBuffOut].bSwitches & 0x07; // isolate switch state bits
          bSwStateChange    = bPrevSwitchState ^ bCurrSwitchState;         // detect changes from last state
          bSwStateChange   &= bCurrSwitchState;                            // prevent/mask processing switch release events
          bPrevSwitchState  = bCurrSwitchState;                            // store current state for next round
          
          if ( bSwStateChange & SW_LEFT )
             putMsg(nCtrlTask, SN_BUMPER_SW, SN_LEFT_ON, DW_DONT_CARE);
             
          if ( bSwStateChange & SW_RIGHT )
             putMsg(nCtrlTask, SN_BUMPER_SW, SN_RIGHT_ON, DW_DONT_CARE);
          
          /* disable mid switch processing 
          if ( bSwStateChange & SW_MID )
             putMsg(nCtrlTask, SN_BUMPER_SW, SN_MID_ON, DW_DONT_CARE);
          */
                   
          // accumulate distance data and save time-tick stamp
          nRightClicks += (signed char) senseBlockBuffer[nBuffOut].bRightWheel;
          nLeftClicks  += (signed char) senseBlockBuffer[nBuffOut].bLeftWheel;
          dwTx          = senseBlockBuffer[nBuffOut].dwTicks;

          // circular buffer index update
          nBuffOut++;
          if ( nBuffOut == MAX_SENSE_BLOCKS )
             nBuffOut = 1;
          nCount--;

          nDataToSend   = 1;
       } // end process mouse input packets on nCount > 0

       // send distance and speed data if interval time has elapsed
       if ( dwTxEventTime <= getGlobalTicks() )
       {
          //  pack and send distance data
          PACK_DW(dwTemp, nRightClicks, nLeftClicks);
          wMeasureInterval = (WORD) (dwTx - dwTo); // should be ok ...

          putMsg(nCtrlTask, SN_CLICKS, wMeasureInterval, dwTemp);

          //  re-initialize variables for next period
          nDataToSend     = 0;
          nRightClicks    = 0;
          nLeftClicks     = 0;
          dwTo            = dwTx;
          dwTxEventTime   = getGlobalTicks() + wDataTxRate;
       } // end send distance and speed data
    } // process while/if nCount > 0 or nDataToSend
 } // if ( nCanRun == 1 )
}
