/* ***************************************************************************

  MOUSE.C

  Bumper switch and shaft encoder test.

  January 13 2000 - Created

*************************************************************************** */

#include <stdio.h>
#include <mem.h>
#include <dos.h>

#include "v25.h"
#include "ctrlio.h"

#pragma intrinsic memcpy        /* compile memcpy() as inline               */

/* -----------------------------------------
   private module definitions
----------------------------------------- */

#define  AUX_COMM_VEC     17    /* serial recieve interrupt                 */
#define  AUX_COMM_MODE    0x69  /* 1-stop, 8-data, P-odd, Rx-ena, Tx-dis    */
#define  AUX_COMM_CTRL    0x03  /* base rate Fclk/16                        */
#define  AUX_COMM_BAUD    50    /* 12500 baud                               */
#define  AUX_COMM_RX_INT  0x20  /* enable and set to macro service          */
#define  AUX_COMM_RX_MAC  0x10  /* Rx macro ctrl 8-bit, SFR to memory, ch 0 */
#define  AUX_COMM_DIS     0x3f  /* Disable Rx, and Tx on aux. comm port     */
#define  AUX_COMM_ENA     0x40  /* Enable Rx, disable Tx on aux. comm port  */

#define  SFR_REGISTER     0x70  /* Rx register                              */

//#define  P2_MODE          0xfc  /* port 2 mode                              */
//#define  P2_CTRL          0xfc  /* port 2 control                           */
//#define  P2_INIT          0x00  /* port 2 initial bit values                */

#define  MAX_SENSE_BLOCKS 41

#define  SW_LEFT          0x01
#define  SW_RIGHT         0x02
#define  SW_MID           0x04  /* never pressed                            */

typedef unsigned char  BYTE;
typedef unsigned int   WORD;
typedef unsigned long  DWORD;

/* -----------------------------------------
   private type definitions
----------------------------------------- */

typedef struct senseBlock_tag
        {
         BYTE   bSwitches;
         BYTE   bDx;
         BYTE   bDy;
         DWORD  dwTimeTag;
        } senseBlock;

/* -----------------------------------------
   static variables
----------------------------------------- */

/* NOTE: index 0 in the buffer is for temp use,
         indexes should circulate between 1 and 4 */

volatile struct senseBlock_tag senseBlockBuffer[MAX_SENSE_BLOCKS]; /* circular buffer */
volatile BYTE                  bBuffIn  = 1;                       /* input index     */
volatile BYTE                  bBuffOut = 1;                       /* output index    */
volatile BYTE                  bCount   = 0;                       /* sense blocks    */

BYTE                           bNavTask = 0;
BYTE                           bPrevSwitchState = 0x00;

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
   auxCommDis()

   Aux. communication port disable Rx and Tx.
----------------------------------------- */
void
auxCommDis(void)
{
  ((struct SFR _far*) MK_FP(0xf000, 0xff00))->scm1 &= AUX_COMM_DIS;
}

/* -----------------------------------------
   auxCommEna()

   Aux. communication port enable Rx.
----------------------------------------- */
void
auxCommEna(void)
{
 ((struct SFR _far*) MK_FP(0xf000, 0xff00))->scm1 |= AUX_COMM_ENA;
}

/* -----------------------------------------
   configPS2()

   configures ps2 interface.
----------------------------------------- */
int
configPS2(void)
{
 struct SFR _far* pSfr;

 pSfr = MK_FP(0xf000, 0xff00);
 
 printf("configPS2(): ps2 initialization.\n");
 
 auxCommDis();

 /* port-2 setup */

 pSfr->portmc2  = P2_MODE_CTRL;                // IO port-2 setup for wheel encoder
 pSfr->portm2   = P2_MODE;
 pSfr->port2    = P2_INIT;

 /* set PS2 device to stream mode  */

 ps2_write(0xea);

 /* enable PS2 device */

 ps2_write(0xf4);

 auxCommEna();

 return 0;
}

/* -----------------------------------------
   auxCommIsr()

   Aux. communication port ISR routine.
----------------------------------------- */
void interrupt
auxCommIsr(void)
{
 /* copy sensor data, switched and wheel encoders */
 memcpy((void*) &senseBlockBuffer[bBuffIn],
        (void*) &senseBlockBuffer[0],
        sizeof(struct senseBlock_tag) - sizeof(DWORD));

 /* get time tag from global ticks counter */
 senseBlockBuffer[bBuffIn].dwTimeTag = 0xdeadbeefL;

 /* circulate buffer */
 bBuffIn++;
 if ( bBuffIn == MAX_SENSE_BLOCKS )
    bBuffIn = 1;
 bCount++;

 /* re-initialize macro mode */
 ((struct macroChannel_tag _far*) MK_FP(0xf000, 0xfe00))->bMSC  = sizeof(struct senseBlock_tag) - sizeof(DWORD);
 ((struct macroChannel_tag _far*) MK_FP(0xf000, 0xfe00))->wMSP  = FP_OFF(&senseBlockBuffer[0]);

 /* reselect macro mode */
 ((struct SFR _far*) MK_FP(0xf000, 0xff00))->sric1 |= AUX_COMM_RX_INT;

 /* ISR epilog */
 asm { db  0x0f   /* the function will only be hooked if __V25__ is defined */
       db  0x92   /* so ths is ok.                                          */
     }
}

/* -----------------------------------------
   auxCommInit()

   Aux. communication port setup:
   1200 baud, 8 bit, 1 stop, no parity.

   Interuupt vector set to point to ISR, and
   operate in macro service mode.
   Macro service executes to get 5 Rx bytes
   and ISR transfers 5 byte block to circuar
   buffer common to 't_sense' task.
----------------------------------------- */
#pragma argsused

void
auxCommInit(void interrupt fpIsrAdd ())
{
 #ifdef __V25__

 WORD _far*                    wpVector;
 struct SFR _far*              pSfr;
 struct macroChannel_tag _far* pMacroChannel;

 asm { cli }

 /* setup aux comm. interrupt vector */

 printf("auxCommInit(): setup aux comm. interrupt vector.\n");

 wpVector      = MK_FP(0, (AUX_COMM_VEC * 4));
 *wpVector++   = FP_OFF(fpIsrAdd);
 *wpVector     = FP_SEG(fpIsrAdd);

 configPS2();

 /* aux comm port setup */

 printf("auxCommInit(): aux comm port setup.\n");

 pSfr = MK_FP(0xf000, 0xff00);

 pSfr->scm1  = AUX_COMM_MODE;
 pSfr->scc1  = AUX_COMM_CTRL;
 pSfr->brg1  = AUX_COMM_BAUD;

 /* enable aux comm macro service */

 printf("auxCommInit(): enable aux comm macro service.\n");

 pSfr->srms1 = AUX_COMM_RX_MAC;
 pSfr->sric1 = (pSfr->sric1 & 0x8f) | AUX_COMM_RX_INT;

  /* macro channel setup */

 printf("auxCommInit(): aux comm port macro service setup.\n");

 pMacroChannel = MK_FP(0xf000, 0xfe00);

 pMacroChannel->bSFRP = SFR_REGISTER;
 pMacroChannel->bMSC  = sizeof(struct senseBlock_tag) - sizeof(DWORD);
 pMacroChannel->bSCHR = 0;
 pMacroChannel->wMSP  = FP_OFF(&senseBlockBuffer[0]);
 pMacroChannel->wMSS  = _DS;

 asm { sti }

 #endif /* __V25__ */
}

/* -----------------------------------------
   task function code
----------------------------------------- */

void
t_sense(void)
{
 register BYTE   bSwitchState;
 register BYTE   bTempSwitches;

 while ( 1 )
    {

 while ( bCount > 0 )
    {
     /* test buffer overrun */
     if ( bCount >= MAX_SENSE_BLOCKS )
        printf("S_DATA_OVR\n");

     printf("S=%x  X=%x  Y=%x\n", senseBlockBuffer[bBuffOut].bSwitches,
                                  senseBlockBuffer[bBuffOut].bDx,
                                  senseBlockBuffer[bBuffOut].bDy);

     /* distance data */
     printf("S_CLICKS 0x%x, 0x%x\n", senseBlockBuffer[bBuffOut].bDx, senseBlockBuffer[bBuffOut].bDy);
     printf("S_TIME_TAG 0x%lx\n", senseBlockBuffer[bBuffOut].dwTimeTag);

     /* micro switches

     0 sw up
     1 sw down

     <prev> xor <current>  <ischange>
        0          0         0
        0          1         1
        1          0         1
        1          1         0

     (a)  <prev> xor <current> = <ischange>
     (b)  ( <current> and <ischange> ) and <sw_mask> = <state>
     (3)  if <state> == SW_MASK then switch pressed

     */

     bTempSwitches     = senseBlockBuffer[bBuffOut].bSwitches & 0x03;
     bSwitchState      = bPrevSwitchState ^ bTempSwitches;
     bSwitchState     &= bTempSwitches;
     bPrevSwitchState  = bTempSwitches;

     if ( (bSwitchState & SW_LEFT) == SW_LEFT )
        printf("S_BUMPER_SW, S_BUMPER_LEFT\n");

     if ( (bSwitchState & SW_RIGHT) == SW_RIGHT )
        printf("S_BUMPER_SW, S_BUMPER_RIGHT\n");

     if ( (bSwitchState & SW_MID) == SW_MID )
        printf("S_BUMPER_SW, S_BUMPER_MID\n");

     if ( (bSwitchState & (SW_LEFT | SW_MID)) == (SW_LEFT | SW_MID) )
        return;

     /* circular buffer index update */
     bBuffOut++;
     if ( bBuffOut == MAX_SENSE_BLOCKS )
        bBuffOut = 1;
     bCount--;
    } /* end while bCount > 0 */

    } /* endless while loop */
}

void
main(void)
{
 printf("main(): auxCommInit()\n");
 auxCommInit(auxCommIsr);
 printf("main(): t_sense()\n");
 t_sense();
 printf("main(): quiting.\n");
}

