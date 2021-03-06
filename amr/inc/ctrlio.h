/* ***************************************************************************

  CTRLIO.H

  Control I/O definitions for system board.

  July 27 1999 - E.A. Created
  Dec. 31 2011 - modified for HC08 communication

*************************************************************************** */

#ifndef __CTRLIO_H__
#define __CTRLIO_H__

/* =========================================
   O U T P U T
========================================= */

/* -----------------------------------------------------------------------
   address : $300
   function: watch-dog trigger, Motor ON/OFF, 7-segment.
             this register will clear (0x00) on system power-up and reset

     d7     d6     d5     d4     d3     d2     d1     d0
  +------+------+------+------+------+------+------+------+
  | RMON | LMON | WDOG | BLK  | DIS3 | DIS2 | DIS1 | DIS0 |
  +------+------+------+------+------+------+------+------+
     |      |      |       |     |      |       |     |
     |      |      |       |     |      |       |     |
     |      |      |       |     +------+-------+-----+---  7-segment BCD code
     |      |      |       +------------------------------  7-segment blanking
     |      |      +--------------------------------------  watchdog trigger
     |      +---------------------------------------------  left motor on
     +----------------------------------------------------  right motor on

----------------------------------------------------------------------- */

#define  UTIL           0x0300

#define  MASK_DISP_BLK  0x10
#define  MASK_WDOG      0x20
#define  MASK_LMON      0x40
#define  MASK_RMON      0x80       

/* -----------------------------------------------------------------------
   address : $301
   function: Motor control register

     d7     d6     d5     d4     d3     d2     d1     d0
  +------+------+------+------+------+------+------+------+
  | LREV | LSC2 | LSC1 | LSC0 | RREV | RSC2 | RSC1 | RSC0 |
  +------+------+------+------+------+------+------+------+
     |      |      |       |     |      |       |     |
     |      |      |       |     |      |       |     |
     |      |      |       |     |      +-------+-----+---  right motor speed
     |      |      |       |     +------------------------  right motor reverse
     |      +------+-------+------------------------------  left motor speed
     +----------------------------------------------------  left motor reverse


----------------------------------------------------------------------- */

#define  MOTOR_CTRL     0x0301

/* -----------------------------------------------------------------------
   address : $302
   function: growth
----------------------------------------------------------------------- */

#define  GROWTH_O1      0x0302

/* -----------------------------------------------------------------------
   address : $303
   function: growth
----------------------------------------------------------------------- */

#define  GROWTH_O2      0x0303

/* =========================================
   I N P U T
========================================= */

/* -----------------------------------------------------------------------
   address : $300
   function: Service Request Register
             seven equel priority service request bits.

             Run flag:  '1' application auto executes on boot
                        '0' no auto-execute on boot.
             BAUD rate: '1' 9600 BAUD (system default)
                        '0' 19200 BAUD

     d7     d6     d5     d4     d3     d2     d1     d0
  +------+------+------+------+------+------+------+------+
  | RUN  | SR6  | SR5  | SR4  | SR3  | SR2  | SR1  | SR0  |
  +------+------+------+------+------+------+------+------+
     |      |      |       |     |      |       |     |
     |      |      |       |     |      |       |     +---  service #0
     |      |      |       |     |      |       +---------  service #1
     |      |      |       |     |      +-----------------  service #2
     |      |      |       |     +------------------------  service #3
     |      |      |       +------------------------------  service #4
     |      |      +--------------------------------------  service #5
     |      +---------------------------------------------  Auto-Run
     +----------------------------------------------------  BAUD rate


----------------------------------------------------------------------- */

#define  SRR            0x0300

#define  ENA_19200_BAUD 0x80
#define  AUTO_RUN       0x40

/* -----------------------------------------------------------------------
   address : $301
   function: growth
----------------------------------------------------------------------- */

#define  GROWTH_I1      0x0301

/* -----------------------------------------------------------------------
   address : $302
   function: growth
----------------------------------------------------------------------- */

#define  GROWTH_I2      0x0302

/* -----------------------------------------------------------------------
   address : $303
   function: growth
----------------------------------------------------------------------- */

#define  GROWTH_I3      0x0303

/* =========================================
   V25 I/O port and pin usage
========================================= */

/* -----------------------------------------------------------------------
   V25 port1
   function: HC08 comm

     d7     d6     d5     d4     d3     d2     d1     d0
  +------+------+------+------+------+------+------+------+
  | n/c  | n/c  | HC08 | HC08 | INT2^| INT1^| INT0^| NMI  |
  |      |      | CLK  | DATA |      |      |      |      |
  +------+------+------+------+------+------+------+------+
     |      |      |       |     |      |       |     |
     |      |      |       |     |      |       |     |
     |      |      |       |     |      |       |     +---
     |      |      |       |     |      |       +---------
     |      |      |       |     |      +-----------------
     |      |      |       |     +------------------------
     |      |      |       +---------------------------->< HC08 serial data in/out ["turret" connector pin 4]
     |      |      +-------------------------------------> HC08 clock ["turret" connector pin 3]
     |      +--------------------------------------------> SONAR ON
     +---------------------------------------------------- ["turret" connector pin 1]

----------------------------------------------------------------------- */

#define  P1_MODE_CTRL   0x00
#define  P1_MODE        0x9f
#define  P1_INIT        0x30

/* -----------------------------------------------------------------------
   V25 port2
   function: Misc. control

     d7     d6     d5     d4     d3     d2     d1     d0
  +------+------+------+------+------+------+------+------+
  | n/c  | n/c  | n/c  | n/c  | n/c  | PS2Ci| PS2Co| PS2D |
  +------+------+------+------+------+------+------+------+
     |      |      |       |     |      |       |     |
     |      |      |       |     |      |       |     |
     |      |      |       |     |      |       |     +-->  PS/2 data [out]
     |      |      |       |     |      |       +-------->  PS/2 clock [out]
     |      |      |       |     |      +----------------<  PS/2 clock [in]
     |      |      |       |     +------------------------
     |      |      |       +------------------------------
     |      |      +--------------------------------------
     |      +---------------------------------------------  (scheduler task 1)
     +----------------------------------------------------  (task flag)

----------------------------------------------------------------------- */

#define  P2_MODE_CTRL   0x00
#define  P2_MODE        0xfc
#define  P2_INIT        0x00

#endif /* __CTRLIO_H__ */
