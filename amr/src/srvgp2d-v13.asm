;
;*********************************************************************
;* srvogp2d-v13.asm
;*
;* Dec. 29, 2011   - v1.0 created
;* Jan. 6,  2012   - v1.2 get/send, servo, GP2D and main loop
;* Jan. 27, 2012   - v1.3 separated GP2D byte read from trigger
;*
;* code to support servo motor PWM and GP2D distance sensor.
;* micro will link through simple serial connection to V25 board for
;* command/data and to servo + GP2D sensor.
;*
;* application's controller pin functionality:
;*  PTA0, [dig out] - PWM servo control (TCH0)
;*  PTA1, [ana in]  - battery level (AD1) [acting as ext. trigger for timing GP2D]
;*  PTA2, [dig in]  - from GP2D Vout
;*  PTA3, [dig out] - to GP2D Vin
;*  PTA4, [dig out] - Data out (in) to (from) V25
;*  PTA5, [dig in]  - Clock in from V25
;*
;* communication protocol:
;*  always initiated by V25
;*  data sent or received in 8 bit length
;*  4 bit command format (b0 through b3):
;*    0     - no command / invalid command / error
;*    1     - get revision (two 8-bit values will follow to V25:
;*            revision code out to V25 and last reset status/reason)
;*    2     - servo position (8-bit servo position will follow from V25)
;*    3     - trigger GP2D (8-bit true($ff)/false($00) to V25 will follow)
;*    4     - read GP2D (8-bit GP2D distance data out to V25 will follow)
;*    5     - read battary (8-bit battary voltage out to V25 will follow)
;*    6..15 - not used
;*    $aa   - echo $aa byte (echo comm test)
;*
;*********************************************************************
;
; TO-DO
;  1) take advantage of brset/brclr Carry flag setup
;  2) use CLRA instead of LDA #0
;
;--------------------------------------------------------------
;             ROM routine
;--------------------------------------------------------------
;*** DELNUS renamed to ROM_DELAY [p.14 sec 3.6 and 9.6 AN1831]
; uses two parameters in Accumulator (A) and X register (X)
; delay (cycles) resulting from this routine is:
;  3 × (A value) × (X value) + 8 cycles (where a value of A>=4, X>=1)
;
; ROM_DELAY: deca
; DEC_X:     psha
;            deca
;            deca
; DEC_Acc:   dbnza  DEC_Acc
;            pula
;            dbnzx  DEC_X
;            rts
;
ROM_DELAY:    equ       $280C                  ; DELNUS delay routine in QT4 ROM
;
;
; Include derivative-specific definitions
              include   'MC68HC908QT4.inc'
;**************************************************************
;*            Definitions
;**************************************************************

MAC_watchdog: MACRO
              sta COPCTL                       ; re-trigger watchdog
              ENDM

MAC_2mSec:    MACRO                            ; delay macros using DELNUS
              psha
              pshx
              lda       #105
              ldx       #20
              jsr       ROM_DELAY
              pulx
              pula
              ENDM

MAC_1mSec:    MACRO
              psha
              pshx
              lda       #105
              ldx       #10
              jsr       ROM_DELAY
              pulx
              pula
              ENDM

MAC_500uSec:  MACRO
              psha
              pshx
              lda       #105
              ldx       #5
              jsr       ROM_DELAY
              pulx
              pula
              ENDM

MAC_100uSec:  MACRO
              psha
              pshx
              lda       #105
              ldx       #1
              jsr       ROM_DELAY
              pulx
              pula
              ENDM

CODEREV:      equ       $13                    ; revision "1.3"

CMD_GETREV:   equ       1                      ; V25/HC08 command set
CMD_SRVPOS:   equ       2
CMD_GP2DTRG:  equ       3
CMD_GP2DRD:   equ       4
CMD_GETVOLT:  equ       5
ECHO:         equ       $aa                    ; special echo command
;
GP2D_DATA     equ       PTA_PTA2               ; port a bit aliases
GP2D_CLK      equ       PTA_PTA3
COMM_DATA     equ       PTA_PTA4
COMM_CLK      equ       PTA_PTA5

;--------------------------------------------------------------
;             CONFIG reg.
;--------------------------------------------------------------
CONFIG1_INIT: equ       %00111001              ;default initialization
;                        ||||||||       CONFIG1 is a write-once register
;                        |||||||+-COPD    - 1 disable COP watchdog
;                        ||||||+--STOP    - 0 disable STOP instruction
;                        |||||+---SSREC   - 0 4096 cycle STOP recovery
;                        ||||+----LVI5OR3 - 1 set LVI for 5v system
;                        |||+-----LVIPWRD - 1 disable power to LVI system
;                        ||+------LVIRSTD - 1 disable reset on LVI trip
;                        |+-------LVISTOP - 0 enable LVI in STOP mode
;                        +--------COPRS   - 0 long COP timeout

;--------------------------------------------------------------
;             Port A reg.
;--------------------------------------------------------------
PORTA_DEF:    equ       %00011000              ; PTA(x) default output pin levels
PORTA_DDR:    equ       %00011010
;                        ||||||||       PORTA DDR 
;                        |||||||+-PTA0    - 0 (PWM clock channel)
;                        ||||||+--PTA1    - 1 output, *** ext. timing trigger
;                        |||||+---PTA2    - 0 input , GP2D Vo
;                        ||||+----PTA3    - 1 output, GP2D Vi
;                        |||+-----PTA4    - 1 output, V25 data
;                        ||+------PTA5    - 0 input , V25 clock
;                        |+-------n/a     - 0
;                        +--------n/a     - 0
PORTA_PUP:    equ       %00100000
;                        ||||||||       PORTA Pull-up config 
;                        |||||||+-PTA0    - 0 disable, (clock channel)
;                        ||||||+--PTA1    - 0 disable, output, ext. timing trigger
;                        |||||+---PTA2    - 0 disable, input , GP2D Vo
;                        ||||+----PTA3    - 0 disable, output, GP2D Vi
;                        |||+-----PTA4    - 1 enable,  output/input, V25/HC08 data
;                        ||+------PTA5    - 1 enable,  input,  V25 clock
;                        |+-------n/a     - 0
;                        +--------n/a     - 0

;--------------------------------------------------------------
;             Timer channel 0
;--------------------------------------------------------------
TIM_INIT:     equ       %00110110              ; timer control/mode
;                        ||||||||       TIM initialization
;                        |||||||+-PS0     - 0 |
;                        ||||||+--PS1     - 1 +- div. by 64 pre-scaler
;                        |||||+---PS2     - 1 |
;                        ||||+----n/a     - 0
;                        |||+-----TRST    - 1 Prescaler and TIM counter cleared
;                        ||+------TSTOP   - 1 TIM counter stopped
;                        |+-------TOIE    - 0 TIM overflow interrupts disabled
;                        +--------TOF     - 0 overflow flag
TSC0_INIT:    equ       %00011010
;                        ||||||||       TIM Channel 0 PWM initialization
;                        |||||||+-CH0MAX  - 0 not max. on TOV
;                        ||||||+--TOV0    - 1 channel 0 pin toggles on TIM counter overflow
;                        |||||+---ELS0A   - 0 | Clear output on compare
;                        ||||+----ELS0B   - 1 |
;                        |||+-----MS0A    - 1 Unbuffered output compare/PWM operation
;                        ||+------MS0B    - 0 Buffered output compare/PWM operation disabled
;                        |+-------CH0IE   - 0 Channel 0 interrupt requests disabled
;                        +--------CH0F    - 0 n/a
mSTOP_TIM:    equ       %00100000              ; 'or' mask to stop timer
mSTART_TIM:   equ       %11011111              ; 'and' mask to start timer
mRST_TIM:     equ       %00010000              ; 'or' mask to reset timer
mSET_SCALER:  equ       %00000110              ; 'or' mask to set pre-scalar mask (bus/64)
TIM_MODH:     equ       $03                    ; modulo counter values for 20mSec PWM cycle time
TIM_MODL:     equ       $e8
TIM_CHANH:    equ       $00                    ; default compare value for PWM period
TIM_CHANL:    equ       $50                    ; (d80) for mid servo position of 1.5mSec
MIN_PWM:      equ       $23                    ; (d35) min. PWM value in TCH0L for 0.5mSec
MAX_PWM:      equ       $7d                    ; (d125) max. PWM value in TCH0L for 2.5mSec

;--------------------------------------------------------------
;             ADC
;--------------------------------------------------------------
ADSCR_INIT:   equ       %00100001
;                        ||||||||       ADC channel 1 initialization
;                        |||||||+-CH      - 1 |
;                        ||||||+--CH      - 0 |
;                        |||||+---CH      - 0 +- Channle select 1
;                        ||||+----CH      - 0 |
;                        |||+-----CH      - 0 |
;                        ||+------ADCO    - 1 Continuous ADC conversion
;                        |+-------AIEN    - 0 ADC interrupt disabled
;                        +--------COCO    - 0 n/a
ADC_OFF:      equ       $1f                    ; ADC off
ADC_DUMMY:    equ       $aa                    ; dummy return value for ADC
ADICLK_INIT:  equ       $00                    ; conversion clock = bus speed

;FLBPR:        equ       $FFBE                  ;flash block protect reg (flash)
;              org       FLBPR                  ;flash block protect location
;              fcb       $FE                    ;protect this code, FLBPR,& vectors

;**************************************************************
;*            Data
;**************************************************************
              org       RAMStart
RST_STATUS:   fcb       0                      ; last system reset status (copy of SRSR reg.)

;**************************************************************
;*            Code
;**************************************************************
              org       ROMStart
;--------------------------------------------------------------
;             HC908 processor initialization
;--------------------------------------------------------------
_Startup:     lda       SRSR
              sta       RST_STATUS             ; save last reset status
              lda       #CONFIG1_INIT
              sta       CONFIG1                ; initialize CONFIG1 reg.
              lda       TRIMLOC
              sta       OSCTRIM                ; set working trim value in ICG
              ldhx      #RAMEnd+1
              txs                              ; initialize the stack pointer
              sei                              ; disable interrupts
;--------------------------------------------------------------
;             configure PTA(x) port pins
;              PTA0, [dig out] - PWM servo control (TCH0)
;              PTA1, [ana in]  - battery level (AD1)
;              PTA2, [dig in]  - from GP2D Vo
;              PTA3, [dig out] - to GP2D Vi
;              PTA4, [dig out] - Data out (in) to (from) V25
;              PTA5, [dig in]  - Clock in from V25
;--------------------------------------------------------------
              lda       #PORTA_DEF
              sta       PTA                    ; setup init output levels before direction
              lda       #PORTA_PUP
              sta       PTAPUE
              lda       #PORTA_DDR
              sta       DDRA
;--------------------------------------------------------------
;             timer channel 0 setup for PWM
;--------------------------------------------------------------
              lda       #TIM_INIT
              sta       TSC                    ; initialize timer and set pre-scaler
              lda       #TIM_MODH
              sta       TMODH
              lda       #TIM_MODL
              sta       TMODL                  ; setup modulo counter to 20mSec PWM cycle time
              lda       #TIM_CHANH
              sta       TCH0H
              lda       #TIM_CHANL
              sta       TCH0L                  ; setup channel 0 compare value for PWM period
              lda       #TSC0_INIT
              sta       TSC0                   ; setup channel 0 for PWM
              lda       TSC
              and       #mSTART_TIM
              sta       TSC                    ; start the timer (PWM)
;--------------------------------------------------------------
;             ADC channel 1 setup
;--------------------------------------------------------------
              lda       #ADSCR_INIT            ; use ADSCR_INIT if ADC1 is implemented
              sta       ADSCR                  ; setup ADC channel 1
              lda       #ADICLK_INIT
              sta       ADICLK                 ; conversion clock = bus clock
;--------------------------------------------------------------
;             main program loop
; commands:
;  CMD_GETREV:   equ       1
;  CMD_SRVPOS:   equ       2
;  CMD_GP2DTRG:  equ       3
;  CMD_GP2DRD:   equ       4
;  CMD_GETVOLT:  equ       5
;  ECHO:         equ       $aa
;--------------------------------------------------------------
Mainloop:     jsr       _GetByte               ; get command
              cmp       #CMD_GETREV
              bne       SERVO
              jsr       _SendRev               ; revision read handler
              bra       Mainloop
SERVO:        cmp       #CMD_SRVPOS
              bne       GP2D_TRG
              jsr       _GetByte               ; servo PWM handler
              jsr       _SetServoPos           ; send position to servo
              bra       Mainloop
GP2D_TRG:     cmp       #CMD_GP2DTRG
              bne       GP2D_RD
              jsr       _TrigGP2D              ; GP2D trigger handler
              jsr       _SendByte              ; return trigger status
              bra       Mainloop
GP2D_RD:      cmp       #CMD_GP2DRD
              bne       VOLT
              jsr       _ReadGP2D              ; GP2D handler
              jsr       _SendByte              ; send GP2D readout
              bra       Mainloop
VOLT:         cmp       #CMD_GETVOLT
              bne       RCV_ECHO
              jsr       _ReadBattVolt          ; battery voltage read handler
              jsr       _SendByte              ; send ADC readout
              bra       Mainloop
RCV_ECHO:     cmp       #ECHO
              bne       Mainloop
              jsr       _SendByte              ; echo received byte
              bra       Mainloop
;
;--------------------------------------------------------------
;  _GetByte
;
;   get byte from serial connection from V25
;   byte returned in Accumulator (A)
;   all affected registers are preserved
;   this is a blocking function and will return only after
;   8 bit data is received from V25 LSB first
;   serial comm PTA4 data direction is entered as 'out',
;   changed to 'in' for the duraiton of the transmission,
;   and reverted back to 'out'.
;   IO pins:
;    PTA4, [dig out] - Data out (in) to (from) V25
;    PTA5, [dig in]  - Clock in from V25
;--------------------------------------------------------------
;
_GetByte:     pshx                             ; save registers
              lda       #0
RX_SIG:       brset     COMM_CLK,PTA,RX_SIG    ; wait for V25 to lower CLK line
              MAC_100uSec                      ; wait another 100uSec and check again...
              brset     COMM_CLK,PTA,EXIT_GET  ; exit if V25 did not hold CLK at LO
              bclr      COMM_DATA,PTA          ; signal ready by lowering DATA line
WAIT_SWAP:    brclr     COMM_CLK,PTA,WAIT_SWAP ; wait for clock HI before swapping data pin direction
              bclr      DDRA_DDRA4,DDRA        ; set DATA to input
              ldx       #4                     ; load counter for 8-bit size (4 pairs)
RX_CLKl:      brset     COMM_CLK,PTA,RX_CLKl   ; wait for CLK to go LO              
              bsr       GET_BIT
RX_CLKh:      brclr     COMM_CLK,PTA,RX_CLKh   ; wait for CLK to go HI
              bsr       GET_BIT
              dbnzx     RX_CLKl                ; cycle through next bits
              bset      COMM_DATA,PTA          ; set DATA line to HI when done
              bset      DDRA_DDRA4,DDRA        ; set DATA back to output
              MAC_100uSec
EXIT_GET:     pulx                             ; restore registers
              rts
;
GET_BIT:      brset     COMM_DATA,PTA,RX_HI_BIT; did we received HI or LO?
              clc                              ; a LO
              bra       STORE_BIT
RX_HI_BIT:    sec                              ; a HI
STORE_BIT:    rora
              rts
;
;--------------------------------------------------------------
;  _SendByte
;
;   send data from Accumulator (A) through serial to V25
;   Accumulator and all registers are preserved
;   this function returnes after sending 8 bits LSB first
;   IO pins:
;    PTA4, [dig out] - Data out (in) to (from) V25
;    PTA5, [dig in]  - Clock in from V25
;--------------------------------------------------------------
;
_SendByte:    psha                             ; save registers
              pshx
TX_SIG:       brset     COMM_CLK,PTA,TX_SIG    ; wait for V25 to lower CLK line
              MAC_100uSec                      ; wait another 100uSec and check again...
              brset     COMM_CLK,PTA,EXIT_SEND ; exit if V25 did not hold CLK at LO
              bclr      COMM_DATA,PTA          ; signal ready by lowering DATA line
              ldx       #4                     ; load counter for 8-bit size (4 pairs)
TX_CLKh:      brclr     COMM_CLK,PTA,TX_CLKh   ; wait for CLK to go HI
              bsr       SEND_BIT
TX_CLKl:      brset     COMM_CLK,PTA,TX_CLKl   ; wait for CLK to go LO
              bsr       SEND_BIT
              dbnzx     TX_CLKh                ; cycle through next bits
WAIT_CLKh:    brclr     COMM_CLK,PTA,WAIT_CLKh ; wait for CLK to go HI
              bset      COMM_DATA,PTA          ; set DATA line to HI when done
              MAC_100uSec
EXIT_SEND:    pulx                             ; restore registers
              pula
              rts
;
SEND_BIT:     rora
              bcc       TX_LO_BITe
              bset      COMM_DATA,PTA           ; set bit HI
              rts
TX_LO_BITe:   bclr      COMM_DATA,PTA           ; set bit LO
              rts
;
;--------------------------------------------------------------
;  _SendRev
;
;   send code revision and reset status to V25
;   all registers are preserved
;--------------------------------------------------------------
;
_SendRev:     psha
              lda       #CODEREV
              bsr       _SendByte              ; load and send code rev to V25
              lda       RST_STATUS
              bsr       _SendByte              ; load and send last Reser status/reason to V25
              pula
              rts
;
;--------------------------------------------------------------
;  _SetServoPos
;
;   set servo PWM from count passed in Accumulator (A)
;   all registeres are preserved
;--------------------------------------------------------------
;
_SetServoPos: psha
              cmp       #MAX_PWM               ; range-check value is <=max. AND >=min.
              bhi       TOO_HIGH               ; treat as unsigned value when comparing
              cmp       #MIN_PWM
              blo       TOO_LOW
              bra       SET_PWM
TOO_HIGH:     lda       #MAX_PWM
              bra       SET_PWM
TOO_LOW:      lda       #MIN_PWM
SET_PWM:      sta       TCH0L                  ; store new PWM into counter
              pula
              rts

;--------------------------------------------------------------
;  _TrigGP2D
;
;   trigger GP2D sensor and return with true/false in Accumulator (A)
;   true($ff) trigger ok, false($00) did not trigger
;   all other registers are preserved.
;   IO pins:
;    PTA2, [dig in]  GP2D_DATA - from GP2D Vout
;    PTA3, [dig out] GP2D_CLK  - to GP2D Vin
;--------------------------------------------------------------
;
_TrigGP2D     lda       #0
              brclr     GP2D_DATA,PTA,NOT_RDY  ; exit if GP2D Vo is LO, can't trigger
              bclr      GP2D_CLK,PTA           ; pull GP2D Vin LO to signal start of measurment
              MAC_100uSec                      ; delay 100uSec to give sensor time to power on
              brset     GP2D_DATA,PTA,NO_TRIG  ; exit if GP2D Vo is HI, didn't trigger
              lda       #$ff                   ; exit with 'true' if GP2D did trigger
              rts
;
NO_TRIG:      bset      GP2D_CLK,PTA           ; exit with clock HI
NOT_RDY:      rts
;
;--------------------------------------------------------------
;  _ReadGP2D
;
;   read GP2D sensor and return distance data byte in Accumulator (A)
;   all other registers are preserved.
;   IO pins:
;    PTA2, [dig in]  GP2D_DATA - from GP2D Vout
;    PTA3, [dig out] GP2D_CLK  - to GP2D Vin
;--------------------------------------------------------------
;
_ReadGP2D:    pshx                             ; save registers
              lda       #0
MEASURE:      brclr     GP2D_DATA,PTA,MEASURE  ; wait for measurement to complete (total ~45mSec)
              ldx       #8                     ; load bit counter
              bset      GP2D_CLK,PTA           ; raise clock in prepapration for reading
              MAC_100uSec                      ; delay 100uSec
NEXT_BIT:     bclr      GP2D_CLK,PTA           ; clock a bit
              MAC_100uSec                      ; delay 100uSec
              brset     GP2D_DATA,PTA,GP2D_HI  ; did we receive HI or LO?
              clc                              ; a LO
              bra       ROTATE_IN
GP2D_HI:      sec                              ; a HI
ROTATE_IN:    rola                             ; rotate left, MSB in first
              bset      GP2D_CLK,PTA           ; raise clock
              MAC_100uSec                      ; delay 100uSec
              dbnzx     NEXT_BIT               ; get next bit from GP2D
              bset      GP2D_CLK,PTA           ; exit with clock HI
              MAC_2mSec                        ; delay 2mSec before next conversion can start
              pulx                             ; restore registers
              rts
;
;--------------------------------------------------------------
;  _ReadBattVolt
;
;   read AD1 voltage and return value in Accumulator
;--------------------------------------------------------------
;
_ReadBattVolt:brclr     ADSCR_COCO,ADSCR,_ReadBattVolt ; is convertion complete?
              lda       ADR                    ; yes, return ADC reading
              rts
;
;**************************************************************
;*            Internal Oscillator Trim
;**************************************************************
              org       TRIMLOC
              fcb       $66

;**************************************************************
;*            Interrupt Vectors
;**************************************************************
              org       INT_RESET
              fdb       _Startup               ; Reset
