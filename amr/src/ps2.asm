;
;=====================================================
;  file: ps2.asm
;
; PS/2 mouse device data write source code.
;
; April 10 2000 - Created
;
;=====================================================
;
            .MODEL     SMALL
;
;
            .CODE
;
;
;----------------------------------------
; definitions
;----------------------------------------
;
PS2_OK       equ        -1
PS2_ERR      equ         0
INHIBIT_TIME equ         2000 ;
P2           equ         16   ; ps2 control bits port address
;
;=====================================================
; ps2_write()
;
; this function accepts a BYTE variable to send to
; the PS/2 device. it returns an integer value as status.
;
; WARNING: this function will not return if the PS/2
;          device misbehaves and CLK line is not
;          properly clocked by the device.
;=====================================================
;
	         public     _ps2_write
;
_ps2_write  proc       near
;
            push       bp
            mov        bp,sp
;
            push       bx
            push       cx
            push       dx
            push       es
;
;----------------------------------------
; get data byte to send and invert bits
; before sending  due to 74HC05 inverter
; on Data line.
;----------------------------------------
;
            mov        al,byte ptr [bp+4]
            not        al
;
;----------------------------------------
; setup pointer to V25 register/ports
; assuming ports are setup correctly!
;----------------------------------------
;
            mov        bx, 0f000h
            mov        es, bx
            mov        bx, 0ff00h
;
;----------------------------------------
; start data send sequence
;----------------------------------------
;
           mov         ah,0                    ;  setup parity value.
;
;----------------------------------------
; set inhibit mode on PS/2 device
;----------------------------------------
;
           mov         dl,byte ptr es:[bx+P2]
           or          dl,2                    ; CLK to '0'.
           and         dl,0feh                 ; Data to '1'.
           mov         byte ptr es:[bx+P2],dl

           mov         cx,INHIBIT_TIME         ; should be >100uSec...
INHIBIT_WAIT:
           dec         cx
           jnz         short INHIBIT_WAIT
;
;----------------------------------------
; set 'request to send' mode
;----------------------------------------
;
           or          dl,1                    ; DATA to '0'.
           mov         byte ptr es:[bx+P2],dl
           and         dl,0fdh                 ; CLK to '1'.
           mov         byte ptr es:[bx+P2],dl
;
;----------------------------------------
; loop to send 8 data bits
;----------------------------------------
;
           xor         cx,cx
           jmp         short START

WAIT_CLK_LOX:
           test        byte ptr es:[bx+P2],4   ; wait for CLK to go low before
           jnz         short WAIT_CLK_LOX      ; sending bit.

           mov         dh,al
           and         dh,1
           and         dl,0feh
           or          dl,dh
           mov         byte ptr es:[bx+P2],dl  ; send bit.

           add         ah,al                   ; accumulate parity.

WAIT_CLK_HIX:
           test        byte ptr es:[bx+P2],4   ; wait for CLK to go high before
           jz          short WAIT_CLK_HIX      ; processing next bit.

           shr	        al,1

	        inc         cx

START:
           cmp         cx,8                    ; check if done sending data
	        jz          short WAIT_CLK_LOP      ; yes, proceed to parity
	        jmp         short WAIT_CLK_LOX      ; no, next data bit.
;
;----------------------------------------
; send parity bit
;----------------------------------------
;
WAIT_CLK_LOP:
           test        byte ptr es:[bx+P2],4   ; wait for CLK to go low before
           jnz         short WAIT_CLK_LOP      ; sending parity.

           mov         dh,ah
           and         dh,1
           and         dl,0feh
           or          dl,dh
           mov         byte ptr es:[bx+P2],dl  ; send parity

WAIT_CLK_HIP:
           test        byte ptr es:[bx+P2],4   ; wait for CLK to go high before
           jz          short WAIT_CLK_HIP      ; processing stop bit.
;
;----------------------------------------
; send stop bit
;----------------------------------------
;
WAIT_CLK_LOS:
           test        byte ptr es:[bx+P2],4   ; wait for CLK to go low before
           jnz         short WAIT_CLK_LOS      ; sending stop bit.

           and         dl,0feh
           mov         byte ptr es:[bx+P2],dl  ; send stop bit.

WAIT_CLK_HIS:
           test        byte ptr es:[bx+P2],4   ; wait for CLK to go high before
           jz          short WAIT_CLK_HIS      ; processing 'line control'.
;
;----------------------------------------
; wait for 'line control'
;----------------------------------------
;
WAIT_CLK_LOLC:
           test        byte ptr es:[bx+P2],4   ; wait for CLK to go low
           jnz         short WAIT_CLK_LOLC
;
; need to sample line control to make sure Data line is
; pulled low by PS/2 device...
; signal error if not.
;
WAIT_CLK_HILC:
           test        byte ptr es:[bx+P2],4   ; wait for CLK to go high
           jz          short WAIT_CLK_HILC
;
;----------------------------------------
; completion
;----------------------------------------
;
            pop        es
            pop        dx
            pop        cx
            pop        bx

            mov        ax,PS2_OK
            pop        bp
            ret
;
_ps2_write  endp
;
;
            end
;
