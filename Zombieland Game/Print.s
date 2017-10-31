; Print.s
; Student names: change this to your names or look very silly
; Last modification date: change this to the last modification date or look very silly
; Runs on LM4F120 or TM4C123
; EE319K lab 7 device driver for any LCD
;
; As part of Lab 7, students need to implement these LCD_OutDec and LCD_OutFix
; This driver assumes two low-level LCD functions
; ST7735_OutChar   outputs a single 8-bit ASCII character
; ST7735_OutString outputs a null-terminated string 

BIG	EQU	9999
	
    IMPORT   ST7735_OutChar
    IMPORT   ST7735_OutString
    EXPORT   LCD_OutDec
    EXPORT   LCD_OutFix

    AREA    |.text|, CODE, READONLY, ALIGN=2
		PRESERVE8
    THUMB

star	DCB	0x2A
		DCB	0x2E
		DCB	0x2A
		DCB	0x2A
		DCB	0x2A
		DCB	0x00
  

;-----------------------LCD_OutDec-----------------------
; Output a 32-bit number in unsigned decimal format
; Input: R0 (call by value) 32-bit unsigned number
; Output: none
; Invariables: This function must not permanently modify registers R4 to R11
decCount EQU 0			;binded decCount with an offset of 0 relative to SP
LCD_OutDec
	PUSH{R4, R5, LR}
	MOV R1, #10			;R1 is a constant used as a divisor
	MOV R4, #0
	PUSH {R4}			;allocated 1 byte in stack and initialized to 0
	ADD R5, SP, #0
	
	UDIV R2, R0, R1		;R2 = R0/10
	ADD R4, #1			;counter++
	STR R4, [R5, #decCount]
	MLS R3, R1, R2, R0	;R3 = R0 - (R1 * R2), R3 now holds the remainder
	ADD R3, #0x30		;convert to ASCII
	PUSH {R3, R11}		;push to stack
	UDIV R0, R0, R1		;R0 = R0/10, clears rightmost digit of original value
	CMP R0, #0
	BEQ decPopLoop
	
decPushLoop
	UDIV R2, R0, R1		;R2 = R0/R1, effectively shifts original value right by one digit (ex. 630->63)
	LDR R4, [R5, #decCount]
	ADD R4, #1
	STR R4, [R5, #decCount]
	MLS R3, R1, R2, R0	;R3 = R0 - (R1 * R2)
	ADD R3, #0x30
	PUSH {R3, R11}
	UDIV R0, R1
	CMP R0, #0
	BEQ decPopLoop
	B decPushLoop
	
decPopLoop
	POP {R0, R11}
	BL	ST7735_OutChar
	LDR R4, [R5, #decCount]
	SUB R4, #1
	STR R4, [R5, #decCount]
	CMP R4, #0
	BNE decPopLoop
	
	ADD SP, #4			;Deallocated space for local variables
	POP{R4, R5, LR}
	BX  LR
;* * * * * * * * End of LCD_OutDec * * * * * * * *

; -----------------------LCD _OutFix----------------------
; Output characters to LCD display in fixed-point format
; unsigned decimal, resolution 0.001, range 0.000 to 9.999
; Inputs:  R0 is an unsigned 32-bit number
; Outputs: none
; E.g., R0=0,    then output "0.000 "
;       R0=3,    then output "0.003 "
;       R0=89,   then output "0.089 "
;       R0=123,  then output "0.123 "
;       R0=9999, then output "9.999 "
;       R0>9999, then output "*.*** "
; Invariables: This function must not permanently modify registers R4 to R11
n	EQU	0 ;Binding
LCD_OutFix
	PUSH{R4,R5,R6,R7,R8,LR}
	SUB		SP,#4 ;Allocation
	STR		R0,[SP,#n] ;Access
	MOV		R1, #BIG
	CMP		R0,R1
	BHI		tooBig
	
	MOV		R4,#1000
	UDIV	R0,R4
	ADD		R0,#0x30
	BL		ST7735_OutChar ;Output first number
	
	MOV		R0,#0x2E
	BL		ST7735_OutChar ;Output Decimal
	
	LDR		R0,[SP,#n] ;Access
	MOV		R4,#1000
	UDIV	R5,R0,R4 ;Obtain Thousands Digit
	MUL		R4,R5	
	SUB		R0,R4 ;R0 = Remainder of n/1000
	STR		R0,[SP,#n] ;Access
	
	MOV		R4,#100
	UDIV	R0,R4
	ADD		R0,#0x30
	BL		ST7735_OutChar
	
	LDR		R0,[SP,#n] ;Access
	MOV		R4,#100
	UDIV	R5,R0,R4 ;Obtain Hundreds Digit
	MUL		R4,R5	
	SUB		R0,R4 ;R0 = Remainder of n/100
	STR		R0,[SP,#n] ;Access
	
	MOV		R4,#10
	UDIV	R0,R4
	ADD		R0,#0x30
	BL		ST7735_OutChar
	
	LDR		R0,[SP,#n] ;Access
	MOV		R4,#10
	UDIV	R5,R0,R4 ;Obtain Tens Digit
	MUL		R4,R5	
	SUB		R0,R4 ;R0 = Remainder of n/10
	STR		R0,[SP,#n] ;Access
	
	ADD		R0,#0x30 
	BL		ST7735_OutChar
	
	B		fixdone
	
tooBig
	LDR	R0,=star
	BL	ST7735_OutString
	
fixdone
	ADD	SP,#4 ;Deallocation
	POP{R4,R5,R6,R7,R8,LR}
     BX   LR
 
     ALIGN
;* * * * * * * * End of LCD_OutFix * * * * * * * *

     ALIGN                           ; make sure the end of this section is aligned
     END                             ; end of file
