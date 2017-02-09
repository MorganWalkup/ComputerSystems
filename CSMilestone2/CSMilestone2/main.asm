  ;
; CSAssignment2.asm
;
; Created: 2/2/2017 1:48:32 PM
; Author : Morgan Walkup
;

RJMP INITIALIZE 			; Jump to INITIALIZE
RJMP INITIALIZE				; Jump to INITIALIZE
RJMP INITIALIZE				; Jump to INITIALIZE
RJMP INITIALIZE				; Jump to INITIALIZE
RJMP INITIALIZE				; Jump to INITIALIZE
RJMP INITIALIZE				; Jump to INITIALIZE
RJMP INITIALIZE				; Jump to INITIALIZE
RJMP INITIALIZE				; Jump to INITIALIZE
RJMP PCINT					; Jump to PCINT routine

INITIALIZE:
	LDI R17, 0xFF			; Load 0xFF into R17
	OUT DDRB, R17			; Set PORTB as output
	OUT DDRD, R17			; Set PORTD as output
	LDI R17, 0x00			; Load 0x00 into R17
	OUT DDRC, R17			; Set PORTC as input
	CBI PORTB, 0			; Select Register 0 of the display

	CBI PORTB, 3			; Clear PortB, bit 3 to select PIA
	CBI PORTB, 4			; Clear PortB, bit 4 to select PIA

	SEI						; Enable external interrrupts
	LDI R17, 0x02			; Loads 2 into register 17
	STS 0x68, R17			; Store R17's value into memory to enable pcint
	LDI R17, 0x20			; Loads 0x20 into R17
	STS 0x6C, R17			; Store R17's value into memory to enable pcint

	LDI R16, 0x01			; Load 0x01 into R16 (Clear Display)
	RCALL OUTCOMM			; Call the OUTCOMM subroutine to output this command to the display

	LDI R16, 0x30			; Load 0x30 into R16 
	RCALL OUTCOMM			; Call the OUTCOMM subroutine

	LDI R16, 0x08			; Load 0x08 into R16
	RCALL OUTCOMM			; Call the OUTCOMM subroutine

	LDI R16, 0x06			; Load 0x06 into R16
	RCALL OUTCOMM			; Call the OUTCOMM subroutine

	LDI R16, 0x3C			; Load 0x3C into R16
	RCALL OUTCOMM			; Call the OUTCOMM subroutine

	LDI R16, 0x0F			; Load 0x0F into R16 (Turns on Display and Cursor Blink)
	RCALL OUTCOMM			; Call the OUTCOMM subroutine

	LDI R16, 0x02			; Load 0x02 into R16 (Cursor Home)
	RCALL OUTCOMM			; Call the OUTCOMM subroutine

FINISH:
	NOP						; Pause for one cycle
	RJMP FINISH				; Jump back to FINISH

;=========================================================================

OUTCOMM:
	CBI PORTB, 0			; Select Register 0 of the display
	LDI R17, 0x00			; Load 0x00 into R17
	OUT DDRD, R17			; Set PIND as input
	RCALL CHECKBUSY			; Call the checkbusy routine
	LDI R17, 0xFF			; Load 0xFF into R17
	OUT DDRD, R17			; Set PORTD as output
	OUT PORTD, R16			; OUTPUT the contents of R16 to PORTD
	RCALL EXEC				; Call the EXEC subroutine 
	RET						; Return to the origin of subroutine call

OUTCHAR:
	RCALL CHECKBUSY			; Call the CHECKBUSY subroutine
	CBI PORTB, 2			; Set E = 0
	CBI PORTB, 1			; Set R/!W = 0
	SBI PORTB, 0			; Set RS = 1
	OUT PORTD, R16			; Output R16 to PORTD
	RCALL EXEC				; Call the EXEC subroutine 
	RET						; Return to the origin of the subroutine call

CHECKBUSY:
	CBI PORTB, 2			; Set E = 0
	SBI PORTB, 1			; Set R/!W = 1
	CBI PORTB, 0			; Set RS = 0
	SBI PORTB, 2			; Set E = 1
	RCALL DELAY				; Call the DELAY subroutine
	CBI PORTB, 2			; Set E = 0
	CBI PORTB, 1			; Set R/!W = 0
	CBI PORTB, 0			; Set RS = 0
	RET						; Return to origin of subroutine call

DELAY:
	LDI R23, 1 				; Load 255 into R23
LP3:LDI R22, 255			; Load 255 into R22
LP2:LDI R21, 255			; Load 255 into R21
LP1:DEC R21					; Decrement R21
	BRNE LP1				; Branch to LP1 if R21 != 0
	DEC R22					; Decrement R22
	BRNE LP2				; Branch to LP2 if R22 != 0
	DEC R23					; Decrement R23
	BRNE LP3				; Branch to LP3 if R23 != 0
	RET						; Return to origin of subroutine calls

EXEC:
	SBI PORTB, 2			; Set E=1
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	CBI PORTB, 2			; Set E=0
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	RET						; Return to origin of subroutine call

NEWLINE:
	LDI R16, 0b00100000		; Load the value for " " into R16
	RCALL OUTCHAR			; Call the OUTCHAR subroutine to output this character to the display
	DEC R20					; Decrement R20
	BRNE NEWLINE			; If R20 != 0, call NEWLINE again
	RET						; Return to the origin of the subroutine call

REALCHECKBUSY:
	CBI PORTB, 2			; Set E = 0
	SBI PORTB, 1			; Set R/!W = 1
	CBI PORTB, 0			; Set RS = 0
	SBI PORTB, 2			; Set E = 1
CHK:NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	NOP						; Wait for one cycle
	SBIC PIND, 7			; Test PORTD Bit 7. If high, display is busy
	RJMP CHK				; Jump to REALCHECKBUSY
	CBI PORTB, 2			; Set E = 0
	RET						; Return to origin of subroutine call

PCINT:
	COM R25					; Complement the redundancy register
	SBRS R25, 0				; Skip next line if R25 bit 0 is set
	RETI					; Return from interrupt
	IN R16, PINC			; Put data from PINC into R16
	ANDI R16, 0x1F			; AND R16 with 0x1F
	CPI R16, 0x12			; Compare R16 with the hex value for the "shift" key
	BREQ SHIFT				; If R16 = Shift, branch to SHIFT routine
	CPI R16, 0x13			; Compare R16 with the hex value for the "mode" key
	BREQ MODE				; If R16 = Mode, branch to MODE routine
	RJMP CHARSELECT			; Jump to CHARSELECT

CHARSELECT:
	CPI R24, 0xFF			; Compare Shift Register to 0xFF
	BREQ SHIFTSELECT		; If Shift Register = 0xFF, branch to SHIFTSELECT
	LDI ZH, high(CHARTABLE*2)	; Set Z-pointer to CHARTABLE beginning
	LDI ZL, low(CHARTABLE*2)	; Set Z-pointer to CHARTABLE beginning
	ADD ZL, R16				; ADD R16 value to low z-pointer
	ADD ZL, R16				; ADD R16 value to low z-pointer again
	LPM						; Load the value pointed to by Z, store it in R0
	MOV R16, R0				; Load the value for R0 into R16
	RCALL OUTCHAR			; Call the OUTCHAR subroutine to output this character to the display
	RETI					; Return from the interrupt

SHIFTSELECT:
	LDI ZH, high(SHIFTTABLE*2)	; Set Z-pointer to CHARTABLE beginning
	LDI ZL, low(SHIFTTABLE*2)	; Set Z-pointer to CHARTABLE beginning
	ADD ZL, R16				; ADD R16 value to low z-pointer
	ADD ZL, R16				; ADD R16 value to low z-pointer again
	LPM						; Load the value pointed to by Z, store it in R0
	MOV R16, R0				; Load the value for R0 into R16
	RCALL OUTCHAR			; Call the OUTCHAR subroutine to output this character to the display
	RETI					; Return from the interrupt

SHIFT:
	COM R24					; Complement the R1 (shift) register
	RETI					; Return from the interrupt

MODE:
	RETI					; Return from the interrupt

.org 0x100					; Set origin to FLASH 0x100
CHARTABLE:
    .db 'A'					; Store value for 'A'
    .db 'B'					; Store value for 'B'
    .db 'C'					; Store value for 'C'
    .db 'D'					; Store value for 'D'
    .db 'E'					; Store value for 'E'
    .db 'F'					; Store value for 'F'
    .db 'G'					; Store value for 'G'
    .db 'H'					; Store value for 'H'
    .db 'I'					; Store value for 'I'
    .db 'J'					; Store value for 'J'
    .db 'K'					; Store value for 'K'
    .db 'L'					; Store value for 'L'
    .db 'M'					; Store value for 'M'
    .db 'N'					; Store value for 'N'
    .db 'O'					; Store value for 'O'
    .db 'P'					; Store value for 'P'
    .db 'Q'					; Store value for 'Q'
    .db 'R'					; Store value for 'R'
    RET					    ; Return to origin of subroutine call

.org 0x200                  ; Set origin to FLASH 0x200
SHIFTTABLE:
    .db 'S'					; Store value for 'S'
    .db 'T'					; Store value for 'T'
    .db 'U'					; Store value for 'U'
    .db 'V'					; Store value for 'V'
    .db 'W'					; Store value for 'W'
    .db 'X'					; Store value for 'X'
    .db 'Y'					; Store value for 'Y'
    .db 'Z'					; Store value for 'Z'
    .db '0'					; Store value for '0'
    .db '1'					; Store value for '1'
    .db '2'					; Store value for '2'
    .db '3'					; Store value for '3'
    .db '4'					; Store value for '4'
    .db '5'					; Store value for '5'
    .db '6'					; Store value for '6'
    .db '7'					; Store value for '7'
    .db '8'					; Store value for '8'
    .db '9'					; Store value for '9'
    RET						; Return to origin of subroutine call