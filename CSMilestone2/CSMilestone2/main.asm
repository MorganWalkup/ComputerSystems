;
; CSProject.asm
;
; Created: 2/27/2017 1:48:32 PM
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
	; Initialize registers
	LDI R17, 0xFF			; Load 0xFF into R17
	OUT DDRB, R17			; Set PORTB as output
	OUT DDRD, R17			; Set PORTD as output
	LDI R17, 0x00			; Load 0x00 into R17
	OUT DDRC, R17			; Set PORTC as input

	; Set up LCD
	SBI PORTB, 3			; Set PortB, bit 3 to select LCD
	SBI PORTB, 4			; Set PortB, bit 4 to select LCD
	CBI PORTB, 0			; Select Register 0 of the display

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

	; Set up pin change interrupts
	SEI						; Enable external interrrupts
	LDI R17, 0x02			; Loads 2 into register 17
	STS 0x68, R17			; Store R17's value into memory to enable pcint
	LDI R17, 0x20			; Loads 0x20 into R17
	STS 0x6C, R17			; Store R17's value into memory to enable pcint

PIAINIT:
	; Select PIA
	CBI PORTB, 3			; Clear PortB, bit 3 to select PIA
	CBI PORTB, 4			; Clear PortB, bit 4 to select PIA

	; Set Bit 2 of CRA to 0
	CBI PORTB, 1			; Clear RS1 to select CRA
	SBI PORTB, 0			; Set RS0 to select CRA with previous line
	LDI R18, 0x00			; Clears CRA bit 2, to access DDRA
	OUT PORTD, R18			; Writes the data to the PIA
	RCALL EXEC				; Execute Instruction
	; Set DDRA to 0xFF
	CBI PORTB, 0			; Clears RS0 select DDRA
	LDI R18, 0xFF			; Sets DDRA to output
	OUT PORTD, R18			; Writes the data to the PIA
	RCALL EXEC				; Execute Instruction
	; Set bit 2 of CRA to 1
	SBI PORTB, 0			; Set RS0 to select CRA again
	LDI R18, 0x04			; Sets CRA bit 2, to access DRA
	OUT PORTD, R18			; Writes the data to the PIA
	RCALL EXEC				; Execute Instruction

	; Set bit 2 of CRB to 0
	SBI PORTB, 1			; Set RS1 to select CRB
	SBI PORTB, 0			; Set RSO to select CRB with previous line
	LDI R18, 0x00			; Clears CRB bit 2, to access DDRB
	OUT PORTD, R18			; Writes the data to the PIA
	RCALL EXEC				; Execute Instruction
	; Set DDRB to 0xFF
	CBI PORTB, 0			; Clears RS0 to select DDRB
	LDI R18, 0xFF			; Sets DDRB to output
	OUT PORTD, R18			; Writes the data to the PIA
	RCALL EXEC				; Execute Instruction
	; Set bit 2 of CRB to 1
	SBI PORTB, 0			; Set RS0 to select CRB again
	LDI R18, 0x04			; Set bit 2 of CRB to select DRB
	OUT PORTD, R18			; Writes the data to the PIA
	RCALL EXEC				; Execute Instruction

 LASERTESTPATTERN:
	CBI PORTB, 0			; Selects Data Registers
	;(-10,-10)
	CBI PORTB, 1			; Selects DRA
	LDI R18, 0x00			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRA of the PIA
	RCALL EXEC				; Execute Instruction
	SBI PORTB, 1			; Selects DRB
	LDI R18, 0x00			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRB of the PIA
	RCALL EXEC				; Execute Instruction
	RCALL LASERDELAY		; Delay (So this change can be seen on the logic analyzer)

	;(-10,10)
	CBI PORTB, 1			; Selects DRA
	LDI R18, 0x00			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRA of the PIA
	RCALL EXEC				; Execute Instruction
	SBI PORTB, 1			; Selects DRB
	LDI R18, 0xFF			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRB of the PIA
	RCALL EXEC				; Execute Instruction
	RCALL LASERDELAY		; Delay (So this change can be seen on the logic analyzer)

	;(10,10)
	CBI PORTB, 1			; Selects DRA
	LDI R18, 0xFF			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRA of the PIA
	RCALL EXEC				; Execute Instruction
	SBI PORTB, 1			; Selects DRB
	LDI R18, 0xFF			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRB of the PIA
	RCALL EXEC				; Execute Instruction
	RCALL LASERDELAY		; Delay (So this change can be seen on the logic analyzer)

	;(10,-10)
	CBI PORTB, 1			; Selects DRA
	LDI R18, 0xFF			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRA of the PIA
	RCALL EXEC				; Execute Instruction
	SBI PORTB, 1			; Selects DRB
	LDI R18, 0x00			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRB of the PIA
	RCALL EXEC				; Execute Instruction
	RCALL LASERDELAY		; Delay (So this change can be seen on the logic analyzer)

	;(-10,-10)
	CBI PORTB, 1			; Selects DRA
	LDI R18, 0x00			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRA of the PIA
	RCALL EXEC				; Execute Instruction
	SBI PORTB, 1			; Selects DRB
	LDI R18, 0x00			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRB of the PIA
	RCALL EXEC				; Execute Instruction
	RCALL LASERDELAY		; Delay (So this change can be seen on the logic analyzer)
	
	;===========BEGIN STAR=================
	
	;(0,10)
	CBI PORTB, 1			; Selects DRA
	LDI R18, 0x80			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRA of the PIA
	RCALL EXEC				; Execute Instruction
	SBI PORTB, 1			; Selects DRB
	LDI R18, 0xFF			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRB of the PIA
	RCALL EXEC				; Execute Instruction
	RCALL LASERDELAY		; Delay (So this change can be seen on the logic analyzer)
	;(10,-10)
	CBI PORTB, 1			; Selects DRA
	LDI R18, 0xFF			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRA of the PIA
	RCALL EXEC				; Execute Instruction
	SBI PORTB, 1			; Selects DRB
	LDI R18, 0x00			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRB of the PIA
	RCALL EXEC				; Execute Instruction
	RCALL LASERDELAY		; Delay (So this change can be seen on the logic analyzer)*
	;(-10,2)
	CBI PORTB, 1			; Selects DRA
	LDI R18, 0x00			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRA of the PIA
	RCALL EXEC				; Execute Instruction
	SBI PORTB, 1			; Selects DRB
	LDI R18, 0xA0			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRB of the PIA
	RCALL EXEC				; Execute Instruction
	RCALL LASERDELAY		; Delay (So this change can be seen on the logic analyzer)*
	;(10,2)
	CBI PORTB, 1			; Selects DRA
	LDI R18, 0xFF			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRA of the PIA
	RCALL EXEC				; Execute Instruction
	SBI PORTB, 1			; Selects DRB
	LDI R18, 0xA0			; Loads value into R18
	OUT PORTD, R18			; Writes the data to DRB of the PIA
	RCALL EXEC				; Execute Instruction
	RCALL LASERDELAY		; Delay (So this change can be seen on the logic analyzer)*

	RJMP LASERTESTPATTERN	; Jump back to PIA test pattern beginning

FINISH:
	NOP						; Pause for one cycle
	RJMP FINISH				; Jump back to FINISH

;=========================================================================

OUTCOMM:
	SBI PORTB, 3			; Set PortB, bit 3 to select LCD
	SBI PORTB, 4			; Set PortB, bit 4 to select LCD
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
	SBI PORTB, 3			; Set PortB, bit 3 to select LCD
	SBI PORTB, 4			; Set PortB, bit 4 to select LCD
	RCALL CHECKBUSY			; Call the CHECKBUSY subroutine
	CBI PORTB, 2			; Set E = 0
	CBI PORTB, 1			; Set R/!W = 0
	SBI PORTB, 0			; Set RS = 1
	OUT PORTD, R16			; Output R16 to PORTD
	RCALL EXEC				; Call the EXEC subroutine 
	RET						; Return to the origin of the subroutine call

CHECKBUSY:
	SBI PORTB, 3			; Set PortB, bit 3 to select LCD
	SBI PORTB, 4			; Set PortB, bit 4 to select LCD
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
	LDI R23, 1				; Load 1 into R23
LP3:LDI R22, 255			; Load 255 into R22
LP2:LDI R21, 255			; Load 255 into R21
LP1:DEC R21					; Decrement R21
	BRNE LP1				; Branch to LP1 if R21 != 0
	DEC R22					; Decrement R22
	BRNE LP2				; Branch to LP2 if R22 != 0
	DEC R23					; Decrement R23
	BRNE LP3				; Branch to LP3 if R23 != 0
	RET						; Return to origin of subroutine calls

LASERDELAY:
	LDI R23, 2				; Load 2 into R23
LR3:LDI R22, 27				; Load 27 into R22
LR2:LDI R21, 255			; Load 255 into R21
LR1:DEC R21					; Decrement R21
	BRNE LR1				; Branch to LP1 if R21 != 0
	DEC R22					; Decrement R22
	BRNE LR2				; Branch to LP2 if R22 != 0
	DEC R23					; Decrement R23
	BRNE LR3				; Branch to LP3 if R23 != 0
	RET						; Return to origin of subroutine calls
	RET

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

.org 0x1000					; Set origin to FLASH 0x100
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

.org 0x1100                  ; Set origin to FLASH 0x200
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

.org 0x1200
TABLEA:
	.db 0x02, 0x40, 0x1E, 0xC0, 0x3D, 0x40, 0x00, 0xFF, 0x10, 0x82, 0x2D, 0x82 
.org 0x1300
TABLEB:
	.db 0x02, 0x40, 0x02, 0xC0, 0x28, 0xC0, 0x3D, 0xAA, 0x3D, 0x96, 0x28, 0x82, 0x02, 0x82, 0x28, 0x82, 0x3D, 0x6E, 0x3D, 0x50, 0x28, 0x40, 0x02, 0x40
.org 0x1400
TABLEC:
	.db 0x3D, 0xA0, 0x1D, 0xC0, 0x02, 0xA0, 0x02, 0x58, 0x1F, 0x40, 0x3D, 0x58
.org 0x1500
TABLED:
	.db 0x02, 0x40, 0x02, 0xC1, 0x28, 0xC0, 0x3D, 0x96, 0x3D, 0x6E, 0x28, 0x40, 0x02, 0x40
.org 0x1600
TABLEE:
	.db 0x3D, 0xC0, 0x02, 0xC0, 0x02, 0x40, 0x3D, 0x40, 0x02, 0x40, 0x02, 0x82, 0x28, 0x82
.org 0x1700
TABLEF:
	.db 0x3D, 0xC0, 0x02, 0xC0, 0x02, 0x40, 0x02, 0x82, 0x28, 0x82
.org 0x1800
TABLEG:
	.db 0x3E, 0xA4, 0x33, 0xBF, 0x0D, 0xC0, 0x02, 0xA3, 0x02, 0x5A, 0x0D, 0x41, 0x32, 0x41, 0x3E, 0x58, 0x3E, 0x7C, 0x24, 0x7B
.org 0x1900
TABLEH:
	.db 0x02, 0xC0, 0x02, 0x41, 0x02, 0x82, 0x3E, 0x82, 0x3E, 0xC0, 0x3E, 0x40
.org 0x2000
TABLEI:
	.db 0x02, 0xC0, 0x3E, 0xC0, 0x1F, 0xC0, 0x1F, 0x40, 0x02, 0x40, 0x3E, 0x40
.org 0x2100
TABLEJ:
	.db 0x3E, 0xC0, 0x3E, 0x6F, 0x21, 0x40, 0x02, 0x6F
.org 0x2200
TABLEK:
	.db 0x02, 0xC0, 0x02, 0x82, 0x3E, 0xC0, 0x02, 0x82, 0x3E, 0x40, 0x02, 0x82, 0x02, 0x40
.org 0x2300
TABLEL:
	.db 0x02, 0xC0, 0x02, 0x40, 0x3E, 0x40
.org 0x2400
TABLEM:
	.db 0x02, 0x40, 0x11, 0xC0, 0x20, 0x40, 0x2E, 0xBF, 0x3E, 0x40
.org 0x2500
TABLEN:
	.db 0x02, 0x40, 0x02, 0xC0, 0x3E, 0x40, 0x3E, 0xC0
.org 0x2600
TABLEO:
	.db 0x14, 0xC0, 0x02, 0xA0, 0x02, 0x5C, 0x14, 0x40, 0x29, 0x40, 0x3E, 0x5B, 0x3E, 0xA0, 0x29, 0xC0, 0x14, 0xC0
.org 0x2700
TABLEP:
	.db 0x02, 0x40, 0x02, 0xC0, 0x3E, 0xC0, 0x3E, 0x82, 0x02, 0x82
.org 0x2800
TABLEQ:
	.db 0x14, 0xC0, 0x02, 0xA0, 0x02, 0x5C, 0x14, 0x40, 0x29, 0x40, 0x3E, 0x5B, 0x3E, 0xA0, 0x29, 0xC0, 0x14, 0xC0, 0x00, 0xFF, 0x28, 0x5A, 0x3E, 0x40
.org 0x2900
TABLER:
	.db 0x02, 0x40, 0x02, 0xC0, 0x3E, 0xC0, 0x3E, 0x82, 0x02, 0x82, 0x3E, 0x40
.org 0x3000
TABLES:
	.db 0x3E, 0xC0, 0x02, 0xC0, 0x02, 0x82, 0x3E, 0x82, 0x3E, 0x40, 0x02, 0x40
.org 0x3100
TABLET:
	.db 0x21, 0x40, 0x21, 0xC0, 0x02, 0xC0, 0x3E, 0xC0
.org 0x3200
TABLEU:
	.db 0x02, 0xC0, 0x02, 0x40, 0x3E, 0x40, 0x3E, 0xC0
.org 0x3300
TABLEV:
	.db 0x02, 0xC0, 0x20, 0x40, 0x3E, 0xC0
.org 0x3400
TABLEW:
	.db 0x02, 0xC0, 0x14, 0x40, 0x21, 0x9B, 0x2D, 0x40, 0x3E, 0xC0
.org 0x3500
TABLEX:
	.db 0x02, 0xC0, 0x3E, 0x40, 0x00, 0xFF, 0x02, 0x40, 0x3E, 0xC0
.org 0x3600
TABLEY:
	.db 0x02, 0xC0, 0x20, 0x82, 0x20, 0x40, 0x20, 0x82, 0x3E, 0xC0
.org 0x3700
TABLEZ:
	.db 0x02, 0xC0, 0x3E, 0xC0, 0x02, 0x40, 0x3E, 0x40
.org 0x3800
TABLE0:
	.db 0x14, 0xC0, 0x02, 0xA0, 0x02, 0x5C, 0x14, 0x40, 0x29, 0x40, 0x3E, 0x5B, 0x3E, 0xA0, 0x29, 0xC0, 0x14, 0xC0
.org 0x3900
TABLE1:
	.db 0x1F, 0xC0, 0x1F, 0x40
.org 0x4000
TABLE2:
	.db 0x02, 0xC0, 0x3E, 0xC0, 0x3E, 0x82, 0x02, 0x82, 0x02, 0x40, 0x3E, 0x40
.org 0x4100
TABLE3:
	.db 0x02, 0xC0, 0x3E, 0xC0, 0x3E, 0x82, 0x14, 0x82, 0x3E, 0x82, 0x3E, 0x40, 0x02, 0x40
.org 0x4200
TABLE4:
	.db 0x3E, 0x6E, 0x02, 0x6E, 0x36, 0xC0, 0x36, 0x40
.org 0x4300
TABLE5:
	.db 0x3E, 0xC0, 0x02, 0xC0, 0x02, 0x89, 0x3E, 0x89, 0x3E, 0x5D, 0x31, 0x40, 0x11, 0x40, 0x02, 0x5C
.org 0x4400
TABLE6:
	.db 0x3E, 0xC0, 0x02, 0xC0, 0x02, 0x40, 0x3E, 0x40, 0x3E, 0x82, 0x02, 0x82
.org 0x4500
TABLE7:
	.db 0x02, 0xC0, 0x3E, 0xC0, 0x02, 0x40
.org 0x4600
TABLE8:
	.db 0x02, 0x82, 0x02, 0x57, 0x0F, 0x40, 0x32, 0x40, 0x3E, 0x58, 0x3E, 0xAA, 0x33, 0xC0, 0x0F, 0xC0, 0x02, 0xAA, 0x02, 0x82, 0x3E, 0x82
.org 0x4700
TABLE9:
	.db 0x3E, 0x40, 0x3E, 0xC0, 0x02, 0xC0, 0x02, 0x82, 0x3E, 0x82