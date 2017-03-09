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

;==================================================================================
; INITIALIZE
;==================================================================================
INITIALIZE:
	; Initialize registers
	LDI R17, 0xFF			; Load 0xFF into R17
	OUT DDRB, R17			; Set PORTB as output
	OUT DDRD, R17			; Set PORTD as output
	LDI R17, 0x00			; Load 0x00 into R17
	OUT DDRC, R17			; Set PORTC as input
	CLR R24					; Clear SHIFT Register
	CLR R26					; Clear MODE Register
	CLR R19					; Clear LaserTemp Register
	CLR R1					; Clear Char1L Register
	CLR R2					; Clear Char1H Register
	CLR R3
	CLR R4
	CLR R5
	CLR R6
	CLR R7
	CLR R8

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

FINISH:
	; Call particular routines based on MODE register
	CPI R26, 0				; Compare MODE register with zero
	BREQ KEYFINISH			; Branch to KEYFINISH if MODE = 0
	CPI R26, 1				; Compare MODE register with one
	BREQ TESTFINISH			; Branch to TESTFINISH if MODE = 1
	CPI R26, 2				; Compare MODE register with two
	BREQ LASERFINISH		; Branch to LASERFINISH if MODE = 2
	CPI R26, 3				; Compare MODE register with three
	BREQ ANIMFINISH			; Branch to ANIMFINISH if MODE = 3
	RJMP FINISH				; Jump back to start of FINISH

KEYFINISH:
	; Select LCD
	SBI PORTB, 3			; Set PortB, bit 3 to select LCD
	SBI PORTB, 4			; Set PortB, bit 4 to select LCD
	RJMP FINISH				; Jump back to FINISH

TESTFINISH:
	; Select PIA
	CBI PORTB, 3			; Clear PortB, bit 3 to select PIA
	CBI PORTB, 4			; Clear PortB, bit 4 to select PIA
	; Initialize z-pointer
	LDI ZH, high(TABLESTAR*2) ; Set z-pointer to TABLESTAR beginning
	LDI ZL, low(TABLESTAR*2)  ; Set z-pointer to TABLESTAR beginning
	; Output coordinates
	CLR R19					; Clear the laser offset register
	RCALL LASERTESTPATTERN	; Call the laser test pattern subroutine
	; Jump back
	RJMP FINISH				; Jump back to FINISH

LASERFINISH:
	; Select PIA
	CBI PORTB, 3			; Clear PortB, bit 3 to select PIA
	CBI PORTB, 4			; Clear PortB, bit 4 to select PIA
	; Initialize Laser Character Registers
	LDI R19, high(TABLEA*2) ; Load TABLEA beginning address into R19  
	MOV R2, R19			    ; Set Char1L register to TABLEA beginning
	LDI R19, low(TABLEA*2)  ; Load TABLEA beginning address into R19
	MOV R1, R19			    ; Set Char1H to TABLEA beginning
	LDI R19, high(TABLE1*2) ; Load TABLEA beginning address into R19  
	MOV R4, R19			    ; Set Char1L register to TABLEA beginning
	LDI R19, low(TABLE1*2)  ; Load TABLEA beginning address into R19
	MOV R3, R19			    ; Set Char1H to TABLEA beginning
	LDI R19, high(TABLE2*2) ; Load TABLEA beginning address into R19  
	MOV R6, R19			    ; Set Char1L register to TABLEA beginning
	LDI R19, low(TABLE2*2)  ; Load TABLEA beginning address into R19
	MOV R5, R19			    ; Set Char1H to TABLEA beginning
	LDI R19, high(TABLE3*2) ; Load TABLEA beginning address into R19  
	MOV R8, R19			    ; Set Char1L register to TABLEA beginning
	LDI R19, low(TABLE3*2)  ; Load TABLEA beginning address into R19
	MOV R7, R19			    ; Set Char1H to TABLEA beginning
	CLR R19					; Clear the R19 register
	; Initialize z-pointer for first character
	MOV ZH, R2				; Set z-pointer to TABLEA beginning
	MOV ZL, R1				; Set z-pointer to TABLEA beginning
	; Output coordinates for first character
	CLR R19					; Clear the laser offset register
	RCALL LASERTESTPATTERN	; Call the laser test pattern subroutine
	; Initialize z-pointer for second character
	MOV ZH, R4				; Set z-pointer to TABLEA beginning
	MOV ZL, R3				; Set z-pointer to TABLEA beginning
	; Output coordinates for second character
	LDI R19, 0x40			; Set the laser offset for the second character
	RCALL LASERTESTPATTERN	; Call the laser test pattern subroutine
	; Initialize z-pointer for third character
	MOV ZH, R6				; Set z-pointer to TABLEA beginning
	MOV ZL, R5				; Set z-pointer to TABLEA beginning
	; Output coordinates for third character
	LDI R19, 0x80			; Set the laser offset for the third character
	RCALL LASERTESTPATTERN	; Call the laser test pattern subroutine
	; Initialize z-pointer for fourth character
	MOV ZH, R8				; Set z-pointer to TABLEA beginning
	MOV ZL, R7				; Set z-pointer to TABLEA beginning
	; Output coordinates for fourth  character
	LDI R19, 0xC0			; Set the laser offset for the fourth character
	RCALL LASERTESTPATTERN	; Call the laser test pattern subroutine
	; Jump back
	RJMP FINISH				; Jump back to FINISH

ANIMFINISH:
	; Select PIA
	CBI PORTB, 3			; Clear PortB, bit 3 to select PIA
	CBI PORTB, 4			; Clear PortB, bit 4 to select PIA
	;RCALL LASERANIMATION	; Call the Laser Animation subroutine
	RJMP FINISH				; Jump back to FINISH

;==================================================================================
; FUNCTIONS
;==================================================================================

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
	
LASERTESTPATTERN:
	; X-Coordinate
	LPM						; Load the value pointed to by Z, store it in R0
	MOV R16, R0				; Load the value for R0 into R16
	CPI R16, 0x11			; Compares the value in R16 to "end of table" code
	BREQ LASERTESTEND		; If end of table, branch to LASERTESTEND
	ADD R16, R19			; Add the laser offset value to the x-coordinate
	CBI PORTB, 0			; Selects Data Registers
	CBI PORTB, 1			; Selects DRA
	OUT PORTD, R16			; Writes the data to DRA of the PIA
	RCALL EXEC				; Executes Instruction 
	INC ZL					; Increments the z-pointer to the next point in the table
	; Y-Coordinate
	LPM						; Load the value pointed to by Z, store it in R0
	MOV R16, R0				; Load the value for R0 into R16
	CPI R16, 0x11			; Compares the value in R16 to "end of table" code
	BREQ LASERTESTEND		; If end of table, branch to LASERTESTEND
	CBI PORTB, 0			; Selects Data Registers
	SBI PORTB, 1			; Selects DRB
	OUT PORTD, R16			; Writes the data to DRB of the PIA
	RCALL EXEC				; Execute Instruction	
	INC ZL
	RCALL LASERDELAY		; Delay so the mirrors of the laser can catch up
	RJMP LASERTESTPATTERN	; Jump back to LASERTESTPATTERN	
LASERTESTEND:
	RET						; Return to where it was called
	
PCINT:
	COM R25					; Complement the redundancy register
	SBRS R25, 0				; Skip next line if R25 bit 0 is set
	RETI					; Return from interrupt
	IN R16, PINC			; Put data from PINC into R16
	ANDI R16, 0x1F			; AND R16 with 0x1F
	CPI R16, 0x13			; Compare R16 with the hex value for the "mode" key
	BREQ MODE				; If R16 = Mode, branch to MODE routine
	CPI R26, 0				; Compare MODE register to 0
	BRNE DUMBRET			; Branch to DUMBRET if MODE != 0			
	CPI R16, 0x12			; Compare R16 with the hex value for the "shift" key
	BREQ SHIFT				; If R16 = Shift, branch to SHIFT routine
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
	CPI R26, 3				; Compare R26 to 3
	BREQ RESETMODE			; Branch to RESETMODE if MODE = 3
	INC R26					; Increment the MODE register
	RETI					; Return from the interrupt

RESETMODE:
	CLR R26					; Clear the MODE register
	RETI					; Return from the interrupt

DUMBRET:
	RETI					; Return from the interrupt

;===================================================================================================
;TABLES
;===================================================================================================

.org 0x1000					; Set origin to FLASH 0x1000
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

.org 0x1100                  ; Set origin to FLASH 0x1100
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
	.db 0x02, 0x40, 0x1E, 0xC0, 0x3D, 0x40, 0x00, 0xFF, 0x10, 0x82, 0x2D, 0x82, 0x11
.org 0x1240
TABLEB:
	.db 0x02, 0x40, 0x02, 0xC0, 0x28, 0xC0, 0x3D, 0xAA, 0x3D, 0x96, 0x28, 0x82, 0x02, 0x82, 0x28, 0x82, 0x3D, 0x6E, 0x3D, 0x50, 0x28, 0x40, 0x02, 0x40, 0x11
.org 0x1280
TABLEC:
	.db 0x3D, 0xA0, 0x1D, 0xC0, 0x02, 0xA0, 0x02, 0x58, 0x1F, 0x40, 0x3D, 0x58, 0x11
.org 0x12C0
TABLED:
	.db 0x02, 0x40, 0x02, 0xC1, 0x28, 0xC0, 0x3D, 0x96, 0x3D, 0x6E, 0x28, 0x40, 0x02, 0x40, 0x11
.org 0x1300
TABLEE:
	.db 0x3D, 0xC0, 0x02, 0xC0, 0x02, 0x40, 0x3D, 0x40, 0x02, 0x40, 0x02, 0x82, 0x28, 0x82, 0x11
.org 0x1340
TABLEF:
	.db 0x3D, 0xC0, 0x02, 0xC0, 0x02, 0x40, 0x02, 0x82, 0x28, 0x82, 0x11
.org 0x1480
TABLEG:
	.db 0x3E, 0xA4, 0x33, 0xBF, 0x0D, 0xC0, 0x02, 0xA3, 0x02, 0x5A, 0x0D, 0x41, 0x32, 0x41, 0x3E, 0x58, 0x3E, 0x7C, 0x24, 0x7B, 0x11
.org 0x14C0
TABLEH:
	.db 0x02, 0xC0, 0x02, 0x41, 0x02, 0x82, 0x3E, 0x82, 0x3E, 0xC0, 0x3E, 0x40, 0x11
.org 0x1500
TABLEI:
	.db 0x02, 0xC0, 0x3E, 0xC0, 0x1F, 0xC0, 0x1F, 0x40, 0x02, 0x40, 0x3E, 0x40, 0x11
.org 0x1540
TABLEJ:
	.db 0x3E, 0xC0, 0x3E, 0x6F, 0x21, 0x40, 0x02, 0x6F, 0x11
.org 0x1580
TABLEK:
	.db 0x02, 0xC0, 0x02, 0x82, 0x3E, 0xC0, 0x02, 0x82, 0x3E, 0x40, 0x02, 0x82, 0x02, 0x40, 0x11
.org 0x15C0
TABLEL:
	.db 0x02, 0xC0, 0x02, 0x40, 0x3E, 0x40, 0x11
.org 0x1600
TABLEM:
	.db 0x02, 0x40, 0x11, 0xC0, 0x20, 0x40, 0x2E, 0xBF, 0x3E, 0x40, 0x11
.org 0x1640
TABLEN:
	.db 0x02, 0x40, 0x02, 0xC0, 0x3E, 0x40, 0x3E, 0xC0, 0x11
.org 0x1680
TABLEO:
	.db 0x14, 0xC0, 0x02, 0xA0, 0x02, 0x5C, 0x14, 0x40, 0x29, 0x40, 0x3E, 0x5B, 0x3E, 0xA0, 0x29, 0xC0, 0x14, 0xC0, 0x11
.org 0x16C0
TABLEP:
	.db 0x02, 0x40, 0x02, 0xC0, 0x3E, 0xC0, 0x3E, 0x82, 0x02, 0x82, 0x11
.org 0x1700
TABLEQ:
	.db 0x14, 0xC0, 0x02, 0xA0, 0x02, 0x5C, 0x14, 0x40, 0x29, 0x40, 0x3E, 0x5B, 0x3E, 0xA0, 0x29, 0xC0, 0x14, 0xC0, 0x00, 0xFF, 0x28, 0x5A, 0x3E, 0x40, 0x11
.org 0x1740
TABLER:
	.db 0x02, 0x40, 0x02, 0xC0, 0x3E, 0xC0, 0x3E, 0x82, 0x02, 0x82, 0x3E, 0x40, 0x11
.org 0x1780
TABLES:
	.db 0x3E, 0xC0, 0x02, 0xC0, 0x02, 0x82, 0x3E, 0x82, 0x3E, 0x40, 0x02, 0x40, 0x11
.org 0x17C0
TABLET:
	.db 0x21, 0x40, 0x21, 0xC0, 0x02, 0xC0, 0x3E, 0xC0, 0x11
.org 0x1800
TABLEU:
	.db 0x02, 0xC0, 0x02, 0x40, 0x3E, 0x40, 0x3E, 0xC0, 0x11
.org 0x1840
TABLEV:
	.db 0x02, 0xC0, 0x20, 0x40, 0x3E, 0xC0, 0x11
.org 0x1880
TABLEW:
	.db 0x02, 0xC0, 0x14, 0x40, 0x21, 0x9B, 0x2D, 0x40, 0x3E, 0xC0, 0x11
.org 0x18C0
TABLEX:
	.db 0x02, 0xC0, 0x3E, 0x40, 0x00, 0xFF, 0x02, 0x40, 0x3E, 0xC0, 0x11
.org 0x1900
TABLEY:
	.db 0x02, 0xC0, 0x20, 0x82, 0x20, 0x40, 0x20, 0x82, 0x3E, 0xC0, 0x11
.org 0x1940
TABLEZ:
	.db 0x02, 0xC0, 0x3E, 0xC0, 0x02, 0x40, 0x3E, 0x40, 0x11
.org 0x1980
TABLE0:
	.db 0x14, 0xC0, 0x02, 0xA0, 0x02, 0x5C, 0x14, 0x40, 0x29, 0x40, 0x3E, 0x5B, 0x3E, 0xA0, 0x29, 0xC0, 0x14, 0xC0, 0x11
.org 0x19C0
TABLE1:
	.db 0x1F, 0xC0, 0x1F, 0x40, 0x11
.org 0x2000
TABLE2:
	.db 0x02, 0xC0, 0x3E, 0xC0, 0x3E, 0x82, 0x02, 0x82, 0x02, 0x40, 0x3E, 0x40, 0x11
.org 0x2040
TABLE3:
	.db 0x02, 0xC0, 0x3E, 0xC0, 0x3E, 0x82, 0x14, 0x82, 0x3E, 0x82, 0x3E, 0x40, 0x02, 0x40, 0x11
.org 0x2080
TABLE4:
	.db 0x3E, 0x6E, 0x02, 0x6E, 0x36, 0xC0, 0x36, 0x40, 0x11
.org 0x20C0
TABLE5:
	.db 0x3E, 0xC0, 0x02, 0xC0, 0x02, 0x89, 0x3E, 0x89, 0x3E, 0x5D, 0x31, 0x40, 0x11, 0x40, 0x02, 0x5C, 0x11
.org 0x2100
TABLE6:
	.db 0x3E, 0xC0, 0x02, 0xC0, 0x02, 0x40, 0x3E, 0x40, 0x3E, 0x82, 0x02, 0x82, 0x11
.org 0x2140
TABLE7:
	.db 0x02, 0xC0, 0x3E, 0xC0, 0x02, 0x40, 0x11
.org 0x2180
TABLE8:
	.db 0x02, 0x82, 0x02, 0x57, 0x0F, 0x40, 0x32, 0x40, 0x3E, 0x58, 0x3E, 0xAA, 0x33, 0xC0, 0x0F, 0xC0, 0x02, 0xAA, 0x02, 0x82, 0x3E, 0x82, 0x11
.org 0x21C0
TABLE9:
	.db 0x3E, 0x40, 0x3E, 0xC0, 0x02, 0xC0, 0x02, 0x82, 0x3E, 0x82, 0x11
.org 0x2200
TABLESTAR: ; Character table for the test pattern (square for now)
	.db 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x11