.equ sck=5    ;set up pins for spi 
.equ miso=4
.equ mosi=3
.equ ss=2
.org 0
ldi r16, high(RAMEND)
out SPH, r16
ldi r16, low(RAMEND)
out SPL, r16
rcall spi_init

ldi r16,$0f
out ddrd,r16 ; 0-3 is output 4-7 is input pull-up 
ldi r16,$f0  ; here we set up for keypad
out portd,r16


sbi ddrc,0   ; we set up pin that master can know to set pin ss to low
sbi portc,0

main:
		call	keypad_scan
	
		
		jmp		main
	keypad_scan:
		

		ldi r22, 0b11111110 ; initial row mask
		ldi r23, 0 ; initial pressed row value
		ldi	r24,0	;scanning col index

	keypad_scan_loop:
		out PORTD, r22 ; scan current row
		nop				;need to have 1us delay to stablize
		sbic PIND, 4 ; check col 0

		rjmp keypad_scan_check_col1
debau1:		sbis pind,4
				rjmp debau1
		rjmp keypad_scan_found ; col 0 is pressed
	keypad_scan_check_col1:
		sbic PIND, 5 ; check col 1
		rjmp keypad_scan_check_col2
		debau2:		sbis pind,5
				rjmp debau2
		ldi r23, 1 ; col1 is pressed
		rjmp keypad_scan_found
	keypad_scan_check_col2:
		sbic PIND, 6 ; check row 2
		
		rjmp keypad_scan_check_col3
		debau3:		sbis pind,6
				rjmp debau3
		ldi r23, 2 ; col 2 is pressed
		rjmp keypad_scan_found
	keypad_scan_check_col3:
		sbic PIND, 7 ; check col 3
		rjmp keypad_scan_next_row
		debau7:		sbis pind,7
				rjmp debau7
		ldi r23, 3 ; col 3 is pressed
		rjmp keypad_scan_found

	keypad_scan_next_row:
		; check if all rows have been scanned
		cpi r24,3
		breq keypad_scan_not_found

		; shift row mask to scan next row
		sec	
		rol r22
		inc	r24		;increase row index
		rjmp keypad_scan_loop

	keypad_scan_found:
		; combine row and column to get key value (0-15)
		;key code = row*4 + col
		lsl r24 ; shift row value 4 bits to the left
		lsl	r24
		add r23, r24 ; add row value to column value
		
		ldi r17,$30
		add r17,r23
		call spi_transmit
		call DELAY
	

		rjmp keypad_scan
		ret
	keypad_scan_not_found:
		ldi r23,1 ; no key pressed
		ret
spi_transmit:
	cbi portc,0     ; we send signal to master
	nop             ; and master will set pin ss to low
	nop             ; and slave can transfer data to master
	nop
	nop
	out spdr,r17

	wait_transmit:  ; code here we check data is transfer from slave to master
	in r18,spsr     ; it is finished or not
	sbrs r18,spif
	rjmp wait_transmit
	in r17,spdr
	sbi portc,0
	ret


spi_init:
	ldi r16,(1<<miso)|(0<<sck)|(0<<ss)|(0<<mosi)
	out ddrb,r16
	ldi r16,(1<<SPE)|(1<<spr0)
	out spcr,r16
	;ldi r16,(1<<spi2x0)
	;sts spsr0,r16
	ret
DELAY:
L3: LDI R21,100 ;1MC
L1: LDI R20,200 ;1MC
L2: DEC R20 ;1MC
NOP ;1MC
BRNE L2 ;2/1MC
DEC R21 ;1MC
BRNE L1 ;2/1MC
RET
%git clone https://github.com/Kysubi/slave.github.io.git
