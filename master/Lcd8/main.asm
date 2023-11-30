; AVR Assembly Program for SPI Communication and LCD Control

; Define hardware-specific constants
.equ sck=5         ; SPI Clock Pin for Master
.equ miso=4        ; SPI MISO (Master In Slave Out) Pin
.equ mosi=3        ; SPI MOSI (Master Out Slave In) Pin
.equ ss=2          ; SPI Slave Select Pin
.equ RS=2          ; LCD Register Select Bit
.equ RW=3          ; LCD Read/Write Bit
.equ E=4           ; LCD Enable Bit

.org 0             ; Start of the program

; Initialize Stack Pointer
ldi r16, high(RAMEND)  ; Set stack pointer high byte
out SPH, r16           ; Update Stack Pointer High
ldi r16, low(RAMEND)   ; Set stack pointer low byte
out SPL, r16           ; Update Stack Pointer Low

; Setup LCD Control Pins
ldi r16, (1 << RS) | (1 << RW) | (1 << E) ; Set RS, RW, E as output
out DDRC, r16          ; Configure Data Direction Register C for LCD

; Clear LCD Control Pins
CBI PORTC, RS          ; Clear Register Select (RS) pin
CBI PORTC, RW          ; Clear Read/Write (RW) pin
CBI PORTC, E           ; Clear Enable (E) pin

; Initialize LCD and SPI
RCALL CALL_3TIME       ; Call LCD initialization routine three times
RCALL INIT_LCD         ; Initialize LCD

rcall spi_init         ; Initialize SPI
rcall USART_Init       ; Initialize USART
rcall spi_init         ; Re-initialize SPI
ldi r16, $FF           ; Set PORTD as output
out DDRD, r16          ; Configure Data Direction Register D for output

; Setup pins for SPI communication
sbi PORTC, 0           ; Set pin to receive signal from slave
sbi PORTB, 2           ; Set pin for Slave Select (SS) to control slave

; Main Program Loop
loop:
    rcall spi_transmit; Call SPI transmit routine
	rcall Display_it  
    rjmp loop           ; Repeat indefinitely

; SPI Transmit Routine
spi_transmit:
    check: 
        sbic PINC, 0    ; Check if data is ready in SPDR of slave
        rjmp check
    cbi PORTB, 2       ; Set Slave Select (SS) to low, enabling slave to transmit data
    nop                ; No operation (timing adjustment)
    out SPDR, r16      ; Load data into SPI Data Register
    wait_transmit: 
        in r18, SPSR   ; Check if SPI transmission is complete
        sbrs r18, SPIF
        rjmp wait_transmit
    in r16, SPDR        ; Read data from SPI Data Register (data from slave)
    sbi PORTB, 2        ; Set Slave Select (SS) to high, disabling slave
    rcall USART_SendChar; Send data received from SPI to USART
	mov r20,r16
	ret
	Display_it:
    ; Display Data on LCD
    CBI PORTC, RS       ; Set RS for command mode (clear display)
    LDI R17, 0x01
    RCALL OUT_LCD
    LDI R16, 20
    RCALL DELAY_US      ; Delay for LCD processing

    mov r17, r20        ; Move received data to R17 for display
    sbi PORTC, RS       ; Set RS for data mode
    rcall OUT_LCD2      ; Output data to LCD
    ldi r16, 1
    RCALL DELAY_US      ; Short delay for LCD processing
    ret                 ; Return from routine


spi_init:
	ldi r16,(1<<sck)|(1<<ss)|(1<<mosi)|(0<<miso)
	out ddrb,r16
	ldi r16,(1<<SPE)|(1<<mstr)|(1<<spr0)
	out spcr,r16
	
	ret
USART_Init:
   ldi r16, 103           ; Set baud rate for 9600 bps with 1 MHz clock
    sts UBRR0L, r16       ; Set baud rate low byte
    ldi r16, (1<< U2X0)  ; Set double speed
    sts UCSR0A, r16
    ldi r16, (1 << UCSZ01) | (1 << UCSZ00) ; 8 data bits, no parity, 1 stop bit
    sts UCSR0C, r16
    ldi r16, (1 << RXEN0) | (1 << TXEN0)   ; Enable transmitter and receiver
    sts UCSR0B, r16
    ret

USART_SendChar:
    push r17
    USART_SendChar_Wait:
    lds r17, UCSR0A
    sbrs r17, UDRE0       ; Wait for data register to be empty 
    rjmp USART_SendChar_Wait
    sts UDR0, r16         ; Send character in r16
    pop r17
    ret

USART_ReceiveChar:
    push r17
    USART_ReceiveChar_Wait:
    lds r17, UCSR0A
    sbrs r17, RXC0        ; Wait for receive complete
    rjmp USART_ReceiveChar_Wait
    lds r16, UDR0         ; Store received character in r16
    pop r17
    ret
INIT_LCD:                       
				
				CBI PORTC,RS
				LDI	R17,0X02
				RCALL OUT_LCD  
				LDI R16,1
				RCALL DELAY_US
				CBI PORTC,RS
				LDI R17,0X0C
				RCALL OUT_LCD   
				LDI R16,1
				RCALL DELAY_US
				CBI PORTC,RS
				LDI R17,0X01
				RCALL OUT_LCD  
				LDI R16,20
				RCALL DELAY_US

				
				RET
CALL_3TIME: 
				LDI		R16,200				
				RCALL	DELAY_US		
			    CBI		PORTC,RS
				LDI		R17,$30		
				RCALL	OUT_LCD
				LDI		R16,42
				RCALL	DELAY_US

				CBI		PORTC,RS
				LDI		R17,$30			
				RCALL	OUT_LCD
				LDI		R16,2
				RCALL	DELAY_US

				CBI		PORTC,RS
				LDI		R17,$30
				RCALL	OUT_LCD
				LDI		R16,2
				RCALL	DELAY_US		
				RET


OUT_LCD2: 
				MOV		R21,R17
				ANDI    R21,$F0   
				OUT		PORTD,R21 
				SBI		PORTC,E   
				CBI		PORTC,E

				LDI		R16,1
				RCALL	DELAY_US

				SWAP	R17
				ANDI	R17,$F0
				OUT		PORTD,R17
				SBI		PORTC,E				
				CBI		PORTC,E
				LDI		R16,1
				RCALL	DELAY_US

				RET
				
OUT_LCD: 	
				MOV		R21,R17
				ANDI    R21,$F0  
				OUT		PORTD,R21 
				SBI		PORTC,E   
				CBI		PORTC,E

				LDI		R16,1
				RCALL	DELAY_US

				SWAP	R17
				ANDI	R17,$F0
				OUT		PORTD,R17
				SBI		PORTC,E				
				CBI		PORTC,E

				LDI		R16,1
				RCALL	DELAY_US
				RET

DELAY_US:	MOV	R15,R16		
			LDI	R16,200			
L1:			MOV	R14,R16		
L2:			DEC	R14				
			NOP					
			BRNE	L2		
			DEC		R15				
			BRNE	L1				
			RET