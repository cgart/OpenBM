
#define F_CPU 1000000UL // Frquency in Hz

#include <avr/io.h>
#include <inttypes.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/pgmspace.h>

int main(void)
{
	/*--------------------------------------
	* Test PWM hardware output
	* 
		Falstad: Schematics
		$ 1 5.0E-6 38.696464541249114 50 5.0 50
		O 368 144 480 144 1
		g 368 272 368 304 0
		c 368 208 368 272 0 3.3E-7 0.16999210928700548
		R 368 32 448 32 0 0 40.0 5.0 0.0 0.0 0.5
		r 368 144 368 208 0 10.0
		t 160 160 224 160 0 -1 4.999999988999999 4.819753085890323 100.0
		g 224 176 224 304 0
		r 368 144 368 32 0 4700.0
		r 160 160 96 160 0 220000.0
		R 96 160 48 160 0 2 15625.0 5.0 0.0 0.0 0.5
		w 368 144 224 144 0
		o 0 32 0 34 1.25 9.765625E-5 0 -1

	*--------------------------------------*/
	DDRB = (1 << PB3); // OC2 as output
	
	TCCR2  = (1 << WGM21) | (1 << WGM20); 	// Fast PWM 
	TCCR2 |= (1 << COM21) | (1 << COM20); 	// inverting mode
	TCCR2 |= (1 << CS21) | (1 << CS20);		// 1/32 prescaler (max 15.6KHz)

	OCR2 = 0x7F;  // 50% duty cycle
	
	/* ----------------------------------
	* Peform a clock on a PIN, can be used to reactive falsly set fusebits on other atmegas
	*------------------------------------*/
	DDRD = 0xFF;
	while (1)
	{
		PORTD = ~PIND;
		//nop();  // added some nop, to slow the clock a bit
		//nop();
		//_delay_ms(10);
	}
	
	
}
