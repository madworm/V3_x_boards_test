/*
2009 - robert:aT:spitzenpfeil_d*t:org - V3_x board test - V5
*/

#define __spi_clock 13		// SCK - hardware SPI
#define __spi_data_in 12	// MISO - hardware SPI (unused)
#define __spi_data 11		// MOSI - hardware SPI
#define __spi_latch 10
#define __LATCH_LOW PORTB &= ~_BV(PB2)	// PB2 = Arduino Diecimila pin 10
#define __LATCH_HIGH PORTB |= _BV(PB2)	// PB2 = Arduino Diecimila pin 10

#define __display_enable 9
#define __DISPLAY_ON PORTB &= ~_BV(PB1)	// PB1 = Arduino Diecimila pin 9
#define __DISPLAY_OFF PORTB |= _BV(PB1)	// PB1 = Arduino Diecimila pin 9

#define __rows 8
#define __max_row __rows-1
#define __leds_per_row 8
#define __max_led __leds_per_row-1
#define __brightness_levels 32	// higher numbers at your own risk ;-)
#define __max_brightness __brightness_levels-1

#define __TRUE_RGB_OCR1A 0x0088	// 88 is suitable for 32 levels

#define __led_pin 4
#define __button_pin 8
#define PRESSED LOW

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>

uint8_t brightness_red[__rows][__leds_per_row];	/* memory for RED LEDs - valid values: 0 - __max_brightness */
uint8_t brightness_green[__rows][__leds_per_row];	/* memory for GREEN LEDs */
uint8_t brightness_blue[__rows][__leds_per_row];	/* memory for BLUE LEDs */

#define YES 1
#define NO 0
#define DOTCORR NO		/* enable/disable dot correction */

#if (DOTCORR == YES)
const int8_t PROGMEM dotcorr_red[__rows][__leds_per_row] =
    { {0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0}
};

const int8_t PROGMEM dotcorr_green[__rows][__leds_per_row] =
    { {0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0}
};

const int8_t PROGMEM dotcorr_blue[__rows][__leds_per_row] =
    { {0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0}
};

#define __fade_delay 0
#else
#define __fade_delay 4
#endif

void setup(void)
{
	randomSeed(555);
	pinMode(__spi_clock, OUTPUT);
	pinMode(__spi_latch, OUTPUT);
	pinMode(__spi_data, OUTPUT);
	pinMode(__spi_data_in, INPUT);
	pinMode(__display_enable, OUTPUT);
	pinMode(__button_pin, INPUT);
	digitalWrite(__button_pin, HIGH);	/* turn on pullup */
	pinMode(__led_pin, OUTPUT);
	digitalWrite(__spi_latch, LOW);
	digitalWrite(__spi_data, LOW);
	digitalWrite(__spi_clock, LOW);
	setup_hardware_spi();
	delay(10);
	set_matrix_rgb(0, 0, 0);	/* set the display to BLACK */
	setup_timer1_ctc();	/* enable the framebuffer display */
	Serial.begin(9600);
}

void loop(void)
{

	demo();
//demo_2();
//demo_3();

}

/*
other functions
*/

void blink_led(uint8_t times, uint8_t wait)
{
	uint8_t ctr;
	for (ctr = 0; ctr < times; ctr++) {
		digitalWrite(__led_pin, HIGH);
		delay(wait);
		digitalWrite(__led_pin, LOW);
		delay(wait);
	}
}

void random_leds(void)
{
	set_led_hue((uint8_t) (random(__rows)),
		    (uint8_t) (random(__leds_per_row)), (int)(random(360)));
}

void smile_on(int hue)
{				/* smily with open eyes */
	set_row_byte_hue(0, B00000000, hue);
	set_row_byte_hue(1, B01100110, hue);
	set_row_byte_hue(2, B01100110, hue);
	set_row_byte_hue(3, B00000000, hue);
	set_row_byte_hue(4, B00011000, hue);
	set_row_byte_hue(5, B10011001, hue);
	set_row_byte_hue(6, B01000010, hue);
	set_row_byte_hue(7, B00111100, hue);
}

void smile_off(uint16_t hue)
{				/* smily with closed eyes */
	set_row_byte_hue(0, B00000000, hue);
	set_row_byte_hue(1, B00000000, hue);
	set_row_byte_hue(2, B01100110, hue);
	set_row_byte_hue(3, B00000000, hue);
	set_row_byte_hue(4, B00011000, hue);
	set_row_byte_hue(5, B10011001, hue);
	set_row_byte_hue(6, B01000010, hue);
	set_row_byte_hue(7, B00111100, hue);
}

void smile_blink(uint16_t hue, uint8_t times, uint16_t pause)
{				/* blink a smily */
	uint8_t ctr;
	for (ctr = 0; ctr < times; ctr++) {
		delay(pause);
		smile_on(hue);
		delay(pause);
		smile_off(hue);
		delay(pause);
		smile_on(hue);
	}
}

void fader(void)
{				/* fade the matrix form BLACK to WHITE and back */
	uint8_t ctr1;
	uint8_t row;
	uint8_t led;

	for (ctr1 = 0; ctr1 <= __max_brightness; ctr1++) {
		for (row = 0; row <= __max_row; row++) {
			for (led = 0; led <= __max_led; led++) {
				set_led_rgb(row, led, ctr1, ctr1, ctr1);
			}
		}
		delay(__fade_delay);
	}

	for (ctr1 = __max_brightness; (ctr1 >= 0) & (ctr1 != 255); ctr1--) {
		for (row = 0; row <= __max_row; row++) {
			for (led = 0; led <= __max_led; led++) {
				set_led_rgb(row, led, ctr1, ctr1, ctr1);
			}
		}
		delay(__fade_delay);
	}
}

void fader_hue(void)
{				/* cycle the color of the whole matrix */
	uint16_t ctr1;
	for (ctr1 = 0; ctr1 < 360; ctr1 = ctr1 + 3) {
		set_matrix_hue(ctr1);
		delay(__fade_delay);
	}
}

void colors(void)
{				/* some diagonal color pattern */
	uint16_t ctr1;
	uint16_t ctr2;
	uint8_t row;
	uint8_t led;
	uint8_t tmp1;
	uint8_t tmp2;
	uint8_t tmp2_limit = 16;
	for (row = 0; row <= __max_row; row++) {
		for (led = 0; led <= __max_led; led++) {
			tmp1 = (led + 1) * (row + 1) / 2;
			if (tmp1 <= tmp2_limit) {
				tmp2 = tmp1;
			} else {
				tmp2 = tmp2_limit;
			}
			set_led_rgb(row, led, (led + 1) * (row + 1) / 2,
				    tmp2_limit - tmp2, 0);
		}
	}
}

/*
basic functions to set the LEDs
*/

void set_led_red(uint8_t row, uint8_t led, uint8_t red)
{
#if (DOTCORR == YES)
	int8_t dotcorr =
	    (int8_t) (pgm_read_byte(&dotcorr_red[row][led])) * red /
	    __brightness_levels;
	uint8_t value;
	if (red + dotcorr < 0) {
		value = 0;
	} else {
		value = red + dotcorr;
	}
	brightness_red[row][led] = value;
#else
	brightness_red[row][led] = red;
#endif
}

void set_led_green(uint8_t row, uint8_t led, uint8_t green)
{
#if (DOTCORR == YES)
	int8_t dotcorr =
	    (int8_t) (pgm_read_byte(&dotcorr_green[row][led])) * green /
	    __brightness_levels;
	uint8_t value;
	if (green + dotcorr < 0) {
		value = 0;
	} else {
		value = green + dotcorr;
	}
	brightness_green[row][led] = value;
#else
	brightness_green[row][led] = green;
#endif
}

void set_led_blue(uint8_t row, uint8_t led, uint8_t blue)
{
#if (DOTCORR == YES)
	int8_t dotcorr =
	    (int8_t) (pgm_read_byte(&dotcorr_blue[row][led])) * blue /
	    __brightness_levels;
	uint8_t value;
	if (blue + dotcorr < 0) {
		value = 0;
	} else {
		value = blue + dotcorr;
	}
	brightness_blue[row][led] = value;
#else
	brightness_blue[row][led] = blue;
#endif
}

void set_led_rgb(uint8_t row, uint8_t led, uint8_t red, uint8_t green,
		 uint8_t blue)
{
	set_led_red(row, led, red);
	set_led_green(row, led, green);
	set_led_blue(row, led, blue);
}

void set_matrix_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t ctr1;
	uint8_t ctr2;
	for (ctr2 = 0; ctr2 <= __max_row; ctr2++) {
		for (ctr1 = 0; ctr1 <= __max_led; ctr1++) {
			set_led_rgb(ctr2, ctr1, red, green, blue);
		}
	}
}

void set_row_rgb(uint8_t row, uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t ctr1;
	for (ctr1 = 0; ctr1 <= __max_led; ctr1++) {
		set_led_rgb(row, ctr1, red, green, blue);
	}
}

void set_column_rgb(uint8_t column, uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t ctr1;
	for (ctr1 = 0; ctr1 <= __max_row; ctr1++) {
		set_led_rgb(ctr1, column, red, green, blue);
	}
}

void set_row_hue(uint8_t row, uint16_t hue)
{
	uint8_t ctr1;
	for (ctr1 = 0; ctr1 <= __max_led; ctr1++) {
		set_led_hue(row, ctr1, hue);
	}
}

void set_column_hue(uint8_t column, uint16_t hue)
{
	uint8_t ctr1;
	for (ctr1 = 0; ctr1 <= __max_row; ctr1++) {
		set_led_hue(ctr1, column, hue);
	}
}

void set_matrix_hue(uint16_t hue)
{
	uint8_t ctr1;
	uint8_t ctr2;
	for (ctr2 = 0; ctr2 <= __max_row; ctr2++) {
		for (ctr1 = 0; ctr1 <= __max_led; ctr1++) {
			set_led_hue(ctr2, ctr1, hue);
		}
	}
}

void set_led_hue(uint8_t row, uint8_t led, uint16_t hue)
{
	// see wikipeda: HSV
	float S = 100.0, V = 100.0, s = S / 100.0, v =
	    V / 100.0, h_i, f, p, q, t, R, G, B;

	hue = hue % 360;
	h_i = hue / 60;
	f = (float)(hue) / 60.0 - h_i;
	p = v * (1 - s);
	q = v * (1 - s * f);
	t = v * (1 - s * (1 - f));

	if (h_i == 0) {
		R = v;
		G = t;
		B = p;
	} else if (h_i == 1) {
		R = q;
		G = v;
		B = p;
	} else if (h_i == 2) {
		R = p;
		G = v;
		B = t;
	} else if (h_i == 3) {
		R = p;
		G = q;
		B = v;
	} else if (h_i == 4) {
		R = t;
		G = p;
		B = v;
	} else {
		R = v;
		G = p;
		B = q;
	}

	set_led_rgb(row, led, (uint8_t) (R * (float)(__max_brightness)),
		    (uint8_t) (G * (float)(__max_brightness)),
		    (uint8_t) (B * (float)(__max_brightness)));
}

void set_row_byte_hue(uint8_t row, uint8_t data_byte, uint16_t hue)
{
	uint8_t led;
	for (led = 0; led <= __max_led; led++) {
		if (data_byte & _BV(led)) {
			set_led_hue(row, led, hue);
		} else {
			set_led_rgb(row, led, 0, 0, 0);
		}
	}
}

/* demo */
void demo(void)
{
	uint16_t ctr;
	for (ctr = 0; ctr < 200; ctr++) {
		random_leds();
		if (digitalRead(__button_pin) == PRESSED) {
			fader();
			blink_led(2, 50);
			colors();
			Serial.println("button pressed!");
			delay(1500);
		} else {
			blink_led(1, 10);
		}
	}
	smile_blink(200, 8, 100);
	delay(2500);
}

/* demo_2() */
void demo_2(void)
{
	uint8_t counter1;
	uint8_t counter2;

	for (counter1 = 0; counter1 <= 7; counter1++) {
		for (counter2 = 0; counter2 <= 7; counter2++) {
			set_led_rgb(counter1, counter2, 32, 0, 0);
			delay(10);
			set_led_rgb(counter1, counter2, 0, 32, 0);
			delay(10);
			set_led_rgb(counter1, counter2, 0, 0, 32);
			delay(10);
			set_led_rgb(counter1, counter2, 32, 32, 32);
			delay(10);
		}
	}
	set_matrix_rgb(0, 0, 0);

}

/* demo_3() */
void demo_3(void)
{
	static uint8_t counter = 0;

	if (digitalRead(__button_pin) == PRESSED) {
		counter++;
	}
	if (counter > 4) {
		counter = 0;
	}

	switch (counter) {
	case 0:
		set_matrix_rgb(5, 0, 0);
		break;
	case 1:
		set_matrix_rgb(0, 5, 0);
		break;
	case 2:
		set_matrix_rgb(0, 0, 5);
		break;
	case 3:
		set_matrix_rgb(255, 255, 255);
		break;
	case 4:
		set_matrix_rgb(0, 0, 0);
		break;
	default:
		break;
	}

	delay(250);

}

/*
Functions dealing with hardware specific jobs / settings
*/

void setup_hardware_spi(void)
{
	uint8_t clr;
	// spi prescaler:
	//
	// SPCR: SPR1 SPR0
	// SPSR: SPI2X
	//
	// SPI2X SPR1 SPR0
	//   0     0     0    fosc/4
	//   0     0     1    fosc/16
	//   0     1     0    fosc/64
	//   0     1     1    fosc/128
	//   1     0     0    fosc/2
	//   1     0     1    fosc/8
	//   1     1     0    fosc/32
	//   1     1     1    fosc/64

	/* enable SPI as master */
	SPCR |= (_BV(SPE) | _BV(MSTR));
	/* clear registers */
	clr = SPSR;
	clr = SPDR;
	/* set prescaler to fosc/2 */
	SPCR &= ~(_BV(SPR1) | _BV(SPR0));
	SPSR |= _BV(SPI2X);
}

void setup_timer1_ctc(void)
{
	// Arduino runs at 16 Mhz...
	// Timer1 (16bit) Settings:
	// prescaler (frequency divider) values:   CS12    CS11   CS10
	//                                           0       0      0    stopped
	//                                           0       0      1      /1  
	//                                           0       1      0      /8  
	//                                           0       1      1      /64
	//                                           1       0      0      /256 
	//                                           1       0      1      /1024
	//                                           1       1      0      external clock on T1 pin, falling edge
	//                                           1       1      1      external clock on T1 pin, rising edge
	//
	uint8_t _sreg = SREG;	/* save SREG */
	cli();			/* disable all interrupts while messing with the register setup */

	/* multiplexed TRUE-RGB PWM mode (quite dim) */
	/* set prescaler to 256 */
	TCCR1B |= (_BV(CS12));
	TCCR1B &= ~(_BV(CS10) | _BV(CS11));
	/* set WGM mode 4: CTC using OCR1A */
	TCCR1A &= ~(_BV(WGM10) | _BV(WGM11));
	TCCR1B |= _BV(WGM12);
	TCCR1B &= ~_BV(WGM13);
	/* normal operation - disconnect PWM pins */
	TCCR1A &= ~(_BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0));
	/* set top value for TCNT1 */
	OCR1A = __TRUE_RGB_OCR1A;
	/* enable COMPA isr */
	TIMSK1 |= _BV(OCIE1A);
	/* restore SREG with global interrupt flag */
	SREG = _sreg;
}

ISR(TIMER1_COMPA_vect)
{				/* Framebuffer interrupt routine */
	uint8_t pwm_cycle;
	static uint8_t row = 0;

	__LATCH_LOW;		// clear the display, so old (invalid) data doesn't show up from last run (ghost lines...)
	spi_transfer(0x00);
	spi_transfer(0xFF);
	spi_transfer(0xFF);
	spi_transfer(0xFF);
	//spi_transfer(0x00);
	__LATCH_HIGH;

	__DISPLAY_ON;		// only enable the drivers when we actually have time to talk to them

	for (pwm_cycle = 0; pwm_cycle <= __max_brightness; pwm_cycle++) {

		uint8_t led;
		uint8_t red = B11111111;	// off
		uint8_t green = B11111111;	// off
		uint8_t blue = B11111111;	// off

		for (led = 0; led <= __max_led; led++) {
			if (pwm_cycle < brightness_red[row][led]) {
				red &= ~_BV(led);
			}
			//else {
			//  red |= _BV(led);
			//}

			if (pwm_cycle < brightness_green[row][led]) {
				green &= ~_BV(led);
			}
			//else {
			//  green |= _BV(led);
			//}

			if (pwm_cycle < brightness_blue[row][led]) {
				blue &= ~_BV(led);
			}
			//else { 
			//  blue |= _BV(led);
			//}
		}

		__LATCH_LOW;
		spi_transfer(_BV(row));
		spi_transfer(blue);
		spi_transfer(green);
		spi_transfer(red);
		//spi_transfer( _BV(row) );
		__LATCH_HIGH;

	}

	__DISPLAY_OFF;		// we're done with this line, turn the driver's off until next time

	row++;			// next time the ISR runs, the next row will be dealt with

	if (row > __max_row) {
		row = 0;
	}

}

uint8_t spi_transfer(uint8_t data)
{
	SPDR = data;		// Start the transmission
	while (!(SPSR & _BV(SPIF)))	// Wait the end of the transmission
	{
	};
	return SPDR;		// return the received byte. (we don't need that here)
}
