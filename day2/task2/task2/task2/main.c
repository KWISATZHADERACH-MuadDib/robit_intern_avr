#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "i2c_lcd.h"
#include "adc.h"

int main(void)
{
	char buf[17];

	lcd_init();
	adc_init();

	lcd_set_cursor(0, 0);
	lcd_print("21th_YHW");

	while (1)
	{
		uint16_t adc_value = adc_read(0);

		unsigned long mv = (unsigned long)adc_value * 500UL / 1023UL;
		unsigned int volts = mv / 100;
		unsigned int frac  = mv % 100;

		snprintf(buf, sizeof(buf), "%4u  %u.%02uV", adc_value, volts, frac);

		lcd_set_cursor(0, 1);
		lcd_print(buf);

		_delay_ms(200);
	}
}