#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "i2c_lcd.h"

void adc_init(void)
{
<<<<<<< HEAD
	// ±âÁØÀü¾Ð: AVCC (REFS0=1, REFS1=0)
=======
	// ê¸°ì¤€ì „ì••: AVCC (REFS0=1, REFS1=0)
>>>>>>> 1ec295ce80c4231c2e678941d75e34c673c31bc9
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel)
{
	// ì±„ë„ ì„ íƒ (MUX0~4), ìƒìœ„ REFS ë¹„íŠ¸ëŠ” ìœ ì§€
	ADMUX = (ADMUX & 0xE0) | (channel & 0x1F);
	ADCSRA |= (1 << ADSC);        // ë³€í™˜ ì‹œìž‘
	while (ADCSRA & (1 << ADSC)); // ë³€í™˜ ì™„ë£Œê¹Œì§€ ëŒ€ê¸°
	return ADC; // ADCL, ADCH í•©ì³ì§„ 10bit ê°’ (0~1023)
}

// ADC(0~1023) ê°’ì— ë”°ë¼ LEDê°€ ìˆœì°¨ì ìœ¼ë¡œ ì¼œì§
void led_sequential(uint16_t adc_value)
{
	uint8_t count = adc_value / 128;   // 1024/128 = 8ë‹¨ê³„ (0~8)
	if (count > 8) count = 8;

	uint8_t leds = 0;
	for (uint8_t i = 0; i < count; i++)
	{
		leds |= (1 << i);   // countê°œë§Œí¼ LSBë¶€í„° ìˆœì„œëŒ€ë¡œ ì¼¤ ìœ„ì¹˜ í‘œì‹œ
	}

	PORTA = (uint8_t)~leds;   // ì•¡í‹°ë¸Œ ë¡œìš° -> ë°˜ì „
}

// ADC(0~1023) °ª¿¡ µû¶ó LED°¡ ¼øÂ÷ÀûÀ¸·Î ÄÑÁü
void led_sequential(uint16_t adc_value)
{
	uint8_t count = adc_value / 128;   // 1024/128 = 8´Ü°è (0~8)
	if (count > 8) count = 8;

	uint8_t leds = 0;
	for (uint8_t i = 0; i < count; i++)
	{
		leds |= (1 << i);   // count°³¸¸Å­ LSBºÎÅÍ ¼ø¼­´ë·Î ÄÓ À§Ä¡ Ç¥½Ã
	}

	PORTA = (uint8_t)~leds;   // ¾×Æ¼ºê ·Î¿ì: 0 = ÄÑÁü, 1 = ²¨ÁüÀÌ¹Ç·Î ¹ÝÀü
}

int main(void)
{
<<<<<<< HEAD
	DDRA = 0xFF;      // PORTA ÀüÃ¼¸¦ Ãâ·ÂÀ¸·Î ¼³Á¤ (LED ±¸µ¿¿ë)
	PORTA = 0xFF;     // LED ´Ù ²û
=======
	DDRA = 0xFF;      // PORTA ì „ì²´ë¥¼ ì¶œë ¥ìœ¼ë¡œ ì„¤ì •
	PORTA = 0xFF;     // LED ë‹¤ ë”
>>>>>>> 1ec295ce80c4231c2e678941d75e34c673c31bc9

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

<<<<<<< HEAD
		led_sequential(adc_value);   // ADC °ª¿¡ µû¶ó LED ¼øÂ÷ Á¡µî
=======
		led_sequential(adc_value);   // ADC ê°’ì— ë”°ë¼ LED ìˆœì°¨ ì ë“±
>>>>>>> 1ec295ce80c4231c2e678941d75e34c673c31bc9

		snprintf(buf, sizeof(buf), "%4u  %u.%02uV", adc_value, volts, frac);
		lcd_set_cursor(0, 1);
		lcd_print(buf);
		_delay_ms(200);
	}
}
