#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "i2c_lcd.h"

void adc_init(void)
{
	// 기준전압: AVCC (REFS0=1, REFS1=0)
	ADMUX = (1 << REFS0);
	// ADC 활성화(ADEN) + 분주비 128 (ADPS2,1,0 = 1,1,1)
	// 16MHz / 128 = 125kHz (ADC 권장 범위 50~200kHz)
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel)
{
	// 채널 선택 (MUX0~4), 상위 REFS 비트는 유지
	ADMUX = (ADMUX & 0xE0) | (channel & 0x1F);
	ADCSRA |= (1 << ADSC);        // 변환 시작
	while (ADCSRA & (1 << ADSC)); // 변환 완료까지 대기
	return ADC; // ADCL, ADCH 합쳐진 10bit 값 (0~1023)
}

// ADC(0~1023) 값에 따라 LED가 순차적으로 켜짐
void led_sequential(uint16_t adc_value)
{
	uint8_t count = adc_value / 128;   // 1024/128 = 8단계 (0~8)
	if (count > 8) count = 8;

	uint8_t leds = 0;
	for (uint8_t i = 0; i < count; i++)
	{
		leds |= (1 << i);   // count개만큼 LSB부터 순서대로 켤 위치 표시
	}

	PORTA = (uint8_t)~leds;   // 액티브 로우: 0 = 켜짐, 1 = 꺼짐이므로 반전
}

int main(void)
{
	DDRA = 0xFF;      // PORTA 전체를 출력으로 설정 (LED 구동용)
	PORTA = 0xFF;     // LED 다 끔

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

		led_sequential(adc_value);   // ADC 값에 따라 LED 순차 점등

		snprintf(buf, sizeof(buf), "%4u  %u.%02uV", adc_value, volts, frac);
		lcd_set_cursor(0, 1);
		lcd_print(buf);
		_delay_ms(200);
	}
}
