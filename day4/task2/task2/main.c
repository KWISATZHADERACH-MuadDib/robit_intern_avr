// 달력 시계 - 가변저항으로 날짜/시간 세팅하고 SW2 누르면 시간 흐름
// SW1: 값 확정하고 다음 항목으로, SW2: 시작

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

#include "i2c.h"
#include "i2c_lcd.h"

#define SW1_BIT     PD2
#define SW2_BIT     PD3

// PD0/PD1은 I2C라 건드리면 LCD 죽음
#define SW_PIN_REG  PIND
#define SW_DDR_REG  DDRD
#define SW_PORT_REG PORTD

#define YEAR_MIN    1900
#define YEAR_MAX    2099

// 지금 몇 번째 세팅 단계인지
enum {
	SET_YEAR = 0,
	SET_MONTH,
	SET_DAY,
	SET_HOUR,
	SET_MIN,
	SET_SEC,
	ST_READY,       // 세팅 끝, SW2 기다리는 중
	ST_RUNNING      // 시간 가는 중
};

// ISR이 만지는 값들이라 volatile 필수
static volatile uint16_t g_year  = 2026;
static volatile uint8_t  g_month = 1;
static volatile uint8_t  g_day   = 1;
static volatile uint8_t  g_hour  = 0;
static volatile uint8_t  g_min   = 0;
static volatile uint8_t  g_sec   = 0;
static volatile uint8_t  g_centi = 0;   // 0~99, 10ms 단위

static volatile uint8_t  g_running = 0;
static volatile uint8_t  g_tick    = 0;     // ISR이 main한테 "갱신해라" 알리는 용도

// 4로 나눠떨어지면 윤년, 100으로 나눠떨어지면 평년, 400이면 다시 윤년
static uint8_t is_leap_year(uint16_t y)
{
	return ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0));
}

static uint8_t days_in_month(uint16_t y, uint8_t m)
{
	static const uint8_t dim[12] = { 31,28,31,30,31,30,31,31,30,31,30,31 };

	if (m < 1 || m > 12)
		return 31;                 // 혹시 몰라서
	if (m == 2 && is_leap_year(y))
		return 29;

	return dim[m - 1];
}

static char g_shown[2][17];     // 지금 LCD에 뭐가 찍혀 있는지 기억

static void lcd_cache_reset(void)
{
	uint8_t r, c;
	for (r = 0; r < 2; r++) {
		for (c = 0; c < 16; c++)
		g_shown[r][c] = ' ';
		g_shown[r][16] = '\0';
	}
}

// 바뀐 글자만 골라서 씀. s는 16글자 꽉 채워서 넘겨야 함
static void lcd_update_line(uint8_t row, const char *s)
{
	uint8_t i = 0;

	while (i < 16)
	{
		if (s[i] == g_shown[row][i]) { i++; continue; }

		// 다른 데 나오면 거기부터 연속으로 다른 구간까지 쭉 밀어넣음
		lcd_set_cursor(i, row);

		while (i < 16 && s[i] != g_shown[row][i])
		{
			lcd_send_data((uint8_t)s[i]);
			g_shown[row][i] = s[i];
			i++;
		}
	}
}

// 뒤에 공백 채워서 16글자로. 안 하면 이전 글자 잔상 남음
static void pad16(char *buf)
{
	uint8_t i = 0;
	while (buf[i] != '\0' && i < 16) i++;
	while (i < 16) buf[i++] = ' ';
	buf[16] = '\0';
}

static void adc_init(void)
{
	ADMUX  = (1 << REFS0);                                  // AVCC, ADC0(PF0)
	ADCSRA = (1 << ADEN)
	| (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128분주
}

static uint16_t adc_read(void)
{
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADCW;
}

// 그냥 읽으면 값이 계속 떨려서 8번 평균
static uint16_t adc_read_avg(void)
{
	uint8_t  i;
	uint32_t sum = 0;
	for (i = 0; i < 8; i++) 
		sum += adc_read();
	return (uint16_t)(sum >> 3);
}

// 0~1023을 lo~hi로 매핑
static uint16_t adc_map(uint16_t adc, uint16_t lo, uint16_t hi)
{
	uint32_t span = (uint32_t)(hi - lo + 1);
	uint16_t v = lo + (uint16_t)(((uint32_t)adc * span) / 1024UL);

	if (v > hi) 
		v = hi;
	return v;
}

// 회로가 5V --10k-- 핀 --스위치-- GND 라서 누르면 LOW
static uint8_t g_sw_last[2] = { 1, 1 };

static uint8_t sw_pressed(uint8_t idx, uint8_t bit)
{
	uint8_t cur = (SW_PIN_REG & (1 << bit)) ? 1 : 0;
	uint8_t hit = 0;

	if (g_sw_last[idx] == 1 && cur == 0)            // 눌리는 순간만 잡음
	{
		_delay_ms(20);                              // 채터링 가라앉을 때까지
		if (!(SW_PIN_REG & (1 << bit)))
			hit = 1;    // 아직도 눌려있으면 진짜
	}

	g_sw_last[idx] = cur;
	return hit;
}

// Timer1 CTC, 분주비 64로 10ms
// 16MHz/64 = 250kHz, 250000/100 = 2500 이니까 OCR1A는 2499
static void timer1_init(void)
{
	TCCR1A = 0x00;
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
	OCR1A  = 2499;
	TCNT1  = 0;
	TIMSK |= (1 << OCIE1A);
}

// 10ms마다 돔. 여기선 시간 계산만 하고 LCD는 안 건드림
// (I2C가 느려서 ISR 안에서 하면 시간이 계속 밀림)
ISR(TIMER1_COMPA_vect)
{
	if (!g_running)
		return;

	g_centi++;
	if (g_centi < 100) {
		g_tick = 1; return;
	}

	g_centi = 0;
	g_sec++;
	if (g_sec < 60) {
		g_tick = 1; return;
	}

	g_sec = 0;
	g_min++;
	if (g_min < 60) {
		g_tick = 1; return;
	}

	g_min = 0;
	g_hour++;
	if (g_hour < 24) {
		g_tick = 1;
		return; 
	}

	g_hour = 0;
	g_day++;
	if (g_day <= days_in_month(g_year, g_month)) {
		g_tick = 1;
		return; 
	}

	g_day = 1;
	g_month++;
	if (g_month <= 12) { 
		g_tick = 1; 
		return; 
	}

	g_month = 1;
	g_year++;
	if (g_year > YEAR_MAX)
		g_year = YEAR_MIN;

	g_tick = 1;
}

// 1행에 항목 이름, 2행에 지금 고른 값
static void show_setting(const char *label, uint16_t value, uint8_t digits)
{
	char buf[20];

	sprintf(buf, "%s", label);
	pad16(buf);
	lcd_update_line(0, buf);

	if (digits == 4) s
		printf(buf, "    %04u", value);
	else            
		sprintf(buf, "      %02u", value);
	pad16(buf);
	lcd_update_line(1, buf);
}

// 1행 YYMMDD, 2행 HH:MM:SS.cc
// ISR이 값 바꾸는 중에 읽으면 시/분이 섞일 수 있어서 잠깐 막고 통째로 복사
static void show_clock(void)
{
	char     buf[20];
	uint16_t y;
	uint8_t  mo, d, h, mi, s, c;
	uint8_t  sreg = SREG;

	cli();
	y = g_year; mo = g_month; d = g_day;
	h = g_hour; mi = g_min;   s = g_sec;  c = g_centi;
	SREG = sreg;        // sei() 말고 이렇게 해야 원래 상태로 돌아감

	sprintf(buf, "%02u%02u%02u", (uint16_t)(y % 100), mo, d);
	pad16(buf);
	lcd_update_line(0, buf);

	sprintf(buf, "%02u:%02u:%02u.%02u", h, mi, s, c);
	pad16(buf);
	lcd_update_line(1, buf);
}

int main(void)
{
	uint8_t  state = SET_YEAR;
	uint16_t adc;
	uint8_t  maxDay;

	// 스위치 입력으로. 외부에 10k 풀업 달려있어서 내부 풀업은 안 켬
	// PD0/PD1 안 건드리게 해당 비트만 조작
	SW_DDR_REG  &= ~((1 << SW1_BIT) | (1 << SW2_BIT));
	SW_PORT_REG &= ~((1 << SW1_BIT) | (1 << SW2_BIT));

	DDRF  &= ~(1 << PF0);
	PORTF &= ~(1 << PF0);

	adc_init();
	lcd_init();     // i2c_init()도 안에서 같이 해줌
	lcd_clear();
	lcd_cache_reset();

	timer1_init();
	sei();

	lcd_update_line(0, "Calendar Clock  ");
	lcd_update_line(1, "SW1:OK  SW2:GO  ");
	_delay_ms(1500);

	while (1)
	{
		switch (state)
		{
			case SET_YEAR:
			adc = adc_read_avg();
			g_year = adc_map(adc, YEAR_MIN, YEAR_MAX);
			show_setting("SET YEAR", g_year, 4);
			if (sw_pressed(0, SW1_BIT)) 
				state = SET_MONTH;
			break;

			case SET_MONTH:
			adc = adc_read_avg();
			g_month = (uint8_t)adc_map(adc, 1, 12);
			show_setting("SET MONTH", g_month, 2);
			if (sw_pressed(0, SW1_BIT))
				state = SET_DAY;
			break;

			case SET_DAY:
			// 그 달 말일까지만 고르게 해서 2월 30일 같은 게 아예 안 나오게 함
			maxDay  = days_in_month(g_year, g_month);
			adc     = adc_read_avg();
			g_day   = (uint8_t)adc_map(adc, 1, maxDay);
			show_setting("SET DAY", g_day, 2);
			if (sw_pressed(0, SW1_BIT))
				state = SET_HOUR;
			break;

			case SET_HOUR:
			adc = adc_read_avg();
			g_hour = (uint8_t)adc_map(adc, 0, 23);
			show_setting("SET HOUR", g_hour, 2);
			if (sw_pressed(0, SW1_BIT))
				state = SET_MIN;
			break;

			case SET_MIN:
			adc = adc_read_avg();
			g_min = (uint8_t)adc_map(adc, 0, 59);
			show_setting("SET MINUTE", g_min, 2);
			if (sw_pressed(0, SW1_BIT)) 
				state = SET_SEC;
			break;

			case SET_SEC:
			adc = adc_read_avg();
			g_sec = (uint8_t)adc_map(adc, 0, 59);
			show_setting("SET SECOND", g_sec, 2);
			if (sw_pressed(0, SW1_BIT))
			{
				g_centi = 0;
				state = ST_READY;
			}
			break;

			// 세팅은 끝났는데 아직 시간은 멈춰 있는 상태
			case ST_READY:
			show_clock();
			if (sw_pressed(1, SW2_BIT))
			{
				g_centi   = 0;
				g_running = 1;      // 이때부터 ISR이 카운트 시작
				state     = ST_RUNNING;
			}
			break;

			case ST_RUNNING:
			if (g_tick)
			{
				g_tick = 0;
				show_clock();       // 바뀐 글자만 쓰니까 10ms마다 해도 안 밀림
			}
			break;

			default:
			state = SET_YEAR;
			break;
		}
	}

	return 0;
}
