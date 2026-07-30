#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "i2c_lcd.h"

volatile int A = 1;
volatile int B = 1;
volatile int op_index = 0;
char operators[] = {'+', '-', '*', '/'};

static void lcd_clear_line(int row)
{
	lcd_set_cursor(0, row);
	lcd_print("                "); // 16칸 공백으로 지우기
}

static void show_preview(void)
{
	char buf[17];
	snprintf(buf, sizeof(buf), "%d %c %d = ?", A, operators[op_index], B);
	lcd_clear_line(0);
	lcd_set_cursor(0, 0);
	lcd_print(buf);
}

static void show_result(void)
{
	char buf[17];
	char op = operators[op_index];

	lcd_clear_line(0);
	lcd_set_cursor(0, 0);

	if (op == '/') {
		if (B == 0) {
			lcd_print("Error: B=0");
			} else {
			long fixed = ((long)A * 100) / B; // 소수 2자리 고정소수점
			int intpart = fixed / 100;
			int frac = fixed % 100;
			if (frac < 0) frac = -frac;
			snprintf(buf, sizeof(buf), "%d/%d=%d.%02d", A, B, intpart, frac);
			lcd_print(buf);
		}
		} else {
		int result = 0;
		switch (op) {
			case '+': result = A + B; break;
			case '-': result = A - B; break;
			case '*': result = A * B; break;
		}
		snprintf(buf, sizeof(buf), "%d%c%d=%d", A, op, B, result);
		lcd_print(buf);
	}
}

// SW1 (PD2 = INT2) : A값 증가
ISR(INT2_vect)
{
	_delay_ms(20); // 디바운스
	if (!(PIND & (1 << PD2))) { // 여전히 눌려있으면 확정
		A++;
		show_preview();
	}
}

// SW2 (PD3 = INT3) : 연산자 변경
ISR(INT3_vect)
{
	_delay_ms(20);
	if (!(PIND & (1 << PD3))) {
		op_index = (op_index + 1) % 4; // + - * / 순환
		show_preview();
	}
}

// SW3 (PE4 = INT4) : B값 증가
ISR(INT4_vect)
{
	_delay_ms(20);
	if (!(PINE & (1 << PE4))) {
		B++;
		show_preview();
	}
}

// SW4 (PE5 = INT5) : 연산 실행
ISR(INT5_vect)
{
	_delay_ms(20);
	if (!(PINE & (1 << PE5))) {
		show_result();
	}
}

static void switches_init(void)
{
	// PD2, PD3 입력 설정
	DDRD = 0b11110011;
	PORTD |= (1 << PD2) | (1 << PD3);

	// PE4, PE5 입력 설정
	DDRE = 0b11001111;
	PORTE |= (1 << PE4) | (1 << PE5);
}

static void ext_interrupt_init(void)
{
	// INT2, INT3 : falling edge
	EICRA |= (1 << ISC21) | (1 << ISC31);
	EICRA = 0b10101111;

	// INT4, INT5 : falling edge
	EICRB |= (1 << ISC41) | (1 << ISC51);
	EICRB = 0b11110101;

	// INT2, INT3, INT4, INT5 인터럽트 활성화
	EIMSK |= (1 << INT2) | (1 << INT3) | (1 << INT4) | (1 << INT5);
}

int main(void)
{
	switches_init();
	ext_interrupt_init();
	lcd_init();

	show_preview();

	sei();

	while (1)
	{
	}
}