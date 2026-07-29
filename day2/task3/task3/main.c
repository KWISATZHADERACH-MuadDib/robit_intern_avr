#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "i2c_lcd.h"

// 스위치 핀 정의 (풀업 사용, 안누르면 1, 누르면 0)
#define SW1_PRESSED (!(PIND & (1 << PD2)))  // A값 증가
#define SW2_PRESSED (!(PIND & (1 << PD3)))  // 연산자 변경
#define SW3_PRESSED (!(PINE & (1 << PE4)))  // B값 증가
#define SW4_PRESSED (!(PINE & (1 << PE5)))  // 연산 실행

static void switches_init(void)
{
	// PD2, PD3 입력 설정 + 풀업 저항 활성화
	DDRD &= ~((1 << PD2) | (1 << PD3));
	PORTD |= (1 << PD2) | (1 << PD3);

	// PE4, PE5 입력 설정 + 풀업 저항 활성화
	DDRE &= ~((1 << PE4) | (1 << PE5));
	PORTE |= (1 << PE4) | (1 << PE5);
}

// 디바운스 포함 버튼 눌림 감지 (눌리는 순간 1번만 true 반환)
static uint8_t check_button_edge(uint8_t currently_pressed, uint8_t *last_state)
{
	uint8_t edge = 0;

	if (currently_pressed && !(*last_state)) {
		_delay_ms(20); // 디바운스 대기
		edge = 1;      // 눌림 이벤트 발생
	}

	*last_state = currently_pressed;
	return edge;
}

static void lcd_clear_line(uint8_t row)
{
	lcd_set_cursor(0, row);
	lcd_print("                "); // 16칸 공백으로 지우기
}

int main(void)
{
	switches_init();
	lcd_init();

	int A = 1;
	int B = 1;
	char operators[] = {'+', '-', '*', '/'};
	uint8_t op_index = 0;

	uint8_t sw1_last = 0, sw2_last = 0, sw3_last = 0, sw4_last = 0;

	char buf[17];

	// 초기 화면: 현재 A, 연산자, B 미리보기
	snprintf(buf, sizeof(buf), "%d %c %d = ?", A, operators[op_index], B);
	lcd_clear_line(0);
	lcd_set_cursor(0, 0);
	lcd_print(buf);

	while (1)
	{
		uint8_t updated = 0;

		if (check_button_edge(SW1_PRESSED, &sw1_last)) {
			A++;
			updated = 1;
		}

		if (check_button_edge(SW2_PRESSED, &sw2_last)) {
			op_index = (op_index + 1) % 4; // + - * / 순환
			updated = 1;
		}

		if (check_button_edge(SW3_PRESSED, &sw3_last)) {
			B++;
			updated = 1;
		}

		if (check_button_edge(SW4_PRESSED, &sw4_last)) {
			// 연산 실행 후 결과 표시
			char op = operators[op_index];

			lcd_clear_line(0);
			lcd_set_cursor(0, 0);

			if (op == '/') {
				if (B == 0) {
					lcd_print("Error: B=0");
					} else {
					long fixed = ((long)A * 1000) / B; // 소수 3자리 고정소수점
					int intpart = fixed / 1000;
					int frac = fixed % 1000;
					if (frac < 0) frac = -frac;
					snprintf(buf, sizeof(buf), "%d/%d=%d.%03d", A, B, intpart, frac);
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

			updated = 0; // 결과 화면은 다음 스위치 입력 전까지 유지
		}

		// A, 연산자, B 값이 바뀌면 미리보기 갱신 (결과 실행 전)
		if (updated) {
			snprintf(buf, sizeof(buf), "%d %c %d = ?", A, operators[op_index], B);
			lcd_clear_line(0);
			lcd_set_cursor(0, 0);
			lcd_print(buf);
		}

		_delay_ms(10);
	}
}