#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned char counter = 0xFF;        // 2진 카운터 값 (논리값, 반전 전)

// 버튼1(INT2) : 왼쪽 3개 LED가 오른쪽으로 한 칸씩 이동
ISR(INT2_vect)
{
	unsigned char led = 0b00011111;   // LED5,6,7 켜짐 (왼쪽 3개)
	int i;
	PORTA = led;
	_delay_ms(100);
	for (i = 0; i < 2; i++)
	{
		led = (led >> 1) | 0b10000000;   // 오른쪽으로 한 칸, 빈자리(왼쪽)는 꺼짐
		PORTA = led;
		_delay_ms(200);
	}
}

// 버튼2(INT3) : 오른쪽 3개 LED가 왼쪽으로 한 칸씩 이동
ISR(INT3_vect)
{
	unsigned char led = 0b11111000;   // LED0,1,2 켜짐 (오른쪽 3개)
	int i;
	PORTA = led;
	_delay_ms(100);
	for (i = 0; i < 2; i++)
	{
		led = (led << 1) | 0b00000001;   // 왼쪽으로 한 칸, 빈자리(오른쪽)는 꺼짐
		PORTA = led;
		_delay_ms(200);
	}
}

// 버튼3(INT4) : 오른쪽 끝 LED 1개가 왼쪽 끝까지 갔다가 다시 오른쪽 끝으로
ISR(INT4_vect)
{
	unsigned char led = 0b11111110;   // LED0(맨 오른쪽)만 켜짐
	int i;
	PORTA = led;
	_delay_ms(100);
	for (i = 0; i < 7; i++)
	{
		led = (led << 1) | 0b00000001;   // 왼쪽으로 한 칸씩, 7칸 이동
		PORTA = led;
		_delay_ms(100);
	}
	for (i = 0; i < 7; i++)
	{
		led = (led >> 1) | 0b10000000;   // 다시 오른쪽으로 한 칸씩, 7칸 이동
		PORTA = led;
		_delay_ms(100);
	}
}

//2진 카운터 초기화
ISR(INT5_vect)
{
	counter = 0xFF;
}

int main(void)
{
	DDRA = 0xFF;    // LED 출력
	PORTA = 0xFF;   // 처음엔 전부 꺼짐 (active-low)

	DDRD = 0b11110011;
	PORTD |= (1 << PD2) | (1 << PD3);

	DDRE = 0b11001111;
	PORTE |= (1 << PE4) | (1 << PE5);

	EICRA |= (1 << ISC21);   // 버튼1 : falling edge
	EICRA |= (0 << ISC20);

	EICRA |= (1 << ISC31);   // 버튼2 : falling edge
	EICRA |= (0 << ISC30);

	EICRB |= (1 << ISC41);   // 버튼3 : falling edge
	EICRB |= (0 << ISC40);

	EICRB |= (1 << ISC50);   // 버튼4 : 논리값 변화(any logical change) = 01
	EICRB |= (0 << ISC51);

	EIMSK |= (1 << INT2) | (1 << INT3) | (1 << INT4) | (1 << INT5);

	sei();

	while (1)
	{
		PORTA = counter;
		_delay_ms(100);
		counter--;
	}

	return 0;
}