#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// 왼쪽으로 이동
ISR(INT4_vect)
{
	int led = 0b11111110;
	int i;
	for (i = 0; i < 8; i++)
	{
		PORTA = led;
		_delay_ms(100);
		led = (led << 1) | 0b00000001;   // 켜진 자리를 왼쪽으로 한 칸 이동
	}
}

// 오른쪽으로 이동
ISR(INT5_vect)
{
	int led = 0b01111111;
	int i;
	for (i = 0; i < 8; i++)
	{
		PORTA = led;
		_delay_ms(100);
		led = (led >> 1) | 0b10000000;   // 켜진 자리를 오른쪽으로 한 칸 이동
	}
}

int main(void)
{
	int led = 0b00000000;
	int sw1, sw2;

	DDRA = 0xFF;   // PORTA 전체를 출력으로 설정 (LED)
	PORTA = 0b11111111;

	DDRD &= ~((1 << PD2) | (1 << PD3));   // SW1(PD2), SW2(PD3) 입력으로 설정
	PORTD |= (1 << PD2) | (1 << PD3);     // 내부 풀업 저항 켜기 (안 누르면 1, 누르면 0)

	DDRE &= ~((1 << PE4) | (1 << PE5));   // PE4, PE5 입력으로 설정
	PORTE |= (1 << PE4) | (1 << PE5);     // 내부 풀업 저항 켜기

	EICRB |= (1 << ISC41);    // INT4 : falling edge 발생
	EICRB &= ~(1 << ISC40);

	EICRB |= (1 << ISC51);    // INT5 : falling edge 발생
	EICRB &= ~(1 << ISC50);

	EIMSK |= (1 << INT4) | (1 << INT5);

	sei();   // 전체 인터럽트 허용
	// ---- 여기까지 외부인터럽트 설정 ----

	while (1)
	{
		sw1 = !(PIND & (1 << PD2));   // 스위치 1
		sw2 = !(PIND & (1 << PD3));   // 스위치 2

		if (sw1 == 1 && sw2 == 1)
		{
			led = 0x00;
			PORTA = led;
		}
		else if (sw1 == 1)
		{
			PORTA = 0x0F;   // 2) SW1 누르면 4~7번 LED 켜기
		}
		else if (sw2 == 1)
		{
			PORTA = 0xF0;   // 3) SW2 누르면 0~3번 LED 켜기
		}
		else
		{
			led  = ~led;
			PORTA = led;   // 전체 반전
		}
		_delay_ms(500);
	}
	return 0;
}