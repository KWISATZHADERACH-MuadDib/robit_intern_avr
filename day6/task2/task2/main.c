#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	// PORTB 0~3 : IN1, IN2, IN3, IN4
	// PORTB 5,6 : ENA, ENB
	DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3);
	DDRB |= (1 << PB5) | (1 << PB6);

	PORTB |= (1 << PB5) | (1 << PB6);

	// 모터 A: IN1 = HIGH, IN2 = LOW  -> 한쪽 방향 회전
	PORTB |= (1 << PB0);
	PORTB &= ~(1 << PB1);

	// 모터 B: IN3 = HIGH, IN4 = LOW  -> 같은 방향 회전
	PORTB |= (1 << PB2);
	PORTB &= ~(1 << PB3);

	while (1)
	{
	}
}