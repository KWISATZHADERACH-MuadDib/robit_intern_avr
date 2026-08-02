#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

unsigned char Uart_Getch(void);
void Uart_Putch(unsigned char PutData);
void UART_transmit_string(char *str);

volatile unsigned char led_mask = 0x00;   // 1=켜짐, 0=꺼짐 (Active-High 기준으로 관리)
// PORTA에 쓸 때만 반전(~)해서 내보냄

ISR(INT2_vect)
{
	led_mask = 0x00;          // 전부 꺼짐 상태로 초기화
	PORTA = ~led_mask;        // Active-Low로 변환 → 0xFF (전부 꺼짐)
	UART_transmit_string("RESET\r\n");
}

int main(void)
{
	//UART
	UBRR0L = 16;    //57600bps
	UBRR0H = 0;
	UCSR0B = 0x18;  // 송신, 수신 기능 활성화 (RXEN0, TXEN0)
	UCSR0C = 0x06;  // START 1비트 / DATA 8비트 / STOP 1비트
	DDRE = 0x02;    // RXD0(E0) 입력, TXD0(E1) 출력

	//LED
	DDRA = 0xFF;      // PORTA 전체 출력으로 설정
	PORTA = ~led_mask;

	//SW1
	DDRD &= ~(1 << PD2);   // PD2 입력으로 설정
	PORTD |= (1 << PD2);   // 내부 풀업 저항 활성화 (안 눌림 = High 유지)

	EICRA |= (1 << ISC21) | (0 << ISC20);  // INT2 : falling edge 트리거
	EIMSK |= (1 << INT2);                  // INT2 인터럽트 활성화

	sei();   // 글로벌 인터럽트 활성화

	UART_transmit_string("Hello, UART!\r\n");

	char buf[20];  // sprintf로 문자열 만들 때 쓸 임시 버퍼

	while (1)
	{
		char recvData = Uart_Getch();

		if (recvData >= '0' && recvData <= '7')   // 0~7 숫자 입력 → 해당 LED 켜기(누적)
		{
			int num = recvData - '0';
			led_mask |= (1 << num);     // num번째 비트만 추가로 켜기 (기존 켜진 것 유지)
			PORTA = ~led_mask;

			sprintf(buf, "%d LED on\r\n", num);
			UART_transmit_string(buf);
		}
		else if (recvData == '8')   // LEFT
		{
			led_mask <<= 1;
			PORTA = ~led_mask;
			UART_transmit_string("LEFT\r\n");
		}
		else if (recvData == '9')   // RIGHT
		{
			led_mask >>= 1;
			PORTA = ~led_mask;
			UART_transmit_string("RIGHT\r\n");
		}
	}
}

unsigned char Uart_Getch(void)
{
	while(!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

void Uart_Putch(unsigned char PutData)
{
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = PutData;
}

void UART_transmit_string(char *str)
{
	while(*str != '\0') {
		Uart_Putch(*str);
		str++;
	}
}