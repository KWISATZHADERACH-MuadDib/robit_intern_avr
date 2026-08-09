#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define TX_PIN      PD3         //UART TX 핀

#define BIT_TIME    104

static void softUART_init(void)
{
	DDRD  |= (1 << TX_PIN);     // TX_PIN 출력으로 설정
	PORTD |= (1 << TX_PIN);     // Idle 상태 = High (UART는 안 보낼 때 항상 High)

	_delay_ms(10);              // 라인이 Idle(High)로 안정될 때까지 잠깐 대기
}


//[Start(0)] [Data 8bit, LSB first] [Stop(1)] 구조
static void softUART_putch(uint8_t data)
{
	uint8_t i;

	//Start Bit : 0
	PORTD &= ~(1 << TX_PIN);
	_delay_us(BIT_TIME);

	//Data Bit 8개
	for (i = 0; i < 8; i++)
	{
		if (data & 0x01)
			PORTD |=  (1 << TX_PIN);    // 비트가 1이면 High
		else
			PORTD &= ~(1 << TX_PIN);    // 비트가 0이면 Low

		_delay_us(BIT_TIME);
		data >>= 1;       
	}

	//Stop Bit : 1
	PORTD |= (1 << TX_PIN);
	_delay_us(BIT_TIME);
}

static void softUART_transmit_string(const char *str)
{
	while (*str != '\0')	//엔터 입력이 안되었다면
	{
		softUART_putch((uint8_t)(*str));	//계속 보내줌
		str++;
	}
}

//==============================================================================
int main(void)
{
	softUART_init();

	while (1)
	{
		softUART_transmit_string("HelloWorld!\r\n");
		_delay_ms(1000);        // 1초마다 반복
	}

	return 0;
}
