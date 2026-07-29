/*
 * I2C 주소 스캐너
 * LCD의 정확한 I2C 주소(0x27, 0x3F 등)를 모를 때 이 코드로 확인하세요.
 * USART(시리얼)로 결과를 출력하므로, 별도로 UART 초기화 코드가 필요합니다.
 * 이 파일은 main.c 대신 사용하는 참고용 예제입니다.
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "i2c.h"

// 간단한 UART 출력 (ATmega328P 기준, 9600bps)
static void uart_init(void)
{
    UBRR0H = 0;
    UBRR0L = 103; // 16MHz, 9600bps
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_print_hex(uint8_t val)
{
    const char hex[] = "0123456789ABCDEF";
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = hex[(val >> 4) & 0x0F];
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = hex[val & 0x0F];
}

static void uart_print(const char *s)
{
    while (*s) {
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = *s++;
    }
}

int main(void)
{
    uart_init();
    i2c_init();
    _delay_ms(500);

    uart_print("I2C Scan Start\r\n");

    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_start();
        uint8_t status = i2c_write(addr << 1);
        i2c_stop();

        if (status == 0x18) { // ACK 받음 = 장치 존재
            uart_print("Found device at 0x");
            uart_print_hex(addr);
            uart_print("\r\n");
        }
        _delay_ms(10);
    }

    uart_print("Scan Done\r\n");

    while (1);
}
