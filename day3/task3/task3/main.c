// 과제3 - 다이나믹셀 위치/속도 제어
// 가변저항 돌리면 목표 위치, PC에서 0~9 입력하면 목표 속도
// 둘 다 LCD에 표시

// F_CPU는 delay.h보다 위에 있어야 함
// 안 그러면 delay.h가 알아서 1MHz로 잡아버려서 나중에 정의해도 소용없음
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

#include "i2c.h"
#include "i2c_lcd.h"

#define UART0_BAUD          57600UL
#define UART0_USE_U2X       1

#if UART0_USE_U2X
#define UBRR0_VAL   ((((F_CPU) + 4UL * (UART0_BAUD)) / (8UL  * (UART0_BAUD))) - 1)
#else
#define UBRR0_VAL   ((((F_CPU) + 8UL * (UART0_BAUD)) / (16UL * (UART0_BAUD))) - 1)
#endif

// 다이나믹셀 컨트롤 테이블 주소 (Protocol 2.0)
#define DXL_ID                  1
#define ADDR_STATUS_RETURN_LVL  68
#define ADDR_TORQUE_ENABLE      64
#define ADDR_PROFILE_VELOCITY   112     // 4byte
#define ADDR_GOAL_POSITION      116     // 4byte

#define SPEED_MAX               300UL

// Profile Velocity 0은 "속도 제한 없음"이라 최대속도로 튐.
#define SPEED_ZERO_AS_SLOWEST   1

// 0이면 ADC값
// 1이면 0~1023을 0~4092로 늘림 (안 늘리면 90도 정도밖에 안 움직임)
#define SCALE_POS_TO_4096       0

// 이 보드는 PC 시리얼이랑 MAX485가 PE0/PE1 같이 씀.
// PE2를 LOW로 두면 485의 RO가 PE0을 잡아버려서 PC 입력이 깨진다.
// 어차피 다이나믹셀 응답 읽을 일 없으니까
// 응답 자체를 꺼놓고(Status Return Level=1) PE2는 계속 HIGH로.
#define DXL_KEEP_TX_MODE        1

#define DIR_PIN     PE2
#define DIR_TX()    (PORTE |=  (1 << DIR_PIN))
#define DIR_RX()    (PORTE &= ~(1 << DIR_PIN))

static volatile uint8_t rxDigit    = 0;
static volatile uint8_t rxDigitNew = 0;

static void uart0_init(void)
{
	DDRE |=  (1 << PE1);    // TXD0
	DDRE &= ~(1 << PE0);    // RXD0

	// RXD0 풀업 켜두는 이유:
	// KEEP_TX_MODE면 485 RO가 항상 하이임피던스라서
	// PC 케이블 안 꽂으면 PE0이 공중에 뜬다.
	// 그 상태로 두면 노이즈를 데이터로 읽어서 속도가 지멋대로 바뀜.
	PORTE |= (1 << PE0);

	UBRR0H = (uint8_t)(UBRR0_VAL >> 8);
	UBRR0L = (uint8_t)(UBRR0_VAL);

	#if UART0_USE_U2X
	UCSR0A = (1 << U2X0);
	#else
	UCSR0A = 0x00;
	#endif

	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);     // 8-N-1
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
}

static void uart0_putchar(uint8_t data)
{
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

// PC에서 오는 0~9만 챙기고 나머지는 버림. 실제 처리는 main에서.
ISR(USART0_RX_vect)
{
	uint8_t c = UDR0;   // 안 읽으면 RXC0가 안 내려감

	if (c >= '0' && c <= '9')
	{
		rxDigit    = (uint8_t)(c - '0');
		rxDigitNew = 1;
	}
}

static void adc_init(void)
{
	ADMUX  = (1 << REFS0);                          // AVCC 기준, ADC0(PF0)
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);   // 분주비 : 128
}

static uint16_t adc_read(void)
{
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));

	return ADCW;
}

// 8번 읽어서 평균 : 값 보정
static uint16_t adc_read_avg(void)
{
	uint8_t  i;
	uint32_t sum = 0;

	for (i = 0; i < 8; i++) 
		sum += adc_read();

	return (uint16_t)(sum >> 3);
}

// 다이나믹셀 프로토콜 2.0 CRC 테이블
// PROGMEM에 올려야 SRAM 안 잡아먹음
static const uint16_t crc_table[256] PROGMEM = {
	0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
	0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
	0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
	0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
	0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
	0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
	0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
	0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
	0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
	0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
	0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
	0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
	0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
	0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
	0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
	0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
	0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
	0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
	0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
	0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
	0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
	0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
	0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
	0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
	0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
	0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
	0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
	0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
	0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
	0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
	0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
	0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};

// CRC 2바이트 뺀 나머지 전체로 계산
static uint16_t update_crc(uint16_t crc_accum, uint8_t *data, uint16_t len)
{
	uint16_t i;

	for (i = 0; i < len; i++)
	{
		uint16_t idx = (uint16_t)(((crc_accum >> 8) ^ data[i]) & 0xFF);
		crc_accum = (uint16_t)((crc_accum << 8) ^ pgm_read_word(&crc_table[idx]));
	}

	return crc_accum;
}

// WRITE 패킷 만들어서 보내기
// FF FF FD 00 | ID | Len_L Len_H | 0x03 | Addr_L Addr_H | Data... | CRC_L CRC_H
// Length는 Instruction(1) + Addr(2) + Data(n) + CRC(2)
static void dxl_write(uint8_t id, uint16_t addr, uint8_t *data, uint8_t dataLen)
{
	uint8_t  packet[16];
	uint8_t  idx = 0;
	uint8_t  i;
	uint16_t length = (uint16_t)(1 + 2 + dataLen + 2);
	uint16_t crc;

	packet[idx++] = 0xFF;
	packet[idx++] = 0xFF;
	packet[idx++] = 0xFD;
	packet[idx++] = 0x00;
	packet[idx++] = id;
	packet[idx++] = (uint8_t)(length & 0xFF);
	packet[idx++] = (uint8_t)(length >> 8);
	packet[idx++] = 0x03;                   // WRITE
	packet[idx++] = (uint8_t)(addr & 0xFF);
	packet[idx++] = (uint8_t)(addr >> 8);

	for (i = 0; i < dataLen; i++) 
		packet[idx++] = data[i];

	crc = update_crc(0, packet, idx);
	packet[idx++] = (uint8_t)(crc & 0xFF);
	packet[idx++] = (uint8_t)(crc >> 8);

	DIR_TX();
	_delay_us(10);      // 방향 바뀌고 안정될 때까지

	// TXC0는 1을 써야 클리어됨. |=로 안 하면 U2X0가 같이 날아감
	UCSR0A |= (1 << TXC0);

	for (i = 0; i < idx; i++) 
		uart0_putchar(packet[i]);

	while (!(UCSR0A & (1 << TXC0)));    // 마지막 비트까지 다 나갈 때까지

	_delay_us(100);     // 패킷 사이 간격

	#if !DXL_KEEP_TX_MODE
	DIR_RX();
	#endif
}

// 리틀 엔디안으로 1바이트 / 4바이트 쓰기
static void dxl_write1(uint8_t id, uint16_t addr, uint8_t value)
{
	dxl_write(id, addr, &value, 1);
}

static void dxl_write4(uint8_t id, uint16_t addr, uint32_t value)
{
	uint8_t d[4];

	d[0] = (uint8_t)(value);
	d[1] = (uint8_t)(value >> 8);
	d[2] = (uint8_t)(value >> 16);
	d[3] = (uint8_t)(value >> 24);

	dxl_write(id, addr, d, 4);
}

// i2c_lcd 드라이버 씀. set_cursor는 (열, 행) 순서인 거 주의
static void lcdShow(uint16_t speed, uint16_t pos)
{
	char buf[20];

	sprintf(buf, "Speed:%4u", speed);   // %4u로 폭 고정해야 잔상 안 남음
	lcd_set_cursor(0, 0);
	lcd_print(buf);

	sprintf(buf, "Pos  :%4u", pos);
	lcd_set_cursor(0, 1);
	lcd_print(buf);
}

int main(void)
{
	uint16_t adcVal;
	uint16_t goalPos    = 0;
	uint16_t goalSpeed  = 0;
	uint16_t lastPos    = 0xFFFF;   // 마지막으로 보낸 위치
	uint16_t shownPos   = 0xFFFF;   // 마지막으로 LCD에 찍은 값
	uint16_t shownSpeed = 0xFFFF;

	DDRE |= (1 << DIR_PIN);
	#if DXL_KEEP_TX_MODE
	DIR_TX();
	#else
	DIR_RX();
	#endif

	// 가변저항 PF0, 풀업 끄기
	DDRF  &= ~(1 << PF0);
	PORTF &= ~(1 << PF0);

	uart0_init();
	adc_init();
	lcd_init();     // i2c_init()도 안에서 해줌
	lcd_clear();

	lcd_set_cursor(0, 0);
	lcd_print("Dynamixel Ctrl");
	lcd_set_cursor(0, 1);
	lcd_print("Init...");

	sei();

	_delay_ms(500);     // 다이나믹셀 전원 안정화

	#if DXL_KEEP_TX_MODE
	// Status Return Level = 1로 하면 READ에만 응답함.
	// 이래야 PE2 계속 HIGH로 둬도 버스에서 안 부딪힘.
	dxl_write1(DXL_ID, ADDR_STATUS_RETURN_LVL, 1);
	_delay_ms(10);
	#endif

	dxl_write1(DXL_ID, ADDR_TORQUE_ENABLE, 1);
	_delay_ms(10);

	lcd_clear();

	while (1)
	{
		// PC에서 숫자 왔으면 목표 속도 갱신
		if (rxDigitNew)
		{
			uint8_t d;

			cli();      // ISR이랑 겹치면 안 되니까 잠깐 막고 복사
			d = rxDigit;
			rxDigitNew = 0;
			sei();

			goalSpeed = (uint16_t)((uint32_t)d * SPEED_MAX / 9UL);

			#if SPEED_ZERO_AS_SLOWEST
			if (goalSpeed == 0) 
				goalSpeed = 1;
			#endif

			dxl_write4(DXL_ID, ADDR_PROFILE_VELOCITY, goalSpeed);
		}

		// 가변저항으로 목표 위치
		adcVal = adc_read_avg();

		#if SCALE_POS_TO_4096
		goalPos = (uint16_t)((uint32_t)adcVal * 4092UL / 1023UL);
		#else
		goalPos = adcVal;
		#endif

		// ADC가 한두 칸씩 계속 떨려서, 어느 정도 변했을 때만 보냄
		if (lastPos == 0xFFFF || (goalPos > lastPos && goalPos - lastPos >= 4) || (lastPos > goalPos && lastPos - goalPos >= 4))
		{
			dxl_write4(DXL_ID, ADDR_GOAL_POSITION, goalPos);
			lastPos = goalPos;
		}

		// 값 바뀔 때만 LCD 갱신 (매번 하면 깜빡거림)
		// 실제로 보낸 값(lastPos)을 띄움
		if (goalSpeed != shownSpeed || lastPos != shownPos)
		{
			lcdShow(goalSpeed, lastPos);
			shownSpeed = goalSpeed;
			shownPos   = lastPos;
		}

		_delay_ms(50);
	}

	return 0;
}
