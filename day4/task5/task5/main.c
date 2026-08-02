// 과제5 - 서보모터(SG90) 각도 제어
// PC에서 0~180 숫자 입력하면 그 각도로 서보 이동
// 시작하면 일단 90도(원점)로 복귀

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#define UART0_BAUD          57600UL
#define UBRR0_VAL           ((((F_CPU) + 4UL * (UART0_BAUD)) / (8UL * (UART0_BAUD))) - 1)

#define ANGLE_MIN           0
#define ANGLE_MAX           180
#define HOME_ANGLE          90

#define PULSE_US_MIN        600     // 0도 펄스폭
#define PULSE_US_MAX        2400    // 180도 펄스폭

#define LINE_BUF_SIZE       8

// PE0에 MAX485 RO랑 PC TX가 같이 물려있어서 PE2를 LOW로 두면
// RO가 PE0을 잡아버려서 PC 입력이 안 들어옴. 그래서 계속 HIGH로 둠.
// (송신은 PE1로 나가니까 출력은 멀쩡해 보이는 게 함정)
#define DIR_PIN             PE2
#define DIR_TX()            (PORTE |=  (1 << DIR_PIN))
#define DIR_RX()            (PORTE &= ~(1 << DIR_PIN))

static volatile char    g_buf[LINE_BUF_SIZE];
static volatile uint8_t g_len      = 0;
static volatile uint8_t g_ready    = 0;
static volatile uint8_t g_overflow = 0;

// 서보 PWM 세팅 (Timer1, Fast PWM, TOP=ICR1)
// 분주비 8이라 1카운트가 0.5us. 20ms 주기 맞추려면 ICR1 = 39999
static void servo_pwm_init(void)
{
	DDRB |= (1 << PB7);   // OC1C

	TCCR1A = (1 << COM1C1) | (1 << WGM11);
	ICR1   = 39999;
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
}

// 각도 넣으면 알아서 펄스폭 계산해서 OCR1C에 넣어줌
static void servo_set_angle(uint8_t angle)
{
	uint32_t us = PULSE_US_MIN
	+ ((uint32_t)(PULSE_US_MAX - PULSE_US_MIN) * angle)
	/ (ANGLE_MAX - ANGLE_MIN);

	OCR1C = (uint16_t)(us * 2);   // 0.5us 단위라 x2
}

static void uart0_init(void)
{
	DDRE  |=  (1 << PE1);
	DDRE  &= ~(1 << PE0);
	PORTE |=  (1 << PE0);   // 안 꽂았을 때 뜨는 거 방지용 풀업

	DDRE |= (1 << DIR_PIN);
	DIR_TX();

	UBRR0H = (uint8_t)(UBRR0_VAL >> 8);
	UBRR0L = (uint8_t)(UBRR0_VAL);

	UCSR0A = (1 << U2X0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
}

static void uart0_putchar(char c)
{
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = (uint8_t)c;
}

static void uart0_puts(const char *s)
{
	while (*s) uart0_putchar(*s++);
}

// sprintf 안 쓰고 직접 구현 / 인덱스 뒤에서부터 읽으면서 실제값을 맞춤
static void uart0_put_int(int16_t v)
{
	char     tmp[8];
	uint8_t  i = 0;
	uint16_t u;

	if (v < 0) { uart0_putchar('-'); u = (uint16_t)(-v); }
	else       { u = (uint16_t)v; }

	if (u == 0) { uart0_putchar('0'); return; }

	while (u > 0 && i < sizeof(tmp)) { tmp[i++] = (char)('0' + (u % 10)); u /= 10; }
	while (i > 0) uart0_putchar(tmp[--i]);
}

// 글자 하나씩 받아서 버퍼에 쌓다가 엔터 오면 한 줄 완성
// RXCIE0 켰으면 이 ISR 꼭 있어야 함, 없으면 리셋 걸림
ISR(USART0_RX_vect)
{
	uint8_t c = UDR0;

	if (c == '\r' || c == '\n')
	{
		if (g_len > 0 && !g_ready)   // 이전 줄 아직 처리 안 됐으면 덮어쓰지 않음
		{
			g_buf[g_len] = '\0';
			g_ready = 1;
		}
		return;
	}

	if (c == '\b' || c == 0x7F)   // 백스페이스
	{
		if (g_len > 0) g_len--;
		return;
	}

	if (g_len < (LINE_BUF_SIZE - 1)) g_buf[g_len++] = (char)c;
	else                             g_overflow = 1;
}

// "abc", "18a", 빈 문자열 다 걸러냄
static uint8_t parse_int(const char *s, int16_t *out)
{
	int16_t  sign   = 1;
	int32_t  value  = 0;
	uint8_t  i      = 0;
	uint8_t  digits = 0;

	if (s[0] == '\0') return 0;

	if      (s[0] == '-') { sign = -1; i = 1; }
	else if (s[0] == '+') { i = 1; }

	for (; s[i] != '\0'; i++)
	{
		if (s[i] < '0' || s[i] > '9') return 0;
		value = value * 10 + (s[i] - '0');
		digits++;
		if (value > 32000) return 0;
	}

	if (digits == 0) return 0;

	*out = (int16_t)(sign * value);
	return 1;
}

int main(void)
{
	uart0_init();
	servo_pwm_init();
	sei();

	// 이미 원점 근처면 움직이는 게 안 보여서, 확실히 티나게 하려고
	// 0도 안 거치고 바로 90도로 감 (필요하면 0도 먼저 찍고 와도 됨)
	servo_set_angle(HOME_ANGLE);
	_delay_ms(700);   // 실제로 서보 움직일 시간 좀 줘야 함

	uart0_puts("\r\n===== Servo Angle Control (SG90) =====\r\n");
	uart0_puts("Homed to ");
	uart0_put_int(HOME_ANGLE);
	uart0_puts(" deg.\r\n");
	uart0_puts("Enter angle (0-180) + Enter.\r\n");
	uart0_puts("=======================================\r\n");

	while (1)
	{
		if (!g_ready) continue;

		char    line[LINE_BUF_SIZE];
		uint8_t of;
		uint8_t i;
		int16_t value;

		// ISR이 버퍼 만지는 중에 읽으면 섞일 수 있어서 잠깐 막고 복사
		cli();
		for (i = 0; i < LINE_BUF_SIZE; i++) line[i] = g_buf[i];
		line[LINE_BUF_SIZE - 1] = '\0';
		of         = g_overflow;
		g_len      = 0;
		g_ready    = 0;
		g_overflow = 0;
		sei();

		if (of)
		{
			uart0_puts("[WARN] input too long\r\n");
		}
		else if (!parse_int(line, &value))
		{
			uart0_puts("[WARN] not a number: ");
			uart0_puts(line);
			uart0_puts("\r\n");
		}
		else if (value < ANGLE_MIN || value > ANGLE_MAX)
		{
			// 범위 밖이면 그냥 무시하고 이전 각도 유지
			uart0_puts("[WARN] out of range (0-180): ");
			uart0_put_int(value);
			uart0_puts(" -> not moved\r\n");
		}
		else
		{
			servo_set_angle((uint8_t)value);

			uart0_puts("OK -> ");
			uart0_put_int(value);
			uart0_puts(" deg\r\n");
		}
	}

	return 0;
}