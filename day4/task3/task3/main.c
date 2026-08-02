/*
 * 과제 3 - PSD 거리 측정
 *
 * PF1(ADC1)로 PSD 센서 값을 읽어서 cm 단위로 바꾼 뒤 UART로 출력.
 * 센서는 SHARP GP2Y0A02YK0F, 측정범위 20~150cm.
 *
 * 주의: 이 센서는 20cm보다 가까워지면 출력전압이 오히려 떨어진다.
 *       그래서 같은 ADC값이 두 개의 거리를 의미할 수 있어서
 *       20cm 미만은 거리를 계산하지 않고 경고만 띄운다.
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

#define PSD_CH              1       // PF1
#define MEASURE_PERIOD_MS   200

#define BAUD                57600UL
#define UBRR_VAL            ((((F_CPU) + 4UL*(BAUD)) / (8UL*(BAUD))) - 1)

#define DIR_PIN             PE2     // MAX485 방향제어

// 판정 경계값
#define ADC_DISCONNECT      30      // 센서 미연결
#define ADC_FAR_LIMIT       92      // 150cm
#define ADC_NEAR_LIMIT      512     // 20cm

// 센서 특성표. ADC값 -> 거리(0.1cm 단위)
// float을 안 쓰려고 거리를 10배로 저장했다 : AVR에 FPU가 없기 때문에 float을 사용하는것이 비효율적임
typedef struct {
    uint16_t adc;
    uint16_t dist;
} PsdPoint;

static const PsdPoint psd_lut[] PROGMEM = {
    {  92, 1500 }, {  98, 1400 }, { 106, 1300 }, { 115, 1200 },
    { 123, 1100 }, { 133, 1000 }, { 147,  900 }, { 164,  800 },
    { 180,  700 }, { 205,  600 }, { 235,  500 }, { 262,  450 },
    { 297,  400 }, { 327,  350 }, { 368,  300 }, { 440,  250 },
    { 512,  200 }
};

#define LUT_SIZE (sizeof(psd_lut) / sizeof(psd_lut[0]))

enum { PSD_OK, PSD_TOO_CLOSE, PSD_TOO_FAR, PSD_DISCONNECTED };

static volatile uint8_t do_measure = 0;


static void uart_init(void)
{
    DDRE  |=  (1 << PE1);
    DDRE  &= ~(1 << PE0);
    PORTE |=  (1 << PE0);

    DDRE  |= (1 << DIR_PIN);
    PORTE |= (1 << DIR_PIN);    // HIGH로 두어야 PC 입력이 막히지 않는다

    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)UBRR_VAL;

    UCSR0A = (1 << U2X0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    UCSR0B = (1 << TXEN0);
}

static void uart_putchar(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

static void uart_puts(const char *s)
{
    while (*s) uart_putchar(*s++);
}


static void adc_init(void)
{
    DDRF  &= ~(1 << PF1);
    PORTF &= ~(1 << PF1);

    ADMUX  = (1 << REFS0) | PSD_CH;
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

static uint16_t adc_read(void)
{
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADCW;
}


// ADC값을 거리로 변환. 반환값은 상태, 거리는 dist에 담아준다.
static uint8_t adc_to_dist(uint16_t adc, uint16_t *dist)
{
    uint8_t i;
    uint16_t a0, a1, d0, d1;

    *dist = 0;

    if (adc < ADC_DISCONNECT) return PSD_DISCONNECTED;
    if (adc < ADC_FAR_LIMIT)  return PSD_TOO_FAR;
    if (adc > ADC_NEAR_LIMIT) return PSD_TOO_CLOSE;

    // 표에서 해당 구간을 찾아 선형보간
    for (i = 0; i < LUT_SIZE - 1; i++) {
        a0 = pgm_read_word(&psd_lut[i].adc);
        a1 = pgm_read_word(&psd_lut[i+1].adc);

        if (adc >= a0 && adc <= a1) {
            d0 = pgm_read_word(&psd_lut[i].dist);
            d1 = pgm_read_word(&psd_lut[i+1].dist);
            // ADC가 커질수록 거리는 줄어드니까 d0 > d1
            *dist = d0 - (uint16_t)(((uint32_t)(d0 - d1) * (adc - a0)) / (a1 - a0));
            return PSD_OK;
        }
    }

    return PSD_TOO_FAR;
}


// Timer1 CTC로 1ms 만들고, 카운트해서 측정 주기를 만든다.
// 16MHz / 64 = 250kHz, 250개 세면 1ms
static void timer1_init(void)
{
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    OCR1A  = 249;
    TCNT1  = 0;
    TIMSK |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect)
{
    static uint16_t cnt = 0;

    if (++cnt >= MEASURE_PERIOD_MS) {
        cnt = 0;
        do_measure = 1;
    }
}


int main(void)
{
    char line[48];
    uint16_t adc, dist;
    uint8_t status;

    uart_init();
    adc_init();
    timer1_init();
    sei();

    _delay_ms(50);      // 센서 기동 대기
    adc_read();         // 첫 변환은 버림

    uart_puts("\r\nPSD Distance Meter\r\n");
    sprintf(line, "period %dms\r\n\r\n", MEASURE_PERIOD_MS);
    uart_puts(line);

    while (1) {
        if (!do_measure) continue;
        do_measure = 0;

        adc = adc_read();
        status = adc_to_dist(adc, &dist);

        switch (status) {
        case PSD_OK:
            // AVR의 sprintf는 %f가 기본으로 안 되니까 정수부/소수부를 따로 찍는다
            sprintf(line, "ADC:%4u  Distance: %3u.%1u cm\r\n",
                    adc, dist / 10, dist % 10);
            break;

        case PSD_TOO_CLOSE:
            sprintf(line, "ADC:%4u  [WARN] too close (<20cm)\r\n", adc);
            break;

        case PSD_TOO_FAR:
            sprintf(line, "ADC:%4u  [WARN] out of range\r\n", adc);
            break;

        default:
            sprintf(line, "ADC:%4u  [ERROR] sensor not connected\r\n", adc);
            break;
        }

        uart_puts(line);
    }

    return 0;
}	