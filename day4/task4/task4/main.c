/*
 * 과제 4 (심화) - RAW / FILTERED 동시 출력
 *
 * PSD 센서값을 읽어서 필터 적용 전후를 한 줄에 같이 찍는다.
 *   RAW:  412 | FILTERED:  405 | DISTANCE:  15.2cm
 *
 * 필터는 중앙값(median)을 썼다. 과제3에서 값을 찍어보니 PSD가 가끔
 * 확 튀는 값을 내는데, 이동평균은 튄 값이 평균에 그대로 섞여버린다.
 * 중앙값은 정렬해서 가운데를 고르니까 튄 값 하나쯤은 그냥 버려진다.
 * USE_MEDIAN을 0으로 바꾸면 이동평균으로 돌려서 비교해볼 수 있다.
 *
 * 센서: SHARP GP2Y0A02YK0F, 20~150cm
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
#define SAMPLE_PERIOD_MS    20      // 샘플 하나 뽑는 주기
#define PRINT_PERIOD_MS     200     // 화면에 찍는 주기
#define FILTER_N            5       // 필터 윈도우 (홀수여야 중앙값이 딱 떨어짐)
#define USE_MEDIAN          1       // 0으로 바꾸면 이동평균으로 돌려볼 수 있다

#define BAUD                57600UL
#define UBRR_VAL            ((((F_CPU) + 4UL*(BAUD)) / (8UL*(BAUD))) - 1)

#define DIR_PIN             PE2     // MAX485 방향제어

// 판정 경계값
#define ADC_DISCONNECT      30      // 센서 미연결
#define ADC_FAR_LIMIT       92      // 150cm
#define ADC_NEAR_LIMIT      512     // 20cm

// 센서 특성표. ADC값 -> 거리(0.1cm 단위)
// float을 안 쓰려고 거리를 10배로 저장했다.
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

static volatile uint8_t do_sample = 0;
static volatile uint8_t do_print  = 0;


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


#if USE_MEDIAN
// 중앙값 필터. 복사해서 정렬한 뒤 가운데 값을 돌려준다.
static uint16_t median_of(uint16_t *src, uint8_t n)
{
    uint16_t buf[FILTER_N];
    uint8_t i, j;
    uint16_t key;

    for (i = 0; i < n; i++) buf[i] = src[i];

    // 삽입정렬. 5개뿐이라 이걸로 충분하다.
    for (i = 1; i < n; i++) {
        key = buf[i];
        j = i;
        while (j > 0 && buf[j-1] > key) {
            buf[j] = buf[j-1];
            j--;
        }
        buf[j] = key;
    }

    return buf[n / 2];
}
#else
// 이동평균 필터. 구현은 간단한데 튄 값이 평균에 그대로 섞인다.
static uint16_t moving_avg(uint16_t *src, uint8_t n)
{
    uint32_t sum = 0;
    uint8_t i;

    for (i = 0; i < n; i++) sum += src[i];
    return (uint16_t)(sum / n);
}
#endif


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


// Timer1 CTC로 1ms 만들고, 카운트해서 각각의 주기를 만든다.
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
    static uint16_t sample_cnt = 0;
    static uint16_t print_cnt  = 0;

    if (++sample_cnt >= SAMPLE_PERIOD_MS) { sample_cnt = 0; do_sample = 1; }
    if (++print_cnt  >= PRINT_PERIOD_MS)  { print_cnt  = 0; do_print  = 1; }
}


int main(void)
{
    char line[64];
    uint16_t samples[FILTER_N];
    uint8_t idx = 0;
    uint8_t last = 0;           // 방금 넣은 자리 = 가장 최근 원시값
    uint8_t filled = 0;
    uint16_t raw, filtered, dist;
    uint8_t status;

    uart_init();
    adc_init();
    timer1_init();
    sei();

    _delay_ms(50);      // 센서 기동 대기
    adc_read();         // 첫 변환은 버림

    uart_puts("\r\nPSD Distance Meter (RAW vs FILTERED)\r\n");
#if USE_MEDIAN
    uart_puts("filter: median, window 5\r\n");
#else
    uart_puts("filter: moving average, window 5\r\n");
#endif
    sprintf(line, "period: sample %dms / print %dms\r\n\r\n",
            SAMPLE_PERIOD_MS, PRINT_PERIOD_MS);
    uart_puts(line);

    while (1) {
        // 샘플 모으기 (원형 버퍼)
        if (do_sample) {
            do_sample = 0;

            samples[idx] = adc_read();
            last = idx;
            idx = (idx + 1) % FILTER_N;

            if (filled < FILTER_N) filled++;
        }

        // 결과 출력
        if (do_print) {
            do_print = 0;

            if (filled < FILTER_N) continue;    // 아직 버퍼가 안 찼다

            raw = samples[last];                // 필터 안 거친 값
#if USE_MEDIAN
            filtered = median_of(samples, FILTER_N);
#else
            filtered = moving_avg(samples, FILTER_N);
#endif

            status = adc_to_dist(filtered, &dist);

            switch (status) {
            case PSD_OK:
                // AVR의 sprintf는 %f가 기본으로 안 되니까 정수부/소수부를 따로 찍는다
                sprintf(line, "RAW: %4u | FILTERED: %4u | DISTANCE: %3u.%1ucm\r\n",
                        raw, filtered, dist / 10, dist % 10);
                break;

            case PSD_TOO_CLOSE:
                sprintf(line, "RAW: %4u | FILTERED: %4u | [WARN] too close (<20cm)\r\n",
                        raw, filtered);
                break;

            case PSD_TOO_FAR:
                sprintf(line, "RAW: %4u | FILTERED: %4u | [WARN] out of range\r\n",
                        raw, filtered);
                break;

            default:
                sprintf(line, "RAW: %4u | FILTERED: %4u | [ERROR] not connected\r\n",
                        raw, filtered);
                break;
            }

            uart_puts(line);
        }
    }

    return 0;
}