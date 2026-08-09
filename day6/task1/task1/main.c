#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#include "i2c_lcd.h"
#define IR_NUM          6       //PF2~PF7 = 6개
#define IR_ADC_START    2       //PF2 -> ADC2

#define MAF_N           8       // 이동평균 윈도우 크기

#define INVERT_NORM     1       // 1 : 밝을수록 ADC 작음 -> 정규화에서 반전

#define NORM_TH         80      // LED ON  : 0.80
#define NORM_TH_OFF     70      // LED OFF : 0.70 (히스테리시스)
#define MIN_RANGE       60      // max-min 이 이보다 좁으면 캘리브레이션 미완료로 간주

#define LED_DDR         DDRA
#define LED_PORT        PORTA
#define LED_MASK        0x3F    //PA0~PA5
#define LED_ACTIVE_LOW  1       // 이 보드는 0 일 때 LED ON (싱크 구동)

#define BTN_MASK        0x0C    // PD2, PD3
#define BTN1_BIT        0x04    // PD2 : 캘리브레이션 시작
#define BTN2_BIT        0x08    //PD3 : 출력 시작 

#define UART_BAUD       9600UL

#define SAMPLE_MS       5       // 메인 루프 주기
#define PRINT_MS        250     // 터미널/LCD 출력 주기

// 동작 상태 
#define ST_IDLE         0       // 대기 : SW1 을 기다림
#define ST_CALIB        1       // 캘리브레이션 중 : min/max 학습
#define ST_RUN          2       // 출력 중 : min/max 고정

typedef struct {
    uint16_t buf[MAF_N];
    uint8_t  idx;
    uint8_t  init;
    uint32_t sum;

    uint16_t raw;               // 원본 ADC 값
    uint16_t avg;               // MAF 통과 값 
    uint16_t min;               // 필터값 기준 최소 
    uint16_t max;               // 필터값 기준 최대 
    uint8_t  norm100;           // 정규화 값 x100 (0~100) 
} ir_ch_t;

static ir_ch_t ir[IR_NUM];

static void uart0_putc(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = (uint8_t)c;
}

static int uart0_stream_put(char c, FILE *stream)
{
    (void)stream;
    if (c == '\n')
		uart0_putc('\r');
    uart0_putc(c);
    return 0;
}
static FILE uart0_stdout = FDEV_SETUP_STREAM(uart0_stream_put, NULL, _FDEV_SETUP_WRITE);

static void uart0_init(void)
{
    uint16_t ubrr = (uint16_t)((F_CPU / (16UL * UART_BAUD)) - 1);

    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr & 0xFF);
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);      // 8-N-1 

    stdout = &uart0_stdout;
}
static void adc_init(void)
{
    DDRF  &= (uint8_t)~(0xFF << IR_ADC_START);   // PF2~PF7 입력 
    PORTF &= (uint8_t)~(0xFF << IR_ADC_START);   // 내부 풀업 off 

    ADMUX  = (1 << REFS0);                       // 기준전압 AVCC 
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    ADCSRA |= (1 << ADSC);                       // 더미 변환 
    while (ADCSRA & (1 << ADSC));
}

static uint16_t adc_read(uint8_t ch)
{
    ADMUX = (1 << REFS0) | (ch & 0x07);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADCW;
}

// ==================== 버튼 ==================== 
static void btn_init(void)
{
    // PD2, PD3 입력 + 내부 풀업. PD0/PD1 은 I2C 가 쓰므로 건드리지 않는다. 
    DDRD  &= (uint8_t)~BTN_MASK;
    PORTD |= BTN_MASK;
}

// 눌린 순간(High->Low)만 1로 반환. 20ms 디바운스 
static uint8_t btn_scan(void)
{
    static uint8_t last   = BTN_MASK;
    static uint8_t stable = BTN_MASK;
    static uint8_t cnt    = 0;
    uint8_t now  = (uint8_t)(PIND & BTN_MASK);
    uint8_t edge = 0;

    if (now != last) {                  // 변화 감지 -> 카운터 리셋 
        last = now;
        cnt  = 0;
    } else if (cnt < 4) {               // 4회(20ms) 연속 같은 값이면 확정 
        cnt++;
        if (cnt == 4) {
            edge   = (uint8_t)(stable & ~now);   // 1 -> 0 인 비트 = 눌림 
            stable = now;
        }
    }
    return edge;
}

// ==================== MAF + 정규화 ==================== 
static void ir_ch_reset(ir_ch_t *c)
{
    uint8_t i;
    for (i = 0; i < MAF_N; i++)
	 c->buf[i] = 0;
    c->idx = 0; 
	c->init = 0;  
	c->sum = 0;
    c->raw = 0;  
	c->avg  = 0;
    c->min = 1023;
    c->max = 0;
    c->norm100 = 0;
}

// 캘리브레이션 재시작 : 필터는 유지하고 min/max 만 초기화 
static void ir_ch_reset_range(ir_ch_t *c)
{
    c->min = 1023;
    c->max = 0;
    c->norm100 = 0;
}

// learn = 1 이면 min/max 를 갱신(캘리브레이션), 0 이면 고정(출력 모드) 
static void ir_ch_update(ir_ch_t *c, uint16_t sample, uint8_t learn)
{
    uint8_t  i;
    uint16_t range;

    c->raw = sample;

    //이동평균 필터 (재귀 형태) 
    //  xbar_k = xbar_(k-1) + ( x_k - x_(k-n) ) / n
    //  누적합만 갱신하므로 매 샘플 연산량이 O(1)
     
    if (!c->init) {
        for (i = 0; i < MAF_N; i++) 
			c->buf[i] = sample;
        c->sum  = (uint32_t)sample * MAF_N;
        c->init = 1;
    } else {
        c->sum -= c->buf[c->idx];
        c->sum += sample;
        c->buf[c->idx] = sample;
        c->idx = (uint8_t)((c->idx + 1) % MAF_N);
    }
    c->avg = (uint16_t)(c->sum / MAF_N);

    // min / max (캘리브레이션 중에만 학습) 
    if (learn) {
        if (c->avg < c->min) 
			c->min = c->avg;
        if (c->avg > c->max) 
			c->max = c->avg;
    }

    // 정규화
    //  기본형 : (x - min) / (max - min)
    //  이 보드는 밝을수록 ADC 가 작으므로 (max - x) / (max - min) 으로 뒤집어
    //  "빛이 많이 들어올수록 1에 가까운 값" 이 되게 한다.
     
    range = (uint16_t)(c->max - c->min);

    if (range >= MIN_RANGE) {
        uint32_t num;
        uint16_t n;

#if INVERT_NORM
        num = (c->avg >= c->max) ? 0UL : (uint32_t)(c->max - c->avg) * 100UL;
#else
        num = (c->avg <= c->min) ? 0UL : (uint32_t)(c->avg - c->min) * 100UL;
#endif
        n = (uint16_t)(num / range);
        c->norm100 = (n > 100) ? 100 : (uint8_t)n;
    } else {
        // 분모가 너무 작으면 노이즈만으로 0<->100 을 오가므로 0 으로 고정 
        c->norm100 = 0;
    }
}

// ==================== LED ==================== 
static void led_init(void)
{
    LED_DDR |= LED_MASK;
#if LED_ACTIVE_LOW
    LED_PORT |= LED_MASK;
#else
    LED_PORT &= (uint8_t)~LED_MASK;
#endif
}

static void led_all_off(void)
{
#if LED_ACTIVE_LOW
    LED_PORT |= LED_MASK;
#else
    LED_PORT &= (uint8_t)~LED_MASK;
#endif
}

// 정규화 값 0.80 이상이면 같은 번호 LED ON, 0.70 밑으로 내려가야 OFF 
static void led_update(void)
{
    static uint8_t state = 0;
    uint8_t i, out;

    for (i = 0; i < IR_NUM; i++) {
        if (ir[i].norm100 >= NORM_TH)          
			state |=  (uint8_t)(1 << i);
        else if (ir[i].norm100 <  NORM_TH_OFF) 
			state &= (uint8_t)~(1 << i);
    }
    out = (uint8_t)(state & LED_MASK);

#if LED_ACTIVE_LOW
    LED_PORT = (uint8_t)((LED_PORT & ~LED_MASK) | (uint8_t)(~out & LED_MASK));
#else
    LED_PORT = (uint8_t)((LED_PORT & ~LED_MASK) | out);
#endif
}

// ==================== 출력 ==================== 
static void print_uart(void)
{
    uint8_t i;

    // 열 위치 (0부터)
    //   8 : original      19 : filter(MAF)
    //  33 : min           39 : max           45 : norm
    // "IR 0 :  " 가 8칸이라 헤더 들여쓰기도 8칸으로 맞춘다. 
	
    printf("        original / filter(MAF) / min / max / norm\n");
    for (i = 0; i < IR_NUM; i++) {
        printf("IR %u :  %-11u%-14u%-6u%-6u%u.%02u\n",i, ir[i].raw, ir[i].avg, ir[i].min, ir[i].max,(unsigned)(ir[i].norm100 / 100), (unsigned)(ir[i].norm100 % 100));
    }
    printf("\n");
}

// 16x2 LCD 에 정규화 값을 소수로 한 줄 3개씩 표시.
//      0.87 0.02 0.95      <- 윗줄 : IR 0,1,2
//      0.64 0.01 1.00      <- 아랫줄 : IR 3,4,5
// 한 칸이 "0.87" 4글자 + 공백 1칸 = 5칸, 3개면 14칸이라 16x2 에 들어간다. 
static void print_lcd(void)
{
    char    line[17];
    char    tmp[8];
    uint8_t row, col, idx, k, p;

    for (row = 0; row < 2; row++) {
        for (p = 0; p < 16; p++) 
			line[p] = ' ';
        line[16] = '\0';

        for (col = 0; col < 3; col++) {
            idx = (uint8_t)(row * 3 + col);
            if (idx >= IR_NUM) 
				break;

            // norm100 은 0~100 이므로 100 이면 "1.00" 으로 표시된다 
            sprintf(tmp, "%u.%02u", (unsigned)(ir[idx].norm100 / 100), (unsigned)(ir[idx].norm100 % 100));

            for (k = 0; tmp[k] && (col * 5 + k) < 16; k++)
                line[col * 5 + k] = tmp[k];
        }
        lcd_set_cursor(0, row);
        lcd_print(line);
    }
}

// 캘리브레이션 중 UART : 원본 ADC 값만 한 줄로 출력 
static void print_uart_calib(void)
{
    uint8_t i;

    printf("CAL");
    for (i = 0; i < IR_NUM; i++) {
        printf(" %4u", ir[i].raw);
    }
    printf("\n");
}

// 캘리브레이션 중 LCD : 가장 좁은 채널의 range 를 보여준다 
static void print_lcd_calib(void)
{
    char     buf[32];   // sprintf 여유분 (LCD 에는 16자만 출력) 
    uint8_t  i, ok = 0;
    uint16_t worst = 1023;

    for (i = 0; i < IR_NUM; i++) {
        uint16_t r = (uint16_t)(ir[i].max - ir[i].min);
        if (r < worst) 
			worst = r;
        if (r >= MIN_RANGE) 
			ok++;
    }

    lcd_set_cursor(0, 0);
    lcd_print("CALIB  wave hand");
    sprintf(buf, "ok%u/%u min r=%4u", ok, (unsigned)IR_NUM, worst);
    buf[16] = '\0';
    lcd_set_cursor(0, 1);
    lcd_print(buf);
}

// 캘리브레이션 결과 리포트 
static void report_calib(void)
{
    uint8_t i, ng = 0;

    printf("--- calibration result ---\n");
    printf("      min /  max / range\n");
    for (i = 0; i < IR_NUM; i++) {
        uint16_t range = (uint16_t)(ir[i].max - ir[i].min);
        uint8_t  bad   = (range < MIN_RANGE);
        if (bad) ng++;
        printf("IR %u : %4u  %4u  %4u  %s\n", i, ir[i].min, ir[i].max, range, bad ? "NG" : "OK");
    }
    if (ng) 
		printf("warning : %u ch range too narrow. press SW1 to redo.\n", ng);
    printf("\n");
}

// ==================== main ==================== 
int main(void)
{
    uint8_t  i;
    uint8_t  state = ST_IDLE;
    uint8_t  edge;
    uint16_t tick = 0;

    // PF4~PF7 은 기본이 JTAG 핀이라 ADC 로 쓰려면 해제해야 한다.
    // JTD 는 4클럭 안에 두 번 써야 적용된다. (JTAG 디버거 사용 시 주의) 
    MCUCSR |= (1 << JTD);
    MCUCSR |= (1 << JTD);

    uart0_init();
    adc_init();
    led_init();
    btn_init();
    lcd_init();                     // 내부에서 i2c_init() 호출 

    for (i = 0; i < IR_NUM; i++) 
		ir_ch_reset(&ir[i]);

    printf("\n=== IR sensor / MAF / normalize ===\n");
    printf("SW1(PD2) : start calibration\n");
    printf("SW2(PD3) : finish calibration and start output\n\n");

    lcd_set_cursor(0, 0);
    lcd_print("IR filter  ready");
    lcd_set_cursor(0, 1);
    lcd_print("SW1: calibrate  ");

    while (1) {
        edge = btn_scan();

        // 센서 갱신. 캘리브레이션 상태에서만 min/max 학습 
        for (i = 0; i < IR_NUM; i++)
            ir_ch_update(&ir[i], adc_read((uint8_t)(IR_ADC_START + i)), (uint8_t)(state == ST_CALIB));

        // SW1 : 캘리브레이션 시작(또는 재시작)
        if (edge & BTN1_BIT) {
            for (i = 0; i < IR_NUM; i++) 
				ir_ch_reset_range(&ir[i]);
            led_all_off();
            state = ST_CALIB;
            tick  = PRINT_MS;                   // 즉시 화면 갱신 
            printf("--- calibration start : wave your hand ---\n");
            printf("CAL  original ADC per channel  (SW2 = finish)\n");
            lcd_clear();
        }

        // ---- SW2 : 캘리브레이션 종료 -> 출력 시작 ---- 
        if (edge & BTN2_BIT) {
            if (state == ST_CALIB) 
				report_calib();
            state = ST_RUN;
            tick  = PRINT_MS;
            lcd_clear();
        }

        if (state == ST_RUN) 
			led_update();

        //주기적 출력
        tick = (uint16_t)(tick + SAMPLE_MS);
        if (tick >= PRINT_MS) {
            tick = 0;
            if (state == ST_CALIB) {
                print_uart_calib();     
                print_lcd_calib();
            } else if (state == ST_RUN) {
                print_uart();
                print_lcd();
            }
        }

        _delay_ms(SAMPLE_MS);
    }
    return 0;
}