/*
 * 라인트레이서 (ATmega128, UART 미사용)
 *
 *   IR    : PF2~PF7 (ADC2~ADC7) 6채널. IR0=왼쪽 끝 ~ IR5=오른쪽 끝
 *   PSD   : PF0(오른쪽), PF1(앞). 차단바 거리 측정
 *   LED   : PA0~PA5 (IR 과 1:1, Active-Low)
 *   SW1   : PD2  흰색 구간 캘리브레이션 (토글 : 시작 / 저장)
 *   SW2   : PD3  검은색 구간 캘리브레이션 (토글 : 시작 / 저장)
 *   SW3   : PE4  디버그 모드 토글
 *   SW4   : PE5  주행 시작
 *   MOTOR : L298N, PORTB           LCD : I2C PCF8574 (SCL=PD0, SDA=PD1)
 *
 *   === 구간 관리 ===
 *   g_section 이 현재 구간을 나타내고, g_sec_count 는 그 구간 안에서만
 *   쓰이는 카운트다. 구간이 바뀌면 카운트는 0 으로 리셋된다.
 *   구간이 다르면 카운트 값이 같아도 서로 간섭하지 않는다.
 *
 *     SEC_FIG8   : 출발 ~ 8자.  카운트 1,2,3,4 (4 에서 평사 진입)
 *     SEC_LANE   : 평행사변형.  카운트 미사용
 *     SEC_CORNER : ㄱ자 ~ 차단바 통과. ㄱ자 회전 종료 시 카운트 1
 *     SEC_PARK   : T자 회전 종료 후 ~ 주차 탈출.
 *                  1=진입완료 2=허공직진 3=정지/180도 4=탈출 좌회전
 *     SEC_BLACK  : 반전 이후.
 *                  카운트 1        : 6초 잠금
 *                  카운트 2,3,4,5,7,8,9 : 좌우 LED 개수 비교 -> 900ms 직진 후
 *                                     제자리 회전(피벗 아님)
 *                  카운트 6,10     : 회전 없이 0.5초 잠금만
 *                  카운트 11       : 회전 없이 1초 잠금만
 *                  카운트 12       : 무조건 좌회전 (직진 후 제자리 회전)
 *                  카운트 13       : 500ms 정지 -> 앞 PSD 보며 전진 ->
 *                                     정지되는 순간 카운트를 0으로 리셋하고
 *                                     벽 탐지 구간(black_wall_update)으로 전환
 *                  벽 탐지 구간    : 전진 -> 앞 PSD 로 벽 인식 -> 좌회전 90도
 *                                     -> 전진 -> 교차로 인식 시 그 교차로를
 *                                     "새 시퀀스 카운트 1"로 세운다.
 *                  [새 시퀀스]     : 벽 탐지 구간(WALL_ST_DONE) 통과 이후.
 *                                     카운트 2 : 무조건 우회전 (직진 후 제자리 회전)
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>

#include "i2c_lcd.h"

/* ==================== 테스트 옵션 ====================
 * 1 이면 SW4 로 시작할 때 주차를 통과한 직후 상태로 점프한다.
 * 대회 때는 반드시 0 으로 되돌릴 것.
 * 주의 : 1 인 상태에서는 SEC_BLACK (반전 구간) 로 가지 않으므로
 * 반전/ㄱ자 로직을 테스트하려면 0 으로 두고 실제로 주차까지 통과시켜야 한다. */
#define TEST_SKIP_TO_PARK_EXIT  0

/* 1 이면 반전 플래그 감지를 켠다. */
#define ENABLE_INVERT           1

/* ==================== 구간 ==================== */
#define SEC_FIG8    0   /* 출발 ~ 8자 (S자 포함) */
#define SEC_LANE    1   /* 평행사변형 */
#define SEC_CORNER  2   /* ㄱ자 ~ 차단바 통과 */
#define SEC_PARK    3   /* T자 회전 종료 ~ 주차 탈출 */
#define SEC_BLACK   4   /* 반전 이후 */

static uint8_t  g_section   = SEC_FIG8;
static uint16_t g_sec_count = 0;    /* 현재 구간 안의 카운트 */

/* ==================== 주행 시작 직후 카운트 잠금 ==================== */
#define RUN_START_LOCK_MS   8500
static uint16_t g_run_lock_ms = 0;

/* ==================== 센서 ==================== */
#define IR_NUM          6
#define IR_ADC_START    2       /* PF2 -> ADC2 */

#define MAF_N           4       /* 이동평균 창 */

/* 캘리브레이션 유효 범위. 검은 바탕은 반사광 차이가 작아 따로 둔다. */
#define MIN_RANGE       200     /* 흰 구간 */
#define MIN_RANGE_BLACK 50      /* 검은 구간 */

/* 캘리브레이션 세트 */
#define CAL_WHITE       0       /* 흰 바탕 / 검은 선 */
#define CAL_BLACK       1       /* 검은 바탕 / 흰 선 */
#define CAL_NUM         2
#define CAL_NONE        0xFF    /* 학습 안 함 */

/* LED 점등 (히스테리시스). ir_sig 기준.
 * 검은 구간은 정규화 범위가 좁아 신호가 낮게 나오므로 따로 둔다. */
#define LED_SIG_ON          85
#define LED_SIG_OFF         60
#define LED_SIG_ON_BLACK    45
#define LED_SIG_OFF_BLACK   30

#define LED_DDR         DDRA
#define LED_PORT        PORTA
#define LED_MASK        0x3F
#define LED_ACTIVE_LOW  1

/* ==================== 버튼 ==================== */
#define BTN_MASK        0x0C
#define BTN1_BIT        0x04    /* PD2 */
#define BTN2_BIT        0x08    /* PD3 */
#define BTN3_MASK       0x10    /* PE4 */
#define BTN4_MASK       0x20    /* PE5 */

/* ==================== 모터 (L298N) ==================== */
#define MOTOR_DDR       DDRB
#define MOTOR_PORT      PORTB
#define IN1             PB0     /* 왼쪽 */
#define IN2             PB1     /* 왼쪽 */
#define IN3             PB2     /* 오른쪽 */
#define IN4             PB3     /* 오른쪽 */
#define ENA             PB5     /* OC1A, 왼쪽 속도 */
#define ENB             PB6     /* OC1B, 오른쪽 속도 */

#define BASE_SPEED      140
#define MAX_SPEED       255
#define MIN_SPEED       0
#define KP              24      /* 비례 게인 */

#define LINE_LOST_TH        40  /* 신호 합이 이보다 작으면 "선 놓침" */
#define LINE_LOST_TH_BLACK  25  /* 검은 구간 전용 */
#define SEARCH_SPEED        180 /* 재탐색 회전 속도 */
#define SEARCH_FLIP_MS       600 /* 이 시간까지 못 찾으면 반대 방향으로 전환 */

/* 급회전. 모터 데드존이 100 부근이라 안쪽은 0(피벗)으로 둔다.
 * SEC_FIG8 에서만 쓴다. */
#define SHARP_TURN_TH_ON    75
#define SHARP_TURN_TH_OFF   50
#define SHARP_OPPOSITE_TH   45
#define SHARP_TURN_SPEED    220
#define SHARP_INNER_SPEED   0

#define SAMPLE_MS       5       /* 메인 루프 주기 */
#define LCD_MS          150     /* 캘리브레이션 중 LCD 갱신 */
#define DEBUG_LCD_MS    120     /* 디버그 모드 LCD 갱신 */

/* ==================== 주행 상태 ==================== */
#define ST_IDLE         0
#define ST_CALIB        1
#define ST_RUN          2
#define ST_DEBUG        3       /* 모터 정지, 센서값만 표시 */
#define ST_DONE         4       /* 주행 완료 */

/* ==================== IR 채널 ==================== */
typedef struct {
    uint16_t buf[MAF_N];
    uint8_t  idx;
    uint8_t  init;
    uint32_t sum;

    uint16_t avg;
    uint16_t min[CAL_NUM];
    uint16_t max[CAL_NUM];
    uint8_t  norm100;           /* 0~100, 반사광 많을수록 큼 */
} ir_ch_t;

static ir_ch_t ir[IR_NUM];

/* 현재 사용 중인 캘리브레이션 세트 */
static uint8_t g_cal_set = CAL_WHITE;

/* 선 위에 있는 정도 (0~100). 선일수록 크다.
 * 흰 바탕이면 선이 검고, 검은 바탕이면 선이 희다. */
static uint8_t ir_sig(uint8_t i)
{
    if (g_cal_set == CAL_WHITE) return (uint8_t)(100 - ir[i].norm100);
    return ir[i].norm100;
}

/* ==================== EEPROM ==================== */
#define EE_MAGIC_VAL    0x5B

static uint8_t  ee_magic EEMEM;
static uint16_t ee_min[CAL_NUM][IR_NUM] EEMEM;
static uint16_t ee_max[CAL_NUM][IR_NUM] EEMEM;

static uint8_t calib_load_from_eeprom(void)
{
    uint8_t s, i;

    if (eeprom_read_byte(&ee_magic) != EE_MAGIC_VAL) return 0;

    for (s = 0; s < CAL_NUM; s++)
        for (i = 0; i < IR_NUM; i++) {
            ir[i].min[s] = eeprom_read_word(&ee_min[s][i]);
            ir[i].max[s] = eeprom_read_word(&ee_max[s][i]);
        }
    return 1;
}

static void calib_save_to_eeprom(void)
{
    uint8_t s, i;

    for (s = 0; s < CAL_NUM; s++)
        for (i = 0; i < IR_NUM; i++) {
            eeprom_update_word(&ee_min[s][i], ir[i].min[s]);
            eeprom_update_word(&ee_max[s][i], ir[i].max[s]);
        }
    eeprom_update_byte(&ee_magic, EE_MAGIC_VAL);
}

/* ==================== ADC ==================== */
static void adc_init(void)
{
    /* IR : PF2~PF7 */
    DDRF  &= (uint8_t)~(0xFF << IR_ADC_START);
    PORTF &= (uint8_t)~(0xFF << IR_ADC_START);

    /* PSD : PF0, PF1 */
    DDRF  &= (uint8_t)~0x03;
    PORTF &= (uint8_t)~0x03;

    ADMUX  = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    ADCSRA |= (1 << ADSC);                       /* 더미 변환 */
    while (ADCSRA & (1 << ADSC));
}

static uint16_t adc_read(uint8_t ch)
{
    ADMUX = (1 << REFS0) | (ch & 0x07);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADCW;
}

/* ==================== 버튼 ==================== */
static void btn_init(void)
{
    /* PD0/PD1 은 I2C 가 쓰므로 건드리지 않는다 */
    DDRD  &= (uint8_t)~BTN_MASK;
    PORTD |= BTN_MASK;

    DDRE  &= (uint8_t)~(BTN3_MASK | BTN4_MASK);
    PORTE |= (BTN3_MASK | BTN4_MASK);
}

/* 눌린 순간만 1 반환 (20ms 디바운스) */
static uint8_t btn_scan(void)
{
    static uint8_t last   = BTN_MASK;
    static uint8_t stable = BTN_MASK;
    static uint8_t cnt    = 0;
    uint8_t now  = (uint8_t)(PIND & BTN_MASK);
    uint8_t edge = 0;

    if (now != last) {
        last = now;
        cnt  = 0;
    } else if (cnt < 4) {
        cnt++;
        if (cnt == 4) {
            edge   = (uint8_t)(stable & ~now);
            stable = now;
        }
    }
    return edge;
}

static uint8_t btn3_scan(void)
{
    static uint8_t last   = BTN3_MASK;
    static uint8_t stable = BTN3_MASK;
    static uint8_t cnt    = 0;
    uint8_t now  = (uint8_t)(PINE & BTN3_MASK);
    uint8_t edge = 0;

    if (now != last) {
        last = now;
        cnt  = 0;
    } else if (cnt < 4) {
        cnt++;
        if (cnt == 4) {
            edge   = (uint8_t)(stable & ~now);
            stable = now;
        }
    }
    return edge;
}

static uint8_t btn4_scan(void)
{
    static uint8_t last   = BTN4_MASK;
    static uint8_t stable = BTN4_MASK;
    static uint8_t cnt    = 0;
    uint8_t now  = (uint8_t)(PINE & BTN4_MASK);
    uint8_t edge = 0;

    if (now != last) {
        last = now;
        cnt  = 0;
    } else if (cnt < 4) {
        cnt++;
        if (cnt == 4) {
            edge   = (uint8_t)(stable & ~now);
            stable = now;
        }
    }
    return edge;
}

/* ==================== MAF + 정규화 ==================== */
static void ir_ch_reset(ir_ch_t *c)
{
    uint8_t i, s;
    for (i = 0; i < MAF_N; i++) c->buf[i] = 0;
    c->idx = 0;  c->init = 0;  c->sum = 0;
    c->avg = 0;
    for (s = 0; s < CAL_NUM; s++) { c->min[s] = 1023; c->max[s] = 0; }
    c->norm100 = 0;
}

static void ir_ch_reset_range(ir_ch_t *c, uint8_t set)
{
    c->min[set] = 1023;
    c->max[set] = 0;
    c->norm100  = 0;
}

/* learn_set : CAL_NONE 이면 학습 안 함, 아니면 그 세트를 학습 */
static void ir_ch_update(ir_ch_t *c, uint16_t sample, uint8_t learn_set)
{
    uint8_t  i;
    uint8_t  s = g_cal_set;
    uint16_t mn, mx, min_range;

    if (!c->init) {
        for (i = 0; i < MAF_N; i++) c->buf[i] = sample;
        c->sum  = (uint32_t)sample * MAF_N;
        c->init = 1;
    } else {
        c->sum -= c->buf[c->idx];
        c->sum += sample;
        c->buf[c->idx] = sample;
        c->idx = (uint8_t)((c->idx + 1) % MAF_N);
    }
    c->avg = (uint16_t)(c->sum / MAF_N);

    if (learn_set < CAL_NUM) {
        if (c->avg < c->min[learn_set]) c->min[learn_set] = c->avg;
        if (c->avg > c->max[learn_set]) c->max[learn_set] = c->avg;
        s = learn_set;      /* 캘리브레이션 중에는 그 세트로 표시 */
    }

    mn        = c->min[s];
    mx        = c->max[s];
    min_range = (s == CAL_BLACK) ? MIN_RANGE_BLACK : MIN_RANGE;

    /* max > min 검사가 먼저다. 캘리브레이션 전에는 min=1023, max=0 이라
     * (max-min) 이 언더플로우로 거대한 값이 된다. */
    if (mx > mn && (uint16_t)(mx - mn) >= min_range) {
        uint16_t range = (uint16_t)(mx - mn);
        uint32_t num;
        uint16_t n;

        num = (c->avg <= mn) ? 0UL : (uint32_t)(c->avg - mn) * 100UL;
        n = (uint16_t)(num / range);
        c->norm100 = (n > 100) ? 100 : (uint8_t)n;
    } else {
        /* 캘리브레이션 미완료 -> "선 없음" 에 해당하는 안전한 기본값 */
        c->norm100 = (g_cal_set == CAL_WHITE) ? 100 : 0;
    }
}

/* ==================== LED ==================== */
static uint8_t g_led_state = 0;

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
    g_led_state = 0;
#if LED_ACTIVE_LOW
    LED_PORT |= LED_MASK;
#else
    LED_PORT &= (uint8_t)~LED_MASK;
#endif
}

/* ir_sig 기준으로 판정한다. 세트가 바뀌면 임계값도 따라 바뀐다. */
static void led_update(void)
{
    uint8_t i, out;
    uint8_t on_th  = (g_cal_set == CAL_BLACK) ? LED_SIG_ON_BLACK  : LED_SIG_ON;
    uint8_t off_th = (g_cal_set == CAL_BLACK) ? LED_SIG_OFF_BLACK : LED_SIG_OFF;

    for (i = 0; i < IR_NUM; i++) {
        uint8_t sig = ir_sig(i);
        if (sig >= on_th)       g_led_state |=  (uint8_t)(1 << i);
        else if (sig <= off_th) g_led_state &= (uint8_t)~(1 << i);
    }
    out = (uint8_t)(g_led_state & LED_MASK);

#if LED_ACTIVE_LOW
    LED_PORT = (uint8_t)((LED_PORT & ~LED_MASK) | (uint8_t)(~out & LED_MASK));
#else
    LED_PORT = (uint8_t)((LED_PORT & ~LED_MASK) | out);
#endif
}

/* 켜진 비트가 몇 덩어리인지 센다. 001101 -> 2 */
static uint8_t led_group_count(uint8_t mask)
{
    uint8_t i, groups = 0, prev = 0;

    for (i = 0; i < IR_NUM; i++) {
        uint8_t bit = (uint8_t)((mask >> i) & 0x01);
        if (bit && !prev) groups++;
        prev = bit;
    }
    return groups;
}

/* 켜진 LED 개수 */
static uint8_t led_on_count(uint8_t mask)
{
    uint8_t i, cnt = 0;
    for (i = 0; i < IR_NUM; i++)
        if (mask & (uint8_t)(1 << i)) cnt++;
    return cnt;
}

/* ==================== 모터 ==================== */
#define KICKSTART_TH    220     /* 목표 속도가 이보다 낮을 때만 킥스타트 */
#define KICKSTART_SPEED 255
#define KICKSTART_MS    60
#define KICK_HOLD_TH_MS 25      /* 이만큼 멈춰 있었어야 "진짜 정지" */

typedef enum { MOTOR_FWD, MOTOR_BWD, MOTOR_STOP } motor_dir_t;

static uint8_t  g_kick_active = 0;
static uint16_t g_kick_timer_ms = 0;

static void kick_tick(void)
{
    if (!g_kick_active) return;
    g_kick_timer_ms = (uint16_t)(g_kick_timer_ms + SAMPLE_MS);
    if (g_kick_timer_ms >= KICKSTART_MS) g_kick_active = 0;
}

typedef struct {
    uint16_t zero_ms;
    uint8_t  kicking;
    uint16_t kick_ms;
} kick_state_t;

static kick_state_t g_kick_left  = { 0, 0, 0 };
static kick_state_t g_kick_right = { 0, 0, 0 };

/* 정지 마찰 보완 : 멈춰 있다 재출발할 때 잠깐 최대 출력을 준다 */
static uint8_t kick_apply(kick_state_t *k, uint8_t target)
{
    uint8_t genuine_restart;

    if (target == 0) {
        k->zero_ms = (uint16_t)(k->zero_ms + SAMPLE_MS);
        k->kicking = 0;
        k->kick_ms = 0;
        return 0;
    }

    genuine_restart = (uint8_t)(k->zero_ms >= KICK_HOLD_TH_MS);
    k->zero_ms = 0;

    if ((genuine_restart || g_kick_active) && target < KICKSTART_TH) {
        k->kicking = 1;
        k->kick_ms = 0;
    }

    if (k->kicking) {
        k->kick_ms = (uint16_t)(k->kick_ms + SAMPLE_MS);
        if (k->kick_ms >= KICKSTART_MS) k->kicking = 0;
    }

    return k->kicking ? KICKSTART_SPEED : target;
}

static void motor_init(void)
{
    MOTOR_DDR |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4)
               | (1 << ENA) | (1 << ENB);

    /* Timer1 Fast PWM 8bit, 분주 8 -> 약 7.8kHz */
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11);
    OCR1A = 0;
    OCR1B = 0;
}

static void motor_left(motor_dir_t dir, uint8_t speed)
{
    switch (dir) {
    case MOTOR_FWD:  MOTOR_PORT &= (uint8_t)~(1 << IN1); MOTOR_PORT |= (1 << IN2); break;
    case MOTOR_BWD:  MOTOR_PORT |= (1 << IN1); MOTOR_PORT &= (uint8_t)~(1 << IN2); break;
    default:         MOTOR_PORT &= (uint8_t)~((1 << IN1) | (1 << IN2)); break;
    }
    OCR1A = kick_apply(&g_kick_left, speed);
}

static void motor_right(motor_dir_t dir, uint8_t speed)
{
    switch (dir) {
    case MOTOR_FWD:  MOTOR_PORT &= (uint8_t)~(1 << IN3); MOTOR_PORT |= (1 << IN4); break;
    case MOTOR_BWD:  MOTOR_PORT |= (1 << IN3); MOTOR_PORT &= (uint8_t)~(1 << IN4); break;
    default:         MOTOR_PORT &= (uint8_t)~((1 << IN3) | (1 << IN4)); break;
    }
    OCR1B = kick_apply(&g_kick_right, speed);
}

static void motor_stop_all(void)
{
    motor_left(MOTOR_STOP, 0);
    motor_right(MOTOR_STOP, 0);
}

static uint8_t clamp_speed(int16_t v)
{
    if (v < MIN_SPEED) return MIN_SPEED;
    if (v > MAX_SPEED) return MAX_SPEED;
    return (uint8_t)v;
}

/* ==================== PSD (차단바 / 벽) ====================
 * PF1(앞쪽)으로 차단바까지의 거리를 잰다. PF0(오른쪽)은 오른쪽 벽 거리.
 * ADC 가 클수록 가깝다. */
#define PSD_CH_FRONT        1       /* PF1 : 앞쪽 벽 */
#define PSD_CH_RIGHT        0       /* PF0 : 오른쪽 벽 */
#define PSD_PERIOD_MS       50      /* 측정 주기 */

#define BAR_STOP_ADC        600     /* 이 값 이상이면 정지 (가까움) */
#define BAR_OPEN_ADC        200     /* 이 값 이하면 차단바 열림 (멂) */
#define BAR_OPEN_HOLD_MS    100     /* 이 시간 연속 열림이어야 인정 */

/* 차단바 상태 */
#define BAR_ST_OFF      0   /* 비활성 */
#define BAR_ST_WATCH    1   /* 접근 중, 거리 감시 */
#define BAR_ST_WAIT     2   /* 정지하고 열리기를 기다림 */
#define BAR_ST_DONE     3   /* 통과 완료 */

static uint8_t  g_bar_st      = BAR_ST_OFF;
static uint16_t g_bar_ms      = 0;
static uint16_t g_bar_open_ms = 0;

static uint16_t g_psd_adc       = 0;   /* 앞쪽 PSD (PF1) */
static uint16_t g_psd_right_adc = 0;   /* 오른쪽 PSD (PF0) */

static void psd_measure(void)
{
    g_psd_adc = adc_read(PSD_CH_FRONT);
}

static void psd_measure_right(void)
{
    g_psd_right_adc = adc_read(PSD_CH_RIGHT);
}

/* ==================== 교차점 ==================== */
#define CROSS_TURN_MIN_MS           180 /* 이 시간 전에는 정렬 판정을 안 한다 */
#define CROSS_TURN_MAX_MS           1000
#define CROSS_TURN_OUTER            255 /* 바깥쪽 바퀴 */
#define CROSS_TURN_INNER            0   /* 0=피벗, >0=역회전(100 이상 필요) */
#define CROSS_TURN_CENTER_TH        35
#define CROSS_TURN_OUTER_TH         25
#define CROSS_TURN_OUTER_TOLERANCE  1

/* 8자 접점 (SEC_FIG8 카운트 2). 방향 고정 + 전용 최소 회전 시간 */
#define FIG8_C2_COUNT           2
#define FIG8_C2_TURN_MIN_MS     400
#define FIG8_C2_TURN_DIR        0   /* 1=우, 0=좌 */

/* 8자 탈출 접점 (SEC_FIG8 카운트 3) */
#define FIG8_EXIT_COUNT         3
#define FIG8_EXIT_TURN_MIN_MS   100
#define FIG8_EXIT_TURN_DIR      0   /* 1=우, 0=좌 */

/* 이 카운트 직후 일정 시간 카운트를 잠근다. 0 이면 기능 끔. */
#define FIG8_LOCK_COUNT         2
#define FIG8_LOCK_MS            2000

/* 평사 진입 (SEC_FIG8 카운트 4) */
#define FIG8_LANE_COUNT         4

/* 접점 예고 감지 (바깥은 선, 가운데는 조용). SEC_FIG8 카운트 1 에서만 */
#define OUTER_GAP_ON_TH        65
#define OUTER_GAP_QUIET_TH     25
#define OUTER_GAP_ON_COUNT     3
#define OUTER_GAP_QUIET_COUNT  1

/* ==================== 두 선 감지 -> 직진 ==================== */
#define TWO_LINE_GROUP_MIN     2
#define TWO_LINE_FWD_MS        300
#define TWO_LINE_FWD_SPEED     BASE_SPEED

/* ==================== 차선(평행사변형) 구간 ==================== */
#define LANE_STOP_MS           300  /* 진입 직후 정지 시간 */
#define LANE_ENTER_FWD_MS      500  /* 진입 후 센서 무시하고 직진할 시간 */
#define LANE_FWD_SPEED         140
#define LANE_BACK_MS           200  /* 후진 시간 */
#define LANE_BACK_SPEED        140
#define LANE_TURN_MS           400  /* 튕김 회전 시간 */
#define LANE_TURN_OUTER        180
#define LANE_TURN_INNER        0    /* 0=피벗 */

#define LANE_EXIT_TURN_MS      1800 /* 탈출 회전 최대 시간 (안전장치) */
#define LANE_EXIT_TURN_MIN_MS  1200 /* 이 시간까지는 무조건 회전 */
#define LANE_EXIT_CENTER_MASK  0x0C /* 정렬 판정에 볼 센서. IR2, IR3 */
#define LANE_EXIT_TURN_DIR     0    /* 1=우, 0=좌 */

#define LANE_LEFT_MASK         0x01 /* IR0 */
#define LANE_RIGHT_MASK        0x38 /* IR3~IR5 */

/* 차선 구간 내부 상태 */
#define LANE_ST_OFF         0
#define LANE_ST_STOP        1   /* 진입 직후 정지 */
#define LANE_ST_FWD         2   /* 직진 */
#define LANE_ST_BACK        3   /* 후진 */
#define LANE_ST_TURN        4   /* 튕김 회전 */
#define LANE_ST_ENTER       5   /* 진입 직후 센서 무시 직진 */
#define LANE_ST_EXIT_TURN   6   /* 탈출 : 정렬 회전 */

/* ==================== ㄱ자 / T자 회전 ====================
 * 왼쪽 끝 3개(IR0~IR2)가 선을 잡으면 좌회전 시작. */
#define CORNER_TRIG_MASK    0x07    /* IR0, IR1, IR2 */
#define CORNER_TRIG_COUNT   3       /* 이 개수 이상이면 트리거 */
#define CORNER_TURN_DIR     0       /* 1=우, 0=좌 */
#define CORNER_TURN_OUTER   220
#define CORNER_TURN_MIN_MS  200     /* 이 시간까지는 무조건 회전 */
#define CORNER_TURN_MAX_MS  500     /* 안전장치 */
#define CORNER_CENTER_TH    35      /* 정렬 판정 : 가운데 */
#define CORNER_OUTER_TH     25      /* 정렬 판정 : 바깥 */
#define CORNER_OUTER_TOL    1
#define CORNER_LOCK_MS      2000    /* 회전 종료 후 카운트 잠금 */

/* ==================== 반전 구간 ㄱ자 (SEC_BLACK) ====================
 * 검은 구간 카운트 4 에서 나오는 ㄱ자였으나, 현재는 SEC_BLACK 카운트 2~9
 * 회전 그룹으로 대체되어 사실상 비활성화 상태다 (ARM_COUNT = 99). */
#define BCORNER_ARM_COUNT      99    /* 이 카운트 이상일 때만 감지 */
#define BCORNER_TRIG_COUNT     3    /* 트리거 : 이 개수 이상 켜지면 */
#define BCORNER_TURN_OUTER     220
#define BCORNER_TURN_MIN_MS    200  /* 이 시간까지는 무조건 회전 */
#define BCORNER_TURN_MAX_MS    500  /* 안전장치 */
#define BCORNER_OUTER_TOL      1

#define BCORNER_ST_OFF   0
#define BCORNER_ST_TURN  1
#define BCORNER_ST_DONE  2

/* ==================== 반전 구간 교차점 (SEC_BLACK 카운트 2~9,12) ====================
 * 카운트 1 직후 일정 시간 카운트를 무시한다.
 * 회전 카운트에서는 좌우 LED 개수를 비교해 방향을 정하고,
 * 곧바로 회전하면 중심이 안 맞아 오정렬이 나므로 짧게 직진한 뒤 제자리 회전한다. */
#define BLACK_LOCK_COUNT     1
#define BLACK_LOCK_MS        1000

#define BLACK_TURN_MIN_MS    500    /* 카운트 2,3,4,5,7,8,12 전용 최소 회전 시간 */
#define BLACK_TURN_MAX_MS    800   /* 카운트 2,3,4,5,7,8,12 전용 최대 회전 시간 (안전장치) */

#define BLACK_C9_TURN_MIN_MS 800    /* 카운트 9 전용 최소 회전 시간 (더 많이 돌아야 함) */
#define BLACK_C9_TURN_MAX_MS 1000   /* 카운트 9 전용 최대 회전 시간 (안전장치) */

#define TURN_PREP_FWD_MS     900    /* 교차점 인식 후 회전 전 직진 시간 */
#define TURN_PREP_FWD_SPEED  BASE_SPEED

#define BLACK_SPIN_SPEED     200
#define BLACK_C3_LOCK_MS     500    /* 카운트 3 회전 종료 후 카운트 잠금 시간 */
#define BLACK_C9_LOCK_MS     900    /* 카운트 9 회전 종료 후 카운트 잠금 시간 (회전량이 더 커서 더 길게) */
#define BLACK_C6_LOCK_MS     500    /* 카운트 6, 10 인식 후 카운트 잠금 시간 */
#define BLACK_C11_LOCK_MS	 2000   /* 카운트 11 인식 후 카운트 잠금 시간 */

#define BLACK13_PAUSE_MS     500    /* 카운트 13 : 정지 시간 */
#define BLACK13_FWD_SPEED    BASE_SPEED
#define BLACK13_PSD_STOP_ADC 700    /* 이 값 이상이면 정지 */

#define BLACK13_ST_OFF    0
#define BLACK13_ST_PAUSE  1
#define BLACK13_ST_FWD    2
#define BLACK13_ST_DONE   3

/* ==================== 반전 구간 : 벽 탐지 구간 (카운트 13 정지 이후) ====================
 * 전진하다 앞쪽 PSD 로 벽을 인식하면 제자리 좌회전 90도.
 * 그 뒤 다시 전진하다 교차로(cross_line_detect)를 인식하면, 그 교차로를
 * 곧바로 "새 시퀀스 카운트 1" 로 세우고 종료 -> 일반 라인 추종 복귀.
 * 이후 그 새 시퀀스의 카운트 2 에서는 무조건 우회전한다
 * (cross_count_check() 의 SEC_BLACK case 맨 위 참고). */
#define WALL_FWD_SPEED        BASE_SPEED
#define WALL_PSD_STOP_ADC     700    /* 이 값 이상이면 벽으로 판정 */

#define WALL_TURN_SPEED       200    /* 제자리 회전 속도 */
#define WALL_TURN_MIN_MS      500    /* 90도 고정 회전 시간 (실측 조정) */
#define WALL_TURN_DIR         0      /* 0=좌, 1=우 -- 요청은 좌회전 */

#define WALL_ST_OFF        0
#define WALL_ST_FWD1       1   /* 벽까지 전진 */
#define WALL_ST_TURN       2   /* 90도 좌회전 */
#define WALL_ST_FWD2       3   /* 회전 후 교차로까지 전진 */
#define WALL_ST_DONE       4   /* 완료. 라인 추종으로 복귀. 이후 새 시퀀스 시작 */

/* T자(주차 진입) 회전. 트리거 조건은 ㄱ자와 동일하다. */
#define TCORNER_TURN_DIR      0     /* 1=우, 0=좌 */
#define TCORNER_TURN_OUTER    220
#define TCORNER_TURN_MIN_MS   200   /* 실측 조정 */
#define TCORNER_TURN_MAX_MS   850   /* 실측 조정 */
#define TCORNER_LOCK_MS       0     /* 회전 종료 후 카운트 잠금 */
#define TCORNER_ARM_DELAY_MS  1500  /* 차단바 통과 후 이 시간 뒤 T자 감지 시작 */

/* ㄱ자 / T자 공용 상태 */
#define CORNER_ST_OFF     0   /* ㄱ자 대기 */
#define CORNER_ST_TURN    1   /* ㄱ자 회전 중 */
#define CORNER_ST_DONE    2   /* ㄱ자 완료. T자 대기는 차단바 통과 후 */
#define CORNER_ST_T_WAIT  3   /* T자 대기 */
#define CORNER_ST_T_TURN  4   /* T자 회전 중 */
#define CORNER_ST_T_DONE  5   /* 전부 완료 */

/* ==================== 주차 구간 (SEC_PARK) ==================== */
#define PARK_ENTER_COUNT    1       /* T자 통과 완료 시점의 카운트 */

#define PARK_FWD_COUNT      2       /* 이 카운트에서 정지 후 허공 직진 */
#define PARK_PAUSE_MS       200     /* 허공 직진 전 정지 시간 (관성 제거) */
#define PARK_FWD_SPEED      140
#define PARK_FWD_TIMEOUT_MS 1000    /* 선을 못 찾으면 이 시간 뒤 강제 정지 */
#define PARK_SEEK_DELAY_MS  300     /* 직진 시작 후 이 시간까지는 정지 판정 안 함 */
#define PARK_STOP_MS        1000    /* 정지 유지 시간 */

/* 주차 탈출 : 제자리 180도 회전 */
#define PARK_OUT_TURN_DIR      0     /* 1=우, 0=좌 */
#define PARK_OUT_TURN_SPEED    150   /* 제자리 회전 속도 (양쪽 동일) */
#define PARK_OUT_TURN_MIN_MS   800  /* 이 시간까지는 무조건 회전 */
#define PARK_OUT_TURN_MAX_MS   1900  /* 안전장치 */
#define PARK_OUT_CENTER_TH     30    /* 정렬 판정 : 가운데 */
#define PARK_OUT_OUTER_TH      25    /* 정렬 판정 : 바깥 */
#define PARK_OUT_OUTER_TOL     1

/* 주차 구간 전용 선 판정 */
#define PARK_DETECT_ON_COUNT   5    /* 이 개수 이상 켜지면 선으로 본다 */

/* 주차 탈출 : 이 카운트에서 좌회전 */
#define PARK_EXIT_COUNT        4
#define PARK_EXIT_TURN_MIN_MS  400  /* 실측 조정 */
#define PARK_EXIT_TURN_DIR     0    /* 1=우, 0=좌 */

/* 주차 구간 내부 상태 */
#define PARK_ST_OFF       0
#define PARK_ST_PAUSE     1  /* 직진 전 잠깐 정지 */
#define PARK_ST_FWD       2  /* 허공 직진 */
#define PARK_ST_STOP      3  /* 정지 유지 */
#define PARK_ST_OUT_TURN  4  /* 탈출 : 180도 제자리 회전 */
#define PARK_ST_DONE      5  /* 회전 완료. 라인 추종 복귀 */

/* ==================== 반전 플래그 ====================
 * 가운데 센서가 선을 잡으면(교차점 판정과 동일) 반전 구간 진입.
 * SEC_PARK 에서 주차가 끝난 뒤에만 감지한다. */
#define INV_FWD_MS       400    /* 전환 후 센서 무시 직진 시간 */
#define INV_FWD_SPEED    140

#define INV_ST_WAIT   0   /* 플래그 대기 */
#define INV_ST_FWD    1   /* 전환 후 직진 */
#define INV_ST_DONE   2   /* 완료. 다시 발동하지 않는다 */

#define C4_LOCK_FOLLOW_MS   700    /* 카운트4 회전 후 : 라인 추종 유지 시간 */
#define C4_LOCK_BLIND_MS    1000   /* 그 다음 : 라인 무시 무조건 직진 시간 */
#define C4_LOCK_FWD_SPEED   BASE_SPEED

#define C4LOCK_ST_OFF      0
#define C4LOCK_ST_FOLLOW   1   /* 0.5초 라인 추종 */
#define C4LOCK_ST_BLIND    2   /* 1초 라인 무시 직진 */



#define C5_BLIND_MS         1000   /* 카운트5 : 라인 무시 무조건 직진 시간 */
#define C5_BLIND_SPEED      BASE_SPEED

#define C6_BLIND_MS         300    /* 카운트6 : 정지 전 무조건 직진 시간 */
#define C6_BLIND_SPEED      BASE_SPEED

#define C12_BACK_MS         250    /* 카운트12 회전 후 후진 시간 */
#define C12_BACK_SPEED      250



/* ==================== 전역 상태 ==================== */
static uint8_t  g_two_line_active = 0;
static uint16_t g_two_line_ms     = 0;
static uint8_t  g_two_line_prev   = 0;

static uint8_t  g_cross_turn_active = 0;
static uint8_t  g_cross_turn_dir    = 1;   /* 1=우, 0=좌 */
static uint16_t g_cross_turn_ms     = 0;

static uint8_t  g_cross_line_prev = 0;
static uint8_t  g_outer_gap_prev  = 0;
static uint8_t  g_approach_armed  = 1;
static uint16_t g_cross_lock_ms   = 0;     /* >0 이면 카운트 잠금 중 */

static uint8_t  g_lane_st       = LANE_ST_OFF;
static uint16_t g_lane_ms       = 0;
static uint8_t  g_lane_turn_dir = 1;

static int16_t  g_last_error_x10 = 0;      /* 재탐색 방향에 쓴다 */
static uint8_t  g_sharp_state    = 0;      /* 0=일반, 1=왼쪽, 2=오른쪽 */

static uint16_t g_search_ms       = 0;   /* 재탐색 회전 경과 시간 */
static uint8_t  g_search_reversed = 0;   /* 1이면 이미 반대 방향으로 전환됨 */
static uint8_t  g_center_stop = 0;   /* 1이면 중앙 정지 구간 : 영구 정지 */


static uint8_t  g_park_st = PARK_ST_OFF;
static uint16_t g_park_ms = 0;

static uint8_t  g_corner_st      = CORNER_ST_OFF;
static uint16_t g_corner_ms      = 0;
static uint16_t g_tcorner_arm_ms = 0;

static uint8_t  g_bcorner_st  = BCORNER_ST_OFF;
static uint16_t g_bcorner_ms  = 0;
static uint8_t  g_bcorner_dir = 0;   /* 1=우, 0=좌 (SEC_BLACK 카운트4 ㄱ자, 현재 비활성) */

/* SEC_BLACK 회전 카운트 : 교차점 인식 직후 직진 -> 제자리 회전 */
static uint8_t  g_turn_prep_active = 0;
static uint16_t g_turn_prep_ms     = 0;
static uint8_t  g_turn_prep_dir    = 0;   /* 1=우, 0=좌 */

/* SEC_BLACK 카운트 13 : 정지 -> PSD 전진 */
static uint8_t  g_black13_st  = BLACK13_ST_OFF;
static uint16_t g_black13_ms  = 0;
static uint16_t g_black13_psd_ms = 0;

/* SEC_BLACK 벽 탐지 구간 */
static uint8_t  g_wall_st     = WALL_ST_OFF;
static uint16_t g_wall_ms     = 0;
static uint16_t g_wall_psd_ms = 0;

static uint8_t  g_inv_st = INV_ST_WAIT;
static uint16_t g_inv_ms = 0;

static uint8_t  g_c4lock_st = C4LOCK_ST_OFF;
static uint16_t g_c4lock_ms = 0;

static uint8_t  g_c5_blind_active = 0;
static uint16_t g_c5_blind_ms     = 0;

static uint8_t  g_c6_blind_active = 0;
static uint16_t g_c6_blind_ms     = 0;

static uint8_t  g_c12_back_active = 0;
static uint16_t g_c12_back_ms     = 0;

static void print_lcd_count(void);

/* 구간 전환. 카운트와 엣지 상태를 함께 초기화한다. */
static void section_enter(uint8_t sec)
{
    g_section         = sec;
    g_sec_count       = 0;
    g_cross_line_prev = 1;      /* 전환 직후 중복 카운트 방지 */
    g_cross_lock_ms   = 0;
    print_lcd_count();
}

/* ==================== 감지 ==================== */
/* 가운데 4개(IR1~IR4) 중 3개 이상이면 교차점 */
static uint8_t cross_line_detect(void)
{
    uint8_t i, hit = 0;

    for (i = 1; i <= 4; i++)
        if (g_led_state & (uint8_t)(1 << i)) hit++;

    return (hit >= 3);
}

/* 주차 구간 전용 : 5개 이상 켜지면 선으로 본다 */
static uint8_t park_line_detect(void)
{
    return (led_on_count((uint8_t)(g_led_state & LED_MASK))
            >= PARK_DETECT_ON_COUNT);
}

/* 바깥은 선을 보고 가운데는 조용한 패턴 (개수 기준) */
static uint8_t cross_outer_gap_detect(void)
{
    uint8_t outer_on = 0, center_quiet = 0;

    if (ir_sig(0) >= OUTER_GAP_ON_TH) outer_on++;
    if (ir_sig(1) >= OUTER_GAP_ON_TH) outer_on++;
    if (ir_sig(4) >= OUTER_GAP_ON_TH) outer_on++;
    if (ir_sig(5) >= OUTER_GAP_ON_TH) outer_on++;

    if (ir_sig(2) < OUTER_GAP_QUIET_TH) center_quiet++;
    if (ir_sig(3) < OUTER_GAP_QUIET_TH) center_quiet++;

    return (outer_on >= OUTER_GAP_ON_COUNT &&
            center_quiet >= OUTER_GAP_QUIET_COUNT);
}

/* ==================== 두 선 직진 ==================== */
static void two_line_check(void)
{
    uint8_t two = (uint8_t)(led_group_count((uint8_t)(g_led_state & LED_MASK))
                            >= TWO_LINE_GROUP_MIN);

    if (two && !g_two_line_prev && !g_two_line_active && !g_cross_turn_active) {
        g_two_line_active = 1;
        g_two_line_ms     = 0;
    }
    g_two_line_prev = two;
}

/* 직진 유지 중이면 1 반환 */
static uint8_t two_line_update(void)
{
    if (!g_two_line_active) return 0;

    g_two_line_ms = (uint16_t)(g_two_line_ms + SAMPLE_MS);
    if (g_two_line_ms >= TWO_LINE_FWD_MS) {
        g_two_line_active = 0;
        g_two_line_ms     = 0;
        g_sharp_state     = 0;
        return 0;
    }

    motor_left(MOTOR_FWD,  TWO_LINE_FWD_SPEED);
    motor_right(MOTOR_FWD, TWO_LINE_FWD_SPEED);
    return 1;
}

/* ==================== 반전 플래그 ====================
 * 처리했으면 1 반환. */
static uint8_t invert_update(void)
{
#if !ENABLE_INVERT
    return 0;
#else
    if (g_inv_st == INV_ST_DONE) return 0;

    if (g_inv_st == INV_ST_WAIT) {
        /* 주차 구간에서 180도 회전까지 끝난 뒤에만 감지한다 */
        if (g_section != SEC_PARK)      return 0;
        if (g_park_st != PARK_ST_DONE)  return 0;
        if (g_sec_count < PARK_EXIT_COUNT) return 0;
        if (g_cross_turn_active)           return 0;
        if (!cross_line_detect())       return 0;

        /* 반전 구간 진입 : 세트를 바꾸고 잠시 직진 */
        g_cal_set = CAL_BLACK;
        g_inv_st  = INV_ST_FWD;
        g_inv_ms  = 0;
        led_all_off();          /* 판정 잔상 제거 */
    }

    /* INV_ST_FWD */
    g_inv_ms = (uint16_t)(g_inv_ms + SAMPLE_MS);

    motor_left(MOTOR_FWD,  INV_FWD_SPEED);
    motor_right(MOTOR_FWD, INV_FWD_SPEED);

    if (g_inv_ms >= INV_FWD_MS) {
        g_inv_st            = INV_ST_DONE;
        g_inv_ms            = 0;
        g_last_error_x10    = 0;
        g_cross_turn_active = 0;
        g_cross_turn_ms     = 0;

        g_park_st = PARK_ST_OFF;    /* 주차 상태를 완전히 내린다 */
        g_park_ms = 0;

        section_enter(SEC_BLACK);
        return 0;
    }
    return 1;
#endif
}

/* ==================== 차선 구간 ====================
 * 처리했으면 1 반환. */
static uint8_t lane_follow_update(void)
{
    uint8_t mask, left_hit, right_hit;

    if (g_section != SEC_LANE) return 0;

    mask      = (uint8_t)(g_led_state & LED_MASK);
    left_hit  = (uint8_t)(mask & LANE_LEFT_MASK);
    right_hit = (uint8_t)(mask & LANE_RIGHT_MASK);

    g_lane_ms = (uint16_t)(g_lane_ms + SAMPLE_MS);

    switch (g_lane_st) {
    case LANE_ST_STOP:
        motor_stop_all();
        if (g_lane_ms >= LANE_STOP_MS) {
            g_lane_st = LANE_ST_ENTER;
            g_lane_ms = 0;
        }
        break;

    case LANE_ST_ENTER:
        /* 진입 지점의 가로선을 벗어날 때까지 센서를 무시하고 직진 */
        motor_left(MOTOR_FWD,  LANE_FWD_SPEED);
        motor_right(MOTOR_FWD, LANE_FWD_SPEED);
        if (g_lane_ms >= LANE_ENTER_FWD_MS) {
            g_lane_st = LANE_ST_FWD;
            g_lane_ms = 0;
        }
        break;

    case LANE_ST_FWD:
        if (right_hit) {
            /* 오른쪽 변 도달 -> 탈출 */
            g_lane_st = LANE_ST_EXIT_TURN;
            g_lane_ms = 0;
        } else if (left_hit) {
            /* 왼쪽 변 또는 가로선 -> 튕김 */
            g_lane_turn_dir = 1;
            g_lane_st = LANE_ST_BACK;
            g_lane_ms = 0;
        } else {
            motor_left(MOTOR_FWD,  LANE_FWD_SPEED);
            motor_right(MOTOR_FWD, LANE_FWD_SPEED);
        }
        break;

    case LANE_ST_BACK:
        motor_left(MOTOR_BWD,  LANE_BACK_SPEED);
        motor_right(MOTOR_BWD, LANE_BACK_SPEED);
        if (g_lane_ms >= LANE_BACK_MS) {
            g_lane_st = LANE_ST_TURN;
            g_lane_ms = 0;
        }
        break;

    case LANE_ST_TURN:
        if (g_lane_turn_dir) {
            motor_left(MOTOR_FWD, LANE_TURN_OUTER);
            if (LANE_TURN_INNER > 0) motor_right(MOTOR_BWD, LANE_TURN_INNER);
            else                     motor_right(MOTOR_STOP, 0);
        } else {
            motor_right(MOTOR_FWD, LANE_TURN_OUTER);
            if (LANE_TURN_INNER > 0) motor_left(MOTOR_BWD, LANE_TURN_INNER);
            else                     motor_left(MOTOR_STOP, 0);
        }
        if (g_lane_ms >= LANE_TURN_MS) {
            g_lane_st = LANE_ST_FWD;
            g_lane_ms = 0;
        }
        break;

    case LANE_ST_EXIT_TURN:
        if (LANE_EXIT_TURN_DIR) {
            motor_left(MOTOR_FWD, LANE_TURN_OUTER);
            motor_right(MOTOR_STOP, 0);
        } else {
            motor_right(MOTOR_FWD, LANE_TURN_OUTER);
            motor_left(MOTOR_STOP, 0);
        }

        /* MIN_MS 까지는 무조건 회전. 그 뒤 가운데 센서가 선을 잡으면 종료. */
        if ((g_lane_ms >= LANE_EXIT_TURN_MIN_MS &&
             (mask & LANE_EXIT_CENTER_MASK)) ||
            g_lane_ms >= LANE_EXIT_TURN_MS) {
            g_lane_st = LANE_ST_OFF;
            g_lane_ms = 0;
            g_cross_turn_active = 0;

            section_enter(SEC_CORNER);  /* 평사 종료 -> ㄱ자 구간 */
            g_cross_line_prev = 0;
            return 0;                   /* 일반 라인 추종으로 복귀 */
        }
        break;

    default:
        motor_stop_all();
        break;
    }

    return 1;
}

/* ==================== ㄱ자 / T자 회전 ====================
 * 처리했으면 1 반환.
 * 트리거는 둘 다 왼쪽 끝 3개(IR0~IR2). T자는 차단바 통과 후 활성화. */
static uint8_t corner_update(void)
{
    uint8_t  mask, trig_cnt, aligned = 0;
    uint8_t  is_t, turn_dir, turn_outer;
    uint16_t min_ms, max_ms;
    uint8_t  center_on, outer_busy;

    /* SEC_CORNER 안에서만 동작한다 */
    if (g_section != SEC_CORNER) return 0;
    if (g_corner_st == CORNER_ST_T_DONE) return 0;

    mask     = (uint8_t)(g_led_state & LED_MASK);
    trig_cnt = led_on_count((uint8_t)(mask & CORNER_TRIG_MASK));

    /* ㄱ자 대기 */
    if (g_corner_st == CORNER_ST_OFF) {
        if (trig_cnt < CORNER_TRIG_COUNT) return 0;
        g_corner_st = CORNER_ST_TURN;
        g_corner_ms = 0;
    }

    /* ㄱ자 완료 상태 : 차단바 통과 후 일정 시간 지나면 T자 대기로.
     * 지연이 없으면 차단바 앞 정지 위치에서 바로 트리거될 수 있다. */
    if (g_corner_st == CORNER_ST_DONE) {
        if (g_bar_st == BAR_ST_DONE) {
            g_tcorner_arm_ms = (uint16_t)(g_tcorner_arm_ms + SAMPLE_MS);
            if (g_tcorner_arm_ms >= TCORNER_ARM_DELAY_MS) {
                g_corner_st = CORNER_ST_T_WAIT;
            }
        }
        return 0;
    }

    /* T자 대기 */
    if (g_corner_st == CORNER_ST_T_WAIT) {
        if (trig_cnt < CORNER_TRIG_COUNT) return 0;
        g_corner_st = CORNER_ST_T_TURN;
        g_corner_ms = 0;
    }

    /* 회전 중 : ㄱ자인지 T자인지에 따라 파라미터를 고른다 */
    is_t = (uint8_t)(g_corner_st == CORNER_ST_T_TURN);

    if (is_t) {
        turn_dir   = TCORNER_TURN_DIR;
        turn_outer = TCORNER_TURN_OUTER;
        min_ms     = TCORNER_TURN_MIN_MS;
        max_ms     = TCORNER_TURN_MAX_MS;
    } else {
        turn_dir   = CORNER_TURN_DIR;
        turn_outer = CORNER_TURN_OUTER;
        min_ms     = CORNER_TURN_MIN_MS;
        max_ms     = CORNER_TURN_MAX_MS;
    }

    g_corner_ms = (uint16_t)(g_corner_ms + SAMPLE_MS);

    center_on  = 0;
    outer_busy = 0;

    if (g_corner_ms >= min_ms) {
        if (g_cal_set == CAL_BLACK) {
            center_on  = (g_led_state & 0x0C) == 0x0C;
            outer_busy = led_on_count((uint8_t)(g_led_state & 0x33));
        } else {
            center_on = (ir_sig(2) >= CORNER_CENTER_TH &&
                        ir_sig(3) >= CORNER_CENTER_TH);
            outer_busy = 0;
            if (ir_sig(0) >= CORNER_OUTER_TH) outer_busy++;
            if (ir_sig(1) >= CORNER_OUTER_TH) outer_busy++;
            if (ir_sig(4) >= CORNER_OUTER_TH) outer_busy++;
            if (ir_sig(5) >= CORNER_OUTER_TH) outer_busy++;
        }

        if (center_on && outer_busy <= CORNER_OUTER_TOL) aligned = 1;
    }

    if (aligned || g_corner_ms >= max_ms) {
        g_corner_ms = 0;

        g_kick_left.zero_ms  = 0;
        g_kick_right.zero_ms = 0;
        g_last_error_x10     = (turn_dir) ? 640 : -640;
        g_cross_turn_active  = 0;
        g_cross_turn_ms      = 0;

        if (is_t) {
            /* T자 완료 : 주차 구간으로 전환하고 카운트를 1 로 맞춘다 */
            g_corner_st = CORNER_ST_T_DONE;
            section_enter(SEC_PARK);
            g_sec_count = PARK_ENTER_COUNT;
            print_lcd_count();
            g_cross_lock_ms = TCORNER_LOCK_MS;
        } else {
            /* ㄱ자 완료 : 카운트를 올리고 차단바 감시 시작 */
            g_corner_st = CORNER_ST_DONE;
            g_sec_count++;
            print_lcd_count();
            g_cross_line_prev = 1;
            g_cross_lock_ms   = CORNER_LOCK_MS;

            if (g_bar_st == BAR_ST_OFF) {
                g_bar_st = BAR_ST_WATCH;
                g_bar_ms = 0;
            }
        }
        return 0;
    }

    /* 피벗 회전 : 바깥 바퀴만 돌린다 */
    if (turn_dir) {
        motor_left(MOTOR_FWD, turn_outer);
        motor_right(MOTOR_STOP, 0);
    } else {
        motor_right(MOTOR_FWD, turn_outer);
        motor_left(MOTOR_STOP, 0);
    }
    return 1;
}

/* ==================== 반전 구간 ㄱ자 (SEC_BLACK 카운트 4, 현재 비활성) ====================
 * BCORNER_ARM_COUNT = 99 이라 사실상 발동하지 않는다.
 * 처리했으면 1 반환. corner_update() 와 같은 방식(정렬될 때까지 피벗)이지만
 * 방향은 고정이 아니라 좌우 ir_sig 합이 큰 쪽으로 돈다. */
static uint8_t bcorner_update(void)
{
    uint8_t  mask, left_cnt, right_cnt, aligned = 0;
    uint8_t  center_on, outer_busy, i;
    uint16_t left_sum = 0, right_sum = 0;

    if (g_section != SEC_BLACK)       return 0;
    if (g_bcorner_st == BCORNER_ST_DONE) return 0;

    mask      = (uint8_t)(g_led_state & LED_MASK);
    left_cnt  = led_on_count((uint8_t)(mask & 0x07));  /* IR0~IR2 */
    right_cnt = led_on_count((uint8_t)(mask & 0x38));  /* IR3~IR5 */

    if (g_bcorner_st == BCORNER_ST_OFF) {
        if (g_sec_count < BCORNER_ARM_COUNT) return 0;
        if (left_cnt < BCORNER_TRIG_COUNT &&
            right_cnt < BCORNER_TRIG_COUNT) return 0;

        for (i = 0; i < 3; i++) left_sum  = (uint16_t)(left_sum  + ir_sig(i));
        for (i = 3; i < 6; i++) right_sum = (uint16_t)(right_sum + ir_sig(i));

        g_bcorner_dir = (right_sum > left_sum) ? 1 : 0;
        g_bcorner_st  = BCORNER_ST_TURN;
        g_bcorner_ms  = 0;
        g_cross_turn_active = 0;
    }

    g_bcorner_ms = (uint16_t)(g_bcorner_ms + SAMPLE_MS);

    center_on  = 0;
    outer_busy = 0;

    if (g_bcorner_ms >= BCORNER_TURN_MIN_MS) {
        center_on  = (g_led_state & 0x0C) == 0x0C;
        outer_busy = led_on_count((uint8_t)(g_led_state & 0x33));
        if (center_on && outer_busy <= BCORNER_OUTER_TOL) aligned = 1;
    }

    if (aligned || g_bcorner_ms >= BCORNER_TURN_MAX_MS) {
        g_bcorner_st = BCORNER_ST_DONE;
        g_bcorner_ms = 0;

        g_kick_left.zero_ms  = 0;
        g_kick_right.zero_ms = 0;
        g_last_error_x10 = (g_bcorner_dir) ? 640 : -640;

        g_cross_line_prev = 1;
        g_cross_lock_ms   = CORNER_LOCK_MS;
        return 0;
    }

    if (g_bcorner_dir) {
        motor_left(MOTOR_FWD, BCORNER_TURN_OUTER);
        motor_right(MOTOR_STOP, 0);
    } else {
        motor_right(MOTOR_FWD, BCORNER_TURN_OUTER);
        motor_left(MOTOR_STOP, 0);
    }
    return 1;
}

/* ==================== 반전 구간 교차점 회전 준비 (직진 -> 제자리 회전) ====================
 * 처리했으면 1 반환. 교차점 인식 직후 곧바로 회전하면 중심이 안 맞아
 * 오정렬이 나므로, 먼저 짧게 직진해 중심으로 들어간 뒤 일반 교차점 회전
 * (cross_turn_update) 로 넘긴다. */
static uint8_t turn_prep_update(void)
{
    if (!g_turn_prep_active) return 0;

    g_turn_prep_ms = (uint16_t)(g_turn_prep_ms + SAMPLE_MS);

    if (g_turn_prep_ms >= TURN_PREP_FWD_MS) {
        g_turn_prep_active = 0;
        g_turn_prep_ms     = 0;

        g_cross_turn_dir    = g_turn_prep_dir;
        g_cross_turn_active = 1;
        g_cross_turn_ms     = 0;
        return 1;   /* 이번 프레임은 그대로 넘기고, 다음 프레임부터 cross_turn_update 가 돈다 */
    }

    motor_left(MOTOR_FWD,  TURN_PREP_FWD_SPEED);
    motor_right(MOTOR_FWD, TURN_PREP_FWD_SPEED);
    return 1;
}

/* ==================== 반전 구간 카운트 13 : 정지 후 PSD 감시 전진 ====================
 * 처리했으면 1 반환. 500ms 정지한 뒤, PSD 를 보며 전진하다가
 * 값이 BLACK13_PSD_STOP_ADC 이상이 되면 정지하고, 그 즉시
 * 벽 탐지 구간(black_wall_update)으로 넘긴다. 이 전환 순간 g_sec_count 를
 * 0 으로 리셋해서, 벽 탐지 구간이 새 시퀀스를 다시 카운트 1 부터 셀 수 있게 한다. */
static uint8_t black13_update(void)
{
	if (g_section != SEC_BLACK) return 0;
	if (g_black13_st == BLACK13_ST_OFF) return 0;

	if (g_black13_st == BLACK13_ST_DONE) {
		return 0;   /* 벽 탐지 구간(black_wall_update)에게 넘긴다 */
	}

	g_black13_ms = (uint16_t)(g_black13_ms + SAMPLE_MS);

	switch (g_black13_st) {
    case BLACK13_ST_PAUSE:
        motor_stop_all();
        if (g_black13_ms >= BLACK13_PAUSE_MS) {
            g_black13_st      = BLACK13_ST_FWD;
            g_black13_ms      = 0;
            g_black13_psd_ms  = 0;
        }
        break;

    case BLACK13_ST_FWD:
        motor_left(MOTOR_FWD,  BLACK13_FWD_SPEED);
        motor_right(MOTOR_FWD, BLACK13_FWD_SPEED);

        g_black13_psd_ms = (uint16_t)(g_black13_psd_ms + SAMPLE_MS);
        if (g_black13_psd_ms >= PSD_PERIOD_MS) {
            g_black13_psd_ms = 0;
            psd_measure();
        }

        if (g_psd_adc >= BLACK13_PSD_STOP_ADC) {
	        motor_stop_all();
	        g_black13_st = BLACK13_ST_DONE;
	        g_wall_st    = WALL_ST_FWD1;
	        g_wall_ms    = 0;
	        g_wall_psd_ms = 0;

	        /* 벽 탐지 구간으로 넘어가는 시점에 카운트를 0으로 리셋.
	         * 실제 새 시퀀스의 카운트 1은 black_wall_update() 의
	         * WALL_ST_FWD2 에서 교차로를 처음 인식하는 순간 세워진다. */
	        g_sec_count       = 0;
	        print_lcd_count();
	        g_cross_line_prev = 0;
        }
        break;

    default:
        break;
    }

    return 1;
}

/* ==================== 반전 구간 : 벽 탐지 구간 ====================
 * 처리했으면 1 반환. 카운트 13 정지 직후 자동으로 시작된다.
 * 전진 -> 벽(앞 PSD) 인식 -> 좌회전 90도(고정 시간) -> 전진 -> 교차로 인식.
 * 교차로를 인식하는 순간 그 교차로를 새 시퀀스의 카운트 1로 세우고 종료한다. */
static uint8_t black_wall_update(void)
{
    if (g_section != SEC_BLACK) return 0;
    if (g_wall_st == WALL_ST_OFF || g_wall_st == WALL_ST_DONE) return 0;

    g_wall_ms = (uint16_t)(g_wall_ms + SAMPLE_MS);

    switch (g_wall_st) {
    case WALL_ST_FWD1:
        motor_left(MOTOR_FWD,  WALL_FWD_SPEED);
        motor_right(MOTOR_FWD, WALL_FWD_SPEED);

        g_wall_psd_ms = (uint16_t)(g_wall_psd_ms + SAMPLE_MS);
        if (g_wall_psd_ms >= PSD_PERIOD_MS) {
            g_wall_psd_ms = 0;
            psd_measure();
        }

        if (g_psd_adc >= WALL_PSD_STOP_ADC) {
            motor_stop_all();
            g_wall_st = WALL_ST_TURN;
            g_wall_ms = 0;
        }
        break;

    case WALL_ST_TURN:
        /* 고정 시간(WALL_TURN_MIN_MS)만큼 제자리 회전하고, 그 시간에
         * 도달하는 즉시 종료한다. */
        if (g_wall_ms < WALL_TURN_MIN_MS) {
            if (WALL_TURN_DIR) {
                motor_left(MOTOR_FWD,  WALL_TURN_SPEED);
                motor_right(MOTOR_BWD, WALL_TURN_SPEED);
            } else {
                motor_left(MOTOR_BWD,  WALL_TURN_SPEED);
                motor_right(MOTOR_FWD, WALL_TURN_SPEED);
            }
        } else {
            motor_stop_all();
            g_wall_st = WALL_ST_FWD2;
            g_wall_ms = 0;
        }
        break;

        case WALL_ST_FWD2:
        motor_left(MOTOR_FWD,  WALL_FWD_SPEED);
        motor_right(MOTOR_FWD, WALL_FWD_SPEED);

        /* 교차점 모양(가운데 4개 중 3개)을 기다리지 않고,
         * IR 중 하나라도 선을 인식하는 순간 곧바로 카운트 1을 세운다. */
        if ((g_led_state & LED_MASK) != 0) {
            motor_stop_all();
            g_wall_st = WALL_ST_DONE;

            /* 카운트 1이 된 직후 500ms 동안은 다음 교차점을 감지하지 않는다. */
            g_sec_count        = 1;
            print_lcd_count();
            g_cross_line_prev  = 1;
            g_cross_lock_ms    = 500;
        }
        break;

    default:
        break;
    }

    return 1;
}

/* ==================== 중앙 정지 구간 ====================
 * 처리했으면 1 반환. 새 시퀀스 카운트 6 이후 영구 정지한다.
 * 리셋(SW4 재시작) 전까지는 절대 풀리지 않는다. */
static uint8_t center_stop_update(void)
{
    if (!g_center_stop) return 0;
    motor_stop_all();
    return 1;
}
/* ==================== 새 시퀀스 카운트4 이후 : 라인추종 -> 무조건 직진 ====================
 * 처리했으면 1 반환.
 * FOLLOW(0.5초) 동안은 이 함수가 아무것도 안 하고 0을 반환해서
 * 일반 라인 추종이 그대로 동작하게 둔다.
 * BLIND(1초) 동안은 센서 무시하고 그냥 직진한다. */
static uint8_t c4lock_update(void)
{
    if (g_c4lock_st == C4LOCK_ST_OFF) return 0;

    g_c4lock_ms = (uint16_t)(g_c4lock_ms + SAMPLE_MS);

    if (g_c4lock_st == C4LOCK_ST_FOLLOW) {
        if (g_c4lock_ms >= C4_LOCK_FOLLOW_MS) {
            g_c4lock_st = C4LOCK_ST_BLIND;
            g_c4lock_ms = 0;
        }
        return 0;   /* 라인 추종에 맡긴다 */
    }

    /* C4LOCK_ST_BLIND */
    motor_left(MOTOR_FWD,  C4_LOCK_FWD_SPEED);
    motor_right(MOTOR_FWD, C4_LOCK_FWD_SPEED);

    if (g_c4lock_ms >= C4_LOCK_BLIND_MS) {
        g_c4lock_st = C4LOCK_ST_OFF;
        g_c4lock_ms = 0;
    }
    return 1;
}
/* ==================== 새 시퀀스 카운트5 : 라인 무시 무조건 직진 ====================
 * 처리했으면 1 반환. 1500ms 동안 센서 무시하고 그냥 직진한다. */
static uint8_t c5_blind_update(void)
{
    if (!g_c5_blind_active) return 0;

    motor_left(MOTOR_FWD,  C5_BLIND_SPEED);
    motor_right(MOTOR_FWD, C5_BLIND_SPEED);

    g_c5_blind_ms = (uint16_t)(g_c5_blind_ms + SAMPLE_MS);
    if (g_c5_blind_ms >= C5_BLIND_MS) {
        g_c5_blind_active = 0;
        g_c5_blind_ms     = 0;
    }
    return 1;
}

/* ==================== 새 시퀀스 카운트6 : 무조건 직진 -> 무기한 정지 ====================
 * 처리했으면 1 반환. 0.3초 동안 센서 무시하고 직진한 뒤,
 * center_stop 을 세워 무기한 정지로 넘긴다. */
static uint8_t c6_blind_update(void)
{
    if (!g_c6_blind_active) return 0;

    if (g_c6_blind_ms < C6_BLIND_MS) {
        motor_left(MOTOR_FWD,  C6_BLIND_SPEED);
        motor_right(MOTOR_FWD, C6_BLIND_SPEED);
        g_c6_blind_ms = (uint16_t)(g_c6_blind_ms + SAMPLE_MS);
    } else {
        g_c6_blind_active = 0;
        g_c6_blind_ms     = 0;
        g_center_stop     = 1;   /* 이후 center_stop_update() 가 무기한 정지 */
    }
    return 1;
}

/* ==================== 카운트12 : 회전 후 후진 ====================
 * 처리했으면 1 반환. 500ms 동안 후진한 뒤, 라인 추종으로 복귀한다. */
static uint8_t c12_back_update(void)
{
    if (!g_c12_back_active) return 0;

    motor_left(MOTOR_BWD,  C12_BACK_SPEED);
    motor_right(MOTOR_BWD, C12_BACK_SPEED);

    g_c12_back_ms = (uint16_t)(g_c12_back_ms + SAMPLE_MS);
    if (g_c12_back_ms >= C12_BACK_MS) {
        g_c12_back_active = 0;
        g_c12_back_ms     = 0;
    }
    return 1;
}

/* ==================== 차단바 구간 ====================
 * 처리했으면 1 반환. WATCH 중에는 라인 추종을 계속하고 거리만 본다. */
static uint8_t bar_update(void)
{
    if (g_bar_st != BAR_ST_WATCH && g_bar_st != BAR_ST_WAIT) return 0;

    g_bar_ms = (uint16_t)(g_bar_ms + SAMPLE_MS);
    if (g_bar_ms >= PSD_PERIOD_MS) {
        g_bar_ms = 0;
        psd_measure();
    }

    if (g_bar_st == BAR_ST_WATCH) {
        /* 가까워지면(ADC 큼) 정지 */
        if (g_psd_adc >= BAR_STOP_ADC) {
            g_bar_st      = BAR_ST_WAIT;
            g_bar_open_ms = 0;
            motor_stop_all();
            return 1;
        }
        return 0;   /* 아직은 평소대로 라인 추종 */
    }

    /* BAR_ST_WAIT : 정지한 채 차단바가 올라가기를 기다린다 */
    motor_stop_all();

    /* 멀어지면(ADC 작음) 차단바가 올라간 것 */
    if (g_psd_adc <= BAR_OPEN_ADC) {
        g_bar_open_ms = (uint16_t)(g_bar_open_ms + SAMPLE_MS);
        if (g_bar_open_ms >= BAR_OPEN_HOLD_MS) {
            g_bar_st = BAR_ST_DONE;

            /* 구간은 그대로 SEC_CORNER. 카운트만 리셋한다.
             * T자 감지는 corner_update() 가 지연 후 활성화한다. */
            g_sec_count       = 0;
            print_lcd_count();
            g_cross_line_prev = 0;

            return 0;
        }
    } else {
        g_bar_open_ms = 0;
    }
    return 1;
}

/* ==================== 주차 구간 ====================
 * 처리했으면 1 반환.
 * 정지 -> 허공 직진 -> 왼쪽 세로선 인식 정지 -> 180도 회전 -> 라인 추종 복귀 */
static uint8_t park_update(void)
{
    if (g_section != SEC_PARK)   return 0;
    if (g_park_st == PARK_ST_OFF) return 0;

    g_park_ms = (uint16_t)(g_park_ms + SAMPLE_MS);

    switch (g_park_st) {
    case PARK_ST_PAUSE:
        /* 관성으로 회전이 남아 있을 수 있으므로 잠깐 세운다 */
        motor_stop_all();
        if (g_park_ms >= PARK_PAUSE_MS) {
            g_park_st = PARK_ST_FWD;
            g_park_ms = 0;
        }
        break;

    case PARK_ST_FWD:
        motor_left(MOTOR_FWD,  PARK_FWD_SPEED);
        motor_right(MOTOR_FWD, PARK_FWD_SPEED);

        /* 출발 직후에는 방금 지난 세로선이 잡히므로 판정을 미룬다 */
        if (g_park_ms >= PARK_SEEK_DELAY_MS) {
            if (park_line_detect()) {
                motor_stop_all();
                g_park_st = PARK_ST_STOP;
                g_park_ms = 0;
                break;
            }
        }

        /* 선을 못 찾으면 강제 정지 (안전장치) */
        if (g_park_ms >= PARK_FWD_TIMEOUT_MS) {
            motor_stop_all();
            g_park_st = PARK_ST_STOP;
            g_park_ms = 0;
        }
        break;

    case PARK_ST_STOP:
        motor_stop_all();
        if (g_park_ms >= PARK_STOP_MS) {
            g_park_st = PARK_ST_OUT_TURN;
            g_park_ms = 0;
        }
        break;

    case PARK_ST_OUT_TURN: {
	    /* 정렬 판정 없이 고정 시간(PARK_OUT_TURN_MAX_MS) 동안만 무조건 회전한다. */
	    if (g_park_ms >= PARK_OUT_TURN_MAX_MS) {
		    g_park_st = PARK_ST_DONE;
		    g_park_ms = 0;

		    g_kick_left.zero_ms  = 0;
		    g_kick_right.zero_ms = 0;
		    g_last_error_x10     = (PARK_OUT_TURN_DIR) ? 640 : -640;
		    return 0;               /* 라인 추종으로 복귀 */
	    }

	    /* 제자리 회전 : 양쪽 바퀴를 반대로 돌린다 */
	    if (PARK_OUT_TURN_DIR) {
		    motor_left(MOTOR_FWD,  PARK_OUT_TURN_SPEED);
		    motor_right(MOTOR_BWD, PARK_OUT_TURN_SPEED);
		    } else {
		    motor_left(MOTOR_BWD,  PARK_OUT_TURN_SPEED);
		    motor_right(MOTOR_FWD, PARK_OUT_TURN_SPEED);
	    }
	    break;
    }

    case PARK_ST_DONE:
    default:
        /* 회전 완료. 이후는 라인 추종에 맡기고
         * 카운트 PARK_EXIT_COUNT 에서 좌회전해 탈출한다. */
        return 0;
    }

    return 1;
}

/* ==================== 교차점 처리 ==================== */
/* 8자 접점에 다가가는 걸 미리 감지해 반대로 돌기 시작 */
static void cross_approach_check(void)
{
    uint8_t gap;

    if (g_section != SEC_FIG8) return;

    gap = cross_outer_gap_detect();

    if (gap && !g_outer_gap_prev && g_sec_count == 1 &&
        !g_cross_turn_active && g_approach_armed) {
        g_cross_turn_dir    = (uint8_t)!g_cross_turn_dir;
        g_cross_turn_active = 1;
        g_cross_turn_ms     = 0;
        g_approach_armed    = 0;
    }
    g_outer_gap_prev = gap;
}

/* 1행 : 구간 문자 + 그 구간의 카운트 */
static void print_lcd_count(void)
{
    char    line[17];
    char    tmp[16];
    uint8_t k, p;
    char    sc;

    switch (g_section) {
    case SEC_FIG8:   sc = '8'; break;
    case SEC_LANE:   sc = 'L'; break;
    case SEC_CORNER: sc = 'C'; break;
    case SEC_PARK:   sc = 'P'; break;
    default:         sc = 'B'; break;
    }

    for (p = 0; p < 16; p++) line[p] = ' ';
    line[16] = '\0';

    sprintf(tmp, "%c cnt:%3u", sc, g_sec_count);
    for (k = 0; tmp[k] && k < 16; k++) line[k] = tmp[k];

    lcd_set_cursor(0, 0);
    lcd_print(line);
}

/* 교차점 진입 순간에만 카운트 + 구간별 동작.
 * 카운트는 g_sec_count 이므로 구간이 다르면 서로 간섭하지 않는다. */
static void cross_count_check(void)
{
    uint8_t hit;

    /* 주차 진행 중에만 5개 기준. 180도 회전이 끝나면 일반 기준으로 */
    if (g_section == SEC_PARK && g_park_st != PARK_ST_DONE)
    hit = park_line_detect();
    else if (g_section == SEC_BLACK && g_sec_count == 11)
    /* 카운트 11 -> 12 로 넘어가는 트리거만 왼쪽 끝 IR0 단독 인식으로 판정 */
    hit = (uint8_t)((g_led_state & 0x01) != 0);
    else if (g_section == SEC_BLACK && g_wall_st == WALL_ST_DONE && g_sec_count == 2)
    /* 벽 탐지 이후 새 시퀀스 카운트 2 : "+" 구간만 인식하도록,
     * 가운데 4개(IR1~4)가 전부(4/4) 켜질 때만 카운트를 올린다.
     * 기존 cross_line_detect()는 3/4 이라 "ㅓ" 자 형태도 걸릴 수 있어
     * 여기서는 더 엄격한 4/4 조건을 쓴다. */
    hit = (uint8_t)((g_led_state & 0x1E) == 0x1E);
    else
    hit = cross_line_detect();

    /* 주행 시작 직후 일정 시간은 카운트하지 않는다 */
    if (g_run_lock_ms > 0) {
        if (g_run_lock_ms > SAMPLE_MS) g_run_lock_ms -= SAMPLE_MS;
        else                           g_run_lock_ms = 0;
        g_cross_line_prev = hit;
        return;
    }

    /* 잠금 중이면 카운트하지 않는다 */
    if (g_cross_lock_ms > 0) {
        if (g_cross_lock_ms > SAMPLE_MS) g_cross_lock_ms -= SAMPLE_MS;
        else                             g_cross_lock_ms = 0;
        g_cross_line_prev = hit;
        return;
    }

    /* 차선 구간, ㄱ자/T자 회전 중, 반전 직진 중, 그리고 카운트13 이후
     * (정지/PSD전진/벽탐지) 구간에는 카운트 판정을 하지 않는다.
     * 단, 벽탐지 구간이 WALL_ST_DONE 으로 끝나 라인 추종으로 복귀한 뒤에는
     * 다시 정상적으로 카운트해야 하므로 DONE 은 예외로 둔다. */
    if (g_section  == SEC_LANE ||
        g_inv_st   == INV_ST_FWD ||
        g_corner_st == CORNER_ST_TURN ||
        g_corner_st == CORNER_ST_T_TURN ||
        g_bcorner_st == BCORNER_ST_TURN ||
        (g_black13_st != BLACK13_ST_OFF && g_black13_st != BLACK13_ST_DONE) ||
        (g_wall_st != WALL_ST_OFF && g_wall_st != WALL_ST_DONE)) {
        g_cross_line_prev = hit;
        return;
    }

    if (hit && !g_cross_line_prev && !g_cross_turn_active) {
        g_sec_count++;
        print_lcd_count();
        g_approach_armed  = 1;
        g_two_line_active = 0;
        g_two_line_ms     = 0;

        switch (g_section) {

        case SEC_FIG8:
#if FIG8_LOCK_COUNT > 0
            if (g_sec_count == FIG8_LOCK_COUNT) g_cross_lock_ms = FIG8_LOCK_MS;
#endif
            /* 평사 진입 : 회전하지 않고 곧바로 차선 주행으로 */
            if (g_sec_count == FIG8_LANE_COUNT) {
                g_cross_turn_active = 0;
                g_lane_st = LANE_ST_STOP;
                g_lane_ms = 0;
                section_enter(SEC_LANE);
                return;
            }

            if (g_sec_count == FIG8_EXIT_COUNT) {
                g_cross_turn_dir = FIG8_EXIT_TURN_DIR;
            } else if (g_sec_count == FIG8_C2_COUNT) {
                g_cross_turn_dir = FIG8_C2_TURN_DIR;
            } else {
                if (g_cal_set == CAL_BLACK) {
                    uint8_t left_cnt  = led_on_count((uint8_t)(g_led_state & 0x07));
                    uint8_t right_cnt = led_on_count((uint8_t)(g_led_state & 0x38));
                    g_cross_turn_dir = (left_cnt > right_cnt) ? 0 : 1;
                } else {
                    g_cross_turn_dir = (ir_sig(0) > ir_sig(5)) ? 0 : 1;
                }
            }
            break;

        case SEC_CORNER:
            /* ㄱ자와 T자는 corner_update() 가 전담한다.
             * 여기서는 회전을 걸지 않는다. */
            g_cross_line_prev = hit;
            return;

        case SEC_PARK:
            /* 카운트 2 : 잠깐 정지한 뒤 허공 직진. 회전하지 않는다 */
            if (g_sec_count == PARK_FWD_COUNT) {
                g_park_st = PARK_ST_PAUSE;
                g_park_ms = 0;
                g_cross_turn_active = 0;
                g_cross_line_prev   = hit;
                return;
            }
            /* 카운트 4 : 좌회전해서 탈출 */
            if (g_sec_count == PARK_EXIT_COUNT) {
                g_cross_turn_dir = PARK_EXIT_TURN_DIR;
                break;
            }
            /* 그 외에는 회전하지 않는다 */
            g_cross_line_prev = hit;
            return;

        case SEC_BLACK:
            /* 벽 탐지 구간(WALL_ST_DONE) 통과 이후의 새 시퀀스 :
             * 카운트 2 -> 무조건 우회전 (직진 후 제자리 회전) */
            if (g_wall_st == WALL_ST_DONE && g_sec_count == 2) {
                g_turn_prep_dir    = 1;   /* 1=우 */
                g_turn_prep_active = 1;
                g_turn_prep_ms     = 0;
                g_cross_line_prev  = hit;
                return;
            }
            /* 새 시퀀스 카운트 3 : IR 6개 인식으로 이미 여기까지 올라온
             * 상태다. 회전하지 않고 카운트만 세운 채 그대로 직진한다.
             * 다음 교차점(가운데 4개 기준)에서 카운트 4가 되면
             * 아래 좌우비교 회전 그룹(2,3,4,5,7,8,9)에 걸려 회전한다. */
            if (g_wall_st == WALL_ST_DONE && g_sec_count == 3) {
                g_cross_line_prev = hit;
                return;
            }
            /* 새 시퀀스 카운트 5 : 기존 좌우비교 회전 그룹과 별개로,
			* 회전 없이 1500ms 동안 라인 무시하고 무조건 직진한다 */
			if (g_wall_st == WALL_ST_DONE && g_sec_count == 5) {
				g_c5_blind_active = 1;
				g_c5_blind_ms     = 0;
				g_cross_lock_ms   = C5_BLIND_MS;
				g_cross_line_prev = hit;
				 return;
			}
            /* 새 시퀀스 카운트 6 : 중앙 정지 구간. 0.3초 무조건 직진 후 무기한 정지 */
            if (g_wall_st == WALL_ST_DONE && g_sec_count == 6) {
	            g_c6_blind_active  = 1;
	            g_c6_blind_ms      = 0;
	            g_cross_line_prev  = hit;
	            return;
            }
            /* 카운트 1 : 6초 동안 카운트 잠금만 걸고 회전하지 않는다 */
            if (g_sec_count == BLACK_LOCK_COUNT) {
                g_cross_lock_ms   = BLACK_LOCK_MS;
                g_cross_line_prev = hit;
                return;
            }
            /* 카운트 11 : 회전 없이 1초 카운트만 잠근다 */
            if (g_sec_count == 11) {
	            g_cross_lock_ms   = BLACK_C11_LOCK_MS;
	            g_cross_line_prev = hit;
	            return;
            }
            /* 카운트 13 : 500ms 정지 후 PSD 감시하며 전진, 700 이상이면 정지
             * (black13_update() 가 전담. 여기서는 트리거만 건다) */
            if (g_sec_count == 13) {
	            g_black13_st      = BLACK13_ST_PAUSE;
	            g_black13_ms      = 0;
	            g_cross_line_prev = hit;
	            return;
            }
            /* 카운트 6, 10 : 회전 없이 0.5초 카운트만 잠근다 */
            if (g_sec_count == 6 || g_sec_count == 10) {
	            g_cross_lock_ms   = BLACK_C6_LOCK_MS;
	            g_cross_line_prev = hit;
	            return;
            }
            /* 카운트 12 : 무조건 좌회전 (직진 후 제자리 회전) */
            if (g_sec_count == 12) {
                g_turn_prep_dir    = 0;   /* 0=좌 */
                g_turn_prep_active = 1;
                g_turn_prep_ms     = 0;
                g_cross_line_prev  = hit;
                return;
            }
            /* 카운트 2, 3, 4, 5, 7, 8, 9 : 좌우 LED 개수 비교로 방향을 정하고
             * 곧바로 회전하지 않고 짧게 직진부터 시작한다.
             * (원래 시퀀스의 2,3,4,5,7,8,9 뿐 아니라, 새 시퀀스가 IR 6개
             *  인식을 거쳐 카운트 4에 도달했을 때의 회전도 여기서 처리된다.
             *  새 시퀀스 카운트 2, 3 은 위에서 이미 걸러졌으므로
             *  여기 도달하는 카운트 2 는 항상 원래 시퀀스다) */
            if (g_sec_count == 2 || g_sec_count == 3 ||
                g_sec_count == 4 || g_sec_count == 5 ||
                g_sec_count == 7 || g_sec_count == 8 ||
                g_sec_count == 9) {
                uint8_t left_cnt  = led_on_count((uint8_t)(g_led_state & 0x07));
                uint8_t right_cnt = led_on_count((uint8_t)(g_led_state & 0x38));

                g_turn_prep_dir    = (left_cnt > right_cnt) ? 0 : 1;
                g_turn_prep_active = 1;
                g_turn_prep_ms     = 0;
                g_cross_line_prev  = hit;
                return;
            }
            /* 그 외에는 회전 없이 카운트만 센다 */
            g_cross_line_prev = hit;
            return;

        default:
            g_cross_line_prev = hit;
            return;
        }

        g_cross_turn_active = 1;
        g_cross_turn_ms     = 0;
    }
    g_cross_line_prev = hit;
}

/* 회전 중이면 1 반환 */
static uint8_t cross_turn_update(void)
{
    uint8_t  aligned = 0;
    uint16_t min_ms;
    uint8_t  center_on = 0, outer_busy = 0;

    if (!g_cross_turn_active) return 0;

    if (g_section == SEC_FIG8 && g_sec_count == FIG8_EXIT_COUNT)
        min_ms = FIG8_EXIT_TURN_MIN_MS;
    else if (g_section == SEC_FIG8 && g_sec_count == FIG8_C2_COUNT)
        min_ms = FIG8_C2_TURN_MIN_MS;
    else if (g_section == SEC_PARK && g_sec_count == PARK_EXIT_COUNT)
        min_ms = PARK_EXIT_TURN_MIN_MS;
    else if (g_section == SEC_BLACK && g_sec_count == 9)
        min_ms = BLACK_C9_TURN_MIN_MS;
    else if (g_section == SEC_BLACK &&
             (g_sec_count == 2 || g_sec_count == 3 ||
              g_sec_count == 4 || g_sec_count == 5 ||
              g_sec_count == 7 || g_sec_count == 8 || g_sec_count == 12))
        min_ms = BLACK_TURN_MIN_MS;
    else
        min_ms = CROSS_TURN_MIN_MS;

    g_cross_turn_ms = (uint16_t)(g_cross_turn_ms + SAMPLE_MS);

    /* 가운데가 선을 잡고 바깥이 조용해지면 정렬 완료.
     * min_ms 전에는 판정하지 않는다 (안 그러면 회전 시작 즉시 종료됨). */
    if (g_cross_turn_ms >= min_ms) {
        if (g_cal_set == CAL_BLACK) {
            center_on  = (g_led_state & 0x0C) == 0x0C;
            outer_busy = led_on_count((uint8_t)(g_led_state & 0x33));
        } else {
            center_on  = (ir_sig(2) >= CROSS_TURN_CENTER_TH &&
                         ir_sig(3) >= CROSS_TURN_CENTER_TH);
            outer_busy = 0;
            if (ir_sig(0) >= CROSS_TURN_OUTER_TH) outer_busy++;
            if (ir_sig(1) >= CROSS_TURN_OUTER_TH) outer_busy++;
            if (ir_sig(4) >= CROSS_TURN_OUTER_TH) outer_busy++;
            if (ir_sig(5) >= CROSS_TURN_OUTER_TH) outer_busy++;
        }
        if (center_on && outer_busy <= CROSS_TURN_OUTER_TOLERANCE) aligned = 1;
    }

    {
        uint16_t max_ms = CROSS_TURN_MAX_MS;

        if (g_section == SEC_BLACK && g_sec_count == 9)
            max_ms = BLACK_C9_TURN_MAX_MS;
        else if (g_section == SEC_BLACK &&
                 (g_sec_count == 2 || g_sec_count == 3 ||
                  g_sec_count == 4 || g_sec_count == 5 ||
                  g_sec_count == 7 || g_sec_count == 8 || g_sec_count == 12))
            max_ms = BLACK_TURN_MAX_MS;

        if (aligned || g_cross_turn_ms >= max_ms) {
            g_cross_turn_active  = 0;
            g_cross_turn_ms      = 0;
            g_sharp_state        = 0;
            g_kick_left.zero_ms  = 0;
            g_kick_right.zero_ms = 0;

            g_two_line_active    = 0;
            g_two_line_ms        = 0;
            g_two_line_prev      = 0;

            if (g_section == SEC_BLACK && g_sec_count == 9) {
	            g_cross_lock_ms   = BLACK_C9_LOCK_MS;
	            g_cross_line_prev = 1;   /* 회전 끝난 그 자리에서 재카운트 방지 */
            } else if (g_section == SEC_BLACK && g_sec_count == 3) {
	            g_cross_lock_ms   = BLACK_C3_LOCK_MS;
	            g_cross_line_prev = 1;
            } else if (g_section == SEC_BLACK && g_wall_st == WALL_ST_DONE &&
            g_sec_count == 4) {
	            /* 새 시퀀스 카운트 4 : 회전 종료 후 0.5초 라인 추종 + 1초 무조건 직진 */
	            g_c4lock_st       = C4LOCK_ST_FOLLOW;
	            g_c4lock_ms       = 0;
	            g_cross_lock_ms   = C4_LOCK_FOLLOW_MS + C4_LOCK_BLIND_MS;
	            g_cross_line_prev = 1;
	            } else if (g_section == SEC_BLACK && g_sec_count == 12) {
	            /* 카운트 12 : 좌회전 종료 후 500ms 후진, 그 다음 라인 추종 복귀 */
	            g_c12_back_active = 1;
	            g_c12_back_ms     = 0;
            }
            return 0;
        }
    }

    if (g_section == SEC_BLACK) {
        /* 반전 구간은 피벗 대신 제자리 회전(양 바퀴 반대 방향) 을 쓴다 */
        if (g_cross_turn_dir) {
            motor_left(MOTOR_FWD,  BLACK_SPIN_SPEED);
            motor_right(MOTOR_BWD, BLACK_SPIN_SPEED);
            g_last_error_x10 = 640;
        } else {
            motor_left(MOTOR_BWD,  BLACK_SPIN_SPEED);
            motor_right(MOTOR_FWD, BLACK_SPIN_SPEED);
            g_last_error_x10 = -640;
        }
    } else if (g_cross_turn_dir) {
        motor_left(MOTOR_FWD, CROSS_TURN_OUTER);
        if (CROSS_TURN_INNER > 0) motor_right(MOTOR_BWD, CROSS_TURN_INNER);
        else                      motor_right(MOTOR_STOP, 0);
        g_last_error_x10 = 640;
    } else {
        motor_right(MOTOR_FWD, CROSS_TURN_OUTER);
        if (CROSS_TURN_INNER > 0) motor_left(MOTOR_BWD, CROSS_TURN_INNER);
        else                      motor_left(MOTOR_STOP, 0);
        g_last_error_x10 = -640;
    }
    return 1;
}

/* ==================== 라인 추종 ==================== */
/* 센서 위치 가중치 (*10 고정소수점) */
static const int8_t pos_weight_x10[IR_NUM] = { -58, -33, -11, 11, 33, 58 };

static void line_follow_reset(void)
{
    g_psd_adc        = 0;
    g_last_error_x10 = 0;
    g_sharp_state    = 0;

    g_search_ms       = 0;
    g_search_reversed = 0;

    g_kick_active   = 1;
    g_kick_timer_ms = 0;

    g_kick_left.zero_ms  = 0;  g_kick_left.kicking  = 0;  g_kick_left.kick_ms  = 0;
    g_kick_right.zero_ms = 0;  g_kick_right.kicking = 0;  g_kick_right.kick_ms = 0;

    g_cross_turn_active = 0;
    g_cross_turn_ms     = 0;

    g_two_line_active = 0;
    g_two_line_ms     = 0;
    g_two_line_prev   = 0;

    g_lane_st = LANE_ST_OFF;
    g_lane_ms = 0;

    g_bar_st      = BAR_ST_OFF;
    g_bar_ms      = 0;
    g_bar_open_ms = 0;

    g_park_st = PARK_ST_OFF;
    g_park_ms = 0;

    g_corner_st      = CORNER_ST_OFF;
    g_corner_ms      = 0;
    g_tcorner_arm_ms = 0;

    g_bcorner_st  = BCORNER_ST_OFF;
    g_bcorner_ms  = 0;

    g_turn_prep_active = 0;
    g_turn_prep_ms     = 0;

    g_black13_st      = BLACK13_ST_OFF;
    g_black13_ms      = 0;
    g_black13_psd_ms  = 0;

    g_wall_st     = WALL_ST_OFF;
    g_wall_ms     = 0;
    g_wall_psd_ms = 0;

    g_cal_set = CAL_WHITE;
    g_inv_st  = INV_ST_WAIT;
    g_inv_ms  = 0;
	
	g_center_stop = 0;
	
	g_c4lock_st = C4LOCK_ST_OFF;
	g_c4lock_ms = 0;
	
	g_c5_blind_active = 0;
	g_c5_blind_ms     = 0;
	
	g_c6_blind_active = 0;
	g_c6_blind_ms     = 0;
	
	g_c12_back_active = 0;
	g_c12_back_ms     = 0;
}

static void line_follow_update(void)
{
    uint8_t  i;
    int32_t  weighted_sum = 0;
    int32_t  total = 0;
    int16_t  error_x10, turn;
    uint8_t  sig_left, sig_right, lost_th;
    uint16_t sum_left_half = 0, sum_right_half = 0;

    kick_tick();

    /* 우선순위 :
     * 반전 > 반전구간ㄱ자(비활성) > 카운트13(정지/PSD전진) > 벽탐지구간
     * > 반전구간직진+회전 > 차선 > ㄱ자/T자 > 차단바 > 주차
     * > 교차점 회전 > 두 선 > 라인 추종 */
	if (center_stop_update())   return;
    if (invert_update())        return;
    if (bcorner_update())       return;
    if (black13_update())       return;
    if (black_wall_update())    return;
    if (turn_prep_update()) return;
    if (lane_follow_update())   return;
    if (corner_update())        return;
    if (bar_update())           return;
    if (park_update())          return;

    if (cross_turn_update()) return;
    if (!g_cross_turn_active && g_cross_turn_ms) {
        g_sharp_state   = 0;
        g_cross_turn_ms = 0;
    }
	if (c5_blind_update()) return;
	if (c6_blind_update()) return;
	if (c12_back_update()) return;
	if (c4lock_update()) return;

    if (two_line_update()) return;

    for (i = 0; i < IR_NUM; i++) {
        uint8_t sig = ir_sig(i);

        weighted_sum += (int32_t)sig * pos_weight_x10[i];
        total        += sig;

        if (i < IR_NUM / 2) sum_left_half  = (uint16_t)(sum_left_half + sig);
        else                sum_right_half = (uint16_t)(sum_right_half + sig);
    }
    sig_left  = ir_sig(0);
    sig_right = ir_sig(IR_NUM - 1);

    /* 선을 놓침 : 마지막 방향으로 제자리 회전하며 재탐색.
     * 600ms(SEARCH_FLIP_MS) 안에 못 찾으면 반대 방향으로 전환하고,
     * 그 이후로는 선을 찾을 때까지 무제한으로 그 방향을 유지한다. */
    lost_th = (g_cal_set == CAL_BLACK) ? LINE_LOST_TH_BLACK : LINE_LOST_TH;
    if (total < lost_th) {
        uint8_t go_right;   /* 1이면 마지막 방향 기준 오른쪽으로 회전 */

        g_sharp_state = 0;
        g_search_ms   = (uint16_t)(g_search_ms + SAMPLE_MS);

        if (!g_search_reversed && g_search_ms >= SEARCH_FLIP_MS) {
            g_search_reversed = 1;   /* 이후로는 무제한, 반대 방향 유지 */
        }

        go_right = (uint8_t)(g_last_error_x10 >= 0);
        if (g_search_reversed) go_right = (uint8_t)!go_right;

        if (go_right) {
            motor_left(MOTOR_BWD, SEARCH_SPEED);
            motor_right(MOTOR_FWD, SEARCH_SPEED);
        } else {
            motor_left(MOTOR_FWD, SEARCH_SPEED);
            motor_right(MOTOR_BWD, SEARCH_SPEED);
        }
        return;
    }

    /* 선을 다시 찾았으면 재탐색 상태를 초기화 */
    g_search_ms       = 0;
    g_search_reversed = 0;

    /* 급회전 상태 전이 (히스테리시스). SEC_FIG8 에서만 쓴다. */
    switch (g_sharp_state) {
    case 0:
        if (sig_left >= SHARP_TURN_TH_ON &&
            (sum_right_half / (IR_NUM / 2)) < SHARP_OPPOSITE_TH) {
            g_sharp_state = 1;
        } else if (sig_right >= SHARP_TURN_TH_ON &&
                   (sum_left_half / (IR_NUM / 2)) < SHARP_OPPOSITE_TH) {
            g_sharp_state = 2;
        }
        break;

    case 1:
        if (sig_right >= SHARP_TURN_TH_ON && sig_right > sig_left) {
            g_sharp_state = 2;
            g_kick_left.zero_ms = 0;
        } else if (sig_left < SHARP_TURN_TH_OFF) {
            g_sharp_state = 0;
            g_kick_left.zero_ms = 0;
        }
        break;

    case 2:
        if (sig_left >= SHARP_TURN_TH_ON && sig_left > sig_right) {
            g_sharp_state = 1;
            g_kick_right.zero_ms = 0;
        } else if (sig_right < SHARP_TURN_TH_OFF) {
            g_sharp_state = 0;
            g_kick_right.zero_ms = 0;
        }
        break;
    }

    /* 8자 구간이 아니면 급회전을 쓰지 않는다 */
    if (g_section != SEC_FIG8) g_sharp_state = 0;

    if (g_sharp_state == 1) {
        g_last_error_x10 = -640;
#if SHARP_INNER_SPEED > 0
        motor_left(MOTOR_FWD, SHARP_INNER_SPEED);
#else
        motor_left(MOTOR_STOP, 0);
#endif
        motor_right(MOTOR_FWD, SHARP_TURN_SPEED);
        return;
    }
    if (g_sharp_state == 2) {
        g_last_error_x10 = 640;
#if SHARP_INNER_SPEED > 0
        motor_right(MOTOR_FWD, SHARP_INNER_SPEED);
#else
        motor_right(MOTOR_STOP, 0);
#endif
        motor_left(MOTOR_FWD, SHARP_TURN_SPEED);
        return;
    }

    /* 일반 구간 : 비례 제어 */
    error_x10 = (int16_t)(weighted_sum / total);   /* 약 -64 ~ +64 */
    g_last_error_x10 = error_x10;

    turn = (int16_t)(((int32_t)KP * error_x10) / 10);

    motor_left(MOTOR_FWD,  clamp_speed((int16_t)(BASE_SPEED + turn)));
    motor_right(MOTOR_FWD, clamp_speed((int16_t)(BASE_SPEED - turn)));
}

/* ==================== LCD ==================== */
/* 디버그 화면 (SW3) */
static void print_lcd_debug(void)
{
    char    buf[20];
    uint8_t i, hit = 0;
    uint8_t d[IR_NUM];
    char    sc;

    for (i = 0; i < IR_NUM; i++) {
        uint8_t v = (uint8_t)(ir_sig(i) / 10);
        d[i] = (uint8_t)((v > 9) ? 9 : v);
    }

    for (i = 1; i <= 4; i++)
        if (g_led_state & (uint8_t)(1 << i)) hit++;

    sprintf(buf, "%u%u%u%u%u%u H:%u %c    ",
            d[0], d[1], d[2], d[3], d[4], d[5],
            hit, (hit >= 3) ? 'C' : '.');
    buf[16] = '\0';
    lcd_set_cursor(0, 0);
    lcd_print(buf);

    switch (g_section) {
    case SEC_FIG8:   sc = '8'; break;
    case SEC_LANE:   sc = 'L'; break;
    case SEC_CORNER: sc = 'C'; break;
    case SEC_PARK:   sc = 'P'; break;
    default:         sc = 'B'; break;
    }

    /* PSD(앞/오른쪽) / 구간 / 카운트 / 캘리브레이션 세트 */
    sprintf(buf, "F%3u R%3u %c%u%c",
            (g_psd_adc       > 999) ? 999 : g_psd_adc,
            (g_psd_right_adc > 999) ? 999 : g_psd_right_adc,
            sc,
            (unsigned)(g_sec_count > 9 ? 9 : g_sec_count),
            (g_cal_set == CAL_WHITE) ? 'W' : 'B');
    buf[16] = '\0';
    lcd_set_cursor(0, 1);
    lcd_print(buf);
}

/* set : 지금 캘리브레이션 중인 세트 */
static void print_lcd_calib(uint8_t set)
{
    char     buf[24];
    uint8_t  i, ok = 0;
    uint16_t worst = 1023;
    uint16_t min_range = (set == CAL_BLACK) ? MIN_RANGE_BLACK : MIN_RANGE;

    for (i = 0; i < IR_NUM; i++) {
        uint16_t r = (ir[i].max[set] > ir[i].min[set])
                     ? (uint16_t)(ir[i].max[set] - ir[i].min[set]) : 0;
        if (r < worst) worst = r;
        if (r >= min_range) ok++;
    }

    lcd_set_cursor(0, 0);
    lcd_print((set == CAL_WHITE) ? "CAL W  wave hand"
                                 : "CAL B  wave hand");
    sprintf(buf, "ok%u/%u min r=%4u", ok, (unsigned)IR_NUM, worst);
    buf[16] = '\0';
    lcd_set_cursor(0, 1);
    lcd_print(buf);
}

/* ==================== main ==================== */
static void run_state_reset(void)
{
    g_section         = SEC_FIG8;
    g_sec_count       = 0;
    g_psd_adc         = 0;
    g_cross_line_prev = 0;
    g_outer_gap_prev  = 0;
    g_approach_armed  = 1;
    g_cross_lock_ms   = 0;

    g_search_ms       = 0;
    g_search_reversed = 0;

    g_lane_st = LANE_ST_OFF;
    g_lane_ms = 0;

    g_bar_st      = BAR_ST_OFF;
    g_bar_ms      = 0;
    g_bar_open_ms = 0;

    g_park_st = PARK_ST_OFF;
    g_park_ms = 0;

    g_corner_st      = CORNER_ST_OFF;
    g_corner_ms      = 0;
    g_tcorner_arm_ms = 0;

    g_bcorner_st  = BCORNER_ST_OFF;
    g_bcorner_ms  = 0;

    g_turn_prep_active = 0;
    g_turn_prep_ms     = 0;

    g_black13_st      = BLACK13_ST_OFF;
    g_black13_ms      = 0;
    g_black13_psd_ms  = 0;

    g_wall_st     = WALL_ST_OFF;
    g_wall_ms     = 0;
    g_wall_psd_ms = 0;

    g_cal_set = CAL_WHITE;
    g_inv_st  = INV_ST_WAIT;
    g_inv_ms  = 0;
	
	g_center_stop = 0;
	
	g_c4lock_st = C4LOCK_ST_OFF;
	g_c4lock_ms = 0;
	
	g_c5_blind_active = 0;
	g_c5_blind_ms     = 0;
	
	g_c6_blind_active = 0;
	g_c6_blind_ms     = 0;
	
	g_c12_back_active = 0;
	g_c12_back_ms     = 0;

    g_run_lock_ms = RUN_START_LOCK_MS;
}

#if TEST_SKIP_TO_PARK_EXIT
/* 테스트용 : 주차 구간을 통과한 직후 상태로 맞춘다.
 * run_state_reset() 을 먼저 부른 뒤에 호출해야 한다.
 * 주의 : 이 상태에서는 SEC_BLACK (반전) 으로 가지 않으므로
 * 반전/ㄱ자(카운트4)/교차점(카운트2,3) 로직 테스트에는 쓸 수 없다. */
static void skip_to_after_park(void)
{
    g_corner_st = CORNER_ST_T_DONE;     /* ㄱ자 / T자 완료 */
    g_bar_st    = BAR_ST_DONE;          /* 차단바 통과 */

    g_section   = SEC_PARK;
    g_sec_count = PARK_EXIT_COUNT;      /* 주차 탈출 직후 카운트 = 4 */

    g_park_st = PARK_ST_DONE;           /* 주차 완료 */
    g_park_ms = 0;

    g_cross_line_prev = 1;
    g_run_lock_ms     = 0;              /* 바로 카운트를 받는다 */
}
#endif

int main(void)
{
    uint8_t  i;
    uint8_t  state   = ST_IDLE;
    uint8_t  cal_set = CAL_WHITE;   /* 지금 캘리브레이션 중인 세트 */
    uint8_t  edge;
    uint16_t tick = 0;

    /* PF4~PF7 은 기본이 JTAG 핀이라 ADC 로 쓰려면 해제해야 한다.
     * JTD 는 4클럭 안에 두 번 써야 적용된다. */
    MCUCSR |= (1 << JTD);
    MCUCSR |= (1 << JTD);

    adc_init();
    led_init();
    btn_init();
    motor_init();
    motor_stop_all();
    lcd_init();

    for (i = 0; i < IR_NUM; i++) ir_ch_reset(&ir[i]);

    /* ir_ch_reset() 이 min/max 를 되돌리므로 그 다음에 불러와야 한다 */
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Line tracer     ");
    lcd_set_cursor(0, 1);
    lcd_print(calib_load_from_eeprom() ? "SW4:run  SW3:dbg"
                                       : "SW1/2:cal SW3:db");

    while (1) {
        edge = btn_scan();

        /* 배선 순서 때문에 ADC 채널을 역순으로 읽는다 */
        for (i = 0; i < IR_NUM; i++)
            ir_ch_update(&ir[i],
                         adc_read((uint8_t)(IR_ADC_START + (IR_NUM - 1 - i))),
                         (state == ST_CALIB) ? cal_set : CAL_NONE);

        /* SW3 : 디버그 모드 토글 */
        if (btn3_scan()) {
            motor_stop_all();
            if (state == ST_DEBUG) {
                state = ST_IDLE;
                lcd_clear();
                lcd_set_cursor(0, 0);
                lcd_print("Line tracer     ");
                lcd_set_cursor(0, 1);
                lcd_print("SW4:run  SW3:dbg");
            } else {
                state = ST_DEBUG;
                tick  = DEBUG_LCD_MS;
                lcd_clear();
            }
        }

        /* SW1 : 흰색 캘리브레이션 시작 / 저장 (토글) */
        if (edge & BTN1_BIT) {
            motor_stop_all();
            led_all_off();

            if (state == ST_CALIB && cal_set == CAL_WHITE) {
                /* 두 번째 누름 : 저장하고 대기로 */
                calib_save_to_eeprom();
                state     = ST_IDLE;
                g_cal_set = CAL_WHITE;
                lcd_clear();
                lcd_set_cursor(0, 0);
                lcd_print("CAL W  saved    ");
                lcd_set_cursor(0, 1);
                lcd_print("SW4:run  SW3:dbg");
            } else {
                /* 첫 번째 누름 : 캘리브레이션 시작 */
                for (i = 0; i < IR_NUM; i++) ir_ch_reset_range(&ir[i], CAL_WHITE);
                state   = ST_CALIB;
                cal_set = CAL_WHITE;
                tick    = LCD_MS;
                run_state_reset();
                g_cal_set = CAL_WHITE;      /* run_state_reset 뒤에 */
                lcd_clear();
            }
        }

        /* SW2 : 검은색 캘리브레이션 시작 / 저장 (토글) */
        if (edge & BTN2_BIT) {
            motor_stop_all();
            led_all_off();

            if (state == ST_CALIB && cal_set == CAL_BLACK) {
                calib_save_to_eeprom();
                state     = ST_IDLE;
                g_cal_set = CAL_WHITE;
                lcd_clear();
                lcd_set_cursor(0, 0);
                lcd_print("CAL B  saved    ");
                lcd_set_cursor(0, 1);
                lcd_print("SW4:run  SW3:dbg");
            } else {
                for (i = 0; i < IR_NUM; i++) ir_ch_reset_range(&ir[i], CAL_BLACK);
                state   = ST_CALIB;
                cal_set = CAL_BLACK;
                tick    = LCD_MS;
                run_state_reset();
                g_cal_set = CAL_BLACK;      /* 검은 세트로 표시 */
                lcd_clear();
            }
        }

        /* SW4 : 주행 시작 */
        if (btn4_scan()) {
            if (state == ST_CALIB) calib_save_to_eeprom();
            line_follow_reset();
            led_all_off();
            state = ST_RUN;
            run_state_reset();
#if TEST_SKIP_TO_PARK_EXIT
            skip_to_after_park();
#endif
            lcd_clear();
            print_lcd_count();
            lcd_set_cursor(0, 1);
            lcd_print("SW1/2: calib    ");
        }

        if (state == ST_RUN) {
            led_update();
            cross_count_check();

            /* 8자 구간에서만 접점 예고 / 두 선 판정을 쓴다 */
            if (g_section != SEC_LANE) {
                cross_approach_check();
                two_line_check();
            }
            line_follow_update();
        }

        if (state == ST_DONE) {
            motor_stop_all();
        }

        /* 디버그 모드 : 모터는 세워 두고 표시만 갱신 */
        if (state == ST_DEBUG) {
	        led_update();
	        motor_stop_all();
	        psd_measure();
	        psd_measure_right();

	        tick = (uint16_t)(tick + SAMPLE_MS);
            if (tick >= DEBUG_LCD_MS) {
                tick = 0;
                print_lcd_debug();
            }
        }

        /* 주행 중에는 I2C 가 루프를 막으므로 갱신하지 않는다 */
        if (state == ST_CALIB) {
            tick = (uint16_t)(tick + SAMPLE_MS);
            if (tick >= LCD_MS) {
                tick = 0;
                print_lcd_calib(cal_set);
            }
        }

        _delay_ms(SAMPLE_MS);
    }
    return 0;
}