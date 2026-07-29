#include "adc.h"

void adc_init(void)
{
    // 기준전압: AVCC (REFS0=1, REFS1=0)
    ADMUX = (1 << REFS0);

    // ADC 활성화(ADEN) + 분주비 128 (ADPS2,1,0 = 1,1,1)
    // 16MHz / 128 = 125kHz (ADC 권장 범위 50~200kHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel)
{
    // 채널 선택 (MUX0~4), 상위 REFS 비트는 유지
    ADMUX = (ADMUX & 0xE0) | (channel & 0x1F);

    ADCSRA |= (1 << ADSC);        // 변환 시작
    while (ADCSRA & (1 << ADSC)); // 변환 완료까지 대기

    return ADC; // ADCL, ADCH 합쳐진 10bit 값 (0~1023)
}
