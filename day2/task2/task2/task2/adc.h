#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>

void adc_init(void);
uint16_t adc_read(uint8_t channel); // channel: 0~7 (ADC0~ADC7)

#endif /* ADC_H_ */
