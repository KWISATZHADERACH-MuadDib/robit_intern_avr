#ifndef I2C_H_
#define I2C_H_

#include <avr/io.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define I2C_SCL_CLOCK 100000UL  // 100kHz

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
uint8_t i2c_write(uint8_t data);

#endif /* I2C_H_ */