#include "i2c.h"

void i2c_init(void)
{
    // TWBR 계산: SCL = F_CPU / (16 + 2*TWBR*Prescaler)
    TWSR = 0x00; // Prescaler = 1
    TWBR = ((F_CPU / I2C_SCL_CLOCK) - 16) / 2;
    TWCR = (1 << TWEN); // TWI 활성화
}

void i2c_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

uint8_t i2c_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return (TWSR & 0xF8); // 상태 코드 반환
}
