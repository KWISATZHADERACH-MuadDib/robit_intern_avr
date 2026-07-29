#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "i2c_lcd.h"

int main(void)
{
    lcd_init();

    lcd_set_cursor(0, 0);
    lcd_print("Hello, World!");

    lcd_set_cursor(0, 1);
    lcd_print("ATmega I2C LCD");

    while (1)
    {
        // 필요한 동작 추가
    }
}
