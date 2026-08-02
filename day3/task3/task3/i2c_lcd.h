#ifndef I2C_LCD_H_
#define I2C_LCD_H_

#include <avr/io.h>

// PCF8574 백팩 I2C 주소 (0x27 또는 0x3F 중 본인 보드에 맞게 수정)
#define LCD_ADDR 0x27

void lcd_init(void);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_print(const char *str);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_clear(void);

#endif /* I2C_LCD_H_ */