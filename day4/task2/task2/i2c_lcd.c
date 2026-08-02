#include "i2c_lcd.h"
#include "i2c.h"
#include <util/delay.h>

// PCF8574 <-> LCD 핀 매핑 (일반적인 백팩 기준)
// P0=RS, P1=RW, P2=EN, P3=Backlight, P4~P7=D4~D7
#define RS_BIT  0x01
#define RW_BIT  0x02
#define EN_BIT  0x04
#define BL_BIT  0x08  // 백라이트 항상 ON

static void i2c_lcd_write_byte(uint8_t data)
{
    i2c_start();
    i2c_write((LCD_ADDR << 1) | 0); // Write 모드
    i2c_write(data);
    i2c_stop();
}

// 4bit 데이터(상위 니블 또는 하위 니블)를 EN pulse와 함께 전송
static void lcd_pulse_nibble(uint8_t nibble_with_flags)
{
    // EN High
    i2c_lcd_write_byte(nibble_with_flags | EN_BIT);
    _delay_us(1);
    // EN Low (데이터 래치)
    i2c_lcd_write_byte(nibble_with_flags & ~EN_BIT);
    _delay_us(50);
}

static void lcd_send(uint8_t value, uint8_t rs_flag)
{
    uint8_t high_nibble = value & 0xF0;
    uint8_t low_nibble  = (value << 4) & 0xF0;

    uint8_t flags = BL_BIT | (rs_flag ? RS_BIT : 0);

    lcd_pulse_nibble(high_nibble | flags);
    lcd_pulse_nibble(low_nibble | flags);
}

void lcd_send_cmd(uint8_t cmd)
{
    lcd_send(cmd, 0);
    if (cmd == 0x01 || cmd == 0x02) {
        _delay_ms(2); // Clear/Home 명령은 시간이 더 필요
    }
}

void lcd_send_data(uint8_t data)
{
    lcd_send(data, 1);
}

void lcd_print(const char *str)
{
    while (*str) {
        lcd_send_data((uint8_t)*str++);
    }
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01);
}

void lcd_set_cursor(uint8_t col, uint8_t row)
{
    // 16x2 기준 주소 오프셋 (20x4는 0x00,0x40,0x14,0x54 로 조정)
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    lcd_send_cmd(0x80 | (col + row_offsets[row]));
}

void lcd_init(void)
{
    i2c_init();
    _delay_ms(50); // 전원 인가 후 안정화 대기

    // HD44780 4bit 모드 초기화 시퀀스
    lcd_pulse_nibble(0x30 | BL_BIT);
    _delay_ms(5);
    lcd_pulse_nibble(0x30 | BL_BIT);
    _delay_us(150);
    lcd_pulse_nibble(0x30 | BL_BIT);
    _delay_us(150);
    lcd_pulse_nibble(0x20 | BL_BIT); // 4bit 모드로 전환
    _delay_us(150);

    lcd_send_cmd(0x28); // Function set: 4bit, 2line, 5x8 dot
    lcd_send_cmd(0x0C); // Display ON, Cursor OFF, Blink OFF
    lcd_send_cmd(0x06); // Entry mode: 커서 자동 증가
    lcd_send_cmd(0x01); // Clear display
    _delay_ms(2);
}
