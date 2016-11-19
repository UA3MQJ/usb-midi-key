#ifndef HD44780_H
#define HD44780_H

#include <stdint.h>

// It is recommended to supply your own delay function to the lcd_delay_ms
// function pointer.
extern void (*lcd_delay_ms)(uint32_t);

// Allows for smarter functionality if set
extern uint8_t lcd_chars;
extern uint8_t lcd_lines;
extern uint8_t *lcd_line_addresses;

void lcd_clock(void);
void lcd_setup(void);
void lcd_reset(void);
void lcd_write(uint8_t byte, uint8_t rs);
void lcd_clear(void);
void lcd_display_settings(uint8_t on, uint8_t underline, uint8_t blink);
void lcd_display_address(uint8_t address);
void lcd_print(char string[]);

#endif  // HD44780_H
