#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include "hd44780.h"

#define LCD_PORT GPIOB
#define LCD_RCC RCC_GPIOB
#define LCD_RS GPIO10
#define LCD_CLOCK GPIO11
#define LCD_4 GPIO12
#define LCD_5 GPIO13
#define LCD_6 GPIO14
#define LCD_7 GPIO15

// Use _lcd_delay_ms as our default delay function
static void _lcd_delay_ms(uint32_t delay);
void (*lcd_delay_ms)(uint32_t) = _lcd_delay_ms;

// Various displays exist, don't make assumptions
uint8_t lcd_chars = 0;
uint8_t lcd_lines = 0;
uint8_t *lcd_line_addresses = 0;
// "Private" globals
uint8_t _lcd_char = 0;
uint8_t _lcd_line = 0;

static void _lcd_delay_ms(uint32_t delay)
{
    // Quick and ugly delay, inaccurate when microcontroller runs on a
    // different speed, wastes too many cycles if interrupts occured.

    uint32_t cycles = (delay * 72000)/5;  // 8 Mhz, CMP+BEQ+NOP+ADDS+B
    uint32_t i = 0;
    while(i++ < cycles) {
        __asm__("nop");
    }
}

void lcd_clock(void)
{
    // Pulse clock
    gpio_set(LCD_PORT, LCD_CLOCK);
    lcd_delay_ms(1);
    gpio_clear(LCD_PORT, LCD_CLOCK);
    lcd_delay_ms(1);
}

void lcd_setup(void)
{
    rcc_periph_clock_enable(LCD_RCC);
    gpio_set_mode(LCD_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
            LCD_RS | LCD_CLOCK | LCD_4 | LCD_5 | LCD_6 | LCD_7);
}

void lcd_reset(void)
{
    // Resets display from any state to 4-bit mode, first nibble.

    // Set everything low first
    gpio_clear(LCD_PORT, LCD_RS | LCD_CLOCK | LCD_4 | LCD_5 | LCD_6 | LCD_7);

    // Reset strategy below based on Wikipedia description, should recover
    // from any setting

    // Write 0b0011 three times
    // (Everyday Practical Electronics says 3 times, Wikipedia says 2 times,
    // 3 seems to work better).
    gpio_set(LCD_PORT, LCD_5 | LCD_4);
    lcd_clock();
    lcd_clock();
    lcd_clock();
    // LCD now guaranteed to be in 8-bit state

    // Now write 0b0010 (set to 4-bit mode, ready for first nibble)
    gpio_clear(LCD_PORT, LCD_4);
    lcd_clock();

    lcd_clock();
    gpio_clear(LCD_PORT, LCD_4);
    gpio_set(LCD_PORT, LCD_7);
    lcd_clock();

}

/* TODO This function should achieve the same as the lcd_write below, however
 * it appears to be a little problematic.
 * Rather than the LCD_4 and LCD_RS defines, direct integers have to be used
 * for proper masks to be calculated.
 * Aside from this, setting the RS bit seems to go wrong.
void lcd_write(uint8_t byte, uint8_t rs)
{
    // Writes a byte to the display (rs must be either 0 or 1)

    // Retrieve current port state
    uint16_t port_state = gpio_port_read(LCD_PORT);

    // Write second nibble and register select;
    port_state &= ~((0b1111 << 12) | 10);
    port_state |= ((((byte >> 4) & 0b1111) << 12) | (rs ? 10 : 0));
    gpio_port_write(LCD_PORT, port_state);
    lcd_clock();

    // Write first nibble
    port_state &= ~(0b1111 << 12);
    port_state |= ((byte & 0b1111) << 12);
    gpio_port_write(LCD_PORT, port_state);
    lcd_clock();
}
*/

void lcd_write(uint8_t byte, uint8_t rs)
{
    // Writes a byte to the display (rs must be either 0 or 1)

    // Write second nibble and set RS

    if((byte >> 4 ) & 1)
        gpio_set(LCD_PORT, LCD_4);
    else
        gpio_clear(LCD_PORT, LCD_4);

    if((byte >> 5 ) & 1)
        gpio_set(LCD_PORT, LCD_5);
    else
        gpio_clear(LCD_PORT, LCD_5);

    if((byte >> 6 ) & 1)
        gpio_set(LCD_PORT, LCD_6);
    else
        gpio_clear(LCD_PORT, LCD_6);

    if((byte >> 7 ) & 1)
        gpio_set(LCD_PORT, LCD_7);
    else
        gpio_clear(LCD_PORT, LCD_7);

    if(rs)
        gpio_set(LCD_PORT, LCD_RS);
    else
        gpio_clear(LCD_PORT, LCD_RS);

    lcd_clock();

    // Write first nibble

    if(byte & 1)
        gpio_set(LCD_PORT, LCD_4);
    else
        gpio_clear(LCD_PORT, LCD_4);

    if((byte >> 1 ) & 1)
        gpio_set(LCD_PORT, LCD_5);
    else
        gpio_clear(LCD_PORT, LCD_5);

    if((byte >> 2 ) & 1)
        gpio_set(LCD_PORT, LCD_6);
    else
        gpio_clear(LCD_PORT, LCD_6);

    if((byte >> 3 ) & 1)
        gpio_set(LCD_PORT, LCD_7);
    else
        gpio_clear(LCD_PORT, LCD_7);

    lcd_clock();
}

void lcd_clear(void)
{
    // Clears display, resets cursor
    lcd_write(0b00000001, 0);
    _lcd_char = 0;
    _lcd_line = 0;
}

void lcd_display_settings(uint8_t on, uint8_t underline, uint8_t blink)
{
    // "Display On/Off & Cursor" command. All parameters must be either 0 or 1

    lcd_write(0b00001000 | (on << 2) | (underline << 1) | blink, 0);
}

void lcd_display_address(uint8_t address)
{
    lcd_write(0b10000000 | address, 0);
}

void lcd_print(char string[])
{
    uint8_t i;
    for(i = 0; string[i] != 0; i++) {
        // If we know the display properties and a newline character is
        // present, print the rest of the string on the new line.
        if(lcd_lines && string[i] == '\n') {
            if(_lcd_line < lcd_lines) {
                lcd_display_address(lcd_line_addresses[_lcd_line++]);
                _lcd_char = 0;
            }
        }
        else {
            // If we know the display properties and have reached the end of
            // line, print the rest on the next line
            if(lcd_chars)
                if((_lcd_char == lcd_chars) && (_lcd_line < lcd_lines)) {
                    lcd_display_address(lcd_line_addresses[_lcd_line++]);
                    _lcd_char = 0;
                }
            lcd_write(string[i], 1);
            if(lcd_chars) _lcd_char++;
        }
    }
}
