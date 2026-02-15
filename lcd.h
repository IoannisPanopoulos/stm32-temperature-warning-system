#ifndef LCD_H
#define LCD_H

#include "mbed.h"

// LCD defines
#define ENABLE      0x08
#define COMMAND_MODE 0x00
#define DATA_MODE    0x04
#define LCD_LINE1    0x80
#define LCD_LINE2    0xC0
#define LCD_LINE3    0x94

// Alarm flags
#define ALARM_HIGH 0x01
#define ALARM_LOW  0x02

extern DigitalOut CS;
extern SPI ser_port;
extern PwmOut buzzer;
extern DigitalOut led_high;
extern DigitalOut led_low;

void init_lcd(void);
void clr_lcd(void);
void print_lcd(const char *string);
void lcd_task(void);
void high_alarm_pattern(void);
void low_alarm_pattern(void);
void update_lcd(void);

#endif
