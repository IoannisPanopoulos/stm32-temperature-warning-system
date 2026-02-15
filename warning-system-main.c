/**
 * main.c
 * STM32 Temperature Warning System
 *
 * Entry point for the system. Initializes hardware, threads, and queues.
 *
 * Author: Panopoulos Ioannis
 */

#include "mbed.h"
#include "lcd.h"
#include "sensor.h"
#include "alarm.h"

// RTOS objects
Thread sensor_thread;
Thread alarm_thread;
Thread lcd_thread;

Queue<float, 4> temp_queue;
EventFlags alarm_flags;

int main() {
    // Initialize LCD
    init_lcd();
    clr_lcd();
    print_lcd("Temp Warning Sys");
    write_cmd(LCD_LINE2);
    print_lcd("Welcome :)");
    thread_sleep_for(2000);

    // Configure buttons
    configure_buttons();

    // Start threads
    sensor_thread.start(sensor_task);
    alarm_thread.start(alarm_task);
    lcd_thread.start(lcd_task);

    while (true) {
        thread_sleep_for(osWaitForever);
    }
}
