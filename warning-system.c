/*
 * STM32 Temperature Warning System
 *
 * Features:
 * - I2C temperature sensor (DS1631)
 * - User-defined upper/lower thresholds
 * - Buzzer alarm with different patterns
 * - LCD display via SPI shift register
 * - RTOS-based architecture (threads, queues, event flags)
 * - 1-hour min/max temperature tracking
 *
 * Author: Panopoulos Ioannis
 */

#include "mbed.h"

#define ENABLE 0x08
#define COMMAND_MODE 0x00
#define DATA_MODE 0x04
#define LCD_LINE1 0x80
#define LCD_LINE2 0xC0
#define LCD_LINE3 0x94
#define ALARM_HIGH 0x01
#define ALARM_LOW  0x02
// SPI + LCD shift register
DigitalOut CS(D10);
SPI ser_port(D11, D12, D13);

// I2C temperature sensor (DS1631)
I2C  temp_sensor(I2C_SDA, I2C_SCL);
const int  temp_addr = 0x90;
const char CMD_START = 0x51;
const char CMD_READ = 0xAA;
char temp_raw[2];

// Buzzer (PWM)
PwmOut buzzer(D9);

// LEDs
DigitalOut led_high(D6);
DigitalOut led_low(D7);

// Buttons
InterruptIn btn_L_down(D2);
InterruptIn btn_L_up  (D3);
InterruptIn btn_U_down(D4);
InterruptIn btn_U_up  (D5);

// Timer
Timer hour_timer;

volatile float lower_threshold = 18.0f;
volatile float upper_threshold = 30.0f;
float current_temp = 0.0f;
float max_temp = -1000.0f;
float min_temp =  1000.0f;
bool  stats_init = false;

const float THRESH_STEP = 1.0f;

// RTOS objects
Thread sensor_thread;
Thread alarm_thread;
Thread lcd_thread;

Queue<float, 4> temp_queue;
EventFlags alarm_flags;


// Prototypes
void init_lcd(void);
void clr_lcd(void);
void print_lcd(const char *string);
void shift_out(int data);
void write_cmd(int cmd);
void write_data(char c);
void write_4bit(int data, int mode);

void start_conversion(void);
float read_temperature(void);
void reset_hour_stats(float temp);
void update_max_min(float temp);

void beep_with_led(float freq_hz, int duration_ms, DigitalOut &led);
void high_alarm_pattern();
void low_alarm_pattern();

void format_temp(float t, char *buf);
void update_lcd(void);

void isr_lower_down();
void isr_lower_up();
void isr_upper_down();
void isr_upper_up();

// ================= THREADS =================

void sensor_task() {
    start_conversion();
    thread_sleep_for(500);

    hour_timer.reset();
    hour_timer.start();

    while (true) {
        current_temp = read_temperature();

        if (!stats_init) {
            reset_hour_stats(current_temp);
            stats_init = true;
        } else {
            update_max_min(current_temp);
        }

        temp_queue.put(&current_temp);

        if (current_temp > upper_threshold) {
            alarm_flags.set(ALARM_HIGH);
        } else if (current_temp < lower_threshold) {
            alarm_flags.set(ALARM_LOW);
        }

        start_conversion();

        float elapsed_sec = hour_timer.read();
        if (elapsed_sec >= 3600.0f) {
            reset_hour_stats(current_temp);
        }

        thread_sleep_for(1000);
    }
}

void alarm_task() {
    while (true) {
        uint32_t flags = alarm_flags.wait_any(ALARM_HIGH | ALARM_LOW);

        if (flags & ALARM_HIGH) {
            high_alarm_pattern();
        }
        if (flags & ALARM_LOW) {
            low_alarm_pattern();
        }

        buzzer.write(0.0f);
        led_high = 0;
        led_low = 0;
    }
}

void lcd_task() {
    float *temp_ptr;

    while (true) {
        osEvent evt = temp_queue.get();

        if (evt.status == osEventMessage) {
            temp_ptr = (float*)evt.value.p;
            current_temp = *temp_ptr;
            update_lcd();
        }
    }
}

// ================= MAIN =================

int main() {
    ser_port.format(8, 0);
    ser_port.frequency(1000000);
    CS = 1;

    led_high = 0;
    led_low  = 0;

    init_lcd();
    clr_lcd();
    print_lcd("Temp Warning Sys");
    write_cmd(LCD_LINE2);
    print_lcd("Welcome :)");
    thread_sleep_for(5000);

    btn_L_down.mode(PullDown);
    btn_L_up.mode(PullDown);
    btn_U_down.mode(PullDown);
    btn_U_up.mode(PullDown);

    btn_L_down.rise(&isr_lower_down);
    btn_L_up.rise(&isr_lower_up);
    btn_U_down.rise(&isr_upper_down);
    btn_U_up.rise(&isr_upper_up);

    sensor_thread.start(sensor_task);
    alarm_thread.start(alarm_task);
    lcd_thread.start(lcd_task);

    while (true) {
        thread_sleep_for(osWaitForever);
    }
}



void shift_out(int data) {
    CS = 0;
    ser_port.write(data);
    CS = 1;
}

void write_4bit(int data, int mode) {
    int hi_n = (data & 0xF0);
    int lo_n = ((data << 4) & 0xF0);

    // High nibble
    shift_out(hi_n | ENABLE | mode);
    wait_us(1);
    shift_out(hi_n & ~ENABLE);

    // Low nibble
    shift_out(lo_n | ENABLE | mode);
    wait_us(1);
    shift_out(lo_n & ~ENABLE);
}

void write_cmd(int cmd) {
    write_4bit(cmd, COMMAND_MODE);
}

void write_data(char c) {
    write_4bit(c, DATA_MODE);
}

void clr_lcd(void) {
    write_cmd(0x01);
    wait_us(1520);
}

void print_lcd(const char *string) {
    while (*string) {
        write_data(*string++);
        wait_us(40);
    }
}

void init_lcd(void) {
    thread_sleep_for(40);

    shift_out(0x30);
    wait_us(37);
    write_cmd(0x20);
    wait_us(37);
    write_cmd(0x20);
    wait_us(37);
    write_cmd(0x0C);
    wait_us(37);
    write_cmd(0x01);
    wait_us(1520);
    write_cmd(0x06);
    wait_us(37);
    write_cmd(0x28);
    wait_us(37);
}


// =======================
// DS1631 Temperature via I2C
// =======================

void start_conversion(void) {
    char cmd = CMD_START;
    temp_sensor.write(temp_addr, &cmd, 1, false);
}

float read_temperature(void) {
    char cmd = CMD_READ;
    temp_sensor.write(temp_addr, &cmd, 1, false);
    temp_sensor.read(temp_addr, temp_raw, 2);

    int16_t raw = (int16_t)((temp_raw[0] << 8) | (temp_raw[1] & 0xFF));
    float temp_c = raw / 256.0f;
    return temp_c;
}


// =======================
// Min/Max tracking (last hour)
// =======================

void reset_hour_stats(float temp) {
    max_temp = temp;
    min_temp = temp;
    hour_timer.reset();
    hour_timer.start();
}

void update_max_min(float temp) {
    if (temp > max_temp) max_temp = temp;
    if (temp < min_temp) min_temp = temp;
}


// =======================
// Buzzer + LED alarm patterns
// =======================

// Single beep with matching LED flash
void beep_with_led(float freq_hz, int duration_ms, DigitalOut &led) {
    if (freq_hz <= 0.0f) return;

    buzzer.period(1.0f / freq_hz);
    buzzer.write(0.5f);
    led = 1;                            // LED ON with beep
    thread_sleep_for(duration_ms);
    buzzer.write(0.0f);
    led = 0;                            // LED OFF between beeps
}

// Triple high beeps for HIGH temp alarm
void high_alarm_pattern() {
    for (int i = 0; i < 3; i++) {
        beep_with_led(2000.0f, 150, led_high);
        thread_sleep_for(100);          // pause between beeps
    }
}

// Double low beeps for LOW temp alarm
void low_alarm_pattern() {
    for (int i = 0; i < 2; i++) {
        beep_with_led(800.0f, 200, led_low);
        thread_sleep_for(150);          // pause between beeps
    }
}

// Decide which pattern to play
void check_thresholds_and_warn(float temp) {
    float low  = lower_threshold;
    float high = upper_threshold;

    if (temp > high) {
        high_alarm_pattern();
    } else if (temp < low) {
        low_alarm_pattern();
    } else {
        // Normal range: ensure outputs are off
        buzzer.write(0.0f);
        led_high = 0;
        led_low  = 0;
    }
}


// =======================
// LCD display helpers
// =======================

void format_temp(float t, char *buf) {
    int whole = (int)t;
    int frac  = (int)((t - whole) * 10.0f);
    if (frac < 0) frac = -frac;
    snprintf(buf, 6, "%2d.%1d", whole, frac);
}

// Line 1: Cur:xx.xC U:uu.u
// Line 2: Max:xx.xC L:ll.l
// Line 3: Min:xx.xC
void update_lcd(void) {
    char cur_buf[6], max_buf[6], min_buf[6];
    format_temp(current_temp, cur_buf);
    format_temp(max_temp,     max_buf);
    format_temp(min_temp,     min_buf);

    float low  = lower_threshold;
    float high = upper_threshold;

    char low_buf[6], high_buf[6];
    format_temp(low,  low_buf);
    format_temp(high, high_buf);

    clr_lcd();

    // Line 1 – current + upper
    write_cmd(LCD_LINE1);
    wait_us(40);
    char line1[21];
    snprintf(line1, sizeof(line1), "Cur:%sC U:%s", cur_buf, high_buf);
    print_lcd(line1);

    // Line 2 – max + lower
    write_cmd(LCD_LINE2);
    wait_us(40);
    char line2[21];
    snprintf(line2, sizeof(line2), "Max:%sC L:%s", max_buf, low_buf);
    print_lcd(line2);

    // Line 3 – min
    write_cmd(LCD_LINE3);
    wait_us(40);
    char line3[21];
    snprintf(line3, sizeof(line3), "Min:%sC", min_buf);
    print_lcd(line3);
}


// =======================
// Button ISRs for thresholds
// =======================

void isr_lower_down() {
    float new_low = lower_threshold - THRESH_STEP;
    if (new_low < -40.0f) new_low = -40.0f;
    lower_threshold = new_low;

    if (upper_threshold < lower_threshold) {
        upper_threshold = lower_threshold;
    }
}

void isr_lower_up() {
    float new_low = lower_threshold + THRESH_STEP;
    lower_threshold = new_low;

    if (upper_threshold < lower_threshold) {
        upper_threshold = lower_threshold;
    }
}

void isr_upper_down() {
    float new_high = upper_threshold - THRESH_STEP;
    if (new_high < lower_threshold) {
        new_high = lower_threshold;
    }
    upper_threshold = new_high;
}

void isr_upper_up() {
    float new_high = upper_threshold + THRESH_STEP;
    if (new_high > 125.0f) new_high = 125.0f;
    upper_threshold = new_high;
}
