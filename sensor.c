#include "sensor.h"
#include "alarm.h"

// I2C sensor
I2C temp_sensor(I2C_SDA, I2C_SCL);
const int temp_addr = 0x90;
const char CMD_START = 0x51;
const char CMD_READ  = 0xAA;
char temp_raw[2];

// Timer for 1-hour stats
Timer hour_timer;
const float THRESH_STEP = 1.0f;

volatile float lower_threshold = 18.0f;
volatile float upper_threshold = 30.0f;
float current_temp = 0.0f;
float max_temp = -1000.0f;
float min_temp = 1000.0f;
bool stats_init = false;

// ================= SENSOR TASK =================
void sensor_task(void) {
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

        // Trigger alarms
        if (current_temp > upper_threshold) alarm_flags.set(ALARM_HIGH);
        if (current_temp < lower_threshold) alarm_flags.set(ALARM_LOW);

        start_conversion();
        if (hour_timer.read() >= 3600.0f) reset_hour_stats(current_temp);

        thread_sleep_for(1000);
    }
}

// ================= SENSOR HELPERS =================
void start_conversion(void) {
    char cmd = CMD_START;
    temp_sensor.write(temp_addr, &cmd, 1, false);
}

float read_temperature(void) {
    char cmd = CMD_READ;
    temp_sensor.write(temp_addr, &cmd, 1, false);
    temp_sensor.read(temp_addr, temp_raw, 2);
    int16_t raw = (int16_t)((temp_raw[0] << 8) | (temp_raw[1] & 0xFF));
    return raw / 256.0f;
}

void reset_hour_stats(float temp) { max_temp = min_temp = temp; hour_timer.reset(); hour_timer.start(); }
void update_max_min(float temp) { if (temp > max_temp) max_temp = temp; if (temp < min_temp) min_temp = temp; }

// ================= BUTTON ISRs =================
void configure_buttons(void) {
    btn_L_down.mode(PullDown);
    btn_L_up.mode(PullDown);
    btn_U_down.mode(PullDown);
    btn_U_up.mode(PullDown);

    btn_L_down.rise(&isr_lower_down);
    btn_L_up.rise(&isr_lower_up);
    btn_U_down.rise(&isr_upper_down);
    btn_U_up.rise(&isr_upper_up);
}

void isr_lower_down() { lower_threshold = fmax(lower_threshold - THRESH_STEP, -40.0f); if(upper_threshold<lower_threshold) upper_threshold=lower_threshold; }
void isr_lower_up()   { lower_threshold += THRESH_STEP; if(upper_threshold<lower_threshold) upper_threshold=lower_threshold; }
void isr_upper_down() { upper_threshold = fmax(upper_threshold - THRESH_STEP, lower_threshold); }
void isr_upper_up()   { upper_threshold = fmin(upper_threshold + THRESH_STEP, 125.0f); }
