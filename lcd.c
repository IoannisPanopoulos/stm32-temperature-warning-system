#include "lcd.h"
#include "sensor.h"

// SPI + LCD shift register
DigitalOut CS(D10);
SPI ser_port(D11, D12, D13);

// Buzzer and LEDs
PwmOut buzzer(D9);
DigitalOut led_high(D6);
DigitalOut led_low(D7);

// ================= LCD FUNCTIONS =================
void shift_out(int data) { CS=0; ser_port.write(data); CS=1; }
void write_4bit(int data,int mode) { shift_out((data&0xF0)|ENABLE|mode); wait_us(1); shift_out((data&0xF0)&~ENABLE); shift_out(((data<<4)&0xF0)|ENABLE|mode); wait_us(1); shift_out(((data<<4)&0xF0)&~ENABLE); }
void write_cmd(int cmd) { write_4bit(cmd, COMMAND_MODE); }
void write_data(char c) { write_4bit(c, DATA_MODE); }

void clr_lcd(void) { write_cmd(0x01); wait_us(1520); }
void print_lcd(const char *str) { while(*str) write_data(*str++); }

void init_lcd(void) {
    thread_sleep_for(40);
    shift_out(0x30); wait_us(37);
    write_cmd(0x20); wait_us(37);
    write_cmd(0x20); wait_us(37);
    write_cmd(0x0C); wait_us(37);
    write_cmd(0x01); wait_us(1520);
    write_cmd(0x06); wait_us(37);
    write_cmd(0x28); wait_us(37);
}

// ================= THREAD TASK =================
void lcd_task(void) {
    float *temp_ptr;
    while(true){
        osEvent evt = temp_queue.get();
        if(evt.status==osEventMessage){
            temp_ptr = (float*)evt.value.p;
            current_temp = *temp_ptr;
            update_lcd();
        }
    }
}

// ================= ALARMS =================
void beep_with_led(float freq,int dur,DigitalOut &led){if(freq<=0) return; buzzer.period(1.0f/freq); buzzer.write(0.5f); led=1; thread_sleep_for(dur); buzzer.write(0.0f); led=0;}
void high_alarm_pattern(){for(int i=0;i<3;i++){beep_with_led(2000.0f,150,led_high); thread_sleep_for(100);}}
void low_alarm_pattern(){for(int i=0;i<2;i++){beep_with_led(800.0f,200,led_low); thread_sleep_for(150);}}

// ================= UPDATE LCD =================
void format_temp(float t,char *buf){int w=(int)t;int f=(int)((t-w)*10.0f);if(f<0)f=-f; snprintf(buf,6,"%2d.%1d",w,f);}
void update_lcd(void){
    char cur[6],max[6],min[6],low[6],high[6];
    format_temp(current_temp,cur);
    format_temp(max_temp,max);
    format_temp(min_temp,min);
    format_temp(lower_threshold,low);
    format_temp(upper_threshold,high);

    clr_lcd();
    char line[21];
    snprintf(line,sizeof(line),"Cur:%sC U:%s",cur,high); write_cmd(LCD_LINE1); print_lcd(line);
    snprintf(line,sizeof(line),"Max:%sC L:%s",max,low); write_cmd(LCD_LINE2); print_lcd(line);
    snprintf(line,sizeof(line),"Min:%sC",min); write_cmd(LCD_LINE3); print_lcd(line);
}
