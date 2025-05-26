
#ifndef CONFIG_H
#define CONFIG_H

/* *Total definations for LCD1602
* LCD RS pin to digital pin 12
* LCD Enable pin to digital pin 11
* LCD D4 pin to digital pin 5
* LCD D5 pin to digital pin 4
* LCD D6 pin to digital pin 3
* LCD D7 pin to digital pin 2
* LCD R/W pin to ground
* LCD VSS pin to ground
* LCD VCC pin to 5V
* */
#define LCD_RS_PIN 12
#define LCD_EN_PIN 11
#define LCD_D4_PIN 5
#define LCD_D5_PIN 4
#define LCD_D6_PIN 3
#define LCD_D7_PIN 2
// LCD R/W pin to ground
// LCD VSS pin to ground
// LCD VCC pin to 5V


// Button Pin Definitions
#define BUTTON_CHOOSE_PIN A0
#define BUTTON_ADD_PIN A1
#define BUTTON_MINUS_PIN A2

// Buzzer Pin Definition
#define BUZZER_TONE_PIN 13

// DS1302 RTC Pin Definitions
#define DS1302_CE_PIN 8   // RST Pin
#define DS1302_IO_PIN 9   // DAT Pin
#define DS1302_SCLK_PIN 10 // CLK Pin

// Temperature Sensor Pin
#define TEMP_SENSOR_PIN A3

// Constants
#define BUTTON_DEBOUNCE_DELAY 200 // ms, for button debouncing
#define BUTTON_LONGTIME_Multiple 7 // Times for long press
#define DEFAULT_ALARM_TEMP 40.0f // Default temperature alarm threshold
#define HOURLY_CHIME_FREQ 2093   // Hz, frequency for hourly chime
#define ALARM_SOUND_FREQ 2093    // Hz, frequency for alarm sound
#define ALARM_DURATION 5000      // ms, duration for custom alarm
#define HOURLY_CHIME_DURATION 500 // ms, duration for hourly chime
#define TEMP_ALARM_DURATION 500   // ms, duration for temperature alarm beep

// ----------------------------------------------------------------------------
// 全局 U8g2 对象 和 软件 I2C 引脚定义
// ----------------------------------------------------------------------------

#define SW_I2C_PIN_SCL 6 // OLED SCL -> Arduino D6 (示例修改)
#define SW_I2C_PIN_SDA 7 // OLED SDA -> Arduino D7 (示例修改)
#define OLED_PIN_RST U8X8_PIN_NONE // 如果不使用 RST 引脚


// 定义音符频率 (Hz)
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988
#define NOTE_REST 0 // 休止符





#endif // CONFIG_H