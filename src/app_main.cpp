#define MAIN
#ifdef MAIN
#include <Arduino.h>
#include <DS1302.h>
#include <LiquidCrystal.h> // LCD1602 显示头文件
#include <U8g2lib.h> // 引入 U8g2 库
#include "Config.h"      // 配置文件, 包含所有 #define

#define __Serial_DEBUG___ // 定义调试模式，启用串口输出
#define __OLED__
#define __KalmanConfig___ // 定义卡尔曼滤波器配置模式，不启用用串口输出
#define __MEMORY_EFFICIENT__ // 定义高效格式化模式，使用更少的RAM
#define MUSIC_PLAY
#define __KalmanOUTPUT__ // 定义卡尔曼滤波器输出模式，启用串口输出
// LCD 初始化
LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

// DS1302 RTC 对象
DS1302 rtc(DS1302_CE_PIN, DS1302_IO_PIN, DS1302_SCLK_PIN);

#ifdef __OLED__

// --- 修改U8G2构造函数以使用页缓冲模式 (RAM占用更小) ---
// 从 U8G2_SSD1306_128X64_NONAME_F_SW_I2C (全缓冲)
// 改为 U8G2_SSD1306_128X64_NONAME_1_SW_I2C (页缓冲, 1/8屏幕高度的RAM)
// 这将显著减少RAM使用量。_2_SW_I2C 使用两倍于_1_的RAM，但可能稍快。
U8G2_SSD1306_128X64_NONAME_2_SW_I2C u8g2( // <--- 主要修改点在这里
    U8G2_R0,            // 旋转: 无旋转
    SW_I2C_PIN_SCL,     // SCL 引脚
    SW_I2C_PIN_SDA,     // SDA 引脚
    OLED_PIN_RST        // Reset 引脚
);
#endif
// 时间日期结构体
struct DateTime {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int dayOfWeek; // 1 (Mon) to 7 (Sun)
};
DateTime currentTime; // 当前时间日期 (从RTC读取)




#ifdef MUSIC_PLAY
// 定义蜂鸣器连接的引脚
const int buzzerPin = BUZZER_TONE_PIN;
// 定义按键连接的引脚
const int stopButtonPin = BUTTON_ADD_PIN;
enum class MusicMode { NORMAL, MUSIC };
MusicMode currentMusicMode = MusicMode::NORMAL; // 当前音乐模式

// 乐谱数据 (PROGMEM)
// ...《小星星》的乐谱 ...
const int twinkleMelody[] PROGMEM = {
    NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
    NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4,
    NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4,
    NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4,
    NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
    NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4};

const int twinkleDurations[] PROGMEM = {
    500, 500, 500, 500, 500, 500, 1000,
    500, 500, 500, 500, 500, 500, 1000,
    500, 500, 500, 500, 500, 500, 1000,
    500, 500, 500, 500, 500, 500, 1000,
    500, 500, 500, 500, 500, 500, 1000,
    500, 500, 500, 500, 500, 500, 1000};

const int twinkleSongLength = sizeof(twinkleMelody) / sizeof(int);

// 全局变量，用于标记音乐是否应被中止
volatile bool musicShouldStop = false; // 使用 volatile 是因为可能在中断中使用（虽然本例不是）
                                       // 但对于在主循环和函数间共享的、可能意外改变的标志，这是个好习惯
#endif



// 闹钟设置
int alarm_hour = 11;
int alarm_minute = 13;
float temperatureAlarmThreshold = DEFAULT_ALARM_TEMP;

// 系统状态
enum class SystemMode { NORMAL, SET_TIME, SET_ALARM, STOPWATCH, COUNTDOWN_SET, COUNTDOWN_RUNNING };
SystemMode currentMode = SystemMode::NORMAL;
int settingStep = 0; // 设置步骤 (例如：0-小时, 1-分钟 ...)

// 温度相关
float currentTemperature_filtered = 0.0f;

// 卡尔曼滤波器参数
float kalman_x_hat = 0;
float kalman_P = 1.0;
float kalman_Q = 0.0001; // 过程噪声协方差 - 可调
float kalman_R = 0.09;   // 测量噪声协方差 - 可调

// 非阻塞控制相关变量
unsigned long button_last_press_time_choose = 0;
unsigned long button_last_press_time_addminus = 0;
unsigned long button_last_press_time_add = 0;
unsigned long button_last_press_time_add_long = 0;
unsigned long button_last_press_time_minus_long = 0;
unsigned long button_last_press_time_minus = 0;
unsigned long setting_mode_entry_time = 0;
bool buttonADDPressedLong = false;
bool buttonMINUSPressedLong = false;

unsigned long hourly_chime_last_triggered_hour = 25; // 用于整点报时 (初始值确保第一次能响)
unsigned long clock_alarm_sound_start_time = 0;
bool clock_alarm_active = false;
unsigned long temp_alarm_last_beep_time = 0;

// 秒表 (正向计时器) 状态变量
bool stopwatch_running;
unsigned long stopwatch_start_millis; // 记录秒表开始或从暂停继续时的 millis()
unsigned long stopwatch_elapsed_at_pause; // 记录暂停时的已用时间

// 倒计时器状态变量
int countdown_set_hours;
int countdown_set_minutes;
int countdown_set_seconds;
unsigned long countdown_total_set_seconds; // 总设置的倒计时秒数
unsigned long countdown_target_millis;     // 倒计时结束的目标 millis()
bool countdown_running;
bool countdown_beeping; // 是否正在蜂鸣提示
unsigned long countdown_beep_start_millis;
int countdown_setting_step; // 0:H, 1:M, 2:S

// ----------------------------------------------------------------------------
// 时钟参数 
// ----------------------------------------------------------------------------
const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;
const int CLOCK_CENTER_X = SCREEN_WIDTH / 2 -1 ;
const int CLOCK_CENTER_Y = SCREEN_HEIGHT / 2 -1;
const int CLOCK_RADIUS = SCREEN_HEIGHT / 2 - 4;

const int HOUR_HAND_LENGTH = CLOCK_RADIUS * 0.55;
const int MINUTE_HAND_LENGTH = CLOCK_RADIUS * 0.75;
const int SECOND_HAND_LENGTH = CLOCK_RADIUS * 0.85;



// ==== 函数声明 ====
void readAndUpdateTimeFromRTC();
void calculateDayOfWeek(DateTime& dt);
String getDayOfWeekString(int dow);
int daysInMonth(int year, int month); // 返回指定年月的天数

void displayTimeScreen();
void displaySetTimeScreen(const DateTime& tempTime, int step);
void displayStopwatchScreen(unsigned long elapsed_ms, bool is_running);
void displayCountdownSetScreen(int h, int m, int s, int currentSettingStep) ;
void displayCountdownRunningScreen(unsigned long remaining_s, bool is_active, bool is_beeping);
void displaySetAlarmScreen(int tempHour, int tempMin, float tempAlarmTemp, int step);

// OLED显示函数
#ifdef __OLED__ 
void drawClockFaceElements();
void drawClockHands(int h, int m, int s);
void U8G2_Init();
void displayOLEDClock(int hours, int minutes, int seconds);
#endif


void formatNumber(int col, int row, int num, int digits = 2); // 格式化数字输出（前导零）
void printFloat(int col, int row, float val, int decimalPlaces, int totalDigitsBeforeDecimal); // 格式化浮点数输出


void handleNormalMode();
void handleTimeSettingMode();
void handleStopwatchMode();
void handleCountdownSetMode();
void handleCountdownRunningMode();
void handleAlarmSettingMode();
void enterTimeSettingMode();
void enterAlarmSettingMode();
void enterStopwatchMode();
void enterCountdownSetMode();
void enterCountdownRunningMode();

void exitSettingModeAndSave(bool saveTimeToRTC, bool saveAlarm); // 退出设置模式并选择是否保存

void checkAndTriggerHourlyChime();
void checkAndTriggerClockAlarm();
void handleActiveClockAlarmSound(); // 处理正在鸣响的闹钟（非阻塞）
void stopClockAlarmSound();        // 停止闹钟声音
void checkAndTriggerTemperatureAlarm();

void readAndUpdateTemperature();
float applyKalmanFilter(float measurement); // 应用卡尔曼滤波
#ifdef __KalmanConfig__
void handleKalmanSerialConfig();           // 通过串口配置卡尔曼参数
#endif

bool checkButtonPress(int pin, unsigned long& lastPressTime, unsigned long debounceInterval); // 检测按钮单击
bool checkLongButtonPress(int pin, bool& pressStateFlag, unsigned long& pressStartTime, unsigned long thresholdMillis); // 检测长按
bool checkMultiButtonPress(int pin1, int pin2, unsigned long& lastPressTime, unsigned long debounceInterval); // 检测多键组合按下
bool checkButtonHeld(int pin, unsigned long& lastActionTime, unsigned long actionInterval);    // 检测按钮按住状态（用于连续调整）

#ifdef MUSIC_PLAY
void playMusic(const int melody[], const int durations[], int songLength, float tempoFactor = 1.0); // 播放音乐
#endif


void setup() {
    pinMode(LCD_RS_PIN, OUTPUT);
    pinMode(LCD_EN_PIN, OUTPUT);
    pinMode(LCD_D4_PIN, OUTPUT);
    pinMode(LCD_D5_PIN, OUTPUT);
    pinMode(LCD_D6_PIN, OUTPUT);
    pinMode(LCD_D7_PIN, OUTPUT);
    pinMode(BUZZER_TONE_PIN, OUTPUT);

    pinMode(BUTTON_CHOOSE_PIN, INPUT_PULLUP);
    pinMode(BUTTON_ADD_PIN, INPUT_PULLUP);
    pinMode(BUTTON_MINUS_PIN, INPUT_PULLUP);
    
#if defined(__Serial_DEBUG__) || defined(__KalmanOUTPUT__) 
    Serial.begin(9600);
#endif
    lcd.begin(16, 2);

    rtc.writeProtect(false);
    rtc.halt(false);

    Time t = rtc.getTime();
    // 检查RTC时间是否大致有效，如果无效则设置一个默认时间
    if ( t.mon < 1 || t.mon > 12 || t.date < 1 || t.date > 31) { // 更严格的年份下限
        Serial.println("RTC not set or invalid. Setting default time (2025-01-01 12:00:00).");
        rtc.setTime(12, 0, 0);        // 12:00:00
        rtc.setDate(1,1,2025);      // 2025-Jan-01 DD,MM,YYYY for DS1302.h library
    }
    readAndUpdateTimeFromRTC(); // 从RTC获取初始时间

    long initial_adc_val = analogRead(TEMP_SENSOR_PIN);
    kalman_x_hat = (500.0 * initial_adc_val) / 1023.0; // 初始化卡尔曼滤波器估计值
    currentTemperature_filtered = kalman_x_hat;

    hourly_chime_last_triggered_hour = currentTime.hour; // 避免启动时立即报时

    lcd.clear();
    #ifdef __OLED__
    U8G2_Init(); // 初始化 OLED 显示
    #endif
}

void loop() {
    if (currentMode == SystemMode::NORMAL) {
        handleNormalMode();
    } else if (currentMode == SystemMode::SET_TIME) {
        handleTimeSettingMode();
    } else if (currentMode == SystemMode::SET_ALARM) {
        handleAlarmSettingMode();
    } else if (currentMode == SystemMode::STOPWATCH) {
        handleStopwatchMode();
    } else if (currentMode == SystemMode::COUNTDOWN_SET) {
        handleCountdownSetMode();
    } else if (currentMode == SystemMode::COUNTDOWN_RUNNING) {
        handleCountdownRunningMode();
    }

    if (clock_alarm_active) { // 如果时间闹钟正在响，持续处理
        handleActiveClockAlarmSound();
    }
    #ifdef __OLED__
    unsigned long currentMillis = millis();
    static bool oledNeedsRedraw = true; // 初始强制绘制一次
    static unsigned long lastOledUpdateMillis = 0;
    if (oledNeedsRedraw && (currentMode == SystemMode::NORMAL)) {
        #ifdef __Serial_DEBUG__
        Serial.print("OLED Redraw: ");
        #endif
        displayOLEDClock(currentTime.hour, currentTime.minute, currentTime.second); // OLED显示时钟
        oledNeedsRedraw = false;
    }
    if(currentMillis - lastOledUpdateMillis >= 1000) {
        lastOledUpdateMillis = currentMillis;
        oledNeedsRedraw = true; // 每秒强制重绘
    }
    #endif
    #ifdef __KalmanConfig__
    handleKalmanSerialConfig(); // 任何模式下都可以调整卡尔曼参数 删除此功能以减少RAM占用，增加此函数RAM会溢出
    #endif

    delay(50); // 主循环延时，平衡响应速度和CPU占用
}

// --- 时间获取与处理 ---
void readAndUpdateTimeFromRTC() {
    Time t = rtc.getTime();
    currentTime.year = t.year;
    currentTime.month = t.mon;
    currentTime.day = t.date;
    currentTime.hour = t.hour;
    currentTime.minute = t.min;
    currentTime.second = t.sec;
#ifdef __Serial_DEBUG__
    // Serial.print("RTC Time: ");
    // Serial.print(currentTime.year); Serial.print("-");
    // Serial.print(currentTime.month); Serial.print("-");
    // Serial.print(currentTime.day); Serial.print(" ");
    // Serial.print(currentTime.hour); Serial.print(":");
    // Serial.print(currentTime.minute); Serial.print(":");
    // Serial.println(currentTime.second);
#endif
    calculateDayOfWeek(currentTime); 
}

void calculateDayOfWeek(DateTime& dt) {
    int d = dt.day;
    int m = dt.month;
    int y = dt.year;
    int temp_m = m;
    if (temp_m == 1) { temp_m = 13; }
    if (temp_m == 2) { temp_m = 14; }
    dt.dayOfWeek = (d + 2 * temp_m + 3 * (temp_m + 1) / 5 + y + y / 4 - y / 100 + y / 400) % 7 + 1;
}

String getDayOfWeekString(int dow) { 
    switch (dow) {
        case 1: return "Mon"; case 2: return "Tue"; case 3: return "Wed";
        case 4: return "Thu"; case 5: return "Fri"; case 6: return "Sat";
        case 7: return "Sun"; default: return "Err";
    }
}

int daysInMonth(int year, int month) {
    if (month < 1 || month > 12) return 0; 
    if (month == 2) { 
        return ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) ? 29 : 28;
    } else if (month == 4 || month == 6 || month == 9 || month == 11) { 
        return 30;
    }
    return 31; 
}

// --- 显示函数 ---
#ifdef __MEMORY_EFFICIENT__
void formatNumber(int col, int row, int num, int digits) {
    lcd.setCursor(col, row);
    char buffer[12]; // 对于 int 类型足够了 (-32768 到 32767)
    itoa(num, buffer, 10); // 10 表示十进制

    int len = strlen(buffer);
    for (int i = 0; i < digits - len; i++) {
        lcd.print('0'); // 打印字符 '0' 而不是字符串 "0"
    }
    lcd.print(buffer);
}
#else
void formatNumber(int col, int row, int num, int digits) {
    lcd.setCursor(col, row);
    String s = String(num);
    for (int i = 0; i < digits - (int)s.length(); i++) {
        lcd.print("0");
    }
    lcd.print(s);
}
#endif

void printFloat(int col, int row, float val, int decimalPlaces, int totalDigitsBeforeDecimal) {
    lcd.setCursor(col, row);
    bool is_negative = val < 0;
    float abs_val = abs(val);

    if (is_negative) lcd.print("-");

    if (abs_val < 10.0f && totalDigitsBeforeDecimal > 1) { // totalDigitsBeforeDecimal > 1 ensures we only pad if space is allocated for it
        lcd.print("0");
    }
    lcd.print(abs_val, decimalPlaces);
}


void displayTimeScreen() {
#ifdef __Serial_DEBUG__
    // Serial.print("Display Time: "); 
#endif
    // 清除表盘空白
    lcd.setCursor(10, 0);
    lcd.print(" ");
    lcd.setCursor(14, 0);
    lcd.print("  ");
    lcd.setCursor(8, 1);
    lcd.print(" ");
    lcd.setCursor(15, 1);
    lcd.print(" ");
    static unsigned long last_display_time = 0;
    unsigned long current_millis = millis();
    static int last_second_displayed = -1;

    if (currentTime.second != last_second_displayed || current_millis - last_display_time > 1000) {
        last_display_time = current_millis;
        last_second_displayed = currentTime.second;

        formatNumber(0, 0, currentTime.year, 4); lcd.print("-");
        formatNumber(5, 0, currentTime.month, 2); lcd.print("-");
        formatNumber(8, 0, currentTime.day, 2);
        lcd.setCursor(11, 0); lcd.print(getDayOfWeekString(currentTime.dayOfWeek)); lcd.print(" "); 

        formatNumber(0, 1, currentTime.hour, 2); lcd.print(":");
        formatNumber(3, 1, currentTime.minute, 2); lcd.print(":");
        formatNumber(6, 1, currentTime.second, 2);

        printFloat(9, 1, currentTemperature_filtered, 1, 2); 
        lcd.print((char)223); lcd.print("C "); 
    }
    if(clock_alarm_active){
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Alarming!"));
        lcd.setCursor(0, 1);
        lcd.print(F("Time Up!"));
        lcd.print(F("Press to Stop"));
    }
}

void displaySetTimeScreen(const DateTime& tempTime, int step) {
    lcd.clear(); 
    lcd.setCursor(0, 0); 

    lcd.setCursor(0,0);
    if (settingStep == 5) lcd.print(">"); else lcd.print(" "); formatNumber(1, 0, tempTime.year, 4); 
    lcd.setCursor(5,0);
    if (settingStep == 4) lcd.print(">"); else lcd.print("-"); formatNumber(6, 0, tempTime.month, 2);  
    lcd.setCursor(8,0);
    if (settingStep == 3) lcd.print(">"); else lcd.print("-"); formatNumber(9,0, tempTime.day, 2); 
    lcd.setCursor(13,0);
    lcd.print("Set");

    lcd.setCursor(0,1);
    if (settingStep == 0) lcd.print(">"); else lcd.print(" "); formatNumber(1, 1, tempTime.hour, 2); lcd.print(":"); 
    lcd.setCursor(4,1);
    if (settingStep == 1) lcd.print(">"); else lcd.print(" "); formatNumber(5, 1, tempTime.minute, 2); lcd.print(":"); 
    lcd.setCursor(8,1);
    if (settingStep == 2) lcd.print(">"); else lcd.print(" "); formatNumber(9,1, tempTime.second, 2);
    lcd.setCursor(13,1); lcd.print("Tim");
}

#define LCD_LASTLINE 15 // LCD最后一行的列数
void displayStopwatchScreen(unsigned long elapsed_ms, bool is_running) {
    
    unsigned long total_seconds = elapsed_ms / 1000;
    unsigned long disp_hours = total_seconds / 3600;
    unsigned long disp_minutes = (total_seconds % 3600) / 60;
    unsigned long disp_seconds = total_seconds % 60;
    unsigned long disp_tenths = (elapsed_ms % 1000) / 10; // 取十分之一秒

    // 第一行: 状态提示
    lcd.setCursor(0, 0);
    if (is_running) {
        lcd.print("Stopwatch: RUN "); // 状态 + 一个空格清尾
    } else {
        if (elapsed_ms > 0) {
            lcd.print("Stopwatch:PAUSE");
        } else {
            lcd.print("Stopwatch:READY"); // READY后面多一个空格清尾
        }
    }
    // 清理第一行末尾可能存在的旧字符
    for (int i = LCD_LASTLINE; i < 16; ++i) {
        lcd.print(" ");
    }


    // 第二行: 时间显示
    // 如果有小时数，则显示 HH:MM:SS
    // 否则显示 MM:SS.T (T 代表十分之一秒)
    lcd.setCursor(0, 1);
    if (disp_hours > 0) {
        formatNumber(0, 1, disp_hours, 2);
        lcd.print(":");
        formatNumber(3, 1, disp_minutes, 2);
        lcd.print(":");
        formatNumber(6, 1, disp_seconds, 2);
        lcd.print("        "); // 清理后面，确保覆盖 MM:SS.T 格式
    } else {
        formatNumber(0, 1, disp_minutes, 2);
        lcd.print(":");
        formatNumber(3, 1, disp_seconds, 2);
        lcd.print(".");
        formatNumber(6, 1, disp_tenths, 2); // 显示十分之一秒
        lcd.print("          "); // 清理后面，确保覆盖 HH:MM:SS 格式
    }
}


void displayCountdownSetScreen(int h, int m, int s, int currentSettingStep) {
    lcd.clear(); // 通常在进入设置界面时或每次按键后清屏重绘
    lcd.setCursor(0, 0);
    lcd.print("Set Countdown:");

    // 小时设置
    lcd.setCursor(0, 1);
    if (currentSettingStep == 0) lcd.print(">"); else lcd.print(" ");
    formatNumber(1, 1, h, 2);
    lcd.print("H");

    // 分钟设置
    lcd.setCursor(5, 1);
    if (currentSettingStep == 1) lcd.print(">"); else lcd.print(" ");
    formatNumber(6, 1, m, 2);
    lcd.print("M");

    // 秒数设置
    lcd.setCursor(10, 1);
    if (currentSettingStep == 2) lcd.print(">"); else lcd.print(" ");
    formatNumber(11, 1, s, 2);
    lcd.print("S");
    
    // 提示用户如何开始 (可选，如果空间足够)
    // lcd.setCursor(15,0); lcd.print(">"); // 例如用 > 代表 "Next/Start"
}
// display.cpp (部分)

void displayCountdownRunningScreen(unsigned long remaining_s, bool is_active, bool is_beeping) {
    unsigned long disp_hours = remaining_s / 3600;
    unsigned long disp_minutes = (remaining_s % 3600) / 60;
    unsigned long disp_seconds = remaining_s % 60;

    // 第一行: 状态提示
    lcd.setCursor(0, 0);
    if (is_beeping) {
        lcd.print("Countdown: END! "); // 结束并蜂鸣
    } else if (is_active) {
        lcd.print("Countdown: RUN  "); // 正在运行
    } else {
        lcd.print("Countdown:PAUSE "); // 暂停
    }
     // 清理第一行末尾可能存在的旧字符
    for (int i = LCD_LASTLINE; i < 16; ++i) {
        lcd.print(" ");
    }


    // 第二行: 剩余时间显示 HH MM SS (用空格分隔，更清晰)
    lcd.setCursor(0, 1);
    formatNumber(0, 1, disp_hours, 2);
    lcd.print("H ");
    formatNumber(4, 1, disp_minutes, 2);
    lcd.print("M ");
    formatNumber(8, 1, disp_seconds, 2);
    lcd.print("S   "); // 清理后面
}

void displaySetAlarmScreen(int tempHour, int tempMin, float tempAlarmTemp, int step) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Set Alarm & Temp");
    lcd.setCursor(0, 1);

    if (step == 0) lcd.print(">"); else lcd.print(" "); formatNumber(1, 1, tempHour, 2); lcd.print(":");
    lcd.setCursor(4, 1);
    if (step == 1) lcd.print(">"); else lcd.print(" "); formatNumber(6, 1, tempMin, 2);

    lcd.setCursor(10, 1);
    if (step == 2) lcd.print(">"); else lcd.print(" ");
    printFloat(11, 1, tempAlarmTemp, 0, 2); 
    lcd.print((char)223); lcd.print("C");
    if (step == 3){
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("Music Mode Choose:");
        lcd.setCursor(0, 1);
        if (currentMusicMode == MusicMode::NORMAL) {
            lcd.print("Normal Mode");
        } else {
            lcd.print("Music Mode");
        }
    }
}

#ifdef __OLED__
// ----------------------------------------------------------------------------
// 内部辅助函数：绘制表盘元素 (刻度、数字等)
// ----------------------------------------------------------------------------
void drawClockFaceElements() {
  // 1. 绘制表盘外圈 (与之前相同)
  u8g2.drawCircle(CLOCK_CENTER_X, CLOCK_CENTER_Y, CLOCK_RADIUS);
  u8g2.drawCircle(CLOCK_CENTER_X, CLOCK_CENTER_Y, CLOCK_RADIUS - 1);

  // 2. 绘制小时刻度和数字
  u8g2.setFont(u8g2_font_u8glib_4_tf); // 选择一个非常小的字体
  int textHeightApprox = 4;
  int numDisplayRadius = CLOCK_RADIUS - 7;

  char numBuffer[4]; // 用于存储转换后的数字字符串 (e.g., "12\0")

  for (int i = 1; i <= 12; ++i) {
    float angleDeg = (i * 30.0) - 90.0;
    float angleRad = angleDeg * DEG_TO_RAD;

    // --- 绘制刻度线 (与之前相同) ---
    int x1_tick, y1_tick, x2_tick, y2_tick;
    if (i % 3 == 0) {
      x1_tick = CLOCK_CENTER_X + (CLOCK_RADIUS - 5) * cos(angleRad);
      y1_tick = CLOCK_CENTER_Y + (CLOCK_RADIUS - 5) * sin(angleRad);
      x2_tick = CLOCK_CENTER_X + (CLOCK_RADIUS - 1) * cos(angleRad);
      y2_tick = CLOCK_CENTER_Y + (CLOCK_RADIUS - 1) * sin(angleRad);
      u8g2.drawLine(x1_tick, y1_tick, x2_tick, y2_tick);
    } else {
      x1_tick = CLOCK_CENTER_X + (CLOCK_RADIUS - 3) * cos(angleRad);
      y1_tick = CLOCK_CENTER_Y + (CLOCK_RADIUS - 3) * sin(angleRad);
      x2_tick = CLOCK_CENTER_X + (CLOCK_RADIUS - 1) * cos(angleRad);
      y2_tick = CLOCK_CENTER_Y + (CLOCK_RADIUS - 1) * sin(angleRad);
      u8g2.drawLine(x1_tick, y1_tick, x2_tick, y2_tick);
    }

    // --- 绘制小时数字 (修改以避免 String 对象) ---
    // String numStr = String(i); // 旧方法，占用较多RAM且可能导致碎片
    sprintf(numBuffer, "%d", i); // 新方法: 将数字格式化到字符数组中
                                 // 对于1-12的数字, `itoa(i, numBuffer, 10);` 也可以
    
    int strWidth = u8g2.getStrWidth(numBuffer); // 使用 numBuffer

    int numX_center = CLOCK_CENTER_X + numDisplayRadius * cos(angleRad);
    int numY_center = CLOCK_CENTER_Y + numDisplayRadius * sin(angleRad);

    int drawNumX = numX_center - (strWidth / 2);
    int drawNumY = numY_center + (textHeightApprox / 2) -1;

    u8g2.drawStr(drawNumX, drawNumY, numBuffer); // 使用 numBuffer
  }

  // 3. 绘制更突出的中心圆点
  u8g2.drawDisc(CLOCK_CENTER_X, CLOCK_CENTER_Y, 3);
  u8g2.drawCircle(CLOCK_CENTER_X,CLOCK_CENTER_Y, 1);
}

// ----------------------------------------------------------------------------
// 内部辅助函数：绘制时钟指针 
// ----------------------------------------------------------------------------
void drawClockHands(int h, int m, int s) {
  float angleRad;

  h = h % 12;
  if (h == 0) { h = 12; }

  // 时针
  float hourAngleDeg = ((h + m / 60.0 + s / 3600.0) * 30.0) - 90.0;
  angleRad = hourAngleDeg * DEG_TO_RAD;
  int hourHandX = CLOCK_CENTER_X + HOUR_HAND_LENGTH * cos(angleRad);
  int hourHandY = CLOCK_CENTER_Y + HOUR_HAND_LENGTH * sin(angleRad);
  u8g2.drawLine(CLOCK_CENTER_X, CLOCK_CENTER_Y, hourHandX, hourHandY);
  u8g2.drawLine(CLOCK_CENTER_X+1, CLOCK_CENTER_Y, hourHandX+1, hourHandY); // 尝试加粗

  // 分针
  float minuteAngleDeg = ((m + s / 60.0) * 6.0) - 90.0;
  angleRad = minuteAngleDeg * DEG_TO_RAD;
  int minuteHandX = CLOCK_CENTER_X + MINUTE_HAND_LENGTH * cos(angleRad);
  int minuteHandY = CLOCK_CENTER_Y + MINUTE_HAND_LENGTH * sin(angleRad);
  u8g2.drawLine(CLOCK_CENTER_X, CLOCK_CENTER_Y, minuteHandX, minuteHandY);

  // 秒针
  float secondAngleDeg = (s * 6.0) - 90.0;
  angleRad = secondAngleDeg * DEG_TO_RAD;
  int secondHandX = CLOCK_CENTER_X + SECOND_HAND_LENGTH * cos(angleRad);
  int secondHandY = CLOCK_CENTER_Y + SECOND_HAND_LENGTH * sin(angleRad);
  u8g2.drawLine(CLOCK_CENTER_X, CLOCK_CENTER_Y, secondHandX, secondHandY);
}


// ============================================================================
// 封装的函数 1: U8G2 初始化
// ============================================================================
void U8G2_Init() {
  u8g2.begin();
  u8g2.setDrawColor(1); // 确保绘图颜色为1 (亮)
  // u8g2.setFontMode(1); // 设置为透明字体背景模式，如果需要文字叠加而不覆盖背景
                        // 默认是0 (不透明)，对于这个时钟，透明可能更好看
                        // 注意: 透明模式可能比不透明模式稍慢
  // 在页模式下，setFontMode需要在每个绘图周期（do-while内）或每次setFont后设置
}

// ============================================================================
// 封装的函数 2: 显示时钟 
// ============================================================================
void displayOLEDClock(int hours, int minutes, int seconds) {
  u8g2.firstPage();
  do {
    // 在页缓冲模式下，每次循环U8g2会自动清除当前页的缓冲区
    // 因此不需要 u8g2.clearBuffer()

    // 如果设置了特定的字体模式 (如透明)，最好在这里或绘图函数内部再次设置
    // u8g2.setFontMode(1); 

    drawClockFaceElements();
    drawClockHands(hours, minutes, seconds);

  } while (u8g2.nextPage());
}

#endif

// --- 模式处理 ---
void handleNormalMode() {
    readAndUpdateTimeFromRTC();
    readAndUpdateTemperature();
    displayTimeScreen();

    checkAndTriggerHourlyChime();
    checkAndTriggerClockAlarm();
    checkAndTriggerTemperatureAlarm();

    if (checkButtonPress(BUTTON_CHOOSE_PIN, button_last_press_time_choose, BUTTON_DEBOUNCE_DELAY)) {
        enterTimeSettingMode();
        return;
    }

    if(checkLongButtonPress(BUTTON_ADD_PIN, buttonADDPressedLong, button_last_press_time_add, BUTTON_DEBOUNCE_DELAY * BUTTON_LONGTIME_Multiple)) {
        enterStopwatchMode();
        return;
    }
    // 类似地为倒计时添加一个按键逻辑 (例如长按MINUS)
    if (checkLongButtonPress(BUTTON_MINUS_PIN, buttonMINUSPressedLong, button_last_press_time_minus_long, BUTTON_DEBOUNCE_DELAY * BUTTON_LONGTIME_Multiple))
    {
        // (需要为 button_last_press_time_minus_long 在 globals.h 和 .ino 中声明和定义)
        enterCountdownSetMode();
        return;
    }

    static unsigned long combo_press_start_time = 0;
    bool add_low = (digitalRead(BUTTON_ADD_PIN) == LOW);
    bool minus_low = (digitalRead(BUTTON_MINUS_PIN) == LOW);

    if (add_low && minus_low) { 
        if (combo_press_start_time == 0) { 
            combo_press_start_time = millis();
        } else if (millis() - combo_press_start_time > BUTTON_DEBOUNCE_DELAY * 4) { 
            enterAlarmSettingMode();
            combo_press_start_time = 0; 
            return;
        }
    } else { 
        combo_press_start_time = 0;
    }
}

DateTime tempSettingTime; 

void enterTimeSettingMode() {
    currentMode = SystemMode::SET_TIME;
    settingStep = 0; 
    tempSettingTime = currentTime; 
    setting_mode_entry_time = millis(); 
    lcd.clear();
    displaySetTimeScreen(tempSettingTime, settingStep);
}

void enterStopwatchMode() {
    currentMode = SystemMode::STOPWATCH;
    stopwatch_running = false;
    stopwatch_start_millis = 0;
    stopwatch_elapsed_at_pause = 0; // 重置秒表
    setting_mode_entry_time = millis(); // 用于可能的超时或返回逻辑
    lcd.clear();
    displayStopwatchScreen(0, stopwatch_running); // 初始显示 00:00.0
    // (displayStopwatchScreen 需要知道当前状态来显示 "Start", "Pause", "Resume")
}
void enterCountdownSetMode() {
    currentMode = SystemMode::COUNTDOWN_SET;
    countdown_setting_step = 0; // 开始设置小时
    countdown_running = false;  // 确保计时器未运行
    countdown_beeping = false;  // 确保不在蜂鸣
    noTone(BUZZER_TONE_PIN);    // 停止可能存在的蜂鸣
    setting_mode_entry_time = millis();
    lcd.clear();
    displayCountdownSetScreen(countdown_set_hours, countdown_set_minutes, countdown_set_seconds, countdown_setting_step);
}

// 内部辅助函数，从设置模式转换到运行模式
void enterCountdownRunningMode()
{
    countdown_total_set_seconds = (unsigned long)countdown_set_hours * 3600 +
                                (unsigned long)countdown_set_minutes * 60 +
                                countdown_set_seconds;

    if (countdown_total_set_seconds == 0)
    { // 如果设置时间为0，则不启动
        // 可以选择返回设置模式或显示错误提示
        // 为简单起见，我们可能不允许启动0秒倒计时，或让它立即结束并蜂鸣
        // 这里假设如果为0，则停留在设置模式，或者在handleCountdownSetMode中阻止 >2 后直接启动
        return;
    }

    currentMode = SystemMode::COUNTDOWN_RUNNING;
    countdown_running = true;
    countdown_beeping = false;
    countdown_target_millis = millis() + countdown_total_set_seconds * 1000UL;
    // displayCountdownRunningScreen(countdown_total_set_seconds, countdown_running);
}

int tempSettingAlarmHour, tempSettingAlarmMinute;
float tempSettingAlarmTemp;

void enterAlarmSettingMode() {
    currentMode = SystemMode::SET_ALARM;
    settingStep = 0; 
    tempSettingAlarmHour = alarm_hour;
    tempSettingAlarmMinute = alarm_minute;
    tempSettingAlarmTemp = temperatureAlarmThreshold;
    setting_mode_entry_time = millis();
    lcd.clear();
    displaySetAlarmScreen(tempSettingAlarmHour, tempSettingAlarmMinute, tempSettingAlarmTemp, settingStep);
}

void exitSettingModeAndSave(bool saveTimeToRTC, bool saveAlarm) {
    if (saveTimeToRTC) {
        rtc.setTime(tempSettingTime.hour, tempSettingTime.minute, tempSettingTime.second);
        rtc.setDate(tempSettingTime.day, tempSettingTime.month, tempSettingTime.year);
#ifdef __Serial_DEBUG__
        Serial.print("RTC set to: ");
        Serial.print(tempSettingTime.year); Serial.print("-");
        Serial.print(tempSettingTime.month); Serial.print("-"); 
        Serial.print(tempSettingTime.day); Serial.print(" ");
        Serial.print(tempSettingTime.hour); Serial.print(":");
        Serial.print(tempSettingTime.minute); Serial.print(":");
        Serial.println(tempSettingTime.second);
#endif
        readAndUpdateTimeFromRTC(); 
    }
    if (saveAlarm) {
        alarm_hour = tempSettingAlarmHour;
        alarm_minute = tempSettingAlarmMinute;
        temperatureAlarmThreshold = tempSettingAlarmTemp;
    }
    currentMode = SystemMode::NORMAL;
    lcd.clear();
    displayTimeScreen(); 
}

#if 1
// void normalizeDateTime(DateTime& dt) {
//     // 规范化秒 (处理进位和借位)
//     if (dt.second >= 60) {
//         dt.minute += dt.second / 60;
//         dt.second %= 60;
//     } else if (dt.second < 0) {
//         int borrow_minutes = (abs(dt.second) + 59) / 60;
//         dt.minute -= borrow_minutes;
//         dt.second += borrow_minutes * 60; // 结果必然 >= 0
//     }

//     // 规范化分钟
//     if (dt.minute >= 60) {
//         dt.hour += dt.minute / 60;
//         dt.minute %= 60;
//     } else if (dt.minute < 0) {
//         int borrow_hours = (abs(dt.minute) + 59) / 60;
//         dt.hour -= borrow_hours;
//         dt.minute += borrow_hours * 60;
//     }

//     // 规范化小时
//     if (dt.hour >= 24) {
//         dt.day += dt.hour / 24;
//         dt.hour %= 24;
//     } else if (dt.hour < 0) {
//         int borrow_days = (abs(dt.hour) + 23) / 24;
//         dt.day -= borrow_days;
//         dt.hour += borrow_days * 24;
//     }

//     // 规范化月份和年份 (先处理月份，再处理由于月份调整可能导致的年份调整)
//     // 注意: 月份调整可能导致年份变化，所以年份规范化依赖于月份的正确范围 (1-12)
//     while (dt.month > 12) {
//         dt.month -= 12;
//         dt.year++;
//         // 可选: if (dt.year > MAX_YEAR) dt.year = MAX_YEAR;
//     }
//     while (dt.month < 1) {
//         dt.month += 12;
//         dt.year--;
//         // 可选: if (dt.year < MIN_YEAR) dt.year = MIN_YEAR;
//     }

//     // 规范化日期 (在月份和年份都基本确定后再处理日期，因为每月天数依赖年和月)
//     // 这个循环确保即使日期调整幅度很大（例如 day += 100 或 day -= 100），也能正确处理
//     // 并且修正由于月份或年份改变导致当前日期无效的情况 (例如1月31日 -> 2月31日 -> 2月28/29日)
//     while (true) {
//         int days_in_dt_month = daysInMonth(dt.year, dt.month);
//         if (days_in_dt_month == 0) { // 无效月份，可能由极端年份输入导致
//              // 可以选择一个默认行为，例如重置为1月1日，或者保持原样并依赖外部检查
//              // 为安全起见，这里可以尝试修正月份到有效范围，或者直接退出
//              if (dt.month < 1) dt.month = 1;
//              if (dt.month > 12) dt.month = 12;
//              days_in_dt_month = daysInMonth(dt.year, dt.month); // 重新获取天数
//         }

//         if (dt.day > 0 && dt.day <= days_in_dt_month) {
//             break; // 日期有效
//         }

//         if (dt.day > days_in_dt_month) {
//             dt.day -= days_in_dt_month; // 减去当前月份的天数
//             dt.month++;                 // 月份进位
//             if (dt.month > 12) {        // 月份超过12，年份进位
//                 dt.month = 1;
//                 dt.year++;
//                 // 可选: if (dt.year > MAX_YEAR) dt.year = MAX_YEAR;
//             }
//         } else { // dt.day <= 0
//             dt.month--;                 // 月份借位
//             if (dt.month < 1) {         // 月份小于1，年份借位
//                 dt.month = 12;
//                 dt.year--;
//                 // 可选: if (dt.year < MIN_YEAR) dt.year = MIN_YEAR;
//             }
//             // 加上新的上一个月的天数
//             dt.day += daysInMonth(dt.year, dt.month);
//         }
//     }
    
//     // 可选: 再次检查并强制年份范围，如果需要
//     // if (dt.year > 2099) dt.year = 2099;
//     // if (dt.year < 2000) dt.year = 2000;
// }

// void handleTimeSettingMode() {
//     // 设置模式超时检查
//     if (millis() - setting_mode_entry_time > 30000) { // 30秒超时
//         lcd.clear();
//         lcd.setCursor(0, 0);
//         lcd.print(F("Set Timeout")); // 使用 F() 宏
//         delay(500);
//         exitSettingModeAndSave(false, false); // 不保存, 非菜单按钮退出
//         return;
//     }

//     bool needs_redraw = false;

//     // 强制退出检查 (双按钮同时按下)
//     if (checkMultiButtonPress(BUTTON_ADD_PIN, BUTTON_MINUS_PIN, button_last_press_time_addminus, BUTTON_DEBOUNCE_DELAY)) {
//         lcd.clear();
//         lcd.setCursor(0, 0);
//         lcd.print(F("Force Quit!")); // 使用 F() 宏
//         delay(500);
//         exitSettingModeAndSave(false, false); // 不保存, 非菜单按钮退出
//         return;
//     }

//     // 选择/确认按钮
//     if (checkButtonPress(BUTTON_CHOOSE_PIN, button_last_press_time_choose, BUTTON_DEBOUNCE_DELAY)) {
//         setting_mode_entry_time = millis(); // 重置超时计时器
//         settingStep++;
//         if (settingStep > 5) { // 假设设置步骤为 0(时)..5(年)
//             exitSettingModeAndSave(true, false); // 保存更改, 非菜单按钮退出
//             return;
//         }
//         needs_redraw = true;
//     }

//     // 增加按钮 (长按)
//     if (checkButtonHeld(BUTTON_ADD_PIN, button_last_press_time_add, BUTTON_DEBOUNCE_DELAY)) {
//         setting_mode_entry_time = millis(); // 重置超时计时器
//         needs_redraw = true;
//         switch (settingStep) {
//             case 0: tempSettingTime.hour++; break;
//             case 1: tempSettingTime.minute++; break;
//             case 2: tempSettingTime.second++; break;
//             case 3: tempSettingTime.day++; break;
//             case 4: tempSettingTime.month++; break;
//             case 5: tempSettingTime.year++; break;
//         }
//         normalizeDateTime(tempSettingTime); // 规范化日期时间
//     }

//     // 减少按钮 (长按)
//     if (checkButtonHeld(BUTTON_MINUS_PIN, button_last_press_time_minus, BUTTON_DEBOUNCE_DELAY)) {
//         setting_mode_entry_time = millis(); // 重置超时计时器
//         needs_redraw = true;
//         switch (settingStep) {
//             case 0: tempSettingTime.hour--; break;
//             case 1: tempSettingTime.minute--; break;
//             case 2: tempSettingTime.second--; break;
//             case 3: tempSettingTime.day--; break;
//             case 4: tempSettingTime.month--; break;
//             case 5: tempSettingTime.year--; break;
//         }
//         normalizeDateTime(tempSettingTime); // 规范化日期时间
//     }

//     // 如果需要重绘屏幕
//     if (needs_redraw) {
//         // 由于 normalizeDateTime 已经确保了日期的有效性 (例如，调整月份后，日期不会超出新月份的最大天数)
//         // 所以这里通常不需要再次检查 tempSettingTime.day 是否大于 max_days。
//         // normalizeDateTime 内部的日期规范化部分应该已经处理了这个问题。
//         displaySetTimeScreen(tempSettingTime, settingStep);
//     }
// }
void handleTimeSettingMode() {
    if (millis() - setting_mode_entry_time > 30000) {  
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("Set Timeout"); delay(500); 
        exitSettingModeAndSave(false, false);
        return;
    }

    bool needs_redraw = false;
    if (checkMultiButtonPress(BUTTON_ADD_PIN, BUTTON_MINUS_PIN, button_last_press_time_addminus, BUTTON_DEBOUNCE_DELAY)) {
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("Force Quit!"); delay(500); 
        exitSettingModeAndSave(false, false);
        return;
    }

    if (checkButtonPress(BUTTON_CHOOSE_PIN, button_last_press_time_choose, BUTTON_DEBOUNCE_DELAY )) {
        setting_mode_entry_time = millis(); 
        settingStep++;
        if (settingStep > 5) { 
            exitSettingModeAndSave(true, false); 
            return;
        }
        needs_redraw = true;
    }

    if (checkButtonHeld(BUTTON_ADD_PIN, button_last_press_time_add, BUTTON_DEBOUNCE_DELAY)) {
        setting_mode_entry_time = millis(); needs_redraw = true;
        switch (settingStep) {
            case 0: // Hour
                tempSettingTime.hour++;
                if(tempSettingTime.hour >= 24) {
                    tempSettingTime.hour = 0; // 小时在0-23间循环
                    if(tempSettingTime.day == daysInMonth(tempSettingTime.year, tempSettingTime.month)) {
                        tempSettingTime.day = 1;
                        tempSettingTime.month++;
                        if (tempSettingTime.month > 12) {
                            tempSettingTime.month = 1;
                            tempSettingTime.year++;
                            // if (tempSettingTime.year > 2099) tempSettingTime.year = 2099; // 年份上限
                        }
                    } else {
                        tempSettingTime.day++;
                    }
                }
                break;
            case 1: // Minute
                tempSettingTime.minute++;
                if (tempSettingTime.minute >= 60) {
                    tempSettingTime.minute = 0;
                    tempSettingTime.hour++;
                    if (tempSettingTime.hour >= 24) {
                        tempSettingTime.hour = 0; // 小时在0-23间循环
                        if (tempSettingTime.day == daysInMonth(tempSettingTime.year, tempSettingTime.month)) {
                            tempSettingTime.day = 1;
                            tempSettingTime.month++;
                            if (tempSettingTime.month > 12) {
                                tempSettingTime.month = 1;
                                tempSettingTime.year++;
                                // if (tempSettingTime.year > 2099) tempSettingTime.year = 2099; // 年份上限
                            }
                        } else {
                            tempSettingTime.day++;
                        }
                    }
                }
                break;
            case 2: // Second
                tempSettingTime.second++;
                if (tempSettingTime.second >= 60) {
                    tempSettingTime.second = 0;
                    tempSettingTime.minute++;
                    if (tempSettingTime.minute >= 60) {
                        tempSettingTime.minute = 0;
                        tempSettingTime.hour++;
                        if (tempSettingTime.hour >= 24) {
                            tempSettingTime.hour = 0; // 小时在0-23间循环
                            if (tempSettingTime.day == daysInMonth(tempSettingTime.year, tempSettingTime.month)) {
                                tempSettingTime.day = 1;
                                tempSettingTime.month++;
                                if (tempSettingTime.month > 12) {
                                    tempSettingTime.month = 1;
                                    tempSettingTime.year++;
                                    // if (tempSettingTime.year > 2099) tempSettingTime.year = 2099; // 年份上限
                                }
                            } else {
                                tempSettingTime.day++;
                            }
                        }
                    }
                }
                break;
            case 3: // 日期增加
                tempSettingTime.day++;
                if (tempSettingTime.day > daysInMonth(tempSettingTime.year, tempSettingTime.month)) {
                    tempSettingTime.day = 1;
                    tempSettingTime.month++;
                    if (tempSettingTime.month > 12) {
                        tempSettingTime.month = 1;
                        tempSettingTime.year++;
                        // if (tempSettingTime.year > 2099) tempSettingTime.year = 2099; // 年份上限
                    }
                }
                break;
            case 4: // 月份增加
                tempSettingTime.month++;
                if (tempSettingTime.month > 12) {
                    tempSettingTime.month = 1;
                    tempSettingTime.year++;
                    // if (tempSettingTime.year > 2099) tempSettingTime.year = 2099; // 年份上限
                }
                break;
            case 5: // 年份增加
                tempSettingTime.year++;
                // if (tempSettingTime.year > 2099) tempSettingTime.year = 2099; // 年份上限
                break;
        }
    }

    if (checkButtonHeld(BUTTON_MINUS_PIN, button_last_press_time_minus, BUTTON_DEBOUNCE_DELAY)) {
        setting_mode_entry_time = millis(); needs_redraw = true;
        switch (settingStep) {
            case 0: // Hour
                tempSettingTime.hour--;
                if (tempSettingTime.hour < 0) {
                    tempSettingTime.hour = 23; // 小时在0-23间循环
                    tempSettingTime.day--;
                    if (tempSettingTime.day < 1) {
                        tempSettingTime.month--;
                        if (tempSettingTime.month < 1) {
                            tempSettingTime.month = 12;
                            tempSettingTime.year--;
                            // if (tempSettingTime.year < 2000) tempSettingTime.year = 2000; // 年份下限
                        }
                        tempSettingTime.day = daysInMonth(tempSettingTime.year, tempSettingTime.month); 
                    }
                }
                break;
            case 1: // Minute
                tempSettingTime.minute--;
                if (tempSettingTime.minute < 0) {
                    tempSettingTime.minute = 59;
                    tempSettingTime.hour--;
                    if (tempSettingTime.hour < 0) {
                        tempSettingTime.hour = 23; // 小时在0-23间循环
                        tempSettingTime.day--;
                        if (tempSettingTime.day < 1) {
                            tempSettingTime.month--;
                            if (tempSettingTime.month < 1) {
                                tempSettingTime.month = 12;
                                tempSettingTime.year--;
                                // if (tempSettingTime.year < 2000) tempSettingTime.year = 2000; // 年份下限
                            }
                            tempSettingTime.day = daysInMonth(tempSettingTime.year, tempSettingTime.month); 
                        }
                    }
                }
                break;
            case 2: // Second
                tempSettingTime.second--;
                if (tempSettingTime.second < 0) {
                    tempSettingTime.second = 59;
                    tempSettingTime.minute--;
                    if (tempSettingTime.minute < 0) {
                        tempSettingTime.minute = 59;
                        tempSettingTime.hour--;
                        if (tempSettingTime.hour < 0) {
                            tempSettingTime.hour = 23; // 小时在0-23间循环
                            tempSettingTime.day--;
                            if (tempSettingTime.day < 1) {
                                tempSettingTime.month--;
                                if (tempSettingTime.month < 1) {
                                    tempSettingTime.month = 12;
                                    tempSettingTime.year--;
                                    // if (tempSettingTime.year < 2000) tempSettingTime.year = 2000; // 年份下限
                                }
                                tempSettingTime.day = daysInMonth(tempSettingTime.year, tempSettingTime.month); 
                            }
                        }
                    }
                }
                break;
            case 3: // 日期减少
                tempSettingTime.day--;
                if (tempSettingTime.day < 1) {
                    tempSettingTime.month--;
                    if (tempSettingTime.month < 1) {
                        tempSettingTime.month = 12;
                        tempSettingTime.year--;
                        // if (tempSettingTime.year < 2000) tempSettingTime.year = 2000; // 年份下限
                    }
                    tempSettingTime.day = daysInMonth(tempSettingTime.year, tempSettingTime.month); 
                }
                break;
            case 4: // 月份减少
                tempSettingTime.month--;
                if (tempSettingTime.month < 1) {
                    tempSettingTime.month = 12;
                    tempSettingTime.year--;
                    // if (tempSettingTime.year < 2000) tempSettingTime.year = 2000; // 年份下限
                }
                break;
            case 5: // 年份减少
                tempSettingTime.year--;
                // if (tempSettingTime.year < 2000) tempSettingTime.year = 2000; // 年份下限
                break;
        }
    }
    
    if (needs_redraw) { 
        int max_days = daysInMonth(tempSettingTime.year, tempSettingTime.month);
        if (tempSettingTime.day > max_days) {
            tempSettingTime.day = max_days;
        }
        displaySetTimeScreen(tempSettingTime, settingStep); 
    }
}
#endif

void handleStopwatchMode() {
    unsigned long current_millis = millis();
    unsigned long display_elapsed_ms = 0;

    // 按键逻辑:
    // CHOOSE: Start / Pause / Resume
    // ADD (短按/单击): Lap (暂不实现，过于复杂) / Reset (当停止或暂停时)
    // MINUS (长按或其他组合): Exit to NORMAL mode

    if (checkButtonPress(BUTTON_CHOOSE_PIN, button_last_press_time_choose, BUTTON_DEBOUNCE_DELAY )) {
        setting_mode_entry_time = current_millis; // 重置不活动超时
        if (stopwatch_running) { // 正在运行 -> 暂停
            stopwatch_running = false;
            stopwatch_elapsed_at_pause += (current_millis - stopwatch_start_millis); // 累加本次运行时间
        } else { // 已停止或暂停 -> 开始/继续
            stopwatch_running = true;
            stopwatch_start_millis = current_millis; // 记录新的开始点
        }
    }

    // ADD 按钮: 在暂停或停止状态下按ADD键重置秒表
    if (!stopwatch_running && checkButtonPress(BUTTON_ADD_PIN, button_last_press_time_add, BUTTON_DEBOUNCE_DELAY )) {
        setting_mode_entry_time = current_millis;
        stopwatch_elapsed_at_pause = 0; // 完全重置
        stopwatch_start_millis = 0;     // 确保下次开始时从0计时
        // (如果需要，可以加一个确认步骤或长按才重置)
    }
    
    // MINUS 按钮 (示例: 长按退出)
    // 这里用一个简化的逻辑，实际项目中可能需要更复杂的长按检测
    // 或者使用与进入闹钟设置类似的同时按键组合退出
    // static unsigned long minus_hold_start_time = 0;
    // if (digitalRead(BUTTON_MINUS_PIN) == LOW) {
    //     if (minus_hold_start_time == 0) {
    //         minus_hold_start_time = current_millis;
    //     } else if (current_millis - minus_hold_start_time > BUTTON_DEBOUNCE_DELAY) { // 短按
    //         minus_hold_start_time = 0; // 重置
    //         exitSettingModeAndSave(false, false); // 退出设置模式
            
    //         lcd.clear(); // displayTimeScreen 会处理清屏
    //         return; // 退出 handleStopwatchMode
    //     }
    // } else {
    //     minus_hold_start_time = 0;
    // }
    if(checkMultiButtonPress(BUTTON_ADD_PIN, BUTTON_MINUS_PIN, button_last_press_time_addminus, BUTTON_DEBOUNCE_DELAY)) {
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("Quit Timer MODE!"); delay(500); 
        exitSettingModeAndSave(false, false); // 退出设置模式
        #ifdef __Serial_DEBUG__
        Serial.println("Quit Stopwatch Mode");
        #endif
        return; // 退出 handleStopwatchMode
    }

    // 计算要显示的时间
    if (stopwatch_running) {
        display_elapsed_ms = stopwatch_elapsed_at_pause + (current_millis - stopwatch_start_millis);
    } else {
        display_elapsed_ms = stopwatch_elapsed_at_pause;
    }

    displayStopwatchScreen(display_elapsed_ms, stopwatch_running);
    


}


void handleCountdownSetMode() {
    if (millis() - setting_mode_entry_time > 30000) { // 30秒超时
        currentMode = SystemMode::NORMAL; // 返回主时钟界面
        lcd.clear(); 
        return;
    }
    if(checkMultiButtonPress(BUTTON_ADD_PIN, BUTTON_MINUS_PIN, button_last_press_time_addminus, BUTTON_DEBOUNCE_DELAY)) {
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("Quit Counter MODE!"); delay(500); 
        exitSettingModeAndSave(false, false); // 退出设置模式
    }

    bool needs_redraw = false;

    if (checkButtonPress(BUTTON_CHOOSE_PIN, button_last_press_time_choose, BUTTON_DEBOUNCE_DELAY )) {
        setting_mode_entry_time = millis();
        countdown_setting_step++;
        if (countdown_setting_step > 2) { // H, M, S 都设置过了
            // 再次按 CHOOSE 键启动倒计时
            countdown_total_set_seconds = (unsigned long)countdown_set_hours * 3600 +
                                          (unsigned long)countdown_set_minutes * 60 +
                                          countdown_set_seconds;
            if (countdown_total_set_seconds > 0) {
                enterCountdownRunningMode(); // 直接转换到运行模式
                return; // 已切换模式，退出此函数
            } else {
                // 如果总时间为0，可以选择不启动，并将步骤重置为0
                countdown_setting_step = 0; 
            }
        }
        needs_redraw = true;
    }

    if (checkButtonHeld(BUTTON_ADD_PIN, button_last_press_time_add, BUTTON_DEBOUNCE_DELAY)) {
        setting_mode_entry_time = millis(); needs_redraw = true;
        switch (countdown_setting_step) {
            case 0: countdown_set_hours = (countdown_set_hours + 1) % 24; break; // 最多23小时
            case 1: countdown_set_minutes = (countdown_set_minutes + 1) % 60; break;
            case 2: countdown_set_seconds = (countdown_set_seconds + 1) % 60; break;
        }
    }

    if (checkButtonHeld(BUTTON_MINUS_PIN, button_last_press_time_minus, BUTTON_DEBOUNCE_DELAY)) {
        setting_mode_entry_time = millis(); needs_redraw = true;
        switch (countdown_setting_step) {
            case 0: countdown_set_hours = (countdown_set_hours - 1 + 24) % 24; break;
            case 1: countdown_set_minutes = (countdown_set_minutes - 1 + 60) % 60; break;
            case 2: countdown_set_seconds = (countdown_set_seconds - 1 + 60) % 60; break;
        }
    }
    
    // // MINUS 按钮长按退出 (示例)
    // static unsigned long minus_hold_start_time_cds = 0;
    // if (digitalRead(BUTTON_MINUS_PIN) == LOW) {
    //     if (checkButtonHeld(BUTTON_MINUS_PIN, minus_hold_start_time_cds, BUTTON_DEBOUNCE_DELAY*BUTTON_LONGTIME_Multiple) && minus_hold_start_time_cds != button_last_press_time_minus ) { // 确保不是短按触发的
    //         exitSettingModeAndSave(false, false); // 退出设置模式
    //         return; // 退出倒计时设置模式
    //         // 这里可以添加一个提示，例如 "长按退出"
    //         // 上面 checkButtonHeld 会更新 minus_hold_start_time_cds, 所以这里的条件需要小心
    //     }
    // } else {
    //     minus_hold_start_time_cds = 0;
    // }


    if (needs_redraw) {
        displayCountdownSetScreen(countdown_set_hours, countdown_set_minutes, countdown_set_seconds, countdown_setting_step);
        // lcd.clear();
        // lcd.setCursor(0,0); lcd.print("Set Countdown:");
        
        // lcd.setCursor(0,1);
        // if(countdown_setting_step == 0) lcd.print(">"); else lcd.print(" "); formatNumber(1,1,countdown_set_hours,2); lcd.print("H");
        // lcd.setCursor(5,1);
        // if(countdown_setting_step == 1) lcd.print(">"); else lcd.print(" "); formatNumber(6,1,countdown_set_minutes,2); lcd.print("M");
        // lcd.setCursor(10,1);
        // if(countdown_setting_step == 2) lcd.print(">"); else lcd.print(" "); formatNumber(11,1,countdown_set_seconds,2); lcd.print("S");
    }
}

void handleCountdownRunningMode()
{
    unsigned long current_millis = millis();
    unsigned long display_remaining_s = 0;

    // --- 状态更新 ---
    if (countdown_beeping)
    {
        // 蜂鸣逻辑
        unsigned long beep_duration = 3000; // 总蜂鸣时长3秒
        unsigned long beep_interval = 400;  // 响0.2秒，停0.2秒
        if (current_millis - countdown_beep_start_millis > beep_duration)
        {
            countdown_beeping = false;
            noTone(BUZZER_TONE_PIN);
            enterCountdownSetMode(); // 蜂鸣结束，返回设置模式
            return;
        }
        // 间歇鸣响
        if (((current_millis - countdown_beep_start_millis) / (beep_interval / 2)) % 2 == 0)
        {
            tone(BUZZER_TONE_PIN, ALARM_SOUND_FREQ, beep_interval / 2 - 20);
        }
        else
        {
            noTone(BUZZER_TONE_PIN);
        }
        display_remaining_s = 0;
    }
    else if (countdown_running)
    {
        if (current_millis >= countdown_target_millis)
        {
            // 倒计时结束
            countdown_running = false;
            countdown_beeping = true;
            countdown_beep_start_millis = current_millis;
            // 触发第一次蜂鸣
            tone(BUZZER_TONE_PIN, ALARM_SOUND_FREQ, 100); // 短促提示音
            display_remaining_s = 0;
        }
        else
        {
            display_remaining_s = (countdown_target_millis - current_millis + 500) / 1000UL; // +500 for rounding
        }
    }
    else
    { // 暂停状态
        // 在暂停时，显示的是暂停时的剩余秒数，这个秒数在暂停时已经被计算并存储
        // 这里需要一个变量来存储暂停时的剩余秒数，或者重新计算
        // 为简单起见，我们假设 countdown_target_millis 存储的是原始目标，
        // 当暂停并继续时，countdown_target_millis 需要被调整。
        // 或者，我们用一个 countdown_remaining_at_pause_ms 变量
        // 这里我们让显示保持在暂停时的值，但需要确保这个值被正确获取
        // unsigned long paused_remaining_ms = countdown_target_millis - millis(); // 这是如果没暂停会剩余的时间
        //                                                                         // 实际上，应该在暂停时记录剩余时间
        display_remaining_s = countdown_total_set_seconds - ((millis() - (countdown_target_millis - countdown_total_set_seconds * 1000UL)) / 1000UL);
        // 上述计算复杂，更简单的方式是在暂停时存下剩余秒数
        // 此处简化：假设我们有一个变量 `countdown_seconds_at_pause`
        // display_remaining_s = countdown_seconds_at_pause;
        // 临时的简化：直接显示0，表示暂停且未到时间
        if (countdown_target_millis > current_millis)
        { // 仅当目标时间未到
            display_remaining_s = (countdown_target_millis - current_millis + 500) / 1000UL;
        }
        else
        {
            display_remaining_s = 0; // 如果因为某种原因暂停时目标时间已过
        }
        // **修正：暂停逻辑**
        // 当暂停时，我们需要记录还剩下多少毫秒，当继续时，从当前millis加上这些剩余毫秒得到新的目标
        // 为简化，此处的暂停仅停止屏幕更新和蜂鸣器，不精确处理暂停/继续的计时补偿。
        // 也就是说，暂停期间时间仍在流逝，再次“继续”会从当前时间点算起。
        // 一个更健壮的实现需要 `countdown_remaining_ms_at_pause`
    }

    // --- 按键处理 ---
    if (checkButtonPress(BUTTON_CHOOSE_PIN, button_last_press_time_choose, BUTTON_DEBOUNCE_DELAY ))
    {
        if (countdown_beeping)
        {
            countdown_beeping = false;
            noTone(BUZZER_TONE_PIN);
            enterCountdownSetMode(); // 返回设置模式
            return;
        }
        else if (countdown_running)
        { // 正在运行 -> 暂停
            countdown_running = false;
            // 在此记录剩余时间，以便精确继续
            // countdown_remaining_ms_at_pause = countdown_target_millis - current_millis;
            noTone(BUZZER_TONE_PIN); // 如果在倒计时结束前暂停，确保蜂鸣器不响
        }
        else
        { // 已暂停 -> 继续 (或如果时间已到，则直接进入蜂鸣或设置)
            if (current_millis < countdown_target_millis)
            { // 只有当目标时间还没到，才能继续
                countdown_running = true;
                // 如果实现了精确暂停:
                // countdown_target_millis = current_millis + countdown_remaining_ms_at_pause;
            }
            else
            {                              // 如果暂停时时间已经到了或过去了，直接去蜂鸣或设置
                countdown_running = false; // 确保不在运行
                if (!countdown_beeping)
                { // 如果还没开始蜂鸣
                    countdown_beeping = true;
                    countdown_beep_start_millis = current_millis;
                    tone(BUZZER_TONE_PIN, ALARM_SOUND_FREQ, 100);
                }
            }
        }
    }

    // MINUS 按钮 (短按或长按): 停止并返回设置模式
    if (checkButtonPress(BUTTON_MINUS_PIN, button_last_press_time_minus, BUTTON_DEBOUNCE_DELAY ))
    {
        countdown_running = false;
        countdown_beeping = false;
        noTone(BUZZER_TONE_PIN);
        enterCountdownSetMode(); // 返回设置模式
        return;
    }

    // --- 显示 ---
    // displayCountdownRunningScreen(display_remaining_s, countdown_running || countdown_beeping);
    // 示例显示逻辑 (应在 displayCountdownRunningScreen 中实现)
    unsigned long r_h = display_remaining_s / 3600;
    unsigned long r_m = (display_remaining_s % 3600) / 60;
    unsigned long r_s = display_remaining_s % 60;

    lcd.setCursor(0, 0);
    if (countdown_beeping)
    {
        lcd.print("Countdown: END!");
    }
    else if (countdown_running)
    {
        lcd.print("Countdown: RUN ");
    }
    else
    {
        lcd.print("Countdown: PAUSE");
    }

    lcd.setCursor(0, 1);
    formatNumber(0, 1, r_h, 2);
    lcd.print("H ");
    formatNumber(4, 1, r_m, 2);
    lcd.print("M ");
    formatNumber(8, 1, r_s, 2);
    lcd.print("S   "); // 清理后面
}

void handleAlarmSettingMode() {
    if (millis() - setting_mode_entry_time > 30000)
    {
        exitSettingModeAndSave(false, false);
        lcd.setCursor(0, 0);
        lcd.print("Set Timeout");
        delay(1000);
        lcd.clear();
        return;
    }
    bool needs_redraw = false;

    if (checkButtonPress(BUTTON_CHOOSE_PIN, button_last_press_time_choose, BUTTON_DEBOUNCE_DELAY )) {
        setting_mode_entry_time = millis();
        settingStep++;
        if (settingStep > 3) { 
            exitSettingModeAndSave(false, true); 
            return;
        }
        needs_redraw = true;
    }

    if (checkButtonHeld(BUTTON_ADD_PIN, button_last_press_time_add, BUTTON_DEBOUNCE_DELAY)) {
        setting_mode_entry_time = millis(); needs_redraw = true;
        switch (settingStep) {
            case 0: tempSettingAlarmHour = (tempSettingAlarmHour + 1) % 24; break;
            case 1: tempSettingAlarmMinute = (tempSettingAlarmMinute + 1) % 60; break;
            case 2: tempSettingAlarmTemp += 1.0f; if (tempSettingAlarmTemp > 99.0f) tempSettingAlarmTemp = 99.0f; break;
            case 3: currentMusicMode = (currentMusicMode == MusicMode::NORMAL) ? MusicMode::MUSIC : MusicMode::NORMAL; break;
        }
    }

    if (checkButtonHeld(BUTTON_MINUS_PIN, button_last_press_time_minus, BUTTON_DEBOUNCE_DELAY)) {
        setting_mode_entry_time = millis(); needs_redraw = true;
        switch (settingStep) {
            case 0: tempSettingAlarmHour = (tempSettingAlarmHour - 1 + 24) % 24; break;
            case 1: tempSettingAlarmMinute = (tempSettingAlarmMinute - 1 + 60) % 60; break;
            case 2: tempSettingAlarmTemp -= 1.0f; if (tempSettingAlarmTemp < 0.0f) tempSettingAlarmTemp = 0.0f; break;
        }
    }

    if (needs_redraw) {
        displaySetAlarmScreen(tempSettingAlarmHour, tempSettingAlarmMinute, tempSettingAlarmTemp, settingStep);
    }
}


// --- 报警功能 ---
void checkAndTriggerHourlyChime() {
    if (currentTime.minute == 0 && currentTime.second == 0) {
        if (currentTime.hour != (int)hourly_chime_last_triggered_hour) { 
            tone(BUZZER_TONE_PIN, HOURLY_CHIME_FREQ, HOURLY_CHIME_DURATION);
            hourly_chime_last_triggered_hour = currentTime.hour;
        }
    }
}

void checkAndTriggerClockAlarm() {
    if (clock_alarm_active) return; 

    if (currentTime.hour == alarm_hour &&
        currentTime.minute == alarm_minute &&
        currentTime.second == 0) { 
        clock_alarm_active = true;
        clock_alarm_sound_start_time = millis(); 
    } 
}

void handleActiveClockAlarmSound() {
    if (!clock_alarm_active) return;
    #ifdef MUSIC_PLAY
    if(currentMusicMode == MusicMode::MUSIC){
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Alarming!!!");
        playMusic(twinkleMelody, twinkleDurations, twinkleSongLength); // 播放音乐
    }
    else{
        unsigned long current_millis = millis();
        if (current_millis - clock_alarm_sound_start_time > ALARM_DURATION) {
            #ifdef __Serial_DEBUG__
            Serial.println("Alarm duration ended.");
            Serial.print("Alarm start time: ");
            Serial.println(clock_alarm_sound_start_time);
            Serial.print("Current time: ");
            Serial.println(current_millis);
            #endif
            stopClockAlarmSound();
            return;
        }

        if (digitalRead(BUTTON_CHOOSE_PIN) == LOW ||
            digitalRead(BUTTON_ADD_PIN) == LOW ||
            digitalRead(BUTTON_MINUS_PIN) == LOW) {
            stopClockAlarmSound();
            delay(BUTTON_DEBOUNCE_DELAY ); 
            return;
        }

        static unsigned long last_beep_toggle_time = 0;
        unsigned long beep_interval = 500; 

        if (current_millis - last_beep_toggle_time > beep_interval) {
            last_beep_toggle_time = current_millis;
            if (((current_millis - clock_alarm_sound_start_time) / beep_interval) % 2 == 0) {
                tone(BUZZER_TONE_PIN, ALARM_SOUND_FREQ, beep_interval - 50); 
            } else {
                noTone(BUZZER_TONE_PIN); 
            }
        }
    }
    #endif
}

void stopClockAlarmSound() {
    clock_alarm_active = false;
    noTone(BUZZER_TONE_PIN);
}

void checkAndTriggerTemperatureAlarm() {
    if (clock_alarm_active) return; 

    if (currentTemperature_filtered >= temperatureAlarmThreshold) {
        unsigned long current_millis = millis();
        if (current_millis - temp_alarm_last_beep_time > TEMP_ALARM_DURATION * 2) { 
            tone(BUZZER_TONE_PIN, ALARM_SOUND_FREQ, TEMP_ALARM_DURATION);
            temp_alarm_last_beep_time = current_millis;
        }
    }
}

// --- 温度处理 ---
void readAndUpdateTemperature() {
    long adc_val = analogRead(TEMP_SENSOR_PIN);
    float raw_temperature = (500.0 * adc_val) / 1023.0; 
    currentTemperature_filtered = applyKalmanFilter(raw_temperature);
    #if defined(__Serial_DEBUG__) || defined(__KalmanOUTPUT__) 
    Serial.print(raw_temperature, 2); Serial.print(",");
    Serial.println(currentTemperature_filtered, 2);
    #endif
}

float applyKalmanFilter(float measurement) {
    float P_minus = kalman_P + kalman_Q; 
    float K = P_minus / (P_minus + kalman_R); 
    kalman_x_hat = kalman_x_hat + K * (measurement - kalman_x_hat); 
    kalman_P = (1 - K) * P_minus; 
    return kalman_x_hat;
}
#ifdef __KalmanConfig__
void handleKalmanSerialConfig() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        int commaIndex = input.indexOf(',');
        if (commaIndex > 0) {
            String q_str = input.substring(0, commaIndex);
            String r_str = input.substring(commaIndex + 1);
            float new_q = q_str.toFloat();
            float new_r = r_str.toFloat();
            if (new_q > 0 && new_r > 0) { 
                kalman_Q = new_q;
                kalman_R = new_r;
                Serial.print("Kalman Q set to: "); Serial.println(kalman_Q, 6); 
                Serial.print("Kalman R set to: "); Serial.println(kalman_R, 6);
            } else { Serial.println("Invalid Q/R values. Must be positive."); }
        } else { Serial.println("Invalid format. Use: Q_val,R_val (e.g., 0.01,0.1)"); }
    }
}
#endif

// --- 按键辅助函数 ---
bool checkButtonPress(int pin, unsigned long& lastPressTime, unsigned long debounceInterval) {
    unsigned long current_millis = millis();
    if (digitalRead(pin) == LOW) { 
        if (current_millis - lastPressTime > debounceInterval) { 
            lastPressTime = current_millis; 
            return true; 
        }
    }
    return false;
}

bool checkLongButtonPress(int pin, bool& pressStateFlag, unsigned long& pressStartTime, unsigned long thresholdMillis) {
    if (digitalRead(pin) == LOW) {
        if (!pressStateFlag) {
            pressStateFlag = true;
            pressStartTime = millis();
        } else {
            if (millis() - pressStartTime >= thresholdMillis) {
                pressStateFlag = false; // 避免重复触发
                return true; // 长按成功触发
            }
        }
    } else {
        pressStateFlag = false; // 松开按钮，重置状态
    }
    return false;
}


bool checkMultiButtonPress(int pin1, int pin2, unsigned long& lastPressTime, unsigned long debounceInterval) {
    unsigned long current_millis = millis();
    if (digitalRead(pin1) == LOW && digitalRead(pin2) == LOW) { 
        if (current_millis - lastPressTime > debounceInterval) { 
            lastPressTime = current_millis; 
            return true; 
        }
    }
    return false;
    
}

bool checkButtonHeld(int pin, unsigned long& lastActionTime, unsigned long actionInterval) {
     unsigned long current_millis = millis();
    if (digitalRead(pin) == LOW) { 
        if (current_millis - lastActionTime > actionInterval) { 
            lastActionTime = current_millis; 
            return true; 
        }
    }
    return false;
}

#ifdef MUSIC_PLAY
/**
 * @brief 检查按键是否按下，如果按下则设置中止标志
 */
void checkStopButton()
{
    if (digitalRead(stopButtonPin) == LOW)
    { // 按键按下时为低电平
        musicShouldStop = true;
#ifdef __Serial_DEBUG__
        Serial.println("检测到中止按键！");
#endif
    }
}

/**
 * @brief 播放音乐的函数，增加了按键中止功能
 *
 * @param melody 指向存储音符频率的PROGMEM数组的指针
 * @param durations 指向存储音符持续时间的PROGMEM数组的指针
 * @param songLength 歌曲中音符的总数
 * @param tempoFactor 节奏因子，默认为1.0。
 */
void playMusic(const int melody[], const int durations[], int songLength, float tempoFactor = 1.0)
{
    musicShouldStop = false; // 开始播放前，重置中止标志

    for (int i = 0; i < songLength; i++)
    {
        // 在播放每个音符前检查是否需要中止
        checkStopButton(); // 检查物理按键
        if (musicShouldStop)
        {
#ifdef __Serial_DEBUG__
            Serial.println("音乐播放已中止。");
#endif
            noTone(buzzerPin); // 确保停止蜂鸣器
            stopClockAlarmSound();
            return;            // 退出播放函数
        }

        int noteFrequency = pgm_read_word_near(melody + i);
        int noteDuration = pgm_read_word_near(durations + i);
        int actualDuration = noteDuration / tempoFactor;

        if (noteFrequency == NOTE_REST)
        {
            noTone(buzzerPin);
            // 即使是休止符，也要检查按键，以便能快速响应中止
            long startTime = millis();
            while (millis() - startTime < actualDuration)
            {
                checkStopButton();
                if (musicShouldStop)
                {
#ifdef __Serial_DEBUG__
                    Serial.println("音乐播放已中止（在休止符期间）。");
#endif
                    noTone(buzzerPin);
                    stopClockAlarmSound();
                    return;
                }
                delay(10); // 短暂延时，避免过于频繁地检查，给其他处理留出时间（如果需要）
            }
        }
        else
        {
            tone(buzzerPin, noteFrequency, actualDuration * 0.9);
            // 在音符播放期间也允许中止
            long startTime = millis();
            while (millis() - startTime < actualDuration)
            {
                checkStopButton();
                if (musicShouldStop)
                {
#ifdef __Serial_DEBUG__
                    Serial.println("音乐播放已中止（在音符播放期间）。");
#endif
                    noTone(buzzerPin);
                    stopClockAlarmSound();
                    return;
                }
                delay(10); // 短暂延时
            }
            // 确保在下一个音符或检查前，当前音符的tone效果已结束（如果tone的第三个参数未完全覆盖actualDuration）
            // 或者为了安全，可以主动调用noTone，但tone的第三个参数应该已经处理了时长
            // noTone(buzzerPin); // 可选，通常tone(pin, freq, duration)会自动停止
        }
    }
    noTone(buzzerPin); // 播放完毕后确保关闭蜂鸣器
#ifdef __Serial_DEBUG__
    Serial.println("音乐正常播放完毕。");
#endif
}
#endif

#endif

#ifdef MAIN2
#include <Arduino.h>
#include "Config.h"
// 定义蜂鸣器连接的引脚
const int buzzerPin = BUZZER_TONE_PIN;
// 定义按键连接的引脚
const int stopButtonPin = BUTTON_CHOOSE_PIN;

// 乐谱数据 (PROGMEM)
// ...《小星星》的乐谱 ...
const int twinkleMelody[] PROGMEM = {
    NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
    NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4,
    NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4,
    NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4,
    NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
    NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4};

const int twinkleDurations[] PROGMEM = {
    500, 500, 500, 500, 500, 500, 1000,
    500, 500, 500, 500, 500, 500, 1000,
    500, 500, 500, 500, 500, 500, 1000,
    500, 500, 500, 500, 500, 500, 1000,
    500, 500, 500, 500, 500, 500, 1000,
    500, 500, 500, 500, 500, 500, 1000};

const int twinkleSongLength = sizeof(twinkleMelody) / sizeof(int);

// 全局变量，用于标记音乐是否应被中止
volatile bool musicShouldStop = false; // 使用 volatile 是因为可能在中断中使用（虽然本例不是）
                                       // 但对于在主循环和函数间共享的、可能意外改变的标志，这是个好习惯

/**
 * @brief 检查按键是否按下，如果按下则设置中止标志
 */
void checkStopButton()
{
    if (digitalRead(stopButtonPin) == LOW)
    { // 按键按下时为低电平
        musicShouldStop = true;
#ifdef __Serial_DEBUG__
        Serial.println("检测到中止按键！");
#endif
    }
}

/**
 * @brief 播放音乐的函数，增加了按键中止功能
 *
 * @param melody 指向存储音符频率的PROGMEM数组的指针
 * @param durations 指向存储音符持续时间的PROGMEM数组的指针
 * @param songLength 歌曲中音符的总数
 * @param tempoFactor 节奏因子，默认为1.0。
 */
void playMusic(const int melody[], const int durations[], int songLength, float tempoFactor = 1.0)
{
    musicShouldStop = false; // 开始播放前，重置中止标志

    for (int i = 0; i < songLength; i++)
    {
        // 在播放每个音符前检查是否需要中止
        checkStopButton(); // 检查物理按键
        if (musicShouldStop)
        {
#ifdef __Serial_DEBUG__
            Serial.println("音乐播放已中止。");
#endif
            noTone(buzzerPin); // 确保停止蜂鸣器
            return;            // 退出播放函数
        }

        int noteFrequency = pgm_read_word_near(melody + i);
        int noteDuration = pgm_read_word_near(durations + i);
        int actualDuration = noteDuration / tempoFactor;

        if (noteFrequency == NOTE_REST)
        {
            noTone(buzzerPin);
            // 即使是休止符，也要检查按键，以便能快速响应中止
            long startTime = millis();
            while (millis() - startTime < actualDuration)
            {
                checkStopButton();
                if (musicShouldStop)
                {
#ifdef __Serial_DEBUG__
                    Serial.println("音乐播放已中止（在休止符期间）。");
#endif
                    noTone(buzzerPin);
                    return;
                }
                delay(10); // 短暂延时，避免过于频繁地检查，给其他处理留出时间（如果需要）
            }
        }
        else
        {
            tone(buzzerPin, noteFrequency, actualDuration * 0.9);
            // 在音符播放期间也允许中止
            long startTime = millis();
            while (millis() - startTime < actualDuration)
            {
                checkStopButton();
                if (musicShouldStop)
                {
#ifdef __Serial_DEBUG__
                    Serial.println("音乐播放已中止（在音符播放期间）。");
#endif
                    noTone(buzzerPin);
                    return;
                }
                delay(10); // 短暂延时
            }
            // 确保在下一个音符或检查前，当前音符的tone效果已结束（如果tone的第三个参数未完全覆盖actualDuration）
            // 或者为了安全，可以主动调用noTone，但tone的第三个参数应该已经处理了时长
            // noTone(buzzerPin); // 可选，通常tone(pin, freq, duration)会自动停止
        }
    }
    noTone(buzzerPin); // 播放完毕后确保关闭蜂鸣器
#ifdef __Serial_DEBUG__
    Serial.println("音乐正常播放完毕。");
#endif
}

void setup()
{
#ifdef __Serial_DEBUG__
    Serial.begin(9600);
    Serial.println("音乐播放器（带中止功能）初始化完毕！");
#endif

    pinMode(buzzerPin, OUTPUT);
    // 设置按键引脚为输入上拉模式
    // 当按键未按下时，引脚读到HIGH；按下时，引脚读到LOW
    pinMode(stopButtonPin, INPUT_PULLUP);

#ifdef __Serial_DEBUG__
    Serial.println("将在1秒后开始播放《小星星》，您可以随时按下按键中止。");
#endif
    delay(1000);

    playMusic(twinkleMelody, twinkleDurations, twinkleSongLength);

    // 如果需要，可以在这里添加其他逻辑，比如提示用户按另一个键重新播放
}

void loop()
{
    // loop 函数可以留空，或者用于实现其他功能，例如：
    // - 检测一个“播放”按钮来重新开始播放音乐
    // - 显示当前状态等

    // 示例：如果想让按键也能触发重新播放（当前代码是只中止）
    // if (digitalRead(somePlayButtonPin) == LOW && !isPlaying) { // isPlaying 是一个你需要自己管理的状态变量
    //   Serial.println("重新播放音乐...");
    //   playMusic(twinkleMelody, twinkleDurations, twinkleSongLength);
    // }
    delay(100); // 在loop中加入少量延时是个好习惯，避免CPU空转过快
}
#endif