
#if defined(INITIAL)
#include <Arduino.h>
#include <DS1302.h>
#include <LiquidCrystal.h> // LCD1602 显示头文件
#include "Config.h"      // 配置头文件

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
#define BUTTON_DEBOUNCE_DELAY 70 // ms, for button debouncing
#define DEFAULT_ALARM_TEMP 40.0f // Default temperature alarm threshold
#define HOURLY_CHIME_FREQ 2093   // Hz, frequency for hourly chime
#define ALARM_SOUND_FREQ 2093    // Hz, frequency for alarm sound
#define ALARM_DURATION 5000      // ms, duration for custom alarm
#define HOURLY_CHIME_DURATION 500 // ms, duration for hourly chime
#define TEMP_ALARM_DURATION 500   // ms, duration for temperature alarm beep

#endif // CONFIG_H

// LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
// lcd 初始化函数
LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

// ==== 全局变量与对象定义 ====
#define choose BUTTON_CHOOSE_PIN                     // 选择端口
#define add BUTTON_ADD_PIN                        // 加
#define minus BUTTON_MINUS_PIN                      // 减
#define Tone BUZZER_TONE_PIN                       // 蜂鸣器端口

uint8_t CE_PIN = DS1302_CE_PIN;            // DS1302 RST 端口
uint8_t IO_PIN = DS1302_IO_PIN;                   // DS1302 DAT 端口
uint8_t SCLK_PIN = DS1302_SCLK_PIN;                // DS1302 CLK 端口
DS1302 rtc(CE_PIN, IO_PIN, SCLK_PIN); // 创建 DS1302 对象
unsigned long seconds;
int s = 0, m = 0, h = 0, d = 0, mon = 0, y = 0;                     // 时间进位
int second_val = 0, minute_val = 0, hour_val = 0, day_val = 0, month_val = 0, year_val = 0; // 当前时间 (避免与库函数冲突，重命名)
int SECOND = 0, MINUTE = 0, HOUR = 0, DAY = 0, MONTH = 0, YEAR = 0; // 初始时间
int chose = 0, alarm_choose = 0, ButtonDelay = BUTTON_DEBOUNCE_DELAY, frequence = HOURLY_CHIME_FREQ;
int alarm_hour = 7, alarm_minute = 30, alarm_second = 0; // 闹钟时间
float Temperatures, Temp_Alarm = DEFAULT_ALARM_TEMP;

// ==== 非阻塞延时 ====
static unsigned long blink_lastTime = 0;
static bool blink_cursorState = false; // false = 隐藏, true = 显示
static int blink_currentRol = -1;      // 当前光标列，-1表示未激活
static int blink_currentRow = -1;      // 当前光标行

const unsigned int CURSOR_BLINK_INTERVAL = 500; // 光标状态切换间隔 (500ms亮, 500ms灭)

// ==== 卡尔曼滤波器参数 ====
float kalman_x_hat = 0; // 对当前温度的估计值 (k-1 时刻的后验估计)
float kalman_P = 1.0;   // 估计误差协方差 (k-1 时刻的后验误差协方差)
float kalman_Q = 0.0001; // 过程噪声协方差
float kalman_R = 0.09;  // 测量噪声协方差

// ==== 函数声明 ====
void FormatDisplay(int col, int row, int num);        // 格式化输出时间/日期数字（不足两位补零）
void time_display();                                  // 时间计算与显示函数
int Days(int current_year, int current_month);        // 计算指定年月的天数
void Day_display();                                   // 计算并显示当前日期
void Month_display();                                 // 计算并显示当前月份
void Year_display();                                  // 计算并显示当前年份
void Week(int y_val, int m_val, int d_val);           // 根据年月日计算星期几并显示
void Display();                                       // 显示全部时间、日期和星期
void DisplayCursor(int rol, int row);                 // 显示闪烁光标，用于提示设置位置
void set(int y_val, int mon_val, int d_val, int h_val, int m_val, int s_val); // 设置初始时间
void Set_Time_Value(int rol, int row, int &Time_var); // 通过按键设置具体时间值
void Set_Clock();                                     // 设置完整时间（小时、分钟、秒、日期、月份、年份）
void Set_Alarm_Hour();                                // 设置闹钟小时
void Set_Alarm_Minute();                              // 设置闹钟分钟
void Set_Alarm_Temp();                                // 设置报警温度阈值
void Set_Alarm();                                     // 设置闹钟（时间 + 温度）
void Point_Time_Alarm();                              // 整点报时蜂鸣
void Clock_Alarm();                                   // 闹钟时间蜂鸣
void GetTemperatures();                               // 获取 LM35 温度传感器数值
void Temperatures_Alarm();                            // 超过设定温度时蜂鸣报警
float applyKalmanFilter(float measurement);           // 应用卡尔曼滤波器
void kalmanProcess();                                 // 卡尔曼滤波参数调整

void setup()
{
    for (int i = 2; i <= 13; i++)
    {
        pinMode(i, OUTPUT);
    }
    pinMode(add, INPUT_PULLUP); // 使用内部上拉电阻
    pinMode(minus, INPUT_PULLUP);
    pinMode(choose, INPUT_PULLUP);

    lcd.begin(16, 2); // 初始化 LCD1602
    rtc.writeProtect(false); // 关闭 DS1302 芯片写保护
    rtc.halt(false);         // 为 true 时 DS1302 暂停
    Time t;
    t = rtc.getTime();
    set(t.year, t.mon, t.date, t.hour, t.min, t.sec); // 设置 DS1302 芯片初始时间

    // 初始化卡尔曼滤波器的初始状态
    long initial_adc_val = analogRead(A3); // 获取一个初始温度读数
    kalman_x_hat = (500.0 * initial_adc_val) / 1023.0; // 用第一次读数作为初始估计

    Serial.begin(9600); // 初始化串口，用于调试卡尔曼参数
}

void loop()
{
    seconds = millis() / 1000; // 获取单片机当前运行时间
    Display();                 // 显示时间
    Set_Clock();               // 设置时间
    Set_Alarm();               // 设置闹钟
    Point_Time_Alarm();        // 正点蜂鸣
    Clock_Alarm();             // 闹钟时间蜂鸣
    GetTemperatures();         // 获取 LM35 温度
    Temperatures_Alarm();      // 超过指定温度报警
    kalmanProcess();
    
    // 将计算出的当前时间写到 DS1302 芯片中
    rtc.setTime(hour_val, minute_val, second_val);
    rtc.setDate(year_val, month_val, day_val); // 修正: 根据DS1302.h setDate(int year, int mon, day)

    delay(100); // 减慢循环速度，便于观察和调试，实际应用中可能不需要这么大
}

// 卡尔曼滤波参数动态调整
void kalmanProcess() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n'); // 读取输入
        input.trim(); // 去除首尾空格

        int commaIndex = input.indexOf(','); // 查找逗号位置
        if (commaIndex > 0) {
            String qValue = input.substring(0, commaIndex); // 获取逗号前的部分
            String rValue = input.substring(commaIndex + 1); // 获取逗号后的部分

            float newQ = qValue.toFloat();
            float newR = rValue.toFloat();

            if (newQ > 0 && newR > 0) { // 确保Q和R为正数
                kalman_Q = newQ;
                kalman_R = newR;
                Serial.print("Updated kalman_Q to: ");
                Serial.println(kalman_Q);
                Serial.print("Updated kalman_R to: ");
                Serial.println(kalman_R);
            } else {
                Serial.println("Invalid values. Q and R must be positive numbers.");
            }
        } else {
            Serial.println("Invalid format. Use '<Q>,<R>' format (e.g., '0.01,0.09').");
        }
    }
}

/** 应用卡尔曼滤波器 */
float applyKalmanFilter(float measurement) {
    // --- 预测步骤 ---
    float P_minus = kalman_P + kalman_Q;

    // --- 更新（校正）步骤 ---
    float K = P_minus / (P_minus + kalman_R);

    kalman_x_hat = kalman_x_hat + K * (measurement - kalman_x_hat); // 使用上一时刻的x_hat作为先验估计
    kalman_P = (1 - K) * P_minus;

    return kalman_x_hat;
}

/** 格式化输出 */
void FormatDisplay(int col, int row, int num)
{
    lcd.setCursor(col, row);
    if (num < 10)
        lcd.print("0");
    lcd.print(num);
}

/** 时间计算与显示 */
void time_display() 
{
    second_val = (SECOND + seconds) % 60; // 计算秒
    m = (SECOND + seconds) / 60;      // 分钟进位
    FormatDisplay(6, 1, second_val);
    minute_val = (MINUTE + m) % 60; // 计算分钟
    h = (MINUTE + m) / 60;      // 小时进位
    FormatDisplay(3, 1, minute_val);
    hour_val = (HOUR + h) % 24; // 计算小时
    d = (HOUR + h) / 24;    // 天数进位
    FormatDisplay(0, 1, hour_val);
    lcd.setCursor(2, 1);
    lcd.print(":");
    lcd.setCursor(5, 1);
    lcd.print(":");
}
/** 根据年月计算当月天数 */
int Days(int current_year, int current_month)
{
    int days_in_month = 0;
    if (current_month != 2)
    {
        switch (current_month)
        {
        case 1: case 3: case 5: case 7: case 8: case 10: case 12:
            days_in_month = 31; break;
        case 4: case 6: case 9: case 11:
            days_in_month = 30; break;
        }
    }
    else
    { // 闰年
        if ((current_year % 4 == 0 && current_year % 100 != 0) || current_year % 400 == 0)
            days_in_month = 29;
        else
            days_in_month = 28;
    }
    return days_in_month;
}
/** 计算当月天数 */
void Day_display() 
{
    int days_in_current_month = Days(year_val, month_val); // 当前月的天数
    int days_in_prev_month;
    if (month_val == 1)
        days_in_prev_month = Days(year_val - 1, 12);
    else
        days_in_prev_month = Days(year_val, month_val - 1);

    day_val = DAY + d; // 计算当前日期
    while (day_val > days_in_current_month) {
        day_val -= days_in_current_month;
        mon++; // 增加月份
        month_val = (MONTH + mon -1) % 12 + 1;
        year_val = YEAR + (MONTH + mon -1) / 12;
        days_in_current_month = Days(year_val, month_val);
    }
    while (day_val <= 0) { // 处理日期可能为0或负数的情况
       mon--; // 减少月份进位
       month_val = (MONTH + mon -1) % 12 + 1;
       if (month_val == 0) month_val = 12; // 月份应为1-12
       year_val = YEAR + (MONTH + mon -1) / 12;
       if (month_val == 12 && (MONTH + mon -1) < 0) year_val--; // 如果月份向后回绕到上一年，调整年份

       if (month_val == 1 && DAY + d <=0) { // 如果回绕到上一年的12月
           days_in_prev_month = Days(year_val, 12);
       } else if (DAY + d <=0) {
           days_in_prev_month = Days(year_val, month_val); // 或当前月份未回绕到上一年
       } else {
           days_in_prev_month = Days(year_val, month_val-1 == 0 ? 12 : month_val-1); // 或前一个月
       }
       day_val += days_in_prev_month;
    }

    FormatDisplay(8, 0, day_val);
}

/** 计算月份 */
void Month_display() 
{
    month_val = (MONTH + mon -1) % 12 + 1;
    if ((MONTH + mon -1) < 0 && month_val !=12 ) { //处理负数取模的情况
        int temp_mon = (MONTH + mon -1);
        while(temp_mon < 0) temp_mon += 12;
        month_val = temp_mon % 12 + 1;
    }
    y = (MONTH + mon - 1) / 12; // 计算年份进位
    FormatDisplay(5, 0, month_val);
    lcd.setCursor(7, 0);
    lcd.print('-');
}

/** 计算年份 */
void Year_display() 
{
    year_val = YEAR + y;
    lcd.setCursor(0, 0);
    if (year_val < 1000) lcd.print("0");
    if (year_val < 100) lcd.print("0");
    if (year_val < 10) lcd.print("0");
    lcd.print(year_val);
    lcd.setCursor(4, 0);
    lcd.print('-');
}

/** 根据年月日计算星期几 */
void Week(int y_val, int m_val, int d_val)
{
    int Eff_m = m_val;
    int Eff_y = y_val;
    if (Eff_m == 1 || Eff_m == 2) { // Zeller算法
        Eff_m += 12;
        Eff_y--;
    }
    int week_day_zeller = (d_val + (13 * (Eff_m + 1)) / 5 + Eff_y % 100 + (Eff_y % 100) / 4 + (Eff_y / 100) / 4 - 2 * (Eff_y / 100)) % 7;
    // Zeller给出的结果是0代表星期六，1代表星期日，...
    int temp_m = m_val;
    int temp_y = y_val; // 根据原始 Week 函数，这里不改变年份
    if (temp_m == 1) temp_m = 13;
    if (temp_m == 2) temp_m = 14;
    // 如果将1月和2月的月份调整为13和14，则年份应为 y-1（适用于 Zeller 算法）。
    // 原始 `Week` 函数在月份变为13或14时并未调整年份。
    // 这种做法对于 Zeller 算法来说不常见。如果原公式有效，则可能是特定公式版本的特性。
    // 这里直接使用原始公式。

    int original_m_for_week = m_val;
    if (original_m_for_week == 1) original_m_for_week = 13; // 对于此公式版本，不改变年份
    if (original_m_for_week == 2) original_m_for_week = 14;

    int week = (d_val + 2 * original_m_for_week + 3 * (original_m_for_week + 1) / 5 + y_val + y_val / 4 - y_val / 100 + y_val / 400) % 7 + 1; // 计算星期几
    String weekstr = "";
    switch (week)
    {
    case 1: weekstr = "Mon. "; break;
    case 2: weekstr = "Tues."; break;
    case 3: weekstr = "Wed. "; break;
    case 4: weekstr = "Thur."; break;
    case 5: weekstr = "Fri. "; break;
    case 6: weekstr = "Sat. "; break;
    case 7: weekstr = "Sun. "; break;
    default: weekstr = "Err. "; break;
    }
    lcd.setCursor(11, 0);
    lcd.print(weekstr);
}

/** 显示时间、日期、星期 */
void Display()
{
    time_display(); // 显示时间
    Year_display(); // 显示年份
    Month_display(); // 显示月份
    Day_display();  // 显示日期
    Week(year_val, month_val, day_val); // 显示星期
}

/** 显示光标 */
void DisplayCursor(int rol, int row, int N)
{   
    lcd.setCursor(rol, row);
    for(int i = 0; i < N; i++) {
        lcd.print(" ");
    }
}


/** 设置初始时间 */
void set(int y_val, int mon_val, int d_val, int h_val, int m_val, int s_val)
{
    YEAR = y_val;
    MONTH = mon_val;
    DAY = d_val;
    HOUR = h_val;
    MINUTE = m_val;
    SECOND = s_val;

    year_val = YEAR;
    month_val = MONTH;
    day_val = DAY;
    hour_val = HOUR;
    minute_val = MINUTE;
    second_val = SECOND;
}

/** 通过按键设置时间 */
void Set_Time_Value(int rol, int row, int &Time_var)
{
    DisplayCursor(rol, row);
    if (digitalRead(add) == LOW)
    {
        delay(ButtonDelay);
        if (digitalRead(add) == LOW)
        {
            Time_var++;
        }
        Display();
    }
    if (digitalRead(minus) == LOW)
    {
        delay(ButtonDelay);
        if (digitalRead(minus) == LOW)
        {
            Time_var--;
        }
        Display();
    }
}

/** 设置时间 */
void Set_Clock()
{
    if (digitalRead(choose) == LOW)
    {
        delay(ButtonDelay);
        if (digitalRead(choose) == LOW) {
            lcd.setCursor(9, 1);
            lcd.print("SetTime");
            
            chose = 0; // 重置设置选项

            unsigned long entryTime = millis();
            bool settingActive = true;

            while (settingActive)
            {
                if (millis() - entryTime > 15000 && chose == 0) {
                    settingActive = false;
                    lcd.setCursor(9,1); lcd.print("TimeOut");
                    break;
                }

                if (digitalRead(choose) == LOW)
                {
                    delay(ButtonDelay);
                    if (digitalRead(choose) == LOW)
                    {
                        chose++;
                        entryTime = millis(); // 重置超时
                        if (chose >= 7) {
                            settingActive = false;
                            chose = 0;
                        }
                    }
                }
                
                unsigned long current_millis_sec = millis()/1000;
                if (seconds != current_millis_sec) { 
                    seconds = current_millis_sec;
                    Display();
                    if(settingActive && chose == 0) {lcd.setCursor(9, 1); lcd.print("SetTime");}
                }

                switch(chose) {
                    case 1: Set_Time_Value(0, 1, HOUR); break;
                    case 2: Set_Time_Value(3, 1, MINUTE); break;
                    case 3: Set_Time_Value(6, 1, SECOND); break;
                    case 4: Set_Time_Value(8, 0, DAY); break;
                    case 5: Set_Time_Value(5, 0, MONTH); break;
                    case 6: Set_Time_Value(0, 0, YEAR); break;
                    default: break;
                }
                if (!settingActive) break;
                delay(50);
            }
            seconds = millis()/1000;
            s = 0; m = 0; h = 0; d = 0; mon = 0; y = 0;
            rtc.setTime(hour_val, minute_val, second_val);
            rtc.setDate(year_val, month_val, day_val);

            lcd.setCursor(9,1); lcd.print("        ");
            Display();
            delay(200);
        }
    }
}

/** 设置闹钟小时 */
void Set_Alarm_Hour()
{
    DisplayCursor(0, 1);
    if (digitalRead(add) == LOW)
    {
        delay(ButtonDelay);
        if (digitalRead(add) == LOW)
        {
            alarm_hour = (alarm_hour + 1) % 24;
            FormatDisplay(0, 1, alarm_hour);
        }
    }
    if (digitalRead(minus) == LOW)
    {
        delay(ButtonDelay);
        if (digitalRead(minus) == LOW)
        {
            alarm_hour = (alarm_hour - 1 + 24) % 24;
            FormatDisplay(0, 1, alarm_hour);
        }
    }
}

/** 设置闹钟分钟 */
void Set_Alarm_Minute()
{
    DisplayCursor(3, 1);
    if (digitalRead(add) == LOW)
    {
        delay(ButtonDelay);
        if (digitalRead(add) == LOW)
        {
            alarm_minute = (alarm_minute + 1) % 60;
            FormatDisplay(3, 1, alarm_minute);
        }
    }
    if (digitalRead(minus) == LOW)
    {
        delay(ButtonDelay);
        if (digitalRead(minus) == LOW)
        {
            alarm_minute = (alarm_minute - 1 + 60) % 60;
            FormatDisplay(3, 1, alarm_minute);
        }
    }
}

/** 设置报警温度 */
void Set_Alarm_Temp()
{
    DisplayCursor(9, 1);
    if (digitalRead(add) == LOW)
    {
        delay(ButtonDelay);
        if (digitalRead(add) == LOW) {
            Temp_Alarm++;
            if (Temp_Alarm > 99) Temp_Alarm = 99;
            lcd.setCursor(9,1); if(Temp_Alarm < 10 && Temp_Alarm >=0) lcd.print("0"); lcd.print(Temp_Alarm,0); 
        }
    }
    if (digitalRead(minus) == LOW)
    {
        delay(ButtonDelay);
        if (digitalRead(minus) == LOW) {
            Temp_Alarm--;
            if (Temp_Alarm < 0) Temp_Alarm = 0;
            lcd.setCursor(9,1); if(Temp_Alarm < 10 && Temp_Alarm >=0) lcd.print("0"); lcd.print(Temp_Alarm,0);
        }
    }
}

/** 进入报警设置 */
void Set_Alarm()
{
    if (digitalRead(add) == LOW && digitalRead(minus) == LOW)
    {
        delay(ButtonDelay*2);
        if (digitalRead(add) == LOW && digitalRead(minus) == LOW) {
            alarm_choose = 0;

            lcd.clear();
            lcd.setCursor(0, 0); lcd.print("Set Alarm:");
            FormatDisplay(0, 1, alarm_hour); lcd.print(":"); FormatDisplay(3, 1, alarm_minute); lcd.print(":00");
            lcd.setCursor(9,1); if(Temp_Alarm < 10 && Temp_Alarm >=0) lcd.print("0"); lcd.print(Temp_Alarm,0); lcd.print((char)223); lcd.print("C");

            unsigned long entryTime = millis();
            bool settingActive = true;

            while (settingActive)
            {
                 if (millis() - entryTime > 15000 && alarm_choose == 0) { 
                    settingActive = false;
                    break;
                }

                if (digitalRead(choose) == LOW)
                {
                    delay(ButtonDelay);
                    if (digitalRead(choose) == LOW)
                    {
                        alarm_choose++;
                        entryTime = millis(); // 重置超时
                        if (alarm_choose >= 4) {
                            settingActive = false;
                            alarm_choose = 0;
                        }
                    }
                }

                switch(alarm_choose) {
                    case 1: Set_Alarm_Hour(); break;
                    case 2: Set_Alarm_Minute(); break;
                    case 3: Set_Alarm_Temp(); break;
                    default: break;
                }
                if(!settingActive) break;
                delay(50);
            }
            lcd.clear(); 
            Display();
            delay(200);
        }
    }
}

/** 正点蜂鸣 */
void Point_Time_Alarm()
{
    if (minute_val == 0 && second_val == 0 && hour_val != HOUR) 
    {
        tone(Tone, frequence);
        delay(500);
        noTone(Tone);
    }
}

/** 闹钟蜂鸣 */
void Clock_Alarm()
{
    if (hour_val == alarm_hour && minute_val == alarm_minute && second_val == alarm_second)
    {
        unsigned long alarmStartTime = millis();
        while(millis() - alarmStartTime < 5000) { 
            tone(Tone, frequence, 250); 
            delay(500);
            if(digitalRead(choose) == LOW || digitalRead(add) == LOW || digitalRead(minus) == LOW) { 
                noTone(Tone);
                delay(ButtonDelay); 
                break;
            }
        }
        noTone(Tone);
    }
}

/** 获取 LM35 温度 */
void GetTemperatures()
{
    long a = analogRead(A3); 
    Temperatures = (500.0 * a) / 1023.0;

    float filteredTemperature = applyKalmanFilter(Temperatures);

    lcd.setCursor(9, 1);
    if (filteredTemperature < 0 && filteredTemperature > -10) lcd.print("-0");
    else if (filteredTemperature >= 0 && filteredTemperature < 10) lcd.print("0");
    
    lcd.print(filteredTemperature, 1); 

    lcd.setCursor(14, 1); 
    lcd.print((char)223); 
    lcd.setCursor(15, 1);
    lcd.print("C");

    // Serial print for tuning Kalman Filter
    
    // Serial.print(kalman_Q);
    // Serial.print(',');
    // Serial.print(kalman_R, 3);
    // Serial.print(',');
    // Serial.print("Raw: ");
    // Serial.print(Temperatures, 2);
    // Serial.print("\tFiltered: "); 
    // Serial.print(',');
    // Serial.println(filteredTemperature, 2);
}

/** 超过指定温度报警 */
void Temperatures_Alarm()
{
    if (kalman_x_hat >= Temp_Alarm) 
    {
        tone(Tone, frequence, 250);
        delay(500); 
        noTone(Tone);
    }
}
#endif


#if defined(NOW)
#include <Arduino.h>
#include <DS1302.h>
#include <LiquidCrystal.h> // LCD1602 显示头文件
#include "Config.h"      // 配置文件, 包含所有 #define

// LCD 初始化
LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

// DS1302 RTC 对象
DS1302 rtc(DS1302_CE_PIN, DS1302_IO_PIN, DS1302_SCLK_PIN);

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

// 闹钟设置
int alarm_hour = 7;
int alarm_minute = 30;
float temperatureAlarmThreshold = DEFAULT_ALARM_TEMP;

// 系统状态
enum class SystemMode { NORMAL, SET_TIME, SET_ALARM };
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
unsigned long button_last_press_time_add = 0;
unsigned long button_last_press_time_minus = 0;
unsigned long setting_mode_entry_time = 0;

unsigned long hourly_chime_last_triggered_hour = 25; // 用于整点报时 (初始值确保第一次能响)
unsigned long clock_alarm_sound_start_time = 0;
bool clock_alarm_active = false;
unsigned long temp_alarm_last_beep_time = 0;


// ==== 函数声明 ====
void readAndUpdateTimeFromRTC();
void calculateDayOfWeek(DateTime& dt);
String getDayOfWeekString(int dow);
int daysInMonth(int year, int month);

void displayTimeScreen();
void displaySetTimeScreen(const DateTime& tempTime, int step);
void displaySetAlarmScreen(int tempHour, int tempMin, float tempAlarmTemp, int step);
void formatNumber(int col, int row, int num, int digits = 2);
void printFloat(int col, int row, float val, int decimalPlaces, int totalDigitsBeforeDecimal);


void handleNormalMode();
void handleTimeSettingMode();
void handleAlarmSettingMode();
void enterTimeSettingMode();
void enterAlarmSettingMode();
void exitSettingModeAndSave(bool saveTimeToRTC, bool saveAlarm);

void checkAndTriggerHourlyChime();
void checkAndTriggerClockAlarm();
void handleActiveClockAlarmSound();
void stopClockAlarmSound();
void checkAndTriggerTemperatureAlarm();

void readAndUpdateTemperature();
float applyKalmanFilter(float measurement);
void handleKalmanSerialConfig();

bool checkButtonPress(int pin, unsigned long& lastPressTime, unsigned long debounceInterval);
bool checkButtonHeld(int pin, unsigned long& lastActionTime, unsigned long actionInterval);


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

    Serial.begin(9600);
    lcd.begin(16, 2);

    rtc.writeProtect(false);
    rtc.halt(false);

    Time t = rtc.getTime();
    if (t.year < 2024 || t.year > 2099) { // 检查RTC时间是否有效
        Serial.println("RTC not set or invalid. Setting default time.");
        rtc.setTime(12, 0, 0);        // 12:00:00
        rtc.setDate(2024, 5, 16);     // 2024-May-16 (YYYY, MM, DD)
        // rtc.setDOW(FRIDAY); // 如果库支持设置星期 (FRIDAY通常是宏定义)
    }
    readAndUpdateTimeFromRTC(); // 从RTC获取初始时间

    long initial_adc_val = analogRead(TEMP_SENSOR_PIN);
    kalman_x_hat = (500.0 * initial_adc_val) / 1023.0; // 初始化卡尔曼滤波器估计值
    currentTemperature_filtered = kalman_x_hat;

    hourly_chime_last_triggered_hour = currentTime.hour; // 避免启动时立即报时

    lcd.clear();
}

void loop() {
    unsigned long current_millis = millis(); // 获取一次，减少重复调用

    if (currentMode == SystemMode::NORMAL) {
        handleNormalMode();
    } else if (currentMode == SystemMode::SET_TIME) {
        handleTimeSettingMode();
    } else if (currentMode == SystemMode::SET_ALARM) {
        handleAlarmSettingMode();
    }

    if (clock_alarm_active) {
        handleActiveClockAlarmSound();
    }
    
    handleKalmanSerialConfig(); // 任何模式下都可以调整卡尔曼参数

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
    // currentTime.dayOfWeek = t.dow; // 如果DS1302库直接提供星期几 (1-7 or 0-6)
    calculateDayOfWeek(currentTime); // 手动计算星期
}

void calculateDayOfWeek(DateTime& dt) { // Zeller's congruence or similar
    int d = dt.day;
    int m = dt.month;
    int y = dt.year;
    if (m < 3) {
        m += 12;
        y--;
    }
    int k = y % 100;
    int j = y / 100;
    // Tomihiko Sakamoto's method: (0=Sun, 1=Mon, ..., 6=Sat)
    // dt.dayOfWeek = (d + m*2 + (3*(m+1))/5 + y + y/4 - y/100 + y/400) % 7;
    // The formula from your original code (+1 to make it 1=Mon ... 7=Sun)
    int temp_m = dt.month;
    if (temp_m == 1) temp_m = 13; // Original code does not change year here.
    if (temp_m == 2) temp_m = 14;
    dt.dayOfWeek = (d + 2 * temp_m + 3 * (temp_m + 1) / 5 + dt.year + dt.year / 4 - dt.year / 100 + dt.year / 400) % 7 + 1;
}

String getDayOfWeekString(int dow) { // 1 (Mon) to 7 (Sun)
    switch (dow) {
        case 1: return "Mon"; case 2: return "Tue"; case 3: return "Wed";
        case 4: return "Thu"; case 5: return "Fri"; case 6: return "Sat";
        case 7: return "Sun"; default: return "Err";
    }
}

int daysInMonth(int year, int month) {
    if (month == 2) {
        return ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) ? 29 : 28;
    } else if (month == 4 || month == 6 || month == 9 || month == 11) {
        return 30;
    }
    return 31;
}

// --- 显示函数 ---
void formatNumber(int col, int row, int num, int digits) {
    lcd.setCursor(col, row);
    String s = String(num);
    for (int i = 0; i < digits - s.length(); i++) {
        lcd.print("0");
    }
    lcd.print(s);
}

void printFloat(int col, int row, float val, int decimalPlaces, int totalDigitsBeforeDecimal) {
    lcd.setCursor(col, row);
    // Handle padding for digits before decimal if needed (simplification here)
    // Example: if val is 3.5 and totalDigitsBeforeDecimal is 2, print "03.5"
    if (val >= 0 && val < pow(10, totalDigitsBeforeDecimal -1) ) {
        for(int i=0; i < (totalDigitsBeforeDecimal - String(int(val)).length()); ++i) {
            // This logic for leading zeros before decimal needs more care
        }
    }
    // For temperature -0.x to -9.x
    if (val < 0 && val > -10) {
        String tempStr = String(val, decimalPlaces); // "-X.Y"
        if (abs(int(val)) < 10 && tempStr.charAt(1) != '0' && tempStr.indexOf('.') == 2) { // e.g. -3.5, not -0.5 or -13.5
             lcd.print("-"); // print minus
             lcd.print("0"); // print zero
             lcd.print(abs(val), decimalPlaces); // print abs value
             return;
        }
    } else if (val >= 0 && val < 10) {
        lcd.print("0");
    }
    lcd.print(val, decimalPlaces);
}


void displayTimeScreen() {
    static unsigned long last_display_time = 0;
    if (millis() - last_display_time < 200 && last_display_time != 0) return; // 限制刷新率
    last_display_time = millis();

    // Line 0: YYYY-MM-DD DOW
    formatNumber(0, 0, currentTime.year, 4); lcd.print("-");
    formatNumber(5, 0, currentTime.month, 2); lcd.print("-");
    formatNumber(8, 0, currentTime.day, 2);
    lcd.setCursor(11, 0); lcd.print(getDayOfWeekString(currentTime.dayOfWeek)); lcd.print(" ");

    // Line 1: HH:MM:SS TEMP°C
    formatNumber(0, 1, currentTime.hour, 2); lcd.print(":");
    formatNumber(3, 1, currentTime.minute, 2); lcd.print(":");
    formatNumber(6, 1, currentTime.second, 2);

    printFloat(9, 1, currentTemperature_filtered, 1, 2); // Temp display, 1 decimal, 2 digits before
    lcd.print((char)223); lcd.print("C "); // Degree symbol & Celsius
}

void displaySetTimeScreen(const DateTime& tempTime, int step) {
    lcd.clear(); // 清屏以便显示 ">" 符号
    lcd.setCursor(0, 0); 
    // lcd.print("Set Time:");

    // 年月日
    if (step == 5) lcd.print(">"); else lcd.print(" "); formatNumber(1, 0, tempTime.year, 4); lcd.print("-");
    if (step == 4) lcd.print(">"); else lcd.print(" "); formatNumber(7, 0, tempTime.month, 2); lcd.print("-");
    if (step == 3) lcd.print(">"); else lcd.print(" "); formatNumber(11,0, tempTime.day, 2);

    // 时分秒
    lcd.setCursor(0,1);
    if (step == 0) lcd.print(">"); else lcd.print(" "); formatNumber(1, 1, tempTime.hour, 2); lcd.print(":");
    if (step == 1) lcd.print(">"); else lcd.print(" "); formatNumber(5, 1, tempTime.minute, 2); lcd.print(":");
    if (step == 2) lcd.print(">"); else lcd.print(" "); formatNumber(10,1, tempTime.second, 2);
    lcd.setCursor(13, 1); lcd.print("SET"); // 清除光标后面的字符
}

void displaySetAlarmScreen(int tempHour, int tempMin, float tempAlarmTemp, int step) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Set Alarm:");
    lcd.setCursor(0, 1);

    if (step == 0) lcd.print(">"); else lcd.print(" "); formatNumber(1, 1, tempHour, 2); lcd.print(":"); // Alarm Hour
    if (step == 1) lcd.print(">"); else lcd.print(" "); formatNumber(6, 1, tempMin, 2);                 // Alarm Minute

    lcd.setCursor(10, 1);
    if (step == 2) lcd.print(">"); else lcd.print(" ");
    printFloat(11, 1, tempAlarmTemp, 0, 2); // Alarm Temp (integer)
    lcd.print((char)223); lcd.print("C");
}

// --- 模式处理 ---
void handleNormalMode() {
    readAndUpdateTimeFromRTC();
    readAndUpdateTemperature();
    displayTimeScreen();

    checkAndTriggerHourlyChime();
    checkAndTriggerClockAlarm(); // This will set clock_alarm_active if conditions met
    checkAndTriggerTemperatureAlarm();

    // CHOOSE (短按) 进入时间设置
    if (checkButtonPress(BUTTON_CHOOSE_PIN, button_last_press_time_choose, BUTTON_DEBOUNCE_DELAY * 2)) {
        enterTimeSettingMode();
        return;
    }

    // ADD + MINUS (同时长按) 进入闹钟设置
    static unsigned long combo_press_start_time = 0;
    bool add_low = (digitalRead(BUTTON_ADD_PIN) == LOW);
    bool minus_low = (digitalRead(BUTTON_MINUS_PIN) == LOW);

    if (add_low && minus_low) {
        if (combo_press_start_time == 0) {
            combo_press_start_time = millis();
        } else if (millis() - combo_press_start_time > BUTTON_DEBOUNCE_DELAY * 4) { // 稍长一点的组合键确认时间
            enterAlarmSettingMode();
            combo_press_start_time = 0; // 重置
            return;
        }
    } else {
        combo_press_start_time = 0;
    }
}

DateTime tempSettingTime; // 用于设置模式的临时时间存储

void enterTimeSettingMode() {
    currentMode = SystemMode::SET_TIME;
    settingStep = 0; // 0:Hour, 1:Min, 2:Sec, 3:Day, 4:Mon, 5:Year
    tempSettingTime = currentTime; // 复制当前时间到临时变量
    setting_mode_entry_time = millis();
    lcd.clear();
    displaySetTimeScreen(tempSettingTime, settingStep);
}

int tempSettingAlarmHour, tempSettingAlarmMinute;
float tempSettingAlarmTemp;

void enterAlarmSettingMode() {
    currentMode = SystemMode::SET_ALARM;
    settingStep = 0; // 0:Hour, 1:Min, 2:Temp
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
        rtc.setDate(tempSettingTime.year, tempSettingTime.month, tempSettingTime.day);
        // rtc.setDOW(tempSettingTime.dayOfWeek); // If needed and DOW was also set
        readAndUpdateTimeFromRTC(); // 更新 currentTime 以反映更改
    }
    if (saveAlarm) {
        alarm_hour = tempSettingAlarmHour;
        alarm_minute = tempSettingAlarmMinute;
        temperatureAlarmThreshold = tempSettingAlarmTemp;
    }
    currentMode = SystemMode::NORMAL;
    lcd.clear();
    displayTimeScreen(); // 刷新主显示
}

void handleTimeSettingMode() {
    if (millis() - setting_mode_entry_time > 30000) { // 30秒超时
        exitSettingModeAndSave(false, false); // 不保存，直接退出
        lcd.setCursor(0,0); lcd.print("Set Timeout"); delay(1000);
        return;
    }

    bool needs_redraw = false;

    if (checkButtonPress(BUTTON_CHOOSE_PIN, button_last_press_time_choose, BUTTON_DEBOUNCE_DELAY * 2)) {
        setting_mode_entry_time = millis(); // 重置超时
        settingStep++;
        if (settingStep > 5) { // 完成所有步骤
            exitSettingModeAndSave(true, false); // 保存时间设置
            return;
        }
        needs_redraw = true;
    }

    if (checkButtonHeld(BUTTON_ADD_PIN, button_last_press_time_add, BUTTON_DEBOUNCE_DELAY)) {
        setting_mode_entry_time = millis(); needs_redraw = true;
        switch (settingStep) {
            case 0: tempSettingTime.hour = (tempSettingTime.hour + 1) % 24; break;
            case 1: tempSettingTime.minute = (tempSettingTime.minute + 1) % 60; break;
            case 2: tempSettingTime.second = (tempSettingTime.second + 1) % 60; break;
            case 3: tempSettingTime.day++; if (tempSettingTime.day > daysInMonth(tempSettingTime.year, tempSettingTime.month)) tempSettingTime.day = 1; break;
            case 4: tempSettingTime.month++; if (tempSettingTime.month > 12) tempSettingTime.month = 1; break;
            case 5: tempSettingTime.year++; if (tempSettingTime.year > 2099) tempSettingTime.year = 2000; break;
        }
    }

    if (checkButtonHeld(BUTTON_MINUS_PIN, button_last_press_time_minus, BUTTON_DEBOUNCE_DELAY)) {
        setting_mode_entry_time = millis(); needs_redraw = true;
        switch (settingStep) {
            case 0: tempSettingTime.hour = (tempSettingTime.hour - 1 + 24) % 24; break;
            case 1: tempSettingTime.minute = (tempSettingTime.minute - 1 + 60) % 60; break;
            case 2: tempSettingTime.second = (tempSettingTime.second - 1 + 60) % 60; break;
            case 3: tempSettingTime.day--; if (tempSettingTime.day < 1) tempSettingTime.day = daysInMonth(tempSettingTime.year, tempSettingTime.month); break;
            case 4: tempSettingTime.month--; if (tempSettingTime.month < 1) tempSettingTime.month = 12; break;
            case 5: tempSettingTime.year--; if (tempSettingTime.year < 2000) tempSettingTime.year = 2099; break;
        }
    }
    // 日期有效性修正 (例如, 从31号切到2月)
    int max_days = daysInMonth(tempSettingTime.year, tempSettingTime.month);
    if (tempSettingTime.day > max_days) {
        tempSettingTime.day = max_days;
        needs_redraw = true;
    }


    if (needs_redraw) {
        displaySetTimeScreen(tempSettingTime, settingStep);
    }
}

void handleAlarmSettingMode() {
    if (millis() - setting_mode_entry_time > 30000) { // 30秒超时
        exitSettingModeAndSave(false, false);
        lcd.setCursor(0,0); lcd.print("Set Timeout"); delay(1000);
        return;
    }
    bool needs_redraw = false;

    if (checkButtonPress(BUTTON_CHOOSE_PIN, button_last_press_time_choose, BUTTON_DEBOUNCE_DELAY * 2)) {
        setting_mode_entry_time = millis();
        settingStep++;
        if (settingStep > 2) { // 完成所有步骤
            exitSettingModeAndSave(false, true); // 保存闹钟设置
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
        if (currentTime.hour != hourly_chime_last_triggered_hour) {
            tone(BUZZER_TONE_PIN, HOURLY_CHIME_FREQ, HOURLY_CHIME_DURATION);
            hourly_chime_last_triggered_hour = currentTime.hour;
        }
    }
}

void checkAndTriggerClockAlarm() {
    if (clock_alarm_active) return; // 闹钟已在响

    if (currentTime.hour == alarm_hour &&
        currentTime.minute == alarm_minute &&
        currentTime.second == 0) { // 通常闹钟秒为0
        clock_alarm_active = true;
        clock_alarm_sound_start_time = millis();
        // 第一次鸣响由 handleActiveClockAlarmSound 处理
    }
}

void handleActiveClockAlarmSound() {
    if (!clock_alarm_active) return;

    unsigned long current_millis = millis();
    if (current_millis - clock_alarm_sound_start_time > ALARM_DURATION) { // 总闹钟时长
        stopClockAlarmSound();
        return;
    }

    // 按任意键停止闹钟
    if (digitalRead(BUTTON_CHOOSE_PIN) == LOW ||
        digitalRead(BUTTON_ADD_PIN) == LOW ||
        digitalRead(BUTTON_MINUS_PIN) == LOW) {
        stopClockAlarmSound();
        delay(BUTTON_DEBOUNCE_DELAY * 2); // 按键去抖和延迟，避免立即再次触发
        return;
    }

    // 间歇鸣响
    static unsigned long last_beep_toggle_time = 0;
    unsigned long beep_interval = 500; // 响半秒，停半秒

    if (current_millis - last_beep_toggle_time > beep_interval) {
        last_beep_toggle_time = current_millis;
        // 通过取模判断当前是应该响还是不响
        if (( (current_millis - clock_alarm_sound_start_time) / beep_interval) % 2 == 0) {
             tone(BUZZER_TONE_PIN, ALARM_SOUND_FREQ, beep_interval - 50); // 响，留一点间隙
        } else {
             noTone(BUZZER_TONE_PIN); // 停
        }
    }
}

void stopClockAlarmSound() {
    clock_alarm_active = false;
    noTone(BUZZER_TONE_PIN);
}

void checkAndTriggerTemperatureAlarm() {
    if (clock_alarm_active) return; // 如果时间闹钟在响，则不触发温度报警

    if (currentTemperature_filtered >= temperatureAlarmThreshold) {
        unsigned long current_millis = millis();
        if (current_millis - temp_alarm_last_beep_time > TEMP_ALARM_DURATION * 4) { // 温度报警鸣响间隔长一些
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
}

float applyKalmanFilter(float measurement) {
    float P_minus = kalman_P + kalman_Q;
    float K = P_minus / (P_minus + kalman_R);
    kalman_x_hat = kalman_x_hat + K * (measurement - kalman_x_hat);
    kalman_P = (1 - K) * P_minus;
    return kalman_x_hat;
}

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
                Serial.print("Kalman Q set: "); Serial.println(kalman_Q, 6);
                Serial.print("Kalman R set: "); Serial.println(kalman_R, 6);
            } else { Serial.println("Invalid Q/R values."); }
        } else { Serial.println("Format: Q_val,R_val"); }
    }
}

// --- 按键辅助函数 ---
// 检测按钮是否被按下（一次性事件）
bool checkButtonPress(int pin, unsigned long& lastPressTime, unsigned long debounceInterval) {
    unsigned long current_millis = millis();
    if (digitalRead(pin) == LOW && (current_millis - lastPressTime > debounceInterval)) {
        lastPressTime = current_millis;
        return true;
    }
    return false;
}

// 检测按钮是否被按住（可用于连续增加/减少）
bool checkButtonHeld(int pin, unsigned long& lastActionTime, unsigned long actionInterval) {
     unsigned long current_millis = millis();
    if (digitalRead(pin) == LOW && (current_millis - lastActionTime > actionInterval)) {
        // 对于非常快速的重复，可以将 actionInterval 设小一些，如 BUTTON_DEBOUNCE_DELAY / 2
        // 但为了简化，这里使用和 debounce 类似或稍大的间隔
        lastActionTime = current_millis;
        return true;
    }
    return false;
}
#endif
#define OLEDTEST3

#define __Serial_DEBUG__
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

#endif // CONFIG_H
#if defined(NOW1)
#include <Arduino.h>
#include <DS1302.h>
#include <LiquidCrystal.h> // LCD1602 显示头文件
#include "Config.h"      // 配置文件, 包含所有 #define

// LCD 初始化
LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

// DS1302 RTC 对象
DS1302 rtc(DS1302_CE_PIN, DS1302_IO_PIN, DS1302_SCLK_PIN);

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

// 闹钟设置
int alarm_hour = 7;
int alarm_minute = 30;
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

// (可能还需要为这些新模式的按键状态添加 last_press_time 变量，如果它们的控制逻辑复杂)


// ==== 函数声明 ====
void readAndUpdateTimeFromRTC();
void calculateDayOfWeek(DateTime& dt);
String getDayOfWeekString(int dow);
int daysInMonth(int year, int month); // 返回指定年月的天数

void displayTimeScreen();
void displaySetTimeScreen(const DateTime& tempTime, int step);
void displayStopwatchScreen(unsigned long elapsed_ms, bool is_running);
void displayCountdownSetScreen(int h, int m, int s, int currentSettingStep) ;
void displayCountdownRunningScreen(int h, int m, int s, bool is_beeping);
void displaySetAlarmScreen(int tempHour, int tempMin, float tempAlarmTemp, int step);
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
void handleKalmanSerialConfig();           // 通过串口配置卡尔曼参数

bool checkButtonPress(int pin, unsigned long& lastPressTime, unsigned long debounceInterval); // 检测按钮单击
bool checkLongButtonPress(int pin, bool& pressStateFlag, unsigned long& pressStartTime, unsigned long thresholdMillis); // 检测长按
bool checkMultiButtonPress(int pin1, int pin2, unsigned long& lastPressTime, unsigned long debounceInterval); // 检测多键组合按下
bool checkButtonHeld(int pin, unsigned long& lastActionTime, unsigned long actionInterval);    // 检测按钮按住状态（用于连续调整）


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

#ifdef __Serial_DEBUG__
    Serial.begin(9600);
    Serial.println("Initializing...");
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
}

void loop() {
    unsigned long current_millis = millis(); // 获取一次，减少重复调用

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
    
    handleKalmanSerialConfig(); // 任何模式下都可以调整卡尔曼参数

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
    Serial.print("RTC Time: ");
    Serial.print(currentTime.year); Serial.print("-");
    Serial.print(currentTime.month); Serial.print("-");
    Serial.print(currentTime.day); Serial.print(" ");
    Serial.print(currentTime.hour); Serial.print(":");
    Serial.print(currentTime.minute); Serial.print(":");
    Serial.println(currentTime.second);
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
void formatNumber(int col, int row, int num, int digits) {
    lcd.setCursor(col, row);
    String s = String(num);
    for (int i = 0; i < digits - s.length(); i++) {
        lcd.print("0");
    }
    lcd.print(s);
}

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
    Serial.print("Display Time: "); 
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
}

void displaySetTimeScreen(const DateTime& tempTime, int step) {
    lcd.clear(); 
    lcd.setCursor(0, 0); 

    lcd.setCursor(0,0);
    if (settingStep == 5) lcd.print(">"); else lcd.print(" "); formatNumber(1, 0, tempTime.year, 4); 
    lcd.setCursor(5,0);
    if (settingStep == 4) lcd.print(">"); else lcd.print("-"); formatNumber(6, 0, tempTime.month, 2);  
    lcd.setCursor(9,0);
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
    unsigned long disp_tenths = (elapsed_ms % 1000) / 100; // 取十分之一秒

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
        lcd.print(disp_tenths); // 显示十分之一秒
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
    lcd.setCursor(0, 0); lcd.print("Set Alarm:");
    lcd.setCursor(0, 1);

    if (step == 0) lcd.print(">"); else lcd.print(" "); formatNumber(1, 1, tempHour, 2); lcd.print(":");
    lcd.setCursor(5, 1);
    if (step == 1) lcd.print(">"); else lcd.print(" "); formatNumber(6, 1, tempMin, 2);

    lcd.setCursor(10, 1);
    if (step == 2) lcd.print(">"); else lcd.print(" ");
    printFloat(11, 1, tempAlarmTemp, 0, 2); 
    lcd.print((char)223); lcd.print("C");
}


// --- 模式处理 ---
void handleNormalMode() {
    readAndUpdateTimeFromRTC();
    readAndUpdateTemperature();
    displayTimeScreen();

    checkAndTriggerHourlyChime();
    checkAndTriggerClockAlarm(); 
    checkAndTriggerTemperatureAlarm();

    if (checkButtonPress(BUTTON_CHOOSE_PIN, button_last_press_time_choose, BUTTON_DEBOUNCE_DELAY * 2)) {
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

    if (checkButtonPress(BUTTON_CHOOSE_PIN, button_last_press_time_choose, BUTTON_DEBOUNCE_DELAY * 2)) {
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


void handleStopwatchMode() {
    unsigned long current_millis = millis();
    unsigned long display_elapsed_ms = 0;

    // 按键逻辑:
    // CHOOSE: Start / Pause / Resume
    // ADD (短按/单击): Lap (暂不实现，过于复杂) / Reset (当停止或暂停时)
    // MINUS (长按或其他组合): Exit to NORMAL mode

    if (checkButtonPress(BUTTON_CHOOSE_PIN, button_last_press_time_choose, BUTTON_DEBOUNCE_DELAY * 2)) {
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
    if (!stopwatch_running && checkButtonPress(BUTTON_ADD_PIN, button_last_press_time_add, BUTTON_DEBOUNCE_DELAY * 2)) {
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
    }

    // 计算要显示的时间
    if (stopwatch_running) {
        display_elapsed_ms = stopwatch_elapsed_at_pause + (current_millis - stopwatch_start_millis);
    } else {
        display_elapsed_ms = stopwatch_elapsed_at_pause;
    }

    // 调用显示函数 (需要在 display.cpp 中实现)
    displayStopwatchScreen(display_elapsed_ms, stopwatch_running);
    // 示例显示逻辑 (应在 displayStopwatchScreen 中实现)
    unsigned long total_seconds = display_elapsed_ms / 1000;
    unsigned long disp_hours = total_seconds / 3600;
    unsigned long disp_minutes = (total_seconds % 3600) / 60;
    unsigned long disp_seconds = total_seconds % 60;
    unsigned long disp_tenths = (display_elapsed_ms % 1000) / 100;

    lcd.setCursor(0, 0);
    if (stopwatch_running) {
        lcd.print("Timing: RUNNING");
#ifdef __Serial_DEBUG__
        Serial.print("Timing: RUNNING");
        Serial.println(display_elapsed_ms);
#endif
    }
    else if (stopwatch_elapsed_at_pause > 0) {
        lcd.print("Timing: PAUSED");
#ifdef __Serial_DEBUG__
        Serial.print("Timing: PAUSED ");
        Serial.println(display_elapsed_ms);
#endif
    }
    else {
        lcd.print("Timing: READY ");
#ifdef __Serial_DEBUG__
        Serial.print("Timing: READY ");
        Serial.println(display_elapsed_ms);
#endif
    }
        

    lcd.setCursor(0, 1);
    if (disp_hours > 0) {
        formatNumber(0, 1, disp_hours, 2); lcd.print(":");
        formatNumber(3, 1, disp_minutes, 2); lcd.print(":");
        formatNumber(6, 1, disp_seconds, 2); lcd.print("   "); // HH:MM:SS
    } else {
        formatNumber(0, 1, disp_minutes, 2); lcd.print(":"); // MM:SS.T
        formatNumber(3, 1, disp_seconds, 2); lcd.print(".");
        lcd.print(disp_tenths); lcd.print("  ");
    }
    // 在第二行显示操作提示，例如 "CH:S/P ADD:Rst"
    lcd.setCursor(7,0); // 清除之前可能存在的提示
    lcd.print("        "); // 清除提示
    lcd.setCursor(7,0);
    if(stopwatch_running) lcd.print("CH:Pa"); else lcd.print("CH:Go");
    if(!stopwatch_running) {lcd.setCursor(13,0); lcd.print("A:R");}


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

    if (checkButtonPress(BUTTON_CHOOSE_PIN, button_last_press_time_choose, BUTTON_DEBOUNCE_DELAY * 2)) {
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
        unsigned long paused_remaining_ms = countdown_target_millis - millis(); // 这是如果没暂停会剩余的时间
                                                                                // 实际上，应该在暂停时记录剩余时间
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
    if (checkButtonPress(BUTTON_CHOOSE_PIN, button_last_press_time_choose, BUTTON_DEBOUNCE_DELAY * 2))
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
    if (checkButtonPress(BUTTON_MINUS_PIN, button_last_press_time_minus, BUTTON_DEBOUNCE_DELAY * 2))
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

    if (checkButtonPress(BUTTON_CHOOSE_PIN, button_last_press_time_choose, BUTTON_DEBOUNCE_DELAY * 2)) {
        setting_mode_entry_time = millis();
        settingStep++;
        if (settingStep > 2) { 
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
        if (currentTime.hour != hourly_chime_last_triggered_hour) { 
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

    unsigned long current_millis = millis();
    if (current_millis - clock_alarm_sound_start_time > ALARM_DURATION) {
        stopClockAlarmSound();
        return;
    }

    if (digitalRead(BUTTON_CHOOSE_PIN) == LOW ||
        digitalRead(BUTTON_ADD_PIN) == LOW ||
        digitalRead(BUTTON_MINUS_PIN) == LOW) {
        stopClockAlarmSound();
        delay(BUTTON_DEBOUNCE_DELAY * 2); 
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

void stopClockAlarmSound() {
    clock_alarm_active = false;
    noTone(BUZZER_TONE_PIN);
}

void checkAndTriggerTemperatureAlarm() {
    if (clock_alarm_active) return; 

    if (currentTemperature_filtered >= temperatureAlarmThreshold) {
        unsigned long current_millis = millis();
        if (current_millis - temp_alarm_last_beep_time > TEMP_ALARM_DURATION * 10) { 
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
}

float applyKalmanFilter(float measurement) {
    float P_minus = kalman_P + kalman_Q; 
    float K = P_minus / (P_minus + kalman_R); 
    kalman_x_hat = kalman_x_hat + K * (measurement - kalman_x_hat); 
    kalman_P = (1 - K) * P_minus; 
    return kalman_x_hat;
}

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
#endif

#if defined(OLEDTEST1)
#include <Arduino.h>
#include <U8g2lib.h>

// ----------------------------------------------------------------------------
// 定义软件 I2C 使用的引脚
// 你可以根据你的接线修改这些引脚
// ----------------------------------------------------------------------------
#define SW_I2C_PIN_SCL 6 // 将 OLED 的 SCL 引脚连接到 Arduino 的数字引脚 6
#define SW_I2C_PIN_SDA 7  // 将 OLED 的 SDA 引脚连接到 Arduino 的数字引脚 7

// 可选：如果你的 OLED 模块有 RST (Reset) 引脚，定义并连接它
// 如果没有 RST 引脚或者你不打算使用它，可以将其设置为 U8X8_PIN_NONE
#define OLED_PIN_RST U8X8_PIN_NONE // 例如，不使用 RST 引脚
// 或者，如果你连接了 RST 引脚 (例如到数字引脚 8):
// #define OLED_PIN_RST 8


// ----------------------------------------------------------------------------
// U8g2 构造函数选择
// ----------------------------------------------------------------------------
// 选择与你的 OLED 屏幕匹配的构造函数。
// 格式: U8G2_<Controller>_<Resolution>_<BufferMode>_<Interface>
//
// - <Controller>: 例如 SSD1306, SH1106, SSD1309 等。
// - <Resolution>: 例如 128X64, 128X32 等。
// - <BufferMode>:
//   - NONAME_1: 页缓冲模式 (Page Buffer), RAM 占用小，但绘图可能稍慢或有局限。
//   - NONAME_F: 全缓冲模式 (Full Frame Buffer), RAM 占用大，但绘图更灵活快速。
//               对于 Arduino UNO (只有 2KB RAM)，全缓冲 128x64 (1KB RAM) 可能比较紧张，
//               但通常是可行的。如果 RAM 不足，可以尝试页缓冲模式。
// - <Interface>:
//   - _SW_I2C: 软件 I2C
//
// 常用 OLED (SSD1306, 128x64) 的软件 I2C 全缓冲构造函数:
// U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(rotation, scl, sda, reset);
//
// 参数说明:
//   rotation: 屏幕旋转。常用值:
//             U8G2_R0 (无旋转), U8G2_R1 (旋转90度), U8G2_R2 (旋转180度), U8G2_R3 (旋转270度)
//   scl:      软件 I2C 的 SCL 引脚
//   sda:      软件 I2C 的 SDA 引脚
//   reset:    OLED 的 RST 引脚 (如果未使用，则为 U8X8_PIN_NONE)
// ----------------------------------------------------------------------------

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(
    U8G2_R0,            // 旋转: 无旋转
    SW_I2C_PIN_SCL,     // SCL 引脚
    SW_I2C_PIN_SDA,     // SDA 引脚
    OLED_PIN_RST        // Reset 引脚
);

// 如果是 SH1106 控制器的 128x64 屏幕，构造函数可能是：
// U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, SW_I2C_PIN_SCL, SW_I2C_PIN_SDA, OLED_PIN_RST);


void setup(void) {
  // 启动 U8g2 库
  u8g2.begin();

  // 可选：如果你想在串口监视器中看到调试信息
  Serial.begin(9600);
  Serial.println("U8g2 Software I2C OLED Test");
}

void loop(void) {
  // U8g2 使用页循环（picture loop）的概念来绘制屏幕。
  // 你需要将所有的绘图指令放在 u8g2.firstPage() 和 u8g2.nextPage() 之间。
  // 对于全缓冲模式 (_F_), 这个循环只会执行一次。
  // 对于页缓冲模式 (_1_), 这个循环会执行多次，每次绘制屏幕的一部分。

  u8g2.firstPage();
  do {
    // 在这里放置你的绘图指令

    // 示例1: 显示简单文本
    u8g2.setFont(u8g2_font_ncenB08_tr); // 选择一个字体 (ncenB08, 透明背景, 8像素高)
    u8g2.drawStr(0, 10, "Hello World!"); // 在 (x=0, y=10) 的位置绘制字符串

    // 示例2: 显示变化的数字
    u8g2.setFont(u8g2_font_ncenB10_tr); // 选择一个稍大的字体
    u8g2.setCursor(0, 30); // 设置光标位置
    u8g2.print("Count: ");
    u8g2.print(millis() / 1000); // 显示自启动以来的秒数

    // 示例3: 绘制图形
    u8g2.drawBox(0, 40, 30, 10);      // 绘制一个实心矩形 (x, y, width, height)
    u8g2.drawFrame(40, 40, 30, 10);   // 绘制一个空心矩形

  } while (u8g2.nextPage()); // 继续到下一页/帧

  delay(500); // 每0.5秒刷新一次屏幕内容
}

#endif


#if defined(OLEDTEST2)
#include <Arduino.h>
#include <U8g2lib.h> // 引入 U8g2 库

// ----------------------------------------------------------------------------
// 软件 I2C 引脚定义 (根据你的接线修改)
// ----------------------------------------------------------------------------
#define SW_I2C_PIN_SCL 6 // OLED SCL -> Arduino D6
#define SW_I2C_PIN_SDA 7  // OLED SDA -> Arduino D7
#define OLED_PIN_RST U8X8_PIN_NONE // 如果不使用 RST 引脚

// ----------------------------------------------------------------------------
// U8g2 构造函数 (SSD1306 128x64, 全缓冲, 软件 I2C)
// ----------------------------------------------------------------------------
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(
    U8G2_R0,            // 旋转: 无旋转
    SW_I2C_PIN_SCL,     // SCL 引脚
    SW_I2C_PIN_SDA,     // SDA 引脚
    OLED_PIN_RST        // Reset 引脚
);

// ----------------------------------------------------------------------------
// 时钟参数
// ----------------------------------------------------------------------------
const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;
const int CLOCK_CENTER_X = SCREEN_WIDTH / 2;
const int CLOCK_CENTER_Y = SCREEN_HEIGHT / 2;
const int CLOCK_RADIUS = SCREEN_HEIGHT / 2 - 4; // 留出一点边距

const int HOUR_HAND_LENGTH = CLOCK_RADIUS * 0.5;
const int MINUTE_HAND_LENGTH = CLOCK_RADIUS * 0.7;
const int SECOND_HAND_LENGTH = CLOCK_RADIUS * 0.8;

// 模拟时间的变量
unsigned long previousMillis = 0;
int currentHours = 10;    // 初始时钟: 10点
int currentMinutes = 10;  // 初始分钟: 10分
int currentSeconds = 30;  // 初始秒钟: 30秒

// ----------------------------------------------------------------------------
// 辅助函数：绘制表盘背景 (刻度等)
// ----------------------------------------------------------------------------
void drawClockFace() {
  // 绘制表盘外圈
  u8g2.drawCircle(CLOCK_CENTER_X, CLOCK_CENTER_Y, CLOCK_RADIUS);
  // u8g2.drawCircle(CLOCK_CENTER_X, CLOCK_CENTER_Y, CLOCK_RADIUS-1); // 可以画双圈

  // 绘制小时刻度
  for (int i = 0; i < 12; ++i) {
    float angle = i * (360.0 / 12.0); // 每小时30度
    float angleRad = angle * DEG_TO_RAD; // 转换为弧度

    int x1 = CLOCK_CENTER_X + (CLOCK_RADIUS - 5) * sin(angleRad);
    int y1 = CLOCK_CENTER_Y - (CLOCK_RADIUS - 5) * cos(angleRad); // Y轴向上为负
    int x2 = CLOCK_CENTER_X + CLOCK_RADIUS * sin(angleRad);
    int y2 = CLOCK_CENTER_Y - CLOCK_RADIUS * cos(angleRad);

    // 对于 3, 6, 9, 12 点使用稍长或不同的标记 (可选)
    if (i % 3 == 0) {
      // 可以画粗一点的线，但U8g2画线没有粗细参数，可以通过画小圆点或多画几条线模拟
      u8g2.drawLine(x1 - (x2-x1)/3 , y1 - (y2-y1)/3, x2, y2); // 稍微加长一点点
    } else {
      u8g2.drawLine(x1, y1, x2, y2);
    }
  }
  // 绘制中心点
  u8g2.drawDisc(CLOCK_CENTER_X, CLOCK_CENTER_Y, 2);
}

// ----------------------------------------------------------------------------
// 辅助函数：绘制时钟指针
// ----------------------------------------------------------------------------
void drawHands(int h, int m, int s) {
  float angleRad;

  // --- 绘制时针 ---
  // 每小时30度，每分钟0.5度
  angleRad = ( (h % 12) + m / 60.0 ) * 30.0 * DEG_TO_RAD - HALF_PI; // 减 HALF_PI 使0度朝上
  int hourHandX = CLOCK_CENTER_X + HOUR_HAND_LENGTH * cos(angleRad);
  int hourHandY = CLOCK_CENTER_Y + HOUR_HAND_LENGTH * sin(angleRad);
  u8g2.drawLine(CLOCK_CENTER_X, CLOCK_CENTER_Y, hourHandX, hourHandY);
  // 可以尝试画粗一点的时针，例如画两条平行的线或一个细长的三角形
  // u8g2.drawLine(CLOCK_CENTER_X+1, CLOCK_CENTER_Y, hourHandX+1, hourHandY);


  // --- 绘制分针 ---
  // 每分钟6度
  angleRad = (m + s / 60.0) * 6.0 * DEG_TO_RAD - HALF_PI;
  int minuteHandX = CLOCK_CENTER_X + MINUTE_HAND_LENGTH * cos(angleRad);
  int minuteHandY = CLOCK_CENTER_Y + MINUTE_HAND_LENGTH * sin(angleRad);
  u8g2.drawLine(CLOCK_CENTER_X, CLOCK_CENTER_Y, minuteHandX, minuteHandY);

  // --- 绘制秒针 ---
  angleRad = s * 6.0 * DEG_TO_RAD - HALF_PI;
  int secondHandX = CLOCK_CENTER_X + SECOND_HAND_LENGTH * cos(angleRad);
  int secondHandY = CLOCK_CENTER_Y + SECOND_HAND_LENGTH * sin(angleRad);
  u8g2.drawLine(CLOCK_CENTER_X, CLOCK_CENTER_Y, secondHandX, secondHandY);
}


// ----------------------------------------------------------------------------
// Arduino Setup
// ----------------------------------------------------------------------------
void setup(void) {
  u8g2.begin(); // 初始化 U8g2 库
  u8g2.setFont(u8g2_font_6x10_tf); // 设置一个默认字体 (如果需要显示数字时间)
  // u8g2.setFontMode(0); // 0=solid, 1=transparent
}

// ----------------------------------------------------------------------------
// Arduino Loop
// ----------------------------------------------------------------------------
void loop(void) {
  unsigned long currentMillis = millis();

  // 每秒更新一次时间 (模拟)
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    currentSeconds++;
    if (currentSeconds >= 60) {
      currentSeconds = 0;
      currentMinutes++;
      if (currentMinutes >= 60) {
        currentMinutes = 0;
        currentHours++;
        if (currentHours >= 12) { // 可以改成24小时制，但表盘是12小时
          currentHours = 0;
        }
      }
    }
  }

  // U8g2 绘图循环
  u8g2.firstPage();
  do {
    drawClockFace();
    drawHands(currentHours, currentMinutes, currentSeconds);

    // 可选：在屏幕一角显示数字时间
    // u8g2.setCursor(2, 10);
    // if (currentHours < 10) u8g2.print("0");
    // u8g2.print(currentHours);
    // u8g2.print(":");
    // if (currentMinutes < 10) u8g2.print("0");
    // u8g2.print(currentMinutes);
    // u8g2.print(":");
    // if (currentSeconds < 10) u8g2.print("0");
    // u8g2.print(currentSeconds);

  } while (u8g2.nextPage());

  // 不需要额外的 delay，因为绘图和时间更新是基于 millis() 的
}
#endif

#if defined(OLEDTEST3)
#include <Arduino.h>
#include <U8g2lib.h> // 引入 U8g2 库

// ----------------------------------------------------------------------------
// 全局 U8g2 对象 和 软件 I2C 引脚定义
// ----------------------------------------------------------------------------
// !!! 请根据你的实际接线修改 SCL 和 SDA 引脚 !!!
#define SW_I2C_PIN_SCL 6 // OLED SCL -> Arduino D10
#define SW_I2C_PIN_SDA 7  // OLED SDA -> Arduino D9
#define OLED_PIN_RST U8X8_PIN_NONE // 如果不使用 RST 引脚

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(
    U8G2_R0,            // 旋转: 无旋转
    SW_I2C_PIN_SCL,     // SCL 引脚
    SW_I2C_PIN_SDA,     // SDA 引脚
    OLED_PIN_RST        // Reset 引脚
);

// ----------------------------------------------------------------------------
// 时钟参数
// ----------------------------------------------------------------------------
const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;
// 将中心点调整为 دقیق的屏幕中心 (对于奇数/偶数像素宽度/高度)
const int CLOCK_CENTER_X = SCREEN_WIDTH / 2 -1 ; // 通常是 63 (0-indexed)
const int CLOCK_CENTER_Y = SCREEN_HEIGHT / 2 -1; // 通常是 31 (0-indexed)
const int CLOCK_RADIUS = SCREEN_HEIGHT / 2 - 4; // 表盘半径, e.g., 32-4 = 28

// 指针长度
const int HOUR_HAND_LENGTH = CLOCK_RADIUS * 0.55;
const int MINUTE_HAND_LENGTH = CLOCK_RADIUS * 0.75;
const int SECOND_HAND_LENGTH = CLOCK_RADIUS * 0.85;

// 模拟时间变量
unsigned long previousMillis = 0;
int currentHours = 10;    // 初始时间 10:10:30
int currentMinutes = 10;
int currentSeconds = 30;


// ----------------------------------------------------------------------------
// 内部辅助函数：绘制表盘元素 (刻度、数字等)
// ----------------------------------------------------------------------------
void drawClockFaceElements() {
  // 1. 绘制表盘外圈 (双层使其看起来更精致)
  u8g2.drawCircle(CLOCK_CENTER_X, CLOCK_CENTER_Y, CLOCK_RADIUS);
  u8g2.drawCircle(CLOCK_CENTER_X, CLOCK_CENTER_Y, CLOCK_RADIUS - 1);

  // 2. 绘制小时刻度和数字
  u8g2.setFont(u8g2_font_u8glib_4_tf); // 选择一个非常小的字体 (4像素高)
                                      // 可选字体: u8g2_font_tiny5_tf, u8g2_font_4x6_tf
  int textHeightApprox = 4; // 所选字体的高度近似值
  int numDisplayRadius = CLOCK_RADIUS - 7; // 数字显示位置的半径

  for (int i = 1; i <= 12; ++i) {
    // 角度计算: i*30度，再减90度使12点在顶部 (数学角度0度向右)
    float angleDeg = (i * 30.0) - 90.0;
    float angleRad = angleDeg * DEG_TO_RAD;

    // --- 绘制刻度线 ---
    int x1_tick, y1_tick, x2_tick, y2_tick;
    if (i % 3 == 0) { // 主要刻度 (3, 6, 9, 12) - 更长更粗
      x1_tick = CLOCK_CENTER_X + (CLOCK_RADIUS - 5) * cos(angleRad);
      y1_tick = CLOCK_CENTER_Y + (CLOCK_RADIUS - 5) * sin(angleRad);
      x2_tick = CLOCK_CENTER_X + (CLOCK_RADIUS - 1) * cos(angleRad); // 直到内圈
      y2_tick = CLOCK_CENTER_Y + (CLOCK_RADIUS - 1) * sin(angleRad);
      // 尝试画粗一点: 可以画两条线，或者一个小矩形/圆点
      // 这里简单地用默认线宽，但主要刻度从更靠内的地方开始
       u8g2.drawLine(x1_tick, y1_tick, x2_tick, y2_tick);
       // 额外绘制一点使其看起来粗一点 (可选)
       // u8g2.drawLine(x1_tick+cos(angleRad-HALF_PI), y1_tick+sin(angleRad-HALF_PI), x2_tick+cos(angleRad-HALF_PI), y2_tick+sin(angleRad-HALF_PI));
    } else { // 次要刻度
      x1_tick = CLOCK_CENTER_X + (CLOCK_RADIUS - 3) * cos(angleRad);
      y1_tick = CLOCK_CENTER_Y + (CLOCK_RADIUS - 3) * sin(angleRad);
      x2_tick = CLOCK_CENTER_X + (CLOCK_RADIUS - 1) * cos(angleRad);
      y2_tick = CLOCK_CENTER_Y + (CLOCK_RADIUS - 1) * sin(angleRad);
      u8g2.drawLine(x1_tick, y1_tick, x2_tick, y2_tick);
    }

    // --- 绘制小时数字 ---
    String numStr = String(i);
    int strWidth = u8g2.getStrWidth(numStr.c_str());

    // 计算数字中心点的位置
    int numX_center = CLOCK_CENTER_X + numDisplayRadius * cos(angleRad);
    int numY_center = CLOCK_CENTER_Y + numDisplayRadius * sin(angleRad);

    // 根据字符串宽度和字体高度调整，使其居中
    int drawNumX = numX_center - (strWidth / 2);
    int drawNumY = numY_center + (textHeightApprox / 2) -1; // 近似垂直居中

    u8g2.drawStr(drawNumX, drawNumY, numStr.c_str());
  }

  // 3. 绘制更突出的中心圆点
  u8g2.drawDisc(CLOCK_CENTER_X, CLOCK_CENTER_Y, 3); // 中心实心圆
  u8g2.drawCircle(CLOCK_CENTER_X,CLOCK_CENTER_Y, 1); // 中心小亮点
}

// ----------------------------------------------------------------------------
// 内部辅助函数：绘制时钟指针
// ----------------------------------------------------------------------------
void drawClockHands(int h, int m, int s) {
  float angleRad;

  // 将24小时制转换为12小时制用于显示
  h = h % 12;
  if (h == 0) { // 0点或12点都显示为12
    h = 12;
  }

  // --- 绘制时针 ---
  // 角度: (小时 + 分钟部分) * 30度/小时; -90度使12点朝上
  float hourAngleDeg = ((h + m / 60.0 + s / 3600.0) * 30.0) - 90.0;
  angleRad = hourAngleDeg * DEG_TO_RAD;
  int hourHandX = CLOCK_CENTER_X + HOUR_HAND_LENGTH * cos(angleRad);
  int hourHandY = CLOCK_CENTER_Y + HOUR_HAND_LENGTH * sin(angleRad);
  // 为了让时针看起来粗一点，可以画两条稍微错开的线
  u8g2.drawLine(CLOCK_CENTER_X, CLOCK_CENTER_Y, hourHandX, hourHandY);
  u8g2.drawLine(CLOCK_CENTER_X+1, CLOCK_CENTER_Y, hourHandX+1, hourHandY); // 水平偏移一点
  // u8g2.drawLine(CLOCK_CENTER_X, CLOCK_CENTER_Y+1, hourHandX, hourHandY+1); // 垂直偏移一点 (效果可能不好)


  // --- 绘制分针 ---
  // 角度: (分钟 + 秒部分) * 6度/分钟; -90度使12点朝上
  float minuteAngleDeg = ((m + s / 60.0) * 6.0) - 90.0;
  angleRad = minuteAngleDeg * DEG_TO_RAD;
  int minuteHandX = CLOCK_CENTER_X + MINUTE_HAND_LENGTH * cos(angleRad);
  int minuteHandY = CLOCK_CENTER_Y + MINUTE_HAND_LENGTH * sin(angleRad);
  u8g2.drawLine(CLOCK_CENTER_X, CLOCK_CENTER_Y, minuteHandX, minuteHandY);

  // --- 绘制秒针 ---
  // 角度: 秒 * 6度/秒; -90度使12点朝上
  float secondAngleDeg = (s * 6.0) - 90.0;
  angleRad = secondAngleDeg * DEG_TO_RAD;
  int secondHandX = CLOCK_CENTER_X + SECOND_HAND_LENGTH * cos(angleRad);
  int secondHandY = CLOCK_CENTER_Y + SECOND_HAND_LENGTH * sin(angleRad);
  u8g2.drawLine(CLOCK_CENTER_X, CLOCK_CENTER_Y, secondHandX, secondHandY);
  // 可以给秒针末端加一个小圆点装饰
  // u8g2.drawPixel(secondHandX, secondHandY); // 太小了
  // u8g2.drawDisc(secondHandX, secondHandY, 1); // 半径为1的小圆盘 (共3x3像素)
}


// ============================================================================
// 封装的函数 1: U8G2 初始化
// ============================================================================
void U8G2INIT() {
  u8g2.begin(); // 初始化 U8g2 库
  // u8g2.setContrast(100); // 可选: 设置对比度 (0-255)
  // u8g2.setFontMode(1);  // 0=solid background, 1=transparent background for text
  u8g2.setDrawColor(1); // 确保绘图颜色为1 (亮)
}

// ============================================================================
// 封装的函数 2: 显示时钟
// ============================================================================
void displayClock(int hours, int minutes, int seconds) {
  u8g2.firstPage();
  do {
    // 清屏 (对于全缓冲模式，firstPage通常会隐式清空，但显式调用更安全)
    // u8g2.clearBuffer(); // 如果用页缓冲模式，这句通常不需要。全缓冲模式下，firstPage会处理。

    drawClockFaceElements(); // 绘制表盘背景和数字
    drawClockHands(hours, minutes, seconds); // 绘制时分秒针

  } while (u8g2.nextPage());
}


// ----------------------------------------------------------------------------
// Arduino Setup
// ----------------------------------------------------------------------------
void setup(void) {
  // Serial.begin(9600); // 可选，用于调试
  // Serial.println("Clock with U8g2 Starting...");

  U8G2INIT(); // 调用初始化函数

  // 初始时间设置在全局变量中
}

// ----------------------------------------------------------------------------
// Arduino Loop
// ----------------------------------------------------------------------------
void loop(void) {
  unsigned long currentMillis = millis();

  // 每秒更新一次模拟时间
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    currentSeconds++;
    if (currentSeconds >= 60) {
      currentSeconds = 0;
      currentMinutes++;
      if (currentMinutes >= 60) {
        currentMinutes = 0;
        currentHours++;
        if (currentHours >= 24) { // 内部时间使用24小时制
          currentHours = 0;
        }
      }
    }
  }

  // 调用显示时钟函数
  displayClock(currentHours, currentMinutes, currentSeconds);

  // loop本身不需要delay，因为时间的更新和重绘是基于millis的
  // 如果CPU占用过高或闪烁，可以考虑在此处加一个非常小的delay，
  // 但通常对于U8g2的全缓冲模式是不必要的。
  // delay(10);
}
#endif