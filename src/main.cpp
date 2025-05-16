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
/**
 * @brief 非阻塞方式更新光标的闪烁状态。
 * 此函数需要在一个循环中被反复调用以产生闪烁效果。
 * @param rol 光标要闪烁的列 (column)
 * @param row 光标要闪烁的行 (row)
 * @param enabled 如果为true，则光标在该位置闪烁；如果为false，则关闭光标。
 */
void updateDisplayCursor_NonBlocking(int rol, int row, bool enabled) {
    unsigned long currentTime = millis();

    if (!enabled) { // 如果外部逻辑要求关闭光标
        if (blink_currentRol != -1) { // 如果之前光标是激活的
            lcd.noCursor(); // 确保光标关闭
            blink_currentRol = -1; // 标记为未激活
            blink_currentRow = -1;
            blink_cursorState = false;
        }
        return;
    }

    // 如果目标闪烁位置改变了，或者之前是关闭状态现在要开启
    if (blink_currentRol != rol || blink_currentRow != row || blink_currentRol == -1) {
        // 如果旧位置有效，先在旧位置关闭光标（可选，取决于具体显示逻辑）
        // lcd.setCursor(blink_currentRol, blink_currentRow); 
        // lcd.noCursor();

        blink_currentRol = rol;
        blink_currentRow = row;
        
        // 立即在新的位置显示光标，并重置计时器
        lcd.setCursor(blink_currentRol, blink_currentRow);
        lcd.cursor();
        blink_cursorState = true;
        blink_lastTime = currentTime;
        
        // 您的调试输出可以放在这里，但只在位置改变时打印一次，避免刷屏
        Serial.print("DisplayCursor activated at: ");
        Serial.print(blink_currentRol);
        Serial.print(" ");
        Serial.println(blink_currentRow);
        return; // 本次调用完成，等待下一次调用来更新闪烁
    }

    // 位置未变，且光标已启用，检查是否到了切换状态的时间
    if (currentTime - blink_lastTime >= CURSOR_BLINK_INTERVAL) {
        blink_lastTime = currentTime; // 更新上次切换时间
        blink_cursorState = !blink_cursorState; // 翻转光标状态

        lcd.setCursor(blink_currentRol, blink_currentRow); // 确保光标在正确位置
        if (blink_cursorState) {
            lcd.cursor(); // 显示光标
        } else {
            lcd.noCursor(); // 隐藏光标
        }
        Serial.print("blink_cursorState: ");
        Serial.print(blink_cursorState);
        Serial.print(" ");
        Serial.print("Cursor state toggled at: ");
        Serial.print(blink_currentRol);
        Serial.print(" ");
        Serial.println(blink_currentRow);
    }
}

/** 显示光标 */
void DisplayCursor(int rol, int row, int N)
{   
    // updateDisplayCursor_NonBlocking(rol, row, true); // 调用非阻塞光标更新函数
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
