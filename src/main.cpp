#include <Arduino.h>
#include <DS1302.h>
#include <LiquidCrystal.h> //LCD1602 显示头文件
#include "Config.h"      //配置头文件

// LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
//lcd 初始化函数
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
// 用于非阻塞光标闪烁的静态变量
static unsigned long blink_lastTime = 0;
static bool blink_cursorState = false; // false = 隐藏, true = 显示
static int blink_currentRol = -1;      // 当前光标列，-1表示未激活
static int blink_currentRow = -1;      // 当前光标行

const unsigned int CURSOR_BLINK_INTERVAL = 500; // 光标状态切换间隔 (500ms亮, 500ms灭)


// ==== 卡尔曼滤波器参数 ====
float kalman_x_hat = 0; // 对当前温度的估计值 (k-1 时刻的后验估计)
float kalman_P = 1.0;   // 估计误差协方差 (k-1 时刻的后验误差协方差)
                        // 初始P较大，表示对初始估计x_hat不太确定

// 过程噪声协方差 Q: 描述真实温度在两次测量之间可能发生的变化的平方。
// 如果你认为温度变化非常缓慢且稳定, Q应该很小。
// 例如，如果认为温度在每次迭代中变化标准差为0.01°C, 则 Q = 0.01*0.01 = 0.0001
float kalman_Q = 0.0001; //  需要根据实际情况调整

// 测量噪声协方差 R: 描述传感器测量的不确定性的平方。
// 这与ADC的分辨率和传感器本身的噪声有关。
// Arduino 10位ADC, 5V参考, LM35灵敏度10mV/°C, 分辨率约0.488°C。
// R可以基于测量值的方差来估计。例如，如果测量噪声的标准差约为0.2°C, R = 0.2*0.2 = 0.04
// 如果原始数据波动较大，R应适当增大。
float kalman_R = 0.09;  // (例如，假设标准差为0.3°C, R=0.3*0.3=0.09) 需要根据实际情况调整


// ==== 函数声明 ====
void FormatDisplay(int col, int row, int num);        // 格式化输出时间/日期数字（不足两位补零）
void time_display();                                  // 时间计算与显示函数 (重命名以避免冲突)
int Days(int current_year, int current_month);        // 计算指定年月的天数
void Day_display();                                   // 计算并显示当前日期 (重命名)
void Month_display();                                 // 计算并显示当前月份 (重命名)
void Year_display();                                  // 计算并显示当前年份 (重命名)
void Week(int y_val, int m_val, int d_val);           // 根据年月日计算星期几并显示
void Display();                                       // 显示全部时间、日期和星期
void DisplayCursor(int rol, int row);                 // 显示闪烁光标，用于提示设置位置
void set(int y_val, int mon_val, int d_val, int h_val, int m_val, int s_val); // 设置初始时间
void Set_Time_Value(int rol, int row, int &Time_var); // 通过按键设置具体时间值（小时、分钟等） (重命名)
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
void kalmanProcess();                                 //卡尔曼滤波参数调整

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
    // rtc.setDate(day_val, month_val, year_val); // DS1302库的setDate是 (day, mon, year)
    rtc.setDate(year_val, month_val, day_val); // 修正: 根据DS1302.h setDate(int year, int mon, int day)

    delay(100); // 减慢循环速度，便于观察和调试，实际应用中可能不需要这么大
}


/*
KalmanAutoProcess
结合卡尔曼滤波和均值滤波，动态调整卡尔曼参数:
1. 读取传感器数据，存储一定数量的读数。
2. 计算均值和标准差。
3. 根据均值和标准差动态调整卡尔曼滤波器的Q和R参数。
*/



void kalmanProcess() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n'); // Read input until newline
        input.trim(); // Remove any leading/trailing whitespace

        int commaIndex = input.indexOf(','); // Find the position of the comma
        if (commaIndex > 0) {
            String qValue = input.substring(0, commaIndex); // Extract the part before the comma
            String rValue = input.substring(commaIndex + 1); // Extract the part after the comma

            float newQ = qValue.toFloat();
            float newR = rValue.toFloat();

            if (newQ > 0 && newR > 0) { // Ensure both Q and R are positive
                kalman_Q = newQ;
                kalman_R = newR;
                Serial.print("Updated kalman_Q to: ");
                Serial.println(kalman_Q);
                Serial.print("Updated kalman_R to: ");
                Serial.println(kalman_R);
            } else {
                Serial.println("Invalid values. Both Q and R must be positive.");
            }
        } else {
            Serial.println("Invalid format. Use '<Q>,<R>' (e.g., '0.01,0.09').");
        }
    }
}

/** 应用卡尔曼滤波器 */
float applyKalmanFilter(float measurement) {
    // --- 预测步骤 ---
    // 状态预测 (由于我们假设温度变化是随机游走, A=1, B=0, u=0)
    // x_hat_minus = x_hat; (上一时刻的后验估计作为当前时刻的先验估计)
    // 误差协方差预测
    float P_minus = kalman_P + kalman_Q;

    // --- 更新（校正）步骤 ---
    // 计算卡尔曼增益 K
    float K = P_minus / (P_minus + kalman_R);

    // 更新估计值
    kalman_x_hat = kalman_x_hat + K * (measurement - kalman_x_hat); // 使用上一时刻的x_hat作为先验估计

    // 更新误差协方差
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
/** 计算时间 */
void time_display() // Renamed from 'time'
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
    int days_in_current_month = Days(year_val, month_val); // Use current year and month
    int days_in_prev_month;
    if (month_val == 1)
        days_in_prev_month = Days(year_val - 1, 12);
    else
        days_in_prev_month = Days(year_val, month_val - 1);

    // Simplified logic for day increment based on 'd' (days passed since initial setting)
    // This part needs careful review if Set_Time is used to change DAY directly
    day_val = DAY + d; // Start with base day + days passed
    Serial.print(" day_val: ");
    Serial.print(day_val);
    Serial.print("  d: ");
    Serial.println(d);
    while (day_val > days_in_current_month) {
        day_val -= days_in_current_month;
        mon++; // Increment month carry
        // Update month and year if month carry causes overflow
        month_val = (MONTH + mon -1) % 12 + 1; // Corrected month calculation
        year_val = YEAR + (MONTH + mon -1) / 12;
        days_in_current_month = Days(year_val, month_val);

        //使用串口查看数据
        Serial.print(" month_val: ");
        Serial.print(month_val);    
        Serial.print("  year_val: ");
        Serial.print(year_val);
        Serial.print("  days_in_current_month: ");
        Serial.println(days_in_current_month);

    }
     while (day_val <= 0) { // Handles cases where day might be set to 0 or negative
        mon--; // Decrement month carry
        month_val = (MONTH + mon -1) % 12 + 1;
        if (month_val == 0) month_val = 12; // Month should be 1-12
        year_val = YEAR + (MONTH + mon -1) / 12;
        if (month_val == 12 && (MONTH + mon -1) < 0) year_val--; // Adjust year if month wrapped around backwards

        if (month_val == 1 && DAY + d <=0) { // If we went to Dec of prev year
             days_in_prev_month = Days(year_val, 12);
        } else if (DAY + d <=0) {
             days_in_prev_month = Days(year_val, month_val); // Or current month if it didn't wrap to prev year
        } else {
             days_in_prev_month = Days(year_val, month_val-1 == 0 ? 12 : month_val-1);
        }
        day_val += days_in_prev_month;
    }


    FormatDisplay(8, 0, day_val);
    Serial.print(" FormatDisplay day_val: ");
    Serial.print(day_val);
}
/** 计算月份 */
void Month_display() // Renamed from 'Month'
{
    // Month calculation is now partially handled in Day_display due to carry-over
    // This ensures month_val is updated before being displayed
    month_val = (MONTH + mon -1) % 12 + 1; // Month is 1-12
    if ((MONTH + mon -1) < 0 && month_val !=12 ) { // Handle negative modulo if needed
        int temp_mon = (MONTH + mon -1);
        while(temp_mon < 0) temp_mon += 12;
        month_val = temp_mon % 12 + 1;
    }


    y = (MONTH + mon - 1) / 12; // Calculate year carry
    FormatDisplay(5, 0, month_val);
    lcd.setCursor(7, 0);
    lcd.print('-');
}
/** 计算年份 */
void Year_display() // Renamed from 'Year'
{
    // Year calculation is now partially handled in Month_display due to carry-over
    year_val = YEAR + y;
    // Ensure year_val is consistent if changed in Day_display or Month_display logic
    // No simple modulo for year, it just increments/decrements

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
    if (Eff_m == 1 || Eff_m == 2) { // Zeller's congruence: Jan/Feb are 13th/14th month of prev year
        Eff_m += 12;
        Eff_y--;
    }
    int week_day_zeller = (d_val + (13 * (Eff_m + 1)) / 5 + Eff_y % 100 + (Eff_y % 100) / 4 + (Eff_y / 100) / 4 - 2 * (Eff_y / 100)) % 7;
    // Zeller gives 0 for Sat, 1 for Sun, ..., 6 for Fri. We want 1 for Mon.
    // (Original formula gives 1=Sun... 0=Sat, or 1=Mon... 7=Sun. Here it seems to be a variant)
    // The original formula was: (d + 2*m + 3*(m+1)/5 + y + y/4 - y/100 + y/400)%7 + 1 -> Mon=1
    // Let's stick to the provided one if it was working.
    // The formula in the original code: (d + 2*m + 3*(m+1)/5 + y + y/4 - y/100 + y/400)%7 + 1
    // Let's re-use that if m=1,2 are treated as 13,14 without year change.
    int temp_m = m_val;
    int temp_y = y_val; // Not changing year here as per original Week function
    if (temp_m == 1) temp_m = 13;
    if (temp_m == 2) temp_m = 14;
    // If using m=13,14 for Jan,Feb, the year should be y-1 for Zeller's.
    // The original `Week` function did NOT adjust `y` when `m` became 13 or 14.
    // This is unusual for Zeller's. If it worked, it was specific to that formula version.
    // Let's use the original formula directly.

    int original_m_for_week = m_val;
    if (original_m_for_week == 1) original_m_for_week = 13; // Does not change y_val for this formula version
    if (original_m_for_week == 2) original_m_for_week = 14;

    int week = (d_val + 2 * original_m_for_week + 3 * (original_m_for_week + 1) / 5 + y_val + y_val / 4 - y_val / 100 + y_val / 400) % 7 + 1;
    
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
    default: weekstr = "Err. "; break; // Should not happen
    }
    lcd.setCursor(11, 0);
    lcd.print(weekstr);
}
/** 显示时间、日期、星期 */
void Display()
{
    // Order matters for date calculation carries
    time_display(); // Calculates hour_val, minute_val, second_val, and 'd' carry
    Year_display(); // Sets year_val based on YEAR and 'y' carry
    Month_display();// Sets month_val based on MONTH and 'mon' carry, calculates 'y' carry for year
    Day_display();  // Sets day_val based on DAY and 'd' carry, calculates 'mon' carry for month
    Week(year_val, month_val, day_val);
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
void DisplayCursor(int rol, int row)
{   
    updateDisplayCursor_NonBlocking(rol, row, true); // 调用非阻塞光标更新函数
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

    // also set current values
    year_val = YEAR;
    month_val = MONTH;
    day_val = DAY;
    hour_val = HOUR;
    minute_val = MINUTE;
    second_val = SECOND;
}

/** 通过按键设置时间 */
// Note: This function directly modifies global HOUR, MINUTE etc.
// And also the base YEAR, MONTH, DAY.
// The time/date display functions use these base values + carries.
// This might lead to complex interactions if changing mid-calculation.
// It's safer if Set_Time_Value modifies year_val, month_val, etc.
// and then these are stored back to YEAR, MONTH or DS1302.
void Set_Time_Value(int rol, int row, int &Time_var) // Renamed from Set_Time
{
    DisplayCursor(rol, row);
    if (digitalRead(add) == LOW)
    {
        delay(ButtonDelay);
        if (digitalRead(add) == LOW)
        {
            Serial.println("Time_var++");
            Time_var++;
            // Add constraints for Time_var (e.g. hour 0-23, month 1-12 etc.)
            // This is critical. The original Set_Time_Value adjusted global HOUR, MINUTE, etc.
            // which were then used with modulo operations in display.
            // Here, if Time_var is, for example, HOUR, it needs to be handled.
        }
        Serial.print(" Display before : ");
        Serial.println(day_val);
        Display(); // Redisplay immediately. Handled by main loop's Display()
        Serial.print(" Display after : ");
        Serial.println(day_val);
    }
    if (digitalRead(minus) == LOW)
    {
        delay(ButtonDelay);
        if (digitalRead(minus) == LOW)
        {
            Serial.println("Time_var--");
            Time_var--;
             // Add constraints
        }
        Display();
    }
     // After changing YEAR, MONTH, DAY, HOUR, MINUTE, SECOND directly,
     // we need to reset the carry variables s,m,h,d,mon,y and seconds.
     // And update year_val, month_val etc.
     // This part requires careful rework if we allow direct modification of base values.
     // For now, this function will modify the base values.
     // The Display() function will recalculate current view based on these.
}
/** 按键选择 */
void Set_Clock()
{
    if (digitalRead(choose) == LOW)
    {
        delay(ButtonDelay); // Debounce
        if (digitalRead(choose) == LOW) { // Check again after debounce
            lcd.setCursor(9, 1);
            lcd.print("SetTime");
            Serial.println("SetTime");
            
            chose = 0; // Reset sub-menu choice

            unsigned long entryTime = millis();
            bool settingActive = true;

            while (settingActive) // Loop for setting time
            {
                // Serial.println("Setting time");
                Serial.print("chose++ = : ");
                        Serial.println(chose);
                if (millis() - entryTime > 15000 && chose == 0) { // Timeout if no selection after 15s
                    settingActive = false; // Exit set mode
                    Serial.println("SetTime timeout");
                    lcd.setCursor(9,1); lcd.print("TimeOut"); // Clear "SetTime"
                    break;
                }

                if (digitalRead(choose) == LOW)
                {
                    delay(ButtonDelay);
                    if (digitalRead(choose) == LOW)
                    {
                        chose++;
                        Serial.print("chose++ = : ");
                        Serial.println(chose);
                        entryTime = millis(); // Reset timeout on action
                        // Clear previous setting field indication or cursor for clarity
                        lcd.setCursor(9,1); lcd.print("Setting"); // Clear "SetTime" or previous field
                        if (chose >= 7) {
                            settingActive = false; // Exit set mode
                            chose = 0;
                        }
                    }
                }
                
                // Display current time while setting (updates continuously)
                // seconds = (millis() / 1000) - (millis() / 1000 % 60) + SECOND; // Try to keep seconds stable during set
                // This is complex, better to let it run, or briefly pause DS1302 sync
                unsigned long current_millis_sec = millis()/1000;
                if (seconds != current_millis_sec) { // update display once per second
                    seconds = current_millis_sec;
                    Serial.println("Storage millis");
                    Display(); // Update display
                    // Re-display "SetTime" or current field if needed
                    if(settingActive && chose == 0) {lcd.setCursor(9, 1); lcd.print("SetTime");}
                }


                // Handle current setting field
                // Important: When adjusting YEAR, MONTH, DAY, HOUR, MINUTE, SECOND directly,
                // the carry variables (s,m,h,d,mon,y) and the global `seconds` (from millis)
                // might need to be reset or re-synchronized to avoid large jumps.
                // Or, better: modify hour_val, minute_val etc. directly and then commit to DS1302 / base.
                // For this iteration, we modify base values.
                switch(chose) {
                    case 1: Set_Time_Value(0, 1, HOUR); break; // SetHour (modifies HOUR)
                    case 2: Set_Time_Value(3, 1, MINUTE); break; // SetMinute
                    case 3: Set_Time_Value(6, 1, SECOND); break; // SetSecond
                    case 4: Set_Time_Value(8, 0, DAY); break; // SetDay
                    case 5: Set_Time_Value(5, 0, MONTH); break; // SetMonth
                    case 6: Set_Time_Value(0, 0, YEAR); break; // SetYear
                    default: break; // chose = 0 (main SetTime screen) or chose >= 7 (exit)
                }
                //  // Apply constraints after Set_Time_Value
                // if (HOUR > 23) HOUR = 0; if (HOUR < 0) HOUR = 23;
                // if (MINUTE > 59) MINUTE = 0; if (MINUTE < 0) MINUTE = 59;
                // if (SECOND > 59) SECOND = 0; if (SECOND < 0) SECOND = 59;
                // if (MONTH > 12) MONTH = 1; if (MONTH < 1) MONTH = 12;
                // if (DAY > Days(YEAR, MONTH)) DAY = 1; // Basic check
                // if (DAY < 1) DAY = Days(YEAR, MONTH);   // Basic check
                // if (YEAR > 9999) YEAR = 2000; if (YEAR < 0) YEAR = 2024;
                if (!settingActive) break; // Exit if flag is false
                delay(50); // Small delay to make button presses more manageable
            }
            // After exiting set mode:
            // Reset carries and seconds to reflect the new base time
            seconds = millis()/1000; // Re-sync seconds count start
            s = 0; m = 0; h = 0; d = 0; mon = 0; y = 0;
            
            // // Update current display values immediately from new base
            // year_val = YEAR; month_val = MONTH; day_val = DAY;
            // hour_val = HOUR; minute_val = MINUTE; second_val = SECOND;

            // Write the new time to DS1302
            rtc.setTime(hour_val, minute_val, second_val);
            rtc.setDate(year_val, month_val, day_val);

            lcd.setCursor(9,1); lcd.print("        "); // Clear "SetTime"
            Display(); // Final display update
            delay(200); // Debounce for choose button release
        }
    }
}

/** 设置闹钟小时 */
void Set_Alarm_Hour()
{
    DisplayCursor(0, 1); // Cursor at hour part of alarm display
    if (digitalRead(add) == LOW)
    {
        delay(ButtonDelay);
        if (digitalRead(add) == LOW)
        {
            alarm_hour = (alarm_hour + 1) % 24;
            FormatDisplay(0, 1, alarm_hour); // Display on LCD
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
    DisplayCursor(3, 1); // Cursor at minute part
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
    DisplayCursor(9, 1); // Cursor at temperature part
    if (digitalRead(add) == LOW)
    {
        delay(ButtonDelay);
        if (digitalRead(add) == LOW) {
            Temp_Alarm++;
            if (Temp_Alarm > 99) Temp_Alarm = 99; // Example limit
            lcd.setCursor(9,1); if(Temp_Alarm < 10 && Temp_Alarm >=0) lcd.print("0"); lcd.print(Temp_Alarm,0); // Display without decimal for simplicity
        }
    }
    if (digitalRead(minus) == LOW)
    {
        delay(ButtonDelay);
        if (digitalRead(minus) == LOW) {
            Temp_Alarm--;
            if (Temp_Alarm < 0) Temp_Alarm = 0; // Example limit
            lcd.setCursor(9,1); if(Temp_Alarm < 10 && Temp_Alarm >=0) lcd.print("0"); lcd.print(Temp_Alarm,0);
        }
    }
}
/** 进入报警设置 */
void Set_Alarm()
{
    if (digitalRead(add) == LOW && digitalRead(minus) == LOW)
    {
        delay(ButtonDelay*2); // Longer delay for two-button press
        if (digitalRead(add) == LOW && digitalRead(minus) == LOW) { // Check again
            alarm_choose = 0; // Reset sub-menu
            
            // Temporarily use current time for initial alarm display if desired
            // alarm_hour = hour_val;
            // alarm_minute = minute_val;

            lcd.clear();
            lcd.setCursor(0, 0); lcd.print("Set Alarm:");
            // Display initial alarm time and temp
            FormatDisplay(0, 1, alarm_hour); lcd.print(":"); FormatDisplay(3, 1, alarm_minute); lcd.print(":00"); // Seconds fixed to 00
            lcd.setCursor(9,1); if(Temp_Alarm < 10 && Temp_Alarm >=0) lcd.print("0"); lcd.print(Temp_Alarm,0); lcd.print((char)223); lcd.print("C");

            unsigned long entryTime = millis();
            bool settingActive = true;

            while (settingActive)
            {
                 if (millis() - entryTime > 15000 && alarm_choose == 0) { // Timeout if no selection
                    settingActive = false;
                    break;
                }

                if (digitalRead(choose) == LOW)
                {
                    delay(ButtonDelay);
                    if (digitalRead(choose) == LOW)
                    {
                        alarm_choose++;
                        entryTime = millis(); // Reset timeout
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
                    default: break; // alarm_choose = 0 or exit
                }
                if(!settingActive) break;
                delay(50);
            }
            lcd.clear(); // Clear alarm set screen
            Display();   // Restore main display
            delay(200); // Debounce for choose button release
        }
    }
}

/** 正点蜂鸣 */
void Point_Time_Alarm()
{
    if (minute_val == 0 && second_val == 0 && hour_val != HOUR) // Avoid chime when setting time around xx:00:00
    {
        tone(Tone, frequence);
        delay(500);
        noTone(Tone);
    }
}
/** 闹钟 指定时间蜂鸣 */
void Clock_Alarm()
{
    if (hour_val == alarm_hour && minute_val == alarm_minute && second_val == alarm_second)
    {
        unsigned long alarmStartTime = millis();
        while(millis() - alarmStartTime < 5000) { // Beep for 5 seconds
            tone(Tone, frequence, 250); // Beep for 250ms
            delay(500); // Pause 250ms
            if(digitalRead(choose) == LOW || digitalRead(add) == LOW || digitalRead(minus) == LOW) { // Any button stops alarm
                noTone(Tone);
                delay(ButtonDelay); // Debounce
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
    Temperatures = (500.0 * a) / 1023.0; // Raw temperature

    float filteredTemperature = applyKalmanFilter(Temperatures);

    // Display the filtered temperature
    lcd.setCursor(9, 1);
    if (filteredTemperature < 0 && filteredTemperature > -10) lcd.print("-0"); // for -0.xx display
    else if (filteredTemperature >= 0 && filteredTemperature < 10) lcd.print("0");
    
    lcd.print(filteredTemperature, 1); // Display with 1 decimal place

    // Ensure enough space for temp (e.g., "0_._C" or "-_._C" or "__._C")
    // Clear trailing characters if previous temp was longer
    // Example: if temp was 10.5C then 9.5C, clear the '1' position.
    // A simple way is to print spaces after, or ensure fixed width print.
    // For "xx.x", it's 4 chars. If we show "05.2C", it's 5 chars.
    // Let's assume max " -DD.DC" -> 6 chars for display including symbol
    // lcd.print("    "); // Clear a few spaces
    // lcd.setCursor(9,1); // Reset cursor (already there)
    // then print the value. The current print will overwrite.

    lcd.setCursor(14, 1); // Position for degree symbol might need adjustment based on length of temp
    lcd.print((char)223); // Display ° symbol
    lcd.setCursor(15, 1);
    lcd.print("C");       // Display C

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
    // Use filtered temperature for alarm to avoid spurious alarms from noise
    if (kalman_x_hat >= Temp_Alarm) 
    {
        tone(Tone, frequence, 250); // Beep for 250ms
        delay(500); // Total 500ms cycle, then off
        noTone(Tone); // Ensure tone is off if not re-triggered next loop
    }
}