// #include <Arduino.h>
// #include <DS1302.h>
// #include <LiquidCrystal.h> // LCD1602 显示库
// #include <U8g2lib.h>       // U8g2 OLED库

// // --- U8g2 OLED 对象 ---
// // 使用1级页缓冲以最小化RAM占用。如果您使用RST引脚，请调整。
// U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* rst */ U8X8_PIN_NONE);

// // --- OLED 英文文本显示数据 ---
// const char* oledDisplayTexts[] = {
//     "WuBixing6666",
//     "WangJiayin6666"
// };
// const int NUM_OLED_TEXTS = sizeof(oledDisplayTexts) / sizeof(oledDisplayTexts[0]);
// int currentOledTextIndex = 0;
// unsigned long lastOledTextChangeTime = 0;
// #define OLED_TEXT_DISPLAY_DURATION 3000 // 每个字符串显示3秒 (ms)

// // --- LCD1602 引脚定义 ---
// #define LCD_RS_PIN 12
// #define LCD_EN_PIN 11
// #define LCD_D4_PIN 5
// #define LCD_D5_PIN 4
// #define LCD_D6_PIN 3
// #define LCD_D7_PIN 2

// // --- 按键引脚定义 ---
// #define BUTTON_CHOOSE_PIN A0 // “选择/切换” 按键
// #define BUTTON_ADD_PIN A1    // “加” 按键
// #define BUTTON_MINUS_PIN A2  // “减” 按键

// // --- 蜂鸣器引脚定义 ---
// #define BUZZER_TONE_PIN 13

// // --- DS1302 RTC 时钟模块引脚定义 ---
// #define DS1302_CE_PIN 8   // RST (Chip Enable)
// #define DS1302_IO_PIN 9   // DAT (Data I/O)
// #define DS1302_SCLK_PIN 10 // CLK (Serial Clock)

// // --- 温度传感器引脚定义 ---
// #define TEMP_SENSOR_PIN A3

// // --- 常量定义 ---
// #define BUTTON_DEBOUNCE_DELAY 70  // 按键消抖延时 (毫秒)
// #define DEFAULT_ALARM_TEMP 40.0f  // 默认的报警温度阈值
// #define HOURLY_CHIME_FREQ 2093    // 整点报时蜂鸣器频率 (Hz)
// #define ALARM_SOUND_FREQ 2093     // 时间闹钟和温度报警蜂鸣器频率 (Hz)
// #define ALARM_DURATION 5000       // 时间闹钟鸣响持续时间 (毫秒)
// #define HOURLY_CHIME_DURATION 500 // 整点报时鸣响持续时间 (毫秒)
// #define TEMP_ALARM_DURATION 500   // 温度报警单次鸣响持续时间 (毫秒)



// // 初始化LCD对象
// LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

// // 按键引脚别名
// #define choose BUTTON_CHOOSE_PIN
// #define add BUTTON_ADD_PIN
// #define minus BUTTON_MINUS_PIN
// #define TonePin BUZZER_TONE_PIN // 重命名以避免与 tone() 函数潜在冲突

// // DS1302 RTC 对象
// DS1302 rtc(DS1302_CE_PIN, DS1302_IO_PIN, DS1302_SCLK_PIN);

// bool inSetAlarmMode = false;

// // 全局时间日期变量
// int second_val = 0, minute_val = 0, hour_val = 0, day_val = 0, month_val = 0, year_val = 0;
// // 用于设置模式下暂存用户修改的值
// int SECOND_BASE = 0, MINUTE_BASE = 0, HOUR_BASE = 0, DAY_BASE = 0, MONTH_BASE = 0, YEAR_BASE = 0;

// // 菜单及设置相关变量
// int chose_menu = 0;        // 时间设置菜单中的当前选择项 (0:未选择, 1:时, ...)
// int alarm_menu_item = 0;   // 闹钟设置菜单中的当前选择项 (0:未选择, 1:闹钟时, ...)
// int ButtonDelay = BUTTON_DEBOUNCE_DELAY; // 按键消抖延时副本
// int ChimeFrequence = HOURLY_CHIME_FREQ; // 报时频率副本

// // 闹钟和报警温度变量
// int alarm_hour_set = 7, alarm_minute_set = 30, alarm_second_set = 0; // 默认闹钟时间
// float Temperatures_current;                  // 当前读取到的温度值
// float Temp_Alarm_Threshold = DEFAULT_ALARM_TEMP; // 当前报警温度阈值

// // LCD光标闪烁相关变量
// static unsigned long blink_lastTime = 0;
// static bool blink_cursorState = false;
// static int blink_currentRol = -1;  // 记录光标当前列
// static int blink_currentRow = -1; // 记录光标当前行
// const unsigned int CURSOR_BLINK_INTERVAL = 500; // 光标闪烁间隔 (ms)

// // 卡尔曼滤波参数 (用于温度传感器数据平滑)
// float kalman_x_hat = 0;    // 卡尔曼滤波后的估计值
// float kalman_P = 1.0;      // 估计误差协方差
// float kalman_Q = 0.0001;   // 过程噪声协方差 (越小，越相信预测模型，响应越慢但越平滑)
// float kalman_R = 0.09;     // 测量噪声协方差 (越小，越相信测量值，响应越快但噪声可能越大)
// static bool inSetClockMode = false;  // 是否处于时钟设置模式
// // --- 函数声明 ---
// // OLED Functions
// void updateOledTextDisplayState();
// void drawOledEnglishText(); 

// // LCD Display Functions
// void FormatDisplay(int col, int row, int num);
// void time_display_update_format();
// int DaysInMonth(int current_year, int current_month);
// void Day_display_update_format();
// void Month_display_update_format();
// void Year_display_update_format();
// void Week_display_update(int y_val, int m_val, int d_val);
// void DisplayAll(bool refresh_from_rtc); // true: 从RTC刷新, false: 用当前BASE值显示
// void updateDisplayCursor_NonBlocking(int rol, int row, bool enabled);

// // RTC & Time Setting Functions
// void setBaseTimeAndRTC(int y_val, int mon_val, int d_val, int h_val, int m_val, int s_val);
// void Set_Clock_Mode();
// void Set_Alarm_Mode();

// // Alarm Check Functions
// void Point_Time_Alarm_check(); // 整点报时
// void Clock_Alarm_check();      // 时间闹钟
// void Temperatures_Alarm_check(); // 温度报警

// // Temperature Processing Functions
// void GetTemperatures_update();
// float applyKalmanFilter(float measurement);
// void kalmanProcess_Serial();   // 通过串口调整卡尔曼参数

// // 初始化设置函数
// void setup() {
//     Serial.begin(9600);
//     // OLED 初始化
//     u8g2.begin();
//     u8g2.setDrawColor(1); // 像素亮
//     lastOledTextChangeTime = millis(); // 初始化OLED文本切换计时器

//     // 初始化数字引脚模式 (LCD, 蜂鸣器等)
//     for (int i = 2; i <= 13; i++) { // Arduino UNO/Nano 数字引脚 2-13
//         // 跳过特定功能的引脚，这些引脚模式会单独设置
//         if (i != DS1302_IO_PIN && i != BUTTON_CHOOSE_PIN && i != BUTTON_ADD_PIN && i != BUTTON_MINUS_PIN && i != BUZZER_TONE_PIN) {
//              // 注意: TEMP_SENSOR_PIN 是模拟输入, DS1302_SCLK_PIN, DS1302_CE_PIN 通常由库或直接操作管理
//             if (i == LCD_RS_PIN || i == LCD_EN_PIN || i == LCD_D4_PIN || i == LCD_D5_PIN || i == LCD_D6_PIN || i == LCD_D7_PIN) {
//                  pinMode(i, OUTPUT); // LCD引脚设为输出
//             } else if (i == DS1302_SCLK_PIN || i == DS1302_CE_PIN) {
//                  pinMode(i, OUTPUT); // DS1302 CLK 和 CE 设为输出
//             }
//         }
//     }
//     pinMode(DS1302_IO_PIN, OUTPUT); // DS1302 IO引脚初始为输出
//     pinMode(BUZZER_TONE_PIN, OUTPUT); // 蜂鸣器引脚设为输出

//     // 初始化按键引脚为输入上拉模式
//     pinMode(add, INPUT_PULLUP);
//     pinMode(minus, INPUT_PULLUP);
//     pinMode(choose, INPUT_PULLUP);
//     pinMode(TEMP_SENSOR_PIN, INPUT); // 温度传感器引脚为输入

//     // LCD 初始化
//     lcd.begin(16, 2); // 初始化LCD1602屏幕 (16列 x 2行)
    
//     // RTC 初始化
//     rtc.writeProtect(false); // 关闭DS1302写保护
//     rtc.halt(false);         // 启动DS1302时钟

//     // 从RTC读取初始时间并设置系统时间变量
//     Time t_rtc = rtc.getTime();
//     // DS1302库可能返回两位数年份，需要转换为四位数
//     YEAR_BASE = (t_rtc.year < 100) ? t_rtc.year + 2000 : t_rtc.year; 
//     MONTH_BASE = t_rtc.mon; 
//     DAY_BASE = t_rtc.date;
//     HOUR_BASE = t_rtc.hour; 
//     MINUTE_BASE = t_rtc.min; 
//     SECOND_BASE = t_rtc.sec;
//     // 将读取到的时间（或默认时间）设为当前时间和RTC时间
//     setBaseTimeAndRTC(YEAR_BASE, MONTH_BASE, DAY_BASE, HOUR_BASE, MINUTE_BASE, SECOND_BASE);

//     // 初始化卡尔曼滤波器的初始温度估计
//     long initial_adc_val = analogRead(TEMP_SENSOR_PIN);
//     // 假设传感器为LM35 (10mV/°C), Arduino参考电压5V, ADC 10位
//     kalman_x_hat = (5.0 * initial_adc_val * 100.0) / 1024.0; 

//     Serial.begin(9600); // 初始化串口通讯
//     updateDisplayCursor_NonBlocking(0, 0, false); // 关闭光标初始状态
// }

// // 主循环函数
// void loop() {
//     // 仅当不在任何设置模式时，刷新LCD显示并执行检测任务
    
    
//     // 检查并处理是否进入时间设置模式或闹钟设置模式
//     if(!inSetAlarmMode){
//         if (chose_menu == 0 && alarm_menu_item == 0) {
//             DisplayAll(true); // 从RTC刷新数据来显示
//         }
//         Set_Clock_Mode(); 
//     }
//     Set_Alarm_Mode(); 

//     // 仅当不在任何设置模式时，执行闹钟和温度检测
//     if (chose_menu == 0 && alarm_menu_item == 0 && (!inSetAlarmMode)) {
//         Point_Time_Alarm_check();   // 整点报时检查
//         Clock_Alarm_check();        // 时间闹钟检查
//         GetTemperatures_update();   // 更新温度显示
//         Temperatures_Alarm_check(); // 温度超限报警检查
//     }
    
//     kalmanProcess_Serial(); // 通过串口调整卡尔曼参数

//     // OLED 文本循环显示逻辑
//     updateOledTextDisplayState(); 

//     u8g2.firstPage();
//     do {
//         drawOledEnglishText(); 
//     } while (u8g2.nextPage());

//     delay(50); // 主循环延时，平衡响应与功耗
// }

// // --- OLED 相关函数 --- 
// void updateOledTextDisplayState() {
//     // 每隔一段时间切换显示的文本
//     if (millis() - lastOledTextChangeTime > OLED_TEXT_DISPLAY_DURATION) {
//         lastOledTextChangeTime = millis();
//         currentOledTextIndex = (currentOledTextIndex + 1) % NUM_OLED_TEXTS;
//     }
// }

// void drawOledEnglishText() {
//     // 设置OLED字体
//     u8g2.setFont(u8g2_font_profont17_tf); 
//     u8g2.setFontMode(1); // 设置为透明字体背景，避免覆盖

//     const char* currentText = oledDisplayTexts[currentOledTextIndex];
    
//     // 获取文本尺寸信息
//     u8g2_uint_t textWidth = u8g2.getStrWidth(currentText); 
//     u8g2_uint_t textAscent = u8g2.getAscent();    // 从基线到字符顶部的距离
//     u8g2_uint_t textDescent = u8g2.getDescent();  // 从基线到字符最底部的距离 (通常为负或0)
//     u8g2_uint_t textHeight = textAscent - textDescent; // 文本的实际像素高度

//     // --- 添加装饰线条 ---
//     int linePadding = 3; // 线条与文本之间的垂直间距
//     int lineThickness = 1; // 线条厚度 (drawHLine总是1像素厚)
//     u8g2_uint_t decorativeLineWidth = u8g2.getDisplayWidth() * 0.95; // 线条宽度为屏幕的75%
//     u8g2_uint_t lineXStart = (u8g2.getDisplayWidth() - decorativeLineWidth) / 2; // 线条水平居中

//     // 计算包含线条和文本的整个内容块的总高度
//     u8g2_uint_t totalBlockHeight = textHeight + (2 * linePadding) + (2 * lineThickness);
    
//     // 计算整个内容块垂直居中的起始Y坐标
//     u8g2_uint_t blockYStart = (u8g2.getDisplayHeight() - totalBlockHeight) / 2;

//     // 计算各元素的Y坐标
//     u8g2_uint_t yTopLine = blockYStart;
//     // 文本基线的Y坐标 = 块顶部Y + 上边线厚度 + 上边距 + 字体上升高度
//     u8g2_uint_t yTextBaseline = blockYStart + lineThickness + linePadding + textAscent;
//     // 下边线的Y坐标 = 文本基线Y + 字体下降高度 + 下边距 
//     // 或者: 块顶部Y + 上边线厚度 + 上边距 + 文本总高度 + 下边距
//     u8g2_uint_t yBottomLine = blockYStart + lineThickness + linePadding + textHeight + linePadding;

//     // 绘制顶线 (确保在屏幕内)
//     if (yTopLine >=0 && yTopLine < u8g2.getDisplayHeight()) { 
//          u8g2.drawHLine(lineXStart, yTopLine, decorativeLineWidth);
//     }

//     // 绘制文本 (水平居中)
//     u8g2_uint_t textXPos = (u8g2.getDisplayWidth() - textWidth) / 2;
//     if (textXPos < 0) textXPos = 0; // 如果文本太宽，则从左边0位置开始显示
//     // 确保文本的基线位置在屏幕的有效绘制区域内
//     if (yTextBaseline >= textAscent && yTextBaseline < u8g2.getDisplayHeight() + textDescent ) {
//         u8g2.drawStr(textXPos, yTextBaseline, currentText); 
//     }


//     // 绘制底线 (确保在屏幕内)
//     if (yBottomLine >=0 && yBottomLine < u8g2.getDisplayHeight()) { 
//         u8g2.drawHLine(lineXStart, yBottomLine, decorativeLineWidth);
//     }
// }


// // --- 卡尔曼滤波相关函数 ---
// void kalmanProcess_Serial() {
// }

// float applyKalmanFilter(float measurement) {
//     // 预测步骤
//     float P_minus = kalman_P + kalman_Q; // P_k|k-1 = P_k-1|k-1 + Q
//     // 更新步骤
//     float K = P_minus / (P_minus + kalman_R); // K_k = P_k|k-1 / (P_k|k-1 + R)
//     kalman_x_hat = kalman_x_hat + K * (measurement - kalman_x_hat); // x_k|k = x_k|k-1 + K_k * (z_k - H*x_k|k-1) (H=1 here)
//     kalman_P = (1 - K) * P_minus; // P_k|k = (I - K_k*H) * P_k|k-1 (H=1 here)
//     return kalman_x_hat;
// }

// // --- LCD 显示更新相关函数 ---
// // 在LCD指定位置显示数字，不足两位的前面补'0'
// void FormatDisplay(int col, int row, int num) { 
//     lcd.setCursor(col, row);
//     if (num < 10 && num >= 0) // 只对0-9的数字补零
//         lcd.print("0");
//     lcd.print(num);
// }

// // 格式化并显示时间 (HH:MM:SS)
// void time_display_update_format() { 
//     FormatDisplay(0, 1, hour_val);
//     lcd.setCursor(2, 1); lcd.print(":");
//     FormatDisplay(3, 1, minute_val);
//     lcd.setCursor(5, 1); lcd.print(":");
//     FormatDisplay(6, 1, second_val);
// }

// // 返回指定年月的总天数，考虑闰年
// int DaysInMonth(int current_year, int current_month) { 
//     if (current_month < 1 || current_month > 12) return 30; // 异常月份处理
//     if (current_month == 2) { // 二月
//         if ((current_year % 4 == 0 && current_year % 100 != 0) || (current_year % 400 == 0))
//             return 29; // 闰年29天
//         else
//             return 28; // 平年28天
//     } else if (current_month == 4 || current_month == 6 || current_month == 9 || current_month == 11) {
//         return 30; // 4,6,9,11月为30天
//     } else {
//         return 31; // 其他月份为31天
//     }
// }

// // 格式化并显示日期 (日)
// void Day_display_update_format() { 
//     FormatDisplay(8, 0, day_val);
// }

// // 格式化并显示月份
// void Month_display_update_format() { 
//     FormatDisplay(5, 0, month_val);
//     lcd.setCursor(7, 0); lcd.print('-'); // 月日之间的分隔符
// }

// // 格式化并显示年份 (YYYY)
// void Year_display_update_format() { 
//     lcd.setCursor(0, 0);
//     // 确保年份显示为四位数
//     if (year_val < 10) lcd.print("000"); 
//     else if (year_val < 100) lcd.print("00");
//     else if (year_val < 1000) lcd.print("0");
//     lcd.print(year_val);
//     lcd.setCursor(4, 0); lcd.print('-'); // 年月之间的分隔符
// }

// // 根据年月日计算并显示星期 (Zeller公式)
// void Week_display_update(int y_disp, int m_disp, int d_disp) { 
//     int M = m_disp;
//     int Y = y_disp;
//     // Zeller公式要求1月和2月视为上一年的13月和14月
//     if (M < 3) { 
//         M += 12; 
//         Y--; 
//     } 
//     int K = Y % 100;     // 年份的后两位
//     int J = Y / 100;     // 世纪数 (年份的前两位)
//     // Zeller公式计算结果 (0=周六, 1=周日, ..., 6=周五)
//     int dayOfWeekZeller = (d_disp + (13 * (M + 1)) / 5 + K + K / 4 + J / 4 + (5 * J)) % 7; 

//     int rtcDowEquiv; // 将Zeller结果映射为DS1302库星期表示 (1=周日, ..., 7=周六)
//     // DS1302星期定义：1=Sunday, 2=Monday, ..., 7=Saturday
//     // Zeller星期定义：0=Saturday, 1=Sunday, ..., 6=Friday
//     switch(dayOfWeekZeller) {
//         case 0: rtcDowEquiv = 7; break; // Zeller Sat -> RTC Sat (7)
//         case 1: rtcDowEquiv = 1; break; // Zeller Sun -> RTC Sun (1)
//         case 2: rtcDowEquiv = 2; break; // Zeller Mon -> RTC Mon (2)
//         case 3: rtcDowEquiv = 3; break; // Zeller Tue -> RTC Tue (3)
//         case 4: rtcDowEquiv = 4; break; // Zeller Wed -> RTC Wed (4)
//         case 5: rtcDowEquiv = 5; break; // Zeller Thu -> RTC Thu (5)
//         case 6: rtcDowEquiv = 6; break; // Zeller Fri -> RTC Fri (6)
//         default: rtcDowEquiv = 1; // 默认周日
//     }

//     String weekstr = ""; // 注意: String对象在RAM有限设备上应谨慎使用
//     switch (rtcDowEquiv) { 
//         case 1: weekstr = "Sun. "; break; case 2: weekstr = "Mon. "; break;
//         case 3: weekstr = "Tues."; break; case 4: weekstr = "Wed. "; break;
//         case 5: weekstr = "Thur."; break; case 6: weekstr = "Fri. "; break;
//         case 7: weekstr = "Sat. "; break; default: weekstr = "Err. "; break;
//     }
//     lcd.setCursor(11, 0); lcd.print(weekstr); // 显示星期字符串
// }

// // 刷新LCD上所有信息
// void DisplayAll(bool refresh_from_rtc) {
//     if (refresh_from_rtc) {
//         Time tm = rtc.getTime(); 
//         // DS1302库返回的年份可能是两位数 (如23代表2023)，需转换为四位数
//         year_val = (tm.year < 100) ? tm.year + 2000 : tm.year; 
//         month_val = tm.mon; 
//         day_val = tm.date;
//         hour_val = tm.hour; 
//         minute_val = tm.min; 
//         second_val = tm.sec;
//     }

//     // 调用各部分的格式化显示函数
//     time_display_update_format();
//     Year_display_update_format();
//     Month_display_update_format();
//     Day_display_update_format();
//     Week_display_update(year_val, month_val, day_val); 
// }

// // 非阻塞方式更新LCD光标的闪烁
// void updateDisplayCursor_NonBlocking(int rol, int row, bool enabled) { 
//     unsigned long currentTime = millis();
//     if (!enabled) { // 如果要求关闭光标
//         if (blink_currentRol != -1) { // 如果光标之前是激活的
//             // lcd.setCursor(blink_currentRol, blink_currentRow); // 光标已在原位，无需移动
//             lcd.noCursor(); // 关闭光标
//             blink_currentRol = -1; blink_currentRow = -1; // 重置光标位置记录
//             blink_cursorState = false;
//         }
//         return;
//     }

//     // 如果目标闪烁位置改变，或之前是关闭状态现在要开启
//     if (blink_currentRol != rol || blink_currentRow != row || blink_currentRol == -1) {
//         if (blink_currentRol != -1) { // 如果旧位置有效，先关闭旧位置的光标
//             // lcd.setCursor(blink_currentRol, blink_currentRow); // 光标已在原位
//             lcd.noCursor();
//         }
//         blink_currentRol = rol; blink_currentRow = row; // 更新为新位置
//         lcd.setCursor(blink_currentRol, blink_currentRow); // 移动到新位置
//         lcd.cursor(); // 立即显示光标
//         blink_cursorState = true; // 标记为显示状态
//         blink_lastTime = currentTime; // 重置闪烁计时器
//         return;
//     }

//     // 位置未变，且光标已启用，检查是否到了切换闪烁状态的时间
//     if (currentTime - blink_lastTime >= CURSOR_BLINK_INTERVAL) {
//         blink_lastTime = currentTime; // 更新上次切换时间
//         blink_cursorState = !blink_cursorState; // 翻转光标状态
//         lcd.setCursor(blink_currentRol, blink_currentRow); // 确保在正确位置操作
//         if (blink_cursorState) {
//             lcd.cursor();      
//         } else {
//             lcd.noCursor(); 
//         }
//     }
// }

// // 设置系统的基准时间 (BASE变量)，并将这些值赋给当前显示变量(_val)和写入RTC芯片
// void setBaseTimeAndRTC(int y_val_set, int mon_val_set, int d_val_set, int h_val_set, int m_val_set, int s_val_set) {
//     YEAR_BASE = y_val_set; MONTH_BASE = mon_val_set; DAY_BASE = d_val_set;
//     HOUR_BASE = h_val_set; MINUTE_BASE = m_val_set; SECOND_BASE = s_val_set;

//     // 将BASE值同步到当前显示值_val
//     year_val = YEAR_BASE; month_val = MONTH_BASE; day_val = DAY_BASE;
//     hour_val = HOUR_BASE; minute_val = MINUTE_BASE; second_val = SECOND_BASE;
    
//     // 准备写入RTC的年份 (DS1302库通常需要两位数年份)
//     int yearForRTC = (YEAR_BASE >= 2000) ? YEAR_BASE - 2000 : YEAR_BASE;
//     if (yearForRTC > 99) yearForRTC = yearForRTC % 100; // 确保是两位数

//     rtc.setTime(hour_val, minute_val, second_val); 
//     // DS1302.h (Rinky-Dink Electronics) setDate 接受 (day, month, year_yy)
//     rtc.setDate(day_val, month_val, yearForRTC);
// }

// // 时间设置模式处理函数
// void Set_Clock_Mode() {
//     static bool lastAddState_SC = HIGH;       
//     static bool lastMinusState_SC = HIGH;      
//     static bool lastChooseState_SC_ModeEntry = HIGH; 
//     static bool lastChooseState_SC_Item = HIGH; 
         
//     static unsigned long entryTime_SC;          

//     bool currentChooseStateModeEntry = digitalRead(choose); 

//     // 检测是否按下 "选择" 键进入时间设置模式
//     if (!inSetClockMode && currentChooseStateModeEntry == LOW && lastChooseState_SC_ModeEntry == HIGH) {
//         delay(ButtonDelay); // 消抖
//         if (digitalRead(choose) == LOW) { // 再次确认按下
//             inSetClockMode = true;  
//             chose_menu = 0; // 初始未选择任何具体项 (时/分/秒等)          
//             // Serial.print("In set clock mode1\n");
            
//             // 将当前RTC时间载入BASE变量作为编辑起点
//             Time tempT = rtc.getTime(); 
//             YEAR_BASE = (tempT.year < 100) ? tempT.year + 2000 : tempT.year; // 确保是4位数
//             MONTH_BASE = tempT.mon; DAY_BASE = tempT.date;
//             HOUR_BASE = tempT.hour; MINUTE_BASE = tempT.min; SECOND_BASE = tempT.sec;
           
//             // 将BASE值同步到_val变量，用于设置模式下的显示预览
//             year_val = YEAR_BASE; month_val = MONTH_BASE; day_val = DAY_BASE;
//             hour_val = HOUR_BASE; minute_val = MINUTE_BASE; second_val = SECOND_BASE;

//             // 重置模式内按键状态和超时计时器
//             lastChooseState_SC_Item = HIGH;
//             lastAddState_SC = HIGH;
//             lastMinusState_SC = HIGH;
//             entryTime_SC = millis();
            
            
//             lcd.clear();            
//             DisplayAll(false); // 显示当前待设置的时间 (不从RTC刷新)     
//             lcd.setCursor(9, 1); lcd.print("       "); // 清理右下角区域
//             lcd.setCursor(9, 1); lcd.print("SELECT");  // 提示按"选择"键切换项目
//         }
//     }
//     lastChooseState_SC_ModeEntry = currentChooseStateModeEntry; // 更新用于模式进入判断的按键状态

//     if (inSetClockMode) { // 如果已处于时间设置模式
//         // 超时退出逻辑: 如果15秒没有操作且当前未选中具体设置项
//         // Serial.print("In set clock mode2\n");
//         if (millis() - entryTime_SC > 15000 && chose_menu == 0) {
//             inSetClockMode = false; chose_menu = 0; 
//             updateDisplayCursor_NonBlocking(0, 0, false); 
//             lcd.setCursor(9, 1); lcd.print("TimeOut");    
//             delay(1000);
//             DisplayAll(true); // 恢复正常时间显示 (从RTC刷新)
//             return;
//         }

//         // 读取模式内各按键状态
//         bool currentChooseStateItem = digitalRead(choose);
//         bool currentAddState = digitalRead(add);
//         bool currentMinusState = digitalRead(minus);

//         // "选择"键用于切换设置项 (时、分、秒、日、月、年)
//         if (currentChooseStateItem == LOW && lastChooseState_SC_Item == HIGH) {
//             delay(ButtonDelay);
//             if (digitalRead(choose) == LOW) {
//                 chose_menu++; // 切换到下一个设置项             
//                 entryTime_SC = millis(); // 重置超时计时  
//                 if (chose_menu >= 7) { // 如果完成所有项或超出范围，则保存并退出
//                     inSetClockMode = false; chose_menu = 0;
//                     updateDisplayCursor_NonBlocking(0, 0, false);
//                     setBaseTimeAndRTC(YEAR_BASE, MONTH_BASE, DAY_BASE, HOUR_BASE, MINUTE_BASE, SECOND_BASE); 
//                     lcd.setCursor(9, 1); lcd.print("SAVED  "); 
//                     delay(1000);
//                     DisplayAll(true); // 恢复正常显示
//                     return;
//                 }
//                 lcd.setCursor(9, 1); lcd.print("       "); // 清理提示区
//             }
//         }
//         lastChooseState_SC_Item = currentChooseStateItem;

//         int* targetVar = nullptr; // 指向当前要修改的时间变量
//         int cursorCol = -1, cursorRow = -1; // 光标位置
//         int maxVal = 0, minVal = 0;         // 当前设置项的最大最小值       

//         // 根据 chose_menu 确定当前设置哪个时间单位
//         switch (chose_menu) {
//             case 1: targetVar = &HOUR_BASE;   cursorCol = 0; cursorRow = 1; maxVal = 23; minVal = 0;  lcd.setCursor(9,1); lcd.print("SetHour"); break;
//             case 2: targetVar = &MINUTE_BASE; cursorCol = 3; cursorRow = 1; maxVal = 59; minVal = 0;  lcd.setCursor(9,1); lcd.print("SetMin "); break;
//             case 3: targetVar = &SECOND_BASE; cursorCol = 6; cursorRow = 1; maxVal = 59; minVal = 0;  lcd.setCursor(9,1); lcd.print("SetSec "); break;
//             case 4: targetVar = &DAY_BASE;    cursorCol = 8; cursorRow = 0; maxVal = DaysInMonth(YEAR_BASE, MONTH_BASE); minVal = 1; lcd.setCursor(9,1); lcd.print("SetDay "); break;
//             case 5: targetVar = &MONTH_BASE;  cursorCol = 5; cursorRow = 0; maxVal = 12; minVal = 1;  lcd.setCursor(9,1); lcd.print("SetMon "); break;
//             case 6: targetVar = &YEAR_BASE;   cursorCol = 0; cursorRow = 0; maxVal = 2099; minVal = 2000; lcd.setCursor(9,1); lcd.print("SetYear"); break;
//             default: updateDisplayCursor_NonBlocking(0, 0, false); if(chose_menu==0) {lcd.setCursor(9,1); lcd.print("SELECT ");} break; 
//         }
        
//         if (targetVar != nullptr) { // 如果当前选中了某个时间单位进行设置
//             updateDisplayCursor_NonBlocking(cursorCol, cursorRow, true); // 显示闪烁光标
//             bool valueChanged = false; // 标记值是否被修改

//             // 处理 "加" 按键
//             if (currentAddState == LOW && lastAddState_SC == HIGH) {
//                 delay(ButtonDelay);
//                 if (digitalRead(add) == LOW) {
//                     (*targetVar)++; 
//                     if (chose_menu == 4) maxVal = DaysInMonth(YEAR_BASE, MONTH_BASE); // 日期需动态获取当月最大天数
//                     if (*targetVar > maxVal) *targetVar = minVal; // 超出最大值则回到最小值
//                     valueChanged = true;
//                 }
//             }
//             // 处理 "减" 按键
//             if (currentMinusState == LOW && lastMinusState_SC == HIGH) {
//                 delay(ButtonDelay);
//                 if (digitalRead(minus) == LOW) {
//                     (*targetVar)--; 
//                     if (chose_menu == 4) maxVal = DaysInMonth(YEAR_BASE, MONTH_BASE);
//                     if (*targetVar < minVal) *targetVar = maxVal; // 小于最小值则回到最大值
//                     valueChanged = true;
//                 }
//             }
            
//             if(valueChanged){ // 如果值被修改了
//                 entryTime_SC = millis(); // 重置超时计时
                

//                 // 如果修改的是年或月，需要检查日期的有效性并调整
//                 if (chose_menu == 5 || chose_menu == 6) { 
//                     Serial.print("DAY_BASE IS ");
//                     Serial.println(DAY_BASE);
//                     if (DAY_BASE > DaysInMonth(YEAR_BASE, MONTH_BASE)) { 
//                         DAY_BASE = DaysInMonth(YEAR_BASE, MONTH_BASE); 
//                         day_val = DAY_BASE; 
//                     }
//                 }
//                 // 如果修改的是日 (确保日期在有效范围内，尽管maxVal应该已经处理了上限)
//                 if (chose_menu == 4) { 
//                     Serial.print("DAY_BASE IS ");
//                     Serial.print(DAY_BASE);
//                     Serial.print(" YEARBASE IS ");
//                     Serial.print(YEAR_BASE);
//                     Serial.print(" MONTHBASE IS ");
//                     Serial.println(MONTH_BASE);
//                     if (DAY_BASE > DaysInMonth(YEAR_BASE, MONTH_BASE)) { 
//                         DAY_BASE = DaysInMonth(YEAR_BASE, MONTH_BASE);
//                         Serial.print("DAY_BASE2 IS ");
//                     Serial.print(DAY_BASE);
//                     }
//                     if (DAY_BASE < 1) DAY_BASE = 1; 
//                     day_val = DAY_BASE;
//                     Serial.print("DAY_val IS ");
//                     Serial.print(day_val);
//                 }
//                 // 将修改后的 BASE 值同步到 _val 变量，用于显示预览
//                 year_val = YEAR_BASE; month_val = MONTH_BASE; day_val = DAY_BASE;
//                 hour_val = HOUR_BASE; minute_val = MINUTE_BASE; second_val = SECOND_BASE;
//                 rtc.setDate(day_val, month_val, year_val);
//                 Serial.println("RTC SETDATE");
//                 DisplayAll(false); // 显示修改后的预览 (不从RTC刷新)
//             }
//         }
//         // 更新按键的上一状态
//         lastAddState_SC = currentAddState;
//         lastMinusState_SC = currentMinusState;
//     }
// }

// // 闹钟及报警温度设置模式处理函数
// void Set_Alarm_Mode() {
//     static bool lastAddState_SA_ModeEntry = HIGH;
//     static bool lastMinusState_SA_ModeEntry = HIGH;
//     static bool lastAddState_SA_Item = HIGH;
//     static bool lastMinusState_SA_Item = HIGH;
//     static bool lastChooseState_SA_Item = HIGH;
//     static unsigned long entryTime_SA;
    

//     bool currentAddBtnToEnter = digitalRead(add);
//     bool currentMinusBtnToEnter = digitalRead(minus);

//     // 检测是否需要进入闹钟设置模式：同时按下 "加" 和 "减" 键
//     if (!inSetAlarmMode && currentAddBtnToEnter == LOW && currentMinusBtnToEnter == LOW &&
//         lastAddState_SA_ModeEntry == HIGH && lastMinusState_SA_ModeEntry == HIGH) {
//         delay(ButtonDelay * 2); // 稍长延时以确认是同时按下
//         if (digitalRead(add) == LOW && digitalRead(minus) == LOW) {
//             inSetAlarmMode = true;
//             alarm_menu_item = 0; // 初始未选择任何项

//             // 重置模式内按键状态和超时计时器
//             lastChooseState_SA_Item = HIGH;
//             lastAddState_SA_Item = HIGH;
//             lastMinusState_SA_Item = HIGH;
//             entryTime_SA = millis();

//             // 清屏并显示闹钟设置界面初始状态
//             // Serial.print("PLOT ALARM1 \n");
//             lcd.clear();
//             lcd.setCursor(0, 0); lcd.print("Alarm:"); 
//             FormatDisplay(0, 1, alarm_hour_set); lcd.print(":"); FormatDisplay(3, 1, alarm_minute_set); 
//             lcd.setCursor(9, 0); lcd.print("Temp:");
//             lcd.setCursor(9, 1);
//             if (Temp_Alarm_Threshold < 10 && Temp_Alarm_Threshold >= 0)
//                 lcd.print("0");
//             lcd.print(Temp_Alarm_Threshold, 0);
//             lcd.print((char)223);
//             lcd.print("C");
//             lcd.setCursor(0, 0);
//             lcd.print("SELECT ");
//         }
//     }

//     // 更新用于判断进入模式的按键状态 (无论是否进入模式都更新)
//     lastAddState_SA_ModeEntry = currentAddBtnToEnter;
//     lastMinusState_SA_ModeEntry = currentMinusBtnToEnter;

//     if (inSetAlarmMode) { // 如果已处于闹钟设置模式
//         // 超时退出逻辑
//         if (millis() - entryTime_SA > 15000 && alarm_menu_item == 0) {
//             inSetAlarmMode = false; alarm_menu_item = 0;
//             Serial.println("TIME OUT");
//             updateDisplayCursor_NonBlocking(0, 0, false);
//             lcd.clear(); lcd.print("TimeOut");
//             delay(1000);
//             DisplayAll(true);
//             return;
//         }

//         // 读取模式内各按键状态
//         bool currentChooseStateItem = digitalRead(choose);
//         bool currentAddStateItem = digitalRead(add);
//         bool currentMinusStateItem = digitalRead(minus);

//         // "选择"键用于切换设置项
//         if (currentChooseStateItem == LOW && lastChooseState_SA_Item == HIGH) {
//             delay(ButtonDelay);
//             if (digitalRead(choose) == LOW) {
//                 alarm_menu_item++;
//                 entryTime_SA = millis();
//                 if (alarm_menu_item >= 4) { // 完成或超出范围则保存并退出
//                     inSetAlarmMode = false; alarm_menu_item = 0;
//                     updateDisplayCursor_NonBlocking(0, 0, false);
//                     lcd.clear(); lcd.print("Alarm Saved"); 
                    
//                     Serial.println("Alarm Saved");
//                     delay(1000);
//                     lcd.clear();
//                     DisplayAll(true); // 恢复正常显示
//                     return;
//                 }
//                 lcd.setCursor(0, 0); lcd.print("        "); // 清理左上角提示区
//             }
//         }
//         lastChooseState_SA_Item = currentChooseStateItem;

//         int cursorCol = -1, cursorRow = -1; // 光标位置
//         bool valueChanged_SA = false;       // 标记值是否被修改

//         // 根据 alarm_menu_item 确定当前设置哪个参数
//         switch (alarm_menu_item) {
//             case 1: cursorCol = 0; cursorRow = 1; lcd.setCursor(0, 0); lcd.print("SetHour "); break;
//             case 2: cursorCol = 3; cursorRow = 1; lcd.setCursor(0, 0); lcd.print("SetMin  "); break; 
//             case 3: cursorCol = 9; cursorRow = 1; lcd.setCursor(0, 0); lcd.print("SetTemp "); break; 
//             default: updateDisplayCursor_NonBlocking(0, 0, false); if (alarm_menu_item == 0) { lcd.setCursor(0, 0); lcd.print("SELECT  "); } break; 
//         } 

//         if (alarm_menu_item > 0 && alarm_menu_item < 4) { // 如果选中了具体设置项
//             updateDisplayCursor_NonBlocking(cursorCol, cursorRow, true); // 显示闪烁光标
//             // 处理 "加" 按键
//             if (currentAddStateItem == LOW && lastAddState_SA_Item == HIGH) {
//                 delay(ButtonDelay);
//                 if (digitalRead(add) == LOW) {
//                     if (alarm_menu_item == 1) alarm_hour_set = (alarm_hour_set + 1) % 24; 
//                     else if (alarm_menu_item == 2) alarm_minute_set = (alarm_minute_set + 1) % 60; 
//                     else if (alarm_menu_item == 3) { Temp_Alarm_Threshold++; if (Temp_Alarm_Threshold > 99) Temp_Alarm_Threshold = 0; } // 温度循环 0-99
//                     valueChanged_SA = true;
//                 }
//             }
//             // 处理 "减" 按键
//             if (currentMinusStateItem == LOW && lastMinusState_SA_Item == HIGH) {
//                 delay(ButtonDelay);
//                 if (digitalRead(minus) == LOW) {
//                     if (alarm_menu_item == 1) alarm_hour_set = (alarm_hour_set - 1 + 24) % 24;
//                     else if (alarm_menu_item == 2) alarm_minute_set = (alarm_minute_set - 1 + 60) % 60;
//                     else if (alarm_menu_item == 3) { Temp_Alarm_Threshold--; if (Temp_Alarm_Threshold < 0) Temp_Alarm_Threshold = 99; } // 温度循环 99-0
//                     valueChanged_SA = true;
//                 }
//             }
//             if (valueChanged_SA) { // 如果值被修改
//                 entryTime_SA = millis(); // 重置超时
//                 // 重新显示闹钟和温度设置的当前值
//                 lcd.setCursor(0, 0); lcd.print("Alarm:  "); 
//                 FormatDisplay(0, 1, alarm_hour_set); lcd.print(":"); FormatDisplay(3, 1, alarm_minute_set);
//                 lcd.setCursor(9, 0); lcd.print("Temp:   ");
//                 lcd.setCursor(9, 1); if (Temp_Alarm_Threshold < 10 && Temp_Alarm_Threshold >= 0) lcd.print("0"); lcd.print(Temp_Alarm_Threshold, 0); lcd.print((char)223); lcd.print("C");
                
//                 // 根据当前设置项，更新左上角的提示文本
//                 if (alarm_menu_item == 1) { lcd.setCursor(0, 0); lcd.print("SetHour "); }
//                 else if (alarm_menu_item == 2) { lcd.setCursor(0, 0); lcd.print("SetMin  "); } 
//                 else if (alarm_menu_item == 3) { lcd.setCursor(0, 0); lcd.print("SetTemp "); } 
//             }
//         } 
//         // 更新用于项目内调整的按键状态
//         lastAddState_SA_Item = currentAddStateItem;
//         lastMinusState_SA_Item = currentMinusStateItem;
//     } 
// } 

// // --- 闹钟及温度检测函数 ---
// // 整点报时
// void Point_Time_Alarm_check() { 
//     Time tm = rtc.getTime();
//     if (tm.min == 0 && tm.sec == 0) { 
//         static unsigned long lastChimeHour = 25; // 用无效小时确保第一次能报时
//         if (tm.hour != lastChimeHour) { 
//             tone(TonePin, ChimeFrequence, HOURLY_CHIME_DURATION); // 使用 TonePin
//             lastChimeHour = tm.hour; 
//         }
//     }
// }

// // 时间闹钟检测
// void Clock_Alarm_check() { 
//     Time tm = rtc.getTime();
//     if (tm.hour == alarm_hour_set && tm.min == alarm_minute_set && tm.sec == alarm_second_set) {
//         unsigned long alarmStartTime = millis();
//         bool alarmSilenced = false; 
//         while (millis() - alarmStartTime < ALARM_DURATION && !alarmSilenced) {
//             unsigned long currentBeepCycle = millis() - alarmStartTime;
//             // 间歇鸣响：响250ms，停250ms
//             if ((currentBeepCycle / 250) % 2 == 0) { 
//                 // tone(TonePin, ALARM_SOUND_FREQ, 200); // 使用tone可以指定频率，但会持续响，不适合间歇
//                 digitalWrite(TonePin, HIGH); // 或者直接驱动蜂鸣器引脚
//             } else {
//                 // noTone(TonePin);
//                 digitalWrite(TonePin, LOW); 
//             }
//             // 非阻塞检查按键以停止闹钟
//             if (digitalRead(choose) == LOW || digitalRead(add) == LOW || digitalRead(minus) == LOW) {
//                 unsigned long pressTime = millis();
//                 // 等待按键释放或消抖延时
//                 while (digitalRead(choose) == LOW || digitalRead(add) == LOW || digitalRead(minus) == LOW) {
//                     if (millis() - pressTime > ButtonDelay) { 
//                         alarmSilenced = true;
//                         break;
//                     }
//                 }
//                 if (alarmSilenced) break; // 如果已确认按键，跳出鸣响循环
//             }
//             delay(10); // 短延时，让循环有机会检查按键
//         }
//         digitalWrite(TonePin, LOW); // 确保蜂鸣器最终关闭
//         noTone(TonePin); // 如果之前用了tone()，这里确保停止
//     }
// }

// // 获取温度并更新显示
// void GetTemperatures_update() { 
//     long adc_val = analogRead(TEMP_SENSOR_PIN); 
//     // 假设 LM35: 10mV/°C, Arduino ADC 参考电压为 5V, 10位ADC (1024级)
//     // 温度 = (ADC读数 / 1024) * 5V * 100 (°C/V) 
//     Temperatures_current = (5.0 * adc_val * 100.0) / 1024.0; 
//     float filteredTemperature = applyKalmanFilter(Temperatures_current); 

//     lcd.setCursor(9, 1); lcd.print("       "); // 清理温度显示区域 (7个空格)
//     lcd.setCursor(9, 1); 

//     // 格式化显示温度
//     if (filteredTemperature <= -10) { 
//         lcd.print(filteredTemperature, 1); // 显示一位小数
//     } else if (filteredTemperature < 0) { // -9.9 到 -0.1
//         lcd.print("-");
//         if (abs(filteredTemperature) < 10) lcd.print("0"); // 例如 -0.5, -9.5
//         lcd.print(abs(filteredTemperature), 1);
//     } else if (filteredTemperature >= 0 && filteredTemperature < 10) { // 0.0 到 9.9
//         lcd.print("0"); // 例如 0.5, 9.5
//         lcd.print(filteredTemperature, 1);
//     } else { // >= 10.0
//         lcd.print(filteredTemperature, 1);
//     }

//     lcd.print((char)223); // 显示摄氏度符号 °
//     lcd.print("C");       
// }

// // 温度报警检测
// void Temperatures_Alarm_check() { 
//     // 使用滤波后的温度 (kalman_x_hat) 进行报警判断
//     if (kalman_x_hat >= Temp_Alarm_Threshold) {
//         static unsigned long lastTempAlarmTime = 0;
//         // 避免持续报警过于频繁，设置最小间隔 (例如2.5秒)
//         if (millis() - lastTempAlarmTime > (TEMP_ALARM_DURATION + 2000)) { 
//             tone(TonePin, ALARM_SOUND_FREQ, TEMP_ALARM_DURATION); // 使用 TonePin
//             lastTempAlarmTime = millis(); // 更新上次报警时间
//         }
//     }
// }

