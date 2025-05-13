#ifndef TIMEMANAGER_H
#define TIMEMANAGER_H

#include <Arduino.h>
#include <DS1302.h>

// 结构体用于存储和传递时间日期信息
struct TimeDate {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int dayOfWeek; // 1 for Monday, 7 for Sunday
};

void TimeManager_init(DS1302 &rtc_chip); // 初始化时间管理器，传入DS1302对象引用
void TimeManager_updateFromRTC();       // 从RTC读取时间并更新内部时间
void TimeManager_saveToRTC();           // 将当前内部时间保存到RTC

TimeDate TimeManager_getCurrentTimeDate(); // 获取当前时间日期结构体

void TimeManager_setTime(int hr, int mn, int sc); // 设置内部时间
void TimeManager_setDate(int yr, int mo, int dy); // 设置内部日期

// 供设置模块使用的单个时间日期组件设置函数
void TimeManager_setHour(int hr);
void TimeManager_setMinute(int mn);
void TimeManager_setSecond(int sc);
void TimeManager_setDay(int dy);
void TimeManager_setMonth(int mo);
void TimeManager_setYear(int yr);

// 辅助函数
String TimeManager_getDayOfWeekString(int y, int m, int d);
int TimeManager_getDaysInMonth(int y, int m);
void TimeManager_calculateCurrentTime(); // 基于millis()和初始RTC时间计算当前时间（如果需要更高精度或RTC仅用于断电保存）

#endif // TIMEMANAGER_H