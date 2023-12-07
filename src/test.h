#ifndef __TEST_H__
#define __TEST_H__
#include <Arduino.h>
#include "config.h" //引脚配置
#include <HardwareSerial.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <SPI.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern int txid;
void stop(int);                   // 急停
void set_status(int, float);      // 设置电机状态
int read_status(int);             // 读取电机状态
void set_mode(int, float);        // 设置电机模式
int read_mode(int);               // 读取电机模式
void set_zp(int);                 // 设置当前位置为零点
void set_PID(int, float, float);  // 设置PID参数
float read_PID(int, float);       // 读取PID参数
void set_lim(int, float, float);  // 设置限制参数
float read_lim(int, float);       // 读取限制参数
void spr(int, float);             // 单点运行
void spt(int, float, float);      // 单点轨迹
void set_ctp(int, float, float);  // 设置连续轨迹位置数据
void set_ctv(int, float, float);  // 设置连续轨迹速度数据
void set_ctmf(int, float, float); // 设置连续轨迹矩流数据
void tdr(int, float);             // 指定轨迹数据运行
void record(int, float);          // 记录数据到指定索引位置
float read_rd(int, float);        // 读取运行数据
void set_CAN_ID(int, float);      // 设置CAN ID
void reset(int);                  // 还原设置
void receive_data();              // 接收数据

#endif