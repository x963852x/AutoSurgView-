/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of liftingcolumn control

**************************************************************************/

//#pragma once
//
//#include <thread>
//#include <iostream>
//#include <mutex>
//
//#include "serial/serial.h"
//#include "ssbot_config.h"
//
////立柱运动状态
//#define LC_UP 0		//立柱向上
//#define LC_DOWN 1	//立柱向下
//#define LC_STOP 2	//立柱停止
//
////立柱工作状态
//#define LC_RESET 1 //立柱向目标运动状态
//#define LC_MOVING 1 //立柱向目标运动状态
//#define LC_DEBUG 2	//立柱调试状态（对应机器人面板按键控制）
//#define LC_FREE 0	//立柱空闲状态
//
////升降立柱类
//class LiftingColumn
//{
//public:
//	LiftingColumn();
//	~LiftingColumn();
//
//	void LiftingColumn::reset();	//升降立柱类成员函数：重启指令
//	void up();	//升降立柱类成员函数：上升指令
//	void down();	//升降立柱类成员函数：下降指令
//	void stop();	//升降立柱类成员函数：停止指令
//	double get_height();	//升降立柱类成员函数：获取立柱高度
//	void move_to_height(double height);	//升降立柱成员函数：运动至指定高度
//
//	void get_CheckSum(uint8_t buf[], int len, uint8_t& high, uint8_t& low);
//};
//
//uint8_t ascii_to_hex(uint8_t ascii);
//uint8_t hex_to_ascii(uint8_t hex);
//
//void liftcolumn_work();	//升降立柱伺服程序
//void liftcolumn_init();

#ifndef _Lift_Colum_Class_
#define _Lift_Colum_Class_

#include <thread>
#include <iostream>
#include <mutex>

#include "LiftingColumn.h"
#include "serial/serial.h"
#include "ssbot_config.h"
using namespace std;

//立柱运动状态
#define LC_UP 0		//立柱向上
#define LC_DOWN 1	//立柱向下
#define LC_STOP 2	//立柱停止

//立柱工作状态
#define LC_RESET 1 //立柱向目标运动状态
#define LC_MOVING 1 //立柱向目标运动状态
#define LC_DEBUG 2	//立柱调试状态（对应机器人面板按键控制）
#define LC_FREE 0	//立柱空闲状态
void liftcolumn_init();
void liftcolumn_work();
uint16_t Crc_Compute(const std::vector<uint8_t>& data);
void creat_crc(vector<uint8_t>& cw);
bool check_crc(vector<uint8_t>& cr);
class LiftingColumn
{
public:
	LiftingColumn();
	~LiftingColumn();
	void reset();	//升降立柱类成员函数：重启指令
	void up();	//升降立柱类成员函数：上升指令
	void down();	//升降立柱类成员函数：下降指令
	void stop();	//升降立柱类成员函数：停止指令
	int dis_2_pulse(int dis);
	void set_relative_dis(int dis);
	void set_to_height(int height);
	void move_to_height(int height);
	int get_position();
	int get_height();
	void LC_Enable();
	void LC_Disable();
	void set_speed(double speed);
	void GHGSTP_High();
	void GHGSTP_Low();
	void set_pulse_per_circle();
	void set_angle_mode();
	void set_effective_segments();
	void set_starting_segments();
};
#endif // !_Lift_Colum_Class