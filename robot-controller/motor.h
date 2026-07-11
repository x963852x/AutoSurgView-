/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of motor

**************************************************************************/

#pragma once
#include <stdio.h>
#include <iostream>
#include <mutex>
#include "ECANVci.h"
#include "IQmath.h"

#pragma region 宏定义值
//关节电机CAN通信ID
#define FIRST_MOTOR_CAN 1
#define SECOND_MOTOR_CAN 2
#define THIRD_MOTOR_CAN 3

//关节电机控制模式
#define MODE_CURRENT 1	//电流模式
#define MODE_SPEED 2	//速度模式
#define MODE_POSITION 3	//位置模式
#define MODE_POSITION_S 6	//梯形位置模式
#define MODE_SPEED_S 7	//梯形速度模式
#define MODE_HOMING 8		//HOMING模式

#define MAX_CAN_TIME 10	//CAN通信失败重复尝试次数

//关节电机限制角度，单位°
#define FIRST_MOTOR_MAX 90		//关节电机1最大角度
#define FIRST_MOTOR_MIN -180	//关节电机1最小角度
#define SECOND_MOTOR_MAX 150	//关节电机2最大角度
#define SECOND_MOTOR_MIN -150	//关节电机2最小角度
#define THIRD_MOTOR_MAX 149		//关节电机3最大角度
#define THIRD_MOTOR_MIN -155	//关节电机3最小角度

#define TRAJ_TIME_STEP 0.006	//轨迹插值步长，秒
#pragma endregion

//带时间戳的轨迹点
struct traj_point {
	double positions[5];
	double time_from_start;
};

#pragma region 关节电机类
class motor
{
private:
	//CAN通信变量
	int nDeviceType = 3;
	int nDeviceInd = 0;
	int nCANInd = 0;
	int nReserved = 0;
public:
	motor();	//关节电机类默认构造函数
	~motor();	//关节电机类默认析构函数
	bool openCAN();	//关节电机类成员函数：打开CAN通信设备
	void closeCAN();	//关节电机类成员函数：关闭CAN通信设备
	bool set_param(int number = 4, int mode = MODE_POSITION_S, double rpm = 5);	// 关节电机类成员函数：关节电机初始化
	bool enable(int number = 4);	// 关节电机类成员函数：使能关节电机
	void disable(int number = 4);	// 关节电机类成员函数：失能关节电机
	bool handshake(int number = 4);	// 关节电机类成员函数：关节电机心跳通信
	bool setPositionLimit(int number = 4);	// 关节电机类成员函数：设置关节电机位置限制
	bool setSpeedLimit(int number = 4);	// 关节电机类成员函数：设置关节电机速度限制
	bool setMode(int mode, int number = 4);	// 关节电机类成员函数：设置关节电机控制模式
	void setPosition(double position, int number = 4);	// 关节电机类成员函数：设置关节电机位置值
	double getPosition(int number = 4);	// 关节电机类成员函数：获取关节电机位置值
	bool setSpeed(double rpm, int number = 4);	// 关节电机类成员函数：设置关节电机速度限制值
	bool setSpeedModeSpeed(double rpm, int number);
	double getSpeed(int number = 4);	// 关节电机类成员函数：获取关节电机速度限制值
	void stop(int number = 4);	// 关节电机类成员函数：关节电机急停
	void setAngle(double angle, int number = 4);	// 关节电机类成员函数：设置关节电机角度值
	double getAngle(int number = 4);	// 关节电机类成员函数：获取关节电机角度值
	bool setProportional(double p, int number = 4);	// 关节电机类成员函数：设置关节电机PID参数P
	bool setIntegral(double i, int number = 4);	// 关节电机类成员函数：设置关节电机PID参数I
	double getProportional(int number = 4);	// 关节电机类成员函数：获取关节电机PID参数P
	double getIntegral(int number = 4);	// 关节电机类成员函数：获取关节电机PID参数I
};
#pragma endregion

#pragma region 关节电机功能函数
double AngletoPosition(double angle, int number);	//角度转化为r（转）
double PositiontoAngle(double position, int number);	//r（转）转化为角度
#pragma endregion